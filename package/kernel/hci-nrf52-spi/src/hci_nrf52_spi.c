// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2026 Willian G. Silva <sk4rfx@proton.me>
 *
 * Bluetooth HCI over SPI for the Nordic nRF52832 found on MikroTik boards
 * (e.g. KNOT, RB924i-2nD-BT5&BG77), running the vendor HCI firmware (Zephyr
 * based, LE only, no public address).
 *
 * Transport, as observed on the board:
 *  - H4 framing: one packet type byte followed by the standard HCI header
 *    and payload. Events (0x04) and ACL (0x02) come from the controller;
 *    commands (0x01) and ACL go to it.
 *  - One interrupt line, active low, with two meanings: the controller
 *    drives it active for a short while after each transaction it receives
 *    (busy), and drives it active when it has a packet for the host. It
 *    releases the line when it is ready for the next transaction.
 *  - Host to controller: the whole H4 packet in one transaction, only after
 *    the line was released.
 *  - Controller to host: the packet in one transaction, preceded by up to
 *    two filler bytes, so the host clocks a fixed-size window and looks for
 *    the type byte at its start.
 *  - The controller needs a few microseconds between chip select and the
 *    first clock ("spi-cs-setup-delay-ns"), or it loses the first bit.
 *
 * The interrupt fires on both edges and the handler reads the line level:
 * the ath79 GPIO irqchip does not mask level interrupts, so a level-low line
 * would fire continuously.
 */

#include <linux/gpio/consumer.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/skbuff.h>
#include <linux/slab.h>
#include <linux/spi/spi.h>
#include <linux/unaligned.h>
#include <linux/wait.h>
#include <linux/workqueue.h>

#include <net/bluetooth/bluetooth.h>
#include <net/bluetooth/hci_core.h>

/* largest frame: an event with 255 parameter bytes */
#define NRF52_RX_MAX_FRAME	(1 + HCI_EVENT_HDR_SIZE + 255)
#define NRF52_LEAD		3	/* type byte found within these */
#define NRF52_RX_LEN		(NRF52_LEAD + NRF52_RX_MAX_FRAME)
/* the vendor driver refuses to send 256 bytes or more */
#define NRF52_TX_MAX_FRAME	255
#define NRF52_TXQ_MAX		64	/* queued packets before -ENOBUFS */
#define NRF52_RX_BURST		4	/* packets per work run */
#define NRF52_RX_GAP_MS		5	/* pause between bursts (SPI shared with NAND) */
#define NRF52_RX_MAX_FAIL	32	/* consecutive failures before a reset */
#define NRF52_REL_MS		5	/* line release after a transaction */
#define NRF52_TX_READY_MS	40	/* vendor driver: 10 jiffies */
#define NRF52_TX_READY_TRIES	3
#define NRF52_TX_BUSY_MS	10	/* longest busy pulse before reading */
#define NRF52_STORM_EDGES	500	/* edges per jiffy treated as a storm */
#define NRF52_STORM_PAUSE_MS	100	/* interrupt off after a storm */

/* Zephyr vendor command: status, count, then 6 address + 16 IR bytes each */
#define NRF52_OP_READ_STATIC_ADDRS	0xfc09

/* flags */
#define NRF52_RUNNING		0	/* hdev open: reception allowed */
#define NRF52_IRQ_OFF		1	/* our disable_irq() is in effect */
#define NRF52_TX_BUSY		2	/* sent a packet, controller not done yet */

struct nrf52_hci {
	struct spi_device *spi;
	struct hci_dev *hdev;
	struct gpio_desc *irq_gpio;
	int irq;
	struct mutex xfer_lock;		/* one SPI transaction at a time */
	struct sk_buff_head txq;
	struct work_struct tx_work;
	struct delayed_work rx_work;
	struct delayed_work storm_work;	/* re-enables the irq after a storm */
	wait_queue_head_t line_wq;	/* woken on every edge of the line */
	atomic_t releases;		/* count of line releases */
	unsigned long storm_jiffies;
	unsigned int storm_edges;
	unsigned int rx_fail;
	unsigned long flags;
	u8 *rxbuf;
	u8 *txbuf;
};

/* Balanced with nrf52_irq_on(): only one disable outstanding at a time. */
static void nrf52_irq_off(struct nrf52_hci *nrf, bool sync)
{
	if (test_and_set_bit(NRF52_IRQ_OFF, &nrf->flags))
		return;
	if (sync)
		disable_irq(nrf->irq);
	else
		disable_irq_nosync(nrf->irq);
}

static void nrf52_irq_on(struct nrf52_hci *nrf)
{
	if (test_and_clear_bit(NRF52_IRQ_OFF, &nrf->flags))
		enable_irq(nrf->irq);
}

/* Line active: the controller is busy or has a packet for us. */
static bool nrf52_active(struct nrf52_hci *nrf)
{
	return gpiod_get_value(nrf->irq_gpio) > 0;
}

/* At HZ=100 a 5 ms timeout is one jiffy, which may expire at once. */
static unsigned long nrf52_ms(unsigned int ms)
{
	return msecs_to_jiffies(ms) + 1;
}

/* Wait for the release that follows the transaction started at @seq. */
static void nrf52_wait_release(struct nrf52_hci *nrf, int seq)
{
	wait_event_timeout(nrf->line_wq,
			   atomic_read(&nrf->releases) != seq ||
			   !test_bit(NRF52_RUNNING, &nrf->flags),
			   nrf52_ms(NRF52_REL_MS));
}

static int nrf52_hdr_len(u8 type)
{
	return type == HCI_EVENT_PKT ? HCI_EVENT_HDR_SIZE : HCI_ACL_HDR_SIZE;
}

static int nrf52_payload_len(u8 type, const u8 *hdr)
{
	if (type == HCI_EVENT_PKT)
		return hdr[1];
	return get_unaligned_le16(hdr + 2);	/* ACL */
}

/*
 * Find the H4 frame after the filler bytes and check that its header is
 * plausible. Returns the offset of the type byte, or a negative errno.
 */
static int nrf52_find_frame(const u8 *buf, u8 *type, int *hlen, int *plen)
{
	int i;

	for (i = 0; i < NRF52_LEAD; i++) {
		u8 t = buf[i];

		if (t == 0x00)
			continue;			/* filler */
		if (t != HCI_EVENT_PKT && t != HCI_ACLDATA_PKT)
			return -EBADMSG;
		*type = t;
		*hlen = nrf52_hdr_len(t);
		*plen = nrf52_payload_len(t, &buf[i + 1]);
		if (1 + *hlen + *plen > NRF52_RX_MAX_FRAME)
			return -EPROTO;
		if (t == HCI_EVENT_PKT && buf[i + 1] == 0x00)
			return -EPROTO;			/* no event code 0 */
		return i;
	}
	return -EBUSY;					/* only filler */
}

/*
 * Read one packet and pass it to the stack. Returns -EAGAIN when there is
 * nothing to read now (line released, or the controller is busy with a
 * packet we sent).
 */
static int nrf52_read_packet(struct nrf52_hci *nrf)
{
	struct spi_transfer t = {
		.rx_buf = nrf->rxbuf,
		.len = NRF52_RX_LEN,
	};
	struct sk_buff *skb;
	int i, hlen, plen, ret, seq;
	u8 type = 0;

	mutex_lock(&nrf->xfer_lock);
	/* checked under the lock: no transmission can start in between */
	if (!nrf52_active(nrf) || test_bit(NRF52_TX_BUSY, &nrf->flags)) {
		mutex_unlock(&nrf->xfer_lock);
		return -EAGAIN;
	}
	seq = atomic_read(&nrf->releases);
	ret = spi_sync_transfer(nrf->spi, &t, 1);

	/*
	 * The controller releases its line some tens of microseconds after
	 * the transaction: wait for that before letting anyone else talk to
	 * it, or the next look at the line sees it active and clocks the
	 * controller while it re-arms.
	 */
	nrf52_wait_release(nrf, seq);
	mutex_unlock(&nrf->xfer_lock);
	if (ret)
		return ret;

	i = nrf52_find_frame(nrf->rxbuf, &type, &hlen, &plen);
	if (i < 0)
		return i;

	skb = bt_skb_alloc(hlen + plen, GFP_KERNEL);
	if (!skb)
		return -ENOMEM;

	hci_skb_pkt_type(skb) = type;
	skb_put_data(skb, &nrf->rxbuf[i + 1], hlen + plen);
	nrf->hdev->stat.byte_rx += 1 + hlen + plen;

	return hci_recv_frame(nrf->hdev, skb);
}

static void nrf52_rx_work(struct work_struct *work)
{
	struct nrf52_hci *nrf = container_of(to_delayed_work(work),
					     struct nrf52_hci, rx_work);
	int i, ret;

	if (!test_bit(NRF52_RUNNING, &nrf->flags))
		return;

	for (i = 0; i < NRF52_RX_BURST; i++) {
		ret = nrf52_read_packet(nrf);
		if (ret == -EAGAIN)
			return;			/* the next assertion requeues us */
		if (!ret) {
			nrf->rx_fail = 0;
			continue;
		}

		nrf->hdev->stat.err_rx++;
		if (++nrf->rx_fail >= NRF52_RX_MAX_FAIL) {
			/* stop reading and let the core reset the controller */
			clear_bit(NRF52_RUNNING, &nrf->flags);
			nrf52_irq_off(nrf, false);
			bt_dev_err(nrf->hdev,
				   "%u rx failures in a row (last %d), resetting",
				   nrf->rx_fail, ret);
			hci_reset_dev(nrf->hdev);
			return;
		}
		break;				/* back off before retrying */
	}

	/* still active (burst done or after an error): come back later */
	if (nrf52_active(nrf) && !test_bit(NRF52_TX_BUSY, &nrf->flags) &&
	    test_bit(NRF52_RUNNING, &nrf->flags))
		schedule_delayed_work(&nrf->rx_work,
				      msecs_to_jiffies(NRF52_RX_GAP_MS));
}

/*
 * Assertion while a packet we sent is being handled: the controller is
 * busy, not data. Any other assertion: a packet to read. Release: ready
 * for the next transaction (and the end of any busy period).
 */
static irqreturn_t nrf52_irq(int irq, void *data)
{
	struct nrf52_hci *nrf = data;

	/* a line toggling without pause would starve the system */
	if (time_after(jiffies, nrf->storm_jiffies)) {
		nrf->storm_jiffies = jiffies;
		nrf->storm_edges = 0;
	}
	if (++nrf->storm_edges > NRF52_STORM_EDGES) {
		nrf52_irq_off(nrf, false);
		schedule_delayed_work(&nrf->storm_work,
				      nrf52_ms(NRF52_STORM_PAUSE_MS));
		return IRQ_HANDLED;
	}

	if (nrf52_active(nrf)) {
		if (!test_bit(NRF52_TX_BUSY, &nrf->flags) &&
		    test_bit(NRF52_RUNNING, &nrf->flags))
			schedule_delayed_work(&nrf->rx_work, 0);
	} else {
		clear_bit(NRF52_TX_BUSY, &nrf->flags);
		atomic_inc(&nrf->releases);
	}
	wake_up(&nrf->line_wq);
	return IRQ_HANDLED;
}

static void nrf52_storm_work(struct work_struct *work)
{
	struct nrf52_hci *nrf = container_of(to_delayed_work(work),
					     struct nrf52_hci, storm_work);

	if (!test_bit(NRF52_RUNNING, &nrf->flags))
		return;
	bt_dev_warn_ratelimited(nrf->hdev, "interrupt storm, paused");
	nrf->storm_edges = 0;
	nrf52_irq_on(nrf);
	schedule_delayed_work(&nrf->rx_work, 0);
}

/* Ready for a packet: line released and not busy with the previous one. */
static bool nrf52_tx_ready(struct nrf52_hci *nrf)
{
	return !nrf52_active(nrf) && !test_bit(NRF52_TX_BUSY, &nrf->flags);
}

static int nrf52_write_packet(struct nrf52_hci *nrf, unsigned int len)
{
	struct spi_transfer t = {
		.tx_buf = nrf->txbuf,
		.len = len,
	};
	int tries, ret = -EBUSY;

	for (tries = 0; tries < NRF52_TX_READY_TRIES; tries++) {
		wait_event_timeout(nrf->line_wq,
				   nrf52_tx_ready(nrf) ||
				   !test_bit(NRF52_RUNNING, &nrf->flags),
				   nrf52_ms(NRF52_TX_READY_MS));
		if (!test_bit(NRF52_RUNNING, &nrf->flags))
			return -ENODEV;

		mutex_lock(&nrf->xfer_lock);
		/* checked under the lock: no reception can start in between */
		if (nrf52_tx_ready(nrf)) {
			set_bit(NRF52_TX_BUSY, &nrf->flags);
			ret = spi_sync_transfer(nrf->spi, &t, 1);
			if (ret)
				clear_bit(NRF52_TX_BUSY, &nrf->flags);
			mutex_unlock(&nrf->xfer_lock);
			break;
		}
		mutex_unlock(&nrf->xfer_lock);

		/* a packet for us may be holding the line: drain it */
		if (nrf52_active(nrf))
			schedule_delayed_work(&nrf->rx_work, 0);
	}
	if (ret)
		return ret;

	/*
	 * The busy pulse can be too short to be seen, merging with the
	 * assertion for the reply: if the line is not released soon, stop
	 * treating it as busy and read.
	 */
	if (!wait_event_timeout(nrf->line_wq,
				!test_bit(NRF52_TX_BUSY, &nrf->flags),
				nrf52_ms(NRF52_TX_BUSY_MS))) {
		clear_bit(NRF52_TX_BUSY, &nrf->flags);
		if (test_bit(NRF52_RUNNING, &nrf->flags))
			schedule_delayed_work(&nrf->rx_work, 0);
	}
	return 0;
}

static void nrf52_tx_work(struct work_struct *work)
{
	struct nrf52_hci *nrf = container_of(work, struct nrf52_hci, tx_work);
	struct sk_buff *skb;

	while ((skb = skb_dequeue(&nrf->txq))) {
		unsigned int len = skb->len + 1;
		int ret;

		nrf->txbuf[0] = hci_skb_pkt_type(skb);
		memcpy(&nrf->txbuf[1], skb->data, skb->len);

		ret = nrf52_write_packet(nrf, len);
		if (ret == -ENODEV) {		/* closing */
			kfree_skb(skb);
			continue;
		}
		if (ret) {
			bt_dev_err(nrf->hdev, "tx failed (%d)", ret);
			nrf->hdev->stat.err_tx++;
		} else {
			nrf->hdev->stat.byte_tx += len;
			if (hci_skb_pkt_type(skb) == HCI_COMMAND_PKT)
				nrf->hdev->stat.cmd_tx++;
			else
				nrf->hdev->stat.acl_tx++;
		}
		kfree_skb(skb);
	}
}

static int nrf52_open(struct hci_dev *hdev)
{
	struct nrf52_hci *nrf = hci_get_drvdata(hdev);

	nrf->rx_fail = 0;
	/* jiffies starts 5 minutes before wrapping: never compare with 0 */
	nrf->storm_jiffies = jiffies;
	nrf->storm_edges = 0;
	clear_bit(NRF52_TX_BUSY, &nrf->flags);
	set_bit(NRF52_RUNNING, &nrf->flags);
	nrf52_irq_on(nrf);

	/* the line may already be active (no edge): drain it once */
	schedule_delayed_work(&nrf->rx_work, 0);
	return 0;
}

static int nrf52_flush(struct hci_dev *hdev)
{
	struct nrf52_hci *nrf = hci_get_drvdata(hdev);

	skb_queue_purge(&nrf->txq);
	return 0;
}

static void nrf52_stop(struct nrf52_hci *nrf)
{
	clear_bit(NRF52_RUNNING, &nrf->flags);
	skb_queue_purge(&nrf->txq);
	wake_up(&nrf->line_wq);		/* tx waiters see !RUNNING */
	/* before irq_off: it must not re-enable the irq behind our back */
	cancel_delayed_work_sync(&nrf->storm_work);
	nrf52_irq_off(nrf, true);
	/* the irq may have been disabled without waiting (storm, failures) */
	synchronize_irq(nrf->irq);
	/* the handler may have queued it again before being disabled */
	cancel_delayed_work_sync(&nrf->storm_work);
	cancel_delayed_work_sync(&nrf->rx_work);
	cancel_work_sync(&nrf->tx_work);
	/* the tx work may have queued rx again before it was cancelled */
	cancel_delayed_work_sync(&nrf->rx_work);
	skb_queue_purge(&nrf->txq);
}

static int nrf52_close(struct hci_dev *hdev)
{
	nrf52_stop(hci_get_drvdata(hdev));
	return 0;
}

/*
 * Random static address (Core spec Vol 6 Part B 1.3.2.1): the two most
 * significant bits are 1 and the other 46 bits are neither all 0 nor all 1.
 */
static bool nrf52_is_static_addr(const u8 *a)
{
	bool zero = !(a[5] & 0x3f), ones = (a[5] & 0x3f) == 0x3f;
	int i;

	if ((a[5] & 0xc0) != 0xc0)
		return false;
	for (i = 0; i < 5; i++) {
		zero &= !a[i];
		ones &= a[i] == 0xff;
	}
	return !zero && !ones;
}

/* No public address: use the first static random address it reports. */
static int nrf52_setup(struct hci_dev *hdev)
{
	struct sk_buff *skb;
	const u8 *a;
	int ret = 0;

	skb = __hci_cmd_sync(hdev, NRF52_OP_READ_STATIC_ADDRS, 0, NULL,
			     HCI_INIT_TIMEOUT);
	if (IS_ERR(skb)) {
		bt_dev_err(hdev, "reading the static address failed (%ld)",
			   PTR_ERR(skb));
		return PTR_ERR(skb);
	}

	/* status, count, then { 6 address bytes, 16 IR bytes } each */
	a = &skb->data[2];
	if (skb->len < 2 + 22 || skb->data[0] || !skb->data[1] ||
	    skb->len < 2 + 22 * skb->data[1]) {
		bt_dev_err(hdev, "invalid static address reply");
		ret = -EPROTO;
	} else if (!nrf52_is_static_addr(a)) {
		bt_dev_err(hdev, "controller address is not a static one");
		ret = -EPROTO;
	} else {
		bacpy(&hdev->static_addr, (const bdaddr_t *)a);
	}

	kfree_skb(skb);
	return ret;
}

/* On error the HCI core frees the skb itself: do not free it here. */
static int nrf52_send(struct hci_dev *hdev, struct sk_buff *skb)
{
	struct nrf52_hci *nrf = hci_get_drvdata(hdev);

	switch (hci_skb_pkt_type(skb)) {
	case HCI_COMMAND_PKT:
	case HCI_ACLDATA_PKT:
		break;
	default:
		return -EILSEQ;
	}

	if (skb->len + 1 > NRF52_TX_MAX_FRAME)
		return -EMSGSIZE;
	if (skb_queue_len(&nrf->txq) >= NRF52_TXQ_MAX)
		return -ENOBUFS;

	skb_queue_tail(&nrf->txq, skb);
	schedule_work(&nrf->tx_work);
	return 0;
}

static int nrf52_probe(struct spi_device *spi)
{
	struct device *dev = &spi->dev;
	struct nrf52_hci *nrf;
	struct hci_dev *hdev;
	int ret;

	nrf = devm_kzalloc(dev, sizeof(*nrf), GFP_KERNEL);
	if (!nrf)
		return -ENOMEM;

	nrf->rxbuf = devm_kzalloc(dev, NRF52_RX_LEN, GFP_KERNEL);
	nrf->txbuf = devm_kzalloc(dev, NRF52_TX_MAX_FRAME, GFP_KERNEL);
	if (!nrf->rxbuf || !nrf->txbuf)
		return -ENOMEM;

	nrf->spi = spi;
	mutex_init(&nrf->xfer_lock);
	skb_queue_head_init(&nrf->txq);
	INIT_WORK(&nrf->tx_work, nrf52_tx_work);
	INIT_DELAYED_WORK(&nrf->rx_work, nrf52_rx_work);
	INIT_DELAYED_WORK(&nrf->storm_work, nrf52_storm_work);
	init_waitqueue_head(&nrf->line_wq);
	atomic_set(&nrf->releases, 0);

	nrf->irq_gpio = devm_gpiod_get(dev, "irq", GPIOD_IN);
	if (IS_ERR(nrf->irq_gpio))
		return dev_err_probe(dev, PTR_ERR(nrf->irq_gpio),
				     "no irq gpio\n");
	if (gpiod_cansleep(nrf->irq_gpio))
		return dev_err_probe(dev, -EINVAL,
				     "irq gpio must not sleep\n");

	nrf->irq = gpiod_to_irq(nrf->irq_gpio);
	if (nrf->irq < 0)
		return dev_err_probe(dev, nrf->irq, "irq gpio has no irq\n");

	hdev = hci_alloc_dev();
	if (!hdev)
		return -ENOMEM;

	nrf->hdev = hdev;
	hdev->bus = HCI_SPI;
	hci_set_drvdata(hdev, nrf);
	SET_HCIDEV_DEV(hdev, dev);
	hdev->open = nrf52_open;
	hdev->close = nrf52_close;
	hdev->flush = nrf52_flush;
	hdev->setup = nrf52_setup;
	hdev->send = nrf52_send;

	spi_set_drvdata(spi, nrf);

	/* left disabled until the device is opened: no SPI traffic before */
	set_bit(NRF52_IRQ_OFF, &nrf->flags);
	ret = request_irq(nrf->irq, nrf52_irq,
			  IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING |
			  IRQF_NO_AUTOEN,
			  "hci_nrf52_spi", nrf);
	if (ret) {
		hci_free_dev(hdev);
		return dev_err_probe(dev, ret, "failed to request irq\n");
	}

	ret = hci_register_dev(hdev);
	if (ret < 0) {
		nrf52_stop(nrf);
		free_irq(nrf->irq, nrf);
		hci_free_dev(hdev);
		return ret;
	}

	return 0;
}

static void nrf52_remove(struct spi_device *spi)
{
	struct nrf52_hci *nrf = spi_get_drvdata(spi);

	hci_unregister_dev(nrf->hdev);	/* closes the device if it was open */
	/* also when the core skipped .close() (HCI_UP already clear) */
	nrf52_stop(nrf);
	free_irq(nrf->irq, nrf);
	hci_free_dev(nrf->hdev);
}

static const struct of_device_id nrf52_of_match[] = {
	{ .compatible = "mikrotik,nrf52-hci-spi" },
	{ }
};
MODULE_DEVICE_TABLE(of, nrf52_of_match);

static const struct spi_device_id nrf52_spi_ids[] = {
	{ "nrf52-hci-spi" },
	{ }
};
MODULE_DEVICE_TABLE(spi, nrf52_spi_ids);

static struct spi_driver nrf52_driver = {
	.driver = {
		.name = "hci_nrf52_spi",
		.of_match_table = nrf52_of_match,
	},
	.id_table = nrf52_spi_ids,
	.probe = nrf52_probe,
	.remove = nrf52_remove,
};
module_spi_driver(nrf52_driver);

MODULE_AUTHOR("Willian G. Silva <sk4rfx@proton.me>");
MODULE_DESCRIPTION("Bluetooth HCI over SPI for MikroTik nRF52 controllers");
MODULE_LICENSE("GPL");
