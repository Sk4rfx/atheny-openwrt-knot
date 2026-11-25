# SPDX-License-Identifier: GPL-2.0-only
#
# Bluetooth HCI over SPI for the nRF52 on MikroTik boards (KNOT r2).

include $(TOPDIR)/rules.mk
include $(INCLUDE_DIR)/kernel.mk

PKG_NAME:=hci-nrf52-spi
PKG_RELEASE:=1
PKG_LICENSE:=GPL-2.0-only
PKG_MAINTAINER:=Willian G. Silva <sk4rfx@proton.me>

include $(INCLUDE_DIR)/package.mk

define KernelPackage/hci-nrf52-spi
  SUBMENU:=Other modules
  TITLE:=Bluetooth HCI over SPI for MikroTik nRF52 controllers
  DEPENDS:=@TARGET_ath79 +kmod-bluetooth
  FILES:=$(PKG_BUILD_DIR)/hci_nrf52_spi.ko
  AUTOLOAD:=$(call AutoProbe,hci_nrf52_spi)
endef

define KernelPackage/hci-nrf52-spi/description
  H4 transport over SPI for the Nordic nRF52832 Bluetooth LE controller
  running the vendor HCI firmware on MikroTik boards such as the KNOT r2.
endef

define Build/Compile
	$(KERNEL_MAKE) M="$(PKG_BUILD_DIR)" modules
endef

$(eval $(call KernelPackage,hci-nrf52-spi))
