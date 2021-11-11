ifeq ($(CONFIG_CPU_RK3399),y)
        DTC_FLAGS ?= -@ -H epapr
endif

dtb-$(CONFIG_ARCH_ROCKCHIP) += seco-rk3399-c31.dtb
dtb-$(CONFIG_ARCH_ROCKCHIP) += seco-px30-d23-revc.dtb
dtb-$(CONFIG_ARCH_ROCKCHIP) += seco-px30-d23-revc1.dtb

# dtb-$(CONFIG_ARCH_ROCKCHIP) += seco-px30-d23-revc-hdmi.dtb
# dtb-$(CONFIG_ARCH_ROCKCHIP) += seco-px30-d23-revc-lvds-800x480.dtb
# dtb-$(CONFIG_ARCH_ROCKCHIP) += seco-px30-d23-revc-lvds-1024x600.dtb
# dtb-$(CONFIG_ARCH_ROCKCHIP) += seco-px30-d23-revc-lvds-1280x800.dtb
# dtb-$(CONFIG_ARCH_ROCKCHIP) += seco-px30-d23-revc-hdmi-lvds.dtb
# dtb-$(CONFIG_ARCH_ROCKCHIP) += seco-px30-d23-revc-hdmi-lvds-1280x800.dtb

dts-dirs += overlays

always		:= $(dtb-y)
subdir-y	:= $(dts-dirs)
clean-files	:= *.dtb
