ifeq ($(CONFIG_CPU_RK3399),y)
        DTC_FLAGS ?= -@ -H epapr
endif

dtb-$(CONFIG_ARCH_ROCKCHIP) += seco-rk3399-c31.dtb
# dtb-$(CONFIG_ARCH_ROCKCHIP) += seco-px30-d23-reva-hdmi.dtb
# dtb-$(CONFIG_ARCH_ROCKCHIP) += seco-px30-d23-reva-lvds.dtb
# dtb-$(CONFIG_ARCH_ROCKCHIP) += seco-px30-d23-revb-hdmi.dtb
# dtb-$(CONFIG_ARCH_ROCKCHIP) += seco-px30-d23-revb-lvds.dtb
# dtb-$(CONFIG_ARCH_ROCKCHIP) += seco-px30-d23-revb-hdmi-lvds.dtb
dtb-$(CONFIG_ARCH_ROCKCHIP) += seco-px30-d23-revc-hdmi.dtb
dtb-$(CONFIG_ARCH_ROCKCHIP) += seco-px30-d23-revc-lvds.dtb
dtb-$(CONFIG_ARCH_ROCKCHIP) += seco-px30-d23-revc-hdmi-lvds.dtb

dts-dirs += overlays

always		:= $(dtb-y)
subdir-y	:= $(dts-dirs)
clean-files	:= *.dtb
