ifeq ($(CONFIG_CPU_RK3399),y)
        DTC_FLAGS ?= -@ -H epapr
endif

dtb-$(CONFIG_ARCH_ROCKCHIP) += seco-rk3399-c31.dtb
dtb-$(CONFIG_ARCH_ROCKCHIP) += seco-px30-d23-hdmi.dtb
dtb-$(CONFIG_ARCH_ROCKCHIP) += seco-px30-d23-lvds.dtb

dts-dirs += overlays

always		:= $(dtb-y)
subdir-y	:= $(dts-dirs)
clean-files	:= *.dtb
