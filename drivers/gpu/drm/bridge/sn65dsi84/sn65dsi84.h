/*
 * TI SN65DSI84 LVDS transmitter driver
 */

#ifndef __DRM_I2C_SN65DSI84_H__
#define __DRM_I2C_SN65DSI84_H__

#include <linux/i2c.h>
#include <linux/regmap.h>

#include <drm/drm_crtc_helper.h>
#include <drm/drm_mipi_dsi.h>

//#define SN65DSI84_REG_CHIP_REVISION                 0x00
#define SN65DSI84_REG_RESET                           0x09
#define SN65DSI84_REG_LVDS_CLK                        0x0a
#define SN65DSI84_REG_CLK_DIV_MUL                     0x0b
#define SN65DSI84_REG_PLL_EN                          0x0d
/* DSI REGISTERS */
#define SN65DSI84_REG_DSI_LANES                       0x10
#define SN65DSI84_REG_DSI_EQ                          0x11
#define SN65DSI84_REG_DSI_CLK_RANGE                   0x12
/* LVDS REGISTERS */
#define SN65DSI84_REG_LVDS_PARAMETER                  0x18
#define SN65DSI84_REG_LVDS_STRENGTH	                  0x19  //
#define SN65DSI84_REG_LVDS_SWAP                       0x1a  //
#define SN65DSI84_REG_LVDS_COMMON_MODE                0x1b  //
/* VIDEO REGISTERS */
#define SN65DSI84_REG_CHA_ACTIVE_LINE_LENGTH_LOW      0x20
#define SN65DSI84_REG_CHA_ACTIVE_LINE_LENGTH_HIGH     0x21
#define SN65DSI84_REG_CHA_VERTICAL_DISPLAY_SIZE_LOW	  0x24
#define SN65DSI84_REG_CHA_VERTICAL_DISPLAY_SIZE_HIGH  0x25
#define SN65DSI84_REG_CHA_SYNC_DELAY_LOW              0x28
#define SN65DSI84_REG_CHA_SYNC_DELAY_HIGH             0x29
#define SN65DSI84_REG_CHA_HSYNC_PULSE_WIDTH_LOW	      0x2c
#define SN65DSI84_REG_CHA_HSYNC_PULSE_WIDTH_HIGH      0x2d
#define SN65DSI84_REG_CHA_VSYNC_PULSE_WIDTH_LOW       0x30
#define SN65DSI84_REG_CHA_VSYNC_PULSE_WIDTH_HIGH      0x31
#define SN65DSI84_REG_CHA_HORIZONTAL_BACK_PORCH	      0x34
#define SN65DSI84_REG_CHA_VERTICAL_BACK_PORCH         0x36
#define SN65DSI84_REG_CHA_HORIZONTAL_FRONT_PORCH      0x38
#define SN65DSI84_REG_CHA_VERTICAL_FRONT_PORCH        0x3a
/* TEST PATTERN */
#define SN65DSI84_REG_CHA_TEST_PATTERN                0x3c
/* Enable */
#define SN65DSI84_REG_IRQ_EN                          0xe0
#define SN65DSI84_REG_ERR_EN                          0xe1
#define SN65DSI84_REG_DSI_PROTOCOL_ERR                0xe5




enum sn65dsi84_type {
	SN65DSI84,
};


struct sn65dsi84 {
	struct regmap           *regmap;

	struct videomode        *lvds_timing;
	const char              *lvds_color_depth;
	const char              *lvds_datamap;
	
	bool                    lvds_dual_channel;
	bool                    lvds_channel_swap;
	bool                    lvds_channel_reverse;
	bool                    lvds_test_mode;
	bool                    lvds_preserve_dsi_timings;
	u32                     mode_flags;
	
	struct drm_bridge       bridge;
	struct drm_connector    connector;

	struct gpio_desc        *pd_gpio;
	struct gpio_desc	    *bkl_gpio;
	struct gpio_desc 	    *lcd_gpio;

	struct device_node      *host_node;
	struct mipi_dsi_device  *dsi;
	u8                      num_dsi_lanes;

	enum sn65dsi84_type     type;
	

	struct drm_panel        *panel;
	u32 			 	    refclk_rate;
	struct clk              *refclk;
    struct device           *dev;
};


#endif /* __DRM_I2C_SN65DSI84_H__ */
