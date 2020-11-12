
#include <linux/module.h>
#include <linux/param.h>
#include <linux/err.h>
#include <linux/platform_device.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/of_regulator.h>
#include <linux/of_device.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/regmap.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>


#define REG_MANUFACTURER_ID   0x0
#define REG_FREQ              0x1
#define REG_SEL               0x2
#define REG_DCDCCTRL          0x3
#define REG_STATUS            0x4
#define REG_DCDC_SET          0x5

#define MASK_FREQ             (0x3 << 0)
#define SHIFT_FREQ            0
#define MASK_TSTEP            (0x3 << 2)
#define SHIFT_TSTEP           2

#define MASK_SEL              0x7F

#define MASK_EN               (0x1 << 1)
#define SHIFT_EN              1
#define MASK_PWM              (0x1 << 2)
#define SHIFT_PWM             2
#define MASK_DISCHARGE        (0x1 << 3)
#define SHIFT_DISCHARGE       3

#define MASK_STATUS_UV        (0x1 << 0)
#define MASK_STATUS_OT        (0x1 << 1)

#define MASK_VIDSET           (0x1 << 1)
#define MASK_PGDSET           (0x3 << 2)
#define MASK_OTSET            (0x3 << 4)
#define MASK_OCSET            (0x3 << 6)

#define RT5759_NVOLTAGES_90   90


struct rt57xx_platform_data {
        struct regulator_init_data *regulator;
        unsigned int slew_rate;
        /* Sleep VSEL ID */
      //  unsigned int sleep_vsel_id;
       // struct gpio_desc *vsel_gpio;
};



struct rt57xx_device_info {
	struct regmap                *regmap;
	struct device                *dev;
	struct regulator_desc        desc;
	struct regulator_dev         *rdev;
	struct regulator_init_data   *regulator;
	/* IC Type and Rev */
	int chip_id;
	int chip_rev;
	/* Voltage setting register */
	unsigned int                 vol_reg;
	unsigned int                 vol_mask;
	unsigned int                 ctrl_reg;
	unsigned int                 set_reg;
	unsigned int                 status_reg;
	unsigned int                 en_reg;
	unsigned int                 en_mask;
	unsigned int                 mode_reg;
	unsigned int                 mode_mask;
	unsigned int                 slew_reg;
	unsigned int                 slew_mask;
	unsigned int                 slew_shift;
	/* Voltage range and step(linear) */
	unsigned int                 fb_ratio;
	unsigned int                 vsel_min;
	unsigned int                 vsel_step;
	unsigned int                 n_voltages;
	/* Voltage slew rate limiting */
	unsigned int                 slew_rate;
	/* Sleep voltage cache */
	unsigned int                 sleep_vol_cache;
	struct gpio_desc             *vsel_gpio;
	unsigned int                 sleep_vsel_id;
};



// static inline set_voltage( struct rt57xx_device_info *di , int vsel ) {
// 	int ret;

// 	ret = regmap_update_bits( di->regmap, di->vsel_reg, di->vol_mask, ret);
// 	if ( ret < 0 )
// 		return ret;
// 	/* Cache the sleep voltage setting.
// 	 * Might not be the real voltage which is rounded */
// 	//di->sleep_vol_cache = uV;
// }




static unsigned int rt35xx_map_mode( unsigned int mode )
{
	return mode == REGULATOR_MODE_FAST ?
		REGULATOR_MODE_FAST : REGULATOR_MODE_NORMAL;
}


// static int rt35xx_set_suspend_disable( struct regulator_dev *rdev ) {
// 	struct rt57xx_device_info *di = rdev_get_drvdata(rdev);

// 	return regmap_update_bits(di->regmap, di->sleep_reg,
// 				  VSEL_BUCK_EN, 0);
// }


// static int rt35xx_set_enable( struct regulator_dev *rdev ) {
// 	struct rt57xx_device_info *di = rdev_get_drvdata(rdev);

// 	// return regmap_update_bits( di->regmap, di->vol_reg,
// 	// 			  VSEL_BUCK_EN, VSEL_BUCK_EN);
// }


// static int rt35xx_set_disable( struct regulator_dev *rdev ) {
// 	struct rt57xx_device_info *di = rdev_get_drvdata(rdev);

// 	// return regmap_update_bits(di->regmap, di->vol_reg,
// 	// 			  VSEL_BUCK_EN, VSEL_BUCK_EN);
// }




static int rt57xx_set_mode( struct regulator_dev *rdev, unsigned int mode ) {
	struct rt57xx_device_info *di = rdev_get_drvdata(rdev);

	switch (mode) {
	case REGULATOR_MODE_FAST:
		regmap_update_bits( di->regmap, di->mode_reg,
				   di->mode_mask, di->mode_mask );
		break;
	case REGULATOR_MODE_NORMAL:
		regmap_update_bits( di->regmap, di->mode_reg, di->mode_mask, 0 );
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static unsigned int rt57xx_get_mode( struct regulator_dev *rdev ) {
	struct rt57xx_device_info *di = rdev_get_drvdata(rdev);
	unsigned int val;
	int ret = 0;

	ret = regmap_read( di->regmap, di->mode_reg, &val );
	if (ret < 0)
		return ret;
	if (val & di->mode_mask)
		return REGULATOR_MODE_FAST;
	else
		return REGULATOR_MODE_NORMAL;
}


static const int slew_rates[] = {
	20000,
	15000,
	10000,
	 5000,
};


static int rt57xx_set_ramp( struct regulator_dev *rdev, int ramp ) {
	struct rt57xx_device_info *di = rdev_get_drvdata(rdev);
	int regval = -1, i;
	const int *slew_rate_t;
	int slew_rate_n;

	slew_rate_t = slew_rates;
	slew_rate_n = ARRAY_SIZE(slew_rates);

	for ( i = 0 ; i < slew_rate_n ; i++ ) {
		if ( ramp <= slew_rate_t[i] )
			regval = i;
		else
			break;
	}

	if (regval < 0) {
		dev_err(di->dev, "unsupported ramp value %d\n", ramp);
		return -EINVAL;
	}

	return regmap_update_bits(di->regmap, di->slew_reg,
				  di->slew_mask, regval << di->slew_shift);
}


static inline unsigned int rt57xx_step2microvolt( struct regulator_dev *rdev, unsigned sel ) {
	struct rt57xx_device_info *di = rdev_get_drvdata(rdev);
	return ( sel * di->vsel_step ) + di->vsel_min;
}


static inline unsigned rt57xx_microvolt2step( struct regulator_dev *rdev, unsigned int volt ) {
	struct rt57xx_device_info *di = rdev_get_drvdata(rdev);
	return ( volt - di->vsel_min ) / di->vsel_step;
}

static int rt57xx_set_voltage_sel_regmap( struct regulator_dev *rdev, unsigned sel ) {
	struct rt57xx_device_info *di = rdev_get_drvdata(rdev);
	int ret;
	unsigned int volt;

	volt = rt57xx_step2microvolt( rdev, sel );
	volt /= di->fb_ratio;
	volt *= 1000;
	sel = rt57xx_microvolt2step( rdev, volt );
	sel <<= ffs(rdev->desc->vsel_mask) - 1;
	ret = regmap_update_bits( rdev->regmap, rdev->desc->vsel_reg,
				  rdev->desc->vsel_mask, sel );
	if ( ret )
		return ret;

	if ( rdev->desc->apply_bit )
		ret = regmap_update_bits( rdev->regmap, rdev->desc->apply_reg,
					 rdev->desc->apply_bit,
					 rdev->desc->apply_bit );
	return ret;
}


static int rt57xx_get_voltage_sel_regmap( struct regulator_dev *rdev ) {
	struct rt57xx_device_info *di = rdev_get_drvdata(rdev);
	unsigned int val;
	int ret;
	unsigned int volt;

	ret = regmap_read( rdev->regmap, rdev->desc->vsel_reg, &val );
	if ( ret != 0 )
		return ret;

	val &= rdev->desc->vsel_mask;
	volt = rt57xx_step2microvolt( rdev, val );
	volt /= 1000;
	volt *= di->fb_ratio;
	val = rt57xx_microvolt2step( rdev, volt );
	val >>= ffs(rdev->desc->vsel_mask) - 1;
	return val;
}


static struct regulator_ops rt57xx_regulator_ops = {
	.set_voltage_sel = rt57xx_set_voltage_sel_regmap,
	.get_voltage_sel = rt57xx_get_voltage_sel_regmap,
	.set_voltage_time_sel = regulator_set_voltage_time_sel,
	.map_voltage = regulator_map_voltage_linear,
	.list_voltage = regulator_list_voltage_linear,
	// .set_suspend_voltage = fan53555_set_suspend_voltage,
	// .enable = fan53555_set_enable,
	// .disable = fan53555_set_disable,
	// .is_enabled = fan53555_is_enabled,
	.set_mode = rt57xx_set_mode,
	.get_mode = rt57xx_get_mode,
	.set_ramp_delay = rt57xx_set_ramp,
	// .set_suspend_enable = fan53555_set_suspend_enable,
	// .set_suspend_disable = fan53555_set_suspend_disable,
};


static int rt57xx_device_setup( struct rt57xx_device_info *di,
								struct rt57xx_platform_data *pdata )
{
	di->vol_reg    = REG_SEL;
	di->vol_mask   = MASK_SEL;
	di->vsel_min   = 600000;
	di->vsel_step  =  10000; 
	di->n_voltages = RT5759_NVOLTAGES_90;

	di->ctrl_reg   = REG_DCDCCTRL;
	di->set_reg    = REG_DCDC_SET;
	di->status_reg = REG_DCDC_SET;

	di->slew_reg   = REG_FREQ;
	di->slew_mask  = MASK_TSTEP;
	di->slew_shift = SHIFT_TSTEP;

	di->mode_reg   = di->ctrl_reg;
	di->mode_mask  = MASK_PWM;

	di->en_reg     = di->ctrl_reg;
	di->en_mask    = MASK_EN;
	
	return 0;
}


static int rt57xx_regulator_register( struct rt57xx_device_info *di,
			struct regulator_config *config )
{
	struct regulator_desc *rdesc = &di->desc;

	rdesc->name             = "rt57xx-reg";
	rdesc->supply_name      = "vin";
	rdesc->ops              = &rt57xx_regulator_ops;
	rdesc->type             = REGULATOR_VOLTAGE;
	rdesc->n_voltages       = di->n_voltages;
	rdesc->enable_reg       = di->en_reg;
	rdesc->enable_mask      = di->en_mask;
	rdesc->min_uV           = di->vsel_min;
	rdesc->uV_step          = di->vsel_step;
	rdesc->vsel_reg         = di->vol_reg;
	rdesc->vsel_mask        = di->vol_mask;
	rdesc->owner            = THIS_MODULE;
	rdesc->enable_time      = 400;

	di->rdev = devm_regulator_register( di->dev, &di->desc, config );
	return PTR_ERR_OR_ZERO( di->rdev );
}


static const struct regmap_config rt57xx_regmap_config = {
	.reg_bits     = 8,
	.val_bits     = 8,
	.max_register = REG_DCDC_SET,
	.cache_type   = REGCACHE_NONE, //REGCACHE_RBTREE,
};





static ssize_t sys_manufacturer_show (struct device *dev, struct device_attribute *attr,
	   								char *buf)
{
	struct rt57xx_device_info *di = (struct rt57xx_device_info *)dev_get_drvdata( dev );
	unsigned int val = 0;

	if ( !di ) {
		return sprintf( buf, "Data error\n" );
	}

	regmap_read( di->regmap, REG_MANUFACTURER_ID, &val );
	return sprintf( buf, "%02X\n", (uint8_t)val );
}


static ssize_t sys_dump_show (struct device *dev, struct device_attribute *attr,
	   								char *buf)
{
	struct rt57xx_device_info *di = (struct rt57xx_device_info *)dev_get_drvdata( dev );
	unsigned int val = 0;
	unsigned char msg[500];
	unsigned char tmp[50];
	int i;

	if ( !di ) {
		return sprintf( buf, "Data error\n" );
	}

	sprintf (msg, "\nReg\tvalue");
	for ( i = 0 ; i <= 5 ; i++ ) {
		regmap_read( di->regmap, i, &val );
		sprintf( tmp, "\n%02d\t0x%02X", i, val );
		strcat( msg, tmp );
	}

	regmap_read( di->regmap, REG_MANUFACTURER_ID, &val );
	sprintf (tmp, "\n%02d\t0x%04X", i, val);

	return sprintf (buf, "%s\n", msg);
}


static DEVICE_ATTR( manufacturer, S_IRUGO, sys_manufacturer_show, NULL );
static DEVICE_ATTR( dump_regs, S_IRUGO, sys_dump_show, NULL );



static struct attribute *sysfs_attrs[] = {
	&dev_attr_manufacturer.attr,
	&dev_attr_dump_regs.attr,
	NULL,
};


static struct attribute_group sysfs_attr_group = {
	.attrs = sysfs_attrs,
};




static struct rt57xx_platform_data *rt57xx_parse_dt(struct device *dev,
					      struct device_node *np,
						  struct rt57xx_device_info *di,
					      const struct regulator_desc *desc)
{
	struct rt57xx_platform_data *pdata;

    pdata = devm_kzalloc( dev, sizeof(*pdata), GFP_KERNEL );
	if ( !pdata )
		return NULL;

    pdata->regulator = of_get_regulator_init_data(dev, np, desc);
	pdata->regulator->constraints.initial_state = PM_SUSPEND_MEM;

	of_property_read_u32( np, "rate-offset-fb", &di->fb_ratio );

    return pdata;
}



static const struct of_device_id rt57xx_dt_ids[] = {
	{
		.compatible = "richtek,rt5759",
		.data       = (void *)0
	}, 
	{ }
};
MODULE_DEVICE_TABLE(of, rt57xx_dt_ids);


static int rt57xx_regulator_probe( struct i2c_client *client,
				const struct i2c_device_id *id )
{
    struct device_node             *np = client->dev.of_node;
	struct rt57xx_device_info      *di;
	struct rt57xx_platform_data    *pdata;
	struct regulator_config        config = { };
	unsigned int                   val;
    int                            ret;

    di = devm_kzalloc( &client->dev, sizeof(struct rt57xx_device_info), GFP_KERNEL );
	if (! di )
		return -ENOMEM;

    di->desc.of_map_mode = rt35xx_map_mode;

    pdata = rt57xx_parse_dt( &client->dev, np, di, &di->desc );

	if ( !pdata || !pdata->regulator ) {
		dev_err( &client->dev, "Platform data not found!\n" );
		return -ENODEV;
	}

    di->regulator = pdata->regulator;
	if ( client->dev.of_node ) {
		const struct of_device_id *match;

		match = of_match_device( of_match_ptr( rt57xx_dt_ids), &client->dev );
		if (!match)
			return -ENODEV;

	//	di->vendor = (unsigned long) match->data;
	} else {
		/* if no ramp constraint set, get the pdata ramp_delay */
		// if ( !di->regulator->constraints.ramp_delay ) {
		// 	int slew_idx = (p data->slew_rate & 0x7 ) ? pdata->slew_rate : 0;

		// 	di->regulator->constraints.ramp_delay = slew_rates[slew_idx];
		// }

		// di->vendor = id->driver_data;
	}

    di->regmap = devm_regmap_init_i2c( client, &rt57xx_regmap_config );
	if ( IS_ERR( di->regmap ) ) {
		dev_err( &client->dev, "Failed to allocate regmap!\n" );
		return PTR_ERR( di->regmap );
	}

	di->dev = &client->dev;
	
	/* Get chip ID */
	ret = regmap_read( di->regmap, REG_MANUFACTURER_ID, &val );
	if (ret < 0) {
		dev_err( &client->dev, "Failed to get chip ID!\n" );
		return ret;
	}
	di->chip_id = val & 0xFF;
	dev_info(&client->dev, "RT57xx with ID=%02X Detected!\n",	di->chip_id );

	/* Device init */
	ret = rt57xx_device_setup(di, pdata);
	if (ret < 0) {
		dev_err(&client->dev, "Failed to setup device!\n");
		return ret;
	}

    /* Register regulator */
	config.dev         = di->dev;
	config.init_data   = di->regulator;
	config.regmap      = di->regmap;
	config.driver_data = di;
	config.of_node     = np;

    ret = rt57xx_regulator_register( di, &config );
    if ( ret < 0 ) {
		dev_err( &client->dev, "Failed to register regulator!\n" ); 
		return ret;
	}

	ret = sysfs_create_group( &client->dev.kobj, &sysfs_attr_group );

	i2c_set_clientdata( client, di );
	dev_set_drvdata( &client->dev, di );

    return 0;
}


static const struct i2c_device_id rt57xx_id[] = {
	{
		.name        = "rt5759",
		.driver_data = 0
	},
	{ },
};
MODULE_DEVICE_TABLE(i2c, rt57xx_id);


static struct i2c_driver rt57xx_regulator_driver = {
	.driver = {
		.name           = "rt57xx-regulator",
		.of_match_table = of_match_ptr( rt57xx_dt_ids ),
	},
	.probe    = rt57xx_regulator_probe,
	.id_table = rt57xx_id,
};

module_i2c_driver(rt57xx_regulator_driver);


MODULE_AUTHOR("Davide Cardillo <davide.cardillo@seco.com>");
MODULE_DESCRIPTION("RT57xx regulator driver");
MODULE_LICENSE("GPL v2");
