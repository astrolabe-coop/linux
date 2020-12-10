/*
 * GSM MODEM initialization driver
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <asm/uaccess.h>
#include <linux/device.h>
#include <linux/timer.h>
#include <linux/platform_device.h>
#include <linux/init.h>
#include <linux/smp.h>



typedef struct gsm_pin {
        int    gpio;
        bool   active_high;
} gsm_pin_t;


typedef struct gsm_power_manager {
        gsm_pin_t usben;
        gsm_pin_t rst;
        gsm_pin_t pwrkey;
        gsm_pin_t vgsm;
        /* state */
        bool      enabled;      
} gsm_power_t;



static void gsm_turn_on( gsm_power_t *gsm_data ) {
        printk (KERN_INFO "davide %s %d\n", __func__, __LINE__ );
        if ( gsm_data->enabled == false ) {
                printk (KERN_INFO "davide %s %d\n", __func__, __LINE__ );
                // gpio_direction_output( gsm_data->pwrkey.gpio, (int)gsm_data->pwrkey.active_high );
                // mdelay( 30 );
                gpio_direction_output( gsm_data->pwrkey.gpio, ! (int)gsm_data->pwrkey.active_high );
                mdelay( 700 );
                gpio_direction_output( gsm_data->pwrkey.gpio, (int)gsm_data->pwrkey.active_high );
        }
        gsm_data->enabled = true;
}


static void gsm_turn_off( gsm_power_t *gsm_data ) {
        printk (KERN_INFO "davide %s %d\n", __func__, __LINE__ );
        if ( gsm_data->enabled == true ) {
                printk (KERN_INFO "davide %s %d\n", __func__, __LINE__ );
                // gpio_direction_output( gsm_data->pwrkey.gpio, (int)gsm_data->pwrkey.active_high );
                // mdelay( 30 );
                gpio_direction_output( gsm_data->pwrkey.gpio, ! (int)gsm_data->pwrkey.active_high );
                mdelay( 850 );
                gpio_direction_output( gsm_data->pwrkey.gpio, (int)gsm_data->pwrkey.active_high );
        }
        gsm_data->enabled = false;
}


static void gsm_reset( gsm_power_t *gsm_data ) {
        if ( gsm_data->enabled == false ) {
                gpio_direction_output( gsm_data->rst.gpio, (int)gsm_data->rst.active_high );
                mdelay( 30 );
                gpio_direction_output( gsm_data->rst.gpio, ! (int)gsm_data->rst.active_high );
                mdelay( 800 );
                gpio_direction_output( gsm_data->rst.gpio, (int)gsm_data->rst.active_high );
        }
}



static ssize_t sys_get_gsm_pwr_enable (struct device *dev, struct device_attribute *attr,
	   								char *buf)
{
        gsm_power_t *gsm_data = (gsm_power_t *)dev_get_drvdata( dev );
        if ( !gsm_data ) {
                return -ENODEV;        
        }

	return sprintf( buf, "%sX\n", gsm_data->enabled == true ? "enable" : "disable" );
}


static ssize_t sys_set_gsm_pwr_enable ( struct device *dev,
                struct device_attribute *attr, const char *buf, size_t count ) 
{
        int            rc = count;
        unsigned long  enable = 0;
        gsm_power_t    *gsm_data = (gsm_power_t *)dev_get_drvdata( dev );

        if ( gsm_data ) {
                rc = kstrtoul ( buf, 0, &enable );
                if ( enable != 0 ) {
                        gsm_turn_on( gsm_data );
                } else {
                        gsm_turn_off( gsm_data );
                }
        }
        return count;
}


static DEVICE_ATTR( enable, S_IRUGO | S_IWUSR, sys_get_gsm_pwr_enable, sys_set_gsm_pwr_enable );


static ssize_t sys_set_gsm_reset ( struct device *dev,
                struct device_attribute *attr, const char *buf, size_t count ) 
{
        int            rc = count;
        unsigned long  enable = 0;
        gsm_power_t    *gsm_data = (gsm_power_t *)dev_get_drvdata( dev );

        if ( gsm_data ) {
                rc = kstrtoul ( buf, 0, &enable );
                if ( enable != 0 ) {
                        gsm_reset( gsm_data );
                }
        }
        return count;
}


static DEVICE_ATTR( reset, S_IWUSR, NULL, sys_set_gsm_reset );


static struct attribute *sysfs_attrs[] = {
	&dev_attr_enable.attr,
        &dev_attr_reset.attr,
	NULL,
};


static struct attribute_group sysfs_attr_group = {
	.attrs = sysfs_attrs,
};



int gsm_pwr_mng_config_of( struct platform_device *pdev, struct device_node *node, gsm_power_t *gsm_data ) {

	int retval;

	/* request reset pin ( mandatory ) */
        gsm_data->rst.gpio = of_get_named_gpio( node, "rst-gpios", 0 );
        if ( ! gpio_is_valid( gsm_data->rst.gpio ) ) {
                dev_err( &pdev->dev, "no modem reset pin available gpios = %d\n", gsm_data->rst.gpio );
                return -EINVAL;
        }
        retval = gpio_request( gsm_data->rst.gpio, "gsm-modem_reset" );
        if ( retval < 0 ) {
                dev_err( &pdev->dev, "error during request of reset gpio (err %d)\n", retval );
                goto err_request_rst;
        }
        gsm_data->rst.active_high = ! of_property_read_bool( node, "rst-active-low" );
	
	
	/* request pwrkey pin ( mandatory ) */
        gsm_data->pwrkey.gpio = of_get_named_gpio( node, "pwrkey-gpios", 0 );
        if ( ! gpio_is_valid( gsm_data->pwrkey.gpio ) ) {
                dev_err( &pdev->dev, "no modem pwrkey pin available gpios = %d\n", gsm_data->pwrkey.gpio );
                return -EINVAL;
        }
        retval = gpio_request( gsm_data->pwrkey.gpio, "gsm-modem_pwrkey" );
        if ( retval < 0 ) {
                dev_err( &pdev->dev, "error during request of pwrkey gpio (err %d)\n", retval );
                goto err_request_pwr;
        }
        gsm_data->pwrkey.active_high = ! of_property_read_bool( node, "pwrkey-active-low" );

	
	/* request usben pin (optional )*/
        gsm_data->usben.gpio = of_get_named_gpio( node, "usben-gpios", 0 );
        if ( ! gpio_is_valid( gsm_data->usben.gpio ) ) {
                dev_info( &pdev->dev, "no modem usben pin not used\n" );
        } else {
                retval = gpio_request( gsm_data->usben.gpio, "gsm-modem_usben" );
                if ( retval < 0 ) {
                        dev_err( &pdev->dev, "error during request of usben gpio (err %d)\n", retval );
                        goto err_request_usben;
                }
        }
        gsm_data->usben.active_high = ! of_property_read_bool( node, "usben-active-low" );


        /* request vgsm pin (optional )*/
        gsm_data->vgsm.gpio = of_get_named_gpio( node, "vgsm-gpios", 0 );
        if ( ! gpio_is_valid( gsm_data->vgsm.gpio ) ) {
                dev_info( &pdev->dev, "no modem vgsm pin not used\n" );
        } else {
                retval = gpio_request( gsm_data->vgsm.gpio, "gsm-modem_vgsm" );
                if ( retval < 0 ) {
                        dev_err( &pdev->dev, "error during request of vgsm gpio (err %d)\n", retval );
                        goto err_request_vgsm;
                }
        }
        gsm_data->vgsm.active_high = ! of_property_read_bool( node, "vgsm-active-low" );      

        return 0;
err_request_vgsm:
        gpio_free( gsm_data->usben.gpio );
err_request_usben:
        gpio_free( gsm_data->pwrkey.gpio );
err_request_pwr:
        gpio_free( gsm_data->rst.gpio );
err_request_rst:
        return retval;
}


int gsm_pwr_mng_probe( struct platform_device *pdev ) {

        gsm_power_t *gsm_data;
        int err = 0;
	struct device_node *np = pdev->dev.of_node;
       
        dev_info( &pdev->dev, "gsm modem probing...\n" );
       
	if( ! np ) {
                dev_err( &pdev->dev, "no dts config found\n" );
		return -ENODEV;	
	}

        gsm_data = devm_kzalloc( &pdev->dev, sizeof(*gsm_data), GFP_KERNEL );
        if ( ! gsm_data ) {
                dev_err( &pdev->dev, "is not possible to allocate resources\n" );
		return -ENOMEM;
        }

        /* DTS init */
	if ( gsm_pwr_mng_config_of( pdev, np, gsm_data ) < 0 ) {
                dev_err( &pdev->dev, "no gsm-modem dts config found - probe failed\n" );
                err = -EINVAL;
                goto err_read_of;
	}
        dev_set_drvdata( &pdev->dev, gsm_data );

        /* Data init */
        gsm_data->enabled = false;

        err = sysfs_create_group( &pdev->dev.kobj, &sysfs_attr_group );

        return 0;
err_read_of:
        kfree( gsm_data );
	return err;

}

static int gsm_pwr_mng_remove( struct platform_device *pdev ) 
{
        // if ( gpio_is_valid(vgsm_gpio) )
	//         gpio_free(vgsm_gpio);
        // if ( gpio_is_valid(usben_gpio) )
	//         gpio_free(usben_gpio);
	// gpio_free(pwrkey_gpio);
	// gpio_free(rst_gpio);
        return 0;
}

static void gsm_pwr_mng_shutdown(struct platform_device *pdev)
{
        // if ( gpio_is_valid(vgsm_gpio) )
	//         gpio_free(vgsm_gpio);
        // if ( gpio_is_valid(usben_gpio) )
        //         gpio_free(usben_gpio);
        // gpio_free(pwrkey_gpio);
        // gpio_free(rst_gpio);

        // pr_info("gsm-modem exit\n");
 
}

static const struct of_device_id gsm_pwr_mng_dt_ids[] = {
        { .compatible = "seco,gsm-pwr-mng", },
        { /*sentinel */ }
};
MODULE_DEVICE_TABLE(of, gsm_pwr_mng_dt_ids);


static struct platform_driver gsm_pwr_mng_driver = {
        .probe          = gsm_pwr_mng_probe,
        .remove         = gsm_pwr_mng_remove,
        .shutdown       = gsm_pwr_mng_shutdown,
        .driver         = {
                .name   = "gsm-modem",
                .of_match_table = gsm_pwr_mng_dt_ids,
        },
};
module_platform_driver(gsm_pwr_mng_driver);

MODULE_ALIAS("platform:gsm_power_manager");
MODULE_AUTHOR("Davide Cardillo <davide.cardillo@seco.com>");
MODULE_DESCRIPTION("GSM Modem power manager Driver");
MODULE_LICENSE("GPL");

