/*
 *  Copyright (C) 2014, Daeho Choi <daeho.choi@lge.com>
 *  Driver for lg adc
 *
 *  This program is free software; you can redistribute it and/or modify it
 *  under  the terms of the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the License, or (at your
 *  option) any later version.
 *
 *
 */
#define pr_fmt(fmt) "%s:%s " fmt, KBUILD_MODNAME, __func__

#include <linux/device.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <soc/qcom/lge/power/lge_power_class.h>
#include <linux/slab.h>
#include <linux/of.h>
#include <linux/qpnp/qpnp-adc.h>
#include <linux/wakelock.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/workqueue.h>

#define DEFAULT_TEMP		250

struct lge_adc {
	struct device *dev;
	struct lge_power lge_adc_lpc;
	struct power_supply *fuelgauge_psy;
	struct power_supply *ac_psy;
	struct power_supply *battery_psy;
	struct power_supply *usb_psy;
	struct power_supply *bms_psy;
	struct power_supply *wlc_psy;
	struct qpnp_vadc_chip *vadc_dev;
	uint32_t xo_therm_channel;
	uint32_t pa_therm_channel;
	uint32_t batt_therm_channel;
	uint32_t usb_id_channel;
	int status;
};

static enum lge_power_property lge_power_adc_properties[] = {
	LGE_POWER_PROP_STATUS,
	LGE_POWER_PROP_XO_THERM_PHY,
	LGE_POWER_PROP_XO_THERM_RAW,
	LGE_POWER_PROP_BATT_THERM_PHY,
	LGE_POWER_PROP_BATT_THERM_RAW,
	LGE_POWER_PROP_USB_ID_PHY,
	LGE_POWER_PROP_USB_ID_RAW,
	LGE_POWER_PROP_PA_THERM_PHY,
	LGE_POWER_PROP_PA_THERM_RAW,
	LGE_POWER_PROP_TEMP,
};


static int lge_power_adc_get_property(struct lge_power *lpc,
				enum lge_power_property lpp,
				union lge_power_propval *val)
{
	int ret_val = 0;

	struct qpnp_vadc_result results;
	int rc = 0;
	struct lge_adc *chip
			= container_of(lpc, struct lge_adc, lge_adc_lpc);

	switch (lpp) {
	case LGE_POWER_PROP_STATUS:
		val->intval = chip->status;
		break;

	case LGE_POWER_PROP_XO_THERM_PHY:
		if (!IS_ERR(chip->vadc_dev)) {
			if (chip->xo_therm_channel != 0xFF) {
				rc = qpnp_vadc_read(chip->vadc_dev,
					chip->xo_therm_channel, &results);
			} else {
				pr_err("[LG_CHG] VADC is not used for \
						XO_THERM!!!\n");
				rc = -1;
			}
			if (rc) {
				pr_err("[LG_CHG] Unable to read \
						xo temperature adc!!!\n");
				ret_val = -EINVAL;
			} else {
				val->int64val = results.physical;
			}
		} else {
			val->intval = 0;
		}
		break;

	case LGE_POWER_PROP_XO_THERM_RAW:
		if (!IS_ERR(chip->vadc_dev)) {
			if (chip->xo_therm_channel != 0xFF) {
				rc = qpnp_vadc_read(chip->vadc_dev,
				chip->xo_therm_channel, &results);
			} else {
				pr_err("[LG_CHG] VADC is not used for XO_THERM!!!\n");
				rc = -1;
			}
			if (rc) {
				pr_err("[LG_CHG] Unable to read xo temperature adc!!!\n");
				ret_val = -EINVAL;
			} else {
				val->int64val = results.adc_code;
			}
		} else {
			val->intval = 0;
		}
		break;

	case LGE_POWER_PROP_BATT_THERM_PHY:
		if (!IS_ERR(chip->vadc_dev)) {
			if (chip->batt_therm_channel != 0xFF) {
				rc = qpnp_vadc_read(chip->vadc_dev,
					chip->batt_therm_channel, &results);
			} else {
				pr_err("[LG_CHG] VADC is not used for BATT_THERM!!!\n");
				rc = -1;
			}
			if (rc) {
				pr_err("[LG_CHG] Unable to read batt temperature adc!!!\n");
				ret_val = -EINVAL;
			} else {
				val->int64val = results.physical;
			}
		} else {
			val->intval = 0;
		}
		break;

	case LGE_POWER_PROP_BATT_THERM_RAW:
		if (!IS_ERR(chip->vadc_dev)) {
			if (chip->batt_therm_channel != 0xFF) {
				rc = qpnp_vadc_read(chip->vadc_dev,
					chip->batt_therm_channel, &results);
			} else {
				pr_err("[LG_CHG] VADC is not used for BATT_THERM!!!\n");
				rc = -1;
			}
			if (rc) {
				pr_err("[LG_CHG] Unable to read battery \
						temperature adc!!!\n");
				ret_val = -EINVAL;
			} else {
				val->int64val = results.adc_code;
			}
		} else {
			val->intval = 0;
		}
		break;
	case LGE_POWER_PROP_USB_ID_PHY:
		if (!IS_ERR(chip->vadc_dev)) {
			if (chip->usb_id_channel != 0xFF) {
				pr_err("[LG_CHG] USB ID channel : %d!!\n",
					chip->usb_id_channel);
				rc = qpnp_vadc_read(chip->vadc_dev,
					chip->usb_id_channel, &results);
			} else {
				pr_err("[LG_CHG] VADC is not used for \
						USB_ID!!!\n");
				rc = -1;
			}
			if (rc) {
				pr_err("[LG_CHG] Unable to read usb id adc! rc : %d!!\n", rc);
				ret_val = -EINVAL;
			} else {
				val->int64val = results.physical;
			}
		} else {
			val->intval = 0;
		}
		break;

	case LGE_POWER_PROP_USB_ID_RAW:
		if (!IS_ERR(chip->vadc_dev)) {
			if (chip->usb_id_channel != 0xFF) {
				rc = qpnp_vadc_read(chip->vadc_dev,
					chip->usb_id_channel, &results);
			} else {
				pr_err("[LG_CHG] VADC is not used for USB_ID!!!\n");
				rc = -1;
			}
			if (rc) {
				pr_err("[LG_CHG] Unable to read usb id adc!!! rc : %d!!\n",
						rc);
				ret_val = -EINVAL;
			} else {
				val->int64val = results.adc_code;
			}
		} else {
			val->intval = 0;
		}
		break;

	case LGE_POWER_PROP_PA_THERM_PHY:
		if (!IS_ERR(chip->vadc_dev)) {
			if (chip->pa_therm_channel != 0xFF) {
				rc = qpnp_vadc_read(chip->vadc_dev,
					chip->pa_therm_channel, &results);
			} else {
				pr_err("[LG_CHG] VADC is not used for \
						PA_THERM!!!\n");
				rc = -1;
			}
			if (rc) {
				pr_err("[LG_CHG] Unable to read \
					pa temperature adc!!!\n");
				ret_val = -EINVAL;
			} else {
				val->int64val = results.physical;
			}
		} else {
			val->intval = 0;
		}
		break;

	case LGE_POWER_PROP_PA_THERM_RAW:
		if (!IS_ERR(chip->vadc_dev)) {
			if (chip->pa_therm_channel != 0xFF) {
				rc = qpnp_vadc_read(chip->vadc_dev,
					chip->pa_therm_channel, &results);
			} else {
				pr_err("[LG_CHG] VADC is not used for \
						PA_THERM!!!\n");
				rc = -1;
			}
			if (rc) {
				pr_err("[LG_CHG] Unable to read \
				pa temperature adc!!!\n");
				ret_val = -EINVAL;
			} else {
				val->int64val = results.adc_code;
			}
		} else {
				val->intval = 0;
		}
		break;


	default:
		ret_val = -EINVAL;
		break;
	}

	return ret_val;
}

static int lge_adc_qct_probe(struct platform_device *pdev)
{
	struct lge_adc *lge_adc_chip;
	struct lge_power *lge_power_adc;
	int ret;

	pr_info("[LGE_CHG] lge_battery_probe starts!!!\n");

	lge_adc_chip = kzalloc(sizeof(struct lge_adc),
						GFP_KERNEL);
	if (!lge_adc_chip) {
		dev_err(&pdev->dev, "Failed to alloc driver structure\n");
		return -ENOMEM;
	}

	lge_adc_chip->vadc_dev = qpnp_get_vadc(&pdev->dev, "lge_adc_qct");
	if (IS_ERR(lge_adc_chip->vadc_dev)) {
		ret = PTR_ERR(lge_adc_chip->vadc_dev);
		if (ret != -EPROBE_DEFER)
			pr_err("[LGE_CHG] vadc property missing\n");
		else
			pr_err("[LGE_CHG] probe defer due to not initializing \
					vadc\n");
		goto err_free;
	}

	ret = of_property_read_u32(pdev->dev.of_node, "lge,xo_therm_chan",
				&lge_adc_chip->xo_therm_channel);

	ret = of_property_read_u32(pdev->dev.of_node, "lge,pa_therm_chan",
				&lge_adc_chip->pa_therm_channel);

	ret = of_property_read_u32(pdev->dev.of_node, "lge,batt_therm_chan",
				&lge_adc_chip->batt_therm_channel);

	ret = of_property_read_u32(pdev->dev.of_node, "lge,usb_id_chan",
				&lge_adc_chip->usb_id_channel);

	pr_info("[LG_CHG] XO_THERM_CH : %d,\
				PA_THERM_CH : %d,\
				BATT_THERM_CH : %d,\
				USB_ID_CH : %d\n",
				lge_adc_chip->xo_therm_channel,
				lge_adc_chip->pa_therm_channel,
				lge_adc_chip->batt_therm_channel,
				lge_adc_chip->usb_id_channel);

	lge_power_adc = &lge_adc_chip->lge_adc_lpc;

	lge_power_adc->name = "adc";

	lge_power_adc->properties = lge_power_adc_properties;
	lge_power_adc->num_properties = ARRAY_SIZE(lge_power_adc_properties);
	lge_power_adc->get_property = lge_power_adc_get_property;



	ret = lge_power_register(&pdev->dev, lge_power_adc);
	if (ret < 0) {
		pr_err("[LGE_CHG] Failed to register lge power class: %d\n",
		ret);
		goto err_free;
	}
	lge_adc_chip->dev = &pdev->dev;

	pr_info("[LGE_CHG] lge_battery_probe done!!!\n");
	return 0;
err_free:
	kfree(lge_adc_chip);
	return ret;
}

static int lge_adc_qct_remove(struct platform_device *pdev)
{
	struct lge_adc *lge_adc_chip = platform_get_drvdata(pdev);


	lge_power_unregister(&lge_adc_chip->lge_adc_lpc);

	platform_set_drvdata(pdev, NULL);
	kfree(lge_adc_chip);
	return 0;
}
#ifdef CONFIG_OF
static struct of_device_id lge_adc_qct_match_table[] = {
	{.compatible = "lge,adc_qct"},
	{},
};
#endif
static struct platform_driver lge_adc_qct_driver = {
	.probe = lge_adc_qct_probe,
	.remove = lge_adc_qct_remove,
	.driver = {
	.name = "lge_adc_qct",
	.owner = THIS_MODULE,
#ifdef CONFIG_OF
	.of_match_table = lge_adc_qct_match_table,
#endif
	},
};

static int __init lge_adc_qct_init(void)
{
	return platform_driver_register(&lge_adc_qct_driver);
}


static void __init lge_adc_qct_exit(void)
{
	platform_driver_unregister(&lge_adc_qct_driver);
}

module_init(lge_adc_qct_init);
module_exit(lge_adc_qct_exit);

MODULE_AUTHOR("Daeho Choi <daeho.choi@lge.com>");
MODULE_DESCRIPTION("Driver for LGE ADC driver for QCT");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:LGE ADC for QCT");
