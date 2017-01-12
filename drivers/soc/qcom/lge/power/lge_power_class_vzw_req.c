/* driver/soc/qcom/lge/power/lge_power_class_vzw_req.c
 *
 * LGE VZW requirement Driver.
 *
 * Copyright (C) 2015 LGE
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#define pr_fmt(fmt) "[LG_VZW] %s : " fmt, __func__

#include <linux/platform_device.h>
#include <linux/workqueue.h>
#include <linux/power_supply.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/mutex.h>
#ifdef CONFIG_LGE_PM_LGE_POWER_CLASS_CABLE_DETECT
#include <soc/qcom/lge/power/lge_power_class.h>
#include <soc/qcom/lge/power/lge_cable_detect.h>
#endif
#include <linux/of.h>
#include <linux/of_device.h>

#define LLK_MAX_THR_SOC 35
#define LLK_MIN_THR_SOC 30
typedef enum vzw_chg_state {
	VZW_NO_CHARGER,
	VZW_NORMAL_CHARGING,
	VZW_INCOMPATIBLE_CHARGING,
	VZW_UNDER_CURRENT_CHARGING,
	VZW_USB_DRIVER_UNINSTALLED,
	VZW_CHARGER_STATUS_MAX,
}chg_state;


struct lge_vzw{
	struct lge_power lge_vzw_lpc;
	struct lge_power *lge_cd_lpc;
	struct power_supply *batt_psy;
	int current_settled;
	int input_current_trim;
	int floated_chager;
	int store_demo_enabled;
	int capacity;
	int charging_enable;
	chg_state vzw_chg_mode;
	int under_chg_current;
	int chg_present;
	struct work_struct set_vzw_chg_work;
	struct work_struct llk_work;
};

static void lge_vzw_set_vzw_chg_work(struct work_struct *work)
{
	struct lge_vzw *chip = container_of(work, struct lge_vzw,
							set_vzw_chg_work);
	chg_state pre_vzw_chg_mode = chip->vzw_chg_mode;
	if (chip->floated_chager)
		chip->vzw_chg_mode = VZW_INCOMPATIBLE_CHARGING;
	else if (chip->chg_present)
		chip->vzw_chg_mode = VZW_NORMAL_CHARGING;
	else
		chip->vzw_chg_mode = VZW_NO_CHARGER;

	if (chip->current_settled > 0) {
		if (chip->input_current_trim < chip->under_chg_current) {
			chip->vzw_chg_mode = VZW_UNDER_CURRENT_CHARGING;
		}
	}

	if (pre_vzw_chg_mode != chip->vzw_chg_mode)
		lge_power_changed(&chip->lge_vzw_lpc);
}

static void lge_vzw_llk_work(struct work_struct *work)
{
	struct lge_vzw *chip = container_of(work, struct lge_vzw,
						llk_work);
	int prev_chg_enable = chip->charging_enable;

	if (chip->chg_present) {
		if (chip->capacity > LLK_MAX_THR_SOC) {
			chip->charging_enable = 0;
			pr_info("Stop charging by LLK_mode.\n");
		}
		if (chip->capacity < LLK_MIN_THR_SOC) {
			chip->charging_enable = 1;
			pr_info("Start Charging by LLK_mode.\n");
		}
		if (chip->charging_enable != prev_chg_enable) {
			pr_info("lge_power_changed in LLK_mode.\n");
			lge_power_changed(&chip->lge_vzw_lpc);
		}
	}
}


static char *vzw_req_supplied_from[] = {
	"battery",
};

static char *vzw_req_lge_supplied_from[] = {
	"cable_detect",
};

static char *vzw_req_supplied_to[] = {
	"battery",
	"rt5058-charger",
};

static void lge_vzw_external_power_changed(struct lge_power *lpc)
{
	int rc = 0;

	union power_supply_propval ret = {0,};
	struct lge_vzw *chip
			= container_of(lpc,
					struct lge_vzw, lge_vzw_lpc);

	int prev_input_current_trim = chip->input_current_trim;
	if (!chip->batt_psy)
		chip->batt_psy = power_supply_get_by_name("battery");

	if (!chip->batt_psy) {
		pr_err("%s : battery is not yet ready\n", __func__);
	} else {
		if (chip->current_settled >= 0) {
			rc = chip->batt_psy->get_property(chip->batt_psy,
					POWER_SUPPLY_PROP_INPUT_CURRENT_SETTLED, &ret);
			if (rc) {
				pr_info ("don't support AICL!\n");
				chip->current_settled = -1;
			} else {
				pr_info("input current settled : %d\n", ret.intval);
				chip->current_settled = ret.intval;
			}
		}
		if (chip->current_settled != -1) {
			rc = chip->batt_psy->get_property(chip->batt_psy,
				POWER_SUPPLY_PROP_INPUT_CURRENT_TRIM, &ret);
			if (rc) {
				pr_err ("cannot get input current trim!\n");
			} else {
				chip->input_current_trim = ret.intval;
				if (chip->input_current_trim
					!= prev_input_current_trim) {
					pr_info("input current trim : %d\n",
								chip->input_current_trim);
					schedule_work(&chip->set_vzw_chg_work);
				}
			}
		}
		if (chip->store_demo_enabled == 1) {
			rc = chip->batt_psy->get_property(chip->batt_psy,
					POWER_SUPPLY_PROP_CAPACITY, &ret);
			if (rc) {
				pr_err ("cannot get capacity!\n");
			} else {
				chip->capacity = ret.intval;
				pr_info("capacity : %d\n", chip->capacity);
				schedule_work(&chip->llk_work);
			}
		}
	}

}

static void lge_vzw_external_lge_power_changed(struct lge_power *lpc)
{
	int rc = 0;
	union lge_power_propval lge_val = {0,};
	struct lge_vzw *chip
			= container_of(lpc,
					struct lge_vzw, lge_vzw_lpc);
	static int prev_floated_chager = 0;
	static int prev_chg_present = 0;

	if (!chip->lge_cd_lpc)
		chip->lge_cd_lpc = lge_power_get_by_name("cable_detect");
	if (!chip->lge_cd_lpc) {
		pr_err("%s : cable_detect is not yet ready\n", __func__);
	} else {
		rc = chip->lge_cd_lpc->get_property(chip->lge_cd_lpc,
				LGE_POWER_PROP_CHG_PRESENT, &lge_val);
		chip->chg_present = lge_val.intval;

		rc = chip->lge_cd_lpc->get_property(chip->lge_cd_lpc,
				LGE_POWER_PROP_FLOATED_CHARGER, &lge_val);
		chip->floated_chager = lge_val.intval;
		if ((chip->floated_chager != prev_floated_chager)
				|| (prev_chg_present != chip->chg_present)){
			pr_info("floated charger : %d\n", chip->floated_chager);
			pr_info("chg_present : %d\n", chip->chg_present);
			schedule_work(&chip->set_vzw_chg_work);
		} else if (prev_chg_present != chip->chg_present) {
			pr_info("chg_present : %d\n", chip->chg_present);
			schedule_work(&chip->set_vzw_chg_work);
		}
		prev_floated_chager = chip->floated_chager;
		prev_chg_present =  chip->chg_present;
	}

}

static enum lge_power_property lge_power_lge_vzw_properties[] = {
	LGE_POWER_PROP_FLOATED_CHARGER,
	LGE_POWER_PROP_STORE_DEMO_ENABLED,
	LGE_POWER_PROP_VZW_CHG,
	LGE_POWER_PROP_CHARGING_ENABLED,
};

static enum lge_power_property
lge_power_lge_vzw_uevent_properties[] = {
	LGE_POWER_PROP_VZW_CHG,
};

static int
lge_power_lge_vzw_property_is_writeable(struct lge_power *lpc,
				enum lge_power_property lpp)
{
	int ret = 0;
	switch (lpp) {
	case LGE_POWER_PROP_STORE_DEMO_ENABLED:
		ret = 1;
		break;
	default:
		break;
	}
	return ret;
}

static int lge_power_lge_vzw_set_property(struct lge_power *lpc,
			enum lge_power_property lpp,
			const union lge_power_propval *val)
{
	int ret_val = 0;
	struct lge_vzw *chip
			= container_of(lpc,	struct lge_vzw, lge_vzw_lpc);

	switch (lpp) {
	case LGE_POWER_PROP_STORE_DEMO_ENABLED:
		chip->store_demo_enabled = val->intval;
		break;

	default:
		pr_info("Invalid VZW REQ property value(%d)\n", (int)lpp);
		ret_val = -EINVAL;
		break;
	}
	return ret_val;
}

static int lge_power_lge_vzw_get_property(struct lge_power *lpc,
			enum lge_power_property lpp,
			union lge_power_propval *val)
{
	int ret_val = 0;

	struct lge_vzw *chip
			= container_of(lpc, struct lge_vzw, lge_vzw_lpc);
	switch (lpp) {
	case LGE_POWER_PROP_FLOATED_CHARGER:
		val->intval = chip->floated_chager;
		break;

	case LGE_POWER_PROP_STORE_DEMO_ENABLED:
		val->intval = chip->store_demo_enabled;
		break;

	case LGE_POWER_PROP_VZW_CHG:
		val->intval = chip->vzw_chg_mode;
		break;

	case LGE_POWER_PROP_CHARGING_ENABLED:
		val->intval = chip->charging_enable;
		break;

	default:
		ret_val = -EINVAL;
		break;
	}

	return ret_val;
}
static int lge_vzw_probe(struct platform_device *pdev)
{
	struct lge_vzw *chip;
	struct lge_power *lge_power_vzw;
	int ret;
	pr_info("LG VZW requirement probe Start~!!\n");
	chip = kzalloc(sizeof(struct lge_vzw), GFP_KERNEL);

	if(!chip){
		pr_err("lge_vzw memory allocation failed.\n");
		return -ENOMEM;
	}
	ret = of_property_read_u32(pdev->dev.of_node, "lge,under_chg_current",
						&chip->under_chg_current);
	if (ret < 0)
		chip->under_chg_current = 500;
	chip->charging_enable = 1;
	chip->store_demo_enabled = 0;
	lge_power_vzw = &chip->lge_vzw_lpc;
	lge_power_vzw->name = "vzw";

	lge_power_vzw->properties = lge_power_lge_vzw_properties;
	lge_power_vzw->num_properties
		= ARRAY_SIZE(lge_power_lge_vzw_properties);
	lge_power_vzw->get_property
		= lge_power_lge_vzw_get_property;
	lge_power_vzw->set_property
		= lge_power_lge_vzw_set_property;
	lge_power_vzw->property_is_writeable
		= lge_power_lge_vzw_property_is_writeable;
	lge_power_vzw->supplied_to = vzw_req_supplied_to;
	lge_power_vzw->num_supplicants	= ARRAY_SIZE(vzw_req_supplied_to);
	lge_power_vzw->lge_supplied_from = vzw_req_lge_supplied_from;
	lge_power_vzw->supplied_from = vzw_req_supplied_from;
	lge_power_vzw->num_supplies	= ARRAY_SIZE(vzw_req_supplied_from);
	lge_power_vzw->lge_supplied_from = vzw_req_lge_supplied_from;
	lge_power_vzw->num_lge_supplies	= ARRAY_SIZE(vzw_req_lge_supplied_from);
	lge_power_vzw->external_lge_power_changed
		= lge_vzw_external_lge_power_changed;
	lge_power_vzw->external_power_changed
		= lge_vzw_external_power_changed;
	lge_power_vzw->uevent_properties
		= lge_power_lge_vzw_uevent_properties;
	lge_power_vzw->num_uevent_properties
		= ARRAY_SIZE(lge_power_lge_vzw_uevent_properties);
	ret = lge_power_register(&pdev->dev, lge_power_vzw);
	if (ret < 0) {
		pr_err("Failed to register lge power class: %d\n",
			ret);
		goto err_free;
	}
	INIT_WORK(&chip->set_vzw_chg_work, lge_vzw_set_vzw_chg_work);
	INIT_WORK(&chip->llk_work, lge_vzw_llk_work);
	pr_info("LG VZW probe done~!!\n");
	return 0;
err_free:
	kfree(chip);
	return ret;
}

static int lge_vzw_remove(struct platform_device *pdev)
{
	struct lge_vzw *chip = platform_get_drvdata(pdev);

	lge_power_unregister(&chip->lge_vzw_lpc);
	return 0;
}
#ifdef CONFIG_OF
static struct of_device_id lge_vzw_match_table[] = {
	{.compatible = "lge,vzw"},
	{},
};
#endif
static struct platform_driver vzw_driver = {
	.probe  = lge_vzw_probe,
	.remove = lge_vzw_remove,
	.driver = {
		.name = "lge_vzw",
		.owner = THIS_MODULE,
#ifdef CONFIG_OF
		.of_match_table = lge_vzw_match_table,
#endif
	},
};

static int __init lge_vzw_init(void)
{
	int rc;
	rc = platform_driver_register(&vzw_driver);

	return rc;
}

static void __exit lge_vzw_exit(void)
{
	platform_driver_unregister(&vzw_driver);
}

module_init(lge_vzw_init);
module_exit(lge_vzw_exit);

MODULE_DESCRIPTION("LGE VZW requirement driver");
MODULE_LICENSE("GPL");
