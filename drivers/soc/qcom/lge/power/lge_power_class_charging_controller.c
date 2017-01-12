/* Copyright (c) 2013-2014, LG Eletronics. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#define pr_fmt(fmt) "[LGCC] %s : " fmt, __func__


#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/of.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/wakelock.h>
#include <linux/power_supply.h>

#include <soc/qcom/lge/lge_charging_scenario.h>
#include <soc/qcom/lge/power/lge_power_class.h>
#ifdef CONFIG_LGE_PM_LGE_POWER_CLASS_CHARGER_SLEEP
#include <linux/alarmtimer.h>
#endif

#define MODULE_NAME "lge_charging_controller"
#define MONITOR_BATTEMP_POLLING_PERIOD  (60 * HZ)
#ifdef CONFIG_LGE_PM_LGE_POWER_CLASS_CHARGER_SLEEP
#define BTM_ALARM_TIME(DELAY) (DELAY##LL * NSEC_PER_SEC)
#define BTM_ALARM_PERIOD BTM_ALARM_TIME(60) /* 60sec */
#endif

struct lge_charging_controller{

	struct power_supply		*batt_psy;
	struct power_supply		*usb_psy;

	struct power_supply		*bms_psy;
	struct lge_power lge_cc_lpc;
	struct lge_power *lge_cd_lpc;

	struct delayed_work battemp_work;
	struct wake_lock lcs_wake_lock;
#ifndef CONFIG_LGE_PM_LGE_POWER_CLASS_CHARGER_SLEEP
	struct wake_lock chg_wake_lock;
#endif
	enum lge_charging_states battemp_chg_state;

	int chg_current_te;
	int chg_current_max;
	int otp_ibat_current;
	int pseudo_chg_ui;
	int before_battemp;
	int batt_temp;
	int btm_state;
	int start_batt_temp;
	int stop_batt_temp;
	int chg_enable;
	int test_chg_scenario;
	int test_batt_therm_value;
	int before_usb_present;
	int is_usb_present;
	int before_hvdcp_present;
	int is_hvdcp_present;
	int update_hvdcp_state;
#ifdef CONFIG_LGE_PM_LGE_POWER_CLASS_CHARGER_SLEEP
	struct power_supply	*chg_psy;
	struct alarm btm_polling_alarm;
	struct wake_lock alarm_wake_lock;
	u8 btm_alarm_enable:1;
	u8 pre_btm_alarm_enable:1;
	bool charger_eoc;
#endif
};

static enum lge_power_property lge_power_lge_cc_properties[] = {
	LGE_POWER_PROP_PSEUDO_BATT_UI,
	LGE_POWER_PROP_BTM_STATE,
	LGE_POWER_PROP_CHARGING_ENABLED,
	LGE_POWER_PROP_OTP_CURRENT,
	LGE_POWER_PROP_TEST_CHG_SCENARIO,
	LGE_POWER_PROP_TEST_BATT_THERM_VALUE,
	LGE_POWER_PROP_TEST_CHG_SCENARIO,
	LGE_POWER_PROP_TEST_BATT_THERM_VALUE,
};
static char *cc_supplied_to[] = {
	"battery",
	"rt5058-charger"
};

static struct lge_charging_controller *the_controller;

static int lgcc_thermal_mitigation;
ssize_t lgcc_set_thermal_chg_current(const char *val,
		struct kernel_param *kp){

	int ret;

	ret = param_set_int(val, kp);

	if (ret) {
		pr_err("error setting value %d\n", ret);
		return ret;
	}

	if (!the_controller) {
		pr_err("lgcc is not ready\n");
		return 0;
	}

	if (the_controller->is_hvdcp_present == 1) {
		pr_info("hvdcp is present.. skip this settings\n");
		return 0;
	}
	if (lgcc_thermal_mitigation <= 0)
		the_controller->chg_current_te
			= the_controller->chg_current_max;
	else if (lgcc_thermal_mitigation < 300)
		the_controller->chg_current_te = 0;
	else
		the_controller->chg_current_te = lgcc_thermal_mitigation;

	pr_info("lgcc_thermal_mitigation = %d, chg_current_te_te = %d\n",
			lgcc_thermal_mitigation,
			the_controller->chg_current_te);

	cancel_delayed_work_sync(&the_controller->battemp_work);
	schedule_delayed_work(&the_controller->battemp_work, HZ*1);

	return 0;
}
module_param_call(lgcc_thermal_mitigation,
		lgcc_set_thermal_chg_current,
		NULL, &lgcc_thermal_mitigation, 0644);

static int lgcc_hvdcp_thermal_mitigation;
ssize_t lgcc_set_hvdcp_thermal_chg_current(const char *val,
		struct kernel_param *kp){

	int ret;

	ret = param_set_int(val, kp);

	if (ret) {
		pr_err("error setting value %d\n", ret);
		return ret;
	}

	if (!the_controller) {
		pr_err("lgcc is not ready\n");
		return 0;
	}
	if (lgcc_hvdcp_thermal_mitigation <= 0)
		the_controller->chg_current_te
			= the_controller->chg_current_max;
	else if (lgcc_hvdcp_thermal_mitigation < 300)
		the_controller->chg_current_te = 0;
	else
		the_controller->chg_current_te = lgcc_hvdcp_thermal_mitigation;

	pr_info("lgcc_hvdcp_thermal_mitigation = %d, chg_current_te_te = %d\n",
			lgcc_hvdcp_thermal_mitigation,
			the_controller->chg_current_te);

	cancel_delayed_work_sync(&the_controller->battemp_work);
	schedule_delayed_work(&the_controller->battemp_work, HZ*1);

	return 0;
}
module_param_call(lgcc_hvdcp_thermal_mitigation,
		lgcc_set_hvdcp_thermal_chg_current,
		NULL, &lgcc_hvdcp_thermal_mitigation, 0644);
#ifdef CONFIG_LGE_PM_LGE_POWER_CLASS_CHARGER_SLEEP
static void lgcc_btm_set_polling_alarm(struct lge_charging_controller *chip, u64 delay)
{
	ktime_t kt = ns_to_ktime(delay);

	if (chip->btm_alarm_enable)
		alarm_start_relative(&chip->btm_polling_alarm, kt);
}

static enum alarmtimer_restart lgcc_btm_alarm(struct alarm *alarm, ktime_t kt)
{
	struct lge_charging_controller *chip = container_of(alarm,
			struct lge_charging_controller, btm_polling_alarm);

	if (chip->btm_alarm_enable)
		wake_lock(&chip->alarm_wake_lock);

	schedule_delayed_work(&chip->battemp_work, msecs_to_jiffies(50));

	return ALARMTIMER_NORESTART;
}
#endif
static void lge_monitor_batt_temp_work(struct work_struct *work){

	struct charging_info req;
	struct charging_rsp res;
	bool is_changed = false;
	union power_supply_propval ret = {0,};
#ifdef CONFIG_LGE_PM_LGE_POWER_CLASS_CABLE_DETECT
	union lge_power_propval lge_val = {0,};
#endif
	struct lge_charging_controller *chip =
			container_of(work, struct lge_charging_controller,
				battemp_work.work);

	chip->usb_psy = power_supply_get_by_name("usb");

	if(!chip->usb_psy){
		pr_err("usb power_supply not found deferring probe\n");
		schedule_delayed_work(&chip->battemp_work,
			MONITOR_BATTEMP_POLLING_PERIOD);
		return;
	}

	chip->batt_psy = power_supply_get_by_name("battery");

	if(!chip->batt_psy){
		pr_err("battery power_supply not found deferring probe\n");
		schedule_delayed_work(&chip->battemp_work,
			MONITOR_BATTEMP_POLLING_PERIOD);
		return;
	}

#ifdef CONFIG_LGE_PM_LGE_POWER_CLASS_CABLE_DETECT
	chip->lge_cd_lpc = lge_power_get_by_name("cable_detect");
	if (!chip->lge_cd_lpc) {
		pr_err("%s : lge_cd_lpc is not yet ready\n", __func__);
		schedule_delayed_work(&chip->battemp_work,
			MONITOR_BATTEMP_POLLING_PERIOD);
		return;
	}
#endif
	chip->batt_psy->get_property(chip->batt_psy,
			POWER_SUPPLY_PROP_TEMP, &ret);
	if (chip->test_chg_scenario == 1)
		req.batt_temp = chip->test_batt_therm_value;
	else
		req.batt_temp = ret.intval / 10;
	chip->batt_temp = req.batt_temp;

	chip->batt_psy->get_property(chip->batt_psy,
			POWER_SUPPLY_PROP_VOLTAGE_NOW, &ret);
	req.batt_volt = ret.intval;

	chip->batt_psy->get_property(chip->batt_psy,
			POWER_SUPPLY_PROP_CURRENT_NOW, &ret);
	req.current_now = ret.intval / 1000;
#ifdef CONFIG_LGE_PM_LGE_POWER_CLASS_CABLE_DETECT
	chip->lge_cd_lpc->get_property(chip->lge_cd_lpc,
			LGE_POWER_PROP_CHARGING_CURRENT_MAX, &lge_val);
	chip->chg_current_max = lge_val.intval / 1000;
#else
	chip->usb_psy->get_property(
			chip->usb_psy, POWER_SUPPLY_PROP_CURRENT_MAX, &ret);
	chip->chg_current_max = ret.intval / 1000;
#endif
	req.chg_current_ma = chip->chg_current_max;

	if (chip->chg_current_te != -EINVAL)
		req.chg_current_te = chip->chg_current_te;
	else
		req.chg_current_te = chip->chg_current_max;

	pr_debug("chg_current_max = %d\n", chip->chg_current_max);
	pr_debug("chg_curren_te = %d\n", chip->chg_current_te);
	chip->usb_psy->get_property(chip->usb_psy,
			POWER_SUPPLY_PROP_PRESENT, &ret);
	req.is_charger = ret.intval;

	lge_monitor_batt_temp(req, &res);

#ifdef CONFIG_LGE_PM_LGE_POWER_CLASS_CHARGER_SLEEP
	chip->batt_psy->get_property(chip->batt_psy,
			POWER_SUPPLY_PROP_CAPACITY, &ret);
	if (ret.intval == 100)
		chip->charger_eoc = 1;
	else
		chip->charger_eoc = 0;

	chip->lge_cd_lpc->get_property(chip->lge_cd_lpc,
			LGE_POWER_PROP_CHG_PRESENT, &lge_val);
	chip->btm_alarm_enable = (lge_val.intval && !chip->charger_eoc);
	pr_err("(%s)->(%s)\n",
			chip->pre_btm_alarm_enable ? "alarm":"work_queue",
			chip->btm_alarm_enable ? "alarm":"work_queue");
	chip->pre_btm_alarm_enable = chip->btm_alarm_enable;
#endif
	if (((res.change_lvl != STS_CHE_NONE) && req.is_charger) ||
			(res.force_update == true)) {
		if (res.change_lvl == STS_CHE_NORMAL_TO_DECCUR ||
				(res.state == CHG_BATT_DECCUR_STATE &&
				 res.dc_current != DC_CURRENT_DEF &&
				 res.change_lvl != STS_CHE_STPCHG_TO_DECCUR)) {
			pr_info("ibatmax_set STS_CHE_NORMAL_TO_DECCUR\n");
			chip->otp_ibat_current = res.dc_current;
			chip->chg_enable = 1;
			//lgcc_set_ibat_current(chip->otp_ibat_current);
			//lgcc_set_charging_enable(1);
			lge_power_changed(&chip->lge_cc_lpc);

		} else if (res.change_lvl == STS_CHE_NORMAL_TO_STPCHG ||
				(res.state == CHG_BATT_STPCHG_STATE)) {
			pr_info("ibatmax_set STS_CHE_NORMAL_TO_STPCHG\n");
#ifdef CONFIG_LGE_PM_LGE_POWER_CLASS_CHARGER_SLEEP
			if (!chip->btm_alarm_enable)
				wake_lock(&chip->lcs_wake_lock);
#else
			wake_lock(&chip->lcs_wake_lock);
#endif
			chip->otp_ibat_current = 0;
			chip->chg_enable = 0;
			//lgcc_set_ibat_current(chip->otp_ibat_current);
			//lgcc_set_charging_enable(0);
			lge_power_changed(&chip->lge_cc_lpc);
		} else if (res.change_lvl == STS_CHE_DECCUR_TO_NORAML) {
			pr_info("ibatmax_set STS_CHE_DECCUR_TO_NORAML\n");
			chip->otp_ibat_current = res.dc_current;
			chip->chg_enable = 1;
			//lgcc_set_ibat_current(chip->otp_ibat_current);
			//lgcc_set_charging_enable(1);
			lge_power_changed(&chip->lge_cc_lpc);
		} else if (res.change_lvl == STS_CHE_DECCUR_TO_STPCHG) {
			pr_info("ibatmax_set STS_CHE_DECCUR_TO_STPCHG\n");
#ifdef CONFIG_LGE_PM_LGE_POWER_CLASS_CHARGER_SLEEP
			if (!chip->btm_alarm_enable)
				wake_lock(&chip->lcs_wake_lock);
#else
			wake_lock(&chip->lcs_wake_lock);
#endif
			chip->otp_ibat_current = 0;
			chip->chg_enable = 0;
			//lgcc_set_ibat_current(chip->otp_ibat_current);
			//lgcc_set_charging_enable(0);
			lge_power_changed(&chip->lge_cc_lpc);
		} else if (res.change_lvl == STS_CHE_STPCHG_TO_NORMAL) {
			pr_info("ibatmax_set STS_CHE_STPCHG_TO_NORMAL\n");
#ifdef CONFIG_LGE_PM_LGE_POWER_CLASS_CHARGER_SLEEP
			if (!chip->btm_alarm_enable)
				wake_unlock(&chip->lcs_wake_lock);
#else
			wake_unlock(&chip->lcs_wake_lock);
#endif
			chip->otp_ibat_current = res.dc_current;
			chip->chg_enable = 1;
			//lgcc_set_ibat_current(chip->otp_ibat_current);
			//lgcc_set_charging_enable(1);
			lge_power_changed(&chip->lge_cc_lpc);
		} else if (res.change_lvl == STS_CHE_STPCHG_TO_DECCUR) {
			pr_info("ibatmax_set STS_CHE_STPCHG_TO_DECCUR\n");
			chip->otp_ibat_current = res.dc_current;
			chip->chg_enable = 1;
			//lgcc_set_ibat_current(chip->otp_ibat_current);
			//lgcc_set_charging_enable(1);
			lge_power_changed(&chip->lge_cc_lpc);
#ifdef CONFIG_LGE_PM_LGE_POWER_CLASS_CHARGER_SLEEP
			if (!chip->btm_alarm_enable)
				wake_unlock(&chip->lcs_wake_lock);
#else
			wake_unlock(&chip->lcs_wake_lock);
#endif

		} else if (res.force_update == true &&
				res.state == CHG_BATT_NORMAL_STATE &&
				res.dc_current != DC_CURRENT_DEF) {
			pr_info("ibatmax_set CHG_BATT_NORMAL_STATE\n");
			chip->otp_ibat_current = res.dc_current;
			chip->chg_enable = 1;
			//lgcc_set_ibat_current(chip->otp_ibat_current);
			//lgcc_set_charging_enable(1);
			lge_power_changed(&chip->lge_cc_lpc);
		}
	}

	if (chip->chg_current_te == 0) {
		pr_info("thermal_miti ibat < 300, stop charging!\n");
		chip->otp_ibat_current = 0;
		chip->chg_enable = 0;
		lge_power_changed(&chip->lge_cc_lpc);
	}

	pr_err("otp_ibat_current=%d\n", chip->otp_ibat_current);

	pr_debug("chip->pseudo_chg_ui = %d, res.pseudo_chg_ui = %d\n",
			chip->pseudo_chg_ui, res.pseudo_chg_ui);

	if (chip->pseudo_chg_ui ^ res.pseudo_chg_ui) {
		is_changed = true;
		chip->pseudo_chg_ui = res.pseudo_chg_ui;
	}

	pr_debug("chip->btm_state = %d, res.btm_state = %d\n",
			chip->btm_state, res.btm_state);
	if (chip->btm_state ^ res.btm_state) {
		is_changed = true;
		chip->btm_state = res.btm_state;
	}

	if (chip->before_battemp != req.batt_temp) {
		is_changed = true;
		chip->before_battemp = req.batt_temp;
	}

	if (chip->update_hvdcp_state == 1){
		is_changed = true;
		chip->update_hvdcp_state = 0;
	}

	chip->batt_psy->get_property(chip->batt_psy,
			POWER_SUPPLY_PROP_CAPACITY, &ret);
#ifndef CONFIG_LGE_PM_LGE_POWER_CLASS_CHARGER_SLEEP
	if ((!chip->is_hvdcp_present && !chip->is_usb_present)
			|| ((chip->is_hvdcp_present || chip->is_usb_present)
				&& ret.intval == 100)) {
		if (wake_lock_active(&chip->chg_wake_lock)) {
			pr_info("chg_wake_unlocked\n");
			wake_unlock(&chip->chg_wake_lock);
		}
	} else if((chip->is_hvdcp_present || chip->is_usb_present) && ret.intval < 100) {
		if (!wake_lock_active(&chip->chg_wake_lock)) {
			pr_info("chg_wake_locked\n");
			wake_lock(&chip->chg_wake_lock);
		}
	}
#endif
	if (is_changed == true)
		lge_power_changed(&chip->lge_cc_lpc);

	pr_info("Reported Capacity : %d / voltage : %d\n",
			ret.intval, req.batt_volt/1000);


//	lgcc_charger_reginfo();
#ifdef CONFIG_LGE_PM_LGE_POWER_CLASS_CHARGER_SLEEP
	if (chip->btm_alarm_enable) {
		lgcc_btm_set_polling_alarm(chip, BTM_ALARM_PERIOD);

		if (wake_lock_active(&chip->lcs_wake_lock))
			wake_unlock(&chip->lcs_wake_lock);
	}
	else {
		schedule_delayed_work(&the_controller->battemp_work,
				MONITOR_BATTEMP_POLLING_PERIOD);
	}

	if (wake_lock_active(&chip->alarm_wake_lock))
		wake_unlock(&chip->alarm_wake_lock);
#else
	schedule_delayed_work(&the_controller->battemp_work,
			MONITOR_BATTEMP_POLLING_PERIOD);
#endif
}

static int lg_cc_get_pseudo_ui(struct lge_charging_controller *chip){

	if( !(chip == NULL) ){
		return chip->pseudo_chg_ui;
	}
	return 0;
}

static int lg_cc_get_btm_state(struct lge_charging_controller *chip){

	if( !(chip == NULL) ){
		return chip->btm_state;
	}
	return 0;
}

#ifdef CONFIG_LGE_PM_LGE_POWER_CLASS_CHARGER_SLEEP
static void
lg_cc_start_battemp_alarm
(struct lge_charging_controller *chip, u64 delay){
	pr_debug("start_battemp_alarm~!!\n");

	if (!chip->btm_alarm_enable)
		chip->btm_alarm_enable = 1;

	lgcc_btm_set_polling_alarm(chip, delay);
}

static void
lg_cc_stop_battemp_alarm(struct lge_charging_controller *chip){
	pr_debug("stop_battemp_alarm~!!\n");

	if (chip->btm_alarm_enable)
		chip->btm_alarm_enable = 0;

	if (wake_lock_active(&chip->alarm_wake_lock))
		wake_unlock(&chip->alarm_wake_lock);
	cancel_delayed_work(&chip->battemp_work);
	alarm_cancel(&chip->btm_polling_alarm);
}
#endif
static void
lg_cc_start_battemp_work
(struct lge_charging_controller *chip, int delay){
	pr_debug("start_battemp_work~!!\n");
	schedule_delayed_work(&chip->battemp_work, (delay * HZ));
}


#ifndef CONFIG_LGE_PM_LGE_POWER_CLASS_CHARGER_SLEEP
static void
lg_cc_stop_battemp_work(struct lge_charging_controller *chip){
	pr_debug("stop_battemp_work~!!\n");
	cancel_delayed_work(&chip->battemp_work);
}
#endif

static int
lge_power_lge_cc_property_is_writeable(struct lge_power *lpc,
				enum lge_power_property lpp)
{
	int ret = 0;
	switch (lpp) {
	case LGE_POWER_PROP_TEST_CHG_SCENARIO:
	case LGE_POWER_PROP_TEST_BATT_THERM_VALUE:
		ret = 1;
		break;
	default:
		break;
	}
	return ret;
}
static int lge_power_lge_cc_set_property(struct lge_power *lpc,
				enum lge_power_property lpp,
				const union lge_power_propval *val)
{

	int ret_val = 0;
#ifdef TEMP_BLOCK
	union lge_power_propval ret = {0,};
#endif
	struct lge_charging_controller *chip
			= container_of(lpc,
				struct lge_charging_controller, lge_cc_lpc);

	switch (lpp) {
	case LGE_POWER_PROP_TEST_CHG_SCENARIO:
		chip->test_chg_scenario = val->intval;
		break;
	case LGE_POWER_PROP_TEST_BATT_THERM_VALUE:
		chip->test_batt_therm_value = val->intval;
		break;
	default:
		pr_err("[LGE_CHG] lpp:%d is not supported!!!\n", lpp);
		ret_val = -EINVAL;
		break;
	}
	lge_power_changed(&chip->lge_cc_lpc);
	return ret_val;
}

static int lge_power_lge_cc_get_property(struct lge_power *lpc,
			enum lge_power_property lpp,
			union lge_power_propval *val)
{
	int ret_val = 0;

	struct lge_charging_controller *chip
			= container_of(lpc, struct lge_charging_controller,
					lge_cc_lpc);
	switch (lpp) {
	case LGE_POWER_PROP_PSEUDO_BATT_UI:
		val->intval = lg_cc_get_pseudo_ui(chip);
		break;

	case LGE_POWER_PROP_BTM_STATE:
		val->intval = lg_cc_get_btm_state(chip);
		break;

	case LGE_POWER_PROP_CHARGING_ENABLED:
		val->intval = chip->chg_enable;
		break;
	case LGE_POWER_PROP_OTP_CURRENT:
		val->intval = chip->otp_ibat_current;
		break;

	case LGE_POWER_PROP_TEST_CHG_SCENARIO:
		val->intval = chip->test_chg_scenario;
		break;
	case LGE_POWER_PROP_TEST_BATT_THERM_VALUE:
		val->intval = chip->test_batt_therm_value;
		break;
	default:
		ret_val = -EINVAL;
		break;
	}

	return ret_val;
}
static void lge_cc_external_lge_power_changed(struct lge_power *lpc)
{
	union lge_power_propval lge_val = {0,};
#ifdef CONFIG_LGE_PM_LGE_POWER_CLASS_CHARGER_SLEEP
	union power_supply_propval value = {0,};
#endif
	int rc = 0;
	struct lge_charging_controller *chip
			= container_of(lpc, struct lge_charging_controller,
					lge_cc_lpc);
	chip->lge_cd_lpc = lge_power_get_by_name("cable_detect");

	if(!chip->lge_cd_lpc){
		pr_err("cable detection not found deferring probe\n");
	} else {
		rc = chip->lge_cd_lpc->get_property(chip->lge_cd_lpc,
					LGE_POWER_PROP_CHG_PRESENT, &lge_val);
		if (rc == 0) {
			if (chip->before_usb_present != lge_val.intval) {
				pr_err("usb present : %d\n",lge_val.intval);
#ifdef CONFIG_LGE_PM_LGE_POWER_CLASS_CHARGER_SLEEP
		if (!chip->batt_psy) {
			chip->batt_psy = power_supply_get_by_name("battery");
			if (!chip->batt_psy) {
				pr_err("%s : batt_psy is not yet ready\n", __func__);
				value.intval = 0;
			}
		} else {
			chip->batt_psy->get_property(chip->batt_psy,
				POWER_SUPPLY_PROP_CAPACITY, &value);
		}
		lg_cc_stop_battemp_alarm(chip);

		if (lge_val.intval && (value.intval < 100)) {
			pr_err("start alarm\n");
			lg_cc_start_battemp_alarm(chip, BTM_ALARM_TIME(2));
		} else {
			pr_err("start work_queue\n");
			lg_cc_start_battemp_work(chip,2);
		}
#else
		lg_cc_stop_battemp_work(chip);
		lg_cc_start_battemp_work(chip,2);
#endif
				chip->before_usb_present = lge_val.intval;
				chip->is_usb_present = lge_val.intval;
			}
		}
		rc = chip->lge_cd_lpc->get_property(chip->lge_cd_lpc,
					LGE_POWER_PROP_HVDCP_PRESENT, &lge_val);
		if (rc == 0) {
			if (chip->before_hvdcp_present != lge_val.intval) {
				pr_err("hvdcp present : %d\n",lge_val.intval);
#ifdef CONFIG_LGE_PM_LGE_POWER_CLASS_CHARGER_SLEEP
				if (!chip->batt_psy) {
					chip->batt_psy = power_supply_get_by_name("battery");
					if (!chip->batt_psy) {
						pr_err("%s : batt_psy is not yet ready\n", __func__);
						value.intval = 0;
					}
				} else {
					chip->batt_psy->get_property(chip->batt_psy,
						POWER_SUPPLY_PROP_CAPACITY, &value);
				}
				lg_cc_stop_battemp_alarm(chip);

				if (lge_val.intval && (value.intval < 100)) {
					pr_err("start alarm\n");
					lg_cc_start_battemp_alarm(chip, BTM_ALARM_TIME(3));
				} else {
					pr_err("start work_queue\n");
					lg_cc_start_battemp_work(chip,3);
				}
#else
				lg_cc_stop_battemp_work(chip);
				lg_cc_start_battemp_work(chip,3);
#endif
				chip->is_hvdcp_present = lge_val.intval;
				chip->before_hvdcp_present
					= chip->is_hvdcp_present;
				chip->update_hvdcp_state = 1;
			}
		}
	}
}
static int
lge_charging_controller_probe(struct platform_device *pdev)
{
	struct lge_charging_controller *controller;
	struct lge_power *lge_power_cc;
	int ret;

	controller = kzalloc(sizeof(struct lge_charging_controller),
								GFP_KERNEL);

	if(!controller){
		pr_err("lge_charging_controller memory alloc failed.\n");
		return -ENOMEM;
	}

	the_controller = controller;
	controller->chg_enable = -1;


	wake_lock_init(&controller->lcs_wake_lock,
			WAKE_LOCK_SUSPEND, "lge_charging_scenario");
#ifndef CONFIG_LGE_PM_LGE_POWER_CLASS_CHARGER_SLEEP
	wake_lock_init(&controller->chg_wake_lock,
			WAKE_LOCK_SUSPEND, "lge_charging_wake_lock");
#endif
	INIT_DELAYED_WORK(&controller->battemp_work,
			lge_monitor_batt_temp_work);
#ifdef CONFIG_LGE_PM_LGE_POWER_CLASS_CHARGER_SLEEP
	alarm_init(&controller->btm_polling_alarm, ALARM_REALTIME,
			lgcc_btm_alarm);
	wake_lock_init(&controller->alarm_wake_lock,
			WAKE_LOCK_SUSPEND, "lge_charging_scenario_alarm");
#endif
	lge_power_cc = &controller->lge_cc_lpc;

	lge_power_cc->name = "cc";

	lge_power_cc->properties = lge_power_lge_cc_properties;
	lge_power_cc->num_properties =
		ARRAY_SIZE(lge_power_lge_cc_properties);
	lge_power_cc->get_property = lge_power_lge_cc_get_property;
	lge_power_cc->set_property = lge_power_lge_cc_set_property;

	lge_power_cc->property_is_writeable =
		lge_power_lge_cc_property_is_writeable;
	lge_power_cc->supplied_to = cc_supplied_to;
	lge_power_cc->num_supplicants = ARRAY_SIZE(cc_supplied_to);
	lge_power_cc->external_lge_power_changed
			= lge_cc_external_lge_power_changed;
	ret = lge_power_register(&pdev->dev, lge_power_cc);
	if (ret < 0) {
		pr_err("[LGE_CC] Failed to register lge power class: %d\n",
			ret);
		goto err_free;
	}
	controller->chg_current_max = -EINVAL;
	controller->chg_current_te = controller->chg_current_max;

	controller->otp_ibat_current = -1;

	controller->start_batt_temp = 5;
	lg_cc_start_battemp_work(controller,
			controller->start_batt_temp);
	controller->test_chg_scenario = 0;
	controller->test_batt_therm_value = 25;
	controller->before_usb_present = 0;
	controller->before_hvdcp_present = 0;
	controller->is_hvdcp_present = 0;
	controller->update_hvdcp_state = 0;
	pr_info("LG Charging controller probe done~!!\n");

	return 0;
err_free:
	kfree(the_controller);
	return ret;
}
#ifdef CONFIG_OF
static struct of_device_id lge_charging_controller_match_table[] = {
	{.compatible = "lge,charging_controller"},
	{},
};
#endif
static int
lge_charging_controller_remove(struct platform_device *pdev)
{
	lge_power_unregister(&the_controller->lge_cc_lpc);
#ifdef CONFIG_LGE_PM_LGE_POWER_CLASS_CHARGER_SLEEP
	alarm_cancel(&the_controller->btm_polling_alarm);
#endif
	kfree(the_controller);
	return 0;
}

static struct platform_driver lge_charging_controller_driver = {
	.probe = lge_charging_controller_probe,
	.remove = lge_charging_controller_remove,
	.driver = {
		.name = MODULE_NAME,
		.owner = THIS_MODULE,
#ifdef CONFIG_OF
	.of_match_table = lge_charging_controller_match_table,
#endif
	},
};

static int __init lge_charging_controller_init(void)
{
	return platform_driver_register(&lge_charging_controller_driver);
}

static void lge_charging_controller_exit(void)
{
	platform_driver_unregister(&lge_charging_controller_driver);
}

late_initcall(lge_charging_controller_init);
module_exit(lge_charging_controller_exit);

MODULE_DESCRIPTION("LGE Charging Controller driver");
MODULE_LICENSE("GPL v2");
