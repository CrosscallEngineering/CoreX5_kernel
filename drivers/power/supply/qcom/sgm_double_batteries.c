/* Copyright (c) 2015 The Linux Foundation. All rights reserved.
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
#define pr_fmt(fmt) "--PM-SGM: %s: " fmt, __func__

#include <linux/debugfs.h>
#include <linux/errno.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/power_supply.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/of_regulator.h>
#include <linux/regulator/machine.h>
#include <linux/gpio.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/usb.h>
#include <linux/platform_device.h>
#include <linux/debugfs.h>
#include "smb5-lib.h"
#include <linux/his_debug_base.h>

#define	MAG_BUS_VLD		0
#define	TYPEC_BUS_VLD	1
#define	EXT_OTG_VLD		2
#define	TYPEC_OTG_VLD	3
#define	MAG_USB_VLD		4
#define	BAT1_CHARGING_VLD	5
#define	BAT2_CHARGING_VLD	6
#define	SGM_INIT			7

#define	SGM_MAG_CHARGER_VLD		(1 <<MAG_BUS_VLD)
#define	SGM_TYPEC_CHARGER_VLD		(1 <<TYPEC_BUS_VLD)
#define	SGM_DOUBLE_CHARGER_VLD		((1 <<MAG_BUS_VLD) | (1 <<TYPEC_BUS_VLD))
#define	SGM_NO_VLD_INPUT			(0)

/*battery status flags*/
#define	BAT1_DET_VLD				(0)
#define	BAT2_DET_VLD				(1)
#define	BAT1_ID_VLD					(2)
#define	BAT2_ID_VLD					(3)
#define	BAT1_CHG_VLD				(4)
#define	BAT2_CHG_VLD				(5)
#define	BAT1_PRECHG_VLD			(6)
#define	BAT2_PRECHG_VLD			(7)


#define	BAT_INIT					(8)
#define	SGM_ONLY_BAT1_EXIST		(1 <<BAT1_ID_VLD)
#define	SGM_ONLY_BAT2_EXIST		(1 <<BAT2_ID_VLD)
#define	SGM_BAT_ID_VLD		(SGM_ONLY_BAT1_EXIST | SGM_ONLY_BAT2_EXIST)
#define	SGM_BAT_DET_VLD	((1 <<BAT1_DET_VLD) | (1 <<BAT2_DET_VLD))
#define	SGM_BAT_CHG_VLD	(0xF <<BAT1_CHG_VLD) /*battery charging status, Prechg and fast charge*/
#define	SGM_BAT1_FC_BAT2_DISC		(0x1)
#define	SGM_BAT1_DISC_BAT2_FC		(0x2)
#define	SGM_BAT1_PRE_BAT2_FC		(0x6)
#define	SGM_BAT1_FC_BAT2_PRE		(0x9)


#define	CHARGE_SOC_STAGE_DEAD		(2)
#define	CHARGE_SOC_STAGE_VERY_LOW	(5)
#define	CHARGE_SOC_STAGE_LOW		(35)
#define	CHARGE_SOC_STAGE_MID		(70)
#define	CHARGE_SOC_STAGE_HIGH		(100)

#define	MAX_CAP_THRESHOLD			(100)
#define	MAX_CAP_STEP_CHG_ENTRIES	(6)
#define	MAX_CAP_STEP_DISCHG_ENTRIES	(6)
#define	CHARGE_STAGE_0				(0)

#define	CHG_SCHEDULE_PERIOD_MS		10000

#define	DEFAULT_ITERM_UA				(200*1000) /*200mA*/
#define	DEFAULT_CAP100_TIME_COUNT		(5) /*10*10s = 100s = 5min*/

extern struct device_bootinfo dev_bi;

struct cap_range_data {
	int low_threshold;
	int high_threshold;
	u32 value;
};
struct cap_step_chg_cfg {
	struct cap_range_data		cap_chg_cfg[MAX_CAP_STEP_CHG_ENTRIES];
};
struct cap_step_dischg_cfg {
	struct cap_range_data		cap_dischg_cfg[MAX_CAP_STEP_DISCHG_ENTRIES];
};
struct sgm_double_batt_chip {
	struct platform_device *pdev;
	struct delayed_work	chg_det_work;
	struct delayed_work	 chg_schedule_work;
	struct delayed_work	 bat1_det_work;
	struct delayed_work	 bat2_det_work;
	/*battery id work*/
	struct delayed_work	 batt_id1_work;
	struct delayed_work	 batt_id2_work;
	
	struct power_supply	*batt_psy;
	struct power_supply *sy_chg1_psy;
	struct power_supply *sy_chg2_psy;

	struct cap_step_chg_cfg	*step_chg_config;
	struct cap_step_dischg_cfg	*dischg_stage_config;
	/*batt charing stage and dischgaring stage*/
	int		bat1_chg_stage;
	int		bat2_chg_stage;
	int		bat1_dischg_stage;
	int		bat2_dischg_stage;
	int		bat1_capacity;
	int		bat2_capacity;
	/*battery health_status*/
	u32		batt_health_status;

	u32		typec_vbus_en_gpio;
	u32		typec_vbus_fcusb_en_gpio;
	int		mag_chg_det_gpio;	
	int		mag_chg_det_irq;
	/*battery selet pins*/
	u32		bat1_sel_gpio;
	u32		bat2_sel_gpio;
	/*battery id pins for battery exist or not*/
	u32		batt_id1_gpio;
	u32		batt_id2_gpio;
	int		batt_id1_init_irq;
	int		batt_id2_init_irq;
	/*battery det for battery pack*/
	u32		bat1_det_int_gpio;
	u32		bat2_det_int_gpio;
	int		bat1_det_int_irq;
	int		bat2_det_int_irq;
	u32		batt_thermal_set_gpio;
	bool		typec_otg_present;
	unsigned long inputs;
	unsigned long batt_status;
	bool		typec_ret_vbus;
	bool		use_batt1_flag; /*batt1_use: true, */
	bool 		force_next_chgstage;
	bool		force_next_dischgstage;
};



#define is_between(left, right, value) \
		(((left) >= (right) && (left) >= (value) \
			&& (value) >= (right)) \
		|| ((left) <= (right) && (left) <= (value) \
			&& (value) <= (right)))

enum sgm_mode
{
    sgm_default_mode = 0,
    usb_otg_mode,
    magcon_chg_mode,
    usb_chg_mode,
    magcon_usb_chg_mode,
};
typedef enum BATT_ID {
	BAT1 = 0,
	BAT2 = 1,
	BAT_MAX = 2,
}battery_id;
static struct sgm_double_batt_chip *global_batt_chip = NULL;
static char *doubleinput[2] = { "AF_BF_EVENT=AF_BF_EVENT", NULL };
static battery_id last_batteryID = BAT_MAX;
//static bool ext_otg_status = false;
static unsigned long input_mag_typec_last  = 0x0;
static bool batt1_det_last_status = false;
static bool batt2_det_last_status = false;

/*battery id last status*/
static bool batt_id1_last_status = false;
static bool batt_id2_last_status = false;

extern struct smb_charger *global_smb_charger;
static int sgm_clear_battery_allchg_flag(void);
#ifdef 	CONFIG_HISENSE_CW2015_FG
extern int g_cw_get_bat1_capacity(void);
extern int g_cw_get_bat1_voltage(void);
extern int g_cw_get_bat2_capacity(void);
extern int g_cw_get_bat2_voltage(void);
extern int g_cw_reset_bat1_fuelgauge(void);
extern int g_cw_reset_bat2_fuelgauge(void);
#endif /*CONFIG_HISENSE_CW2015_FG*/

unsigned long sgm_2bat_get_chg_flag(void)
{
	if (!global_batt_chip) {
		pr_buf_err("sgm: err,no chip\n");
		return 0;
	}

	return ((global_batt_chip->batt_status & SGM_BAT_CHG_VLD) >> BAT1_CHG_VLD);
}
EXPORT_SYMBOL_GPL(sgm_2bat_get_chg_flag);

bool sgm_use_battery1_ok(void)
{
	if (!global_batt_chip) {
		pr_buf_err("sgm: err,no chip\n");
		return 0;
	}
	return (global_batt_chip->use_batt1_flag);
}
EXPORT_SYMBOL_GPL(sgm_use_battery1_ok);



//extern int smb5_usbin_plugin_irq_enable(bool enable);
bool sgm_check_battery1_exist(void)
{
	/*battery id gpio =0, represent battery exist*/
	bool bat1_id =false;
	if (!global_batt_chip) {
		pr_buf_err("sgm: err,no chip\n");
		return 0;
	}
	bat1_id =  !gpio_get_value(global_batt_chip->batt_id1_gpio);
	return ((!!test_bit(BAT1_ID_VLD, &(global_batt_chip->batt_status)))||(bat1_id));
}
EXPORT_SYMBOL_GPL(sgm_check_battery1_exist);

bool sgm_check_battery2_exist(void)
{
	/*battery id gpio =0, represent battery exist*/
	bool bat2_id = false;
	if (!global_batt_chip) {
		pr_buf_err("sgm: err,no chip\n");
		return 0;
	}
	bat2_id = !gpio_get_value(global_batt_chip->batt_id2_gpio);
	return ((!!test_bit(BAT2_ID_VLD, &(global_batt_chip->batt_status)))||(bat2_id));
}
EXPORT_SYMBOL_GPL(sgm_check_battery2_exist);

bool sgm_2btt_mag_charger_ok(void)
{
	if (!global_batt_chip) {
		pr_buf_err("sgm: err,no chip\n");
		return 0;
	}

	return !!test_bit(MAG_BUS_VLD, &(global_batt_chip->inputs));
}
EXPORT_SYMBOL_GPL(sgm_2btt_mag_charger_ok);

bool sgm_2batt_typec_charger_ok(void)
{
	if (!global_batt_chip) {
		pr_buf_err("sgm: err,no chip\n");
		return 0;
	}

	return !!test_bit(TYPEC_BUS_VLD, &(global_batt_chip->inputs));
}
EXPORT_SYMBOL_GPL(sgm_2batt_typec_charger_ok);

bool sgm_2batt_typec_otg_ok(void)
{
	if (!global_batt_chip) {
		pr_buf_err("sgm: err,no chip\n");
		return 0;
	}

	return !!test_bit(TYPEC_OTG_VLD, &(global_batt_chip->inputs));
}
EXPORT_SYMBOL_GPL(sgm_2batt_typec_otg_ok);

bool sgm_2batt_typec_port_online(void)
{
	if (!global_batt_chip) {
		pr_buf_err("sgm: err,no chip\n");
		return 0;
	}

	return !!(test_bit(TYPEC_OTG_VLD, &(global_batt_chip->inputs))
		|| test_bit(TYPEC_BUS_VLD, &(global_batt_chip->inputs)));
}
EXPORT_SYMBOL_GPL(sgm_2batt_typec_port_online);

int sgm_2batt_mag_chg_irq_enable(bool enable)
{
	pr_buf_err("sgm: %s , %s mag_chg_irq\n",__func__,enable?"enable":"disable");
	if (!global_batt_chip) {
		pr_buf_err("sgm: err,no chip\n");
		return -1;
	}
	if(enable){
		enable_irq(global_batt_chip->mag_chg_det_irq);
	} else {
		disable_irq(global_batt_chip->mag_chg_det_irq);
	}
	pr_buf_err("sgm: end %s\n",__func__);
	return 0;
}
EXPORT_SYMBOL_GPL(sgm_2batt_mag_chg_irq_enable);

int sgm_2batt_typec_chg_enable(bool enable)
{
 	bool ext_vbus_status =false;
	if (!global_batt_chip) {
		pr_buf_err("sgm: err,no chip\n");
		return -1;
	}
	ext_vbus_status = !!gpio_get_value(global_batt_chip->mag_chg_det_gpio);
	pr_buf_err("sgm:  %s, typec_ret_vbus %d\n",enable?"enable":"disable", global_batt_chip->typec_ret_vbus);
	if(enable){
		set_bit(TYPEC_BUS_VLD, &global_batt_chip->inputs);
		global_batt_chip->force_next_chgstage =false;/*recalculate the force next chgstage */
		cancel_delayed_work_sync(&global_batt_chip->chg_schedule_work);
		schedule_delayed_work(&global_batt_chip->chg_schedule_work,msecs_to_jiffies(20));
	} else {
		clear_bit(TYPEC_BUS_VLD, &global_batt_chip->inputs);
		/*Charger is remove, so clear those flags.*/
		sgm_clear_battery_allchg_flag();
		pr_buf_err("sgm: clear chg flags  0x%lx\n",global_batt_chip->batt_status);
		if(ext_vbus_status){
			pr_buf_err("sgm: TypeC was remove, but mag is still plug in\n");
			cancel_delayed_work_sync(&global_batt_chip->chg_det_work);
			schedule_delayed_work(&global_batt_chip->chg_det_work, msecs_to_jiffies(20));
		}
	}
	kobject_uevent_env(&global_batt_chip->pdev->dev.kobj, KOBJ_CHANGE, doubleinput);
	pr_buf_err("sgm: end %s\n",__func__);
	return 0;
}
EXPORT_SYMBOL_GPL(sgm_2batt_typec_chg_enable);

static int read_range_data_from_node(struct device_node *node,
		const char *prop_str, struct cap_range_data *ranges,
		int max_threshold, u32 max_value)
{
	int rc = 0, i, length, per_tuple_length, tuples;

	if (!node || !prop_str || !ranges) {
		pr_err("Invalid parameters passed\n");
		return -EINVAL;
	}

	rc = of_property_count_elems_of_size(node, prop_str, sizeof(u32));
	if (rc < 0) {
		pr_err("Count %s failed, rc=%d\n", prop_str, rc);
		return rc;
	}

	length = rc;
	per_tuple_length = sizeof(struct cap_range_data) / sizeof(u32);
	if (length % per_tuple_length) {
		pr_err("%s length (%d) should be multiple of %d\n",
				prop_str, length, per_tuple_length);
		return -EINVAL;
	}
	tuples = length / per_tuple_length;

	if (tuples > MAX_CAP_STEP_CHG_ENTRIES) {
		pr_err("too many entries(%d), only %d allowed\n",
				tuples, MAX_CAP_STEP_CHG_ENTRIES);
		return -EINVAL;
	}

	rc = of_property_read_u32_array(node, prop_str,
			(u32 *)ranges, length);
	if (rc) {
		pr_err("Read %s failed, rc=%d", prop_str, rc);
		return rc;
	}

	for (i = 0; i < tuples; i++) {
		if (ranges[i].low_threshold >
				ranges[i].high_threshold) {
			pr_err("%s thresholds should be in ascendant ranges\n",
						prop_str);
			rc = -EINVAL;
			goto clean;
		}

		if (i != 0) {
			if (ranges[i - 1].high_threshold >
					ranges[i].low_threshold) {
				pr_err("%s thresholds should be in ascendant ranges\n",
							prop_str);
				rc = -EINVAL;
				goto clean;
			}
		}

		if (ranges[i].low_threshold > max_threshold)
			ranges[i].low_threshold = max_threshold;
		if (ranges[i].high_threshold > max_threshold)
			ranges[i].high_threshold = max_threshold;
		if (ranges[i].value > max_value)
			ranges[i].value = max_value;
		pr_err("%s %d low_threshold%d, high_threshold %d,  value %d\n",prop_str, i, ranges[i].low_threshold,ranges[i].high_threshold,ranges[i].value);
	}

	return rc;
clean:
	memset(ranges, 0, tuples * sizeof(struct cap_range_data));
	return rc;
}
static int get_charge_stage(struct cap_range_data *range, int current_index,
		int threshold,
		int *new_index, int *val)
{
	int i;

	*new_index = -EINVAL;

	/*
	 * If the threshold is lesser than the minimum allowed range,
	 * return -ENODATA.
	 */
	if (threshold < range[0].low_threshold)
		return -ENODATA;

	/* First try to find the matching index without hysteresis */
	for (i = 0; i < MAX_CAP_STEP_CHG_ENTRIES; i++) {
		if (!range[i].high_threshold && !range[i].low_threshold) {
			/* First invalid table entry; exit loop */
			break;
		}

		if (is_between(range[i].low_threshold,
			range[i].high_threshold, threshold)) {
			*new_index = i;
			*val = range[i].value;
			break;
		}
	}

	/*
	 * If nothing was found, the threshold exceeds the max range for sure
	 * as the other case where it is lesser than the min range is handled
	 * at the very beginning of this function. Therefore, clip it to the
	 * max allowed range value, which is the one corresponding to the last
	 * valid entry in the battery profile data array.
	 */
	if (*new_index == -EINVAL) {
		if (i == 0) {
			/* Battery profile data array is completely invalid */
			return -ENODATA;
		}

		*new_index = (i - 1);
		*val = range[*new_index].value;
	}

	/*
	 * If we don't have a current_index return this
	 * newfound value. There is no hysterisis from out of range
	 * to in range transition
	 */
	if (current_index == -EINVAL)
		return 0;

	return 0;
}
static bool sgm_double_charger_ok(void)
{
	if (!global_batt_chip) {
		pr_buf_err("no chip\n");
		return 0;
	}

	return !!(test_bit(MAG_BUS_VLD, &global_batt_chip->inputs)
		&& test_bit(TYPEC_BUS_VLD, &global_batt_chip->inputs));
}
static int sgm_clear_battery_allchg_flag(void)
{
	if (!global_batt_chip) {
		pr_buf_err("no chip\n");
		return -1;
	}
	clear_bit(BAT1_CHG_VLD, &global_batt_chip->batt_status);
	clear_bit(BAT2_CHG_VLD, &global_batt_chip->batt_status);
	clear_bit(BAT1_PRECHG_VLD, &global_batt_chip->batt_status);
	clear_bit(BAT2_PRECHG_VLD, &global_batt_chip->batt_status);
	global_batt_chip->force_next_dischgstage =false;
	global_batt_chip->force_next_chgstage =false;
	return 0;
}
static int sgm_enable_USBvbus_forSy6974(struct sgm_double_batt_chip *chip, bool enable){
	//ret = gpio_direction_output(global_batt_chip->typec_vbus_en_gpio, 0);
	int ret = 0;
	if(enable){
		ret = gpio_direction_output(global_batt_chip->typec_vbus_fcusb_en_gpio, 1);
	}else{
		ret = gpio_direction_output(global_batt_chip->typec_vbus_fcusb_en_gpio, 0);
	}
	if(ret)
		pr_buf_err("sgm: %s typec_vbus_fcusb_en_gpio %d, ret %d.\n", (enable ?"enable":"disable"),global_batt_chip->typec_vbus_fcusb_en_gpio,ret);
	return ret;	
}
static int sgm_enable_TypeCvbus(struct sgm_double_batt_chip *chip, bool enable){
	//ret = gpio_direction_output(global_batt_chip->typec_vbus_en_gpio, 0);
	int ret = 0;
	if(enable){
		ret = gpio_direction_output(global_batt_chip->typec_vbus_en_gpio, 1);
	}else{
		ret = gpio_direction_output(global_batt_chip->typec_vbus_en_gpio, 0);
	}
	if(ret)
		pr_buf_err("sgm: %s typec_vbus_fcusb_en_gpio %d, ret %d.\n", (enable ?"enable":"disable"),global_batt_chip->typec_vbus_en_gpio,ret);
	return ret;	
}
static irqreturn_t sgm_2batt_mag_chg_irq(int irq, void *data)
{
	struct sgm_double_batt_chip *chip = data;

	pr_buf_err("sgm:  trigger.\n");

	schedule_delayed_work(&chip->chg_det_work, msecs_to_jiffies(20));

	return IRQ_HANDLED;
}

static irqreturn_t sgm_bat1_det_int_irq(int irq, void *data)
{
	struct sgm_double_batt_chip *chip = data;

	pr_buf_err("sgm:  trigger.\n");
	cancel_delayed_work_sync(&chip->bat1_det_work);	
	schedule_delayed_work(&chip->bat1_det_work, msecs_to_jiffies(20));
	return IRQ_HANDLED;
}

static irqreturn_t sgm_bat2_det_int_irq(int irq, void *data)
{
	struct sgm_double_batt_chip *chip = data;

	pr_buf_err("sgm:  triggered\n");
	cancel_delayed_work_sync(&chip->bat2_det_work);
	schedule_delayed_work(&chip->bat2_det_work, msecs_to_jiffies(20));
	return IRQ_HANDLED;
}

static irqreturn_t sgm_batt_id1_int_irq(int irq, void *data)
{
	struct sgm_double_batt_chip *chip = data;

	pr_buf_err("sgm:  triggered\n");
	cancel_delayed_work_sync(&chip->batt_id1_work);
	schedule_delayed_work(&chip->batt_id1_work, msecs_to_jiffies(20));
	return IRQ_HANDLED;
}

static irqreturn_t sgm_batt_id2_int_irq(int irq, void *data)
{
	struct sgm_double_batt_chip *chip = data;

	pr_buf_err("sgm:  triggered\n");
	cancel_delayed_work_sync(&chip->batt_id2_work);
	schedule_delayed_work(&chip->batt_id2_work, msecs_to_jiffies(20));
	return IRQ_HANDLED;
}

static int sgm_check_power_supply_psy(struct sgm_double_batt_chip *chip)
{

	if (!chip->batt_psy)
	{
		chip->batt_psy = power_supply_get_by_name("battery");
		if (!chip->batt_psy) {
			dev_err(&chip->pdev->dev, "Could not get battery power_supply\n");
			return -ENODEV;
		}
	}
	
	if(!chip->sy_chg1_psy)
	{
		chip->sy_chg1_psy = power_supply_get_by_name(SY6974X_CHG_BAT1_PSY_NAME);
		if(!chip->sy_chg1_psy){
			dev_err(&chip->pdev->dev, "Could not get sy6974 battery1  power_supply\n");
			return -ENODEV;
		}
	}
	
	if(!chip->sy_chg2_psy)
	{
		chip->sy_chg2_psy = power_supply_get_by_name(SY6974X_CHG_BAT2_PSY_NAME);
		if(!chip->sy_chg2_psy){
			dev_err(&chip->pdev->dev, "Could not get sy6974 battery2  power_supply\n");
			return -ENODEV;
		}
	}

	if (!global_smb_charger)
		return  -ENODEV;

	return 0;
}

static int sgm_select_battery_and_thermal(battery_id batteryid){
	int ret = 1;

	if (!global_batt_chip) {
		pr_buf_err("sgm: err,no chip\n");
		return -EINVAL;
	}
#if 0
	pr_buf_err("sgm: select battery %d to power whole system.\n", (batteryid+1));
	/*TWO battery are all exit*/
	/*Get battery1  and battery2 capacity*/
	bat1_cap = g_cw_get_bat1_capacity();
	bat2_cap = g_cw_get_bat2_capacity();

	/*Need to do latter*/
	/*when the battery cap is too low, don't change to it */
	if((batteryid == BAT1) &&(bat1_cap <= CHARGE_SOC_STAGE_VERY_LOW)){
		pr_buf_err("sgm: select battery %d is to low, not change.\n", (batteryid+1));
		return -1;
	}
	if((batteryid == BAT2) &&(bat2_cap <= CHARGE_SOC_STAGE_VERY_LOW)){
		pr_buf_err("sgm: select battery %d is to low, not change.\n", (batteryid+1));
		return -1;
	}
#endif		
	/*pull down to select battery 1, pull high to select battery 2
	BAT1 = 0 
	BAT2 = 1*/
	if(batteryid == BAT2){
		ret = gpio_direction_output(global_batt_chip->bat1_sel_gpio, 1);
		ret = gpio_direction_output(global_batt_chip->bat2_sel_gpio, 1);
		ret = gpio_direction_output(global_batt_chip->batt_thermal_set_gpio, 1);
		global_batt_chip->use_batt1_flag = false;
	}else{
		ret = gpio_direction_output(global_batt_chip->bat1_sel_gpio, 0);
		ret = gpio_direction_output(global_batt_chip->bat2_sel_gpio, 0);
		ret = gpio_direction_output(global_batt_chip->batt_thermal_set_gpio, 0);	
		global_batt_chip->use_batt1_flag = true;
	}
	pr_buf_err("sgm: select battery %d  to power whole system.\n", (batteryid+1));

	return ret;
}
static int sgm_get_batt_health(struct sgm_double_batt_chip *chip )
{
	union power_supply_propval pval = {0, };

	if (sgm_check_power_supply_psy(global_batt_chip))
		return -EINVAL;

	power_supply_get_property(chip->batt_psy, POWER_SUPPLY_PROP_HEALTH, &pval);
	chip->batt_health_status = pval.intval;
	pr_buf_info("batt_health_status is %d\n", chip->batt_health_status);

	return 0;
}
static int sgm_batt_mag_charging_config(unsigned long input, battery_id batteryid){
	int ret = 1;
	union power_supply_propval pval = {2000000, };	//default usb_in_icl is 2A
	union power_supply_propval val = {0, };
	if (!global_batt_chip) {
		pr_buf_err("sgm: err,no chip\n");
		return -EINVAL;
	}

	/* sel battery according to batteryid*/
	sgm_select_battery_and_thermal(batteryid);

	if (sgm_check_power_supply_psy(global_batt_chip))
		return -EINVAL;

	if(BAT1 == batteryid){
		/*start  sy6974 for battery 1*/
		val.intval = 2;
		power_supply_set_property(global_batt_chip->sy_chg1_psy,
			POWER_SUPPLY_PROP_STATUS, &val);
		/*stop sy6974 IC2 , discharger battery2*/
		val.intval = 3;
		power_supply_set_property(global_batt_chip->sy_chg2_psy,
			POWER_SUPPLY_PROP_STATUS, &val);
		power_supply_set_property(global_batt_chip->sy_chg1_psy,
			POWER_SUPPLY_PROP_HEALTH, &pval);
	}else if (BAT2== batteryid){
		/*stop  sy6974 for battery 1*/
		val.intval = 3;
		power_supply_set_property(global_batt_chip->sy_chg1_psy,
			POWER_SUPPLY_PROP_STATUS, &val);
		/*start sy6974 IC2 , charger battery2*/
		val.intval = 2;
		power_supply_set_property(global_batt_chip->sy_chg2_psy,
			POWER_SUPPLY_PROP_STATUS, &val);
		power_supply_set_property(global_batt_chip->sy_chg2_psy,
			POWER_SUPPLY_PROP_HEALTH, &pval);
	}else{
		pr_buf_info("sgm: Error unknow batteryid %d\n", batteryid);		
	}
	power_supply_changed(global_batt_chip->batt_psy);
	kobject_uevent_env(&global_batt_chip->pdev->dev.kobj, KOBJ_CHANGE, doubleinput);
	pr_buf_info("sgm: Magcon set fcc %d ma\n", pval.intval / 1000);

	return ret;
}

static int sgm_batt_typec_charging_config(unsigned long input, battery_id batteryid){
	int ret = 1;
	union power_supply_propval pval = {3000000, };	//default usb_in_icl is 2A
	union power_supply_propval val = {0, };
	if (!global_batt_chip) {
		pr_buf_err("sgm: err,no chip\n");
		return -EINVAL;
	}

	//ret = gpio_direction_output(global_batt_chip->typec_vbus_en_gpio, 1);
	//pr_buf_err("sgm: typec_vbus_en_gpio disable Vbus\n");
	/* sel battery 1*/
	sgm_select_battery_and_thermal(batteryid);

	/* Wait 30ms to get Vbus ready */	
	//usleep_range(30000, 30100);	
	/*typec_vbus_en_gpio  set low*/
	//ret = gpio_direction_output(global_batt_chip->typec_vbus_en_gpio, 0);
	
	/*typec_vbus_en_gpio  disable typec vbus, fake usb plugin action*/
	//global_batt_chip->typec_ret_vbus = true; 
	//pr_buf_err("sgm: typec_vbus_en_gpio Enable Vbus\n");

	//
	//smblib_rerun_apsd_if_required(global_smb_charger);
	//pr_buf_info("sgm: charger need redet, usb_switch_gpio to low & rerun apsd.\n");

	//To do
	if (sgm_check_power_supply_psy(global_batt_chip))
		return -EINVAL;
	if(input &SGM_MAG_CHARGER_VLD){
		/*Mag charging input, stop mag charging*/
		clear_bit(MAG_BUS_VLD, &(global_batt_chip->inputs));
		val.intval = 3;
		power_supply_set_property(global_batt_chip->sy_chg1_psy,
			POWER_SUPPLY_PROP_STATUS, &val);
		power_supply_set_property(global_batt_chip->sy_chg2_psy,
			POWER_SUPPLY_PROP_STATUS, &val);
		pr_buf_info("sgm: stop mag charging, since  typec was already plugin.\n", pval.intval / 1000);
	}
	if(global_batt_chip->batt_health_status ==POWER_SUPPLY_HEALTH_GOOD ){
		power_supply_set_property(global_batt_chip->batt_psy,POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX, &pval);
	}else{
		pr_buf_info("sgm: batt_health_status %d\n",global_batt_chip->batt_health_status);
	}
	pr_buf_info("sgm: Typec set fcc %d ma\n", pval.intval / 1000);	
	return ret;

}
static int sgm_batt_charging_config(unsigned long  input_typec_mag, battery_id batteryid){
	int ret =0;
	pr_buf_info("sgm: charging  battery %d via 0x%lx\n", (batteryid+1), input_typec_mag);
	sgm_enable_USBvbus_forSy6974(global_batt_chip,false);	
	switch(input_typec_mag){
		case SGM_TYPEC_CHARGER_VLD:
		case SGM_DOUBLE_CHARGER_VLD:
			sgm_batt_typec_charging_config(input_typec_mag, batteryid);
			break;
		case SGM_MAG_CHARGER_VLD:
			sgm_batt_mag_charging_config(input_typec_mag, batteryid);
			break;
		case SGM_NO_VLD_INPUT:
		default:
			ret = -1;
			pr_buf_info("sgm: No typec or mag  (all) plug in\n");
			break;
	}
	return ret ;
}
static int sgm_batt_mag_precharging_config(unsigned long input, unsigned long chgstatus){
	int ret = 0;
	union power_supply_propval pval = {1000000, };	//default usb_in_icl is 1A
	union power_supply_propval val = {0, };
	battery_id battid = BAT_MAX;
	unsigned long batt_chg_status = 0;	
	if (!global_batt_chip) {
		pr_buf_err("sgm: err,no chip\n");
		return -EINVAL;
	}
	batt_chg_status= (global_batt_chip->batt_status & SGM_BAT_CHG_VLD);	
	sgm_enable_USBvbus_forSy6974(global_batt_chip,false);
	pr_buf_info("sgm: mag chager comming , disable USBvbus to sy6974 0x%x\n",chgstatus);
	if(SGM_BAT1_PRE_BAT2_FC ==chgstatus){
		battid = BAT2;
	}else if(SGM_BAT1_FC_BAT2_PRE ==chgstatus){
		battid = BAT1;
	}else{
		pr_buf_info("sgm: Warning unknow chgstatus 0x%x\n",chgstatus);
		if(global_batt_chip->use_batt1_flag)
			battid = BAT1;
		else
			battid = BAT2;
	}	
	/* sel battery according to batteryid*/
	sgm_select_battery_and_thermal(battid);

	if (sgm_check_power_supply_psy(global_batt_chip))
		return -EINVAL;
	val.intval = 2;
	power_supply_set_property(global_batt_chip->sy_chg1_psy,
			POWER_SUPPLY_PROP_STATUS, &val);
	power_supply_set_property(global_batt_chip->sy_chg2_psy,
			POWER_SUPPLY_PROP_STATUS, &val);
	/*if battery health Reset Fcc current to 1000mA, Since the total FCC  limit <= 2A*/
	if((!!test_bit(BAT1_PRECHG_VLD,&(global_batt_chip->batt_status)))||
		(!!test_bit(BAT2_PRECHG_VLD,&(global_batt_chip->batt_status)))){
		power_supply_set_property(global_batt_chip->sy_chg1_psy,
			POWER_SUPPLY_PROP_HEALTH, &pval);
		power_supply_set_property(global_batt_chip->sy_chg2_psy,
			POWER_SUPPLY_PROP_HEALTH, &pval);
	}else{
		pr_buf_info("sgm: Error unknow batt_status 0x%x\n", global_batt_chip->batt_status);
	}

	power_supply_changed(global_batt_chip->batt_psy);
	kobject_uevent_env(&global_batt_chip->pdev->dev.kobj, KOBJ_CHANGE, doubleinput);
	pr_buf_info("sgm: Magcon set fcc %d ma\n", pval.intval / 1000);

	return ret;
}

static int sgm_batt_typec_precharging_config(unsigned long input, unsigned long chgstatus){
	int ret = 1;

	union power_supply_propval pval = {1000000, };	//default usb_in_icl is 1A
	union power_supply_propval val = {0, };
	battery_id battid = BAT_MAX;
	if (!global_batt_chip) {
		pr_buf_err("sgm: err,no chip\n");
		return -EINVAL;
	}
	sgm_enable_USBvbus_forSy6974(global_batt_chip,true);
	pr_buf_info("sgm: mag chager comming , enable USBvbus to sy6974 0x%x\n",chgstatus);	

	if(SGM_BAT1_PRE_BAT2_FC ==chgstatus){
		battid = BAT2;
	}else if(SGM_BAT1_FC_BAT2_PRE ==chgstatus){
		battid = BAT1;
	}else{
		pr_buf_info("sgm: Warning unknow chgstatus 0x%x\n",chgstatus);
		if(global_batt_chip->use_batt1_flag)
			battid = BAT1;
		else
			battid = BAT2;
	}

	sgm_select_battery_and_thermal(battid);
	//To do
	if (sgm_check_power_supply_psy(global_batt_chip))
		return -EINVAL;

	if(!!test_bit(BAT1_PRECHG_VLD,&(global_batt_chip->batt_status))){
		val.intval = 2;
		power_supply_set_property(global_batt_chip->sy_chg1_psy,
			POWER_SUPPLY_PROP_STATUS, &val);
		power_supply_get_property(global_batt_chip->batt_psy,POWER_SUPPLY_PROP_CHARGER_TYPE,&val);
	        if(val.intval == POWER_SUPPLY_TYPE_USB ||
	                        val.intval == POWER_SUPPLY_TYPE_USB_CDP ||
	                         val.intval == POWER_SUPPLY_TYPE_USB_FLOAT) {
	                         pval.intval = 300000;
		}
		pr_buf_info("sgm: Typec plugin and chargerType is %d, set precharge current %dmA.\n",val.intval, (pval.intval / 1000));
		power_supply_set_property(global_batt_chip->sy_chg1_psy,
			POWER_SUPPLY_PROP_HEALTH, &pval);
		val.intval = 3;
		power_supply_set_property(global_batt_chip->sy_chg2_psy,
			POWER_SUPPLY_PROP_STATUS, &val);
	}else if(!!test_bit(BAT2_PRECHG_VLD,&(global_batt_chip->batt_status))){
		val.intval = 3;
		power_supply_set_property(global_batt_chip->sy_chg1_psy,
			POWER_SUPPLY_PROP_STATUS, &val);
		val.intval = 2;
		power_supply_set_property(global_batt_chip->sy_chg2_psy,
			POWER_SUPPLY_PROP_STATUS, &val);
		power_supply_get_property(global_batt_chip->batt_psy,POWER_SUPPLY_PROP_CHARGER_TYPE,&val);
	        if(val.intval == POWER_SUPPLY_TYPE_USB ||
	                        val.intval == POWER_SUPPLY_TYPE_USB_CDP ||
	                         val.intval == POWER_SUPPLY_TYPE_USB_FLOAT) {
	                         pval.intval = 300000;
		}
		pr_buf_info("sgm: Typec plugin and chargerType is %d, set precharge current %dmA.\n",val.intval, (pval.intval / 1000));
		power_supply_set_property(global_batt_chip->sy_chg2_psy,
			POWER_SUPPLY_PROP_HEALTH, &pval);
	}

	if(global_batt_chip->batt_health_status ==POWER_SUPPLY_HEALTH_GOOD ){
		power_supply_set_property(global_batt_chip->batt_psy,POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX, &pval);
	}else{
		pr_buf_info("sgm: batt_health_status %d\n",global_batt_chip->batt_health_status);
	}
	
	pr_buf_info("sgm: Typec set fcc %d ma\n", pval.intval / 1000);	
	return ret;

}
static int sgm_batt_precharging_config(unsigned long  input_typec_mag, unsigned long chgstatus){
	int ret =0;
	pr_buf_info("sgm: charging  battery 0x%x via 0x%lx\n",chgstatus,input_typec_mag);
	switch(input_typec_mag){
		case SGM_TYPEC_CHARGER_VLD:
		case SGM_DOUBLE_CHARGER_VLD:
			sgm_batt_typec_precharging_config(input_typec_mag,chgstatus);
			break;
		case SGM_MAG_CHARGER_VLD:
			sgm_batt_mag_precharging_config(input_typec_mag,chgstatus);
			break;
		case SGM_NO_VLD_INPUT:
		default:
			ret = -1;
			pr_buf_info("sgm: No typec or mag plug in\n");
			break;
	}
	return ret ;
}

static void sgm_chg_det_work(struct work_struct *w)
{
	struct sgm_double_batt_chip *chip = container_of(w, struct sgm_double_batt_chip,
		chg_det_work.work);
	union power_supply_propval val = {3, };
	bool ext_vbus_status = !!gpio_get_value(chip->mag_chg_det_gpio);
	bool typec_vbus_status = sgm_2batt_typec_charger_ok();
	
	pr_buf_info("sgm: input 0x%lx, ext_status=%d typec_status=%d\n", chip->inputs, ext_vbus_status, typec_vbus_status);

	if(((ext_vbus_status << MAG_BUS_VLD) | (typec_vbus_status << TYPEC_BUS_VLD))
		== (chip->inputs & (BIT(MAG_BUS_VLD) | BIT(TYPEC_BUS_VLD)))) {
		pr_buf_info("sgm: the same as last, do nothing\n");
		return;
	}

	if (typec_vbus_status){
		pr_buf_info("sgm: typec is already online, return \n");
		return;
	}

	/*If ext otg is working, ignore ext vbus status.*/
	if (ext_vbus_status){
		set_bit(MAG_BUS_VLD, &global_batt_chip->inputs);
		global_batt_chip->force_next_chgstage =false;/*recalculate the force next chgstage */
		cancel_delayed_work_sync(&chip->chg_schedule_work);
		//delay 300ms to make sure charger type detected.
		schedule_delayed_work(&chip->chg_schedule_work,msecs_to_jiffies(300));
	} else {
		clear_bit(MAG_BUS_VLD, &global_batt_chip->inputs);
		//cancel_delayed_work(&chip->chg_schedule_work);
		/*Mag Charger is remove, so clear those flags.*/
		sgm_clear_battery_allchg_flag();
		if (sgm_check_power_supply_psy(global_batt_chip)){
			return;
		}
		val.intval = 3;
		power_supply_set_property(global_batt_chip->sy_chg1_psy,
			POWER_SUPPLY_PROP_STATUS, &val);
		/*stop sy6974 IC2 , discharger battery2*/
		power_supply_set_property(global_batt_chip->sy_chg2_psy,
			POWER_SUPPLY_PROP_STATUS, &val);
		power_supply_changed(global_batt_chip->batt_psy);		
		pr_buf_err("sgm: Mag charger is remove , clear chg flags  0x%lx\n",global_batt_chip->batt_status);		
	}
	pr_buf_info("sgm: input 0x%lx, ext_status=%d typec_status=%d\n", chip->inputs, ext_vbus_status, typec_vbus_status);
	return;
}
/**/
static int sgm_set_batt_fastchg_flag(battery_id batteryid)
{

	bool typec_vbus_status = sgm_2batt_typec_charger_ok();
	pr_buf_info("sgm: battery %d set fastchg flag, last batteryID %d.\n", (batteryid +1), (last_batteryID +1));
	if (!global_batt_chip) {
		pr_buf_err("sgm: err,no chip\n");
		return -1;
	}
	/*typec_vbus_en_gpio  disable typec vbus, fake usb plugin action*/
	pr_buf_err("sgm: battery changed from %d to %d, typec valid %d,batt_status 0x%x\n",  (last_batteryID +1), (batteryid +1),typec_vbus_status, global_batt_chip->batt_status);
	if((last_batteryID != batteryid) &&(typec_vbus_status)){
		global_batt_chip->typec_ret_vbus = true; 
		last_batteryID = batteryid;
	}

	if(BAT1 == batteryid){
		set_bit(BAT1_CHG_VLD,&global_batt_chip->batt_status);
		clear_bit(BAT2_CHG_VLD,&global_batt_chip->batt_status);
	}else{
		clear_bit(BAT1_CHG_VLD,&global_batt_chip->batt_status);
		set_bit(BAT2_CHG_VLD,&global_batt_chip->batt_status);
	}
	pr_buf_err("sgm: batt_status 0x%x\n", global_batt_chip->batt_status);
	return 0;
}

static int sgm_set_batt_prechg_flag(battery_id batteryid)
{
	pr_buf_info("sgm: battery %d set prechg flag .\n", (batteryid +1));
	if (!global_batt_chip) {
		pr_buf_err("sgm: err,no chip\n");
		return -1;
	}	
	if(BAT1 == batteryid){
		set_bit(BAT1_PRECHG_VLD,&global_batt_chip->batt_status);
		clear_bit(BAT2_PRECHG_VLD,&global_batt_chip->batt_status);		
	}else if(BAT2 == batteryid){
		clear_bit(BAT1_PRECHG_VLD,&global_batt_chip->batt_status);
		set_bit(BAT2_PRECHG_VLD,&global_batt_chip->batt_status);	
	}else{
		clear_bit(BAT1_PRECHG_VLD,&global_batt_chip->batt_status);
		clear_bit(BAT2_PRECHG_VLD,&global_batt_chip->batt_status);
		pr_buf_info("sgm: No prechg flag for batt1 or batt2, so clear all flag\n",(batteryid +1));
	}
	return 0;
}

static int sgm_get_battery_stage(void)
{
	int bat1_stage = 0, bat2_stage = 0, value = 0;
	int bat1_cap = 50 ,  bat2_cap = 50;
	if (!global_batt_chip) {
		pr_buf_err("sgm: err,no chip\n");
		return -1;
	}	
	/*Get battery1  and battery2 capacity*/
	bat1_cap = g_cw_get_bat1_capacity();
	bat2_cap = g_cw_get_bat2_capacity();
	if(bat1_cap < 0){
		pr_buf_err("sgm: Get battery1 capacity error!use last cap%d bat1_cap=%d\n",
			global_batt_chip->bat1_capacity, bat1_cap);
	}else{
		global_batt_chip->bat1_capacity  =bat1_cap;
	}
	if(bat2_cap < 0){
		pr_buf_err("sgm: Get battery2 capacity error!use last cap=%d bat2_cap=%d\n",
			global_batt_chip->bat2_capacity,bat2_cap);
	}else{
		global_batt_chip->bat2_capacity = bat2_cap;
	}

	/*Update battery and battery2 charge stage*/
	get_charge_stage(global_batt_chip->step_chg_config->cap_chg_cfg, 1,bat1_cap,&bat1_stage,&value);
	get_charge_stage(global_batt_chip->step_chg_config->cap_chg_cfg, 1,bat2_cap,&bat2_stage,&value);
	//update battery charging stage
	global_batt_chip->bat1_chg_stage = bat1_stage;
	global_batt_chip->bat2_chg_stage = bat2_stage;

	/*Update battery and battery2 discharging stage*/
	get_charge_stage(global_batt_chip->dischg_stage_config->cap_dischg_cfg, 1,bat1_cap,&bat1_stage,&value);
	get_charge_stage(global_batt_chip->dischg_stage_config->cap_dischg_cfg, 1,bat2_cap,&bat2_stage,&value);
	global_batt_chip->bat1_dischg_stage = bat1_stage;
	global_batt_chip->bat2_dischg_stage = bat2_stage;

	pr_buf_err("sgm: input 0x%lx, bat1_cap=%d bat2_cap=%d, Charging stage[ %d  %d], Discharging stage [%d  %d] \n",
		global_batt_chip->inputs, global_batt_chip->bat1_capacity, global_batt_chip->bat2_capacity,
		global_batt_chip->bat1_chg_stage , global_batt_chip->bat2_chg_stage ,
		global_batt_chip->bat1_dischg_stage,global_batt_chip->bat2_dischg_stage);	
	return 0;
}
static int sgm_chg_bat1_or_bat2(void){
	int rc = 0;
	static battery_id batChg = BAT1;
	static u32   batteryfull_count = 0;
	battery_id bat_prechg = BAT_MAX;
	union power_supply_propval pval = {2000000, };	//default usb_in_icl is 2A
	if (!global_batt_chip) {
		pr_buf_err("sgm: err,no chip\n");
		return -1;
	}

	if(global_batt_chip->bat1_chg_stage < global_batt_chip->bat2_chg_stage){
		if(global_batt_chip->bat1_chg_stage  <= CHARGE_STAGE_0){
			/*battery's capacity is very low, so need enable battery1 precharage battery2 fastcharge,
			when the battery1 charging high than CHARGE_STAGE_0, clear battery1 prechg flag*/
			bat_prechg = BAT1;
			batChg = BAT2;
		}else{
			bat_prechg = BAT_MAX;
			batChg = BAT1;
		}
	}else if (global_batt_chip->bat1_chg_stage == global_batt_chip->bat2_chg_stage){
		pr_buf_err("sgm:battery chg stage[%d %d],Cap [%d %d],force next flag %d\n", 
			global_batt_chip->bat1_chg_stage,global_batt_chip->bat2_chg_stage,
			global_batt_chip->bat1_capacity,global_batt_chip->bat2_capacity,
			global_batt_chip->force_next_chgstage);

		if((global_batt_chip->bat1_capacity <= global_batt_chip->bat2_capacity) &&
			(!global_batt_chip->force_next_chgstage)){
			batChg = BAT1;
			bat_prechg = BAT_MAX;
			global_batt_chip->force_next_chgstage = true;
		}else{
			if((global_batt_chip->bat1_capacity > global_batt_chip->bat2_capacity) &&
				(!global_batt_chip->force_next_chgstage)){
				batChg = BAT2;
				bat_prechg = BAT_MAX;		
				global_batt_chip->force_next_chgstage =true;
			}
		}
		if(global_batt_chip->bat2_chg_stage  == CHARGE_STAGE_0){
			/*global_batt_chip->bat1_chg_stage  == CHARGE_STAGE_0*/
			if(global_batt_chip->use_batt1_flag){
				bat_prechg = BAT2;
				batChg = BAT1;
			}else{
				bat_prechg = BAT1;
				batChg = BAT2;
			}
		}
		if((global_batt_chip->bat1_capacity == CHARGE_SOC_STAGE_HIGH) ||
			(global_batt_chip->bat2_capacity == CHARGE_SOC_STAGE_HIGH)){
			/*Clear force next chgstage flag, since battery 1 or battery was charge full*/
			if(!sgm_check_power_supply_psy(global_batt_chip)){
				rc = power_supply_get_property(global_batt_chip->batt_psy, POWER_SUPPLY_PROP_CURRENT_NOW, &pval);
				if (rc < 0) {
						pr_buf_err( "Couldn't get batt current now property rc=%d\n",rc);
						return rc;
				}
			}
			pr_buf_err("sgm:battery charging current_now %d, batteryfull_count %d\n", pval.intval,batteryfull_count);
			/*battery 1 or battery2 is full*/
			if(((global_batt_chip->bat1_capacity == CHARGE_SOC_STAGE_HIGH) &&(batChg == BAT1)) ||
				(((global_batt_chip->bat2_capacity == CHARGE_SOC_STAGE_HIGH)&&(batChg == BAT2)))){
				if((pval.intval >  0 ) &&(pval.intval <= DEFAULT_ITERM_UA)){
					batteryfull_count ++;
				}else{
					batteryfull_count = 0;
					if(global_batt_chip->inputs & SGM_MAG_CHARGER_VLD){
						/*Magcon charging is present*/
						if(global_batt_chip->use_batt1_flag){
							if(!sgm_check_power_supply_psy(global_batt_chip)){
								rc = power_supply_get_property(global_batt_chip->sy_chg1_psy, POWER_SUPPLY_PROP_STATUS, &pval);
								if (rc < 0) {
									pr_buf_err("Couldn't get sy_chg1_psy prop status property rc=%d\n",rc);
									return rc;
								}
							}
						}else{
							if(!sgm_check_power_supply_psy(global_batt_chip)){
								rc = power_supply_get_property(global_batt_chip->sy_chg2_psy, POWER_SUPPLY_PROP_STATUS, &pval);
								if (rc < 0) {
									pr_buf_err("Couldn't get sy_chg2_psy prop status property rc=%d\n",rc);
									return rc;
								}
							}
						}
						pr_buf_err("sgm: %s magcon charger status %d\n",((batChg == BAT1) ?"BAT1":"BAT2"), pval.intval);
						if(pval.intval == POWER_SUPPLY_STATUS_FULL)
							batteryfull_count = 3;
					}
				}
			}
			pr_buf_err("sgm: %s charging current_now %d, batteryfull_count %d\n",((batChg == BAT1) ?"BAT1":"BAT2"), pval.intval,batteryfull_count);
			if(batteryfull_count  >=3){
				global_batt_chip->force_next_chgstage =false;
				batteryfull_count = 0;
				pr_buf_err("sgm:Charging Full battery chg stage[%d %d],Cap [%d %d],force next flag %d\n",
				global_batt_chip->bat1_chg_stage,global_batt_chip->bat2_chg_stage,
				global_batt_chip->bat1_capacity,global_batt_chip->bat2_capacity,
				global_batt_chip->force_next_chgstage);
			}
		}else{
			batteryfull_count = 0;
		}
	}else{
		/*global_batt_chip->bat1_chg_stage > global_batt_chip->bat2_chg_stage*/
		if(global_batt_chip->bat2_chg_stage  <= CHARGE_STAGE_0){
			/*battery's capacity is very low, so need enable battery1 precharage battery2 fastcharge,
			when the battery1 charging high than CHARGE_STAGE_0, clear battery1 prechg flag*/
			bat_prechg = BAT2;
			batChg = BAT1;
		}else{
			bat_prechg = BAT_MAX;
			batChg = BAT2;
		}
	}
	sgm_set_batt_fastchg_flag(batChg);
	sgm_set_batt_prechg_flag(bat_prechg);	
	if(global_batt_chip->bat1_chg_stage  != global_batt_chip->bat2_chg_stage){
		/*bat1 or bat2 charge stage is not equal, so we clear this force charge stage flag*/
		global_batt_chip->force_next_chgstage =false;
	}
	return 0;
}
static int sgm_dischg_bat1_or_bat2(unsigned long batteryexist){
	static battery_id dischg_batid = BAT1;
	if (!global_batt_chip) {
		pr_buf_err("sgm: err,no chip\n");
		return -1;
	}
	switch(batteryexist){
		case SGM_ONLY_BAT1_EXIST:
			dischg_batid = BAT1;
			break;
		case SGM_ONLY_BAT2_EXIST:
			dischg_batid = BAT2;
			break;
		case SGM_BAT_ID_VLD:
			{
				if(global_batt_chip->bat1_dischg_stage < global_batt_chip->bat2_dischg_stage){
					dischg_batid = BAT2;
				}else if (global_batt_chip->bat1_dischg_stage == global_batt_chip->bat2_dischg_stage){
					pr_buf_err("sgm:battery chg stage[%d %d],Cap [%d %d],force next flag %d \n", 
						global_batt_chip->bat1_dischg_stage,global_batt_chip->bat2_dischg_stage,
						global_batt_chip->bat1_capacity,global_batt_chip->bat2_capacity,
						global_batt_chip->force_next_dischgstage);

					if((global_batt_chip->bat1_capacity <= global_batt_chip->bat2_capacity) &&
						(!global_batt_chip->force_next_dischgstage)){
						dischg_batid = BAT2;
						global_batt_chip->force_next_dischgstage = true;
					}else{
						if((global_batt_chip->bat1_capacity > global_batt_chip->bat2_capacity) &&
							(!global_batt_chip->force_next_dischgstage)){
							dischg_batid = BAT1;
							global_batt_chip->force_next_dischgstage =true;
						}
					}
					if(global_batt_chip->bat2_dischg_stage ==CHARGE_STAGE_0)
						dischg_batid =	BAT2;				
					if((global_batt_chip->bat1_capacity == CHARGE_SOC_STAGE_DEAD) ||
						(global_batt_chip->bat2_capacity == CHARGE_SOC_STAGE_DEAD)){
						/*Clear force next chgstage flag, since battery 1 or battery was charge full*/
						global_batt_chip->force_next_dischgstage =false;
					}
				}else{
					/*batt2 discharging */
					dischg_batid = BAT1;
				}
				if(global_batt_chip->bat1_dischg_stage  != global_batt_chip->bat2_dischg_stage){
					/*bat1 or bat2 discharge stage is not equal, so we clear this force charge stage flag*/
					global_batt_chip->force_next_dischgstage =false;
				}
			}
			break;
		default :
			dischg_batid = BAT1;
			if(global_batt_chip->use_batt1_flag){
				dischg_batid = BAT1;
			}else{
				dischg_batid = BAT2;
			}
			pr_buf_err("sgm: unknow batt status 0x%lx, force set battery1 charging\n", batteryexist);
			break;
	}
	/*use_batt1_flag is true, but dischg_batid is BAT2 change to bat2
	  *else*
	  *use_batt1_flag is false (battery 2 is using), but dischg_batid is BAT1, then changed to BAT1*/
	if(((global_batt_chip->use_batt1_flag) &&(dischg_batid == BAT2)) ||
		((!global_batt_chip->use_batt1_flag) &&(dischg_batid == BAT1)))
		sgm_select_battery_and_thermal(dischg_batid);
	pr_buf_err("Select battery %d to power whole system.\n", (dischg_batid+1));
	return 0;
}
static void sgm_chg_schedule_work(struct work_struct *w)
{
	struct sgm_double_batt_chip *chip = container_of(w, struct sgm_double_batt_chip,
		chg_schedule_work.work);
	bool ext_vbus_status = gpio_get_value(chip->mag_chg_det_gpio);
	bool typec_vbus_status = sgm_2batt_typec_charger_ok();
	unsigned long input_mag_typec = (chip->inputs & SGM_DOUBLE_CHARGER_VLD);
	unsigned long batt_id_status = (chip->batt_status & SGM_BAT_ID_VLD);
	unsigned long batt_chg_status = (chip->batt_status & SGM_BAT_CHG_VLD);
	static u32 batt_health_last = POWER_SUPPLY_HEALTH_UNKNOWN;
	bool bat1_id = !gpio_get_value(chip->batt_id1_gpio);
	bool bat2_id = !gpio_get_value(chip->batt_id2_gpio);
	unsigned long batt_ID_now =  ((bat1_id <<BAT1_ID_VLD)|(bat2_id << BAT2_ID_VLD));
	unsigned long batt_chg_flag = 0;

	sgm_get_battery_stage();
	pr_buf_err("sgm: input 0x%lx, typecVbus=%d, ext vbus %d,batt_id_status 0x%x, batt_chg_status 0x%x\n", chip->inputs,typec_vbus_status,ext_vbus_status,batt_id_status,batt_chg_status);

	if(batt_ID_now != batt_id_status){
		//update it 
		//chip->batt_status |=batt_ID_now;			
		pr_buf_info("sgm: batt_ID_now 0x%lx, batt_id_status 0x%lx,chip->batt_status %lx \n",batt_ID_now, batt_id_status,chip->batt_status);
	}

	if(SGM_NO_VLD_INPUT !=  input_mag_typec){
		sgm_get_batt_health(chip);
		global_batt_chip->force_next_dischgstage =false;
		if(dev_bi.bootmode == BOOT_FACTORY_MODE){
			/*Factory mode, default use battery1*/
			if(chip->use_batt1_flag){
				set_bit(BAT1_CHG_VLD, &chip->batt_status);
				clear_bit(BAT2_CHG_VLD, &chip->batt_status);
			}else{
				clear_bit(BAT1_CHG_VLD, &chip->batt_status);
				set_bit(BAT2_CHG_VLD, &chip->batt_status);
			}
			pr_buf_err("sgm:  use_batt1_flag%d, batt_status0x%x\n", chip->use_batt1_flag,chip->batt_status);
		}else{
			switch(batt_id_status){
				case SGM_ONLY_BAT1_EXIST:
					set_bit(BAT1_CHG_VLD, &chip->batt_status);
					clear_bit(BAT2_CHG_VLD, &chip->batt_status);
					break;
				case SGM_ONLY_BAT2_EXIST:
					clear_bit(BAT1_CHG_VLD, &chip->batt_status);
					set_bit(BAT2_CHG_VLD, &chip->batt_status);
					break;
				case SGM_BAT_ID_VLD:
					//calculate bat1, bat2 charging stage.
					sgm_chg_bat1_or_bat2();
					break;
				default :
					if(chip->use_batt1_flag){
						set_bit(BAT1_CHG_VLD, &chip->batt_status);
						clear_bit(BAT2_CHG_VLD, &chip->batt_status);
					}else{
						clear_bit(BAT1_CHG_VLD, &chip->batt_status);
						set_bit(BAT2_CHG_VLD, &chip->batt_status);
					}
					pr_buf_err("sgm: unknow batt status 0x%lx, battery%s is using now.\n", batt_id_status,(chip->use_batt1_flag ?"1":"2"));
					break;
			}
		}
		pr_buf_err("sgm:  batt_id_status 0x%x, batt_chg_status 0x%x\n", batt_id_status,batt_chg_status);
		/*check battery charging stage if change or not ,  not change, re_check*
		* battery health changed ,  charger input change can also to go config charging  current*/
		if((batt_chg_status !=(chip->batt_status &SGM_BAT_CHG_VLD)) ||(chip->typec_ret_vbus)
			||(input_mag_typec != input_mag_typec_last) ||(batt_health_last != chip->batt_health_status)){

			pr_buf_err("--sgm: batt_chg_status (last: 0x%lx  curr: 0x%lx)  change, typec_ret_vbus %d, redet.\n",
				batt_chg_status, (chip->batt_status & SGM_BAT_CHG_VLD), chip->typec_ret_vbus);
			//update battery charging status  and go set fcc nex
			if(batt_chg_status !=(chip->batt_status &SGM_BAT_CHG_VLD))
				batt_chg_status = (chip->batt_status & SGM_BAT_CHG_VLD);

			if(input_mag_typec != input_mag_typec_last){
				input_mag_typec_last = (chip->inputs & SGM_DOUBLE_CHARGER_VLD);
				pr_buf_err("sgm: input_mag_typec (last: 0x%lx  curr: 0x%lx)  change, update it .\n",input_mag_typec, input_mag_typec_last);
			}
		}else{
			//do next loop, check the battery charge status.
			pr_buf_info("sgm: batt_chg_status (last: 0x%lx  curr: 0x%lx) not change, typec_ret_vbus %d, redet.\n",
				batt_chg_status, (chip->batt_status & SGM_BAT_CHG_VLD), chip->typec_ret_vbus);
			goto re_det;
		}
		pr_buf_err("sgm: batt_chg_status (last: 0x%lx  curr: 0x%lx), typec_ret_vbus %d.\n",batt_chg_status, (chip->batt_status & SGM_BAT_CHG_VLD), chip->typec_ret_vbus);
		if(chip->typec_ret_vbus){
			chip->typec_ret_vbus = false;
			pr_buf_err("sgm: Clear typec_ret_vbus flag .\n");
		}
		batt_chg_flag = (batt_chg_status >> BAT1_CHG_VLD);
		switch(batt_chg_flag){
			pr_buf_err("sgm:  battery charging status 0x%x\n", (batt_chg_status >> BAT1_CHG_VLD));
			case SGM_BAT1_FC_BAT2_DISC:
				sgm_batt_charging_config(input_mag_typec, BAT1);
				break;
			case SGM_BAT1_DISC_BAT2_FC:
				sgm_batt_charging_config(input_mag_typec, BAT2);
				break;
			case SGM_BAT1_PRE_BAT2_FC:
			case SGM_BAT1_FC_BAT2_PRE:
				//need to do,
				sgm_batt_precharging_config(input_mag_typec,batt_chg_flag);
				pr_buf_err("sgm: set battery charge flagh %s 0x%x\n", ((batt_chg_flag==SGM_BAT1_PRE_BAT2_FC)?"SGM_BAT1_PRE_BAT2_FC":"SGM_BAT1_FC_BAT2_PRE"),batt_chg_flag);
				break;
			default :
				pr_buf_err("sgm: unknown battery charging status 0x%x\n", batt_chg_flag);
				break;
		}
		if(batt_health_last != chip->batt_health_status){
			pr_buf_info("sgm: batt_health_status %d, last batt_health_status %d .\n",chip->batt_health_status,batt_health_last);
			batt_health_last = chip->batt_health_status;
		}
	}else{
		global_batt_chip->force_next_chgstage =false;
		sgm_dischg_bat1_or_bat2(batt_id_status);
		if(gpio_get_value(global_batt_chip->typec_vbus_fcusb_en_gpio))
			sgm_enable_USBvbus_forSy6974(global_batt_chip,false);		
	}
	pr_buf_info("sgm: %s \n",  (ext_vbus_status|| typec_vbus_status)?"charging":"discharging");

re_det:
	schedule_delayed_work(&chip->chg_schedule_work,
			msecs_to_jiffies(CHG_SCHEDULE_PERIOD_MS));
	return;
}
static void sgm_bat1_det_work(struct work_struct *w)
{
	struct sgm_double_batt_chip *chip = container_of(w, struct sgm_double_batt_chip,
		bat1_det_work.work);
	/*battery id gpio =0, represent battery exist*/
	bool bat1_id = !gpio_get_value(chip->batt_id1_gpio);
	bool bat2_id = !gpio_get_value(chip->batt_id2_gpio);
	bool bat1_det_status =!!gpio_get_value(chip->bat1_det_int_gpio);
	unsigned long batt_det_stat = (bat1_det_status <<BAT1_DET_VLD);

	/*Update batt ID*/
	(bat1_det_status) ?(set_bit(BAT1_DET_VLD, &chip->batt_status)):(clear_bit(BAT1_DET_VLD, &chip->batt_status));

	pr_buf_err("sgm: batt_det_status 0x%lx, batt_status 0x%lx, \n", (chip->batt_status & SGM_BAT_DET_VLD),chip->batt_status);
	if( bat1_det_status != batt1_det_last_status){
		/*battery one  pack was open*/
		if(bat1_det_status){
			pr_buf_err("sgm: battery 1 pack was open = %d, bat1_id = %d, bat2_id= %d\n", bat1_det_status,bat1_id,bat2_id);
			if(bat2_id){
				/*battery two is exist, so change to change to battery Two, battery one maybe remove*/
				pr_buf_err("sgm: switch to baterry2 , since bat1 pack was open and batt2 exist\n");
				sgm_select_battery_and_thermal(BAT2);
				//clear battery1 chg status
				clear_bit(BAT1_ID_VLD, &chip->batt_status);
				clear_bit(BAT1_CHG_VLD, &chip->batt_status);
				clear_bit(BAT1_PRECHG_VLD, &chip->batt_status);
				sgm_check_power_supply_psy(chip);
				power_supply_changed(chip->batt_psy);
			}
		}else{
			/*battery1 pack was closed,check the battery id status*/
			(bat1_id) ?(set_bit(BAT1_ID_VLD, &chip->batt_status)):(clear_bit(BAT1_ID_VLD, &chip->batt_status));
		}
		batt1_det_last_status = bat1_det_status;
		chip->force_next_dischgstage = false;
		chip->force_next_chgstage = false;
	}

	pr_buf_info("sgm: batt_status 0x%lx, 0x%x\n", chip->batt_status, batt_det_stat);	
}
static void sgm_bat2_det_work(struct work_struct *w)
{
	struct sgm_double_batt_chip *chip = container_of(w, struct sgm_double_batt_chip,
		bat2_det_work.work);
	/*battery id gpio =0, represent battery exist*/
	bool bat1_id = !gpio_get_value(chip->batt_id1_gpio);
	bool bat2_id = !gpio_get_value(chip->batt_id2_gpio);

	bool bat2_det_status =!!gpio_get_value(chip->bat2_det_int_gpio);
	unsigned long batt_det_stat = (bat2_det_status << BAT2_DET_VLD);

	/*Update batt ID*/
	(bat2_det_status) ?(set_bit(BAT2_DET_VLD, &chip->batt_status)):(clear_bit(BAT2_DET_VLD, &chip->batt_status));
	
	if( bat2_det_status != batt2_det_last_status){
		/*batter TWO  pack was open*/
		if(bat2_det_status){
			pr_buf_info("sgm:  battery 2 pack was open = %d,bat1_id = %d, bat1_id= %d\n",bat2_det_status,bat1_id,bat2_id);
			if(bat1_id){
				/*battery two is exist, so change to change to battery Two, battery one maybe remove*/
				pr_buf_err("sgm: switch to baterry1 , since bat2 pack was open and batt1 exist\n");
				sgm_select_battery_and_thermal(BAT1);
				//clear battery2 chg status
				clear_bit(BAT2_ID_VLD, &chip->batt_status);
				clear_bit(BAT2_CHG_VLD, &chip->batt_status);
				clear_bit(BAT2_PRECHG_VLD, &chip->batt_status);
				sgm_check_power_supply_psy(chip);
				power_supply_changed(chip->batt_psy);
			}
		}else{
			/*battery2 pack was closed,check the battery id status*/
			(bat2_id) ?(set_bit(BAT2_ID_VLD, &chip->batt_status)):(clear_bit(BAT2_ID_VLD, &chip->batt_status));
		}
		batt2_det_last_status = bat2_det_status;
		chip->force_next_dischgstage = false;
		chip->force_next_chgstage = false;			
	}

	pr_buf_info("sgm: batt_status 0x%lx, 0x%x\n", chip->batt_status, batt_det_stat);	
}

static void sgm_batt_id1_work(struct work_struct *w)
{
	struct sgm_double_batt_chip *chip = container_of(w, struct sgm_double_batt_chip,
		batt_id1_work.work);
	/*battery id gpio =0, represent battery exist*/
	bool bat1_id = !gpio_get_value(chip->batt_id1_gpio);
	union power_supply_propval val = {0, };

	if( bat1_id != batt_id1_last_status){
		if(bat1_id){
			/*battery1 is insert.*/
			pr_buf_err("sgm: battery1 insert\n");
			if (sgm_check_power_supply_psy(chip))
				goto exit_id1;
			power_supply_set_property(chip->sy_chg1_psy,
			POWER_SUPPLY_PROP_FORCE_RECHARGE, &val);
			/*reset battery1  fuel gauge*/
			g_cw_reset_bat1_fuelgauge();
		}
		/*update battery1 id flag*/
		power_supply_changed(chip->batt_psy);
		batt_id1_last_status = bat1_id;
		chip->force_next_dischgstage = false;
		chip->force_next_chgstage = false;
	}
exit_id1:
	pr_buf_info("sgm: batt_status 0x%lx, %d\n", chip->batt_status, bat1_id);
}

static void sgm_batt_id2_work(struct work_struct *w)
{
	struct sgm_double_batt_chip *chip = container_of(w, struct sgm_double_batt_chip,
		batt_id2_work.work);
	/*battery id gpio =0, represent battery exist*/
	bool bat2_id = !gpio_get_value(chip->batt_id2_gpio);
	union power_supply_propval val = {0, };

	if( bat2_id != batt_id2_last_status){
		if(bat2_id){
			/*battery2 is insert.*/
			pr_buf_err("sgm: battery2 insert\n");
			if (sgm_check_power_supply_psy(chip))
				goto exit_id2;
			power_supply_set_property(chip->sy_chg2_psy,
			POWER_SUPPLY_PROP_FORCE_RECHARGE, &val);
			/*reset battery2  fuel gauge*/
			g_cw_reset_bat2_fuelgauge();
		}
		/*update battery1 id flag */
		power_supply_changed(chip->batt_psy);
		batt_id2_last_status = bat2_id;
		chip->force_next_dischgstage = false;
		chip->force_next_chgstage = false;
	}
exit_id2:
	pr_buf_info("sgm: batt_status 0x%lx, bat2_id %d\n", chip->batt_status, bat2_id);
}
static void sgm_hw_2batt_init(struct device *dev, struct sgm_double_batt_chip *chip)
{
	//init typec_ret_vbus
	chip->typec_ret_vbus = false;
	chip->force_next_chgstage = false;
	chip->force_next_dischgstage = false;
	sgm_enable_USBvbus_forSy6974(chip,false);
	sgm_enable_TypeCvbus(chip,false);
	schedule_delayed_work(&chip->batt_id1_work, msecs_to_jiffies(20));
	schedule_delayed_work(&chip->batt_id2_work, msecs_to_jiffies(20));

	schedule_delayed_work(&chip->chg_det_work, msecs_to_jiffies(5500));
	schedule_delayed_work(&chip->chg_schedule_work, msecs_to_jiffies(5000));
}
static int sgm_battID_init_status(struct sgm_double_batt_chip *chip){
		/*battery id gpio =0, represent battery exist*/
	bool bat1_id = !gpio_get_value(chip->batt_id1_gpio);
	bool bat2_id = !gpio_get_value(chip->batt_id2_gpio);
	int batt_ID =  ((bat1_id <<BAT1_ID_VLD)|(bat2_id << BAT2_ID_VLD));
	int bat1_cap = 50 ,  bat2_cap = 50;
	battery_id use_batt = BAT_MAX;

	batt_id2_last_status=bat2_id;
	batt_id1_last_status=bat1_id;

	if (!global_batt_chip) {
		pr_buf_err("sgm: err,no chip\n");
		return -1;
	}

	switch(batt_ID){
		case SGM_ONLY_BAT2_EXIST:
			set_bit(BAT2_ID_VLD,&chip->batt_status);
			use_batt = BAT2;
			break;
		case SGM_ONLY_BAT1_EXIST:
			set_bit(BAT1_ID_VLD,&chip->batt_status);
			use_batt = BAT1;
			break;
		case SGM_BAT_ID_VLD:
			set_bit(BAT1_ID_VLD,&chip->batt_status);
			set_bit(BAT2_ID_VLD,&chip->batt_status);
			/*Get battery1  and battery2 capacity*/
			bat1_cap = g_cw_get_bat1_capacity();
			bat2_cap = g_cw_get_bat2_capacity();
			if(bat1_cap > bat2_cap)
				use_batt = BAT1;
			else
				use_batt = BAT2;
			break;
		default :
			use_batt = BAT1;
			break;
	}
	if(use_batt == BAT1){
		chip->use_batt1_flag = true;
	}else{
		chip->use_batt1_flag = false;
	}
	sgm_select_battery_and_thermal(use_batt);
	pr_buf_err("sgm_init: batt_ID status 0x%x,battery cap [ %d  %d ]   select %s \n", batt_ID,bat1_cap,bat2_cap,(use_batt == BAT2) ? "battery 2":"battery 1");
	return 0;
}
static int sgm_2batt_parse_dt(struct device *dev,
		struct sgm_double_batt_chip *chip)
{
	struct device_node *np = dev->of_node;
	int ret = 0;
	/*charging stage range data from dts*/
	ret = read_range_data_from_node(np,
			"sgm,cap-stage-ranges",
			chip->step_chg_config->cap_chg_cfg,
			MAX_CAP_THRESHOLD, CHARGE_SOC_STAGE_HIGH);
	if (ret < 0) {
		pr_err("Read sgm,cap-stage-ranges from dts, rc=%d\n",ret);
	}
	
	/*dis charging stage range data from dts.*/
	ret = read_range_data_from_node(np,
			"sgm,dis-chg-cap-stage-ranges",
			chip->dischg_stage_config->cap_dischg_cfg,
			MAX_CAP_THRESHOLD, CHARGE_SOC_STAGE_HIGH);
	if (ret < 0) {
		pr_err("Read sgm,dis-chg-cap-stage-ranges dts, rc=%d\n",ret);
	}	


	/* typec vbus en gpio */
	ret = of_get_named_gpio(np, "sgm,typec-vbus-en-gpio", 0);
	chip->typec_vbus_en_gpio = ret;
	if(!gpio_is_valid(chip->typec_vbus_en_gpio )) {
		dev_err(dev, "invalid typec-vbus-en-gpio. ret = %d\n", ret);
		return ret;
	}
	ret = gpio_request(chip->typec_vbus_en_gpio,"sgm-typec-vbus-en-gpio");
	if (ret < 0) {
		dev_err(dev,
			"%s: gpio %d request failed. ret = %d\n",
			__func__, chip->typec_vbus_en_gpio, ret);
		return ret;
	}

	/*typec vbus fc usb en gpio*/
	ret = of_get_named_gpio(np, "sgm,typec-vbus-fc-usb-en-gpio", 0);
	chip->typec_vbus_fcusb_en_gpio = ret;
	if(!gpio_is_valid(chip->typec_vbus_fcusb_en_gpio )) {
		dev_err(dev, "invalid typec-vbus-fcusb-en-gpio. ret = %d\n", ret);
		return ret;
	}

	ret = gpio_request(chip->typec_vbus_fcusb_en_gpio, "sgm-typec-vbus-fc-usb-en-gpio");
	if (ret < 0) {
		dev_err(dev,
			"%s: gpio %d request failed. ret = %d\n",
			__func__, chip->typec_vbus_fcusb_en_gpio, ret);
		return ret;
	}

	/*mag-chg-det-gpio*/
	ret = of_get_named_gpio(np, "sgm,mag-chg-det-gpio", 0);
	chip->mag_chg_det_gpio = ret;
	if(!gpio_is_valid(chip->mag_chg_det_gpio)) {
		dev_err(dev, "invalid mag-chg-det gpio: %d\n", ret);
		return ret;
	}

	ret = gpio_request(chip->mag_chg_det_gpio, "sgm-mag-chg-det-gpio");
	if (ret < 0) {
		dev_err(dev,
			"%s: gpio %d request failed. ret = %d\n",
			__func__, chip->mag_chg_det_gpio, ret);
		return ret;
	}

	/*mag-bat1-sel-gpio*/
	ret = of_get_named_gpio(np, "sgm,mag-bat1-sel-gpio", 0);
	chip->bat1_sel_gpio = ret;
	if(!gpio_is_valid(chip->bat1_sel_gpio))
		dev_err(dev, "invalid bat1-sel-gpio rc = %d\n", ret);

	ret = gpio_request(chip->bat1_sel_gpio, "sgm-mag-bat1-sel-gpio");
	if (ret < 0)
		dev_err(dev,
			"gpio %d request failed. rc = %d\n", chip->bat1_sel_gpio, ret);


	/*mag-bat2-sel-gpio*/
	ret = of_get_named_gpio(np, "sgm,mag-bat2-sel-gpio", 0);
	chip->bat2_sel_gpio = ret;
	if(!gpio_is_valid(chip->bat2_sel_gpio))
		dev_err(dev, "invalid bat2-sel-gpio rc = %d\n", ret);

	ret = gpio_request(chip->bat2_sel_gpio, "sgm-mag-bat2-sel-gpio");
	if (ret < 0)
		dev_err(dev,
			"gpio %d request failed. rc = %d\n", chip->bat2_sel_gpio, ret);


	/*mag-batt id1-gpio*/
	ret = of_get_named_gpio(np, "sgm,mag-batt-id1-gpio", 0);
	chip->batt_id1_gpio= ret;
	if(!gpio_is_valid(chip->batt_id1_gpio))
		dev_err(dev, "invalid batt-id1-gpio rc = %d\n", ret);

	ret = gpio_request(chip->batt_id1_gpio, "sgm-batt-id1-gpio");
	if (ret < 0)
		dev_err(dev,
			"gpio %d request failed. rc = %d\n", chip->batt_id1_gpio, ret);

	/*mag-batt id2-gpio*/
	ret = of_get_named_gpio(np, "sgm,mag-batt-id2-gpio", 0);
	chip->batt_id2_gpio= ret;
	if(!gpio_is_valid(chip->batt_id2_gpio))
		dev_err(dev, "invalid batt-id2-gpio rc = %d\n", ret);

	ret = gpio_request(chip->batt_id2_gpio, "sgm-batt-id2-gpio");
	if (ret < 0)
		dev_err(dev,
			"gpio %d request failed. rc = %d\n", chip->batt_id2_gpio, ret);

	/*mag-bat1_det_int*/
	ret = of_get_named_gpio(np, "sgm,mag-bat1-det-int-gpio", 0);
	chip->bat1_det_int_gpio= ret;
	if(!gpio_is_valid(chip->bat1_det_int_gpio))
		dev_err(dev, "invalid batt-id1-gpio rc = %d\n", ret);

	ret = gpio_request(chip->bat1_det_int_gpio, "sgm-bat1-det-int-gpio");
	if (ret < 0)
		dev_err(dev,
			"gpio %d request failed. rc = %d\n", chip->bat1_det_int_gpio, ret);

	/*mag-bat2_det_int*/
	ret = of_get_named_gpio(np, "sgm,mag-bat2-det-int-gpio", 0);
	chip->bat2_det_int_gpio= ret;
	if(!gpio_is_valid(chip->bat2_det_int_gpio))
		dev_err(dev, "invalid batt-id2-gpio rc = %d\n", ret);

	ret = gpio_request(chip->bat2_det_int_gpio, "sgm-bat2-det-int-gpio");
	if (ret < 0)
		dev_err(dev,
			"gpio %d request failed. rc = %d\n", chip->bat2_det_int_gpio, ret);

	/* mag batt thermal sel gpio*/
	ret = of_get_named_gpio(np, "sgm,mag-batt-thermal-sel-gpio", 0);
	chip->batt_thermal_set_gpio= ret;

	if(!gpio_is_valid(chip->batt_thermal_set_gpio)) {
		dev_err(dev, "invalid mag-batt-thermal-sel-gpio. ret = %d\n", ret);
		return ret;
	}
	ret = gpio_request(chip->batt_thermal_set_gpio,"sgm-mag-batt-thermal-sel-gpio");
	if (ret < 0) {
		dev_err(dev,
			"%s: gpio %d request failed. ret = %d\n",
			__func__, chip->batt_thermal_set_gpio, ret);
		return ret;
	}


	return 0;
}

static ssize_t sgm_mode_show(struct device *dev,
                struct device_attribute *attr, char *buf)
{
	 int mode = 0;

	 if (test_bit(TYPEC_OTG_VLD, &global_batt_chip->inputs))
		mode = usb_otg_mode;
	else if ((test_bit(MAG_BUS_VLD, &global_batt_chip->inputs))
		&& (test_bit(TYPEC_BUS_VLD, &global_batt_chip->inputs)))
		mode = magcon_usb_chg_mode;
	else if (test_bit(MAG_BUS_VLD, &global_batt_chip->inputs))
		mode = magcon_chg_mode;
	else if (test_bit(TYPEC_BUS_VLD, &global_batt_chip->inputs))
		mode = usb_chg_mode;
	else
		mode = sgm_default_mode;

	pr_buf_info("sgm: global_batt_chip->inputs=0x%x\n", global_batt_chip->inputs);

	return sprintf(buf, "%d\n", mode);
}

static ssize_t typec_attached_state_show(struct device *dev,
                struct device_attribute *attr, char *buf)
{
	int attached_state = 0;
	bool typec_vbus_status = !gpio_get_value(global_batt_chip->mag_chg_det_gpio);

	if (global_batt_chip->typec_otg_present || typec_vbus_status)
		attached_state = 1;
	else
		attached_state = 0;

	pr_buf_info("attached_state=0x%x\n", attached_state);

	return sprintf(buf, "%d\n", attached_state);
}

static ssize_t double_charger_show(struct device *dev,
                struct device_attribute *attr, char *buf)
{
	int double_charger_state = sgm_double_charger_ok();

	pr_buf_info("attached_state=0x%x\n", double_charger_state);

	return sprintf(buf, "%d\n", double_charger_state);
}
static ssize_t batteryID_status_show(struct device *dev,
                struct device_attribute *attr, char *buf)
{
	pr_buf_info("Using %s\n",( global_batt_chip->use_batt1_flag)? "battery 1":"battery 2");

	return sprintf(buf, "%d\n", (!global_batt_chip->use_batt1_flag));
}
static ssize_t batteryID_status_store(struct device *dev,
                struct device_attribute *attr,const char *buf, size_t count)
{
	int ret = 0;
	u8 battery_mode = 0;
	ret = kstrtou8(buf,0,&battery_mode);
	if(ret < 0){
		pr_buf_info("Get battery_mode error %d\n", ret);
		return ret;
	}
	if((battery_mode != BAT1) && (battery_mode !=BAT2)){
		pr_buf_info("Invalid  battery_mode (0 or 1)\n", ret);
		return -EINVAL;
	}
	pr_buf_info("battery_mode %s\n", (!!battery_mode )? "BAT2":"BAT1");
	if(battery_mode == BAT2){
		ret = gpio_direction_output(global_batt_chip->bat1_sel_gpio, 1);
		ret = gpio_direction_output(global_batt_chip->bat2_sel_gpio, 1);
		ret = gpio_direction_output(global_batt_chip->batt_thermal_set_gpio, 1);
		global_batt_chip->use_batt1_flag = false;
	}else{
		ret = gpio_direction_output(global_batt_chip->bat1_sel_gpio, 0);
		ret = gpio_direction_output(global_batt_chip->bat2_sel_gpio, 0);
		ret = gpio_direction_output(global_batt_chip->batt_thermal_set_gpio, 0);	
		global_batt_chip->use_batt1_flag = true;
	}
	return count;
}

static DEVICE_ATTR(sgm2540_mode, S_IRUGO, sgm_mode_show, NULL);
static DEVICE_ATTR(typec_attached_state, S_IRUGO, typec_attached_state_show, NULL);
static DEVICE_ATTR(double_charger, S_IRUGO, double_charger_show, NULL);
static DEVICE_ATTR(batteryID_status,  S_IWUSR | S_IRUGO, batteryID_status_show, batteryID_status_store);

static struct attribute *sgm_2batt_sys_node_attrs[] = {
	&dev_attr_sgm2540_mode.attr,
	&dev_attr_typec_attached_state.attr,
	&dev_attr_double_charger.attr,
	&dev_attr_batteryID_status.attr,
	NULL,
};

static struct attribute_group sgm_2batt_sys_node_attr_group = {
	.attrs = sgm_2batt_sys_node_attrs,
};

static int  sgm_2batt_debugfs_init(struct sgm_double_batt_chip *chip)
{
	int ret;

	ret = his_register_sysfs_attr_group(&sgm_2batt_sys_node_attr_group);
	if (ret < 0)
		pr_buf_err("Error create sgm_sys_node_attr_group %d\n", ret);

	return 0;
}

static int sgm_2batt_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct sgm_double_batt_chip *chip;

	pr_err("sgm:sgm_double_batteries probe start\n");
	
	chip = devm_kzalloc(&pdev->dev, sizeof(*chip), GFP_KERNEL);
	if (!chip) {
		dev_err(&pdev->dev, "Unable to allocate memory\n");
		return -ENOMEM;
	}
	pr_buf_err("sgm:  step_chg_config 0x%x,dischg_stage_config 0x%x \n",chip->step_chg_config,chip->dischg_stage_config);
	pr_buf_err("sgm:  batt_psy 0x%x,sy_chg1_psy 0x%x, sy_chg2_psy 0x%x, global_smb_charger 0x%x\n",chip->batt_psy,chip->sy_chg1_psy,chip->sy_chg2_psy,global_smb_charger);
	
	chip->step_chg_config = devm_kzalloc(&pdev->dev,
			sizeof(struct cap_step_chg_cfg), GFP_KERNEL);
	if (!chip->step_chg_config){
		dev_err(&pdev->dev, "Unable to allocate memory for cap_step_chg_cfg\n");
		return -ENOMEM;
	}

	chip->dischg_stage_config = devm_kzalloc(&pdev->dev,
		sizeof(struct cap_step_dischg_cfg), GFP_KERNEL);
	if(!chip->dischg_stage_config){

		dev_err(&pdev->dev, "Unable to allocate memory for cap_step_dischg_cfg\n");
		return -ENOMEM;
	}
	pr_buf_err("sgm:  batt_psy 0x%x,sy_chg1_psy 0x%x, sy_chg2_psy 0x%x, global_smb_charger 0x%x\n",chip->batt_psy,chip->sy_chg1_psy,chip->sy_chg2_psy,global_smb_charger);

	chip->pdev = pdev;
	dev_set_drvdata(&pdev->dev, chip);

	if (pdev->dev.of_node) {
		ret = sgm_2batt_parse_dt(&pdev->dev, chip);
		if (ret) {
			dev_err(&pdev->dev,"%s: sgm_parse_dt() err\n", __func__);
			ret = -EPROBE_DEFER;
			goto err_parse_dt;
		}
	} else {
		dev_err(&pdev->dev, "No dts data\n");
		ret = -EPROBE_DEFER;
		goto err_parse_dt;
	}

	global_batt_chip = chip;
	INIT_DELAYED_WORK(&chip->chg_det_work, sgm_chg_det_work);
	INIT_DELAYED_WORK(&chip->chg_schedule_work, sgm_chg_schedule_work);
	INIT_DELAYED_WORK(&chip->bat1_det_work,sgm_bat1_det_work);
	INIT_DELAYED_WORK(&chip->bat2_det_work,sgm_bat2_det_work);
	/*battery id work for battery remove and insert*/
	INIT_DELAYED_WORK(&chip->batt_id1_work,sgm_batt_id1_work);
	INIT_DELAYED_WORK(&chip->batt_id2_work,sgm_batt_id2_work);
	chip->mag_chg_det_irq = gpio_to_irq(chip->mag_chg_det_gpio);
	/*battery detect pin request interrupts*/
	chip->bat1_det_int_irq = gpio_to_irq(chip->bat1_det_int_gpio);
	chip->bat2_det_int_irq = gpio_to_irq(chip->bat2_det_int_gpio);

	/*battery id pins to request interrupts*/
	chip->batt_id1_init_irq = gpio_to_irq(chip->batt_id1_gpio);
	chip->batt_id2_init_irq = gpio_to_irq(chip->batt_id2_gpio);

	ret = request_threaded_irq(chip->mag_chg_det_irq,
			NULL,
			sgm_2batt_mag_chg_irq,
			IRQF_TRIGGER_RISING |
			IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
			"mag_chg_det_irq", chip);
	if (ret) {
		dev_err(&pdev->dev, "request irq failed for typec chg det\n");
		goto err_parse_dt;
	}

	ret = request_threaded_irq(chip->bat1_det_int_irq,
			NULL,
			sgm_bat1_det_int_irq,
			IRQF_TRIGGER_RISING |
			IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
			"bat1_det_int_irq", chip);
	if (ret) {
		dev_err(&pdev->dev, "request irq failed for sgm_bat1_det\n");
		goto err_parse_dt;
	}

	ret = request_threaded_irq(chip->bat2_det_int_irq,
			NULL,
			sgm_bat2_det_int_irq,
			IRQF_TRIGGER_RISING |
			IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
			"bat2_det_int_irq", chip);
	if (ret) {
		dev_err(&pdev->dev, "request irq failed for sgm_bat2_det\n");
		goto err_parse_dt;
	}

	ret = request_threaded_irq(chip->batt_id1_init_irq,
			NULL,
			sgm_batt_id1_int_irq,
			IRQF_TRIGGER_RISING |
			IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
			"batt_id1_int_irq", chip);
	if (ret) {
		dev_err(&pdev->dev, "request irq failed for sgm_batt_id1\n");
		goto err_parse_dt;
	}

	ret = request_threaded_irq(chip->batt_id2_init_irq,
			NULL,
			sgm_batt_id2_int_irq,
			IRQF_TRIGGER_RISING |
			IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
			"batt_id2_int_irq", chip);
	if (ret) {
		dev_err(&pdev->dev, "request irq failed for sgm_batt_id2\n");
		goto err_parse_dt;
	}

	enable_irq_wake(chip->mag_chg_det_irq);
	enable_irq_wake(chip->bat1_det_int_irq);
	enable_irq_wake(chip->bat2_det_int_irq);

	enable_irq_wake(chip->batt_id1_init_irq);
	enable_irq_wake(chip->batt_id2_init_irq);

	set_bit(SGM_INIT, &chip->inputs);
	//if battery1 , 2 all exists. 
	set_bit(BAT_INIT,&chip->batt_status);
	sgm_battID_init_status(chip);
	sgm_hw_2batt_init(&pdev->dev, chip);
	
	sgm_2batt_debugfs_init(chip);

	pr_buf_err("sgm:sgm_double_batteries success\n");
	return 0;

err_parse_dt:
	//devm_kfree(&pdev->dev,chip);
	dev_set_drvdata(&pdev->dev, NULL);
	return ret;
}

static void  sgm_2batt_shutdown(struct platform_device *pdev)
{
	struct sgm_double_batt_chip *chip = platform_get_drvdata(pdev);

	if (chip) {
		cancel_delayed_work_sync(&chip->chg_det_work);
	}

	global_batt_chip = NULL;
}

static int sgm_2batt_remove(struct platform_device *pdev)
{
	struct sgm_double_batt_chip *chip = platform_get_drvdata(pdev);

	if (chip) {
		cancel_delayed_work_sync(&chip->chg_det_work);
	}

	global_batt_chip = NULL;

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int sgm_2batt_suspend(struct device *dev)
{
	//struct sgm_chip *chip  = dev_get_drvdata(dev);

	return 0;
}

static int sgm_2batt_resume(struct device *dev)
{
	//struct sgm_chip *chip  = dev_get_drvdata(dev);

	return 0;
}
#endif

static SIMPLE_DEV_PM_OPS(sgm_2batt_pm_ops, sgm_2batt_suspend,
			  sgm_2batt_resume);

static struct of_device_id sgm_2batt_match_table[] = {
	{	.compatible = "sgmicro,double-batteries",},
	{}
};

static struct platform_driver sgm_2batt_driver = {
	.driver		= {
		.name		= "sgm_double_batteries",
		.owner		= THIS_MODULE,
		.of_match_table	=sgm_2batt_match_table,
		.pm	= &sgm_2batt_pm_ops,
	},
	.probe		= sgm_2batt_probe,
	.remove		= sgm_2batt_remove,
	.shutdown 	= sgm_2batt_shutdown,
};

module_platform_driver(sgm_2batt_driver);

MODULE_DESCRIPTION("SGM Double Charger Driver");
MODULE_AUTHOR("lishaoxiang@hisense.com");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:sgm");
