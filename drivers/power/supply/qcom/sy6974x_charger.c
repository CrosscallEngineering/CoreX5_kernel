/*
 * SY6974 charger driver
 *
 * Version: v1.0.0
 *
 * Copyright (C) 2021 Hisense Corporation, All rights reserved
 *
 *  Author: lishaoxiang <lishaoxiang@hisense.com>
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#define pr_fmt(fmt) "--PM-SY6974: %s: " fmt, __func__

#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/regmap.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/device.h>
#include <linux/of_device.h>
#include <linux/of_irq.h>
#include <linux/workqueue.h>
#include <linux/pmic-voter.h>
#include <linux/string.h>
#include <linux/extcon.h>
//#include <linux/extcon-provider.h>
#include <linux/debugfs.h>
#include "smb5-lib.h"
#include <linux/power/sy6974x_charger.h>
#include <linux/his_debug_base.h>
#define SY6974_REG_0				0x0
#define SY6974_REG_1				0x1
#define SY6974_REG_2				0x2
#define SY6974_REG_3				0x3
#define SY6974_REG_4				0x4
#define SY6974_REG_5				0x5
#define SY6974_REG_6				0x6
#define SY6974_REG_7				0x7
#define SY6974_REG_8				0x8
#define SY6974_REG_9				0x9
#define SY6974_REG_A				0xa
#define SY6974_REG_B				0xb

#define BATTERY_PSY_NAME			"battery"
#define BIT_DP_DM_BC_ENB				BIT(0)

#define SY6974_REG_IINLIM_BASE			100

#define SY6974_REG_ICHG_LSB			60

#define SY6974_REG_ICHG_MASK			GENMASK(5, 0)

#define SY6974_REG_CHG_MASK			GENMASK(4, 4)

#define SY6974_REG_CHG_SHIFT			4

#define SY6974_REG_RESET_MASK		GENMASK(6, 6)

#define SY6974_REG_OTG_MASK			GENMASK(5, 5)

#define SY6974_REG_WATCHDOG_MASK		GENMASK(6, 6)

#define SY6974_REG_WATCHDOG_CTL_MASK		GENMASK(5, 4)

#define SY6974_REG_TERMINAL_VOLTAGE_MASK	GENMASK(7, 3)
#define SY6974_REG_TERMINAL_VOLTAGE_SHIFT	3

#define SY6974_REG_TERMINAL_CUR_MASK		GENMASK(3, 0)

#define SY6974_REG_VINDPM_VOLTAGE_MASK		GENMASK(3, 0)
#define SY6974_REG_OVP_MASK					GENMASK(7, 6)
#define SY6974_REG_OVP_SHIFT					6

#define SY6974_REG_LIMIT_CURRENT_MASK		GENMASK(4, 0)

#define SY6974_REG_EN_TIMER_MASK				GENMASK(3, 3)
#define SY6974_REG_EN_TIMER_SHIFT      			3

#define SNK_DAM_MASK				GENMASK(6, 4)
#define SNK_DAM_500MA_BIT			BIT(6)

//REG08 status bits, Read only.
#define SY6974_REG_VBUS_STAT_MASK			GENMASK(7, 5)
#define SY6974_REG_VBUS_STAT_SHIFT			5
#define SY6974_REG_CHRG_STAT_MASK			GENMASK(4, 3)
#define SY6974_REG_CHRG_STAT_SHIFT			3
#define SY6974_REG_VBUS_GOOD_STAT_BIT		BIT(2)
#define SY6974_REG_THERM_STAT_BIT			BIT(1)
#define SY6974_REG_VSYS_STAT_BIT				BIT(0)

//REG09 fault bits, Read only.
#define SY6974_REG_WATCHDOG_FAULT_BIT      	BIT(7)
#define SY6974_REG_BOOST_FAULT_BIT			BIT(6)
#define SY6974_REG_CHRG_FAULT_MASK			GENMASK(5, 4)
#define SY6974_REG_BATTERY_FAULT_BIT			BIT(3)
#define SY6974_REG_NTC_FAULT_MASK				GENMASK(2, 0)

//REG0A stat bits, Read and R/W.
#define SY6974_REG_VBUS_GOOD_BIT      			BIT(7)
#define SY6974_REG_INPUT_VOLTAGE_STAT_BIT	BIT(6)
#define SY6974_REG_INPUT_CURRENT_STAT_BIT	BIT(5)
#define SY6974_REG_INPUT_OVLO_STAT_BIT		BIT(2)
#define SY6974_REG_VIN_INT_PULSE_BIT			BIT(1)
#define SY6974_REG_IIN_INT_PULSE_BIT			BIT(0)

#define SY6974_OTG_VALID_MS					500
#define SY6974_FEED_WATCHDOG_VALID_MS		50
#define SY6974_OTG_RETRY_TIMES				5
#define SY6974_LIMIT_CURRENT_MAX				3000000

#define FCHG_OVP_6P5V					6500

#define SDP_CURRENT_500MA				500000
#define DCP_CURRENT_2000MA				2000000
#define CDP_CURRENT_1500MA				1500000
#define FLOAT_CURRENT_900MA				900000
#define FLOAT_CURRENT_2000MA				2200000
#define FCHG_CURRENT_2000MA				2000000
#define UNKNOWN_CURRENT_500MA		500000

#define FCC_ICHG_CURRENT_500MA			500000
#define FCC_ICHG_CURRENT_2000MA			2000000
#define FCC_WARM_CURRENT				1100000
#define FCC_COLD_CURRENT					780000

#define TERMINATION_CURRENT_LIMIT		500
#define CHARGE_WARM_VOLTAGE_LIMIT		4110
#define CHARGE_NORMAL_VOLTAGE_LIMIT	4200
#define MAX_RETRIES_TIMES	(3)
enum sy6974_charger_type {
	NO_INPUT = 0,
	SDP_TYPE = 1,
	CDP_TYPE = 2,
	ADAPTER_TYPE = 3,
	UNKNOWN_TYPE = 5,
	NON_STANDARD_TYPE = 6,
	OTG_TYPE = 7,
};

enum sy6974_charger_state {
	CHARGER_DISABLE = 0,
	CHARGER_PRE_CHARGE = 1,
	CHARGER_FAST_CHARGING = 2,
	CHARGER_TERMINATED = 3,
};

enum sy6974_batt_state {
	BATT_NORMAL = 0,
	BATT_WARM = 2,
	BATT_COOL = 3,
	BATT_COLD = 5,
	BATT_HOT = 6,
};

enum sy6974x_chip {
	SY6974BAT1=0,
	SY6974BAT2,
};
/*
static char *sy6974x_chip_name[] = {
	"sy6974_bat1",
	"sy6974_bat2",
};
*/
struct sy6974_charge_current {
	int sdp_cur;
	int dcp_cur;
	int cdp_cur;
	int float_cur;
	int unknown_cur;
};
struct sy6974x_device {
	struct device *dev;
	struct mutex lock;
	struct power_supply *charger;
	struct power_supply_desc charger_desc;
	struct power_supply *batt_psy;
	struct sy6974_charge_current cur;
	struct dentry		*dfs_root;
	bool good_vbus_detected;
	bool charging;
	bool usb_mode;
	bool isHWInit;
	u32 limit;
	u32 charger_type;
	u32 charger_status;
	u32 batt_health_status;
	bool therm_status;
	bool vsys_status;
	u32 chg_en_gpio;
	int chg_en_gpio_high;
	u32 int_gpio;
	int int_irq;
	int voltage_max_microvolt;
	enum sy6974x_chip chip;
	char *name;
	int id;
};

/*each registered chip must have unique id*/
static DEFINE_IDR(sy6974x_id);
static DEFINE_MUTEX(sy6974x_id_mutex);
static DEFINE_MUTEX(sy6974x_i2c_mutex);
extern struct smb_charger *global_smb_charger;
extern bool sgm_2batt_typec_charger_ok(void);
extern unsigned long sgm_2bat_get_chg_flag(void);

static int sy6974_read(struct sy6974x_device *info, u8 reg, u8 *data)
{
	int ret;
	struct i2c_client *client = to_i2c_client(info->dev);
	if(!client->adapter)
		return -ENODEV;
	mutex_lock(&sy6974x_i2c_mutex);
	ret = i2c_smbus_read_byte_data(client, reg);
	mutex_unlock(&sy6974x_i2c_mutex);
	if (ret < 0)
		return ret;

	*data = ret;
	return 0;
}

static int sy6974_write(struct sy6974x_device *info, u8 reg, u8 data)
{
	int ret;
	struct i2c_client *client = to_i2c_client(info->dev);
	if(!client->adapter)
		return -ENODEV;
	mutex_lock(&sy6974x_i2c_mutex);
	ret = i2c_smbus_write_byte_data(client, reg, data);
	mutex_unlock(&sy6974x_i2c_mutex);
	return ret;
}

static int sy6974_update_bits(struct sy6974x_device *info, u8 reg,
			       u8 mask, u8 data)
{
	u8 v;
	int ret;

	ret = sy6974_read(info, reg, &v);
	if (ret < 0)
		return ret;

	v &= ~mask;
	v |= (data & mask);

	return sy6974_write(info, reg, v);
}

static int
sy6974_charger_set_vindpm(struct sy6974x_device *info, u32 vol)
{
	u8 reg_val;

	if (vol <= 3900)
		reg_val = 0x0;
	else if (vol >= 5400)
		reg_val = 0x0f;
	else
		reg_val = (vol - 3900) / 100;

	pr_buf_info("reg_val is 0x%x\n", reg_val);
	return sy6974_update_bits(info, SY6974_REG_6,
				   SY6974_REG_VINDPM_VOLTAGE_MASK, reg_val);
}

static int
sy6974_charger_set_ovp(struct sy6974x_device *info, u32 vol)
{
	u8 reg_val;

	if (vol <= 5500)
		reg_val = 0x0;
	else if (vol > 5500 && vol <= 6500)
		reg_val = 0x01;
	else if (vol > 6500 && vol <= 10500)
		reg_val = 0x02;
	else
		reg_val = 0x03;

	return sy6974_update_bits(info, SY6974_REG_6,
				   SY6974_REG_OVP_MASK,
				   reg_val << SY6974_REG_OVP_SHIFT);
}

static int
sy6974_charger_set_termina_vol(struct sy6974x_device *info, u32 vol)
{
	u8 reg_val;

	if (vol <= 3850) {
		reg_val = 0x0;
	} else if (vol >= 4610) {
		reg_val = 0x2d;
	} else {
		reg_val = (vol - 3848) / 32;
	}

	pr_buf_info("reg_val is 0x%x\n", reg_val);
	return sy6974_update_bits(info, SY6974_REG_4,
				   SY6974_REG_TERMINAL_VOLTAGE_MASK,
				   reg_val << SY6974_REG_TERMINAL_VOLTAGE_SHIFT);
}

#if 0
static int
sy6974_charger_set_termina_cur(struct sy6974x_device *info, u32 cur)
{
	u8 reg_val;

	if (cur <= 60)
		reg_val = 0x0;
	else if (cur >= 960)
		reg_val = 0xf;
	else
		reg_val = (cur - 60) / 60;

	return sy6974_update_bits(info, SY6974_REG_3,
				   SY6974_REG_TERMINAL_CUR_MASK,
				   reg_val);
}
#endif

static int sy6974_update_charger_status(struct sy6974x_device *info)
{
	u8 reg_val;
	int ret;

	ret = sy6974_read(info, SY6974_REG_8, &reg_val);
	if (ret < 0)
		return ret;

	//pr_buf_info("REG_8 val is 0x%x\n", reg_val);
	switch ((reg_val & SY6974_REG_VBUS_STAT_MASK) >> SY6974_REG_VBUS_STAT_SHIFT) {
	case SDP_TYPE:
	case ADAPTER_TYPE:
		info->charger_type = POWER_SUPPLY_TYPE_USB_FLOAT;
		break;
	case OTG_TYPE:
		info->charger_type = POWER_SUPPLY_TYPE_UFP;
		break;
	case NO_INPUT:
	default:
		info->charger_type = POWER_SUPPLY_TYPE_UNKNOWN;
		break;
	}

	if (global_smb_charger && (!sgm_2batt_typec_charger_ok())) {
		global_smb_charger->real_charger_type = info->charger_type;
		dev_info(info->dev,"real_charger_type set to %d\n", global_smb_charger->real_charger_type);
	}

	switch ((reg_val & SY6974_REG_CHRG_STAT_MASK) >> SY6974_REG_CHRG_STAT_SHIFT) {
	case CHARGER_PRE_CHARGE:
	case CHARGER_FAST_CHARGING:
		info->charger_status = POWER_SUPPLY_STATUS_CHARGING;
		break;
	case CHARGER_TERMINATED:
		info->charger_status = POWER_SUPPLY_STATUS_FULL;
		pr_buf_info("charging is completed, status set to full\n");
		break;
	case CHARGER_DISABLE:
	default:
		info->charger_status = POWER_SUPPLY_STATUS_DISCHARGING;
		break;
	}

	//Input power source is good.
	if (reg_val & SY6974_REG_VBUS_GOOD_STAT_BIT)
		info->good_vbus_detected = true;
	else
		info->good_vbus_detected = false;

	if (reg_val & SY6974_REG_THERM_STAT_BIT) {
		info->therm_status = true;
	} else
		info->therm_status = false;

	if (reg_val & SY6974_REG_VSYS_STAT_BIT) {
		info->vsys_status = true;
	} else
		info->vsys_status = false;

	pr_buf_err("REG_8 val is 0x%x, charger_type %d, charger_status %d, good_vbus %d, therm_status %d, vsys_status %d\n",
		reg_val, info->charger_type, info->charger_status, info->good_vbus_detected, info->therm_status, info->vsys_status);

	return ret;
}

static int sy6974_update_fault_status(struct sy6974x_device *info)
{
	u8 reg_val;
	int ret;

	ret = sy6974_read(info, SY6974_REG_9, &reg_val);
	if (ret < 0)
		return ret;

	pr_buf_info("REG_9 val is 0x%x\n", reg_val);

	if (reg_val & SY6974_REG_WATCHDOG_FAULT_BIT)
		dev_err(info->dev,"REG_9 WATCHDOG_FAULT\n");

	if (reg_val & SY6974_REG_BOOST_FAULT_BIT)
		dev_err(info->dev,"REG_9 BOOST_FAULT\n");

	if (reg_val & SY6974_REG_CHRG_FAULT_MASK)
		dev_err(info->dev,"REG_9 CHRG_FAULT 0x%x\n",
			reg_val & SY6974_REG_CHRG_FAULT_MASK);

	if (reg_val & SY6974_REG_BATTERY_FAULT_BIT)
		dev_err(info->dev,"REG_9 BATTERY_FAULT\n");

	return ret;
}

static int sy6974_charger_set_fcc_current(struct sy6974x_device *info,
				       u32 cur)
{
	u8 reg_val;

	pr_buf_info("FCC current = %d\n", cur);
	cur = cur / 1000;
	if (cur > 3000) {
		reg_val = 0x32;
	} else {
		reg_val = cur / SY6974_REG_ICHG_LSB;
		reg_val &= SY6974_REG_ICHG_MASK;
	}

	return sy6974_update_bits(info, SY6974_REG_2,
				   SY6974_REG_ICHG_MASK,
				   reg_val);
}

static int sy6974_charger_get_fcc_current(struct sy6974x_device *info,
				       u32 *cur)
{
	u8 reg_val;
	int ret;

	ret = sy6974_read(info, SY6974_REG_2, &reg_val);
	if (ret < 0)
		return ret;

	reg_val &= SY6974_REG_ICHG_MASK;
	*cur = reg_val * SY6974_REG_ICHG_LSB * 1000;
	dev_info(info->dev,"get_current = %d\n", *cur);

	return 0;
}

static int sy6974_charger_get_charge_voltage(struct sy6974x_device *info,
				       u32 *charge_voltage)
{
	u8 reg_val;
	int ret;

	ret = sy6974_read(info, SY6974_REG_4, &reg_val);
	if (ret < 0)
		return ret;

	reg_val &= SY6974_REG_TERMINAL_VOLTAGE_MASK;
	*charge_voltage = 3856 + 32 * (reg_val >> 3);
	pr_buf_info("get charge_voltage = %d mV\n", *charge_voltage);

	return 0;
}

static int sy6974_charger_set_input_current(struct sy6974x_device *info,
				  u32 limit_cur)
{
	u8 reg_val;
	int ret;

	pr_buf_err("USBIN limit_current = %d\n", limit_cur);
	if (limit_cur >= SY6974_LIMIT_CURRENT_MAX)
		limit_cur = SY6974_LIMIT_CURRENT_MAX;

	limit_cur = limit_cur / 1000;
	//input current limit value = 100 + 100 * reg_val
	reg_val = (limit_cur / SY6974_REG_IINLIM_BASE) - 1;

	ret = sy6974_update_bits(info, SY6974_REG_0,
				  SY6974_REG_LIMIT_CURRENT_MASK,
				  reg_val);
	if (ret)
		dev_err(info->dev, "set SY6974 limit cur failed (%d)\n",ret);

	return ret;
}

static u32 sy6974_charger_get_input_current(struct sy6974x_device *info,
				  u32 *limit_cur)
{
	u8 reg_val;
	int ret;

	ret = sy6974_read(info, SY6974_REG_0, &reg_val);
	if (ret < 0)
		return ret;

	reg_val &= SY6974_REG_LIMIT_CURRENT_MASK;
	*limit_cur = (reg_val + 1) * SY6974_REG_IINLIM_BASE * 1000;
	if (*limit_cur >= SY6974_LIMIT_CURRENT_MAX)
		*limit_cur = SY6974_LIMIT_CURRENT_MAX;
	pr_buf_info("get_limit_current = %d\n",*limit_cur);

	return 0;
}

static int sy6974_charger_hw_init(struct sy6974x_device *info);
#define MAX_GOOD_VBUS_RETRIES	10
static int sy6974_waitfor_goodvbus(struct sy6974x_device *info)
{
	int rc=0, count = 0;
	do {
		sy6974_update_charger_status(info);
		if(info->good_vbus_detected){
			pr_buf_info("detect good vbus (%d)\n", (count+1));
			break;
		}
		msleep(100);
		count++;
	} while (count < MAX_GOOD_VBUS_RETRIES);

	if (count == MAX_GOOD_VBUS_RETRIES) {
		pr_buf_err("####Not detect Good vbus\n");
		rc = -1;
	}
	return rc;
}
static int sy6974_charger_start_charge(struct sy6974x_device *info)
{
	int usbin_cur, ret;
	unsigned long batt_chg_flag = 0;
	struct device *dev = info->dev;
	int retry_count = 0;
	if(!info->isHWInit){
		ret = sy6974_charger_hw_init(info);
		if (ret) {
			dev_err(dev, "sy6974_charger_hw_init failed\n");
			goto err_hw_init;
		}
		info->isHWInit = true;
	}
	sy6974_update_charger_status(info);
	if (!info->good_vbus_detected){
		//update the good vbus detet
		ret = sy6974_waitfor_goodvbus(info);
		if(ret){
			goto out;
		}
	}
	batt_chg_flag = sgm_2bat_get_chg_flag();

	if (global_smb_charger && (sgm_2batt_typec_charger_ok())) {
		if(batt_chg_flag & 0xc){
		 	info->charger_type = global_smb_charger->real_charger_type ;
		}
		dev_info(info->dev,"TypeC real_charger_type is  %d, and precharge flag %d\n", global_smb_charger->real_charger_type);
	}else{
		//force charger type to FLOAT
		info->charger_type = POWER_SUPPLY_TYPE_USB_FLOAT;
		dev_info(info->dev,"Only Mag %d\n", info->charger_type);
	}
	/* set current limitation and start to charge */
	switch (info->charger_type) {
	case POWER_SUPPLY_TYPE_USB:
		usbin_cur = info->cur.sdp_cur;
		break;
	case POWER_SUPPLY_TYPE_USB_DCP:
		usbin_cur = info->cur.dcp_cur;
		break;
	case POWER_SUPPLY_TYPE_USB_CDP:
		usbin_cur = info->cur.cdp_cur;
		break;
	case POWER_SUPPLY_TYPE_USB_FLOAT:
		if (global_smb_charger && (!sgm_2batt_typec_charger_ok())){
			usbin_cur = info->cur.float_cur;
		}else{
			usbin_cur = FLOAT_CURRENT_900MA;
		}
		break;
	default:
		usbin_cur = info->cur.unknown_cur;
	}

	pr_buf_info("sy6974_charger_set_input_current %d\n",usbin_cur);
	ret = sy6974_charger_set_input_current(info, usbin_cur);
	if (ret){
		do {
			ret = sy6974_charger_set_input_current(info, usbin_cur);
			if(ret == 0){
				pr_buf_info("Set sy6974 input current limit  retry %d times.\n", (retry_count+1));
				break;
			}
			msleep(20);
			retry_count++;
		} while (retry_count < MAX_RETRIES_TIMES);
		if(retry_count >= MAX_RETRIES_TIMES){
			pr_buf_err("Set sy6974 input current limit fail (%d), use default input current limit.\n", (retry_count+1));
		}
	}
	dev_info(info->dev,"info->chg_en_gpio %d\n",info->chg_en_gpio);
	if (!info->chg_en_gpio) {
		ret = sy6974_update_bits(info, SY6974_REG_1,
			   SY6974_REG_CHG_MASK, 1 << SY6974_REG_CHG_SHIFT);
		if (ret) {
			dev_err(info->dev, "enable SY6974 charge failed\n");
			goto out;
		}
		pr_buf_info("cmd to set enable charge\n");

	} else {
		gpio_direction_output(info->chg_en_gpio, info->chg_en_gpio_high);
		dev_info(info->dev, " %s enable sy6974 charger via pull %s gpio%d\n",
			info->charger_desc.name, (info->chg_en_gpio_high?"High":"Low"),info->chg_en_gpio);
	}

	info->charging = true;
#if 0
	if (info->batt_psy)
		power_supply_changed(info->batt_psy);
#endif	
	dev_info(info->dev, "##start charge, chg_type=%d, good_vbus_det=%d, usb_mode=%d\n", info->charger_type, info->good_vbus_detected, info->usb_mode);
	return 0;
err_hw_init:
	dev_err(info->dev, "##start charge, Init HW failed %d\n", ret);
out:
	dev_err(info->dev, "##start charge, Not detect good vbus or sy6974_charger_set_input_current failed %d\n", ret);
	return ret;

}

static void sy6974_charger_stop_charge(struct sy6974x_device *info)
{
	int ret = 0;
	struct device *dev = info->dev;

	if(!info->isHWInit){
		ret = sy6974_charger_hw_init(info);
		if (ret) {
			dev_err(dev, "sy6974_charger_hw_init failed\n");
			goto err_hw_init;
		}
		info->isHWInit = true;
	}
	if(!info->chg_en_gpio) {
		ret = sy6974_update_bits(info, SY6974_REG_1,
				SY6974_REG_CHG_MASK, 0 << SY6974_REG_CHG_SHIFT);
		if(ret) {
			dev_err(info->dev, "disable SY6974 charge failed\n");
			return;
		}
		pr_buf_err("###STOP charger### cmd to set disable charge\n");
	} else {
		gpio_direction_output(info->chg_en_gpio,(!info->chg_en_gpio_high));
		dev_info(info->dev, " %s disable sy6974 charger via pull %s gpio%d\n",
			info->charger_desc.name, (info->chg_en_gpio_high?"High":"Low"),info->chg_en_gpio);		
	}

	info->charging = false;
	info->charger_type = POWER_SUPPLY_TYPE_UNKNOWN;
	info->charger_status = POWER_SUPPLY_STATUS_DISCHARGING;
#if 0
	if (info->batt_psy)
		power_supply_changed(info->batt_psy);
#endif	
	dev_info(info->dev, "## stop_charge chg_type=%d, good_vbus_det=%d, usb_mode=%d\n", info->charger_type, info->good_vbus_detected, info->usb_mode);
err_hw_init:
	dev_err(info->dev, "##stop charge, Init HW failed %d\n", ret);	
}

static int sy6974_batt_health_sync(struct sy6974x_device *info)
{
	union power_supply_propval pval = {0, };

	if (!info->batt_psy) {
		info->batt_psy = power_supply_get_by_name(BATTERY_PSY_NAME);
		if (!info->batt_psy) {
			dev_err(info->dev, "Could not get battery power_supply\n");
			return PTR_ERR(info->batt_psy);
		}
	}

	power_supply_get_property(info->batt_psy, POWER_SUPPLY_PROP_HEALTH, &pval);
	info->batt_health_status = pval.intval;
	dev_info(info->dev,"batt_health_status is %d\n", info->batt_health_status);

	return 0;
}

static int sy6974_set_prop_temp_control(struct sy6974x_device *info,u32 val)
{
	int ret = 0;

	// update battery health status.
	ret = sy6974_batt_health_sync(info);
	if (ret) {
		dev_info(info->dev,"get batt_health_sync %d\n", ret);
		return ret;
	}

	dev_err(info->dev,"batt_health is %d, gd_vbus %d\n",
		info->batt_health_status, info->good_vbus_detected);

	//check charging status.
	if ( !info->good_vbus_detected)
		return 0;

	switch (info->batt_health_status) {
	case POWER_SUPPLY_HEALTH_OVERHEAT:
	case POWER_SUPPLY_HEALTH_COLD:
		sy6974_charger_stop_charge(info);
		break;
	case POWER_SUPPLY_HEALTH_WARM:
		sy6974_charger_start_charge(info);
		if(val >=FCC_WARM_CURRENT)
			sy6974_charger_set_fcc_current(info, FCC_WARM_CURRENT);
		else
			sy6974_charger_set_fcc_current(info, val);
		sy6974_charger_set_termina_vol(info, CHARGE_WARM_VOLTAGE_LIMIT);
		break;
	case POWER_SUPPLY_HEALTH_COOL:
		sy6974_charger_start_charge(info);
		if(val >=FCC_COLD_CURRENT)
			sy6974_charger_set_fcc_current(info, FCC_COLD_CURRENT);
		else
			sy6974_charger_set_fcc_current(info, val);
		break;
	case POWER_SUPPLY_HEALTH_GOOD:
	default:
		if(val >=FCC_ICHG_CURRENT_2000MA)
			sy6974_charger_set_fcc_current(info, FCC_ICHG_CURRENT_2000MA);
		else
			sy6974_charger_set_fcc_current(info, val);
		sy6974_charger_set_termina_vol(info, CHARGE_NORMAL_VOLTAGE_LIMIT);
		break;
	}

	return 0;
}

static int sy6974_charger_get_health(struct sy6974x_device *info,
				      u32 *health)
{
	int ret = 0;

	// update battery health status.
	ret = sy6974_batt_health_sync(info);
	if (ret) {
		dev_info(info->dev,"get batt_health_sync %d\n", ret);
		return ret;
	}

	*health = info->batt_health_status;

	return 0;
}

static int sy6974_charger_get_online(struct sy6974x_device *info,
				      u32 *online)
{
	if (info->charging)
		*online = true;
	else
		*online = false;

	return 0;
}

static int sy6974_charger_get_status(struct sy6974x_device *info,
				      u32 *status)
{
	//pr_buf_info("start\n");
	sy6974_update_charger_status(info);

	*status = info->charger_status;
	dev_err(info->dev,"info->charger_status %d\n", info->charger_status);
	return 0;
}

static int sy6974_watchdog_ctl(struct sy6974x_device *info, int enable)
{
	int ret;

	if(enable)
		ret = sy6974_update_bits(info, SY6974_REG_5,
				  SY6974_REG_WATCHDOG_CTL_MASK, 0x10);
	else
		ret = sy6974_update_bits(info, SY6974_REG_5,
				  SY6974_REG_WATCHDOG_CTL_MASK, 0x0);
	if (ret)
		dev_err(info->dev, "set watchdog failed\n");

	return ret;
}

static int sy6974_charger_set_status(struct sy6974x_device *info,
				      int val)
{
	int ret = 0;

	if (val < 2)
		return 0;

	pr_buf_err("info->charging = %d, val = %d\n", info->charging, val);
	if (val == 3 && info->charging) {
	//if(val == 3){
		/*To resolve the default charging IC was enable but, charging flag was false*/
		sy6974_charger_stop_charge(info);
		dev_info(info->dev,"sy6974_charger_stop_charge\n");
	}
	else if (val == 2)
	{
		ret = sy6974_charger_start_charge(info);
		if (ret){
			dev_err(info->dev, "start charge failed (%d), retry \n",ret);
			ret = sy6974_charger_start_charge(info);
			if(ret)
				dev_err(info->dev, "start charge failed again (%d)\n",ret);
		}else{
		    dev_info(info->dev,"sy6974_charger start charge\n");
		}
	}

	return 0;
}

static int sy6974_get_safety_timer_enable(struct sy6974x_device *info, u32 *en)
{
	int ret = 0;
	u8 val = 0;

	ret = sy6974_read(info, SY6974_REG_5, &val);
	if (ret < 0) {
		dev_err(info->dev, "read SY6974 safety timer enable failed\n");
		return ret;
	}
	*en = !! (val & SY6974_REG_EN_TIMER_MASK);

	pr_buf_info("get_safety_timer_enable=%x\n",*en);
	return ret;
}

static int sy6974_enable_safety_timer(struct sy6974x_device *info,u8 val)
{
	int ret = 0;
	val = val << SY6974_REG_EN_TIMER_SHIFT;

	ret = sy6974_update_bits(info, SY6974_REG_5,SY6974_REG_EN_TIMER_MASK,
			val);
	if (ret)
		dev_err(info->dev, "set enable_safety_timer failed\n");

	pr_buf_info("set_safety_timer_enable=%x rc=%d\n", val, ret);
	return ret;
}

static int sy6974_charger_usb_get_property(struct power_supply *psy,
					    enum power_supply_property psp,
					    union power_supply_propval *val)
{
	struct sy6974x_device *info = power_supply_get_drvdata(psy);
	u32 cur, online, health, en, status, voltage;
	int ret = 0;

	mutex_lock(&info->lock);

	switch (psp) {
	case POWER_SUPPLY_PROP_CHARGE_ENABLED:
		sy6974_update_charger_status(info);
		val->intval = info->charging;
		break;
	case POWER_SUPPLY_PROP_STATUS:
		ret = sy6974_charger_get_status(info, &status);
		if (ret)
			goto out;

		val->intval = status;
		break;
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT:
		ret = sy6974_charger_get_fcc_current(info, &cur);
		if (ret)
			goto out;

		val->intval = cur;
		break;

	case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT:
		ret = sy6974_charger_get_input_current(info, &cur);
		if (ret)
			goto out;

		val->intval = cur;
		break;

	case POWER_SUPPLY_PROP_ONLINE:
		ret = sy6974_charger_get_online(info, &online);
		if (ret)
			goto out;

		val->intval = online;

		break;

	case POWER_SUPPLY_PROP_HEALTH:
		ret = sy6974_charger_get_health(info, &health);
		if (ret)
			goto out;

		val->intval = health;
		break;

	case POWER_SUPPLY_PROP_TYPE:
		//sy6974_update_charger_status(info);
		val->intval = info->charger_type;
		break;

	case POWER_SUPPLY_PROP_SAFETY_TIMER_ENABLE:
		ret = sy6974_get_safety_timer_enable(info, &en);
		if (ret)
			goto out;
		val->intval = en;
		break;

	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE_MAX:
		ret = sy6974_charger_get_charge_voltage(info, &voltage);
		if (ret)
			goto out;
		val->intval = voltage;
		break;

	default:
		ret = -EINVAL;
	}

out:
	mutex_unlock(&info->lock);
	return ret;
}

static int sy6974_charger_usb_set_property(struct power_supply *psy,
				enum power_supply_property psp,
				const union power_supply_propval *val)
{
	struct sy6974x_device *info = power_supply_get_drvdata(psy);
	int ret;

	mutex_lock(&info->lock);

	switch (psp) {
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT:
		ret = sy6974_charger_set_fcc_current(info, val->intval);
		if (ret < 0)
			dev_err(info->dev, "set charge current failed\n");
		break;
	case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT:
		ret = sy6974_charger_set_input_current(info, val->intval);
		if (ret < 0)
			dev_err(info->dev, "set input current limit failed\n");
		break;

	case POWER_SUPPLY_PROP_STATUS:
		ret = sy6974_charger_set_status(info, val->intval);
		if (ret < 0)
			dev_err(info->dev, "set charge status failed\n");
		break;

	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE_MAX:
		ret = sy6974_charger_set_termina_vol(info, val->intval / 1000);
		if (ret < 0)
			dev_err(info->dev, "failed to set terminate voltage\n");
		break;

	case POWER_SUPPLY_PROP_SAFETY_TIMER_ENABLE:
		ret = sy6974_enable_safety_timer(info,val->intval);
		if (ret < 0)
			dev_err(info->dev, "failed to set enable safety timer\n");
		break;

	case POWER_SUPPLY_PROP_FORCE_RECHARGE:
		/*Reset isHWInit when battery is remove.*/
		info->isHWInit = false;
		dev_err(info->dev, "Reset isHWInit  when battery is came back\n");
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		ret = sy6974_set_prop_temp_control(info, val->intval);
		break;
	default:
		ret = -EINVAL;
	}

	mutex_unlock(&info->lock);
	return ret;
}

static int sy6974_charger_property_is_writeable(struct power_supply *psy,
						 enum power_supply_property psp)
{
	int ret;

	switch (psp) {
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT:
	case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT:
	case POWER_SUPPLY_PROP_STATUS:
	case	POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE_MAX:
	case POWER_SUPPLY_PROP_SAFETY_TIMER_ENABLE:
	case POWER_SUPPLY_PROP_FORCE_RECHARGE:
	case POWER_SUPPLY_PROP_HEALTH:
		ret = 1;
		break;

	default:
		ret = 0;
	}

	return ret;
}

static enum power_supply_property sy6974_usb_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT,
	POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT,
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_TYPE,
	POWER_SUPPLY_PROP_SAFETY_TIMER_ENABLE,
	POWER_SUPPLY_PROP_CHARGE_ENABLED,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE_MAX,
	POWER_SUPPLY_PROP_FORCE_RECHARGE,
};

static int sy6974x_power_supply_init(struct sy6974x_device *info)
{	
	struct power_supply_config psy_cfg = { .drv_data = info, };

	info->charger_desc.name = info->name;
	info->charger_desc.type = POWER_SUPPLY_TYPE_USB;
	info->charger_desc.properties = sy6974_usb_props;
	info->charger_desc.num_properties =
			ARRAY_SIZE(sy6974_usb_props);
	info->charger_desc.get_property = sy6974_charger_usb_get_property;
	info->charger_desc.set_property = sy6974_charger_usb_set_property;
	info->charger_desc.property_is_writeable = sy6974_charger_property_is_writeable,
	
	psy_cfg.drv_data = info;
	psy_cfg.of_node = info->dev->of_node;

	info->charger = power_supply_register(info->dev, &info->charger_desc,
					    &psy_cfg);
	if (IS_ERR(info->charger)) {
		return PTR_ERR(info->charger);
	}

	return 0;
}

static void sy6974x_power_supply_exit(struct sy6974x_device *info)
{
	power_supply_unregister(info->charger);
}

static void sy6974_determine_initial_status(struct sy6974x_device *info)
{
	//sy6974_update_charger_status(info);
	sy6974_update_fault_status(info);

	//delay work to make sure smb driver registered.
}
static int sy6974_charger_hw_init(struct sy6974x_device *info)
{
	int ret = 0;

	pr_buf_info("start\n");

	info->cur.sdp_cur = SDP_CURRENT_500MA;
	info->cur.dcp_cur = DCP_CURRENT_2000MA;
	info->cur.cdp_cur = CDP_CURRENT_1500MA;
	info->cur.float_cur = FLOAT_CURRENT_2000MA;
	info->cur.unknown_cur = UNKNOWN_CURRENT_500MA;

	//set REG0A bit0, bit1, to not allow INT pulse.
	ret = sy6974_update_bits(info, SY6974_REG_A,
				  SY6974_REG_VIN_INT_PULSE_BIT,
				  SY6974_REG_VIN_INT_PULSE_BIT);
	if (ret) {
		dev_err(info->dev, "set SY6974_REG_VIN_INT_PULSE_BIT failed\n");
		return ret;
	}

	ret = sy6974_update_bits(info, SY6974_REG_A,
				  SY6974_REG_IIN_INT_PULSE_BIT,
				  SY6974_REG_IIN_INT_PULSE_BIT);
	if (ret) {
		dev_err(info->dev, "set SY6974_REG_IIN_INT_PULSE_BIT failed\n");
		return ret;
	}

	//set vindpm threshold(aicl).
	info->voltage_max_microvolt = CHARGE_NORMAL_VOLTAGE_LIMIT;
	ret = sy6974_charger_set_vindpm(info, info->voltage_max_microvolt);
	if (ret) {
		dev_err(info->dev, "set SY6974 vindpm vol failed\n");
		return ret;
	}

	//set terminated voltage 4.2v
	ret = sy6974_charger_set_termina_vol(info, info->voltage_max_microvolt);
	if (ret) {
		dev_err(info->dev, "set SY6974 terminal vol failed\n");
		return ret;
	}

#if 0	//iterm current use default 180mA.
	//set iterm current to 500mA
	ret = sy6974_charger_set_termina_cur(info, TERMINATION_CURRENT_LIMIT);
	if (ret) {
		dev_err(info->dev, "set SY6974 terminal cur failed\n");
		return ret;
	}
#endif

	//set fcc to 2A
	ret = sy6974_charger_set_fcc_current(info, FCC_ICHG_CURRENT_2000MA);
	if (ret) {
		dev_err(info->dev, "set SY6974 fcc cur failed\n");
		return ret;
	}

	//set usbin ovp threshold to 6.5V
	ret = sy6974_charger_set_ovp(info, FCHG_OVP_6P5V);
	if (ret) {
		dev_err(info->dev, "set SY6974 fcc cur failed\n");
		return ret;
	}

	//disable watchdog.
	ret = sy6974_watchdog_ctl(info, 0);
	if (ret) {
		dev_err(info->dev, "set sy6974_watchdog_ctl failed\n");
		return ret;
	}

	return ret;
}

static int sy6974_parse_dt(struct device *dev,
		struct sy6974x_device *info)
{
	struct device_node *np = dev->of_node;
	int ret = 0;

	/*sy6974_int_gpio*/
	ret = of_get_named_gpio(np, "silergy,int-gpio", 0);
	info->int_gpio = ret;
	if(!gpio_is_valid(info->int_gpio)) {
		dev_err(dev, "invalid sy6974_int_gpio. ret=%d\n", ret);
		return ret;
	}
	ret = gpio_request(info->int_gpio, "sy6974_int_gpio");
	if (ret < 0) {
		dev_err(dev,
			"gpio %d request failed. ret=%d\n", info->int_gpio, ret);
		return ret;
	}

	/*sy6974_chg_en_gpio*/
	ret = of_get_named_gpio(np, "silergy,chg-en-gpio", 0);
	info->chg_en_gpio = ret;
	if(!gpio_is_valid(info->chg_en_gpio)) {
		dev_err(dev, "invalid chg_en_gpio gpio: %d\n", ret);
		return ret;
	}
	ret = gpio_request(info->chg_en_gpio, "sy6974_chg_en_gpio");
	if (ret < 0) {
		dev_err(dev,
			"gpio %d request failed. ret=%d\n", info->chg_en_gpio, ret);
		return ret;
	}

	ret = of_property_read_u32(np, "silergy,chg-en-gpio-high",
			&info->chg_en_gpio_high);
	if (ret < 0){
		dev_err(dev,
			"Warning get chg-en-gpio-high failed, use default value 1 . ret=%d\n", ret);		
		info->chg_en_gpio_high = 1;
	}
	dev_err(dev,"silergy,chg-en-gpio-high %d . \n", info->chg_en_gpio_high);	

	return ret;
}
static int show_registers(struct seq_file *m, void *data)
{
	struct sy6974x_device *info = m->private;
	u8 addr;
	int ret;
	u8 val;
	for (addr = 0x0; addr <= 0x0B; addr++) {
		ret = sy6974_read(info, addr, &val);
		if (!ret)
			seq_printf(m, "Reg[%02X] = 0x%02X\n", addr, val);
	}
	return 0;
}
static int reg_debugfs_open(struct inode *inode, struct file *file)
{
	struct sy6974x_device *info = inode->i_private;
	return single_open(file, show_registers, info);
}
static const struct file_operations reg_debugfs_ops = {
	.owner		= THIS_MODULE,
	.open		= reg_debugfs_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};
static void sy6974x_create_debugfs(struct sy6974x_device *info )
{
	int ret = 0;
	char *regname;
	regname = kasprintf(GFP_KERNEL,"%s_registers",info->name);

	//if (info->dfs_root) 
	{
		//file = debugfs_create_file(regname, S_IFREG | S_IRUGO,
		//	info->dfs_root, info, &reg_debugfs_ops);
		ret = his_register_debugfs_file(regname, S_IFREG | S_IRUGO,
			info, &reg_debugfs_ops);
	}

	if(ret)
		dev_err(info->dev,"Couldn't create registers file rc=%ld\n", ret);
	dev_err(info->dev,"SY6974 debusfs  create registers file %s\n", regname);
}
static int sy6974x_charger_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);
	struct device *dev = &client->dev;
	struct sy6974x_device *info;
	int ret;
	int num;
	char *name;
	struct device_node *np = client->dev.of_node;
	if(!np){
		dev_err(dev,"no devicetree\n");
		return -ENODEV;
	}
	/*Get new ID for the new device*/
	mutex_lock(&sy6974x_id_mutex);
	num = idr_alloc(&sy6974x_id,client,0,0,GFP_KERNEL);
	mutex_unlock(&sy6974x_id_mutex);
	if(id){
		name = kasprintf(GFP_KERNEL,"%s_batt%d",id->name,(num+1));
	}

	if(!name){
		dev_err(dev,"failed to allocate device name\n");
		ret = -ENOMEM;
		goto error_1;
	}
	dev_err(dev, "dev name %s \n",name);

	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_BYTE_DATA)) {
		dev_err(dev, "No support for SMBUS_BYTE_DATA\n");
		return -ENODEV;
	}

	info = devm_kzalloc(dev, sizeof(*info), GFP_KERNEL);
	if (!info){

		ret = -ENOMEM;
		goto error_2;
	}

	i2c_set_clientdata(client, info);
	
	info->id = num;
	info->dev = dev;
	info->isHWInit = false;	
	if(id)
		info->chip = id->driver_data;
	info->name = name;
	
	if(np){

		ret = sy6974_parse_dt(dev, info);
		if (ret) {
			dev_err(dev,"sy6974_parse_dt() err\n");
			goto err_parse_dt;
		}
	}

	mutex_init(&info->lock);

	info->int_irq = gpio_to_irq(info->int_gpio);
	ret = sy6974x_power_supply_init(info);
	if (ret) {
		dev_err(dev, "failed to register power supply: %d\n", ret);
		goto error_2;
	}

	ret = sy6974_charger_hw_init(info);
	if(ret){
		dev_err(dev, "failed to init sy6974 charger hw init , retry later %d\n", ret);
		info->isHWInit = false;
	}else{
		dev_err(dev, " Init sy6974 charger hw success %d\n", ret);
		info->isHWInit = true;
	}
	sy6974_determine_initial_status(info);

	//enable_irq_wake(info->int_irq);
	disable_irq(info->int_irq);
	
	sy6974x_create_debugfs(info);
	
	pr_buf_info("sy6974x %s probe success\n",name);
	return 0;
error_2:
	kfree(name);
error_1:
	mutex_lock(&sy6974x_id_mutex);
	idr_remove(&sy6974x_id, num);
	mutex_unlock(&sy6974x_id_mutex);
err_parse_dt:
	devm_kfree(dev, info);
	return ret;
}

static int sy6974x_charger_remove(struct i2c_client *client)
{
	struct sy6974x_device *info = i2c_get_clientdata(client);

	if (!IS_ERR_OR_NULL(info)) {
		sy6974x_power_supply_exit(info);
		mutex_lock(&sy6974x_id_mutex);
		idr_remove(&sy6974x_id, info->id);
		mutex_unlock(&sy6974x_id_mutex);
		kfree(info->name);		
		devm_kfree(&client->dev, info);
	}
	dev_info(info->dev, "driver unregistered\n");
	return 0;
}

static const struct i2c_device_id sy6974_i2c_id[] = {
	{"sy6974_chg1", SY6974BAT1},
	{"sy6974_chg2", SY6974BAT2},
	{},
};

static const struct of_device_id sy6974_charger_of_match[] = {
	{ .compatible = "silergy,sy6974_chg1", },
	{ .compatible = "silergy,sy6974_chg2", },
	{},
};

MODULE_DEVICE_TABLE(of, sy6974_charger_of_match);

static struct i2c_driver sy6974_charger_driver = {
	.driver = {
		.name = "sy6974x-charger",
		.of_match_table = sy6974_charger_of_match,
	},
	.probe = sy6974x_charger_probe,
	.remove = sy6974x_charger_remove,
	.id_table = sy6974_i2c_id,
};

module_i2c_driver(sy6974_charger_driver);

MODULE_DESCRIPTION("SY6974 Charger Driver");
MODULE_AUTHOR("hmct@hisense.com");
MODULE_LICENSE("GPL v2");
