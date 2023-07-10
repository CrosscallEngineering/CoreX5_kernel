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

struct sgm_usb_chip {
	struct platform_device *pdev;
	struct delayed_work	typea_otg_detect_work;
	struct extcon_dev *extcon;
	u32		hub_reset_gpio;
	u32		usb_switch2_hub_gpio;
	u32		usb_switch1_hub_gpio;
	u32		typea_id_gpio;
	u32		typea_5v_pwr_en_gpio;
	int		typea_otg_irq;
	u32		usb_witch_30_gpio;
	bool	typec_otg_present;
	unsigned long inputs;
};

#define TYPEA_BUS_VLD	0
#define TYPEC_BUS_VLD	1
#define TYPEA_OTG_VLD	2
#define TYPEC_OTG_VLD	3
#define SGM_INIT		7

enum sgm_usb_mode
{
    sgm_default_mode = 0,
    typea_otg_mode,
    typec_otg_mode,
	typea_typec_otg_mode,
};

static struct sgm_usb_chip *global_chip = NULL;
static char *doubleinput[2] = { "TYPEA_TYPEC_EVENT=TYPEA_TYPEC_EVENT", NULL };
bool global_typea_otg_status = false;

extern struct smb_charger *global_smb_charger;

bool sgm_typea_otg_ok(void)
{
	if (!global_chip) {
		pr_buf_err("sgm: err,no chip\n");
		return 0;
	}

	return !!test_bit(TYPEA_OTG_VLD, &(global_chip->inputs));
}
EXPORT_SYMBOL_GPL(sgm_typea_otg_ok);
 
bool sgm_typec_otg_ok(void)
{
	if (!global_chip) {
		pr_buf_err("sgm: err,no chip\n");
		return 0;
	}

	return !!test_bit(TYPEC_OTG_VLD, &(global_chip->inputs));
}
EXPORT_SYMBOL_GPL(sgm_typec_otg_ok);


static irqreturn_t sgm_typea_otg_irq(int irq, void *data)
{
	struct sgm_usb_chip *chip = data;

	pr_buf_info("sgm: typea otg irq triggered\n");

	schedule_delayed_work(&chip->typea_otg_detect_work, msecs_to_jiffies(50));

	return IRQ_HANDLED;
}

int sgm_fusb_typec_otg_usb_switch(bool enable)
{
	int ret = 0;

	if (!global_chip) {
		pr_buf_err("sgm: err,no chip\n");
		return -EINVAL;
	}

	if (sgm_typea_otg_ok()) {
		if(enable){
			pr_buf_err("typea otg working, turn off typec usb3.0,turn on usb2.0\n");
			kobject_uevent_env(&global_chip->pdev->dev.kobj, KOBJ_CHANGE, doubleinput);
			//ret = gpio_direction_output(global_chip->usb_witch_30_gpio, enable);
			ret = gpio_direction_output(global_chip->usb_switch2_hub_gpio, enable);
		}else{
			pr_buf_err("typea otg working, turn on typec usb3.0,turn off usb2.0\n");
			ret = gpio_direction_output(global_chip->usb_witch_30_gpio, enable);
			ret = gpio_direction_output(global_chip->usb_switch2_hub_gpio, enable);
		}
	}

	enable ? set_bit(TYPEC_OTG_VLD, &global_chip->inputs) : clear_bit(TYPEC_OTG_VLD, &global_chip->inputs);

	pr_buf_err("sgm: %s usb_switch_gpio %s\n",
		enable ? "enable" : "disable", !ret ? "success" : "fail");

	return ret;
}
EXPORT_SYMBOL_GPL(sgm_fusb_typec_otg_usb_switch);

static int sgm_typea_otg_usb_switch(bool enable)
{
	int ret = 1;

	if (!global_chip) {
		pr_buf_err("sgm: err,no chip\n");
		return -EINVAL;
	}
	ret = gpio_direction_output(global_chip->usb_switch1_hub_gpio, enable);
	//ret = gpio_direction_output(global_chip->usb_switch2_hub_gpio, enable);
	if (sgm_typec_otg_ok()) {
		if(enable){
			pr_buf_err("typec otg working, turn off typec usb3.0,turn on usb2.0\n");
			kobject_uevent_env(&global_chip->pdev->dev.kobj, KOBJ_CHANGE, doubleinput);
			//ret = gpio_direction_output(global_chip->usb_witch_30_gpio, enable);
			ret = gpio_direction_output(global_chip->usb_switch2_hub_gpio, enable);
		}else{
			pr_buf_err("typea otg working, turn on typec usb3.0,turn off usb2.0\n");
			//ret = gpio_direction_output(global_chip->usb_witch_30_gpio, enable);
			ret = gpio_direction_output(global_chip->usb_switch2_hub_gpio, enable);
		}
	}

	pr_err("###usb_switch1_hub_gpio = %d hub_reset_gpio = %d\n",gpio_get_value(global_chip->usb_switch1_hub_gpio),gpio_get_value(global_chip->hub_reset_gpio));
	enable ? set_bit(TYPEA_OTG_VLD, &global_chip->inputs) : clear_bit(TYPEA_OTG_VLD, &global_chip->inputs);

	pr_buf_err("sgm: %s sgm_typea_otg_usb_switch1 %s\n",
		enable ? "enable" : "disable", !ret ? "success" : "fail");

	return ret;
}

static int sgm_typea_otg_vbus_5v_en(bool enable)
{
	int ret = 1;

	if (!global_chip) {
		pr_buf_err("sgm: err,no chip\n");
		return -EINVAL;
	}
	
	ret = gpio_direction_output(global_chip->typea_5v_pwr_en_gpio, enable);

	pr_buf_err("sgm: %s typea_vbus_boost %s\n",
		enable ? "enable" : "disable", !ret ? "success" : "fail");

	return ret;
}

static const unsigned int usb_phy_extcon_cable[] = {
	EXTCON_USB,
	EXTCON_USB_HOST,
	EXTCON_NONE,
};

static int sgm_usb_extcon_register(struct sgm_usb_chip *chip)
{
	int ret;

	/* Register extcon to notify USB driver */
	chip->extcon = devm_extcon_dev_allocate(&chip->pdev->dev, usb_phy_extcon_cable);
	if (IS_ERR(chip->extcon)) {
		dev_err(&chip->pdev->dev, "failed to allocate extcon device\n");
		return PTR_ERR(chip->extcon);
	}

	ret = devm_extcon_dev_register(&chip->pdev->dev, chip->extcon);
	if (ret) {
		dev_err(&chip->pdev->dev, "failed to register extcon device\n");
		return ret;
	}

	extcon_set_property_capability(chip->extcon, EXTCON_USB,
			EXTCON_PROP_USB_TYPEC_POLARITY);
	extcon_set_property_capability(chip->extcon, EXTCON_USB,
			EXTCON_PROP_USB_SS);
	extcon_set_property_capability(chip->extcon, EXTCON_USB_HOST,
			EXTCON_PROP_USB_TYPEC_POLARITY);
	extcon_set_property_capability(chip->extcon, EXTCON_USB_HOST,
			EXTCON_PROP_USB_SS);

	return 0;
}

static int sgm_typea_enable_otg(struct sgm_usb_chip *chip,u32 enable)
{
	union extcon_property_value val = {1};

	if (enable) {
		extcon_set_property(chip->extcon, EXTCON_USB_HOST, EXTCON_PROP_USB_TYPEC_POLARITY, val);
		extcon_set_property(chip->extcon, EXTCON_USB_HOST, EXTCON_PROP_USB_SS, val);
		extcon_set_state_sync(chip->extcon, EXTCON_USB_HOST, 1);
	} else{
		if(!sgm_typec_otg_ok())
			extcon_set_state_sync(chip->extcon, EXTCON_USB_HOST, 0);
		else
			pr_buf_info("sgm: typec exist don't change usb mode\n");
		}

	pr_buf_err("%s success.\n", enable ? "enable" : "disable");

	return 0;
}

static void sgm_typea_otg_detect_work(struct work_struct *work)
{
	struct sgm_usb_chip *chip = container_of(work, struct sgm_usb_chip,
		typea_otg_detect_work.work);
	union power_supply_propval val = {0, };
	bool otg_id_status = !!gpio_get_value(chip->typea_id_gpio);

	pr_buf_info("sgm: !otg_id=%d otg_mode=%d\n", !otg_id_status, global_typea_otg_status);

	if (!otg_id_status != global_typea_otg_status)
	{
			pr_buf_info("sgm: switch to ext otg mode = %d\n", !otg_id_status);
			val.intval = !otg_id_status;

			//enable typea otg.
			if (!otg_id_status){
				sgm_typea_otg_usb_switch(true);
				sgm_typea_otg_vbus_5v_en(true);
				sgm_typea_enable_otg(chip,true);
				}

			//disable typea otg.
			if (otg_id_status){
				sgm_typea_otg_usb_switch(false);
				sgm_typea_otg_vbus_5v_en(false);
				sgm_typea_enable_otg(chip,false);
				}

			if (global_smb_charger->usb_psy)
				power_supply_changed(global_smb_charger->usb_psy);
		}
		global_typea_otg_status = !otg_id_status;
	}

static void sgm_hw_init(struct device *dev, struct sgm_usb_chip *chip)
{
	schedule_delayed_work(&chip->typea_otg_detect_work, msecs_to_jiffies(5500));
}

static int sgm_parse_dt(struct device *dev,
		struct sgm_usb_chip *chip)
{
	struct device_node *np = dev->of_node;
	int ret = 0;

	/* typea-id-gpio */
	ret = of_get_named_gpio(np, "sgm,typea-id-gpio", 0);
	chip->typea_id_gpio= ret;
	if(!gpio_is_valid(chip->typea_id_gpio)) {
		dev_err(dev, "invalid typea-id-gpio. ret = %d\n", ret);
		return ret;
	}

	ret = gpio_request(chip->typea_id_gpio, "sgm-typea-id-gpio");
	if (ret < 0) {
		dev_err(dev,
			"%s: gpio %d request failed. ret = %d\n",
			__func__, chip->typea_id_gpio, ret);
		return ret;
	}

	/*typea-5v-pwr-en-gpio*/
	ret = of_get_named_gpio(np, "sgm,typea-5v-pwr-en-gpio", 0);
	chip->typea_5v_pwr_en_gpio = ret;
	if(!gpio_is_valid(chip->typea_5v_pwr_en_gpio)) {
		dev_err(dev, "invalid typea-5v-pwr-en-gpio.ret = %d\n", ret);
		return ret;
	}

	ret = gpio_request(chip->typea_5v_pwr_en_gpio, "sgm-typea-5v-pwr-en-gpio");
	if (ret < 0) {
		dev_err(dev,
			"%s: gpio %d request failed. ret = %d\n",
			__func__, chip->typea_5v_pwr_en_gpio, ret);
		return ret;
	}

	/*usb-witch-30-gpio*/
	ret = of_get_named_gpio(np, "sgm,usb-witch-30-gpio", 0);
	chip->usb_witch_30_gpio = ret;
	if(!gpio_is_valid(chip->usb_witch_30_gpio))
		dev_err(dev, "invalid usb_witch_30_gpio ret = %d\n", ret);

	ret = gpio_request(chip->usb_witch_30_gpio, "sgm-usb-witch-30-gpio");
	if (ret < 0)
		dev_err(dev,
			"gpio %d request failed. rc = %d\n", chip->usb_witch_30_gpio, ret);

	/*usb-switch1-hub-gpio*/
	ret = of_get_named_gpio(np, "sgm,usb-switch1-hub-gpio", 0);
	chip->usb_switch1_hub_gpio= ret;
	if(!gpio_is_valid(chip->usb_switch1_hub_gpio))
		dev_err(dev, "invalid usb_switch1_hub_gpio ret = %d\n", ret);

	ret = gpio_request(chip->usb_switch1_hub_gpio, "sgm-usb-switch1-hub-gpio");
	if (ret < 0)
		dev_err(dev,
			"gpio %d request failed. rc = %d\n", chip->usb_switch1_hub_gpio, ret);

	/*usb-switch2-hub-gpio*/
	ret = of_get_named_gpio(np, "sgm,usb-switch2-hub-gpio", 0);
	chip->usb_switch2_hub_gpio= ret;
	if(!gpio_is_valid(chip->usb_switch2_hub_gpio))
		dev_err(dev, "invalid usb_switch2_hub_gpio ret = %d\n", ret);

	ret = gpio_request(chip->usb_switch2_hub_gpio, "sgm-usb-switch2-hub-gpio");
	if (ret < 0)
		dev_err(dev,
			"gpio %d request failed. rc = %d\n", chip->usb_switch2_hub_gpio, ret);

	/*hub-reset-gpio*/
	ret = of_get_named_gpio(np, "sgm,hub-reset-gpio", 0);
	chip->hub_reset_gpio= ret;
	if(!gpio_is_valid(chip->hub_reset_gpio))
		dev_err(dev, "invalid hub_reset_gpio ret = %d\n", ret);

	ret = gpio_request(chip->hub_reset_gpio, "sgm-hub-reset-gpio");
	if (ret < 0)
		dev_err(dev,
			"gpio %d request failed. rc = %d\n", chip->hub_reset_gpio, ret);
	return 0;
}

static ssize_t sgm_double_usb_mode_show(struct device *dev,
                struct device_attribute *attr, char *buf)
{
	 int mode = 0;

	if (test_bit(TYPEA_OTG_VLD, &global_chip->inputs) && !test_bit(TYPEC_OTG_VLD, &global_chip->inputs))
		mode = typea_otg_mode;
	else if (!test_bit(TYPEA_OTG_VLD, &global_chip->inputs) && test_bit(TYPEC_OTG_VLD, &global_chip->inputs))
		mode = typec_otg_mode;
	else if (test_bit(TYPEA_OTG_VLD, &global_chip->inputs) && test_bit(TYPEC_OTG_VLD, &global_chip->inputs))
		mode = typea_typec_otg_mode;
	else
	mode = sgm_default_mode;

	pr_buf_err("sgm: global_chip->inputs=0x%x\n", global_chip->inputs);

	return sprintf(buf, "%d\n", mode);
}

static DEVICE_ATTR(sgm_double_usb_mode, S_IRUGO, sgm_double_usb_mode_show, NULL);


static struct attribute *sgm_sys_node_attrs[] = {
	&dev_attr_sgm_double_usb_mode.attr,
	NULL,
};

static struct attribute_group sgm_sys_node_attr_group = {
	.attrs = sgm_sys_node_attrs,
};

static int sgm_debugfs_init(struct sgm_usb_chip *chip)
{
	int ret;

	ret = his_register_sysfs_attr_group(&sgm_sys_node_attr_group);
	if (ret < 0)
		pr_buf_err("Error create sgm_sys_node_attr_group %d\n", ret);

	return 0;
}

static int sgm_double_usb_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct sgm_usb_chip *chip;

	pr_buf_info("sgm:sgm_probe start\n");

	chip = devm_kzalloc(&pdev->dev, sizeof(*chip), GFP_KERNEL);
	if (!chip) {
		dev_err(&pdev->dev, "Unable to allocate memory\n");
		return -ENOMEM;
	}

	chip->pdev = pdev;
	dev_set_drvdata(&pdev->dev, chip);
	if (pdev->dev.of_node) {
		ret = sgm_parse_dt(&pdev->dev, chip);
		if (ret) {
			dev_err(&pdev->dev,"%s: sgm_parse_dt() err\n", __func__);
			goto err_parse_dt;
		}
	} else {
		dev_err(&pdev->dev, "No dts data\n");
		goto err_parse_dt;
	}

	global_chip = chip;
	INIT_DELAYED_WORK(&chip->typea_otg_detect_work, sgm_typea_otg_detect_work);

	chip->typea_otg_irq = gpio_to_irq(chip->typea_id_gpio);

	ret = request_threaded_irq(chip->typea_otg_irq,
			NULL,
			sgm_typea_otg_irq,
			IRQF_TRIGGER_RISING |
			IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
			"sgm_typea_otg_irq", chip);
	if (ret) {
		dev_err(&pdev->dev, "request irq failed for ext otg fault\n");
		goto err_parse_dt;
	}
	enable_irq_wake(chip->typea_otg_irq);
	set_bit(SGM_INIT, &chip->inputs);
	sgm_hw_init(&pdev->dev, chip);
	sgm_usb_extcon_register(chip);
	sgm_debugfs_init(chip);

	pr_buf_info("sgm:sgm_probe success\n");
	return 0;

err_parse_dt:
	devm_kfree(&pdev->dev,chip);
	return ret;
}

static void sgm_double_usb_shutdown(struct platform_device *pdev)
{
	struct sgm_usb_chip *chip = platform_get_drvdata(pdev);

	if (chip) {
		cancel_delayed_work_sync(&chip->typea_otg_detect_work);
	}

	global_chip = NULL;
}

static int sgm_double_usb_remove(struct platform_device *pdev)
{
	struct sgm_usb_chip *chip = platform_get_drvdata(pdev);

	if (chip) {
		cancel_delayed_work_sync(&chip->typea_otg_detect_work);
	}

	global_chip = NULL;

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int sgm_double_usb_suspend(struct device *dev)
{

	return 0;
}

static int sgm_double_usb_resume(struct device *dev)
{

	return 0;
}
#endif

static SIMPLE_DEV_PM_OPS(sgm_pm_ops, sgm_double_usb_suspend,
			  sgm_double_usb_resume);

static struct of_device_id sgm_double_usb_match_table[] = {
	{	.compatible = "sgmicro,double-usb",
	},
	{}
};

static struct platform_driver sgm_double_usb_driver = {
	.driver		= {
		.name		= "sgm_double_usb",
		.owner		= THIS_MODULE,
		.of_match_table	= sgm_double_usb_match_table,
		.pm	= &sgm_pm_ops,
	},
	.probe		= sgm_double_usb_probe,
	.remove		= sgm_double_usb_remove,
	.shutdown 	= sgm_double_usb_shutdown,
};

module_platform_driver(sgm_double_usb_driver);

MODULE_DESCRIPTION("SGM Double Charger Driver");
MODULE_AUTHOR("lishaoxiang@hisense.com");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:sgm");
