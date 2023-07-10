#include <linux/module.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/workqueue.h>
#include <linux/i2c.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/time.h>
#include <linux/slab.h>
#include <linux/version.h>
#include <linux/his_debug_base.h>

#define CWFG_ENABLE_LOG 1 //CHANGE   Customer need to change this for enable/disable log
#define CWFG_I2C_BUSNUM 2 //CHANGE   Customer need to change this number according to the principle of hardware
#define DOUBLE_SERIES_BATTERY 0
/*
#define USB_CHARGING_FILE "/sys/class/power_supply/usb/online" // Chaman
#define DC_CHARGING_FILE "/sys/class/power_supply/ac/online"
*/
//#define CW_PROPERTIES "cw-bat"

#define REG_VERSION             0x0
#define REG_VCELL               0x2
#define REG_SOC                 0x4
#define REG_RRT_ALERT           0x6
#define REG_CONFIG              0x8
#define REG_MODE                0xA
#define REG_VTEMPL              0xC
#define REG_VTEMPH              0xD
#define REG_BATINFO             0x10


#define MODE_SLEEP_MASK         (0x3<<6)
#define MODE_SLEEP              (0x3<<6)
#define MODE_NORMAL             (0x0<<6)
#define MODE_QUICK_START        (0x3<<4)
#define MODE_RESTART            (0xf<<0)

#define CONFIG_UPDATE_FLG       (0x1<<1)
#define ATHD                    (0x0<<3)        // ATHD = 0%

#define queue_delayed_work_time  8000
#define BATTERY_CAPACITY_ERROR  40*1000
#define BATTERY_CHARGING_ZERO   1800*1000

#define UI_FULL 100
#define DECIMAL_MAX 70
#define DECIMAL_MIN 30

#define CHARGING_ON 1
#define NO_CHARGING 0


#define cw_bat1_printk(fmt, arg...)        \
	({                                    \
	 if(CWFG_ENABLE_LOG){              \
	 printk("FG_CW2015 BAT1 : %s : " fmt, __FUNCTION__ ,##arg);  \
	 }else{}                           \
	 })     //need check by Chaman


#define CWFG_NAME "cw2015_bat1_gauge"
#define SIZE_BATINFO    64

static unsigned char config_info[SIZE_BATINFO] = {
	0x15,0x7E,0x67,0x5D,0x59,0x58,0x53,0x4E,
	0x4A,0x47,0x45,0x49,0x52,0x45,0x2E,0x23,
	0x1D,0x13,0x0E,0x10,0x1E,0x2D,0x41,0x50,
	0x39,0x52,0x0B,0x85,0x44,0x67,0x70,0x84,
	0x89,0x87,0x85,0x86,0x3E,0x1A,0x5B,0x3A,
	0x0A,0x35,0x44,0x75,0x8D,0x91,0x91,0x41,
	0x58,0x81,0x96,0xA9,0x80,0x8A,0x95,0xCB,
	0x2F,0x00,0x64,0xA5,0xB5,0x05,0x50,0x39,
};

static struct power_supply *chrg_usb_psy;
static struct power_supply *chrg_dc_psy;

#ifdef CONFIG_PM
static struct timespec suspend_time_before;
static struct timespec after;
static int suspend_resume_mark = 0;
#endif

struct cw_battery {
	struct i2c_client *client;

	struct workqueue_struct *cwfg_workqueue;
	struct delayed_work battery_delay_work;

#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 1, 0)
	struct power_supply cw_bat;
#else
	struct power_supply *cw_bat;
#endif
	int charger_mode;
	int capacity;
	int voltage;
	int status;
	int change;
	bool isDataInit;
	//int alt;
};

int g_cw2015_bat1_capacity = 0;
int g_cw2015_bat1_vol = 0;
struct cw_battery *g_cw_bat1 = NULL;
#ifdef CONFIG_HISENSE_SGM_DOUBLE_BATTERIES
extern bool sgm_check_battery1_exist(void);
extern bool sgm_check_battery2_exist(void);
#endif /* CONFIG_HISENSE_SGM_DOUBLE_BATTERIES*/

extern struct device_bootinfo dev_bi;
static int cw_quick_soft_reset(struct cw_battery *cw_bat);
/*Define CW2015 iic read function*/
static int cw_read(struct i2c_client *client, unsigned char reg, unsigned char buf[])
{
	int ret = 0;
	ret = i2c_smbus_read_i2c_block_data( client, reg, 1, buf);
	return ret;
}
/*Define CW2015 iic write function*/		
static int cw_write(struct i2c_client *client, unsigned char reg, unsigned char const buf[])
{
	int ret = 0;
	ret = i2c_smbus_write_i2c_block_data( client, reg, 1, &buf[0] );
	return ret;
}
/*Define CW2015 iic read word function*/	
static int cw_read_word(struct i2c_client *client, unsigned char reg, unsigned char buf[])
{
	int ret = 0;
	ret = i2c_smbus_read_i2c_block_data( client, reg, 2, buf );
	return ret;
}

/*CW2015 update profile function, Often called during initialization*/
int cw_update_config_info_bat1(struct cw_battery *cw_bat)
{
	int ret;
	unsigned char reg_val;
	int i;
	unsigned char reset_val;

	cw_bat1_printk("\n");
	cw_bat1_printk("[FGADC] test config_info = 0x%x\n",config_info[0]);
	printk(KERN_INFO "%s\n", __func__);


	// make sure no in sleep mode
	ret = cw_read(cw_bat->client, REG_MODE, &reg_val);
	if(ret < 0) {
		return ret;
	}

	reset_val = reg_val;
	if((reg_val & MODE_SLEEP_MASK) == MODE_SLEEP) {
		return -1;
	}

	// update new battery info
	for (i = 0; i < SIZE_BATINFO; i++) {
		printk(KERN_INFO "%X\n", config_info[i]);
		ret = cw_write(cw_bat->client, REG_BATINFO + i, &config_info[i]);
		if(ret < 0) 
			return ret;
	}

	reg_val = 0x00;
	reg_val |= CONFIG_UPDATE_FLG;   // set UPDATE_FLAG
	reg_val &= 0x07;                // clear ATHD
	reg_val |= ATHD;                // set ATHD
	ret = cw_write(cw_bat->client, REG_CONFIG, &reg_val);
	if(ret < 0) 
		return ret;

	msleep(50);
	// reset
	reg_val = 0x00;
	reset_val &= ~(MODE_RESTART);
	reg_val = reset_val | MODE_RESTART;
	ret = cw_write(cw_bat->client, REG_MODE, &reg_val);
	if(ret < 0) 
		return ret;

	msleep(10);

	ret = cw_write(cw_bat->client, REG_MODE, &reset_val);
	if(ret < 0) 
		return ret;

	msleep(100);
	cw_bat1_printk("cw2015 update config success!\n");

	return 0;
}

/*This function called when get voltage from cw2015*/
static int cw_get_voltage(struct cw_battery *cw_bat)
{    
	int ret;
	unsigned char reg_val[2];
	u16 value16, value16_1, value16_2, value16_3;
	int voltage;

	ret = cw_read_word(cw_bat->client, REG_VCELL, reg_val);
	if(ret < 0) {
		return ret;
	}
	value16 = (reg_val[0] << 8) + reg_val[1];

	ret = cw_read_word(cw_bat->client, REG_VCELL, reg_val);
	if(ret < 0) {
		return ret;
	}
	value16_1 = (reg_val[0] << 8) + reg_val[1];

	ret = cw_read_word(cw_bat->client, REG_VCELL, reg_val);
	if(ret < 0) {
		return ret;
	}
	value16_2 = (reg_val[0] << 8) + reg_val[1];

	if(value16 > value16_1) {     
		value16_3 = value16;
		value16 = value16_1;
		value16_1 = value16_3;
	}

	if(value16_1 > value16_2) {
		value16_3 =value16_1;
		value16_1 =value16_2;
		value16_2 =value16_3;
	}

	if(value16 >value16_1) {     
		value16_3 =value16;
		value16 =value16_1;
		value16_1 =value16_3;
	}            

	voltage = value16_1 * 625 / 2048;

	if(DOUBLE_SERIES_BATTERY)
		voltage = voltage * 2;
	return voltage;
}
int g_cw_get_bat1_voltage(void)
{
	int vol;
	if(!g_cw_bat1){
		pr_err( "Could not get cw2015_bat1\n");
		return 0;
	}
	vol = cw_get_voltage(g_cw_bat1);
	return vol;
}
EXPORT_SYMBOL(g_cw_get_bat1_voltage);
int g_cw_reset_bat1_fuelgauge(void)
{
	int vol =0;
	if(!g_cw_bat1){
		pr_err( "Could not get cw2015_bat1\n");
		return -1;
	}
	g_cw_bat1->isDataInit = false;
	//battery is out , you can send new battery capacity vol here what you want set
	//for example
	g_cw_bat1->capacity = 0;//100;
	g_cw_bat1->voltage = 3700;//4200;
	g_cw_bat1->change = 1;
	cw_quick_soft_reset(g_cw_bat1);
	cancel_delayed_work_sync(&g_cw_bat1->battery_delay_work);
	queue_delayed_work(g_cw_bat1->cwfg_workqueue, &g_cw_bat1->battery_delay_work, msecs_to_jiffies(1000));
	return vol;
}
EXPORT_SYMBOL(g_cw_reset_bat1_fuelgauge);
static void cw_update_vol(struct cw_battery *cw_bat)
{
	int ret;
	ret = cw_get_voltage(cw_bat);
	if ((ret >= 0) && (cw_bat->voltage != ret)) {
		cw_bat->voltage = ret;
		cw_bat->change = 1;
	}
}
static int get_cw_bat_psy(struct cw_battery *cw_batt)
{
	if(!cw_batt->cw_bat){
		cw_batt->cw_bat = power_supply_get_by_name("battery");
		if (!cw_batt->cw_bat) {
			pr_err( "Could not get battery power_supply\n");
			return -ENODEV;
		}
	}
	return 0;
}
static int  get_chrg_psy(void)
{
	if(!chrg_usb_psy){
		chrg_usb_psy = power_supply_get_by_name("usb");
		if (!chrg_usb_psy) {
			pr_err( "Could not get usb power_supply\n");
			return -ENODEV;
		}
	}
	if(!chrg_dc_psy){
		chrg_dc_psy = power_supply_get_by_name("dc");
		if (!chrg_dc_psy) {
			pr_err( "Could not get dc power_supply\n");
			return -ENODEV;
		}
	}
	return 0;
}

static int get_charge_state(void)
{
	union power_supply_propval val;
	int ret = -ENODEV;
	int usb_online = 0;
	int ac_online = 0;

	if (!chrg_usb_psy || !chrg_dc_psy)
		get_chrg_psy();

	if(chrg_usb_psy) {
#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 1, 0)
		ret = chrg_usb_psy->get_property(chrg_usb_psy, POWER_SUPPLY_PROP_ONLINE, &val);
#else
		//ret = chrg_usb_psy->desc->get_property(chrg_usb_psy, POWER_SUPPLY_PROP_ONLINE, &val);
		ret =power_supply_get_property(chrg_usb_psy,POWER_SUPPLY_PROP_ONLINE,&val);
#endif

		if (!ret)
			usb_online = val.intval;
		/* Check whether USB is present or not */
		ret = power_supply_get_property(chrg_usb_psy,
				POWER_SUPPLY_PROP_PRESENT, &val);
		if (ret < 0)
			pr_err("Couldn't get USB Present status, rc=%d\n", ret);
		pr_err("chrg_usb_psy, val=%d\n", val.intval);
	}
	if(chrg_dc_psy) {
#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 1, 0)
		ret = chrg_dc_psy->get_property(chrg_dc_psy, POWER_SUPPLY_PROP_ONLINE, &val);
#else
		//ret = chrg_dc_psy->desc->get_property(chrg_dc_psy, POWER_SUPPLY_PROP_ONLINE, &val);
		ret = power_supply_get_property(chrg_usb_psy,POWER_SUPPLY_PROP_ONLINE,&val);
#endif
		if (!ret)
			ac_online = val.intval;			
	}
	if(!chrg_usb_psy){
		cw_bat1_printk("Usb online didn't find\n");
	}
	if(!chrg_dc_psy){
		cw_bat1_printk("Ac online didn't find\n");
	}
	cw_bat1_printk("ac_online = %d    usb_online = %d\n", ac_online, usb_online);
	if(ac_online || usb_online){
		return 1;
	}
	return 0;
}

static void cw_update_charge_status(struct cw_battery *cw_bat)
{
	int cw_charger_mode;
	cw_charger_mode = get_charge_state();
	if(cw_bat->charger_mode != cw_charger_mode){
		cw_bat->charger_mode = cw_charger_mode;
		cw_bat->change = 1;		
	}
}

static void cw_update_status(struct cw_battery *cw_bat)
{
	int status;

	if (cw_bat->charger_mode > 0) {
		if (cw_bat->capacity >= 100) 
			status = POWER_SUPPLY_STATUS_FULL;
		else
			status = POWER_SUPPLY_STATUS_CHARGING;
	} else {
		status = POWER_SUPPLY_STATUS_DISCHARGING;
	}

	if (cw_bat->status != status) {
		cw_bat->status = status;
		cw_bat->change = 1;
	} 
}

static int cw_init_data(struct cw_battery *cw_bat)
{
	unsigned char reg_SOC[2];
	int real_SOC = 0;
	int digit_SOC = 0;
	int ret;
	int UI_SOC = 0;



	ret = cw_read_word(cw_bat->client, REG_SOC, reg_SOC);
	if (ret < 0)
		return ret;

	real_SOC = reg_SOC[0];
	digit_SOC = reg_SOC[1];
	UI_SOC = ((real_SOC * 256 + digit_SOC) * 100)/ (UI_FULL*256);
	cw_bat1_printk("[%d]: real_SOC = %d, digit_SOC = %d\n", __LINE__, real_SOC, digit_SOC);



	cw_update_vol(cw_bat);
	cw_update_charge_status(cw_bat);
	cw_bat->capacity = UI_SOC;
	cw_update_status(cw_bat);
	return 0;
}
/*CW2015 quick soft reset*/
static int cw_quick_soft_reset(struct cw_battery *cw_bat)
{
	int ret= -1;
	/*Since Quick start bits not effect, chanage to write POR bits*/
	unsigned char reg_val =MODE_RESTART;

	ret = cw_write(cw_bat->client, REG_MODE, &reg_val);
	if (ret < 0) {
		/*Write MODE RESTART Fail*/
		cw_bat1_printk("Failed Write REG_MODE POR 0x%x, try agian.!\n",reg_val);
		ret = cw_write(cw_bat->client, REG_MODE, &reg_val);
		if (ret < 0)
			return ret;
	}
	msleep(10);
	cw_bat1_printk("Success Write REG_MODE POR 0x%x!\n",reg_val);
	return ret;
}
/*CW2015 init function, Often called during initialization*/
static int cw_init(struct cw_battery *cw_bat)
{
	int ret;
	int i;
	unsigned char reg_val = MODE_SLEEP;

	if ((reg_val & MODE_SLEEP_MASK) == MODE_SLEEP) {
		reg_val = MODE_NORMAL;
		ret = cw_write(cw_bat->client, REG_MODE, &reg_val);
		if (ret < 0) 
			return ret;
	}

	ret = cw_read(cw_bat->client, REG_CONFIG, &reg_val);
	if (ret < 0)
		return ret;

	if ((reg_val & 0xf8) != ATHD) {
		reg_val &= 0x07;    /* clear ATHD */
		reg_val |= ATHD;    /* set ATHD */
		ret = cw_write(cw_bat->client, REG_CONFIG, &reg_val);
		if (ret < 0)
			return ret;
	}

	ret = cw_read(cw_bat->client, REG_CONFIG, &reg_val);
	if (ret < 0) 
		return ret;

	if (!(reg_val & CONFIG_UPDATE_FLG)) {
		cw_bat1_printk("update config flg is true, need update config\n");
		ret = cw_update_config_info_bat1(cw_bat);
		if (ret < 0) {
			printk("%s : update config fail\n", __func__);
			return ret;
		}
	} else {
		for(i = 0; i < SIZE_BATINFO; i++) { 
			ret = cw_read(cw_bat->client, (REG_BATINFO + i), &reg_val);
			if (ret < 0)
				return ret;

			printk(KERN_INFO "%X\n", reg_val);
			if (config_info[i] != reg_val)
				break;
		}
		if (i != SIZE_BATINFO) {
			cw_bat1_printk("config didn't match, need update config\n");
			ret = cw_update_config_info_bat1(cw_bat);
			if (ret < 0){
				return ret;
			}
		}
	}

	msleep(10);
	for (i = 0; i < 30; i++) {
		ret = cw_read(cw_bat->client, REG_SOC, &reg_val);
		if (ret < 0)
			return ret;
		else if (reg_val <= 0x64) 
			break;
		msleep(120);
	}

	if (i >= 30 ){
		reg_val = MODE_SLEEP;
		ret = cw_write(cw_bat->client, REG_MODE, &reg_val);
		cw_bat1_printk("cw2015 input unvalid power error, cw2015 join sleep mode\n");
		return -1;
	} 

	cw_bat1_printk("cw2015 init success!\n");	
	return 0;
}

static int cw_por(struct cw_battery *cw_bat)
{
	int ret;
	unsigned char reset_val;

	reset_val = MODE_SLEEP; 			  
	ret = cw_write(cw_bat->client, REG_MODE, &reset_val);
	if (ret < 0)
		return ret;
	reset_val = MODE_NORMAL;
	msleep(10);
	ret = cw_write(cw_bat->client, REG_MODE, &reset_val);
	if (ret < 0)
		return ret;
	ret = cw_init(cw_bat);
	if (ret) 
		return ret;
	return 0;
}



static int cw_get_capacity(struct cw_battery *cw_bat)
{
	int ui_100 = UI_FULL;
	int remainder = 0;
	int real_SOC = 0;
	int digit_SOC = 0;
	int UI_SOC = 0;
	//int cw_capacity;
	int ret;
	unsigned char reg_val[2];
	//unsigned char reg_0x4f;
	//unsigned char temp_value;
	static int reset_loop = 0;
	static int charging_zero_loop = 0;


	ret = cw_read_word(cw_bat->client, REG_SOC, reg_val);
	if (ret < 0)
		return ret;

	/*
	   ret = cw_read(cw_bat->client, REG_LAST_FULL_CHARGE, &reg_0x4f);
	   if (ret < 0)
	   return ret;
	   */


	real_SOC = reg_val[0];
	digit_SOC = reg_val[1];
	printk(KERN_INFO "CW2015_bat1[%d]: real_SOC = %d, digit_SOC = %d\n", __LINE__, real_SOC, digit_SOC);

	/*case 1 : avoid IC error, read SOC > 100*/
	if ((real_SOC < 0) || (real_SOC > 100)) {
		cw_bat1_printk("Error:  real_SOC = %d\n", real_SOC);
		reset_loop++;			
		if (reset_loop > (BATTERY_CAPACITY_ERROR / queue_delayed_work_time)){ 
			cw_por(cw_bat);
			reset_loop =0;							 
		}

		return cw_bat->capacity; //cw_capacity Chaman change because I think customer didn't want to get error capacity.
	}else {
		reset_loop =0;
	}

	/*case 2 : avoid IC error, battery SOC is 0% when long time charging*/
	if((cw_bat->charger_mode > 0) &&(real_SOC == 0))
	{
		charging_zero_loop++;
		if (charging_zero_loop > BATTERY_CHARGING_ZERO / queue_delayed_work_time) {
			cw_por(cw_bat);
			charging_zero_loop = 0;
		}
	}else if(charging_zero_loop != 0){
		charging_zero_loop = 0;
	}


	UI_SOC = ((real_SOC * 256 + digit_SOC) * 100)/ (ui_100*256);
	remainder = (((real_SOC * 256 + digit_SOC) * 100 * 100) / (ui_100*256)) % 100;
	cw_bat1_printk("CW2015[%d]: ui_100 = %d, UI_SOC = %d, remainder = %d\n", __LINE__, ui_100, UI_SOC, remainder);
	/*case 3 : aviod swing*/
	if(UI_SOC >= 100){
		printk(KERN_INFO "CW2015[%d]: UI_SOC = %d larger 100!!!!\n", __LINE__, UI_SOC);
		UI_SOC = 100;
	}else{
		if((remainder > 70 || remainder < 30) && UI_SOC >= cw_bat->capacity - 1 && UI_SOC <= cw_bat->capacity + 1)
		{
			UI_SOC = cw_bat->capacity;
			printk(KERN_INFO "CW2015[%d]: UI_SOC = %d, cw_bat->capacity = %d\n", __LINE__, UI_SOC, cw_bat->capacity);
		}
	}

#ifdef CONFIG_PM
	if(suspend_resume_mark == 1)
		suspend_resume_mark = 0;
#endif
	return UI_SOC;
}
int g_cw_get_bat1_capacity(void)
{
	int capacity= 0;
#ifdef CONFIG_HISENSE_SGM_DOUBLE_BATTERIES
	/*In factory mode not care battery exist or not*/
	if(dev_bi.bootmode != BOOT_FACTORY_MODE){
		if(!(sgm_check_battery1_exist())){
			/*battery1 not exist*/
			capacity = -1;
			pr_buf_info( "bat1 not exist\n");
			return capacity;
		}
	}
#endif /*CONFIG_HISENSE_SGM_DOUBLE_BATTERIES*/

	if(!g_cw_bat1){
		pr_buf_err( "Could not get cw2015_bat1\n");
		return capacity;
	}
	
	//capacity = cw_get_capacity(g_cw_bat1);
	capacity = g_cw_bat1->capacity;
	pr_buf_info( "bat1 cap %d\n",capacity);
	return capacity;
}

EXPORT_SYMBOL(g_cw_get_bat1_capacity);
static void cw_update_capacity(struct cw_battery *cw_bat)
{
	int cw_capacity;
	cw_capacity = cw_get_capacity(cw_bat);

	if ((cw_capacity >= 0) && (cw_capacity <= 100) && (cw_bat->capacity != cw_capacity)) {
		if (cw_capacity > cw_bat->capacity) {
			/* SOC increased */
			if (cw_bat->charger_mode) /* Increment if input is present */
				cw_bat->capacity = cw_capacity;
			cw_bat1_printk( "charger_mode %d, status %d, capacity %d, cw_capacity %d\n",cw_bat->charger_mode,cw_bat->status,cw_bat->capacity,cw_capacity);
		}else{
			cw_bat->capacity = cw_capacity;
		}
		cw_bat->change = 1;
	}
}

static void cw_bat_work(struct work_struct *work)
{
	struct delayed_work *delay_work;
	struct cw_battery *cw_bat;
	/*Add for battery swap start*/
	int ret;
	unsigned char reg_val;
	//u16 value16;
	int i = 0;
	/*Add for battery swap end*/

	delay_work = container_of(work, struct delayed_work, work);
	cw_bat = container_of(delay_work, struct cw_battery, battery_delay_work);
       if(!cw_bat->isDataInit){
               cw_bat1_printk("Try Data Init again....\n");
               cw_bat->capacity = 0;//100;
               g_cw2015_bat1_capacity = cw_bat->capacity;
               ret = cw_init(cw_bat);
		while ((i++ < 3) && (ret != 0)) {
			msleep(200);
			ret = cw_init(cw_bat);
		}
                cw_bat1_printk(" cw2015 init retry %d times\n", i);
		if(i >=3)
			goto next_cw1_work;
               ret = cw_init_data(cw_bat);
               if (ret) {
                       cw_bat1_printk(" cw2015 init data fail,retry later!\n");
                       goto next_cw1_work;
               }
               cw_bat->isDataInit = true;
       }

	/*Add for battery swap start*/
	ret = cw_read(cw_bat->client, REG_MODE, &reg_val);
	cw_bat1_printk("REG_MODE = 0x%x\n", reg_val);
	if(ret < 0){
		//battery is out , you can send new battery capacity vol here what you want set
		//for example
		cw_bat->capacity = 0;//100;
		cw_bat->voltage = 3700;//4200;
		cw_bat->change = 1;
	}else{
		if((reg_val & MODE_SLEEP_MASK) == MODE_SLEEP){
			for(i = 0; i < 5; i++){
				if(cw_por(cw_bat) == 0)
					break;
			}
		}
		cw_update_vol(cw_bat);
		cw_update_charge_status(cw_bat);
		cw_update_capacity(cw_bat);
		cw_update_status(cw_bat);
	}
	/*Add for battery swap end*/
	cw_bat1_printk("charger_mod = %d\n", cw_bat->charger_mode);
	cw_bat1_printk("status = %d\n", cw_bat->status);
	cw_bat1_printk("capacity = %d\n", cw_bat->capacity);
	cw_bat1_printk("voltage = %d\n", cw_bat->voltage);

#ifdef CONFIG_PM
	if(suspend_resume_mark == 1)
		suspend_resume_mark = 0;
#endif

#ifdef CW_PROPERTIES
	if (cw_bat->change == 1){
#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 1, 0)
		power_supply_changed(&cw_bat->cw_bat); 
#else
		power_supply_changed(cw_bat->cw_bat); 
#endif
		cw_bat->change = 0;
	}
#endif
	if (cw_bat->change == 1){
		if(get_cw_bat_psy(cw_bat)){
			 goto next_cw1_work;
		}
		power_supply_changed(cw_bat->cw_bat);
		cw_bat->change = 0;
	}
	g_cw2015_bat1_capacity = cw_bat->capacity;
	g_cw2015_bat1_vol = cw_bat->voltage;
next_cw1_work:
	queue_delayed_work(cw_bat->cwfg_workqueue, &cw_bat->battery_delay_work, msecs_to_jiffies(queue_delayed_work_time));
}

#ifdef CW_PROPERTIES
static int cw_battery_get_property(struct power_supply *psy,
		enum power_supply_property psp,
		union power_supply_propval *val)
{
	int ret = 0;

#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 1, 0)
	struct cw_battery *cw_bat;
	cw_bat = container_of(psy, struct cw_battery, cw_bat); 
#else
	struct cw_battery *cw_bat = power_supply_get_drvdata(psy); 
#endif

	switch (psp) {
		case POWER_SUPPLY_PROP_CAPACITY:
			val->intval = cw_bat->capacity;
			break;
			/*
			   case POWER_SUPPLY_PROP_STATUS:   //Chaman charger ic will give a real value
			   val->intval = cw_bat->status; 
			   break;                 
			   */        
		case POWER_SUPPLY_PROP_HEALTH:   //Chaman charger ic will give a real value
			val->intval= POWER_SUPPLY_HEALTH_GOOD;
			break;
		case POWER_SUPPLY_PROP_PRESENT:
			val->intval = cw_bat->voltage <= 0 ? 0 : 1;
			break;

		case POWER_SUPPLY_PROP_VOLTAGE_NOW:
			val->intval = cw_bat->voltage * 1000;
			break;
		case POWER_SUPPLY_PROP_TECHNOLOGY:  //Chaman this value no need
			val->intval = POWER_SUPPLY_TECHNOLOGY_LION;	
			break;

		default:
			ret = -EINVAL; 
			break;
	}
	return ret;
}

static enum power_supply_property cw_battery_properties[] = {
	POWER_SUPPLY_PROP_CAPACITY,
	//POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_TECHNOLOGY,
};
#endif 

static int cw2015_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int ret;
	int loop = 0;
	struct cw_battery *cw_bat;

#ifdef CW_PROPERTIES
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 1, 0)	
	struct power_supply_desc *psy_desc;
	struct power_supply_config psy_cfg = {0};
#endif
#endif
	//struct device *dev;
	cw_bat1_printk("\n");

	cw_bat = devm_kzalloc(&client->dev, sizeof(*cw_bat), GFP_KERNEL);
	if (!cw_bat) {
		cw_bat1_printk("cw_bat create fail!\n");
		return -ENOMEM;
	}

	i2c_set_clientdata(client, cw_bat);

	cw_bat->client = client;
	cw_bat->capacity = 0;
	cw_bat->voltage = 0;
	cw_bat->status = 0;
	cw_bat->charger_mode = NO_CHARGING;
	cw_bat->change = 0;
	ret = cw_init(cw_bat);
	while ((loop++ < 3) && (ret != 0)) {
		msleep(200);
		ret = cw_init(cw_bat);
	}
	if (ret) {
		printk("%s : cw2015 init fail, retry later!\n", __func__);
		cw_bat->isDataInit = false;
		//return ret;
	}

	ret = cw_init_data(cw_bat);
	if (ret) {
		printk("%s : cw2015 init data fail,retry later!\n", __func__);
		cw_bat->isDataInit = false;
		//return ret;
	}

#ifdef CW_PROPERTIES
#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 1, 0)
	cw_bat->cw_bat.name = CW_PROPERTIES;
	cw_bat->cw_bat.type = POWER_SUPPLY_TYPE_BATTERY;
	cw_bat->cw_bat.properties = cw_battery_properties;
	cw_bat->cw_bat.num_properties = ARRAY_SIZE(cw_battery_properties);
	cw_bat->cw_bat.get_property = cw_battery_get_property;
	ret = power_supply_register(&client->dev, &cw_bat->cw_bat);
	if(ret < 0) {
		power_supply_unregister(&cw_bat->cw_bat);
		return ret;
	}
#else
	psy_desc = devm_kzalloc(&client->dev, sizeof(*psy_desc), GFP_KERNEL);
	if (!psy_desc)
		return -ENOMEM;

	psy_cfg.drv_data = cw_bat;
	psy_desc->name = CW_PROPERTIES;
	psy_desc->type = POWER_SUPPLY_TYPE_BATTERY;
	psy_desc->properties = cw_battery_properties;
	psy_desc->num_properties = ARRAY_SIZE(cw_battery_properties);
	psy_desc->get_property = cw_battery_get_property;
	cw_bat->cw_bat = power_supply_register(&client->dev, psy_desc, &psy_cfg);
	if(IS_ERR(cw_bat->cw_bat)) {
		ret = PTR_ERR(cw_bat->cw_bat);
		printk(KERN_ERR"failed to register battery: %d\n", ret);
		return ret;
	}
#endif
#endif
	get_cw_bat_psy(cw_bat);
	cw_bat->cwfg_workqueue = create_singlethread_workqueue("cwfg_bat1_gauge");
	INIT_DELAYED_WORK(&cw_bat->battery_delay_work, cw_bat_work);
	queue_delayed_work(cw_bat->cwfg_workqueue, &cw_bat->battery_delay_work , msecs_to_jiffies(50));
	g_cw_bat1 = cw_bat;
	if(!ret)
		cw_bat->isDataInit = true;
	cw_bat1_printk("cw2015 driver probe success and cw_init_data %s!\n", ((cw_bat->isDataInit) ?" Success":"Fail"));
	return 0;
}

/*
   static int cw2015_detect(struct i2c_client *client, struct i2c_board_info *info) 
   {	 
   cw_bat1_printk("\n");
   strcpy(info->type, CWFG_NAME);
   return 0;
   }
   */

#ifdef CONFIG_PM
static int cw_bat_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct cw_battery *cw_bat = i2c_get_clientdata(client);
	read_persistent_clock(&suspend_time_before);
	cancel_delayed_work(&cw_bat->battery_delay_work);
	return 0;
}

static int cw_bat_resume(struct device *dev)
{	
	struct i2c_client *client = to_i2c_client(dev);
	struct cw_battery *cw_bat = i2c_get_clientdata(client);
	suspend_resume_mark = 1;
	read_persistent_clock(&after);
	after = timespec_sub(after, suspend_time_before);
	queue_delayed_work(cw_bat->cwfg_workqueue, &cw_bat->battery_delay_work, msecs_to_jiffies(2));
	return 0;
}

static const struct dev_pm_ops cw_bat_pm_ops = {
	.suspend  = cw_bat_suspend,
	.resume   = cw_bat_resume,
};
#endif

static int cw2015_remove(struct i2c_client *client)	 
{
	struct cw_battery *cw_bat = i2c_get_clientdata(client);
	if (!IS_ERR_OR_NULL(cw_bat)) {
		cw_bat->isDataInit = false;
	}
	cw_bat1_printk("\n");
	return 0;
}

static const struct i2c_device_id cw2015_id_table[] = {
	{CWFG_NAME, 0},
	{}
};

static struct of_device_id cw2015_match_table[] = {
	{ .compatible = "cellwise,bat1_cw2015", },
	{ },
};

static struct i2c_driver cw2015_driver = {
	.driver 	  = {
		.name = CWFG_NAME,
#ifdef CONFIG_PM
		.pm     = &cw_bat_pm_ops,
#endif
		.owner	= THIS_MODULE,
		.of_match_table = cw2015_match_table,
	},
	.probe		  = cw2015_probe,
	.remove 	  = cw2015_remove,
	//.detect 	  = cw2015_detect,
	.id_table = cw2015_id_table,
};

/*
   static struct i2c_board_info __initdata fgadc_dev = { 
   I2C_BOARD_INFO(CWFG_NAME, 0x62) 
   };
   */

static int __init cw215_bat1_init(void)
{
	//struct i2c_client *client;
	///struct i2c_adapter *i2c_adp;
	cw_bat1_printk("\n");

	//i2c_register_board_info(CWFG_I2C_BUSNUM, &fgadc_dev, 1);
	//i2c_adp = i2c_get_adapter(CWFG_I2C_BUSNUM);
	//client = i2c_new_device(i2c_adp, &fgadc_dev);

	i2c_add_driver(&cw2015_driver);
	return 0; 
}

/*
//Add to dsti file
cw2015@62 { 
compatible = "cellwise,cw2015";
reg = <0x62>;
} 
*/

static void __exit cw215_bat1_exit(void)
{
	i2c_del_driver(&cw2015_driver);
}

module_init(cw215_bat1_init);
module_exit(cw215_bat1_exit);

MODULE_AUTHOR("Chaman Qi");
MODULE_DESCRIPTION("CW2015 FGADC Device Driver V3.0");
MODULE_LICENSE("GPL");
