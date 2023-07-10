/*
 * Copyright (C) 2013-2014 Hisense, Inc.
 *
 * Author:
 *   zhaoyufeng <zhaoyufeng@hisense.com>
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
#include <linux/of.h>
#include <linux/io.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/utsname.h>
#include <linux/export.h>
#include <linux/productinfo.h>
#include <linux/module.h>
#include <linux/his_debug_base.h>

#define PRODUCTINFO_BUFF_LEN  200
#define MAX_CPUINFO_NUM  10

#define CPU_FEATURE_ID_ADDR  0x01B44180
#define CPU_FEATURE_ID_MASK  0xFF00000
#define CPU_FEATURE_ID_SHIFT 20
#define FEATURE_ID_STR_LEN 	 20


const char *deviceclassname[PRODUCTINFO_ID_NUM] = {
	"Vendor",
	"Board and product",
	"Hardware version",
	"LCD",
	"HWTCONFW",
	"HWTCONBL",
	"EPDVCOM",
	"CTP",
	"SUB CTP",
	"HDMI",
	"Main camera",
	"Sub camera",
	"Macro camera",
	"Wide-angle camera",
	"Front camera",
	"DDR",
	"EMMC",
	"EMMC more",
	"UFS",
	"UFS more",
	"NAND",
	"Accelerometer sensor",
	"Compass sensor",
	"Alps sensor",
	"BT",
	"WIFI",
	"Codec",
	"Modem",
	"LED",
	"Main anx camera",
	"Gyro sensor",
	"Hall sensor",
	"CPU_ID",
	"Fused",
	"FingerPrint",
	"NFC",
	"ALS_1",
	"ALS_2",
	"EPD",
	"Barometer",
	"Temperature_Humidity",
	"SAR sensor",
	"HIFI",
	"CPU Varlant name"
};

struct productinfo_struct {
	int   used;
	char  productinfo_data[PRODUCTINFO_BUFF_LEN];
};

struct cpuinfo_struct {
	int  		cpuinfo_num;
	const char 	*cpuinfo_list[MAX_CPUINFO_NUM][2];
};

struct productinfo_struct productinfo_data[PRODUCTINFO_ID_NUM];
char *productinfo_data_ptr;

static struct cpuinfo_struct cpuinfo_data;

static uint cpu_reg_read(u32 addr)
{
	void __iomem *base;
	uint r_value;

	base = ioremap(addr, 4);
	if (!base) {
		pr_err("%s: Error read cpu register\n", __func__);
		return 0;
	}
	r_value = __raw_readl(base);
	iounmap(base);

	return r_value;
}

static int get_cpu_feature_id(void)
{
	int data = 0, feature_id = 0;

	data = cpu_reg_read(CPU_FEATURE_ID_ADDR);
	feature_id = (data & CPU_FEATURE_ID_MASK) >> CPU_FEATURE_ID_SHIFT;

	pr_err("%s, feature_id:0x%x\n", __func__, feature_id);

	return feature_id;
}

static int of_parse_productinfo_config(void)
{
	int i = 0, j = 0;
	struct device_node *np = NULL;
	int size;

	np = of_find_node_by_path("/soc/his_devinfo");
	if (!np) {
		np = of_find_node_by_path("/his_devinfo");
		if (!np) {
			pr_err("Can not find his_devinfo node\n");
			return -EINVAL;
		}
	}

	size = of_property_count_strings(np, "dev,cpuinfo-list");
	if ((size <= 0) || ((size/2) > MAX_CPUINFO_NUM)) {
		pr_err("the cpuinfo list size is error\n");
		return -EINVAL;
	}

	cpuinfo_data.cpuinfo_num = size / 2;
	pr_err("found cpuinfo num: %d\n", cpuinfo_data.cpuinfo_num);
	for (i = 0, j = 0; i < size; i++) {
		if ((i % 2) == 0) {
			of_property_read_string_index(np, "dev,cpuinfo-list",
				i, &cpuinfo_data.cpuinfo_list[j][0]);
		} else {
			of_property_read_string_index(np, "dev,cpuinfo-list",
				i, &cpuinfo_data.cpuinfo_list[j][1]);
			j++;
		}
	}

	return 0;
}

int productinfo_register(int id, const char *devname, const char *devinfo)
{
	int len = 0;

	if (id >= PRODUCTINFO_ID_NUM)
		return -ENOMEM;

	if (!deviceclassname[id])
		return -ENOMEM;

	len = strlen(deviceclassname[id]);
	if (devname)
		len += strlen(devname);

	if (devinfo)
		len += strlen(devinfo);

	if (len >= PRODUCTINFO_BUFF_LEN - 5)
		return -ENOMEM;

	memset(productinfo_data[id].productinfo_data, 0,
			sizeof(productinfo_data[id].productinfo_data));
	productinfo_data_ptr = productinfo_data[id].productinfo_data;
	productinfo_data[id].used = 1;
	strlcat(productinfo_data_ptr, deviceclassname[id], PRODUCTINFO_BUFF_LEN);
	if (devname) {
		strlcat(productinfo_data_ptr, ": ", PRODUCTINFO_BUFF_LEN);
		strlcat(productinfo_data_ptr, devname, PRODUCTINFO_BUFF_LEN);
	}
	if (devinfo) {
		strlcat(productinfo_data_ptr, "--", PRODUCTINFO_BUFF_LEN);
		strlcat(productinfo_data_ptr, devinfo, PRODUCTINFO_BUFF_LEN);
	}
	strlcat(productinfo_data_ptr, "\n", PRODUCTINFO_BUFF_LEN);

	return 0;
}
EXPORT_SYMBOL(productinfo_register);

int productinfo_dump(char *buf, int offset)
{
	int i = 0;
	int len = 0;
	char *ptr = NULL;

	for (i = 0; i < PRODUCTINFO_ID_NUM; i++) {
		if (productinfo_data[i].used) {
			ptr = productinfo_data[i].productinfo_data;
			len = strlen(ptr);
			memcpy(buf + offset, ptr, len);
			offset += len;
		}
	}

	return 0;
}
EXPORT_SYMBOL(productinfo_dump);

char *get_product_hw_info(int id)
{
	return productinfo_data[id].productinfo_data;
}

static int productinfo_proc_show(struct seq_file *m, void *v)
{
	int i;

	for (i = 0; i < PRODUCTINFO_ID_NUM; i++) {
		if (productinfo_data[i].used) {
			productinfo_data_ptr = productinfo_data[i].productinfo_data;
			seq_write(m, productinfo_data_ptr, strlen(productinfo_data_ptr));
		}
	}

	return 0;
}

static int productinfo_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, productinfo_proc_show, NULL);
}

static const struct file_operations productinfo_proc_fops = {
	.open       = productinfo_proc_open,
	.read       = seq_read,
	.llseek     = seq_lseek,
	.release    = single_release,
};

static void get_cpu_varlantname_info(void)
{
	int feature_id = 0, cur_len = 0;
	char str[FEATURE_ID_STR_LEN] = {0};
	int i;

	feature_id = get_cpu_feature_id();
	cur_len = sprintf(str, "0x%x", feature_id);

	pr_err("%s, Feature id str: %s\n", __func__, str);

	for (i = 0; i < cpuinfo_data.cpuinfo_num; i++) {
		const char *feature_id = NULL;
		const char *varlant_name = NULL;

		feature_id = cpuinfo_data.cpuinfo_list[i][0];
		varlant_name = cpuinfo_data.cpuinfo_list[i][1];
		if (!strcmp(str, feature_id)) {
			productinfo_register(PRODUCTINFO_CPU_VARLANT_ID, varlant_name, NULL);
			break;
		}
	}
}

static int __init proc_productinfo_init(void)
{
	int ret = -EPERM;

	ret = his_create_procfs_file("productinfo", S_IRUGO,
			&productinfo_proc_fops);
	if (ret < 0)
		return -ENOMEM;

	ret = of_parse_productinfo_config();
	if(ret < 0)
		pr_err("%s, parse productinfo config failed\n", __func__);

	get_cpu_varlantname_info();

#ifdef CONFIG_MACH_HISENSE_SMARTPHONE
	productinfo_register(PRODUCTINFO_VENDOR_ID, CONFIG_HISENSE_VENDOR_NAME, NULL);
	productinfo_register(PRODUCTINFO_BOARD_ID, CONFIG_HISENSE_PRODUCT_NAME, NULL);
#endif /* CONFIG_MACH_HISENSE_SMARTPHONE */

	return 0;
}
module_init(proc_productinfo_init);

