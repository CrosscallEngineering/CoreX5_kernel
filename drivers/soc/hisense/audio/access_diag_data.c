/*
 * Copyright (C) 2017-2018 Hisense, Inc.
 *
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

#include <linux/kernel.h>
#include <linux/err.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/his_debug_base.h>

#ifdef CONFIG_ARCH_SPRD
#define DIAG_PARTITION       "/dev/block/by-name/diag"
#else
#define DIAG_PARTITION       "/dev/block/bootdevice/by-name/diag"
#endif
#define SMARTPA_OFFSET       501760
//#define SMARTPA_LENGTH       2048
#define DEBUSSY_CALIDATA_OFFSET       500736
//#define DEBUSSY_CALIDATA_LENGTH 256


int read_smartPA_from_diag_partition(void *buff, u32 size)
{
	int  ret=0;

	pr_err("read_smartPA_from_diag_partition : E\n");
	if (buff == NULL) {
		pr_err("read_smartPA_from_diag_partition buf is null\n");
		return 0;
	}

	ret = his_read_file(DIAG_PARTITION, SMARTPA_OFFSET,
			buff, size);
	if (ret <= 0) {
		pr_err("read_smartPA_from_diag_partition read file failed!\n");
		return 0;
	}

	pr_err("read_smartPA is SUCCESSFUL!\n");

	return 1;
}
EXPORT_SYMBOL(read_smartPA_from_diag_partition);

int read_dual_smartPA_from_diag_partition(void *buff, u32 size, loff_t pos)
{
	int  ret=0;

	pr_err("read_smartPA_from_diag_partition : E\n");
	if (buff == NULL) {
		pr_err("read_smartPA_from_diag_partition buf is null\n");
		return 0;
	}

	ret = his_read_file(DIAG_PARTITION, (SMARTPA_OFFSET+pos),
			buff, size);
	if (ret <= 0) {
		pr_err("read_smartPA_from_diag_partition read file failed!\n");
		return 0;
	}

	pr_err("read_smartPA is SUCCESSFUL!\n");

	return 1;
}
EXPORT_SYMBOL(read_dual_smartPA_from_diag_partition);

int write_smartPA_from_diag_partition(void *buff, u32 size)
{
	int  ret=0;

	pr_err("write_smartPA_from_diag_partition : E\n");
	if (buff == NULL) {
		pr_err("write_smartPA_from_diag_partition buf is null\n");
		return 0;
	}

	ret = his_write_file(DIAG_PARTITION, SMARTPA_OFFSET,
			buff, size);

	if (ret < 0) {
		pr_err("write_smartPA_from_diag_partition write file failed!\n");
		return 0;
	}

	pr_err("write_smartPA_calidata is SUCCESSFUL!\n");

	return 1;
}
EXPORT_SYMBOL(write_smartPA_from_diag_partition);

int write_dual_smartPA_from_diag_partition(void *buff, u32 size, loff_t pos)
{
	int  ret=0;

	pr_err("write_dual_smartPA_from_diag_partition : E\n");
	if (buff == NULL) {
		pr_err("write_dual_smartPA_from_diag_partition buf is null\n");
		return 0;
	}

	ret = his_write_file(DIAG_PARTITION, (SMARTPA_OFFSET+pos),
			buff, size);

	if (ret < 0) {
		pr_err("write_dual_smartPA_from_diag_partition write file failed!\n");
		return 0;
	}

	pr_err("write_dual_smartPA_from_diag_partition is SUCCESSFUL!\n");

	return 1;
}
EXPORT_SYMBOL(write_dual_smartPA_from_diag_partition);


int read_debussy_calidata_from_diag_partition(void *buff, u32 size)
{
	int  ret=0;

	pr_err("read_debussy_calidata_from_diag_partition : E\n");
	if (buff == NULL) {
		pr_err("read_debussy_calidata_from_diag_partition buf is null\n");
		return 0;
	}

	ret = his_read_file(DIAG_PARTITION, DEBUSSY_CALIDATA_OFFSET,
			buff, size);
	if (ret <= 0) {
		pr_err("read_debussy_calidata_from_diag_partition read file failed!\n");
		return 0;
	}

	pr_err("read_debussy_calidata is SUCCESSFUL!\n");

	return 1;
}
EXPORT_SYMBOL(read_debussy_calidata_from_diag_partition);

int write_debussy_calidata_to_diag_partition(void *buff, u32 size)
{
	int  ret=0;

	pr_err("write_debussy_calidata_to_diag_partition : E\n");
	if (buff == NULL) {
		pr_err("write_debussy_calidata_to_diag_partition buf is null\n");
		return 0;
	}

	ret = his_write_file(DIAG_PARTITION, DEBUSSY_CALIDATA_OFFSET,
			buff, size);

	if (ret < 0) {
		pr_err("write_debussy_calidata_to_diag_partition write file failed!\n");
		return 0;
	}

	pr_err("write_debussy_calidata is SUCCESSFUL!\n");

	return 1;
}
EXPORT_SYMBOL(write_debussy_calidata_to_diag_partition);
