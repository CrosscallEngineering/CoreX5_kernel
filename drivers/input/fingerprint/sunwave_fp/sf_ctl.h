/**
 * User space driver API for Sunwave's fingerprint device.
 * ATTENTION: Do NOT edit this file unless the corresponding driver changed.
 *
 * Copyright (C) 2016 Sunwave Corporation. <http://www.sunwavecorp.com>
 * Copyright (C) 2016 Langson Leung <mailto:liangzh@sunwavecorp.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the Free
 * Software Foundation; either version 2 of the License, or (at your option)
 * any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General
 * Public License for more details.
**/

#ifndef __SF_CTRL_API_H__
#define __SF_CTRL_API_H__

#include "sf_user.h"
#include "linux/version.h"

#include <linux/miscdevice.h>

#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#else
#include <linux/fb.h>
#include <linux/notifier.h>
#endif
#include <linux/pm_wakeup.h>
#if (SF_POWER_MODE_SEL == PWR_MODE_REGULATOR)
#include <linux/regulator/consumer.h>
#endif

#if SF_TRUSTKERNEL_COMPATIBLE
#include <tee_fp.h>
#endif

#if SF_BEANPOD_COMPATIBLE_V2
#include <fp_vendor.h>
#endif

#if REE_MTK_ANDROID_L
#include <cust_eint.h>
#include <cust_eint_md1.h>
#include "cust_gpio_usage.h"
#include <mach/mt_gpio.h>
#include <mach/mt_spi.h>
#include <mach/eint.h>
#include <cust_eint.h>
#endif

#if MTK_6739_SPEED_MODE
#include "sf_spi.h"
#endif

#if SF_SPI_RW_EN
#if (SF_MTK_CPU && !REE_MTK_ANDROID_L)
#if LINUX_VERSION_CODE < KERNEL_VERSION(4,0,0)
#include "mt_spi.h"
#include "mt_spi_hal.h"
#elif !REE_MTK_ANDROID_L
#include "mtk_spi.h"
#include "mtk_spi_hal.h"
#endif
#endif
typedef struct spi_device sf_device_t;
typedef struct spi_driver sf_driver_t;
#define SW_BUS_NAME "spi bus"
#else
#include <linux/platform_device.h>
typedef struct platform_device sf_device_t;
typedef struct platform_driver sf_driver_t;
#define SW_BUS_NAME "platform bus"
#endif

struct sf_ctl_device {
    struct miscdevice miscdev;
    int rst_num;
    int irq_pin;
    int irq_num;
    int pwr_num;
    struct work_struct work_queue;
    struct input_dev *input;
    struct regulator *vdd_reg;
    int  (*gpio_init) (struct sf_ctl_device *ctl_dev);
    int  (*free_gpio) (struct sf_ctl_device *ctl_dev);
    int  (*power_on)  (bool on);
    int  (*spi_clk_on)(bool on);
    int  (*reset)     (void);

    struct wakeup_source wakelock;

#ifdef CONFIG_HAS_EARLYSUSPEND
    struct early_suspend early_suspend;
#else
    struct notifier_block notifier;
#endif
#if defined(CONFIG_DRM_PANEL_NOUSE)
	struct workqueue_struct *checkdrm_workqueue;
	struct delayed_work checkdrm_delay_work;
#endif
    char *spi_buffer;
    int attribute;
    sf_device_t *pdev;	
};

/* int event. */
#define SF_INT_EVENT_NAME "SPI_STATE=finger"

/* Device node. */
#define SF_CTL_DEV_NAME   "/dev/sunwave_fp"

/* Max driver version buffer length. */
#define SF_DRV_VERSION_LEN 32

typedef enum {
    SF_KEY_NONE = 0,
    SF_KEY_HOME,
    SF_KEY_MENU,
    SF_KEY_BACK,
    SF_KEY_F11,
    SF_KEY_ENTER,
    SF_KEY_UP,
    SF_KEY_LEFT,
    SF_KEY_RIGHT,
    SF_KEY_DOWN,
    SF_KEY_WAKEUP,
} sf_key_type_t;

//finger nv key start
#define SF_NVKEY_CLICK 			530
#define SF_NVKEY_LONG_CLICK		531
#define SF_NVKEY_MOVE_RIGHT		532
#define SF_NVKEY_MOVE_LEFT		533
#define SF_NVKEY_MOVE_DOWN		534
#define SF_NVKEY_MOVE_UP		535
//finger nv key end

typedef struct {
    sf_key_type_t key;
    int value; /* for key type, 0 means up, 1 means down. */
} sf_key_event_t;

#define SF_MAX_VER_INFO_LEN 32

typedef struct {
    char tee_solution[SF_MAX_VER_INFO_LEN];
    char ca_version  [SF_MAX_VER_INFO_LEN * 2];
    char ta_version  [SF_MAX_VER_INFO_LEN * 2];
    char algorithm   [SF_MAX_VER_INFO_LEN];
    char algo_nav    [SF_MAX_VER_INFO_LEN];
    char driver      [SF_MAX_VER_INFO_LEN];
    char firmware    [SF_MAX_VER_INFO_LEN];
    char fortsense_id  [SF_MAX_VER_INFO_LEN];
    char vendor_id   [SF_MAX_VER_INFO_LEN];
} __attribute__((__packed__)) fsfp_version_info_t;

/* Magic code for IOCTL-subsystem, 's'(0x73) means 'Sunwave'. */
#define SF_IOC_MAGIC 's'

/* Allocate/Release driver resource (GPIO/SPI etc.). */
#define SF_IOC_INIT_DRIVER      _IO(SF_IOC_MAGIC, 0x00)
#define SF_IOC_DEINIT_DRIVER    _IO(SF_IOC_MAGIC, 0x01)

/* HW reset the fingerprint module. */
#define SF_IOC_RESET_DEVICE     _IO(SF_IOC_MAGIC, 0x02)

/* Low-level IRQ control. */
#define SF_IOC_ENABLE_IRQ       _IO(SF_IOC_MAGIC, 0x03)
#define SF_IOC_DISABLE_IRQ      _IO(SF_IOC_MAGIC, 0x04)
#define SF_IOC_REQUEST_IRQ      _IO(SF_IOC_MAGIC, 0x21)

/* SPI bus clock control, for power-saving purpose. */
#define SF_IOC_ENABLE_SPI_CLK   _IO(SF_IOC_MAGIC, 0x05)
#define SF_IOC_DISABLE_SPI_CLK  _IO(SF_IOC_MAGIC, 0x06)

/* Fingerprint module power control. */
#define SF_IOC_ENABLE_POWER     _IO(SF_IOC_MAGIC, 0x07)
#define SF_IOC_DISABLE_POWER    _IO(SF_IOC_MAGIC, 0x08)

/* Androind system-wide key event, for navigation purpose. */
#define SF_IOC_REPORT_KEY_EVENT _IOW(SF_IOC_MAGIC, 0x09, sf_key_event_t *)

/* Sync 'sf_driver_config_t', the driver configuration. */
#define SF_IOC_SYNC_CONFIG      _IOWR(SF_IOC_MAGIC, 0x0a, void *)

/* SPI speed related. */
#define SF_IOC_SPI_SPEED        _IO(SF_IOC_MAGIC, 0x0b)

/* patform attribute */
#define SF_IOC_ATTRIBUTE        _IO(SF_IOC_MAGIC, 0x0d)

/* Query the driver version string. */
#define SF_IOC_GET_VERSION      _IOR(SF_IOC_MAGIC, 0x20, const char *)

/* fortsense hal lib version info set&get. */
#define SF_IOC_SET_LIB_VERSION  _IO(SF_IOC_MAGIC, 0x30)
#define SF_IOC_GET_LIB_VERSION  _IO(SF_IOC_MAGIC, 0x31)
#endif /* __SF_CTRL_API_H__ */
