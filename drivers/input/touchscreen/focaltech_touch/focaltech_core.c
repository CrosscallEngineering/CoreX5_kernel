/*
 *
 * FocalTech TouchScreen driver.
 *
 * Copyright (c) 2012-2020, FocalTech Systems, Ltd., all rights reserved.
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
/*****************************************************************************
*
* File Name: focaltech_core.c
*
* Author: Focaltech Driver Team
*
* Created: 2016-08-08
*
* Abstract: entrance for focaltech ts driver
*
* Version: V1.0
*
*****************************************************************************/

/*****************************************************************************
* Included header files
*****************************************************************************/
#include <linux/module.h>
#include <linux/init.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/of_irq.h>
#if defined(CONFIG_DRM)
#if 0//defined(CONFIG_DRM_PANEL)
#include <drm/drm_panel.h>
#else
#include <linux/msm_drm_notify.h>
#endif
#elif defined(CONFIG_FB)
#include <linux/notifier.h>
#include <linux/fb.h>
#elif defined(CONFIG_HAS_EARLYSUSPEND)
#include <linux/earlysuspend.h>
#define FTS_SUSPEND_LEVEL 1     /* Early-suspend level */
#endif
#include "focaltech_core.h"
#include <linux/productinfo.h>

/*****************************************************************************
* Private constant and macro definitions using #define
*****************************************************************************/
#define FTS_DRIVER_NAME                     "fts_ts"
#define FTS_DRIVER_PEN_NAME                 "fts_ts,pen"
#define INTERVAL_READ_REG                   200  /* unit:ms */
#define TIMEOUT_READ_REG                    1000 /* unit:ms */
#if FTS_POWER_SOURCE_CUST_EN
#define FTS_VTG_MIN_UV                      2800000
#define FTS_VTG_MAX_UV                      3300000
#define FTS_I2C_VTG_MIN_UV                  1800000
#define FTS_I2C_VTG_MAX_UV                  1800000
#endif
enum {
	FW_UPGRADE_FAILED = 0,
	FW_UPGRADE_SUCCESS,
	FW_IS_UPDATETING,
};
#if defined(CONFIG_TOUCHSCREEN_FTS_FH)
extern int usb_flag;
#endif

/*****************************************************************************
* Global variable or extern global variabls/functions
*****************************************************************************/
struct fts_ts_data *fts_data;
static bool fts_probed = false;

/*****************************************************************************
* Static function prototypes
*****************************************************************************/
static int fts_ts_suspend(struct device *dev);
static int fts_ts_resume(struct device *dev);

int fts_check_cid(struct fts_ts_data *ts_data, u8 id_h)
{
    int i = 0;
    struct ft_chip_id_t *cid = &ts_data->ic_info.cid;
    u8 cid_h = 0x0;

    if (cid->type == 0)
        return -ENODATA;

    for (i = 0; i < FTS_MAX_CHIP_IDS; i++) {
        cid_h = ((cid->chip_ids[i] >> 8) & 0x00FF);
        if (cid_h && (id_h == cid_h)) {
            return 0;
        }
    }

    return -ENODATA;
}

/*****************************************************************************
*  Name: fts_wait_tp_to_valid
*  Brief: Read chip id until TP FW become valid(Timeout: TIMEOUT_READ_REG),
*         need call when reset/power on/resume...
*  Input:
*  Output:
*  Return: return 0 if tp valid, otherwise return error code
*****************************************************************************/
int fts_wait_tp_to_valid(void)
{
    int ret = 0;
    int cnt = 0;
    u8 idh = 0;
    struct fts_ts_data *ts_data = fts_data;
    u8 chip_idh = ts_data->ic_info.ids.chip_idh;

    do {
        ret = fts_read_reg(FTS_REG_CHIP_ID, &idh);
        if ((idh == chip_idh) || (fts_check_cid(ts_data, idh) == 0)) {
            FTS_INFO("TP Ready,Device ID:0x%02x", idh);
            return 0;
        } else
            FTS_DEBUG("TP Not Ready,ReadData:0x%02x,ret:%d", idh, ret);

        cnt++;
        msleep(INTERVAL_READ_REG);
    } while ((cnt * INTERVAL_READ_REG) < TIMEOUT_READ_REG);

    return -EIO;
}

/*****************************************************************************
*  Name: fts_tp_state_recovery
*  Brief: Need execute this function when reset
*  Input:
*  Output:
*  Return:
*****************************************************************************/
void fts_tp_state_recovery(struct fts_ts_data *ts_data)
{
    FTS_FUNC_ENTER();
    /* wait tp stable */
    fts_wait_tp_to_valid();
    /* recover TP charger state 0x8B */
    /* recover TP glove state 0xC0 */
    /* recover TP cover state 0xC1 */
    fts_ex_mode_recovery(ts_data);
    /* recover TP gesture state 0xD0 */
#ifdef CONFIG_TOUCHSCREEN_FTS_GESTURE
    fts_gesture_recovery(ts_data);
#endif/*CONFIG_TOUCHSCREEN_FTS_GESTURE*/
    FTS_FUNC_EXIT();
}

int fts_reset_proc(int hdelayms)
{
    FTS_INFO("tp reset");
    gpio_direction_output(fts_data->pdata->reset_gpio, 0);
    msleep(1);
    gpio_direction_output(fts_data->pdata->reset_gpio, 1);
    if (hdelayms) {
        msleep(hdelayms);
    }

    return 0;
}

void fts_irq_disable(void)
{
    unsigned long irqflags;

    FTS_FUNC_ENTER();
    spin_lock_irqsave(&fts_data->irq_lock, irqflags);

    if (!fts_data->irq_disabled) {
        disable_irq_nosync(fts_data->irq);
        fts_data->irq_disabled = true;
    }

    spin_unlock_irqrestore(&fts_data->irq_lock, irqflags);
    FTS_FUNC_EXIT();
}

void fts_irq_enable(void)
{
    unsigned long irqflags = 0;

    FTS_FUNC_ENTER();
    spin_lock_irqsave(&fts_data->irq_lock, irqflags);

    if (fts_data->irq_disabled) {
        enable_irq(fts_data->irq);
        fts_data->irq_disabled = false;
    }

    spin_unlock_irqrestore(&fts_data->irq_lock, irqflags);
    FTS_FUNC_EXIT();
}

void fts_hid2std(void)
{
    int ret = 0;
    u8 buf[3] = {0xEB, 0xAA, 0x09};

    if (fts_data->bus_type != BUS_TYPE_I2C)
        return;

    ret = fts_write(buf, 3);
    if (ret < 0) {
        FTS_ERROR("hid2std cmd write fail");
    } else {
        msleep(10);
        buf[0] = buf[1] = buf[2] = 0;
        ret = fts_read(NULL, 0, buf, 3);
        if (ret < 0) {
            FTS_ERROR("hid2std cmd read fail");
        } else if ((0xEB == buf[0]) && (0xAA == buf[1]) && (0x08 == buf[2])) {
            FTS_DEBUG("hidi2c change to stdi2c successful");
        } else {
            FTS_DEBUG("hidi2c change to stdi2c not support or fail");
        }
    }
}

static int fts_match_cid(struct fts_ts_data *ts_data,
                         u16 type, u8 id_h, u8 id_l, bool force)
{
#ifdef FTS_CHIP_ID_MAPPING
    u32 i = 0;
    u32 j = 0;
    struct ft_chip_id_t chip_id_list[] = FTS_CHIP_ID_MAPPING;
    u32 cid_entries = sizeof(chip_id_list) / sizeof(struct ft_chip_id_t);
    u16 id = (id_h << 8) + id_l;

    memset(&ts_data->ic_info.cid, 0, sizeof(struct ft_chip_id_t));
    for (i = 0; i < cid_entries; i++) {
        if (!force && (type == chip_id_list[i].type)) {
            break;
        } else if (force && (type == chip_id_list[i].type)) {
            FTS_INFO("match cid,type:0x%x", (int)chip_id_list[i].type);
            ts_data->ic_info.cid = chip_id_list[i];
            return 0;
        }
    }

    if (i >= cid_entries) {
        return -ENODATA;
    }

    for (j = 0; j < FTS_MAX_CHIP_IDS; j++) {
        if (id == chip_id_list[i].chip_ids[j]) {
            FTS_DEBUG("cid:%x==%x", id, chip_id_list[i].chip_ids[j]);
            FTS_INFO("match cid,type:0x%x", (int)chip_id_list[i].type);
            ts_data->ic_info.cid = chip_id_list[i];
            return 0;
        }
    }

    return -ENODATA;
#else
    return -EINVAL;
#endif
}


static int fts_get_chip_types(
    struct fts_ts_data *ts_data,
    u8 id_h, u8 id_l, bool fw_valid)
{
    u32 i = 0;
    struct ft_chip_t ctype[] = FTS_CHIP_TYPE_MAPPING;
    u32 ctype_entries = sizeof(ctype) / sizeof(struct ft_chip_t);

    if ((0x0 == id_h) || (0x0 == id_l)) {
        FTS_ERROR("id_h/id_l is 0");
        return -EINVAL;
    }

    FTS_DEBUG("verify id:0x%02x%02x", id_h, id_l);
    for (i = 0; i < ctype_entries; i++) {
        if (VALID == fw_valid) {
            if (((id_h == ctype[i].chip_idh) && (id_l == ctype[i].chip_idl))
                || (!fts_match_cid(ts_data, ctype[i].type, id_h, id_l, 0)))
                break;
        } else {
            if (((id_h == ctype[i].rom_idh) && (id_l == ctype[i].rom_idl))
                || ((id_h == ctype[i].pb_idh) && (id_l == ctype[i].pb_idl))
                || ((id_h == ctype[i].bl_idh) && (id_l == ctype[i].bl_idl))) {
                break;
            }
        }
    }

    if (i >= ctype_entries) {
        return -ENODATA;
    }

    fts_match_cid(ts_data, ctype[i].type, id_h, id_l, 1);
    ts_data->ic_info.ids = ctype[i];
    return 0;
}

static int fts_read_bootid(struct fts_ts_data *ts_data, u8 *id)
{
    int ret = 0;
    u8 chip_id[2] = { 0 };
    u8 id_cmd[4] = { 0 };
    u32 id_cmd_len = 0;

    id_cmd[0] = FTS_CMD_START1;
    id_cmd[1] = FTS_CMD_START2;
    ret = fts_write(id_cmd, 2);
    if (ret < 0) {
        FTS_ERROR("start cmd write fail");
        return ret;
    }

    msleep(FTS_CMD_START_DELAY);
    id_cmd[0] = FTS_CMD_READ_ID;
    id_cmd[1] = id_cmd[2] = id_cmd[3] = 0x00;
    if (ts_data->ic_info.is_incell)
        id_cmd_len = FTS_CMD_READ_ID_LEN_INCELL;
    else
        id_cmd_len = FTS_CMD_READ_ID_LEN;
    ret = fts_read(id_cmd, id_cmd_len, chip_id, 2);
    if ((ret < 0) || (0x0 == chip_id[0]) || (0x0 == chip_id[1])) {
        FTS_ERROR("read boot id fail,read:0x%02x%02x", chip_id[0], chip_id[1]);
        return -EIO;
    }

    id[0] = chip_id[0];
    id[1] = chip_id[1];
    return 0;
}

/*****************************************************************************
* Name: fts_get_ic_information
* Brief: read chip id to get ic information, after run the function, driver w-
*        ill know which IC is it.
*        If cant get the ic information, maybe not focaltech's touch IC, need
*        unregister the driver
* Input:
* Output:
* Return: return 0 if get correct ic information, otherwise return error code
*****************************************************************************/
static int fts_get_ic_information(struct fts_ts_data *ts_data)
{
    int ret = 0;
    int cnt = 0;
    u8 chip_id[2] = { 0 };

    ts_data->ic_info.is_incell = FTS_CHIP_IDC;
    ts_data->ic_info.hid_supported = FTS_HID_SUPPORTTED;


    do {
        ret = fts_read_reg(FTS_REG_CHIP_ID, &chip_id[0]);
        ret = fts_read_reg(FTS_REG_CHIP_ID2, &chip_id[1]);
        if ((ret < 0) || (0x0 == chip_id[0]) || (0x0 == chip_id[1])) {
            FTS_DEBUG("chip id read invalid, read:0x%02x%02x",
                      chip_id[0], chip_id[1]);
        } else {
            ret = fts_get_chip_types(ts_data, chip_id[0], chip_id[1], VALID);
            if (!ret)
                break;
            else
                FTS_DEBUG("TP not ready, read:0x%02x%02x",
                          chip_id[0], chip_id[1]);
        }

        cnt++;
        msleep(INTERVAL_READ_REG);
    } while ((cnt * INTERVAL_READ_REG) < TIMEOUT_READ_REG);

    if ((cnt * INTERVAL_READ_REG) >= TIMEOUT_READ_REG) {
        FTS_INFO("fw is invalid, need read boot id");
        if (ts_data->ic_info.hid_supported) {
            fts_hid2std();
        }


        ret = fts_read_bootid(ts_data, &chip_id[0]);
        if (ret <  0) {
            FTS_ERROR("read boot id fail");
            return ret;
        }

        ret = fts_get_chip_types(ts_data, chip_id[0], chip_id[1], INVALID);
        if (ret < 0) {
            FTS_ERROR("can't get ic informaton");
            return ret;
        }
    }

    FTS_INFO("get ic information, chip id = 0x%02x%02x(cid type=0x%x)",
             ts_data->ic_info.ids.chip_idh, ts_data->ic_info.ids.chip_idl,
             ts_data->ic_info.cid.type);

    return 0;
}

/*****************************************************************************
*  Reprot related
*****************************************************************************/
static void fts_show_touch_buffer(u8 *data, int datalen)
{
    int i = 0;
    int count = 0;
    char *tmpbuf = NULL;

    tmpbuf = kzalloc(1024, GFP_KERNEL);
    if (!tmpbuf) {
        FTS_ERROR("tmpbuf zalloc fail");
        return;
    }

    for (i = 0; i < datalen; i++) {
        count += snprintf(tmpbuf + count, 1024 - count, "%02X,", data[i]);
        if (count >= 1024)
            break;
    }
    FTS_DEBUG("point buffer:%s", tmpbuf);

    if (tmpbuf) {
        kfree(tmpbuf);
        tmpbuf = NULL;
    }
}

void fts_release_all_finger(void)
{
    struct fts_ts_data *ts_data = fts_data;
    struct input_dev *input_dev = ts_data->input_dev;
#if FTS_MT_PROTOCOL_B_EN
    u32 finger_count = 0;
    u32 max_touches = ts_data->pdata->max_touch_number;
#endif

    mutex_lock(&ts_data->report_mutex);
#if FTS_MT_PROTOCOL_B_EN
    for (finger_count = 0; finger_count < max_touches; finger_count++) {
        input_mt_slot(input_dev, finger_count);
        input_mt_report_slot_state(input_dev, MT_TOOL_FINGER, false);
    }
#else
    input_mt_sync(input_dev);
#endif
    input_report_key(input_dev, BTN_TOUCH, 0);
    input_sync(input_dev);

#if FTS_PEN_EN
    input_report_key(ts_data->pen_dev, BTN_TOOL_PEN, 0);
    input_report_key(ts_data->pen_dev, BTN_TOUCH, 0);
    input_sync(ts_data->pen_dev);
#endif

    ts_data->touchs = 0;
    ts_data->key_state = 0;
    mutex_unlock(&ts_data->report_mutex);
}

/*****************************************************************************
* Name: fts_input_report_key
* Brief: process key events,need report key-event if key enable.
*        if point's coordinate is in (x_dim-50,y_dim-50) ~ (x_dim+50,y_dim+50),
*        need report it to key event.
*        x_dim: parse from dts, means key x_coordinate, dimension:+-50
*        y_dim: parse from dts, means key y_coordinate, dimension:+-50
* Input:
* Output:
* Return: return 0 if it's key event, otherwise return error code
*****************************************************************************/
static int fts_input_report_key(struct fts_ts_data *data, int index)
{
    int i = 0;
    int x = data->events[index].x;
    int y = data->events[index].y;
    int *x_dim = &data->pdata->key_x_coords[0];
    int *y_dim = &data->pdata->key_y_coords[0];

    if (!data->pdata->have_key) {
        return -EINVAL;
    }
    for (i = 0; i < data->pdata->key_number; i++) {
        if ((x >= x_dim[i] - FTS_KEY_DIM) && (x <= x_dim[i] + FTS_KEY_DIM) &&
            (y >= y_dim[i] - FTS_KEY_DIM) && (y <= y_dim[i] + FTS_KEY_DIM)) {
            if (EVENT_DOWN(data->events[index].flag)
                && !(data->key_state & (1 << i))) {
                input_report_key(data->input_dev, data->pdata->keys[i], 1);
                data->key_state |= (1 << i);
                FTS_DEBUG("Key%d(%d,%d) DOWN!", i, x, y);
            } else if (EVENT_UP(data->events[index].flag)
                       && (data->key_state & (1 << i))) {
                input_report_key(data->input_dev, data->pdata->keys[i], 0);
                data->key_state &= ~(1 << i);
                FTS_DEBUG("Key%d(%d,%d) Up!", i, x, y);
            }
            return 0;
        }
    }
    return -EINVAL;
}

#if FTS_MT_PROTOCOL_B_EN
static int fts_input_report_b(struct fts_ts_data *data)
{
    int i = 0;
    int uppoint = 0;
    int touchs = 0;
    bool va_reported = false;
    u32 max_touch_num = data->pdata->max_touch_number;
    struct ts_event *events = data->events;

    for (i = 0; i < data->touch_point; i++) {
        if (fts_input_report_key(data, i) == 0) {
            continue;
        }

        va_reported = true;
        input_mt_slot(data->input_dev, events[i].id);

        if (EVENT_DOWN(events[i].flag)) {
            input_mt_report_slot_state(data->input_dev, MT_TOOL_FINGER, true);

#if FTS_REPORT_PRESSURE_EN
            if (events[i].p <= 0) {
                events[i].p = 0x3f;
            }
            input_report_abs(data->input_dev, ABS_MT_PRESSURE, events[i].p);
#endif
            if (events[i].area <= 0) {
                events[i].area = 0x09;
            }
            input_report_abs(data->input_dev, ABS_MT_TOUCH_MAJOR, events[i].area);
            input_report_abs(data->input_dev, ABS_MT_POSITION_X, events[i].x);
            input_report_abs(data->input_dev, ABS_MT_POSITION_Y, events[i].y);

            touchs |= BIT(events[i].id);
            data->touchs |= BIT(events[i].id);

            if ((data->log_level >= 2) ||
                ((1 == data->log_level) && (FTS_TOUCH_DOWN == events[i].flag))) {
                FTS_DEBUG("[B]P%d(%d, %d)[p:%d,tm:%d] DOWN!",
                          events[i].id,
                          events[i].x, events[i].y,
                          events[i].p, events[i].area);
            }
        } else {
            uppoint++;
            input_mt_report_slot_state(data->input_dev, MT_TOOL_FINGER, false);
            data->touchs &= ~BIT(events[i].id);
            if (data->log_level >= 1) {
                FTS_DEBUG("[B]P%d UP!", events[i].id);
            }
        }
    }

    if (unlikely(data->touchs ^ touchs)) {
        for (i = 0; i < max_touch_num; i++)  {
            if (BIT(i) & (data->touchs ^ touchs)) {
                if (data->log_level >= 1) {
                    FTS_DEBUG("[B]P%d UP!", i);
                }
                va_reported = true;
                input_mt_slot(data->input_dev, i);
                input_mt_report_slot_state(data->input_dev, MT_TOOL_FINGER, false);
            }
        }
    }
    data->touchs = touchs;

    if (va_reported) {
        /* touchs==0, there's no point but key */
        if (EVENT_NO_DOWN(data) || (!touchs)) {
            if (data->log_level >= 1) {
                FTS_DEBUG("[B]Points All Up!");
            }
            input_report_key(data->input_dev, BTN_TOUCH, 0);
        } else {
            input_report_key(data->input_dev, BTN_TOUCH, 1);
        }
    }

    input_sync(data->input_dev);
    return 0;
}

#else
static int fts_input_report_a(struct fts_ts_data *data)
{
    int i = 0;
    int touchs = 0;
    bool va_reported = false;
    struct ts_event *events = data->events;

    for (i = 0; i < data->touch_point; i++) {
        if (fts_input_report_key(data, i) == 0) {
            continue;
        }

        va_reported = true;
        if (EVENT_DOWN(events[i].flag)) {
            input_report_abs(data->input_dev, ABS_MT_TRACKING_ID, events[i].id);
#if FTS_REPORT_PRESSURE_EN
            if (events[i].p <= 0) {
                events[i].p = 0x3f;
            }
            input_report_abs(data->input_dev, ABS_MT_PRESSURE, events[i].p);
#endif
            if (events[i].area <= 0) {
                events[i].area = 0x09;
            }
            input_report_abs(data->input_dev, ABS_MT_TOUCH_MAJOR, events[i].area);

            input_report_abs(data->input_dev, ABS_MT_POSITION_X, events[i].x);
            input_report_abs(data->input_dev, ABS_MT_POSITION_Y, events[i].y);

            input_mt_sync(data->input_dev);

            if ((data->log_level >= 2) ||
                ((1 == data->log_level) && (FTS_TOUCH_DOWN == events[i].flag))) {
                FTS_DEBUG("[A]P%d(%d, %d)[p:%d,tm:%d] DOWN!",
                          events[i].id,
                          events[i].x, events[i].y,
                          events[i].p, events[i].area);
            }
            touchs++;
        }
    }

    /* last point down, current no point but key */
    if (data->touchs && !touchs) {
        va_reported = true;
    }
    data->touchs = touchs;

    if (va_reported) {
        if (EVENT_NO_DOWN(data)) {
            if (data->log_level >= 1) {
                FTS_DEBUG("[A]Points All Up!");
            }
            input_report_key(data->input_dev, BTN_TOUCH, 0);
            input_mt_sync(data->input_dev);
        } else {
            input_report_key(data->input_dev, BTN_TOUCH, 1);
        }
    }

    input_sync(data->input_dev);
    return 0;
}
#endif

#if FTS_PEN_EN
static int fts_input_pen_report(struct fts_ts_data *data)
{
    struct input_dev *pen_dev = data->pen_dev;
    struct pen_event *pevt = &data->pevent;
    u8 *buf = data->point_buf;


    if (buf[3] & 0x08)
        input_report_key(pen_dev, BTN_STYLUS, 1);
    else
        input_report_key(pen_dev, BTN_STYLUS, 0);

    if (buf[3] & 0x02)
        input_report_key(pen_dev, BTN_STYLUS2, 1);
    else
        input_report_key(pen_dev, BTN_STYLUS2, 0);

    pevt->inrange = (buf[3] & 0x20) ? 1 : 0;
    pevt->tip = (buf[3] & 0x01) ? 1 : 0;
    pevt->x = ((buf[4] & 0x0F) << 8) + buf[5];
    pevt->y = ((buf[6] & 0x0F) << 8) + buf[7];
    pevt->p = ((buf[8] & 0x0F) << 8) + buf[9];
    pevt->id = buf[6] >> 4;
    pevt->flag = buf[4] >> 6;
    pevt->tilt_x = (buf[10] << 8) + buf[11];
    pevt->tilt_y = (buf[12] << 8) + buf[13];
    pevt->tool_type = BTN_TOOL_PEN;

    if (data->log_level >= 2  ||
        ((1 == data->log_level) && (FTS_TOUCH_DOWN == pevt->flag))) {
        FTS_DEBUG("[PEN]x:%d,y:%d,p:%d,inrange:%d,tip:%d,flag:%d DOWN",
                  pevt->x, pevt->y, pevt->p, pevt->inrange,
                  pevt->tip, pevt->flag);
    }

    if ( (data->log_level >= 1) && (!pevt->inrange)) {
        FTS_DEBUG("[PEN]UP");
    }

    input_report_abs(pen_dev, ABS_X, pevt->x);
    input_report_abs(pen_dev, ABS_Y, pevt->y);
    input_report_abs(pen_dev, ABS_PRESSURE, pevt->p);

    /* check if the pen support tilt event */
    if ((pevt->tilt_x != 0) || (pevt->tilt_y != 0)) {
        input_report_abs(pen_dev, ABS_TILT_X, pevt->tilt_x);
        input_report_abs(pen_dev, ABS_TILT_Y, pevt->tilt_y);
    }

    input_report_key(pen_dev, BTN_TOUCH, pevt->tip);
    input_report_key(pen_dev, BTN_TOOL_PEN, pevt->inrange);
    input_sync(pen_dev);

    return 0;
}
#endif


static int fts_read_touchdata(struct fts_ts_data *data)
{
    int ret = 0;
    int size = 0;
    u8 *buf = data->point_buf;

    memset(buf, 0xFF, data->pnt_buf_size);
    buf[0] = 0x01;

    //if (data->gesture_mode) {
#ifdef CONFIG_TOUCHSCREEN_FTS_GESTURE
    if (data->gesture_en) {
        if (0 == fts_gesture_readdata(data, NULL)) {
            FTS_INFO("succuss to get gesture data in irq handler");
            return 1;
        }
    }
#endif
#if defined(FTS_HIGH_REPORT) && FTS_HIGH_REPORT
    if (data->point_num >= data->pdata->max_touch_number)
        size = data->pnt_buf_size - 1;
    else if (data->point_num > 2)
        size = data->point_num * 6 + 3;
    else
        size = FTS_SIZE_DEFAULT;
#if defined(FTS_PEN_EN) && FTS_PEN_EN
    size = FTS_SIZE_DEFAULT;
#endif

    ret = fts_read(buf, 1, buf + 1, size);
    if (ret < 0) {
        FTS_ERROR("read touchdata failed, ret:%d", ret);
        return ret;
    }

    if ((buf[size] != 0xFF) && (size < (data->pnt_buf_size - 1))
#if defined(FTS_PEN_EN) && FTS_PEN_EN
        && ((buf[2] & 0xF0) != 0xB0)
#endif
       ) {
        buf[0] += size;
        ret = fts_read(buf, 1, buf + 1 + size, data->pnt_buf_size - 1 - size);
        if (ret < 0) {
            FTS_ERROR("read touchdata2 failed, ret:%d", ret);
            return ret;
        }
    }
#else
    size = data->pnt_buf_size - 1;
    ret = fts_read(buf, 1, buf + 1, size);
    if (ret < 0) {
        FTS_ERROR("read touchdata failed, ret:%d", ret);
        return ret;
    }
#endif

    if (data->log_level >= 3) {
        fts_show_touch_buffer(buf, data->pnt_buf_size);
    }

    return 0;
}

static int fts_read_parse_touchdata(struct fts_ts_data *data)
{
    int ret = 0;
    int i = 0;
    u8 pointid = 0;
    int base = 0;
    struct ts_event *events = data->events;
    int max_touch_num = data->pdata->max_touch_number;
    u8 *buf = data->point_buf;

    ret = fts_read_touchdata(data);
    if (ret) {
        return ret;
    }

#if FTS_PEN_EN
    if ((buf[2] & 0xF0) == 0xB0) {
        fts_input_pen_report(data);
        return 2;
    }
#endif

    data->point_num = buf[FTS_TOUCH_POINT_NUM] & 0x0F;
    data->touch_point = 0;

    if (data->ic_info.is_incell) {
        if ((data->point_num == 0x0F) && (buf[2] == 0xFF) && (buf[3] == 0xFF)
            && (buf[4] == 0xFF) && (buf[5] == 0xFF) && (buf[6] == 0xFF)) {
            FTS_DEBUG("touch buff is 0xff, need recovery state");
            fts_release_all_finger();
            fts_tp_state_recovery(data);
            data->point_num = 0;
            return -EIO;
        }
    }

    if (data->point_num > max_touch_num) {
        FTS_INFO("invalid point_num(%d)", data->point_num);
        data->point_num = 0;
        return -EIO;
    }

    for (i = 0; i < max_touch_num; i++) {
        base = FTS_ONE_TCH_LEN * i;
        pointid = (buf[FTS_TOUCH_ID_POS + base]) >> 4;
        if (pointid >= FTS_MAX_ID)
            break;
        else if (pointid >= max_touch_num) {
            FTS_ERROR("ID(%d) beyond max_touch_number", pointid);
            return -EINVAL;
        }

        data->touch_point++;
        events[i].x = ((buf[FTS_TOUCH_X_H_POS + base] & 0x0F) << 8) +
                      (buf[FTS_TOUCH_X_L_POS + base] & 0xFF);
        events[i].y = ((buf[FTS_TOUCH_Y_H_POS + base] & 0x0F) << 8) +
                      (buf[FTS_TOUCH_Y_L_POS + base] & 0xFF);
        events[i].flag = buf[FTS_TOUCH_EVENT_POS + base] >> 6;
        events[i].id = buf[FTS_TOUCH_ID_POS + base] >> 4;
        events[i].area = buf[FTS_TOUCH_AREA_POS + base] >> 4;
        events[i].p =  buf[FTS_TOUCH_PRE_POS + base];

        if (EVENT_DOWN(events[i].flag) && (data->point_num == 0)) {
            FTS_INFO("abnormal touch data from fw");
            return -EIO;
        }
    }

    if (data->touch_point == 0) {
        FTS_INFO("no touch point information(%02x)", buf[2]);
        return -EIO;
    }

    return 0;
}

static void fts_irq_read_report(void)
{
    int ret = 0;
    struct fts_ts_data *ts_data = fts_data;

#if FTS_ESDCHECK_EN
    fts_esdcheck_set_intr(1);
#endif

#if FTS_POINT_REPORT_CHECK_EN
    fts_prc_queue_work(ts_data);
#endif

    ret = fts_read_parse_touchdata(ts_data);
    if (ret == 0) {
        mutex_lock(&ts_data->report_mutex);
#if FTS_MT_PROTOCOL_B_EN
        fts_input_report_b(ts_data);
#else
        fts_input_report_a(ts_data);
#endif
        mutex_unlock(&ts_data->report_mutex);
    }

#if FTS_ESDCHECK_EN
    fts_esdcheck_set_intr(0);
#endif
}

static irqreturn_t fts_irq_handler(int irq, void *data)
{
#if defined(CONFIG_PM) && FTS_PATCH_COMERR_PM
    int ret = 0;
    struct fts_ts_data *ts_data = fts_data;

    if ((ts_data->suspended) && (ts_data->pm_suspend)) {
        ret = wait_for_completion_timeout(
                  &ts_data->pm_completion,
                  msecs_to_jiffies(FTS_TIMEOUT_COMERR_PM));
        if (!ret) {
            FTS_ERROR("Bus don't resume from pm(deep),timeout,skip irq");
            return IRQ_HANDLED;
        }
    }
#endif

    fts_irq_read_report();
    return IRQ_HANDLED;
}

static int fts_irq_registration(struct fts_ts_data *ts_data)
{
    int ret = 0;
    struct fts_ts_platform_data *pdata = ts_data->pdata;

    ts_data->irq = gpio_to_irq(pdata->irq_gpio);
    pdata->irq_gpio_flags = IRQF_TRIGGER_FALLING | IRQF_ONESHOT;
    FTS_INFO("irq:%d, flag:%x", ts_data->irq, pdata->irq_gpio_flags);
    ret = request_threaded_irq(ts_data->irq, NULL, fts_irq_handler,
                               pdata->irq_gpio_flags,
                               FTS_DRIVER_NAME, ts_data);

    return ret;
}

#if FTS_PEN_EN
static int fts_input_pen_init(struct fts_ts_data *ts_data)
{
    int ret = 0;
    struct input_dev *pen_dev;
    struct fts_ts_platform_data *pdata = ts_data->pdata;

    FTS_FUNC_ENTER();
    pen_dev = input_allocate_device();
    if (!pen_dev) {
        FTS_ERROR("Failed to allocate memory for input_pen device");
        return -ENOMEM;
    }

    pen_dev->dev.parent = ts_data->dev;
    pen_dev->name = FTS_DRIVER_PEN_NAME;
    pen_dev->evbit[0] |= BIT_MASK(EV_KEY) | BIT_MASK(EV_ABS);
    __set_bit(ABS_X, pen_dev->absbit);
    __set_bit(ABS_Y, pen_dev->absbit);
    __set_bit(BTN_STYLUS, pen_dev->keybit);
    __set_bit(BTN_STYLUS2, pen_dev->keybit);
    __set_bit(BTN_TOUCH, pen_dev->keybit);
    __set_bit(BTN_TOOL_PEN, pen_dev->keybit);
    __set_bit(INPUT_PROP_DIRECT, pen_dev->propbit);
    input_set_abs_params(pen_dev, ABS_X, pdata->x_min, pdata->x_max, 0, 0);
    input_set_abs_params(pen_dev, ABS_Y, pdata->y_min, pdata->y_max, 0, 0);
    input_set_abs_params(pen_dev, ABS_PRESSURE, 0, 4096, 0, 0);

    ret = input_register_device(pen_dev);
    if (ret) {
        FTS_ERROR("Input device registration failed");
        input_free_device(pen_dev);
        pen_dev = NULL;
        return ret;
    }

    ts_data->pen_dev = pen_dev;
    FTS_FUNC_EXIT();
    return 0;
}
#endif

static int fts_input_init(struct fts_ts_data *ts_data)
{
    int ret = 0;
    int key_num = 0;
    struct fts_ts_platform_data *pdata = ts_data->pdata;
    struct input_dev *input_dev;

    FTS_FUNC_ENTER();
    input_dev = input_allocate_device();
    if (!input_dev) {
        FTS_ERROR("Failed to allocate memory for input device");
        return -ENOMEM;
    }

    /* Init and register Input device */
    input_dev->name = FTS_DRIVER_NAME;
    if (ts_data->bus_type == BUS_TYPE_I2C)
        input_dev->id.bustype = BUS_I2C;
    else
        input_dev->id.bustype = BUS_SPI;
    input_dev->dev.parent = ts_data->dev;

    input_set_drvdata(input_dev, ts_data);

    __set_bit(EV_SYN, input_dev->evbit);
    __set_bit(EV_ABS, input_dev->evbit);
    __set_bit(EV_KEY, input_dev->evbit);
    __set_bit(BTN_TOUCH, input_dev->keybit);
    __set_bit(INPUT_PROP_DIRECT, input_dev->propbit);

    if (pdata->have_key) {
        FTS_INFO("set key capabilities");
        for (key_num = 0; key_num < pdata->key_number; key_num++)
            input_set_capability(input_dev, EV_KEY, pdata->keys[key_num]);
    }

#if FTS_MT_PROTOCOL_B_EN
    input_mt_init_slots(input_dev, pdata->max_touch_number, INPUT_MT_DIRECT);
#else
    input_set_abs_params(input_dev, ABS_MT_TRACKING_ID, 0, 0x0F, 0, 0);
#endif
    input_set_abs_params(input_dev, ABS_MT_POSITION_X, pdata->x_min, pdata->x_max, 0, 0);
    input_set_abs_params(input_dev, ABS_MT_POSITION_Y, pdata->y_min, pdata->y_max, 0, 0);
    input_set_abs_params(input_dev, ABS_MT_TOUCH_MAJOR, 0, 0xFF, 0, 0);
#if FTS_REPORT_PRESSURE_EN
    input_set_abs_params(input_dev, ABS_MT_PRESSURE, 0, 0xFF, 0, 0);
#endif

    ret = input_register_device(input_dev);
    if (ret) {
        FTS_ERROR("Input device registration failed");
        input_set_drvdata(input_dev, NULL);
        input_free_device(input_dev);
        input_dev = NULL;
        return ret;
    }

#if FTS_PEN_EN
    ret = fts_input_pen_init(ts_data);
    if (ret) {
        FTS_ERROR("Input-pen device registration failed");
        input_set_drvdata(input_dev, NULL);
        input_free_device(input_dev);
        input_dev = NULL;
        return ret;
    }
#endif

    ts_data->input_dev = input_dev;
    FTS_FUNC_EXIT();
    return 0;
}

static int fts_report_buffer_init(struct fts_ts_data *ts_data)
{
    int point_num = 0;
    int events_num = 0;


    point_num = ts_data->pdata->max_touch_number;
    ts_data->pnt_buf_size = point_num * FTS_ONE_TCH_LEN + 3;
    ts_data->point_buf = (u8 *)kzalloc(ts_data->pnt_buf_size + 1, GFP_KERNEL);
    if (!ts_data->point_buf) {
        FTS_ERROR("failed to alloc memory for point buf");
        return -ENOMEM;
    }

    events_num = point_num * sizeof(struct ts_event);
    ts_data->events = (struct ts_event *)kzalloc(events_num, GFP_KERNEL);
    if (!ts_data->events) {
        FTS_ERROR("failed to alloc memory for point events");
        kfree_safe(ts_data->point_buf);
        return -ENOMEM;
    }

    return 0;
}

#if FTS_POWER_SOURCE_CUST_EN
/*****************************************************************************
* Power Control
*****************************************************************************/
#if FTS_PINCTRL_EN
static int fts_pinctrl_init(struct fts_ts_data *ts)
{
    int ret = 0;

    ts->pinctrl = devm_pinctrl_get(ts->dev);
    if (IS_ERR_OR_NULL(ts->pinctrl)) {
        FTS_ERROR("Failed to get pinctrl, please check dts");
        ret = PTR_ERR(ts->pinctrl);
        goto err_pinctrl_get;
    }

    ts->pins_active = pinctrl_lookup_state(ts->pinctrl, "pmx_ts_active");
    if (IS_ERR_OR_NULL(ts->pins_active)) {
        FTS_ERROR("Pin state[active] not found");
        ret = PTR_ERR(ts->pins_active);
        goto err_pinctrl_lookup;
    }

    ts->pins_suspend = pinctrl_lookup_state(ts->pinctrl, "pmx_ts_suspend");
    if (IS_ERR_OR_NULL(ts->pins_suspend)) {
        FTS_ERROR("Pin state[suspend] not found");
        ret = PTR_ERR(ts->pins_suspend);
        goto err_pinctrl_lookup;
    }

    ts->pins_release = pinctrl_lookup_state(ts->pinctrl, "pmx_ts_release");
    if (IS_ERR_OR_NULL(ts->pins_release)) {
        FTS_ERROR("Pin state[release] not found");
        ret = PTR_ERR(ts->pins_release);
    }

    return 0;
err_pinctrl_lookup:
    if (ts->pinctrl) {
        devm_pinctrl_put(ts->pinctrl);
    }
err_pinctrl_get:
    ts->pinctrl = NULL;
    ts->pins_release = NULL;
    ts->pins_suspend = NULL;
    ts->pins_active = NULL;
    return ret;
}

static int fts_pinctrl_select_normal(struct fts_ts_data *ts)
{
    int ret = 0;

    if (ts->pinctrl && ts->pins_active) {
        ret = pinctrl_select_state(ts->pinctrl, ts->pins_active);
        if (ret < 0) {
            FTS_ERROR("Set normal pin state error:%d", ret);
        }
    }

    return ret;
}

static int fts_pinctrl_select_suspend(struct fts_ts_data *ts)
{
    int ret = 0;

    if (ts->pinctrl && ts->pins_suspend) {
        ret = pinctrl_select_state(ts->pinctrl, ts->pins_suspend);
        if (ret < 0) {
            FTS_ERROR("Set suspend pin state error:%d", ret);
        }
    }

    return ret;
}

static int fts_pinctrl_select_release(struct fts_ts_data *ts)
{
    int ret = 0;

    if (ts->pinctrl) {
        if (IS_ERR_OR_NULL(ts->pins_release)) {
            devm_pinctrl_put(ts->pinctrl);
            ts->pinctrl = NULL;
        } else {
            ret = pinctrl_select_state(ts->pinctrl, ts->pins_release);
            if (ret < 0)
                FTS_ERROR("Set gesture pin state error:%d", ret);
        }
    }

    return ret;
}
#endif /* FTS_PINCTRL_EN */

static int fts_power_source_ctrl(struct fts_ts_data *ts_data, int enable)
{
    int ret = 0;
	/*
    if (IS_ERR_OR_NULL(ts_data->vdd)) {
        FTS_ERROR("vdd is invalid");
        return -EINVAL;
    }
	*/
    FTS_FUNC_ENTER();
    if (enable) {
        if (ts_data->power_disabled) {
            FTS_DEBUG("regulator enable !");
            gpio_direction_output(ts_data->pdata->reset_gpio, 0);
            msleep(1);
			gpio_direction_output(ts_data->pdata->vdd_gpio, 1);
			msleep(2);
			gpio_direction_output(ts_data->pdata->reset_gpio, 1);
			/*
            ret = regulator_enable(ts_data->vdd);
            if (ret) {
                FTS_ERROR("enable vdd regulator failed,ret=%d", ret);
            }
			*/
            if (!IS_ERR_OR_NULL(ts_data->vcc_i2c)) {
                ret = regulator_enable(ts_data->vcc_i2c);
                if (ret) {
                    FTS_ERROR("enable vcc_i2c regulator failed,ret=%d", ret);
                }
            }
            ts_data->power_disabled = false;
        }
    } else {
        if (!ts_data->power_disabled) {
            FTS_DEBUG("regulator disable !");
            gpio_direction_output(ts_data->pdata->reset_gpio, 0);
            msleep(1);
			gpio_direction_output(ts_data->pdata->vdd_gpio, 0);
			/*
	            ret = regulator_disable(ts_data->vdd);
	            if (ret) {
	                FTS_ERROR("disable vdd regulator failed,ret=%d", ret);
	            }
	            */
            if (!IS_ERR_OR_NULL(ts_data->vcc_i2c)) {
                ret = regulator_disable(ts_data->vcc_i2c);
                if (ret) {
                    FTS_ERROR("disable vcc_i2c regulator failed,ret=%d", ret);
                }
            }
            ts_data->power_disabled = true;
        }
    }

    FTS_FUNC_EXIT();
    return ret;
}

/*****************************************************************************
* Name: fts_power_source_init
* Brief: Init regulator power:vdd/vcc_io(if have), generally, no vcc_io
*        vdd---->vdd-supply in dts, kernel will auto add "-supply" to parse
*        Must be call after fts_gpio_configure() execute,because this function
*        will operate reset-gpio which request gpio in fts_gpio_configure()
* Input:
* Output:
* Return: return 0 if init power successfully, otherwise return error code
*****************************************************************************/
static int fts_power_source_init(struct fts_ts_data *ts_data)
{
    int ret = 0;

    FTS_FUNC_ENTER();
	/*
    ts_data->vdd = regulator_get(ts_data->dev, "vdd");
    if (IS_ERR_OR_NULL(ts_data->vdd)) {
        ret = PTR_ERR(ts_data->vdd);
        FTS_ERROR("get vdd regulator failed,ret=%d", ret);
        return ret;
    }

    if (regulator_count_voltages(ts_data->vdd) > 0) {
        ret = regulator_set_voltage(ts_data->vdd, FTS_VTG_MIN_UV,
                                    FTS_VTG_MAX_UV);
        if (ret) {
            FTS_ERROR("vdd regulator set_vtg failed ret=%d", ret);
            regulator_put(ts_data->vdd);
            return ret;
        }
    }
	*/
    ts_data->vcc_i2c = regulator_get(ts_data->dev, "vcc_i2c");
    if (!IS_ERR_OR_NULL(ts_data->vcc_i2c)) {
        if (regulator_count_voltages(ts_data->vcc_i2c) > 0) {
            ret = regulator_set_voltage(ts_data->vcc_i2c,
                                        FTS_I2C_VTG_MIN_UV,
                                        FTS_I2C_VTG_MAX_UV);
            if (ret) {
                FTS_ERROR("vcc_i2c regulator set_vtg failed,ret=%d", ret);
                regulator_put(ts_data->vcc_i2c);
            }
        }
    }

#if FTS_PINCTRL_EN
    fts_pinctrl_init(ts_data);
    fts_pinctrl_select_normal(ts_data);
#endif

    ts_data->power_disabled = true;
    ret = fts_power_source_ctrl(ts_data, ENABLE);
    if (ret) {
        FTS_ERROR("fail to enable power(regulator)");
    }

    FTS_FUNC_EXIT();
    return ret;
}

static int fts_power_source_exit(struct fts_ts_data *ts_data)
{
#if FTS_PINCTRL_EN
    fts_pinctrl_select_release(ts_data);
#endif

    fts_power_source_ctrl(ts_data, DISABLE);

    if (!IS_ERR_OR_NULL(ts_data->vdd)) {
        if (regulator_count_voltages(ts_data->vdd) > 0)
            regulator_set_voltage(ts_data->vdd, 0, FTS_VTG_MAX_UV);
        regulator_put(ts_data->vdd);
    }

    if (!IS_ERR_OR_NULL(ts_data->vcc_i2c)) {
        if (regulator_count_voltages(ts_data->vcc_i2c) > 0)
            regulator_set_voltage(ts_data->vcc_i2c, 0, FTS_I2C_VTG_MAX_UV);
        regulator_put(ts_data->vcc_i2c);
    }

    return 0;
}

static int fts_power_source_suspend(struct fts_ts_data *ts_data)
{
    int ret = 0;

#if FTS_PINCTRL_EN
    fts_pinctrl_select_suspend(ts_data);
#endif

    ret = fts_power_source_ctrl(ts_data, DISABLE);
    if (ret < 0) {
        FTS_ERROR("power off fail, ret=%d", ret);
    }

    return ret;
}

static int fts_power_source_resume(struct fts_ts_data *ts_data)
{
    int ret = 0;

#if FTS_PINCTRL_EN
    fts_pinctrl_select_normal(ts_data);
#endif

    ret = fts_power_source_ctrl(ts_data, ENABLE);
    if (ret < 0) {
        FTS_ERROR("power on fail, ret=%d", ret);
    }

    return ret;
}
#endif /* FTS_POWER_SOURCE_CUST_EN */

static int fts_gpio_configure(struct fts_ts_data *data)
{
    int ret = 0;

    FTS_FUNC_ENTER();
    /* request irq gpio */
    if (gpio_is_valid(data->pdata->irq_gpio)) {
        ret = gpio_request(data->pdata->irq_gpio, "fts_irq_gpio");
        if (ret) {
            FTS_ERROR("[GPIO]irq gpio request failed");
            goto err_irq_gpio_req;
        }

        ret = gpio_direction_input(data->pdata->irq_gpio);
        if (ret) {
            FTS_ERROR("[GPIO]set_direction for irq gpio failed");
            goto err_irq_gpio_dir;
        }
    }

    /* request reset gpio */
    if (gpio_is_valid(data->pdata->reset_gpio)) {
        ret = gpio_request(data->pdata->reset_gpio, "fts_reset_gpio");
        if (ret) {
            FTS_ERROR("[GPIO]reset gpio request failed");
            goto err_irq_gpio_dir;
        }

        ret = gpio_direction_output(data->pdata->reset_gpio, 0);
        if (ret) {
            FTS_ERROR("[GPIO]set_direction for reset gpio failed");
            goto err_reset_gpio_dir;
        }
    }
    /* request vdd gpio */
    if (gpio_is_valid(data->pdata->vdd_gpio)) {
        ret = gpio_request(data->pdata->vdd_gpio, "fts_vdd_gpio");
        if (ret) {
            FTS_ERROR("[GPIO]reset gpio request failed");
            goto err_reset_gpio_dir;
        }

        ret = gpio_direction_output(data->pdata->vdd_gpio, 1);
        if (ret) {
            FTS_ERROR("[GPIO]set_direction for vdd gpio failed");
            goto err_vdd_gpio_dir;
        }
    }

    FTS_FUNC_EXIT();
    return 0;
err_vdd_gpio_dir:
		if (gpio_is_valid(data->pdata->vdd_gpio))
			gpio_free(data->pdata->vdd_gpio);

err_reset_gpio_dir:
    if (gpio_is_valid(data->pdata->reset_gpio))
        gpio_free(data->pdata->reset_gpio);
err_irq_gpio_dir:
    if (gpio_is_valid(data->pdata->irq_gpio))
        gpio_free(data->pdata->irq_gpio);
err_irq_gpio_req:
    FTS_FUNC_EXIT();
    return ret;
}

static int fts_get_dt_coords(struct device *dev, char *name,
                             struct fts_ts_platform_data *pdata)
{
    int ret = 0;
    u32 coords[FTS_COORDS_ARR_SIZE] = { 0 };
    struct property *prop;
    struct device_node *np = dev->of_node;
    int coords_size;

    prop = of_find_property(np, name, NULL);
    if (!prop)
        return -EINVAL;
    if (!prop->value)
        return -ENODATA;

    coords_size = prop->length / sizeof(u32);
    if (coords_size != FTS_COORDS_ARR_SIZE) {
        FTS_ERROR("invalid:%s, size:%d", name, coords_size);
        return -EINVAL;
    }

    ret = of_property_read_u32_array(np, name, coords, coords_size);
    if (ret < 0) {
        FTS_ERROR("Unable to read %s, please check dts", name);
        pdata->x_min = FTS_X_MIN_DISPLAY_DEFAULT;
        pdata->y_min = FTS_Y_MIN_DISPLAY_DEFAULT;
        pdata->x_max = FTS_X_MAX_DISPLAY_DEFAULT;
        pdata->y_max = FTS_Y_MAX_DISPLAY_DEFAULT;
        return -ENODATA;
    } else {
        pdata->x_min = coords[0];
        pdata->y_min = coords[1];
        pdata->x_max = coords[2];
        pdata->y_max = coords[3];
    }

    FTS_INFO("display x(%d %d) y(%d %d)", pdata->x_min, pdata->x_max,
             pdata->y_min, pdata->y_max);
    return 0;
}

static int factory_get_calibration_ret(struct device *dev)
{
	return 1;
}

static int factory_get_chip_type(struct device *dev, char *buf)
{
	return sprintf(buf, "focaltech\n");
}
static int factory_proc_hibernate_test(struct device *dev)
{
	int err = 0;
	u8 reg_value = 0;
	int i;
	struct fts_ts_data *data = dev_get_drvdata(dev);

	pr_buf_info(" %s enter \n",  __func__);
	gpio_direction_output(data->pdata->reset_gpio, 1);
	mdelay(1);
	gpio_direction_output(data->pdata->reset_gpio, 0);
	mdelay(5);
	FTS_INFO("reset is low.");

	err = fts_read_reg(FTS_REG_FW_VER, &reg_value);
	if (err >= 0) {
		FTS_ERROR("read i2c ok when reset is low,with result=%d\n", err);
		goto out;
	}

	gpio_direction_output(data->pdata->reset_gpio, 1);
	msleep(200);

	err = fts_read_reg(FTS_REG_FW_VER, &reg_value);
	if (err < 0) {
		FTS_ERROR("read i2c ok when reset is high, reset test Fail!\n");
		goto out;
	}
		FTS_ERROR("read i2c ok when reset is high, reset test success!\n");

	/* release all touches */
	for (i = 0; i < data->pdata->max_touch_number; i++) {
		input_mt_slot(data->input_dev, i);
		input_mt_report_slot_state(data->input_dev, MT_TOOL_FINGER, 0);
	}
	input_mt_report_pointer_emulation(data->input_dev, false);
	input_sync(data->input_dev);

	return 1;
out:
	/* release all touches */
	for (i = 0; i < data->pdata->max_touch_number; i++) {
		input_mt_slot(data->input_dev, i);
		input_mt_report_slot_state(data->input_dev, MT_TOOL_FINGER, 0);
	}
	input_mt_report_pointer_emulation(data->input_dev, false);
	input_sync(data->input_dev);

	return 0;
}

static int factory_get_ic_fw_version(struct device *dev, char *buf)
{
	int ret = 0;
	struct fts_ts_data *data = dev_get_drvdata(dev);
	ret = fts_read_reg(FTS_REG_FW_VER, &data->fw_ver[0]);
	FTS_INFO("read chip id-version:0x%02x", data->fw_ver[0]);
	return sprintf(buf, "0x%02X\n", data->fw_ver[0]);
}

static int factory_get_module_id(struct device *dev, char *buf)
{
	struct fts_ts_data *data = dev_get_drvdata(dev);

	u8 reg_value;
	int err = 0;
	err = fts_read_reg(FTS_REG_VENDOR_ID, &reg_value);
    if (err < 0)
        FTS_ERROR("read Pannel Id  from tp fail");
	FTS_INFO("Pannel Id version = 0x%x\n", reg_value);
	data->module_id =  reg_value;

	return snprintf(buf, PAGE_SIZE, "0x%02X\n", data->module_id);
}
#ifdef CONFIG_TOUCHSCREEN_FTS_GLOVE
static bool get_glove_switch(struct device *dev)
{
	struct fts_ts_data *data = dev_get_drvdata(dev);
	
	pr_buf_info("%s: glove_enable = %d\n", __func__, data->glove_enable);
	return data->glove_enable;
}

static int set_glove_switch(struct device *dev, bool enable)
{
	struct fts_ts_data *data = dev_get_drvdata(dev);
	u8 reg_val = enable;
	u8 reg_addr = 0xC0;
	int err = 0;

	data->glove_enable = enable;
	FTS_INFO("glove_enable = %d\n", data->glove_enable);
	err = fts_write_reg(reg_addr, reg_val);
	if (err < 0) {
		FTS_ERROR("write glove reg failed,with result=%d\n", err);
		return -EFAULT;
	}

	return 0;
}
#endif

static bool get_tp_enable_switch(struct device *dev)
{
	struct fts_ts_data *data = dev_get_drvdata(dev);

	return !data->suspended;
}

static int set_tp_enable_switch(struct device *dev, bool enable)
{

	static bool is_the_first_set = 1;
	#if defined(CONFIG_TOUCHSCREEN_FTS_GESTURE)
	static bool gesture_switch;
	struct fts_ts_data *data = dev_get_drvdata(dev);
	#endif

	if (is_the_first_set) {
	#if defined(CONFIG_TOUCHSCREEN_FTS_GESTURE)
		gesture_switch = data->gesture_en;
	#endif
		is_the_first_set = 0;
	}
	FTS_INFO("%d.\n", enable);
	if (enable) {
		fts_ts_resume(dev);
	#if defined(CONFIG_TOUCHSCREEN_FTS_GESTURE)
		data->gesture_en = gesture_switch;
	#endif
		return 0;
	} else {
	#if defined(CONFIG_TOUCHSCREEN_FTS_GESTURE)
		gesture_switch = data->gesture_en;
		data->gesture_en = 0;
	#endif
		return fts_ts_suspend(dev);
	}
}
#ifdef CONFIG_TOUCHSCREEN_WORK_MODE
static int get_tp_work_mode(struct device *dev, char *buf)
{
	u8 reg_val;
	u8 reg_addr = 0xA5;
	int err = 0;
	struct fts_ts_data *data = dev_get_drvdata(dev);

	if(data->suspended)
	{
		return snprintf(buf, PAGE_SIZE, "%s\n", "IDLE");
	} else {
		err = fts_read_reg(reg_addr, &reg_val);
		if (err < 0)
			return snprintf(buf, PAGE_SIZE, "%s\n", "READ MODE ERROR");

		FTS_INFO("read 0xA5 reg value = %#x)\n", reg_val);

		if (reg_val == 0x00)
			return snprintf(buf, PAGE_SIZE, "%s\n", "OPERATING");
		else if (reg_val == 0x01)
			return snprintf(buf, PAGE_SIZE, "%s\n", "MONITOR");
		else
			return snprintf(buf, PAGE_SIZE, "%s\n", "UNKNOWN ERROR");
	}
}

static int set_tp_work_mode(struct device *dev, const char *mode)
{
	u8 reg_addr = 0xA5;
	u8 reg_val;

	u8 reg_val_readback;
	int err = 0;
	struct fts_ts_data *data = dev_get_drvdata(dev);

	if (strncmp(mode, "IDLE", 4)==0) {
		if(!data->suspended)
		set_tp_enable_switch(dev, false);
	} else {
		if (strncmp(mode, "OPERATING", 9)==0)
			reg_val = 0x00;
		else if (strncmp(mode, "MONITOR", 7)==0)
			reg_val = 0x01;
		else
			return -EFAULT;

		if(data->suspended)
			set_tp_enable_switch(dev, true);

		err = fts_write_reg(reg_addr, reg_val);
		if (err < 0) {
			FTS_ERROR("write reg failed, [err]=%d\n", err);
			return -EFAULT;
		}

		err = fts_read_reg(reg_addr, &reg_val_readback);
		if (err < 0 || reg_val_readback!=reg_val) {
			FTS_ERROR("set operating mode failed, write val==%d, read back val==%d, [read err]=%d, []=\n", 
				reg_val, reg_val_readback, err);
			return -EFAULT;
		}
	}
	FTS_INFO("tp set to %s work mode\n", mode);

	return 0;
}
#endif/*CONFIG_TOUCHSCREEN_WORK_MODE*/
static	int factory_get_fw_update_progress(struct device *dev)
{
	struct fts_ts_data *data = dev_get_drvdata(dev);

	return data->fw_loading ? FW_IS_UPDATETING : FW_UPGRADE_SUCCESS;
}
#if defined(CONFIG_TOUCHSCREEN_FTS_GESTURE)
static unsigned int asic_to_hex(unsigned char val)
{
	if ((val >= '0') && (val <= '9'))
		val -= '0';
	else if ((val >= 'a') && (val <= 'z'))
		val = val - 'a' + 10;
	else if ((val >= 'A') && (val <= 'Z'))
		val = val - 'A' + 10;

	return (unsigned int)val;
}

static int set_gesture_switch(struct device *dev, const char *buf)
{
	struct fts_ts_data *data = dev_get_drvdata(dev);
	unsigned char gesture[10], len;

	strlcpy(gesture, buf, sizeof(gesture));
	len = strlen(gesture);
	if (len > 0)
		if ((gesture[len-1] == '\n') || (gesture[len-1] == '\0'))
			len--;

	FTS_INFO("%s len: %d gtp_state: %d,%d,%d.\n",
					__func__, len, gesture[0], gesture[1], gesture[2]);
	if (len == 1) {
		if (gesture[0] == '1')
			data->gesture_state = 0xffff;
		else if (gesture[0] == '0')
			data->gesture_state = 0x0;
	} else if (len == 4) {
		data->gesture_state = asic_to_hex(gesture[0])*0x1000
						+ asic_to_hex(gesture[1]) * 0x100
						+ asic_to_hex(gesture[2]) * 0x10
						+ asic_to_hex(gesture[3]);
	} else {
		FTS_INFO("[set_gesture_switch]write wrong cmd.");
		return 0;
	}
	if (!data->gesture_state)
		data->gesture_en = false;
	else
		data->gesture_en = true;

	FTS_INFO("%s is %x.\n", __func__, data->gesture_state);

	return 0;
}

static bool get_gesture_switch(struct device *dev)
{
	struct fts_ts_data *data = dev_get_drvdata(dev);

	return data->gesture_en;
}

static int get_gesture_pos(struct device *dev, char *buf)
{
	struct fts_ts_data *data = dev_get_drvdata(dev);
	char temp[20] = {0};
	char gesture_pos[512] = {0};
	int i = 0;

	for (i = 0; i < data->gesture_track_pointnum; i++) {
		snprintf(temp, PAGE_SIZE, "%u,%u;", (unsigned int)data->gesture_track_x[i], (unsigned int)data->gesture_track_y[i]);
		strlcat(gesture_pos, temp, 512);
	}

	return snprintf(buf, PAGE_SIZE, "%s\n", gesture_pos);
}
#endif
static int factory_ts_func_test_register(struct fts_ts_data* data)
{
	ts_gen_func_test_init();
	data->ts_test_dev.dev = &data->client->dev;
	data->ts_test_dev.get_chip_type = factory_get_chip_type;
	data->ts_test_dev.get_calibration_ret = factory_get_calibration_ret;
	data->ts_test_dev.get_module_id = factory_get_module_id;
	data->ts_test_dev.proc_hibernate_test = factory_proc_hibernate_test;
	data->ts_test_dev.get_ic_fw_version = factory_get_ic_fw_version;

	data->ts_test_dev.get_fs_fw_version = ft5526_get_fs_fw_version;
	data->ts_test_dev.proc_fw_update = ft5526_proc_fw_update;
	data->ts_test_dev.check_fw_update_need = ft5526_check_fw_update_need;
	data->ts_test_dev.get_fw_update_progress = factory_get_fw_update_progress;
	data->ts_test_dev.get_tp_enable_switch = get_tp_enable_switch;
	data->ts_test_dev.set_tp_enable_switch = set_tp_enable_switch;
#ifdef CONFIG_TOUCHSCREEN_FTS_GLOVE
	data->ts_test_dev.get_glove_switch = get_glove_switch;
	data->ts_test_dev.set_glove_switch = set_glove_switch;
#endif
#ifdef CONFIG_TOUCHSCREEN_WORK_MODE
	data->ts_test_dev.get_tp_work_mode = get_tp_work_mode;
	data->ts_test_dev.set_tp_work_mode = set_tp_work_mode;
#endif
#if defined(CONFIG_TOUCHSCREEN_FTS_GESTURE)
	data->ts_test_dev.get_gesture_switch = get_gesture_switch;
	data->ts_test_dev.set_gesture_switch = set_gesture_switch;
	data->ts_test_dev.get_gesture_pos = get_gesture_pos;
#endif
	register_ts_func_test_device(&data->ts_test_dev);
	return 0;
}
void ft5526_ts_register_productinfo(struct fts_ts_data *ts_data)
{
	/* format as flow: version:0x01 Module id:0x57 */
	char deviceinfo[64];
	u8 reg_value;
	int err = 0;
	err = fts_read_reg(FTS_REG_FW_VER, &ts_data->fw_ver[0]);
	if (err < 0)
        FTS_ERROR("read FW version  from tp fail");
	err = fts_read_reg(FTS_REG_VENDOR_ID, &reg_value);
    if (err < 0)
        FTS_ERROR("read Pannel Id  from tp fail");
	ts_data->module_id =  reg_value;

	snprintf(deviceinfo, ARRAY_SIZE(deviceinfo), "FW version:0x%2x Module id:0x%2x",
			ts_data->fw_ver[0], ts_data->module_id);

	productinfo_register(PRODUCTINFO_CTP_ID, NULL, deviceinfo);

}
#if defined(CONFIG_TOUCHSCREEN_FTS_FH)
int ft5526_ctp_work_with_ac_usb_plugin(int plugin)
{
	struct fts_ts_data *ts_data = fts_data;

	FTS_FUNC_ENTER();

	if ((fts_data == NULL) || (!fts_probed))
		return -ENODEV;
	if ((!ts_data->suspended) && ts_data->client) {
       if (0 == plugin)
			printk("USB is plugged Out(%d,%s)\n", __LINE__, __func__);
       else
			printk("USB is plugged In(%d,%s)\n", __LINE__, __func__);
       fts_write_reg(0x8B, plugin);
       }
	return 0;

}
EXPORT_SYMBOL_GPL(ft5526_ctp_work_with_ac_usb_plugin);
#endif

static int fts_parse_dt(struct device *dev, struct fts_ts_data *data)
{
    int ret = 0;
    struct device_node *np = dev->of_node;
    u32 temp_val = 0;
	struct fts_ts_platform_data *pdata = data->pdata;
#if defined(CONFIG_TOUCHSCREEN_FTS_GESTURE)
	struct property *prop;
	int i = 0;
	u32 gesture_map[MAX_GESTURE];
	u32 num_buttons = 0;
#endif

    FTS_FUNC_ENTER();
#if defined(CONFIG_TOUCHSCREEN_FTS_GESTURE)
	prop = of_find_property(np, "focaltech,gesture-func-map", NULL);
	if (prop) {
		num_buttons = prop->length / sizeof(temp_val);
		if (num_buttons > MAX_GESTURE)
			return -EINVAL;
		ret = of_property_read_u32_array(np,
			"focaltech,gesture-func-map", gesture_map, num_buttons);
		if (ret) {
			FTS_ERROR("Unable to read gesture func map\n");
			return ret;
		}
		for (i = 0; i < num_buttons; i++)
			data->gesture_func_map[i] = gesture_map[i];
		data->gesture_num = num_buttons;
	}
	prop = of_find_property(np, "focaltech,gesture-figure-map", NULL);
	if (prop) {
		ret = of_property_read_u32_array(np, "focaltech,gesture-figure-map", gesture_map,
			num_buttons);
		if (ret) {
			FTS_ERROR("Unable to read gesture figure map\n");
			return ret;
		}
		for (i = 0; i < num_buttons; i++){
			data->gesture_figure_map[i] = gesture_map[i];
		}
	}
#endif
    ret = fts_get_dt_coords(dev, "focaltech,display-coords", pdata);
    if (ret < 0)
        FTS_ERROR("Unable to get display-coords");

    /* key */
    pdata->have_key = of_property_read_bool(np, "focaltech,have-key");
    if (pdata->have_key) {
        ret = of_property_read_u32(np, "focaltech,key-number", &pdata->key_number);
        if (ret < 0)
            FTS_ERROR("Key number undefined!");

        ret = of_property_read_u32_array(np, "focaltech,keys",
                                         pdata->keys, pdata->key_number);
        if (ret < 0)
            FTS_ERROR("Keys undefined!");
        else if (pdata->key_number > FTS_MAX_KEYS)
            pdata->key_number = FTS_MAX_KEYS;

        ret = of_property_read_u32_array(np, "focaltech,key-x-coords",
                                         pdata->key_x_coords,
                                         pdata->key_number);
        if (ret < 0)
            FTS_ERROR("Key Y Coords undefined!");

        ret = of_property_read_u32_array(np, "focaltech,key-y-coords",
                                         pdata->key_y_coords,
                                         pdata->key_number);
        if (ret < 0)
            FTS_ERROR("Key X Coords undefined!");

        FTS_INFO("VK Number:%d, key:(%d,%d,%d), "
                 "coords:(%d,%d),(%d,%d),(%d,%d)",
                 pdata->key_number,
                 pdata->keys[0], pdata->keys[1], pdata->keys[2],
                 pdata->key_x_coords[0], pdata->key_y_coords[0],
                 pdata->key_x_coords[1], pdata->key_y_coords[1],
                 pdata->key_x_coords[2], pdata->key_y_coords[2]);
    }

    /* reset, irq gpio info */
	   pdata->vdd_gpio = of_get_named_gpio_flags(np, "focaltech,vdd-gpio",
                        0, &pdata->vdd_gpio_flags);
    if (pdata->vdd_gpio < 0)
        FTS_ERROR("Unable to get vdd_gpio");
    pdata->reset_gpio = of_get_named_gpio_flags(np, "focaltech,reset-gpio",
                        0, &pdata->reset_gpio_flags);
    if (pdata->reset_gpio < 0)
        FTS_ERROR("Unable to get reset_gpio");

    pdata->irq_gpio = of_get_named_gpio_flags(np, "focaltech,irq-gpio",
                      0, &pdata->irq_gpio_flags);
    if (pdata->irq_gpio < 0)
        FTS_ERROR("Unable to get irq_gpio");

    ret = of_property_read_u32(np, "focaltech,max-touch-number", &temp_val);
    if (ret < 0) {
        FTS_ERROR("Unable to get max-touch-number, please check dts");
        pdata->max_touch_number = FTS_MAX_POINTS_SUPPORT;
    } else {
        if (temp_val < 2)
            pdata->max_touch_number = 2; /* max_touch_number must >= 2 */
        else if (temp_val > FTS_MAX_POINTS_SUPPORT)
            pdata->max_touch_number = FTS_MAX_POINTS_SUPPORT;
        else
            pdata->max_touch_number = temp_val;
    }

    FTS_INFO("max touch number:%d, irq gpio:%d, reset gpio:%d",
             pdata->max_touch_number, pdata->irq_gpio, pdata->reset_gpio);
	pdata->fw_name_pref = "ft_fw.bin";
	ret = of_property_read_string(np, "focaltech,fw-name", &pdata->fw_name_pref);
	if (ret && (ret != -EINVAL)) {
		printk(KERN_ERR "ft5x06 Unable to read fw name\n");
	}
	pdata->name = "ft5xx6";
	ret = of_property_read_string(np, "focaltech,factory-info", &pdata->factory_info);
	if (ret < 0) {
		FTS_ERROR("ft5x06 factory info read failed!");
		pdata->factory_info = "NULL";
	}

    FTS_FUNC_EXIT();
    return 0;
}

static void fts_resume_work(struct work_struct *work)
{
    struct fts_ts_data *ts_data = container_of(work, struct fts_ts_data,
                                  resume_work);

    fts_ts_resume(ts_data->dev);
}

#if defined(CONFIG_DRM)
#if 0//defined(CONFIG_DRM_PANEL)
static struct drm_panel *active_panel;

static int drm_check_dt(struct device_node *np)
{
    int i = 0;
    int count = 0;
    struct device_node *node = NULL;
    struct drm_panel *panel = NULL;

    count = of_count_phandle_with_args(np, "panel", NULL);
    if (count <= 0) {
        FTS_ERROR("find drm_panel count(%d) fail", count);
        return -ENODEV;
    }

    for (i = 0; i < count; i++) {
        node = of_parse_phandle(np, "panel", i);
        panel = of_drm_find_panel(node);
        of_node_put(node);
        if (!IS_ERR(panel)) {
            FTS_INFO("find drm_panel successfully");
            active_panel = panel;
            return 0;
        }
    }

    FTS_ERROR("no find drm_panel");
    return -ENODEV;
}

static int drm_notifier_callback(struct notifier_block *self,
                                 unsigned long event, void *data)
{
    struct msm_drm_notifier *evdata = data;
    int *blank = NULL;
    struct fts_ts_data *ts_data = container_of(self, struct fts_ts_data,
                                  fb_notif);

    if (!evdata) {
        FTS_ERROR("evdata is null");
        return 0;
    }

    if (!((event == DRM_PANEL_EARLY_EVENT_BLANK )
          || (event == DRM_PANEL_EVENT_BLANK))) {
        FTS_INFO("event(%lu) do not need process\n", event);
        return 0;
    }

    blank = evdata->data;
    FTS_INFO("DRM event:%lu,blank:%d", event, *blank);
    switch (*blank) {
    case DRM_PANEL_BLANK_UNBLANK:
        if (DRM_PANEL_EARLY_EVENT_BLANK == event) {
            FTS_INFO("resume: event = %lu, not care\n", event);
        } else if (DRM_PANEL_EVENT_BLANK == event) {
            queue_work(fts_data->ts_workqueue, &fts_data->resume_work);
        }
        break;
    case DRM_PANEL_BLANK_POWERDOWN:
        if (DRM_PANEL_EARLY_EVENT_BLANK == event) {
            cancel_work_sync(&fts_data->resume_work);
            fts_ts_suspend(ts_data->dev);
        } else if (DRM_PANEL_EVENT_BLANK == event) {
            FTS_INFO("suspend: event = %lu, not care\n", event);
        }
        break;
    default:
        FTS_INFO("DRM BLANK(%d) do not need process\n", *blank);
        break;
    }

    return 0;
}
#else
static int drm_notifier_callback(struct notifier_block *self,
                                 unsigned long event, void *data)
{
    struct msm_drm_notifier *evdata = data;
    int *blank = NULL;
    struct fts_ts_data *ts_data = container_of(self, struct fts_ts_data,
                                  fb_notif);

    if (!evdata) {
        FTS_ERROR("evdata is null");
        return 0;
    }

    if (!((event == MSM_DRM_EARLY_EVENT_BLANK )
          || (event == MSM_DRM_EVENT_BLANK))) {
        FTS_INFO("event(%lu) do not need process\n", event);
        return 0;
    }

    blank = evdata->data;
    FTS_INFO("DRM event:%lu,blank:%d", event, *blank);
    switch (*blank) {
    case MSM_DRM_BLANK_UNBLANK:
        if (MSM_DRM_EARLY_EVENT_BLANK == event) {
            FTS_INFO("resume: event = %lu, not care\n", event);
        } else if (MSM_DRM_EVENT_BLANK == event) {
            queue_work(fts_data->ts_workqueue, &fts_data->resume_work);
        }
        break;
    case MSM_DRM_BLANK_POWERDOWN:
        if (MSM_DRM_EARLY_EVENT_BLANK == event) {
            cancel_work_sync(&fts_data->resume_work);
            fts_ts_suspend(ts_data->dev);
        } else if (MSM_DRM_EVENT_BLANK == event) {
            FTS_INFO("suspend: event = %lu, not care\n", event);
        }
        break;
    default:
        FTS_INFO("DRM BLANK(%d) do not need process\n", *blank);
        break;
    }

    return 0;
}
#endif
#elif defined(CONFIG_FB)
static int fb_notifier_callback(struct notifier_block *self,
                                unsigned long event, void *data)
{
    struct fb_event *evdata = data;
    int *blank = NULL;
    struct fts_ts_data *ts_data = container_of(self, struct fts_ts_data,
                                  fb_notif);

    if (!evdata) {
        FTS_ERROR("evdata is null");
        return 0;
    }

    if (!(event == FB_EARLY_EVENT_BLANK || event == FB_EVENT_BLANK)) {
        FTS_INFO("event(%lu) do not need process\n", event);
        return 0;
    }

    blank = evdata->data;
    FTS_INFO("FB event:%lu,blank:%d", event, *blank);
    switch (*blank) {
    case FB_BLANK_UNBLANK:
        if (FB_EARLY_EVENT_BLANK == event) {
            FTS_INFO("resume: event = %lu, not care\n", event);
        } else if (FB_EVENT_BLANK == event) {
            queue_work(fts_data->ts_workqueue, &fts_data->resume_work);
        }
        break;
    case FB_BLANK_POWERDOWN:
        if (FB_EARLY_EVENT_BLANK == event) {
            cancel_work_sync(&fts_data->resume_work);
            fts_ts_suspend(ts_data->dev);
        } else if (FB_EVENT_BLANK == event) {
            FTS_INFO("suspend: event = %lu, not care\n", event);
        }
        break;
    default:
        FTS_INFO("FB BLANK(%d) do not need process\n", *blank);
        break;
    }

    return 0;
}
#elif defined(CONFIG_HAS_EARLYSUSPEND)
static void fts_ts_early_suspend(struct early_suspend *handler)
{
    struct fts_ts_data *ts_data = container_of(handler, struct fts_ts_data,
                                  early_suspend);

    cancel_work_sync(&fts_data->resume_work);
    fts_ts_suspend(ts_data->dev);
}

static void fts_ts_late_resume(struct early_suspend *handler)
{
    struct fts_ts_data *ts_data = container_of(handler, struct fts_ts_data,
                                  early_suspend);

    queue_work(fts_data->ts_workqueue, &fts_data->resume_work);
}
#endif

static int fts_ts_probe_entry(struct fts_ts_data *ts_data)
{
    int ret = 0;
    int pdata_size = sizeof(struct fts_ts_platform_data);

    FTS_FUNC_ENTER();
    FTS_INFO("%s", FTS_DRIVER_VERSION);
    ts_data->pdata = kzalloc(pdata_size, GFP_KERNEL);
    if (!ts_data->pdata) {
        FTS_ERROR("allocate memory for platform_data fail");
        return -ENOMEM;
    }

    if (ts_data->dev->of_node) {
        ret = fts_parse_dt(ts_data->dev, ts_data);
        if (ret)
            FTS_ERROR("device-tree parse fail");
#if defined(CONFIG_DRM)
#if 0//defined(CONFIG_DRM_PANEL)
        ret = drm_check_dt(ts_data->dev->of_node);
        if (ret) {
            FTS_ERROR("parse drm-panel fail");
        }
#endif
#elif defined(CONFIG_FB)
	FTS_INFO("config_fb.");
#endif
    } else {
        if (ts_data->dev->platform_data) {
            memcpy(ts_data->pdata, ts_data->dev->platform_data, pdata_size);
        } else {
            FTS_ERROR("platform_data is null");
            return -ENODEV;
        }
    }

    ts_data->ts_workqueue = create_singlethread_workqueue("fts_wq");
    if (!ts_data->ts_workqueue) {
        FTS_ERROR("create fts workqueue fail");
    }

    spin_lock_init(&ts_data->irq_lock);
    mutex_init(&ts_data->report_mutex);
    mutex_init(&ts_data->bus_lock);

    /* Init communication interface */
    ret = fts_bus_init(ts_data);
    if (ret) {
        FTS_ERROR("bus initialize fail");
        goto err_bus_init;
    }

    ret = fts_input_init(ts_data);
    if (ret) {
        FTS_ERROR("input initialize fail");
        goto err_input_init;
    }

    ret = fts_report_buffer_init(ts_data);
    if (ret) {
        FTS_ERROR("report buffer init fail");
        goto err_report_buffer;
    }

    ret = fts_gpio_configure(ts_data);
    if (ret) {
        FTS_ERROR("configure the gpios fail");
        goto err_gpio_config;
    }

#if FTS_POWER_SOURCE_CUST_EN
    ret = fts_power_source_init(ts_data);
    if (ret) {
        FTS_ERROR("fail to get power(regulator)");
        goto err_power_init;
    }
#endif

#if (!FTS_CHIP_IDC)
    fts_reset_proc(200);
#endif

    ret = fts_get_ic_information(ts_data);
    if (ret) {
        FTS_ERROR("not focal IC, unregister driver");
        goto err_irq_req;
    }

    ret = fts_create_apk_debug_channel(ts_data);
    if (ret) {
        FTS_ERROR("create apk debug node fail");
    }

    ret = fts_create_sysfs(ts_data);
    if (ret) {
        FTS_ERROR("create sysfs node fail");
    }

#if FTS_POINT_REPORT_CHECK_EN
    ret = fts_point_report_check_init(ts_data);
    if (ret) {
        FTS_ERROR("init point report check fail");
    }
#endif

    ret = fts_ex_mode_init(ts_data);
    if (ret) {
        FTS_ERROR("init glove/cover/charger fail");
    }
#ifdef CONFIG_TOUCHSCREEN_FTS_GESTURE
    ret = fts_gesture_init(ts_data);
    if (ret) {
        FTS_ERROR("init gesture fail");
    }
#endif

#if FTS_ESDCHECK_EN
    ret = fts_esdcheck_init(ts_data);
    if (ret) {
        FTS_ERROR("init esd check fail");
    }
#endif

    ret = fts_irq_registration(ts_data);
    if (ret) {
        FTS_ERROR("request irq failed");
        goto err_irq_req;
    }

    ret = fts_fwupg_init(ts_data);
    if (ret) {
        FTS_ERROR("init fw upgrade fail");
    }

    if (ts_data->ts_workqueue) {
        INIT_WORK(&ts_data->resume_work, fts_resume_work);
    }

#if defined(CONFIG_PM) && FTS_PATCH_COMERR_PM
    init_completion(&ts_data->pm_completion);
    ts_data->pm_suspend = false;
#endif

#if defined(CONFIG_DRM)
    ts_data->fb_notif.notifier_call = drm_notifier_callback;
#if 0//defined(CONFIG_DRM_PANEL)
    if (active_panel) {
        ret = drm_panel_notifier_register(active_panel, &ts_data->fb_notif);
        if (ret)
            FTS_ERROR("[DRM]drm_panel_notifier_register fail: %d\n", ret);
    }
#else
    ret = msm_drm_register_client(&ts_data->fb_notif);
    if (ret) {
        FTS_ERROR("[DRM]Unable to register fb_notifier: %d\n", ret);
    }
#endif
#elif defined(CONFIG_FB)
    ts_data->fb_notif.notifier_call = fb_notifier_callback;
    ret = fb_register_client(&ts_data->fb_notif);
    if (ret) {
        FTS_ERROR("[FB]Unable to register fb_notifier: %d", ret);
    }
#elif defined(CONFIG_HAS_EARLYSUSPEND)
    ts_data->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + FTS_SUSPEND_LEVEL;
    ts_data->early_suspend.suspend = fts_ts_early_suspend;
    ts_data->early_suspend.resume = fts_ts_late_resume;
    register_early_suspend(&ts_data->early_suspend);
#endif

    FTS_FUNC_EXIT();
    return 0;

err_irq_req:
#if FTS_POWER_SOURCE_CUST_EN
err_power_init:
    fts_power_source_exit(ts_data);
#endif
    if (gpio_is_valid(ts_data->pdata->reset_gpio))
        gpio_free(ts_data->pdata->reset_gpio);
    if (gpio_is_valid(ts_data->pdata->irq_gpio))
        gpio_free(ts_data->pdata->irq_gpio);
err_gpio_config:
    kfree_safe(ts_data->point_buf);
    kfree_safe(ts_data->events);
err_report_buffer:
    input_unregister_device(ts_data->input_dev);
#if FTS_PEN_EN
    input_unregister_device(ts_data->pen_dev);
#endif
err_input_init:
    if (ts_data->ts_workqueue)
        destroy_workqueue(ts_data->ts_workqueue);
err_bus_init:
    kfree_safe(ts_data->bus_tx_buf);
    kfree_safe(ts_data->bus_rx_buf);
    kfree_safe(ts_data->pdata);

    FTS_FUNC_EXIT();
    return ret;
}

static int fts_ts_remove_entry(struct fts_ts_data *ts_data)
{
    FTS_FUNC_ENTER();

#if FTS_POINT_REPORT_CHECK_EN
    fts_point_report_check_exit(ts_data);
#endif

    fts_release_apk_debug_channel(ts_data);
    fts_remove_sysfs(ts_data);
    fts_ex_mode_exit(ts_data);

    fts_fwupg_exit(ts_data);


#if FTS_ESDCHECK_EN
    fts_esdcheck_exit(ts_data);
#endif

#ifdef CONFIG_TOUCHSCREEN_FTS_GESTURE
    fts_gesture_exit(ts_data);
#endif/*CONFIG_TOUCHSCREEN_FTS_GESTURE*/
    fts_bus_exit(ts_data);

    free_irq(ts_data->irq, ts_data);
    input_unregister_device(ts_data->input_dev);
#if FTS_PEN_EN
    input_unregister_device(ts_data->pen_dev);
#endif

    if (ts_data->ts_workqueue)
        destroy_workqueue(ts_data->ts_workqueue);

#if defined(CONFIG_DRM)
#if 0//defined(CONFIG_DRM_PANEL)
    if (active_panel)
        drm_panel_notifier_unregister(active_panel, &ts_data->fb_notif);
#else
    if (msm_drm_unregister_client(&ts_data->fb_notif))
        FTS_ERROR("[DRM]Error occurred while unregistering fb_notifier.\n");
#endif
#elif defined(CONFIG_FB)
    if (fb_unregister_client(&ts_data->fb_notif))
        FTS_ERROR("[FB]Error occurred while unregistering fb_notifier.");
#elif defined(CONFIG_HAS_EARLYSUSPEND)
    unregister_early_suspend(&ts_data->early_suspend);
#endif

    if (gpio_is_valid(ts_data->pdata->reset_gpio))
        gpio_free(ts_data->pdata->reset_gpio);

    if (gpio_is_valid(ts_data->pdata->irq_gpio))
        gpio_free(ts_data->pdata->irq_gpio);

#if FTS_POWER_SOURCE_CUST_EN
    fts_power_source_exit(ts_data);
#endif

    kfree_safe(ts_data->point_buf);
    kfree_safe(ts_data->events);

    kfree_safe(ts_data->pdata);
    kfree_safe(ts_data);

    FTS_FUNC_EXIT();

    return 0;
}

static int fts_ts_suspend(struct device *dev)
{
    int ret = 0;
    struct fts_ts_data *ts_data = fts_data;

    FTS_FUNC_ENTER();
    if (ts_data->suspended) {
        FTS_INFO("Already in suspend state");
        return 0;
    }

    if (ts_data->fw_loading) {
        FTS_INFO("fw upgrade in process, can't suspend");
        return 0;
    }

#if FTS_ESDCHECK_EN
    fts_esdcheck_suspend();
#endif

    //if (ts_data->gesture_mode) {
#ifdef CONFIG_TOUCHSCREEN_FTS_GESTURE
	if(ts_data->gesture_en){
        if( 0 == fts_gesture_suspend(ts_data)){
			fts_release_all_finger();
			ts_data->suspended = true;
			FTS_FUNC_EXIT();
			return 0;
		}
    } 
#endif /*CONFIG_TOUCHSCREEN_FTS_GESTURE*/
	//else {
    fts_irq_disable();

    FTS_INFO("make TP enter into sleep mode");
    ret = fts_write_reg(FTS_REG_POWER_MODE, FTS_REG_POWER_MODE_SLEEP);
    if (ret < 0)
        FTS_ERROR("set TP to sleep mode fail, ret=%d", ret);

    if (!ts_data->ic_info.is_incell) {
#if FTS_POWER_SOURCE_CUST_EN
        ret = fts_power_source_suspend(ts_data);
        if (ret < 0) {
            FTS_ERROR("power enter suspend fail");
        }
#endif
    }
    //}

    fts_release_all_finger();
    ts_data->suspended = true;
    FTS_FUNC_EXIT();
    return 0;
}

static int fts_ts_resume(struct device *dev)
{
    struct fts_ts_data *ts_data = fts_data;
#ifdef CONFIG_TOUCHSCREEN_FTS_GLOVE
	int err;
#endif
    FTS_FUNC_ENTER();
    if (!ts_data->suspended) {
        FTS_DEBUG("Already in awake state");
        return 0;
    }

    fts_release_all_finger();

    if (!ts_data->ic_info.is_incell) {
#if FTS_POWER_SOURCE_CUST_EN
        fts_power_source_resume(ts_data);
#endif
        fts_reset_proc(200);
    }

    fts_wait_tp_to_valid();
    fts_ex_mode_recovery(ts_data);

#if FTS_ESDCHECK_EN
    fts_esdcheck_resume();
#endif

//    if (ts_data->gesture_mode) {
#ifdef CONFIG_TOUCHSCREEN_FTS_GESTURE
	if(ts_data->gesture_en)
        fts_gesture_resume(ts_data);
#endif /*CONFIG_TOUCHSCREEN_FTS_GESTURE*/
//    } else {
	fts_irq_enable();
//    }

    ts_data->suspended = false;
#ifdef CONFIG_TOUCHSCREEN_FTS_GLOVE
	if (ts_data->glove_enable) {
		err = fts_write_reg(0xC0, 1);
		if (err < 0) {
			FTS_ERROR("write glove reg failed,with result=%d\n", err);
			return -EFAULT;
		}
	}
#endif
#if defined(CONFIG_TOUCHSCREEN_FTS_FH)
	if (usb_flag)
		ft5526_ctp_work_with_ac_usb_plugin(true);
	else
		ft5526_ctp_work_with_ac_usb_plugin(false);
#endif

    FTS_FUNC_EXIT();
    return 0;
}

#if defined(CONFIG_PM)
#if defined(CONFIG_TOUCHSCREEN_FTS_GESTURE)
static int fts_ts_suspend_gesture(struct device *dev)
{
    struct fts_ts_data *ts_data = dev_get_drvdata(dev);

	if (ts_data->gesture_en)
	{
		FTS_INFO("fts_ts_suspend_gesture\n");
		disable_irq(ts_data->client->irq);
		/* make tp can wake the system */
		enable_irq_wake(ts_data->client->irq);
	}
	return 0;
}
static int fts_ts_resume_gesture(struct device *dev)
	{
    struct fts_ts_data *ts_data = dev_get_drvdata(dev);

	if (ts_data->gesture_en)
	{
		pr_info("fts_ts_resume_gesture\n");
		/* make tp cannot wake the system */
		disable_irq_wake(ts_data->client->irq);
		enable_irq(ts_data->client->irq);
	}
	return 0;
}
#endif
#if FTS_PATCH_COMERR_PM
static int fts_pm_suspend(struct device *dev)
{
    struct fts_ts_data *ts_data = dev_get_drvdata(dev);

    FTS_INFO("system enters into pm_suspend");
    ts_data->pm_suspend = true;
    reinit_completion(&ts_data->pm_completion);
    return 0;
}

static int fts_pm_resume(struct device *dev)
{
    struct fts_ts_data *ts_data = dev_get_drvdata(dev);

    FTS_INFO("system resumes from pm_suspend");
    ts_data->pm_suspend = false;
    complete(&ts_data->pm_completion);
    return 0;
}
#endif
static const struct dev_pm_ops fts_dev_pm_ops = {
#if defined(CONFIG_TOUCHSCREEN_FTS_GESTURE)
	.suspend = fts_ts_suspend_gesture,
	.resume = fts_ts_resume_gesture,
#endif
#if FTS_PATCH_COMERR_PM
	.suspend = fts_pm_suspend,
	.resume = fts_pm_resume,
#endif
};
#endif

/*****************************************************************************
* TP Driver
*****************************************************************************/

static int fts_ts_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    int ret = 0;
    struct fts_ts_data *ts_data = NULL;

    FTS_INFO("Touch Screen(I2C BUS) driver prboe...");
    if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
        FTS_ERROR("I2C not supported");
        return -ENODEV;
    }

    /* malloc memory for global struct variable */
    ts_data = (struct fts_ts_data *)kzalloc(sizeof(*ts_data), GFP_KERNEL);
    if (!ts_data) {
        FTS_ERROR("allocate memory for fts_data fail");
        return -ENOMEM;
    }

    fts_data = ts_data;
    ts_data->client = client;
    ts_data->dev = &client->dev;
    ts_data->log_level = 0;
    ts_data->fw_is_running = 0;
    ts_data->bus_type = BUS_TYPE_I2C;
    i2c_set_clientdata(client, ts_data);

    ret = fts_ts_probe_entry(ts_data);
    if (ret) {
        FTS_ERROR("Touch Screen(I2C BUS) driver probe fail");
        kfree_safe(ts_data);
        return ret;
    }

	factory_ts_func_test_register(ts_data);
	ft5526_ts_register_productinfo(ts_data);
    FTS_INFO("Touch Screen(I2C BUS) driver prboe successfully");
	fts_probed = true;
    return 0;
}

static int fts_ts_remove(struct i2c_client *client)
{
    return fts_ts_remove_entry(i2c_get_clientdata(client));
}

static const struct i2c_device_id fts_ts_id[] = {
    {FTS_DRIVER_NAME, 0},
    {},
};
static const struct of_device_id fts_dt_match[] = {
    {.compatible = "focaltech,fts", },
    {},
};
MODULE_DEVICE_TABLE(of, fts_dt_match);

static struct i2c_driver fts_ts_driver = {
    .probe = fts_ts_probe,
    .remove = fts_ts_remove,
    .driver = {
        .name = FTS_DRIVER_NAME,
        .owner = THIS_MODULE,
#if defined(CONFIG_PM) || FTS_PATCH_COMERR_PM
        .pm = &fts_dev_pm_ops,
#endif
        .of_match_table = of_match_ptr(fts_dt_match),
    },
    .id_table = fts_ts_id,
};

static int __init fts_ts_init(void)
{
    int ret = 0;

    FTS_FUNC_ENTER();
    ret = i2c_add_driver(&fts_ts_driver);
    if ( ret != 0 ) {
        FTS_ERROR("Focaltech touch screen driver init failed!");
    }
    FTS_FUNC_EXIT();
    return ret;
}

static void __exit fts_ts_exit(void)
{
    i2c_del_driver(&fts_ts_driver);
}
module_init(fts_ts_init);
module_exit(fts_ts_exit);

MODULE_AUTHOR("FocalTech Driver Team");
MODULE_DESCRIPTION("FocalTech Touchscreen Driver");
MODULE_LICENSE("GPL v2");
