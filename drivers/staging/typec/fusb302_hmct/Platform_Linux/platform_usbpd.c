/* Copyright (c) 2016-2018, Linux Foundation. All rights reserved.
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

#include <linux/delay.h>
#include <linux/extcon.h>
#include <linux/kernel.h>
#include <linux/list.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/power_supply.h>

#include <linux/usb/class-dual-role.h>
#include <linux/usb/usbpd.h>
#include <linux/completion.h>

#include "fusb30x_global.h"
#include "platform_helpers.h"
#include "platform_usbpd.h"
#include "../core/core.h"

#ifdef CONFIG_HISENSE_SGM41512_CHARGER
extern int sgm_typec_otg_usb_switch(bool enable);
#endif /*CONFIG_HISENSE_SGM41512_CHARGER*/

#ifdef CONFIG_HISENSE_SGM_DOUBLE_USB
extern int sgm_fusb_typec_otg_usb_switch(bool enable);
#endif
extern void stop_usb_host(struct fusb30x_chip* chip);

static enum dual_role_property usbpd_dr_properties[] = {
	DUAL_ROLE_PROP_SUPPORTED_MODES,
	DUAL_ROLE_PROP_MODE,
	DUAL_ROLE_PROP_PR,
	DUAL_ROLE_PROP_DR,
};

static void fusb_force_source(struct dual_role_phy_instance *dual_role)
{
	struct fusb30x_chip* chip = fusb30x_GetChip();
	pr_err("FUSB - %s, try_src\n", __func__);
	//core_set_source(&chip->port);
	core_set_try_src(&chip->port);
	if (dual_role)
		dual_role_instance_changed(dual_role);
}

static void fusb_force_sink(struct dual_role_phy_instance *dual_role)
{
	struct fusb30x_chip* chip = fusb30x_GetChip();
	pr_err("FUSB - %s, try_snk\n", __func__);
	//core_set_sink(&chip->port);
	core_set_try_snk(&chip->port);
	if (dual_role)
		dual_role_instance_changed(dual_role);
}

 static unsigned fusbpd_typec_dr_set(void)
 {
 	struct fusb30x_chip* chip = fusb30x_GetChip();
 	pr_err("FUSB-%s - pr=%d, dr=%d\n",
 		__func__, chip->port.sourceOrSink, chip->port.PolicyIsDFP);
 
 	//if (role == TYPEC_HOST) {
 		if (chip->port.PolicyIsDFP == FALSE) {
 			if (chip->port.PolicyState == peSinkReady) {
 				SetPEState(&chip->port, peSinkSendDRSwap);
 			} else if (chip->port.PolicyState == peSourceReady) {
 				SetPEState(&chip->port, peSourceSendDRSwap);
 			}
 			chip->port.PEIdle = FALSE;
 			queue_work(chip->highpri_wq, &chip->sm_worker);
 			pr_err("FUSB %s-%d: run pe---SendDRSwap\n", __func__, __LINE__);
 		}
 	//} else if (role == TYPEC_DEVICE) {
 		else
 		if (chip->port.PolicyIsDFP == TRUE) {
 			if (chip->port.PolicyState == peSinkReady) {
 				SetPEState(&chip->port, peSinkSendDRSwap);
 			} else if (chip->port.PolicyState == peSourceReady) {
 				SetPEState(&chip->port, peSourceSendDRSwap);
 			}
 			chip->port.PEIdle = FALSE;
 			queue_work(chip->highpri_wq, &chip->sm_worker);
 			pr_err("FUSB %s-%d: run pe---SendDRSwap\n", __func__, __LINE__);
 		}
 	//}
 	return 0;
 }
 
 static unsigned fusbpd_typec_pr_set(void)
 {
 	struct fusb30x_chip* chip = fusb30x_GetChip();
 
 	pr_err("FUSB-%s - tc_st=%d, PE_ST=%d, pr=%d, dr=%d, PolicyIsSource=%d\n",
               __func__, chip->port.ConnState, chip->port.PolicyState,chip->port.sourceOrSink,
 		chip->port.PolicyIsDFP, chip->port.PolicyIsSource);
 
 	if (chip->port.ConnState == AttachedSink) {
 		if (chip->port.PolicyState == peSinkReady) {
 			if (chip->port.PolicyIsSource == FALSE) {
 				SetPEState(&chip->port, peSinkSendPRSwap);
 				chip->port.PEIdle = FALSE;
 				queue_work(chip->highpri_wq, &chip->sm_worker);
 				pr_err("FUSB %s: run peSinkSendPRSwap\n", __func__);
 			}
 		} else {
 		      pr_err("FUSB %s: core_set_try_src \n", __func__);
 			//core_set_try_src(&chip->port); //deleted by zhaotingfa for cts test JIRA LJRG935-906
 		}
 		pr_err("FUSB %s start try source or prswap to source \n", __func__);
 	} else if(chip->port.ConnState == AttachedSource) {
 		if (chip->port.PolicyState == peSourceReady) {
 			if (chip->port.PolicyIsSource == TRUE) {
 				SetPEState(&chip->port, peSourceSendPRSwap);
 				chip->port.PEIdle = FALSE;
 				queue_work(chip->highpri_wq, &chip->sm_worker);
 				pr_err("FUSB %s: run peSourceSendPRSwap\n", __func__);
 			}
 		} else {
 		      pr_err("FUSB %s: core_set_try_snk \n", __func__);
 			core_set_try_snk(&chip->port);
 		}
 		pr_err("FUSB %s start try sink or prswap to sink \n", __func__);
 	}
 	return 0;
 }
static unsigned int fusb_get_dual_role_mode(void)
{
	struct fusb30x_chip* chip = fusb30x_GetChip();
	int mode = DUAL_ROLE_PROP_MODE_NONE;

	if (chip->port.CCPin != CCNone) {
		if (chip->port.sourceOrSink == SOURCE) {
			mode = DUAL_ROLE_PROP_MODE_DFP;
			pr_err("FUSB - %s DUAL_ROLE_PROP_MODE_DFP, mode = %d\n",
					__func__, mode);
		} else {
			mode = DUAL_ROLE_PROP_MODE_UFP;
			pr_err("FUSB - %s DUAL_ROLE_PROP_MODE_UFP, mode = %d\n",
					__func__, mode);
		}
	}
	pr_err("FUSB - %s mode = %d\n", __func__, mode);
	return mode;
}

static unsigned int fusb_get_dual_role_power(void)
{
	struct fusb30x_chip* chip = fusb30x_GetChip();
	int current_pr = DUAL_ROLE_PROP_PR_NONE;

	pr_err("FUSB %s\n", __func__);

	if (chip->port.CCPin != CCNone) {
		if (chip->port.sourceOrSink == SOURCE) {
			current_pr = DUAL_ROLE_PROP_PR_SRC;
			pr_err("FUSB - %s DUAL_ROLE_PROP_PR_SRC, current_pr = %d\n",
					__func__, current_pr);
		} else {
			current_pr = DUAL_ROLE_PROP_PR_SNK;
			pr_err("FUSB - %s DUAL_ROLE_PROP_PR_SNK, current_pr = %d\n",
					__func__, current_pr);
		}
	}
	pr_err("FUSB - %s current_pr = %d\n", __func__, current_pr);
	return current_pr;
}

static unsigned int fusb_get_dual_role_data(void)
{
	struct fusb30x_chip* chip = fusb30x_GetChip();
	int current_dr = DUAL_ROLE_PROP_DR_NONE;

	pr_err("FUSB %s\n", __func__);

	if (chip->port.CCPin != CCNone) {
		if (chip->port.PolicyIsDFP) {
			current_dr = DUAL_ROLE_PROP_DR_HOST;
			pr_err("FUSB - %s DUAL_ROLE_PROP_DR_HOST, current_dr = %d\n",
					__func__, current_dr);
		} else {
			current_dr = DUAL_ROLE_PROP_DR_DEVICE;
			pr_err("FUSB - %s DUAL_ROLE_PROP_DR_DEVICE, current_dr = %d\n",
					__func__, current_dr);
		}
	}
	pr_err("FUSB - %s current_dr = %d\n", __func__, current_dr);
	return current_dr;
}

static int usbpd_dr_get_property(struct dual_role_phy_instance *dual_role,
		enum dual_role_property prop, unsigned int *val)
{
	unsigned int mode = DUAL_ROLE_PROP_MODE_NONE;
	switch (prop) {
	case DUAL_ROLE_PROP_MODE:
		mode = fusb_get_dual_role_mode();
		*val = mode;
		break;
	case DUAL_ROLE_PROP_PR:
		mode = fusb_get_dual_role_power();
		*val = mode;
		break;
	case DUAL_ROLE_PROP_DR:
		mode = fusb_get_dual_role_data();
		*val = mode;
		break;
	default:
		pr_err("FUSB unsupported property %d\n", prop);
		return -ENODATA;
	}
	pr_err("FUSB %s + prop=%d, val=%d\n", __func__, prop, *val);
	return 0;
}

static int usbpd_dr_set_property(struct dual_role_phy_instance *dual_role,
		enum dual_role_property prop, const unsigned int *val)
{
	struct fusb30x_chip* chip = fusb30x_GetChip();
	unsigned int mode = fusb_get_dual_role_mode();

	pr_err("FUSB %s\n", __func__);

	if (!chip) {
		pr_err("FUSB %s - Error: Chip structure is NULL!\n", __func__);
		return -1;
	}
	pr_err("FUSB %s + prop=%d,val=%d,mode=%d\n",
			__func__, prop, *val, mode);
	switch (prop) {
	case DUAL_ROLE_PROP_MODE:
		if (*val != mode) {
			if (mode == DUAL_ROLE_PROP_MODE_UFP)
				fusb_force_source(dual_role);
			else if (mode == DUAL_ROLE_PROP_MODE_DFP)
				fusb_force_sink(dual_role);
		}
		break;
	case DUAL_ROLE_PROP_PR:
		pr_err("FUSB - %s DUAL_ROLE_PROP_PR\n", __func__);
		fusbpd_typec_pr_set();
		break;
	case DUAL_ROLE_PROP_DR:
		pr_err("FUSB - %s DUAL_ROLE_PROP_DR\n", __func__);
		fusbpd_typec_dr_set();
		break;
	default:
		pr_err("FUSB - %s default case\n", __func__);
		break;
	}
	return 0;
}

static int usbpd_dr_prop_writeable(struct dual_role_phy_instance *dual_role,
		enum dual_role_property prop)
{
	pr_err("FUSB - %s\n", __func__);
	switch (prop) {
	case DUAL_ROLE_PROP_MODE:
		return 1;
		break;
	case DUAL_ROLE_PROP_DR:
	case DUAL_ROLE_PROP_PR:
		return 0;
		break;
	default:
		break;
	}
	return 1;
}

struct usbpd *usbpd_create(struct device *parent)
{
	struct usbpd *pd;

	pd = devm_kzalloc(parent, sizeof(*pd), GFP_KERNEL);
	if (!pd)
		return ERR_PTR(-ENOMEM);

	mutex_init(&pd->svid_handler_lock);
	INIT_LIST_HEAD(&pd->svid_handlers);

	pd->dr_desc.name = "otg_default";
	pd->dr_desc.supported_modes = DUAL_ROLE_SUPPORTED_MODES_DFP_AND_UFP;
	pd->dr_desc.properties = usbpd_dr_properties;
	pd->dr_desc.num_properties = ARRAY_SIZE(usbpd_dr_properties);
	pd->dr_desc.get_property = usbpd_dr_get_property;
	pd->dr_desc.set_property = usbpd_dr_set_property;
	pd->dr_desc.property_is_writeable = usbpd_dr_prop_writeable;

	pd->dual_role = devm_dual_role_instance_register(parent,
		&pd->dr_desc);
	if (IS_ERR(pd->dual_role)) {
		pr_info("FUSB could not register dual_role instance\n");
	} else {
		pd->dual_role->drv_data = pd;
	}

	pr_err("FUSB %s\n", __func__);

	return pd;
}

void usbpd_destroy(struct usbpd *pd)
{
}

void reset_usbpd(struct usbpd *pd)
{
	pd->ss_lane_svid = 0x0;
}

/**
 * This API allows client driver to request for releasing SS lanes. It should
 * not be called from atomic context.
 */
int usbpd_release_ss_lane(struct usbpd *pd,
				struct usbpd_svid_handler *handler)
{
	int ret = 0;
	struct fusb30x_chip* chip = fusb30x_GetChip();
	pr_debug("FUSB - %s +++\n", __func__);

	if (!handler || !chip) {
		pr_err("FUSB - %s ss lanes are already used by %d\n",
			__func__, chip->usbpd->ss_lane_svid);
		return -EINVAL;	
	}

	pr_debug("FUSB %s handler:%pK svid:%d", __func__, handler, handler->svid);
	/*
	 * If USB SS lanes are already used by one client, and other client is
	 * requesting for same or same client requesting again, return -EBUSY.
	 */
	if (chip->usbpd->ss_lane_svid) {
		pr_err("FUSB %s: ss_lanes are already used by %d",
				__func__, chip->usbpd->ss_lane_svid);
		ret = -EBUSY;
		goto err_exit;
	}

	extcon_blocking_sync(chip->extcon, EXTCON_USB_HOST, 1);
	stop_usb_host(chip);

	/* blocks until USB host is completely stopped */
	ret = extcon_blocking_sync(chip->extcon, EXTCON_USB_HOST, 0);
	if (ret) {
		pr_err("FUSB %s err %d stopping host", __func__, ret);
		goto err_exit;
	}

	start_usb_host(chip, false);
	extcon_blocking_sync(chip->extcon, EXTCON_USB_HOST, 1);
	chip->usbpd->ss_lane_svid = handler->svid;

err_exit:
	return ret;
}

static struct usbpd_svid_handler *find_svid_handler(struct usbpd *pd, u16 svid)
{
	struct usbpd_svid_handler *handler;

	mutex_lock(&pd->svid_handler_lock);
	list_for_each_entry(handler, &pd->svid_handlers, entry) {
		if (svid == handler->svid) {
			mutex_unlock(&pd->svid_handler_lock);
			return handler;
		}
	}
	mutex_unlock(&pd->svid_handler_lock);
	return NULL;
}

struct usbpd *devm_usbpd_get_by_phandle(struct device *dev,
		const char *phandle)
{
	struct fusb30x_chip* chip = fusb30x_GetChip();

	pr_debug("FUSB enter %s\n", __func__);

	if (!chip) {
		pr_err("FUSB Chip not ready!\n");
		return ERR_PTR(-EAGAIN);
	}
	return chip->usbpd;
}

static int usbpd_dp_release_ss_lane(struct usbpd *pd,
	struct usbpd_svid_handler *handler)
{
	pr_info("FUSB dp request us to release sslane\n");
	return 0;
}

int usbpd_register_svid(struct usbpd *pd, struct usbpd_svid_handler *hdlr)
{
	struct fusb30x_chip* chip = fusb30x_GetChip();
	int ret;

	if (find_svid_handler(pd, hdlr->svid)) {
		pr_err("FUSB SVID 0x%04x already registered\n",
			hdlr->svid);
		return -EINVAL;
	}

	/* require connect/disconnect callbacks be implemented */
	if (!hdlr->connect || !hdlr->disconnect) {
		pr_err("FUSB SVID 0x%04x connect/disconnect must be non-NULL\n",
				hdlr->svid);
		return -EINVAL;
	}

	pr_debug("FUSB registered handler(%pK) for SVID 0x%04x\n",
							hdlr, hdlr->svid);
	mutex_lock(&pd->svid_handler_lock);
	list_add_tail(&hdlr->entry, &pd->svid_handlers);
	mutex_unlock(&pd->svid_handler_lock);
	hdlr->request_usb_ss_lane = usbpd_dp_release_ss_lane;

	/* reset fusb302 */
	fusb_reset();

	/* already connected with this SVID discovered? */
	if (!chip) {
		pr_err("FUSB Chip not ready!\n");
		return -EINVAL;
	}

	/* need to check with dp team
	 * get better solution
	 * to cover no dp function
	 */

	/* Enable interrupts after successful core/GPIO initialization */
    ret = fusb_EnableInterrupts();
    if (ret)
    {
        pr_err("FUSB  %s - Error: Unable to enable interrupts! Error code: %d\n", __func__, ret);
        return -EIO;
    }

    /* Initialize the core and enable the state machine (NOTE: timer and GPIO must be initialized by now)
    *  Interrupt must be enabled before starting 302 initialization */
    fusb_InitializeCore();
    pr_info("FUSB  %s - Core is initialized!\n", __func__);
#if 0	
	if (chip->port.svid_discvry_done) {
		for (int i = 0; i < chip->port.core_svid_info.num_svids; i++) {
			if (chip->port.core_svid_info.svids[i] == hdlr->svid) {
				hdlr->connect(hdlr);
				hdlr->discovered = true;

				ret = usbpd_release_ss_lane(chip->usbpd, hdlr);
				pr_info("FUSB - %s: usbpd_release_ss_lane return ret=%d\n",
					__func__, ret);
				chip->usbpd->block_dp_event = FALSE;
				break;
			}
		}
	}
#endif

	return 0;
}
EXPORT_SYMBOL(usbpd_register_svid);

void usbpd_unregister_svid(struct usbpd *pd, struct usbpd_svid_handler *hdlr)
{

	pr_info("FUSB unregistered handler(%pK) for SVID 0x%04x\n",
							hdlr, hdlr->svid);
	mutex_lock(&pd->svid_handler_lock);
	list_del_init(&hdlr->entry);
	mutex_unlock(&pd->svid_handler_lock);
}
EXPORT_SYMBOL(usbpd_unregister_svid);

int usbpd_send_vdm(struct usbpd *pd, u32 vdm_hdr, const u32 *vdos, int num_vdos)
{
	struct Port* port;
	int i = 0;
	struct fusb30x_chip* chip = fusb30x_GetChip();

	pr_info("FUSB enter: %s, vdm_hdr=%8x\n", __func__, vdm_hdr);
	if (!chip || !chip->port.PolicyHasContract) {
		pr_info("FUSB:%s No PDPHY or PD isn't supported\n");
		return -1;
	}
	port = &chip->port;

	port->PolicyMsgTxSop = SOP_TYPE_SOP;

	port->PDTransmitHeader.word = 0;
	port->PDTransmitHeader.MessageType = DMTVenderDefined;
	port->PDTransmitHeader.NumDataObjects = num_vdos + 1;
	port->PDTransmitHeader.PortDataRole = port->PolicyIsDFP;
	port->PDTransmitHeader.PortPowerRole = port->PolicyIsSource;
	port->PDTransmitHeader.SpecRevision = DPM_SpecRev(port, SOP_TYPE_SOP);

	port->PDTransmitObjects[0].object = vdm_hdr;
	/* Data objects */
	for (i = 1; i < port->PDTransmitHeader.NumDataObjects; ++i)
	{
		port->PDTransmitObjects[i].object = *vdos++;
	}
	port->USBPDTxFlag = TRUE;
	return 0;
}
EXPORT_SYMBOL(usbpd_send_vdm);

#define SVDM_HDR(svid, ver, obj, cmd_type, cmd) \
	(((svid) << 16) | (1 << 15) | ((ver) << 13) \
	| ((obj) << 8) | ((cmd_type) << 6) | (cmd))

int usbpd_send_svdm(struct usbpd *pd, u16 svid, u8 cmd,
		enum usbpd_svdm_cmd_type cmd_type, int obj_pos,
		const u32 *vdos, int num_vdos)
{
	u32 svdm_hdr = SVDM_HDR(svid, 0, obj_pos, cmd_type, cmd);
	return usbpd_send_vdm(pd, svdm_hdr, vdos, num_vdos);
}
EXPORT_SYMBOL(usbpd_send_svdm);

void usbpd_vdm_in_suspend(struct usbpd *pd, bool in_suspend)
{
	pr_info("FUSB - %s :%d\n", __func__, in_suspend);
}
EXPORT_SYMBOL(usbpd_vdm_in_suspend);

enum plug_orientation usbpd_get_plug_orientation(struct usbpd *pd)
{
	struct fusb30x_chip* chip = fusb30x_GetChip();
	return (int)chip->port.CCPin;
}
extern bool global_typea_otg_status;
void stop_usb_host(struct fusb30x_chip* chip)
{
	pr_info("FUSB - %s\n", __func__);

#ifdef CONFIG_HISENSE_SGM_DOUBLE_USB
	if(!global_typea_otg_status)
		extcon_set_state_sync(chip->extcon, EXTCON_USB_HOST, 0);
	else
		pr_info("FUSB - %s,typea otg exist,don't change usb mode\n", __func__);
	sgm_fusb_typec_otg_usb_switch(false);
#else
	extcon_set_state_sync(chip->extcon, EXTCON_USB_HOST, 0);
#endif

#ifdef CONFIG_HISENSE_SGM41512_CHARGER
	sgm_typec_otg_usb_switch(false);
#endif /*CONFIG_HISENSE_SGM41512_CHARGER*/
}

void start_usb_host(struct fusb30x_chip* chip, bool ss)
{
	union extcon_property_value val;
	
	pr_info("FUSB - %s, ss=%d\n", __func__, ss);

	val.intval = (chip->port.CCPin == CC2);
	extcon_set_property(chip->extcon, EXTCON_USB_HOST,
			EXTCON_PROP_USB_TYPEC_POLARITY, val);

	val.intval = ss;
	extcon_set_property(chip->extcon, EXTCON_USB_HOST,
			EXTCON_PROP_USB_SS, val);

	extcon_set_state_sync(chip->extcon, EXTCON_USB_HOST, 1);
#ifdef CONFIG_HISENSE_SGM41512_CHARGER
	sgm_typec_otg_usb_switch(true);
#endif /*CONFIG_HISENSE_SGM41512_CHARGER*/

#ifdef CONFIG_HISENSE_SGM_DOUBLE_USB
	sgm_fusb_typec_otg_usb_switch(true);
#endif
}

void stop_usb_peripheral(struct fusb30x_chip* chip)
{
	pr_info("FUSB - %s\n", __func__);
	extcon_set_state_sync(chip->extcon, EXTCON_USB, 0);
}

void start_usb_peripheral(struct fusb30x_chip* chip)
{
	union extcon_property_value val;
	
	pr_info("FUSB - %s\n", __func__);

	val.intval = (chip->port.CCPin == CC2);
	extcon_set_property(chip->extcon, EXTCON_USB,
			EXTCON_PROP_USB_TYPEC_POLARITY, val);
	pr_debug("FUSB - %s, EXTCON_PROP_USB_TYPEC_POLARITY=%d\n",
		__func__, val.intval);

	val.intval = 1;
	extcon_set_property(chip->extcon, EXTCON_USB, EXTCON_PROP_USB_SS, val);
	pr_debug("FUSB - %s, EXTCON_PROP_USB_SS=%d\n", __func__, val.intval);

	val.intval = chip->port.SinkCurrent > utccDefault ? 1 : 0;
	extcon_set_property(chip->extcon, EXTCON_USB,
		EXTCON_PROP_USB_TYPEC_MED_HIGH_CURRENT, val);
	pr_debug("FUSB - %s, EXTCON_PROP_USB_TYPEC_MED_HIGH_CURRENT=%d\n",
		__func__, val.intval);

	extcon_set_state_sync(chip->extcon, EXTCON_USB, 1);
}

void fusb_cancel_apsd_check_work(void)
{
    struct fusb30x_chip* chip = fusb30x_GetChip();

    cancel_delayed_work_sync(&chip->apsd_recheck_work);
}

void fusb_notify_apsd_complete(void)
{
    struct fusb30x_chip* chip = fusb30x_GetChip();

    complete(&chip->apsd_done_complete);
}

void fusb_reinit_apsd_complete(void)
{
    struct fusb30x_chip* chip = fusb30x_GetChip();

    reinit_completion(&chip->apsd_done_complete);
}

bool get_fusb_probe_status(void)
{
    return fusb_probe_done;
}

void handle_core_event(FSC_U32 event, FSC_U8 portId,
		void *usr_ctx, void *app_ctx)
{
	int i = 0;
	int ret = 0;
	doDataObject_t vdmh_in = { 0 };
	FSC_U32* arr_in = NULL;
	struct usbpd_svid_handler* handler = NULL;
	union power_supply_propval val = {0};
	static bool start_power_swap = FALSE;
	FSC_U32 set_voltage;
	FSC_U32 op_current;
	struct fusb30x_chip* chip = fusb30x_GetChip();

	if (!chip) {
		pr_err("FUSB %s - Error: Chip structure is NULL!\n", __func__);
		return;
	}

	pr_buf_info("FUSB %s - Notice, event=0x%x,TC_ST=%d, PE_ST=%d \n", __func__, event,
		chip->port.ConnState,chip->port	.PolicyState);

	pr_buf_info("FUSB %s - Notice, pr =%d, dr =%d \n", __func__,
		chip->port.sourceOrSink,chip->port.PolicyIsDFP);
	
	switch (event) {
	case CC1_ORIENT:
	case CC2_ORIENT:
		pr_buf_info("FUSB %s:CC Changed=0x%x\n", __func__, event);

		if (chip->port.sourceOrSink == SINK) {
			fusb_cancel_apsd_check_work();
			schedule_delayed_work(&chip->apsd_recheck_work, 0);

			//val.intval = POWER_SUPPLY_TYPEC_PR_SINK;
			//power_supply_set_property(chip->usbpd->usb_psy,
				//POWER_SUPPLY_PROP_TYPEC_POWER_ROLE, &val);
		} else if (chip->port.sourceOrSink == SOURCE) {
			start_usb_host(chip, true);
			chip->usb_state = 2;
			chip->usbpd->ss_lane_svid = 0x0;
			pr_buf_info("FUSB %s start_usb_host\n", __func__);
			//val.intval = POWER_SUPPLY_TYPEC_PR_SOURCE;
			
			//power_supply_set_property(chip->usbpd->usb_psy,
				//POWER_SUPPLY_PROP_TYPEC_POWER_ROLE, &val);
		}

		dual_role_instance_changed(chip->usbpd->dual_role);
		break;
	case CC_NO_ORIENT:
		pr_buf_info("FUSB %s:CC_NO_ORIENT=0x%x\n", __func__, event);
		fusb_cancel_apsd_check_work();
		if (chip->usbpd->has_dp) {
			chip->usbpd->has_dp = false;
			handler = find_svid_handler(chip->usbpd, 0xFF01);
			if (handler && handler->disconnect) {
				handler->disconnect(handler);
				handler->discovered = true;
				chip->usbpd->ss_lane_svid = 0x0;
			}

			/* Set to USB only mode when cable disconnected */
			extcon_blocking_sync(chip->extcon, EXTCON_DISP_DP, 0);
		}

		start_power_swap = false;
		if (chip->usb_state == 1) {
			stop_usb_peripheral(chip);
			chip->usb_state = 0;
			pr_buf_info("FUSB - %s stop_usb_peripheral,event=0x%x,usb_state=%d\n",
				__func__, event, chip->usb_state);
		} else if (chip->usb_state == 2) {
			stop_usb_host(chip);
			chip->usb_state = 0;
			pr_debug("FUSB - %s stop_usb_host,event=0x%x,usb_state=%d\n",
				__func__, event, chip->usb_state);
		}

		val.intval = POWER_SUPPLY_PD_INACTIVE;
		power_supply_set_property(chip->usbpd->usb_psy,
			POWER_SUPPLY_PROP_PD_ACTIVE, &val);

		val.intval = 0;
		power_supply_set_property(chip->usbpd->usb_psy,
			POWER_SUPPLY_PROP_PD_CURRENT_MAX, &val);

		val.intval = 5000000;
		power_supply_set_property(chip->usbpd->usb_psy,
			POWER_SUPPLY_PROP_PD_VOLTAGE_MIN, &val);
		power_supply_set_property(chip->usbpd->usb_psy,
			POWER_SUPPLY_PROP_PD_VOLTAGE_MAX, &val);

		dual_role_instance_changed(chip->usbpd->dual_role);
		break;
	case PD_STATE_CHANGED:
		pr_buf_info("FUSB %s:PD_STATE_CHANGED=0x%x, PE_ST=%d\n",
			__func__, event, chip->port.PolicyState);
		if (chip->port.PolicyState == peSinkReady &&
			chip->port.PolicyHasContract == TRUE) {
			pr_info("FUSB %s update power_supply properties\n",
				__func__);

			val.intval = POWER_SUPPLY_PD_ACTIVE;
			power_supply_set_property(chip->usbpd->usb_psy,
				POWER_SUPPLY_PROP_PD_ACTIVE, &val);

			/*PD cherger type recognized, notify apsd complete*/
			fusb_notify_apsd_complete();

			set_voltage = chip->port.SrcCapsReceived[
				chip->port.USBPDContract.FVRDO.ObjectPosition - 1].FPDOSupply.Voltage;
			op_current = chip->port.USBPDContract.FVRDO.OpCurrent;

			if (op_current > 0){
				val.intval = op_current * 10 * 1000;
				power_supply_set_property(chip->usbpd->usb_psy,
					POWER_SUPPLY_PROP_PD_CURRENT_MAX, &val);

				val.intval = 5000000;
				power_supply_set_property(chip->usbpd->usb_psy,
					POWER_SUPPLY_PROP_PD_VOLTAGE_MIN, &val);
				if (set_voltage > 100){
					val.intval = 9000000;
					power_supply_set_property(chip->usbpd->usb_psy,
						POWER_SUPPLY_PROP_PD_VOLTAGE_MAX, &val);
				} else {
					val.intval = 5000000;
					power_supply_set_property(chip->usbpd->usb_psy,
						POWER_SUPPLY_PROP_PD_VOLTAGE_MAX, &val);
				}
			}
		}
		
            if ((chip->port.PolicyState == peSinkReady ||
                          chip->port.PolicyState == peSourceReady) &&
                          chip->port.PolicyHasContract == TRUE) {
                  // val.intval = POWER_SUPPLY_PD_ACTIVE;
			//power_supply_set_property(chip->usbpd->usb_psy,
			//	POWER_SUPPLY_PROP_PD_ACTIVE, &val);
			dual_role_instance_changed(chip->usbpd->dual_role);

		}
		
		break;

	case PD_NO_CONTRACT:
		pr_buf_info("FUSB %s:PD_NO_CONTRACT=0x%x, PE_ST=%d\n",
			__func__, event, chip->port.PolicyState);

		val.intval = POWER_SUPPLY_PD_INACTIVE;
		power_supply_set_property(chip->usbpd->usb_psy,
			POWER_SUPPLY_PROP_PD_ACTIVE, &val);

		val.intval = 5000000;
		power_supply_set_property(chip->usbpd->usb_psy,
			POWER_SUPPLY_PROP_PD_VOLTAGE_MIN, &val);
		power_supply_set_property(chip->usbpd->usb_psy,
			POWER_SUPPLY_PROP_PD_VOLTAGE_MAX, &val);
		break;
	case SVID_EVENT:
		chip->usbpd->has_dp = FALSE;
		chip->usbpd->block_dp_event = TRUE;
		for (i = 0; i < chip->port.core_svid_info.num_svids; i++)
		{
			handler = find_svid_handler(chip->usbpd, chip->port.core_svid_info.svids[i]);
			if (handler) {
				pr_buf_info("FUSB %s:Get handler for %4x\n", __func__, chip->port.core_svid_info.svids[i]);
				ret = usbpd_release_ss_lane(chip->usbpd, handler);
				pr_buf_info("FUSB %s: usbpd_release_ss_lane return ret=%d\n", __func__, ret);

				handler->connect(handler, true);
				handler->discovered = true;
				chip->usbpd->block_dp_event = FALSE;
			}
		}
		break;
	case DP_EVENT:
		pr_buf_info("FUSB %s:DP_EVENT=0x%x\n", __func__, event);
		pr_buf_info("FUSB %s:chip->port.AutoVdmState=%d\n",
			__func__, chip->port.AutoVdmState);

		if (chip->usbpd->has_dp == FALSE) {
			chip->usbpd->has_dp = TRUE;
		}
		handler = find_svid_handler(chip->usbpd, 0xFF01);
		if (handler && handler->svdm_received && !chip->usbpd->block_dp_event) {
			arr_in = (FSC_U32*)app_ctx;
			vdmh_in.object = arr_in[0];

			handler->svdm_received(handler, vdmh_in.SVDM.Command,
				vdmh_in.SVDM.CommandType,
				arr_in + 1,
				chip->port.PolicyRxHeader.NumDataObjects - 1);
		}
		break;
	case DATA_ROLE:
		pr_buf_info("FUSB %s:DATA_ROLE=0x%x\n", __func__, event);
		fusb_cancel_apsd_check_work();

		if (chip->port.PolicyIsDFP == FALSE) {
			if (chip->usb_state == 2)
				stop_usb_host(chip);
			start_usb_peripheral(chip);
			chip->usb_state = 1;
		} else if (chip->port.PolicyIsDFP == TRUE) {
			if (chip->usb_state == 1)
				stop_usb_peripheral(chip);
			start_usb_host(chip, true);
			chip->usb_state = 2;

			/* ensure host is started before allowing DP */
			//extcon_blocking_sync(chip->extcon, EXTCON_USB_HOST, 0);
		}

		dual_role_instance_changed(chip->usbpd->dual_role);
		break;
	case POWER_ROLE:
		pr_buf_info("FUSB - %s:POWER_ROLE=0x%x", __func__, event);
		if (start_power_swap == FALSE) {
			start_power_swap = true;
			val.intval = 1;
			power_supply_set_property(chip->usbpd->usb_psy,
				POWER_SUPPLY_PROP_PR_SWAP, &val);
		} else {
			start_power_swap = false;
			val.intval = 0;
			power_supply_set_property(chip->usbpd->usb_psy,
				POWER_SUPPLY_PROP_PR_SWAP, &val);
		}
             dual_role_instance_changed(chip->usbpd->dual_role);
		break;
	default:
		pr_buf_info("FUSB - %s:default=0x%x", __func__, event);
		break;
	}
}

static void fusb_apsd_recheck_work(struct work_struct *work)
{
    struct fusb30x_chip *chip = container_of(work, struct fusb30x_chip,
							apsd_recheck_work.work);
    union power_supply_propval val = {0};
    static u8 check_cnt = 0;
    int ret = 0;


    pr_err("FUSB %s: sourceOrSink:%d, CCPin:%d, check_cnt:%d\n",
                    __func__, chip->port.sourceOrSink, chip->port.CCPin, check_cnt);

    if ((chip->port.sourceOrSink == SINK) &&
                (chip->port.CCPin != CCNone)) {
        pr_err("FUSB %s wait for APSD complete\n", __func__);
        ret = wait_for_completion_timeout(&chip->apsd_done_complete, msecs_to_jiffies(300));

        power_supply_get_property(chip->usbpd->usb_psy,
                        POWER_SUPPLY_PROP_REAL_TYPE, &val);
        pr_err("FUSB %s:Charger type:%d\n", __func__, val.intval);

        if(!ret && check_cnt <= 2) {
            pr_err("FUSB %s wait APSD timeout, schedule re-check\n", __func__);
            schedule_delayed_work(&chip->apsd_recheck_work, msecs_to_jiffies(200));
            check_cnt++;
            return;
        } else {
            if(val.intval == POWER_SUPPLY_TYPE_USB ||
                        val.intval == POWER_SUPPLY_TYPE_USB_CDP) {
                start_usb_peripheral(chip);
                pr_buf_info("FUSB %s start_usb_peripheral\n", __func__);
                chip->usb_state = 1;
            }
        }
    }
    check_cnt = 0;
}

void fusb_init_event_handler(void)
{
	struct fusb30x_chip* chip = fusb30x_GetChip();

	init_completion(&chip->apsd_done_complete);
	INIT_DELAYED_WORK(&chip->apsd_recheck_work, fusb_apsd_recheck_work);
	register_observer(CC_ORIENT_ALL|PD_CONTRACT_ALL|POWER_ROLE|
			PD_STATE_CHANGED|DATA_ROLE|EVENT_ALL,
			handle_core_event, NULL);
}
