#ifndef USBPD_H_
#define USBPD_H_

#include <linux/kernel.h>
#include <linux/list.h>
#include <linux/usb/class-dual-role.h>
#include <linux/power_supply.h>

#include "../core/core.h"

struct usbpd {
	struct mutex                svid_handler_lock;
	struct list_head            svid_handlers;
	u16                         ss_lane_svid;
	FSC_BOOL                    has_dp;
	FSC_BOOL                    peer_usb_comm;
	struct dual_role_phy_instance *dual_role;
	struct dual_role_phy_desc   dr_desc;
	struct power_supply         *usb_psy;
	/* default false, when received svid,
	 * set to true to block_dp_event and wait usb host start
	 */
	FSC_BOOL                    block_dp_event;
};

struct usbpd *usbpd_create(struct device *parent);
void reset_usbpd(struct usbpd *pd);
void usbpd_destroy(struct usbpd *pd);

/*******************************************************************************
* Function:        fusb_init_event_handler
* Input:           none
* Return:          0 on success, error code on failure
* Description:     register core event handler.
*******************************************************************************/
void fusb_init_event_handler(void);

void stop_usb_host(struct fusb30x_chip* chip);
void start_usb_host(struct fusb30x_chip* chip, bool ss);
void stop_usb_peripheral(struct fusb30x_chip* chip);
void start_usb_peripheral(struct fusb30x_chip* chip);

void handle_core_event(FSC_U32 event, FSC_U8 portId,
		void *usr_ctx, void *app_ctx);
int usbpd_release_ss_lane(struct usbpd *pd,
		struct usbpd_svid_handler *handler);
#endif
