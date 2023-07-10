#include "pps_example.h"
#include "../core/core.h"
#include "../core/platform.h"

#define MAX_PPS_V 250   /* 20mV Units */
#define MAX_PPS_I 60    /* 50mA Units */

void init_sink_pps_example(struct Port *port, struct charger_object *charger)
{
	/* Init charger object */
	charger->port = port;
	charger->requested_pdo = 0;
	charger->direct_charge_active = 0;

	/* Register observers */
	register_observer(EVENT_SRC_CAPS_UPDATED, src_caps_updated_handler, charger);
	register_observer(SRC_CAPS_EXT_RECEIVED, src_caps_ext_received_handler, charger);
	register_observer(PD_NEW_CONTRACT, new_contract_handler, charger);
	register_observer(ALERT_EVENT, alert_received_handler, charger);
	register_observer(STATUS_RECEIVED, status_received_handler, charger);
	register_observer(PPS_STATUS_RECEIVED, pps_status_received_handler, charger);
	register_observer(EVENT_PD_CONTRACT_FAILED, pd_failed_handler, charger);
	register_observer(EVENT_HARD_RESET, hard_reset_handler, charger);
	register_observer(CC_NO_ORIENT, typec_detach_handler, charger);
}

void src_caps_updated_handler(FSC_U32 event, FSC_U8 port_id, void *usr_ctx, void *app_ctx)
{
	struct charger_object *charger = (struct charger_object *)usr_ctx;
	struct Port *port = charger->port;
	unsigned int num_caps;
	unsigned int obj_pos;
	unsigned int i;
    
    if(port->PolicyIsSource == TRUE) return;

    obj_pos = 0;
	num_caps = port->SrcCapsHeaderReceived.NumDataObjects;

	/* Select APDO - choose highest APDO */
	for(i = 0; i < num_caps; i++) {
		if(port->SrcCapsReceived[i].PDO.SupplyType == pdoTypeAugmented
				&& port->SrcCapsReceived[i].APDO.APDOType == apdoTypePPS) {
			obj_pos = i + 1;
		}
	}
    
    /* Save Request object */
	charger->requested_pdo = obj_pos;

	/* Evaluate voltage and current for request */
    if(obj_pos > 0) {
        charger->req_voltage = port->SrcCapsReceived[obj_pos - 1].PPSAPDO.MaxVoltage * 5;
        charger->req_current = port->SrcCapsReceived[obj_pos - 1].PPSAPDO.MaxCurrent;
        
        if(charger->req_voltage > MAX_PPS_V) {
            charger->req_voltage = MAX_PPS_V;
        }
        
        if(charger->req_current > MAX_PPS_I) {
            charger->req_current = MAX_PPS_I;
        }
    }

	/* Queue Get Source Caps Extended to be sent */
    if(port->PdRevSop == USBPDSPECREV3p0) {
        DPM_SendControlMessage(port, CMTGetSourceCapExt);
    }
}

void src_caps_ext_received_handler(FSC_U32 event, FSC_U8 port_id, void *usr_ctx, void *app_ctx)
{
	struct charger_object *charger = (struct charger_object *)usr_ctx;
	struct Port *port = charger->port;
	ExtSrcCapBlock_t *ext_src_caps = (ExtSrcCapBlock_t *)app_ctx;

    /* If app_ctx is 0, message was Not Supported */
    if(app_ctx) {
        /* Evaluate Extended Source Cap info, e.g. check VID for whitelist */
    }
    
	if(charger->requested_pdo != 0 && port->PolicyIsSource == FALSE) {
		DPM_SendPPSRequest(port, charger->req_voltage, charger->req_current, charger->requested_pdo);
	}
}

void new_contract_handler(FSC_U32 event, FSC_U8 port_id, void *usr_ctx, void *app_ctx)
{
	struct charger_object *charger = (struct charger_object *)usr_ctx;
	struct Port *port = charger->port;
	doDataObject_t *request = (doDataObject_t *)app_ctx;

	/*
	 * Check the app_ctx - if it's the requested PPS, configure the charging path
	 * Note: Valid object positions are 1-7
	 */
	if(request->PPSRDO.ObjectPosition == charger->requested_pdo) {
		charger->direct_charge_active = 1;
	} else {
		charger->direct_charge_active = 0;
	}
}

void alert_received_handler(FSC_U32 event, FSC_U8 port_id, void *usr_ctx, void *app_ctx)
{
	struct charger_object *charger = (struct charger_object *)usr_ctx;
	struct Port *port = charger->port;
	doDataObject_t *alert = (doDataObject_t *)app_ctx;

	/* Open battery path if OVP, OTP, OCP */
    if(alert->ADO.OVP || alert->ADO.OTP || alert->ADO.OCP) {
        /* Disable direct charging */
		charger->requested_pdo = 0;
		charger->direct_charge_active = 0;
    }

	/* Possibly reduce load on next request to avoid issue */

	/* Request Status */
	if(!alert->ADO.Battery) {
		DPM_SendControlMessage(port, CMTGetStatus);
	}
}

void status_received_handler(FSC_U32 event, FSC_U8 port_id, void *usr_ctx, void *app_ctx)
{
	struct charger_object *charger = (struct charger_object *)usr_ctx;
	struct Port *port = charger->port;
	Status_t *status = (Status_t *)app_ctx;

	/* Check status info */
}

void pps_status_received_handler(FSC_U32 event, FSC_U8 port_id, void *usr_ctx, void *app_ctx)
{
	struct charger_object *charger = (struct charger_object *)usr_ctx;
	struct Port *port = charger->port;
	PPSStatus_t *pps_status = (PPSStatus_t *)app_ctx;

	/* Check pps status info */

}

void pd_failed_handler(FSC_U32 event, FSC_U8 port_id, void *usr_ctx, void *app_ctx)
{
	struct charger_object *charger = (struct charger_object *)usr_ctx;
	struct Port *port = charger->port;
	doDataObject_t *request = (doDataObject_t *)app_ctx;
    
    if(port->PolicyIsSource == TRUE) return;
    if(!app_ctx) return;

	/*
	 * Check the app_ctx - if its the requested PPS, send get_source_caps
	 * to check for new source capabilities.
	 * Note: Valid object positions are 1-7
	 */
	if(request->PPSRDO.ObjectPosition == charger->requested_pdo) {
		DPM_SendControlMessage(port, CMTGetSourceCap);
	}
}

void hard_reset_handler(FSC_U32 event, FSC_U8 port_id, void *usr_ctx, void *app_ctx)
{
	struct charger_object *charger = (struct charger_object *)usr_ctx;

	/* Disable direct charging */
	charger->requested_pdo = 0;
	charger->direct_charge_active = 0;
}

void typec_detach_handler(FSC_U32 event, FSC_U8 port_id, void *usr_ctx, void *app_ctx)
{
	struct charger_object *charger = (struct charger_object *)usr_ctx;

	/* Disable direct charging */
	charger->requested_pdo = 0;
	charger->direct_charge_active = 0;
}
