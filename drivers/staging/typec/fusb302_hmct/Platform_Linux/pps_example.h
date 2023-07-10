#ifndef _PPS_EXAMPLE_H_
#define _PPS_EXAMPLE_H_

#include "../core/modules/observer.h"
#include "../core/port.h"
#include "../core/PD_Types.h"

struct charger_object {
	struct Port *port;
	unsigned int requested_pdo;
	unsigned int req_voltage; /* 20mV units */
	unsigned int req_current; /* 50mA units */
	unsigned int direct_charge_active;
};

void init_sink_pps_example(struct Port *port, struct charger_object *charger);
void src_caps_updated_handler(FSC_U32 event, FSC_U8 port_id, void *usr_ctx, void *app_ctx);
void src_caps_ext_received_handler(FSC_U32 event, FSC_U8 port_id, void *usr_ctx, void *app_ctx);
void new_contract_handler(FSC_U32 event, FSC_U8 port_id, void *usr_ctx, void *app_ctx);
void alert_received_handler(FSC_U32 event, FSC_U8 port_id, void *usr_ctx, void *app_ctx);
void status_received_handler(FSC_U32 event, FSC_U8 port_id, void *usr_ctx, void *app_ctx);
void pps_status_received_handler(FSC_U32 event, FSC_U8 port_id, void *usr_ctx, void *app_ctx);
void pd_failed_handler(FSC_U32 event, FSC_U8 port_id, void *usr_ctx, void *app_ctx);
void hard_reset_handler(FSC_U32 event, FSC_U8 port_id, void *usr_ctx, void *app_ctx);
void typec_detach_handler(FSC_U32 event, FSC_U8 port_id, void *usr_ctx, void *app_ctx);






#endif /* _PPS_EXAMPLE_H_ */
