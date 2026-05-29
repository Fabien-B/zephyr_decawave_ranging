#pragma once

#include <stdbool.h>

#include "pdec_common.h"
#include "pdec_database.h"

void send_sync();
bool handle_sync_msg(sync_msg_t* msg);
bool handle_blink_msg(blink_msg_t* msg);
bool handle_blink_report_msg(blink_report_msg_t* msg);
tdoa_role_t pdecTdoaGetRole(void);
void pdecTdoaSetRole(tdoa_role_t role);
void pdecTdoaStart(void);
