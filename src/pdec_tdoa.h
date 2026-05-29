#pragma once

#include <stdbool.h>

#include "pdec_common.h"

typedef enum {
    TDOA_ROLE_NONE,
    TDOA_ROLE_BEACON,
    TDOA_ROLE_BLINKER,
    TDOA_ROLE_MASTER,
} tdoa_role_t;

void send_sync();
bool handle_sync_msg(sync_msg_t* msg);
bool handle_blink_msg(blink_msg_t* msg);
bool handle_blink_report_msg(blink_report_msg_t* msg);
tdoa_role_t pdecTdoaGetRole(void);
void pdecTdoaSetRole(tdoa_role_t role);
void pdecTdoaStart(void);
