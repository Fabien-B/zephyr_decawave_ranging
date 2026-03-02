#pragma once

#include "pdec_common.h"

typedef enum {
    TDOA_ROLE_BLINKER,
    TDOA_ROLE_MASTER,
    TDOA_ROLE_BEACON,
} tdoa_role_t;

void send_sync();
void handle_sync_msg(sync_msg_t* msg);
void handle_blink_msg(blink_msg_t* msg);
void handle_blink_report_msg(blink_report_msg_t* msg);
void pdecTdoaStart(tdoa_role_t role);

