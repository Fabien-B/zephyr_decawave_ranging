#pragma once

#include <stdbool.h>

#include <zephyr/shell/shell.h>

#include "pdec_common.h"
#include "pdec_database.h"

void tdoa_init(void);
void send_sync();
void handle_sync_msg(sync_msg_t* msg);
void handle_blink_msg(blink_msg_t* msg);
void handle_blink_report_msg(blink_report_msg_t* msg);
tdoa_role_t pdecTdoaGetRole(void);
void pdecTdoaSetRole(tdoa_role_t role);
void pdecTdoaStart(void);
void start_tdoa_monitor(const struct shell *shell);
void stop_tdoa_monitor(void);
