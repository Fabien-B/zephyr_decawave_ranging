#pragma once

#include"pdec_common.h"
#include <zephyr/shell/shell.h>

enum ranging_monitor_scope {
    RANGING_MONITOR_SCOPE_LOCAL = 0,
    RANGING_MONITOR_SCOPE_ALL,
};

void ranging_init();
void handle_resp_msg(resp_msg_t* msg, bool double_sided);
void handle_poll_msg(poll_msg_t* msg, bool double_sided);
void handle_final_msg(final_msg_t* msg);
void send_poll(uint16_t dst_id, bool double_sided);

void start_monitor(const struct shell *shell, enum ranging_monitor_scope scope);
void stop_monitor(void);
