#pragma once

#include"pdec_common.h"
#include <zephyr/shell/shell.h>

void ranging_init();
void handle_resp_msg(resp_msg_t* msg, bool double_sided);
void handle_poll_msg(poll_msg_t* msg, bool double_sided);
void handle_final_msg(final_msg_t* msg);
void send_poll(uint16_t dst_id, bool double_sided);

void start_monitor(const struct shell *shell);
void stop_monitor();