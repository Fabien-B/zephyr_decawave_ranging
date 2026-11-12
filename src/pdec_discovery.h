#pragma once

#include <stdbool.h>

#include "pdec_common.h"

int pdec_discovery_start(void);
int pdec_discovery_send_now(void);
void handle_discovery_msg(discovery_msg_t *msg);
