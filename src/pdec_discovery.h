#pragma once

#include <stdbool.h>

#include "pdec_common.h"

int pdec_discovery_start(void);
int pdec_discovery_send_now(void);
void handle_discovery_msg(discovery_msg_t *msg);

/* Disabling gates both periodic submissions and already queued discoveries.
 * Reception of discovery frames remains enabled. Setting is volatile.
 */
void pdec_discovery_set_enabled(bool enabled);
bool pdec_discovery_is_enabled(void);
