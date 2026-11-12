#pragma once

#include <stdbool.h>
#include <stdint.h>

/* Internal request identity, independent of the on-air 8-bit ranging token.
 * Identity zero is reserved for background ranging without a waiting caller.
 */
bool pdec_range_request_is_pending(uint32_t request_id);
void pdec_range_request_complete(uint32_t request_id, int result, double distance);
