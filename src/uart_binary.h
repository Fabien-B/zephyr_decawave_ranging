#pragma once

#include <stdint.h>

enum pdec_binary_ranging_event_kind {
    PDEC_BINARY_RANGING_EVENT_SS = 0,
    PDEC_BINARY_RANGING_EVENT_DS = 1,
    PDEC_BINARY_RANGING_EVENT_IN = 2,
};

void pdec_uart_binary_publish_ranging_event(uint8_t kind, uint16_t src_id,
                                            uint16_t dst_id, double distance);
void pdec_uart_binary_publish_data_event(uint16_t src_id, const uint8_t *data,
                                         uint8_t len);
