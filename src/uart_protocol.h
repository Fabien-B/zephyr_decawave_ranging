#pragma once

#include <stddef.h>
#include <stdint.h>

enum pdec_uart_mode {
    PDEC_UART_MODE_BINARY = 0,
    PDEC_UART_MODE_ASCII,
};

int pdec_uart_protocol_init(void);
enum pdec_uart_mode pdec_uart_protocol_get_mode(void);
void pdec_uart_protocol_set_mode(enum pdec_uart_mode mode);

void pdec_uart_protocol_write(const uint8_t *data, size_t len);
void pdec_uart_protocol_write_str(const char *str);

/*
 * Default checksum is XOR(LEN, PAYLOAD...). Override this symbol if your wire
 * protocol uses a different 8-bit checksum.
 */
uint8_t pdec_uart_binary_checksum(const uint8_t *payload, uint8_t len);

void pdec_uart_on_binary_frame(const uint8_t *payload, uint8_t len, uint8_t chk);
