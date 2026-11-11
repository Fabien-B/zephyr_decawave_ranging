#pragma once

#include <stddef.h>
#include <stdint.h>

enum pdec_uart_mode {
    PDEC_UART_MODE_BINARY = 0,
    PDEC_UART_MODE_ASCII,
};

/* Each serial port has its own parser, shell and binary/ASCII mode. */
enum pdec_serial_port {
    PDEC_SERIAL_UART = 0,
    PDEC_SERIAL_USB,
    PDEC_SERIAL_PORT_COUNT,
};

int pdec_uart_protocol_init(void);
enum pdec_uart_mode pdec_uart_protocol_get_mode(enum pdec_serial_port port);
void pdec_uart_protocol_set_mode(enum pdec_serial_port port, enum pdec_uart_mode mode);

void pdec_uart_protocol_write(enum pdec_serial_port port, const uint8_t *data, size_t len);
void pdec_uart_protocol_write_str(enum pdec_serial_port port, const char *str);
int pdec_uart_protocol_write_binary_frame(enum pdec_serial_port port, const uint8_t *payload, uint8_t len);

/* Publish events only to ports currently in binary mode. */
void pdec_uart_protocol_publish_binary_frame(const uint8_t *payload, uint8_t len);

/*
 * Default checksum is XOR(LEN, PAYLOAD...). Override this symbol if your wire
 * protocol uses a different 8-bit checksum.
 */
uint8_t pdec_uart_binary_checksum(const uint8_t *payload, uint8_t len);

void pdec_uart_on_binary_frame(enum pdec_serial_port port, const uint8_t *payload, uint8_t len, uint8_t chk);
