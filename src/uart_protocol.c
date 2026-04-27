#include "uart_protocol.h"

#include <errno.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/ring_buffer.h>
#include <zephyr/sys/util.h>

#include "app_print.h"
#include "uart_shell.h"

#define UART_DEVICE_NODE DT_CHOSEN(zephyr_console)

#define UART_PROTOCOL_SYNC_BYTE 0xFF
#define UART_PROTOCOL_MODE_SWITCH_MS 1000
#define UART_PROTOCOL_RING_SIZE 512
#define UART_PROTOCOL_PAYLOAD_MAX UINT8_MAX

BUILD_ASSERT(DT_NODE_HAS_STATUS(UART_DEVICE_NODE, okay), "Console UART is not available");

enum binary_rx_state {
    BINARY_RX_SYNC_1 = 0,
    BINARY_RX_SYNC_2,
    BINARY_RX_LEN,
    BINARY_RX_PAYLOAD,
    BINARY_RX_CHK,
};

struct binary_parser {
    enum binary_rx_state state;
    uint8_t len;
    uint8_t index;
    uint8_t payload[UART_PROTOCOL_PAYLOAD_MAX];
};

struct uart_protocol_ctx {
    const struct device *uart;
    struct ring_buf rx_ring;
    struct k_work rx_work;
    enum pdec_uart_mode mode;
    struct binary_parser parser;
    uint8_t ring_storage[UART_PROTOCOL_RING_SIZE];
    int64_t last_stray_newline_ms;
    uint32_t dropped_bytes;
    bool last_byte_was_cr;
};

static struct uart_protocol_ctx uart_protocol = {
    .last_stray_newline_ms = -1,
};

static void binary_parser_reset(void)
{
    uart_protocol.parser.state = BINARY_RX_SYNC_1;
    uart_protocol.parser.len = 0;
    uart_protocol.parser.index = 0;
}

static void set_mode(enum pdec_uart_mode mode)
{
    int ret;

    if (mode == PDEC_UART_MODE_ASCII) {
        ret = pdec_uart_shell_start();
        if (ret < 0 && ret != -EALREADY) {
            return;
        }
    } else {
        ret = pdec_uart_shell_stop();
        ARG_UNUSED(ret);
    }

    uart_protocol.mode = mode;
    uart_protocol.last_stray_newline_ms = -1;
    uart_protocol.last_byte_was_cr = false;
    binary_parser_reset();

    if (mode != PDEC_UART_MODE_ASCII) {
        pdec_uart_protocol_write_str("\r\nBinary mode enabled\r\n");
    }
}

static void maybe_switch_to_ascii(void)
{
    int64_t now = k_uptime_get();

    if ((uart_protocol.last_stray_newline_ms >= 0) &&
        ((now - uart_protocol.last_stray_newline_ms) < UART_PROTOCOL_MODE_SWITCH_MS)) {
        set_mode(PDEC_UART_MODE_ASCII);
        return;
    }

    uart_protocol.last_stray_newline_ms = now;
}

static bool handle_binary_mode_newline(uint8_t c)
{
    if (uart_protocol.parser.state != BINARY_RX_SYNC_1 &&
        uart_protocol.parser.state != BINARY_RX_SYNC_2) {
        uart_protocol.last_byte_was_cr = false;
        return false;
    }

    if (c == '\r') {
        maybe_switch_to_ascii();
        uart_protocol.last_byte_was_cr = true;
        return true;
    }

    if (c == '\n') {
        if (uart_protocol.last_byte_was_cr) {
            uart_protocol.last_byte_was_cr = false;
            return true;
        }

        maybe_switch_to_ascii();
        return true;
    }

    uart_protocol.last_byte_was_cr = false;
    return false;
}

static void handle_binary_frame_byte(uint8_t c)
{
    struct binary_parser *parser = &uart_protocol.parser;

    switch (parser->state) {
    case BINARY_RX_SYNC_1:
        if (c == UART_PROTOCOL_SYNC_BYTE) {
            parser->state = BINARY_RX_SYNC_2;
        }
        break;

    case BINARY_RX_SYNC_2:
        if (c == UART_PROTOCOL_SYNC_BYTE) {
            parser->state = BINARY_RX_LEN;
        } else {
            parser->state = BINARY_RX_SYNC_1;
            if (c == UART_PROTOCOL_SYNC_BYTE) {
                parser->state = BINARY_RX_SYNC_2;
            }
        }
        break;

    case BINARY_RX_LEN:
        parser->len = c;
        parser->index = 0;
        parser->state = (parser->len == 0) ? BINARY_RX_CHK : BINARY_RX_PAYLOAD;
        break;

    case BINARY_RX_PAYLOAD:
        parser->payload[parser->index++] = c;
        if (parser->index >= parser->len) {
            parser->state = BINARY_RX_CHK;
        }
        break;

    case BINARY_RX_CHK:
        if (c == pdec_uart_binary_checksum(parser->payload, parser->len)) {
            pdec_uart_on_binary_frame(parser->payload, parser->len, c);
        }
        binary_parser_reset();
        break;
    }
}

static void process_byte(uint8_t c)
{
    if (uart_protocol.mode == PDEC_UART_MODE_ASCII) {
        pdec_uart_shell_feed_char(c);
        return;
    }

    if (handle_binary_mode_newline(c)) {
        return;
    }

    handle_binary_frame_byte(c);
}

static void uart_protocol_work_handler(struct k_work *work)
{
    ARG_UNUSED(work);

    uint8_t buf[32];
    uint32_t count;

    do {
        count = ring_buf_get(&uart_protocol.rx_ring, buf, sizeof(buf));
        for (uint32_t i = 0; i < count; ++i) {
            process_byte(buf[i]);
        }
    } while (count > 0);

    uart_protocol.dropped_bytes = 0;
}

static void uart_protocol_isr(const struct device *dev, void *user_data)
{
    ARG_UNUSED(dev);
    ARG_UNUSED(user_data);

    if (!uart_irq_update(uart_protocol.uart) || !uart_irq_rx_ready(uart_protocol.uart)) {
        return;
    }

    while (1) {
        uint8_t buf[16];
        int read = uart_fifo_read(uart_protocol.uart, buf, sizeof(buf));

        if (read <= 0) {
            break;
        }

        uint32_t written = ring_buf_put(&uart_protocol.rx_ring, buf, read);
        if (written < (uint32_t)read) {
            uart_protocol.dropped_bytes += (uint32_t)(read - written);
        }
    }

    k_work_submit(&uart_protocol.rx_work);
}

int pdec_uart_protocol_init(void)
{
    uart_protocol.uart = DEVICE_DT_GET(UART_DEVICE_NODE);
    if (!device_is_ready(uart_protocol.uart)) {
        return -ENODEV;
    }

    ring_buf_init(&uart_protocol.rx_ring, sizeof(uart_protocol.ring_storage),
                  uart_protocol.ring_storage);
    k_work_init(&uart_protocol.rx_work, uart_protocol_work_handler);
    binary_parser_reset();
    uart_protocol.mode = PDEC_UART_MODE_BINARY;

    int ret = pdec_uart_shell_init();
    if (ret < 0) {
        return ret;
    }

    ret = uart_irq_callback_user_data_set(uart_protocol.uart, uart_protocol_isr, NULL);
    if (ret < 0) {
        return ret;
    }

    uart_irq_rx_enable(uart_protocol.uart);
    return 0;
}

enum pdec_uart_mode pdec_uart_protocol_get_mode(void)
{
    return uart_protocol.mode;
}

void pdec_uart_protocol_set_mode(enum pdec_uart_mode mode)
{
    if (uart_protocol.mode == mode) {
        return;
    }

    set_mode(mode);
}

void pdec_uart_protocol_write(const uint8_t *data, size_t len)
{
    if (uart_protocol.uart == NULL) {
        return;
    }

    for (size_t i = 0; i < len; ++i) {
        uart_poll_out(uart_protocol.uart, data[i]);
    }
}

void pdec_uart_protocol_write_str(const char *str)
{
    pdec_uart_protocol_write((const uint8_t *)str, strlen(str));
}

int pdec_uart_protocol_write_binary_frame(const uint8_t *payload, uint8_t len)
{
    uint8_t header[] = {
        UART_PROTOCOL_SYNC_BYTE,
        UART_PROTOCOL_SYNC_BYTE,
        len,
    };
    uint8_t chk = pdec_uart_binary_checksum(payload, len);

    pdec_uart_protocol_write(header, sizeof(header));
    pdec_uart_protocol_write(payload, len);
    pdec_uart_protocol_write(&chk, 1);

    return 0;
}

__attribute__((weak)) uint8_t pdec_uart_binary_checksum(const uint8_t *payload, uint8_t len)
{
    uint8_t checksum = len;

    for (uint8_t i = 0; i < len; ++i) {
        checksum ^= payload[i];
    }

    return checksum;
}

__attribute__((weak)) void pdec_uart_on_binary_frame(const uint8_t *payload, uint8_t len, uint8_t chk)
{
    ARG_UNUSED(payload);
    ARG_UNUSED(len);
    ARG_UNUSED(chk);
}
