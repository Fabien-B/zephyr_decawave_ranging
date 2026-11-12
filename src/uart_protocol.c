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
#define USB_DEVICE_NODE DT_NODELABEL(pdec_usb_uart)
#define USB_TX_RING_SIZE 2048
#define UART_WORK_STACK_SIZE 2048
#define UART_WORK_PRIORITY 5

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
    enum pdec_serial_port port;
    struct ring_buf tx_ring;
    struct k_spinlock tx_lock;
    uint8_t tx_storage[USB_TX_RING_SIZE];
    struct ring_buf rx_ring;
    struct k_work rx_work;
    enum pdec_uart_mode mode;
    struct binary_parser parser;
    uint8_t ring_storage[UART_PROTOCOL_RING_SIZE];
    int64_t last_stray_newline_ms;
    uint32_t dropped_bytes;
    bool last_byte_was_cr;
};

static struct uart_protocol_ctx protocols[PDEC_SERIAL_PORT_COUNT];

static K_MUTEX_DEFINE(uart_tx_lock);
/* Protected by uart_tx_lock; COUNT means no command port has been selected. */
static enum pdec_serial_port report_port = PDEC_SERIAL_PORT_COUNT;
static bool reports_follow_commands = true;
/* Parsing can emit polled UART replies. Keep it preemptible and off the
 * cooperative system workqueue, so it cannot hold up the radio owner.
 */
K_THREAD_STACK_DEFINE(uart_work_stack, UART_WORK_STACK_SIZE);
static struct k_work_q uart_work_queue;

static int uart_protocol_write_unlocked(struct uart_protocol_ctx *ctx,
                                       const uint8_t *data, size_t len)
{
    if (ctx->uart == NULL) {
        return -ENODEV;
    }

    if (ctx->port == PDEC_SERIAL_USB) {
        uint32_t dtr = 0;

        uart_line_ctrl_get(ctx->uart, UART_LINE_CTRL_DTR, &dtr);
        k_spinlock_key_t key = k_spin_lock(&ctx->tx_lock);
        if (!dtr) {
            ring_buf_reset(&ctx->tx_ring);
            k_spin_unlock(&ctx->tx_lock, key);
            return -ENOTCONN;
        }
        /* Queue complete writes: congestion must never truncate a frame. */
        if (ring_buf_space_get(&ctx->tx_ring) < len) {
            k_spin_unlock(&ctx->tx_lock, key);
            return -ENOSPC;
        }
        ring_buf_put(&ctx->tx_ring, data, len);
        k_spin_unlock(&ctx->tx_lock, key);
        uart_irq_tx_enable(ctx->uart);
        return 0;
    }

    for (size_t i = 0; i < len; ++i) {
        uart_poll_out(ctx->uart, data[i]);
    }
    return 0;
}

static void binary_parser_reset(struct uart_protocol_ctx *ctx)
{
    ctx->parser.state = BINARY_RX_SYNC_1;
    ctx->parser.len = 0;
    ctx->parser.index = 0;
}

static void set_mode(struct uart_protocol_ctx *ctx, enum pdec_uart_mode mode)
{
    int ret;

    if (mode == PDEC_UART_MODE_ASCII) {
        /* uart binary can call shell_stop() from a shell command handler.
         * Zephyr's command loop sets the shell ACTIVE again on handler return.
         * Restore INITIALIZED here, outside that handler, before restarting.
         * Stopping an already stopped shell returns -ENOTSUP.
         */
        ret = pdec_uart_shell_stop(ctx->port);
        if (ret < 0 && ret != -ENOTSUP) {
            return;
        }
        ret = pdec_uart_shell_start(ctx->port);
        if (ret < 0 && ret != -EALREADY) {
            return;
        }
    } else {
        ret = pdec_uart_shell_stop(ctx->port);
        ARG_UNUSED(ret);
    }

    ctx->mode = mode;
    ctx->last_stray_newline_ms = -1;
    ctx->last_byte_was_cr = false;
    binary_parser_reset(ctx);

    if (mode != PDEC_UART_MODE_ASCII) {
        pdec_uart_protocol_write_str(ctx->port, "\r\nBinary mode enabled\r\n");
    }
}

static void maybe_switch_to_ascii(struct uart_protocol_ctx *ctx)
{
    int64_t now = k_uptime_get();

    if ((ctx->last_stray_newline_ms >= 0) &&
        ((now - ctx->last_stray_newline_ms) < UART_PROTOCOL_MODE_SWITCH_MS)) {
        set_mode(ctx, PDEC_UART_MODE_ASCII);
        return;
    }

    ctx->last_stray_newline_ms = now;
}

static bool handle_binary_mode_newline(struct uart_protocol_ctx *ctx, uint8_t c)
{
    if (ctx->parser.state != BINARY_RX_SYNC_1 &&
        ctx->parser.state != BINARY_RX_SYNC_2) {
        ctx->last_byte_was_cr = false;
        return false;
    }

    if (c == '\r') {
        maybe_switch_to_ascii(ctx);
        ctx->last_byte_was_cr = true;
        return true;
    }

    if (c == '\n') {
        if (ctx->last_byte_was_cr) {
            ctx->last_byte_was_cr = false;
            return true;
        }

        maybe_switch_to_ascii(ctx);
        return true;
    }

    ctx->last_byte_was_cr = false;
    return false;
}

static void handle_binary_frame_byte(struct uart_protocol_ctx *ctx, uint8_t c)
{
    struct binary_parser *parser = &ctx->parser;

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
            if (parser->len != 0) {
                k_mutex_lock(&uart_tx_lock, K_FOREVER);
                report_port = ctx->port;
                k_mutex_unlock(&uart_tx_lock);
            }
            pdec_uart_on_binary_frame(ctx->port, parser->payload, parser->len, c);
        }
        binary_parser_reset(ctx);
        break;
    }
}

static void process_byte(struct uart_protocol_ctx *ctx, uint8_t c)
{
    if (ctx->mode == PDEC_UART_MODE_ASCII) {
        pdec_uart_shell_feed_char(ctx->port, c);
        return;
    }

    if (handle_binary_mode_newline(ctx, c)) {
        return;
    }

    handle_binary_frame_byte(ctx, c);
}

static void uart_protocol_work_handler(struct k_work *work)
{
    struct uart_protocol_ctx *ctx = CONTAINER_OF(work, struct uart_protocol_ctx, rx_work);

    uint8_t buf[32];
    uint32_t count;

    do {
        count = ring_buf_get(&ctx->rx_ring, buf, sizeof(buf));
        for (uint32_t i = 0; i < count; ++i) {
            process_byte(ctx, buf[i]);
        }
    } while (count > 0);

    ctx->dropped_bytes = 0;
}

static void uart_protocol_isr(const struct device *dev, void *user_data)
{
    struct uart_protocol_ctx *ctx = user_data;

    if (!uart_irq_update(dev)) {
        return;
    }

    if (ctx->port == PDEC_SERIAL_USB && uart_irq_tx_ready(dev)) {
        k_spinlock_key_t key = k_spin_lock(&ctx->tx_lock);
        uint8_t *data;
        uint32_t count = ring_buf_get_claim(&ctx->tx_ring, &data, USB_TX_RING_SIZE);
        int sent = count ? uart_fifo_fill(dev, data, count) : 0;

        ring_buf_get_finish(&ctx->tx_ring, sent > 0 ? sent : 0);
        if (ring_buf_is_empty(&ctx->tx_ring)) {
            uart_irq_tx_disable(dev);
        }
        k_spin_unlock(&ctx->tx_lock, key);
    }

    if (!uart_irq_rx_ready(dev)) {
        return;
    }

    while (1) {
        uint8_t buf[16];
        int read = uart_fifo_read(ctx->uart, buf, sizeof(buf));

        if (read <= 0) {
            break;
        }

        uint32_t written = ring_buf_put(&ctx->rx_ring, buf, read);
        if (written < (uint32_t)read) {
            ctx->dropped_bytes += (uint32_t)(read - written);
        }
    }

    k_work_submit_to_queue(&uart_work_queue, &ctx->rx_work);
}

int pdec_uart_protocol_init(void)
{
    k_work_queue_start(&uart_work_queue, uart_work_stack,
                       K_THREAD_STACK_SIZEOF(uart_work_stack), UART_WORK_PRIORITY, NULL);
    const struct device *devices[PDEC_SERIAL_PORT_COUNT] = {
        DEVICE_DT_GET(UART_DEVICE_NODE),
        DEVICE_DT_GET(USB_DEVICE_NODE),
    };

    for (enum pdec_serial_port port = 0; port < PDEC_SERIAL_PORT_COUNT; ++port) {
        struct uart_protocol_ctx *ctx = &protocols[port];

        if (!device_is_ready(devices[port])) {
            return -ENODEV;
        }
        ctx->port = port;
        ctx->last_stray_newline_ms = -1;
        ring_buf_init(&ctx->rx_ring, sizeof(ctx->ring_storage), ctx->ring_storage);
        ring_buf_init(&ctx->tx_ring, sizeof(ctx->tx_storage), ctx->tx_storage);
        k_work_init(&ctx->rx_work, uart_protocol_work_handler);
        binary_parser_reset(ctx);
        ctx->mode = PDEC_UART_MODE_BINARY;

        int ret = pdec_uart_shell_init(port);
        if (ret < 0) {
            return ret;
        }
        ret = uart_irq_callback_user_data_set(devices[port], uart_protocol_isr, ctx);
        if (ret < 0) {
            return ret;
        }
        ctx->uart = devices[port];
        uart_irq_rx_enable(ctx->uart);
    }
    return 0;
}

enum pdec_uart_mode pdec_uart_protocol_get_mode(enum pdec_serial_port port)
{
    return protocols[port].mode;
}

void pdec_uart_protocol_set_mode(enum pdec_serial_port port, enum pdec_uart_mode mode)
{
    struct uart_protocol_ctx *ctx = &protocols[port];

    if (ctx->mode != mode) {
        set_mode(ctx, mode);
    }
}

void pdec_uart_protocol_write(enum pdec_serial_port port, const uint8_t *data, size_t len)
{
    k_mutex_lock(&uart_tx_lock, K_FOREVER);
    uart_protocol_write_unlocked(&protocols[port], data, len);
    k_mutex_unlock(&uart_tx_lock);
}

void pdec_uart_protocol_write_str(enum pdec_serial_port port, const char *str)
{
    pdec_uart_protocol_write(port, (const uint8_t *)str, strlen(str));
}

/* Caller holds uart_tx_lock, including report selection and transmission. */
static int write_binary_frame_unlocked(enum pdec_serial_port port,
                                       const uint8_t *payload, uint8_t len)
{
    uint8_t frame[UART_PROTOCOL_PAYLOAD_MAX + 4] = {
        UART_PROTOCOL_SYNC_BYTE, UART_PROTOCOL_SYNC_BYTE, len,
    };

    memcpy(&frame[3], payload, len);
    frame[3 + len] = pdec_uart_binary_checksum(payload, len);
    return uart_protocol_write_unlocked(&protocols[port], frame, len + 4);
}

int pdec_uart_protocol_write_binary_frame(enum pdec_serial_port port,
                                         const uint8_t *payload, uint8_t len)
{
    k_mutex_lock(&uart_tx_lock, K_FOREVER);
    int ret = write_binary_frame_unlocked(port, payload, len);
    k_mutex_unlock(&uart_tx_lock);
    return ret;
}

void pdec_uart_protocol_publish_binary_frame(const uint8_t *payload, uint8_t len)
{
    k_mutex_lock(&uart_tx_lock, K_FOREVER);
    for (enum pdec_serial_port port = 0; port < PDEC_SERIAL_PORT_COUNT; ++port) {
        if (reports_follow_commands &&
            report_port != PDEC_SERIAL_PORT_COUNT && report_port != port) {
            continue;
        }
        if (pdec_uart_protocol_get_mode(port) == PDEC_UART_MODE_BINARY) {
            (void)write_binary_frame_unlocked(port, payload, len);
        }
    }
    k_mutex_unlock(&uart_tx_lock);
}

void pdec_uart_protocol_set_reports_follow_commands(bool enabled)
{
    k_mutex_lock(&uart_tx_lock, K_FOREVER);
    reports_follow_commands = enabled;
    k_mutex_unlock(&uart_tx_lock);
}

bool pdec_uart_protocol_reports_follow_commands(void)
{
    k_mutex_lock(&uart_tx_lock, K_FOREVER);
    bool enabled = reports_follow_commands;
    k_mutex_unlock(&uart_tx_lock);
    return enabled;
}

__attribute__((weak)) uint8_t pdec_uart_binary_checksum(const uint8_t *payload, uint8_t len)
{
    uint8_t checksum = len;

    for (uint8_t i = 0; i < len; ++i) {
        checksum ^= payload[i];
    }

    return checksum;
}

__attribute__((weak)) void pdec_uart_on_binary_frame(enum pdec_serial_port port, const uint8_t *payload, uint8_t len, uint8_t chk)
{
    ARG_UNUSED(port);
    ARG_UNUSED(payload);
    ARG_UNUSED(len);
    ARG_UNUSED(chk);
}
