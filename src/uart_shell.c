#include "uart_shell.h"

#include <errno.h>
#include <stdbool.h>
#include <string.h>

#include <zephyr/shell/shell.h>
#include <zephyr/sys/ring_buffer.h>

#include "uart_protocol.h"
#include "pdec_ranging.h"
#include "pdec_database.h"

#define PDEC_SHELL_RX_RING_SIZE 256

struct pdec_shell_transport_ctx {
    struct ring_buf rx_ring;
    shell_transport_handler_t evt_handler;
    void *evt_context;
    bool initialized;
    uint8_t rx_buf[PDEC_SHELL_RX_RING_SIZE];
};

static int shell_transport_init(const struct shell_transport *transport,
                                const void *config,
                                shell_transport_handler_t evt_handler,
                                void *context)
{
    struct pdec_shell_transport_ctx *ctx = transport->ctx;

    ARG_UNUSED(config);

    if (ctx->initialized) {
        return -EALREADY;
    }

    ring_buf_init(&ctx->rx_ring, sizeof(ctx->rx_buf), ctx->rx_buf);
    ctx->evt_handler = evt_handler;
    ctx->evt_context = context;
    ctx->initialized = true;

    return 0;
}

static int shell_transport_uninit(const struct shell_transport *transport)
{
    struct pdec_shell_transport_ctx *ctx = transport->ctx;

    if (!ctx->initialized) {
        return -ENODEV;
    }

    ctx->initialized = false;
    ctx->evt_handler = NULL;
    ctx->evt_context = NULL;
    ring_buf_reset(&ctx->rx_ring);

    return 0;
}

static int shell_transport_enable(const struct shell_transport *transport, bool blocking_tx)
{
    struct pdec_shell_transport_ctx *ctx = transport->ctx;

    ARG_UNUSED(blocking_tx);

    return ctx->initialized ? 0 : -ENODEV;
}

static int shell_transport_write(const struct shell_transport *transport,
                                 const void *data, size_t length, size_t *cnt)
{
    struct pdec_shell_transport_ctx *ctx = transport->ctx;

    if (!ctx->initialized) {
        *cnt = 0;
        return -ENODEV;
    }

    pdec_uart_protocol_write(data, length);
    *cnt = length;
    return 0;
}

static int shell_transport_read(const struct shell_transport *transport,
                                void *data, size_t length, size_t *cnt)
{
    struct pdec_shell_transport_ctx *ctx = transport->ctx;

    if (!ctx->initialized) {
        *cnt = 0;
        return -ENODEV;
    }

    *cnt = ring_buf_get(&ctx->rx_ring, data, length);
    return 0;
}

static const struct shell_transport_api pdec_shell_transport_api = {
    .init = shell_transport_init,
    .uninit = shell_transport_uninit,
    .enable = shell_transport_enable,
    .write = shell_transport_write,
    .read = shell_transport_read,
};

static struct pdec_shell_transport_ctx pdec_shell_transport_ctx;
static struct shell_transport pdec_shell_transport = {
    .api = &pdec_shell_transport_api,
    .ctx = &pdec_shell_transport_ctx,
};

SHELL_DEFINE(pdec_shell, "pdec> ", &pdec_shell_transport, 0, 0, SHELL_FLAG_OLF_CRLF);

static int cmd_uart_binary(const struct shell *sh, size_t argc, char **argv)
{
    ARG_UNUSED(argc);
    ARG_UNUSED(argv);

    shell_print(sh, "Switching to binary mode");
    pdec_uart_protocol_set_mode(PDEC_UART_MODE_BINARY);
    return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(sub_uart,
    SHELL_CMD(binary, NULL, "Switch UART back to binary mode.", cmd_uart_binary),
    SHELL_SUBCMD_SET_END
);

SHELL_CMD_REGISTER(uart, &sub_uart, "UART protocol commands.", NULL);




static int cmd_ranging_range(const struct shell *sh, size_t argc, char **argv)
{
    if(argc < 2) {
        shell_print(sh, "Usage: >ranging range 0x9e7a");
        return -EDESTADDRREQ;
    }

    int err = 0;
    uint16_t dst = shell_strtoul(argv[1], 0, &err);

    if(err) {
        shell_print(sh, "Error: %d", err);
        return err;
    }

    double old_distance = get_distance(get_my_id(), dst);
    send_poll(dst, true);
    k_msleep(20);
    double new_distance = get_distance(get_my_id(), dst);
    if(new_distance == old_distance) {
        shell_print(sh, "Ranging with 0x%04x failed !", dst);
    } else {
        shell_print(sh, "0x%04x: %.02fm", dst, new_distance);
    }
    
    return 0;
}

static int cmd_ranging_monitor(const struct shell *sh, size_t argc, char **argv)
{
    ARG_UNUSED(argc);
    ARG_UNUSED(argv);
    
    start_monitor(sh);
    
    return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(sub_ranging,
    SHELL_CMD(range, NULL, "Perform ranging with target.", cmd_ranging_range),
    SHELL_CMD(monitor, NULL, "Monitor ranges", cmd_ranging_monitor),
    SHELL_SUBCMD_SET_END
);

SHELL_CMD_REGISTER(ranging, &sub_ranging, "UART protocol commands.", NULL);




int pdec_uart_shell_init(void)
{
    static bool initialized;
    static const struct shell_backend_config_flags cfg_flags =
        SHELL_DEFAULT_BACKEND_CONFIG_FLAGS;

    if (initialized) {
        return 0;
    }

    int ret = shell_init(&pdec_shell, NULL, cfg_flags, false, 0);
    if (ret == 0) {
        initialized = true;
    }

    return ret;
}

int pdec_uart_shell_start(void)
{
    return shell_start(&pdec_shell);
}

int pdec_uart_shell_stop(void)
{
    return shell_stop(&pdec_shell);
}

void pdec_uart_shell_feed_char(uint8_t c)
{
    uint32_t written;

    if (!pdec_shell_transport_ctx.initialized || pdec_shell_transport_ctx.evt_handler == NULL) {
        return;
    }

    written = ring_buf_put(&pdec_shell_transport_ctx.rx_ring, &c, 1);
    if (written == 0) {
        return;
    }

    pdec_shell_transport_ctx.evt_handler(SHELL_TRANSPORT_EVT_RX_RDY,
                                         pdec_shell_transport_ctx.evt_context);
}

const struct shell *pdec_uart_shell_backend_get(void)
{
    return &pdec_shell;
}
