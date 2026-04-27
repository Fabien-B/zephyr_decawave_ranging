#include "uart_shell.h"

#include <errno.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include <zephyr/shell/shell.h>
#include <zephyr/sys/ring_buffer.h>

#include "pdec_common.h"
#include "pdec_initiator.h"
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

static int cmd_uart_send(const struct shell *sh, size_t argc, char **argv)
{
    uint8_t payload[PAYLOAD_FRAME_LEN_MAX];
    size_t len = 0;
    int err = 0;
    uint16_t dst_id;

    if (argc < 3) {
        shell_print(sh, "Usage: uart send <dst_id> <message>");
        return -EINVAL;
    }

    dst_id = (uint16_t)shell_strtoul(argv[1], 0, &err);
    if (err != 0) {
        shell_print(sh, "Invalid ID: %s", argv[1]);
        return err;
    }

    for (size_t i = 2; i < argc; i++) {
        size_t part_len = strlen(argv[i]);
        size_t extra = part_len + ((i > 2) ? 1U : 0U);

        if ((len + extra) > PAYLOAD_FRAME_LEN_MAX) {
            shell_print(sh, "Message too long: %u bytes max", PAYLOAD_FRAME_LEN_MAX);
            return -EMSGSIZE;
        }

        if (i > 2) {
            payload[len++] = ' ';
        }

        memcpy(&payload[len], argv[i], part_len);
        len += part_len;
    }

    send_data_msg(dst_id, payload, (uint8_t)len);
    shell_print(sh, "Sent %u byte(s) to 0x%04x", (unsigned int)len, dst_id);

    return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(sub_uart,
    SHELL_CMD(binary, NULL, "Switch UART back to binary mode.", cmd_uart_binary),
    SHELL_CMD(send, NULL, "Send an ASCII data message.", cmd_uart_send),
    SHELL_SUBCMD_SET_END
);

SHELL_CMD_REGISTER(uart, &sub_uart, "UART protocol commands.", NULL);




static int cmd_ranging_range(const struct shell *sh, size_t argc, char **argv)
{
    if(argc < 2) {
        shell_print(sh, "Usage: >ranging range 0x9e7a");
        return -EINVAL;
    }

    int err = 0;
    uint16_t dst = shell_strtoul(argv[1], 0, &err);

    if(err) {
        shell_print(sh, "Invalid ID: %s", argv[1]);
        return err;
    }

    double old_distance = get_distance(get_my_id(), dst);
    send_poll(dst, true);
    // TODO replace by event based trigger
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
    enum ranging_monitor_scope scope = RANGING_MONITOR_SCOPE_LOCAL;

    if (argc >= 2) {
        if (strcmp(argv[1], "local") == 0) {
            scope = RANGING_MONITOR_SCOPE_LOCAL;
        } else if (strcmp(argv[1], "all") == 0) {
            scope = RANGING_MONITOR_SCOPE_ALL;
        } else {
            shell_print(sh, "Usage: ranging monitor [local|all]");
            return -EINVAL;
        }
    }

    start_monitor(sh, scope);
    
    return 0;
}

static int cmd_ranging_list(const struct shell *sh, size_t argc, char **argv)
{
    ARG_UNUSED(argc);
    ARG_UNUSED(argv);

    shell_print(sh, "Known devices:");
    for (int i = 0; get_device_id(i) != 0; i++) {
        uint16_t id = get_device_id(i);
        shell_print(sh, " [%d] 0x%04x %s", i, id, (id == get_my_id()) ? "(me)" : "");
    }
    return 0;
}

static int cmd_ranging_get(const struct shell *sh, size_t argc, char **argv)
{
    if (argc < 2) {
        shell_print(sh, "Usage: ranging get <id>");
        return -EINVAL;
    }

    int err = 0;
    uint16_t dst = (uint16_t)shell_strtoul(argv[1], 0, &err);
    if (err) {
        shell_print(sh, "Invalid ID: %s", argv[1]);
        return err;
    }

    double dist = get_distance(get_my_id(), dst);
    if (dist < 0) {
        shell_print(sh, "No distance known for 0x%04x", dst);
    } else {
        shell_print(sh, "Last distance to 0x%04x: %.02fm", dst, dist);
    }

    return 0;
}

static int cmd_ranging_continuous_start(const struct shell *sh, size_t argc, char **argv)
{
    ARG_UNUSED(argc);
    ARG_UNUSED(argv);

    int ret = continuous_start();

    if (ret == -EALREADY) {
        shell_print(sh, "Continuous ranging is already running");
        return 0;
    }

    shell_print(sh, "Continuous ranging started");
    return 0;
}

static int cmd_ranging_continuous_stop(const struct shell *sh, size_t argc, char **argv)
{
    ARG_UNUSED(argc);
    ARG_UNUSED(argv);

    int ret = continuous_stop();

    if (ret == -EALREADY) {
        shell_print(sh, "Continuous ranging is already stopped");
        return 0;
    }

    shell_print(sh, "Continuous ranging stopping");
    return 0;
}

static int cmd_ranging_continuous_freq(const struct shell *sh, size_t argc, char **argv)
{
    if (argc < 2) {
        shell_print(sh, "Usage: ranging continuous freq <ms>");
        return -EINVAL;
    }

    int err = 0;
    uint32_t period_ms = shell_strtoul(argv[1], 0, &err);

    if (err != 0 || period_ms == 0) {
        shell_print(sh, "Invalid period: %s", argv[1]);
        return -EINVAL;
    }

    err = continuous_set_period_ms(period_ms);
    if (err != 0) {
        shell_print(sh, "Failed to set period");
        return err;
    }

    shell_print(sh, "Continuous ranging period set to %u ms", period_ms);
    return 0;
}

static int cmd_ranging_continuous_add(const struct shell *sh, size_t argc, char **argv)
{
    if (argc < 2) {
        shell_print(sh, "Usage: ranging continuous add <id>");
        return -EINVAL;
    }

    int err = 0;
    uint16_t id = (uint16_t)shell_strtoul(argv[1], 0, &err);

    if (err != 0) {
        shell_print(sh, "Invalid ID: %s", argv[1]);
        return err;
    }

    err = continuous_add_target(id);
    if (err == -EINVAL) {
        shell_print(sh, "Cannot add self ID 0x%04x", id);
        return err;
    }
    if (err == -EALREADY) {
        shell_print(sh, "0x%04x is already in the target list", id);
        return 0;
    }
    if (err == -ENOMEM) {
        shell_print(sh, "Target list is full");
        return err;
    }

    shell_print(sh, "Added 0x%04x to continuous targets", id);
    return 0;
}

static int cmd_ranging_continuous_remove(const struct shell *sh, size_t argc, char **argv)
{
    if (argc < 2) {
        shell_print(sh, "Usage: ranging continuous remove <id>");
        return -EINVAL;
    }

    int err = 0;
    uint16_t id = (uint16_t)shell_strtoul(argv[1], 0, &err);

    if (err != 0) {
        shell_print(sh, "Invalid ID: %s", argv[1]);
        return err;
    }

    err = continuous_remove_target(id);
    if (err == -EINVAL) {
        shell_print(sh, "Cannot remove self ID 0x%04x", id);
        return err;
    }
    if (err == -ENOENT) {
        shell_print(sh, "0x%04x is not in the target list", id);
        return err;
    }

    shell_print(sh, "Removed 0x%04x from continuous targets", id);
    return 0;
}

static int cmd_ranging_continuous_reset(const struct shell *sh, size_t argc, char **argv)
{
    ARG_UNUSED(argc);
    ARG_UNUSED(argv);

    continuous_reset_targets();
    shell_print(sh, "Continuous targets reset to all known devices");
    return 0;
}

static int cmd_ranging_continuous_list(const struct shell *sh, size_t argc, char **argv)
{
    ARG_UNUSED(argc);
    ARG_UNUSED(argv);

    uint16_t targets[PDEC_CONTINUOUS_TARGETS_MAX];
    size_t count = continuous_get_effective_targets(targets, ARRAY_SIZE(targets));
    const char *mode = continuous_get_target_mode() == CONTINUOUS_TARGET_MODE_ALL ? "all" : "custom";

    shell_print(sh, "Continuous ranging: %s", continuous_is_running() ? "running" : "stopped");
    shell_print(sh, "Mode: %s", mode);
    shell_print(sh, "Full-cycle period: %u ms", continuous_get_period_ms());
    shell_print(sh, "Targets (%u):", (unsigned int)count);

    if (count == 0) {
        shell_print(sh, " none");
        return 0;
    }

    for (size_t i = 0; i < count; i++) {
        shell_print(sh, " 0x%04x", targets[i]);
    }

    return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(sub_ranging_continuous,
    SHELL_CMD(start, NULL, "Start continuous ranging.", cmd_ranging_continuous_start),
    SHELL_CMD(stop, NULL, "Stop continuous ranging.", cmd_ranging_continuous_stop),
    SHELL_CMD(freq, NULL, "Set full-cycle period in milliseconds.", cmd_ranging_continuous_freq),
    SHELL_CMD(add, NULL, "Add a target ID to the custom target list.", cmd_ranging_continuous_add),
    SHELL_CMD(remove, NULL, "Remove a target ID from the custom target list.", cmd_ranging_continuous_remove),
    SHELL_CMD(reset, NULL, "Reset targets to all known devices.", cmd_ranging_continuous_reset),
    SHELL_CMD(list, NULL, "List continuous ranging status and targets.", cmd_ranging_continuous_list),
    SHELL_SUBCMD_SET_END
);

SHELL_STATIC_SUBCMD_SET_CREATE(sub_ranging,
    SHELL_CMD(range, NULL, "Perform ranging with target.", cmd_ranging_range),
    SHELL_CMD(get, NULL, "Get last known distance to a device.", cmd_ranging_get),
    SHELL_CMD(monitor, NULL, "Monitor ranges", cmd_ranging_monitor),
    SHELL_CMD(continuous, &sub_ranging_continuous, "Continuous ranging control.", NULL),
    SHELL_CMD(list, NULL, "List all devices in database", cmd_ranging_list),
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
