#include "uart_shell.h"

#include <errno.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include <zephyr/shell/shell.h>
#include <zephyr/sys/ring_buffer.h>

#include "pdec_common.h"
#include "pdec_initiator.h"
#include "pdec_discovery.h"
#include "pdec_tdoa.h"
#include "uart_protocol.h"
#include "pdec_ranging.h"
#include "pdec_radio.h"
#include "pdec_database.h"

#define PDEC_SHELL_RX_RING_SIZE 256

struct pdec_shell_transport_ctx {
    enum pdec_serial_port port;
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

    pdec_uart_protocol_write(ctx->port, data, length);
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

static struct pdec_shell_transport_ctx transport_contexts[PDEC_SERIAL_PORT_COUNT] = {
    [PDEC_SERIAL_UART] = {.port = PDEC_SERIAL_UART},
    [PDEC_SERIAL_USB] = {.port = PDEC_SERIAL_USB},
};
static struct shell_transport uart_transport = {
    .api = &pdec_shell_transport_api,
    .ctx = &transport_contexts[PDEC_SERIAL_UART],
};
static struct shell_transport usb_transport = {
    .api = &pdec_shell_transport_api,
    .ctx = &transport_contexts[PDEC_SERIAL_USB],
};

SHELL_DEFINE(pdec_shell, "pdec> ", &uart_transport, 0, 0, SHELL_FLAG_OLF_CRLF);
SHELL_DEFINE(pdec_usb_shell, "pdec> ", &usb_transport, 0, 0, SHELL_FLAG_OLF_CRLF);

static const struct shell *const shells[PDEC_SERIAL_PORT_COUNT] = {
    &pdec_shell, &pdec_usb_shell,
};

static int cmd_uart_binary(const struct shell *sh, size_t argc, char **argv)
{
    ARG_UNUSED(argc);
    ARG_UNUSED(argv);

    shell_print(sh, "Switching to binary mode");
    enum pdec_serial_port port = (sh == &pdec_usb_shell) ? PDEC_SERIAL_USB : PDEC_SERIAL_UART;
    pdec_uart_protocol_set_mode(port, PDEC_UART_MODE_BINARY);
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

static int cmd_uart_reports(const struct shell *sh, size_t argc, char **argv)
{
    if (argc > 2) {
        shell_error(sh, "Usage: uart reports [active|all|status]");
        return -EINVAL;
    }
    if (argc == 2) {
        if (strcmp(argv[1], "active") == 0) {
            pdec_uart_protocol_set_reports_follow_commands(true);
        } else if (strcmp(argv[1], "all") == 0) {
            pdec_uart_protocol_set_reports_follow_commands(false);
        } else if (strcmp(argv[1], "status") != 0) {
            shell_error(sh, "Usage: uart reports [active|all|status]");
            return -EINVAL;
        }
    }
    shell_print(sh, "Binary reports: %s",
                pdec_uart_protocol_reports_follow_commands() ? "active (latest binary command port)" : "all binary ports");
    return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(sub_uart,
    SHELL_CMD(binary, NULL, "Switch this port back to binary mode.", cmd_uart_binary),
    SHELL_CMD(send, NULL, "Send an ASCII data message.", cmd_uart_send),
    SHELL_CMD(reports, NULL, "Route binary reports: active|all|status.", cmd_uart_reports),
    SHELL_SUBCMD_SET_END
);

SHELL_CMD_REGISTER(uart, &sub_uart, "Serial protocol commands.", NULL);




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

    double distance;
    int ret = request_range(dst, true, &distance, K_MSEC(100));
    if(ret != 0) {
        shell_print(sh, "Ranging with 0x%04x failed !", dst);
    } else {
        shell_print(sh, "0x%04x: %.02fm", dst, distance);
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

static const char *tdoa_role_label(uint8_t role)
{
    switch (role) {
    case TDOA_ROLE_NONE:
        return "none";
    case TDOA_ROLE_BEACON:
        return "beacon";
    case TDOA_ROLE_BLINKER:
        return "blinker";
    case TDOA_ROLE_MASTER:
        return "master";
    default:
        return "unknown";
    }
}

static int cmd_ranging_list(const struct shell *sh, size_t argc, char **argv)
{
    ARG_UNUSED(argc);
    ARG_UNUSED(argv);

    shell_print(sh, "Known devices:");
    for (int i = 0; get_device_id(i) != 0; i++) {
        uint16_t id = get_device_id(i);
        shell_print(sh, " [%d] 0x%04x role=%s %s", i, id,
                    tdoa_role_label(get_device_tdoa_role(id)),
                    (id == get_my_id()) ? "(me)" : "");
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

static int cmd_ranging_discovery(const struct shell *sh, size_t argc, char **argv)
{
    if (argc > 2) {
        shell_error(sh, "Usage: ranging discovery [on|off|status]");
        return -EINVAL;
    }
    if (argc == 2) {
        if (strcmp(argv[1], "on") == 0) {
            pdec_discovery_set_enabled(true);
        } else if (strcmp(argv[1], "off") == 0) {
            pdec_discovery_set_enabled(false);
        } else if (strcmp(argv[1], "status") != 0) {
            shell_error(sh, "Usage: ranging discovery [on|off|status]");
            return -EINVAL;
        }
    }
    shell_print(sh, "Discovery transmission: %s",
                pdec_discovery_is_enabled() ? "on" : "off");
    return 0;
}

static void print_timing_metric(const struct shell *sh, const char *name,
                                const struct pdec_radio_timing_metric *metric)
{
    if (metric->count == 0) { return; }
    shell_print(sh, "  %s min/avg/max: %d/%lld/%d us (n=%u)", name,
                (int)metric->min_us, (long long)(metric->total_us / metric->count),
                (int)metric->max_us, metric->count);
}

static K_MUTEX_DEFINE(timing_report_lock);

static int cmd_ranging_timing(const struct shell *sh, size_t argc, char **argv)
{
    if (argc > 2) {
        shell_error(sh, "Usage: ranging timing [on|off|reset|status]");
        return -EINVAL;
    }
    if (argc == 2) {
        if (strcmp(argv[1], "on") == 0) {
            pdec_radio_timing_enable(true);
        } else if (strcmp(argv[1], "off") == 0) {
            pdec_radio_timing_enable(false);
        } else if (strcmp(argv[1], "reset") == 0) {
            pdec_radio_timing_reset();
        } else if (strcmp(argv[1], "status") != 0) {
            shell_error(sh, "Usage: ranging timing [on|off|reset|status]");
            return -EINVAL;
        }
    }

    /* Keep the detailed snapshot off the small shell stack. Serialize readers
     * on UART/USB without holding the radio statistics spinlock while printing.
     */
    static struct pdec_radio_timing_stats stats[PDEC_TIMING_PATH_COUNT];
    k_mutex_lock(&timing_report_lock, K_FOREVER);
    pdec_radio_timing_snapshot(stats);
    static const char *const stages[PDEC_TIMING_STAGE_COUNT] = {
        "RX callback->TX entry", "TX validation", "SYS_TIME read (existing)",
        "forcetrxoff (if RX active)", "clear status (if RX active)", "write TX data",
        "write frame control (if changed)", "set delayed TX time", "starttx incl. checks",
    };
    shell_print(sh, "RX/TX timing: %s (local elapsed timer + existing DW anchor; no added SPI)",
                pdec_radio_timing_enabled() ? "on" : "off");
    for (int i = 0; i < PDEC_TIMING_PATH_COUNT; i++) {
        const struct pdec_radio_timing_stats *s = &stats[i];
        shell_print(sh, "%s: attempts=%u samples=%u start_failures=%u expired=%u write_failures=%u",
                    i == PDEC_TIMING_RESPONSE ? "RESPONSE" : "FINAL",
                    s->attempts, s->samples, s->start_failures, s->expired, s->write_failures);
        print_timing_metric(sh, "Observed peer turnaround (TX->RX)", &s->peer_turnaround);
        if (s->attempts != 0) {
            print_timing_metric(sh, "RX->DW anchor at TX entry", &s->rx_to_entry);
            for (int j = 0; j < PDEC_TIMING_STAGE_COUNT; j++) {
                print_timing_metric(sh, stages[j], &s->stage[j]);
            }
        }
        if (s->samples == 0) {
            continue;
        }
        print_timing_metric(sh, "RX->TX command lower bound", &s->rx_request_lower);
        print_timing_metric(sh, "RX->TX command upper bound", &s->rx_request_upper);
        print_timing_metric(sh, "TX command lead lower bound", &s->lead_lower);
        print_timing_metric(sh, "TX command lead upper bound", &s->lead_upper);
    }
    k_mutex_unlock(&timing_report_lock);
    return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(sub_ranging,
    SHELL_CMD(timing, NULL, "Measure RX to delayed-TX request: on|off|reset|status.", cmd_ranging_timing),
    SHELL_CMD(discovery, NULL, "Enable/disable discovery transmissions: on|off|status.", cmd_ranging_discovery),
    SHELL_CMD(range, NULL, "Perform ranging with target.", cmd_ranging_range),
    SHELL_CMD(get, NULL, "Get last known distance to a device.", cmd_ranging_get),
    SHELL_CMD(monitor, NULL, "Monitor ranges", cmd_ranging_monitor),
    SHELL_CMD(continuous, &sub_ranging_continuous, "Continuous ranging control.", NULL),
    SHELL_CMD(list, NULL, "List all devices in database", cmd_ranging_list),
    SHELL_SUBCMD_SET_END
);

SHELL_CMD_REGISTER(ranging, &sub_ranging, "Serial protocol commands.", NULL);

static int cmd_tdoa_monitor(const struct shell *sh, size_t argc, char **argv)
{
    ARG_UNUSED(argc);
    ARG_UNUSED(argv);

    start_tdoa_monitor(sh);
    return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(sub_tdoa,
    SHELL_CMD(monitor, NULL, "Monitor TDOA reports.", cmd_tdoa_monitor),
    SHELL_SUBCMD_SET_END
);

SHELL_CMD_REGISTER(tdoa, &sub_tdoa, "TDOA commands.", NULL);




int pdec_uart_shell_init(enum pdec_serial_port port)
{
    static bool initialized[PDEC_SERIAL_PORT_COUNT];
    static const struct shell_backend_config_flags cfg_flags =
        SHELL_DEFAULT_BACKEND_CONFIG_FLAGS;

    if (initialized[port]) {
        return 0;
    }

    int ret = shell_init(shells[port], NULL, cfg_flags, false, 0);
    if (ret == 0) {
        initialized[port] = true;
    }

    return ret;
}

int pdec_uart_shell_start(enum pdec_serial_port port)
{
    return shell_start(shells[port]);
}

int pdec_uart_shell_stop(enum pdec_serial_port port)
{
    return shell_stop(shells[port]);
}

void pdec_uart_shell_feed_char(enum pdec_serial_port port, uint8_t c)
{
    uint32_t written;
    struct pdec_shell_transport_ctx *ctx = &transport_contexts[port];

    if (!ctx->initialized || ctx->evt_handler == NULL) {
        return;
    }

    written = ring_buf_put(&ctx->rx_ring, &c, 1);
    if (written == 0) {
        return;
    }

    ctx->evt_handler(SHELL_TRANSPORT_EVT_RX_RDY, ctx->evt_context);
}

const struct shell *pdec_uart_shell_backend_get(enum pdec_serial_port port)
{
    return shells[port];
}
