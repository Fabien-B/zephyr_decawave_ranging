#include"pdec_common.h"
#include "pdec_radio.h"
#include "pdec_database.h"
#include "pdec_ranging.h"
#include "uart_binary.h"

#include <errno.h>
#include <stdio.h>
#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/types.h>
#include <zephyr/random/random.h>
#include <zephyr/shell/shell.h>

#include <deca_device_api.h>
#include <deca_probe_interface.h>
#include <dw3000_hw.h>
#include <dw3000_spi.h>

#include "app_print.h"

static volatile bool monitor_stop_requested;
#define MONITOR_STACK_SIZE 1024
#define RESULT_STACK_SIZE 1024
#define PRIORITY 5
#define MONITOR_QUEUE_LEN 8
#define RESULT_QUEUE_LEN 8
static K_THREAD_STACK_DEFINE(monitor_stack, MONITOR_STACK_SIZE);
static K_THREAD_STACK_DEFINE(result_stack, RESULT_STACK_SIZE);
static struct k_thread monitor_thread;
static struct k_thread result_thread;
static bool result_thread_started;
static bool monitor_running;
static enum ranging_monitor_scope monitor_scope = RANGING_MONITOR_SCOPE_LOCAL;
#define ASCII_CTRL_C 0x03

enum ranging_event_kind {
    RANGING_EVENT_KIND_SS = 0,
    RANGING_EVENT_KIND_DS,
    RANGING_EVENT_KIND_IN,
    RANGING_EVENT_KIND_STOP,
};

struct ranging_event {
    uint16_t src_id;
    uint16_t dst_id;
    double distance;
    enum ranging_event_kind kind;
};

enum result_event_kind {
    RESULT_EVENT_RANGE,
    RESULT_EVENT_COM,
};

struct result_event {
    enum result_event_kind kind;
    union {
        struct ranging_event range;
        struct {
            uint16_t src_id;
            uint8_t len;
            uint8_t payload[PAYLOAD_FRAME_LEN_MAX];
        } com;
    };
};

K_MSGQ_DEFINE(monitor_queue, sizeof(struct ranging_event), MONITOR_QUEUE_LEN, 8);
K_MSGQ_DEFINE(result_queue, sizeof(struct result_event), RESULT_QUEUE_LEN, 8);
// delay between frame RX, and the delayed TX
#define POLL_RX_TO_RESP_TX_DLY_UUS 1500




extern uint8_t frame_seq_nb;

static union ranging_msg_u tx_msg;

static const char *ranging_event_kind_label(enum ranging_event_kind kind)
{
    switch (kind) {
    case RANGING_EVENT_KIND_SS:
        return "SS";
    case RANGING_EVENT_KIND_DS:
        return "DS";
    case RANGING_EVENT_KIND_IN:
        return "IN";
    case RANGING_EVENT_KIND_STOP:
    default:
        return "STOP";
    }
}

static uint8_t binary_ranging_event_kind(enum ranging_event_kind kind)
{
    switch (kind) {
    case RANGING_EVENT_KIND_SS:
        return PDEC_BINARY_RANGING_EVENT_SS;
    case RANGING_EVENT_KIND_DS:
        return PDEC_BINARY_RANGING_EVENT_DS;
    case RANGING_EVENT_KIND_IN:
    case RANGING_EVENT_KIND_STOP:
    default:
        return PDEC_BINARY_RANGING_EVENT_IN;
    }
}

static bool should_print_event(const struct ranging_event *event)
{
    if (monitor_scope == RANGING_MONITOR_SCOPE_ALL) {
        return true;
    }

    uint16_t my_id = get_my_id();

    return event->src_id == my_id || event->dst_id == my_id;
}

static void publish_ranging_event(uint16_t src_id, uint16_t dst_id, double distance,
                                  enum ranging_event_kind kind)
{
    struct result_event event = {
        .kind = RESULT_EVENT_RANGE,
        .range = {
            .src_id = src_id,
            .dst_id = dst_id,
            .distance = distance,
            .kind = kind,
        },
    };

    (void)k_msgq_put(&result_queue, &event, K_NO_WAIT);
}

static void enqueue_stop_event(void)
{
    struct ranging_event event = {
        .kind = RANGING_EVENT_KIND_STOP,
    };

    if (k_msgq_put(&monitor_queue, &event, K_NO_WAIT) != 0) {
        k_msgq_purge(&monitor_queue);
        (void)k_msgq_put(&monitor_queue, &event, K_NO_WAIT);
    }
}

static void process_results(void *p1, void *p2, void *p3)
{
    ARG_UNUSED(p1);
    ARG_UNUSED(p2);
    ARG_UNUSED(p3);

    struct result_event event;

    while (true) {
        if (k_msgq_get(&result_queue, &event, K_FOREVER) != 0) {
            continue;
        }

        if (event.kind == RESULT_EVENT_COM) {
            pdec_uart_binary_publish_data_event(event.com.src_id, event.com.payload,
                                                event.com.len);
            pdec_printf("[0x%04x] %.*s\n", event.com.src_id,
                        (int)event.com.len, event.com.payload);
            continue;
        }

        if (event.range.kind != RANGING_EVENT_KIND_STOP) {
            pdec_uart_binary_publish_ranging_event(binary_ranging_event_kind(event.range.kind),
                                                   event.range.src_id, event.range.dst_id,
                                                   event.range.distance);
        }

        (void)k_msgq_put(&monitor_queue, &event.range, K_NO_WAIT);
    }
}

static void start_result_thread(void)
{
    if (result_thread_started) {
        return;
    }

    k_thread_create(&result_thread, result_stack, K_THREAD_STACK_SIZEOF(result_stack),
                    process_results, NULL, NULL, NULL, PRIORITY, 0, K_NO_WAIT);
    result_thread_started = true;
}

void ranging_init() {
    tx_msg.header.frame_control = FRAME_CONTROL;
    tx_msg.header.pan_id = MY_PAN_ID;
    tx_msg.header.src_id = get_my_id();
    start_result_thread();
}

void publish_com_event(uint16_t src_id, const uint8_t *payload, uint8_t len)
{
    struct result_event event = {
        .kind = RESULT_EVENT_COM,
        .com = {
            .src_id = src_id,
            .len = len,
        },
    };

    if (event.com.len > PAYLOAD_FRAME_LEN_MAX) {
        event.com.len = PAYLOAD_FRAME_LEN_MAX;
    }

    memcpy(event.com.payload, payload, event.com.len);
    (void)k_msgq_put(&result_queue, &event, K_NO_WAIT);
}

void handle_resp_msg(resp_msg_t* msg, bool double_sided) {
    if (!pdec_radio_matches_response(msg, double_sided))
    {
        return;
    }

    uint64_t poll_tx_ts = get_tx_timestamp_u64();
    uint64_t resp_rx_ts = get_rx_timestamp_u64();
    uint32_t t_round = (uint32_t)(resp_rx_ts - poll_tx_ts);

    uint32_t t_reply_b = wrap32_diff(msg->resp_tx_ts, msg->poll_rx_ts);
    /* Compute time of flight and distance, using clock offset ratio to correct for differing local and remote clock rates */
    double tof = ((t_round - t_reply_b * get_clock_offset(msg->header.src_id)) / 2.0) * DWT_TIME_UNITS;
    double distance = tof * SPEED_OF_LIGHT;

    if(double_sided) {
        // en dtu (499.2MHz*128)
        uint32_t deltat = (POLL_RX_TO_RESP_TX_DLY_UUS * UUS_TO_DWT_TIME);

        /* Compute response message transmission time. See NOTE 7 below. */
        uint64_t final_tx_time = resp_rx_ts + deltat;

        /* Response TX timestamp is the transmission time we programmed plus the antenna delay. */
        //uint64_t resp_tx_ts = (((uint64_t)(resp_tx_time & 0xFFFFFFFEUL)) << 8) + TX_ANT_DLY;
        // low order bit is ignored by dwt_setdelayedtrxtime.
        uint64_t final_tx_ts = (final_tx_time & 0xFFFFFFFE00) + TX_ANT_DLY;

        // on répond à celui qui nous a posé la question
        tx_msg.header.dst_id = msg->header.src_id;
        tx_msg.header.seq_number = frame_seq_nb;
        tx_msg.header.frame_role = DF_DS_RANGING_FINAL;
        tx_msg.final.resp_id = msg->resp_id;
        tx_msg.final.poll_tx_ts = (uint32_t)poll_tx_ts;
        tx_msg.final.poll_rx_ts = msg->poll_rx_ts;
        tx_msg.final.resp_tx_ts = msg->resp_tx_ts;
        tx_msg.final.resp_rx_ts = (uint32_t)resp_rx_ts;
        tx_msg.final.final_tx_ts = (uint32_t)final_tx_ts;

        if (pdec_radio_transmit(&tx_msg.final, sizeof(tx_msg.final), true,
                                true, final_tx_time) == 0) {
            frame_seq_nb++;
        }
    } else {
        pdec_radio_complete_exchange();
    }

    pdec_radio_range_measured(distance);
    set_distance(get_my_id(), msg->header.src_id, distance);
    publish_ranging_event(get_my_id(), msg->header.src_id, distance, RANGING_EVENT_KIND_SS);

#if CONFIG_PDEC_PRINT_RANGING
    printf(">[%04x] SS DIST(%04x): %.2f\n", get_my_id(), msg->header.src_id, distance);
#endif

}

void handle_poll_msg(poll_msg_t* msg, bool double_sided) {
    if (!pdec_radio_can_respond() ||
        (msg->header.dst_id != get_my_id() && msg->header.dst_id != BROADCAST_ID)) {
        return;
    }


    tx_msg.header.dst_id = msg->header.src_id;
    tx_msg.header.src_id = get_my_id();
    if(double_sided) {
        tx_msg.header.frame_role = DF_DS_RANGING_RESP;
    }
    else {
        tx_msg.header.frame_role = DF_SS_RANGING_RESP;
    }
    
    tx_msg.header.seq_number = frame_seq_nb;
    tx_msg.resp.poll_id = msg->poll_id;
    tx_msg.resp.resp_id = sys_rand8_get();

    /* Retrieve poll reception timestamp. */
    uint64_t poll_rx_ts = get_rx_timestamp_u64();

    // en dtu (499.2MHz*128)
    uint32_t deltat = (POLL_RX_TO_RESP_TX_DLY_UUS * UUS_TO_DWT_TIME);

    /* Compute response message transmission time. See NOTE 7 below. */
    uint64_t resp_tx_time = poll_rx_ts + deltat;

    uint64_t resp_tx_ts = (resp_tx_time & 0xFFFFFFFE00) + TX_ANT_DLY;

    tx_msg.resp.poll_rx_ts = (uint32_t)poll_rx_ts;
    tx_msg.resp.resp_tx_ts = (uint32_t)resp_tx_ts;

    if (pdec_radio_transmit(&tx_msg.resp, sizeof(tx_msg.resp), true,
                            true, resp_tx_time) == 0) {
        frame_seq_nb++;
    }
}


void handle_final_msg(final_msg_t* msg) {

    if (msg->header.dst_id == get_my_id() && !pdec_radio_matches_final(msg)) {
        return;
    }
    if (pdec_radio_matches_final(msg)) {
        pdec_radio_complete_exchange();
        // final message from a DS ranging directed to us.
        /* Retrieve poll transmission and response reception timestamps. See NOTE 9 below. */
        uint64_t resp_tx_ts = get_tx_timestamp_u64(); //should be the same as msg->resp_tx_ts, maybe more precise.
        uint64_t final_rx_ts = get_rx_timestamp_u64();
        
        double RA = (double)wrap32_diff(msg->resp_rx_ts, msg->poll_tx_ts);
        double rA = (double)wrap32_diff(msg->final_tx_ts, msg->resp_rx_ts);
        double RB = (double)wrap32_diff(final_rx_ts, resp_tx_ts);
        double rB = (double)wrap32_diff(resp_tx_ts, msg->poll_rx_ts);

        double tof_dtu = (RA * RB - rA * rB) / (RA + RB + rA + rB);
        
        double tof = tof_dtu * DWT_TIME_UNITS;
        double distance = tof * SPEED_OF_LIGHT;

        set_distance(get_my_id(), msg->header.src_id, distance);
        publish_ranging_event(get_my_id(), msg->header.src_id, distance, RANGING_EVENT_KIND_DS);
#if CONFIG_PDEC_PRINT_RANGING
        printf(">[%04x] DS DIST(%04x): %.2f\n", get_my_id(), msg->header.src_id, distance);
#endif
    }
    else {
        double RA = (double)wrap32_diff(msg->resp_rx_ts, msg->poll_tx_ts);
        double rB = (double)wrap32_diff(msg->resp_tx_ts, msg->poll_rx_ts);

        double tof = ((RA* get_clock_offset(msg->header.src_id) - rB * get_clock_offset(msg->header.dst_id) ) / 2.0) * DWT_TIME_UNITS;
        double distance = tof * SPEED_OF_LIGHT;

        set_distance(msg->header.src_id, msg->header.dst_id, distance);
        publish_ranging_event(msg->header.src_id, msg->header.dst_id, distance, RANGING_EVENT_KIND_IN);

#if CONFIG_PDEC_PRINT_RANGING
        printf(">[%04x] IN DIST(%04x): %.2f\n", msg->header.src_id, msg->header.dst_id, distance);
#endif
    }

}

int send_poll(uint16_t dst_id, bool double_sided)
{
    return pdec_radio_submit_poll(dst_id, double_sided);
}

void send_poll_now(uint16_t dst_id, bool double_sided) {
    pdec_radio_assert_owner();
    tx_msg.header.frame_control = FRAME_CONTROL;
    tx_msg.header.pan_id = MY_PAN_ID;
    tx_msg.header.dst_id = dst_id;
    tx_msg.header.src_id = get_my_id();
    tx_msg.header.seq_number = frame_seq_nb;

    if(double_sided) {
        tx_msg.header.frame_role = DF_DS_RANGING_POLL;
    } else {
        tx_msg.header.frame_role = DF_SS_RANGING_POLL;
    }
    tx_msg.poll.poll_id =  sys_rand8_get();

    if (pdec_radio_transmit(&tx_msg.poll, sizeof(tx_msg.poll), true, false, 0) == 0) {
        frame_seq_nb++;
    }
}


static void monitor_bypass(const struct shell *sh, uint8_t *data, size_t len)
{
	ARG_UNUSED(sh);

	for (size_t i = 0; i < len; i++) {
		if (data[i] == ASCII_CTRL_C) {
			stop_monitor();
		}
	}
}

static void monitor(void *p1, void *p2, void *p3)
{
    const struct shell *shell = p1;
    struct ranging_event event;

    shell_set_bypass(shell, monitor_bypass);

    while (true) {
        if (k_msgq_get(&monitor_queue, &event, K_FOREVER) != 0) {
            continue;
        }

        if (event.kind == RANGING_EVENT_KIND_STOP && monitor_stop_requested) {
            break;
        }

        if (event.kind == RANGING_EVENT_KIND_STOP || !should_print_event(&event)) {
            continue;
        }

        shell_print(shell, "[%s] 0x%04x <-> 0x%04x: %.02fm",
                    ranging_event_kind_label(event.kind),
                    event.src_id, event.dst_id, event.distance);
    }

    shell_set_bypass(shell, NULL);
    monitor_running = false;
    shell_print(shell, "Stopped");
}

void start_monitor(const struct shell *shell, enum ranging_monitor_scope scope) {
    if (monitor_running) {
        shell_print(shell, "Monitor already running");
        return;
    }

    monitor_stop_requested = false;
    monitor_scope = scope;
    monitor_running = true;
    k_msgq_purge(&monitor_queue);

    k_thread_create(&monitor_thread, monitor_stack, MONITOR_STACK_SIZE,
                    monitor, (void *)shell, NULL, NULL,
                    PRIORITY, 0, K_NO_WAIT);

    return;
}

void stop_monitor() {
    if (!monitor_running) {
        return;
    }

    monitor_stop_requested = true;
    enqueue_stop_event();
}
