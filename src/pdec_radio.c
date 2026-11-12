#include "pdec_radio.h"
#include "pdec_range_request.h"
#include "pdec_discovery.h"

#include <errno.h>
#include <string.h>
#include <zephyr/sys/atomic.h>
#include <deca_device_api.h>

#define RADIO_STACK_SIZE 2048
#define RADIO_PRIORITY -2
#define RADIO_QUEUE_LEN 16

enum radio_command_kind {
    RADIO_COMMAND_SEND_POLL,
    RADIO_COMMAND_SEND_DATA,
    RADIO_COMMAND_SEND_SYNC,
    RADIO_COMMAND_SEND_DISCOVERY,
};

struct radio_command {
    enum radio_command_kind kind;
    union {
        struct {
            uint16_t dst_id;
            bool double_sided;
            uint32_t request_id;
        } poll;
        struct {
            uint16_t dst_id;
            uint8_t len;
            uint8_t payload[PAYLOAD_FRAME_LEN_MAX];
        } data;
    };
};

K_MSGQ_DEFINE(radio_commands, sizeof(struct radio_command), RADIO_QUEUE_LEN, 4);
K_THREAD_STACK_DEFINE(radio_stack, RADIO_STACK_SIZE);
static struct k_thread radio_thread_data;

static K_SEM_DEFINE(radio_wake, 0, 1);
static K_SEM_DEFINE(radio_init_done, 0, 1);
static atomic_t radio_ready;
static atomic_t radio_start_requested;
static int radio_init_result;
static uint32_t radio_device_id;

/* Preserve the old ~2 ms ranging response window, independently of unrelated
 * receptions. Hardware RX stays enabled with no frame-wait timeout.
 */
#define EXCHANGE_TIMEOUT_US 2100
#define TX_WATCHDOG_US 10000
#define RECOVERY_RETRY_MS 10
#define IRQ_DRAIN_LIMIT 32
#define DW_TIME_MASK ((1ULL << 40) - 1)

static enum pdec_radio_state radio_state = PDEC_RADIO_OFF;
static struct {
    enum pdec_exchange_phase phase;
    uint16_t peer;
    uint8_t token;
    bool double_sided;
    int64_t deadline;
} exchange;
static int64_t state_deadline;
static uint32_t active_request_id;
static bool range_measured;
static double measured_distance;

static void finish_request(int result)
{
    uint32_t id = active_request_id;
    active_request_id = 0;
    range_measured = false;
    if (id != 0) {
        pdec_range_request_complete(id, result, measured_distance);
    }
}

static void expire_exchange(void)
{
    exchange.phase = PDEC_EXCHANGE_NONE;
    finish_request(-ETIMEDOUT);
}

void pdec_radio_range_measured(double distance)
{
    pdec_radio_assert_owner();
    if (active_request_id == 0) {
        return;
    }
    measured_distance = distance;
    range_measured = true;
    /* SS completes at the response; DS waits until its final has transmitted. */
    if (exchange.phase == PDEC_EXCHANGE_NONE &&
        (radio_state == PDEC_RADIO_LISTENING || radio_state == PDEC_RADIO_PROCESSING_RX)) {
        finish_request(0);
    }
}

static int64_t deadline_after_us(uint64_t us)
{
    return k_uptime_ticks() + k_us_to_ticks_ceil64(us);
}

enum pdec_radio_state pdec_radio_get_state(void)
{
    pdec_radio_assert_owner();
    return radio_state;
}

enum pdec_exchange_phase pdec_radio_get_exchange(void)
{
    pdec_radio_assert_owner();
    return exchange.phase;
}

static int enable_rx(void)
{
    dwt_setrxtimeout(0);
    if (dwt_rxenable(DWT_START_RX_IMMEDIATE) != DWT_SUCCESS) {
        return -EIO;
    }
    radio_state = PDEC_RADIO_LISTENING;
    state_deadline = 0;
    return 0;
}

static void recover_radio(int result)
{
    radio_state = PDEC_RADIO_RECOVERING;
    exchange.phase = PDEC_EXCHANGE_NONE;
    finish_request(result);
    dwt_forcetrxoff();
    dwt_writesysstatuslo(DWT_INT_RX | DWT_INT_TXFRS_BIT_MASK);
    dwt_setrxaftertxdelay(0);
    if (enable_rx() != 0) {
        state_deadline = deadline_after_us(RECOVERY_RETRY_MS * 1000);
    }
}

bool pdec_radio_rx_begin(void)
{
    pdec_radio_assert_owner();
    if (radio_state != PDEC_RADIO_LISTENING) {
        return false;
    }
    if (exchange.phase != PDEC_EXCHANGE_NONE &&
        k_uptime_ticks() >= exchange.deadline) {
        expire_exchange();
    }
    radio_state = PDEC_RADIO_PROCESSING_RX;
    return true;
}

void pdec_radio_rx_end(void)
{
    pdec_radio_assert_owner();
    /* A handler may have armed TX or entered recovery. Do not undo it. */
    if (radio_state == PDEC_RADIO_PROCESSING_RX && enable_rx() != 0) {
        recover_radio(-EIO);
    }
}

void pdec_radio_tx_done(void)
{
    pdec_radio_assert_owner();
    if (radio_state != PDEC_RADIO_WAIT_TX_DONE) {
        return;
    }
    /* RESPONSE_EXPECTED already enabled RX; do not restart an incoming frame. */
    radio_state = PDEC_RADIO_LISTENING;
    state_deadline = 0;
    if (exchange.phase != PDEC_EXCHANGE_NONE) {
        exchange.deadline = deadline_after_us(EXCHANGE_TIMEOUT_US);
    } else if (range_measured) {
        finish_request(0);
    }
}

void pdec_radio_rx_error(void)
{
    pdec_radio_assert_owner();
    /* dwt_isr may deliver old RX error bits after a good-RX callback has
     * already armed a new TX. Those events must not cancel that TX.
     */
    if (radio_state == PDEC_RADIO_WAIT_TX_DONE) {
        return;
    }
    if (radio_state == PDEC_RADIO_LISTENING ||
        radio_state == PDEC_RADIO_PROCESSING_RX) {
        if (enable_rx() != 0) {
            recover_radio(-EIO);
        }
    }
}

bool pdec_radio_can_respond(void)
{
    pdec_radio_assert_owner();
    return radio_state == PDEC_RADIO_PROCESSING_RX &&
           exchange.phase == PDEC_EXCHANGE_NONE;
}

bool pdec_radio_matches_response(const resp_msg_t *msg, bool double_sided)
{
    pdec_radio_assert_owner();
    return radio_state == PDEC_RADIO_PROCESSING_RX &&
           exchange.phase == PDEC_EXCHANGE_RESPONSE &&
           (exchange.peer == BROADCAST_ID || exchange.peer == msg->header.src_id) &&
           exchange.token == msg->poll_id &&
           exchange.double_sided == double_sided &&
           msg->header.dst_id == get_my_id();
}

bool pdec_radio_matches_final(const final_msg_t *msg)
{
    pdec_radio_assert_owner();
    return radio_state == PDEC_RADIO_PROCESSING_RX &&
           exchange.phase == PDEC_EXCHANGE_FINAL &&
           exchange.peer == msg->header.src_id &&
           exchange.token == msg->resp_id &&
           msg->header.dst_id == get_my_id();
}

void pdec_radio_complete_exchange(void)
{
    pdec_radio_assert_owner();
    exchange.phase = PDEC_EXCHANGE_NONE;
}

int pdec_radio_transmit(const void *frame, uint16_t len, bool ranging,
                        bool delayed, uint64_t deadline)
{
    pdec_radio_assert_owner();
    if (frame == NULL || len < sizeof(frame_header_t) + 2 || len > FRAME_LEN_MAX) {
        return -EINVAL;
    }
    if (radio_state != PDEC_RADIO_LISTENING &&
        radio_state != PDEC_RADIO_PROCESSING_RX) {
        return -EBUSY;
    }
    const frame_header_t *header = frame;
    bool final = header->frame_role == DF_DS_RANGING_FINAL;
    if (exchange.phase != PDEC_EXCHANGE_NONE &&
        !(final && exchange.phase == PDEC_EXCHANGE_RESPONSE &&
          exchange.double_sided &&
          (exchange.peer == BROADCAST_ID || exchange.peer == header->dst_id))) {
        return -EBUSY;
    }
    if (((header->frame_role == DF_SS_RANGING_POLL ||
          header->frame_role == DF_DS_RANGING_POLL) && len < sizeof(poll_msg_t)) ||
        (header->frame_role == DF_DS_RANGING_RESP && len < sizeof(resp_msg_t))) {
        return -EINVAL;
    }

    uint64_t wait_us = 0;
    if (delayed) {
        uint64_t now = get_systime_u64() << 8;
        uint64_t dt = (deadline - now) & DW_TIME_MASK;
        if (dt & (1ULL << 39)) {
            recover_radio(-ETIMEDOUT);
            return -ETIMEDOUT;
        }
        wait_us = dt / UUS_TO_DWT_TIME;
    }

    radio_state = PDEC_RADIO_WAIT_TX_DONE;
    dwt_forcetrxoff();
    dwt_writesysstatuslo(DWT_INT_RX | DWT_INT_TXFRS_BIT_MASK);
    if (dwt_writetxdata(len, (uint8_t *)frame, 0) != DWT_SUCCESS) {
        recover_radio(-EIO);
        return -EIO;
    }
    dwt_writetxfctrl(len, 0, ranging);
    if (delayed) {
        dwt_setdelayedtrxtime((uint32_t)(deadline >> 8));
    }
    dwt_setrxaftertxdelay(0);
    dwt_setrxtimeout(0);
    uint8_t mode = (delayed ? DWT_START_TX_DELAYED : DWT_START_TX_IMMEDIATE)
                 | DWT_RESPONSE_EXPECTED;
    if (dwt_starttx(mode) != DWT_SUCCESS) {
        recover_radio(-EIO);
        return -EIO;
    }

    exchange.phase = PDEC_EXCHANGE_NONE;
    if (header->frame_role == DF_SS_RANGING_POLL ||
        header->frame_role == DF_DS_RANGING_POLL) {
        exchange.phase = PDEC_EXCHANGE_RESPONSE;
        exchange.token = ((const poll_msg_t *)frame)->poll_id;
        exchange.double_sided = header->frame_role == DF_DS_RANGING_POLL;
    } else if (header->frame_role == DF_DS_RANGING_RESP) {
        exchange.phase = PDEC_EXCHANGE_FINAL;
        exchange.token = ((const resp_msg_t *)frame)->resp_id;
        exchange.double_sided = true;
    }
    exchange.peer = header->dst_id;
    exchange.deadline = 0; /* Response window starts at TX completion. */
    state_deadline = deadline_after_us(wait_us + TX_WATCHDOG_US);
    return 0;
}

static bool expire_deadlines(void)
{
    int64_t now = k_uptime_ticks();
    if (state_deadline != 0 && now >= state_deadline) {
        recover_radio(-ETIMEDOUT);
        return true;
    }
    if (radio_state == PDEC_RADIO_LISTENING &&
        exchange.phase != PDEC_EXCHANGE_NONE && now >= exchange.deadline) {
        /* This is a protocol timeout: RX is already on, do not interrupt it. */
        expire_exchange();
    }
    return false;
}

static k_timeout_t next_wakeup(void)
{
    int64_t deadline = state_deadline;
    if (radio_state == PDEC_RADIO_LISTENING && exchange.phase != PDEC_EXCHANGE_NONE) {
        deadline = exchange.deadline;
    }
    return deadline ? K_TICKS(MAX(0, deadline - k_uptime_ticks())) : K_FOREVER;
}

void pdec_radio_irq_notify(void)
{
    /* Coalescing is safe: the owner drains the hardware IRQ level. */
    k_sem_give(&radio_wake);
}

bool pdec_radio_is_owner(void)
{
    return k_current_get() == &radio_thread_data;
}

static int submit_radio_command(const struct radio_command *event)
{
    if (!atomic_get(&radio_ready)) {
        return -ENODEV;
    }

    int ret = k_msgq_put(&radio_commands, event, K_NO_WAIT);
    if (ret == 0) {
        k_sem_give(&radio_wake);
    }
    return ret;
}

static void radio_thread(void *p1, void *p2, void *p3)
{
    ARG_UNUSED(p1);
    ARG_UNUSED(p2);
    ARG_UNUSED(p3);

    radio_init_result = pdec_radio_hw_init(&radio_device_id);
    if (radio_init_result == 0) {
        radio_init_result = enable_rx();
    }
    if (radio_init_result == 0) {
        atomic_set(&radio_ready, 1);
    }
    k_sem_give(&radio_init_done);
    if (radio_init_result != 0) {
        return;
    }

    while (true) {
        struct radio_command event;

        /* Hardware events never compete for space in the command queue.
         * Read their payload/timestamps in dwt_isr callbacks before any TX.
         */
        unsigned int irq_count = 0;
        while (radio_state != PDEC_RADIO_RECOVERING && pdec_dw3000_irq_active()) {
            if (++irq_count > IRQ_DRAIN_LIMIT) {
                recover_radio(-EIO);
                /* An unhandled/stuck IRQ must not spin a cooperative thread. */
                radio_state = PDEC_RADIO_RECOVERING;
                state_deadline = deadline_after_us(RECOVERY_RETRY_MS * 1000);
                break;
            }
            dwt_isr();
        }
        if (expire_deadlines()) {
            /* Recheck IRQs after recovery before dispatching a command. */
            continue;
        }

        if (radio_state != PDEC_RADIO_LISTENING || exchange.phase != PDEC_EXCHANGE_NONE) {
            k_sem_take(&radio_wake, next_wakeup());
            continue;
        }

        if (k_msgq_get(&radio_commands, &event, K_NO_WAIT) != 0) {
            /* A producer racing this check leaves a latched wake token. */
            k_sem_take(&radio_wake, next_wakeup());
            continue;
        }

        switch (event.kind) {
        case RADIO_COMMAND_SEND_POLL:
            if (event.poll.request_id != 0 &&
                !pdec_range_request_is_pending(event.poll.request_id)) {
                break; /* Caller timed out while this command was queued. */
            }
            active_request_id = event.poll.request_id;
            range_measured = false;
            measured_distance = 0;
            send_poll_now(event.poll.dst_id, event.poll.double_sided);
            if (radio_state != PDEC_RADIO_WAIT_TX_DONE) {
                finish_request(-EIO);
            }
            break;
        case RADIO_COMMAND_SEND_DATA:
            pdec_radio_send_data_now(event.data.dst_id, event.data.payload, event.data.len);
            break;
        case RADIO_COMMAND_SEND_SYNC:
            send_sync_now();
            break;
        case RADIO_COMMAND_SEND_DISCOVERY:
            /* Also discard discoveries queued before the user disabled them. */
            if (pdec_discovery_is_enabled()) {
                send_discovery_now();
            }
            break;
        }
    }
}

int pdecInit(uint32_t *dev_id)
{
    if (!atomic_cas(&radio_start_requested, 0, 1)) {
        return -EALREADY;
    }

    k_thread_create(&radio_thread_data, radio_stack, K_THREAD_STACK_SIZEOF(radio_stack),
                    radio_thread, NULL, NULL, NULL, RADIO_PRIORITY, 0, K_NO_WAIT);
    k_sem_take(&radio_init_done, K_FOREVER);
    if (radio_init_result == 0 && dev_id != NULL) {
        *dev_id = radio_device_id;
    }
    return radio_init_result;
}

int pdec_radio_submit_poll(uint16_t dst_id, bool double_sided)
{
    return pdec_radio_submit_range(dst_id, double_sided, 0);
}

int pdec_radio_submit_range(uint16_t dst_id, bool double_sided, uint32_t request_id)
{
    struct radio_command event = {
        .kind = RADIO_COMMAND_SEND_POLL,
        .poll = {
            .dst_id = dst_id,
            .double_sided = double_sided,
            .request_id = request_id,
        },
    };

    return submit_radio_command(&event);
}

int pdec_radio_submit_data(uint16_t dst_id, const uint8_t *buffer, uint8_t len)
{
    struct radio_command event = {
        .kind = RADIO_COMMAND_SEND_DATA,
        .data = {
            .dst_id = dst_id,
            .len = len,
        },
    };

    if (len > PAYLOAD_FRAME_LEN_MAX) {
        return -EMSGSIZE;
    }

    memcpy(event.data.payload, buffer, len);
    return submit_radio_command(&event);
}

int pdec_radio_submit_sync(void)
{
    struct radio_command event = {
        .kind = RADIO_COMMAND_SEND_SYNC,
    };

    return submit_radio_command(&event);
}

int pdec_radio_submit_discovery(void)
{
    struct radio_command event = {
        .kind = RADIO_COMMAND_SEND_DISCOVERY,
    };

    return submit_radio_command(&event);
}

