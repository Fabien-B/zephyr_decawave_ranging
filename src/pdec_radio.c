#include "pdec_radio.h"
#include "pdec_range_request.h"
#include "pdec_discovery.h"

#include <errno.h>
#include <string.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/timing/timing.h>
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
static bool tx_control_valid;
static uint16_t tx_control_len;
static bool tx_control_ranging;
static uint64_t poll_tx_timestamp;
static bool poll_tx_timestamp_valid;

static atomic_t timing_enabled;
static struct k_spinlock timing_lock;
static struct pdec_radio_timing_stats timing_stats[PDEC_TIMING_PATH_COUNT];
static bool timing_rx_valid;
static uint64_t timing_rx_timestamp;
static timing_t timing_callback_start;
static bool timing_rx_capture;

void pdec_radio_timing_enable(bool enabled)
{
    atomic_set(&timing_enabled, enabled);
}

bool pdec_radio_timing_enabled(void)
{
    return atomic_get(&timing_enabled) != 0;
}

void pdec_radio_timing_reset(void)
{
    k_spinlock_key_t key = k_spin_lock(&timing_lock);
    memset(timing_stats, 0, sizeof(timing_stats));
    k_spin_unlock(&timing_lock, key);
}

void pdec_radio_timing_snapshot(struct pdec_radio_timing_stats stats[PDEC_TIMING_PATH_COUNT])
{
    k_spinlock_key_t key = k_spin_lock(&timing_lock);
    memcpy(stats, timing_stats, sizeof(timing_stats));
    k_spin_unlock(&timing_lock, key);
}

void pdec_radio_timing_rx_timestamp(uint64_t timestamp)
{
    pdec_radio_assert_owner();
    timing_rx_timestamp = timestamp;
    timing_rx_valid = true;
}

/* Actual microseconds, independent of the protocol's configurable UUS constant.
 * DW clock = 499.2 MHz * 128 = 63897.6 ticks/us. Handle 40-bit wrap.
 */
static int32_t timing_diff_us(uint64_t a, uint64_t b)
{
    int64_t ticks = (a - b) & DW_TIME_MASK;
    if (ticks & (1ULL << 39)) {
        ticks -= 1LL << 40;
    }
    return (int32_t)(ticks * 5 / 319488);
}

static void timing_metric_add(struct pdec_radio_timing_metric *metric, int32_t us)
{
    if (metric->count == 0 || us < metric->min_us) { metric->min_us = us; }
    if (metric->count == 0 || us > metric->max_us) { metric->max_us = us; }
    metric->total_us += us;
    metric->count++;
}

static int32_t timing_elapsed_us(timing_t start, timing_t end)
{
    return (int32_t)(timing_cycles_to_ns(timing_cycles_get(&start, &end)) / 1000);
}

void pdec_radio_timing_peer_turnaround(enum pdec_radio_timing_path path,
                                      uint64_t tx, uint64_t rx)
{
    pdec_radio_assert_owner();
    if (!pdec_radio_timing_enabled()) { return; }
    int32_t us = timing_diff_us(rx, tx);
    k_spinlock_key_t key = k_spin_lock(&timing_lock);
    timing_metric_add(&timing_stats[path].peer_turnaround, us);
    k_spin_unlock(&timing_lock, key);
}

/* points bound each consecutive stage, starting at RX callback entry. */
static void timing_record(enum pdec_radio_timing_path path, uint64_t rx,
                          uint64_t entry, uint64_t deadline, timing_t *points,
                          unsigned int completed, int result)
{
    int32_t stage_us[PDEC_TIMING_STAGE_COUNT];
    for (unsigned int i = 0; i < completed; i++) {
        stage_us[i] = timing_elapsed_us(points[i], points[i + 1]);
    }
    int32_t entry_us = timing_diff_us(entry, rx);
    int32_t lead_at_entry = timing_diff_us(deadline, entry);
    int32_t lower = 0, upper = 0;
    bool started = completed == PDEC_TIMING_STAGE_COUNT;
    if (started) {
        /* The existing SYS_TIME register was sampled somewhere between its
         * local before/after marks. Bound the fast command inside starttx().
         * No additional SPI reads, or per-stage conversions before TX arming.
         */
        lower = timing_elapsed_us(points[PDEC_TIMING_SYS_TIME + 1],
                                  points[PDEC_TIMING_START_TX]);
        upper = timing_elapsed_us(points[PDEC_TIMING_SYS_TIME],
                                  points[PDEC_TIMING_START_TX + 1]) + 2;
        /* Allow for the two integer-us roundings at the clock anchor. */
    }
    k_spinlock_key_t key = k_spin_lock(&timing_lock);
    struct pdec_radio_timing_stats *stats = &timing_stats[path];
    timing_metric_add(&stats->rx_to_entry, entry_us);
    for (unsigned int i = 0; i < completed; i++) {
        timing_metric_add(&stats->stage[i], stage_us[i]);
    }
    stats->attempts++;
    if (started) {
        timing_metric_add(&stats->rx_request_lower, entry_us + lower);
        timing_metric_add(&stats->rx_request_upper, entry_us + upper);
        timing_metric_add(&stats->lead_lower, lead_at_entry - upper);
        timing_metric_add(&stats->lead_upper, lead_at_entry - lower + 2);
        stats->samples++;
        stats->start_failures += result != DWT_SUCCESS;
    } else if (result == -ETIMEDOUT) {
        stats->expired++;
    } else {
        stats->write_failures++;
    }
    k_spin_unlock(&timing_lock, key);
}

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
    poll_tx_timestamp_valid = false;
    finish_request(result);
    dwt_forcetrxoff();
    dwt_writesysstatuslo(DWT_INT_RX | DWT_INT_TXFRS_BIT_MASK);
    tx_control_valid = false;
    dwt_setrxaftertxdelay(0);
    dwt_setrxtimeout(0);
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
    timing_rx_valid = false;
    timing_rx_capture = pdec_radio_timing_enabled();
    if (timing_rx_capture) { timing_callback_start = timing_counter_get(); }
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
        if (exchange.phase == PDEC_EXCHANGE_RESPONSE) {
            /* Read once after POLL finishes, outside RESPONSE->FINAL setup.
             * dwt_isr handles TX done before RXOK if both are pending.
             */
            poll_tx_timestamp = get_tx_timestamp_u64();
            poll_tx_timestamp_valid = true;
        }
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

uint64_t pdec_radio_poll_tx_timestamp(void)
{
    pdec_radio_assert_owner();
    __ASSERT(poll_tx_timestamp_valid && exchange.phase == PDEC_EXCHANGE_RESPONSE,
             "POLL TX timestamp requested outside the active exchange");
    return poll_tx_timestamp;
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
    timing_t setup_entry = delayed && timing_rx_capture && pdec_radio_timing_enabled()
        ? timing_counter_get() : 0;
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

    bool measure = delayed && radio_state == PDEC_RADIO_PROCESSING_RX &&
        timing_rx_valid && timing_rx_capture && pdec_radio_timing_enabled() &&
        (final || header->frame_role == DF_DS_RANGING_RESP ||
         header->frame_role == DF_SS_RANGING_RESP);
    enum pdec_radio_timing_path timing_path = final ? PDEC_TIMING_FINAL : PDEC_TIMING_RESPONSE;
    timing_t points[PDEC_TIMING_STAGE_COUNT + 1];
    if (measure) {
        points[0] = timing_callback_start;
        points[PDEC_TIMING_TX_VALIDATION] = setup_entry;
        points[PDEC_TIMING_SYS_TIME] = timing_counter_get();
    }
    uint64_t timing_entry = 0;
    uint64_t wait_us = 0;
    if (delayed) {
        uint64_t now = get_systime_u64() << 8;
        if (measure) { points[PDEC_TIMING_SYS_TIME + 1] = timing_counter_get(); }
        timing_entry = now;
        uint64_t dt = (deadline - now) & DW_TIME_MASK;
        if (dt & (1ULL << 39)) {
            if (measure) {
                timing_record(timing_path, timing_rx_timestamp, timing_entry, deadline,
                              points, PDEC_TIMING_SYS_TIME + 1, -ETIMEDOUT);
            }
            recover_radio(-ETIMEDOUT);
            return -ETIMEDOUT;
        }
        wait_us = dt / UUS_TO_DWT_TIME;
    }

    /* The real RXOK callback runs after status acknowledgement, with manual
     * single-buffer RX stopped. LISTENING may still be receiving a frame.
     */
    bool rx_callback = radio_state == PDEC_RADIO_PROCESSING_RX;
    radio_state = PDEC_RADIO_WAIT_TX_DONE;
    if (!rx_callback) { dwt_forcetrxoff(); }
    if (measure) { points[PDEC_TIMING_TRX_OFF + 1] = timing_counter_get(); }
    if (!rx_callback) { dwt_writesysstatuslo(DWT_INT_RX | DWT_INT_TXFRS_BIT_MASK); }
    if (measure) { points[PDEC_TIMING_CLEAR_STATUS + 1] = timing_counter_get(); }
    int write_result = dwt_writetxdata(len, (uint8_t *)frame, 0);
    if (measure) { points[PDEC_TIMING_WRITE_DATA + 1] = timing_counter_get(); }
    if (write_result != DWT_SUCCESS) {
        if (measure) {
            timing_record(timing_path, timing_rx_timestamp, timing_entry, deadline,
                          points, PDEC_TIMING_WRITE_DATA + 1, -EIO);
        }
        recover_radio(-EIO);
        return -EIO;
    }
    /* Only this owner writes TX_FCTRL, always with buffer offset zero. */
    if (!tx_control_valid || tx_control_len != len || tx_control_ranging != ranging) {
        dwt_writetxfctrl(len, 0, ranging);
        tx_control_valid = true;
        tx_control_len = len;
        tx_control_ranging = ranging;
    }
    if (measure) { points[PDEC_TIMING_FRAME_CONTROL + 1] = timing_counter_get(); }
    if (delayed) {
        dwt_setdelayedtrxtime((uint32_t)(deadline >> 8));
    }
    if (measure) { points[PDEC_TIMING_DELAY_TIME + 1] = timing_counter_get(); }
    uint8_t mode = (delayed ? DWT_START_TX_DELAYED : DWT_START_TX_IMMEDIATE)
                 | DWT_RESPONSE_EXPECTED;
    if (measure) { points[PDEC_TIMING_START_TX] = timing_counter_get(); }
    int tx_result = dwt_starttx(mode);
    if (measure) {
        points[PDEC_TIMING_START_TX + 1] = timing_counter_get();
        timing_record(timing_path, timing_rx_timestamp, timing_entry, deadline,
                      points, PDEC_TIMING_STAGE_COUNT, tx_result);
    }
    if (tx_result != DWT_SUCCESS) {
        recover_radio(-EIO);
        return -EIO;
    }

    exchange.phase = PDEC_EXCHANGE_NONE;
    poll_tx_timestamp_valid = false;
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

    /* Nordic timing uses a free-running TIMER2, including SPI/thread waits. */
    timing_init();
    timing_start();
    radio_init_result = pdec_radio_hw_init(&radio_device_id);
    if (radio_init_result == 0) {
        /* Fixed settings: protocol deadlines are software-owned. */
        dwt_setrxaftertxdelay(0);
        dwt_setrxtimeout(0);
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

