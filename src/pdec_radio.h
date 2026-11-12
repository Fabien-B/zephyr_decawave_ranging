#pragma once

#include "pdec_common.h"
#include <zephyr/kernel.h>

/* Internal radio/port interface. Only the GPIO callback may call notify from ISR.
 * All *_now functions and all DW3000 accesses belong to the radio thread.
 */
void pdec_radio_irq_notify(void);
bool pdec_dw3000_irq_active(void);
bool pdec_radio_is_owner(void);

static inline void pdec_radio_assert_owner(void)
{
    __ASSERT(pdec_radio_is_owner(), "DW3000 access outside the radio thread");
}

int pdec_radio_hw_init(uint32_t *dev_id);
void pdec_radio_send_data_now(uint16_t dst_id, uint8_t *buffer, uint8_t len);
void send_poll_now(uint16_t dst_id, bool double_sided);
void send_sync_now(void);
void send_discovery_now(void);

/* Owner-only state machine interface. Protocol handlers describe a TX; only
 * the radio service programs RX/TX transitions and handles their failures.
 */
enum pdec_radio_state {
    PDEC_RADIO_OFF,
    PDEC_RADIO_LISTENING,
    PDEC_RADIO_PROCESSING_RX,
    PDEC_RADIO_WAIT_TX_DONE,
    PDEC_RADIO_RECOVERING,
};

enum pdec_exchange_phase {
    PDEC_EXCHANGE_NONE,
    PDEC_EXCHANGE_RESPONSE,
    PDEC_EXCHANGE_FINAL,
};

enum pdec_radio_state pdec_radio_get_state(void);
enum pdec_exchange_phase pdec_radio_get_exchange(void);
bool pdec_radio_rx_begin(void);
void pdec_radio_rx_end(void);
void pdec_radio_tx_done(void);
void pdec_radio_rx_error(void);
bool pdec_radio_can_respond(void);
bool pdec_radio_matches_response(const resp_msg_t *msg, bool double_sided);
bool pdec_radio_matches_final(const final_msg_t *msg);
void pdec_radio_complete_exchange(void);

/* deadline is a 40-bit DW timestamp (not shifted), ignored for immediate TX. */
int pdec_radio_transmit(const void *frame, uint16_t len, bool ranging,
                        bool delayed, uint64_t deadline);

/* Requested polls carry a host-side identity; ordinary submit_poll uses zero. */
int pdec_radio_submit_range(uint16_t dst_id, bool double_sided, uint32_t request_id);
void pdec_radio_range_measured(double distance);

/* Runtime instrumentation; no SPI access from shell/snapshot callers. */
enum pdec_radio_timing_path {
    PDEC_TIMING_RESPONSE,
    PDEC_TIMING_FINAL,
    PDEC_TIMING_PATH_COUNT,
};
enum pdec_radio_timing_stage {
    PDEC_TIMING_RX_PROCESSING,
    PDEC_TIMING_TX_VALIDATION,
    PDEC_TIMING_SYS_TIME,
    PDEC_TIMING_TRX_OFF,
    PDEC_TIMING_CLEAR_STATUS,
    PDEC_TIMING_WRITE_DATA,
    PDEC_TIMING_FRAME_CONTROL,
    PDEC_TIMING_DELAY_TIME,
    PDEC_TIMING_START_TX,
    PDEC_TIMING_STAGE_COUNT,
};
struct pdec_radio_timing_metric {
    int32_t min_us, max_us;
    int64_t total_us;
    uint32_t count;
};
struct pdec_radio_timing_stats {
    uint32_t attempts, samples, start_failures, expired, write_failures;
    struct pdec_radio_timing_metric rx_to_entry;
    /* Bounds on the fast TX command, from one existing DW-clock anchor. */
    struct pdec_radio_timing_metric rx_request_lower, rx_request_upper;
    struct pdec_radio_timing_metric lead_lower, lead_upper;
    struct pdec_radio_timing_metric peer_turnaround;
    struct pdec_radio_timing_metric stage[PDEC_TIMING_STAGE_COUNT];
};
void pdec_radio_timing_enable(bool enabled);
bool pdec_radio_timing_enabled(void);
void pdec_radio_timing_reset(void);
void pdec_radio_timing_snapshot(struct pdec_radio_timing_stats stats[PDEC_TIMING_PATH_COUNT]);
/* Owner only: reuse the timestamp already read by the protocol handler. */
void pdec_radio_timing_rx_timestamp(uint64_t timestamp);
/* Reuse existing TX/RX stamps to observe the peer's on-air turnaround. */
void pdec_radio_timing_peer_turnaround(enum pdec_radio_timing_path path,
                                      uint64_t tx, uint64_t rx);
