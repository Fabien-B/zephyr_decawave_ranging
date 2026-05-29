#include"pdec_common.h"
#include "pdec_database.h"
#include "pdec_tdoa.h"
#include "pdec_ranging.h"
#include "uart_binary.h"

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

#define SYNC_DELAY 2000  //us
#define SYNC_SLOT_TIME 5000  //us
#define SYNC_PERIOD_MS 100
#define SYNC_PER_BEACON_RANGE 10
#define BLINK_REPORT_TIMEOUT_MARGIN_MS 10
#define BLINK_REPORTS_MAX 10


#define SYNC_TASK_STACK_SIZE 1000
#define SYNC_TASK_PRIORITY 5
#define SYNC_EVENT_STOP BIT(0)
#define TDOA_REPORT_QUEUE_LEN 4
#define TDOA_REPORT_STACK_SIZE 1024
#define TDOA_MONITOR_STACK_SIZE 1024
#define ASCII_CTRL_C 0x03

K_THREAD_STACK_DEFINE(sync_stack_area, SYNC_TASK_STACK_SIZE);
K_EVENT_DEFINE(sync_events);
struct k_thread sync_thread_data;

static tdoa_role_t tdoa_role;
static bool sync_thread_active;

uint64_t _master_blink_rx_ts = 0;

extern uint8_t frame_seq_nb;
extern volatile status_t status;

static union ranging_msg_u tx_msg;


static uint64_t last_sync_rx_ts = 0;    // in my clock
static uint64_t last_sync_tx_ts = 0;    // in master clock

uint16_t blink_seq_nb = 0;

uint32_t last_blink_id = 0;


double alpha = 1;
int64_t beta = 0;

uint16_t master_id = 0;

struct blink_report_entry {
    uint16_t reporter_id;
    uint64_t blink_rx_ts;
};

struct tdoa_report_event {
    uint32_t blink_id;
    uint8_t expected_count;
    uint8_t count;
    bool timed_out;
    uint16_t reporter_id[BLINK_REPORTS_MAX];
    double distance_m[BLINK_REPORTS_MAX];
};

static K_MUTEX_DEFINE(blink_report_lock);
static void blink_report_timeout_handler(struct k_work *work);
K_WORK_DELAYABLE_DEFINE(blink_report_timeout_work, blink_report_timeout_handler);
K_MSGQ_DEFINE(tdoa_report_queue, sizeof(struct tdoa_report_event), TDOA_REPORT_QUEUE_LEN, 8);
K_MSGQ_DEFINE(tdoa_monitor_queue, sizeof(struct tdoa_report_event), TDOA_REPORT_QUEUE_LEN, 8);
K_THREAD_STACK_DEFINE(tdoa_report_stack, TDOA_REPORT_STACK_SIZE);
K_THREAD_STACK_DEFINE(tdoa_monitor_stack, TDOA_MONITOR_STACK_SIZE);
static struct k_thread tdoa_report_thread_data;
static struct k_thread tdoa_monitor_thread_data;
static bool tdoa_report_thread_started;
static bool tdoa_monitor_running;
static volatile bool tdoa_monitor_stop_requested;

static bool blink_report_collection_active;
static uint32_t blink_report_collection_id;
static uint16_t blink_report_sender_id;
static size_t blink_report_expected_count;
static size_t blink_report_count;
static struct blink_report_entry blink_reports[BLINK_REPORTS_MAX];

static bool tdoa_role_reports_blinks(tdoa_role_t role)
{
    return role == TDOA_ROLE_BEACON || role == TDOA_ROLE_MASTER;
}

static void publish_tdoa_report_event(struct tdoa_report_event *event)
{
    if (k_msgq_put(&tdoa_report_queue, event, K_NO_WAIT) != 0) {
        k_msgq_purge(&tdoa_report_queue);
        (void)k_msgq_put(&tdoa_report_queue, event, K_NO_WAIT);
    }
}

static void tdoa_monitor_bypass(const struct shell *sh, uint8_t *data, size_t len)
{
    ARG_UNUSED(sh);

    for (size_t i = 0; i < len; i++) {
        if (data[i] == ASCII_CTRL_C) {
            stop_tdoa_monitor();
        }
    }
}

static void tdoa_monitor(void *p1, void *p2, void *p3)
{
    const struct shell *shell = p1;
    struct tdoa_report_event event;

    ARG_UNUSED(p2);
    ARG_UNUSED(p3);

    shell_set_bypass(shell, tdoa_monitor_bypass);

    while (true) {
        if (k_msgq_get(&tdoa_monitor_queue, &event, K_FOREVER) != 0) {
            continue;
        }

        if (tdoa_monitor_stop_requested) {
            break;
        }

        shell_print(shell, "TDOA blink %u reports %u/%u%s",
                    event.blink_id, event.count, event.expected_count,
                    event.timed_out ? " timeout" : "");
        for (uint8_t i = 0; i < event.count; i++) {
            shell_print(shell, " 0x%04x: %.02fm", event.reporter_id[i], event.distance_m[i]);
        }
    }

    shell_set_bypass(shell, NULL);
    tdoa_monitor_running = false;
    shell_print(shell, "Stopped");
}

static void tdoa_report_worker(void *p1, void *p2, void *p3)
{
    ARG_UNUSED(p1);
    ARG_UNUSED(p2);
    ARG_UNUSED(p3);

    struct tdoa_report_event event;

    while (true) {
        if (k_msgq_get(&tdoa_report_queue, &event, K_FOREVER) != 0) {
            continue;
        }

        pdec_uart_binary_publish_tdoa_report_event(event.blink_id, event.expected_count,
                                                   event.timed_out, event.reporter_id,
                                                   event.distance_m, event.count);

        if (tdoa_monitor_running) {
            (void)k_msgq_put(&tdoa_monitor_queue, &event, K_NO_WAIT);
        }
    }
}

static void start_tdoa_report_thread(void)
{
    if (tdoa_report_thread_started) {
        return;
    }

    k_thread_create(&tdoa_report_thread_data, tdoa_report_stack,
                    K_THREAD_STACK_SIZEOF(tdoa_report_stack),
                    tdoa_report_worker,
                    NULL, NULL, NULL,
                    SYNC_TASK_PRIORITY, 0, K_NO_WAIT);
    tdoa_report_thread_started = true;
}

static size_t count_expected_blink_reports(void)
{
    size_t count = 0;

    for(int i=0; get_device_id(i) != 0; i++) {
        uint16_t dev_id = get_device_id(i);
        if(dev_id != get_my_id() &&
           tdoa_role_reports_blinks(get_device_tdoa_role(dev_id))) {
            count++;
        }
    }

    return count;
}

static uint32_t expected_blink_report_timeout_ms(void)
{
    uint8_t max_tdoa_index = 0;

    for(int i=0; get_device_id(i) != 0; i++) {
        uint16_t dev_id = get_device_id(i);
        if(dev_id != get_my_id() &&
           tdoa_role_reports_blinks(get_device_tdoa_role(dev_id))) {
            uint8_t tdoa_index = get_tdoa_index(dev_id);
            if(tdoa_index != 255 && tdoa_index > max_tdoa_index) {
                max_tdoa_index = tdoa_index;
            }
        }
    }

    return (uint32_t)(((uint64_t)SYNC_SLOT_TIME * (max_tdoa_index + 1)) / 1000)
         + BLINK_REPORT_TIMEOUT_MARGIN_MS;
}

static void finish_blink_report_collection_locked(bool timed_out)
{
    uint64_t earliest_ts = 0;
    struct tdoa_report_event event = {
        .blink_id = blink_report_collection_id,
        .expected_count = (uint8_t)blink_report_expected_count,
        .count = (uint8_t)blink_report_count,
        .timed_out = timed_out,
    };

    for(size_t i=0; i<blink_report_count; i++) {
        if(i == 0 ||
           timestamp_diff_signed(blink_reports[i].blink_rx_ts, earliest_ts) < 0) {
            earliest_ts = blink_reports[i].blink_rx_ts;
        }
    }

    for(size_t i=0; i<blink_report_count; i++) {
        int64_t dt = timestamp_diff_signed(blink_reports[i].blink_rx_ts, earliest_ts);
        double distance = dt * DWT_TIME_UNITS * SPEED_OF_LIGHT;

        event.reporter_id[i] = blink_reports[i].reporter_id;
        event.distance_m[i] = distance;
    }

    blink_report_collection_active = false;
    publish_tdoa_report_event(&event);
}

static void blink_report_timeout_handler(struct k_work *work)
{
    ARG_UNUSED(work);

    k_mutex_lock(&blink_report_lock, K_FOREVER);
    if(blink_report_collection_active) {
        finish_blink_report_collection_locked(true);
    }
    k_mutex_unlock(&blink_report_lock);
}

static void start_blink_report_collection(uint32_t blink_id)
{
    uint32_t timeout_ms;

    k_mutex_lock(&blink_report_lock, K_FOREVER);

    blink_report_collection_active = true;
    blink_report_collection_id = blink_id;
    blink_report_sender_id = get_my_id();
    blink_report_expected_count = count_expected_blink_reports();
    blink_report_count = 0;

    if(blink_report_expected_count == 0) {
        blink_report_collection_active = false;
        k_mutex_unlock(&blink_report_lock);
        return;
    }

    timeout_ms = expected_blink_report_timeout_ms();
    k_mutex_unlock(&blink_report_lock);

    k_work_reschedule(&blink_report_timeout_work, K_MSEC(timeout_ms));
}

static void collect_blink_report(const blink_report_msg_t *msg)
{
    bool complete = false;

    if(msg->blink_snd_id != get_my_id() ||
       !tdoa_role_reports_blinks(get_device_tdoa_role(msg->header.src_id))) {
        return;
    }

    k_mutex_lock(&blink_report_lock, K_FOREVER);

    if(!blink_report_collection_active ||
       msg->blink_id != blink_report_collection_id ||
       msg->blink_snd_id != blink_report_sender_id) {
        k_mutex_unlock(&blink_report_lock);
        return;
    }

    for(size_t i=0; i<blink_report_count; i++) {
        if(blink_reports[i].reporter_id == msg->header.src_id) {
            blink_reports[i].blink_rx_ts = msg->blink_rx_ts;
            complete = blink_report_count >= blink_report_expected_count;
            if(complete) {
                finish_blink_report_collection_locked(false);
            }
            k_mutex_unlock(&blink_report_lock);
            if(complete) {
                (void)k_work_cancel_delayable(&blink_report_timeout_work);
            }
            return;
        }
    }

    if(blink_report_count < ARRAY_SIZE(blink_reports)) {
        blink_reports[blink_report_count].reporter_id = msg->header.src_id;
        blink_reports[blink_report_count].blink_rx_ts = msg->blink_rx_ts;
        blink_report_count++;
    }

    complete = blink_report_count >= blink_report_expected_count;
    if(complete) {
        finish_blink_report_collection_locked(false);
    }

    k_mutex_unlock(&blink_report_lock);

    if(complete) {
        (void)k_work_cancel_delayable(&blink_report_timeout_work);
    }
}

static void pdec_sync_mark_stopped(void)
{
    sync_thread_active = false;
    k_event_clear(&sync_events, SYNC_EVENT_STOP);
}

static bool pdec_sync_wait_or_stop(k_timeout_t timeout)
{
    return (k_event_wait(&sync_events, SYNC_EVENT_STOP, false, timeout) & SYNC_EVENT_STOP) != 0;
}

void tdoa_init() {
    tx_msg.header.frame_control = FRAME_CONTROL;
    tx_msg.header.pan_id = MY_PAN_ID;
    tx_msg.header.src_id = get_my_id();
    start_tdoa_report_thread();
}

void send_sync() {
    (void)pdec_radio_submit_sync();
}

void send_sync_now() {
    tx_msg.header.frame_control = FRAME_CONTROL;
    tx_msg.header.seq_number = frame_seq_nb;
    tx_msg.header.pan_id = MY_PAN_ID;
    tx_msg.header.dst_id = BROADCAST_ID;
    tx_msg.header.src_id = get_my_id();
    tx_msg.header.frame_role = DF_TDOA_SYNC;

    uint64_t systime = get_systime_u64()<<8;

    // en dtu (499.2MHz*128)
    uint32_t deltat = (SYNC_DELAY * UUS_TO_DWT_TIME);

    /* Compute response message transmission time. See NOTE 7 below. */
    uint64_t sync_tx_time = systime + deltat;
    dwt_setdelayedtrxtime(sync_tx_time>>8);

    tx_msg.sync.sync_ts = (sync_tx_time & 0xFFFFFFFE00) + TX_ANT_DLY;


    dwt_writetxdata(sizeof(tx_msg.sync), (uint8_t*)&tx_msg.sync, 0); /* Zero offset in TX buffer. */
    dwt_writetxfctrl(sizeof(tx_msg.sync), 0, 1);          /* Zero offset in TX buffer, ranging. */

    
    // int ret;
    // dwt_forcetrxoff();
    // dwt_writesysstatuslo(DWT_INT_TXFRS_BIT_MASK);
    // dwt_setrxtimeout(0);
    // ret = dwt_starttx(DWT_START_TX_DELAYED);
    // // ret = dwt_starttx(DWT_START_TX_IMMEDIATE);

    int ret;
    dwt_forcetrxoff();
    dwt_setrxaftertxdelay(0);
    dwt_setrxtimeout(0);
    status = POLLING;
    ret = dwt_starttx(DWT_START_TX_DELAYED | DWT_RESPONSE_EXPECTED);

    if (ret == DWT_SUCCESS)
    {
        // Increment frame sequence number after transmission of the poll message (modulo 256).
        frame_seq_nb++;
    } else {
        pdec_radio_restart_rx();
    }

}



bool handle_sync_msg(sync_msg_t* msg) {
    tdoa_role_t role = pdecTdoaGetRole();

    if(role == TDOA_ROLE_NONE) {
        return true;
    }

    uint64_t sync_rx_ts = get_rx_timestamp_u64();

    ////////////////////////////////////////////////////////////////////
    // update sync parameters
    
    master_id = msg->header.src_id;

    if(last_sync_rx_ts != 0 && last_sync_tx_ts != 0) {
        alpha = (double)timestamp_diff(sync_rx_ts, last_sync_rx_ts) / timestamp_diff(msg->sync_ts, last_sync_tx_ts);
        
        double distance = get_distance(get_my_id(), msg->header.src_id);
        uint64_t dt = distance / (SPEED_OF_LIGHT*DWT_TIME_UNITS);
        beta = timestamp_diff(sync_rx_ts, (uint64_t)(alpha*msg->sync_ts) + dt);
    }
    last_sync_rx_ts = sync_rx_ts;
    last_sync_tx_ts = msg->sync_ts;

    ////////////////////////////////////////////////////////////////////
    
    if(role != TDOA_ROLE_BLINKER) {
        // beacons just wait for a BLINK or an other SYNC
        status = POLLING;
        return true;
    } else {
        // BLINKER will send a BLINK soon (first time slot after SYNC)

        tx_msg.header.frame_control = FRAME_CONTROL;
        tx_msg.header.seq_number = frame_seq_nb;
        tx_msg.header.pan_id = MY_PAN_ID;
        tx_msg.header.dst_id = BROADCAST_ID;
        tx_msg.header.src_id = get_my_id();
        tx_msg.header.frame_role = DF_TDOA_BLINK;

        // en dtu (499.2MHz*128)
        uint32_t deltat = (SYNC_SLOT_TIME * UUS_TO_DWT_TIME);

        /* Compute response message transmission time. See NOTE 7 below. */
        uint64_t blink_tx_time = sync_rx_ts + deltat;
        dwt_setdelayedtrxtime(blink_tx_time>>8);

        tx_msg.blink.blink_id = blink_seq_nb;

        dwt_writetxdata(sizeof(tx_msg.sync), (uint8_t*)&tx_msg.sync, 0); /* Zero offset in TX buffer. */
        dwt_writetxfctrl(sizeof(tx_msg.sync), 0, 1);          /* Zero offset in TX buffer, ranging. */

        dwt_setrxaftertxdelay(0);
        dwt_setrxtimeout(0);
        status = POLLING;
        int ret = dwt_starttx(DWT_START_TX_DELAYED | DWT_RESPONSE_EXPECTED);

        if (ret == DWT_SUCCESS)
        {
            // Increment frame sequence number after transmission of the poll message (modulo 256).
            frame_seq_nb++;
            start_blink_report_collection(tx_msg.blink.blink_id);
            blink_seq_nb++;
            return false;
        } else {
            return true;
        }
    }


}


bool handle_blink_msg(blink_msg_t* msg) {
    tdoa_role_t role = pdecTdoaGetRole();

    if(role == TDOA_ROLE_NONE) {
        return true;
    }

    last_blink_id = msg->blink_id;

    int64_t blink_rx_ts = get_rx_timestamp_u64();
    int64_t blink_ts_master = blink_rx_ts;

    if(role != TDOA_ROLE_MASTER) {
        blink_ts_master = blink_rx_ts - beta;
        if(blink_ts_master < 0) {
            blink_ts_master += (int64_t)1<<40;
        }
        blink_ts_master /= alpha;
    }

    if(role == TDOA_ROLE_MASTER) {
        _master_blink_rx_ts = blink_rx_ts;
    }

    tx_msg.header.frame_control = FRAME_CONTROL;
    tx_msg.header.frame_role = DF_TDOA_BLINK_REPORT;
    tx_msg.header.dst_id = BROADCAST_ID;
    tx_msg.header.pan_id = MY_PAN_ID;
    tx_msg.header.src_id = get_my_id();
    tx_msg.header.seq_number = frame_seq_nb;

    tx_msg.blink_report.blink_id = msg->blink_id;
    tx_msg.blink_report.blink_snd_id = msg->header.src_id;
    tx_msg.blink_report.blink_rx_ts = blink_ts_master;

    // dwt_setrxtimeout(0);
    // dwt_rxenable(DWT_START_RX_IMMEDIATE);

    dwt_writetxdata(sizeof(tx_msg.blink_report), (uint8_t*)&tx_msg.blink_report, 0); /* Zero offset in TX buffer. */
    dwt_writetxfctrl(sizeof(tx_msg.blink_report), 0, 1);          /* Zero offset in TX buffer, ranging. */


    // en dtu (499.2MHz*128)
    uint32_t deltat = (SYNC_SLOT_TIME * UUS_TO_DWT_TIME) * (get_tdoa_index(get_my_id()) + 1);
    uint64_t blink_report_tx_time = blink_rx_ts + deltat;
    dwt_setdelayedtrxtime(blink_report_tx_time>>8);
    

    dwt_setrxaftertxdelay(0);
    dwt_setrxtimeout(0);
    status = POLLING;
    int ret = dwt_starttx(DWT_START_TX_DELAYED | DWT_RESPONSE_EXPECTED);

    if (ret == DWT_SUCCESS)
    {
        // Increment frame sequence number after transmission of the poll message (modulo 256).
        frame_seq_nb++;
        return false;
    } else {
        return true;
    }
}

bool handle_blink_report_msg(blink_report_msg_t* msg) {
    tdoa_role_t role = pdecTdoaGetRole();

    if(role != TDOA_ROLE_BLINKER) {
        return true;
    }

    collect_blink_report(msg);

    status = POLLING;
    // dwt_setrxaftertxdelay(0);
    return true;
}

static void pdec_sync_task(void *, void *, void *) {
    int sync_count = 0;
    int next_beacon_index = 0;

    // Do a few ranging rounds with known TDOA beacons before starting sync.
    for(int i=0; i<10; i++) {
        if(pdec_sync_wait_or_stop(K_NO_WAIT)) {
            pdec_sync_mark_stopped();
            return;
        }

        for(int j=0; get_device_id(j) != 0; j++) {
            uint16_t dev_id = get_device_id(j);
            if(dev_id != get_my_id() &&
               get_device_tdoa_role(dev_id) == TDOA_ROLE_BEACON) {
                send_poll(dev_id, true);
                if(pdec_sync_wait_or_stop(K_MSEC(100))) {
                    pdec_sync_mark_stopped();
                    return;
                }
            }
        }
    }

    // Periodically send sync while this node remains the active master.
    while(!pdec_sync_wait_or_stop(K_NO_WAIT)) {
        send_sync();
        sync_count++;
        if(pdec_sync_wait_or_stop(K_MSEC(SYNC_PERIOD_MS))) {
            break;
        }

        if(sync_count >= SYNC_PER_BEACON_RANGE) {
            bool ranged = false;

            // Range one beacon between sync bursts so new or moved beacons are folded in.
            for(int searched=0; searched < 10; searched++) {
                if(get_device_id(next_beacon_index) == 0) {
                    next_beacon_index = 0;
                }

                int j = next_beacon_index++;
                uint16_t dev_id = get_device_id(j);
                if(dev_id != get_my_id() &&
                   get_device_tdoa_role(dev_id) == TDOA_ROLE_BEACON) {
                    send_poll(dev_id, true);
                    if(pdec_sync_wait_or_stop(K_MSEC(100))) {
                        break;
                    }
                    ranged = true;
                    break;
                }
            }

            if(ranged) {
                sync_count = 0;
            }
        }
    }

    pdec_sync_mark_stopped();
}

tdoa_role_t pdecTdoaGetRole(void)
{
    return tdoa_role;
}

void pdecTdoaSetRole(tdoa_role_t role)
{
    if(tdoa_role == TDOA_ROLE_MASTER && role != TDOA_ROLE_MASTER) {
        k_event_post(&sync_events, SYNC_EVENT_STOP);
    }

    tdoa_role = role;

    set_device_tdoa_role(get_my_id(), (uint8_t)role);
}

void pdecTdoaStart(void) {
    if(tdoa_role == TDOA_ROLE_MASTER && !sync_thread_active) {
        sync_thread_active = true;
        k_event_clear(&sync_events, SYNC_EVENT_STOP);

        (void)k_thread_create(&sync_thread_data, sync_stack_area,
                              K_THREAD_STACK_SIZEOF(sync_stack_area),
                              pdec_sync_task,
                              NULL, NULL, NULL,
                              SYNC_TASK_PRIORITY, 0, K_NO_WAIT);
    }

}

void start_tdoa_monitor(const struct shell *shell)
{
    if (tdoa_monitor_running) {
        shell_print(shell, "TDOA monitor already running");
        return;
    }

    tdoa_monitor_stop_requested = false;
    tdoa_monitor_running = true;
    k_msgq_purge(&tdoa_monitor_queue);

    k_thread_create(&tdoa_monitor_thread_data, tdoa_monitor_stack,
                    K_THREAD_STACK_SIZEOF(tdoa_monitor_stack),
                    tdoa_monitor,
                    (void *)shell, NULL, NULL,
                    SYNC_TASK_PRIORITY, 0, K_NO_WAIT);
}

void stop_tdoa_monitor(void)
{
    struct tdoa_report_event event = {0};

    if (!tdoa_monitor_running) {
        return;
    }

    tdoa_monitor_stop_requested = true;
    if (k_msgq_put(&tdoa_monitor_queue, &event, K_NO_WAIT) != 0) {
        k_msgq_purge(&tdoa_monitor_queue);
        (void)k_msgq_put(&tdoa_monitor_queue, &event, K_NO_WAIT);
    }
}
