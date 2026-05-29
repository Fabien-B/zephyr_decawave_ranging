#include"pdec_common.h"
#include "pdec_database.h"
#include "pdec_tdoa.h"
#include "pdec_ranging.h"

#include <stdio.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/types.h>
#include <zephyr/random/random.h>

#include <deca_device_api.h>
#include <deca_probe_interface.h>
#include <dw3000_hw.h>
#include <dw3000_spi.h>

#include "app_print.h"

#define SYNC_DELAY 2000  //us
#define SYNC_SLOT_TIME 5000  //us
#define SYNC_PERIOD_MS 1000
#define SYNC_PER_BEACON_RANGE 10


#define SYNC_TASK_STACK_SIZE 1000
#define SYNC_TASK_PRIORITY 5
#define SYNC_EVENT_STOP BIT(0)

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

    bool plop = false;
    int64_t blink_rx_ts = get_rx_timestamp_u64();
    
    if(role == TDOA_ROLE_MASTER) {
        // the master does not need to report
        _master_blink_rx_ts = blink_rx_ts;
        status = POLLING;
        return true;
    }
    

    
    // double distance = get_distance(get_my_id(), master_id);
    // int64_t dt = distance / (SPEED_OF_LIGHT*DWT_TIME_UNITS);
    
    int64_t blink_ts_master = (blink_rx_ts - (beta));
    if(blink_ts_master < 0) {
        blink_ts_master += (int64_t)1<<40;
        plop = true;
    }

    blink_ts_master /= alpha;

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

    if(role == TDOA_ROLE_NONE) {
        return true;
    }

    // only the master process blik reports
    if(role == TDOA_ROLE_MASTER) {
        if(msg->blink_id == last_blink_id) {
            int64_t dt = timestamp_diff_signed(_master_blink_rx_ts, msg->blink_rx_ts) ;
            double distance = dt * DWT_TIME_UNITS * SPEED_OF_LIGHT;
            ARG_UNUSED(distance);
        }
    }


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
