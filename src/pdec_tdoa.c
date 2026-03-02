#include"pdec_common.h"
#include "pdec_database.h"
#include "pdec_tdoa.h"

#include <stdio.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/types.h>
#include <zephyr/random/random.h>

#include <deca_device_api.h>
#include <deca_probe_interface.h>
#include <dw3000_hw.h>
#include <dw3000_spi.h>

#define SYNC_DELAY 2000  //us
#define SYNC_SLOT_TIME 10000  //us


#define SYNC_TASK_STACK_SIZE 1000
#define SYNC_TASK_PRIORITY 5
K_THREAD_STACK_DEFINE(sync_stack_area, SYNC_TASK_STACK_SIZE);
struct k_thread sync_thread_data;

static tdoa_role_t tdoa_role;

uint64_t _master_blink_rx_ts = 0;

extern uint8_t frame_seq_nb;
extern volatile status_t status;

static union ranging_msg_u tx_msg;


static uint64_t last_sync_rx_ts = 0;    // in my clock
static uint64_t last_sync_tx_ts = 0;    // in master clock

uint16_t blink_seq_nb = 0;


double alpha = 1;
int64_t beta = 0;

void tdoa_init() {
    tx_msg.header.frame_control = FRAME_CONTROL;
    tx_msg.header.pan_id = MY_PAN_ID;
    tx_msg.header.src_id = get_my_id();
}

void send_sync() {
    tx_msg.header.frame_control = FRAME_CONTROL;
    tx_msg.header.pan_id = MY_PAN_ID;
    tx_msg.header.dst_id = BROADCAST_ID;
    tx_msg.header.src_id = get_my_id();
    tx_msg.header.seq_number = frame_seq_nb;
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
    // status = LISTENING;
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
    }
    else {
        printf("failed to send SYNC. ret: %d\n", ret);
    }

}



void handle_sync_msg(sync_msg_t* msg) {
    uint64_t sync_rx_ts = get_rx_timestamp_u64();

    ////////////////////////////////////////////////////////////////////
    // update sync parameters
    if(last_sync_rx_ts != 0 && last_sync_tx_ts != 0) {
        alpha = (double)timestamp_diff(sync_rx_ts, last_sync_rx_ts) / timestamp_diff(msg->sync_ts, last_sync_tx_ts);
        
        double distance = get_distance(get_my_id(), msg->header.src_id);
        uint64_t dt = distance / (SPEED_OF_LIGHT*DWT_TIME_UNITS);
        beta = timestamp_diff(sync_rx_ts, (uint64_t)(alpha*msg->sync_ts) + dt);
        double beta_s = beta * DWT_TIME_UNITS;
        //printf("Sync: alpha=%f,  beta=%f\n", alpha, beta_s);
    }
    last_sync_rx_ts = sync_rx_ts;
    last_sync_tx_ts = msg->sync_ts;
    ////////////////////////////////////////////////////////////////////
    
    if(tdoa_role != TDOA_ROLE_BLINKER) {
        // beacons just wait for a BLINK or an other SYNC
        dwt_setrxtimeout(0);
        status = LISTENING;
        dwt_rxenable(DWT_START_RX_IMMEDIATE);
    } else {
        // BLINKER will send a BLINK soon (first time slot after SYNC)

        tx_msg.header.frame_control = FRAME_CONTROL;
        tx_msg.header.pan_id = MY_PAN_ID;
        tx_msg.header.dst_id = BROADCAST_ID;
        tx_msg.header.src_id = get_my_id();
        tx_msg.header.seq_number = frame_seq_nb;
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
            printf(">BlinkTX:%llu\n", blink_tx_time);
        }
        else {
            printf("failed to send SYNC. ret: %d\n", ret);
        }
    }


}


void handle_blink_msg(blink_msg_t* msg) {
    bool plop = false;
    int64_t blink_rx_ts = get_rx_timestamp_u64();
    
    if(tdoa_role == TDOA_ROLE_MASTER) {
        // the master does not need to report
        _master_blink_rx_ts = blink_rx_ts;
        printf(">MBlink: %lld\n", blink_rx_ts);
        dwt_setrxtimeout(0);
        status = LISTENING;
        dwt_rxenable(DWT_START_RX_IMMEDIATE);
        return;
    }
    

    
    double distance = 0;    //get_distance(get_my_id(), msg->header.src_id);
    int64_t dt = distance / (SPEED_OF_LIGHT*DWT_TIME_UNITS);
    
    int64_t blink_ts_master = (blink_rx_ts - (beta + dt));
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
    // status = LISTENING;
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
        if(plop) {
            printf(">OBlink[%x]:%lld\n", get_my_id(), blink_ts_master);
        } else {
            printf(">XBlink[%x]:%lld\n", get_my_id(), blink_ts_master);
        }
    }
    else {
        printf("failed to send blink report. ret: %d\n", get_tdoa_index(get_my_id()));
    }
}

void handle_blink_report_msg(blink_report_msg_t* msg) {

    // only the master process blik reports
    if(tdoa_role == TDOA_ROLE_MASTER) {
        static uint32_t blink_id = 0;
        int64_t dt = timestamp_diff_signed(msg->blink_rx_ts, _master_blink_rx_ts) ;
        double distance = dt * DWT_TIME_UNITS * SPEED_OF_LIGHT;
        printf("dt: %llu - %llu = %lld -> %.2f\n", msg->blink_rx_ts, _master_blink_rx_ts, dt, distance);
        if(msg->blink_id != blink_id) {
            printf("\n\n");
        }
        blink_id = msg->blink_id;
    }


    dwt_setrxtimeout(0);
    status = LISTENING;
    // dwt_setrxaftertxdelay(0);
    dwt_rxenable(DWT_START_RX_IMMEDIATE);
}

static void pdec_sync_task(void *, void *, void *) {
    // do a few ranging before static TDOA
    for(int i=0; i<10; i++) {
        for(int i=0; devices[i] != 0; i++) {
            if(devices[i] != get_my_id()) {
                send_poll(devices[i], true);
                k_msleep(100);
            }
        }
    
    }
    // Then a few SYNCs
    for(int i=0; i<10; i++) {
        send_sync();
        k_msleep(100);
    }

    // then BLINK
    while(true) {
        send_sync();
        k_msleep(200);
    }
}

void pdecTdoaStart(tdoa_role_t role) {
    
    tdoa_role = role;

    if(role == TDOA_ROLE_MASTER) {
        k_tid_t my_tid = k_thread_create(&sync_thread_data, sync_stack_area,
                                         K_THREAD_STACK_SIZEOF(sync_stack_area),
                                         pdec_sync_task,
                                         NULL, NULL, NULL,
                                         SYNC_TASK_PRIORITY, 0, K_NO_WAIT);
    }

}

