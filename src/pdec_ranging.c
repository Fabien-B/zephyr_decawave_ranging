#include"pdec_common.h"
#include "pdec_database.h"

#include <stdio.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/types.h>
#include <zephyr/random/random.h>

#include <deca_device_api.h>
#include <deca_probe_interface.h>
#include <dw3000_hw.h>
#include <dw3000_spi.h>

extern uint8_t frame_seq_nb;
extern volatile status_t status;

static union ranging_msg_u tx_msg;

void ranging_init() {
    tx_msg.header.frame_control = FRAME_CONTROL;
    tx_msg.header.pan_id = MY_PAN_ID;
    tx_msg.header.src_id = get_my_id();
}

void handle_resp_msg(resp_msg_t* msg, bool double_sided) {
    if( status != POLLING ||
        msg->header.dst_id != get_my_id() ||
        msg->poll_id != tx_msg.poll.poll_id)
    {
        status = LISTENING;
        dwt_rxenable(DWT_START_RX_IMMEDIATE);
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
        dwt_setdelayedtrxtime(final_tx_time>>8);

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

        /* Write and send the response message. See NOTE 9 below. */
        dwt_writetxdata(sizeof(tx_msg.final), (uint8_t*)&tx_msg.final, 0); /* Zero offset in TX buffer. */
        dwt_writetxfctrl(sizeof(tx_msg.final), 0, 1);          /* Zero offset in TX buffer, ranging. */

        dwt_writesysstatuslo(SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);
        // dwt_setrxtimeout(0);
        // status = LISTENING;
        int ret = dwt_starttx(DWT_START_TX_DELAYED);

        /* If dwt_starttx() returns an error, abandon this ranging exchange and proceed to the next one. See NOTE 10 below. */
        if (ret == DWT_SUCCESS)
        {
            // /* Increment frame sequence number after transmission of the poll message (modulo 256). */
            frame_seq_nb++;
        }
        else {
            printf("failed to send response. ret: %d\n", ret);
        }
    }

    set_distance(get_my_id(), msg->header.src_id, distance);

#if CONFIG_PDEC_PRINT_RANGING
    printf(">[%04x] SS DIST(%04x): %.2f\n", get_my_id(), msg->header.src_id, distance);
#endif

    dwt_setrxtimeout(0);
    status = LISTENING;

    if(!double_sided) {
        dwt_rxenable(DWT_START_RX_IMMEDIATE);
    }
}

void handle_poll_msg(poll_msg_t* msg, bool double_sided) {
    if( status != LISTENING ||
        (msg->header.dst_id != get_my_id() && msg->header.dst_id != BROADCAST_ID))
    {
        status = LISTENING;
        dwt_rxenable(DWT_START_RX_IMMEDIATE);
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
    dwt_setdelayedtrxtime(resp_tx_time>>8);

    uint64_t resp_tx_ts = (resp_tx_time & 0xFFFFFFFE00) + TX_ANT_DLY;

    tx_msg.resp.poll_rx_ts = (uint32_t)poll_rx_ts;
    tx_msg.resp.resp_tx_ts = (uint32_t)resp_tx_ts;

    dwt_writetxdata(sizeof(tx_msg.resp), (uint8_t*)&tx_msg.resp, 0); /* Zero offset in TX buffer. */
    dwt_writetxfctrl(sizeof(tx_msg.resp), 0, 1);          /* Zero offset in TX buffer, ranging. */

    dwt_writesysstatuslo(SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);
    
    int ret;

    if(double_sided) {
        dwt_setrxaftertxdelay(POLL_TX_TO_RESP_RX_DLY_UUS);
        dwt_setrxtimeout(RESP_RX_TIMEOUT_UUS);
        status = POLLING;
        ret = dwt_starttx(DWT_START_TX_DELAYED | DWT_RESPONSE_EXPECTED);
    } else {
        dwt_setrxtimeout(0);
        status = LISTENING;
        ret = dwt_starttx(DWT_START_TX_DELAYED);
    }
    
    /* If dwt_starttx() returns an error, abandon this ranging exchange and proceed to the next one. See NOTE 10 below. */
    if (ret == DWT_SUCCESS)
    {
        // /* Increment frame sequence number after transmission of the poll message (modulo 256). */
        frame_seq_nb++;
    }
    else {
        printf("failed to send response. ret: %d\n", ret);
    }
    
}

void handle_final_msg(final_msg_t* msg) {

    if(status == POLLING && msg->header.dst_id == get_my_id() && msg->resp_id == tx_msg.resp.resp_id) {
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
#if CONFIG_PDEC_PRINT_RANGING
        printf(">[%04x] DS DIST(%04x): %.2f\n", get_my_id(), msg->header.src_id, distance);
#endif
    }
    else if(status != POLLING) {
        double RA = (double)wrap32_diff(msg->resp_rx_ts, msg->poll_tx_ts);
        double rB = (double)wrap32_diff(msg->resp_tx_ts, msg->poll_rx_ts);

        double tof = ((RA* get_clock_offset(msg->header.src_id) - rB * get_clock_offset(msg->header.dst_id) ) / 2.0) * DWT_TIME_UNITS;
        double distance = tof * SPEED_OF_LIGHT;

        set_distance(msg->header.src_id, msg->header.dst_id, distance);

#if CONFIG_PDEC_PRINT_RANGING
        printf(">[%04x] IN DIST(%04x): %.2f\n", msg->header.src_id, msg->header.dst_id, distance);
#endif
    }
    else {
        // we are polling, but the message received is not for us. Fail.
    }
    status = LISTENING;
    dwt_setrxtimeout(0);
    dwt_rxenable(DWT_START_RX_IMMEDIATE);
}

void send_poll(uint16_t dst_id, bool double_sided) {
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

    dwt_writesysstatuslo(DWT_INT_TXFRS_BIT_MASK);
    dwt_writetxdata(sizeof(tx_msg.poll), (uint8_t*)&tx_msg.poll, 0); /* Zero offset in TX buffer. */
    dwt_writetxfctrl(sizeof(tx_msg.poll), 0, 1);          /* Zero offset in TX buffer, ranging. */
    
    // interrompt le RX permanent
    dwt_forcetrxoff();
    dwt_writesysstatuslo(DWT_INT_RX | DWT_INT_TXFRS_BIT_MASK);


    dwt_setrxaftertxdelay(POLL_TX_TO_RESP_RX_DLY_UUS);
    dwt_setrxtimeout(RESP_RX_TIMEOUT_UUS);
    status = POLLING;
    dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED);

    /* Increment frame sequence number after transmission of the poll message (modulo 256). */
    frame_seq_nb++;
}

