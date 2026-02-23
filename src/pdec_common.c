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


static void cbTxDone(const dwt_cb_data_t *cb_data);
static void cbRxOk(const dwt_cb_data_t *cb_data);
static void cbRxTo(const dwt_cb_data_t *cb_data);
static void cbRxErr(const dwt_cb_data_t *cb_data);

static void handle_poll_msg(poll_msg_t* msg, bool double_sided);
static void handle_resp_msg(resp_msg_t* msg, bool double_sided);
static void handle_final_msg(final_msg_t* msg);

static uint32_t wrap32_diff(uint32_t hi, uint32_t lo);


// frame control:
// 2:0    001  data
// 3      0    no security
// 4      0    no pending
// 5      0    no ack
// 6      1    pan id compression  --> no src pan id
// 9:7    000  
// 11:10  10   dest addr mode: 16-bit short address
// 13:12  00   IEEE 802.15.4‑2003
// 15:14  10   src addr mode:  16-bit short address
#define FRAME_CONTROL 0x8841

/* Default antenna delay values for 64 MHz PRF. See NOTE 2 below. */
#define TX_ANT_DLY 16385
#define RX_ANT_DLY 16385



// delay between frame RX, and the delayed TX
#define POLL_RX_TO_RESP_TX_DLY_UUS 1500

// delay before enabling RX after end of TX
#define POLL_TX_TO_RESP_RX_DLY_UUS 1000


#define RESP_RX_TIMEOUT_UUS 1200


#define SPEED_OF_LIGHT   (299702547)    //in m/s
#define FRAME_LEN_MAX    (127)
#define FRAME_LEN_MAX_EX (1023)

#define RXFLEN_MASK    0x0000007FUL /* Receive Frame Length (0 to 127) */
#define RXFL_MASK_1023 0x000003FFUL /* Receive Frame Length Extension (0 to 1023) */

/* UWB microsecond (uus) to device time unit (dtu, around 15.65 ps) conversion factor.
 * 1 uus = 512 / 499.2 µs and 1 µs = 499.2 * 128 dtu. */
 // F*dt = (499.2MHz*128) * 1e-6s = 63897.6   timestamp increment per microseconds
#define UUS_TO_DWT_TIME 63898



dwt_config_t config = {
    5,                /* Channel number. */
    DWT_PLEN_128,     /* Preamble length. Used in TX only. */
    DWT_PAC8,         /* Preamble acquisition chunk size. Used in RX only. */
    9,                /* TX preamble code. Used in TX only. */
    9,                /* RX preamble code. Used in RX only. */
    1,                /* 0 to use standard 8 symbol SFD, 1 to use non-standard 8 symbol, 2 for non-standard 16 symbol SFD and 3 for 4z 8 symbol SDF type */
    DWT_BR_6M8,       /* Data rate. */
    DWT_PHRMODE_STD,  /* PHY header mode. */
    DWT_PHRRATE_STD,  /* PHY header rate. */
    (129 + 8 - 8),    /* SFD timeout (preamble length + 1 + SFD length - PAC size). Used in RX only. */
    DWT_STS_MODE_OFF, /* No STS mode enabled (STS Mode 0). */
    DWT_STS_LEN_64,   /* STS length, see allowed values in Enum dwt_sts_lengths_e */
    DWT_PDOA_M0       /* PDOA mode off */
};

/*
 * TX Power Configuration Settings
 */
/* Values for the PG_DELAY and TX_POWER registers reflect the bandwidth and power of the spectrum at the current
 * temperature. These values can be calibrated prior to taking reference measurements. */
dwt_txconfig_t txconfig_options = {
    .PGdly = 0x34,       // PG delay
    .power = 0xfdfdfdfd, // TX power
    .PGcount = 0x0       //PG count
};


dwt_callbacks_s dw_callbacks = {
    .cbTxDone = cbTxDone,
    .cbRxOk = cbRxOk,
    .cbRxTo = cbRxTo,
    .cbRxErr = cbRxErr,
    .cbSPIErr = NULL,
    .cbSPIRDErr = NULL,
    .cbSPIRdy = NULL,
    .cbDualSPIEv = NULL,
    .cbFrmRdy = NULL,
    .cbCiaDone = NULL,
    .devErr = NULL,
    .cbSysEvent = NULL,
};

/* Frame sequence number, incremented after each transmission. */
static uint8_t frame_seq_nb = 0;
volatile status_t status = IDLE;

union ranging_msg_u rx_msg;
union ranging_msg_u tx_msg;

extern const struct gpio_dt_spec led;


static void cbRxTo(const dwt_cb_data_t *cb_data) {
    status = LISTENING;
    dwt_writesysstatuslo(DWT_INT_TXFRS_BIT_MASK);
    dwt_setrxtimeout(0);
    dwt_rxenable(DWT_START_RX_IMMEDIATE);
}

static void cbRxErr(const dwt_cb_data_t *cb_data) {
    printf("RxErr\n");
}

static void cbTxDone(const dwt_cb_data_t *cb_data) {
    gpio_pin_toggle_dt(&led);

    switch (status)
    {
    case IDLE:
        // doing nothing ? ??
        break;
    case POLLING:
        // should already be in RX
        break;
    case LISTENING:
        dwt_writesysstatuslo(DWT_INT_TXFRS_BIT_MASK);
        dwt_rxenable(DWT_START_RX_IMMEDIATE);
        break;
    }
}






static void cbRxOk(const dwt_cb_data_t *cb_data) {
    gpio_pin_toggle_dt(&led);
    uint8_t rng;
    
    dwt_writesysstatuslo(DWT_INT_RXFCG_BIT_MASK);
    uint16_t frame_len = dwt_getframelength(&rng);

    // to check if the range bit is set: rng & DWT_CB_DATA_RX_FLAG_RNG

    if (frame_len > sizeof(rx_msg)) {
        return;
    }

    dwt_readrxdata((uint8_t*)&rx_msg, frame_len, 0);
    frame_header_t* header = (frame_header_t*)&rx_msg;

    // check the type of the frame and the PAN ID.
    if(header->frame_control != FRAME_CONTROL || header->pan_id != MY_PAN_ID) {
        return;
    }

    // Read carrier integrator value and calculate clock offset ratio.
    double clockOffsetRatio = ((float)dwt_readclockoffset()) / (uint32_t)(1 << 26);
    set_clock_offset(header->src_id, clockOffsetRatio);

    switch (header->frame_role)
    {
    case DF_SS_RANGING_POLL:
        handle_poll_msg((poll_msg_t*)header, false);
        break;
    case DF_DS_RANGING_POLL:
        handle_poll_msg((poll_msg_t*)header, true);
        break;
    case DF_SS_RANGING_RESP:
        handle_resp_msg((resp_msg_t*)header, false);
        break;
    case DF_DS_RANGING_RESP:
        handle_resp_msg((resp_msg_t*)header, true);
        break;
    case DF_DS_RANGING_FINAL:
        handle_final_msg((final_msg_t*)header);
        break;
    }
}

static void handle_resp_msg(resp_msg_t* msg, bool double_sided) {
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

    printf(">[%04x] SS DIST(%04x): %d\n", get_my_id(), msg->header.src_id, (int)(distance*100));

    dwt_setrxtimeout(0);
    status = LISTENING;

    if(!double_sided) {
        dwt_rxenable(DWT_START_RX_IMMEDIATE);
    }
}

static void handle_poll_msg(poll_msg_t* msg, bool double_sided) {
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

static void handle_final_msg(final_msg_t* msg) {

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

        printf(">[%04x] DS DIST(%04x): %d\n", get_my_id(), msg->header.src_id, (int)(distance*100));
    }
    else if(status != POLLING) {
        double RA = (double)wrap32_diff(msg->resp_rx_ts, msg->poll_tx_ts);
        double rB = (double)wrap32_diff(msg->resp_tx_ts, msg->poll_rx_ts);

        double tof = ((RA* get_clock_offset(msg->header.src_id) - rB * get_clock_offset(msg->header.dst_id) ) / 2.0) * DWT_TIME_UNITS;
        double distance = tof * SPEED_OF_LIGHT;
        printf(">[%04x] IN DIST(%04x): %d\n", msg->header.src_id, msg->header.dst_id, (int)(distance*100));
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


int pdecInit(uint32_t* dev_id) {
    // Init Tx frame with all constant values
    tx_msg.header.frame_control = FRAME_CONTROL;
    tx_msg.header.pan_id = MY_PAN_ID;
    tx_msg.header.src_id = get_my_id();

    // init DW driver
    dw3000_hw_init();
    dw3000_hw_reset();
    dwt_probe((struct dwt_probe_s *)&dw3000_probe_interf);
    dw3000_hw_init_interrupt();
    dwt_setcallbacks(&dw_callbacks);

    k_sleep(K_MSEC(500));


    uint32_t id = dwt_readdevid();
	int ret = dwt_check_dev_id();
    if(ret == DWT_SUCCESS) {
		printf("Good device ID: %x\n", id);
	} else {
		printf("Bad device ID: %x\n", id);
        return ret;
	}
    *dev_id = id;

    // Wait for DW IC to be in IDLE_RC
    while (!dwt_checkidlerc())  {
       k_sleep(K_MSEC(100));
    };

    ret = dwt_initialise(DWT_DW_INIT);
    if (ret != DWT_SUCCESS) {
        printf("deca INIT FAILED\n");
        return ret;
    }
    dw3000_spi_speed_fast();
    
    ret = dwt_configure(&config);
    if (ret != DWT_SUCCESS) {
        printf("deca CONFIG FAILED\n");
        return ret;
    }
    
    dwt_setinterrupt(
        DWT_INT_TXFRS_BIT_MASK   |  // TX done
        DWT_INT_RXFCG_BIT_MASK   |  // RX OK
        DWT_INT_RXFCE_BIT_MASK   |  // CRC error
        DWT_INT_RXFTO_BIT_MASK   |  // RX timeout
        DWT_INT_RXSTO_BIT_MASK   |  // SFD timeout
        DWT_INT_RXPHE_BIT_MASK,     // PHY header error
        0,
        DWT_ENABLE_INT
    );


    /* Configure the TX spectrum parameters (power PG delay and PG Count) */
    dwt_configuretxrf(&txconfig_options);

    /* Apply default antenna delay value. See NOTE 2 below. */
    dwt_setrxantennadelay(RX_ANT_DLY);
    dwt_settxantennadelay(TX_ANT_DLY);

    /* Next can enable TX/RX states output on GPIOs 5 and 6 to help debug, and also TX/RX LEDs
     * Note, in real low power applications the LEDs should not be used. */
    dwt_setlnapamode(DWT_LNA_ENABLE | DWT_PA_ENABLE);

    // Enabling LEDs so that for each TX the D1 LED will flash on DW3000 red eval-shield boards.
    dwt_setleds(DWT_LEDS_ENABLE | DWT_LEDS_INIT_BLINK);

    dwt_setrxtimeout(0);
    dwt_rxenable(DWT_START_RX_IMMEDIATE);
    status = LISTENING;


    return ret;
}



uint64_t get_rx_timestamp_u64(void)
{
    uint64_t ts = 0;
    dwt_readrxtimestamp((uint8_t*)&ts, DWT_STS0_M);
    return ts;
}

uint64_t get_tx_timestamp_u64(void)
{
    uint64_t ts = 0;
    // it works because of little-endian
    dwt_readtxtimestamp(((uint8_t*)&ts));
    return ts;
}

uint16_t get_my_id() {
    return (uint16_t)NRF_FICR->DEVICEID[0];
}

static uint32_t wrap32_diff(uint32_t hi, uint32_t lo) {
    if(hi > lo) {
        return hi - lo;
    } else {
        return ((uint64_t)hi + ((uint64_t)1<<32)) - (uint64_t)lo;
    }
}
