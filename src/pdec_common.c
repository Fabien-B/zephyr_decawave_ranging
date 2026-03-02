#include"pdec_common.h"
#include "pdec_database.h"
#include "pdec_ranging.h"
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


static void cbTxDone(const dwt_cb_data_t *cb_data);
static void cbRxOk(const dwt_cb_data_t *cb_data);
static void cbRxTo(const dwt_cb_data_t *cb_data);
static void cbRxErr(const dwt_cb_data_t *cb_data);


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
uint8_t frame_seq_nb = 0;
volatile status_t status = IDLE;

static union ranging_msg_u rx_msg;


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
        printf("Bad frame: 0x%x, 0x%x\n", header->frame_control, header->frame_role);
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
    case DF_TDOA_SYNC:
        handle_sync_msg((sync_msg_t*)header);
        break;
    case DF_TDOA_BLINK:
        handle_blink_msg((blink_msg_t*)header);
        break;
    case DF_TDOA_BLINK_REPORT:
        handle_blink_report_msg((blink_report_msg_t*)header);
        break;
    default:
        printf("Unknown frame role: %d\n", header->frame_role);
    }
}

int pdecInit(uint32_t* dev_id) {

    ranging_init();

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

uint64_t get_systime_u64(void)
{
    uint64_t ts = 0;
    // it works because of little-endian
    dwt_readsystime(((uint8_t*)&ts));
    return ts;
}

uint16_t get_my_id() {
    return (uint16_t)NRF_FICR->DEVICEID[0];
}

uint32_t wrap32_diff(uint32_t hi, uint32_t lo) {
    if(hi > lo) {
        return hi - lo;
    } else {
        return ((uint64_t)hi + ((uint64_t)1<<32)) - (uint64_t)lo;
    }
}

/**
 * compute 40 bit-timestamp diff
 */
uint64_t timestamp_diff(uint64_t a, uint64_t b) {
    return (a - b) & 0xFFFFFFFFFFULL;
}

int64_t timestamp_diff_signed(uint64_t a, uint64_t b) {
    const uint64_t MASK40 = (1ULL << 40) - 1;
    const uint64_t SIGN40 = 1ULL << 39;

    uint64_t d = (a - b) & MASK40;

    // conversion en signé sur 40 bits
    if (d & SIGN40)
        d |= ~MASK40;

    return (int64_t)d;
}
