#include"pdec_common.h"
#include "pdec_database.h"
#include "pdec_ranging.h"
#include "pdec_tdoa.h"

#include <errno.h>
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

#define RADIO_STACK_SIZE 2048
#define RADIO_PRIORITY -2
#define RADIO_QUEUE_LEN 16

enum radio_event_kind {
    RADIO_EVENT_TX_DONE,
    RADIO_EVENT_RX_OK,
    RADIO_EVENT_RX_TIMEOUT,
    RADIO_EVENT_RX_ERROR,
    RADIO_EVENT_SEND_POLL,
    RADIO_EVENT_SEND_DATA,
    RADIO_EVENT_SEND_SYNC,
};

struct radio_event {
    enum radio_event_kind kind;
    union {
        struct {
            uint16_t dst_id;
            bool double_sided;
        } poll;
        struct {
            uint16_t dst_id;
            uint8_t len;
            uint8_t payload[PAYLOAD_FRAME_LEN_MAX];
        } data;
    };
};

K_MSGQ_DEFINE(radio_queue, sizeof(struct radio_event), RADIO_QUEUE_LEN, 4);
K_THREAD_STACK_DEFINE(radio_stack, RADIO_STACK_SIZE);
static struct k_thread radio_thread_data;
static bool radio_thread_started;

static void cbTxDone(const dwt_cb_data_t *cb_data);
static void cbRxOk(const dwt_cb_data_t *cb_data);
static void cbRxTo(const dwt_cb_data_t *cb_data);
static void cbRxErr(const dwt_cb_data_t *cb_data);
static void handle_com_msg(const com_msg_t *msg);
static void process_rx_ok(void);
static void send_data_msg_now(uint16_t dst_id, uint8_t* buffer, uint8_t len);
void send_poll_now(uint16_t dst_id, bool double_sided);
void send_sync_now(void);


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

static int submit_radio_event(const struct radio_event *event)
{
    return k_msgq_put(&radio_queue, event, K_NO_WAIT);
}

static void radio_thread(void *p1, void *p2, void *p3)
{
    ARG_UNUSED(p1);
    ARG_UNUSED(p2);
    ARG_UNUSED(p3);

    struct radio_event event;

    while (true) {
        if (k_msgq_get(&radio_queue, &event, K_FOREVER) != 0) {
            continue;
        }

        switch (event.kind) {
        case RADIO_EVENT_TX_DONE:
            dwt_writesysstatuslo(DWT_INT_TXFRS_BIT_MASK);
            break;
        case RADIO_EVENT_RX_OK:
            process_rx_ok();
            break;
        case RADIO_EVENT_RX_TIMEOUT:
            dwt_writesysstatuslo(DWT_INT_TXFRS_BIT_MASK);
            pdec_radio_restart_rx();
            break;
        case RADIO_EVENT_RX_ERROR:
            pdec_radio_restart_rx();
            break;
        case RADIO_EVENT_SEND_POLL:
            send_poll_now(event.poll.dst_id, event.poll.double_sided);
            break;
        case RADIO_EVENT_SEND_DATA:
            send_data_msg_now(event.data.dst_id, event.data.payload, event.data.len);
            break;
        case RADIO_EVENT_SEND_SYNC:
            send_sync_now();
            break;
        }
    }
}

static void start_radio_thread(void)
{
    if (radio_thread_started) {
        return;
    }

    k_thread_create(&radio_thread_data, radio_stack, K_THREAD_STACK_SIZEOF(radio_stack),
                    radio_thread, NULL, NULL, NULL, RADIO_PRIORITY, 0, K_NO_WAIT);
    radio_thread_started = true;
}

static void cbRxTo(const dwt_cb_data_t *cb_data) {
    ARG_UNUSED(cb_data);

    struct radio_event event = {
        .kind = RADIO_EVENT_RX_TIMEOUT,
    };

    (void)submit_radio_event(&event);
}

static void cbRxErr(const dwt_cb_data_t *cb_data) {
    ARG_UNUSED(cb_data);

    struct radio_event event = {
        .kind = RADIO_EVENT_RX_ERROR,
    };

    (void)submit_radio_event(&event);
}

static void cbTxDone(const dwt_cb_data_t *cb_data) {
    ARG_UNUSED(cb_data);

    struct radio_event event = {
        .kind = RADIO_EVENT_TX_DONE,
    };

    (void)submit_radio_event(&event);
}

static void handle_com_msg(const com_msg_t *msg)
{
    uint8_t len = msg->len;

    if (len > PAYLOAD_FRAME_LEN_MAX) {
        len = PAYLOAD_FRAME_LEN_MAX;
    }

    publish_com_event(msg->header.src_id, msg->payload, len);
    pdec_radio_restart_rx();
}






static void process_rx_ok(void)
{
    uint8_t rng;
    
    dwt_writesysstatuslo(DWT_INT_RXFCG_BIT_MASK);
    uint16_t frame_len = dwt_getframelength(&rng);

    // to check if the range bit is set: rng & DWT_CB_DATA_RX_FLAG_RNG

    if (frame_len > sizeof(rx_msg)) {
        pdec_radio_restart_rx();
        return;
    }

    dwt_readrxdata((uint8_t*)&rx_msg, frame_len, 0);
    frame_header_t* header = (frame_header_t*)&rx_msg;

    // check the type of the frame and the PAN ID.
    if(header->frame_control != FRAME_CONTROL || header->pan_id != MY_PAN_ID) {
        pdec_radio_restart_rx();
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
        //printf("BLINK REPORT\n");
        break;
    case DF_COM_MSG:
        handle_com_msg(&rx_msg.com);
        break;
    default:
        pdec_radio_restart_rx();
        break;
    }
}

static void cbRxOk(const dwt_cb_data_t *cb_data)
{
    ARG_UNUSED(cb_data);

    struct radio_event event = {
        .kind = RADIO_EVENT_RX_OK,
    };

    (void)submit_radio_event(&event);
}

void send_data_msg(uint16_t dst_id, uint8_t* buffer, uint8_t len) {
    (void)pdec_radio_submit_data(dst_id, buffer, len);
}

static void send_data_msg_now(uint16_t dst_id, uint8_t* buffer, uint8_t len) {
    com_msg_t msg;
    if(len > PAYLOAD_FRAME_LEN_MAX) {
        pdec_radio_restart_rx();
        return;
    }

    msg.header.frame_control = FRAME_CONTROL;
    msg.header.seq_number = frame_seq_nb;
    msg.header.pan_id = MY_PAN_ID;
    msg.header.dst_id = dst_id;
    msg.header.src_id = get_my_id();
    msg.header.frame_role = DF_COM_MSG;
    msg.len = len;
    memcpy(msg.payload, buffer, len);

    dwt_writesysstatuslo(DWT_INT_TXFRS_BIT_MASK);
    dwt_writetxdata(COM_MSG_SIZE(len), (uint8_t*)&msg, 0); /* Zero offset in TX buffer. */
    dwt_writetxfctrl(COM_MSG_SIZE(len), 0, 0);          /* Zero offset in TX buffer, no ranging. */
    
    // interrompt le RX permanent
    dwt_forcetrxoff();
    dwt_writesysstatuslo(DWT_INT_RX | DWT_INT_TXFRS_BIT_MASK);

    dwt_setrxaftertxdelay(0);
    int ret = dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED);
    if (ret != DWT_SUCCESS) {
        pdec_radio_restart_rx();
        return;
    }

    frame_seq_nb++;
}

void pdec_radio_restart_rx(void)
{
    dwt_setrxtimeout(0);
    dwt_rxenable(DWT_START_RX_IMMEDIATE);
}

int pdec_radio_submit_poll(uint16_t dst_id, bool double_sided)
{
    struct radio_event event = {
        .kind = RADIO_EVENT_SEND_POLL,
        .poll = {
            .dst_id = dst_id,
            .double_sided = double_sided,
        },
    };

    return submit_radio_event(&event);
}

int pdec_radio_submit_data(uint16_t dst_id, const uint8_t *buffer, uint8_t len)
{
    struct radio_event event = {
        .kind = RADIO_EVENT_SEND_DATA,
        .data = {
            .dst_id = dst_id,
            .len = len,
        },
    };

    if (len > PAYLOAD_FRAME_LEN_MAX) {
        return -EMSGSIZE;
    }

    memcpy(event.data.payload, buffer, len);
    return submit_radio_event(&event);
}

int pdec_radio_submit_sync(void)
{
    struct radio_event event = {
        .kind = RADIO_EVENT_SEND_SYNC,
    };

    return submit_radio_event(&event);
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
    if(ret != DWT_SUCCESS) {
        ARG_UNUSED(id);
        return ret;
	}
    
    if(dev_id != NULL) {
        *dev_id = id;
    }

    // Wait for DW IC to be in IDLE_RC
    while (!dwt_checkidlerc())  {
       k_sleep(K_MSEC(100));
    };

    ret = dwt_initialise(DWT_DW_INIT);
    if (ret != DWT_SUCCESS) {
        return ret;
    }
    dw3000_spi_speed_fast();
    
    ret = dwt_configure(&config);
    if (ret != DWT_SUCCESS) {
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

    start_radio_thread();

    dwt_setrxtimeout(0);
    dwt_rxenable(DWT_START_RX_IMMEDIATE);


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
