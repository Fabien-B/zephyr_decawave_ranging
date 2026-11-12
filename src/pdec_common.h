#pragma once
#include <stddef.h>
#include <inttypes.h>
#include <stdbool.h>

#define MY_PAN_ID 0xDEAD
#define BROADCAST_ID 0xFFFE

#define PAYLOAD_FRAME_LEN_MAX 113


typedef struct __attribute__((packed)) {
    uint16_t frame_control;
    uint8_t seq_number;
    uint16_t pan_id;
    uint16_t dst_id;
    uint16_t src_id;
    uint8_t  frame_role;
} frame_header_t;


typedef struct __attribute__((packed)) {
    frame_header_t header;
    uint8_t poll_id; // for matching the response with the poll
    uint16_t chk;   // checksum automatically set by DW IC
} poll_msg_t;


typedef struct __attribute__((packed)) {
    frame_header_t header;
    uint8_t poll_id; // follow-up on the poll
    uint32_t poll_rx_ts;
    uint32_t resp_tx_ts;
    uint8_t resp_id; // for matching the final with the response
    uint16_t chk;   // checksum automatically set by DW IC
} resp_msg_t;


typedef struct __attribute__((packed)) {
    frame_header_t header;
    uint8_t resp_id; // follow-up on the response
    uint32_t poll_tx_ts;
    uint32_t poll_rx_ts;
    uint32_t resp_tx_ts;
    uint32_t resp_rx_ts;
    uint32_t final_tx_ts;
    uint16_t chk;   // checksum automatically set by DW IC
} final_msg_t;



typedef struct __attribute__((packed)) {
    frame_header_t header;
    uint64_t sync_ts;   // sync timestamp, in the clock of the master (the sender)
    uint16_t chk;       // checksum automatically set by DW IC
} sync_msg_t;

typedef struct __attribute__((packed)) {
    frame_header_t header;
    uint32_t blink_id;
    uint16_t chk;   // checksum automatically set by DW IC
} blink_msg_t;

typedef struct __attribute__((packed)) {
    frame_header_t header;
    uint32_t blink_id;
    uint64_t blink_rx_ts;   // blink receive timestamp, in the clock of the master
    uint16_t blink_snd_id;  // blink sender id
    uint16_t chk;   // checksum automatically set by DW IC
} blink_report_msg_t;

typedef struct __attribute__((packed)) {
    frame_header_t header;
    uint8_t protocol_version;
    uint8_t tdoa_role;
    uint16_t chk;   // checksum automatically set by DW IC
} discovery_msg_t;

#define COM_MSG_SIZE(len) (len + sizeof(frame_header_t) + 1 + 3)

typedef struct __attribute__((packed)) {
    frame_header_t header;
    // lenght of payload. Don't forget the checksum !
    uint8_t len;
    // 2 extra bytes for the checksum, automatically set by DW IC
    uint8_t payload[PAYLOAD_FRAME_LEN_MAX+2];
} com_msg_t;

union ranging_msg_u
{
    frame_header_t header;
    poll_msg_t poll;
    resp_msg_t resp;
    final_msg_t final;

    sync_msg_t sync;
    blink_msg_t blink;
    blink_report_msg_t blink_report;
    discovery_msg_t discovery;

    com_msg_t com;
};



typedef enum {
    DF_SS_RANGING_POLL =  0xE0,
    DF_SS_RANGING_RESP =  0xE1,
    DF_DS_RANGING_POLL =  0xE2,
    DF_DS_RANGING_RESP =  0xE3,
    DF_DS_RANGING_FINAL = 0xE4,

    DF_TDOA_SYNC = 0xE5,
    DF_TDOA_BLINK = 0xE6,
    DF_TDOA_BLINK_REPORT = 0xE7,
    DF_COM_MSG = 0xE8,
    DF_DISCOVERY_HELLO = 0xE9,
} frame_role_t;


/* Starts the radio owner and waits for hardware initialization. Call once from
 * thread context. Commands submitted before success return -ENODEV.
 */
int pdecInit(uint32_t* dev_id);
/* Hardware timestamp access is reserved for the radio owner thread. */
uint64_t get_rx_timestamp_u64(void);
uint64_t get_tx_timestamp_u64(void);
uint64_t get_systime_u64(void);
uint32_t wrap32_diff(uint32_t hi, uint32_t lo);
uint64_t timestamp_diff(uint64_t hi, uint64_t lo);
int64_t timestamp_diff_signed(uint64_t a, uint64_t b);

uint16_t get_my_id();


void send_data_msg(uint16_t dst_id, uint8_t* buffer, uint8_t len);

int pdec_radio_submit_poll(uint16_t dst_id, bool double_sided);
int pdec_radio_submit_data(uint16_t dst_id, const uint8_t *buffer, uint8_t len);
int pdec_radio_submit_sync(void);
int pdec_radio_submit_discovery(void);

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



#define SPEED_OF_LIGHT   (299702547)    //in m/s
#define FRAME_LEN_MAX    (127)
#define FRAME_LEN_MAX_EX (1023)

#define RXFLEN_MASK    0x0000007FUL /* Receive Frame Length (0 to 127) */
#define RXFL_MASK_1023 0x000003FFUL /* Receive Frame Length Extension (0 to 1023) */

/* UWB microsecond (uus) to device time unit (dtu, around 15.65 ps) conversion factor.
 * 1 uus = 512 / 499.2 µs and 1 µs = 499.2 * 128 dtu. */
 // F*dt = (499.2MHz*128) * 1e-6s = 63897.6   timestamp increment per microseconds
#define UUS_TO_DWT_TIME 63898
