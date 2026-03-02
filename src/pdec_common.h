#pragma once
#include <stddef.h>
#include <inttypes.h>
#include <stdbool.h>

#define MY_PAN_ID 0xDEAD
#define BROADCAST_ID 0xFFFE


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


union ranging_msg_u
{
    frame_header_t header;
    poll_msg_t poll;
    resp_msg_t resp;
    final_msg_t final;
};



typedef enum {
    IDLE,       // doing nothing
    POLLING,    // poll (or response) sent, waiting for response (or final)
    LISTENING,  // waiting to receive a poll
} status_t;


typedef enum {
    DF_SS_RANGING_POLL =  0xE0,
    DF_SS_RANGING_RESP =  0xE1,
    DF_DS_RANGING_POLL =  0xE2,
    DF_DS_RANGING_RESP =  0xE3,
    DF_DS_RANGING_FINAL = 0xE4,
} frame_role_t;


int pdecInit(uint32_t* dev_id);
uint64_t get_rx_timestamp_u64(void);
uint64_t get_tx_timestamp_u64(void);

uint16_t get_my_id();

void send_poll(uint16_t dst_id, bool double_sided);

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
