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