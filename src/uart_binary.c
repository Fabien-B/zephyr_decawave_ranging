#include "uart_binary.h"

#include <stdint.h>

#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>

#include "pdec_common.h"
#include "pdec_database.h"
#include "pdec_ranging.h"
#include "uart_protocol.h"

enum pdec_binary_cmd {
    PDEC_BINARY_CMD_RANGE = 0x01,
    PDEC_BINARY_CMD_GET_DISTANCE = 0x02,
    PDEC_BINARY_CMD_LIST_DEVICES = 0x03,
    PDEC_BINARY_CMD_SEND_DATA = 0x04,
};

enum pdec_binary_msg {
    PDEC_BINARY_MSG_ERROR = 0x7f,
    PDEC_BINARY_MSG_RANGE_RESULT = 0x81,
    PDEC_BINARY_MSG_DISTANCE = 0x82,
    PDEC_BINARY_MSG_DEVICE_LIST = 0x83,
    PDEC_BINARY_MSG_SEND_DATA_RESULT = 0x84,
    PDEC_BINARY_MSG_RANGING_EVENT = 0x90,
};

enum pdec_binary_status {
    PDEC_BINARY_STATUS_OK = 0,
    PDEC_BINARY_STATUS_ERROR = 1,
    PDEC_BINARY_STATUS_INVALID_LENGTH = 2,
    PDEC_BINARY_STATUS_NOT_FOUND = 3,
    PDEC_BINARY_STATUS_RANGE_FAILED = 4,
    PDEC_BINARY_STATUS_MESSAGE_TOO_LONG = 5,
};

typedef struct __attribute__((packed)) {
    uint8_t cmd;
    uint16_t dst_id;
} pdec_binary_range_cmd_t;

typedef struct __attribute__((packed)) {
    uint8_t cmd;
    uint16_t dst_id;
} pdec_binary_get_distance_cmd_t;

typedef struct __attribute__((packed)) {
    uint8_t cmd;
} pdec_binary_list_devices_cmd_t;

typedef struct __attribute__((packed)) {
    uint8_t cmd;
    uint16_t dst_id;
} pdec_binary_send_data_cmd_header_t;

typedef struct __attribute__((packed)) {
    uint8_t msg;
    uint8_t cmd;
    uint8_t status;
} pdec_binary_error_msg_t;

typedef struct __attribute__((packed)) {
    uint8_t msg;
    uint16_t dst_id;
    uint8_t status;
    float distance;
} pdec_binary_distance_msg_t;

typedef struct __attribute__((packed)) {
    uint8_t msg;
    uint8_t count;
    uint16_t device_id[10];
} pdec_binary_device_list_msg_t;

typedef struct __attribute__((packed)) {
    uint8_t msg;
    uint16_t dst_id;
    uint8_t status;
    uint8_t sent_len;
} pdec_binary_send_data_result_msg_t;

typedef struct __attribute__((packed)) {
    uint8_t msg;
    uint8_t kind;
    uint16_t src_id;
    uint16_t dst_id;
    float distance;
} pdec_binary_ranging_event_msg_t;

BUILD_ASSERT(sizeof(pdec_binary_range_cmd_t) == 3);
BUILD_ASSERT(sizeof(pdec_binary_get_distance_cmd_t) == 3);
BUILD_ASSERT(sizeof(pdec_binary_list_devices_cmd_t) == 1);
BUILD_ASSERT(sizeof(pdec_binary_send_data_cmd_header_t) == 3);
BUILD_ASSERT(sizeof(pdec_binary_error_msg_t) == 3);
BUILD_ASSERT(sizeof(pdec_binary_distance_msg_t) == 8);
BUILD_ASSERT(sizeof(pdec_binary_device_list_msg_t) == 22);
BUILD_ASSERT(sizeof(pdec_binary_send_data_result_msg_t) == 5);
BUILD_ASSERT(sizeof(pdec_binary_ranging_event_msg_t) == 10);

static void send_error(uint8_t cmd, uint8_t status)
{
    pdec_binary_error_msg_t payload = {
        .msg = PDEC_BINARY_MSG_ERROR,
        .cmd = cmd,
        .status = status,
    };

    (void)pdec_uart_protocol_write_binary_frame((const uint8_t *)&payload,
                                                sizeof(payload));
}

static void send_distance_msg(uint8_t msg, uint16_t dst_id, uint8_t status, double distance)
{
    pdec_binary_distance_msg_t payload = {
        .msg = msg,
        .dst_id = dst_id,
        .status = status,
        .distance = (float)distance,
    };

    (void)pdec_uart_protocol_write_binary_frame((const uint8_t *)&payload,
                                                sizeof(payload));
}

static void handle_range_cmd(const uint8_t *payload, uint8_t len)
{
    if (len != sizeof(pdec_binary_range_cmd_t)) {
        send_error(PDEC_BINARY_CMD_RANGE, PDEC_BINARY_STATUS_INVALID_LENGTH);
        return;
    }

    const pdec_binary_range_cmd_t *cmd = (const pdec_binary_range_cmd_t *)payload;
    double old_distance = get_distance(get_my_id(), cmd->dst_id);

    send_poll(cmd->dst_id, true);
    // TODO replace by event based trigger
    k_msleep(20);

    double new_distance = get_distance(get_my_id(), cmd->dst_id);
    if (new_distance == old_distance) {
        send_distance_msg(PDEC_BINARY_MSG_RANGE_RESULT, cmd->dst_id,
                          PDEC_BINARY_STATUS_RANGE_FAILED, 0.0);
        return;
    }

    send_distance_msg(PDEC_BINARY_MSG_RANGE_RESULT, cmd->dst_id,
                      PDEC_BINARY_STATUS_OK, new_distance);
}

static void handle_get_distance_cmd(const uint8_t *payload, uint8_t len)
{
    if (len != sizeof(pdec_binary_get_distance_cmd_t)) {
        send_error(PDEC_BINARY_CMD_GET_DISTANCE, PDEC_BINARY_STATUS_INVALID_LENGTH);
        return;
    }

    const pdec_binary_get_distance_cmd_t *cmd =
        (const pdec_binary_get_distance_cmd_t *)payload;
    double distance = get_distance(get_my_id(), cmd->dst_id);

    if (distance < 0.0) {
        send_distance_msg(PDEC_BINARY_MSG_DISTANCE, cmd->dst_id,
                          PDEC_BINARY_STATUS_NOT_FOUND, 0.0);
        return;
    }

    send_distance_msg(PDEC_BINARY_MSG_DISTANCE, cmd->dst_id,
                      PDEC_BINARY_STATUS_OK, distance);
}

static void handle_list_devices_cmd(const uint8_t *payload, uint8_t len)
{
    ARG_UNUSED(payload);

    if (len != sizeof(pdec_binary_list_devices_cmd_t)) {
        send_error(PDEC_BINARY_CMD_LIST_DEVICES, PDEC_BINARY_STATUS_INVALID_LENGTH);
        return;
    }

    pdec_binary_device_list_msg_t response = {
        .msg = PDEC_BINARY_MSG_DEVICE_LIST,
        .count = 0,
    };

    for (int i = 0;
         get_device_id(i) != 0 && response.count < ARRAY_SIZE(response.device_id);
         i++) {
        response.device_id[response.count] = get_device_id(i);
        response.count++;
    }

    (void)pdec_uart_protocol_write_binary_frame((const uint8_t *)&response,
                                                sizeof(response.msg) +
                                                sizeof(response.count) +
                                                (response.count *
                                                 sizeof(response.device_id[0])));
}

static void handle_send_data_cmd(const uint8_t *payload, uint8_t len)
{
    if (len < sizeof(pdec_binary_send_data_cmd_header_t)) {
        send_error(PDEC_BINARY_CMD_SEND_DATA, PDEC_BINARY_STATUS_INVALID_LENGTH);
        return;
    }

    const pdec_binary_send_data_cmd_header_t *cmd =
        (const pdec_binary_send_data_cmd_header_t *)payload;
    uint8_t data_len = len - sizeof(*cmd);
    pdec_binary_send_data_result_msg_t response = {
        .msg = PDEC_BINARY_MSG_SEND_DATA_RESULT,
        .dst_id = cmd->dst_id,
        .status = PDEC_BINARY_STATUS_OK,
        .sent_len = data_len,
    };

    if (data_len > PAYLOAD_FRAME_LEN_MAX) {
        response.status = PDEC_BINARY_STATUS_MESSAGE_TOO_LONG;
        response.sent_len = 0;
        (void)pdec_uart_protocol_write_binary_frame((const uint8_t *)&response,
                                                    sizeof(response));
        return;
    }

    send_data_msg(cmd->dst_id, (uint8_t *)&payload[sizeof(*cmd)], data_len);
    (void)pdec_uart_protocol_write_binary_frame((const uint8_t *)&response,
                                                sizeof(response));
}

void pdec_uart_on_binary_frame(const uint8_t *payload, uint8_t len, uint8_t chk)
{
    ARG_UNUSED(chk);

    if (len == 0) {
        send_error(0, PDEC_BINARY_STATUS_INVALID_LENGTH);
        return;
    }

    switch (payload[0]) {
    case PDEC_BINARY_CMD_RANGE:
        handle_range_cmd(payload, len);
        break;
    case PDEC_BINARY_CMD_GET_DISTANCE:
        handle_get_distance_cmd(payload, len);
        break;
    case PDEC_BINARY_CMD_LIST_DEVICES:
        handle_list_devices_cmd(payload, len);
        break;
    case PDEC_BINARY_CMD_SEND_DATA:
        handle_send_data_cmd(payload, len);
        break;
    default:
        send_error(payload[0], PDEC_BINARY_STATUS_ERROR);
        break;
    }
}

void pdec_uart_binary_publish_ranging_event(uint8_t kind, uint16_t src_id,
                                            uint16_t dst_id, double distance)
{
    if (pdec_uart_protocol_get_mode() != PDEC_UART_MODE_BINARY) {
        return;
    }

    pdec_binary_ranging_event_msg_t payload = {
        .msg = PDEC_BINARY_MSG_RANGING_EVENT,
        .kind = kind,
        .src_id = src_id,
        .dst_id = dst_id,
        .distance = (float)distance,
    };

    (void)pdec_uart_protocol_write_binary_frame((const uint8_t *)&payload,
                                                sizeof(payload));
}
