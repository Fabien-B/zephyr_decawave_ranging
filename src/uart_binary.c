#include "uart_binary.h"

#include <stdint.h>
#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>

#include "pdec_common.h"
#include "pdec_database.h"
#include "pdec_ranging.h"
#include "uart_protocol.h"

static void send_distance_msg(enum pdec_serial_port port, uint8_t msg, uint16_t dst_id, uint8_t status, double distance);

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
    PDEC_BINARY_MSG_DATA_EVENT = 0x91,
    PDEC_BINARY_MSG_TDOA_REPORT_EVENT = 0x92,
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

struct pdec_binary_range_request {
    enum pdec_serial_port port;
    uint16_t dst_id;
};

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

typedef struct __attribute__((packed)) {
    uint8_t msg;
    uint16_t src_id;
    uint8_t len;
    uint8_t data[PAYLOAD_FRAME_LEN_MAX];
} pdec_binary_data_event_msg_t;

BUILD_ASSERT(sizeof(pdec_binary_range_cmd_t) == 3);
BUILD_ASSERT(sizeof(pdec_binary_get_distance_cmd_t) == 3);
BUILD_ASSERT(sizeof(pdec_binary_list_devices_cmd_t) == 1);
BUILD_ASSERT(sizeof(pdec_binary_send_data_cmd_header_t) == 3);
BUILD_ASSERT(sizeof(pdec_binary_error_msg_t) == 3);
BUILD_ASSERT(sizeof(pdec_binary_distance_msg_t) == 8);
BUILD_ASSERT(sizeof(pdec_binary_device_list_msg_t) == 22);
BUILD_ASSERT(sizeof(pdec_binary_send_data_result_msg_t) == 5);
BUILD_ASSERT(sizeof(pdec_binary_ranging_event_msg_t) == 10);
BUILD_ASSERT(sizeof(pdec_binary_data_event_msg_t) == 117);

#define BINARY_RANGE_STACK_SIZE 2048
#define BINARY_RANGE_PRIORITY 5
#define BINARY_RANGE_QUEUE_LEN 1

K_MSGQ_DEFINE(binary_range_queue, sizeof(struct pdec_binary_range_request),
              BINARY_RANGE_QUEUE_LEN, 2);
K_THREAD_STACK_DEFINE(binary_range_stack, BINARY_RANGE_STACK_SIZE);
static struct k_thread binary_range_thread;
static bool binary_range_thread_started;

static void send_error(enum pdec_serial_port port, uint8_t cmd, uint8_t status)
{
    pdec_binary_error_msg_t payload = {
        .msg = PDEC_BINARY_MSG_ERROR,
        .cmd = cmd,
        .status = status,
    };

    (void)pdec_uart_protocol_write_binary_frame(port, (const uint8_t *)&payload,
                                                sizeof(payload));
}

static void binary_range_worker(void *p1, void *p2, void *p3)
{
    ARG_UNUSED(p1);
    ARG_UNUSED(p2);
    ARG_UNUSED(p3);

    struct pdec_binary_range_request request;

    while (true) {
        if (k_msgq_get(&binary_range_queue, &request, K_FOREVER) != 0) {
            continue;
        }

        double distance;
        int ret = request_range(request.dst_id, true, &distance, K_MSEC(100));
        if (ret != 0) {
            send_distance_msg(request.port, PDEC_BINARY_MSG_RANGE_RESULT, request.dst_id,
                              PDEC_BINARY_STATUS_RANGE_FAILED, 0.0);
            continue;
        }

        send_distance_msg(request.port, PDEC_BINARY_MSG_RANGE_RESULT, request.dst_id,
                          PDEC_BINARY_STATUS_OK, distance);
    }
}

static void start_binary_range_thread(void)
{
    if (binary_range_thread_started) {
        return;
    }

    k_thread_create(&binary_range_thread, binary_range_stack,
                    K_THREAD_STACK_SIZEOF(binary_range_stack),
                    binary_range_worker, NULL, NULL, NULL,
                    BINARY_RANGE_PRIORITY, 0, K_NO_WAIT);
    binary_range_thread_started = true;
}

static void send_distance_msg(enum pdec_serial_port port, uint8_t msg, uint16_t dst_id, uint8_t status, double distance)
{
    pdec_binary_distance_msg_t payload = {
        .msg = msg,
        .dst_id = dst_id,
        .status = status,
        .distance = (float)distance,
    };

    (void)pdec_uart_protocol_write_binary_frame(port, (const uint8_t *)&payload,
                                                sizeof(payload));
}

static void handle_range_cmd(enum pdec_serial_port port, const uint8_t *payload, uint8_t len)
{
    if (len != sizeof(pdec_binary_range_cmd_t)) {
        send_error(port, PDEC_BINARY_CMD_RANGE, PDEC_BINARY_STATUS_INVALID_LENGTH);
        return;
    }

    const pdec_binary_range_cmd_t *cmd = (const pdec_binary_range_cmd_t *)payload;
    struct pdec_binary_range_request request = {
        .port = port,
        .dst_id = cmd->dst_id,
    };

    start_binary_range_thread();

    if (k_msgq_put(&binary_range_queue, &request, K_NO_WAIT) != 0) {
        send_distance_msg(port, PDEC_BINARY_MSG_RANGE_RESULT, cmd->dst_id,
                          PDEC_BINARY_STATUS_RANGE_FAILED, 0.0);
    }
}

static void handle_get_distance_cmd(enum pdec_serial_port port, const uint8_t *payload, uint8_t len)
{
    if (len != sizeof(pdec_binary_get_distance_cmd_t)) {
        send_error(port, PDEC_BINARY_CMD_GET_DISTANCE, PDEC_BINARY_STATUS_INVALID_LENGTH);
        return;
    }

    const pdec_binary_get_distance_cmd_t *cmd =
        (const pdec_binary_get_distance_cmd_t *)payload;
    double distance = get_distance(get_my_id(), cmd->dst_id);

    if (distance < 0.0) {
        send_distance_msg(port, PDEC_BINARY_MSG_DISTANCE, cmd->dst_id,
                          PDEC_BINARY_STATUS_NOT_FOUND, 0.0);
        return;
    }

    send_distance_msg(port, PDEC_BINARY_MSG_DISTANCE, cmd->dst_id,
                      PDEC_BINARY_STATUS_OK, distance);
}

static void handle_list_devices_cmd(enum pdec_serial_port port, const uint8_t *payload, uint8_t len)
{
    ARG_UNUSED(payload);

    if (len != sizeof(pdec_binary_list_devices_cmd_t)) {
        send_error(port, PDEC_BINARY_CMD_LIST_DEVICES, PDEC_BINARY_STATUS_INVALID_LENGTH);
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

    (void)pdec_uart_protocol_write_binary_frame(port, (const uint8_t *)&response,
                                                sizeof(response.msg) +
                                                sizeof(response.count) +
                                                (response.count *
                                                 sizeof(response.device_id[0])));
}

static void handle_send_data_cmd(enum pdec_serial_port port, const uint8_t *payload, uint8_t len)
{
    if (len < sizeof(pdec_binary_send_data_cmd_header_t)) {
        send_error(port, PDEC_BINARY_CMD_SEND_DATA, PDEC_BINARY_STATUS_INVALID_LENGTH);
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
        (void)pdec_uart_protocol_write_binary_frame(port, (const uint8_t *)&response,
                                                    sizeof(response));
        return;
    }

    send_data_msg(cmd->dst_id, (uint8_t *)&payload[sizeof(*cmd)], data_len);
    (void)pdec_uart_protocol_write_binary_frame(port, (const uint8_t *)&response,
                                                sizeof(response));
}

void pdec_uart_on_binary_frame(enum pdec_serial_port port, const uint8_t *payload, uint8_t len, uint8_t chk)
{
    ARG_UNUSED(chk);

    if (len == 0) {
        send_error(port, 0, PDEC_BINARY_STATUS_INVALID_LENGTH);
        return;
    }

    switch (payload[0]) {
    case PDEC_BINARY_CMD_RANGE:
        handle_range_cmd(port, payload, len);
        break;
    case PDEC_BINARY_CMD_GET_DISTANCE:
        handle_get_distance_cmd(port, payload, len);
        break;
    case PDEC_BINARY_CMD_LIST_DEVICES:
        handle_list_devices_cmd(port, payload, len);
        break;
    case PDEC_BINARY_CMD_SEND_DATA:
        handle_send_data_cmd(port, payload, len);
        break;
    default:
        send_error(port, payload[0], PDEC_BINARY_STATUS_ERROR);
        break;
    }
}

void pdec_uart_binary_publish_ranging_event(uint8_t kind, uint16_t src_id,
                                            uint16_t dst_id, double distance)
{
    pdec_binary_ranging_event_msg_t payload = {
        .msg = PDEC_BINARY_MSG_RANGING_EVENT,
        .kind = kind,
        .src_id = src_id,
        .dst_id = dst_id,
        .distance = (float)distance,
    };

    pdec_uart_protocol_publish_binary_frame((const uint8_t *)&payload,
                                                sizeof(payload));
}

void pdec_uart_binary_publish_data_event(uint16_t src_id, const uint8_t *data,
                                         uint8_t len)
{
    if (len > PAYLOAD_FRAME_LEN_MAX) {
        len = PAYLOAD_FRAME_LEN_MAX;
    }

    pdec_binary_data_event_msg_t payload = {
        .msg = PDEC_BINARY_MSG_DATA_EVENT,
        .src_id = src_id,
        .len = len,
    };

    memcpy(payload.data, data, len);

    pdec_uart_protocol_publish_binary_frame((const uint8_t *)&payload,
                                                sizeof(payload.msg) +
                                                sizeof(payload.src_id) +
                                                sizeof(payload.len) + len);
}

void pdec_uart_binary_publish_tdoa_report_event(uint32_t blink_id, uint8_t expected_count,
                                                bool timed_out, const uint16_t *reporter_id,
                                                const double *distance_m, uint8_t count)
{
    uint8_t payload[1 + sizeof(uint32_t) + 3 + (10 * (sizeof(uint16_t) + sizeof(float)))];
    size_t offset = 0;

    if (count > 10) {
        count = 10;
    }

    payload[offset++] = PDEC_BINARY_MSG_TDOA_REPORT_EVENT;
    memcpy(&payload[offset], &blink_id, sizeof(blink_id));
    offset += sizeof(blink_id);
    payload[offset++] = expected_count;
    payload[offset++] = count;
    payload[offset++] = timed_out ? 1 : 0;

    for (uint8_t i = 0; i < count; i++) {
        float distance = (float)distance_m[i];

        memcpy(&payload[offset], &reporter_id[i], sizeof(reporter_id[i]));
        offset += sizeof(reporter_id[i]);
        memcpy(&payload[offset], &distance, sizeof(distance));
        offset += sizeof(distance);
    }

    pdec_uart_protocol_publish_binary_frame(payload, offset);
}
