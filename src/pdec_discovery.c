#include "pdec_discovery.h"

#include <errno.h>
#include <stdbool.h>

#include <zephyr/kernel.h>

#include "pdec_database.h"

#define DISCOVERY_PROTOCOL_VERSION 1
#define DISCOVERY_INTERVAL_MS 2000
#define DISCOVERY_STACK_SIZE 768
#define DISCOVERY_PRIORITY 5

K_THREAD_STACK_DEFINE(discovery_stack_area, DISCOVERY_STACK_SIZE);
static struct k_thread discovery_thread_data;
static bool discovery_thread_started;

static void discovery_thread(void *p1, void *p2, void *p3)
{
    ARG_UNUSED(p1);
    ARG_UNUSED(p2);
    ARG_UNUSED(p3);

    while (true) {
        (void)pdec_discovery_send_now();
        k_msleep(DISCOVERY_INTERVAL_MS);
    }
}

int pdec_discovery_start(void)
{
    if (discovery_thread_started) {
        return -EALREADY;
    }

    discovery_thread_started = true;
    k_thread_create(&discovery_thread_data, discovery_stack_area,
                    K_THREAD_STACK_SIZEOF(discovery_stack_area),
                    discovery_thread,
                    NULL, NULL, NULL,
                    DISCOVERY_PRIORITY, 0, K_NO_WAIT);

    return 0;
}

int pdec_discovery_send_now(void)
{
    return pdec_radio_submit_discovery();
}

void handle_discovery_msg(discovery_msg_t *msg)
{
    if (msg->protocol_version != DISCOVERY_PROTOCOL_VERSION) {
        return;
    }

    if (msg->header.src_id != get_my_id()) {
        set_device_tdoa_role(msg->header.src_id, msg->tdoa_role);
    }

    return;
}
