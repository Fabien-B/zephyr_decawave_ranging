#include "pdec_initiator.h"
#include "pdec_common.h"
#include "pdec_database.h"

#include <stdio.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/types.h>
#include <zephyr/random/random.h>

#include <deca_device_api.h>
#include <deca_probe_interface.h>
#include <dw3000_hw.h>


#define INITIATOR_STACK_SIZE 1000
#define INITIATOR_PRIORITY 5

K_THREAD_STACK_DEFINE(initiator_stack_area, INITIATOR_STACK_SIZE);
struct k_thread initiator_thread_data;


/* Inter-ranging delay period, in milliseconds. */
#define RNG_DELAY_MS 500



static void pdec_initiator(void *, void *, void *) {
    while(true) {
        for(int i=0; devices[i] != 0; i++) {
            if(devices[i] != get_my_id()) {
                send_poll(devices[i], true);
                k_msleep(RNG_DELAY_MS - sys_rand8_get());
            }
        }
    }
}

void pdecInitiatorStart() {
    k_tid_t my_tid = k_thread_create(&initiator_thread_data, initiator_stack_area,
                                     K_THREAD_STACK_SIZEOF(initiator_stack_area),
                                     pdec_initiator,
                                     NULL, NULL, NULL,
                                     INITIATOR_PRIORITY, 0, K_NO_WAIT);
}

