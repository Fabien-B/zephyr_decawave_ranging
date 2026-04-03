/*
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/types.h>

#include <deca_device_api.h>
#include <deca_probe_interface.h>
#include <dw3000_hw.h>

#include "pdec_common.h"
#include "pdec_initiator.h"
#include "pdec_tdoa.h"

/* 1000 msec = 1 sec */
#define SLEEP_TIME_MS   800

/* The devicetree node identifier for the "led0" alias. */
#define LED0_NODE DT_ALIAS(led0)


const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED0_NODE, gpios);


typedef struct {
    uint16_t dev_id;
    tdoa_role_t role;
} role_record_t;

// TDOA_ROLE_MASTER, TDOA_ROLE_BEACON, TDOA_ROLE_BLINKER
role_record_t roles[] = {
	{.dev_id = 0xc550, .role = TDOA_ROLE_MASTER},
	{.dev_id = 0x0839, .role = TDOA_ROLE_BEACON},
	{.dev_id = 0xdc12, .role = TDOA_ROLE_BEACON},
    {.dev_id = 0xcfdc, .role = TDOA_ROLE_BEACON},
    {.dev_id = 0x842c, .role = TDOA_ROLE_BLINKER},
	{0, 0}
};

#define BLINKER_STACK_SIZE 500
#define BLINKER_PRIORITY 5

extern void blinker(void *, void *, void *) {
	if (!gpio_is_ready_dt(&led)) {
		return;
	}

	int ret = gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE);
	if (ret < 0) {
		return;
	}

	while (1) {
		// int ret = gpio_pin_toggle_dt(&led);
		// if (ret < 0) {
		// 	return;
		// }
		k_msleep(SLEEP_TIME_MS);
	}
}

K_THREAD_DEFINE(blinker_tid, BLINKER_STACK_SIZE,
			blinker, NULL, NULL, NULL,
			BLINKER_PRIORITY, 0, 0);



int main(void)
{
	pdecInit(NULL);
	// k_msleep(100);
	
	printf("device ID: %x\n", get_my_id());

	//pdecInitiatorStart();

	for(int i=0; roles[i].dev_id != 0; i++) {
		if(roles[i].dev_id == get_my_id()) {
			pdecTdoaStart(roles[i].role);
		}
	}


	int ret = testBLE();
	while (true)
	{
		k_msleep(1000);
		print_ble_addr();
	}
	
	
    return 0;
}
