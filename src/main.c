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
	uint32_t dev_id;
	pdecInit(&dev_id);
    
	uint32_t dede0 = get_my_id();

	printf("dede: %x\n", dede0);

	const char* api_version = dwt_version_string();
	printf("DWT API version: %s\n", api_version);

	

	//pdecInitiatorStart();

	k_msleep(100);

	if(dede0 == 0x0839) {
		pdecTdoaStart(TDOA_ROLE_MASTER);
	}
	else if (dede0 == 0xc550) {
		pdecTdoaStart(TDOA_ROLE_BEACON);
	}
	else if (dede0 == 0xdc12) {
		pdecTdoaStart(TDOA_ROLE_BLINKER);
	}
	
    return 0;
}
