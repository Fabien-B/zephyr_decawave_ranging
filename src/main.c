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

#include "ble.h"

/* 1000 msec = 1 sec */
#define SLEEP_TIME_MS   800

/* The devicetree node identifier for the "led0" alias. */
#define LED0_NODE DT_ALIAS(led0)
const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED0_NODE, gpios);

#define JP1_NODE DT_ALIAS(sw1)
const struct gpio_dt_spec jp1 = GPIO_DT_SPEC_GET(JP1_NODE, gpios);

#define JP2_NODE DT_ALIAS(sw2)
const struct gpio_dt_spec jp2 = GPIO_DT_SPEC_GET(JP2_NODE, gpios);

#define JP3_NODE DT_ALIAS(sw3)
const struct gpio_dt_spec jp3 = GPIO_DT_SPEC_GET(JP3_NODE, gpios);

#define JP4_NODE DT_ALIAS(sw4)
const struct gpio_dt_spec jp4 = GPIO_DT_SPEC_GET(JP4_NODE, gpios);


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
		gpio_pin_toggle_dt(&led);
		// if (ret < 0) {
		// 	return;
		// }
		uint32_t period = ble_get_blink_period_ms();
		k_msleep(period);
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

	gpio_pin_configure_dt(&jp1, GPIO_INPUT);
	gpio_pin_configure_dt(&jp2, GPIO_INPUT);
	gpio_pin_configure_dt(&jp3, GPIO_INPUT);
	gpio_pin_configure_dt(&jp4, GPIO_INPUT);


	int j1 = gpio_pin_get_dt(&jp1);
	int j2 = gpio_pin_get_dt(&jp2);
	// int j3 = gpio_pin_get_dt(&jp3);
	// int j4 = gpio_pin_get_dt(&jp4);

	printf("j1: %d,  j2: %d\n", j1, j2);

	tdoa_role_t role = (tdoa_role_t) (j2 << 1) | (j1 << 0);
	printf("Starts TDOA role: %d\n", role);

	//pdecInitiatorStart();

	pdecTdoaStart(role);


	ble_init();

	
	
    return 0;
}
