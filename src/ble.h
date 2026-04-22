#pragma once

#include <inttypes.h>

// int testBLE();
// void print_ble_addr();
// void ble_periph();


/**
 * @brief Initialise Bluetooth, register the GATT service and start advertising.
 *
 * Call once from main() after all drivers are ready.
 *
 * @return 0 on success, negative errno on failure.
 */
int ble_init(void);

/**
 * @brief Read the current LED blink period written by the GATT client.
 *
 * @return Period in milliseconds.
 */
uint32_t ble_get_blink_period_ms(void);
