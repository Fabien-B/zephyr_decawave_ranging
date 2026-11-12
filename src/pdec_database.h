#pragma once
#include <inttypes.h>

typedef enum {
    TDOA_ROLE_NONE,
    TDOA_ROLE_BEACON,
    TDOA_ROLE_BLINKER,
    TDOA_ROLE_MASTER,
    TDOA_ROLE_UNKNOWN = 0xff,
} tdoa_role_t;

/**
 * @brief Find the internal index of a device by its ID.
 * 
 * @param id 16-bit device short address.
 * @return The index in the devices array, or -1 if not found.
 */
int get_device_index(uint16_t id);

/**
 * @brief Get the device ID at a specific internal index.
 * 
 * @param index The internal array index.
 * @return The 16-bit device ID, or 0 if index is out of bounds or empty.
 */
uint16_t get_device_id(int index);

/**
 * @brief Add a device ID to the global devices list.
 *
 * @param id The 16-bit short address of the device.
 * @return The index where the device was added, or -1 if the list is full.
 */
int add_device(uint16_t id);

/**
 * @brief Refresh the last-seen timestamp for a device.
 *
 * @param id 16-bit device short address.
 */
void touch_device(uint16_t id);
/* Capture raw offset in RX; defer bookkeeping/conversion until TX is armed. */
void pdec_database_update_rx(uint16_t id, int16_t raw_clock_offset);

/**
 * @brief Store the advertised TDOA role for a device.
 *
 * @param id 16-bit device short address.
 * @param role TDOA role value.
 */
void set_device_tdoa_role(uint16_t id, uint8_t role);

/**
 * @brief Get the last advertised TDOA role for a device.
 *
 * @param id 16-bit device short address.
 * @return TDOA role value, or TDOA_ROLE_UNKNOWN if unknown.
 */
uint8_t get_device_tdoa_role(uint16_t id);

/**
 * @brief Store the measured clock offset ratio for a specific device.
 *
 * @param id 16-bit device short address.
 * @param clk_offset_ratio The ratio calculated from carrier integrator.
 */
void set_clock_offset(uint16_t id, float clk_offset_ratio);

/**
 * @brief Get the clock correction factor for a device.
 *
 * @param id 16-bit device short address.
 * @return The multiplier for time correction (defaults to 1.0 if unknown).
 */
double get_clock_offset(uint16_t id);

/**
 * @brief Store the distance between two devices in the triangular matrix.
 *
 * @param a First device ID.
 * @param b Second device ID.
 * @param distance Measured distance in meters.
 */
void set_distance(uint16_t a, uint16_t b, double distance);

/**
 * @brief Retrieve the last known distance between two devices.
 *
 * @param a First device ID.
 * @param b Second device ID.
 * @return Distance in meters, or -1 if no data exists.
 */
double get_distance(uint16_t a, uint16_t b);

/**
 * @brief Get a deterministic rank for a device based on sorted IDs.
 *
 * @param id 16-bit device short address.
 * @return The sorted index (0 to N), or 255 if not found.
 */
uint8_t get_tdoa_index(uint16_t id);
