#pragma once
#include <inttypes.h>

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
