/*
 * ble.c – BLE peripheral with a two-characteristic custom GATT service.
 *
 * Service UUID : 12345678-1234-5678-1234-56789abcdef0
 *
 * Characteristic 1 – Device Info (read-only)
 *   UUID : 12345678-1234-5678-1234-56789abcdef1
 *   Value: a fixed ASCII string, e.g. "nRF5340 v1.0"
 *
 * Characteristic 2 – Blink Period ms (read / write)
 *   UUID : 12345678-1234-5678-1234-56789abcdef2
 *   Value: uint32_t, little-endian (default 500 ms)
 *   A connected central can write a new period; the main loop reads it
 *   via ble_get_blink_period_ms().
 */

#include "ble.h"

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/gap.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/conn.h>

#include <string.h>

#include "app_print.h"

LOG_MODULE_REGISTER(ble, LOG_LEVEL_INF);

/* -------------------------------------------------------------------------
 * UUIDs
 * ---------------------------------------------------------------------- */

/* Service */
#define BT_UUID_CUSTOM_SERVICE_VAL \
    BT_UUID_128_ENCODE(0x12345678, 0x1234, 0x5678, 0x1234, 0x56789abcdef0)

/* Characteristic 1 – Device Info (read-only) */
#define BT_UUID_CHAR_DEV_INFO_VAL \
    BT_UUID_128_ENCODE(0x12345678, 0x1234, 0x5678, 0x1234, 0x56789abcdef1)

/* Characteristic 2 – Blink Period (read/write) */
#define BT_UUID_CHAR_BLINK_PERIOD_VAL \
    BT_UUID_128_ENCODE(0x12345678, 0x1234, 0x5678, 0x1234, 0x56789abcdef2)

static struct bt_uuid_128 uuid_service      = BT_UUID_INIT_128(BT_UUID_CUSTOM_SERVICE_VAL);
static struct bt_uuid_128 uuid_dev_info     = BT_UUID_INIT_128(BT_UUID_CHAR_DEV_INFO_VAL);
static struct bt_uuid_128 uuid_blink_period = BT_UUID_INIT_128(BT_UUID_CHAR_BLINK_PERIOD_VAL);

/* -------------------------------------------------------------------------
 * Characteristic values
 * ---------------------------------------------------------------------- */

/* Char 1 – fixed read-only string */
static const char dev_info_value[] = "DWM3001C beacon";

/* Char 2 – blink period, protected by a mutex so main() can read it safely */
static K_MUTEX_DEFINE(blink_mutex);
static uint32_t blink_period_ms = 500U;   /* default: 500 ms */

/* -------------------------------------------------------------------------
 * GATT read / write callbacks
 * ---------------------------------------------------------------------- */

/* --- Char 1: Device Info read ------------------------------------------ */
static ssize_t read_dev_info(struct bt_conn *conn,
                             const struct bt_gatt_attr *attr,
                             void *buf, uint16_t len, uint16_t offset)
{
    return bt_gatt_attr_read(conn, attr, buf, len, offset,
                             dev_info_value, sizeof(dev_info_value) - 1);
}

/* --- Char 2: Blink Period read ----------------------------------------- */
static ssize_t read_blink_period(struct bt_conn *conn,
                                 const struct bt_gatt_attr *attr,
                                 void *buf, uint16_t len, uint16_t offset)
{
    uint32_t val;

    k_mutex_lock(&blink_mutex, K_FOREVER);
    val = blink_period_ms;
    k_mutex_unlock(&blink_mutex);

	pdec_printf("read blink period: %u\n", val);

    return bt_gatt_attr_read(conn, attr, buf, len, offset,
                             &val, sizeof(val));
}

/* --- Char 2: Blink Period write ---------------------------------------- */
static ssize_t write_blink_period(struct bt_conn *conn,
                                  const struct bt_gatt_attr *attr,
                                  const void *buf, uint16_t len,
                                  uint16_t offset, uint8_t flags)
{
    if (offset != 0) {
        LOG_WRN("write_blink_period: unexpected offset %u", offset);
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
    }

    if (len != sizeof(uint32_t)) {
        LOG_WRN("write_blink_period: bad length %u (expected %zu)", len, sizeof(uint32_t));
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
    }

    uint32_t new_period;
    memcpy(&new_period, buf, sizeof(new_period));

    if (new_period < 50U || new_period > 60000U) {
        LOG_WRN("write_blink_period: value %u ms out of [50, 60000] range", new_period);
        return BT_GATT_ERR(BT_ATT_ERR_VALUE_NOT_ALLOWED);
    }

    k_mutex_lock(&blink_mutex, K_FOREVER);
    blink_period_ms = new_period;
    k_mutex_unlock(&blink_mutex);

    LOG_INF("Blink period updated to %u ms", new_period);
    return (ssize_t)len;
}

/* -------------------------------------------------------------------------
 * GATT service definition
 * ---------------------------------------------------------------------- */

BT_GATT_SERVICE_DEFINE(custom_svc,
    /* Primary service declaration */
    BT_GATT_PRIMARY_SERVICE(&uuid_service),

    /* --- Characteristic 1: Device Info (read-only) --- */
    BT_GATT_CHARACTERISTIC(&uuid_dev_info.uuid,
                           BT_GATT_CHRC_READ,
                           BT_GATT_PERM_READ,
                           read_dev_info, NULL, NULL),

    /* --- Characteristic 2: Blink Period (read + write) --- */
    BT_GATT_CHARACTERISTIC(&uuid_blink_period.uuid,
                           BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE,
                           BT_GATT_PERM_READ | BT_GATT_PERM_WRITE,
                           read_blink_period, write_blink_period, NULL),
);

/* -------------------------------------------------------------------------
 * Advertising data
 * ---------------------------------------------------------------------- */

static const struct bt_data ad[] = {
    BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
    BT_DATA(BT_DATA_NAME_COMPLETE, CONFIG_BT_DEVICE_NAME, sizeof(CONFIG_BT_DEVICE_NAME) - 1),
};

/* Scan-response: expose the custom service UUID so scanners can filter on it */
static const struct bt_data sd[] = {
    BT_DATA_BYTES(BT_DATA_UUID128_ALL, BT_UUID_CUSTOM_SERVICE_VAL),
};

/* -------------------------------------------------------------------------
 * Connection callbacks
 * ---------------------------------------------------------------------- */

static void on_connected(struct bt_conn *conn, uint8_t err)
{
    if (err) {
        LOG_ERR("Connection failed (err %u)", err);
        return;
    }
    LOG_INF("Connected");
}

static void on_disconnected(struct bt_conn *conn, uint8_t reason)
{
    LOG_INF("Disconnected (reason %u)", reason);

    /* Restart advertising so a new central can connect */
    int ret = bt_le_adv_start(BT_LE_ADV_CONN_FAST_1, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));
    if (ret) {
        LOG_ERR("Failed to restart advertising (err %d)", ret);
    }
}

BT_CONN_CB_DEFINE(conn_callbacks) = {
    .connected    = on_connected,
    .disconnected = on_disconnected,
};

/* -------------------------------------------------------------------------
 * Public API
 * ---------------------------------------------------------------------- */

int ble_init(void)
{
    int err;

    err = bt_enable(NULL);
    if (err) {
        LOG_ERR("bt_enable failed (err %d)", err);
        return err;
    }
    LOG_INF("Bluetooth enabled");

    err = bt_le_adv_start(BT_LE_ADV_CONN_FAST_1, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));
    if (err) {
        LOG_ERR("bt_le_adv_start failed (err %d)", err);
        return err;
    }
    LOG_INF("Advertising started as \"%s\"", CONFIG_BT_DEVICE_NAME);

    return 0;
}

uint32_t ble_get_blink_period_ms(void)
{
    uint32_t val;

    k_mutex_lock(&blink_mutex, K_FOREVER);
    val = blink_period_ms;
    k_mutex_unlock(&blink_mutex);

    return val;
}
