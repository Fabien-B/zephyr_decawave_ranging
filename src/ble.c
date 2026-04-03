#include <zephyr/kernel.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include "ble.h"

static const struct bt_data ad[] = {
    BT_DATA(BT_DATA_NAME_COMPLETE, CONFIG_BT_DEVICE_NAME, sizeof(CONFIG_BT_DEVICE_NAME) - 1),
    BT_DATA(BT_DATA_BROADCAST_NAME, CONFIG_BT_DEVICE_NAME, sizeof(CONFIG_BT_DEVICE_NAME) - 1),
};

int testBLE() {
    int err;

    err = bt_enable(NULL);
    if (err) {
        return -1;
    }

    err = bt_le_adv_start(BT_LE_ADV_NCONN, ad, ARRAY_SIZE(ad), NULL, 0);
    if (err) {
        return -1;
    }
    
    return err;
}


void print_ble_addr()
{
    bt_addr_le_t addr[1];
    size_t count = 1;

    bt_id_get(addr, &count);
    if (count > 0) {
        char addr_str[BT_ADDR_LE_STR_LEN];
        bt_addr_le_to_str(&addr[0], addr_str, sizeof(addr_str));
        printk("BLE MAC: %s\n", addr_str);
    } else {
        printk("Failed to get BLE address\n");
    }
}