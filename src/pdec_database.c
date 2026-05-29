#include "pdec_database.h"
#include "pdec_common.h"

#include <string.h>

#include <zephyr/kernel.h>

#define NUM_DEVICES_MAX 10
#define DEVICE_TIMEOUT_MS 10000

/* Calculate triangular matrix index: i*N - i*(i+1)/2 + (j-i-1) */
#define GET_TRIANGULAR_INDEX(i, j, N) ((i) * (N) - ((i) * ((i) + 1) / 2) + ((j) - (i) - 1))

// Low 16bits of the nrf52833 device ID: NRF_FICR->DEVICEID[0]
static uint16_t devices[NUM_DEVICES_MAX+1] = {0};
// uint16_t devices[] = {
// 	//0xc550,
// 	//0x0839,
// 	//0xdc12,
//     0x7223,
//     0x9e7a,
//     0x9591,
//     0xcfdc,
//     0x842c,
//     0 // Must end with 0 to detect the end of the array
// };

double distances[(NUM_DEVICES_MAX*(NUM_DEVICES_MAX-1))/2] = {0};


// to store the clock offset of every other agents
float clock_offsets[NUM_DEVICES_MAX+1] = {0};
static uint8_t tdoa_roles[NUM_DEVICES_MAX+1] = {
    [0 ... NUM_DEVICES_MAX] = TDOA_ROLE_UNKNOWN,
};
static int64_t last_seen_ms[NUM_DEVICES_MAX+1] = {0};

static void reset_distances(void)
{
    memset(distances, 0, sizeof(distances));
}

static int get_device_count(void)
{
    int count = 0;

    while (count < NUM_DEVICES_MAX && devices[count] != 0) {
        count++;
    }

    return count;
}

static int find_device_index(uint16_t id)
{
    for(int i=0; i<NUM_DEVICES_MAX; i++) {
        if(devices[i] == id) {
            return i;
        }
    }
    return -1;
}

static void remove_device_at(int index)
{
    if (index < 0 || index >= NUM_DEVICES_MAX) {
        return;
    }

    for (int i = index; i < NUM_DEVICES_MAX - 1; i++) {
        devices[i] = devices[i + 1];
        clock_offsets[i] = clock_offsets[i + 1];
        tdoa_roles[i] = tdoa_roles[i + 1];
        last_seen_ms[i] = last_seen_ms[i + 1];
    }

    devices[NUM_DEVICES_MAX - 1] = 0;
    clock_offsets[NUM_DEVICES_MAX - 1] = 0;
    tdoa_roles[NUM_DEVICES_MAX - 1] = TDOA_ROLE_UNKNOWN;
    last_seen_ms[NUM_DEVICES_MAX - 1] = 0;
    reset_distances();
}

static int insert_device_sorted(uint16_t id)
{
    int count = get_device_count();

    if (count >= NUM_DEVICES_MAX) {
        return -1;
    }

    int insert_index = 0;
    while (insert_index < count && devices[insert_index] < id) {
        insert_index++;
    }

    for (int i = count; i > insert_index; i--) {
        devices[i] = devices[i - 1];
        clock_offsets[i] = clock_offsets[i - 1];
        tdoa_roles[i] = tdoa_roles[i - 1];
        last_seen_ms[i] = last_seen_ms[i - 1];
    }

    devices[insert_index] = id;
    clock_offsets[insert_index] = 0;
    tdoa_roles[insert_index] = TDOA_ROLE_UNKNOWN;
    last_seen_ms[insert_index] = k_uptime_get();
    reset_distances();

    return insert_index;
}

static void ensure_self_device(void)
{
    if (find_device_index(get_my_id()) == -1) {
        (void)insert_device_sorted(get_my_id());
    }
}

static void prune_stale_devices(void)
{
    int64_t now = k_uptime_get();

    ensure_self_device();

    for (int i = 0; i < NUM_DEVICES_MAX && devices[i] != 0; ) {
        if (devices[i] != get_my_id() &&
            last_seen_ms[i] != 0 &&
            (now - last_seen_ms[i]) > DEVICE_TIMEOUT_MS) {
            remove_device_at(i);
            continue;
        }

        i++;
    }
}

int get_device_index(uint16_t id) {
    prune_stale_devices();
    return find_device_index(id);
}

uint16_t get_device_id(int index) {
    prune_stale_devices();

    if (index >= 0 && index < NUM_DEVICES_MAX) {
        return devices[index];
    }
    return 0;
}


/**
 * @brief Add a device ID to the global devices list.
 * 
 * @param id The 16-bit short address of the device.
 * @return The index where the device was added, or -1 if the list is full.
 */
int add_device(uint16_t id) {
    ensure_self_device();

    int existing_index = get_device_index(id);
    if (existing_index != -1) {
        last_seen_ms[existing_index] = k_uptime_get();
        return existing_index;
    }

    return insert_device_sorted(id);
}

void touch_device(uint16_t id)
{
    int index = get_device_index(id);
    if (index == -1) {
        index = add_device(id);
    }

    if (index != -1) {
        last_seen_ms[index] = k_uptime_get();
    }
}

void set_device_tdoa_role(uint16_t id, uint8_t role)
{
    int index = get_device_index(id);
    if (index == -1) {
        index = add_device(id);
    }

    if (index != -1) {
        tdoa_roles[index] = role;
    }
}

uint8_t get_device_tdoa_role(uint16_t id)
{
    int index = get_device_index(id);

    if (index != -1) {
        return tdoa_roles[index];
    }

    return TDOA_ROLE_UNKNOWN;
}


void set_clock_offset(uint16_t id, float clk_offset_ratio) {
    int index = get_device_index(id);
    if (index == -1) {
        index = add_device(id);
    }

    if (index != -1) {
        clock_offsets[index] = clk_offset_ratio;
    }
}

double get_clock_offset(uint16_t id) {
    int index = get_device_index(id);
    if (index != -1) {
        return (1.0 - (double)clock_offsets[index]);
    }

    // not found, return 1
    return 1.0;
}


void set_distance(uint16_t a, uint16_t b, double distance) {
    int idx_a = get_device_index(a);
    int idx_b = get_device_index(b);
    
    if (idx_a == -1) {
        idx_a = add_device(a);
    }
    if (idx_b == -1) {
        idx_b = add_device(b);
    }

    if(idx_a == -1 || idx_b == -1) {
        return;
    }

    int i = (idx_a < idx_b) ? idx_a : idx_b;
    int j = (idx_a < idx_b) ? idx_b : idx_a;

    int index = GET_TRIANGULAR_INDEX(i, j, NUM_DEVICES_MAX);
    distances[index] = distance;
}

double get_distance(uint16_t a, uint16_t b) {
    int idx_a = get_device_index(a);
    int idx_b = get_device_index(b);

    if (idx_a == -1 || idx_b == -1 || idx_a == idx_b) {
        return -1;
    }

    int i = (idx_a < idx_b) ? idx_a : idx_b;
    int j = (idx_a < idx_b) ? idx_b : idx_a;

    int index = GET_TRIANGULAR_INDEX(i, j, NUM_DEVICES_MAX);

    return distances[index];
}

uint8_t get_tdoa_index(uint16_t id) {
    prune_stale_devices();

    int count = 0;

    for (int i = 0; i < NUM_DEVICES_MAX && devices[i] != 0; i++) {
        if (tdoa_roles[i] == TDOA_ROLE_BEACON ||
            tdoa_roles[i] == TDOA_ROLE_MASTER) {
            if (devices[i] == id) {
                return (uint8_t)count;
            }

            count++;
        }
    }

    return 255; // not found
}
