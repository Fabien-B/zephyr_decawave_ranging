#include "pdec_initiator.h"

#include <errno.h>

#include <zephyr/kernel.h>

#include "pdec_common.h"
#include "pdec_database.h"
#include "pdec_ranging.h"

#define INITIATOR_STACK_SIZE 1000
#define INITIATOR_PRIORITY 5
#define CONTINUOUS_DEFAULT_PERIOD_MS 500
#define CONTINUOUS_IDLE_WAIT_MS 500

K_THREAD_STACK_DEFINE(initiator_stack_area, INITIATOR_STACK_SIZE);
static struct k_thread initiator_thread_data;
static K_MUTEX_DEFINE(continuous_lock);
static K_SEM_DEFINE(continuous_wake_sem, 0, 1);

static bool continuous_thread_active;
static bool continuous_stop_requested;
static uint32_t continuous_period_ms = CONTINUOUS_DEFAULT_PERIOD_MS;
static enum continuous_target_mode continuous_mode = CONTINUOUS_TARGET_MODE_ALL;
static uint16_t continuous_targets[PDEC_CONTINUOUS_TARGETS_MAX];
static size_t continuous_target_count;

static void continuous_signal_worker(void)
{
    (void)k_sem_give(&continuous_wake_sem);
}

static size_t collect_all_targets(uint16_t *targets, size_t max_targets)
{
    size_t count = 0;
    uint16_t my_id = get_my_id();

    for (int i = 0; get_device_id(i) != 0 && count < max_targets; i++) {
        uint16_t dev_id = get_device_id(i);

        if (dev_id != my_id) {
            targets[count++] = dev_id;
        }
    }

    return count;
}

static size_t collect_effective_targets_locked(uint16_t *targets, size_t max_targets)
{
    if (continuous_mode == CONTINUOUS_TARGET_MODE_ALL) {
        return collect_all_targets(targets, max_targets);
    }

    size_t count = continuous_target_count;

    if (count > max_targets) {
        count = max_targets;
    }

    for (size_t i = 0; i < count; i++) {
        targets[i] = continuous_targets[i];
    }

    return count;
}

static void seed_custom_targets_from_all_locked(void)
{
    continuous_target_count = collect_all_targets(continuous_targets, ARRAY_SIZE(continuous_targets));
    continuous_mode = CONTINUOUS_TARGET_MODE_CUSTOM;
}

static int find_custom_target_locked(uint16_t id)
{
    for (size_t i = 0; i < continuous_target_count; i++) {
        if (continuous_targets[i] == id) {
            return (int)i;
        }
    }

    return -1;
}

static bool continuous_should_stop_locked(void)
{
    return continuous_stop_requested;
}

static void continuous_thread(void *p1, void *p2, void *p3)
{
    ARG_UNUSED(p1);
    ARG_UNUSED(p2);
    ARG_UNUSED(p3);

    while (true) {
        uint16_t targets[PDEC_CONTINUOUS_TARGETS_MAX];
        uint32_t period_ms;
        size_t target_count;

        (void)k_sem_take(&continuous_wake_sem, K_NO_WAIT);

        k_mutex_lock(&continuous_lock, K_FOREVER);
        if (continuous_should_stop_locked()) {
            continuous_thread_active = false;
            continuous_stop_requested = false;
            k_mutex_unlock(&continuous_lock);
            return;
        }

        period_ms = continuous_period_ms;
        target_count = collect_effective_targets_locked(targets, ARRAY_SIZE(targets));
        k_mutex_unlock(&continuous_lock);

        if (target_count == 0) {
            if (k_sem_take(&continuous_wake_sem, K_MSEC(CONTINUOUS_IDLE_WAIT_MS)) == 0) {
                continue;
            }
            continue;
        }

        for (size_t i = 0; i < target_count; i++) {
            uint32_t wait_ms = (uint32_t)(((uint64_t)(i + 1) * period_ms) / target_count)
                             - (uint32_t)(((uint64_t)i * period_ms) / target_count);

            send_poll(targets[i], true);

            if (k_sem_take(&continuous_wake_sem, K_MSEC(wait_ms)) == 0) {
                break;
            }
        }
    }
}

int continuous_start(void)
{
    k_mutex_lock(&continuous_lock, K_FOREVER);
    if (continuous_thread_active) {
        k_mutex_unlock(&continuous_lock);
        return -EALREADY;
    }

    continuous_stop_requested = false;
    continuous_thread_active = true;
    k_mutex_unlock(&continuous_lock);

    (void)k_thread_create(&initiator_thread_data, initiator_stack_area,
                          K_THREAD_STACK_SIZEOF(initiator_stack_area),
                          continuous_thread,
                          NULL, NULL, NULL,
                          INITIATOR_PRIORITY, 0, K_NO_WAIT);

    return 0;
}

int continuous_stop(void)
{
    k_mutex_lock(&continuous_lock, K_FOREVER);
    if (!continuous_thread_active) {
        k_mutex_unlock(&continuous_lock);
        return -EALREADY;
    }

    continuous_stop_requested = true;
    k_mutex_unlock(&continuous_lock);

    continuous_signal_worker();
    return 0;
}

bool continuous_is_running(void)
{
    bool running;

    k_mutex_lock(&continuous_lock, K_FOREVER);
    running = continuous_thread_active;
    k_mutex_unlock(&continuous_lock);

    return running;
}

int continuous_set_period_ms(uint32_t period_ms)
{
    if (period_ms == 0) {
        return -EINVAL;
    }

    k_mutex_lock(&continuous_lock, K_FOREVER);
    continuous_period_ms = period_ms;
    k_mutex_unlock(&continuous_lock);

    continuous_signal_worker();
    return 0;
}

uint32_t continuous_get_period_ms(void)
{
    uint32_t period_ms;

    k_mutex_lock(&continuous_lock, K_FOREVER);
    period_ms = continuous_period_ms;
    k_mutex_unlock(&continuous_lock);

    return period_ms;
}

int continuous_add_target(uint16_t id)
{
    int ret = 0;

    if (id == get_my_id()) {
        return -EINVAL;
    }

    k_mutex_lock(&continuous_lock, K_FOREVER);

    if (continuous_mode == CONTINUOUS_TARGET_MODE_ALL) {
        seed_custom_targets_from_all_locked();
    }

    if (find_custom_target_locked(id) >= 0) {
        ret = -EALREADY;
    } else if (continuous_target_count >= ARRAY_SIZE(continuous_targets)) {
        ret = -ENOMEM;
    } else {
        continuous_targets[continuous_target_count++] = id;
    }

    k_mutex_unlock(&continuous_lock);

    if (ret == 0) {
        continuous_signal_worker();
    }

    return ret;
}

int continuous_remove_target(uint16_t id)
{
    int index;

    if (id == get_my_id()) {
        return -EINVAL;
    }

    k_mutex_lock(&continuous_lock, K_FOREVER);

    if (continuous_mode == CONTINUOUS_TARGET_MODE_ALL) {
        seed_custom_targets_from_all_locked();
    }

    index = find_custom_target_locked(id);
    if (index < 0) {
        k_mutex_unlock(&continuous_lock);
        continuous_signal_worker();
        return -ENOENT;
    }

    for (size_t i = (size_t)index + 1; i < continuous_target_count; i++) {
        continuous_targets[i - 1] = continuous_targets[i];
    }
    continuous_target_count--;

    k_mutex_unlock(&continuous_lock);

    continuous_signal_worker();
    return 0;
}

void continuous_reset_targets(void)
{
    k_mutex_lock(&continuous_lock, K_FOREVER);
    continuous_mode = CONTINUOUS_TARGET_MODE_ALL;
    continuous_target_count = 0;
    k_mutex_unlock(&continuous_lock);

    continuous_signal_worker();
}

enum continuous_target_mode continuous_get_target_mode(void)
{
    enum continuous_target_mode mode;

    k_mutex_lock(&continuous_lock, K_FOREVER);
    mode = continuous_mode;
    k_mutex_unlock(&continuous_lock);

    return mode;
}

size_t continuous_get_effective_targets(uint16_t *targets, size_t max_targets)
{
    size_t count;

    k_mutex_lock(&continuous_lock, K_FOREVER);
    count = collect_effective_targets_locked(targets, max_targets);
    k_mutex_unlock(&continuous_lock);

    return count;
}

void pdecInitiatorStart(void)
{
    (void)continuous_start();
}
