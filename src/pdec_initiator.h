#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define PDEC_CONTINUOUS_TARGETS_MAX 10

enum continuous_target_mode {
    CONTINUOUS_TARGET_MODE_ALL = 0,
    CONTINUOUS_TARGET_MODE_CUSTOM,
};

int continuous_start(void);
int continuous_stop(void);
bool continuous_is_running(void);

int continuous_set_period_ms(uint32_t period_ms);
uint32_t continuous_get_period_ms(void);

int continuous_add_target(uint16_t id);
int continuous_remove_target(uint16_t id);
void continuous_reset_targets(void);

enum continuous_target_mode continuous_get_target_mode(void);
size_t continuous_get_effective_targets(uint16_t *targets, size_t max_targets);

void pdecInitiatorStart(void);
