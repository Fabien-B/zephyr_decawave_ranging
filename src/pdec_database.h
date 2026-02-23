#pragma once
#include <inttypes.h>

extern uint16_t devices[];


typedef struct {
    uint16_t agent_id;
    float clock_offset_ratio;
} clk_offset_t;

void set_clock_offset(uint16_t id, float clk_offset_ratio);

// return 1 if not found
double get_clock_offset(uint16_t id);
