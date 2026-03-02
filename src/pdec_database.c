#include "pdec_database.h"

#define NUM_DEVICES_MAX 10

// Low 16bits of the nrf52833 device ID: NRF_FICR->DEVICEID[0]
uint16_t devices[] = {
	0xc550,
	0x0839,
	0xdc12,
    0 // Must end with 0 to detect the end of the array
};

double distances[(NUM_DEVICES_MAX*(NUM_DEVICES_MAX-1))/2] = {0};


// to store the clock offset of every other agents
#define NB_CLOCK_OFFSETS 20
clk_offset_t clock_offsets[NB_CLOCK_OFFSETS] = {0};

void set_clock_offset(uint16_t id, float clk_offset_ratio) {
    for(int i=0; i<NB_CLOCK_OFFSETS; i++) {
        if(clock_offsets[i].agent_id == id) {
            // update
            clock_offsets[i].clock_offset_ratio = clk_offset_ratio;
            return;
        }
        else if(clock_offsets[i].agent_id == 0) {
            // new
            clock_offsets[i].agent_id = id;
            clock_offsets[i].clock_offset_ratio = clk_offset_ratio;
            return;
        }
    }
}

// return 1 if not found
double get_clock_offset(uint16_t id) {
    for(int i=0; i<NB_CLOCK_OFFSETS; i++) {
        if(clock_offsets[i].agent_id == id) {
            return (1 - clock_offsets[i].clock_offset_ratio);
        }
    }
    // not found, return 1
    return 1;
}


void set_distance(uint16_t a, uint16_t b, double distance) {
    int index = 0;
    for(int i=0; devices[i] != 0; i++) {
        for(int j=i+1; devices[j] != 0; j++) {
            if((devices[i] == a && devices[j] == b) || (devices[i] == b && devices[j] == a)) {
                // TODO filtrage ?
                distances[index] = distance;
            }
            index += 1;
        }
    }
}

double get_distance(uint16_t a, uint16_t b) {
    int index = 0;
    for(int i=0; devices[i] != 0; i++) {
        for(int j=i+1; devices[j] != 0; j++) {
            if((devices[i] == a && devices[j] == b) || (devices[i] == b && devices[j] == a)) {
                return distances[index];
            }
            index += 1;
        }
    }
    
    // devices not found
    return -1;
}

