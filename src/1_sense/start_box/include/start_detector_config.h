#ifndef TOKYODRIFT_START_DETECTOR_CONFIG_H
#define TOKYODRIFT_START_DETECTOR_CONFIG_H

#include <stdint.h>

typedef struct StartDetectorParams{ 
    uint8_t window_size = 50;
    uint8_t pixels_threshold = 70;
    uint8_t squares_threshold = 50;
    uint8_t step_size = 10;
    uint64_t total_pixels = 50 * 50;
}StartDetectorParams;

//TODO I think this one is unneccessary 
//void post_init_start_detector_params(StartDetectorParams* config);

#endif // TOKYODRIFT_START_DETECTOR_CONFIG_H