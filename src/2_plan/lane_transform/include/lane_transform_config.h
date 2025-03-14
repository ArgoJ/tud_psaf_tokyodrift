#ifndef TOKYODRIFT_LANE_TRANSFORM_CONFIG_H
#define TOKYODRIFT_LANE_TRANSFORM_CONFIG_H

#include <stdint.h>

typedef struct {
    double wheelbase;
    double max_speed;
    double min_speed;
    double max_acceleration;
    double min_acceleration;
    double max_delta;
    double min_delta;
    double ts;
} LaneTransformParams;

#endif //TOKYODRIFT_LANE_TRANSFORM_CONFIG_H