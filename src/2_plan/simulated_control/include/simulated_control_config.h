#ifndef TOKYODRIFT_SIMULATED_CONTROL_CONFIG_H
#define TOKYODRIFT_SIMULATED_CONTROL_CONFIG_H

#include <stdint.h>

typedef struct {
    double wheelbase;
    double ts;
    double max_speed;
    double min_speed;
    double max_acceleration;
    double min_acceleration;
    double max_delta;
    double min_delta;
    double angle_factor_offset;
    double lambda_use_sensor;
    double min_lookahead;
    double max_lookahead;
} SimulatedControlParams;

#endif //TOKYODRIFT_SIMULATED_CONTROL_CONFIG_H