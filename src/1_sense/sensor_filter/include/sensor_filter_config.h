#ifndef TOKYODRIFT_SENSOR_FILTER_CONFIG_H
#define TOKYODRIFT_SENSOR_FILTER_CONFIG_H

#include <stdint.h>

typedef struct {
    double wheelbase;
    double wheelradius;

    double max_speed;
    double min_speed;
    double max_acceleration;
    double min_acceleration;
    double max_delta;
    double min_delta;

    double x_linear_acceleration_offset;
    double z_angular_velocity_offset;

    double tau_hall;
} SensorFilterParams;

#endif //TOKYODRIFT_SENSOR_FILTER_CONFIG_H