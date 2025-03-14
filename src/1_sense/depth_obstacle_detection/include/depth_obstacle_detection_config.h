#ifndef TOKYODRIFT_DEPTH_OBSTACLE_DETECTION_CONFIG_H
#define TOKYODRIFT_DEPTH_OBSTACLE_DETECTION_CONFIG_H

#include <cmath>
#include <stdint.h>
#include <omp.h>

typedef struct {
    double depth_scale;
    double camera_pitch;
    double camera_z_offset;
    double camera_x_offset;
    double lane_width;
    double z_low_filter;
    double z_high_filter;
    double x_high_filter;
    uint32_t point_threshold;
    uint8_t num_threads;

    double fx;
    double fy;
    double cx;
    double cy;
} DepthObstacleDetectionParams;

void post_init_occupancy_grid_params(DepthObstacleDetectionParams* config) {
    config->num_threads = std::clamp(config->num_threads, static_cast<uint8_t>(1), static_cast<uint8_t>(omp_get_num_threads()));
}

#endif // TOKYODRIFT_DEPTH_OBSTACLE_DETECTION_CONFIG_H