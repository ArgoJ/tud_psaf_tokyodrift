#ifndef TOKYODRIFT_CONTROL_NODE_CONFIG_H
#define TOKYODRIFT_CONTROL_NODE_CONFIG_H

typedef struct {
    double long_max_speed;
    double long_min_angle_diff_ACC;
    double long_min_angle_diff_DEACC;
    double long_switch_steps;

    double lat_step_width; 
    double lat_x_start; 
    double lat_x_end; 
    double lat_x_distance;
    double lat_wheel_base;
}

PurepursuitControlParams;


#endif // TOKYODRIFT_CONTROL_NODE_CONFIG_H