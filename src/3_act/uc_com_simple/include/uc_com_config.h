#ifndef TOKYODRIFT_CONTROL_NODE_CONFIG_H
#define TOKYODRIFT_CONTROL_NODE_CONFIG_H

typedef struct {
    double min_angle_ucb;
    double max_angle_ucb;
    double min_speed_ucb;
    double max_speed_ucb;
    double min_angle_diff;
    double switch_steps;
    double steering_speed_factor;
    bool avoid_crash;

}UcComParams;

void post_init_control_node_params(UcComParams* config);

#endif // TOKYODRIFT_CONTROL_NODE_CONFIG_H