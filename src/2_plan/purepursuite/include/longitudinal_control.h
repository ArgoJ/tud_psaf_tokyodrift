//
// Created by aaron on 10/17/24.
//

#ifndef TOKYODRIFT_LONGITUDINAL_CONTROL_H
#define TOKYODRIFT_LONGITUDINAL_CONTROL_H

#include <rclcpp/rclcpp.hpp>

#include "utility/msg/trajectory.hpp"
#include "utility/msg/vehicle_state.hpp"
#include <utility>
#include <optional>
#include "purepursuit_control_config.h"
#include "point_msg_helper.hpp"


enum class Speed {
    START,
    SLOW,
    FAST,
    ACC,
    DEACC
};

class LongitudinalControl : public rclcpp::Node {
public:
    LongitudinalControl(const PurepursuitControlParams& config);

    /**
     * Method calculates the speed by using only the angle of a fix lookahead point and comparing it with previous ones to
     * decide whether it the absolute value rises or decreases. If it rises the car is supposed to slow down (DEACC) till an min speed (SLOW) is reached
     * On the other side should the car accelerate (ACC) if the angle decreases.
     * 
     * @param distance_alpha angle from a fix lookahead point
     * @return speed in m/s
     */
    double calculateSpeed(const double distance_alpha);

private:
    const PurepursuitControlParams& config;
    
    Speed speed = Speed::FAST;

    double previous_distance_alpha = 0.0;
    int speed_switch_counter = 0;
    bool current_speed_switch = false;

};


#endif //TOKYODRIFT_LONGITUDINAL_CONTROL_H
