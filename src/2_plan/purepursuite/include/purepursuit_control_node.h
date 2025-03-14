

#ifndef TOKYODRIFT_CONTROL_NODE_H
#define TOKYODRIFT_CONTROL_NODE_H

#include <rclcpp/rclcpp.hpp>
#include <cmath>
#include <iostream>
#include <utility>
#include <optional>
#include <chrono>


#include "utility/msg/trajectory.hpp"
#include "utility/msg/control.hpp"
#include "std_msgs/msg/int16.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/bool.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "sensor_msgs/msg/imu.hpp"

#include "longitudinal_control.h"
#include "lateral_control.h"

#include "timer.hpp"

#include "rclcpp/parameter.hpp"
#include "purepursuit_control_config.h"


class PurepursuitControlNode : public rclcpp::Node {
public:
    PurepursuitControlNode();

private:
//Parameter
    PurepursuitControlParams config;
    OnSetParametersCallbackHandle::SharedPtr callback_handle_;
    void declare_parameters();
    void load_config();
    void log_config();
    rcl_interfaces::msg::SetParametersResult on_parameter_update(
        const std::vector<rclcpp::Parameter>& params
    );

// ROS-Subscriber
    rclcpp::Subscription<utility::msg::Trajectory>::SharedPtr subTrajectory;
    
// ROS-Publisher
    rclcpp::Publisher<utility::msg::Control>::SharedPtr pubControl;

// ROS Callbacks
    void trajectory_callback(const utility::msg::Trajectory::SharedPtr trajectory);

// Publish Methods

    /**
    * Publishes all driving information in non ucb values, which can be acceced by uc_com
    *
    * @param speed in m/s
    * @param delta steering_angle in rad
    * @param delta_dist angle from a point towards the end of the trajectory (not a steering angle) 
    * @param jump True if jump in trajectory was detected
    */
    void control_publisher(const double speed, const double delta, const double delta_dist, const bool jump);

// Instanzen der Controller-Klassen
    std::unique_ptr<LateralControl> lateral_control;
    std::unique_ptr<LongitudinalControl> longitudinal_control;
};


#endif //TOKYODRIFT_CONTROL_NODE_H
