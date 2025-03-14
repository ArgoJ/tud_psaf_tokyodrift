#ifndef TOKYODRIFT_SIMULATED_CONTROL_NODE_H
#define TOKYODRIFT_SIMULATED_CONTROL_NODE_H

#include <rclcpp/rclcpp.hpp>
#include <cmath>
#include <iostream>
#include <optional>
#include <thread>
#include <chrono>

#include "utility/msg/trajectory.hpp"
#include "utility/msg/control.hpp"
#include "utility/msg/fused_sensor.hpp"
#include "utility/msg/filtered_hall.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "std_msgs/msg/float32.hpp"

#include "marker_publisher.hpp"
#include "point_msg_helper.hpp"
#include "timer.hpp"
#include "filter.hpp"
#include "simulated_control.h"
#include "simulated_control_config.h"


class SimulatedControlNode : public rclcpp::Node {
private:
    SimulatedControlParams config;
    std::unique_ptr<SimulatedController> controller;
    OnSetParametersCallbackHandle::SharedPtr callback_handle_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Clock steady_clock_{RCL_STEADY_TIME};
    std::unique_ptr<LowpassFilter> hall_filter_;
    bool is_first_step;

    // Subscriber
    rclcpp::Subscription<utility::msg::Trajectory>::SharedPtr subTrajectory;
    rclcpp::Subscription<utility::msg::FilteredHall>::SharedPtr subFilteredHall;
    rclcpp::Subscription<utility::msg::FusedSensor>::SharedPtr subFusedSensor;
    
    // Publisher
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pubIntegratedMarker;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pubTargetMarker;
    rclcpp::Publisher<utility::msg::Control>::SharedPtr pubControl;
    
public:
    SimulatedControlNode();


private:
    void declare_parameters();
    void load_config();
    void log_config();
    // void create_timer();

    void trajectory_callback(
        const utility::msg::Trajectory::SharedPtr trajectory
    );

    void simulate_next_step();

    void control_publisher(
        const double speed,
        const double delta
    );

    rcl_interfaces::msg::SetParametersResult on_parameter_update(
        const std::vector<rclcpp::Parameter>& params
    );

    void fused_sensor_callback(
        const utility::msg::FusedSensor::SharedPtr fused_sensor_value
    );

    void hall_callback(
        const utility::msg::FilteredHall::SharedPtr hall_ptr
    );

    void timer_callback();

    void reset_timer();
};


#endif //TOKYODRIFT_SIMULATED_CONTROL_NODE_H
