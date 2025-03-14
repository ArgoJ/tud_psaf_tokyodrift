#ifndef TOKYODRIFT_CONTROL_NODE_H
#define TOKYODRIFT_CONTROL_NODE_H

#include <rclcpp/rclcpp.hpp>
#include <cmath>
#include <iostream>
#include <utility>
#include <optional>
#include <chrono>
#include <functional>

#include "utility/msg/trajectory.hpp"
#include "utility/msg/control.hpp"
#include "utility/msg/startbox.hpp"

#include "std_msgs/msg/int16.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "sensor_msgs/msg/range.hpp"


#include "timer.hpp"

#include "uc_com_config.h"

/**
* Enum describs the current used steering function due to the hysteresis
*
* LEFT: fct_left is used
* LEFT_TO_RIGHT: Transition from fct_left to fct_right
* RIGHT: fct_right is used
* RIGHT_TO_LEFT: Transition from fct_right to fct_left
* NEUTRAL: Used for start, when wheels are straight, combination of fct_right and fct_left
* 
*/
enum class Direction {
    LEFT,
    LEFT_TO_RIGHT,
    RIGHT,
    RIGHT_TO_LEFT,
    NEUTRAL
};


class UcComNode : public rclcpp::Node {
public:
    UcComNode();

private:

//Parameter
    UcComParams config;
    OnSetParametersCallbackHandle::SharedPtr callback_handle_;
    void declare_parameters();
    void load_config();
    void log_config();
    rcl_interfaces::msg::SetParametersResult on_parameter_update(
        const std::vector<rclcpp::Parameter>& params
    );

// ROS-Subscriber
    rclcpp::Subscription<utility::msg::Control>::SharedPtr subControl;
    rclcpp::Subscription<utility::msg::Startbox>::SharedPtr subStartDetector;
    rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr subUsFront;

// ROS-Publisher
    rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr pubSpeedUcBridge;
    rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr pubSteeringUcBridge;

// Publish Methods
    void speed_publisher();
    void angle_publisher(const int steering_angle);

// Timer
    rclcpp::TimerBase::SharedPtr timer;

// ROS Callbacks

    /**
    * Receives driving information including steering, speed and detected errors in the trajectory
    *
    * @param control Shared Pointer including driving information
    */
    void control_callback(const utility::msg::Control::SharedPtr control);

    /**
    * Receives information about start box, in particular whether to start or to wait.
    * If false -> wait, else drive
    * 
    * @param finished Shared Pointer including signal information
    */ 
    void start_detector_callback(const utility::msg::Startbox::SharedPtr finished);

    /**
    * Receives information about frontal ultrasonic sensor
    *
    * @param us_front Shared Pointer containing information how far object is away in meter
    */
    void us_front_callback(const sensor_msgs::msg::Range::SharedPtr us_front);
    


// Function methods

    /**
    * Calculates speed ucb value after receiving a speed in m/s
    * 
    * @param speed the speed in m/s
    * @return speed value as a ucb value
    */
    int speed_ms_to_ucb(const double speed);


    /**
    * Calculates steering angle ucb value after receiving a steering angle in radiant
    * Note: Focuses on deciding whether fct_left or fct_right should be used
    *       Achieves that by comparing the angle of lookahead point in a fix distance (angle_dist) to 
    *       the previous angle of the lookaheadpoint (previous_angle_radiant)
    * 
    * @param angle the angle in radiant
    * @param angle_dist a angle (not steering angle) calculated for a lookahead far in front to detect incoming curves early enough
    * @param jump True if a jump was detected in lateral_control, which means an error in the received trajectory
    * @return steering angle as a ucb value
    */
    int steering_angle_to_ucb(const double angle, const double angle_dist, const bool jump);


    /**
    * Includes the steering transform function for steering angles turning negative (right_to_left)
    * 
    * @param angle the angle in radiant
    * @return steering angle as a ucb value
    */
    int fct_left(const double angle);


    /**
    * Includes the steering transform function for steering angles turning positive (left_to_right)
    * 
    * @param angle the angle in radiant
    * @return steering angle as a ucb value
    */
    int fct_right(const double angle);


// Variables  

    // Startbox
    bool is_driving = false;

    // Crash avoidance
    bool obstacle_front = false;

    // Speed publisher
    int current_speed_ucb = 0;
    int previous_speed_ucb = 0;
    double speed_factor = 0;


    // Steering hysteresis
    Direction direction = Direction::NEUTRAL;
    bool current_direction_switch_right = false;
    bool current_direction_switch_left = false;
    bool current_jump = false;
    double previous_angle_radiant = 0;
    int direction_switch_counter = 0;

};


#endif //TOKYODRIFT_CONTROL_NODE_H
