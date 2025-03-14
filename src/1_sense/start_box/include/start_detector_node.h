#ifndef TOKYODRIFT_START_DETECTOR_NODE_H
#define TOKYODRIFT_START_DETECTOR_NODE_H

#include <rclcpp/rclcpp.hpp>
#include <cmath>
#include <iostream>
#include <utility>
#include <optional>

#include <cv_bridge/cv_bridge.h>
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/bool.hpp"
#include "start_detector.h"

#include "utility/msg/startbox.hpp"

#include "start_detector_config.h"



class StartDetectorNode : public rclcpp::Node {
public:
    
    StartDetectorNode();

    // ROS-Subscriber
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subCamera;
    
    // ROS-Publisher
    rclcpp::Publisher<utility::msg::Startbox>::SharedPtr pubStartSequenceFinished;

    // ROS Callbacks
    void camera_callback(const sensor_msgs::msg::Image::SharedPtr image_message);
    
    // Publish Methods
    void publish_start_sequence_finished(bool finished);

    //Params Methods and attributes
    StartDetectorParams config;
    OnSetParametersCallbackHandle::SharedPtr callback_handle_;
    void load_config();
    void log_config();
    rcl_interfaces::msg::SetParametersResult on_parameter_update(
        const std::vector<rclcpp::Parameter>& params
    );

private:
// Instanzen der Controller-Klassen
    StartDetector start_detector;
    bool stop_sign_visible = true; //true while stop sign is visible; if this stays false for WAITING_DURATION seconds, is driving will be set to true and the car will start driving
    bool is_driving = false; //true as soon as we start driving (oneshot variable!)
    rclcpp::Time start_time; //point in time in which the stop sign switched from visible to not visible
    rclcpp::Time initialization_start_time;///start time of the initialization phase (i.e. the sign has to be visible for atleast 1 second to be ready to detect the missing sign)
    bool start_time_initialized = false; //true if start_time was already initialized
    bool stop_sign_initialized = false; //true if stop sign was visible atleast once
    bool initialization_start_time_initialized = false; //true if the initialization start time has been initialized
    const rclcpp::Duration WAITING_DURATION = rclcpp::Duration::from_seconds(3.0); //the duration we wait before confirming the sign is missing after detecting a missing sign (continuously)
    const rclcpp::Duration INITIALIZATION_WAITING_DURATION = rclcpp::Duration::from_seconds(1.0); //the duration we wait before acknowledging a detected sign
};


#endif
