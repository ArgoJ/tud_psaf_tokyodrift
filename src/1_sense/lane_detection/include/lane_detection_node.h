#ifndef TOKYODRIFT_LANE_DETECTION_NODE_H
#define TOKYODRIFT_LANE_DETECTION_NODE_H

#include <rclcpp/rclcpp.hpp>
#include <vector>
#include <cv_bridge/cv_bridge.h>
#include <iostream>

#include "rclcpp/parameter.hpp"
#include "std_msgs/msg/bool.hpp"
#include "utility/msg/trajectory.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "visualization_msgs/msg/marker.hpp"

#include "point_msg_helper.hpp"
#include "marker_publisher.hpp"
#include "timer.hpp"
#include "lane_detection_helper.hpp"
#include "lane_detection.h"
#include "lane_detection_config.h"




class LaneDetectionNode : public rclcpp::Node {
private:
    LaneDetectionParams config;
    CurrentLane current_lane_;
    std::unique_ptr<LaneDetection> lane_detection;
    OnSetParametersCallbackHandle::SharedPtr callback_handle_;

    // Subscriber
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subCamera;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr subCurrentLane;

    // Publisher
    rclcpp::Publisher<utility::msg::Trajectory>::SharedPtr pubTargetLane;
    rclcpp::Publisher<utility::msg::Trajectory>::SharedPtr pubRightLane;
    rclcpp::Publisher<utility::msg::Trajectory>::SharedPtr pubLeftLane;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pubDebugImage;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pubTargetMarkers;
    
public:
    LaneDetectionNode();
    
private:
    void declare_parameters();
    void load_config();
    void log_config();

    void camera_callback(
        const sensor_msgs::msg::Image::SharedPtr msg
    );

    void current_lane_callback(
        const std_msgs::msg::Bool::SharedPtr msg
    );

    void publish_lane(
        const std::vector<cv::Point2f>& lane_fit, 
        const uint8_t current_lane,
        const rclcpp::Publisher<utility::msg::Trajectory>::SharedPtr& lane_publisher, 
        const std::string& frame_id = "lane_detection_frame"
    );

    rcl_interfaces::msg::SetParametersResult on_parameter_update(
        const std::vector<rclcpp::Parameter>& params
    );
};

#endif //TOKYODRIFT_LANE_DETECTION_NODE_H
