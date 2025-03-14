#ifndef TOKYODRIFT_OBSTACLE_DETECTION_NODE_H
#define TOKYODRIFT_OBSTACLE_DETECTION_NODE_H

#include <rclcpp/rclcpp.hpp>
#include <cmath>
#include <iostream>
#include <utility>
#include <optional>
#include <chrono>
#include <functional>

#include "utility/msg/obstacle_detection.hpp"
#include "std_msgs/msg/bool.hpp"



#include "sensor_msgs/msg/range.hpp"


// NOT IN CURRENT USE BECAUSE OTHER APPROACH WORKS BETTER


class ObstacleDetectionNode : public rclcpp::Node {
public:
    ObstacleDetectionNode();

private:

// ROS-Subscriber
    rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr subUsFront;
    rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr subUsRight;

    
// ROS-Publisher
    rclcpp::Publisher<utility::msg::ObstacleDetection>::SharedPtr pubObstacleDetected;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pubLane;
   

// ROS Callbacks

    /**
     * Method checks if an object in front is detected. If a object is detected twice closer than 0.8 meter, 
     * obstacle_front is set to true, till no one is detected anymore and stay_left is set to true
     * 
     * @param us_front distance to next object in meter
     */
    void us_front_callback(const sensor_msgs::msg::Range::SharedPtr us_front);
    /**
     * Method checks if an object on the right side is detected. If a object is detected closer than 0.2 meter, 
     * obstacle_right is set to true, till no one is detected anymore and stay_left is set to false
     * 
     * @param us_front distance to next object in meter
     */
    void us_right_callback(const sensor_msgs::msg::Range::SharedPtr us_right);

    
// Publish Methods
    void object_detected_publisher();

    /**
     * Method publishes the driving lane.
     * Left lane is published if object is detected in front (us_front_callback) till no object in front is detected anymore
     * and for 50 more feedbacks calls from the front us-sensor AND no object on the right was detected for the last 10 feedback calls
     * from the right us-sensor.
     * Then the right lane gets published again
     */
    void lane_publisher();

    bool obstacle_front = false;
    bool obstacle_right = false;

    bool stay_left = false;

    int front_counter = 0;
    int right_counter = 0;
};



#endif // TOKYODRIFT_OBSTACLE_DETECTION_NODE_H