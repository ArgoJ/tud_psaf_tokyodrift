#ifndef TOKYODRIFT_BEZIER_CURVE_NODE_H
#define TOKYODRIFT_BEZIER_CURVE_NODE_H

#include <rclcpp/rclcpp.hpp>
#include <cmath>
#include <iostream>
#include <optional>
#include <thread>
#include <chrono>

#include "utility/msg/trajectory.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "geometry_msgs/msg/point.hpp"

#include "marker_publisher.hpp"
#include "point_msg_helper.hpp"
#include "timer.hpp"
#include "cubic_bezier.hpp"
#include "bezier_curve_config.h"


class BezierCurveNode : public rclcpp::Node {
private:
    BezierCurveParams config;
    OnSetParametersCallbackHandle::SharedPtr callback_handle_;

    // Subscriber
    rclcpp::Subscription<utility::msg::Trajectory>::SharedPtr subTrajectory;
    
    // Publisher
    rclcpp::Publisher<utility::msg::Trajectory>::SharedPtr pubBezierCurve;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pubTargetMarkerLine;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pubBezierControlMarker;
    
public:
    /**
     * @brief Constructs the BezierCurveNode and initializes parameters, subscribers, and publishers.
     *
     * This constructor initializes the BezierCurveNode by setting up necessary parameters, subscribing to the trajectory topic,
     * and publishing Bezier curve and marker data. It also sets up a callback to handle parameter updates.
     */
    BezierCurveNode();


private:
    /**
     * @brief Declares the parameters used by the BezierCurveNode.
     *
     * This function declares the necessary parameters for configuring the Bezier curve computation, including the wheelbase,
     * sampling time for Bezier, Bezier distance, and control point fraction.
     */
    void declare_parameters();

    /**
     * @brief Loads the configuration parameters from the ROS parameter server.
     *
     * This function retrieves the parameter values for the Bezier curve computation settings (sampling time, Bezier distance, 
     * and control point fraction) from the ROS parameter server and stores them in the node's configuration.
     */
    void load_config();

    /**
     * @brief Logs the current configuration parameters for debugging purposes.
     *
     * This function logs the current parameter values for the Bezier curve computation, such as sampling time, adjusted distance,
     * and control point fraction.
     */
    void log_config();

    /**
     * @brief Callback function for handling incoming trajectory messages.
     *
     * This function is invoked when a new trajectory message is received. It processes the trajectory points and computes a cubic
     * Bezier curve using the provided parameters. The Bezier curve and control points are then published, along with visual markers.
     *
     * @param trajectory The incoming trajectory message containing the trajectory points.
     */
    void trajectory_callback(
        const utility::msg::Trajectory::SharedPtr trajectory
    );

    /**
     * @brief Publishes the computed Bezier curve to the appropriate topic.
     *
     * This function constructs a Trajectory message from the computed Bezier curve and publishes it to the `tokyodrift/plan/bezier_curve/trajectory`
     * topic with the current lane information.
     *
     * @param trajectory A vector of geometry_msgs::msg::Point representing the computed Bezier curve.
     * @param current_lane The current lane index of the trajectory.
     */
    void publish_bezier_curve(
        const std::vector<geometry_msgs::msg::Point>& trajectory,
        const uint8_t current_lane
    );

    /**
     * @brief Callback function for handling parameter updates.
     *
     * This function is invoked when parameters are updated via the ROS parameter server. It updates the corresponding configuration
     * values and logs the updated configuration.
     *
     * @param params A vector of updated parameters.
     * @return A result message indicating whether the update was successful.
     */
    rcl_interfaces::msg::SetParametersResult on_parameter_update(
        const std::vector<rclcpp::Parameter>& params
    );
};


#endif //TOKYODRIFT_BEZIER_CURVE_NODE_H
