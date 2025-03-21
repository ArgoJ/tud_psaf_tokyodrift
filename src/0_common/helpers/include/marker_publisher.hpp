#ifndef MARKER_PUBLISHER_HPP
#define MARKER_PUBLISHER_HPP

#include <vector>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/point.hpp>

typedef struct {
    double r;
    double g;
    double b;
} Color;

/**
 * @brief Publishes a series of lane marker points as a default line strip to the given marker publisher.
 *
 * This function creates a `visualization_msgs::msg::Marker` message with the provided parameters and 
 * publishes it to the specified marker publisher.
 * The points will be rendered as a line strip in the given frame of reference.
 *
 * @param points The list of points to be published as a marker.
 * @param marker_publisher A shared pointer to the publisher for the marker.
 * @param frame_id The reference frame in which the marker will be published.
 * @param clock The shared pointer to the ROS clock for timestamping the marker.
 * @param color The color of the marker.
 * @param ns The namespace for the marker (default is an empty string).
 * @param id The unique identifier for the marker (default is 0).
 * @param scale The scale of the marker (default is 0.03).
 * @param alpha The alpha (transparency) value of the marker (default is 1.0).
 * @param type The type of the marker (default is `LINE_STRIP`).
 */
inline void publish_marker_points(
        const std::vector<geometry_msgs::msg::Point>& points,
        const rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr& marker_publisher,
        const std::string& frame_id,
        const rclcpp::Clock::SharedPtr& clock,
        const Color color,
        const std::string ns = "",
        const int id = 0,
        const double scale = 0.03,
        const double alpha = 1.0,
        const int32_t type = visualization_msgs::msg::Marker::LINE_STRIP
) {
    visualization_msgs::msg::Marker marker_msg;

    marker_msg.header.stamp = clock->now();
    marker_msg.header.frame_id = frame_id;
    marker_msg.ns = ns;
    marker_msg.id = id;
    marker_msg.type = type;
    marker_msg.action = visualization_msgs::msg::Marker::ADD;
    marker_msg.lifetime = rclcpp::Duration(0, 0);

    marker_msg.scale.x = scale;
    marker_msg.scale.y = scale;
    marker_msg.scale.z = scale;
    marker_msg.color.a = alpha;
    marker_msg.color.r = color.r;
    marker_msg.color.g = color.g;
    marker_msg.color.b = color.b;

    marker_msg.points = points;
    marker_publisher->publish(marker_msg);
}

/**
 * @brief Publishes a single marker point (e.g., sphere) to the given marker publisher.
 *
 * This function creates a `visualization_msgs::msg::Marker` message with the provided parameters and 
 * publishes it to the specified marker publisher.
 * The marker will be rendered as a sphere at the given point in the specified frame of reference.
 *
 * @param point The position of the marker.
 * @param marker_publisher A shared pointer to the publisher for the marker.
 * @param frame_id The reference frame in which the marker will be published.
 * @param clock The shared pointer to the ROS clock for timestamping the marker.
 * @param color The color of the marker.
 * @param ns The namespace for the marker (default is an empty string).
 * @param id The unique identifier for the marker (default is 0).
 * @param scale The scale of the marker (default is 0.03).
 * @param alpha The alpha (transparency) value of the marker (default is 1.0).
 */
inline void publish_marker(
        const geometry_msgs::msg::Point& point,
        const rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr& marker_publisher,
        const std::string& frame_id,
        const rclcpp::Clock::SharedPtr& clock,
        const Color color,
        const std::string ns = "",
        const int id = 0,
        const double scale = 0.03,
        const double alpha = 1.0
) {
    visualization_msgs::msg::Marker marker_msg;

    marker_msg.header.stamp = clock->now();
    marker_msg.header.frame_id = frame_id;
    marker_msg.ns = ns;
    marker_msg.id = id;
    marker_msg.type = visualization_msgs::msg::Marker::SPHERE;
    marker_msg.action = visualization_msgs::msg::Marker::ADD;
    marker_msg.lifetime = rclcpp::Duration(0, 0);

    marker_msg.scale.x = scale;
    marker_msg.scale.y = scale;
    marker_msg.scale.z = scale;
    marker_msg.color.a = alpha;
    marker_msg.color.r = color.r;
    marker_msg.color.g = color.g;
    marker_msg.color.b = color.b;

    marker_msg.pose.position = point;
    marker_publisher->publish(marker_msg);
}

#endif