#pragma once

#include <rclcpp/rclcpp.hpp>
#include "geometry_msgs/msg/point.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "utility/msg/trajectory.hpp"
#include "utility/msg/reference_line.hpp"

// Point struct with velocity
struct PointHermite {
    double x, y;
    double vx, vy;
};

class TrajectoryPlaner : public rclcpp::Node {
public:
    TrajectoryPlaner();
    // Cubic Hermite Equations
    double h00(double t) { return 2*t*t*t - 3*t*t + 1; }
    double h01(double t) { return -2*t*t*t + 3*t*t; }
    double h10(double t) { return t*t*t - 2*t*t + t; }
    double h11(double t) { return t*t*t - t*t; }
    // Callbacks
    void callback_refline(const utility::msg::ReferenceLine::SharedPtr msg);
    // Publish
    void publish_trajectory_vis(const std::vector<PointHermite> refline);
    void publish_trajectory(const std::vector<PointHermite> refline);
    // Functionality
    std::vector<PointHermite> compute_velocities(std::vector<PointHermite>& points, std::vector<double>& times);
    PointHermite evaluate_hermite(double t, PointHermite& p0, PointHermite& p1, double t0, double t1);
    std::vector<PointHermite> generate_trajectory(std::vector<PointHermite>& waypoints, std::vector<double>& times, int resolution);
    // Subs & Pubs
    rclcpp::Subscription<utility::msg::ReferenceLine>::SharedPtr subRefLine;
    rclcpp::Publisher<utility::msg::Trajectory>::SharedPtr pubTraj;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pubTrajVis;
    // Parameter
    int trajResolution = 10;
};