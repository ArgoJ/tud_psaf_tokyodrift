/**
 * @file trajectory_planer.cpp
 * @brief Implementation of the TrajectoryPlaner class for trajectory planning.
 * 
 * This file contains the constructor of the `TrajectoryPlaner` class, which handles 
 * the subscription to reference line messages and the publication of trajectory messages.
 * The `TrajectoryPlaner` class listens for incoming reference line data and processes it 
 * to generate trajectory information for a robot or vehicle.
 */

#include "../include/trajectory_planer.h"

/**
 * @brief Constructs a TrajectoryPlaner node and initializes subscriptions and publications.
 * 
 * This constructor initializes the ROS 2 node with the name "TrajectoryPlaner" and sets up 
 * a subscription to the "topic" topic for receiving reference line messages. It also creates 
 * a publisher for sending trajectory messages on the "topic" topic.
 * 
 * The subscription to the reference line will trigger the `callback_refline` method whenever 
 * a message is received. The publisher will be used to send computed trajectory data.
 */
TrajectoryPlaner::TrajectoryPlaner() : Node("TrajectoryPlaner"){
    // Subscription to reference line data
    subRefLine = this->create_subscription<utility::msg::ReferenceLine>(
        "topic", 5, std::bind(&TrajectoryPlaner::callback_refline, this, std::placeholders::_1)
    );

    // Publisher for trajectory data
    pubTraj = this->create_publisher<utility::msg::Trajectory>("topic", 5);
}

// Functionality 

/**

    @brief Computes the velocities at each point using finite difference methods.



    Uses forward, backward, and central difference to approximate derivatives.



    @param points Vector of Hermite points.

    @param times Vector of corresponding time values.

    @return std::vector Modified list of points with computed velocities.
*/

std::vector<PointHermite> TrajectoryPlaner::compute_velocities(std::vector<PointHermite>& points, std::vector<double>& times) {
    std::vector<PointHermite> result = points;
    int n = points.size();

    for (int i = 0; i < n; ++i) {
        if (i == 0) {  // Forward difference for first point
            result[i].vx = (points[i + 1].x - points[i].x) / (times[i + 1] - times[i]);
            result[i].vy = (points[i + 1].y - points[i].y) / (times[i + 1] - times[i]);
        } 
        else if (i == n - 1) {  // Backward difference for last point
            result[i].vx = (points[i].x - points[i - 1].x) / (times[i] - times[i - 1]);
            result[i].vy = (points[i].y - points[i - 1].y) / (times[i] - times[i - 1]);
        } 
        else {  // Central difference for middle points
            result[i].vx = (points[i + 1].x - points[i - 1].x) / (times[i + 1] - times[i - 1]);
            result[i].vy = (points[i + 1].y - points[i - 1].y) / (times[i + 1] - times[i - 1]);
        }
    }

    return result;
}


/**

    @brief Generates a trajectory using Hermite interpolation.



    Computes velocities first, then interpolates between waypoints with the given resolution.



    @param waypoints Vector of Hermite waypoints.

    @param times Vector of corresponding time values.

    @param resolution The number of interpolated points between waypoints.

    @return std::vector The interpolated trajectory.
*/

std::vector<PointHermite> TrajectoryPlaner::generate_trajectory(std::vector<PointHermite>& waypoints, std::vector<double>& times, int resolution) {
    std::vector<PointHermite> points = compute_velocities(waypoints, times);
    std::vector<PointHermite> trajectory;
    for (size_t i = 0; i < points.size() - 1; ++i) {
        double t0 = times[i], t1 = times[i + 1];

        for (int j = 0; j <= resolution; ++j) {
            double t = t0 + j * (t1 - t0) / resolution;
            PointHermite interpolated = evaluate_hermite(t, points[i], points[i + 1], t0, t1);
            trajectory.push_back(interpolated);
        }
    }
    return trajectory;
}



/**

    @brief Evaluates a Hermite interpolated point at a given time.



    Uses Hermite basis functions to interpolate position values between two points.



    @param t The time at which to evaluate.

    @param p0 The starting point.

    @param p1 The ending point.

    @param t0 The start time.

    @param t1 The end time.

    @return PointHermite The interpolated point.
*/


PointHermite TrajectoryPlaner::evaluate_hermite(double t, PointHermite& p0, PointHermite& p1, double t0, double t1) {
    double dt = t1 - t0;
    double s = (t - t0) / dt;  // Normalize t to [0,1]

    PointHermite result;
    result.x = h00(s) * p0.x + h01(s) * p1.x + h10(s) * p0.vx * dt + h11(s) * p1.vx * dt;
    result.y = h00(s) * p0.y + h01(s) * p1.y + h10(s) * p0.vy * dt + h11(s) * p1.vy * dt;

    return result;
}

// Publishing 

void TrajectoryPlaner::publish_trajectory(const std::vector<PointHermite> refline){
    
}

/**

    @brief Publishes a visualization of the trajectory as a marker.



    Uses visualization_msgs::msg::Marker to visualize the trajectory in the map frame.



    @param refline Vector of Hermite points representing the trajectory.
*/
void TrajectoryPlaner::publish_trajectory_vis(const std::vector<PointHermite> refline){
    auto marker = visualization_msgs::msg::Marker();
    marker.header.frame_id = "map";
    marker.header.stamp = this->now();
    marker.ns = "trajectory";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.action = visualization_msgs::msg::Marker::ADD;

    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;

    marker.scale.x = 0.1;

    for (auto& p : refline){
        geometry_msgs::msg::Point point;
        point.x = p.x;
        point.y = p.y;
        point.z = 0;
        marker.points.push_back(point);
    }

    pubTrajVis->publish(marker);
}


// Callbacks


void publish_trajectory_vis(const std::vector<PointHermite> refline){
    
}

/**

    @brief Callback function for processing incoming reference line messages.



    Converts message data into a list of PointHermite waypoints with adjusted positions,

    computes a trajectory, and publishes it.



    @param msg Shared pointer to the received RefLine message.
*/

void TrajectoryPlaner::callback_refline(const utility::msg::ReferenceLine::SharedPtr msg){
    std::vector<PointHermite> refline;
    std::vector<double> times;
    for (int i = 0; i < msg->points.size(); i++){
        times.push_back(static_cast<double>(i));
        PointHermite p;
        p.x = static_cast<double>(msg->points[i].x) + 0.5;
        p.y = static_cast<double>(msg->points[i].y) + 0.5;
        refline.push_back(p);
    }
    this->generate_trajectory(refline, times, trajResolution);
    this->publish_trajectory(refline);
}