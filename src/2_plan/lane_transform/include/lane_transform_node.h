#ifndef TOKYODRIFT_LANE_TRANSFORM_NODE_H
#define TOKYODRIFT_LANE_TRANSFORM_NODE_H

#include <rclcpp/rclcpp.hpp>
#include <cmath>
#include <iostream>
#include <optional>
#include <thread>
#include <chrono>
#include <mutex>

#include "utility/msg/trajectory.hpp"
#include "utility/msg/control.hpp"
#include "utility/msg/fused_sensor.hpp"
#include "utility/msg/filtered_hall.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/float32.hpp"
#include "geometry_msgs/msg/point.hpp"

#include "lane_transform_config.h"
#include "marker_publisher.hpp"
#include "point_msg_helper.hpp"
#include "timer.hpp"
#include "integrator.hpp"



class LaneTransformNode : public rclcpp::Node {
private:
    std::mutex state_mutex;

    LaneTransformParams config;
    BicycleState current_state_;
    bool integrator_running_;
    bool block_velocity_;

    rclcpp::Clock steady_clock_{RCL_STEADY_TIME};
    rclcpp::Time time_;
    bool fist_integration = false;
    double last_delta;
    double last_velocity;
    double last_acceleration;

    std::vector<geometry_msgs::msg::Point> previous_lane_;
    uint8_t previous_current_lane;

    std::unique_ptr<Integrator> integrator_;
    OnSetParametersCallbackHandle::SharedPtr callback_handle_;

    // Subscriber
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subCamera;
    rclcpp::Subscription<utility::msg::Trajectory>::SharedPtr subTrajectory;
    rclcpp::Subscription<utility::msg::FusedSensor>::SharedPtr subFusedSensor;
    rclcpp::Subscription<utility::msg::FilteredHall>::SharedPtr subFilteredHall;
    
    // Publisher
    rclcpp::Publisher<utility::msg::Trajectory>::SharedPtr pubTrajectory;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pubTransformedMarkers;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pubIntegratedState;
    

public:
    /**
     * @brief Constructs a LaneTransformNode instance.
     *
     * This constructor initializes the node by declaring parameters, loading configuration, 
     * setting up callback handlers for parameter updates, and creating subscriptions 
     * and publishers for relevant topics. It also initializes the integrator and 
     * sets initial values for various variables used in processing the lane transformations.
     */
    LaneTransformNode();


private:
    /**
     * @brief Declares parameters for the LaneTransformNode.
     * 
     * This function defines the parameters that can be set via the ROS parameter server. The parameters include values like 
     * wheelbase, max/min speed, acceleration, delta, and time step.
     */
    void declare_parameters(); 

    /**
     * @brief Loads configuration parameters from the ROS parameter server.
     * 
     * This function loads the values of configuration parameters such as wheelbase, max/min speed, acceleration, delta, and time step 
     * from the parameter server into the node's internal configuration structure.
     */
    void load_config();

    /**
     * @brief Logs the current configuration of the LaneTransformNode.
     * 
     * This function logs the values of key configuration parameters, such as wheelbase, speed, acceleration, delta, and time step, 
     * to the console for debugging purposes.
     */
    void log_config();

    /**
     * @brief Sets the integrator as running and initializes the state.
     *
     * This method marks the integrator as running and initializes the time, 
     * current state, and the first integration flag to start the integration process.
     */
    void set_integrator_running();

    /**
     * @brief Transforms a trajectory wrt the integrated state.
     *
     * This method takes a given state and trajectory points and applies a 2D 
     * Helmert transformation to each point in the trajectory based on the current state.
     *
     * @param state The current state of the bicycle model.
     * @param trajectory The trajectory to be transformed.
     * @return A vector of transformed trajectory points.
     */
    std::vector<geometry_msgs::msg::Point> transform(
        const BicycleState& state,
        const std::vector<geometry_msgs::msg::Point>& trajectory
    );

    /**
     * @brief Converts a bicycle state into a geometry_msgs::msg::Point.
     *
     * This method converts the provided bicycle model state to a point representation
     * in the x, y plane.
     *
     * @param state The bicycle state to be converted.
     * @return A point representing the bicycle state.
     */
    geometry_msgs::msg::Point state2point(
        const BicycleState& state
    );

    /**
     * @brief Starts the integration process.
     *
     * This callback is triggered when a camera image message is received, and it 
     * ensures that the integrator is started for further processing.
     *
     * @param msg The camera image message.
     */
    void camera_callback(
        const sensor_msgs::msg::Image::SharedPtr msg
    );

    /**
     * @brief Callback function for processing trajectory messages.
     *
     * This callback is triggered when a trajectory message is received, and it processes 
     * the trajectory by transforming it and publishing the transformed trajectory and markers.
     *
     * @param traj_msg The trajectory message to be processed.
     */
    void trajectory_callback(
        const utility::msg::Trajectory::SharedPtr trajectory
    );

    /**
     * @brief Callback function for processing fused sensor messages.
     *
     * This callback is triggered when a fused sensor message is received, and it updates 
     * the integrator's state based on the provided sensor data.
     *
     * @param fused_sensor_ptr The fused sensor message.
     */
    void fused_sensor_callback(
        const utility::msg::FusedSensor::SharedPtr hall_ptr
    );

    /**
     * @brief Callback function for processing filtered hall sensor messages.
     *
     * This callback is triggered when a filtered hall sensor message is received, and 
     * it updates the velocity of the system based on the provided data.
     *
     * @param hall_ptr The filtered hall sensor message.
     */
    void hall_callback(
        const utility::msg::FilteredHall::SharedPtr hall_ptr
    );

    /**
     * @brief Simulates the next step in the lane transformation model.
     * 
     * This function integrates the current state of the bicycle model, using the last known velocity, delta, and acceleration values. 
     * It handles invalid input values by checking if the last velocity, delta, or acceleration is finite. If any of these values are 
     * invalid, a warning message is logged, and the function returns without further processing. The new state is then computed and 
     * published as a marker.
     *
     * @param ts Time step for the simulation.
     */
    void simulate_next_step(
        double ts
    );

    /**
     * @brief Publishes the transformed lane to a ROS topic.
     * 
     * This function creates a trajectory message with the transformed lane points, current lane, and frame ID, and publishes it 
     * using the given lane_publisher.
     *
     * @param transformed_lane Vector of points representing the transformed lane.
     * @param current_lane The current lane index or identifier.
     * @param lane_publisher The ROS publisher to which the transformed lane message is published.
     * @param frame_id The reference frame for the lane message.
     */
    void publish_lane(
        const std::vector<geometry_msgs::msg::Point>& transformed_lane, 
        const uint8_t current_lane,
        const rclcpp::Publisher<utility::msg::Trajectory>::SharedPtr& lane_publisher, 
        const std::string& frame_id =  "transformed_lane_frame"
    );

    /**
     * @brief Callback for handling parameter updates from the ROS parameter server.
     * 
     * This function processes a list of parameters, updating the configuration values such as wheelbase, max/min speed, acceleration, 
     * delta, and time step based on the received parameters.
     *
     * @param params A list of parameters to be updated.
     * @return A result object indicating whether the parameter update was successful.
     */
    rcl_interfaces::msg::SetParametersResult on_parameter_update(
        const std::vector<rclcpp::Parameter> &params
    );

    
};


#endif //TOKYODRIFT_LANE_TRANSFORM_NODE_H
