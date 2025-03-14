#ifndef TOKYODRIFT_SIMULATED_CONTROL_H
#define TOKYODRIFT_SIMULATED_CONTROL_H

#include <rclcpp/rclcpp.hpp>
#include <stdint.h>
#include <iostream>
#include <bits/stdc++.h>
#include <stdexcept>
#include <cmath>
#include <mutex>

#include "geometry_msgs/msg/point.hpp"
#include "simulated_control_config.h"
#include "integrator.hpp"
#include "point_msg_helper.hpp"


enum DRIVE_STATE {
    STOP,
    BACKWARD,
    FORWARD
};


class SimulatedController {
private:
    const SimulatedControlParams& config;
    std::unique_ptr<Integrator> integrator;
    
    rclcpp::Logger logger_{rclcpp::get_logger("simulated_control")};

    std::mutex trajectory_mutex_;
    std::vector<geometry_msgs::msg::Point> control_trajectory;
    uint32_t trajectory_index = 0;
    DRIVE_STATE start_stop = DRIVE_STATE::FORWARD; 
    bool temp_stop = false;

    BicycleState current_state {0.0, 0.0, 0.0, 0.0};
    BicycleInput current_input {0.0, 0.0};
    BicycleInput actual_input {0.0, 0.0};
    double actual_velocity = 0.0;
    double ts = 0.0;
    
public:
    /**
     * @brief Constructor for the SimulatedController class.
     * 
     * Initializes the SimulatedController with the provided configuration parameters.
     * Creates an integrator instance using the wheelbase parameter from the config.
     * 
     * @param config The configuration parameters used to initialize the controller.
     */
    SimulatedController(
        const SimulatedControlParams& config
    );

    // no default constructor
    SimulatedController() = delete;

    // Prevent copying
    SimulatedController(const SimulatedController&) = delete;
    SimulatedController& operator=(const SimulatedController&) = delete;

    /**
     * @brief Sets the control trajectory for the controller.
     * 
     * This function updates the control trajectory and resets the trajectory index and current state to the initial state.
     * It also sets a temporary stop flag to false.
     * 
     * @param trajectory A vector of geometry_msgs::msg::Point representing the trajectory.
     */
    void set_control_trajectory(
        const std::vector<geometry_msgs::msg::Point>& trajectory
    );

    /**
     * @brief Sets the starting and stopping state of the controller.
     * 
     * This function sets the current drive state to either forward, backward, or stop.
     * 
     * @param start_stop The desired drive state (FORWARD, BACKWARD, or STOP).
     */
    void set_start_stop(
        const DRIVE_STATE start_stop
    );

    /**
     * @brief Sets the actual velocity of the system.
     * 
     * This function updates the actual velocity used in the control calculations.
     * 
     * @param velocity The current velocity of the system.
     */
    void set_actual_velocity(
        const double velocity
    );

    /**
     * @brief Sets the actual input values (steering angle and acceleration).
     * 
     * This function updates the actual input for the system.
     * 
     * @param input The input values representing steering angle (delta) and acceleration (a).
     */
    void set_actual_input(
        const BicycleInput input
    );

    /**
     * @brief Retrieves the current state of the bicycle system.
     * 
     * This function returns the current state of the system, including position and velocity.
     * 
     * @return BicycleState The current state of the system.
     */
    BicycleState get_state();

    /**
     * @brief Retrieves the current input and target purepursuit point for the control trajectory.
     * 
     * This function calculates the current control input based on the trajectory and current state.
     * It returns the calculated input along with the target point from the trajectory.
     * 
     * @return std::pair<BicycleInput, geometry_msgs::msg::Point> A pair containing the control input and target point.
     */
    std::pair<BicycleInput, geometry_msgs::msg::Point> get_input();

    /**
     * @brief Simulates the next state of the system based on the current input and model.
     * 
     * This function integrates the bicycle model using the current state and control input to compute the next state.
     * It clamps the resulting velocity to within the specified speed range.
     * 
     * @return BicycleState The next state of the system.
     */
    BicycleState simulate_next_state();
    

private:
    /**
     * @brief Updates the control input based on the trajectory and current state.
     * 
     * This function calculates the steering angle (delta) and acceleration (a) based on the trajectory 
     * and the current position with the purepursuit algorithm. 
     * It is using the provided configuration parameters to adjust the input.
     * 
     * @param trajectory A vector of points representing the control trajectory.
     * @param current The current state of the system.
     * 
     * @return std::pair<BicycleInput, geometry_msgs::msg::Point> A pair containing the updated control input and target point.
     */
    std::pair<BicycleInput, geometry_msgs::msg::Point> update_input(
        const std::vector<geometry_msgs::msg::Point>& trajectory,
        const BicycleState& current
    );

    /**
     * @brief Computes the desired acceleration based on the current velocity and velocity factor.
     * 
     * This function calculates the desired acceleration to reach a target velocity, considering the current drive state and 
     * clamping the acceleration to lie within the configured limits.
     * 
     * @param current_velocity The current velocity of the system.
     * @param velocity_factor A factor that adjusts the target velocity based on the trajectory.
     * 
     * @return double The calculated desired acceleration.
     */
    double get_desired_acceleration(
        const double current_velocity,
        const double angle_factor
    );

    /**
     * @brief Fuses the actual input and the current input to generate a final control input.
     * 
     * This function combines the actual input (e.g., from sensors) with the current input (e.g., from previous control) to generate
     * a fused control input using a weighted average based on the configured lambda value.
     * 
     * @return BicycleInput The fused control input.
     */
    BicycleInput get_fused_input();

    /**
     * @brief Sets the current trajectory index.
     * 
     * This function updates the trajectory index and ensures that it is within the bounds of the control trajectory.
     * 
     * @param index The new trajectory index to be set.
     */
    void set_trajectory_index(
        const uint32_t index
    );

    static geometry_msgs::msg::Point state2point(
        const BicycleState& state
    );
};

#endif //TOKYODRIFT_SIMULATED_CONTROL_H