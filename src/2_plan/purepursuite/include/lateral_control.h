//TODO: Lenkwinkel neu kalibrieren und STEERING_ANGLE_OFFSET updaten

#ifndef TOKYODRIFT_LATERAL_CONTROL_H
#define TOKYODRIFT_LATERAL_CONTROL_H

#include <rclcpp/rclcpp.hpp>

#include "utility/msg/trajectory.hpp"
#include "utility/msg/vehicle_state.hpp"
#include <vector>
#include <cmath>
#include <iostream>
#include <utility>
#include <optional>
#include <string>

#include "purepursuit_control_config.h"
#include "point_msg_helper.hpp"


class LateralControl : public rclcpp::Node {
public:

    LateralControl(const PurepursuitControlParams& config);
    
    // return value for calculateSteeringAngle
    struct Result{
        double steering_angle_radiant;
        double max_alpha;
        bool jump;
    };

// Methods

    /**
    * Method calculates steering angle after receiving a interpolated trajectory containing 500 points
    * The method first checks for edge cases and then calculates three significant angles of different points
    * Between close_alpha and distance_alpha is the algorithm looking for the smallest angel 
    * but checks only every lat_step_width point, which is in most config files 10. 
    * After algorithm found smallest angle it safes the index as current_x and checks if it harms the boundaries.
    * 
    * FIRST JUMP DETECTION (In a deep curve suddenly a lookahead far away is picked!):
    * In particular, if the current index is bigger than the old, which means the new lookahead is further away than the old
    * AND the previous lookaheadpoint had a horizontal offsest bigger than 0.25 meters ...
    * -> Old values are used 
    *
    * Lookahead difference is calculated and steering angle as well
    *
    * SECOND JUMP DETECTION (Absolute steering angle suddenly drops)
    * In particular, if the new steering angle is more than 0.2 rad smaller than the old one
    * OR the previous absolute horizontal value from the lookahead is 0.25 meter bigger ...
    * -> Old values are used
    */
    Result calculateSteeringAngle(const std::vector<geometry_msgs::msg::Point>& points);

    /**
    * Currently NOT used method, worse results than outer attempt bc of missing jump detection
    *  
    * Another method to compute the steering angle based on a given trajectory without interpolation needed
    * The method defines the start point as the origin and finds the target point along with the unit gradient. 
    * It computes the steering angle using the calculate_delta function and determines the distant_alpha and x_distant_alpha angles 
    * before returning the final values. 
    * @param trajectory A constant reference to a vector of geometry_msgs::msg::Point representing the planned trajectory.
    * @return A pair of double values where the first value is the computed steering angle and the second value is the normalized angle at a predefined x-distance.
    */
    std::pair<double, double> calculateSteeringAngle_new(const std::vector<geometry_msgs::msg::Point>& points);


    
private:
    const PurepursuitControlParams& config;

    std::optional<int> previous_x;

    // Steering angle
    double previous_steering_ucb; 
    double previous_angle_radiant;

    // Point angle relative to car
    double previous_alpha;
    double previous_max_alpha = 0.0;

    // Point angle far away relative to car 
    double previous_distance_alpha = 0.0;

    // Horizontal shift of a point
    double previous_y = 0.0;

    bool jump = false;
};


#endif //TOKYODRIFT_LATERAL_CONTROL_H
