#ifndef TOKYODRIFT_SENSOR_FILTER_NODE_H
#define TOKYODRIFT_SENSOR_FILTER_NODE_H

#include <rclcpp/rclcpp.hpp>
#include <cmath>
#include <iostream>
#include <optional>
#include <thread>
#include <chrono>

#include "std_msgs/msg/float32.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "utility/msg/fused_sensor.hpp"
#include "utility/msg/filtered_hall.hpp"
#include "utility/msg/startbox.hpp"

#include "timer.hpp"
#include "filter.hpp"
#include "sensor_filter_config.h"
#include "kalman_estimator.hpp"

#define DEFAULT_DELTA_MAX 18.0 * M_PI / 180.0


class SensorFilterNode : public rclcpp::Node {
private:
    SensorFilterParams config;
    bool is_driving = false;
    double sum_lin_acc = 0.0;
    double sum_ang_vel = 0.0;
    uint16_t imu_count = 0;

    OnSetParametersCallbackHandle::SharedPtr callback_handle_;

    // Filter
    std::unique_ptr<KalmanStateEstimator> kalman_filter_;
    std::unique_ptr<LowpassFilter> hall_filter_;

    // Subscriber
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subIMU;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr subHallSensor;
    rclcpp::Subscription<utility::msg::Startbox>::SharedPtr subStartbox;
    
    // Publisher
    rclcpp::Publisher<utility::msg::FusedSensor>::SharedPtr pubFilteredIMU;
    rclcpp::Publisher<utility::msg::FilteredHall>::SharedPtr pubFilteredHall;
    
public:
    SensorFilterNode();


private:
    void declare_parameters();
    void load_config();
    void log_config();

    rcl_interfaces::msg::SetParametersResult on_parameter_update(
        const std::vector<rclcpp::Parameter>& params
    );

    void imu_callback(
        const sensor_msgs::msg::Imu::SharedPtr hall_value
    );

    void hall_callback(
        const std_msgs::msg::Float32::SharedPtr hall_ptr
    );

    void startbox_callback(
        const utility::msg::Startbox::SharedPtr startbox_ptr
    );

    double calculate_delta(
        double velocity,
        double angular_velocity
    );

    void publish_imu(
        double velocity,
        double longitudinal_acceleration,
        double angular_velocity,
        double delta
    );
    
    void publish_hall(
        double hall_value,
        double velocity
    );
};


#endif //TOKYODRIFT_SENSOR_FILTER_NODE_H
