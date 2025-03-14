#include <chrono>
#include <memory>
#include <thread>

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>

#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "utility/msg/fused_sensor.hpp"  // Adjust include path as needed
#include "sensor_filter_node.h"

using namespace std::chrono_literals;

class SensorFilterIntegrationTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Initialize ROS 2 if it hasn't been initialized already.
        if (!rclcpp::ok()) {
            rclcpp::init(0, nullptr);
        }
        
        srand(static_cast<unsigned>(time(nullptr)));

        // Create a node for publishing and subscribing (this is your test node)
        test_node_ = rclcpp::Node::make_shared("sensor_filter_integration_test_node");

        // Publishers for sensor messages.
        hall_pub_ = test_node_->create_publisher<std_msgs::msg::Float32>("dt8_data", 10);
        imu_pub_ = test_node_->create_publisher<sensor_msgs::msg::Imu>("imu_data", 10);

        // Subscription for fused sensor output.
        rclcpp::QoS qos(10);
        qos.reliable();
        fused_sub_ = test_node_->create_subscription<utility::msg::FusedSensor>(
            "fused_sensor", qos,
            [this](const utility::msg::FusedSensor::SharedPtr msg) {
                last_msg_ = msg;
                message_received_ = true;
            });

        // Run the executor in a background thread.
        executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
        auto sensor_filter_node = std::make_shared<SensorFilterNode>();  // Dein Node

        executor_->add_node(sensor_filter_node);
        std::this_thread::sleep_for(100ms);  // Kurze Wartezeit für Subscription Setup
        executor_->add_node(test_node_);

        executor_thread_ = std::thread([this]() { executor_->spin(); });
    }

    void TearDown() override {
        executor_->cancel();
        if (executor_thread_.joinable()) {
        executor_thread_.join();
        }
        rclcpp::shutdown();
    }

    void publishTestData(
            float hall_value, 
            float accel_x, 
            float angular_vel_z
    ) {
        auto hall_msg = std::make_shared<std_msgs::msg::Float32>();
        hall_msg->data = hall_value;
        hall_pub_->publish(*hall_msg);

        auto imu_msg = std::make_shared<sensor_msgs::msg::Imu>();
        imu_msg->linear_acceleration.x = accel_x;
        imu_msg->angular_velocity.z = angular_vel_z;
        imu_pub_->publish(*imu_msg);
    }

    bool waitForMessage(
            const std::chrono::milliseconds timeout = 1s
    ) {
        auto start = std::chrono::steady_clock::now();
        message_received_ = false;
        while (!message_received_) {
            if (std::chrono::steady_clock::now() - start > timeout) {
                return false;
            }
            std::this_thread::sleep_for(10ms);
        }
        return true;
    }

    // Test node, publishers, and subscriber callback variables.
    rclcpp::Node::SharedPtr test_node_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr hall_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    rclcpp::Subscription<utility::msg::FusedSensor>::SharedPtr fused_sub_;
    std::shared_ptr<utility::msg::FusedSensor> last_msg_;
    bool message_received_ = false;

    std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
    std::thread executor_thread_;
};

TEST_F(SensorFilterIntegrationTest, TestZeroInputs) {
    // Test with zero inputs - should result in zero outputs
    publishTestData(0.0f, 0.0f, 0.0f);
    
    ASSERT_TRUE(waitForMessage()) << "Timeout waiting for fused sensor message";
    
    const double tol = 0.01;
    EXPECT_NEAR(last_msg_->velocity, 0.0, tol);
    EXPECT_NEAR(last_msg_->longitudinal_acceleration, 0.0, tol);
    EXPECT_NEAR(last_msg_->angular_velocity, 0.0, tol);
    EXPECT_NEAR(last_msg_->delta, 0.0, tol);
}

// TEST_F(SensorFilterIntegrationTest, TestPositiveAcceleration) {
//     // Test with realistic positive acceleration
//     const float hall_speed = 5.0f;  // 5 m/s
//     const float accel = 2.0f;       // 2 m/s²
//     const float ang_vel = 0.5f;     // 0.5 rad/s
    
//     publishTestData(hall_speed, accel, ang_vel);
    
//     ASSERT_TRUE(waitForMessage()) << "Timeout waiting for fused sensor message";
    
//     const double tol = 0.1;
//     EXPECT_NEAR(last_msg_->velocity, hall_speed, tol);
//     EXPECT_NEAR(last_msg_->longitudinal_acceleration, accel, tol);
//     EXPECT_NEAR(last_msg_->angular_velocity, ang_vel, tol);
//     // Delta calculation depends on your implementation
//     EXPECT_GE(last_msg_->delta, -M_PI/2);
//     EXPECT_LE(last_msg_->delta, M_PI/2);
// }

// TEST_F(SensorFilterIntegrationTest, TestMultipleMessages) {
//     // Test filter behavior with multiple sequential messages
//     const std::vector<float> speeds = {0.0f, 2.0f, 4.0f, 6.0f};
//     const float accel = 1.0f;
//     const float ang_vel = 0.0f;
    
//     for (float speed : speeds) {
//         publishTestData(speed, accel, ang_vel);
//         ASSERT_TRUE(waitForMessage(500ms)) << "Timeout waiting for message at speed " << speed;
        
//         const double tol = 0.1;
//         EXPECT_NEAR(last_msg_->velocity, speed, tol)
//             << "Velocity mismatch at input speed " << speed;
//     }
// }

// TEST_F(SensorFilterIntegrationTest, TestNoiseHandling) {
//     // Test how the filter handles noisy inputs
//     const float base_speed = 5.0f;
//     const float base_accel = 1.0f;
//     const float noise_amplitude = 0.1f;
    
//     for (int i = 0; i < 10; ++i) {
//         float noisy_speed = base_speed + (rand() % 100 - 50) * noise_amplitude / 50.0f;
//         float noisy_accel = base_accel + (rand() % 100 - 50) * noise_amplitude / 50.0f;
        
//         publishTestData(noisy_speed, noisy_accel, 0.0f);
//         ASSERT_TRUE(waitForMessage(500ms));
        
//         // The filtered output should stay close to the base values
//         const double tol = 0.5;  // Larger tolerance for noisy data
//         EXPECT_NEAR(last_msg_->velocity, base_speed, tol);
//         EXPECT_NEAR(last_msg_->longitudinal_acceleration, base_accel, tol);
//     }
// }