#ifndef TOKYODRIFT_BUTTON_LAUNCH_H
#define TOKYODRIFT_BUTTON_LAUNCH_H

#include <rclcpp/rclcpp.hpp>
#include <cstdint>
#include <cstdlib>
#include <cstdio>
#include <signal.h>
#include <sys/wait.h>
#include <sys/types.h>
#include <mutex>
#include <iostream>

#include "std_msgs/msg/int8.hpp"
#include "psaf_ucbridge_msgs/msg/pbs.hpp"
#include "button_launch_config.hpp"


class ButtonLaunchNode : public rclcpp::Node {
private:
    ButtonLaunchParams config;
    int8_t button_a_state{0}, button_b_state{0}, button_c_state{0};
    pid_t a_pid{-1}, b_pid{-1}, c_pid{-1};
    std::mutex pid_mutex;

    OnSetParametersCallbackHandle::SharedPtr callback_handle_;

    // Subscriber
    rclcpp::Subscription<psaf_ucbridge_msgs::msg::Pbs>::SharedPtr subButton;
    
public:
    ButtonLaunchNode();
    ~ButtonLaunchNode();


private:
    void declare_parameters();
    void load_config();
    void log_config();

    rcl_interfaces::msg::SetParametersResult on_parameter_update(
        const std::vector<rclcpp::Parameter>& params
    );

    bool stop_current_launch(
        pid_t pid
    );

    bool start_launch(
        int8_t num
    );

    pid_t launch_process(
        const std::string &command
    );

    void button_callback(
        const psaf_ucbridge_msgs::msg::Pbs::SharedPtr msg
    );

    void set_states(
        const int8_t a,
        const int8_t b,
        const int8_t c
    );
};


#endif //TOKYODRIFT_BUTTON_LAUNCH_H
