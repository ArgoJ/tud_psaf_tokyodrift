#ifndef IMAGE_PUBLISHER_TEST_H
#define IMAGE_PUBLISHER_TEST_H

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <filesystem>
#include <string>
#include <vector>
#include <algorithm>


class ImagePublisherNode : public rclcpp::Node {
private:
    std::string image_dir_;
    std::vector<std::string> image_files_;
    size_t current_image_index_;
    uint32_t ts_;
    OnSetParametersCallbackHandle::SharedPtr callback_handle_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher_;
    rclcpp::TimerBase::SharedPtr publish_timer_;
    
public:
    ImagePublisherNode();
    
private:
    void publish_images();
    void load_params();
    void create_timer();
    void log_params();
    void get_all_images();
    
    rcl_interfaces::msg::SetParametersResult on_parameter_update(
        const std::vector<rclcpp::Parameter>& params
    );
};

#endif //IMAGE_PUBLISHER_TEST_H
