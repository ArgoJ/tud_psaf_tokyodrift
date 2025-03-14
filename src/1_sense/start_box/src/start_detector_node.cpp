#include "../include/start_detector_node.h"



StartDetectorNode::StartDetectorNode() : Node("start_box") {
    RCLCPP_INFO(this->get_logger(),"Start detector node created.");
    //Declare the default parameter values
    
    this->declare_parameter("window_size", 50);
    this->declare_parameter("pixels_threshold", 70);
    this->declare_parameter("squares_threshold", 50);
    this->declare_parameter("step_size", 10);
    this->declare_parameter("total_pixels", 50 * 50);
    std::cout << "Parameters declared. \n";
    this->load_config();
    this->log_config();
    
    std::cout << "Config has been loaded and logged. \n";

    this->callback_handle_ = this->add_on_set_parameters_callback(
        std::bind(&StartDetectorNode::on_parameter_update, this, std::placeholders::_1)
    );

    // Subscribe to camera topic
    this->subCamera = this->create_subscription<sensor_msgs::msg::Image>(
        "camera/realsense2_camera_node/color/image_raw", 1, std::bind(&StartDetectorNode::camera_callback, this, std::placeholders::_1)
    );

    //Publisher
    this->pubStartSequenceFinished = this -> create_publisher<utility::msg::Startbox>(
        "tokyodrift/sense/startbox", 1);
}



void StartDetectorNode::publish_start_sequence_finished(bool finished) {
    auto message = utility::msg::Startbox();
    message.start_sequence_finished = finished;
    // std::cout << "Message contains: " << message.start_sequence_finished << std::endl;
    this->pubStartSequenceFinished->publish(message);
}

void StartDetectorNode::camera_callback(const sensor_msgs::msg::Image::SharedPtr image_message) {
    std::cout << "start detector node camera callback. \n";
    //as soon as we start driving, we don't need to look for the stop sign anymore -> early return
    if (is_driving){
        return;
    }

    // Image processing (copy to binary image to boolean)
    try {
        cv::Mat image_bgr = cv_bridge::toCvCopy(image_message, "bgr8")->image;
        cv::Mat binary_image = start_detector.convert_image(image_bgr);
        stop_sign_visible =  start_detector.is_stop_sign_still_visible(binary_image, this->config.window_size, this->config.pixels_threshold, this->config.squares_threshold, this->config.step_size);

        if (stop_sign_visible){//if visible
            std::cout << "visible \n";
            start_time_initialized = false;
            if (!initialization_start_time_initialized){//and not yet initialized
                initialization_start_time_initialized = true;
                initialization_start_time = this -> now();
            }//start initialization
            rclcpp::Time current_time = this -> now();
            //if initialized and we waited long enough
            if(initialization_start_time_initialized && current_time - initialization_start_time > INITIALIZATION_WAITING_DURATION){
                stop_sign_initialized = true;
            }//-> sign is initialized and we start waiting for it to disappear
            publish_start_sequence_finished(false); //hence: start sequence is not yet finished
        }
        else{ //not visible
            if (!start_time_initialized && stop_sign_initialized){ //if it's the start of a period in which we don't see it
                start_time = this -> now();
                start_time_initialized = true;
            }//start the period
            initialization_start_time = this -> now();

            //if stop sign hasn't been visible for long enough
            rclcpp::Time current_time = this -> now();
            if (start_time_initialized && current_time - start_time > WAITING_DURATION){
                is_driving = true;
                std::cout << "started driving" << std::endl;
                publish_start_sequence_finished(true);
            }//-> we can be sure that it's gone and won't come back i.e. we can start driving
        }

    //exception handling
    } catch (const cv_bridge::Exception& e) {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Exception in camera_callback (of StartDetectorNode): %s", e.what());
    }
}


int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<StartDetectorNode>());
    rclcpp::shutdown();
    return 0;
}

void StartDetectorNode::load_config(){
    this->get_parameter("window_size", this->config.window_size);
    this->get_parameter("pixels_threshold", this->config.pixels_threshold);
    this->get_parameter("squares_threshold", this->config.squares_threshold);
    this->get_parameter("step_size", this->config.step_size);
    this->get_parameter("total_pixels", this->config.total_pixels);
}

void StartDetectorNode::log_config(){
    RCLCPP_DEBUG(this->get_logger(), "####################################");
    RCLCPP_DEBUG(this->get_logger(), "Window's side length: %d", this->config.window_size);
    RCLCPP_DEBUG(this->get_logger(), "Pixels threshold: %d", this->config.pixels_threshold);
    RCLCPP_DEBUG(this->get_logger(), "Squares threshold: %d", this->config.squares_threshold);
    RCLCPP_DEBUG(this->get_logger(), "Step size: %d", this->config.step_size);
    RCLCPP_DEBUG(this->get_logger(), "Total number of pixels: %ld", this->config.total_pixels);
}

rcl_interfaces::msg::SetParametersResult StartDetectorNode::on_parameter_update(
        const std::vector<rclcpp::Parameter>& params){
    for (const auto& param : params){
        if (param.get_name() == "window_size"){
            this->config.window_size = param.as_int();
            this->config.total_pixels = this->config.window_size * this->config.window_size;
        } else if (param.get_name() == "pixels_threshold"){
            this->config.pixels_threshold = param.as_int();
        } else if (param.get_name() == "squares_threshold"){
            this->config.squares_threshold = param.as_int();
        } else if (param.get_name() == "step_size"){
            this->config.step_size = param.as_int();
        } else if (param.get_name() == "total_pixels"){
            this->config.total_pixels = param.as_int();
        }
    }

    this->log_config();

    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    return result;
}