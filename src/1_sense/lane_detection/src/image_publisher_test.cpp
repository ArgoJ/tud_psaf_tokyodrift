#include "image_publisher_test.h"



ImagePublisherNode::ImagePublisherNode(): Node("image_publisher")
{
    this->declare_parameter("image_dir", "/home/argo/programmier_stuff/UNI/psaf/recordings/Selbstfahrende_Streckenaufnahmen/inner_lane/");
    this->declare_parameter("ts", 1000);

    this->load_params();
    this->log_params();
    this->get_all_images();

    this->callback_handle_ = this->add_on_set_parameters_callback(
        std::bind(&ImagePublisherNode::on_parameter_update, this, std::placeholders::_1)
    );

    // Initialize the publisher
    image_publisher_ = this->create_publisher<sensor_msgs::msg::Image>(
        "camera/realsense2_camera_node/color/image_raw", 10
    );
    this->create_timer();
}


void ImagePublisherNode::publish_images() {
    if (image_files_.empty()) {
        RCLCPP_WARN(this->get_logger(), "No images to publish.");
        return;
    }

    // Load the current image
    const std::string& image_path = image_files_[current_image_index_];
    cv::Mat image = cv::imread(image_path, cv::IMREAD_COLOR);
    if (image.empty()) {
        RCLCPP_WARN(this->get_logger(), "Failed to load image: %s", image_path.c_str());
        return;
    }

    // Convert the OpenCV image to a ROS message
    auto message = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", image).toImageMsg();

    // Publish the image
    image_publisher_->publish(*message);

    RCLCPP_DEBUG(this->get_logger(), "Published image: %s", image_path.c_str());

    // Move to the next image, loop back to the start if necessary
    current_image_index_ = (current_image_index_ + 1) % image_files_.size();
}

void ImagePublisherNode::load_params() {
    this->get_parameter("image_dir", this->image_dir_);
    this->get_parameter("ts", this->ts_);
}

void ImagePublisherNode::log_params() {
    RCLCPP_DEBUG(this->get_logger(), "Image Dircetory: %s", this->image_dir_.c_str());
    RCLCPP_DEBUG(this->get_logger(), "Sampling Time: %i", this->ts_);
}

void ImagePublisherNode::create_timer() {
    publish_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(this->ts_),
        std::bind(&ImagePublisherNode::publish_images, this)
    );
}

void ImagePublisherNode::get_all_images() {
    RCLCPP_INFO(this->get_logger(), "Using path: %s", this->image_dir_.c_str());
    if (!std::filesystem::exists(this->image_dir_) || !std::filesystem::is_directory(this->image_dir_)) {
        RCLCPP_ERROR(this->get_logger(), "Invalid path provided: %s", this->image_dir_.c_str());
    }

    for (const auto &entry : std::filesystem::directory_iterator(this->image_dir_))
    {
        if (entry.is_regular_file())
        {
            std::string extension = entry.path().extension().string();
            if (extension == ".jpg" || extension == ".jpeg" || extension == ".png") {
                this->image_files_.push_back(entry.path().string());
            }
        }
    }

    std::sort(this->image_files_.begin(), this->image_files_.end());
    if (this->image_files_.empty()) {
        RCLCPP_WARN(this->get_logger(), "No .jpg files found in the directory: %s", this->image_dir_.c_str());
    }
}

rcl_interfaces::msg::SetParametersResult ImagePublisherNode::on_parameter_update(
        const std::vector<rclcpp::Parameter>& params
) {
    RCLCPP_DEBUG(this->get_logger(), "Parameter update received");
    for (const auto& param : params) {
        if (param.get_name() == "image_dir") {
            this->image_dir_ = param.as_string();
            this->get_all_images();
        } else if (param.get_name() == "ts") {
            this->ts_ = param.as_int();
        }
    }
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    return result;
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ImagePublisherNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
