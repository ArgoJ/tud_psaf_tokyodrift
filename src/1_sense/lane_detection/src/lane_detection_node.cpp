#include "lane_detection_node.h"


LaneDetectionNode::LaneDetectionNode() : Node("lane_detection") {
    RCLCPP_INFO(this->get_logger(), "Starting lane detection node!");

    this->declare_parameters();
    this->load_config();
    this->log_config();

    this->callback_handle_ = this->add_on_set_parameters_callback(
        std::bind(&LaneDetectionNode::on_parameter_update, this, std::placeholders::_1)
    );

    // Create LaneDetection instance
    this->lane_detection = std::make_unique<LaneDetection>(config);
    this->current_lane_ = CurrentLane::RIGHT;

    // Subscribe to camera topic
    this->subCamera = this->create_subscription<sensor_msgs::msg::Image>(
        "camera/realsense2_camera_node/color/image_raw", 1, std::bind(&LaneDetectionNode::camera_callback, this, std::placeholders::_1)
    );
    this->subCurrentLane = this->create_subscription<std_msgs::msg::Bool>(
        "current_lane", 1, std::bind(&LaneDetectionNode::current_lane_callback, this, std::placeholders::_1)
    );

    // Pubisher of lanes
    this->pubTargetLane = this->create_publisher<utility::msg::Trajectory>("tokyodrift/sense/lane_detection/mid_lane", 1);
    this->pubRightLane = this->create_publisher<utility::msg::Trajectory>("tokyodrift/sense/lane_detection/right_lane", 1);
    this->pubLeftLane = this->create_publisher<utility::msg::Trajectory>("tokyodrift/sense/lane_detection/left_lane", 1);

    // Publisher for debug info
    this->pubDebugImage = this->create_publisher<sensor_msgs::msg::Image>("tokyodrift/sense/debug/lanes", 3);
    this->pubTargetMarkers = this->create_publisher<visualization_msgs::msg::Marker>("tokyodrift/sense/debug/lane_markers", 3);
}


void LaneDetectionNode::camera_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
    // Image processing
    try {
        CurrentLane local_curr_lane = this->current_lane_;
        cv::Mat image_bgr = cv_bridge::toCvCopy(msg, msg->encoding)->image;
        auto [left_points, right_points, mid_points] = this->lane_detection->process_image(image_bgr, local_curr_lane);

        std::vector<cv::Point2f> mid_fit_ego;
        std::vector<cv::Point2f> mid_fit_ego_pub;
        if (this->config.interpolate) {
            START_TIMER("Interpolation")
            auto mid_fit = this->lane_detection->interpolate_line_points(mid_points);
            STOP_TIMER("Interpolation")

            START_TIMER("Px2Ego")
            mid_fit_ego = this->lane_detection->px2ego_m(mid_fit);
            STOP_TIMER("Px2Ego")
            
            mid_fit_ego_pub = sample_lane_points(mid_fit_ego, 50);
        } else {
            START_TIMER("Px2Ego")
            mid_fit_ego = this->lane_detection->px2ego_m(mid_points);
            STOP_TIMER("Px2Ego")

            mid_fit_ego_pub = mid_fit_ego;
        }

        this->publish_lane(mid_fit_ego, local_curr_lane, this->pubTargetLane);
        
        publish_marker_points(
            cv2geo_points(mid_fit_ego_pub), 
            this->pubTargetMarkers, 
            "map", 
            this->get_clock(), 
            Color{1.0, 0.0, 0.0},
            "lane_detection",
            0,
            0.03,
    	    0.7
        );
    } catch (const cv_bridge::Exception& e) {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Exception in camera_callback: %s", e.what());
    }


    if (this->config.pub_images) {
        try {
            std::shared_ptr<cv_bridge::CvImage> cv_image = std::make_shared<cv_bridge::CvImage>();
            cv_image->header.stamp = this->now();
            cv_image->header.frame_id = "debug_frame";
            cv_image->encoding = "bgr8";
            cv_image->image = this->lane_detection->get_debug_image();
            this->pubDebugImage->publish(*cv_image->toImageMsg());
            //std::cout << "published image" << std::endl;
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to publish debug lanes image: %s", e.what());
        }
    }
}


void LaneDetectionNode::current_lane_callback(
        const std_msgs::msg::Bool::SharedPtr msg
) {
    this->current_lane_ = (msg->data) ? CurrentLane::LEFT : CurrentLane::RIGHT;
}


void LaneDetectionNode::publish_lane(
        const std::vector<cv::Point2f>& lane_fit, 
        const uint8_t current_lane,
        const rclcpp::Publisher<utility::msg::Trajectory>::SharedPtr& lane_publisher, 
        const std::string& frame_id
) {
    utility::msg::Trajectory lane_msg;
    lane_msg.header.stamp = this->now();
    lane_msg.header.frame_id = frame_id; 
    lane_msg.points = cv2geo_points(lane_fit);
    lane_msg.current_lane = current_lane;
    lane_publisher->publish(lane_msg);

    RCLCPP_DEBUG(this->get_logger(), "Published lane!");
}

void LaneDetectionNode::declare_parameters() {
    this->declare_parameter("show_images", false);
    this->declare_parameter("pub_images", true);
    this->declare_parameter("interpolate", false);
    this->declare_parameter("nwindows", 30);
    this->declare_parameter("min_pixels", 95);
    this->declare_parameter("x_shift_factor", 0.02f);
    this->declare_parameter("x_shift_next_factor", 0.01f);
    this->declare_parameter("window_width_factor", 0.038f);
    this->declare_parameter("lane_distance_factor", 0.14f);
    this->declare_parameter("safe_lane_distance_factor", 0.6f);
    this->declare_parameter("left_init_factor", 0.43f);
    this->declare_parameter("right_init_factor", 0.57f);
    this->declare_parameter("next_init_factor", 0.012f);
    this->declare_parameter("max_init_offset_factor", 0.4f);
    this->declare_parameter("valid_curve_factor", 0.03f);
    this->declare_parameter("valid_band", 6.f);
    this->declare_parameter("first_iters_each_side", 4);
    this->declare_parameter("iter_each_side", 2);
    this->declare_parameter("binary_adaptive_box", 151);
    this->declare_parameter("binary_adaptive_subtract", -48);
    this->declare_parameter("num_threads", 4);
    this->declare_parameter("camcal_yaml", "src/1_sense/lane_detection/config/camcalout.yaml");
}

void LaneDetectionNode::load_config() {
    this->get_parameter("show_opencv_images", this->config.show_opencv_images);
    this->get_parameter("pub_images", this->config.pub_images);
    this->get_parameter("interpolate", this->config.interpolate);
    this->get_parameter("nwindows", this->config.nwindows);
    this->get_parameter("min_pixels", this->config.min_pixels);
    this->get_parameter("x_shift_factor", this->config.x_shift_factor);
    this->get_parameter("x_shift_next_factor", this->config.x_shift_next_factor);
    this->get_parameter("window_width_factor", this->config.window_width_factor);
    this->get_parameter("lane_distance_factor", this->config.lane_distance_factor);
    this->get_parameter("safe_lane_distance_factor", this->config.safe_lane_distance_factor);
    this->get_parameter("left_init_factor", this->config.left_init_factor);
    this->get_parameter("right_init_factor", this->config.right_init_factor);
    this->get_parameter("next_init_factor", this->config.next_init_factor);
    this->get_parameter("max_init_offset_factor", this->config.max_init_offset_factor);
    this->get_parameter("valid_curve_factor", this->config.valid_curve_factor);
    this->get_parameter("valid_band", this->config.valid_band);
    this->get_parameter("first_iters_each_side", this->config.first_iters_each_side);
    this->get_parameter("iter_each_side", this->config.iter_each_side);
    this->get_parameter("bird_view_width", this->config.bird_view_width);
    this->get_parameter("bird_view_height", this->config.bird_view_height);
    this->get_parameter("binary_adaptive_box", this->config.binary_adaptive_box);
    this->get_parameter("binary_adaptive_subtract", this->config.binary_adaptive_subtract);
    this->get_parameter("num_threads", this->config.num_threads);
    this->get_parameter("camcal_yaml", this->config.camcal_yaml);

    // Rebuild the configuration
    post_init_lane_detection_params(&config);
}


void LaneDetectionNode::log_config() {
    RCLCPP_DEBUG(this->get_logger(), "####################################");
    RCLCPP_DEBUG(this->get_logger(), "Show Images: %d", this->config.show_opencv_images);
    RCLCPP_DEBUG(this->get_logger(), "Publish Images: %d", this->config.pub_images);
    RCLCPP_DEBUG(this->get_logger(), "Interpolate: %d", this->config.interpolate);
    RCLCPP_DEBUG(this->get_logger(), "N Windows: %i", this->config.nwindows);
    RCLCPP_DEBUG(this->get_logger(), "Min Pixels: %i", this->config.min_pixels);
    RCLCPP_DEBUG(this->get_logger(), "X Shift Factor: %f", this->config.x_shift_factor);
    RCLCPP_DEBUG(this->get_logger(), "X Shift Next Factor: %f", this->config.x_shift_next_factor);
    RCLCPP_DEBUG(this->get_logger(), "Window Width Factor: %f", this->config.window_width_factor);
    RCLCPP_DEBUG(this->get_logger(), "Lane Distance Factor: %f", this->config.lane_distance_factor);
    RCLCPP_DEBUG(this->get_logger(), "Lane Distance Factor: %f", this->config.safe_lane_distance_factor);
    RCLCPP_DEBUG(this->get_logger(), "Left Init Factor: %f", this->config.left_init_factor);
    RCLCPP_DEBUG(this->get_logger(), "Right Init Factor: %f", this->config.right_init_factor);
    RCLCPP_DEBUG(this->get_logger(), "Next Init Factor: %f", this->config.next_init_factor);
    RCLCPP_DEBUG(this->get_logger(), "Max Init Offset Factor: %f", this->config.max_init_offset_factor);
    RCLCPP_DEBUG(this->get_logger(), "Valid Curve Factor: %f", this->config.valid_curve_factor);
    RCLCPP_DEBUG(this->get_logger(), "Valid Band: %f", this->config.valid_band);
    RCLCPP_DEBUG(this->get_logger(), "First Iters Each Side: %i", this->config.first_iters_each_side);
    RCLCPP_DEBUG(this->get_logger(), "Iters Each Side: %i", this->config.iter_each_side);
    RCLCPP_DEBUG(this->get_logger(), "Bird View Width: %i", this->config.bird_view_width);
    RCLCPP_DEBUG(this->get_logger(), "Bird View Height: %i", this->config.bird_view_height);
    RCLCPP_DEBUG(this->get_logger(), "m to Pixel: %f", this->config.px2m);
    RCLCPP_DEBUG(this->get_logger(), "x0 in m: %f", this->config.x0_m);
    RCLCPP_DEBUG(this->get_logger(), "Binary Adaptive Box Size: %i", this->config.binary_adaptive_box);
    RCLCPP_DEBUG(this->get_logger(), "Binary Mean Subtraction: %i", this->config.binary_adaptive_subtract);
    RCLCPP_DEBUG(this->get_logger(), "Window Width: %i", this->config.window_width);
    RCLCPP_DEBUG(this->get_logger(), "Window Height: %i", this->config.window_height);
    RCLCPP_DEBUG(this->get_logger(), "X Shift: %i", this->config.x_shift);
    RCLCPP_DEBUG(this->get_logger(), "Lane Distance: %i", this->config.lane_distance);
    RCLCPP_DEBUG(this->get_logger(), "Safe Lane Distance: %i", this->config.safe_lane_distance);
    RCLCPP_DEBUG(this->get_logger(), "Maximal Number of Threads: %i", this->config.num_threads);
}


rcl_interfaces::msg::SetParametersResult LaneDetectionNode::on_parameter_update(
    const std::vector<rclcpp::Parameter>& params)
{
    RCLCPP_DEBUG(this->get_logger(), "Parameter update received");
    for (const auto& param : params) {
        if (param.get_name() == "show_opencv_images") {
            this->config.show_opencv_images = param.as_bool();
        } else if (param.get_name() == "pub_images") {
            this->config.pub_images = param.as_bool();
        } else if (param.get_name() == "interpolate") {
            this->config.interpolate = param.as_bool();
        } else if (param.get_name() == "nwindows") {
            this->config.nwindows = param.as_int();
        } else if (param.get_name() == "min_pixels") {
            this->config.min_pixels = param.as_int();
        } else if (param.get_name() == "x_shift_factor") {
            this->config.x_shift_factor = param.as_double();
        } else if (param.get_name() == "x_shift_next_factor") {
            this->config.x_shift_next_factor = param.as_double();
        } else if (param.get_name() == "window_width_factor") {
            this->config.window_width_factor = param.as_double();
        } else if (param.get_name() == "lane_distance_factor") {
            this->config.lane_distance_factor = param.as_double();
        } else if (param.get_name() == "safe_lane_distance_factor") {
            this->config.safe_lane_distance_factor = param.as_double();
        } else if (param.get_name() == "left_init_factor") {
            this->config.left_init_factor = param.as_double();
        } else if (param.get_name() == "right_init_factor") {
            this->config.right_init_factor = param.as_double();
        } else if (param.get_name() == "next_init_factor") {
            this->config.next_init_factor= param.as_double();
        } else if (param.get_name() == "max_init_offset_factor") {
            this->config.max_init_offset_factor= param.as_double();
        } else if (param.get_name() == "valid_curve_factor") {
            this->config.valid_curve_factor = param.as_double();
        } else if (param.get_name() == "valid_band") {
            this->config.valid_band = param.as_double();
        } else if (param.get_name() == "first_iters_each_side") {
            this->config.first_iters_each_side = param.as_int();
        } else if (param.get_name() == "iter_each_side") {
            this->config.iter_each_side = param.as_int();
        } else if (param.get_name() == "binary_adaptive_box") {
            this->config.binary_adaptive_box = param.as_int();
        } else if (param.get_name() == "binary_adaptive_subtract") {
            this->config.binary_adaptive_subtract = param.as_int();
        } else if (param.get_name() == "camcal_yaml") {
            this->config.camcal_yaml = param.as_string();
        } else if (param.get_name() == "num_threads") {
            this->config.num_threads = param.as_int();
        }
    }

    post_init_lane_detection_params(&config);
    this->log_config();

    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    return result;
}


int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LaneDetectionNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}