#include <cstdio>
#include "lane_transform_node.h"


LaneTransformNode::LaneTransformNode() : Node("lane_transform") {
    this->declare_parameters();
    this->load_config();
    this->load_config();

    this->callback_handle_ = this->add_on_set_parameters_callback(
        std::bind(&LaneTransformNode::on_parameter_update, this, std::placeholders::_1)
    );

    // Initialize the integrator and the initial state
    this->integrator_ = std::make_unique<Integrator>(this->config.wheelbase);

    this->last_delta = 0.0;
    this->last_velocity = 0.0;
    this->last_acceleration = 0.0;
    this->integrator_running_ = false;
    this->block_velocity_ = false;

    // Subscriptions
    this->subCamera = this->create_subscription<sensor_msgs::msg::Image>(
        "camera/realsense2_camera_node/color/image_raw", 1,
        std::bind(&LaneTransformNode::camera_callback, this, std::placeholders::_1)
    );
    this->subTrajectory = this->create_subscription<utility::msg::Trajectory>(
        "tokyodrift/sense/lane_detection/mid_lane", 1,
        std::bind(&LaneTransformNode::trajectory_callback, this, std::placeholders::_1)
    );
    this->subFusedSensor = this->create_subscription<utility::msg::FusedSensor>(
        "fused_sensor", 1,
        std::bind(&LaneTransformNode::fused_sensor_callback, this, std::placeholders::_1)
    );
    this->subFilteredHall = this->create_subscription<utility::msg::FilteredHall>(
        "filtered_hall", 1,
        std::bind(&LaneTransformNode::hall_callback, this, std::placeholders::_1)
    );

    // Publisher
    this->pubTrajectory = this->create_publisher<utility::msg::Trajectory>(
        "tokyodrift/plan/transformed_lane", 1
    );
    this->pubTransformedMarkers = this->create_publisher<visualization_msgs::msg::Marker>(
        "tokyodrift/plan/transformed_markers", 3
    );
    this->pubIntegratedState = this->create_publisher<visualization_msgs::msg::Marker>(
        "tokyodrift/plan/integrated_state", 3
    );
}


void LaneTransformNode::set_integrator_running() {
    if (!this->integrator_running_) {
        std::lock_guard<std::mutex> lock(state_mutex);
        this->time_ = this->steady_clock_.now();
        this->integrator_running_ = true;
        this->fist_integration = true;
        this->current_state_ = {0.0, 0.0, 0.0, 0.0};
    }
}


std::vector<geometry_msgs::msg::Point> LaneTransformNode::transform(
        const BicycleState& state,
        const std::vector<geometry_msgs::msg::Point>& trajectory
) {
    std::vector<geometry_msgs::msg::Point> transformed_lane;
    for (size_t i = 0; i < trajectory.size(); i ++) {
        geometry_msgs::msg::Point point = trajectory[i];
        geometry_msgs::msg::Point transformed_point;

        // 2D-Helmert-Transformation
        transformed_point.x = - state.x + point.x * cos(state.omega) + point.y * sin(state.omega);
        transformed_point.y = - state.y - point.x * sin(state.omega) + point.y * cos(state.omega);
        transformed_lane.push_back(transformed_point);
    }
    return transformed_lane;
}


geometry_msgs::msg::Point LaneTransformNode::state2point(const BicycleState& state) {
    geometry_msgs::msg::Point point;
    point.x = state.x;
    point.y = state.y;
    return point;
}


void LaneTransformNode::camera_callback(
    const sensor_msgs::msg::Image::SharedPtr msg)
{
    this->set_integrator_running();
}


void LaneTransformNode::trajectory_callback(
        const utility::msg::Trajectory::SharedPtr traj_msg
) {
    std::vector<geometry_msgs::msg::Point> trajectory;
    uint8_t current_lane;
    if (this->integrator_running_) {
        if (traj_msg->points.empty()) {
            trajectory = this->previous_lane_;
            current_lane = this->previous_current_lane;
        } else {
            trajectory = traj_msg->points;
            current_lane = traj_msg->current_lane;
            this->integrator_running_ = false;
            this->previous_lane_ = trajectory;
            this->previous_current_lane = current_lane;
        }
        
        double ts = (this->steady_clock_.now() - this->time_).seconds();
        ts = std::clamp(ts, 1e-5, this->config.ts);

        this->simulate_next_step(ts);
        BicycleState local_state = this->current_state_;

        START_TIMER("Transform")
        auto transformed_trajectory = this->transform(local_state, trajectory);
        STOP_TIMER("Transform")

        this->publish_lane(
            transformed_trajectory,
            current_lane,
            this->pubTrajectory
        );

        publish_marker_points(
            transformed_trajectory,
            this->pubTransformedMarkers,
            "map",
            this->get_clock(),
            Color{0.8, 0.32, 0.2},
            "transformed_lane",
            0,
            0.03,
            0.7
        );
    }
}


void LaneTransformNode::fused_sensor_callback(
        const utility::msg::FusedSensor::SharedPtr fused_sensor_ptr
) {
    double ts;
    if (this->fist_integration) {
        rclcpp::Time end_time = this->steady_clock_.now();
        ts = (end_time - this->time_).seconds();
        ts = std::clamp(ts, 1e-5, this->config.ts);

        this->time_ = end_time;
        this->fist_integration = false;
    } else {
        ts = this->config.ts;
    }

    this->simulate_next_step(ts);
    this->last_delta = std::clamp(fused_sensor_ptr->delta, this->config.min_delta, this->config.max_delta);
    this->last_acceleration = std::clamp(fused_sensor_ptr->longitudinal_acceleration, this->config.min_acceleration, this->config.max_acceleration);
    this->block_velocity_ = false;
}


void LaneTransformNode::hall_callback(
        const utility::msg::FilteredHall::SharedPtr hall_ptr
) {
    if (this->block_velocity_) {
        return;
    }
    this->last_velocity = std::clamp(hall_ptr->longitudinal_velocity, this->config.min_speed, this->config.max_speed);
    this->block_velocity_ = true;

}


void LaneTransformNode::simulate_next_step(double ts) {
    std::lock_guard<std::mutex> lock(state_mutex);
    
    if (!std::isfinite(this->last_velocity) || 
        !std::isfinite(this->last_delta) || 
        !std::isfinite(this->last_acceleration)) {
        RCLCPP_WARN(this->get_logger(), "Invalid input values detected!");
        return;
    }

    // integrate model
    this->current_state_.v = this->last_velocity;
    START_TIMER("Integrate")
    BicycleState next_state = this->integrator_->integrate_bicycle_model(
        this->current_state_,
        {this->last_delta, this->last_acceleration},
        ts
    );
    STOP_TIMER("Integrate")

    this->current_state_ = next_state;

    // publish marker
    publish_marker(
        this->state2point(next_state),
        this->pubIntegratedState,
        "map",
        this->get_clock(),
        Color{0.9, 0.32, 0.7},
        "integrated_state",
        0,
        0.08
    );
}


void LaneTransformNode::publish_lane(
        const std::vector<geometry_msgs::msg::Point>& transformed_lane, 
        const uint8_t current_lane,
        const rclcpp::Publisher<utility::msg::Trajectory>::SharedPtr& lane_publisher, 
        const std::string& frame_id
) {
    utility::msg::Trajectory lane_msg;
    lane_msg.header.stamp = this->now();
    lane_msg.header.frame_id = frame_id; 
    lane_msg.points = transformed_lane;
    lane_msg.current_lane = current_lane;
    lane_publisher->publish(lane_msg);
    RCLCPP_DEBUG(this->get_logger(), "Published transformed lane!");
}


rcl_interfaces::msg::SetParametersResult LaneTransformNode::on_parameter_update(
        const std::vector<rclcpp::Parameter> &params
) {
    RCLCPP_DEBUG(this->get_logger(), "Parameter update received");
    for (const auto& param : params) {
        if (param.get_name() == "wheelbase") {
            this->config.wheelbase = param.as_double();
        } else if (param.get_name() == "max_speed") {
            this->config.max_speed = param.as_double();
        } else if (param.get_name() == "min_speed") {
            this->config.min_speed = param.as_double();
        } else if (param.get_name() == "max_acceleration") {
            this->config.max_acceleration = param.as_double();
        } else if (param.get_name() == "min_acceleration") {
            this->config.min_acceleration = param.as_double();
        } else if (param.get_name() == "max_delta") {
            this->config.max_delta = param.as_double();
        } else if (param.get_name() == "min_delta") {
            this->config.min_delta = param.as_double();
        } else if (param.get_name() == "ts") {
            this->config.ts = param.as_double();
        }
    }
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    return result;
}

void LaneTransformNode::declare_parameters() {
    this->declare_parameter("wheelbase", 0.258);
    this->declare_parameter("max_speed", 3.0);
    this->declare_parameter("min_speed", -0.5);
    this->declare_parameter("max_acceleration", 6.0);
    this->declare_parameter("min_acceleration", -10.0);
    this->declare_parameter("max_delta", 0.314159); // 18° * M_PI / 180
    this->declare_parameter("min_delta", -0.314159); // -18° * M_PI / 180
    this->declare_parameter("ts", 0.015);
}

void LaneTransformNode::load_config() {
    this->get_parameter("wheelbase", this->config.wheelbase);
    this->get_parameter("max_speed", this->config.max_speed);
    this->get_parameter("min_speed", this->config.min_speed);
    this->get_parameter("max_acceleration", this->config.max_acceleration);
    this->get_parameter("min_acceleration", this->config.min_acceleration);
    this->get_parameter("max_delta", this->config.max_delta);
    this->get_parameter("min_delta", this->config.min_delta);
    this->get_parameter("ts", this->config.ts);
}

void LaneTransformNode::log_config() {
    RCLCPP_DEBUG(this->get_logger(), "####################################");
    RCLCPP_DEBUG(this->get_logger(), "Wheelbase: %f", this->config.wheelbase);
    RCLCPP_DEBUG(this->get_logger(), "Maximal Speed Forwards: %f", this->config.max_speed);
    RCLCPP_DEBUG(this->get_logger(), "Maximal Speed Backwards: %f", this->config.min_speed);
    RCLCPP_DEBUG(this->get_logger(), "Maximal Acceleration: %f", this->config.max_acceleration);
    RCLCPP_DEBUG(this->get_logger(), "Minimal Acceleration: %f", this->config.min_acceleration);
    RCLCPP_DEBUG(this->get_logger(), "Maximal Delta: %f", this->config.max_delta);
    RCLCPP_DEBUG(this->get_logger(), "Minimal Delta: %f", this->config.min_delta);
    RCLCPP_DEBUG(this->get_logger(), "Sampling Time IMU: %f", this->config.ts);
}


int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LaneTransformNode>());
    rclcpp::shutdown();
    return 0;
}
