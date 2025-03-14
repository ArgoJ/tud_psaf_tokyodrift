#include "bezier_curve_node.h"
#include "rclcpp/parameter.hpp"



BezierCurveNode::BezierCurveNode() : Node("bezier_curve") {
    RCLCPP_INFO(this->get_logger(), "Starting bezier curve node!");

    this->declare_parameters();
    this->load_config();
    this->log_config();

    this->callback_handle_ = this->add_on_set_parameters_callback(
        std::bind(&BezierCurveNode::on_parameter_update, this, std::placeholders::_1)
    );

    //Subscriber
    this->subTrajectory = this->create_subscription<utility::msg::Trajectory>(
        "tokyodrift/plan/transformed_lane", 1, // plan/transformed_lane, sense/lane_detection/mid_lane
        std::bind(&BezierCurveNode::trajectory_callback, this, std::placeholders::_1)
    );

    // Publisher
    // Bezier Points
    this->pubBezierCurve = this->create_publisher<utility::msg::Trajectory>(
        "tokyodrift/plan/bezier_curve/trajectory", 1
    ); 
    this->pubTargetMarkerLine = this->create_publisher<visualization_msgs::msg::Marker>(
        "tokyodrift/plan/bezier_curve/trajectory_marker", 3
    ); 
    this->pubBezierControlMarker = this->create_publisher<visualization_msgs::msg::Marker>(
        "tokyodrift/plan/bezier_curve/control_points_marker", 3
    );
}

void BezierCurveNode::trajectory_callback(const utility::msg::Trajectory::SharedPtr trajectory) {   
    if (trajectory->points.empty()) {
        RCLCPP_WARN(this->get_logger(), "Trajectory is empty.");
        return;
    }
    START_TIMER("Get Control Trajectory")
    auto [bezier_trajectory, control_points] = get_cubic_bezier_complete(
        trajectory->points, this->config.bezier_distance, this->config.control_point_frac, this->config.ts_bezier
    );
    STOP_TIMER("Get Control Trajectory")

    this->publish_bezier_curve(bezier_trajectory, trajectory->current_lane);
    
    publish_marker_points(
        bezier_trajectory,
        this->pubTargetMarkerLine,
        "map",
        this->get_clock(),
        Color{0.0, 1.0, 0.0},
        "bezier_control",
        0,
        0.03,
    	0.7,
        visualization_msgs::msg::Marker::LINE_STRIP
    );

    publish_marker_points(
        control_points,
        this->pubBezierControlMarker,
        "map",
        this->get_clock(),
        Color{1.0, 1.0, 1.0},
        "bezier_control_points",
        1,
        0.03,
    	1.0,
        visualization_msgs::msg::Marker::SPHERE_LIST
    );
}

void BezierCurveNode::publish_bezier_curve(
        const std::vector<geometry_msgs::msg::Point>& trajectory,
        const uint8_t current_lane
) {
    utility::msg::Trajectory lane_msg;
    lane_msg.header.stamp = this->now();
    lane_msg.header.frame_id = "bezier_curve_frame"; 
    lane_msg.points = trajectory;
    lane_msg.current_lane = current_lane;
    this->pubBezierCurve->publish(lane_msg);
}

void BezierCurveNode::declare_parameters() {
    this->declare_parameter("wheelbase", 0.258);
    this->declare_parameter("ts_bezier", 0.1);
    this->declare_parameter("bezier_distance", 1.3);
    this->declare_parameter("control_point_frac", 0.3);
}

void BezierCurveNode::load_config() {
    this->get_parameter("ts_bezier", this->config.ts_bezier);
    this->get_parameter("bezier_distance", this->config.bezier_distance);
    this->get_parameter("control_point_frac", this->config.control_point_frac);
}

void BezierCurveNode::log_config() {
    RCLCPP_DEBUG(this->get_logger(), "####################################");
    RCLCPP_DEBUG(this->get_logger(), "Sampling Time Bezier: %f", this->config.ts_bezier);
    RCLCPP_DEBUG(this->get_logger(), "Bezier Adjusted Distance: %f", this->config.bezier_distance);
    RCLCPP_DEBUG(this->get_logger(), "Control Point Fraction: %f", this->config.control_point_frac);
}

rcl_interfaces::msg::SetParametersResult BezierCurveNode::on_parameter_update(
    const std::vector<rclcpp::Parameter> &params
) {
    RCLCPP_DEBUG(this->get_logger(), "Parameter update received");
    for (const auto& param : params) {
        if (param.get_name() == "ts_bezier") {
            this->config.ts_bezier = param.as_double();
        } else if (param.get_name() == "bezier_distance") {
            this->config.bezier_distance = param.as_double();
        } else if (param.get_name() == "control_point_frac") {
            this->config.control_point_frac = param.as_double();
        } 
    }

    this->log_config();

    // Indicate success
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    return result;
}


int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BezierCurveNode>());
    rclcpp::shutdown();
    return 0;
}