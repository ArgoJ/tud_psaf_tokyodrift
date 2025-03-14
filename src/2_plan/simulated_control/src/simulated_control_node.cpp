#include "simulated_control_node.h"
#include "rclcpp/parameter.hpp"



SimulatedControlNode::SimulatedControlNode() : Node("simulated_control") {
    RCLCPP_INFO(this->get_logger(), "Starting simulated control node!");

    this->declare_parameters();
    this->load_config();
    this->log_config();

    this->callback_handle_ = this->add_on_set_parameters_callback(
        std::bind(&SimulatedControlNode::on_parameter_update, this, std::placeholders::_1)
    );

    this->controller = std::make_unique<SimulatedController>(this->config);
    this->is_first_step = true;

    //Subscriber
    this->subTrajectory = this->create_subscription<utility::msg::Trajectory>(
        "tokyodrift/plan/bezier_curve/trajectory", 1,  // plan/transformed_lane, sense/lane_detection/mid_lane
        std::bind(&SimulatedControlNode::trajectory_callback, this, std::placeholders::_1)
    );
    this->subFusedSensor = this->create_subscription<utility::msg::FusedSensor>(
        "fused_sensor", 1,
        std::bind(&SimulatedControlNode::fused_sensor_callback, this, std::placeholders::_1)
    );
    this->subFilteredHall = this->create_subscription<utility::msg::FilteredHall>(
        "filtered_hall", 1,
        std::bind(&SimulatedControlNode::hall_callback, this, std::placeholders::_1)
    );


    // Publisher
    this->pubIntegratedMarker = this->create_publisher<visualization_msgs::msg::Marker>(
        "tokyodrift/plan/bezier_control/integrated_points", 3
    ); 
    this->pubTargetMarker = this->create_publisher<visualization_msgs::msg::Marker>(
        "tokyodrift/plan/bezier_control/target_points", 3
    ); 
    this->pubControl = this->create_publisher<utility::msg::Control>(
        "tokyodrift/plan/control", 1
    );
}

void SimulatedControlNode::trajectory_callback(const utility::msg::Trajectory::SharedPtr trajectory) {
    this->controller->set_control_trajectory(trajectory->points);
    this->is_first_step = true;
    this->simulate_next_step();
    this->is_first_step = false;
    this->reset_timer();
}

void SimulatedControlNode::simulate_next_step() {
    BicycleState current_state;
    if (this->is_first_step) {
        current_state = this->controller->get_state();
    } else {
        START_TIMER("Simulate")
        current_state = this->controller->simulate_next_state();
        STOP_TIMER("Simulate")
    }

    START_TIMER("Get Input")
    auto [input, target] = this->controller->get_input();
    STOP_TIMER("Get Input")

    // RCLCPP_DEBUG(this->get_logger(), "Input Acceleration: %f", input.a);
    if (target.x > 3.0 || std::abs(target.y) > 1.5 || target.x == 0.0) {
        RCLCPP_DEBUG(this->get_logger(), "Target unrealistic value: {%f, %f}", target.x, target.y);
    }

    double control_velocity = current_state.v + 0.5 * input.a * this->config.ts; // current_state.v

    this->control_publisher(control_velocity, input.delta);

    geometry_msgs::msg::Point state_point;
    state_point.x = current_state.x;
    state_point.y = current_state.y;
    publish_marker(
        state_point,
        this->pubIntegratedMarker,
        "map",
        this->get_clock(),
        Color{1.0, 1.0, 0.},
        "integrated_point",
        2,
        0.08,
    	1.0
    );

    publish_marker(
        target,
        this->pubTargetMarker,
        "map",
        this->get_clock(),
        Color{0.1, 0.5, 0.6},
        "target_point",
        3,
        0.08,
    	1.0
    );
}

void SimulatedControlNode::fused_sensor_callback(
    const utility::msg::FusedSensor::SharedPtr fused_sensor_value
) {
    this->controller->set_actual_input(
        {fused_sensor_value->delta, fused_sensor_value->longitudinal_acceleration}
    );
}

void SimulatedControlNode::hall_callback(
    const utility::msg::FilteredHall::SharedPtr hall_ptr
) {
    this->controller->set_actual_velocity(hall_ptr->longitudinal_velocity);
}

void SimulatedControlNode::reset_timer() {
    if (this->timer_) {
        this->timer_->cancel();
    }
    auto steady_clock = this->get_clock();
    this->timer_ = rclcpp::create_timer(
        this,
        steady_clock,
        std::chrono::duration<double>(this->config.ts),
        std::bind(&SimulatedControlNode::simulate_next_step, this)
    );
}

void SimulatedControlNode::control_publisher(const double velocity, const double delta) {
    auto message = utility::msg::Control();

    message.header.stamp = this->get_clock()->now();
    message.longitudinal_control = velocity;
    message.delta = delta;
    message.jump = false;
    this->pubControl -> publish(message);
    // RCLCPP_DEBUG(this->get_logger(), "Published velocity: %f and delta: %f", velocity, delta);
}

void SimulatedControlNode::declare_parameters() {
    this->declare_parameter("wheelbase", 0.258);
    this->declare_parameter("ts", 0.015);
    this->declare_parameter("max_speed", 3.0);
    this->declare_parameter("min_speed", -0.5);
    this->declare_parameter("max_acceleration", 2.0);
    this->declare_parameter("min_acceleration", -3.0);
    this->declare_parameter("max_delta", 0.314159); // 18° * M_PI / 180
    this->declare_parameter("min_delta", -0.314159); // -18° * M_PI / 180
    this->declare_parameter("angle_factor_offset", 0.7);
    this->declare_parameter("lambda_use_sensor", 0.0);
    this->declare_parameter("min_lookahead", 0.6);
    this->declare_parameter("max_lookahead", 1.0);
}

void SimulatedControlNode::load_config() {
    this->get_parameter("wheelbase", this->config.wheelbase);
    this->get_parameter("ts", this->config.ts);
    this->get_parameter("max_speed", this->config.max_speed);
    this->get_parameter("min_speed", this->config.min_speed);
    this->get_parameter("max_acceleration", this->config.max_acceleration);
    this->get_parameter("min_acceleration", this->config.min_acceleration);
    this->get_parameter("max_delta", this->config.max_delta);
    this->get_parameter("min_delta", this->config.min_delta);
    this->get_parameter("angle_factor_offset", this->config.angle_factor_offset);
    this->get_parameter("lambda_use_sensor", this->config.lambda_use_sensor);
    this->get_parameter("min_lookahead", this->config.min_lookahead);
    this->get_parameter("max_lookahead", this->config.max_lookahead);
}

void SimulatedControlNode::log_config() {
    RCLCPP_DEBUG(this->get_logger(), "####################################");
    RCLCPP_DEBUG(this->get_logger(), "Wheelbase: %f", this->config.wheelbase);
    RCLCPP_DEBUG(this->get_logger(), "Sampling Time: %f", this->config.ts);
    RCLCPP_DEBUG(this->get_logger(), "Maximal Speed Forwards: %f", this->config.max_speed);
    RCLCPP_DEBUG(this->get_logger(), "Maximal Speed Backwards: %f", this->config.min_speed);
    RCLCPP_DEBUG(this->get_logger(), "Maximal Acceleration: %f", this->config.max_acceleration);
    RCLCPP_DEBUG(this->get_logger(), "Minimal Acceleration: %f", this->config.min_acceleration);
    RCLCPP_DEBUG(this->get_logger(), "Maximal Delta: %f", this->config.max_delta);
    RCLCPP_DEBUG(this->get_logger(), "Minimal Delta: %f", this->config.min_delta);
    RCLCPP_DEBUG(this->get_logger(), "Angle Factor Offset: %f", this->config.angle_factor_offset);
    RCLCPP_DEBUG(this->get_logger(), "Lambda Use Sensor: %f", this->config.lambda_use_sensor);
    RCLCPP_DEBUG(this->get_logger(), "Minimal Lookahead: %f", this->config.min_lookahead);
    RCLCPP_DEBUG(this->get_logger(), "Maximal Lookahead: %f", this->config.max_lookahead);
}

rcl_interfaces::msg::SetParametersResult SimulatedControlNode::on_parameter_update(
    const std::vector<rclcpp::Parameter> &params
) {
    RCLCPP_DEBUG(this->get_logger(), "Parameter update received");
    // bool ts_change = false;
    for (const auto& param : params) {
        if (param.get_name() == "wheelbase") {
            this->config.wheelbase = param.as_double();
        } else if (param.get_name() == "ts") {
            // this->config.ts = param.as_double();
            // ts_change = true;
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
        } else if (param.get_name() == "angle_factor_offset") {
            this->config.angle_factor_offset = param.as_double();
        } else if (param.get_name() == "lambda_use_sensor") {
            this->config.lambda_use_sensor = param.as_double();
        } else if (param.get_name() == "min_lookahead") {
            this->config.min_lookahead = param.as_double();
        } else if (param.get_name() == "max_lookahead") {
            this->config.max_lookahead = param.as_double();
        }
    }

    this->log_config();

    // if (ts_change) {
    //     this->create_timer();
    // }

    // Indicate success
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    return result;
}


int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SimulatedControlNode>());
    rclcpp::shutdown();
    return 0;
}