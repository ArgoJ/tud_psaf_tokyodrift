#include "sensor_filter_node.h"


SensorFilterNode::SensorFilterNode() : Node("sensor_filter") {
    this->declare_parameters();
    this->load_config();
    this->log_config();

    this->kalman_filter_ = std::make_unique<KalmanStateEstimator>(
        0.015, // Initial dt (adjust if needed)
        0.0,   // Initial velocity
        0.0,   // Initial acceleration
        0.0    // Initial delta
    );

    // Subscriptions
    this->subHallSensor = this->create_subscription<std_msgs::msg::Float32>(
        "dt8_data", 1,
        std::bind(&SensorFilterNode::hall_callback, this, std::placeholders::_1)
    );
    this->subIMU = this->create_subscription<sensor_msgs::msg::Imu>(
        "imu_data", 1,
        std::bind(&SensorFilterNode::imu_callback, this, std::placeholders::_1)
    );
    this->subStartbox = this->create_subscription<utility::msg::Startbox>(
        "tokyodrift/sense/startbox", 1,
        std::bind(&SensorFilterNode::startbox_callback, this, std::placeholders::_1)
    );

    // Publisher
    this->pubFilteredIMU = this->create_publisher<utility::msg::FusedSensor>(
        "fused_sensor", 1
    );
    this->pubFilteredHall = this->create_publisher<utility::msg::FilteredHall>(
        "filtered_hall", 1
    );
}


void SensorFilterNode::hall_callback(
        const std_msgs::msg::Float32::SharedPtr hall_ptr
) {
    this->is_driving = true;

    double hall_value = static_cast<double>(hall_ptr->data);
    if (!this->hall_filter_) {
        this->hall_filter_ = std::make_unique<LowpassFilter>(this->config.tau_hall, hall_value);
        return;
    }

    START_TIMER("Lowpass Filter")
    double filtered_hall_value = this->hall_filter_->update(hall_value, hall_value);
    STOP_TIMER("Lowpass Filter")

    double filtered_velocity = this->config.wheelradius * M_PI / filtered_hall_value;
    filtered_velocity = std::clamp(filtered_velocity, this->config.min_speed, this->config.max_speed);

    this->publish_hall(filtered_hall_value, filtered_velocity);

    double velocity = this->config.wheelradius * M_PI / hall_value;
    velocity = std::clamp(velocity, this->config.min_speed, this->config.max_speed);
    
    START_TIMER("Hall Update")
    this->kalman_filter_->update_hall(velocity);
    STOP_TIMER("Hall Update")
}


void SensorFilterNode::imu_callback(
        const sensor_msgs::msg::Imu::SharedPtr imu_ptr
) {
    double long_acc_value = static_cast<double>(imu_ptr->linear_acceleration.x);
    double ang_vel_value = static_cast<double>(imu_ptr->angular_velocity.z);

    if (!is_driving) {
        this->sum_lin_acc += long_acc_value;
        this->sum_ang_vel += ang_vel_value;
        this->imu_count ++;

    } else {
        long_acc_value -= this->config.x_linear_acceleration_offset;
        ang_vel_value -= this->config.z_angular_velocity_offset;

        START_TIMER("Predict")
        this->kalman_filter_->predict();
        STOP_TIMER("Predict")

        START_TIMER("IMU Update")
        this->kalman_filter_->update_imu(long_acc_value, ang_vel_value);
        STOP_TIMER("IMU Update")

        double filtered_long_acc = this->kalman_filter_->get_acceleration();
        double filtered_ang_vel = this->kalman_filter_->get_ang_vel();
        double filtered_velocity = this->kalman_filter_->get_velocity();

        START_TIMER("Delta Calculation")
        double delta = this->calculate_delta(filtered_velocity, filtered_ang_vel);
        STOP_TIMER("Delta Calculation") 

        this->publish_imu(filtered_velocity, filtered_long_acc, filtered_ang_vel, delta);
    }
}

void SensorFilterNode::startbox_callback(
        const utility::msg::Startbox::SharedPtr startbox_ptr
) {
    if (startbox_ptr->start_sequence_finished && !is_driving) {
        this->is_driving = true;

        if (this->imu_count > 0) {
            this->config.x_linear_acceleration_offset = this->sum_lin_acc / this->imu_count;
            this->config.z_angular_velocity_offset = this->sum_ang_vel / this->imu_count;

            this->set_parameter(rclcpp::Parameter("x_linear_acceleration_offset", this->config.x_linear_acceleration_offset));
            this->set_parameter(rclcpp::Parameter("z_angular_velocity_offset", this->config.z_angular_velocity_offset));
        } else {
            RCLCPP_WARN(this->get_logger(), "IMU count is zero, skipping offset update.");
        }
    }
}

double SensorFilterNode::calculate_delta(
    double velocity,
    double angular_velocity)
{
    double delta = std::atan2(this->config.wheelbase * angular_velocity, velocity);
    return std::clamp(delta, this->config.min_delta, this->config.max_delta);
}

void SensorFilterNode::publish_imu(
        double velocity, 
        double longitudinal_acceleration, 
        double angular_velocity, 
        double delta
) {
    utility::msg::FusedSensor msg;
    msg.header.frame_id = "filtered_imu";
    msg.header.stamp = this->now();
    msg.velocity = velocity;
    msg.longitudinal_acceleration = longitudinal_acceleration;
    msg.angular_velocity = angular_velocity;
    msg.delta = delta;
    this->pubFilteredIMU->publish(msg);
}

void SensorFilterNode::publish_hall(
    double hall_value,
    double velocity
) {
    utility::msg::FilteredHall msg;
    msg.header.frame_id = "filtered_hall";
    msg.header.stamp = this->now();
    msg.longitudinal_velocity = velocity;
    msg.dt8 = hall_value;
    this->pubFilteredHall->publish(msg);
}

void SensorFilterNode::declare_parameters() {
    this->declare_parameter("wheelbase", 0.258);
    this->declare_parameter("wheelradius", 0.065);
    this->declare_parameter("max_speed", 3.0);
    this->declare_parameter("min_speed", -1.0);
    this->declare_parameter("max_acceleration", 6.0);
    this->declare_parameter("min_acceleration", -10.0);
    this->declare_parameter("max_delta", 0.314159); // 18° * M_PI / 180
    this->declare_parameter("min_delta", -0.314159); // -18° * M_PI / 180
    this->declare_parameter("x_linear_acceleration_offset", 0.001);
    this->declare_parameter("z_angular_velocity_offset", 0.001);
    this->declare_parameter("tau_hall", 0.015);
}

void SensorFilterNode::load_config() {
    this->get_parameter("wheelbase", this->config.wheelbase);
    this->get_parameter("wheelradius", this->config.wheelradius);
    this->get_parameter("max_speed", this->config.max_speed);
    this->get_parameter("min_speed", this->config.min_speed);
    this->get_parameter("max_acceleration", this->config.max_acceleration);
    this->get_parameter("min_acceleration", this->config.min_acceleration);
    this->get_parameter("max_delta", this->config.max_delta);
    this->get_parameter("min_delta", this->config.min_delta);
    this->get_parameter("x_linear_acceleration_offset", this->config.x_linear_acceleration_offset);
    this->get_parameter("z_angular_velocity_offset", this->config.z_angular_velocity_offset);
    this->get_parameter("tau_hall", this->config.tau_hall);
}

void SensorFilterNode::log_config() {
    RCLCPP_DEBUG(this->get_logger(), "####################################");
    RCLCPP_DEBUG(this->get_logger(), "Wheelbase: %f", this->config.wheelbase);
    RCLCPP_DEBUG(this->get_logger(), "Wheelradius: %f", this->config.wheelradius);
    RCLCPP_DEBUG(this->get_logger(), "Maximal Speed Forwards: %f", this->config.max_speed);
    RCLCPP_DEBUG(this->get_logger(), "Maximal Speed Backwards: %f", this->config.min_speed);
    RCLCPP_DEBUG(this->get_logger(), "Maximal Acceleration: %f", this->config.max_acceleration);
    RCLCPP_DEBUG(this->get_logger(), "Minimal Acceleration: %f", this->config.min_acceleration);
    RCLCPP_DEBUG(this->get_logger(), "Maximal Delta: %f", this->config.max_delta);
    RCLCPP_DEBUG(this->get_logger(), "Minimal Delta: %f", this->config.min_delta);
    RCLCPP_DEBUG(this->get_logger(), "Maximal Delta: %f", this->config.max_delta);
    RCLCPP_DEBUG(this->get_logger(), "X Linear Acceleration Offset: %f", this->config.x_linear_acceleration_offset);
    RCLCPP_DEBUG(this->get_logger(), "Z Angular Velocity Offset: %f", this->config.z_angular_velocity_offset);
    RCLCPP_DEBUG(this->get_logger(), "Tau Hall: %f", this->config.tau_hall);
}

rcl_interfaces::msg::SetParametersResult SensorFilterNode::on_parameter_update(
        const std::vector<rclcpp::Parameter> &params
) {
    RCLCPP_DEBUG(this->get_logger(), "Parameter update received");
    for (const auto& param : params) {
        if (param.get_name() == "wheelbase") {
            this->config.wheelbase = param.as_double();
        } else if (param.get_name() == "wheelradius") {
            this->config.wheelradius = param.as_double();
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
        } else if (param.get_name() == "x_linear_acceleration_offset") {
            this->config.x_linear_acceleration_offset = param.as_double();
        } else if (param.get_name() == "z_angular_velocity_offset") {
            this->config.z_angular_velocity_offset = param.as_double();
        } else if (param.get_name() == "tau_hall") {
            this->config.tau_hall = param.as_double();
        }
    }
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    return result;
}


int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SensorFilterNode>());
    rclcpp::shutdown();
    return 0;
}
