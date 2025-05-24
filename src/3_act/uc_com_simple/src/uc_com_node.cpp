#include "uc_com_node.h"



UcComNode::UcComNode() : Node("uc_com"){

    RCLCPP_INFO(this->get_logger(), "Starting UcCom node!");


    this->declare_parameters();
    this->load_config();
    this->log_config();

    this->callback_handle_ = this->add_on_set_parameters_callback(
        std::bind(&UcComNode::on_parameter_update, this, std::placeholders::_1)
    );


//Subscriber
    this -> subControl = this->create_subscription<utility::msg::Control>(
            "tokyodrift/plan/control", 1,
            std::bind(&UcComNode::control_callback, this, std::placeholders::_1));

    this -> subStartDetector = this->create_subscription<utility::msg::Startbox>(
            "tokyodrift/sense/startbox", 1,
            std::bind(&UcComNode::start_detector_callback, this, std::placeholders::_1));

//Publisher
    this->pubSpeedUcBridge = this->create_publisher<std_msgs::msg::Int16>(
        "uc_bridge/set_motor_level_forward", 1); 
    

    this->pubSteeringUcBridge = this->create_publisher<std_msgs::msg::Int16>(
        "uc_bridge/set_steering", 1); 
}


void UcComNode::start_detector_callback(const utility::msg::Startbox::SharedPtr finished){
    RCLCPP_INFO(this->get_logger(), "start_detector_callback.");
    this->is_driving = finished->start_sequence_finished;
    RCLCPP_DEBUG(this->get_logger(), "Is driving: %d",this->is_driving);
}

void UcComNode::control_callback(const utility::msg::Control::SharedPtr control) { 
    if (!this->is_driving){
        return;
    }

    if (control->delta == 0.0  && control->longitudinal_control == 0.0) {
        RCLCPP_WARN(this->get_logger(), "control is empty.");
        return;
    }
    
    START_TIMER("speed ms to ucb");
    int speed = this->speed_ms_to_ucb(control->longitudinal_control);
    STOP_TIMER("speed ms to ucb");

    START_TIMER("Radiant to ucb");
    int delta = this->steering_angle_to_ucb(control->delta);
    STOP_TIMER("Radiant to ucb");

    speed_publisher(speed);
    delta_publisher(delta);
}

int UcComNode::speed_ms_to_ucb(const double speed){
    return static_cast<int>(std::clamp(365.5 * speed + 269, this->config.min_speed_ucb, this->config.max_speed_ucb)); 
}

int UcComNode::steering_angle_to_ucb(const double delta){
    return static_cast<int>(std::clamp( 765. * delta + 26.9, this->config.min_angle_ucb , this->config.max_angle_ucb));
}

void UcComNode::speed_publisher(const int speed_ucb){
    auto message = std_msgs::msg::Int16();
    message.data = speed_ucb;
    pubSpeedUcBridge -> publish(message);
}

void UcComNode::delta_publisher(const int steering_angle_ucb){
    auto message = std_msgs::msg::Int16();
    message.data = steering_angle_ucb;
    pubSteeringUcBridge -> publish(message);
    RCLCPP_DEBUG(this->get_logger(), "Published steering angle: %d", message.data);
}


void UcComNode::declare_parameters(){
    this->declare_parameter("min_angle_ucb", -175.0);
    this->declare_parameter("max_angle_ucb", 225.0);
    this->declare_parameter("min_speed_ucb", 300.0); 
    this->declare_parameter("max_speed_ucb", 1000.0);
    this->declare_parameter("min_angle_diff", 0.01); 
    this->declare_parameter("switch_steps", 4.0); 
    this->declare_parameter("steering_speed_factor", 800.0); 
    this->declare_parameter("avoide_crash", false);
}

void UcComNode::load_config(){
    this->get_parameter("min_angle_ucb", this->config.min_angle_ucb);
    this->get_parameter("max_angle_ucb", this->config.max_angle_ucb);
    this->get_parameter("min_speed_ucb", this->config.min_speed_ucb);
    this->get_parameter("max_speed_ucb", this->config.max_speed_ucb);
    this->get_parameter("min_angle_diff", this->config.min_angle_diff);  
    this->get_parameter("switch_steps", this->config.switch_steps);
    this->get_parameter("steering_speed_factor", this->config.steering_speed_factor);
    this->get_parameter("avoide_crash", this->config.avoid_crash);
}

void UcComNode::log_config(){
    RCLCPP_DEBUG(this->get_logger(), "####################################");
    RCLCPP_DEBUG(this->get_logger(), "Min angle ucb: %f", this->config.min_angle_ucb);
    RCLCPP_DEBUG(this->get_logger(), "Max angle ucb: %f", this->config.max_angle_ucb);
    RCLCPP_DEBUG(this->get_logger(), "Min speed ucb: %f", this->config.min_speed_ucb);
    RCLCPP_DEBUG(this->get_logger(), "Max speed ucb: %f", this->config.max_speed_ucb);
    RCLCPP_DEBUG(this->get_logger(), "Min angle diff: %f", this->config.min_angle_diff);
    RCLCPP_DEBUG(this->get_logger(), "Switch steps: %f", this->config.switch_steps);
    RCLCPP_DEBUG(this->get_logger(), "Steering speed factor: %f", this->config.steering_speed_factor);
    RCLCPP_DEBUG(this->get_logger(), "Avoid crash: %d", this->config.avoid_crash);
}

rcl_interfaces::msg::SetParametersResult UcComNode::on_parameter_update(
    const std::vector<rclcpp::Parameter>& params){

    RCLCPP_DEBUG(this->get_logger(), "Parameter update received");
    for (const auto& param : params){
        if (param.get_name() == "min_angle_ucb"){
            this->config.min_angle_ucb = param.as_double();
        } else if (param.get_name() == "max_angle_ucb"){
            this->config.max_angle_ucb = param.as_double();
        } else if (param.get_name() == "min_speed_ucb"){
            this->config.min_speed_ucb = param.as_double();
        } else if (param.get_name() == "max_speed_ucb"){
            this->config.max_speed_ucb = param.as_double();
        } else if (param.get_name() == "min_angle_diff"){
            this->config.min_angle_diff = param.as_double();
        } else if (param.get_name() == "switch_steps"){
            this->config.switch_steps = param.as_double();
        } else if (param.get_name() == "steering_speed_factor"){
            this->config.steering_speed_factor = param.as_double();
        } else if (param.get_name() == "avoid_crash"){
            this->config.avoid_crash = param.as_bool();
        }
    }
    
    this->log_config();

    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    return result;
}

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<UcComNode>());
    rclcpp::shutdown();
    return 0;
}