#include "purepursuit_control_node.h"



PurepursuitControlNode::PurepursuitControlNode() : Node("purepursuit_control_node") {

    RCLCPP_INFO(this->get_logger(), "Starting purepursuit control node!");
    
    this->longitudinal_control = std::make_unique<LongitudinalControl>(this->config);
    this->lateral_control = std::make_unique<LateralControl> (this->config);

    this->declare_parameters();
    this->load_config();
    this->log_config();
    this->callback_handle_ = this->add_on_set_parameters_callback(
        std::bind(&PurepursuitControlNode::on_parameter_update, this, std::placeholders::_1));

//Subscriber
    this -> subTrajectory = this->create_subscription<utility::msg::Trajectory>(
        "tokyodrift/sense/lane_detection/mid_lane", 1,
        std::bind(&PurepursuitControlNode::trajectory_callback, this, std::placeholders::_1));

//Publisher
    this->pubControl = this->create_publisher<utility::msg::Control>(
        "tokyodrift/plan/control", 1);
}


void PurepursuitControlNode::trajectory_callback(const utility::msg::Trajectory::SharedPtr trajectory) {   
    if (trajectory->points.empty()) {
        RCLCPP_WARN(this->get_logger(), "Trajectory is empty.");
        return;
    }else{

    std::vector<geometry_msgs::msg::Point> points;

    for (const auto& traj_point : trajectory->points) {
        geometry_msgs::msg::Point point;
        point.x = traj_point.x;
        point.y = traj_point.y;
        point.z = 0;
        points.push_back(point);
    }

    START_TIMER("Calculate Steering Angle");
    LateralControl::Result res= lateral_control->calculateSteeringAngle(points);
    STOP_TIMER("Calculate Steering Angle");

    

    double steering_angle_radiant = res.steering_angle_radiant;
    double distance_alpha = res.max_alpha;
    bool jump = res.jump;

    START_TIMER("Calculate Speed");
    double speed_ms = longitudinal_control->calculateSpeed(distance_alpha);
    STOP_TIMER("Calculate Speed");

    this->control_publisher(speed_ms, steering_angle_radiant, distance_alpha, jump);
    
    }
}

void PurepursuitControlNode::control_publisher(const double speed, const double delta, const double delta_dist, const bool jump) {
    auto message = utility::msg::Control();
   
    message.header.stamp = this->now();
    message.longitudinal_control = speed;
    message.delta = delta;
    message.delta_dist = delta_dist;
    message.jump = jump;
    this->pubControl -> publish(message);
}


void PurepursuitControlNode::declare_parameters(){
    this->declare_parameter("long_max_speed", 0.4); // Außen 1.6 Innen 1.6
    this->declare_parameter("long_min_angle_diff_ACC", 0.2); // Außen 0.2 Inner 0.016
    this->declare_parameter("long_min_angle_diff_DEACC", 0.15); // Außen 0.008 Inner 0.005
    this->declare_parameter("long_switch_steps", 7.0); // Außen 10.0 Inner 5.0

    this->declare_parameter("lat_step_width", 20.0);  // Schrittweite, um die von der x-Entfernung zurückgegangen wird, um einen näheren Punkt zu wählen
    this->declare_parameter("lat_x_start", 150.0);    //Erster Punkt auf Trajektorie, Formel: X_START * 3mm + 50cm |  150 -> 0.95m
    this->declare_parameter("lat_x_end", 300.0);      //Außen 330.0 Letzter Punkt auf Trajektorie, Formel: X_END * 3mm + 50cm | 300-> 1,4 m
    this->declare_parameter("lat_x_distance", 350.0);
    this->declare_parameter("lat_wheel_base", 0.258);
}

void PurepursuitControlNode::load_config(){
    this->get_parameter("long_max_speed", this->config.long_max_speed);
    this->get_parameter("long_min_angle_diff_ACC", this->config.long_min_angle_diff_ACC);  
    this->get_parameter("long_min_angle_diff_DEACC", this->config.long_min_angle_diff_DEACC);  
    this->get_parameter("long_switch_steps", this->config.long_switch_steps);


    this->get_parameter("lat_step_width", this->config.lat_step_width);
    this->get_parameter("lat_x_start", this->config.lat_x_start);
    this->get_parameter("lat_x_end", this->config.lat_x_end);
    this->get_parameter("lat_x_distance", this->config.lat_x_distance);
    this->get_parameter("lat_wheel_base", this->config.lat_wheel_base);
}

void PurepursuitControlNode::log_config(){
    RCLCPP_DEBUG(this->get_logger(), "####################################");
    RCLCPP_DEBUG(this->get_logger(), "Max speed: %f", this->config.long_max_speed);
    RCLCPP_DEBUG(this->get_logger(), "Min angle diff ACC: %f", this->config.long_min_angle_diff_ACC);
    RCLCPP_DEBUG(this->get_logger(), "Min angle diff DEACC: %f", this->config.long_min_angle_diff_DEACC);
    RCLCPP_DEBUG(this->get_logger(), "Switch steps: %f", this->config.long_switch_steps);


    RCLCPP_DEBUG(this->get_logger(), "Step width: %f", this->config.lat_step_width);
    RCLCPP_DEBUG(this->get_logger(), "X start: %f", this->config.lat_x_start);
    RCLCPP_DEBUG(this->get_logger(), "X end: %f", this->config.lat_x_end);
    RCLCPP_DEBUG(this->get_logger(), "X distance: %f", this->config.lat_x_distance);
    RCLCPP_DEBUG(this->get_logger(), "wheel base: %f", this->config.lat_wheel_base);
}

rcl_interfaces::msg::SetParametersResult PurepursuitControlNode::on_parameter_update(
    const std::vector<rclcpp::Parameter>& params){

    RCLCPP_DEBUG(this->get_logger(), "Parameter update received");
    for (const auto& param : params){
        if (param.get_name() == "long_max_speed"){
            this->config.long_max_speed = param.as_double();
        } else if (param.get_name() == "long_min_angle_diff_ACC"){
            this->config.long_min_angle_diff_ACC = param.as_double();
        } else if (param.get_name() == "long_min_angle_diff_DEACC"){
            this->config.long_min_angle_diff_DEACC = param.as_double();
        } else if (param.get_name() == "long_switch_steps"){
            this->config.long_switch_steps = param.as_double();
        } 
        
        else if (param.get_name() == "lat_step_width"){
            this->config.lat_step_width = param.as_double();
        } else if (param.get_name() == "lat_x_start"){
            this->config.lat_x_start = param.as_double();
        } else if (param.get_name() == "lat_x_end"){
            this->config.lat_x_end = param.as_double();
        } else if (param.get_name() == "lat_x_distance"){
            this->config.lat_x_distance = param.as_double();
        } else if (param.get_name() == "lat_wheel_base"){
            this->config.lat_wheel_base = param.as_double();
        } 
    }
    
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    return result;
}


int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PurepursuitControlNode>());
    rclcpp::shutdown();
    return 0;
}