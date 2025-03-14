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

    this->subUsFront = this->create_subscription<sensor_msgs::msg::Range>(
            "/uc_bridge/us_front_center", 1, std::bind(&UcComNode::us_front_callback, this, std::placeholders::_1));


//Publisher
    this->pubSpeedUcBridge = this->create_publisher<std_msgs::msg::Int16>(
        "uc_bridge/set_motor_level_forward", 1); 
    

    this->pubSteeringUcBridge = this->create_publisher<std_msgs::msg::Int16>(
        "uc_bridge/set_steering", 1); 


//Timer speed
    timer = this->create_wall_timer(
            std::chrono::milliseconds(30),
            std::bind(&UcComNode::speed_publisher,  this)
        );
}


void UcComNode::start_detector_callback(const utility::msg::Startbox::SharedPtr finished){
    RCLCPP_INFO(this->get_logger(), "start_detector_callback.");
    this->is_driving = finished->start_sequence_finished;
    RCLCPP_DEBUG(this->get_logger(), "Is driving: %d",this->is_driving);
}

void UcComNode::us_front_callback(const sensor_msgs::msg::Range::SharedPtr us_front){
    RCLCPP_DEBUG(this->get_logger(), "Us front callback.");
    if (us_front->range < 0.5 && us_front->range != 0.0 && this->config.avoid_crash){
        this->obstacle_front = true;    
    }
    else{
        this->obstacle_front = false;
    }
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
    int angle = this->steering_angle_to_ucb(control->delta, control->delta_dist, control->jump);
    STOP_TIMER("Radiant to ucb");


    this->current_speed_ucb = speed;
    angle_publisher(angle);    
}

//Transform speed from m/s into uc_bridge values-> 0 to 1000 with 269 as speed_offset for 0 m/s
int UcComNode::speed_ms_to_ucb(const double speed){

    // Speed is calculated and clamped between min and max speed params to allow different speed settings, 
    // only min_speed is available to change, max_speed should only change if the motor gets exchanged through a faster one
    int speed_uc_bridge = static_cast<int>(std::clamp(365.5 * speed + 269, this->config.min_speed_ucb, this->config.max_speed_ucb)); 
    
    return speed_uc_bridge;
}






int UcComNode::steering_angle_to_ucb(const double angle, const double angle_dist, const bool jump){
    //If we change the switch_steps parameter while driving this one is important to introduce the new upper and lower bounds
    if(abs(this->direction_switch_counter) > this->config.switch_steps) this->direction_switch_counter = 0;

    int steering_angle_ucb = 0;

    // This part is called when we are in a transition state between the two functions
    // It checks if the transition is finished, to change to RIGHT or LEFT
    if (this->current_direction_switch_left || this->current_direction_switch_right){
        if( abs(this->direction_switch_counter) >= this->config.switch_steps ){

            this->current_direction_switch_left = false;
            this->current_direction_switch_right = false;
            
            if(this->direction == Direction::LEFT_TO_RIGHT) this->direction = Direction::RIGHT;
            if(this->direction == Direction::RIGHT_TO_LEFT) this->direction = Direction::LEFT;

        }else{
            if(this->direction == Direction::LEFT_TO_RIGHT) this->direction_switch_counter++;
            if(this->direction == Direction::RIGHT_TO_LEFT) this->direction_switch_counter--;
        }
        RCLCPP_DEBUG(this->get_logger(),"Counter: %i",this->direction_switch_counter);

    }

    // If a jump in the trajectory was detected, there should be no impact on the hysteresis state
    if(!jump){
        this->current_jump = false;
        
        // Angle of a lookahead is decreasing (not steering angle), heading to fct_left
        if( angle_dist < previous_angle_radiant - this->config.min_angle_diff ){ 

            // Don't enter if we already know the steering angle is decreasing, the first part in method will handle it
            if(this->direction != Direction::LEFT                                       && 
                this->direction != Direction::RIGHT_TO_LEFT)
            {
                
                if (this->direction == Direction::NEUTRAL){
                    this->direction = Direction::LEFT;
                    this->direction_switch_counter = -this->config.switch_steps;
                } 
                else{
                    if(!this->current_direction_switch_left){
                        this->direction = Direction::RIGHT_TO_LEFT;
                        this->current_direction_switch_left = true;
                        if(this->current_direction_switch_right && abs(this->direction_switch_counter - 2) <= this->config.switch_steps ){
                            // Substracting one more if a change in opposite direction occured beforehand to fasten the process
                            this->direction_switch_counter -= 2;
                        }else{
                            this->direction_switch_counter--;
                        }
                    }
                } 
                
            }
            this->current_direction_switch_right = false;
            this->previous_angle_radiant = angle_dist;

        }
        else{
            // Steering angle is increasing, heading to fct_right
            if( angle_dist > previous_angle_radiant + this->config.min_angle_diff ){
                
                // Don't enter if we already know the steering angle is increasing, the first part in method will handle it
                if (this->direction != Direction::RIGHT                                      &&
                    this->direction != Direction::LEFT_TO_RIGHT)
                {

                    if (this->direction == Direction::NEUTRAL){
                        this->direction = Direction::RIGHT;
                        this->direction_switch_counter = this->config.switch_steps;
                    }
                    else{
                        if(!this->current_direction_switch_right){
                            this->direction = Direction::LEFT_TO_RIGHT;
                            this->current_direction_switch_right = true;
                            if(this->current_direction_switch_left && abs(this->direction_switch_counter + 2) <= this->config.switch_steps ){
                                // Adding one more if a change in opposite direction occured beforehand to fasten the process
                                this->direction_switch_counter += 2;
                            }else{
                                this->direction_switch_counter++;
                            }
                        }
                    }
                }

                this->previous_angle_radiant = angle_dist;
                this->current_direction_switch_left = false;
            }
        }     
    }else{
        if(!this->current_jump){
            // If a jump occured its probably because we lost track of the lane due to false hysterese setting, 
            // to get back we might need to switch the direction
            if(this->direction == Direction::LEFT_TO_RIGHT){
                this->direction = Direction::RIGHT_TO_LEFT;
                // To have a bigger impact we substracting 2 if possible
                if(abs(this->direction_switch_counter - 2) <= this->config.switch_steps){
                    this->direction_switch_counter -= 2;
                }else{
                    this->direction_switch_counter -= 1;
                }
            }else{
                if(this->direction == Direction::RIGHT_TO_LEFT){
                    this->direction = Direction::LEFT_TO_RIGHT;
                    // To have a bigger impact we substracting 2 if possible
                    if(abs(this->direction_switch_counter + 2) <= this->config.switch_steps){
                        this->direction_switch_counter += 2;
                    }else{
                        this->direction_switch_counter += 1;
                    }
                }
            }
            
        }else{
            this->current_jump = true;
        }
    }

    // Speed factor to maximize steering_angle for higher speeds because the motor is a bit sluggish when it comes to higher speeds
    this->speed_factor = 1 + static_cast<double>(this->previous_speed_ucb - 300) / this->config.steering_speed_factor;

    switch (this->direction){
        
        case Direction::NEUTRAL:  

            steering_angle_ucb = 0.5 * fct_left(angle) + 0.5 * fct_right(angle);
            break;

        case Direction::RIGHT:

            steering_angle_ucb = fct_right(angle);
            break;
        
        case Direction::LEFT:

            steering_angle_ucb = fct_left(angle);     
            break; 

        case Direction::RIGHT_TO_LEFT:
        case Direction::LEFT_TO_RIGHT:{

            double factor_left  = 0.5 - this->direction_switch_counter * (0.5 / this->config.switch_steps)  ;   
            double factor_right = 0.5 + this->direction_switch_counter * (0.5 / this->config.switch_steps)  ;   
            
            steering_angle_ucb = factor_left * fct_left(angle) + factor_right * fct_right(angle) ;     
            break; 
        }           
    }
    return steering_angle_ucb;
}

int UcComNode::fct_left(const double angle){
    return static_cast<int>(std::clamp( 774.3187 * angle * speed_factor - 18.5259 , this->config.min_angle_ucb , this->config.max_angle_ucb));     
}
int UcComNode::fct_right(const double angle){
    return static_cast<int>(std::clamp( 756.7107 * angle * speed_factor + 72.0256 , this->config.min_angle_ucb , this->config.max_angle_ucb));     
}

void UcComNode::speed_publisher(){
    auto message = std_msgs::msg::Int16();
    if(this->obstacle_front ){
         message.data = 0.0;
    }
    else{
        message.data = this->current_speed_ucb;
    }
    pubSpeedUcBridge -> publish(message);
    this->previous_speed_ucb = this-> current_speed_ucb;
}

void UcComNode::angle_publisher(const int steering_angle){
    auto message = std_msgs::msg::Int16();
    message.data = steering_angle;
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