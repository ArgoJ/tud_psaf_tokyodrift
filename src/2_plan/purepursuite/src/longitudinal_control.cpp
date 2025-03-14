
//Geschwindigkeit
#include "longitudinal_control.h"



LongitudinalControl::LongitudinalControl(const PurepursuitControlParams &config)
    : Node("Longitudinal"), config(config) {

    RCLCPP_INFO(this->get_logger(), "Longitudinal Control Object created.");

}


double LongitudinalControl::calculateSpeed(const double distance_alpha){
    double speed_ms = 0;

    // This part is called when we are in a transition state between the max and min speed
    // It checks if the transition is finished, to change to FAST or SLOW
    if (this->current_speed_switch){
        if( abs(this->speed_switch_counter) >= this->config.long_switch_steps ){

            this->current_speed_switch = false;
            
            if(this->speed == Speed::DEACC) this->speed = Speed::SLOW;
            if(this->speed == Speed::ACC) this->speed = Speed::FAST;

        }else{
            if(this->speed == Speed::DEACC) this->speed_switch_counter++;
            if(this->speed == Speed::ACC) this->speed_switch_counter--;
        }
        
    }

    // Steering angle is increasing and surpasses a difference of 0.08 rad for risky settings -> heading towords DEACC
    if( abs(distance_alpha) > abs( abs(this->previous_distance_alpha) + this->config.long_min_angle_diff_DEACC )){

        // Dont enter if already deaccelerating or slow
        if (this->speed != Speed::SLOW && this->speed != Speed::DEACC)
        {
            if (this->speed == Speed::START){
                this->speed = Speed::DEACC;
                this->speed_switch_counter = 0;
            }
            else{
                this->speed = Speed::DEACC;
                this->speed_switch_counter++;
                this->current_speed_switch = true;
            }
        }

        this->previous_distance_alpha = distance_alpha;

    }else{
        // Steering angle is decreasing and surpasses a difference of 0.1 rad for risky settings -> heading towords ACC
        if( abs(distance_alpha) < abs( abs(this->previous_distance_alpha) - this->config.long_min_angle_diff_ACC)){ 
            // Dont enter if already accelerating or fast
            if( this->speed != Speed::FAST && this->speed != Speed::ACC){
                
                if (this->speed == Speed::START){
                    this->speed = Speed::ACC;
                    this->speed_switch_counter = 0;
                } 
                else{
                    this->speed = Speed::ACC;
                    this->speed_switch_counter--;
                    this->current_speed_switch = true;
                } 
            }

            this->previous_distance_alpha = distance_alpha;
    
        }
    } 

    switch (this->speed){
    
        case Speed::START:  
            speed_ms = this->config.long_max_speed / 2;
            break;

        case Speed::SLOW:
            speed_ms = 0.0;
            break;
        
        case Speed::FAST:
            speed_ms = this->config.long_max_speed;     
            break; 

        case Speed::DEACC:
        case Speed::ACC:{
            double factor_acc    = 0.5 - this->speed_switch_counter * (0.5 / this->config.long_switch_steps);   
             
            speed_ms = factor_acc * this->config.long_max_speed;     
            break; 
        }
    } 
    
    return speed_ms; 
}


