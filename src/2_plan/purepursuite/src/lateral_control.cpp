#include "../include/lateral_control.h"

LateralControl::LateralControl(const PurepursuitControlParams &config)
    :  Node("Lateral"), config(config) {
        
    previous_steering_ucb = previous_angle_radiant = previous_alpha  = 0.0;
    previous_x = this->config.lat_x_end;
    
    RCLCPP_INFO(this->get_logger(), "Lateral Control Object created.");
}


LateralControl::Result LateralControl::calculateSteeringAngle(const std::vector<geometry_msgs::msg::Point>& points){
    std::optional<int> current_x;

    double steering_angle_ucb = previous_steering_ucb;
    double steering_angle_radiant = previous_angle_radiant;

    if (points.size() < this->config.lat_x_end){
        RCLCPP_WARN(this->get_logger(),"Trajectory length too short, points.size(): %zu",points.size());
        return {steering_angle_radiant, previous_distance_alpha, jump};
    }

    if (points[this->config.lat_x_end].x == 0 || points[this->config.lat_x_start].x == 0){ //TODO since x contains a float, checking for equality doesn't make very much sense. maybe change
        std::string output = points[this->config.lat_x_end].x == 0 ? "this->config.lat_x_end" : "this->config.lat_x_start"; 
        RCLCPP_WARN(this->get_logger(),"Single Points are empty: %s steering_ucb: %f", output.c_str(), steering_angle_ucb);
        return {steering_angle_radiant, previous_distance_alpha, jump};
    }
    
   
// Angle at distance of first relevant point, 1,1m from rear axis for slow settings 
    double close_alpha = std::atan(points[this->config.lat_x_start].y / points[this->config.lat_x_start].x);
// Angle at distance of last relevant point, 1,34m from rear axis for slow settings
    double distance_alpha = std::atan(points[this->config.lat_x_end].y / points[this->config.lat_x_end].x);
// Angle further away to detect incoming curves, 1,52m from rear axis for risky settings
    double max_alpha = std::atan(points[this->config.lat_x_distance].y / points[this->config.lat_x_distance].x);

    
// Compare close_alpha and distance_alpha for smaller angle
// Step_direction is used to start from middle_alpha, to select the moving direction (towords smaller angle)
    int step_direction = (std::abs(distance_alpha) - std::abs(close_alpha)) > 0 ? -1 : 1;

//current_x is set to the middle distance
    current_x = (this->config.lat_x_end - this->config.lat_x_start) / 2 + this->config.lat_x_start; 
    double alpha = std::atan(points[*current_x].y / points[*current_x].x);

// Find smallest angle
    while(*current_x >= this->config.lat_x_start && *current_x <= this->config.lat_x_end){

        // Just check ever 10th point
        current_x = *current_x + this->config.lat_step_width * step_direction; 

        //check boundaries 
        if(*current_x < this->config.lat_x_start || *current_x > this->config.lat_x_end) break;

        //Angle at next point
        double next_alpha = std::atan(points[*current_x].y / points[*current_x].x);
        
        /*If the angle gets bigger while we move towards the already found "smallest angle" (close_alpha or distance_alpha),
         we found a even smaller one and return it*/
        if(std::abs(next_alpha) > std::abs(alpha)) break;
        
        alpha = next_alpha;
    }

      

    
/*If the smallest angle was not found in the middle of the given line, 
then it must be the one at 30cm or 1m. We return the correct one by 
having a look at the step_direction*/
    if (*current_x < this->config.lat_x_start || *current_x > this->config.lat_x_end){  
        alpha = step_direction < 0 ? close_alpha : distance_alpha;
        current_x = step_direction < 0 ? this->config.lat_x_start : this->config.lat_x_end;
    }


    bool current_jump = false;

// FIRST JUMP DETECTION
    //Dont allow to big steps to far distance lookahead point to minimize jumps
    if(*current_x > *previous_x && abs(this->previous_y) > 0.25){
        current_x = *previous_x;
        alpha = this->previous_alpha;
        max_alpha = this->previous_max_alpha;

        this->jump = true;
        current_jump = true;
    }else{
        this->jump = false;
    }
    

// lookaheaddifference is distance to point on trajectory with smallest angle
// sqrt(x^2 + y^2)
    double lookahead_difference = std::sqrt(std::pow(points[*current_x].y,2) + std::pow(points[*current_x].x,2));
    

// calculating the steering angle based on the pure pursuit algorithm 
// if lookahead_difference is way too high we use the previous steering angle, max y-value of +-1m is realistic
    if(lookahead_difference > std::sqrt(std::pow(points[this->config.lat_x_end].x,2) + 1)){
        steering_angle_radiant = previous_angle_radiant;
    }else{
        steering_angle_radiant = atan(( 2 * this->config.lat_wheel_base *std::sin(alpha)) / lookahead_difference);
    }
   
// SECOND JUMP DETECTION
    // Steering anlgle is not allowed to drop suddenly
    if ((abs(steering_angle_radiant - previous_angle_radiant) > 0.2 && abs(steering_angle_radiant) < abs(previous_angle_radiant)) || 
    (abs(this->previous_y - points[*current_x].y) > 0.25 && abs(points[*current_x].y) < abs(this->previous_y))){

        steering_angle_radiant = previous_angle_radiant;
        current_x = *previous_x;
        max_alpha = this->previous_max_alpha;

        this->jump = true;
    }else{
        if(!current_jump) this->jump = false;
    }
    
    

    //update previous_angle if we get no data in the next line we will just publish the same
    previous_steering_ucb = steering_angle_ucb;
    previous_angle_radiant = steering_angle_radiant;
    previous_alpha = alpha;
    this->previous_distance_alpha = distance_alpha;
    if(!current_jump){
        this->previous_y = points[*current_x].y;
    }
    previous_x = *current_x;
    this->previous_max_alpha = max_alpha;

    
    return {steering_angle_radiant, max_alpha, jump};
}

std::pair<double, double> LateralControl::calculateSteeringAngle_new(const std::vector<geometry_msgs::msg::Point>& trajectory) {
    geometry_msgs::msg::Point start_point = make_point(0.0, 0.0);
    auto [unit_grad, target, distance] = find_min_y_unit_grad(trajectory, start_point, 0.003*this->config.lat_x_start + 0.5, 0.003*this->config.lat_x_end+0.5);
    double steering_angle = calculate_delta(unit_grad, distance, this->config.lat_wheel_base);

    const double distant_distance = 0.003*this->config.lat_x_distance+0.5;
    geometry_msgs::msg::Point distant_point = find_point_on_line(trajectory, start_point, distant_distance);
    double distant_alpha = calculate_normalized_angle(start_point, distant_point);

    geometry_msgs::msg::Point x_distant_point = find_x_distance_point_on_line(trajectory, start_point, distant_distance);
    double x_distant_alpha = calculate_normalized_angle(start_point, x_distant_point);
    return {steering_angle, x_distant_alpha};
}







