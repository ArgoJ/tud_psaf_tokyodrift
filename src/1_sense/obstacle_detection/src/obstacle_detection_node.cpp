# include "../include/obstacle_detection_node.h"

ObstacleDetectionNode::ObstacleDetectionNode(): Node("obstacle_detection_node"){
    RCLCPP_INFO(this->get_logger(), "Starting ObstacleDetection node!");


    

    // Subscribe to camera topic
    this->subUsFront = this->create_subscription<sensor_msgs::msg::Range>(
        "/uc_bridge/us_front_center", 1, std::bind(&ObstacleDetectionNode::us_front_callback, this, std::placeholders::_1)
    );
    this->subUsRight = this->create_subscription<sensor_msgs::msg::Range>(
        "/uc_bridge/us_mid_right", 1, std::bind(&ObstacleDetectionNode::us_right_callback, this, std::placeholders::_1)
    );
    

    //Publisher
    this->pubObstacleDetected = this -> create_publisher<utility::msg::ObstacleDetection>(
        "tokyodrift/sense/obstacle_detection", 1);
    this->pubLane = this -> create_publisher<std_msgs::msg::Bool>(
        "current_lane", 1);
}


void ObstacleDetectionNode::us_front_callback(const sensor_msgs::msg::Range::SharedPtr us_front){
    if (us_front->range < 0.8 && us_front->range != 0.0){
        this->obstacle_front = true;

        if (this->front_counter >= 2){
            this->stay_left = true;
            this->front_counter = 50;
        } 

        else this->front_counter  = std::min(2, this->front_counter + 1);
          
    }
    else{
        this->front_counter  = std::max(0, this->front_counter - 1);;
        this->obstacle_front = false;
    } 

    ObstacleDetectionNode::lane_publisher();
    ObstacleDetectionNode::object_detected_publisher();


}
void ObstacleDetectionNode::us_right_callback(const sensor_msgs::msg::Range::SharedPtr us_right){
     if (us_right->range < 0.2 && us_right->range != 0.0 ){
        this->obstacle_right = true;
        this->right_counter = 10;
        this->stay_left = false;
        
    }
    else{
        this->right_counter = std::max(0, this->right_counter - 1);
        this->obstacle_right = false;
    } 

    ObstacleDetectionNode::lane_publisher();
    ObstacleDetectionNode::object_detected_publisher();
}


void ObstacleDetectionNode::object_detected_publisher(){
    auto message = utility::msg::ObstacleDetection();
    message.obstacle_detected_front = this->obstacle_front;
    message.obstacle_detected_right = this->obstacle_right;

    pubObstacleDetected -> publish(message);
   
}

void ObstacleDetectionNode::lane_publisher(){
    auto message = std_msgs::msg::Bool();
    if ( this->stay_left && this->front_counter != 0 ) message.data = true;
    else{
        if(this->right_counter == 0 || this->front_counter == 0){
            message.data = false;
        }
    } 
    pubLane -> publish(message);
}



int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ObstacleDetectionNode>());
    rclcpp::shutdown();
    return 0;
}
