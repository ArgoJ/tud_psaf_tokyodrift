/**
 * @file slam.cpp
 * @brief Implementation of the SLAM class for visual-inertial odometry.
 */

#include "../include/slam.h"

/**
 * @brief SLAM class constructor.
 * 
 * Initializes the SLAM system with ORB_SLAM3 and sets up ROS2 subscriptions.
 */
SLAM::SLAM(){
    bool showPangolin = false;
    bool bEqual = false;
    std::string config_path = "/home/psaf/wise-2024-25-tokyodrift/src/2_plan/SLAM/config/SLAM.yaml";
    std::string vocab_path = "/home/psaf/dependencies/ORB_SLAM3/Vocabulary/ORBvoc.txt";
    this->mpSLAM = std::make_shared<ORB_SLAM3::System>(vocab_path, config_path, ORB_SLAM3::System::MONOCULAR, showPangolin);
    
    subImg = this->create_subscription<sensor_msgs::msg::Image>("/color/image_raw", 100, std::bind(&SLAM::callback_imgLeft, this, std::placeholders::_1));
    subDepth = this->create_subscription<sensor_msgs::msg::Image>("/depth/image_rect_raw", 100, std::bind(&SLAM::callback_imgRight, this, std::placeholders::_1));
    subIMU = this->create_subscription<sensor_msgs::msg::Imu>("/imu", 1000, std::bind(&SLAM::callback_imu, this, std::placeholders::_1));
   
    tfBroadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    timer = this->create_wall_timer(100ms, std::bind(&SLAM::broadcast_transform, this));
}

/**
 * @brief Processes an RGB-D image pair.
 * 
 * Retrieves images, applies preprocessing if necessary, and performs SLAM tracking.
 */
void SLAM::process_image(){
    cv::Mat img = get_image();
    cv::Mat depthmap = get_depthmap();
    double tIm = this->lastImage->header.stamp.sec + this->lastImage->header.stamp.nanosec * 1e-9;
    
    if(img.empty()) return;

    if(this->mbClahe) mClahe->apply(img, img);

    Sophus::SE3f currPose = mpSLAM->TrackRGBD(img, depthmap, tIm);
    this->lastPose = currPose;
}

/**
 * @brief Broadcasts the current transform from world to robot base.
 */
void SLAM::broadcast_transform(){
    geometry_msgs::msg::TransformStamped msg;

    msg.header.stamp = this->get_clock()->now();
    msg.header.frame_id = "world";
    msg.child_frame_id = "robot_base";

    msg.transform.translation.x = lastPose.translation().x();
    msg.transform.translation.y = lastPose.translation().y();
    msg.transform.translation.z = lastPose.translation().z();

    msg.transform.rotation.x = lastPose.rotation().x();
    msg.transform.rotation.y = lastPose.rotation().y();
    msg.transform.rotation.z = lastPose.rotation().z();
    msg.transform.rotation.w = lastPose.rotation().w();

    tfBroadcaster->sendTransform(msg);
}

/**
 * @brief Callback function for IMU messages.
 * 
 * Stores IMU data in a buffer with thread safety.
 * @param msg The received IMU message.
 */
void SLAM::callback_imu(const sensor_msgs::msg::Imu::SharedPtr msg){
    bufMutex.lock();
    imuBuf.push(msg);
    bufMutex.unlock();
}

/**
 * @brief Converts a ROS2 Image message to an OpenCV Mat.
 * 
 * @param msg The Image message.
 * @return Converted OpenCV Mat.
 */
cv::Mat SLAM::convert_image(const sensor_msgs::msg::Image::SharedPtr msg){
    cv_bridge::CvImageConstPtr cvPtr;
    try {
        cvPtr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::MONO8);
    } catch (cv_bridge::Exception &e) {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    }
    
    if (cvPtr->image.type() == 0) {
        return cvPtr->image.clone();
    } else {
        RCLCPP_ERROR(this->get_logger(), "Error image type");
        return cvPtr->image.clone();
    }
}

/**
 * @brief Converts a ROS2 timestamp to seconds.
 * 
 * @param stamp The ROS2 timestamp.
 * @return The timestamp in seconds.
 */
double SLAM::stamp_to_sec(builtin_interfaces::msg::Time stamp){
    return stamp.sec + (stamp.nanosec * 1e-9);
}

