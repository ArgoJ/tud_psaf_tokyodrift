#ifndef TOKYODRIFT_DEPTH_OBSTACLE_DETECTION_NODE_H
#define TOKYODRIFT_DEPTH_OBSTACLE_DETECTION_NODE_H

#include <rclcpp/rclcpp.hpp>
#include <mutex>
#include <cmath>
#include <omp.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <atomic>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <std_msgs/msg/bool.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include "utility/msg/trajectory.hpp"
#include "depth_obstacle_detection_config.h"
#include "point_msg_helper.hpp"
#include "marker_publisher.hpp"
#include "cubic_bezier.hpp"
#include "timer.hpp"


class DepthObstacleDetectionNode : public rclcpp::Node {
private:
    DepthObstacleDetectionParams config;
    mutable std::mutex trajectory_mutex;
    std::vector<geometry_msgs::msg::Point> last_trajectory;
    bool use_left_lane = false;
    OnSetParametersCallbackHandle::SharedPtr callback_handle_;
    
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subDepthImage;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr subCameraInfo;
    rclcpp::Subscription<utility::msg::Trajectory>::SharedPtr subTrajectory;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubPointCloud;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pubLaneSwitch;
    
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pubOtherMarkerLine;

public:
    /**
     * @brief Constructs the DepthObstacleDetectionNode, initializes parameters, and sets up subscribers and publishers.
     * 
     * This function initializes the node by declaring parameters, loading the configuration, logging the configuration,
     * and setting up the subscriber and publisher for handling depth images, camera info, trajectory, and lane switch messages.
     * It also adds a callback for parameter updates.
     */
    DepthObstacleDetectionNode();

private:
    /**
     * @brief Declares parameters for the node.
     * 
     * This function declares the parameters that the node can use, including depth scale, camera
     * offsets, filtering parameters, point threshold, and thread count, with default values.
     */
    void declare_parameters();

    /**
     * @brief Loads the configuration parameters from ROS 2 parameters.
     * 
     * This function retrieves the parameter values from the ROS 2 parameter server and assigns them
     * to the configuration structure of the node. The camera intrinsic parameters are also initialized
     * with predefined values.
     */
    void load_config();

    /**
     * @brief Logs the current configuration parameters.
     * 
     * This function logs the current values of the node's configuration parameters, such as depth scale,
     * camera pitch, offsets, filtering parameters, and other related settings, using debug-level logging.
     */
    void log_config();

    /**
     * @brief Callback for handling parameter updates from the ROS 2 parameter server.
     * 
     * This function is triggered when parameters are updated on the parameter server. It checks for specific
     * parameter names and updates the corresponding fields in the configuration structure. The updated 
     * configuration is then applied.
     * 
     * @param params The list of updated parameters.
     * @return rcl_interfaces::msg::SetParametersResult The result of the parameter update, indicating success or failure.
     */
    rcl_interfaces::msg::SetParametersResult on_parameter_update(
        const std::vector<rclcpp::Parameter>& params
    );

    /**
     * @brief Callback function to update the camera intrinsic parameters.
     * 
     * This function is triggered when a new CameraInfo message is received. It updates the 
     * camera parameters (fx, fy, cx, cy) based on the values in the message, and logs any changes.
     * 
     * @param msg The CameraInfo message containing the updated camera parameters.
     */
    void camera_info_callback(
        const sensor_msgs::msg::CameraInfo::SharedPtr msg
    ); 
    
    /**
     * @brief Callback function to process the received depth image and perform obstacle detection.
     * 
     * This function converts the received depth image to a point cloud, checks if there are obstacles on the current
     * or moved trajectory, and publishes a message if a lane switch is needed. It also publishes the trajectory and point cloud.
     * 
     * @param msg The received depth image message.
     */
    void depth_image_callback(
        const sensor_msgs::msg::Image::SharedPtr msg
    );

    /**
     * @brief Callback function to process the received trajectory and update the current trajectory information.
     * 
     * This function updates the `last_trajectory` and `use_left_lane` values based on the received trajectory message.
     * 
     * @param msg The received trajectory message.
     */
    void trajectory_callback(
        const utility::msg::Trajectory::SharedPtr msg
    );

    /**
     * @brief Checks if an obstacle is present on the lane based on the provided point cloud and trajectory.
     * 
     * This function iterates through the trajectory points and checks if there are enough points in the point 
     * cloud within a defined distance to classify as an obstacle. It uses parallel processing to speed up the operation.
     *
     * @param cloud A pointer to the point cloud containing 3D points.
     * @param trajectory A vector of points representing the trajectory.
     * 
     * @return True if an obstacle is detected on the lane, otherwise false.
     */
    bool is_obstacle_on_lane(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
        const std::vector<geometry_msgs::msg::Point>& trajectory
    );

    /**
     * @brief Downsamples the input point cloud using a voxel grid filter.
     * 
     * This function reduces the resolution of the input point cloud by applying a voxel grid filter, which 
     * approximates the points in the cloud by selecting one point per voxel. It reduces the number of points 
     * while preserving the overall structure of the point cloud.
     * 
     * @param input_cloud A pointer to the point cloud to be downsampled.
     * @param leaf_size The size of the voxel grid used for downsampling.
     * 
     * @return A pointer to the downsampled point cloud.
     */
    pcl::PointCloud<pcl::PointXYZ>::Ptr downsample_cloud(
            const pcl::PointCloud<pcl::PointXYZ>::Ptr& input_cloud,
            const double leaf_size = 0.01
    );

    /**
     * @brief Converts a depth image to a 3D point cloud with the pinhole model.
     * 
     * This function transforms the depth information from an image into 3D space using camera parameters 
     * and returns a point cloud representation. The depth values are filtered based on the configured thresholds.
     * 
     * @param depth_img A depth image (single channel) containing depth information.
     * 
     * @return A pointer to the generated 3D point cloud.
     */
    pcl::PointCloud<pcl::PointXYZ>::Ptr depth_image2point_cloud(
        const cv::Mat &depth_img
    );

    /**
     * @brief Publishes a PointCloud2 message based on the provided PCL point cloud.
     * 
     * This function converts a PCL PointCloud of type pcl::PointXYZ to a ROS 2 PointCloud2 message
     * and publishes it using the associated ROS 2 publisher. The message's header is populated with
     * the current timestamp and the "map" frame ID.
     * 
     * @param cloud The input PCL point cloud (shared pointer) to be published.
     */
    void publish_point_cloud(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud
    );
};

#endif //TOKYODRIFT_DEPTH_OBSTACLE_DETECTION_NODE_H