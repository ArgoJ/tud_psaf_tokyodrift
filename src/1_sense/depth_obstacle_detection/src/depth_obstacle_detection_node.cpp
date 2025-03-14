#include "depth_obstacle_detection_node.h"



DepthObstacleDetectionNode::DepthObstacleDetectionNode() : Node("depth_obstacle_detection") {
    this->declare_parameters();
    this->load_config();
    this->log_config();

    this->callback_handle_ = this->add_on_set_parameters_callback(
        std::bind(&DepthObstacleDetectionNode::on_parameter_update, this, std::placeholders::_1)
    );

    // Subscriber
    subDepthImage = this->create_subscription<sensor_msgs::msg::Image>(
        "camera/realsense2_camera_node/depth/image_rect_raw", 1,
        std::bind(&DepthObstacleDetectionNode::depth_image_callback, this, std::placeholders::_1)
    );
    subCameraInfo = this->create_subscription<sensor_msgs::msg::CameraInfo>(
        "camera/realsense2_camera_node/depth/camera_info", 1,
        std::bind(&DepthObstacleDetectionNode::camera_info_callback, this, std::placeholders::_1)
    );
    this->subTrajectory = this->create_subscription<utility::msg::Trajectory>(
        "tokyodrift/plan/transformed_lane", 1, // plan/transformed_lane, sense/lane_detection/mid_lane
        std::bind(&DepthObstacleDetectionNode::trajectory_callback, this, std::placeholders::_1)
    );

    // Publisher
    this->pubLaneSwitch = this -> create_publisher<std_msgs::msg::Bool>(
        "current_lane", 1
    );
    this->pubPointCloud = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "tokyodrift/sense/point_cloud", 3
    );
    this->pubOtherMarkerLine = this->create_publisher<visualization_msgs::msg::Marker>(
        "tokyodrift/sense/obstacle_detection/other_lane", 3
    );
}


void DepthObstacleDetectionNode::depth_image_callback(
        const sensor_msgs::msg::Image::SharedPtr msg
) {
    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1);
    } catch (cv_bridge::Exception &e) {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }

    // Convert depth image to point cloud
    START_TIMER("Depth2Cloud")
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = depth_image2point_cloud(cv_ptr->image);
    if (!cloud || cloud->empty()) {
        RCLCPP_ERROR(this->get_logger(), "Generated point cloud is empty");
        return;
    }
    // auto filtered_cloud = downsample_cloud(cloud, 0.01);
    // if (!filtered_cloud || filtered_cloud->empty()) {
    //     RCLCPP_ERROR(this->get_logger(), "Filtered cloud is empty");
    //     return;
    // }
    STOP_TIMER("Depth2Cloud")

    START_TIMER("Obstacle Detection")
    std::vector<geometry_msgs::msg::Point> current_trajectory;
    bool current_use_left_lane;
    {
        std::lock_guard<std::mutex> lock(this->trajectory_mutex);
        current_trajectory = this->last_trajectory;
        current_use_left_lane = this->use_left_lane;
    }
    
    if (current_trajectory.empty()) {
        return;
    }

    constexpr double ts = 1.0 / 7.0;

    auto [complete_current_traj, _] = get_cubic_bezier_complete(
        current_trajectory, 1.0, 0.37, ts
    );
    bool obstecal_on_current_lane = this->is_obstacle_on_lane(cloud, complete_current_traj);

    std::vector<geometry_msgs::msg::Point> complete_moved_traj;
    if (obstecal_on_current_lane || current_use_left_lane) {
        int sign = current_use_left_lane ? 1 : -1;
        std::vector<geometry_msgs::msg::Point> moved_trajectory = move_trajectory(current_trajectory, sign * this->config.lane_width);
        std::vector<geometry_msgs::msg::Point> smoothed_moved_trajectory = smooth_trajectory(moved_trajectory, 10.0, 0.5, 4);
        auto complete_moved_results = get_cubic_bezier_complete(
            smoothed_moved_trajectory, 1.2, 0.36, ts
        );
        complete_moved_traj = complete_moved_results.first;

        bool obstecal_on_moved_lane = this->is_obstacle_on_lane(cloud, complete_moved_traj);
        if (!obstecal_on_moved_lane) {
            RCLCPP_INFO(this->get_logger(), "Found Obstecals on current lane or on left lane and other Lane free!");
            std_msgs::msg::Bool msg;
            msg.data = !current_use_left_lane;
            this->pubLaneSwitch->publish(msg);
        } 
    }
    STOP_TIMER("Obstacle Detection")

    START_TIMER("Publish Moved Trajectory")
    publish_marker_points(
        complete_moved_traj,
        this->pubOtherMarkerLine,
        "map",
        this->get_clock(),
        Color{0.0, 1.0, 1.0},
        "other_lane",
        0,
        0.03,
        0.7,
        visualization_msgs::msg::Marker::LINE_STRIP
    );
    START_TIMER("Publish Moved Trajectory")

    START_TIMER("Publish Cloud")
    this->publish_point_cloud(cloud);
    STOP_TIMER("Publish Cloud")
}

void DepthObstacleDetectionNode::trajectory_callback(
        const utility::msg::Trajectory::SharedPtr msg
) {
    std::lock_guard<std::mutex> lock(this->trajectory_mutex);
    this->last_trajectory = msg->points;
    this->use_left_lane = (msg->current_lane == 0) ? false : true;
}

bool DepthObstacleDetectionNode::is_obstacle_on_lane(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, 
        const std::vector<geometry_msgs::msg::Point>& trajectory
) {
    
    if (!cloud || cloud->empty()) {
        RCLCPP_ERROR(this->get_logger(), "Point cloud is null or empty");
        return true;
    }

    const double half_lane_width = this->config.lane_width / 2.0;
    const double squared_radius = half_lane_width * half_lane_width;
    const int threshold = this->config.point_threshold;

    if (cloud->points.size() < static_cast<size_t>(threshold)) {
        return false;  // Not enough points to be an obstacle
    }

    std::atomic<bool> obstacle_found(false);

    #pragma omp parallel for num_threads(this->config.num_threads) default(none) \
        shared(obstacle_found, cloud, trajectory, squared_radius, threshold)
    for (size_t i = 0; i < trajectory.size(); ++i) {
        if (obstacle_found.load(std::memory_order_acquire))
            continue;

        const auto &traj_pt = trajectory[i];
        int count = 0;
        // Here, we vectorize the inner loop without early exit:
        #pragma omp simd reduction(+:count)
        for (size_t j = 0; j < cloud->points.size(); ++j) {
            const auto &pc_pt = cloud->points[j];
            double dx = pc_pt.x - traj_pt.x;
            double dy = pc_pt.y - traj_pt.y;
            count += (dx * dx + dy * dy <= squared_radius) ? 1 : 0;
        }
        if (count >= threshold) {
            obstacle_found.store(true, std::memory_order_release);
        }
    }

    return obstacle_found.load(std::memory_order_acquire);
}

pcl::PointCloud<pcl::PointXYZ>::Ptr DepthObstacleDetectionNode::depth_image2point_cloud(
        const cv::Mat &depth_img
) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    double alpha = this->config.camera_pitch * M_PI / 180.0;
    double r00 = cos(alpha), r01 = 0.0, r02 = -sin(alpha);
    double r10 = 0.0,      r11 = 1.0, r12 = 0.0;
    double r20 = sin(alpha), r21 = 0.0, r22 = cos(alpha);

    const double depth_scale = this->config.depth_scale;
    const double inv_fx = 1.0 / this->config.fx;
    const double inv_fy = 1.0 / this->config.fy;
    const double cx = this->config.cx;
    const double cy = this->config.cy;
    const double x_offset = this->config.camera_x_offset;
    const double z_offset = this->config.camera_z_offset;

    std::vector<pcl::PointXYZ, Eigen::aligned_allocator<pcl::PointXYZ>> all_points;
    all_points.reserve(depth_img.rows * depth_img.cols / 4);

    // Parallelize with OpenMP
    #pragma omp parallel num_threads(this->config.num_threads)
    {
        std::vector<pcl::PointXYZ, Eigen::aligned_allocator<pcl::PointXYZ>> local_points;
        local_points.reserve(depth_img.rows * depth_img.cols / (4 * this->config.num_threads));

        #pragma omp for nowait
        for (int y = 0; y < depth_img.rows; ++y) {
            const uint16_t* row_ptr = depth_img.ptr<uint16_t>(y);
            
            #pragma omp simd
            for (int x = 0; x < depth_img.cols; ++x) {
                uint16_t depth_val = row_ptr[x];
                if (depth_val == 0)
                    continue;

                double depth_m = depth_val * depth_scale;

                double X = depth_m;
                double Y = (x - cx) * depth_m * inv_fx;
                double Z = - (y - cy) * depth_m * inv_fy;

                // Manually apply the rotation.
                double X_rot = r00 * X + r01 * Y + r02 * Z + x_offset;
                double Y_rot = r10 * X + r11 * Y + r12 * Z;  // lateral coordinate
                double Z_rot = r20 * X + r21 * Y + r22 * Z + z_offset;

                if (Z_rot > this->config.z_low_filter && Z_rot < this->config.z_high_filter && X_rot < this->config.x_high_filter) {
                    local_points.emplace_back(X_rot, Y_rot, Z_rot);
                }
            }
        }
        // Merge the thread-local results.
        #pragma omp critical
        {
            all_points.insert(all_points.end(), local_points.begin(), local_points.end());
        }
    }

    // Finalize the point cloud.
    cloud->points = std::move(all_points);
    cloud->width = static_cast<uint32_t>(cloud->points.size());
    cloud->height = 1;
    cloud->is_dense = false;

    return cloud;
}


pcl::PointCloud<pcl::PointXYZ>::Ptr DepthObstacleDetectionNode::downsample_cloud(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& input_cloud,
        const double leaf_size
) {
    if (!input_cloud || input_cloud->empty()) {
        RCLCPP_ERROR(this->get_logger(), "Input cloud is null or empty");
        return input_cloud;
    }

    if (leaf_size <= 0.0) {
        RCLCPP_ERROR(this->get_logger(), "Invalid leaf size: %f", leaf_size);
        return input_cloud;
    }

    // Create a new cloud with proper initialization
    auto filtered = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    filtered->points.reserve(input_cloud->points.size() / 4);  // Reserve reasonable space

    try {
        // Create and configure the voxel grid filter
        pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
        voxel_filter.setInputCloud(input_cloud);
        voxel_filter.setLeafSize(leaf_size, leaf_size, leaf_size);
        voxel_filter.setMinimumPointsNumberPerVoxel(1);
        voxel_filter.filter(*filtered);

        if (filtered->empty()) {
            RCLCPP_WARN(this->get_logger(), "Filtered cloud is empty, returning original");
            return input_cloud;
        }

        // Set cloud metadata
        filtered->width = static_cast<uint32_t>(filtered->points.size());
        filtered->height = 1;
        filtered->is_dense = false;
        
        RCLCPP_DEBUG(this->get_logger(), "Downsampling successful: %zu -> %zu points",
                     input_cloud->points.size(), filtered->points.size());
        
        return filtered;

    } catch (const pcl::PCLException& e) {
        RCLCPP_ERROR(this->get_logger(), "PCL error during downsampling: %s", e.what());
        return input_cloud;
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Standard exception during downsampling: %s", e.what());
        return input_cloud;
    } catch (...) {
        RCLCPP_ERROR(this->get_logger(), "Unknown error during downsampling");
        return input_cloud;
    }
}


void DepthObstacleDetectionNode::publish_point_cloud(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud
) {
    sensor_msgs::msg::PointCloud2 cloud_msg;

    // Convert PCL point cloud to ROS 2 message
    pcl::toROSMsg(*cloud, cloud_msg);

    // Fill header information
    cloud_msg.header.stamp = this->now();
    cloud_msg.header.frame_id = "map"; // Use appropriate frame_id

    // Publish the cloud
    this->pubPointCloud->publish(cloud_msg);
}


void DepthObstacleDetectionNode::camera_info_callback(
        const sensor_msgs::msg::CameraInfo::SharedPtr msg
) {
    if (this->config.fx != msg->k[0]) {
        this->config.fx = msg->k[0];
        RCLCPP_INFO(this->get_logger(), "Updated fx=%f", this->config.fx);
    }
    if (this->config.fy != msg->k[4]) {
        this->config.fy = msg->k[4];
        RCLCPP_INFO(this->get_logger(), "Updated fy=%f", this->config.fy);
    }
    if (this->config.cx != msg->k[2]) {
        this->config.cx = msg->k[2];
        RCLCPP_INFO(this->get_logger(), "Updated cx=%f", this->config.cx);
    }
    if (this->config.cy != msg->k[5]) {
        this->config.cy = msg->k[5];
        RCLCPP_INFO(this->get_logger(), "Updated cy=%f", this->config.cy);
    }
}

void DepthObstacleDetectionNode::declare_parameters() {
    this->declare_parameter("depth_scale", 0.001);
    this->declare_parameter("camera_pitch", -30.5);
    this->declare_parameter("camera_x_offset", 0.2);
    this->declare_parameter("camera_z_offset", 0.26);
    this->declare_parameter("lane_width", 0.4);
    this->declare_parameter("z_low_filter", 0.02);
    this->declare_parameter("z_high_filter", 0.5);
    this->declare_parameter("x_high_filter", 2.5);
    this->declare_parameter("point_threshold", 100);
    this->declare_parameter("num_threads", 4);
}


void DepthObstacleDetectionNode::load_config() {
    this->get_parameter("depth_scale", this->config.depth_scale);
    this->get_parameter("camera_pitch", this->config.camera_pitch);
    this->get_parameter("camera_x_offset", this->config.camera_x_offset);
    this->get_parameter("camera_z_offset", this->config.camera_z_offset);
    this->get_parameter("lane_width", this->config.lane_width);
    this->get_parameter("z_low_filter", this->config.z_low_filter);
    this->get_parameter("z_high_filter", this->config.z_high_filter);
    this->get_parameter("x_high_filter", this->config.x_high_filter);
    this->get_parameter("point_threshold", this->config.point_threshold);
    this->get_parameter("num_threads", this->config.num_threads);

    this->config.fx = 432.5694274902344;
    this->config.fy = 432.5694274902344;
    this->config.cx = 428.07177734375;
    this->config.cy = 238.3310089111328;
}

void DepthObstacleDetectionNode::log_config() {
    RCLCPP_DEBUG(this->get_logger(), "####################################");
    RCLCPP_DEBUG(this->get_logger(), "Depth Scale: %f", this->config.depth_scale);
    RCLCPP_DEBUG(this->get_logger(), "Camera Pitch: %f", this->config.camera_pitch);
    RCLCPP_DEBUG(this->get_logger(), "Camera X Offset: %f", this->config.camera_x_offset);
    RCLCPP_DEBUG(this->get_logger(), "Camera Z Offset: %f", this->config.camera_z_offset);
    RCLCPP_DEBUG(this->get_logger(), "Lane Width: %f", this->config.lane_width);
    RCLCPP_DEBUG(this->get_logger(), "Z Low Filter: %f", this->config.z_low_filter);
    RCLCPP_DEBUG(this->get_logger(), "Z High Filter: %f", this->config.z_high_filter);
    RCLCPP_DEBUG(this->get_logger(), "X High Filter: %f", this->config.x_high_filter);
    RCLCPP_DEBUG(this->get_logger(), "Point Threshold: %i", this->config.point_threshold);
    RCLCPP_DEBUG(this->get_logger(), "Number of Threads: %i", this->config.num_threads);
}

rcl_interfaces::msg::SetParametersResult DepthObstacleDetectionNode::on_parameter_update(
        const std::vector<rclcpp::Parameter> &params
) {
    // NO CHANGES YET
    RCLCPP_DEBUG(this->get_logger(), "Parameter update received");
    for (const auto& param : params) {
        if (param.get_name() == "depth_scale") {
            this->config.depth_scale = param.as_double();
        } else if (param.get_name() == "camera_pitch") {
            this->config.camera_pitch = param.as_double();
        } else if (param.get_name() == "camera_x_offset") {
            this->config.camera_x_offset = param.as_double();
        } else if (param.get_name() == "camera_z_offset") {
            this->config.camera_z_offset = param.as_double();
        } else if (param.get_name() == "lane_width") {
            this->config.lane_width = param.as_double();
        } else if (param.get_name() == "z_low_filter") {
            this->config.z_low_filter = param.as_double();
        } else if (param.get_name() == "z_high_filter") {
            this->config.z_high_filter = param.as_double();
        } else if (param.get_name() == "x_high_filter") {
            this->config.x_high_filter = param.as_double();
        } else if (param.get_name() == "point_threshold") {
            this->config.point_threshold = param.as_int();
        } else if (param.get_name() == "num_threads") {
            this->config.num_threads = param.as_int();
        } 
    }

    post_init_occupancy_grid_params(&this->config);

    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    return result;
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DepthObstacleDetectionNode>());
    rclcpp::shutdown();
    return 0;
}
