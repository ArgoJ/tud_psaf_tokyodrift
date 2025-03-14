#include "../include/object_detector.h"
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/search/kdtree.h>

#include "math.h"

/**

    @brief Object detection and segmentation using point cloud processing.



    This class processes 3D point clouds using filtering, clustering,

    and segmentation techniques to detect objects in the environment.
    /
    class ObjectDetector : public rclcpp::Node {
    public:
    /*

    @brief Constructor for ObjectDetector.



    Initializes ROS publishers and subscribers for point cloud processing.
*/
ObjectDetector::ObjectDetector() : Node("objectDetector"){
    // ROS SUBSCRIBER
    this->subDepth = this->create_subscription<sensor_msgs::msg::Image>("/depth/image", 5, std::bind(&ObjectDetector::plane_segmentation, this, std::placeholders::_1));
    // ROS PUBLISHER
    this->pubPCcolor = this->create_publisher<sensor_msgs::msg::PointCloud2>("/objects", 5);
    this->pubPCobj = this->create_publisher<sensor_msgs::msg::PointCloud2>("/objectDetector/objects", 5);
    this->pubPCplane = this->create_publisher<sensor_msgs::msg::PointCloud2>("/objectDetector/plane", 5);
    this->pubPCvox = this->create_publisher<sensor_msgs::msg::PointCloud2>("/objectDetector/voxel", 5);
}

/**

    @brief Computes the distance from a point to a plane.



    Uses the plane equation to calculate perpendicular distance.



    @param point Input point.

    @param coefficients Plane coefficients.

    @return Distance from point to plane.
*/

double ObjectDetector::distance_point2plane(pcl::PointXYZ point, pcl::ModelCoefficients::Ptr coefficients){
    double f1 = fabs(coefficients->values[0]*point.x+coefficients->values[1]*point.y+coefficients->values[2]*point.z+coefficients->values[3]);
    double f2 = sqrt(pow(coefficients->values[0],2)+pow(coefficients->values[1],2)+pow(coefficients->values[2],2));
    return f1/f2;
}

/**

    @brief Computes the tilt angle of a plane.



    Determines the angle between the plane normal and the vertical axis.



    @param coefficients Plane coefficients.

    @return Tilt angle in radians.
*/

double ObjectDetector::angle_tilt(pcl::ModelCoefficients::Ptr coefficients){
    double tilt = acos(coefficients->values[2] / (sqrt(pow(coefficients->values[0],2)+pow(coefficients->values[1],2)+pow(coefficients->values[2],2))));
    if (tilt > M_PI){
        tilt = tilt - 2*(tilt - M_PI);
    }
    return tilt;
}

/**

    @brief Applies voxel grid filtering to downsample the point cloud.



    Reduces the number of points while preserving the overall structure.



    @param cloud Input point cloud.

    @param out Filtered output point cloud.
*/
void ObjectDetector::voxel_grid_filter(pcl::PCLPointCloud2::Ptr cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr out){

    if(_debug_voxel){
        RCLCPP_INFO(this->get_logger(), "VOXELGRID: Point Cloud size before filtering: %d", cloud->width * cloud->height);
    }

    pcl::PCLPointCloud2::Ptr cloudVox(new pcl::PCLPointCloud2());
    pcl::VoxelGrid<pcl::PCLPointCloud2> vg;
    vg.setInputCloud(cloud);
    vg.setLeafSize(0.01f, 0.01f, 0.01f); // downsample parameter
    vg.filter(*cloud);

    pcl::fromPCLPointCloud2(*cloud, *out);

    if(_debug_voxel){
        publish_pointcloud(*out, pubPCvox);
        RCLCPP_INFO(this->get_logger(), "VOXELGRID: Point Cloud size after filtering: %d", cloud->width * cloud->height);
    }
}

/**

    @brief Performs Euclidean clustering to segment objects.



    Groups nearby points into clusters representing individual objects.



    @param cloud Input point cloud.
*/

void ObjectDetector::clustering(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud){

    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(cloud);

    std::vector<pcl::PointIndices> clusterIndices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(0.2); // m 
    ec.setMinClusterSize(100);
    ec.setMaxClusterSize(25000); // may be 2 big
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    ec.extract(clusterIndices);

    if(_debug_clustering){
        RCLCPP_INFO(this->get_logger(), "CLUSTERING: %li number of clusters found", clusterIndices.size());
        publish_pointcloud(*cloud, pubPCvox);
    }

    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> obj;

   

    int j = 0;
    for (const auto& cluster : clusterIndices){
        double maxHeight = 2; // m

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloudCluster (new pcl::PointCloud<pcl::PointXYZ>);

        int n = 0;
        bool isObj = true;
        for (const auto& idx : cluster.indices) {
            cloudCluster->push_back((*cloud)[idx]);
            if (n != 0){
                for (u_int32_t i = 0; i < cloudCluster->width * cloudCluster->height - 1; i++){
                    double height = abs((*cloudCluster)[i].z - (*cloud)[idx].z);
                    if (maxHeight < height) {
                        isObj = false;
                        break;
                    }
                }
            }
            n++;
        }

        if (isObj) obj.push_back(cloudCluster);
        

        if(_debug_clustering){
            RCLCPP_INFO(this->get_logger(), "CLUSTERING: Cluster %i includes %i Points." , j, cloudCluster->width*cloudCluster->height);
        }
        j++;
    }

    if(_debug_clustering){
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudObj(new pcl::PointCloud<pcl::PointXYZRGB>);
        int j = 0;
        for (const auto& c : obj){
            for (u_int32_t i = 0; i < c->height * c->width; i++){
                pcl::PointXYZRGB pointColor;
                pcl::PointXYZ point =  c->points[i];
                pointColor.x = point.x;
                pointColor.y = point.y;
                pointColor.z = point.z; 
                std::uint8_t r = std::min(255, 10 * (j % 3 + j));
                std::uint8_t g = std::min(255, 10 * ((j + 1) % 3 + j));
                std::uint8_t b = std::min(255, 10 * ((j + 2) % 3 + j));
                std::uint32_t rgb = ((std::uint32_t)r << 16 | (std::uint32_t)g << 8 | (std::uint32_t)b);
                pointColor.rgb = *reinterpret_cast<float*>(&rgb);
                cloudObj->push_back(pointColor);
            }
            j++;
        }
        publish_pointcloud_rgb(*cloudObj, pubPCobj);
        project_obstacles(obj);
    }
}

/**

    @brief Segments ground planes using RANSAC.



    Iteratively fits planes to remove ground points and extract obstacles.



    @param cloud Input point cloud.
*/

void ObjectDetector::ransac_ground_filter(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud){

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOut(new pcl::PointCloud<pcl::PointXYZ>);
    // Segment objects
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::ExtractIndices<pcl::PointXYZ> extract;

    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(0.1); // Tolerance for point-to-plane distance

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudVis(new pcl::PointCloud<pcl::PointXYZRGB>);
    std::vector<double> tiltAngles;

    int n_planes = 0;
    int originalSize = cloud->size();

    // Looping over pointcloud fitting planes until size below _minPercentage
    while(cloud->height*cloud->width > originalSize * _minPercentage){
        seg.setInputCloud(cloud);
        seg.segment(*inliers, *coefficients);


        tiltAngles.push_back(angle_tilt(coefficients));

        if (inliers->indices.size() == 0){
            RCLCPP_ERROR(this->get_logger(), "RANSAC: Inlier list empty");
            break;
        }
    

        if (_debug_ransac){
            // Compute metrics for inliers
            double meanError = 0, maxError = 0;
            double minError = 10000;
            std::vector<double> error;
            for (int i = 0; i <inliers->indices.size(); i++){
                // Get point
                pcl::PointXYZ point = cloud->points[inliers->indices[i]];
                
                // Compute distance 
                double dist = distance_point2plane(point, coefficients) * 1000; // mm
                error.push_back(dist);

                // Update metrics
                meanError += dist;
                if(dist > maxError) maxError = dist;
                if(dist < minError) minError = dist;
            }
            meanError /= inliers->indices.size();

            double sigma  = 0;
        
            for (int i = 0; i <inliers->indices.size(); i++){

                sigma += pow(error[i] - meanError, 2);
                
                pcl::PointXYZ point = cloud->points[inliers->indices[i]];

                pcl::PointXYZRGB pointColor;
                pointColor.x = point.x;
                pointColor.y = point.y;
                pointColor.z = point.z; 
                std::uint8_t r = std::min(255, 10 * (n_planes % 3 + n_planes));
                std::uint8_t g = std::min(255, 10 * ((n_planes + 1) % 3 + n_planes));
                std::uint8_t b = std::min(255, 10 * ((n_planes + 2) % 3 + n_planes));
                std::uint32_t rgb = ((std::uint32_t)r << 16 | (std::uint32_t)g << 8 | (std::uint32_t)b);
                pointColor.rgb = *reinterpret_cast<float*>(&rgb);
                cloudVis->push_back(pointColor);

            }
            sigma = sqrt(sigma/inliers->indices.size());

            RCLCPP_INFO(this->get_logger(), "RANSAC: fitted plane %i: %fx%s%fy%s%fz%s%f=0 (inliers: %zu/%i)",
                     n_planes,
                     coefficients->values[0],(coefficients->values[1]>=0?"+":""),
                     coefficients->values[1],(coefficients->values[2]>=0?"+":""),
                     coefficients->values[2],(coefficients->values[3]>=0?"+":""),
                     coefficients->values[3],
                     inliers->indices.size(),originalSize);
            RCLCPP_INFO(this->get_logger(), "RANSAC: tilt angle: %f (rad)", tiltAngles.back());
            RCLCPP_INFO(this->get_logger(), "RANSAC: mean error: %f(mm), standard deviation: %f (mm), max error: %f(mm)",meanError,sigma,maxError);
            RCLCPP_INFO(this->get_logger(), "RANSAC: points left in cloud %i",cloud->width*cloud->height);
        }

        // Planes with certain tilt classify as ground and can be ignored
        if(2.2 < angle_tilt(coefficients) ||  angle_tilt(coefficients)  <1.8){
            for (int i = 0; i <inliers->indices.size(); i++){
                pcl::PointXYZ point = cloud->points[inliers->indices[i]];
                point.x = point.x;
                point.y = point.y;
                point.z = point.z;
                cloudOut->push_back(point);
            }
        }

        extract.setInputCloud(cloud);
        extract.setIndices(inliers);
        extract.setNegative(true);
        pcl::PointCloud<pcl::PointXYZ> cloudF;
        extract.filter(cloudF);
        cloud->swap(cloudF);
        n_planes++;
    }
    if (_debug_ransac) RCLCPP_INFO(this->get_logger(), "RANSAC: Number of found planes %i", n_planes);
    publish_pointcloud_rgb(*cloudVis, pubPCcolor);
    publish_pointcloud(*cloudOut, pubPCplane);
    cloud = cloudOut;
}

/**

    @brief Projects detected obstacles onto a 2D plane.



    Computes the centroids and bounding radii of detected objects. Planned use as input for colision avoidance MPC.



    @param objects Vector of detected object clusters.
*/

void ObjectDetector::project_obstacles(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> objects){
    std::vector<double> radii;
    std::vector<Eigen::Vector4f> centroids;
    for (int i = 0; i < objects.size(); i++){
        Eigen::Vector4f centroid;
        pcl::compute3DCentroid(*objects[i], centroid);
        centroids.push_back(centroid);
        radii.push_back(get_radius(objects[i]));
        
    }
}

/**

    @brief Computes the radius of an object cluster.



    Finds the maximum distance between points in a cluster.



    @param cloud Input point cloud cluster.

    @return Maximum radius of the cluster.
*/

double ObjectDetector::get_radius(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud){
    double maxDist = 0;
    for (int i = 0; i < cloud->width * cloud->height; i++){
        for (int j = 0; j < cloud->width * cloud->height; j++){
            if (i == j) continue;
            pcl::PointXYZ p1 = cloud->points[i], p2 = cloud->points[j];
            double curDist = std::sqrt(std::pow(p1.x - p2.x, 2) + std::pow(p1.y -p1.y, 2));
            if (maxDist < curDist) maxDist = curDist;
        }
    }
    return maxDist;
}

/**

    @brief Converts depth image to point cloud



    Uses pinhole camera model to compute point cloud from depth image.



    @param in Input image
    @return Point cloud
*/
pcl::PCLPointCloud2::Ptr ObjectDetector::depth_image2pc(sensor_msgs::msg::Image::SharedPtr img){
    pcl::PCLPointCloud2::Ptr cloud(new pcl::PCLPointCloud2);

    return cloud;
}

/**

    @brief Processes an incoming point cloud message.



    Applies voxel filtering, ground segmentation, and clustering.



    @param in Input point cloud message.
*/
void ObjectDetector::plane_segmentation(const sensor_msgs::msg::Image::SharedPtr in){

    pcl::PCLPointCloud2::Ptr cloudIn = depth_image2pc(in);

   


    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    // Voxel grid filter
    voxel_grid_filter(cloudIn, cloud);

    // RANSCAC-based ground filter
    ransac_ground_filter(cloud);

    // Clustering
    clustering(cloud);
    
}

/*
    UTILITY
*/


/**

    @brief Publishes a point cloud message.



    Converts a PCL point cloud to ROS format and publishes it.



    @param pc Input point cloud.

    @param pub ROS publisher.
*/

void ObjectDetector::publish_pointcloud(pcl::PointCloud<pcl::PointXYZ> pc, rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub){
    sensor_msgs::msg::PointCloud2 out_msg;
    pcl::toROSMsg(pc, out_msg);
    out_msg.header.frame_id = "map";
    out_msg.header.stamp = this->now();
    pub->publish(out_msg);
}

/**

    @brief Publishes a colored point cloud message.



    Converts a PCL RGB point cloud to ROS format and publishes it.



    @param pc Input RGB point cloud.

    @param pub ROS publisher.
*/

void ObjectDetector::publish_pointcloud_rgb(pcl::PointCloud<pcl::PointXYZRGB> pc, rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub){
    sensor_msgs::msg::PointCloud2 out_msg;
    pcl::toROSMsg(pc, out_msg);
    out_msg.header.frame_id = "map";
    out_msg.header.stamp = this->now();
    pub->publish(out_msg);
}

int main(int argc, char *argv[]) {
}