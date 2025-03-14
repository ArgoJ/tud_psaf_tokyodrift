/**
 * @file path_finder.cpp
 * @brief Path finding and occupancy map generation for navigation tasks.
 * 
 * This file contains the implementation of the PathFinder class, which handles 
 * the functionality for generating paths using A* algorithm and creating occupancy maps 
 * from point cloud data.
 */

#include "../include/path_finder.h"

/**
 * @brief Finds the optimal path from the start point to the goal using the A* algorithm.
 *
 * This method implements the A* pathfinding algorithm to find the shortest path
 * between the start point and the goal point on the occupancy grid.
 * It uses a priority queue to explore the grid and stores the best path by tracking
 * the nodes' "came from" points.
 *
 * @param goal The destination point that we are trying to reach.
 * @return A vector of Points representing the path from start to goal.
 */
std::vector<Point> PathFinder::find_path(Point goal){

    int rows = grid.size(), cols = grid[0].size();
    Point 

    // Directions to move in the grid: Up, Down, Right, Left.
    std::vector<Point> moves = {{-1, 0}, {1,0}, {0,1}, {0,-1}};

    std::vector<Point> path;

    // Open set for A* algorithm with a priority queue.
    std::priority_queue<NodeS, std::vector<NodeS>, std::greater<NodeS>> openSet;
    
    // Maps to track the "came from" node and g-scores (cost from start).
    std::unordered_map<Point, Point, PointHash> cameFrom;
    std::unordered_map<Point, int, PointHash> gScore;

    // Initial push of the starting node into the open set.
    openSet.push({heuristic(start, goal), 0, start});
    gScore[start] = 0;

    // Main loop of A* algorithm
    while (!openSet.empty()){
        NodeS current = openSet.top();
        openSet.pop();

        // If the goal is reached, reconstruct the path.
        if (current.position == goal){
            for (Point at = goal; cameFrom.find(at) != cameFrom.end(); at = cameFrom[at]){
                path.push_back(at);
            }
            std::reverse(path.begin(), path.end());
            return path;
        }

        // Check the 4 neighbors (up, down, left, right).
        for (auto move : moves){
            Point neighbor = {current.position.first + move.first, current.position.second + move.second};

            // If the neighbor is within bounds and is not an obstacle.
            if (neighbor.first >= 0 && neighbor.first < rows && neighbor.second >= 0 && neighbor.second < cols && grid[neighbor.first][neighbor.second] == 0){
                int newGscore = gScore[current.position] + 1;

                // If the new path is shorter, update the scores and add to the open set.
                if (!gScore.count(neighbor) || newGscore < gScore[neighbor]){
                    gScore[neighbor] = newGscore;
                    int fScore = newGscore + heuristic(neighbor, goal);
                    openSet.push({fScore, newGscore, neighbor});
                    cameFrom[neighbor] = current.position;
                } 
            }
        }
    }
    
    // Return the empty path if no path was found.
    return path;
}

/**
 * @brief Generates a path based on the clusters of point clouds.
 *
 * This function first creates the occupancy map from a collection of clusters
 * represented as point clouds and then computes the path from the start to the goal.
 *
 * @param clusters A vector of point cloud pointers representing obstacles and free space.
 * @return A vector of Points representing the computed path.
 */
std::vector<Point> PathFinder::get_path(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clusters){
    create_occupancy_map(clusters);  ///< Generate the occupancy grid from the clusters.
    return find_path();  ///< Find and return the path based on the occupancy grid.
}

/**
 * @brief Publishes the occupancy map visualization.
 *
 * This method converts the occupancy map into a format suitable for ROS to visualize
 * and publishes the result.
 *
 * @param occupancyMap The occupancy map to visualize, represented as a cv::Mat.
 */
void PathFinder::publish_occupancyMapVis(cv::Mat& occupancyMap){
    sensor_msgs::msg::Image::SharedPtr msg = cv_bridge::CvImage(std_msgs::msg::Header(), "8UC1", occupancyMap).toImageMsg();
    pubOccupancyMapVis->publish(*msg.get());
}

/**
 * @brief Creates an occupancy map from point cloud data.
 *
 * This method processes the point cloud clusters and generates an occupancy map.
 * The map is populated based on whether points are considered obstacles or not.
 * It converts the 3D point cloud coordinates into 2D grid coordinates, taking into 
 * account the map resolution and ground filter threshold.
 *
 * @param clusters A vector of point cloud pointers to process and generate the map.
 */
void PathFinder::create_occupancy_map(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clusters){
    int cols = static_cast<int>(mapWidth / resolution);
    int rows = static_cast<int>(mapHeight / resolution);

    std::vector<std::vector<int>> occupancyMap;

    // Process each point cloud cluster.
    for (auto pc : clusters){
        for (int i = 0; i < pc->data.size(); i += 4){
            // Skip points below the ground filter threshold.
            if(pc->data[i + 2] < this->groundFilter){
                continue;
            }
            // Convert point cloud 3D coordinates to 2D grid coordinates.
            int col = static_cast<int>(((pc->data[i] + mapWidth) / 2) / resolution);
            int row = static_cast<int>(((pc->data[i+1] + mapWidth) / 2) / resolution);

            // Mark the corresponding grid cell if it's within bounds.
            if (col >= 0 && col < cols && row >= 0 && row < rows){
                (occupancyMap.at(row)).at(col);
            }
        }
    }
    // Store the generated occupancy map in the grid.
    this->grid = occupancyMap;
}
