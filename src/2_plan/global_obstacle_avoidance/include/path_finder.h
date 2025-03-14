#pragma once

#include <rclcpp/rclcpp.hpp>
#include "sensor_msgs/msg/point_cloud2.hpp"

#include <sensor_msgs/msg/image.hpp>

#include <queue>
#include <unordered_map>
#include <cmath>

#include <opencv2/opencv.hpp>

struct Point {
    int first, second;
    bool operator==(const Point& other) const {
        return first == other.first && second == other.second;
    }
};

struct PointHash {
    std::size_t operator()(const Point& p) const {
        return std::hash<int>()(p.first) ^ (std::hash<int>()(p.second) << 1);
    }
};


struct NodeS {
    int f, g;
    Point position;
    bool operator>(const NodeS& other) const {
        return f > other.f;
    }
};

int heuristic(Point a, Point b) {
    return abs(a.first - b.first) + abs(a.second - b.second);
}

class PathFinder{
public:
    PathFinder();
    std::vector<std::vector<int>> grid;
    float groundFilter = 10;
    float resolution = 20;
    float mapWidth = 5;
    float mapHeight = 5; 

    void create_occupancy_map();
    std::vector<Point> find_path(std::vector<std::vector<int>>& grid, Point start, Point goal);

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharePtr subObj;
    rclcpp::Publisher<occupancy_map::msg::RefLine>::SharePtr pubRefLine;

};

