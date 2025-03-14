#ifndef TOKYODRIFT_LANE_DETECTION_HELPER_H
#define TOKYODRIFT_LANE_DETECTION_HELPER_H

#include <stdint.h>
#include <opencv2/opencv.hpp>
#include <cmath>
#include <iostream>
#include <vector>
#include <string>
#include <iomanip>
#include <algorithm>
#include <numeric>

#include "geometry_msgs/msg/point.hpp"

enum CurrentLine{
    RIGHT_LINE,
    LEFT_LINE
};

enum CurrentLane{
    RIGHT,
    LEFT
};

typedef struct {
    uint16_t x_low;
    uint16_t x_high;
    uint16_t y_low;
    uint16_t y_high;
} WindowBounds;

/**
 * Walks from a point in a direction for a certain length.
 *
 * @param point             The point from where to start.
 * @param gradient          The gradient where to go.
 * @param length            The length of how long to walk on the gradient.
 * @param current_Lane      The specifier if it is right or left lane.
 * @return                  A new Point.
 */
inline cv::Point walk_gradient_x_y(const cv::Point& point, float gradient, float length, CurrentLine current_Lane) {
    if (current_Lane == CurrentLine::RIGHT_LINE) {
        gradient = -gradient;
    }
    // Determine the sign of the gradient
    float normalized_gradient = std::abs(gradient);
    float delta_y = std::round(length / std::sqrt(1 + normalized_gradient * normalized_gradient));
    float delta_x = normalized_gradient * delta_y;

    if (current_Lane == CurrentLine::RIGHT_LINE) {
        delta_x = -delta_x;
    }

    int new_x = static_cast<int>(point.x - delta_x);

    int new_y;
    if (gradient > 0) {
        new_y = static_cast<int>(point.y + delta_y);
    } else {
        new_y = static_cast<int>(point.y - delta_y);
    }

    cv::Point new_point = cv::Point(new_x, new_y);
    return new_point;
}


/**
 * Gets the lower and upper bounds of a midpoint. Returns zero if it becomes negative.
 *
 * @param mid_value   The middle of the bounds.
 * @param size    The size of the bounds.
 * @return        A tuple of the lower and the upper bound.
 */
inline std::tuple<uint16_t, uint16_t> get_low_and_high(uint16_t mid_value, uint16_t size) {
    uint16_t half_size = size / 2;
    uint16_t low_val;
    uint16_t high_val;
    if (mid_value < half_size) {
        low_val = 0;  // Clamp at 0
        high_val = mid_value + half_size;  // Extend the upper bound
    } else {
        low_val = mid_value - half_size;  // Normal case
        high_val = mid_value + half_size;  // Extend the upper bound
    }
    return {low_val, high_val};
}


/**
 * Creates a Window where it specifies the lower and upper bounds for x and y
 *
 * @param point   The middle point of the window.
 * @param width   The width of the window.
 * @param height  The height of the window.
 * @return        WindowBounds with {x_low, x_high, y_low, y_high}.
 */
inline WindowBounds create_window_bounds(const cv::Point& point, uint16_t width, uint16_t height) {
    auto [x_low, x_high] = get_low_and_high(point.x, width);
    auto [y_low, y_high] = get_low_and_high(point.y, height);
    return {x_low, x_high, y_low, y_high};
}


/**
 * Checks if a given Window is outside of the image
 *
 * @param window       The window to check.
 * @param image_size   The cv image size (width, height).
 * @return             True if the window outside of the image, False otherwise.
 */
inline bool window_out_of_image(const WindowBounds& window, const cv::Size& image_size) {
    return (
        window.x_low > image_size.width || 
        window.x_high > image_size.width || 
        window.y_low > image_size.height || 
        window.y_high > image_size.height
    );
}

inline uint16_t calculate_mean(const std::vector<uint16_t>& values) {
    if (values.empty()) {
        return 0;
    }
    uint64_t sum = std::accumulate(values.begin(), values.end(), static_cast<uint64_t>(0));
    return static_cast<uint16_t>(sum / values.size());;
}


inline float discrete_gradient(const cv::Point& new_point, const cv::Point& orig_point){
    if (new_point.y == orig_point.y) {return 0.0f;}
    return static_cast<float>(new_point.x - orig_point.x) / static_cast<float>(new_point.y - orig_point.y);
}


inline std::vector<cv::Point2f> sample_lane_points(const std::vector<cv::Point2f>& lane_points, size_t num_points) {
    std::vector<cv::Point2f> sampled_points;

    if (lane_points.empty() || num_points == 0) {
        return sampled_points;
    }

    size_t step = std::max(static_cast<size_t>(1), lane_points.size() / num_points);
    for (size_t i = 0; i < lane_points.size(); i += step) {
        sampled_points.push_back(lane_points[i]);
        if (sampled_points.size() >= num_points) {
            break;
        }
    }

    return sampled_points;
}


inline std::string pointsToString(const std::vector<cv::Point>& points) {
    std::ostringstream oss;
    oss << "[";
    for (size_t i = 0; i < points.size(); ++i) {
        oss << "(" << points[i].x << ", " << points[i].y << ")";
        if (i < points.size() - 1) {
            oss << ", ";
        }
    }
    oss << "]";
    return oss.str();
}


inline std::vector<geometry_msgs::msg::Point> cv2geo_points(
    const std::vector<cv::Point2f>& cv_points
) {
    std::vector<geometry_msgs::msg::Point> geo_points;
    for (const cv::Point2f& point : cv_points) {
        geometry_msgs::msg::Point msg_point;
        msg_point.x = point.x;
        msg_point.y = point.y;
        geo_points.push_back(msg_point);
    }
    return geo_points;
}


#endif // TOKYODRIFT_LANE_DETECTION_HELPER_H