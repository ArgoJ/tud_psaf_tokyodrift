#ifndef TOKYODRIFT_LANE_DETECTION_H
#define TOKYODRIFT_LANE_DETECTION_H

#include <vector>
#include <tuple>
#include <optional>
#include <unordered_set>
#include <omp.h>
#include <opencv2/opencv.hpp>

#include "timer.hpp"
#include "lane_detection_config.h"
#include "lane_detection_helper.hpp"
#include "interpolation.h"


namespace LaneTests {
    class LaneDetectionTest;
}


namespace std {
    template<>
    struct hash<cv::Point> {
        size_t operator()(const cv::Point& p) const {
            return hash<int>()(p.x) ^ (hash<int>()(p.y) << 1);
        }
    };
}

class LaneDetection {
private:
    friend class LaneTests::LaneDetectionTest;
    const LaneDetectionParams& config;
    std::unique_ptr<Interpolation> interpolator;
    cv::Mat debug_image;
    int next_x_init_offset;
    double curve_sharpness;
    CurrentLane current_lane_;

public:
    LaneDetection(const LaneDetectionParams& config);

    // no default constructor
    LaneDetection() = delete;

    // Prevent copying
    LaneDetection(const LaneDetection&) = delete;
    LaneDetection& operator=(const LaneDetection&) = delete;


    /**
     * @brief Processes an input image to detect line markings and compute the left, right, and middle lanes.
     * 
     * This function performs the following steps:
     * Converts the input image to a binary image highlighting line features using `get_binary()`.  
     * Applies a bird's-eye transformation to the binary image using `get_bird_view()` to simplify line detection.  
     * Detects line points for the left and right lanes using the sliding window algorithm in `sliding_window()`.  
     * Interpolates detected line points into continuous lines for the left and right lanes using `interpolate_line_points()`.  
     * Computes the middle line by averaging the left and right lanes using `find_middle_lane()`.  
     * 
     * @param image The input image (in OpenCV `cv::Mat` format) to be processed for line detection.
     * 
     * @return `tuple` containing three vectors:
     * - `left_fit`: Points representing the left line.
     * - `right_fit`: Points representing the right line.
     * - `mid_fit`: Points representing the middle line calculated as the average of left and right lanes.
     */
    std::tuple<std::vector<cv::Point>, std::vector<cv::Point>, std::vector<cv::Point>> process_image(
        const cv::Mat& image,
        const CurrentLane current_lane
    );


    /**
     * @brief This function is a getter for the debug image
     * 
     * @return The debug image for publishing
     */
    cv::Mat get_debug_image();


    std::vector<cv::Point> interpolate_line_points(
        const std::vector<cv::Point>& line
    );


    /**
     * @brief Converts the given points of the image into
     * ego coordinates.
     * 
     * @return The vector of points in meters
     */
    std::vector<cv::Point2f> px2ego_m(
        const std::vector<cv::Point>& lane_fit_px
    );


private:
    /**
     * @brief Converts an input image to edge-detected format using Canny edge detection.
     * 
     * This function processes the input image to highlight edges by performing the following steps:
     * Converts the input image from BGR to grayscale using `cv::cvtColor()`.
     * Applies a Gaussian blur to the grayscale image using `cv::GaussianBlur()` to reduce noise.
     * Uses the Canny edge detector (`cv::Canny()`) to identify edges in the blurred image.
     * 
     * @param image The input color image (BGR format).
     * 
     * @return `cv::Mat` containing the detected edges in binary format (white pixels represent edges).
     */
    cv::Mat get_edges(const cv::Mat& image); 


    /**
     * @brief Converts the input image to a binary image using adaptive thresholding.
     * 
     * This function first converts the input image to grayscale, then applies Gaussian 
     * blur to reduce noise, and finally uses adaptive thresholding to create a binary image. 
     * The parameters for the adaptive thresholding are configured using class member variables.
     * 
     * @param image The input color image (BGR format).
     * @return `cv::Mat` containing the resulting binary image.
     */
    cv::Mat get_binary(const cv::Mat& image); 


    /**
     * @brief Converts the input image to a binary image using parallel adaptive thresholding.
     * 
     * This function first converts the input image to grayscale, then applies Gaussian 
     * blur to reduce noise, and finally uses adaptive thresholding to create a binary image. 
     * The parameters for the adaptive thresholding are configured using class member variables.
     * 
     * @param image The input color image (BGR format).
     * @return `cv::Mat` containing the resulting binary image.
     */
    cv::Mat parallel_adaptive_thresholding(const cv::Mat& image); 


    /**
     * @brief Applies a perspective transform to the input image to generate a bird's-eye view.
     * 
     * This function selects a region of interest (ROI) from the input image and applies a 
     * perspective transformation to map it into a bird's-eye view. The transformation is 
     * defined by source and destination points, with the configuration parameters used to 
     * determine the ROI and the size of the output bird's-eye view image.
     * 
     * @param image The input image.
     * @return `cv::Mat` The resulting bird's-eye view image.
     */
    cv::Mat get_bird_view(const cv::Mat& image);


    /**
     * @brief Detects line points using the sliding window approach.
     * 
     * This function detects left and right line points in a binary image by moving a sliding 
     * window from the bottom to the top of the image. It starts by looking for initial points 
     * and then tracks the line points in each window. The function adjusts the points and 
     * checks for line validity based on previously found points and a tracker system. If both 
     * lanes are detected and solid, it optionally draws the detected points on the image.
     * 
     * @param image The binary bird's-eye view image where the line detection is performed.
     * @return `std::tuple<std::vector<cv::Point>, std::vector<cv::Point>>` A tuple containing two vectors of detected line points: 
     *         left line points and right line points.
     */
    std::tuple<std::vector<cv::Point>, std::vector<cv::Point>> sliding_window(
        const cv::Mat& image
    );
    
    
    /**
     * @brief Checks the window edges in the given image for line pixels.
     * 
     * This function creates a window around the specified point and finds the pixels 
     * that fall within the window in the provided image. It checks if the window is 
     * within the image boundaries and draws the window on a temporary image for visualization 
     * if enabled. The function returns the pixel positions found within the window.
     * 
     * @param image The image in which the window edges are checked for line pixels.
     * @param point The point around which the window is created for edge detection.
     * @return `std::vector<uint16_t>` A vector of pixel positions (in column indices) within the window.
    */
    std::vector<uint16_t> check_window_edges(
        const cv::Mat& image,
        const cv::Point& point
    ); 


    /**
     * @brief Recursively checks for line pixels in a moving window.
     * 
     * This function recursively checks for line pixels within a moving window centered 
     * around a given point. If the number of pixels found within the window exceeds a 
     * specified threshold, the window's center is updated to the mean position of those pixels, 
     * and the search continues. The recursion stops when the number of pixels found is below 
     * a minimum threshold or the last number of found pixes, or if the window is invalid.
     * 
     * @param image The image in which the window edges are checked for line pixels.
     * @param point The current point around which the window is centered.
     * @param min_pixels The minimum number of pixels required to continue the recursion.
     * @return `std::vector<uint16_t>` A vector of pixel positions within the window.
     */
    std::vector<uint16_t> recursive_window_check(
        const cv::Mat& image, 
        const cv::Point& point, 
        const uint16_t min_pixels
    ); 


    /**
     * @brief Checks the window for line pixels and returns the new point if valid.
     * 
     * This function checks for line pixels within a moving window centered around a given point 
     * using recursive window checking. If the number of pixels found in the window exceeds a 
     * minimum threshold, it calculates the mean position of those pixels and returns a new point 
     * for the next search. If the number of pixels is below the threshold, it returns an empty optional.
     * 
     * @param image The image in which the window is checked for line pixels.
     * @param point The current point around which the window is centered.
     * @return `std::optional<cv::Point>` A new point for the next search, or an empty optional if the search is unsuccessful.
     */
    std::optional<cv::Point> check_window(
        const cv::Mat& image,
        const cv::Point& point
    ); 


    /**
     * @brief Searches for a line point in a sequence of shifted windows.
     * 
     * This function attempts to find a line point by shifting the search window in the x-direction 
     * based on the current line and whether a line was previously found. It iterates through a 
     * sequence of potential window positions and returns the first valid point found. 
     * If no valid points are found after all iterations, it returns an empty optional.
     * 
     * @param image The image in which to search for the line points.
     * @param point The initial point around which the window search starts.
     * @param current_line The line being searched (left or right).
     * @param line_found A flag indicating whether a line was previously found.
     * @return `std::optional<cv::Point>` The next valid line point if found, or an empty optional if not.
     */
    std::optional<cv::Point> check_window_sequence(
        const cv::Mat& image, 
        const cv::Point& point,
        CurrentLine current_line,
        const bool line_found
    ); 


    /**
     * @brief Attempts to find a line point based on the current and other line points.
     * 
     * This function searches for a valid line point in the image by adjusting the search point
     * based on various conditions, such as gradient information from the current line or the other line.
     * It iterates through different strategies for locating the line, including shifting the search window 
     * and using the gradient information of the neighboring line when necessary.
     * 
     * @param image The image in which to search for the line points.
     * @param init_point The initial search point.
     * @param line_points The points of the line being searched.
     * @param other_line_points The points of the other line, used when considering the other gradient.
     * @param current_line The current line being searched (left or right).
     * @param use_other_grad A flag indicating whether to use the other line's gradient.
     * @param is_line_point A reference flag that will be set to true if a valid line point is found.
     * @return `cv::Point` The found line point, or the predicted point if no valid point is found.
     */
    cv::Point find_line(
        const cv::Mat& image, 
        const cv::Point& init_point, 
        std::vector<cv::Point>& line_points, 
        std::vector<cv::Point>& other_line_points,
        CurrentLine current_line,
        const bool use_other_grad,
        bool& is_line_point
    ); 


    /**
     * @brief Predicts the next line point based on the current gradient of the other line.
     * 
     * This function predicts the position of a line point by calculating the gradient between the 
     * current point and the previous point of the other line. 
     * It then adjusts the x-coordinate using the gradient information.
     * 
     * @param point The current other line point.
     * @param prev_point The previous other line point used to calculate the gradient.
     * @param current_line The current line being considered (left or right).
     * @return `cv::Point` The predicted point of the current line.
     */
    cv::Point predict_point_with_other_line(
        const cv::Point& point,
        const cv::Point& prev_point,
        CurrentLine current_line
    ); 


    /**
     * @brief Adjusts the x-coordinate of a point based on a specified shift for the current line.
     * 
     * This function adjusts the x-coordinate of a given point by applying a horizontal shift 
     * based on the specified line (left or right). The y-coordinate remains unchanged.
     * The function is typically used to predict the next line point by shifting the current point 
     * in the appropriate direction based on the line.
     * 
     * @param point The current line point to adjust.
     * @param current_line The current line being considered (left or right).
     * @param x_shift The amount to shift the x-coordinate.
     * @return `cv::Point` The adjusted line point with the new x-coordinate.
     */
    cv::Point adjust_point_with_shift(
        const cv::Point& point, 
        CurrentLine current_line
    ); 


    /**
     * @brief Adjusts the x-coordinate of a point based on the gradient between two previous points.
     * 
     * This function calculates the gradient between two previous points (prev1_point and prev2_point) 
     * and uses this gradient to predict the new x-coordinate for the given point at the same y-coordinate. 
     * The y-coordinate of the point remains unchanged.
     * 
     * @param point The current point for which the x-coordinate is to be adjusted.
     * @param prev1_point The most recent point used for gradient calculation.
     * @param prev2_point The second most recent point used for gradient calculation.
     * @return `cv::Point` The adjusted point with the new x-coordinate based on the calculated gradient.
     */
    cv::Point adjust_point_with_gradient(
        const cv::Point& point,
        const cv::Point& prev1_point, 
        const cv::Point& prev2_point
    ); 


    /**
     * @brief Validates whether a point is suitable for inclusion in a line based on various checks.
     * 
     * This function checks if the point is within a valid region by evaluating:
     * If it lies within a cone in front of the most recent line points.
     * If it is outside of a safe distance from the points of the other line.
     * Depending on the current line (LEFT_LINE or RIGHT_LINE), it checks if the point's x-coordinate 
     *    follows the expected pattern (e.g., should the point be to the left or right of the other line's points).
     * 
     * @param point The point to be validated.
     * @param line_points The points representing the current line.
     * @param other_line_points The points representing the other line for comparison.
     * @param current_line The line type (LEFT_LINE or RIGHT_LINE) the point is being validated for.
     * 
     * @return true if the point is valid, false otherwise.
     * @throws std::invalid_argument if an invalid line type is provided.
     */
    bool is_valid_point(
        const cv::Point& point,
        const std::vector<cv::Point>& line_points,
        const std::vector<cv::Point>& other_line_points,
        CurrentLine current_line
    ); 


    /**
     * @brief Checks if a point is outside of all given circles formed by a set of other points.
     * 
     * This function iterates over a list of points (other_points) and checks if the given point is outside 
     * the circles defined by each point in the list, with a radius specified by `safe_lane_distance`. 
     * If the point lies inside any of the circles, it returns false; otherwise, it returns true.
     * 
     * @param point The point to be checked.
     * @param other_points The set of points that define the circles.
     * @param radius The radius of the circle. 
     * @return true if the point is outside all of the circles, false if it is inside any of them.
     */
    bool is_outside_save_circles(
        const cv::Point& point, 
        const std::vector<cv::Point>& other_points
    ); 


    /**
     * Checks if a given point is outside a circle with a specified center and radius.
     *
     * @param point   The point to check.
     * @param center  The center of the circle.
     * @param radius  The radius of the circle.
     * @return        True if the point is outside the circle, False otherwise.
     */
    static bool is_outside_circle(
        const cv::Point& point, 
        const cv::Point& center, 
        uint16_t radius
    );


    /**
     * Checks if a given point is infront of a previous point inside a cone.
     *
     * @param point             The point to check.
     * @param previous_point    The previous point, where the cone starts.
     * @param gradient          The gradient of the previous points.
     * @return                  True if the point is inside the cone, False otherwise.
     */
    bool is_inside_curved_bounds(
        const cv::Point& point, 
        const cv::Point& previous_point, 
        float gradient
    );
    

    /**
     * @brief Retrieves the iteration values based on the current line and whether the line was found.
     * 
     * This function returns a vector of integers representing the iteration values for searching the line. 
     * The iteration values depend on whether the line has been found (line_found) and the current line type (LEFT_LINE or RIGHT_LINE).
     * 
     * @param current_line The line type (LEFT_LINE or RIGHT_LINE) for which iterations are being determined.
     * @param line_found A boolean indicating whether the line has been found previously.
     * 
     * @return `std::vector<int16_t>` A vector of iteration values for the current line type.
     * @throws std::invalid_argument If an invalid line type or boolean argument is provided.
     */
    std::vector<int16_t> get_iterations(
        CurrentLine current_line, 
        const bool line_found
    ); 


    /**
     * @brief Generates a sequence of iteration values for the searched line.
     * 
     * This function generates a vector of iteration values used to search for the line specified by current_line. 
     * It produces a symmetric range of values from -1 to `iters_each_side` and back, with alternating positive and negative values.
     * 
     * @param iter_each_side The number of iterations to generate on each side of the center (both positive and negative).
     * @param current_line The line (i.e. left or right) that should be searched.
     * 
     * @return `std::vector<int16_t>` A vector of iteration values to be used in the line detection process.
     */
    static std::vector<int16_t> get_iters(
        int16_t iters_each_side,
        CurrentLine current_line
    ); 


    bool is_between_lines(
        const cv::Point& point1,
        const cv::Point& point2, 
        const std::unordered_set<cv::Point>& left_set, 
        const std::unordered_set<cv::Point>& right_set
    );
    

    /**
     * @brief Computes the middle line using Delaunay triangulation of given left and right line points.
     *
     * This function performs Delaunay triangulation on the combined left and right line points,
     * and extracts the midpoints of triangles that lie between the lanes. These midpoints
     * are then sorted top-down based on the y-coordinate to represent the middle line.
     *
     * @param left_points A vector of `cv::Point` representing points on the left line.
     * @param right_points A vector of `cv::Point` representing points on the right line.
     *
     * @return `std::vector<cv::Point>` representing the calculated middle line points.
     */
    std::vector<cv::Point> find_lane_delaunay(
        const std::vector<cv::Point>& left_points, 
        const std::vector<cv::Point>& right_points
    ); 


    std::vector<cv::Point> find_lane_one_line(
        const std::vector<cv::Point>& line_points, 
        const CurrentLine current_line
    ); 


    /**
     * @brief Identifies non-zero pixel indices within a specified window in a binary image.
     * 
     * This function scans through a given rectangular window defined by `WindowBounds` 
     * within a binary image and collects the x-coordinates of all non-zero pixels.
     * 
     * @param image A binary `cv::Mat` where non-zero pixels represent edges or features.
     * @param window A `WindowBounds` object defining the rectangular region to search 
     *               (x_low, x_high, y_low, y_high).
     * 
     * @return A vector of `uint16_t` representing the x-coordinates of the non-zero 
     *         pixels found within the specified window. Returns an empty vector if 
     *         no pixels are found or the window is invalid.
     */
    static std::vector<uint16_t> find_pixels_in_window(
        const cv::Mat& edges, 
        const WindowBounds& window
    ); 


    /**
     * @brief Draws detected line points on a given image.
     * 
     * This function overlays circles on the specified image at the locations of 
     * the provided line points. Each circle represents a detected line point, 
     * facilitating visualization of the detected lanes.
     * 
     * @param line_points A vector of `cv::Point` objects representing the 
     *        coordinates of the detected line points.
     */
    void draw_points(
        const std::vector<cv::Point>& line_points,
        cv::Scalar color = cv::Scalar(0, 0, 255)
    );


    /**
     * @brief Draws line markings on the given image based on detected line points.
     * 
     * This function overlays the detected line markings (left, right, and middle lanes) on a copy 
     * of the provided image. Each line is drawn with a distinct color for visualization:
     * - Left line: Green
     * - Right line: Red
     * - Middle line: Yellow
     * 
     * @param image The input image (in OpenCV `cv::Mat` format) on which lanes will be drawn.
     * @param left_fit A vector of points representing the left line.
     * @param right_fit A vector of points representing the right line.
     * @param mid_fit A vector of points representing the middle line.
     * 
     * @return `cv::Mat` image with the line markings drawn.
     */
    cv::Mat draw_lanes_on_image(
        const cv::Mat& image, 
        const std::vector<cv::Point>& left_fit, 
        const std::vector<cv::Point>& right_fit, 
        const std::vector<cv::Point>& mid_fit
    );


    /**
     * @brief Predicts the x offset for the next initial guess based 
     *        on the gradients of the first five found points.
     * 
     * @param line_points A vector of `cv::Point` objects representing the 
     *        coordinates of the detected line points. 
     */
    void predict_next_x_init_offset(
        const std::vector<cv::Point>& line_points
    );


    /**
     * @brief Filter the line points based on a sliding window moving average 
     *        and variance approach. 
     * 
     * @param line_points The coordinates of the detected line points. 
     * @param window_size The window size of the moving average.
     * @param threshold_factor A threshold factor for the  standard deviation.
     */
    std::vector<cv::Point> moving_average_filter(
        const std::vector<cv::Point>& line_points,
        int window_size,
        float threshold_factor
    );

    std::vector<cv::Point> smooth_line_points(
        const std::vector<cv::Point>& points,
        float max_angle_diff = 30.0f,  // degrees
        float smoothing_factor = 0.5f
    );

    void determine_line(
        std::vector<cv::Point>& left_line_points,
        std::vector<cv::Point>& right_line_points,
        bool is_left_line_solid,
        bool is_right_line_solid,
        const size_t threshold
    );
};

#endif //TOKYODRIFT_LANE_DETECTION_H
