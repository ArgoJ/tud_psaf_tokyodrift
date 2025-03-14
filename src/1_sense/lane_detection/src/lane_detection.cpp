#include "lane_detection.h"


LaneDetection::LaneDetection(const LaneDetectionParams& config) 
    : config(config), 
        interpolator(std::make_unique<Interpolation>()), 
        next_x_init_offset(0), 
        current_lane_(CurrentLane::RIGHT) {}

std::tuple<std::vector<cv::Point>, std::vector<cv::Point>, std::vector<cv::Point>> LaneDetection::process_image(
        const cv::Mat& image,
        const CurrentLane current_lane
) {
    this->current_lane_ = current_lane;

    START_TIMER("Binary Thresholding")
    cv::Mat binary = this->get_binary(image);
    STOP_TIMER("Binary Thresholding")

    START_TIMER("Bird View")
    cv::Mat bird_view_binary = this->get_bird_view(binary);
    STOP_TIMER("Bird View")

    START_TIMER("Debug Image")
    if (this->config.show_opencv_images || this->config.pub_images) {
        this->debug_image = bird_view_binary.clone();
        cv::cvtColor(this->debug_image, this->debug_image, cv::COLOR_GRAY2BGR);
    }
    STOP_TIMER("Debug Image")

    START_TIMER("Sliding Window")
    auto [left_line_points, right_line_points] = this->sliding_window(bird_view_binary);
    STOP_TIMER("Sliding Window")

    std::vector<cv::Point> raw_lane_point;
    if (!left_line_points.empty() && right_line_points.empty()) {
        START_TIMER("Lane from Left")
        raw_lane_point = this->find_lane_one_line(left_line_points, CurrentLine::RIGHT_LINE);
        STOP_TIMER("Lane from Left")
    } else if (left_line_points.empty() && !right_line_points.empty()) {
        START_TIMER("Lane from Right")
        raw_lane_point = this->find_lane_one_line(right_line_points, CurrentLine::LEFT_LINE);
        STOP_TIMER("Lane from Right")
    } else if (!left_line_points.empty() && !right_line_points.empty()) {
        START_TIMER("Delaunay")
        raw_lane_point = this->find_lane_delaunay(left_line_points, right_line_points);
        STOP_TIMER("Delaunay")
    } 

    START_TIMER("Filter")
    std::vector<cv::Point> line_points;
    if (this->config.interpolate) {
        line_points = this->moving_average_filter(raw_lane_point, 5, 2.0);
    } else {
        line_points = this->smooth_line_points(raw_lane_point, 15.0, 0.5);
    }
    STOP_TIMER("Filter")

    START_TIMER("Predict x offset")
    this->predict_next_x_init_offset(line_points);
    STOP_TIMER("Predict x offset")

    START_TIMER("Visualization")
    if (this->config.show_opencv_images || this->config.pub_images) {
        this->draw_points(left_line_points);
        this->draw_points(right_line_points);
        this->draw_points(line_points, cv::Scalar(150, 120, 220));
    }
    if (this->config.show_opencv_images) {
        cv::imshow("Debug Image", this->debug_image);
        cv::waitKey(1);
    }
    STOP_TIMER("Visualization")

    return {left_line_points, right_line_points, line_points};
}


cv::Mat LaneDetection::get_debug_image() {
    return this->debug_image;
}


std::vector<cv::Point> LaneDetection::interpolate_line_points(const std::vector<cv::Point>& points) {
    return this->interpolator->interpolate_line_points(points, this->config.bird_view_height);
}


cv::Mat LaneDetection::get_edges(const cv::Mat& image) {
    cv::Mat output;
    cv::cvtColor(image, output, cv::COLOR_BGR2GRAY);
    cv::Mat blur;
    cv::GaussianBlur(output, blur, cv::Size(7, 7), 0);
    cv::Mat edges;
    cv::Canny(blur, edges, 100, 140);
    return edges;
}


cv::Mat LaneDetection::get_binary(const cv::Mat& image) {
    cv::Mat grey;
    cv::cvtColor(image, grey, cv::COLOR_BGR2GRAY);
    cv::Mat blur;
    cv::GaussianBlur(grey, blur, cv::Size(7, 7), 0);
    return this->parallel_adaptive_thresholding(blur);
}


cv::Mat LaneDetection::parallel_adaptive_thresholding(const cv::Mat& image) {
    cv::Mat output = cv::Mat::zeros(image.size(), image.type());
    int rows_per_thread = (image.rows - this->config.roi_height) / this->config.num_threads;

    #pragma omp parallel for
    for (int i = 0; i < this->config.num_threads; ++i) {
        int start_row = this->config.roi_height + i * rows_per_thread;
        int end_row = (i == this->config.num_threads - 1) ? image.rows : start_row + rows_per_thread;

        // Skip invalid ranges
        if (start_row >= end_row || start_row >= image.rows || end_row > image.rows) {
            continue; 
        }

        cv::Mat region = image.rowRange(start_row, end_row);
        cv::Mat thresholded_region;
        cv::adaptiveThreshold(
            region, 
            thresholded_region, 
            255,
            cv::ADAPTIVE_THRESH_GAUSSIAN_C, 
            cv::THRESH_BINARY, 
            this->config.binary_adaptive_box, 
            this->config.binary_adaptive_subtract
        );
        thresholded_region.copyTo(output.rowRange(start_row, end_row));
    }
    return output;
}


cv::Mat LaneDetection::get_bird_view(const cv::Mat& image) {
    cv::Mat bird_view;
    cv::warpPerspective(image, bird_view, this->config.transformation_matrix, cv::Size(this->config.bird_view_width, this->config.bird_view_height));
    return bird_view;
}


std::tuple<std::vector<cv::Point>, std::vector<cv::Point>> LaneDetection::sliding_window(const cv::Mat& image) {
    cv::Point left_point(this->config.left_init_x + this->next_x_init_offset, 0);
    cv::Point right_point(this->config.right_init_x + this->next_x_init_offset, 0);

    int part_height = this->config.bird_view_height * 1 / 3 ;

    std::vector<cv::Point> left_line_points;
    std::vector<cv::Point> right_line_points;
    int left_line_tracker = 0;
    int right_line_tracker = 0;
    bool is_right_line_solid = false;
    bool is_left_line_solid = false;

    // Loop over the image height, starting from the top and moving down in windows
    for (int y = this->config.bird_view_height - this->config.window_height / 2;
         y > this->config.bird_view_height - this->config.window_height * this->config.nwindows;
         y -= this->config.window_height) {

        right_point.y = y;
        left_point.y = y;

        // Right line
        bool is_right_line_point = false;
        right_point = this->find_line(image, right_point, right_line_points, left_line_points, CurrentLine::RIGHT_LINE, false, is_right_line_point);
        is_right_line_point = is_right_line_point && this->is_valid_point(right_point, right_line_points, left_line_points, CurrentLine::RIGHT_LINE);
        if (is_right_line_point) {
            right_line_points.push_back(right_point);
            right_line_tracker++;
        }

        // Left line
        bool is_left_line_point = false;
        left_point = this->find_line(image, left_point, left_line_points, right_line_points, CurrentLine::LEFT_LINE, false, is_left_line_point);
        is_left_line_point = is_left_line_point && this->is_valid_point(left_point, left_line_points, right_line_points, CurrentLine::LEFT_LINE);
        if (is_left_line_point) {
            left_line_points.push_back(left_point);
            left_line_tracker++;
        }

        // Research Right line if not found
        if (!is_right_line_point && !left_line_points.empty()) {
            right_point = this->find_line(image, right_point, right_line_points, left_line_points, CurrentLine::RIGHT_LINE, true, is_right_line_point);
            is_right_line_point = is_right_line_point && this->is_valid_point(right_point, right_line_points, left_line_points, CurrentLine::RIGHT_LINE);
            if (is_right_line_point) {
                right_line_points.push_back(right_point);
                right_line_tracker++;
            }
        }

        // Track lines and append
        if (!is_right_line_point) {
            right_line_tracker = 0;
        }

        if (!is_left_line_point) {
            left_line_tracker = 0;
        }

        // Check if right line is solid based on tracker count
        if (right_line_tracker > 6 && y > part_height) {
            is_right_line_solid = true;
        }
        if (left_line_tracker > 6 && y > part_height) {
            is_left_line_solid = true;
        }
    }

    // std::cout << "Left solid: " << is_left_line_solid << ", Right solid: " << is_right_line_solid << std::endl;

    this->determine_line(left_line_points, right_line_points, is_left_line_solid, is_right_line_solid, 4);
    return std::make_tuple(left_line_points, right_line_points);
}


std::vector<uint16_t> LaneDetection::check_window_edges(
        const cv::Mat& image,
        const cv::Point& point
) {
    WindowBounds window = create_window_bounds(point, this->config.window_width, this->config.window_height);
    if (window_out_of_image(window, image.size())) {
        return std::vector<uint16_t>();
    }
    std::vector<uint16_t> pixels = this->find_pixels_in_window(image, window);

    if (this->config.show_opencv_images || this->config.pub_images) {
        cv::rectangle( 
            this->debug_image,
            cv::Point(window.x_low, window.y_low), 
            cv::Point(window.x_high, window.y_high), 
            cv::Scalar(255), 2
        );
    }
    return pixels;
}


std::vector<uint16_t> LaneDetection::recursive_window_check(
        const cv::Mat& image,  
        const cv::Point& point, 
        const uint16_t min_pixels
) {
    auto indices = this->check_window_edges(image, point);

    // Stop recursion if the number of indices is below the threshold or window is invalid
    if (indices.size() <= min_pixels) {
        return indices;
    }
    int new_x = calculate_mean(indices);
    int new_y = point.y;
    return this->recursive_window_check(image, cv::Point(new_x, new_y), indices.size());
}


std::optional<cv::Point> LaneDetection::check_window(
        const cv::Mat& image,  
        const cv::Point& point
) {
    auto indices = this->recursive_window_check(image, point, this->config.min_pixels);

    if (indices.size() > this->config.min_pixels) {
        int new_x = calculate_mean(indices);
        int new_y = point.y;
        return cv::Point(new_x, new_y);
    }
    return std::nullopt;
}


std::optional<cv::Point> LaneDetection::check_window_sequence(
        const cv::Mat& image,  
        const cv::Point& point,
        CurrentLine current_line,
        const bool line_found
) {
    int new_y = point.y;
    for (int i : this->get_iterations(current_line, line_found)) {
        int new_x = point.x + i * this->config.x_shift;
        cv::Point search_point(new_x, new_y);
        auto found_point = this->check_window(image, search_point);
        if (found_point.has_value()) {
            return found_point;
        }
    }
    return std::nullopt;
}


cv::Point LaneDetection::find_line(
        const cv::Mat& image,
        const cv::Point& init_point,
        std::vector<cv::Point>& line_points,
        std::vector<cv::Point>& other_line_points,
        CurrentLine current_line,
        bool use_other_grad,
        bool& is_line_point
) {
    // Verify that init_point is valid for the specified line
    bool init_point_possible = this->is_valid_point(init_point, line_points, other_line_points, current_line);

    // Verify if use_other_grad can be used
    bool use_other_grad_possible = (other_line_points.size() >= 2 && init_point.y == other_line_points.back().y);
    use_other_grad = (use_other_grad || line_points.empty() || !init_point_possible) && use_other_grad_possible;

    // Set up the search point based on conditions
    cv::Point search_point;
    if (line_points.size() == 1 && !use_other_grad) {
        search_point = init_point;
    } else if (line_points.size() >= 2 && !use_other_grad) {
        search_point = this->adjust_point_with_gradient(init_point, line_points.back(), line_points[line_points.size() - 2]);
    } else if (!use_other_grad) {
        search_point = this->adjust_point_with_shift(init_point, current_line);
    } else if (use_other_grad) {
        search_point = this->predict_point_with_other_line(other_line_points.back(), other_line_points[other_line_points.size() - 2], current_line);
    } else {
        throw std::logic_error("Error in if statement!");
    }

    bool line_found = line_points.size() >= 2;
    bool search_point_possible = this->is_valid_point(search_point, line_points, other_line_points, current_line);
    if (!search_point_possible && !use_other_grad && use_other_grad_possible) {
        return this->find_line(image, init_point, line_points, other_line_points, current_line, true, is_line_point);
    } else if (!search_point_possible) {
        is_line_point = false;
        return init_point;
    }

    // Main search direction
    std::optional<cv::Point> point = this->check_window_sequence(image, search_point, current_line, line_found);

    // Try another search direction if the first attempt failed
    if (!point.has_value() && line_points.size() >= 3 && !use_other_grad) {
        search_point = this->adjust_point_with_gradient(init_point, line_points[line_points.size() - 2], line_points[line_points.size() - 3]);
        point = this->check_window_sequence(image, search_point, current_line, line_found);

        if (point.has_value()) {
            line_points.pop_back();
        }
    }

    // Recursively try with other gradient if the point is still not found
    if (!point.has_value() && !use_other_grad && use_other_grad_possible) {
        return this->find_line(image, init_point, line_points, other_line_points, current_line, true, is_line_point);
    }

    // Return the result, either a search point or a valid found point
    if (!point.has_value()) {
        is_line_point = false;
        return search_point;
    } else {
        is_line_point = true;
        return point.value();
    }
}


cv::Point LaneDetection::predict_point_with_other_line(
        const cv::Point& point,
        const cv::Point& prev_point,
        CurrentLine current_line
) {
    float gradient = discrete_gradient(point, prev_point);
    int new_x = 0;

    if (gradient == 0.0) {
        if (current_line == CurrentLine::LEFT_LINE) {
            new_x = point.x - this->config.lane_distance;
        } else if (current_line == CurrentLine::RIGHT_LINE) {
            new_x = point.x + this->config.lane_distance;
        }
    } else {
        // Predict the lane adjustment using gradient
        cv::Point line_pred = walk_gradient_x_y(point, 1.f/gradient, this->config.lane_distance, current_line);

        // Adjust the x-coordinate based on the predicted gradient
        new_x = line_pred.x + static_cast<int>(gradient * (point.y - line_pred.y));
    }

    // Return the predicted point
    return cv::Point(new_x, point.y);
}


cv::Point LaneDetection::adjust_point_with_shift(const cv::Point& point, CurrentLine current_line) {
    // if ((current_line == CurrentLine::LEFT_LINE && this->curve_sharpness < -1.0) // sharp right turn
    //         || (current_line == CurrentLine::RIGHT_LINE && this->curve_sharpness > 1.0)) { // sharp left turn
    //     std::cout << "Sharp turn detected! " << this->curve_sharpness << ", Lane: " << current_line << std::endl;
    //     return point;
    // }
    auto new_y = point.y;
    auto new_x = (current_line == CurrentLine::LEFT_LINE) ? point.x - this->config.x_shift_next : point.x + this->config.x_shift_next;
    return cv::Point(new_x, new_y);
}


cv::Point LaneDetection::adjust_point_with_gradient(const cv::Point& point, const cv::Point& prev1_point, const cv::Point& prev2_point) {
    auto grad = discrete_gradient(prev1_point, prev2_point);
    auto new_x = prev1_point.x + rint(grad * (point.y - prev1_point.y));
    auto new_y = point.y;
    return cv::Point(int(new_x), int(new_y));
}


bool LaneDetection::is_valid_point(
        const cv::Point& point, 
        const std::vector<cv::Point>& line_points,
        const std::vector<cv::Point>& other_line_points, 
        CurrentLine current_line
    ) {
    // Check if it is inside a cone in front of it
    if (line_points.size() >= 2) {
        float prev_grad = discrete_gradient(line_points[line_points.size() - 1], line_points[line_points.size() - 2]);
        if (!this->is_inside_curved_bounds(point, line_points[line_points.size() - 1], prev_grad)) {
            return false;
        }
    }

    // Check the point is outside the circles of the other points
    if (!this->is_outside_save_circles(point, other_line_points)) {
        return false;
    }

    int upper_bound = point.y + static_cast<int>(this->config.window_height * 4);
    int lower_bound = point.y - static_cast<int>(this->config.window_height * 4);

    // Find the reference point in the other_line_points vector
    auto reference_point = std::find_if(other_line_points.begin(), other_line_points.end(),
                                            [lower_bound, upper_bound](const cv::Point& p) {
                                                return p.y >= lower_bound && p.y <= upper_bound;
                                            });

    if (reference_point == other_line_points.end()) {
        return true; // No statement possible
    }

    // Compare x-coordinate based on which line
    if (current_line == CurrentLine::LEFT_LINE) {
        return point.x < reference_point->x;
    } else if (current_line == CurrentLine::RIGHT_LINE) {
        return point.x > reference_point->x;
    } else {
        throw std::invalid_argument("Invalid line type, use LEFT_LINE or RIGHT_LINE.");
    }
}


bool LaneDetection::is_outside_save_circles(const cv::Point& point, const std::vector<cv::Point>& other_points) {
    // If there are points to check, iterate over them and check if the point is outside all of them
    if (!other_points.empty()) {
        for (const auto& other_point : other_points) {
            if (!this->is_outside_circle(point, other_point, this->config.safe_lane_distance)) {
                return false;  // If point is inside any circle, return false
            }
        }
    }
    return true;
}


bool LaneDetection::is_outside_circle(const cv::Point& point, const cv::Point& center, uint16_t radius) {
    uint32_t dx = point.x - center.x;
    uint32_t dy = point.y - center.y;
    uint32_t squared_distance = dx * dx + dy * dy;
    uint32_t squared_radius = static_cast<uint32_t>(radius) * radius;
    return squared_distance > squared_radius;
}


bool LaneDetection::is_inside_curved_bounds(const cv::Point& point, const cv::Point& previous_point, float gradient) {
    int dy = point.y - previous_point.y;
    if (dy >= 0) {
        return false;
    }

    // std::cout << "Gradient: " << gradient << std::endl;

    float abs_grad = std::abs(gradient);
    float left_curve_frac = this->config.valid_curve_factor;
    float right_curve_frac = this->config.valid_curve_factor;
    if (gradient <  0.0f) {
        right_curve_frac *= (1.0f + abs_grad);
        left_curve_frac *= (1.0f - 0.3f * abs_grad);
    } else if (gradient >  0.0) {
        left_curve_frac *= (1.0f + abs_grad);
        right_curve_frac *= (1.0f - 0.3f * abs_grad);
    }

    float band = (1.0f + 0.5f * abs_grad) * this->config.valid_band;
    float center_x = static_cast<float>(previous_point.x) + gradient * static_cast<float>(dy);
    float square_dy = static_cast<float>(dy * dy);
    float left_bound_x = center_x - band - left_curve_frac * square_dy;
    float right_bound_x = center_x + band + right_curve_frac * square_dy;
    
    if (this->config.show_opencv_images || this->config.pub_images) {
        int y = point.y;
        cv::Point left_bound_point(static_cast<int>(left_bound_x), y);
        cv::Point right_bound_point(static_cast<int>(right_bound_x), y);
        cv::circle(this->debug_image, left_bound_point, 3, cv::Scalar(10, 255, 255), -1); // Left bound: Red
        cv::circle(this->debug_image, right_bound_point, 3, cv::Scalar(10, 255, 255), -1); // Right bound: Green
    }
    return (left_bound_x <= point.x && point.x <= right_bound_x);
}


std::vector<int16_t> LaneDetection::get_iterations(CurrentLine current_line, const bool line_found) {
    //if you want the left iters, use LEFT_LINE, else use RIGHT_LINE
    if (current_line == CurrentLine::RIGHT_LINE) {
        return line_found ? this->config.right_iters : this->config.notfound_right_iters;
    } else if (current_line == CurrentLine::LEFT_LINE) {
        return line_found ? this->config.left_iters : this->config.notfound_left_iters;
    } else {
        throw std::invalid_argument("Invalid line type or bool argument in get_iterations.");
    }
}

std::vector<int16_t> LaneDetection::get_iters(int16_t iters_each_side, CurrentLine current_line) {
    //if you want the left iters, use LEFT_LINE, else use RIGHT_LINE
    std::vector<int16_t> result = {0};
    
    for (int16_t i = 1; i <= iters_each_side; ++i) {
        if (current_line == CurrentLine::RIGHT_LINE) {
            result.push_back(i);  
        }
        result.push_back(-i);
        if (current_line == CurrentLine::LEFT_LINE) {
            result.push_back(i);  
        }
    }
    
    return result;
}


bool LaneDetection::is_between_lines(
        const cv::Point& point1,
        const cv::Point& point2,
        const std::unordered_set<cv::Point>& left_set,
        const std::unordered_set<cv::Point>& right_set
    ) {
        bool in_left1 = left_set.find(point1) != left_set.end();
        bool in_right1 = right_set.find(point1) != right_set.end();
        bool in_left2 = left_set.find(point2) != left_set.end();
        bool in_right2 = right_set.find(point2) != right_set.end();

        // If both points belong to the same line (either left or right), return false
        if ((in_left1 && in_left2) || (in_right1 && in_right2)) {
            return false;
        }

        // If one point is in the left line and the other is in the right line, return true
        if ((in_left1 && in_right2) || (in_right1 && in_left2)) {
            return true;
        }

        // If neither point belongs to any line, return false
        return false;
    }


std::vector<cv::Point> LaneDetection::find_lane_delaunay(
        const std::vector<cv::Point>& left_points, 
        const std::vector<cv::Point>& right_points
) { 
    std::vector<cv::Point> mid_lane;
    
    if (left_points.empty() || right_points.empty()) {
        return mid_lane; 
    }

    std::unordered_set<cv::Point> left_set(left_points.begin(), left_points.end());
    std::unordered_set<cv::Point> right_set(right_points.begin(), right_points.end());

    std::vector<cv::Point> all_points = left_points;
    all_points.insert(all_points.end(), right_points.begin(), right_points.end());
    cv::Rect bounding_rect = cv::boundingRect(all_points);
    cv::Subdiv2D subdiv(bounding_rect);

    for (const auto& point : all_points) {
        subdiv.insert(point);
    }

    std::vector<cv::Vec6f> triangles;
    subdiv.getTriangleList(triangles);

    for (const auto& triangle : triangles) {
        cv::Point p1(triangle[0], triangle[1]);
        cv::Point p2(triangle[2], triangle[3]);
        cv::Point p3(triangle[4], triangle[5]);

        if (!bounding_rect.contains(p1) || !bounding_rect.contains(p2) || !bounding_rect.contains(p3)) {
            continue;
        }

        if (this->is_between_lines(p1, p2, left_set, right_set)) {
            cv::Point mid1 = (p1 + p2) / 2;
            mid_lane.push_back(mid1);
        }
        if (this->is_between_lines(p2, p3, left_set, right_set)) {
            cv::Point mid2 = (p2 + p3) / 2;
            mid_lane.push_back(mid2);
        }
        if (this->is_between_lines(p3, p1, left_set, right_set)) {
            cv::Point mid3 = (p3 + p1) / 2;
            mid_lane.push_back(mid3);
        }
    }
    std::sort(mid_lane.begin(), mid_lane.end(), [](const cv::Point& a, const cv::Point& b) {
        return a.y > b.y;
    });

    mid_lane.erase(std::unique(mid_lane.begin(), mid_lane.end(), [](const cv::Point& a, const cv::Point& b) {
        return a == b;  // Remove consecutive duplicates
    }), mid_lane.end());

    if (this->config.show_opencv_images || this->config.pub_images) {
        for (const auto& triangle : triangles) {
            cv::Point p1(triangle[0], triangle[1]);
            cv::Point p2(triangle[2], triangle[3]);
            cv::Point p3(triangle[4], triangle[5]);
            cv::line(this->debug_image, p1, p2, cv::Scalar(0, 255, 0), 1);
            cv::line(this->debug_image, p2, p3, cv::Scalar(0, 255, 0), 1);
            cv::line(this->debug_image, p3, p1, cv::Scalar(0, 255, 0), 1);
            this->draw_points(mid_lane);
        }
    }
    return mid_lane;
}


std::vector<cv::Point> LaneDetection::find_lane_one_line(
        const std::vector<cv::Point>& line_points, 
        const CurrentLine current_line 
) {
    // Current lane switched because it then works with walk gradient 
    std::vector<cv::Point> mid_lane;
    if (line_points.size() < static_cast<size_t>(3)) {
        return mid_lane; 
    }

    float prev_grad = discrete_gradient(line_points[1], line_points[0]);
    float next_grad, avg_grad;

    uint16_t half_lane_distance = this->config.lane_distance / 2;
    for (uint16_t i = 1; i < (line_points.size() - 1); ++i) {
        next_grad = discrete_gradient(line_points[i+1], line_points[i]);
        avg_grad = (prev_grad + next_grad) / 2.0f;

        if (avg_grad == 0.0f) {
            cv::Point mid_point;
            mid_point.x = (current_line == CurrentLine::RIGHT_LINE) ? line_points[i].x + half_lane_distance : line_points[i].x - half_lane_distance;
            mid_point.y = line_points[i].y; 
            mid_lane.push_back(mid_point);
            continue;
        }
        cv::Point mid_point = walk_gradient_x_y(line_points[i], 1.f/avg_grad, half_lane_distance, current_line);

        if (!(mid_point.x > this->config.bird_view_width || mid_point.x < 0)) {
            mid_lane.push_back(mid_point);
        }
        prev_grad = next_grad;
    }
    this->draw_points(mid_lane);
    return mid_lane;
}



std::vector<uint16_t> LaneDetection::find_pixels_in_window(const cv::Mat& image, const WindowBounds& window){
    std::vector<uint16_t> indices = {};
    for (uint16_t y = window.y_low; y < window.y_high; ++y) {
        const uchar* row_ptr = image.ptr<uchar>(y);
        for (uint16_t x = window.x_low; x < window.x_high; ++x) {
            if (row_ptr[x] > 0) {
                indices.push_back(x);
            }
        }
    }
    return indices;
}


void LaneDetection::draw_points(const std::vector<cv::Point>& line_points, cv::Scalar color) {
    for (const auto& point : line_points) {
        cv::circle(this->debug_image, point, 5, color, -1); 
    }
}


cv::Mat LaneDetection::draw_lanes_on_image(
    const cv::Mat& image, 
    const std::vector<cv::Point>& left_fit, 
    const std::vector<cv::Point>& right_fit, 
    const std::vector<cv::Point>& mid_fit
) {
    cv::Mat image_with_lanes = image.clone();

    cv::Scalar left_line_color(0, 255, 0);
    cv::Scalar right_line_color(0, 0, 255);
    cv::Scalar mid_lane_color(255, 255, 0);
    if (!left_fit.empty()) {
        cv::polylines(image_with_lanes, left_fit, false, left_line_color, 3);
    }
    if (!right_fit.empty()) {
        cv::polylines(image_with_lanes, right_fit, false, right_line_color, 3);
    }
    if (!mid_fit.empty()) {
        cv::polylines(image_with_lanes, mid_fit, false, mid_lane_color, 3);
    }
    return image_with_lanes;
}


std::vector<cv::Point2f> LaneDetection::px2ego_m(const std::vector<cv::Point>& lane_fit_px) {
    std::vector<cv::Point2f> lane_fit_m;
    lane_fit_m.reserve(lane_fit_px.size());

    for (const cv::Point& pt : lane_fit_px) {
        float x = this->config.x0_m + float(this->config.bird_view_height - pt.y) * this->config.px2m;
        float y = float((pt.x - this->config.bird_view_width / 2)) * this->config.px2m;
        lane_fit_m.push_back(cv::Point2f(x, y));
    }
    return lane_fit_m;
}


void LaneDetection::predict_next_x_init_offset(
        const std::vector<cv::Point>& lane
) {
    float curve = 0.0;
    int counter = 0;
    for (size_t i = 1; i < lane.size(); i++) {
        cv::Point point = lane[i];
        cv::Point last_point = lane[i-1];
        
        if (point.y != last_point.y) {
            curve += discrete_gradient(point, last_point);
            counter++;
        }
    
        if (counter >= 5) {
            break;
        }
    }
    int max_offset = static_cast<int>((this->config.right_init_factor - this->config.left_init_factor) * this->config.max_init_offset_factor * this->config.bird_view_width);
    this->next_x_init_offset = static_cast<int>(-curve * this->config.next_init_factor * static_cast<float>(this->config.bird_view_width));
    this->next_x_init_offset = std::clamp(this->next_x_init_offset, -max_offset, max_offset);

    // set curve sharpness but filter it with a lowpass filter to avoid large deviations
    this->curve_sharpness = curve;
}


std::vector<cv::Point> LaneDetection::moving_average_filter(
    const std::vector<cv::Point>& line_points, 
    int window_size, 
    float threshold_factor
) {
    std::vector<cv::Point> filtered_points;
    int n = static_cast<int>(line_points.size());
    
    // Check if we have enough points for one complete window.
    if(n < window_size) {
        return line_points;
    }

    filtered_points.push_back(line_points[0]);
    
    int half_window = window_size / 2;
    for (int i = 1; i < n; i++) {
        float current_grad = discrete_gradient(line_points[i], line_points[i-1]);

        // Define the local window for computing gradient statistics.
        // Ensure the window is within [1, n-1] because gradients are computed between a point and its previous point.
        int start_idx = std::max(1, i - half_window);
        int end_idx = std::min(n - 1, i + half_window);
        
        float sum = 0.0;
        float sumSq = 0.0;
        int count = 0;
        
        // Compute statistics over the gradients in the local window.
        // Note: The gradient at index k is computed as discrete_gradient(line_points[k], line_points[k-1]).
        for (int k = start_idx; k <= end_idx; k++) {
            if (k == i) 
                continue;
            float grad = discrete_gradient(line_points[k], line_points[k-1]);
            sum   += grad;
            sumSq += grad * grad;
            count++;
        }
        
        // Calculate the local mean and standard deviation.
        float mean = sum / count;
        float variance = (sumSq - (sum * sum) / count) / count;
        float std_dev = (variance > 0) ? std::sqrt(variance) : 0.0;
        
        // If the standard deviation is nearly zero, we assume consistency.
        // Otherwise, check if the gradient is within the acceptable threshold.
        if (std_dev < 1e-3 || std::abs(current_grad - mean) <= threshold_factor * std_dev) {
            filtered_points.push_back(line_points[i]);
        }
    }
    
    return filtered_points;
}

std::vector<cv::Point> LaneDetection::smooth_line_points(
    const std::vector<cv::Point>& points,
    float max_angle_diff,  // degrees
    float smoothing_factor
) {
    if (points.size() < 3) {
        return points;
    }

    std::vector<cv::Point> smoothed_points = points;
    const float max_angle_rad = max_angle_diff * M_PI / 180.0f;

    // Multiple passes for better smoothing
    for (int pass = 0; pass < 2; ++pass) {
        for (size_t i = 1; i < smoothed_points.size() - 1; ++i) {
            cv::Point& prev = smoothed_points[i-1];
            cv::Point& curr = smoothed_points[i];
            cv::Point& next = smoothed_points[i+1];

            // Calculate angles between segments
            float angle1 = std::atan2(curr.y - prev.y, curr.x - prev.x);
            float angle2 = std::atan2(next.y - curr.y, next.x - curr.x);
            float angle_diff = std::abs(angle2 - angle1);
            
            // Normalize angle difference to [-π, π]
            while (angle_diff > M_PI) angle_diff -= 2.0f * M_PI;
            angle_diff = std::abs(angle_diff);

            // If angle difference is too large, smooth the point
            if (angle_diff > max_angle_rad) {
                // Calculate ideal position based on neighbors
                cv::Point ideal_pos;
                ideal_pos.x = (prev.x + next.x) / 2;
                ideal_pos.y = (prev.y + next.y) / 2;

                // Move point towards ideal position based on smoothing factor
                curr.x = curr.x + (ideal_pos.x - curr.x) * smoothing_factor;
                curr.y = curr.y + (ideal_pos.y - curr.y) * smoothing_factor;
            }
        }
    }

    return smoothed_points;
}

void LaneDetection::determine_line(
        std::vector<cv::Point>& left_line_points, 
        std::vector<cv::Point>& right_line_points, 
        bool is_left_line_solid,
        bool is_right_line_solid,
        const size_t threshold
) {
    is_left_line_solid = is_left_line_solid && left_line_points.size() > 6;
    is_right_line_solid = is_right_line_solid && right_line_points.size() > 6;

    if (this->current_lane_ == CurrentLane::RIGHT) {
        // Middle line (right_line) should be non-solid, outer line (left_line) should be solid
        if (!is_right_line_solid && is_left_line_solid) {
            if (right_line_points.size() > threshold) {
                left_line_points = right_line_points;
                right_line_points.clear();
            } else {
                right_line_points.clear();
                left_line_points.clear();
            }
        } else if ((is_right_line_solid && is_left_line_solid) || left_line_points.size() < threshold) {
            left_line_points.clear();
        } else if (right_line_points.size() < threshold) {
            right_line_points.clear();
        }

    } else if (this->current_lane_ == CurrentLane::LEFT) {
        // Middle line (left_line) should be non-solid, outer line (right_line) should be solid
        if (!is_left_line_solid && is_right_line_solid) {
            if (left_line_points.size() > threshold) {
                right_line_points = left_line_points;
                left_line_points.clear();
            } else {
                left_line_points.clear();
                right_line_points.clear();
            }
        } else if ((is_right_line_solid && is_left_line_solid) || right_line_points.size() < threshold) {
            right_line_points.clear();
        } else if (left_line_points.size() < threshold) {
            left_line_points.clear();
        }
    }
}
