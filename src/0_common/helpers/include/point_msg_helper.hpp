#ifndef POINT_MSG_HELPER_HPP
#define POINT_MSG_HELPER_HPP

#include <vector>
#include <cmath>
#include <optional>
#include <geometry_msgs/msg/point.hpp>


/**
 * @brief Creates a 2D point with the specified x and y coordinates.
 *
 * @param x The x-coordinate of the point.
 * @param y The y-coordinate of the point.
 * 
 * @return A geometry_msgs::msg::Point object with the specified x and y coordinates.
 */
inline geometry_msgs::msg::Point make_point(
        const double x,
        const double y 
) {
    geometry_msgs::msg::Point point;
    point.x = x;
    point.y = y;
    return point;
}

/**
 * @brief Calculates a new point after one step along a unit gradient for a given distance from the current point.
 *
 * @param current The current point.
 * @param unit_gradient The unit vector representing the direction of the gradient.
 * @param distance The distance to walk along the gradient.
 *
 * @return A geometry_msgs::msg::Point representing the new point after walking along the gradient.
 */
inline geometry_msgs::msg::Point walk_gradient(
        const geometry_msgs::msg::Point& current,
        const geometry_msgs::msg::Point& unit_gradient,
        const double distance     
) {
    return make_point(
        current.x + unit_gradient.x * distance,
        current.y + unit_gradient.y * distance
    );
}

/**
 * @brief Calculates the euclidian distanz between two points.
 *
 * @param p1 The first point.
 * @param p2 The second point.
 *
 * @return The length of the segment between the two points.
 */
inline double get_segment_length(
        const geometry_msgs::msg::Point& p1, 
        const geometry_msgs::msg::Point& p2
) {
    double dx = p2.x - p1.x;
    double dy = p2.y - p1.y;
    return std::sqrt(dx * dx + dy * dy);
}

/**
 * @brief Calculates the squared euclidian distan between two points.
 *
 * @param p1 The first point.
 * @param p2 The second point.
 *
 * @return The squared length of the segment between the two points.
 */
inline double get_squared_segment_length(
        const geometry_msgs::msg::Point& p1, 
        const geometry_msgs::msg::Point& p2
) {
    double dx = p2.x - p1.x;
    double dy = p2.y - p1.y;
    return dx * dx + dy * dy;
}

/**
 * @brief Calculates the unit gradient vector from point p1 to point p2.
 *
 * @param p1 The first point.
 * @param p2 The second point.
 *
 * @return A geometry_msgs::msg::Point representing the unit gradient from p1 to p2.
 */
inline geometry_msgs::msg::Point get_unit_gradient(
        const geometry_msgs::msg::Point& p1, 
        const geometry_msgs::msg::Point& p2
) {
    double dx = p2.x - p1.x;
    double dy = p2.y - p1.y;
    double length = std::sqrt(dx * dx + dy * dy);
    geometry_msgs::msg::Point unit_gradient;
    if (length == 0.0) {
        unit_gradient.x = 0.0;
        unit_gradient.y = 0.0;
    } else {
        unit_gradient.x = dx / length;
        unit_gradient.y = dy / length;
    }
    return unit_gradient;
}

/**
 * @brief Calculates the unit gradient vector in the direction specified by an angle.
 *
 * @param angle The angle in radians.
 *
 * @return A geometry_msgs::msg::Point representing the unit gradient in the direction of the angle.
 */
inline geometry_msgs::msg::Point get_unit_gradient( 
        const double angle
) {
    return make_point(
        std::cos(angle),
        std::sin(angle)
    );
}

/**
 * @brief Calculates the normalized angle (in radians) between two points relative to the x-axis.
 *
 * @param p1 The first point.
 * @param p2 The second point.
 *
 * @return The normalized angle in radians between the two points.
 */
inline double calculate_normalized_angle(
        const geometry_msgs::msg::Point& p1, 
        const geometry_msgs::msg::Point& p2
) {
    return std::atan2(p2.y - p1.y, p2.x - p1.x);
}

/**
 * @brief Calculates the angle between the unit gradient vectors originating 
 * from a reference point towards two other points.
 *
 * @param p_ref The reference point.
 * @param p1 The first point.
 * @param p2 The second point.
 *
 * @return The angle in radians between the unit gradients from the reference point to the two points.
 */
inline double calculate_angle_between_gradients(
        const geometry_msgs::msg::Point& p_ref,
        const geometry_msgs::msg::Point& p1, 
        const geometry_msgs::msg::Point& p2
) {
    geometry_msgs::msg::Point p1_grad = get_unit_gradient(p_ref, p1);
    geometry_msgs::msg::Point p2_grad = get_unit_gradient(p_ref, p2);
    double dot_product = p1_grad.x * p2_grad.x + p1_grad.y * p2_grad.y;

    double cos_theta = std::clamp(dot_product, -1.0, 1.0);
    return std::acos(cos_theta);
}

/**
 * @brief Calculates the intersection points between a line and a circle.
 *
 * The line is defined by two points and the circle is defined by a center point and a radius.
 * https://math.stackexchange.com/questions/228841/how-do-i-calculate-the-intersections-of-a-straight-line-and-a-circle
 *
 * @param p1 The first point of the line.
 * @param p2 The second point of the line.
 * @param circle_point The center point of the circle.
 * @param radius The radius of the circle.
 *
 * @return An optional pair of points representing the intersection points, or nullopt if there are no intersections.
 */
inline std::optional<std::pair<geometry_msgs::msg::Point, geometry_msgs::msg::Point>> get_circle_line_intersect_point(
    const geometry_msgs::msg::Point& p1, 
    const geometry_msgs::msg::Point& p2,
    const geometry_msgs::msg::Point& circle_point,
    const double radius
) {
    // Case 1: Handle vertical lines (p1.x == p2.x)
    if (p1.x == p2.x) {
        double dx = p1.x - circle_point.x;
        double dy_squared = radius * radius - dx * dx;

        if (dy_squared < 0) {
            return std::nullopt;
        }
        
        double sqrt_dy = std::sqrt(dy_squared);
        return std::make_optional(
            std::make_pair(
                make_point(p1.x, circle_point.y + sqrt_dy),
                make_point(p1.x, circle_point.y - sqrt_dy)
            )
        );
    }

    // Case 2: Non-vertical lines (y = mx + c)
    double m = (p2.y - p1.y) / (p2.x - p1.x);
    double c = p1.y - m * p1.x;

    // Coefficients for the quadratic equation
    double A = m * m + 1.0;
    double B = 2.0 * (m * c - m * circle_point.y - circle_point.x);
    double C = circle_point.y * circle_point.y - radius * radius + circle_point.x * circle_point.x 
        - 2.0 * c * circle_point.y + c * c;

    // Discriminant of the quadratic equation
    double discriminant = B * B - 4.0 * A * C;

    // No intersection
    if (discriminant < 0 || A == 0) {
        return std::nullopt;
    } 

    double sqrt_discriminant = std::sqrt(discriminant);
    double denominator = 1.0 / (2.0 * A);

    double x_intersect1 = (-B + sqrt_discriminant) * denominator;
    double x_intersect2 = (-B - sqrt_discriminant) * denominator;
    double y_intersect1 = m * x_intersect1 + c;
    double y_intersect2 = m * x_intersect2 + c;

    return std::make_optional(
        std::make_pair(
            make_point(x_intersect1, y_intersect1),
            make_point(x_intersect2, y_intersect2)
        )
    );
}

/**
 * @brief Calculates the walking distance along a segment defined by a top and bottom point.
 *
 * The walking distance is determined by the length of the segment and the angle between the gradients formed by the current point, top, and bottom points.
 *
 * @param current_point The current point.
 * @param top_point The top point of the segment.
 * @param bot_point The bottom point of the segment.
 * @param distance The distance to walk.
 *
 * @return The calculated walking distance.
 */
inline double get_walking_distance(
        const geometry_msgs::msg::Point& current_point,
        const geometry_msgs::msg::Point& top_point, 
        const geometry_msgs::msg::Point& bot_point,
        const double distance
) {
    double bot_length = get_segment_length(current_point, bot_point);
    double alpha_bot = calculate_angle_between_gradients(bot_point, current_point, top_point);
    
    double walking_distance;
    if (alpha_bot < M_PI_2) {
        double bot_to_center_cut_gradient = bot_length * std::cos(alpha_bot); 
        double current_to_center_cut_gradient = std::sqrt(bot_length * bot_length - bot_to_center_cut_gradient * bot_to_center_cut_gradient);
        double center_cut_gradient_to_target = std::sqrt(distance * distance - current_to_center_cut_gradient * current_to_center_cut_gradient);

        if (std::isnan(center_cut_gradient_to_target)) {
            center_cut_gradient_to_target = distance;
        }
        walking_distance = center_cut_gradient_to_target + bot_to_center_cut_gradient;

    } else if (alpha_bot >= M_PI_2) {
        double top_length = get_segment_length(current_point, top_point);
        double top_bot_length = get_segment_length(bot_point, top_point);
        double bot_missing = distance - bot_length;
        double top_extra = top_length - distance;
        walking_distance = top_bot_length * bot_missing / (bot_missing + top_extra);

    } else {
        std::ostringstream error_message;
        error_message << "Alpha and Beta range not implemented yet, in get 'walking distance'."
                    << "{Alpha: " << alpha_bot << "}";
        walking_distance = 0.0;
    }
    return walking_distance;
}

/**
 * @brief Finds the target point on a line segment between two points that is at a specified distance from a start point.
 * 
 * This function calculates the intersection points of a circle defined by a distance from the start point and a line segment
 * defined by two points (top_point and bot_point). It checks which of the intersection points lie between the top and bot points,
 * and returns the closest valid intersection point. If no valid intersection is found, it returns the point on the line closest to
 * the circle.
 *
 * @param top_point The top endpoint of the line segment.
 * @param bot_point The bottom endpoint of the line segment.
 * @param start_point The starting point from which the distance is measured.
 * @param distance The distance to be traveled from the start point.
 * @return geometry_msgs::msg::Point The target point that lies on the line segment at the specified distance.
 */
inline geometry_msgs::msg::Point find_target_between_points(
        const geometry_msgs::msg::Point& top_point,
        const geometry_msgs::msg::Point& bot_point,
        const geometry_msgs::msg::Point& start_point,
        const double distance
) {
    auto intersects = get_circle_line_intersect_point(
        bot_point, top_point, start_point, distance
    );  
    if (intersects) {
        auto [intersect1, intersect2] = intersects.value();
        
        // Check if points lie between top and bot point
        double line_length = get_segment_length(bot_point, top_point);
        
        // Check first intersect
        double bot_to_intersect1 = get_segment_length(bot_point, intersect1);
        double intersect1_to_top = get_segment_length(intersect1, top_point);
        bool intersect1_between = std::abs(bot_to_intersect1 + intersect1_to_top - line_length) < 1e-6;
        
        // Check second intersect
        double bot_to_intersect2 = get_segment_length(bot_point, intersect2);
        double intersect2_to_top = get_segment_length(intersect2, top_point);
        bool intersect2_between = std::abs(bot_to_intersect2 + intersect2_to_top - line_length) < 1e-6;
        
        // Return the valid intersect point
        if (intersect1_between && intersect2_between) {
            // If both are valid, return the one closer to bot_point
            return (bot_to_intersect1 < bot_to_intersect2) ? intersect1 : intersect2;
        } else if (intersect1_between) {
            return intersect1;
        } else if (intersect2_between) {
            return intersect2;
        }
    }
    
    // If no valid intersect found, return the point on the line closest to the circle
    double bot_dist = get_segment_length(start_point, bot_point);
    double top_dist = get_segment_length(start_point, top_point);
    return (std::abs(bot_dist - distance) < std::abs(top_dist - distance)) ? bot_point : top_point;
}

/**
 * @brief Finds the index of the point in a trajectory that is at or exceeds a given distance from a start point.
 * 
 * This function searches a trajectory of points and finds the index of the point that is at or exceeds the specified distance
 * from the start point. The search starts from a specified index, and the distance is calculated based on squared Euclidean
 * distance to avoid unnecessary square root computation.
 * 
 * @param trajectory The trajectory of points to search through.
 * @param start_point The starting point from which the distance is measured.
 * @param distance The distance to search for, from the start point.
 * @param start_index The index in the trajectory to start searching from (default is 0).
 * @return std::optional<size_t> The index of the point in the trajectory, or std::nullopt if no point meets the condition.
 * @throws std::invalid_argument If the trajectory is empty.
 */
inline std::optional<size_t> find_top_index(
        const std::vector<geometry_msgs::msg::Point>& trajectory,
        const geometry_msgs::msg::Point& start_point,
        const double distance,
        const size_t start_index = 0
) {
    if (trajectory.empty()) {
        throw std::invalid_argument("Trajectory must not be empty.");
    }

    double squared_distance = distance * distance;
    for (size_t i = start_index; i < trajectory.size(); ++i) {
        double point_distance = get_squared_segment_length(start_point, trajectory[i]);
        if (point_distance >= squared_distance) {
            return i;
        }
    }
    return std::nullopt;
}

/**
 * @brief Finds the point on a trajectory at a specified distance from a start point.
 * 
 * This function locates the point on a trajectory that is at or exceeds a given distance from the start point. If the found
 * point is the first or last point in the trajectory, it returns the respective point. Otherwise, it calculates the exact 
 * position on the line segment between two points in the trajectory using the specified distance.
 * 
 * @param trajectory The trajectory of points to search through.
 * @param start_point The starting point from which the distance is measured.
 * @param distance The distance to search for, from the start point.
 * @param start_index The index in the trajectory to start searching from (default is 0).
 * @return geometry_msgs::msg::Point The point on the trajectory at the specified distance from the start point.
 */
inline geometry_msgs::msg::Point find_point_on_line(
        const std::vector<geometry_msgs::msg::Point>& trajectory,
        const geometry_msgs::msg::Point& start_point,
        const double distance,
        const size_t start_index = 0
) {
    std::optional<size_t> idx_opt = find_top_index(trajectory, start_point, distance, start_index);
    if (!idx_opt.has_value()) {
        return trajectory[trajectory.size() - 1]; // Last trajectory point
    }
    if (idx_opt.value() == 0) {
        return trajectory[0]; // First trajectory point
    }

    size_t idx = idx_opt.value();
    geometry_msgs::msg::Point top_point = trajectory[idx];
    geometry_msgs::msg::Point bot_point = trajectory[idx - 1];
    geometry_msgs::msg::Point target = find_target_between_points(top_point, bot_point, start_point, distance);
    return target;
}

/**
 * @brief Moves a trajectory by a specified distance in the direction of the average gradient.
 * 
 * This function calculates a new trajectory by moving each point of the input trajectory by a specified distance, following
 * the direction of the average gradient between neighboring points. The movement is applied to all points in the trajectory 
 * except the first and last ones.
 * 
 * @param trajectory The original trajectory of points to move.
 * @param distance The distance to move each point in the trajectory.
 * @return std::vector<geometry_msgs::msg::Point> The new trajectory after moving each point.
 */
inline std::vector<geometry_msgs::msg::Point> move_trajectory(
    const std::vector<geometry_msgs::msg::Point>& trajectory,
    const double distance
) {
    if (trajectory.size() < 3) {
        return {};
    }
    
    std::vector<geometry_msgs::msg::Point> new_trajectory;
    geometry_msgs::msg::Point prev_grad, next_grad, avg_grad, transposed_grad;

    for (uint16_t i = 1; i < (trajectory.size() - 1); ++i) {
        double prev_angle = calculate_normalized_angle(trajectory[i-1], trajectory[i]);
        double next_angle = calculate_normalized_angle(trajectory[i], trajectory[i+1]);
        double avg_transposed_angle = (prev_angle + next_angle) / 2.0;
        geometry_msgs::msg::Point unit_grad = get_unit_gradient(avg_transposed_angle + M_PI_2);
        geometry_msgs::msg::Point new_point = walk_gradient(trajectory[i], unit_grad, distance);
        new_trajectory.push_back(new_point);
    }
    return new_trajectory;
}

/**
 * @brief Smooths a trajectory by adjusting points to minimize sudden angle changes.
 * 
 * This function smooths the trajectory by iterating over the points and adjusting their positions to reduce sharp angle
 * differences between consecutive segments. The smoothing is done based on a specified maximum angle difference and a
 * smoothing factor, with multiple iterations for further improvement.
 * 
 * @param trajectory The original trajectory to smooth.
 * @param max_angle_diff The maximum allowed angle difference (in degrees) between consecutive segments.
 * @param smoothing_factor The factor determining the amount of smoothing applied (between 0.0 and 1.0).
 * @param iterations The number of smoothing iterations to perform (default is 2).
 * @return std::vector<geometry_msgs::msg::Point> The smoothed trajectory.
 */
inline std::vector<geometry_msgs::msg::Point> smooth_trajectory(
    const std::vector<geometry_msgs::msg::Point>& trajectory,
    const double max_angle_diff,  // degrees
    const double smoothing_factor,
    const int iterations = 2
) {
    if (smoothing_factor < 0.0 || smoothing_factor > 1.0) {
        return trajectory;
    }
    if (max_angle_diff <= 0.0) {
        return trajectory;
    }
    if (trajectory.size() < 3) {
        return {};
    }

    std::vector<geometry_msgs::msg::Point> smoothed_trajectory = trajectory;
    const double max_angle_rad = max_angle_diff * M_PI / 180.0f;
    const double epsilon = 1e-6;

    // Multiple passes for better smoothing
    for (int pass = 0; pass < iterations; ++pass) {
        for (size_t i = 1; i < smoothed_trajectory.size() - 1; ++i) {
            geometry_msgs::msg::Point& prev = smoothed_trajectory[i-1];
            geometry_msgs::msg::Point& curr = smoothed_trajectory[i];
            geometry_msgs::msg::Point& next = smoothed_trajectory[i+1];

            // Calculate angles between segments
            double angle1 = std::atan2(curr.y - prev.y, curr.x - prev.x);
            double angle2 = std::atan2(next.y - curr.y, next.x - curr.x);
            double angle_diff = std::abs(angle2 - angle1);
            
            // Normalize angle difference to [-π, π]
            while (angle_diff > M_PI) angle_diff -= 2.0f * M_PI;
            angle_diff = std::abs(angle_diff);

            // If angle difference is too large, smooth the point
            if (std::abs(angle_diff - max_angle_rad) > epsilon) {
                // Calculate ideal position based on neighbors
                geometry_msgs::msg::Point ideal_pos;
                ideal_pos.x = (prev.x + next.x) / 2;
                ideal_pos.y = (prev.y + next.y) / 2;

                // Move point towards ideal position based on smoothing factor
                curr.x = curr.x + (ideal_pos.x - curr.x) * smoothing_factor;
                curr.y = curr.y + (ideal_pos.y - curr.y) * smoothing_factor;
            }
        }
    }

    return smoothed_trajectory;
}

/**
 * @brief Calculates the delta angle between a unit gradient vector and the given distance 
 * using the purepursuit algorithm.
 * 
 * This function computes the delta angle for a vehicle's motion, considering the vehicle's wheelbase and the
 * normalized gradient of a vector defined by the input point.
 *
 * @param unit_gradient The unit gradient representing the direction of motion.
 * @param distance The distance over which the delta angle is calculated.
 * @param wheelbase The wheelbase of the vehicle, affecting the turning radius.
 * 
 * @return The calculated delta angle (in radians).
 */
inline double calculate_delta(
    const geometry_msgs::msg::Point& unit_gradient,
    const double distance,
    const double wheelbase
) {
    geometry_msgs::msg::Point zero_point = make_point(0.0, 0.0);
    double alpha = calculate_normalized_angle(zero_point, unit_gradient);
    double L_R_ = 2 * wheelbase * std::sin(alpha); 
    return std::atan2(L_R_, distance);
}

/**
 * @brief Finds the first point where the Y-coordinate crosses zero between two points.
 * 
 * Given two points, this function computes the point where the line connecting them crosses the Y-axis.
 *
 * @param p1 The first point.
 * @param p2 The second point.
 * 
 * @return A point on the X-axis where the Y-coordinate is zero.
 */
inline geometry_msgs::msg::Point find_y_zero_crossing_between_two_points(
        const geometry_msgs::msg::Point& p1,
        const geometry_msgs::msg::Point& p2
) {
    geometry_msgs::msg::Point unit_gradient = get_unit_gradient(p1, p2);
    geometry_msgs::msg::Point zero_point = make_point(0.0, 0.0);
    double alpha = calculate_normalized_angle(zero_point, unit_gradient);
    double x_part = std::abs(p1.y) / std::tan(alpha);
    return make_point(p1.x + x_part, 0.0);
}

/**
 * @brief Compares the unit gradient and squared segment length with a previously known minimum.
 * 
 * This function compares the unit gradient between two points and their squared segment length with a stored
 * minimum and returns the more relevant pair of unit gradient and segment.
 *
 * @param start_point The starting point of the segment.
 * @param point The current point being compared.
 * @param squared_segment_length The squared distance of the current segment.
 * @param smallest_unit_grad_point The point associated with the previously smallest unit gradient.
 * @param smallest_unit_grad The previously smallest unit gradient.
 * @param squared_smallest_unit_grad_segment_length The squared distance of the smallest segment.
 * 
 * @return A tuple containing the relevant unit gradient, point, and squared segment length.
 */
inline std::tuple<geometry_msgs::msg::Point, geometry_msgs::msg::Point, double> compare_unit_grad_with_point(
    const geometry_msgs::msg::Point& start_point,
    const geometry_msgs::msg::Point& point,
    const double squared_segment_length,

    const geometry_msgs::msg::Point& smallest_unit_grad_point,
    const geometry_msgs::msg::Point& smallest_unit_grad,
    const double squared_smallest_unit_grad_segment_length
) {
    geometry_msgs::msg::Point unit_grad = get_unit_gradient(start_point, point);
    double abs_sug_y = std::abs(smallest_unit_grad.y);
    double abs_ug_y = std::abs(unit_grad.y);
    if (abs_sug_y > abs_ug_y || (abs_ug_y == abs_sug_y && squared_segment_length > squared_smallest_unit_grad_segment_length)) {
        return {unit_grad, point, squared_segment_length};
    } else {
        return {smallest_unit_grad, smallest_unit_grad_point, squared_smallest_unit_grad_segment_length};
    }
}

/**
 * @brief Finds the minimum Y unit gradient along a trajectory between a start point and a range defined by bot and top distances.
 *
 * This function iterates through a trajectory and calculates unit gradients based on the distances from a start point. 
 * It returns the unit gradient and the corresponding point along with the distance where the Y coordinate crosses zero, if applicable.
 * The process also accounts for specific edge cases where the trajectory points fall below or above the specified bot and top distances.
 *
 * @param trajectory The trajectory to be evaluated.
 * @param start_point The starting point from which distances are measured.
 * @param distance_bot The minimum distance threshold for the bot (lower bound).
 * @param distance_top The maximum distance threshold for the top (upper bound).
 * 
 * @return A tuple containing:
 *         - A geometry_msgs::msg::Point representing the unit gradient.
 *         - A geometry_msgs::msg::Point representing the point on the trajectory corresponding to the smallest unit gradient.
 *         - A double representing the distance from the start point to the corresponding trajectory point.
 */
inline std::tuple<geometry_msgs::msg::Point, geometry_msgs::msg::Point, double> find_min_y_unit_grad(
        const std::vector<geometry_msgs::msg::Point>& trajectory,
        const geometry_msgs::msg::Point& start_point,
        const double distance_bot,
        const double distance_top
) {
    if (trajectory.empty()) {
        return {make_point(0.0, 0.0), make_point(0.0, 0.0), 0.0};
    }

    const double squared_distance_bot = distance_bot * distance_bot;
    const double squared_distance_top = distance_top * distance_top;

    // Check first point
    double first_point_squared_distance = get_squared_segment_length(start_point, trajectory[0]);
    if (first_point_squared_distance >= squared_distance_top) {
        geometry_msgs::msg::Point unit_grad = get_unit_gradient(start_point, trajectory[0]);
        return {unit_grad, trajectory[0], std::sqrt(first_point_squared_distance)};
    }

    // Check last point
    double last_point_squared_distance = get_squared_segment_length(start_point, trajectory.back());
    if (last_point_squared_distance <= squared_distance_bot) {
        geometry_msgs::msg::Point unit_grad = get_unit_gradient(start_point, trajectory.back());
        return {unit_grad, trajectory.back(), std::sqrt(last_point_squared_distance)};
    }


    bool is_below_bot_distance = true;
    bool is_below_top_distance = true;
    
    double smallest_squared_unit_grad_distance = 0.0;
    geometry_msgs::msg::Point smallest_unit_grad = make_point(0.0, 1.0);
    geometry_msgs::msg::Point smallest_unit_grad_point = make_point(0.0, 0.0);
    const geometry_msgs::msg::Point forward_unit_grad = make_point(1.0, 0.0);

    for (size_t i = 1; i < trajectory.size(); ++i) {
        double squared_point_distance = get_squared_segment_length(start_point, trajectory[i]);

        bool is_on_bot_distance = is_below_bot_distance && is_below_top_distance 
            && squared_point_distance >= squared_distance_bot;
        bool is_on_top_distance = is_below_top_distance && squared_point_distance > squared_distance_top;
        bool is_between_top_and_bot = squared_point_distance >= squared_distance_bot && squared_point_distance <= squared_distance_top;
        
        // Edge case is on bottom distance
        if (is_on_bot_distance) {
            is_below_top_distance = true;
            is_below_bot_distance = false;
            geometry_msgs::msg::Point bot_point = find_target_between_points(
                trajectory[i], 
                trajectory[i-1], 
                start_point, 
                distance_bot
            );
            
            if ((bot_point.y < 0 && trajectory[i].y >= 0) 
                    || (bot_point.y > 0 && trajectory[i].y <= 0)
            ) {
                geometry_msgs::msg::Point target_point = find_y_zero_crossing_between_two_points(
                    bot_point, trajectory[i]
                );
                return {forward_unit_grad, target_point, target_point.x};
            }

            double squared_bot_point_distance = get_squared_segment_length(start_point, bot_point);
            std::tie(smallest_unit_grad, smallest_unit_grad_point, smallest_squared_unit_grad_distance) = compare_unit_grad_with_point(
                start_point, bot_point, squared_bot_point_distance, smallest_unit_grad_point, smallest_unit_grad, smallest_squared_unit_grad_distance
            );

        } 
        
        // Edge case is on top distance
        if (is_on_top_distance) {
            is_below_top_distance = false;
            is_below_bot_distance = false;
            geometry_msgs::msg::Point top_point = find_target_between_points(
                trajectory[i], 
                trajectory[i-1], 
                start_point, 
                distance_top
            );

            if ((trajectory[i-1].y < 0 && top_point.y >= 0) 
                    || (trajectory[i-1].y > 0 && top_point.y <= 0)
            ) {
                geometry_msgs::msg::Point target_point = find_y_zero_crossing_between_two_points(
                    trajectory[i-1], top_point
                );
                return {forward_unit_grad, target_point, target_point.x};
            }

            double squared_top_point_distance = get_squared_segment_length(start_point, top_point);
            std::tie(smallest_unit_grad, smallest_unit_grad_point, smallest_squared_unit_grad_distance) = compare_unit_grad_with_point(
                start_point, top_point, squared_top_point_distance, smallest_unit_grad_point, smallest_unit_grad, smallest_squared_unit_grad_distance
            );
        }

        // Case is between top and bottom distance
        if (is_between_top_and_bot || (is_on_bot_distance && squared_point_distance <= squared_distance_top)) {
            is_below_top_distance = true;
            is_below_bot_distance = false;
            geometry_msgs::msg::Point between_point = trajectory[i];

            if ((trajectory[i-1].y < 0 && between_point.y >= 0) 
                    || (trajectory[i-1].y > 0 && between_point.y <= 0)
            ) {
                geometry_msgs::msg::Point target_point = find_y_zero_crossing_between_two_points(
                    trajectory[i-1], between_point
                );
                return {forward_unit_grad, target_point, target_point.x};
            }

            std::tie(smallest_unit_grad, smallest_unit_grad_point, smallest_squared_unit_grad_distance) = compare_unit_grad_with_point(
                start_point, between_point, squared_point_distance, smallest_unit_grad_point, smallest_unit_grad, smallest_squared_unit_grad_distance
            );
        }

        // Early exit if above top distance
        if (!is_below_top_distance) {
            return {smallest_unit_grad, smallest_unit_grad_point, std::sqrt(smallest_squared_unit_grad_distance)};
        }
    }

    // last Point not above top distance 
    return {smallest_unit_grad, smallest_unit_grad_point, std::sqrt(smallest_squared_unit_grad_distance)};
}

/**
 * @brief Calculates a velocity factor based on the trajectory, current point, and vehicle parameters.
 *
 * This function computes a velocity factor for the vehicle based on its current position along a trajectory,
 * the maximum possible change in velocity, and other vehicle parameters. The velocity factor is calculated 
 * as a function of the gradient along the trajectory and the distance between the current point and the 
 * endpoint of the trajectory segment. The result is clamped within a range and adjusted with the provided offset.
 * 
 * @param trajectory A vector of geometry_msgs::msg::Point representing the trajectory points.
 * @param current_point A geometry_msgs::msg::Point representing the current position of the vehicle.
 * @param wheelbase A double representing the distance between the front and rear axles of the vehicle.
 * @param max_delta A double representing the maximum allowable delta for velocity adjustments.
 * @param offset A double representing the offset value to adjust the final velocity factor.
 * 
 * @return A double representing the calculated velocity factor, adjusted with the offset.
 */
inline double get_velocity_factor(
        const std::vector<geometry_msgs::msg::Point>& trajectory,
        const geometry_msgs::msg::Point& current_point,
        const double wheelbase,
        const double max_delta,
        const double offset
) {
    if (trajectory.size() < 2) {
        return offset;
    }
    geometry_msgs::msg::Point unit_grad = get_unit_gradient(current_point, trajectory.back());
    double distance = get_segment_length(current_point, trajectory.back());
    double delta = calculate_delta(unit_grad, distance, wheelbase);
    double velocity_factor = 1.0 - std::abs(delta) / std::abs(max_delta);
    velocity_factor = std::clamp(velocity_factor, 0.0, 1.0);
    velocity_factor = 1.0 - std::pow((velocity_factor - 1.0), 2);
    velocity_factor = offset + (1.0 - offset) * velocity_factor;
    return velocity_factor;
}

/**
 * @brief Finds a point along the trajectory at a specified x-distance from the start point.
 *
 * This function calculates a point along a given trajectory that is a specific distance (in the x-direction) 
 * from the provided starting point. It iterates over the trajectory, computing the x and y components of each 
 * segment, and determines the point that corresponds to the specified x-distance. If the requested x-distance 
 * falls outside the bounds of the trajectory, the function will return the point closest to the specified x-distance.
 * 
 * @param trajectory A vector of geometry_msgs::msg::Point representing the trajectory points.
 * @param start_point A geometry_msgs::msg::Point representing the starting point from which the x-distance is measured.
 * @param x_distance A double representing the x-distance from the start point where the target point should be found.
 * 
 * @return A geometry_msgs::msg::Point representing the point on the trajectory that is the specified x-distance 
 *         from the start point.
 */
inline geometry_msgs::msg::Point find_x_distance_point_on_line(
        const std::vector<geometry_msgs::msg::Point>& trajectory,
        const geometry_msgs::msg::Point& start_point,
        const double x_distance
) {
    if (trajectory.empty() || x_distance <= 0.0 || trajectory.size() < 2) {
        return start_point;
    }
    double target_x = start_point.x + x_distance;

    double segment_x = 0.0;
    double segment_y = 0.0;
    double target_y = 0.0;
    double t = 0.0;
    
    for (size_t i = 1; i < trajectory.size(); ++i) {
        segment_x = trajectory[i].x - trajectory[i-1].x;
        segment_y = trajectory[i].y - trajectory[i-1].y;

        if (segment_x == 0.0) {
            continue;
        }
        t = (target_x - trajectory[i-1].x) / segment_x;
        target_y = trajectory[i-1].y + t * segment_y;
        if (target_x <= trajectory[0].x || trajectory[i].x >= target_x) {
            break;
        }
    }

    return make_point(target_x, target_y);
}

#endif // POINT_MSG_HELPER_HPP

