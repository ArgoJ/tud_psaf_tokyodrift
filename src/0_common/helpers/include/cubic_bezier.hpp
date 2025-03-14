#ifndef CUBIC_BEZIER_HPP
#define CUBIC_BEZIER_HPP

#include <vector>
#include <geometry_msgs/msg/point.hpp>

#include "point_msg_helper.hpp"



/**
 * @brief Computes the value of a cubic Bézier curve at a given parameter t.
 * 
 * This function evaluates a cubic Bézier curve based on four control points and a parameter t (0 <= t <= 1).
 * 
 * @param p0 First control point.
 * @param p1 Second control point.
 * @param p2 Third control point.
 * @param p3 Fourth control point.
 * @param t Parameter of the curve, typically between 0 and 1.
 * 
 * @return The value of the cubic Bézier curve at t.
 */
inline double cubic_bezier(
        const double p0,
        const double p1,
        const double p2,
        const double p3,
        const double t
) {
    const double one_minus_t = 1.0 - t;
    return one_minus_t * one_minus_t * one_minus_t * p0
            + 3 * one_minus_t * one_minus_t * t * p1
            + 3 * one_minus_t * t * t * p2
            + t * t * t * p3;
}

/**
 * @brief Computes the first derivative of a cubic Bezier curve at a given parameter t.
 * 
 * This function calculates the rate of change of the cubic Bezier curve at the point defined by parameter t,
 * using the control points p0, p1, p2, and p3.
 *
 * @param p0 The first control point.
 * @param p1 The second control point.
 * @param p2 The third control point.
 * @param p3 The fourth control point.
 * @param t The parameter (0 <= t <= 1) at which the derivative is evaluated.
 * @return The first derivative of the cubic Bezier curve at parameter t.
 */
inline double d_cubic_bezier(
        const double p0,
        const double p1,
        const double p2,
        const double p3,
        const double t
) {
    const double one_minus_t = 1.0 - t;
    return 3.0 * one_minus_t * one_minus_t * (p1 - p0)
            + 6.0 * one_minus_t * t * (p2 - p1)
            + 3.0 * t * t * (p3 - p2);
}

/**
 * @brief Computes the second derivative of a cubic Bezier curve at a given parameter t.
 * 
 * This function calculates the acceleration (second derivative) of the cubic Bezier curve at the point defined by parameter t,
 * using the control points p0, p1, p2, and p3.
 *
 * @param p0 The first control point.
 * @param p1 The second control point.
 * @param p2 The third control point.
 * @param p3 The fourth control point.
 * @param t The parameter (0 <= t <= 1) at which the second derivative is evaluated.
 * @return The second derivative of the cubic Bezier curve at parameter t.
 */
inline double d2_cubic_bezier(
        const double p0,
        const double p1,
        const double p2,
        const double p3,
        const double t
) {
    const double one_minus_t = 1.0 - t;
    return 6.0 * one_minus_t * (p2 - 2.0 * p1 + p0)
            + 6.0 * t * (p3 - 2.0 * p2 + p1);
}

/**
 * @brief Computes the maximum curvature of a cubic Bezier curve based on control points.
 * 
 * This function estimates the maximum curvature along the cubic Bezier curve, using control points and a sampling step size ts.
 * It evaluates the curvature at several points along the curve and returns the largest value.
 *
 * @param control_points A vector of four control points that define the cubic Bezier curve.
 * @param ts The step size for sampling along the curve (default is 0.01).
 * @return The maximum curvature of the cubic Bezier curve.
 */
inline double compute_max_curvature(
        const std::vector<geometry_msgs::msg::Point>& control_points,
        const double ts = 0.01
) {
    double max_kappa = 0.0;
    for (double t = 0.0; t <= 1.0; t += ts) {
        double dx = d_cubic_bezier(
            control_points[0].x, control_points[1].x, control_points[2].x, control_points[3].x, t
        );
        double d2x = d2_cubic_bezier(
            control_points[0].x, control_points[1].x, control_points[2].x, control_points[3].x, t
        );
        double dy = d_cubic_bezier(
            control_points[0].y, control_points[1].y, control_points[2].y, control_points[3].y, t
        );
        double d2y = d2_cubic_bezier(
            control_points[0].y, control_points[1].y, control_points[2].y, control_points[3].y, t
        );

        double kappa = (dx * d2y - dy * d2x) / pow(dx * dx + dy * dy, 1.5);
        max_kappa = std::max(max_kappa, std::abs(kappa));
    }
    return max_kappa;
}

/**
 * @brief Generates the control points for a cubic Bezier curve based on start and target points.
 * 
 * This function generates four control points for a cubic Bezier curve, given the start and target points,
 * their corresponding unit gradients, and a distance fraction to control the curve's tension.
 *
 * @param start The starting point of the curve.
 * @param start_unit_grad The unit gradient at the start point.
 * @param target The target point of the curve.
 * @param target_unit_grad The unit gradient at the target point.
 * @param distance_fraction A fraction of the distance between the start and target points to control the curve's tension.
 * @return A vector of four control points defining the cubic Bezier curve.
 */
inline std::vector<geometry_msgs::msg::Point> get_control_points(
        const geometry_msgs::msg::Point& start,
        const geometry_msgs::msg::Point& start_unit_grad,
        const geometry_msgs::msg::Point& target,
        const geometry_msgs::msg::Point& target_unit_grad,
        const double distance_fraction
) {
    double distance = get_segment_length(start, target) * distance_fraction;
    // First two control points on y=0 line
    geometry_msgs::msg::Point p0 = start;
    geometry_msgs::msg::Point p1 = walk_gradient(start, start_unit_grad, distance);

    // Third control and last point on gradient
    geometry_msgs::msg::Point p2 = walk_gradient(target, target_unit_grad, distance);
    geometry_msgs::msg::Point p3 = target;
    return {p0, p1, p2, p3};
}

/**
 * @brief Selects the optimal target point from a trajectory based on multiple criteria.
 * 
 * This function iterates over a trajectory of points, computes the control points for a cubic Bezier curve for each candidate target,
 * and selects the target that minimizes the cost function based on distance and curvature constraints.
 *
 * @param trajectory A vector of candidate points representing the trajectory.
 * @param start The starting point of the curve.
 * @param start_unit_grad The unit gradient at the start point.
 * @param distance_fraction A fraction of the distance between the start and target points.
 * @param kappa_max The maximum allowed curvature for the optimal target.
 * @param desired_distance The desired distance to the target.
 * @param w_distance The weight factor for the distance in the cost function.
 * @param w_kappa The weight factor for the curvature in the cost function.
 * @return A pair containing the optimal target point and its index in the trajectory.
 */
inline std::pair<geometry_msgs::msg::Point, size_t> select_optimal_target(
        const std::vector<geometry_msgs::msg::Point>& trajectory,
        const geometry_msgs::msg::Point& start,           // e.g., zero_point
        const geometry_msgs::msg::Point& start_unit_grad,   // e.g., zero_unit_grad
        const double distance_fraction,
        const double kappa_max,
        const double desired_distance,    // could be total trajectory length or other target
        const double w_distance,          // weight for distance
        const double w_kappa              // weight for curvature
) {
    double best_cost = std::numeric_limits<double>::infinity();
    geometry_msgs::msg::Point best_candidate;
    size_t best_idx = 0;
    bool candidate_found = false;

    // Iterate over each candidate point in the trajectory.
    for (size_t i = 0; i < trajectory.size(); ++i) {
        const auto& candidate = trajectory[i];

        // Estimate gradient at candidate:
        geometry_msgs::msg::Point candidate_grad;
        if (i == 0) {
            candidate_grad = get_unit_gradient(trajectory[i+1], candidate);
        } else if (i == trajectory.size()-1) {
            candidate_grad = get_unit_gradient(candidate, trajectory[i-1]);
        } else {
            candidate_grad = get_unit_gradient(trajectory[i+1], trajectory[i-1]);
        }

        // Compute control points for the candidate Bézier curve.
        auto candidate_control_points = get_control_points(
            start, start_unit_grad, candidate, candidate_grad, distance_fraction
        );

        // Compute maximum curvature along this candidate Bézier curve.
        double candidate_max_kappa = compute_max_curvature(candidate_control_points);

        // Only consider candidates that satisfy the curvature constraint.
        if (candidate_max_kappa > kappa_max) {
            continue;
        }

        // Compute segment distance from the start to the candidate.
        double candidate_distance = get_segment_length(start, candidate);

        // Define a cost function that penalizes candidates that are far from the desired distance
        // and that have high curvature (normalized to kappa_max).
        double distance_residual = desired_distance - candidate_distance;
        double cost = w_distance * distance_residual * distance_residual +
                    w_kappa * candidate_max_kappa * candidate_max_kappa;

        if (cost < best_cost) {
            best_cost = cost;
            best_candidate = candidate;
            best_idx = i;
            candidate_found = true;
        }
    }

    if (!candidate_found) {
        return std::make_pair(start, best_idx); // some fallback behavior
    }
    return std::make_pair(best_candidate, best_idx);
}

/**
 * @brief Computes the point on a cubic Bezier curve for specific control points at a given parameter t.
 * 
 * This function evaluates the cubic Bezier curve for specific control points at a parameter t using the provided control points,
 * and returns the corresponding point on the curve.
 *
 * @param control_points A vector of four control points defining the cubic Bezier curve.
 * @param t The parameter (0 <= t <= 1) at which the point is evaluated.
 * @return The point on the cubic Bezier curve at the given parameter t.
 */
inline geometry_msgs::msg::Point get_cubic_bezier_point(
    const std::vector<geometry_msgs::msg::Point>& control_points, 
    const double t
) {
    geometry_msgs::msg::Point result;
    result.x = cubic_bezier(
        control_points[0].x, control_points[1].x, control_points[2].x, control_points[3].x, t
    );
    result.y = cubic_bezier(
        control_points[0].y, control_points[1].y, control_points[2].y, control_points[3].y, t
    );

    return result;
}

/**
 * @brief Computes the points along a cubic Bezier curve for a range of t values.
 * 
 * This function samples points along the cubic Bezier curve at regular intervals defined by the step size ts,
 * and returns a vector of points along the curve.
 *
 * @param control_points A vector of four control points defining the cubic Bezier curve.
 * @param ts The step size for sampling along the curve.
 * @return A vector of points sampled along the cubic Bezier curve.
 */
inline std::vector<geometry_msgs::msg::Point> compute_cubic_bezier(
        const std::vector<geometry_msgs::msg::Point>& control_points,
        const double ts
) {
    std::vector<geometry_msgs::msg::Point> bezier_trajectory;
    size_t num_points = static_cast<size_t>(1.0 / ts) + 1;
    bezier_trajectory.reserve(num_points);

    // Generate points along the bezier curve
    for (size_t i = 0; i < num_points; ++i) {
        double t = i * ts;
        bezier_trajectory.push_back(get_cubic_bezier_point(control_points, t));
    }
    return bezier_trajectory;
}

/**
 * @brief Computes a complete cubic Bézier trajectory with a static lookahead from a given set of waypoints (trajectory) 
 * and other parameters.
 * 
 * 
 * This function generates a cubic Bézier curve by calculating control points based on the provided trajectory, 
 * distance, and other parameters, and appends the remaining trajectory if necessary. It uses a static lookahead 
 * to find the last control point.
 * It returns a pair of vectors: the complete cubic Bézier trajectory and the corresponding control points.
 *
 * @param trajectory A vector of geometry_msgs::msg::Point representing the trajectory.
 * @param distance A double representing the distance to determine the starting point of the Bezier curve.
 * @param distance_fraction A double representing the fraction of the distance to influence the control points for the cubic Bezier.
 * @param ts A double representing the time step for the cubic Bezier curve.
 * 
 * @return A pair of vectors containing the complete trajectory and the control points used to generate the cubic Bezier curve.
 *         If the function cannot generate a cubic Bezier curve or process the trajectory, it returns empty vectors.
 */
inline std::pair<std::vector<geometry_msgs::msg::Point>, std::vector<geometry_msgs::msg::Point>> get_cubic_bezier_complete_static(
        const std::vector<geometry_msgs::msg::Point>& trajectory, 
        const double distance,
        const double distance_fraction,
        const double ts
) {
    if (trajectory.size() < 2) {
        return {};
    }

    geometry_msgs::msg::Point zero_point = make_point(0.0, 0.0);
    geometry_msgs::msg::Point zero_unit_grad = make_point(1.0, 0.0);
    auto idx = find_top_index(trajectory, zero_point, distance, 0);

    size_t idx_value;
    if (!idx.has_value()) {
        double distance_fist_point = get_segment_length(zero_point, trajectory[0]);
        double distance_last_point = get_segment_length(zero_point, trajectory.back());
        if (distance_fist_point > distance) {
            idx_value = 1;
        } else if (distance_last_point < distance) {
            idx_value = trajectory.size() - 1;
        } else {
            return std::make_pair(std::vector<geometry_msgs::msg::Point>(), std::vector<geometry_msgs::msg::Point>());
        }
    } else {
        idx_value = idx.value();
    }

    const geometry_msgs::msg::Point& top_point = trajectory[idx_value];
    const geometry_msgs::msg::Point& bot_point = trajectory[idx_value - 1];

    geometry_msgs::msg::Point target = find_target_between_points(top_point, bot_point, zero_point, distance);
    geometry_msgs::msg::Point negative_unit_gradient = get_unit_gradient(top_point, bot_point);

    std::vector<geometry_msgs::msg::Point> control_points = get_control_points(
        zero_point, zero_unit_grad, target, negative_unit_gradient, distance_fraction
    );

    if (control_points.empty()) {
        return std::make_pair(std::vector<geometry_msgs::msg::Point>(), std::vector<geometry_msgs::msg::Point>());
    }

    std::vector<geometry_msgs::msg::Point> complete_trajectory = compute_cubic_bezier(control_points, ts);
    if (idx_value < trajectory.size()) {
        complete_trajectory.reserve(complete_trajectory.size() + trajectory.size() - idx_value);
        for (size_t i = idx_value; i < trajectory.size(); ++i) {
            complete_trajectory.push_back(trajectory[i]);
        }
    }

    return std::make_pair(complete_trajectory, control_points);
}

/**
 * @brief Computes a complete cubic Bézier trajectory with an optimal problem from a given set of waypoints (trajectory) 
 * and other parameters.
 * 
 * This function generates a cubic Bézier curve by calculating control points based on the provided trajectory, 
 * distance, and other parameters, and appends the remaining trajectory if necessary. It uses an optimal Problem 
 * to find the last control point.
 * It returns a pair of vectors: the complete cubic Bézier trajectory and the corresponding control points.
 * 
 * @param trajectory A vector of geometry_msgs::msg::Point representing the initial trajectory.
 * @param distance The total distance to be used in trajectory calculation.
 * @param distance_fraction The fraction of the total distance to be used for trajectory generation.
 * @param ts The time step for the cubic Bézier curve computation.
 * @param kappa_max The maximum curvature allowed for the generated trajectory (default is 3.0).
 * @param w_distance The weight applied to the distance in the optimization process (default is 3.0).
 * @param w_kappa The weight applied to the curvature in the optimization process (default is 1.0).
 * 
 * @return A pair of vectors, where the first is the complete cubic Bézier trajectory and the second is the set of control points.
 *         If the trajectory size is less than 2, an empty pair is returned.
 */
inline std::pair<std::vector<geometry_msgs::msg::Point>, std::vector<geometry_msgs::msg::Point>> get_cubic_bezier_complete(
        const std::vector<geometry_msgs::msg::Point>& trajectory, 
        const double distance,
        const double distance_fraction,
        const double ts,
        const double kappa_max = 3.0,
        const double w_distance = 3.0,
        const double w_kappa = 1.0
) {
    if (trajectory.size() < 2) {
        return {};
    }

    geometry_msgs::msg::Point zero_point = make_point(0.0, 0.0);
    geometry_msgs::msg::Point zero_unit_grad = make_point(1.0, 0.0);

    auto [target, idx] = select_optimal_target(
        trajectory, zero_point, zero_unit_grad, distance_fraction, kappa_max, distance, w_distance, w_kappa
    );

    geometry_msgs::msg::Point negative_unit_gradient;
    if (idx < trajectory.size() - 1) {
        negative_unit_gradient = get_unit_gradient(trajectory[idx + 1], target);
    } else if (idx == trajectory.size() - 1) {
        negative_unit_gradient = get_unit_gradient(target, trajectory[idx - 1]);
    }

    std::vector<geometry_msgs::msg::Point> control_points = get_control_points(
        zero_point, zero_unit_grad, target, negative_unit_gradient, distance_fraction
    );

    if (control_points.empty()) {
        return std::make_pair(std::vector<geometry_msgs::msg::Point>(), std::vector<geometry_msgs::msg::Point>());
    }

    std::vector<geometry_msgs::msg::Point> complete_trajectory = compute_cubic_bezier(control_points, ts);
    if (idx < trajectory.size()) {
        complete_trajectory.reserve(complete_trajectory.size() + trajectory.size() - idx);
        for (size_t i = idx; i < trajectory.size(); ++i) {
            complete_trajectory.push_back(trajectory[i]);
        }
    }

    return std::make_pair(complete_trajectory, control_points);
}


#endif // CUBIC_BEZIER_HPP