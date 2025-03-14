#include <gtest/gtest.h>
#include "point_msg_helper.hpp"
#include <geometry_msgs/msg/point.hpp>
#include <cmath>
#include <optional>

// Test suite for PointMsgHelper functions
class PointMsgHelperTest : public ::testing::Test {};

// Test: walk_gradient should move the point according to the unit gradient and distance
TEST_F(PointMsgHelperTest, WalkGradientTest) {
    geometry_msgs::msg::Point current = make_point(0.0, 0.0);
    geometry_msgs::msg::Point unit_gradient = make_point(1.0, 0.0);  // Moving along x-axis
    double distance = 5.0;

    geometry_msgs::msg::Point next_point = walk_gradient(current, unit_gradient, distance);

    EXPECT_NEAR(next_point.x, 5.0, 1e-6);
    EXPECT_NEAR(next_point.y, 0.0, 1e-6);
}

// Test: get_segment_length should return the correct distance between two points
TEST_F(PointMsgHelperTest, GetSegmentLengthTest) {
    geometry_msgs::msg::Point p1 = make_point(0.0, 0.0);
    geometry_msgs::msg::Point p2 = make_point(3.0, 4.0);

    double length = get_segment_length(p1, p2);

    EXPECT_NEAR(length, 5.0, 1e-6);  // 3-4-5 triangle, distance = 5
}

// Test: get_squared_segment_length should return the squared length of the segment
TEST_F(PointMsgHelperTest, GetSquaredSegmentLengthTest) {
    geometry_msgs::msg::Point p1 = make_point(0.0, 0.0);
    geometry_msgs::msg::Point p2 = make_point(3.0, 4.0);

    double squared_length = get_squared_segment_length(p1, p2);

    EXPECT_NEAR(squared_length, 25.0, 1e-6);  // 3^2 + 4^2 = 25
}

// Test: get_unit_gradient should compute the unit vector between two points
TEST_F(PointMsgHelperTest, GetUnitGradientTest) {
    geometry_msgs::msg::Point p1 = make_point(0.0, 0.0);
    geometry_msgs::msg::Point p2 = make_point(1.0, 1.0);

    geometry_msgs::msg::Point unit_gradient = get_unit_gradient(p1, p2);

    EXPECT_NEAR(unit_gradient.x, 1.0 / std::sqrt(2.0), 1e-6);
    EXPECT_NEAR(unit_gradient.y, 1.0 / std::sqrt(2.0), 1e-6);
}

// Test: get_unit_gradient should compute the unit vector from angle
TEST_F(PointMsgHelperTest, GetUnitGradientAngleTest) {
    double angle = 30.0 * M_PI / 180.0;

    geometry_msgs::msg::Point unit_gradient = get_unit_gradient(angle);

    EXPECT_NEAR(unit_gradient.x, std::sqrt(3.0) / 2.0, 1e-6);
    EXPECT_NEAR(unit_gradient.y, 0.5, 1e-6);
}

// Test: calculate_normalized_angle should return the correct angle between points
TEST_F(PointMsgHelperTest, CalculateNormalizedAngleTest) {
    geometry_msgs::msg::Point p1 = make_point(0.0, 0.0);
    geometry_msgs::msg::Point p2 = make_point(1.0, 2.0);

    double angle = calculate_normalized_angle(p1, p2);

    EXPECT_NEAR(angle, 1.10714872, 1e-6); // 
}

// Test: get_walking_distance should compute the walking distance correctly
TEST_F(PointMsgHelperTest, GetWalkingDistanceTest) {
    geometry_msgs::msg::Point current_point = make_point(0.0, 0.0);
    geometry_msgs::msg::Point top_point = make_point(10.0, -1.0);
    geometry_msgs::msg::Point bot_point = make_point(0.0, 1.0);

    double distance = 5.0;
    double walking_distance = get_walking_distance(current_point, top_point, bot_point, distance);

    EXPECT_NEAR(walking_distance, std::sqrt(26.0), 1e-6);
}

// Test: find_target_between_points should return the correct target point
TEST_F(PointMsgHelperTest, FindTargetBetweenPointsTest) {
    // Test case 1: Point between top and bottom distance 
    {
        geometry_msgs::msg::Point top_point = make_point(0.0, 1.0);
        geometry_msgs::msg::Point bot_point = make_point(5.0, 6.0);
        geometry_msgs::msg::Point start_point = make_point(0.0, 1.0);
        double distance = 5.0;

        geometry_msgs::msg::Point target = find_target_between_points(top_point, bot_point, start_point, distance);
        double factor_45_grad = std::sqrt(2.0) / 2.0;
        double expected_x = factor_45_grad * distance;
        double expected_y = 1.0 + factor_45_grad * distance;

        EXPECT_NEAR(target.x, expected_x, 1e-6);
        EXPECT_NEAR(target.y, expected_y, 1e-6);
    }

    // Test case 2: Vertical line
    {
        geometry_msgs::msg::Point top_point = make_point(0.0, 10.0);
        geometry_msgs::msg::Point bot_point = make_point(0.0, 2.0);
        geometry_msgs::msg::Point start_point = make_point(0.0, 0.0);
        double distance = 5.0;

        geometry_msgs::msg::Point target = find_target_between_points(top_point, bot_point, start_point, distance);

        EXPECT_NEAR(target.x, 0.0, 1e-6);
        EXPECT_NEAR(target.y, 5.0, 1e-6);
    }

    // Test case 3: Horizontal line
    {
        geometry_msgs::msg::Point top_point = make_point(10.0, 0.0);
        geometry_msgs::msg::Point bot_point = make_point(2.0, 0.0);
        geometry_msgs::msg::Point start_point = make_point(0.0, 0.0);
        double distance = 5.0;

        geometry_msgs::msg::Point target = find_target_between_points(top_point, bot_point, start_point, distance);

        EXPECT_NEAR(target.x, 5.0, 1e-6);
        EXPECT_NEAR(target.y, 0.0, 1e-6);
    }

    // Test case 4: start_point offset
    {
        geometry_msgs::msg::Point top_point = make_point(10.0, 2.0);
        geometry_msgs::msg::Point bot_point = make_point(0.0, 0.0);
        geometry_msgs::msg::Point start_point = make_point(0.0, 1.0);
        double distance = 5.0;

        geometry_msgs::msg::Point target = find_target_between_points(top_point, bot_point, start_point, distance);

        EXPECT_NEAR(target.x, 5.0, 1e-6);
        EXPECT_NEAR(target.y, 1.0, 1e-6);
    }

    // Test case 5: No intersection
    {
        geometry_msgs::msg::Point top_point = make_point(10.0, 0.0);
        geometry_msgs::msg::Point bot_point = make_point(9.0, 2.0);
        geometry_msgs::msg::Point start_point = make_point(0.0, 1.0);
        double distance = 5.0;

        geometry_msgs::msg::Point target = find_target_between_points(top_point, bot_point, start_point, distance);

        EXPECT_DOUBLE_EQ(target.x, bot_point.x);
        EXPECT_DOUBLE_EQ(target.y, bot_point.y);
    }

    // Test case 6: Both points in circle
    {
        geometry_msgs::msg::Point top_point = make_point(0.0, 0.0);
        geometry_msgs::msg::Point bot_point = make_point(1.0, 1.0);
        geometry_msgs::msg::Point start_point = make_point(0.0, 0.0);
        double distance = 5.0;

        geometry_msgs::msg::Point target = find_target_between_points(top_point, bot_point, start_point, distance);

        EXPECT_NEAR(target.x, 1.0, 1e-6);
        EXPECT_NEAR(target.y, 1.0, 1e-6);
    }
}

// Test: find_top_index should return the correct index of the point that exceeds the given distance
TEST_F(PointMsgHelperTest, FindTopIndexTest) {
    std::vector<geometry_msgs::msg::Point> trajectory = {
        make_point(0.0, 0.0),
        make_point(1.0, 0.0),
        make_point(3.0, 0.0)
    };
    geometry_msgs::msg::Point start_point = make_point(0.0, 0.0);
    double distance = 2.5;

    std::optional<size_t> index = find_top_index(trajectory, start_point, distance);

    EXPECT_TRUE(index.has_value());
    EXPECT_EQ(index.value(), 2);
}

// Test: find_point_on_line should find the point on the trajectory line at the given distance
TEST_F(PointMsgHelperTest, FindPointOnLineTest) {
    std::vector<geometry_msgs::msg::Point> trajectory = {
        make_point(0.0, 0.0),
        make_point(1.0, 0.0),
        make_point(2.0, 0.0)
    };
    geometry_msgs::msg::Point start_point = make_point(0.0, 0.0);
    double distance = 1.0;

    geometry_msgs::msg::Point point_on_line = find_point_on_line(trajectory, start_point, distance);

    EXPECT_NEAR(point_on_line.x, 1.0, 1e-6);
    EXPECT_NEAR(point_on_line.y, 0.0, 1e-6);
}

// Unit test for move_trajectory
TEST_F(PointMsgHelperTest, BasicTrajectoryMoveTrajectoryTest) {
    std::vector<geometry_msgs::msg::Point> trajectory = {
        make_point(0.0, 0.0),
        make_point(1.0, 0.0),
        make_point(2.0, 0.0),
        make_point(3.0, 0.0)
    };

    double distance = 1.0;
    auto result = move_trajectory(trajectory, distance);

    ASSERT_EQ(result.size(), 2);
    EXPECT_NEAR(result[0].x, 1.0, 1e-6);
    EXPECT_NEAR(result[0].y, 1.0, 1e-6);
    EXPECT_NEAR(result[1].x, 2.0, 1e-6);
    EXPECT_NEAR(result[1].y, 1.0, 1e-6);
}

// Unit test for move_trajectory
TEST_F(PointMsgHelperTest, TiltedTrajectoryMoveTrajectoryTest) {
    std::vector<geometry_msgs::msg::Point> trajectory = {
        make_point(0.0, 0.0),
        make_point(1.0, 0.5),
        make_point(2.0, 2.0),
        make_point(3.0, 4.0)
    };

    double distance = 1.0;
    auto result = move_trajectory(trajectory, distance);

    double prev_angle = std::atan2(trajectory[1].y - trajectory[0].y, trajectory[1].x - trajectory[0].x);
    double next_angle = std::atan2(trajectory[2].y - trajectory[1].y, trajectory[2].x - trajectory[1].x);
    double avg_angle = (prev_angle + next_angle) / 2.0;
    double offset_angle = avg_angle + M_PI_2;

    geometry_msgs::msg::Point expected_first = make_point(
        trajectory[1].x + std::cos(offset_angle) * distance,
        trajectory[1].y + std::sin(offset_angle) * distance
    );

    prev_angle = std::atan2(trajectory[2].y - trajectory[1].y, trajectory[2].x - trajectory[1].x);
    next_angle = std::atan2(trajectory[3].y - trajectory[2].y, trajectory[3].x - trajectory[2].x);
    avg_angle = (prev_angle + next_angle) / 2.0;
    offset_angle = avg_angle + M_PI_2;

    geometry_msgs::msg::Point expected_second = make_point(
        trajectory[2].x + std::cos(offset_angle) * distance,
        trajectory[2].y + std::sin(offset_angle) * distance
    );

    ASSERT_EQ(result.size(), 2);
    EXPECT_NEAR(result[0].x, expected_first.x, 1e-6);
    EXPECT_NEAR(result[0].y, expected_first.y, 1e-6);
    EXPECT_NEAR(result[1].x, expected_second.x, 1e-6);
    EXPECT_NEAR(result[1].y, expected_second.y, 1e-6);
}

TEST_F(PointMsgHelperTest, EmptyTrajectoryMoveTrajectoryTest) {
    std::vector<geometry_msgs::msg::Point> trajectory;
    double distance = 1.0;
    auto result = move_trajectory(trajectory, distance);
    ASSERT_TRUE(result.empty());
}

TEST_F(PointMsgHelperTest, SinglePointTrajectoryMoveTrajectoryTest) {
    std::vector<geometry_msgs::msg::Point> trajectory = {make_point(0.0, 0.0)};
    double distance = 1.0;
    auto result = move_trajectory(trajectory, distance);
    ASSERT_TRUE(result.empty());
}

// Test: calculate_delta should return the correct delta (45°) value for steering
TEST_F(PointMsgHelperTest, CalculateDeltaTest) {
    geometry_msgs::msg::Point unit_gradient_45 = make_point(0.5, 0.5);
    double distance = 2.0;
    double wheelbase = 0.258;

    double delta_45 = calculate_delta(unit_gradient_45, distance, wheelbase);
    double true_delta = std::atan2(2 * wheelbase * std::sin(M_PI_4), distance);
    EXPECT_NEAR(delta_45, true_delta, 1e-6);
}

// Test: calculate_delta should return the correct delta (0°) value for steering
TEST_F(PointMsgHelperTest, ZeroCalculateDeltaTest) {
    geometry_msgs::msg::Point unit_gradient_0 = make_point(1.0, 0.0);
    double distance = 2.0;
    double wheelbase = 0.258;

    double delta_0 = calculate_delta(unit_gradient_0, distance, wheelbase);
    EXPECT_NEAR(delta_0, 0.0, 1e-6);
}

// Test: find_y_zero_crossing_between_two_points should compute the zero crossing point
TEST_F(PointMsgHelperTest, FindZeroCrossingBetweenTwoPointsTest) {
    geometry_msgs::msg::Point p1 = make_point(0.0, -1.0);
    geometry_msgs::msg::Point p2 = make_point(1.0, 1.0);

    geometry_msgs::msg::Point zero_crossing = find_y_zero_crossing_between_two_points(p1, p2);

    EXPECT_NEAR(zero_crossing.x, 0.5, 1e-6);  // Y should be 0 for the zero crossing point
    EXPECT_NEAR(zero_crossing.y, 0.0, 1e-6);  // X should be the arctangent of the angle
}

// Test: find_min_y_unit_grad should compute the zero crossing point
TEST_F(PointMsgHelperTest, ZeroCrossFindMinYUnitGradTest) {
    std::vector<geometry_msgs::msg::Point> traj = {
        make_point(0.0, -0.8),
        make_point(1.0, -0.5),
        make_point(2.0, 0.5),
        make_point(3.0, 0.8)
    };
    geometry_msgs::msg::Point start_point = make_point(0.0, 0.0);
    double bot_distance = 1.0;
    double top_distance = 2.0;

    auto [unit_grad, target, distance] = find_min_y_unit_grad(traj, start_point, bot_distance, top_distance);

    EXPECT_NEAR(unit_grad.x, 1.0, 1e-6); 
    EXPECT_NEAR(unit_grad.y, 0.0, 1e-6);
    EXPECT_NEAR(distance, 1.5, 1e-6); 
}

// Test: find_min_y_unit_grad should compute the zero crossing unit gradient
TEST_F(PointMsgHelperTest, RightSideFindMinYUnitGradTest) {
    std::vector<geometry_msgs::msg::Point> traj = {
        make_point(0.0, 0.0),
        make_point(1.5, 2.0),
        make_point(2.0, 4.0),
        make_point(3.0, 5.0)
    };
    geometry_msgs::msg::Point start_point = make_point(0.0, 0.0);
    double bot_distance = 1.0;
    double top_distance = 2.0;

    auto [unit_grad, target, distance] = find_min_y_unit_grad(traj, start_point, bot_distance, top_distance);

    auto expected_unit_grad = get_unit_gradient(start_point, make_point(1.5, 2.0));
    auto expected_point = find_point_on_line(traj, start_point, top_distance);
    auto expected_distance = get_segment_length(start_point, expected_point);

    EXPECT_NEAR(unit_grad.x, expected_unit_grad.x, 1e-6); 
    EXPECT_NEAR(unit_grad.y, expected_unit_grad.y, 1e-6);
    EXPECT_NEAR(target.x, expected_point.x, 1e-6); 
    EXPECT_NEAR(target.y, expected_point.y, 1e-6);
    EXPECT_NEAR(distance, expected_distance, 1e-6);
}

// Test: find_min_y_unit_grad should compute the zero crossing unit gradient
TEST_F(PointMsgHelperTest, LeftSideFindMinYUnitGradTest) {
    std::vector<geometry_msgs::msg::Point> traj = {
        make_point(0.0, 0.0),
        make_point(1.5, -2.0),
        make_point(2.0, -4.0),
        make_point(3.0, -5.0)
    };
    geometry_msgs::msg::Point start_point = make_point(0.0, 0.0);
    double bot_distance = 1.0;
    double top_distance = 2.0;

    auto [unit_grad, target, distance] = find_min_y_unit_grad(traj, start_point, bot_distance, top_distance);

    auto expected_unit_grad = get_unit_gradient(start_point, make_point(1.5, -2.0));
    auto expected_point = find_point_on_line(traj, start_point, top_distance);
    auto expected_distance = get_segment_length(start_point, expected_point);

    EXPECT_NEAR(unit_grad.x, expected_unit_grad.x, 1e-6); 
    EXPECT_NEAR(unit_grad.y, expected_unit_grad.y, 1e-6);
    EXPECT_NEAR(target.x, expected_point.x, 1e-6); 
    EXPECT_NEAR(target.y, expected_point.y, 1e-6);
    EXPECT_NEAR(distance, expected_distance, 1e-6);
}

// Test: find_min_y_unit_grad should compute the zero crossing unit gradient
TEST_F(PointMsgHelperTest, EmptyTrajFindMinYUnitGradTest) {
    std::vector<geometry_msgs::msg::Point> traj;
    geometry_msgs::msg::Point start_point = make_point(0.0, 0.0);
    double bot_distance = 1.0;
    double top_distance = 2.0;

    auto [unit_grad, target, distance] = find_min_y_unit_grad(traj, start_point, bot_distance, top_distance);

    EXPECT_DOUBLE_EQ(unit_grad.y, 0.0); 
    EXPECT_DOUBLE_EQ(unit_grad.x, 0.0);
    EXPECT_DOUBLE_EQ(target.y, 0.0); 
    EXPECT_DOUBLE_EQ(target.x, 0.0);
    EXPECT_DOUBLE_EQ(distance, 0.0);
}

TEST_F(PointMsgHelperTest, FindXDistancePointOnLine) {
    using geometry_msgs::msg::Point;
    
    // Create test trajectory
    std::vector<Point> trajectory = {
        make_point(0.0, 0.0),    // Point 0
        make_point(1.0, 1.0),    // Point 1
        make_point(2.0, 0.0),    // Point 2
        make_point(3.0, -1.0),   // Point 3
        make_point(4.0, 0.0)     // Point 4
    };

    // Test case 1: Empty trajectory
    {
        std::vector<Point> empty_trajectory;
        Point start = make_point(0.0, 0.0);
        Point result = find_x_distance_point_on_line(empty_trajectory, start, 1.0);
        EXPECT_DOUBLE_EQ(result.x, start.x);
        EXPECT_DOUBLE_EQ(result.y, start.y);
    }

    // Test case 2: Negative or zero distance
    {
        Point start = make_point(0.0, 0.0);
        Point result = find_x_distance_point_on_line(trajectory, start, -1.0);
        EXPECT_DOUBLE_EQ(result.x, start.x);
        EXPECT_DOUBLE_EQ(result.y, start.y);
    }

    // Test case 3: Point exactly on trajectory points
    {
        Point start = make_point(0.0, 0.0);
        Point result = find_x_distance_point_on_line(trajectory, start, 2.0);
        EXPECT_DOUBLE_EQ(result.x, 2.0);
        EXPECT_DOUBLE_EQ(result.y, 0.0);
    }

    // Test case 4: Point between trajectory points
    {
        Point start = make_point(0.0, 0.0);
        Point result = find_x_distance_point_on_line(trajectory, start, 1.5);
        EXPECT_DOUBLE_EQ(result.x, 1.5);
        EXPECT_DOUBLE_EQ(result.y, 0.5); // Linear interpolation between (1,1) and (2,0)
    }

    // Test case 5: Point beyond trajectory end
    {
        Point start = make_point(0.0, 0.0);
        Point result = find_x_distance_point_on_line(trajectory, start, 5.0);
        EXPECT_DOUBLE_EQ(result.x, 5.0);
        EXPECT_DOUBLE_EQ(result.y, 1.0); // Extrapolated using last segment gradient
    }

    // Test case 6: Start point not at trajectory beginning
    {
        Point start = make_point(1.0, 0.0);
        Point result = find_x_distance_point_on_line(trajectory, start, 2.0);
        EXPECT_DOUBLE_EQ(result.x, 3.0);
        EXPECT_DOUBLE_EQ(result.y, -1.0);
    }

    // Test case 1: Empty trajectory
    {
        std::vector<Point> empty_trajectory;
        Point start = make_point(0.0, 0.0);
        Point result = find_x_distance_point_on_line(empty_trajectory, start, 1.0);
        EXPECT_DOUBLE_EQ(result.x, start.x);
        EXPECT_DOUBLE_EQ(result.y, start.y);
    }

    // Test case 2: Negative or zero distance
    {
        Point start = make_point(0.0, 0.0);
        Point result = find_x_distance_point_on_line(trajectory, start, -1.0);
        EXPECT_DOUBLE_EQ(result.x, start.x);
        EXPECT_DOUBLE_EQ(result.y, start.y);
    }

    // Test case 3: Point exactly on trajectory points
    {
        Point start = make_point(0.0, 0.0);
        Point result = find_x_distance_point_on_line(trajectory, start, 2.0);
        EXPECT_DOUBLE_EQ(result.x, 2.0);
        EXPECT_DOUBLE_EQ(result.y, 0.0);
    }

    // Test case 4: Point between trajectory points
    {
        Point start = make_point(0.0, 0.0);
        Point result = find_x_distance_point_on_line(trajectory, start, 1.5);
        EXPECT_DOUBLE_EQ(result.x, 1.5);
        EXPECT_DOUBLE_EQ(result.y, 0.5); // Linear interpolation between (1,1) and (2,0)
    }

    // Test case 5: Point beyond trajectory end
    {
        Point start = make_point(0.0, 0.0);
        Point result = find_x_distance_point_on_line(trajectory, start, 5.0);
        EXPECT_DOUBLE_EQ(result.x, 5.0);
        EXPECT_DOUBLE_EQ(result.y, 1.0); // Extrapolated using last segment gradient
    }

    // Test case 6: Start point not at trajectory beginning
    {
        Point start = make_point(1.0, 0.0);
        Point result = find_x_distance_point_on_line(trajectory, start, 2.0);
        EXPECT_DOUBLE_EQ(result.x, 3.0);
        EXPECT_DOUBLE_EQ(result.y, -1.0);
    }

    // Test case 7: Single point trajectory
    {
        std::vector<Point> single_point = {make_point(0.0, 0.0)};
        Point start = make_point(0.0, 0.0);
        Point result = find_x_distance_point_on_line(single_point, start, 1.0);
        EXPECT_DOUBLE_EQ(result.x, start.x);
        EXPECT_DOUBLE_EQ(result.y, start.y);
    }

    // Test case 8: Point before trajectory start
    {
        Point start = make_point(-2.0, 0.0);
        Point result = find_x_distance_point_on_line(trajectory, start, 1.0);
        EXPECT_DOUBLE_EQ(result.x, -1.0);
        EXPECT_DOUBLE_EQ(result.y, -1.0);
    }
}