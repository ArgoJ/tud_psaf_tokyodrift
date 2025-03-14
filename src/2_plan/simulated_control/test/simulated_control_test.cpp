#include <gtest/gtest.h>
#include "simulated_control.h"


class SimulatedControllerTest : public ::testing::Test {
protected:
    SimulatedControlParams params;
    std::unique_ptr<SimulatedController> controller;

    void SetUp() override {
        // Dummy-Konfiguration – passe die Werte so an, wie es in deinem Einsatzszenario passt.
        params.wheelbase = 0.258;
        params.ts = 0.01;
        params.min_lookahead = 1.0;
        params.max_lookahead = 1.2;
        params.min_delta = -0.3;
        params.max_delta = 0.3;
        params.max_speed = 2.0;
        params.min_speed = 0.0;
        params.min_acceleration = -2.0;
        params.max_acceleration = 2.0;
        params.lambda_use_sensor = 0.2;
        params.angle_factor_offset = 0.3;

        controller = std::make_unique<SimulatedController>(params);
    }
};

// Testet, dass beim Abruf der Eingabe ohne gesetzte Trajektorie ein Standardwert zurückgegeben wird.
TEST_F(SimulatedControllerTest, GetInputWithEmptyTrajectory) {
    auto [input, target] = controller->get_input();
    EXPECT_DOUBLE_EQ(input.a, 0.0);
    EXPECT_DOUBLE_EQ(input.delta, 0.0);
    EXPECT_DOUBLE_EQ(target.x, 0.0);
    EXPECT_DOUBLE_EQ(target.y, 0.0);
}

// Testet, dass set_control_trajectory den internen Zustand zurücksetzt.
TEST_F(SimulatedControllerTest, GetInput) {
    std::vector<geometry_msgs::msg::Point> trajectory = {
        make_point(0.0, 0.0),
        make_point(1.0, 0.0),
        make_point(2.0, 0.0)
    };

    controller->set_control_trajectory(trajectory);

    auto [input, target] = controller->get_input();
    EXPECT_GT(input.a, 0.0);
    EXPECT_EQ(input.delta, 0.0);
    EXPECT_GT(target.x, 0.0);
    EXPECT_EQ(target.y, 0.0);
}

// Testet, dass set_control_trajectory den internen Zustand zurücksetzt.
TEST_F(SimulatedControllerTest, MultipleStepsRightTurn) {
    std::vector<geometry_msgs::msg::Point> trajectory = {
        make_point(0.0, 0.0),
        make_point(1.0, 0.5),
        make_point(2.0, 0.8)
    };

    controller->set_control_trajectory(trajectory);

    auto [input1, target1] = controller->get_input();
    EXPECT_GT(input1.a, 0.0);
    EXPECT_GT(input1.delta, 0.0);
    EXPECT_GT(target1.x, 0.0);
    EXPECT_GT(target1.y, 0.0);

    auto current_state = controller->simulate_next_state();

    auto [input2, target2] = controller->get_input();
    EXPECT_GT(input2.a, 0.0);
    EXPECT_GT(input2.delta, 0.0);
    EXPECT_GT(target2.x, target1.x);
    EXPECT_GT(target2.y, target1.y);

    EXPECT_GT(current_state.x, 0.0);
    EXPECT_GT(current_state.y, 0.0);
    EXPECT_GT(current_state.omega, 0.0);
    EXPECT_GT(current_state.v, 0.0);

    EXPECT_LE(current_state.x, 0.5 * input1.a * params.ts * params.ts);
    EXPECT_LE(current_state.y, 0.5 * input1.a * params.ts * params.ts);
    EXPECT_LE(current_state.omega, input1.delta);
    EXPECT_LE(current_state.v, input1.a * params.ts);
}

TEST_F(SimulatedControllerTest, MultipleStepsLeftTurn) {
    std::vector<geometry_msgs::msg::Point> trajectory = {
        make_point(0.0, 0.0),
        make_point(1.0, -0.5),
        make_point(2.0, -0.8)
    };

    controller->set_control_trajectory(trajectory);

    auto [input1, target1] = controller->get_input();
    EXPECT_GT(input1.a, 0.0);
    EXPECT_LT(input1.delta, 0.0);
    EXPECT_GT(target1.x, 0.0);
    EXPECT_LT(target1.y, 0.0);

    auto current_state = controller->simulate_next_state();

    auto [input2, target2] = controller->get_input();
    EXPECT_GT(input2.a, 0.0);
    EXPECT_LT(input2.delta, 0.0);
    EXPECT_GT(target2.x, target1.x);
    EXPECT_LT(target2.y, target1.y);

    EXPECT_GT(current_state.x, 0.0);
    EXPECT_LT(current_state.y, 0.0);
    EXPECT_LT(current_state.omega, 0.0);
    EXPECT_GT(current_state.v, 0.0);

    EXPECT_LE(current_state.x, 0.5 * input1.a * params.ts * params.ts);
    EXPECT_GE(current_state.y, - 0.5 * input1.a * params.ts * params.ts);
    EXPECT_GE(current_state.omega, input1.delta);
    EXPECT_LE(current_state.v, input1.a * params.ts);
}

// Testet simulate_next_state, wobei die Geschwindigkeit innerhalb der Grenzen bleiben soll.
TEST_F(SimulatedControllerTest, SimulateNextStateClamping) {
    // Setze eine einfache Trajektorie, sodass simulate_next_state arbeitet
    std::vector<geometry_msgs::msg::Point> trajectory;
    geometry_msgs::msg::Point p;
    p.x = 0.0;
    p.y = 0.0;
    trajectory.push_back(p);
    controller->set_control_trajectory(trajectory);
    
    // Setze eine zu hohe Geschwindigkeit, die geklammert werden soll
    controller->set_actual_velocity(12.0);  // 12 > max_speed (10)
    auto [input, target] = controller->get_input();
    BicycleState next_state = controller->simulate_next_state();
    EXPECT_LE(next_state.v, params.max_speed);
    EXPECT_GE(next_state.v, params.min_speed);
}
