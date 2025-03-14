#include <gtest/gtest.h>
#include "integrator.hpp"


class IntegratorTest : public ::testing::Test {
    protected:
        double wheelbase = 0.258;
        Integrator integrator{wheelbase};
    };
    
    TEST_F(IntegratorTest, BicycleModelDerivative) {
        BicycleState state{0.0, 0.0, 0.0, 2.0}; // x, y, omega, v
        BicycleInput input{0.1, 2.0};           // delta, a
    
        BicycleState derivative = integrator.bicycle_model(state, input);
    
        EXPECT_DOUBLE_EQ(derivative.x, 2.0);  // v * cos(0) = 10
        EXPECT_DOUBLE_EQ(derivative.y, 0.0);   // v * sin(0) = 0
        EXPECT_DOUBLE_EQ(derivative.omega, 2.0 * std::tan(0.1) / wheelbase);
        EXPECT_DOUBLE_EQ(derivative.v, 2.0);   // acceleration directly affects v
    }
    
    TEST_F(IntegratorTest, IntegrationStep) {
        BicycleState state{0.0, 0.0, 0.0, 2.0}; // Initial state
        BicycleInput input{0.1, 2.0};           // Input
        double ts = 0.1;                        // Time step
    
        BicycleState next_state = integrator.integrate_bicycle_model(state, input, ts);
    
        EXPECT_NEAR(next_state.x, 0.2, 0.05);  // Expected position after small step
        EXPECT_NEAR(next_state.y, 0.0, 0.05);  // Should be close to 0
        EXPECT_NEAR(next_state.omega, 2.0 * std::tan(0.1) / wheelbase * ts, 0.05);
        EXPECT_NEAR(next_state.v, 2.2, 0.05); // v should increase slightly
    }
    