#include <gtest/gtest.h>
#include "filter.hpp"


#include <gtest/gtest.h>
#include "filter.hpp"  // Include the header file of LowpassFilter

class LowpassFilterTest : public ::testing::Test {
protected:
    double tau = 1.0;
    double initial_value = 0.0;
    LowpassFilter filter{tau, initial_value};
};

// Test: The filter should start with the initial value
TEST_F(LowpassFilterTest, InitialValue) {
    EXPECT_DOUBLE_EQ(filter.value(), initial_value);
}

// Test: Update filter with a constant value and check the filtered value
TEST_F(LowpassFilterTest, UpdateConstantValue) {
    double raw_value = 10.0;
    double dt = 1.0;
    filter.update(raw_value, dt);

    // After one update, the filtered value should move towards the raw value
    double expected_value = raw_value * (dt / (tau + dt)) + initial_value * (tau / (tau + dt));
    EXPECT_NEAR(filter.value(), expected_value, 1e-6);  // Allow small floating point error
}

// Test: Apply several updates and check if the filtered value moves towards the raw value
TEST_F(LowpassFilterTest, MultipleUpdates) {
    double raw_value = 10.0;
    double dt = 1.0;
    
    // Apply multiple updates
    filter.update(raw_value, dt);
    double expected_value_after_first_update = raw_value * (dt / (tau + dt)) + initial_value * (tau / (tau + dt));
    EXPECT_NEAR(filter.value(), expected_value_after_first_update, 1e-6);

    filter.update(raw_value, dt);
    double expected_value_after_second_update = raw_value * (dt / (tau + dt)) + expected_value_after_first_update * (tau / (tau + dt));
    EXPECT_NEAR(filter.value(), expected_value_after_second_update, 1e-6);
}

// Test: Edge case where tau is 0.0, this should set the filtered value directly to the raw value
TEST_F(LowpassFilterTest, ZeroTau) {
    filter = LowpassFilter(0.0);  // Reset filter with tau = 0

    double raw_value = 10.0;
    double dt = 1.0;
    double result = filter.update(raw_value, dt);

    EXPECT_DOUBLE_EQ(result, raw_value);  // Filtered value should directly be the raw value
}

// Test: Edge case where dt is 0.0, the filter should not change
TEST_F(LowpassFilterTest, ZeroDt) {
    double raw_value = 10.0;
    double dt = 0.0;
    
    // Initial filtered value
    double initial_filtered_value = filter.value();
    
    // Call update with dt = 0.0
    filter.update(raw_value, dt);

    EXPECT_DOUBLE_EQ(filter.value(), initial_filtered_value);  // Filtered value should stay the same
}

