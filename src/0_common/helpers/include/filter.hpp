#ifndef TOKYODRIFT_FILTER_HPP
#define TOKYODRIFT_FILTER_HPP

#include <cmath>



class LowpassFilter {
public:
    /**
     * @class LowpassFilter
     * 
     * A class representing a low-pass filter that smooths incoming data values 
     * based on a time constant `tau_`. The filter updates its output on each call 
     * to `update()` using the raw input value and time step `dt`. The output 
     * represents the filtered version of the input data.
     */

    /**
     * @brief Constructor for the LowpassFilter class.
     * 
     * Initializes the filter with a given time constant `tau` and an optional 
     * initial value for the filtered value. If no initial value is provided, 
     * it defaults to 0.0.
     * 
     * @param tau The time constant that defines the cutoff frequency of the filter.
     * @param initial_value The initial value for the filtered signal (default is 0.0).
     */
    LowpassFilter(double tau, double initial_value = 0.0)
        : tau_(tau), filtered_value_(initial_value) {}

    /**
     * @brief Updates the filtered value based on the raw input and time step.
     * 
     * Calculates the new filtered value using the raw input `raw_value` and 
     * the time step `dt`. The calculation incorporates the time constant `tau_`.
     * If the time step `dt` is zero or negative relative to `tau_`, the raw value 
     * is returned directly as the filtered value. Otherwise, the filter applies 
     * the low-pass filter equation.
     * 
     * @param raw_value The raw input value to be filtered.
     * @param dt The time step since the last update.
     * 
     * @return The updated filtered value.
     */
    double update(double raw_value, double dt) {
        if ((tau_ + dt) == 0.0) {
            filtered_value_ = raw_value;
        } else {
            double alpha = dt / (tau_ + dt);
            filtered_value_ += alpha * (raw_value - filtered_value_);
        }
        return filtered_value_;
    }

    /**
     * @brief Returns the current filtered value.
     * 
     * Provides the current value of the filtered signal after any updates.
     * 
     * @return The current filtered value.
     */
    double value() const { return filtered_value_; }

private:
    double tau_;
    double filtered_value_;
};

#endif //TOKYODRIFT_FILTER_HPP