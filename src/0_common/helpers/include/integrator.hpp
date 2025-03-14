#ifndef TOKYODRIFT_INTEGRATOR_HPP
#define TOKYODRIFT_INTEGRATOR_HPP

#include <stdint.h>
#include <iostream>
#include <bits/stdc++.h>
#include <cmath>


typedef struct {
    double x;
    double y;
    double omega;
    double v;
} BicycleState;

typedef struct {
    double delta;
    double a;
} BicycleInput;


/**
 * @brief Outputs a BicycleState object to an output stream in a formatted manner.
 *
 * @param os The output stream to which the BicycleState will be written.
 * @param state The BicycleState object to be written to the stream.
 * @return The output stream with the BicycleState written to it.
 */
inline std::ostream& operator<<(std::ostream& os, const BicycleState& state) {
    os << "{ x: " << state.x 
       << ", y: " << state.y 
       << ", theta: " << state.omega 
       << ", v: " << state.v << " }";
    return os;
}

/**
 * @brief Outputs a BicycleInput object to an output stream in a formatted manner.
 *
 * @param os The output stream to which the BicycleInput will be written.
 * @param input The BicycleInput object to be written to the stream.
 * @return The output stream with the BicycleInput written to it.
 */
inline std::ostream& operator<<(std::ostream& os, const BicycleInput& input) {
    os << "{ a: " << input.a 
    << ", delta: " << input.delta << " }";
    return os;
}

/**
 * @brief Compares two BicycleState objects for equality.
 *
 * @param state1 The first BicycleState object to be compared.
 * @param state2 The second BicycleState object to be compared.
 * @return True if the two BicycleState objects are equal, otherwise false.
 */
inline bool operator==(const BicycleState& state1, const BicycleState& state2) {
    return state1.x == state2.x && state1.y == state2.y && state1.omega == state2.omega && state1.v == state2.v;
}

/**
 * @brief Multiplies a BicycleState object by a scalar value.
 *
 * @param scalar The scalar value to multiply the BicycleState by.
 * @param state The BicycleState object to be multiplied.
 * @return A new BicycleState object that is the result of the multiplication.
 */
inline BicycleState operator*(const double scalar, const BicycleState& state) {
    return {
        state.x * scalar, 
        state.y * scalar, 
        state.omega * scalar, 
        state.v * scalar
    };
}

/**
 * @brief Divides a BicycleState object by a scalar value.
 *
 * @param state The BicycleState object to be divided.
 * @param scalar The scalar value to divide the BicycleState by.
 * @return A new BicycleState object that is the result of the division.
 */
inline BicycleState operator/(const BicycleState& state, const double scalar) {
    return {
        state.x / scalar, 
        state.y / scalar,  
        state.omega / scalar, 
        state.v / scalar
    };
}

/**
 * @brief Adds two BicycleState objects together.
 *
 * @param state1 The first BicycleState object to be added.
 * @param state2 The second BicycleState object to be added.
 * @return A new BicycleState object that is the result of the addition.
 */
inline BicycleState operator+(const BicycleState& state1, const BicycleState& state2) {
    return {
        state1.x + state2.x, 
        state1.y + state2.y, 
        state1.omega + state2.omega, 
        state1.v + state2.v
    };
}


/**
 * @brief Class to integrate a bicycle model over time using a specified wheelbase.
 */
class Integrator {
private:
    const double& wheelbase_;

public:
    /**
     * @brief Constructs an Integrator object with the given wheelbase.
     *
     * @param wheelbase The wheelbase of the bicycle used for model integration.
     */
    explicit Integrator(const double& wheelbase) : wheelbase_(wheelbase) {}

    Integrator(const Integrator&) = delete;
    Integrator& operator=(const Integrator&) = delete;

     /**
     * @brief Integrates the bicycle model over a given timestep using 4. Runge-Kutta method.
     *
     * @param state The current state of the bicycle.
     * @param input The current input to the bicycle.
     * @param ts The timestep over which to integrate.
     * @return The new BicycleState after integration.
     */
    BicycleState integrate_bicycle_model(
        const BicycleState& state,
        const BicycleInput& input,
        double ts
    );

    /**
     * @brief Computes the derivative of the bicycle model at a given state and input.
     *
     * @param state The current state of the bicycle.
     * @param input The current input to the bicycle.
     * @return The derivative of the bicycle state.
     */
    BicycleState bicycle_model(
        const BicycleState& state,
        const BicycleInput& input
    );
};


inline BicycleState Integrator::integrate_bicycle_model(
        const BicycleState& state,
        const BicycleInput& input,
        double ts
) {
    BicycleState k1 = ts * this->bicycle_model(state, input);
    BicycleState k2 = ts * this->bicycle_model(state + k1 / 2.0, input);
    BicycleState k3 = ts * this->bicycle_model(state + k2 / 2.0, input);
    BicycleState k4 = ts * this->bicycle_model(state + k3, input);
    BicycleState next_state = state + k1 / 6. + k2 / 3. + k3 / 3. + k4 / 6.;
    return next_state;
}


inline BicycleState Integrator::bicycle_model(
        const BicycleState& state,
        const BicycleInput& input
) {
    BicycleState derivative;
    derivative.x = state.v * std::cos(state.omega);
    derivative.y = state.v * std::sin(state.omega);
    derivative.omega = state.v * std::tan(input.delta) / this->wheelbase_;
    derivative.v = input.a;
    return derivative;
}


#endif //TOKYODRIFT_INTEGRATOR_HPP