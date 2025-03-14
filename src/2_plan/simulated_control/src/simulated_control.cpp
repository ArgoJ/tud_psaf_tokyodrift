#include "simulated_control.h"


SimulatedController::SimulatedController(const SimulatedControlParams& config) 
   : config(config) {
        this->integrator = std::make_unique<Integrator>(this->config.wheelbase);
   }

void SimulatedController::set_control_trajectory(const std::vector<geometry_msgs::msg::Point>& trajectory) {
    std::scoped_lock lock(this->trajectory_mutex_);
    this->control_trajectory = trajectory;
    this->trajectory_index = 0;
    this->current_state = {0.0, 0.0, 0.0, 0.0};
    this->temp_stop = false;
}

void SimulatedController::set_start_stop(const DRIVE_STATE start_stop) {
    std::scoped_lock lock(this->trajectory_mutex_);
    this->start_stop = start_stop;
}

void SimulatedController::set_actual_velocity(const double velocity) {
    std::scoped_lock lock(this->trajectory_mutex_);
    this->actual_velocity = velocity;
}

void SimulatedController::set_actual_input(const BicycleInput input) {
    std::scoped_lock lock(this->trajectory_mutex_);
    this->actual_input = input;
}

BicycleState SimulatedController::get_state() {
    BicycleState local_curr_state;
    {
        std::scoped_lock lock(this->trajectory_mutex_);
        local_curr_state = this->current_state;
        local_curr_state.v = this->actual_velocity;
    }
    return local_curr_state;
}

std::pair<BicycleInput, geometry_msgs::msg::Point> SimulatedController::get_input() {
    if (this->control_trajectory.empty()) {
        return {BicycleInput{0.0, 0.0}, make_point(0.0, 0.0)};
    }

    std::vector<geometry_msgs::msg::Point> local_trajectory;
    BicycleState local_curr_state;
    {
        std::scoped_lock lock(this->trajectory_mutex_);
        this->current_state.v = this->actual_velocity;
        local_curr_state = this->current_state;
        local_trajectory = this->control_trajectory;
    }

    auto [input, target] = this->update_input(local_trajectory, local_curr_state);
    {
        std::scoped_lock lock(this->trajectory_mutex_);
        this->current_input = input;
    }

    return {input, target};
}

BicycleState SimulatedController::simulate_next_state() {
    if (this->control_trajectory.empty()) {
        return this->current_state;
    }

    BicycleInput fused_input = get_fused_input();
    
    BicycleState local_curr_state;
    {
        std::scoped_lock lock(this->trajectory_mutex_);
        local_curr_state = this->current_state;
    }

    BicycleState next_state = this->integrator->integrate_bicycle_model(
        local_curr_state, 
        fused_input, 
        this->config.ts
    );
    next_state.v = std::clamp(next_state.v, this->config.min_speed, this->config.max_speed);
    {
        std::scoped_lock lock(this->trajectory_mutex_);
        this->current_state = next_state;
    }
    // std::cout << "Next State: " << next_state << std::endl;
    return next_state;
}

std::pair<BicycleInput, geometry_msgs::msg::Point> SimulatedController::update_input(
        const std::vector<geometry_msgs::msg::Point>& trajectory,
        const BicycleState& current
) {
    geometry_msgs::msg::Point current_point = state2point(current);
    auto [unit_grad_target, target, distance] = find_min_y_unit_grad(
        trajectory, current_point, this->config.min_lookahead, this->config.max_lookahead
    );

    double velocity_factor = get_velocity_factor(
        trajectory, 
        current_point, 
        this->config.wheelbase, 
        this->config.max_delta,
        this->config.angle_factor_offset
    );

    BicycleInput input;
    input.delta = calculate_delta(unit_grad_target, distance, this->config.wheelbase);
    input.delta = std::clamp(input.delta, this->config.min_delta, this->config.max_delta);
    input.a = this->get_desired_acceleration(current.v, velocity_factor);
    return {input, target};
}

double SimulatedController::get_desired_acceleration(
        const double current_velocity,
        const double velocity_factor
) {
    double target_velocity;
    double min_acceleration = this->config.min_acceleration;
    double max_acceleration = this->config.max_acceleration;

    if (this->start_stop == DRIVE_STATE::FORWARD && !this->temp_stop) {
        target_velocity = this->config.max_speed * velocity_factor;
        if (current_velocity <= 0.0) {min_acceleration = 0.0;}

    } else if (this->start_stop == DRIVE_STATE::BACKWARD && !this->temp_stop) {
        target_velocity = this->config.min_speed * velocity_factor;
        if (current_velocity >= 0.0) {max_acceleration = 0.0;}

    } else if (this->start_stop == DRIVE_STATE::STOP || this->temp_stop) {
        target_velocity = 0.0;
    }
    target_velocity = std::clamp(target_velocity, this->config.min_speed, this->config.max_speed);
    // RCLCPP_DEBUG(this->logger_, "Target Velocity: %f, Current Velocity: %f", target_velocity, current_velocity);
    double desired_acceleration = (target_velocity - current_velocity) / this->ts;
    return std::clamp(desired_acceleration, min_acceleration, max_acceleration);
}

BicycleInput SimulatedController::get_fused_input() {
    BicycleInput local_actual_input;
    BicycleInput local_curr_input;
    {
        std::scoped_lock lock(this->trajectory_mutex_);
        local_actual_input = this->actual_input;
        local_curr_input = this->current_input;
    }
    BicycleInput fused_input;
    fused_input.a = this->config.lambda_use_sensor * local_actual_input.a + (1.0 - this->config.lambda_use_sensor) * local_curr_input.a;
    fused_input.delta = this->config.lambda_use_sensor * local_actual_input.delta + (1.0 - this->config.lambda_use_sensor) * local_curr_input.delta;
    return fused_input;
}

void SimulatedController::set_trajectory_index(const uint32_t index) {
    std::scoped_lock lock(this->trajectory_mutex_);
    this->trajectory_index = index;

    if (this->trajectory_index >= this->control_trajectory.size()) {
        this->temp_stop = true;
    }
}

geometry_msgs::msg::Point SimulatedController::state2point(const BicycleState &state)
{
    geometry_msgs::msg::Point point;
    point.x = state.x;
    point.y = state.y;
    return point;
}