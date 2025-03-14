#ifndef TOKYODRIFT_KALMAN_ESTIMATOR_HPP
#define TOKYODRIFT_KALMAN_ESTIMATOR_HPP

#include <Eigen/Dense>
#include <cmath>


/**
 * @class KalmanStateEstimator
 * @brief A class that implements a Kalman filter for estimating the state of a system.
 * 
 * The Kalman filter estimates the system's state, which includes velocity, acceleration, 
 * and angular velocity, using input measurements from an IMU (accelerometer and gyroscope) 
 * and a hall effect sensor. The filter operates with a constant time step and incorporates 
 * noise models for each sensor.
 */
class KalmanStateEstimator {
public:

    /**
     * @brief Constructor to initialize the Kalman filter with a time step and optional initial state values.
     * 
     * @param dt The time step (in seconds) for the filter.
     * @param initial_velocity The initial velocity estimate (default is 0.0).
     * @param initial_acceleration The initial acceleration estimate (default is 0.0).
     * @param initial_ang_vel The initial angular velocity estimate (default is 0.0).
     */
    KalmanStateEstimator(double dt, double initial_velocity = 0.0, double initial_acceleration = 0.0, double initial_ang_vel = 0.0)
        : dt_(dt),
            state_(Eigen::Vector3d(initial_velocity, initial_acceleration, initial_ang_vel)),
            process_noise_covariance_(Eigen::Matrix3d::Identity() * 0.0075),
            imu_acceleration_noise_(0.18),
            imu_ang_vel_noise_(0.05),
            hall_effect_noise_(0.01),
            P_(Eigen::Matrix3d::Identity() * 0.1)
            {}

    /**
     * @brief Predicts the next state of the system based on the current state and the system's model.
     * 
     * This function uses the state transition model to predict the next state and update the 
     * state covariance matrix.
     * 
     * @return The predicted state vector [velocity, acceleration, angular velocity].
     */
    Eigen::Vector3d predict() {
        // State Transition Model
        Eigen::Matrix3d F = Eigen::Matrix3d::Identity();
        F(0, 1) = dt_;  // v += a * dt

        state_ = F * state_;
        P_ = F * P_ * F.transpose() + process_noise_covariance_;

        return state_;
    }

     /**
     * @brief Updates the state estimate based on IMU measurements (acceleration and angular velocity).
     * 
     * The update step incorporates IMU data (accelerometer and gyroscope readings) into the 
     * Kalman filter to correct the current state estimate.
     * 
     * @param imu_acceleration The measured acceleration from the IMU.
     * @param imu_ang_vel The measured angular velocity from the IMU.
     * @return The updated state vector [velocity, acceleration, angular velocity].
     */
    Eigen::Vector3d update_imu(double imu_acceleration, double imu_ang_vel) {
        Eigen::Matrix<double, 2, 3> H_imu;
        H_imu << 0, 1, 0,  // Measure acceleration
                 0, 0, 1;  // Measure ang_vel

        Eigen::Vector2d z_imu(imu_acceleration, imu_ang_vel);
        Eigen::Vector2d y_imu = z_imu - H_imu * state_;

        Eigen::Matrix2d R_imu = Eigen::Matrix2d::Zero();
        R_imu(0,0) = imu_acceleration_noise_;
        R_imu(1,1) = imu_ang_vel_noise_;
        Eigen::Matrix2d S_imu = H_imu * P_ * H_imu.transpose() + R_imu;

        Eigen::Matrix<double, 3, 2> K_imu = P_ * H_imu.transpose() * S_imu.inverse();

        state_ = state_ + K_imu * y_imu;
        P_ = (Eigen::Matrix3d::Identity() - K_imu * H_imu) * P_;

        return state_;
    }

    /**
     * @brief Updates the state estimate based on the hall effect sensor measurement (velocity).
     * 
     * The update step incorporates velocity measurements from the hall effect sensor into 
     * the Kalman filter to correct the current state estimate.
     * 
     * @param hall_velocity The measured velocity from the hall effect sensor.
     * @return The updated state vector [velocity, acceleration, angular velocity].
     */
    Eigen::Vector3d update_hall(double hall_velocity) {
        Eigen::Matrix<double, 1, 3> H_hall;
        H_hall << 1, 0, 0;  // Measure velocity

        double y_hall = hall_velocity - H_hall * state_;

        double S_hall = H_hall * P_ * H_hall.transpose() + hall_effect_noise_;
        Eigen::Vector3d K_hall = P_ * H_hall.transpose() / S_hall;

        state_ = state_ + K_hall * y_hall;
        P_ = (Eigen::Matrix3d::Identity() - K_hall * H_hall) * P_;

        return state_;
    }

    /**
     * @brief Retrieves the current estimated velocity.
     * 
     * @return The current estimated velocity.
     */
    double get_velocity() const { return state_(0); }

    /**
     * @brief Retrieves the current estimated acceleration.
     * 
     * @return The current estimated acceleration.
     */
    double get_acceleration() const { return state_(1); }

     /**
     * @brief Retrieves the current estimated angular velocity.
     * 
     * @return The current estimated angular velocity.
     */
    double get_ang_vel() const { return state_(2); }

private:
    double dt_;
    Eigen::Vector3d state_;    // [v, a, ang_vel]
    Eigen::Matrix3d process_noise_covariance_;
    double imu_acceleration_noise_;
    double imu_ang_vel_noise_;
    double hall_effect_noise_;
    Eigen::Matrix3d P_; 
};

#endif //TOKYODRIFT_KALMAN_ESTIMATOR_HPP