#include "vision/kalmanFilter/kalmanFilter.h"

KalmanFilter::KalmanFilter(double dt)
    : dt_(dt)
{
    // State transition matrix
    F_.setIdentity();
    F_(0,3) = dt_;
    F_(1,4) = dt_;
    F_(2,5) = dt_;

    // Measurement matrix
    H_.setZero();
    H_(0,0) = 1.0;
    H_(1,1) = 1.0;
    H_(2,2) = 1.0;

    // Initial state
    state_.mu.setZero();
    state_.cov.setIdentity();
    state_.cov *= 0.1; //init cov

    // Noise matrices
    Q_.setIdentity();
    Q_ *= 0.001;

    R_.setIdentity();
    R_ *= 0.0001;
}

void KalmanFilter::predict()
{
    state_.mu = F_ * state_.mu;
    state_.cov = F_ * state_.cov * F_.transpose() + Q_;
    publishSnapshot();
}

void KalmanFilter::update(const GaussBlob<3>& measurement)
{
    // Kalman gain
    Eigen::Matrix<double, 6, 3> K =
        state_.cov * H_.transpose() *
        (H_ * state_.cov * H_.transpose() + R_).inverse();

    // Innovation
    state_.mu += K * (measurement.mu - H_ * state_.mu);

    // Covariance update
    state_.cov =
        (Eigen::Matrix<double, 6, 6>::Identity() - K * H_) * state_.cov;
    
    publishSnapshot();
}