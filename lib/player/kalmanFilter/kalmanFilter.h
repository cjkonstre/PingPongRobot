//code for spatial kalman filter, ie 3 dims of space and 3 dims of velocity

#pragma once

#include <Eigen/Dense>

//model to estimate state of system
//linear, so unfortunately nonlinear effects arent accounted for, like drag and spin effects.
//these non linear second order effects should have a very small time step factor though, so
// they should be very small compared to sensor noise. i.e. covariant matrix is large in comparison to the uncertanty due 
// to these second order factors, so it shouldnt make a ton of difference. also, the process noise may absorb some of this error.
//maybe some way that something could learn the dynamics of the system
class KalmanModel {
    Eigen::Matrix<double, 6, 6> predict_mat; //time stepping matrix. dt dep
    Eigen::Matrix<double, 6, 3> control_mat; //dt dep
    Eigen::Matrix<double, 3, 1> control_vec; 
    double noise_sigma; //process noise?
    Eigen::Matrix<double, 6, 6> noise_covariance; //unknown forces, process noise. dt dep
    Eigen::Matrix<double, 3, 6> sensor_model; 
    Eigen::Matrix<double, 3, 3> sensor_noise; 

    double dt;

    Eigen::Matrix<double, 6, 3> get_kalmanGain(const Eigen::Matrix<double, 3, 3>& sensor_noise) const; //returns simplified kalman gain (K'), dependent on currect covariant matrix

    Eigen::Matrix<double, 6, 1> predict_state() const; // w/ dt
    Eigen::Matrix<double, 6, 6> predict_covariance() const; // w/ dt
public:
    KalmanModel(const double& noise_sigma, const double& dt); //3d etimation noise, process noise

    void apply_mat(Eigen::Matrix<double, 6, 6> mat); //for sudden changes, ie bouncing

    Eigen::Matrix<double, 6, 1> state;
    Eigen::Matrix<double, 6, 6> covariance_mat; //covariance matrix

    void set_dt(const double& dt); //sets all relevant matricies as well to match dt timestep

    void update(const Eigen::Matrix<double, 3, 1>& measurement_mu, const Eigen::Matrix<double, 3, 3>& measurement_cov); //updates internal state w/ measurement, w/ dt
    void update(); //updates internals w/o measurement.
};