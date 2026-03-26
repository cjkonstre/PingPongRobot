#include "player/kalmanFilter/kalmanFilter.h"

Eigen::Matrix<double, 6, 3> KalmanModel::get_kalmanGain(const Eigen::Matrix<double, 3, 3>& sensor_noise) const {
    auto HT = sensor_model.transpose();
    return covariance_mat*HT*(sensor_model*covariance_mat*HT+sensor_noise).inverse();
}


void update(const Eigen::Matrix<double, 3, 1>& measurement_mu, const Eigen::Matrix<double, 3, 3>& measurement_cov) {
    auto K = get_kalmanGain(measurement_cov);
    auto x_k = predict_state();
    auto P_k = predict_covariance();

    state = x_k+K*(measurement_mu-sensor_model*x_k);
    covariance_mat = P_k-K*sensor_model*P_k;
}

void KalmanModel::update() {
    state = predict_state();
    covariance_mat = predict_covariance();
}

Eigen::Matrix<double, 6, 1> KalmanModel::predict_state() const {
    return predict_mat*state+control_mat*control_vec;
}
Eigen::Matrix<double, 6, 6> KalmanModel::predict_covariance() const {
    return predict_mat*covariance_mat*predict_mat.transpose()+noise_covariance;
}

void KalmanModel::apply_mat(Eigen::Matrix<double, 6, 6> mat) {
    state = mat*state;
    covariance_mat=mat*covariance_mat*mat.transpose();
}

void KalmanModel::set_dt(const double& dt) {
    //recomputes mats that depend on dt
    predict_mat = Eigen::Matrix<double, 6, 6>::Identity();
    predict_mat.block<3, 3>(0, 3).diagonal().setConstant(dt);

    control_mat.setZero();
    control_mat.block<3, 3>(0, 0).diagonal().setConstant(0.5*dt*dt);
    control_mat.block<3, 3>(3, 0).diagonal().setConstant(dt);

    double sigma_a2 = noise_sigma; 
    noise_covariance.setZero();
    double dt2 = dt * dt;
    double dt3 = dt2 * dt;
    double dt4 = dt2 * dt2;
    noise_covariance.block<3,3>(0,0).diagonal().setConstant(0.25 * dt4 * sigma_a2);
    noise_covariance.block<3,3>(0,3).diagonal().setConstant(0.5  * dt3 * sigma_a2);
    noise_covariance.block<3,3>(3,0).diagonal().setConstant(0.5  * dt3 * sigma_a2);
    noise_covariance.block<3,3>(3,3).diagonal().setConstant(dt2 * sigma_a2);
}

KalmanModel::KalmanModel(const double& noise_sigma, const double& dt) :
    noise_sigma(noise_sigma) {
    set_dt(dt);
    control_vec << 0, 0, -9.8;
    sensor_model.setZero();
    sensor_model.block<3, 3>(0, 0).setIdentity(); //H, sensor model.
}