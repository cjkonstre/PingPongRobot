#include "player/kalmanFilter/kalmanFilter.h"

Eigen::Matrix<double, 6, 3> KalmanModel::get_kalmanGain(const Eigen::Matrix<double, 3, 3>& sensor_noise) const {
    auto HT = sensor_model.transpose();
    return covariance_mat*HT*(sensor_model*covariance_mat*HT+sensor_noise).inverse();
}


void KalmanModel::combine(const Eigen::Matrix<double, 3, 1>& measurement_mu, 
             const Eigen::Matrix<double, 3, 3>& measurement_cov) {
    auto K = get_kalmanGain(measurement_cov);

    state = state+K*(measurement_mu-sensor_model*state);
    covariance_mat = covariance_mat-K*sensor_model*covariance_mat;
}
void KalmanModel::predict() {
    state = predict_mat*state+control_mat*control_vec;
    covariance_mat= predict_mat*covariance_mat*predict_mat.transpose()+noise_covariance;
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