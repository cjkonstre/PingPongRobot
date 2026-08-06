#pragma once

#include "misc/gaussianBlob.h"

class KalmanFilter
{
public:
    explicit KalmanFilter(double dt);

    void predict();
    void update(const GaussBlob<3>& measurement);

    GaussBlob<6>& state() {return state_;}
    const GaussBlob<6>& state() const {return state_;};

    inline GaussBlob<6> snapshot() const {std::lock_guard lk(snap_mtx); return snap;}

private:
    double dt_;

    Eigen::Matrix<double, 6, 6> F_;
    Eigen::Matrix<double, 3, 6> H_;

    Eigen::Matrix<double, 6, 6> Q_;  //process
    Eigen::Matrix<double, 3, 3> R_; //sensor integration

    GaussBlob<6> state_;

    inline void publishSnapshot() {std::lock_guard lk(snap_mtx); snap = state_;}

    mutable std::mutex snap_mtx;
    GaussBlob<6> snap = [] {
        GaussBlob<6> g;
        g.mu.setZero();
        g.cov = Eigen::Matrix<double,6,6>::Identity() * 1e6;
        return g;
    }();
};