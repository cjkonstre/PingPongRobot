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

private:
    double dt_;

    Eigen::Matrix<double, 6, 6> F_;
    Eigen::Matrix<double, 3, 6> H_;

    Eigen::Matrix<double, 6, 6> Q_;
    Eigen::Matrix<double, 3, 3> R_;

    GaussBlob<6> state_;
};