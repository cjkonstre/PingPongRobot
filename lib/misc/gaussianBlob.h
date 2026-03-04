#pragma once
//here bc both player and visio use it
//just a struct with multiplication, quite simple
//big kalman filter wont use this, but for combining sensor readings in the multistereo its cleaner to do it there

#include <Eigen/Dense>

template <int N>//N is dims
struct GaussBlob {
    Eigen::Matrix<double, N, N> cov; //covariance
    Eigen::Matrix<double, N, 1> mu; //mean

    template <int N>
    GaussBlob<N> operator*(const GaussBlob<N>& a, const GaussBlob<N>& b){
        auto K = a.cov * (a.cov+b.cov).llt().solve(Eigen::Matrix<double, N, N>::Identity());
        GaussBlob<N> c;
        c.mu = a.mu+K*(b.mu-a.mu);
        c.cov = a.cov - K*a.cov();
        return c;
    }

    template <int N>
    GaussBlob<N> operator*(const Eigen::Matrix<double, N, N> mat, const GaussBlob<N>& a){
        GaussBlob<N> c;
        c.mu = mat*a;
        c.cov = mat*cov*mat.transpose();
        return c;
    }
};