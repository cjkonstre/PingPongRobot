#include "vision/stereo/stereoPair.h"

#include <Eigen/Dense>
#include <opencv2/core/eigen.hpp>


cv::Mat StereoPair::makeProjection(
    const cv::Mat& Kcv,
    const cv::Mat& Rcv,
    const cv::Mat& Tcv)
{
    Eigen::Matrix3d K, R;
    Eigen::Vector3d t;

    cv::cv2eigen(Kcv, K);
    cv::cv2eigen(Rcv, R);
    cv::cv2eigen(Tcv, t);

    Eigen::Matrix<double,3,4> Rt;
    Rt.block<3,3>(0,0) = R;
    Rt.col(3) = t;

    Eigen::Matrix<double,3,4> P = K * Rt;

    cv::Mat proj;
    cv::eigen2cv(P, proj);
    return proj;
}

StereoPair::StereoPair(Camera& c1, Camera& c2, std::string stereoConf_path):
    cam1(c1), cam2(c2) {

    std::string filename = "/home/connor/PingPongRobot/core/config/vision/" + cam1.capName + "+" + cam2.capName + "-stereoConf";
    cv::FileStorage fs(filename, cv::FileStorage::WRITE);
    fs["R"] >> R;
    fs["T"] >> T;
    fs["E"] >> E;
    fs["F"] >> F;
    cv::Mat cov; fs["cov"] >> cov; cv::cv2eigen(cov, covariance);

    proj1 = makeProjection(cam1.K, cv::Mat::eye(3,3,CV_64F), cv::Mat::zeros(3,1,CV_64F));
    proj2 = makeProjection(cam2.K, R, T);
}

GaussBlob<3> StereoPair::get3dMeasurement(
    const cv::Point2f& center1,
    const cv::Point2f& center2
){
    std::vector<cv::Point2f> pts1{center1};
    std::vector<cv::Point2f> pts2{center2};

    cv::Mat pts4D;
    cv::triangulatePoints(proj1, proj2, pts1, pts2, pts4D);

    cv::Mat X = pts4D.col(0);
    double w = X.at<double>(3);

    GaussBlob<3> measurement;

    measurement.mu <<
        X.at<double>(0) / w,
        X.at<double>(1) / w,
        X.at<double>(2) / w;

    measurement.cov = covariance;

    return measurement;
}