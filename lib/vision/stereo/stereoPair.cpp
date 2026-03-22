#include "vision/stereo/stereoPair.h"

#include <opencv2/core/eigen.hpp>


StereoPair::StereoPair(Camera& c1, Camera& c2, std::string stereoConf_path):
    cam1(c1), cam2(c2) {

    std::string filename = "/home/connor/PingPongRobot/core/config/vision/" + cam1.capName + "+" + cam2.capName + "-stereoConf";
    cv::FileStorage fs(filename, cv::FileStorage::WRITE);
    fs["R"] >> R;
    fs["T"] >> T;
    fs["E"] >> E;
    fs["F"] >> F;
    cv::Mat cov; fs["cov"] >> cov; cv::cv2eigen(cov, covariance);

    cam1.makeProjection(cam1.intrinsics.K, cv::Mat::eye(3,3,CV_64F), cv::Mat::zeros(3,1,CV_64F));
    cam2.makeProjection(cam2.intrinsics.K, R, T);
}

GaussBlob<3> StereoPair::get3dMeasurement(
    const cv::Point2f& center1,
    const cv::Point2f& center2
){
    std::vector<cv::Point2f> pts1{center1};
    std::vector<cv::Point2f> pts2{center2};

    cv::Mat pts4D;
    cv::triangulatePoints(cam1.proj, cam2.proj, pts1, pts2, pts4D);

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