#pragma once

#include "vision/ballDet/ballDet.h"
#include "vision/camera/camera.h"
#include <string>
#include "misc/gaussianBlob.h"

//class that handles stereo detection
class StereoPair {
private:
    Camera& cam1;
    Camera& cam2;

    cv::Mat R, T, E, F;
    Eigen::Matrix<double, 3, 3> covariance; //sensor noise
public:
    //takes in cam args for intrinsics / info, not to operate on them
    StereoPair(Camera& cam1, Camera& cam2, 
                std::string stereoConf_path);

    GaussBlob<3> get3dMeasurement(const cv::Point2f& center1, const cv::Point2f& center2);

    //solver
    static bool calibratePnP(
        const std::vector<cv::Point3f>& objectPoints,
        const std::vector<cv::Point2f>& imagePoints1,
        const std::vector<cv::Point2f>& imagePoints2,
        const Camera& cam1,
        const Camera& cam2,
        cv::Mat& R,
        cv::Mat& T,
        cv::Mat& E,
        cv::Mat& F
    );

    //solver, but saves cali to file directl
    static bool calibrateToFile(
        const std::vector<cv::Point3f>& objectPoints,
        const std::vector<cv::Point2f>& imagePoints1,
        const std::vector<cv::Point2f>& imagePoints2,
        Camera& cam1,
        Camera& cam2,
        const std::string& outputPath
    );

    StereoPair(const StereoPair&) = delete;
    StereoPair& operator=(const StereoPair&) = delete;
};