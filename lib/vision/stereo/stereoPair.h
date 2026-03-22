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

    StereoPair(const StereoPair&) = delete;
    StereoPair& operator=(const StereoPair&) = delete;
};