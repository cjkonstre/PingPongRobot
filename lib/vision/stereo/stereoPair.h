#pragma once

#include "vision/ballDet/ballDet.h"
#include <opencv2/opencv.hpp>
#include <string>
#include "misc/gaussianBlob.h"

//class that handles stereo detection
class StereoPair {
private:
    cv::VideoCapture cap1;
    cv::VideoCapture cap2;
public:
    StereoPair(cv::VideoCapture& cap1, cv::VideoCapture& cap2, const std::string& extrinsics, const BallDetector& balldet);
    StereoPair(const std::string& cap1path, const std::string& cap2path, const BallDetector& balldet);

    GaussBlob<3>
};