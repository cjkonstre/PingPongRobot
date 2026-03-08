#pragma once

#include <opencv2/opencv.hpp>
#include <string>

class Camera {
private:
    std::string capPath;
    std::string intrinsicsPath;

    cv::VideoCapture cap;

public:
    struct Intrinsics {
        cv::Mat K;   // camera atrix
        cv::Mat D;   // distotion coefficients
    };

    Intrinsics intrinsics;

    std::string capName;

    Camera(const std::string& capPath,
           const std::string& intrinsicsPath,
           int frameWidth,
           int frameHeight,
           int fps,
           int exposureSetting);

    bool grab();
    cv::Mat retrieve();

    inline void read(cv::Mat& frame) const;
};