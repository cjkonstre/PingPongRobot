#pragma once

#include <string>
#include <opencv2/opencv.hpp>

class BallDetector {
public:
    BallDetector(const std::string& configPath);

    void findBall(const cv::Mat& im, cv::Point2f& center, float& rad);
};