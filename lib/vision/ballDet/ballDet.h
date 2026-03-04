#pragma once

#include <string>
#include <opencv2/opencv.hpp>

//gaussian model
class BallDetector {
public:
    explicit BallDetector(const std::string& configPath);
    bool findBall(const cv::Mat& im, cv::Point2f& center, float& rad, bool dobg_masking = false);

private:
    cv::Mat mu;      // 1x2
    cv::Mat Sigma;   // 2x2
    cv::Mat invS;    // 2x2
    float likelihoodThresh = 10.f;

    cv::Ptr<cv::BackgroundSubtractor> bgSub;
};
