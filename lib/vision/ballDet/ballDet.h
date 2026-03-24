#pragma once

#include <string>
#include <opencv2/opencv.hpp>

//#define DO_MAHALANOBIS

#ifndef DO_MAHALANOBIS
//gaussian model
class BallDetector {
public:
    explicit BallDetector(const std::string& configPath, const std::string& camName);
    bool findBall(const cv::Mat& im, cv::Point2f& center, float& rad, bool dobg_masking = false);
    bool findBall(
        const cv::Mat& im,
        cv::Point2f& center,
        float& rad,
        cv::Rect roi);

private:
    cv::Mat mu;      // 1x2
    cv::Mat Sigma;   // 2x2
    cv::Mat invS;    // 2x2
    float likelihoodThresh = 10.f;
    cv::Mat L;      // whitening transform

    cv::Ptr<cv::BackgroundSubtractor> bgSub;
};

#else

//mahanoblis or whatev
//parrellellize, or can implelmt lookup table, ie precompute what values of lab are the ball (or a proablility)
//also, should probably implelment some sort of lighting variation adaptivity. balls are glossy, so somehting to fix that in harsh lighting would be great
class BallDetector {

private:

    cv::Vec3f mu;
    cv::Matx33f invS;

    float threshold;

    cv::Ptr<cv::BackgroundSubtractor> bgSub;

    float mahalanobis(const cv::Vec3f& x) const;

public:

    BallDetector(const std::string& configPath,
                 const std::string& camName);

    bool findBall(
        const cv::Mat& im,
        cv::Point2f& center,
        float& rad,
        bool dobg_masking = false
    ) const;

    //only searches for the ball in the roi
    //careful that the center points to it in the full image, not the cropped
    bool findBall(
        const cv::Mat& im,
        cv::Point2f& center,
        float& rad,
        cv::Rect roi
    ) const;
};

#endif