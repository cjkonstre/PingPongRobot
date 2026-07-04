#pragma once

#include <opencv2/opencv.hpp>
#include <boost/circular_buffer.hpp>
#include <thread>
#include <atomic>
#include <mutex>

#include "vision/camera/camera.h"

struct BallDetection {
    bool found = false;
    cv::Point2f center;
    float radius = 0.f;
    uint64_t timestamp_us = 0;
};

class BallDetector {
public:
    BallDetector(const std::string& configPath, const std::string& camName);

    void beginLoop(Camera& cam);
    void endLoop();

    bool latestDetection(BallDetection& out) const;

private:
    bool findBall(const cv::Mat& im, cv::Point2f& center, float& rad, bool bg);
    bool findBall(const cv::Mat& im, cv::Point2f& center, float& rad);
    bool findBall(const cv::Mat& im, cv::Point2f& center, float& rad, cv::Rect roi);

    void loop();

private:
    Camera* cam = nullptr;

    std::thread worker;
    std::atomic<bool> running{false};

    mutable std::mutex det_mtx;
    boost::circular_buffer<BallDetection> det_buf{5};

    BallDetection lastDet;

    // --- model ---
    cv::Mat mu, Sigma, invS, L;

    cv::Ptr<cv::BackgroundSubtractor> bgSub;

    // --- reused buffers ---
    cv::Mat blurred, hsv, hsvMask, fgMask, combined;
    cv::Mat kernel;

    // --- constants ---
    static constexpr int LOW_H = 5, HIGH_H = 25;
    static constexpr int LOW_S = 100, HIGH_S = 255;
    static constexpr int LOW_V = 100, HIGH_V = 255;

    static constexpr float MIN_R = 5.f;
    static constexpr float MAX_R = 40.f;
    static constexpr double MIN_AREA = 50.0;
    static constexpr double MIN_CIRC = 0.2;
};