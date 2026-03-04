#include "vision/ballDet/ballDet.h"
#include <cmath>

BallDetector::BallDetector(const std::string& configPath) {
    cv::FileStorage fs(configPath, cv::FileStorage::READ);
    fs["mu"] >> mu;
    fs["Sigma"] >> Sigma;
    fs.release();

    if (mu.empty() || Sigma.empty()) {
        throw std::runtime_error("Invalid model contents");
    }


    mu.convertTo(mu, CV_32F);
    Sigma.convertTo(Sigma, CV_32F);
    cv::invert(Sigma, invS, cv::DECOMP_SVD);

    bgSub = cv::createBackgroundSubtractorMOG2(
        200,   // history
        16,    // varThreshold
        false  // no shadow detection
    );
}

bool BallDetector::findBall(
    const cv::Mat& im,
    cv::Point2f& center,
    float& rad,
    bool dobg_masking
) {
    center = {-1, -1};
    rad = -1;
    if (im.empty()) return false;

    cv::Mat frame;
    cv::GaussianBlur(im, frame, cv::Size(7,7), 0);

    cv::Mat fgMask;
    bgSub->apply(frame, fgMask);

    cv::morphologyEx(
        fgMask, fgMask, cv::MORPH_OPEN,
        cv::getStructuringElement(cv::MORPH_ELLIPSE, {3,3})
    );

    cv::Mat lab;
    cv::cvtColor(frame, lab, cv::COLOR_BGR2Lab);

    std::vector<cv::Mat> ch;
    cv::split(lab, ch);

    cv::Mat a, b;
    ch[1].convertTo(a, CV_32F);
    ch[2].convertTo(b, CV_32F);

    cv::Mat ab;
    cv::hconcat(
        a.reshape(1, a.total()),
        b.reshape(1, b.total()),
        ab
    );

    cv::Mat mu_rep;
    cv::repeat(mu, ab.rows, 1, mu_rep);

    cv::Mat d;
    cv::subtract(ab, mu_rep, d);

    cv::Mat temp = d * invS;
    cv::multiply(temp, d, temp);

    cv::Mat score;
    cv::reduce(temp, score, 1, cv::REDUCE_SUM);

    cv::Mat colorMask = score < likelihoodThresh;
    colorMask = colorMask.reshape(1, im.rows);
    colorMask.convertTo(colorMask, CV_8U, 255);

    cv::Mat mask = colorMask;
    if (dobg_masking) cv::bitwise_and(colorMask, fgMask, mask);

    cv::morphologyEx(
        mask, mask, cv::MORPH_OPEN,
        cv::getStructuringElement(cv::MORPH_ELLIPSE, {5,5})
    );
    cv::morphologyEx(
        mask, mask, cv::MORPH_CLOSE,
        cv::getStructuringElement(cv::MORPH_ELLIPSE, {7,7})
    );

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    if (contours.empty()) return false;

    auto best = std::max_element(
        contours.begin(), contours.end(),
        [](const auto& a, const auto& b) {
            return cv::contourArea(a) < cv::contourArea(b);
        }
    );

    if (cv::contourArea(*best) < 50) return false;

    cv::minEnclosingCircle(*best, center, rad);
    return true;
}
