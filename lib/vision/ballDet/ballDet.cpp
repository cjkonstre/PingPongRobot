#include "vision/ballDet/ballDet.h"
#include <cmath>
#include <nlohmann/json.hpp>
using json = nlohmann::json;

#ifndef DO_MAHALANOBIS

cv::Mat jsonToMat(const json& j)
{
    int rows = j.at("rows");
    int cols = j.at("cols");

    std::vector<float> data = j.at("data").get<std::vector<float>>();

    if (data.size() != rows * cols)
        throw std::runtime_error("jsonToMat: data size mismatch");

    cv::Mat m(rows, cols, CV_32F);
    std::memcpy(m.ptr<float>(), data.data(), rows * cols * sizeof(float));

    return m;
}

BallDetector::BallDetector(const std::string& configPath,
                           const std::string& camName)
{
    // ignore config for now
    bgSub = cv::createBackgroundSubtractorMOG2(
        100,   // history
        16,    // varThreshold
        false  // no shadows
    );
}
bool BallDetector::findBall(
    const cv::Mat& im,
    cv::Point2f& center,
    float& rad,
    bool dobg_masking
) {
    if (im.empty()) return false;

    // --- params (hardcoded for now) ---
    static const int lowH = 5,  highH = 25;
    static const int lowS = 100, highS = 255;
    static const int lowV = 100,  highV = 255;

    static const float MIN_RADIUS = 5.0f;
    static const float MAX_RADIUS = 40.0f;

    static const double MIN_AREA = 50.0;
    static const double MIN_CIRCULARITY = 0.2;

    // --- buffers ---
    static thread_local cv::Mat blurred, hsv, hsvMask, fgMask, combined;
    static thread_local cv::Mat kernel =
        cv::getStructuringElement(cv::MORPH_ELLIPSE, {3,3});

    // --- preprocess ---
    cv::GaussianBlur(im, blurred, {5,5}, 0);

    // --- background mask ---
    if (dobg_masking) {
        bgSub->apply(blurred, fgMask);
        cv::morphologyEx(fgMask, fgMask, cv::MORPH_CLOSE, kernel, {-1,-1}, 2);
    } else {
        fgMask = cv::Mat(im.size(), CV_8U, cv::Scalar(255));
    }

    // --- HSV mask ---
    cv::cvtColor(blurred, hsv, cv::COLOR_BGR2HSV);
    cv::inRange(hsv,
        cv::Scalar(lowH, lowS, lowV),
        cv::Scalar(highH, highS, highV),
        hsvMask);

    // --- combine ---
    cv::bitwise_and(hsvMask, fgMask, combined);

    // cleanup
    cv::morphologyEx(combined, combined, cv::MORPH_OPEN, kernel);

    // --- contours ---
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(combined, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    if (contours.empty()) return false;

    // --- select best candidate ---
    float bestRadius = 0.0f;
    double bestCircularity = 0.0;
    cv::Point2f bestCenter;

    for (const auto& contour : contours) {
        double area = cv::contourArea(contour);
        if (area < MIN_AREA) continue;

        double perimeter = cv::arcLength(contour, true);
        if (perimeter < 10.0) continue;

        double circularity = (4.0 * CV_PI * area) / (perimeter * perimeter);
        if (circularity < MIN_CIRCULARITY) continue;

        cv::Point2f c;
        float r;
        cv::minEnclosingCircle(contour, c, r);

        if (r < MIN_RADIUS || r > MAX_RADIUS) continue;

        if (circularity > bestCircularity) {
            bestCircularity = circularity;
            bestCenter = c;
            bestRadius = r;
        }
    }

    if (bestRadius <= 0.0f) return false;

    center = bestCenter;
    rad = bestRadius;
    return true;
}

#else
#include <fstream>
#include <stdexcept>

BallDetector::BallDetector(const std::string& configPath,
                           const std::string& camName)
{
    std::ifstream f(configPath);
    if (!f.is_open()) throw std::runtime_error("Cannot open config");

    json config;
    f >> config;
    f.close();

    const auto& cam = config.at(camName);

    auto mu_vec = cam["mu"]["data"].get<std::vector<float>>();
    auto S_vec  = cam["Sigma"]["data"].get<std::vector<float>>();

    mu = cv::Vec3f(mu_vec[0], mu_vec[1], mu_vec[2]);

    cv::Matx33f S(
        S_vec[0], S_vec[1], S_vec[2],
        S_vec[3], S_vec[4], S_vec[5],
        S_vec[6], S_vec[7], S_vec[8]
    );

    invS = S.inv();

    threshold = cam.value("threshold", 9.0f);

    bgSub = cv::createBackgroundSubtractorMOG2(500,16,false);
}

inline float BallDetector::mahalanobis(const cv::Vec3f& x) const
{
    cv::Vec3f d = x - mu;

    cv::Vec3f y(
        invS(0,0)*d[0] + invS(0,1)*d[1] + invS(0,2)*d[2],
        invS(1,0)*d[0] + invS(1,1)*d[1] + invS(1,2)*d[2],
        invS(2,0)*d[0] + invS(2,1)*d[1] + invS(2,2)*d[2]
    );

    return y.dot(d);
}

//can parrellellize if need be
bool BallDetector::findBall(
    const cv::Mat& im,
    cv::Point2f& center,
    float& rad,
    bool dobg_masking
) const {
    if(im.empty()) return false;

    cv::Mat frame;
    cv::GaussianBlur(im, frame, {7,7}, 0);

    cv::Mat fgMask;

    if(dobg_masking) bgSub->apply(frame, fgMask);
    else fgMask = cv::Mat(frame.size(), CV_8U, cv::Scalar(255));

    cv::Mat lab;
    cv::cvtColor(frame, lab, cv::COLOR_BGR2Lab);

    cv::Mat mask(frame.size(), CV_8U, cv::Scalar(0));

    for(int y=0;y<lab.rows;y++)
    {
        const cv::Vec3b* row = lab.ptr<cv::Vec3b>(y);
        const uchar* fg = fgMask.ptr<uchar>(y);
        uchar* m = mask.ptr<uchar>(y);

        for(int x=0;x<lab.cols;x++) {
            if(!fg[x]) continue;

            cv::Vec3f p(row[x][0],row[x][1],row[x][2]);
            if(mahalanobis(p) < threshold) m[x] = 255;
        }
    }

    cv::morphologyEx(
        mask,
        mask,
        cv::MORPH_OPEN,
        cv::getStructuringElement(cv::MORPH_ELLIPSE,{3,3})
    );

    std::vector<std::vector<cv::Point>> contours;

    cv::findContours(
        mask,
        contours,
        cv::RETR_EXTERNAL,
        cv::CHAIN_APPROX_SIMPLE
    );

    if (contours.empty()) return false;

    float bestArea = 0;
    int bestIdx = -1;

    for(int i=0;i<contours.size();i++)
    {
        float area = cv::contourArea(contours[i]);

        if (area > bestArea)
        {
            bestArea = area;
            bestIdx = i;
        }
    }

    if (bestIdx < 0) return false;
 
    cv::minEnclosingCircle(contours[bestIdx],center,rad);

    return true;
}

#endif

bool BallDetector::findBall(
        const cv::Mat& im,
        cv::Point2f& center,
        float& rad,
        cv::Rect roi
) {
    bool found = findBall(im(roi), center, rad, false);

    if(found) {
        center.x += roi.x;
        center.y += roi.y;
    }

    return found;
}