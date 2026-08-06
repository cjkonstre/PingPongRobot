#include "vision/ballDet/ballDet.h"
#include <fstream>
#include <nlohmann/json.hpp>
#include "misc/timeLog/timeLog.hpp"

using json = nlohmann::json;

static cv::Mat jsonToMat(const json& j)
{
    int r = j.at("rows");
    int c = j.at("cols");

    auto d = j.at("data").get<std::vector<float>>();

    cv::Mat m(r,c,CV_32F);
    std::memcpy(m.ptr<float>(), d.data(), r*c*sizeof(float));

    return m;
}

BallDetector::BallDetector(const std::string& configPath,
                           const std::string& camName,
                           const KalmanFilter& kal): kf(kal)
{
    std::ifstream f(configPath); json j; f >> j;

    auto cam = j.at(camName); //intrinsic info? det info?

    mu = jsonToMat(cam["mu"]); mu.convertTo(mu, CV_32F);
    Sigma = jsonToMat(cam["Sigma"]); Sigma.convertTo(Sigma, CV_32F);
    cv::invert(Sigma, invS, cv::DECOMP_SVD);

    kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE,{3,3});

    bgSub = cv::createBackgroundSubtractorMOG2(100,16,false);
}

void BallDetector::beginLoop(Camera& c)
{
    cam = &c;
    running = true;

    worker = std::thread([this]{ loop(); });
}

void BallDetector::endLoop()
{
    running = false;
    if(worker.joinable()) worker.join();
}

bool BallDetector::latestDetection(BallDetection& out) const
{
    std::lock_guard lk(det_mtx);
    if(det_buf.empty()) return false;
    out = det_buf.back();
    return true;
}

void BallDetector::loop()
{
    uint64_t last_ts = 0;

    while(running) {
        Frame f;

        {
            std::lock_guard lk(cam->frame_buffer_mutex);

            if(cam->frame_buffer.empty()) {
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
                continue;
            }

            f = cam->frame_buffer.back();
        }

        if(f.timestamp_us == last_ts) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            continue;
        }

        last_ts = f.timestamp_us;

        BallDetection d;
        d.timestamp_us = f.timestamp_us;
        
        d.found = findBall(f.frame, d.center, d.radius, true);

        {
            std::lock_guard lk(det_mtx);

            if(det_buf.full()) det_buf.pop_front();

            det_buf.push_back(d);
            lastDet = d;
        }
    }
}
bool BallDetector::findBall(const cv::Mat& im,
                            cv::Point2f& center,
                            float& rad,
                            cv::Rect roi,
                            bool bg)
{

    if (im.empty())
        return false;

    // Clip ROI to image bounds
    roi &= cv::Rect(0, 0, im.cols, im.rows);
    if (roi.width <= 0 || roi.height <= 0)
        return false;

    cv::Mat imROI = im(roi);

    cv::GaussianBlur(imROI, blurred, {5,5}, 0);

    bool useBg = bg && bgSub;

    if (useBg)
    {
        bgSub->apply(blurred, fgMask);
        cv::morphologyEx(fgMask, fgMask, cv::MORPH_CLOSE, kernel);
    }

    cv::cvtColor(blurred, hsv, cv::COLOR_BGR2HSV);

    cv::inRange(hsv,
        cv::Scalar(LOW_H, LOW_S, LOW_V),
        cv::Scalar(HIGH_H, HIGH_S, HIGH_V),
        hsvMask);

    if (useBg)
        cv::bitwise_and(hsvMask, fgMask, combined);
    else
        combined = hsvMask;

    cv::morphologyEx(combined, combined, cv::MORPH_OPEN, kernel);

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(combined, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    if (contours.empty())
        return false;

    double bestScore = 0.0;
    cv::Point2f bestC;
    float bestR = 0.f;

    for (auto& c : contours)
    {
        double area = cv::contourArea(c);
        if (area < MIN_AREA) continue;

        double per = cv::arcLength(c, true);
        if (per < 10) continue;

        double circ = (4 * CV_PI * area) / (per * per);
        if (circ < MIN_CIRC) continue;

        cv::Point2f p;
        float r;
        cv::minEnclosingCircle(c, p, r);

        if (r < MIN_R || r > MAX_R) continue;

        double score = area * circ;

        if (score > bestScore)
        {
            bestScore = score;
            bestC = p;
            bestR = r;
        }
    }

    if (bestScore <= 0.0)
        return false;

    // Convert ROI coordinates back to full-image coordinates
    center.x = bestC.x + roi.x;
    center.y = bestC.y + roi.y;
    rad = bestR;

    return true;
}

bool BallDetector::findBall(const cv::Mat& im,
                            cv::Point2f& center,
                            float& rad,
                            bool bg)
{
    return findBall(im, center, rad, cv::Rect(0, 0, im.cols, im.rows), bg);
}