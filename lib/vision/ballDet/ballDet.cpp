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

BallDetector::BallDetector(const std::string& configPath, const std::string& camName) {
    std::ifstream f(configPath);
    if (!f.is_open()) throw std::runtime_error("Failed to open config file: " + configPath);

    json config;
    f >> config;
    f.close();

    if (!config.contains(camName)) throw std::runtime_error("Camera not found in config: " + camName);

    const auto& cam = config.at(camName);
    mu    = jsonToMat(cam["mu"]);
    Sigma = jsonToMat(cam["Sigma"]);
    sensorNoise = jsonToMat(cam["det_noise"]);

    if (mu.empty() || Sigma.empty()) throw std::runtime_error("Invalid model contents");

    mu.convertTo(mu, CV_32F);
    Sigma.convertTo(Sigma, CV_32F);
    cv::invert(Sigma, invS, cv::DECOMP_SVD);

    cv::Mat chol;
    cv::Cholesky((float*)Sigma.ptr<float>(), Sigma.step, Sigma.rows, nullptr, 0, 0);
    cv::invert(Sigma, invS, cv::DECOMP_CHOLESKY);

    cv::Mat eigvals, eigvecs;
    cv::eigen(invS, eigvals, eigvecs);
    L = eigvecs.t() * cv::Mat::diag(eigvals.mul(eigvals));

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
    if (im.empty()) return false;

    cv::Mat frame;
    cv::GaussianBlur(im, frame, cv::Size(7,7), 0);

    cv::Mat fgMask; bgSub->apply(frame, fgMask);

    cv::morphologyEx(
        fgMask, fgMask, cv::MORPH_OPEN,
        cv::getStructuringElement(cv::MORPH_ELLIPSE, {3,3})
    );

    cv::Mat lab; cv::cvtColor(frame, lab, cv::COLOR_BGR2Lab);

    std::vector<cv::Mat> ch; cv::split(lab, ch);

    cv::Mat a, b;
    ch[1].convertTo(a, CV_32F);
    ch[2].convertTo(b, CV_32F);

    cv::Mat ab;
    cv::hconcat(
        a.reshape(1, a.total()),
        b.reshape(1, b.total()),
        ab
    );

    cv::Mat mu_rep; cv::repeat(mu, ab.rows, 1, mu_rep);

    cv::Mat d; cv::subtract(ab, mu_rep, d);

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