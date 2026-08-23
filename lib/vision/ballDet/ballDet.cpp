#include "vision/ballDet/ballDet.h"
#include <fstream>
#include <nlohmann/json.hpp>
#include "misc/timeLog/timeLog.hpp"
#include "misc/gaussianBlob.h"
#include <pthread.h>

//yeah theres a lot of slop here dw about it. gdmit
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


static bool projectStateToRoi(const Camera& cam,
                              const GaussBlob<6>& x,
                              int imW, int imH,
                              double sigma,
                              cv::Rect& out)
{
    const Eigen::Vector3d mu  = x.mu.head<3>();
    const Eigen::Matrix3d cov = x.cov.topLeftCorner<3,3>();

    cv::Mat Rcv, tcv, K, D;
    cam.R.convertTo(Rcv, CV_64F);
    cam.t.convertTo(tcv, CV_64F);
    cam.K.convertTo(K,   CV_64F);
    cam.D.convertTo(D,   CV_64F);

    Eigen::Matrix3d R; Eigen::Vector3d t;
    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) R(r, c) = Rcv.at<double>(r, c);
        t(r) = tcv.at<double>(r);
    }

    const Eigen::Vector3d p = R * mu + t;      // camera frame

    // Center: OpenCV projection, exact under distortion / fisheye.
    cv::Mat rvec; cv::Rodrigues(Rcv, rvec);
    std::vector<cv::Point3f> obj{
        cv::Point3f((float)mu.x(), (float)mu.y(), (float)mu.z()) };
    std::vector<cv::Point2f> img;

    if (cam.isFisheye) cv::fisheye::projectPoints(obj, img, rvec, tcv, K, D);
    else               cv::         projectPoints(obj, rvec, tcv, K, D, img);

    // Size: pinhole linearization of the covariance.
    const double fx = K.at<double>(0,0), fy = K.at<double>(1,1);
    const double z = p.z(), z2 = z * z;

    Eigen::Matrix<double,2,3> J;
    J << fx/z, 0.0,  -fx * p.x() / z2,
         0.0,  fy/z, -fy * p.y() / z2;

    const Eigen::Matrix2d Spx = J * (R * cov * R.transpose()) * J.transpose();

    double hw = sigma * std::sqrt(std::max(Spx(0,0), 0.0));
    double hh = sigma * std::sqrt(std::max(Spx(1,1), 0.0));

    hw = std::clamp(hw, 24.0, imW * 0.5);
    hh = std::clamp(hh, 24.0, imH * 0.5);

    cv::Rect r(cvRound(img[0].x - hw), cvRound(img[0].y - hh),
               cvRound(2*hw),          cvRound(2*hh));
    r &= cv::Rect(0, 0, imW, imH);

    //if (r.width < 8 || r.height < 8) return false;
    //if (r.area() > 0.4 * imW * imH)  return false;   // crop wouldn't help

    return (out = r), true;
}

void BallDetector::loop()
{
    pthread_setname_np(pthread_self(), (cam->capName+"Det").c_str());

    uint64_t last_ts   = 0;
    bool     prevFound = false;

    while (running) {

        Frame f;
        bool haveFrame = false;

        {   std::lock_guard lk(cam->frame_buffer_mutex);

            if (!cam->frame_buffer.empty() && cam->frame_buffer.back().timestamp_us > last_ts)
            {
                f = cam->frame_buffer.back();
                haveFrame = true;
            }
        }

        if (!haveFrame) {std::this_thread::sleep_for(std::chrono::milliseconds(1)); continue;} //spin till have frame

        BallDetection d;
        bool found = false;

        //auto im = f.image.clone(); //DEBUG!!!

        if (prevFound) { //if the prev was found use roi, else use the reg
            cv::Rect roi;
            if (projectStateToRoi(*cam, kf.snapshot(),
                                  f.image.cols, f.image.rows,
                                  3, roi)) 
                found = findBall(f.image, d.center, d.radius, roi, false);

                //cv::rectangle(im, roi, (found ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 0, 255)), 5);//DEBUG!!!
        }

        if (!found) {
            {//use most recent frame
                std::lock_guard lk(cam->frame_buffer_mutex);
                if (!cam->frame_buffer.empty() && cam->frame_buffer.back().timestamp_us > f.timestamp_us) f = cam->frame_buffer.back();
            } found = findBall(f.image, d.center, d.radius, true);
        }

        //if (found) cv::circle(im, d.center, 3, cv::Scalar(0, 255, 0), -1);
        //cv::imshow(cam->capName + "bdet", im);//DEBUG!!!
        //cv::waitKey(1);

        last_ts   = f.timestamp_us;
        prevFound = found;

        d.found        = found;
        d.timestamp_us = f.timestamp_us;

        { //push det safely
            std::lock_guard lk(det_mtx);
            if (det_buf.full()) det_buf.pop_front();
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

    if (im.empty()) return false;

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