#include "vision/camera/camera.h"
#include "vision/ballDet/ballDet.h"
#include "vision/stereo/multiStereo.h"
#include "config/config.h"
#include <atomic>
#include "utils.h"

std::atomic<bool> mainLooping(true);
void signal_handler(int signal) { if (signal == SIGINT) mainLooping = false; }

// HSV trackbar values — defaults tuned for orange
int lowH = 5,  highH = 25;
int lowS = 100, highS = 255;
int lowV = 100,  highV = 255;

// MOG2 params — tune these directly
constexpr int    MOG2_HISTORY    = 100;
constexpr double MOG2_THRESHOLD  = 16.0;
constexpr double MOG2_LEARN_RATE = 0.05;

// ball detection params — tune these directly
constexpr float MIN_BALL_RADIUS = 5.0f;
constexpr float MAX_BALL_RADIUS = 40.0f;

// one MOG2 subtractor per camera
cv::Ptr<cv::BackgroundSubtractorMOG2> mog_BL, mog_BR, mog_MR;
cv::Mat applyMasks(const cv::Mat& frame,
                   cv::Ptr<cv::BackgroundSubtractorMOG2>& mog)
{
    if (frame.empty()) return frame;

    // --- persistent buffers (avoid realloc each frame) ---
    static thread_local cv::Mat blurred, hsv, hsvMask, fgMask, combined;
    static thread_local cv::Mat kernel =
        cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3));

    // --- output (annotated image) ---
    cv::Mat output;
    frame.copyTo(output);

    // --- blur ---
    cv::GaussianBlur(frame, blurred, cv::Size(5, 5), 0);

    // --- foreground mask ---
    mog->apply(blurred, fgMask, MOG2_LEARN_RATE);
    cv::morphologyEx(fgMask, fgMask, cv::MORPH_CLOSE, kernel, {-1,-1}, 2);

    // --- HSV mask ---
    cv::cvtColor(blurred, hsv, cv::COLOR_BGR2HSV);
    cv::inRange(hsv,
        cv::Scalar(lowH, lowS, lowV),
        cv::Scalar(highH, highS, highV),
        hsvMask);

    // --- combine masks ---
   combined =  hsvMask;//cv::bitwise_and(hsvMask, fgMask, combined);

    // cleanup noise
    cv::morphologyEx(combined, combined, cv::MORPH_OPEN, kernel);

    // --- find contours ---
    std::vector<std::vector<cv::Point>> contours;
    contours.reserve(32);
    cv::findContours(combined, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    // --- best candidate ---
    cv::Point2f bestCenter(-1, -1);
    float bestRadius = 0.0f;
    double bestCircularity = 0.0;

    for (const auto& contour : contours) {
        double area = cv::contourArea(contour);
        if (area < 50.0) continue;

        cv::Rect box = cv::boundingRect(contour);
        if (box.width < MIN_BALL_RADIUS*2 || box.height < MIN_BALL_RADIUS*2)
            continue;

        double perimeter = cv::arcLength(contour, true);
        if (perimeter < 10.0) continue;

        double circularity = (4.0 * CV_PI * area) / (perimeter * perimeter);
        if (circularity < 0.2) continue;

        cv::Point2f center;
        float radius;
        cv::minEnclosingCircle(contour, center, radius);

        if (radius < MIN_BALL_RADIUS || radius > MAX_BALL_RADIUS)
            continue;

        if (circularity > bestCircularity) {
            bestCircularity = circularity;
            bestCenter = center;
            bestRadius = radius;
        }
    }

    // --- draw annotation directly on output ---
    if (bestRadius > 0.0f) {
        cv::circle(output, bestCenter, (int)bestRadius,
                   cv::Scalar(0, 255, 0), 2);

        cv::circle(output, bestCenter, 3,
                   cv::Scalar(0, 0, 255), -1);

        cv::putText(output,
                    "r=" + std::to_string((int)bestRadius),
                    cv::Point(bestCenter.x + bestRadius + 4, bestCenter.y),
                    cv::FONT_HERSHEY_SIMPLEX, 0.45,
                    cv::Scalar(0, 255, 0), 1);
    }

    return output;
}
#define DO_ON_CAMS for (auto*cam:{&cam_BL,&cam_BR,&cam_MR})cam->

int main() {
    const std::string saveedddir = "/home/connor/PingPongRobot/tests/test_data/";
    CameraRec cam_BL(saveedddir + "cam_BL_rev.avi", saveedddir + "cam_BL_ts.txt", CONF_PATH + "vision/cam_BL-intrinsics.yml");
    CameraRec cam_BR(saveedddir + "cam_BR_rev.avi", saveedddir + "cam_BR_ts.txt", CONF_PATH + "vision/cam_BR-intrinsics.yml");
    CameraRec cam_MR(saveedddir + "cam_MR_rev.avi", saveedddir + "cam_MR_ts.txt", CONF_PATH + "vision/cam_MR-intrinsics.yml");

    synchCamrecsToNow(cam_BL, cam_BR, cam_MR);
    DO_ON_CAMS setSpeed(0.5);

    /*Camera cam_BL("/dev/cam_BL", CONF_PATH + "vision/cam_BL-intrinsics.yml", 1280, 800, 120, 35);
    Camera cam_BR("/dev/cam_BR", CONF_PATH + "vision/cam_BR-intrinsics.yml", 1280, 800, 120, 35);
    Camera cam_MR("/dev/cam_MR", CONF_PATH + "vision/cam_MR-intrinsics.yml", 1920, 1080, 120, 300);
*/
    DO_ON_CAMS beginLoop();

    // init one MOG2 per camera (shadow detection off — faster, cleaner)
    mog_BL = cv::createBackgroundSubtractorMOG2(MOG2_HISTORY, MOG2_THRESHOLD, false);
    mog_BR = cv::createBackgroundSubtractorMOG2(MOG2_HISTORY, MOG2_THRESHOLD, false);
    mog_MR = cv::createBackgroundSubtractorMOG2(MOG2_HISTORY, MOG2_THRESHOLD, false);

    // HSV sliders only
    cv::namedWindow("HSV Controls", cv::WINDOW_NORMAL);
    cv::createTrackbar("Low  H", "HSV Controls", &lowH,  179);
    cv::createTrackbar("High H", "HSV Controls", &highH, 179);
    cv::createTrackbar("Low  S", "HSV Controls", &lowS,  255);
    cv::createTrackbar("High S", "HSV Controls", &highS, 255);
    cv::createTrackbar("Low  V", "HSV Controls", &lowV,  255);
    cv::createTrackbar("High V", "HSV Controls", &highV, 255);

    std::signal(SIGINT, signal_handler);
    while (mainLooping.load(std::memory_order_relaxed)) {
        cv::Mat frame, display;

        frame = cam_BL.frame_buffer[0].frame;
        if (!frame.empty()) cv::imshow("cambl_mask", applyMasks(frame, mog_BL));

        frame = cam_BR.frame_buffer[0].frame;
        if (!frame.empty()) cv::imshow("cambr_mask", applyMasks(frame, mog_BR));

        frame = cam_MR.frame_buffer[0].frame;
        if (!frame.empty()) cv::imshow("cammr_mask", applyMasks(frame, mog_MR));

         if (cv::waitKey(1)==27) break;
    }

    std::cout << "exited\n";
    cam_BL.release();
    cam_BR.release();
    cam_MR.release();
    std::cout << "all done! :)\n";
    return 0;
}