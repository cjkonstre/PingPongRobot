#pragma once

#include <opencv2/opencv.hpp>
#include <string>
#include <boost/circular_buffer.hpp>
#include <thread>
#include <atomic>
#include <chrono>
#include <mutex>


//used for the asynch looping. allows a synchronizing thread to align frames temporally
struct Frame {
    cv::Mat frame;
    uint64_t timestamp_us;
};

class Camera {
private:
    std::string capPath;
    std::string intrinsicsPath;

    cv::VideoCapture cap;

    //for asynch looping
    std::thread capture_thread;
    std::atomic<bool> running{false};

    cv::VideoWriter writer;
    std::ofstream tsldr;
    std::atomic<bool> recording{false};

    mutable std::mutex frame_buffer_mutex; //fancy mutable keyword wow
public:
    struct Intrinsics {
        cv::Mat K;   // camera atrix
        cv::Mat D;   // distotion coefficients
    };

    cv::Mat proj; //has dedicated initializer
    void makeProjection(const cv::Mat& Kcv,
                                        const cv::Mat& Rcv,
                                        const cv::Mat& Tcv);

    Intrinsics intrinsics;

    std::string capName;

    Camera(const std::string& capPath,
           const std::string& intrinsicsPath,
           int frameWidth,
           int frameHeight,
           int fps,
           int exposureSetting);

    inline bool grab() {return cap.grab();}
    inline bool retrieve(cv::Mat& frame) {return cap.retrieve(frame);}
    inline bool read(cv::Mat& frame) {return cap.read(frame);}
    inline void release() {endLoop(); cap.release();}

    static constexpr size_t frameBuffer_len = 5;
    boost::circular_buffer<Frame> frame_buffer{frameBuffer_len};

    void beginLoop();
    void endLoop();

    void beginRecordingLoop(const std::string& savedir);
    void endRecordingLoop();
};