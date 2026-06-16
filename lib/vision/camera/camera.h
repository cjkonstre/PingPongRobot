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
protected:
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

    //no external settings to the rec device
    Camera(const std::string& capPath, const std::string& intrinsicsPath); 

    //sets controls on rec device
    Camera(const std::string& capPath,
           const std::string& intrinsicsPath,
           int frameWidth,
           int frameHeight,
           int fps,
           int exposureSetting);

    inline bool grab() {return cap.grab();}
    virtual inline bool retrieve(cv::Mat& frame) {return cap.retrieve(frame);}
    virtual inline bool read(cv::Mat& frame) {return cap.read(frame);}
    virtual inline void release() {endLoop(); cap.release();}

    static constexpr size_t frameBuffer_len = 5;
    boost::circular_buffer<Frame> frame_buffer{frameBuffer_len};

    void beginLoop();
    void endLoop();

    void beginRecordingLoop(const std::string& savedir);
    void endRecordingLoop();
};

//camera from rec. reads/simulates from a video rec. new constructor and oveloads the reading methods
//pretends to be a camera, is 
class CameraRec : public Camera {
private:
    static void blockingWait(uint64_t us);
    void blockingWaitNext();

    uint64_t prev_ts = 0;
    bool started = false;
    bool first_consumed = false;

    double speed_modulation = 1; // this dovodes the simulated time it waits
public: 
    uint64_t first_ts = 0;
    uint64_t offset = 0;
    std::ifstream tsReader; // ikik this isnt areospace code jeez
    std::chrono::steady_clock::time_point start_time;
    //^^IK this rly shouldnt be all exposed im SORRY

    CameraRec(const std::string& recpath, 
              const std::string& tspath,
              const std::string& intrinsicsPath);

    bool retrieve(cv::Mat& frame) override;
    bool read(cv::Mat& frame) override;
    void release() override;

    //2x speed, 0.25x speed ...
    inline void setSpeed(double speed) {speed_modulation = 1/speed;}
};