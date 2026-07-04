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

//sensor class. holds information about itself
//also handles information input
class Camera {
    friend class BallDetector;
protected:
    std::string capPath;
    std::string caliPath;

    cv::VideoCapture cap;


    //for asynch looping
    std::thread capture_thread;
    std::atomic<bool> running{false};

    cv::VideoWriter writer;
    std::ofstream tsldr;
    std::atomic<bool> recording{false};

    mutable std::mutex frame_buffer_mutex; //fancy mutable keyword wow
public:
    std::string capName;

    //sensor info
    cv::Mat K;   // camera matrix 
    cv::Mat D;   // distortion coefficients

    cv::Mat R; //rot mat extrinsics
    cv::Mat t; //translation mat extr
    bool isFisheye = false;

    //instantiations

    Camera() = default;
    //no external settings to the rec device
    Camera(const std::string& capPath, const std::string& caliPath); 

    //sets controls on rec device
    Camera(const std::string& capPath,
           const std::string& caliPath,
           int frameWidth,
           int frameHeight,
           int fps,
           int exposureSetting);

    
    //data grabbing
    
    inline bool grab() {return cap.grab();}
    virtual inline bool retrieve(cv::Mat& frame) {return cap.retrieve(frame);}
    virtual inline bool read(cv::Mat& frame) {return cap.read(frame);}
    virtual inline void release() {endLoop(); cap.release();}

    static constexpr size_t frameBuffer_len = 5;
    boost::circular_buffer<Frame> frame_buffer{frameBuffer_len};

    void beginLoop();
    void endLoop();

    //recs to an external file. can be read from the camerarec class
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
              const std::string& caliPath);

    bool retrieve(cv::Mat& frame) override;
    bool read(cv::Mat& frame) override;
    void release() override;

    //2x speed, 0.25x speed ...
    inline void setSpeed(double speed) {speed_modulation = 1/speed;}
};