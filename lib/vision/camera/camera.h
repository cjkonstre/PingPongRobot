#pragma once

#include <opencv2/opencv.hpp>
#include <string>
#include <thread>
#include <atomic>
#include <chrono>
#include <mutex>


//used for the asynch looping. allows a synchronizing thread to align frames temporally
struct Frame {
    cv::Mat image;
    uint64_t timestamp_us;
};

class FrameBuffer {
public:
    explicit FrameBuffer(size_t capacity)
        : buffer(capacity) {}

    void init(int width, int height, int type = CV_8UC3) {for (auto& frame : buffer) frame.image.create(height, width, type);}

    void push_back(const cv::Mat& image, uint64_t timestamp_us) {
        if (buffer.empty()) return; //??

        size_t index;

        if (count < buffer.size()) {
            // Append after the newest frame.
            index = (start + count) % buffer.size();
            ++count;
        } else {
            // Full: overwrite the oldest frame.
            index = start;
            start = (start + 1) % buffer.size();
        }

        image.copyTo(buffer[index].image);
        buffer[index].timestamp_us = timestamp_us;
    }

    Frame& front() {return buffer[start];}
    const Frame& front() const {return buffer[start];}
    Frame& back() {return buffer[(start + count - 1) % buffer.size()];}
    const Frame& back() const {return buffer[(start + count - 1) % buffer.size()];}
    Frame& operator[](size_t index) {return buffer[(start + index) % buffer.size()];}
    const Frame& operator[](size_t index) const {return buffer[(start + index) % buffer.size()];}

    size_t size() const {return count;}

    size_t capacity() const {return buffer.size();}

    bool empty() const {return count == 0;}

    bool full() const {return count == buffer.size();}

    void pop_front() {
        if (count == 0) return;

        start = (start + 1) % buffer.size();
        --count;
    }

private:
    std::vector<Frame> buffer;
    size_t start = 0;
    size_t count = 0;
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

    std::atomic<double> capture_fps{0.0};
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

    double getFPS() const {return capture_fps.load(std::memory_order_relaxed);}

    static constexpr size_t frameBuffer_len = 5;
    FrameBuffer frame_buffer{frameBuffer_len};

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