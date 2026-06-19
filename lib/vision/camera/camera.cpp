#include "vision/camera/camera.h"

#include <filesystem>
#include <stdexcept>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/videodev2.h>
#include <string>


Camera::Camera(const std::string& capPath,
               const std::string& caliPath)
               : caliPath(caliPath), capPath(capPath), capName(std::filesystem::path(capPath).filename()),
    frame_buffer(frameBuffer_len)
{
    cap.open(capPath); if (!cap.isOpened()) throw std::runtime_error("Failed to open camera: " + capPath);

    cv::FileStorage fs(caliPath, cv::FileStorage::READ);
    if (!fs.isOpened()) throw std::runtime_error("failed to open cali file: " + caliPath);
    fs["camera_matrix"] >> K;
    fs["dist_coeffs"] >> D;
    fs["rot_mat"] >> R;
    fs["trans_mat"] >> t;
    fs.release();

    frame_buffer.assign(frameBuffer_len, Frame{cv::Mat(), 0});
}

Camera::Camera(const std::string& capPath,
               const std::string& caliPath,
               int frameWidth,
               int frameHeight,
               int fps,
               int exposureSetting)
               : Camera(capPath, caliPath)
{
    cap.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M','J','P','G'));
    cap.set(cv::CAP_PROP_FRAME_WIDTH, frameWidth);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, frameHeight);
    cap.set(cv::CAP_PROP_FPS, fps);
    cap.set(cv::CAP_PROP_BUFFERSIZE, 1);

    const std::string command = "v4l2-ctl -d ";
    std::system((command+capPath+" -c auto_exposure=1").c_str());
    std::system((command+capPath+" -c exposure_time_absolute="+ std::to_string(exposureSetting)).c_str());
}

void Camera::beginLoop() {
    if (running.load()) return;
    if (!cap.isOpened()) std::cout << capName << "isnt opened on loop start. trying...";

    running.store(true);

    capture_thread = std::thread([this]() {
        cv::Mat frame; frame.reserve(1);

        while (running.load(std::memory_order_relaxed)) {
            if (!cap.isOpened()) continue;
            if (!grab()) continue; 
            if (!retrieve(frame) || frame.empty()) continue;

            uint64_t ts = std::chrono::duration_cast<std::chrono::microseconds>(
                std::chrono::steady_clock::now().time_since_epoch()
            ).count();

            { //scope block, so mutex gets destructed
            std::lock_guard<std::mutex> lock(frame_buffer_mutex);
            if (frame_buffer.full()) frame_buffer.pop_front(); //so it overwrites
            frame_buffer.push_back(Frame{frame.clone(), ts}); // clone is important , cv reuses internal buffers
            }
        }
    });
}

void Camera::endLoop(){
    if (!running.load()) return;
    running.store(false);
    if (capture_thread.joinable()) capture_thread.join();

    endRecordingLoop();

}

void Camera::beginRecordingLoop(const std::string& savedir) { //a bit slow, not sure why
    if (running.load()) return;

    if (!cap.isOpened())std::cout << capName << " isnt opened on loop start. trying...\n";

    const std::string video_path = savedir + "/" + capName + "_rev.avi";
    const std::string ts_path    = savedir + "/" + capName + "_ts.txt";

    double fps = cap.get(cv::CAP_PROP_FPS);
    if (fps <= 0 || std::isnan(fps)) fps = 120;  // fallback

    tsldr.open(ts_path);
    if (!tsldr.is_open())
        throw std::runtime_error("Failed to open timestamp writer " + ts_path);

    recording.store(true);
    running.store(true);

    capture_thread = std::thread([this, video_path, fps]() {
        cv::Mat frame;

        bool writer_initialized = false;

        while (running.load(std::memory_order_relaxed)) {
            if (!cap.isOpened()) continue;
            if (!grab()) continue;
            if (!retrieve(frame) || frame.empty()) continue;

            uint64_t ts = std::chrono::duration_cast<std::chrono::microseconds>(
                std::chrono::steady_clock::now().time_since_epoch()
            ).count();

            //lazy eval god knows why
            if (!writer_initialized) {
                writer.open(
                    video_path,
                    cv::VideoWriter::fourcc('M','J','P','G'),
                    fps,
                    cv::Size(frame.cols, frame.rows),
                    frame.channels() == 3
                );

                if (!writer.isOpened()) throw std::runtime_error("Failed to open VideoWriter: " + video_path);

                writer_initialized = true;
            }

            writer.write(frame);
            tsldr << ts << '\n';
        }
    });
}

void Camera::endRecordingLoop() {
    if (!recording.load()) return;
    if (!running.load()) return;
    running.store(false);
    if (capture_thread.joinable()) capture_thread.join();

    writer.release(); 
    tsldr.close();

    recording.store(false);

    std::cout << capName << " recording stopped\n";
}


//also just opens the timestamp reader
CameraRec::CameraRec(const std::string& capPath, const std::string& tspath, const std::string& caliPath) :
    Camera(capPath, caliPath) //constructor that doesnt force any reading settings
{ tsReader.open(tspath); if (!tsReader) throw std::runtime_error("Failed to open timestamps: " + tspath); }

void CameraRec::release() {
    tsReader.close();
    Camera::release();
}

void CameraRec::blockingWait(uint64_t us) { //accurate to 200us. some hardcoded values in there but its fine
    auto t_end = std::chrono::steady_clock::now() + std::chrono::microseconds(us);
    std::this_thread::sleep_until(t_end - std::chrono::microseconds(2000)); //does innacurate sleep until a bit before
    while (std::chrono::steady_clock::now() < t_end) {std::this_thread::sleep_for(std::chrono::microseconds(200));} //throttled
}
void CameraRec::blockingWaitNext() {
    uint64_t ts;

    // first frame uses preloaded timestamp
    if (!started) { ts = first_ts; started = true;} 
    else tsReader >> ts;

    uint64_t rel = (ts - first_ts)*speed_modulation;

    auto target = start_time + std::chrono::microseconds(offset + rel);
    std::this_thread::sleep_until(target);
}

//grab is unchanged. if works as it should
bool CameraRec::retrieve(cv::Mat& frame) {blockingWaitNext(); return Camera::retrieve(frame);}
bool CameraRec::read(cv::Mat& frame) {blockingWaitNext(); return Camera::read(frame);}