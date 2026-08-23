#include "vision/camera/camera.h"

#include <filesystem>
#include <stdexcept>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/videodev2.h>
#include <string>
#include <pthread.h>

#include <cerrno>
#include <cstring>
#include <iomanip>

static void printV4L2Config(const std::string& device, const std::string& name)
{
    int fd = open(device.c_str(), O_RDWR);
    if (fd < 0) {
        std::cerr << name << ": failed to open " << device
                  << ": " << std::strerror(errno) << '\n';
        return;
    }

    v4l2_format fmt{};
    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

    if (ioctl(fd, VIDIOC_G_FMT, &fmt) == 0) {
        const auto& p = fmt.fmt.pix;

        char fourcc[5] = {
            char(p.pixelformat & 0xff),
            char((p.pixelformat >> 8) & 0xff),
            char((p.pixelformat >> 16) & 0xff),
            char((p.pixelformat >> 24) & 0xff),
            '\0'
        };

        std::cout
            << name << " V4L2 format:\n"
            << "  resolution: " << p.width << "x" << p.height << '\n'
            << "  pixel format: " << fourcc << '\n'
            << "  bytes/line: " << p.bytesperline << '\n'
            << "  image size: " << p.sizeimage << '\n';
    } else {
        std::cerr << name << ": VIDIOC_G_FMT failed: "
                  << std::strerror(errno) << '\n';
    }

    v4l2_streamparm parm{};
    parm.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

    if (ioctl(fd, VIDIOC_G_PARM, &parm) == 0) {
        const auto& tpf = parm.parm.capture.timeperframe;

        double fps = 0.0;
        if (tpf.numerator != 0) {
            fps = static_cast<double>(tpf.denominator) /
                  static_cast<double>(tpf.numerator);
        }

        std::cout
            << "  timeperframe: "
            << tpf.numerator << "/" << tpf.denominator
            << " = " << fps << " FPS\n";
    } else {
        std::cerr << name << ": VIDIOC_G_PARM failed: "
                  << std::strerror(errno) << '\n';
    }

    close(fd);
}

Camera::Camera(const std::string& capPath,
               const std::string& caliPath)
               : caliPath(caliPath), capPath(capPath), capName(std::filesystem::path(capPath).filename()),
    frame_buffer(frameBuffer_len)
{
    cap.open(capPath, cv::CAP_V4L2); if (!cap.isOpened()) throw std::runtime_error("Failed to open camera: " + capPath);
    std::cout << capName
          << " backend: " << cap.getBackendName()
          << '\n';

    cv::FileStorage fs(caliPath, cv::FileStorage::READ);
    if (!fs.isOpened()) throw std::runtime_error("failed to open cali file: " + caliPath);
    fs["camera_matrix"] >> K;
    fs["dist_coeffs"] >> D;
    fs["rot_mat"] >> R;
    fs["trans_mat"] >> t;


    std::string distmodel = "standard";
    if (!fs["distortion_model"].empty()) fs["distortion_model"] >> distmodel; isFisheye = (distmodel == "fisheye");

    fs.release();

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
    //cap.set(cv::CAP_PROP_BUFFERSIZE, 1); faster for some reason w.o this

    std::cout << capName
          << " width  = " << cap.get(cv::CAP_PROP_FRAME_WIDTH)
          << " height = " << cap.get(cv::CAP_PROP_FRAME_HEIGHT)
          << " fps    = " << cap.get(cv::CAP_PROP_FPS)
          << " fourcc = " << cap.get(cv::CAP_PROP_FOURCC)
          << '\n';

    //printV4L2Config(capPath, capName);

    const std::string command = "v4l2-ctl -d ";
    std::system((command+capPath+" -c auto_exposure=1").c_str());
    std::system((command+capPath+" -c exposure_time_absolute="+ std::to_string(exposureSetting)).c_str());

    frame_buffer.init(frameWidth, frameHeight, CV_8UC3);
}

void Camera::beginLoop() {
    if (running.load()) return;
    if (!cap.isOpened()) std::cout << capName << " isnt opened on loop start. trying...";

    running.store(true);

    capture_thread = std::thread([this]() {
        pthread_setname_np(pthread_self(), capName.c_str());

        cv::Mat frame; frame.reserve(1);

        uint64_t frames = 0;
        auto fps_start = std::chrono::steady_clock::now();
        while (running.load(std::memory_order_relaxed)) {
            
            //tso = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();
            //std::cout << capName << " start\n";
            if (!grab()) continue;
            if (!retrieve(frame) || frame.empty()) continue;

            {
                std::lock_guard<std::mutex> lock(frame_buffer_mutex);

                uint64_t ts = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();
                frame_buffer.push_back(frame, ts); //handles all the stuff
            }

            ++frames;
            auto now = std::chrono::steady_clock::now();
            auto elapsed = std::chrono::duration<double>(now - fps_start).count();
            if (elapsed >= 0.5) { capture_fps.store(frames/elapsed, std::memory_order_relaxed);
                frames = 0; fps_start = now;
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