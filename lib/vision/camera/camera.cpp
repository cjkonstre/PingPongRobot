#include "vision/camera/camera.h"
#include <filesystem>
#include <stdexcept>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/videodev2.h>
#include <Eigen/Dense>
#include <opencv2/core/eigen.hpp>


void Camera::makeProjection(
    const cv::Mat& Kcv,
    const cv::Mat& Rcv,
    const cv::Mat& Tcv)
{
    Eigen::Matrix3d K, R;
    Eigen::Vector3d t;

    cv::cv2eigen(Kcv, K);
    cv::cv2eigen(Rcv, R);
    cv::cv2eigen(Tcv, t);

    Eigen::Matrix<double,3,4> Rt;
    Rt.block<3,3>(0,0) = R;
    Rt.col(3) = t;

    Eigen::Matrix<double,3,4> P = K * Rt;
    cv::eigen2cv(P, proj);
}

static void setControl(int fd, int id, int value)
{
    v4l2_control ctrl{};
    ctrl.id = id;
    ctrl.value = value;

    if (ioctl(fd, VIDIOC_S_CTRL, &ctrl) < 0) perror("VIDIOC_S_CTRL");
}

Camera::Camera(const std::string& capPath,
               const std::string& intrinsicsPath,
               int frameWidth,
               int frameHeight,
               int fps,
               int exposureSetting)
    : capPath(capPath), capName(std::filesystem::path(capPath).filename()),
    frame_buffer(frameBuffer_len)
{
    cap.open(capPath);

    if (!cap.isOpened())
        throw std::runtime_error("Failed to open camera: " + capPath);

    cap.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M','J','P','G'));
    cap.set(cv::CAP_PROP_FRAME_WIDTH, frameWidth);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, frameHeight);
    cap.set(cv::CAP_PROP_FPS, fps);
    cap.set(cv::CAP_PROP_BUFFERSIZE, 1);
    int fd = open(capPath.c_str(), O_RDWR);
    if (fd >= 0) {
        setControl(fd, V4L2_CID_EXPOSURE_AUTO, V4L2_EXPOSURE_MANUAL);
        setControl(fd, V4L2_CID_EXPOSURE_ABSOLUTE, exposureSetting);
        close(fd);
    }

    cv::FileStorage fs(intrinsicsPath, cv::FileStorage::READ);
    if (!fs.isOpened()) throw std::runtime_error("Failed to open intrinsics file: " + intrinsicsPath);
    fs["camera_matrix"] >> intrinsics.K;
    fs["dist_coeffs"] >> intrinsics.D;
    fs.release();

    frame_buffer.assign(frameBuffer_len, Frame{cv::Mat(), 0});
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

            std::cout << capName << " got photo @ " << ts << ' ';

            { //scope block, so mutex gets destructed
            std::lock_guard<std::mutex> lock(frame_buffer_mutex);
            if (frame_buffer.full()) frame_buffer.pop_front(); //so it overwrites
            frame_buffer.push_back(Frame{frame.clone(), ts}); // clone is important , cv reuses internal buffers
            }
        }
    });
}

void Camera::beginRecordingLoop(const std::string& savedir) {
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
    if (!running.load()) return;
    running.store(false);
    if (capture_thread.joinable())
        capture_thread.join();

    writer.release(); 
    tsldr.close();

    recording.store(false);

    std::cout << capName << " recording stopped\n";
}