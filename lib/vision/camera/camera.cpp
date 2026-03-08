#include "vision/camera/camera.h"

#include <stdexcept>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/videodev2.h>

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
    : capPath(capPath),
      capName(capPath)
{
    cap.open(capPath);

    if (!cap.isOpened())
        throw std::runtime_error("Failed to open camera: " + capPath);

    cap.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M','J','P','G'));
    cap.set(cv::CAP_PROP_FRAME_WIDTH, frameWidth);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, frameHeight);
    cap.set(cv::CAP_PROP_FPS, fps);

    int fd = open(capPath.c_str(), O_RDWR);
    if (fd >= 0) {
        setControl(fd, V4L2_CID_EXPOSURE_AUTO, V4L2_EXPOSURE_MANUAL);
        setControl(fd, V4L2_CID_EXPOSURE_ABSOLUTE, exposureSetting);
        close(fd);
    }

    cv::FileStorage fs(intrinsicsPath, cv::FileStorage::READ);
    if (!fs.isOpened()) throw std::runtime_error("Failed to open intrinsics file: " + intrinsicsPath);
    fs["camera_matrix"] >> intr.K;
    fs["dist_coeffs"] >> intr.D;
    fs.release();
}

inline bool Camera::isOpened() const return cap.isOpened();

inline bool Camera::grab() return cap.grab();

inline cv::Mat Camera::retrieve() {
    cv::Mat frame;
    cap.retrieve(frame);
    return frame;
}

inline void Camera::read(cv::Mat& frame) const cap.read(frame);

inline void Camera::release() cap.release();