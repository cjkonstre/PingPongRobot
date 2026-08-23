#include <opencv2/opencv.hpp>

#include <atomic>
#include <chrono>
#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <vector>

struct Camera {
    std::string device;
    cv::VideoCapture cap;

    std::atomic<uint64_t> frames{0};
    std::atomic<uint64_t> failures{0};

    Camera(const std::string& device)
        : device(device),
          cap(device, cv::CAP_V4L2)
    {
        if (!cap.isOpened()) {
            throw std::runtime_error("Failed to open " + device);
        }

        cap.set(cv::CAP_PROP_FOURCC,
                cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
        cap.set(cv::CAP_PROP_FRAME_WIDTH, 1280);
        cap.set(cv::CAP_PROP_FRAME_HEIGHT, 800);
        cap.set(cv::CAP_PROP_FPS, 120);

        std::cout << device
                  << " | "
                  << cap.get(cv::CAP_PROP_FRAME_WIDTH) << "x"
                  << cap.get(cv::CAP_PROP_FRAME_HEIGHT)
                  << " @ "
                  << cap.get(cv::CAP_PROP_FPS)
                  << " FPS\n";
    }

    void run(std::atomic<bool>& running)
    {
        cv::Mat frame;

        while (running) {
            if (!cap.grab()) {
                failures++;
                continue;
            }

            if (!cap.retrieve(frame)) {
                failures++;
                continue;
            }

            frames++;
        }
    }
};

int main(int argc, char** argv)
{
    if (argc < 2) {
        std::cerr << "Usage: " << argv[0]
                  << " /dev/cam_BL /dev/cam_BR /dev/cam_MR ...\n";
        return 1;
    }

    std::vector<std::unique_ptr<Camera>> cams;
    cams.reserve(argc - 1);

    try {
        for (int i = 1; i < argc; ++i) {
            cams.emplace_back(std::make_unique<Camera>(argv[i]));
        }
    }
    catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << '\n';
        return 1;
    }

    std::atomic<bool> running{true};

    // One thread per camera.
    std::vector<std::thread> threads;
    threads.reserve(cams.size());

    for (auto& cam : cams) {
        threads.emplace_back([&running, cam = cam.get()] {
            cam->run(running);
        });
    }

    // Measure for 10 seconds.
    constexpr int testSeconds = 10;

    std::this_thread::sleep_for(
        std::chrono::seconds(testSeconds));

    running = false;

    for (auto& thread : threads) {
        thread.join();
    }

    std::cout << "\nResults:\n";

    for (const auto& cam : cams) {
        double fps =
            static_cast<double>(cam->frames.load()) / testSeconds;

        std::cout << cam->device
                  << ": "
                  << fps
                  << " FPS"
                  << "  failures="
                  << cam->failures.load()
                  << '\n';
    }

    return 0;
}