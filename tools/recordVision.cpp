// script to save video and save it to a file


#include "config/config.h"
#include "utils.h"
#include <opencv2/opencv.hpp>
#include <iostream>
#include <atomic>
#include <csignal>

#include "vision/camera/camera.h"

std::atomic<bool> stop(false);

void signal_handler(int signal) {
    if (signal == SIGINT) stop = true;
}

int main() {
    std::signal(SIGINT, signal_handler);

    const std::string outdir = "/home/connor/PingPongRobot/tests/test_data/";

    Camera camBL("/dev/cam_BL", CONF_PATH + "vision/cam_BL-intrinsics.yml", 1280, 800, 120, 35);
    Camera camBR("/dev/cam_BR", CONF_PATH + "vision/cam_BR-intrinsics.yml", 1280, 800, 120, 35);
    Camera camMR("/dev/cam_MR", CONF_PATH + "vision/cam_MR-intrinsics.yml", 1920, 1080, 90, 35);

    std::cout << "starts recording..." << std::endl;

    camBL.beginRecordingLoop(outdir);
    camBR.beginRecordingLoop(outdir);
    camMR.beginRecordingLoop(outdir);

    while (!stop.load(std::memory_order_relaxed)) std::this_thread::sleep_for(std::chrono::milliseconds(50));

    std::cout << "stoped reccing" << std::endl;

    camBL.endRecordingLoop();
    camBR.endRecordingLoop();
    camMR.endRecordingLoop();

    std::cout << "done" << std::endl;

    return 0;
}