#include <opencv2/opencv.hpp>
#include <iostream>
#include <chrono>

int main() {
    // Open default camera (index 0). Change index if you have multiple cameras.
    cv::VideoCapture cap(2);
    if (!cap.isOpened()) {
        std::cerr << "Error: Could not open camera." << std::endl;
        return -1;
    }

    cv::Mat frame;
    int frameCount = 0;
    double totalTime = 0.0;

    while (true) {
        // Start timing
        auto start = std::chrono::high_resolution_clock::now();

        // Capture frame
        if (!cap.read(frame)) {
            std::cerr << "Error: Could not read frame." << std::endl;
            break;
        }

        // End timing
        auto end = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double, std::milli> duration = end - start;

        double ms = duration.count();
        double fps = 1000.0 / ms;

        frameCount++;
        totalTime += ms;

        std::cout << "Frame " << frameCount
                  << " | Time: " << ms << " ms"
                  << " | FPS: " << fps << std::endl;

        // Display frame
        cv::imshow("Camera", frame);
        if (cv::waitKey(1) == 27) { // press ESC to exit
            break;
        }
    }

    std::cout << "Average FPS: " << (frameCount * 1000.0 / totalTime) << std::endl;

    cap.release();
    cv::destroyAllWindows();
    return 0;
}
