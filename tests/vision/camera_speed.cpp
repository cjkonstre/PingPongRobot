#include <opencv2/opencv.hpp>
#include <iostream>
#include <chrono>
int main(int argc, char** argv) {
    // Default device if none provided
    std::string device = "/dev/video2";

    // If user passed an argument, use it
    if (argc >= 2) {
        device = argv[1];
    }

    std::cout << "Opening camera: " << device << std::endl;

    cv::VideoCapture cap(device, cv::CAP_V4L2);
    if (!cap.isOpened()) {
        std::cerr << "Error: Could not open camera." << std::endl;
        return -1;
    }
    
    cap.set(cv::CAP_PROP_FOURCC,
            cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
    cap.set(cv::CAP_PROP_FRAME_WIDTH, 1280);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, 800);
    cap.set(cv::CAP_PROP_FPS, 120);


    cv::Mat frame;
    int frameCount = 0;
    double totalTime = 0.0;

    while (true) {
        // Start timing
        auto start = std::chrono::high_resolution_clock::now();

        cap.grab();

        auto endgrap = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double, std::milli> grabdur = endgrap - start;

        cap.retrieve(frame);

        // End timing
        auto end = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double, std::milli> duration = end - start;

        double ms = duration.count();
        double fps = 1000.0 / ms;

        frameCount++;
        totalTime += ms;

        std::cout << "Frame " << frameCount
                  << " | Time: " << ms << " ms"
                  << " | FPS: " << fps 
                  << "  time to grab: " << grabdur.count() << "ms"
                  << std::endl;

        // Display frame
        cv::resize(frame, frame, cv::Size(1280*3./4, 800*3./4));
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
