//move this to a lib, along with detection code

#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <stdio.h>
#include <iostream>

int main() {
    cv::Mat markerImage;
    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_5X5_250);

    int mkrId = 23;
    cv::aruco::drawMarker(dictionary, mkrId, 200, markerImage, 1);
    std::string imn = "marker" + std::to_string(mkrId) + ".png";
    cv::imwrite("/home/cjk/PingPongRobot/markers/"+imn, markerImage);
    cv::waitKey();

    return 0;
}
