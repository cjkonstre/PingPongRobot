//move this to a lib, along with detection code

#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <stdio.h>
#include <iostream>

int main() {
    cv::Mat markerImage;
    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);

    int mkrId = 21;
    cv::aruco::drawMarker(dictionary, mkrId, 200, markerImage, 1);
    std::string imn = "4X4_50-" + std::to_string(mkrId) + ".png";
    cv::imwrite("/home/connor/PingPongRobot/core/assets/markers/"+imn, markerImage); //MAKE SURE TO ALIGN
    cv::waitKey();

    return 0;
}
