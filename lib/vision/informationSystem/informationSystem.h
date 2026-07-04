#pragma once

#include <opencv2/opencv.hpp>
#include <Eigen/Dense>

#include <array>
#include <initializer_list>

#include "vision/camera/camera.h"
#include "misc/gaussianBlob.h"

template <int N>
class InformationSystem {
private:
    std::array<const Camera*, N> sensors;
    std::array<double, N> sigmaPx;

public:
    InformationSystem(
        const std::array<const Camera*, N>& cams,
        const std::array<double, N>& sensorNoisesPx
    );
    InformationSystem(
        std::initializer_list<const Camera*> cams,
        std::initializer_list<double> sensorNoisesPx
    );

    GaussBlob<3> combineMeasurements(
        std::initializer_list<cv::Point2f> centers,
        std::initializer_list<bool> rets
    );

    GaussBlob<3> combineMeasurements(
        const std::array<cv::Point2f, N>& centers,
        const std::array<bool, N>& rets
    );
};