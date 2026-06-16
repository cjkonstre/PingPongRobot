#pragma once

#include "vision/camera/camera.h"
#include "misc/gaussianBlob.h"


//main thing is it outputs a combined measurement
template <int N> //N is # of cams. assumed 3d space tho
class InformationSystem {
private:
    std::array<const Camera&, N> sensors; //read only

public:
    GaussBlob<3> combineMeasurements(std::initializer_list<cv::Point2f> centers, std::initializer_list<bool> rets);
    GaussBlob<3> combineMeasurements(const std::array<cv::Point2f, N>& centers, const std::array<bool, N>& rets);
};