//fancy, can maybe speed up detection by doing predictive projective ROI cropping
//souped up stereo pair

#pragma once

#include "vision/stereo/stereoPair.h"

//3 cams = 3 stereo pairs
//does the stuff, doesnt interpret. returns measurement of the ball.
class TriStereo {
private:
    static const std::string confpath;
    Camera& cam1, & cam2, & cam3;
    BallDetector& balldet;
    StereoPair st1, st2, st3;
public:
    TriStereo(Camera& cam1, Camera& cam2, Camera& cam3, BallDetector& balldet);

    GaussBlob<3> getMeasurement(); //oh yeah
    GaussBlob<3> getMeasurement(const GaussBlob<3>& predicted, float uncertaintyF = 1); //predictive ROI
};