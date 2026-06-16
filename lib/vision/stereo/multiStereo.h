//fancy, can maybe speed up detection by doing predictive projective ROI cropping
//souped up stereo pair

#pragma once

#include "vision/stereo/stereoPair.h"
#include "misc/gaussianBlob.h"

//3 cams = 3 stereo pairs
//does the stuff, doesnt interpret. returns measurement of the ball.
//multistereo things arent in charge of initializing cameras. do that beforehand!! if frames are empty, thats probs why
class TriStereo {
private:
    static const std::string confpath;
    Camera& cam1, & cam2, & cam3;
    BallDetector& balldet;
    StereoPair st1, st2, st3;
public:
    TriStereo(Camera& cam1, Camera& cam2, Camera& cam3, BallDetector& balldet);

    std::array<cv::Mat, 3> getAlignedFrames(float thresh = 1); // thresh dets how out of sunch -ness is acceptable. maybe dynamic, idk

    //these guys read into measurement,returning how succesful it was
    int getMeasurement(GaussBlob<3>& measurement); //oh yeah
    int getMeasurement(GaussBlob<3>& measurement, 
                                    const GaussBlob<3>& predicted, float uncertaintyF = 1); //predictive ROI

};