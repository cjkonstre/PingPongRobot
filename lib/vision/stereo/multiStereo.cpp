#include "vision/stereo/multiStereo.h"

#include <opencv2/core/eigen.hpp>
#include <future>

const std::string TriStereo::confpath="/home/connor/PingPongRobot/core/config/vision";

TriStereo::TriStereo(Camera& camera1, Camera& camera2, Camera& camera3, BallDetector& balldet)
 : cam1(camera1), cam2(camera2), cam3(camera3),
   balldet(balldet),
   st1(cam1, cam2, confpath + cam1.capName + "-" + cam2.capName + "-stereoConf"),
   st2(cam2, cam3, confpath + cam2.capName + "-" + cam3.capName + "-stereoConf"),
   st3(cam3, cam1, confpath + cam3.capName + "-" + cam1.capName + "-stereoConf") 
{}

//synch reads from all cams, parallely reads and dets ball
//inline void det3Fast_asynchronous -- be careful about nested threading, cv funcs used in findball might already be threaded

std::array<cv::Mat, 3> TriStereo::getAlignedFrames(float thresh) { //rn nothing fancy, just returns the most recent. worst case is a mismatch of 10 ish ms
    return {cam1.frame_buffer[0].frame, cam2.frame_buffer[0].frame, cam3.frame_buffer[0].frame};
}

GaussBlob<3> TriStereo::getMeasurement() { // not entirely sure, if moving the timing of the cap/det around, may need to change this up
    auto frames = getAlignedFrames();

    cv::Point2f c1, c2, c3; float r;
    balldet.findBall(frames[0], c1, r);
    balldet.findBall(frames[1], c2, r);
    balldet.findBall(frames[2], c3, r);

    return st1.get3dMeasurement(c1, c2) * st2.get3dMeasurement(c2, c3) * st3.get3dMeasurement(c3, c1);
}

inline void project_gaussian3(
    const GaussBlob<3>& g,
    const cv::Mat& P0,
    const cv::Mat& P1,
    const cv::Mat& P2,
    float uncertaintyF,
    cv::Rect2f out[3])
{
    const double X = g.mu(0);
    const double Y = g.mu(1);
    const double Z = g.mu(2);

    const double c0 = g.cov(0,0);
    const double c1 = g.cov(1,1);
    const double c2 = g.cov(2,2);

    const cv::Mat* Ps[3] = {&P0, &P1, &P2};

    for (int i = 0; i < 3; ++i)
    {
        const cv::Mat& P = *Ps[i];

        const double* p0 = P.ptr<double>(0);
        const double* p1 = P.ptr<double>(1);
        const double* p2 = P.ptr<double>(2);

        double w =
              p2[0]*X + p2[1]*Y + p2[2]*Z + p2[3];

        double mx =
            (p0[0]*X + p0[1]*Y + p0[2]*Z + p0[3]) / w;

        double my =
            (p1[0]*X + p1[1]*Y + p1[2]*Z + p1[3]) / w;

        //ignoring perspective effects
        double var_x =
              p0[0]*p0[0]*c0
            + p0[1]*p0[1]*c1
            + p0[2]*p0[2]*c2;

        double var_y =
              p1[0]*p1[0]*c0
            + p1[1]*p1[1]*c1
            + p1[2]*p1[2]*c2;

        float w_box = std::sqrt(var_x) * uncertaintyF;
        float h_box = std::sqrt(var_y) * uncertaintyF;

        out[i] = cv::Rect2f(
            cv::Point2f((float)mx, (float)my),
            cv::Size2f(w_box, h_box)
        );
    }
}

GaussBlob<3> TriStereo::getMeasurement(const GaussBlob<3>& predicted, float uncertaintyF) {
    auto frames = getAlignedFrames();

    cv::Rect2f rois[3]; //predictive rois. should speed up ball detection significantly
    project_gaussian3(
        predicted,
        cam1.proj, cam2.proj, cam3.proj,
        uncertaintyF,
        rois
    );

    cv::Point2f c1, c2, c3; float r;
    balldet.findBall(frames[0], c1, r, rois[0]);
    balldet.findBall(frames[1], c2, r, rois[1]);
    balldet.findBall(frames[2], c3, r, rois[2]);

    return st1.get3dMeasurement(c1, c2) * st2.get3dMeasurement(c2, c3) * st3.get3dMeasurement(c3, c1);
}