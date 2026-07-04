//meant to react to the ball. not terribly usefu but a testing area

#include <iostream>
#include <atomic>
#include <algorithm>

#include "vision/ballDet/ballDet.h"
#include "vision/informationSystem/informationSystem.h"
#include "vision/kalmanFilter/kalmanFilter.h"
#include "misc/gaussianBlob.h"

#include "config/config.h"
#include "utils.h"
#include "renderUtils.h"
#include "misc/timeLog/timeLog.hpp"

//for program halts
std::atomic<bool> mainLooping(true); void signal_handler(int signal) {if(signal==SIGINT)mainLooping=false;}

Camera camBL, camBR, camMR;
BallDetection det_BL, det_BR, det_MR;
GaussBlob<6> state;
GaussBlob<3> measurement;

int main() {
    /* --instantiate and such-- */

    //vision
    Camera camBL("/dev/cam_BL", CONF_PATH + "vision/cam_BL-conf.yml", 1280, 800, 120, 10);
    Camera camBR("/dev/cam_BR", CONF_PATH + "vision/cam_BR-conf.yml", 1280, 800, 120, 10);
    Camera camMR("/dev/cam_MR", CONF_PATH + "vision/cam_MR-conf.yml", 1920, 1080, 120, 50);
    BallDetector bdet_BL(DETCONFIG_PATH, camBL.capName); 
    BallDetector bdet_BR(DETCONFIG_PATH, camBR.capName); 
    BallDetector bdet_MR(DETCONFIG_PATH, camMR.capName); 
    
    InformationSystem<3> vision({&camBL, &camBR, &camMR}, 
                                {5., 5., 10.}); //px error in det

    float dt = 0.01; //secs. doesnt make a ton of sense to go below 10ms
    KalmanFilter kf(dt);

    //manual setup

    /* --start code-- */
    waitInput("begin");
    camBL.beginLoop(); bdet_BL.beginLoop(camBL);
    camBR.beginLoop(); bdet_BR.beginLoop(camBR);
    camMR.beginLoop(); bdet_MR.beginLoop(camMR);

    std::signal(SIGINT, signal_handler); 
    //code VV
    
    //main loop
    BallDetection det_BL, det_BR, det_MR;
    GaussBlob<3> measurement;

    //disp thread to show dets.
    std::atomic<bool> display_running{true};
    std::thread display_thread([&]{
        viz3d::init(1280, 720, "pos");
        viz3d::startRec("/home/connor/PingPongRobot/docs/3dballtrack_01.mp4", 120.0);

        while(display_running){
            auto im1 = camBL.frame_buffer.back().frame.clone();
            if (!im1.empty()){
            if (det_BL.found) cv::circle(im1, det_BL.center, 5, cv::Scalar(0, 255, 0), -1);
            cv::imshow("cam_BL", im1);}

            auto im2 = camBR.frame_buffer.back().frame.clone();
            if (!im2.empty()){
            if (det_BR.found) cv::circle(im2, det_BR.center, 5, cv::Scalar(0, 255, 0), -1);
            cv::imshow("cam_BR", im2);}

            auto im3 = camMR.frame_buffer.back().frame.clone();
            if (!im3.empty()){
            if (det_MR.found) cv::circle(im3, det_MR.center, 5, cv::Scalar(0, 255, 0), -1);
            cv::imshow("cam_MR", im3);}

            cv::waitKey(1);
            vis3d::drawGaussblob(GaussBlob<3>{kf.state().cov.topLeftCorner<3,3>(), 
                                             kf.state().mu.head<3>()}, 3.5, 1.f, 0.f, 0.f);
            viz3d::begin();
            vis3d::drawAxes();
            vis3d::drawTable();

            viz3d::sphere(measurement.mu[0], measurement.mu[2], measurement.mu[1], 2._cm, 0.75f, 0.25f, 0.);
            vis3d::drawGaussblob(measurement, 3.5, 1.f, 1.f, 1.f);
            vis3d::drawGaussblob(GaussBlob<3>{kf.state().cov.topLeftCorner<3,3>(), 
                                             kf.state().mu.head<3>()}, 3.5, 1.f, 0.f, 0.f);

            viz3d::end();
        }
    });

     
    auto next_tick = std::chrono::steady_clock::now();
    const auto waittime = std::chrono::duration_cast<std::chrono::steady_clock::duration>(std::chrono::duration<double>(dt));
    while (mainLooping.load(std::memory_order_relaxed)) {
        next_tick += waittime;

        bdet_BL.latestDetection(det_BL);
        bdet_BR.latestDetection(det_BR);
        bdet_MR.latestDetection(det_MR);
        //fast, >0.1ms

        measurement = vision.combineMeasurements({det_BL.center, det_BR.center, det_MR.center}, 
                                                 {det_BL.found,  det_BR.found,  det_MR.found});

        kf.predict(); kf.update(measurement);

        //viz is being run in a diff thread

        std::this_thread::sleep_until(next_tick); //may not be accurate enough, well see
    } std::cout << "exited \n";

    waitInput("end");
    /* --end code-- */

    camBL.release(); bdet_BL.endLoop();
    camBR.release(); bdet_BR.endLoop();
    camMR.release(); bdet_MR.endLoop();

    display_running = false; if(display_thread.joinable()) display_thread.join();
    viz3d::end();

    std::cout << "all done! :)\n";
    return 0;
}