//meant to react to the ball. not terribly usefu but a testing area

#include <iostream>
#include <atomic>

#include "kinematics/inverseK/inverseKin.hpp"
#include "kinematics/motionPather/motionPather.hpp"

#include "vision/ballDet/ballDet.h"
#include "vision/stereo/multiStereo.h"

#include "misc/gaussianBlob.h"
#include "player/kalmanFilter/kalmanFilter.h"
#include <algorithm>

#include "config/config.h"
#include "utils.h"

//for program halts
std::atomic<bool> mainLooping(false); void signal_handler(int signal) {if(signal==SIGINT)mainLooping=true;}

int main() {
    /* --instantiate and such-- */

    //kinematics
    auto kinConfig = load_configs(KINCONFIG_PATH);
    std::cout << "kinconfigs loaded\n";

    std::unique_ptr<MotorController> teensy; try {
        std::cout << "Trying connection at /dev/ttyACM0...\n";
        teensy = std::make_unique<MotorController>("/dev/ttyACM0");
    } catch (const boost::wrapexcept<boost::system::system_error>&) {
        std::cout << "Trying connection at /dev/ttyACM1...\n";
        teensy = std::make_unique<MotorController>("/dev/ttyACM1");
    } std::cout << "Connected\n";

    KinematicsSolver<DOFS> kin(kinConfig.pulleyPoss, paddle_anchorOffsets, pulley_anchorOffsets_refOri);

    MotionPather<MotorController, KinematicsSolver<DOFS>> mp(
        kinConfig.control_cycle, kinConfig.speeds,
        idle_pose, home_pose,
        *teensy, kin,
        false
    );

    //vision
    Camera camBL("/dev/cam_BL", CONF_PATH + "vision/cam_BL-intrinsics.yml", 1280, 800, 120, 35);
    Camera camBR("/dev/cam_BR", CONF_PATH + "vision/cam_BR-intrinsics.yml", 1280, 800, 120, 35);
    Camera camMR("/dev/cam_MR", CONF_PATH + "vision/cam_MR-intrinsics.yml", 1920, 1080, 120, 300);
    BallDetector balldet(DETCONFIG_PATH, camBL.capName); //need to fix. should couple camera and balldet into a sensor obj
    //change this eventually
    //InformationSystem<3> vision({camBL, camBR, camMR}, balldet.sensorNoise); //rn uses cams as read only for data.

    //manual setup
    doHoming_presetPos(*teensy, kin.doIK(home_pose.pos, home_pose.ori.n(), {0,0,0}, {0,0,0}).qs);

    /* --start code-- */
    waitInput("begin");
    camBL.beginLoop(); camBR.beginLoop(); camMR.beginLoop();
    mp.begin();
    std::signal(SIGINT, signal_handler); 
    //code VV
    
    //KalmanModel filter(process_noise, dt);

    //main loop
    while (mainLooping.load(std::memory_order_relaxed)) {
        //should have a frame alignment thing
        cv::Point2f c1, c2, c3; float rad;
        bool ret1 = balldet.findBall(camBL.frame_buffer[0], c1, rad);
        bool ret2 = balldet.findBall(camBR.frame_buffer[0], c2, rad);
        bool ret3 = balldet.findBall(camMR.frame_buffer[0], c3, rad);

        //disp dets
        {auto im1 = camBL.frame_buffer[0].frame.copy();
        cv::circle(im1, c1, 5, cv::Scalar(0, 255, 0), -1);
        cv::imshow("cam_BL", im1);

        auto im2 = camBR.frame_buffer[0].frame.copy();
        cv::circle(im1, c2, 5, cv::Scalar(0, 255, 0), -1);
        cv::imshow("cam_BR", im1);

        auto im3 = camMR.frame_buffer[0].frame.copy();
        cv::circle(im1, c3, 5, cv::Scalar(0, 255, 0), -1);
        cv::imshow("cam_MR", im1);

        cv::waitKey(0);}

        //GaussBlob<3> measurement = vision.combineMeasurements({c1, c2, c3}, {ret1, ret2, ret3});

    } std::cout << "exited \n";

    waitInput("end");
    /* --end code-- */
    mp.setTarget(home_pose, Pose0vels, 2.5); sleep(3);
    mp.stop();

    camBL.release(); camBR.release(); camMR.release();

    std::cout << "all done! :)\n";
    return 0;
}