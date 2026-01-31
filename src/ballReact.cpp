//meant to react to the ball. not terribly usefu but a testing area

#include <iostream>

#include "kinematics/inverseK/inverseKin.hpp"
#include "kinematics/motionPather/motionPather.hpp"
#include "config/config.h"
#include "utils.h"

#include "kinematics/motionPather/motionScheduler.h"
#include <opencv2/opencv.hpp>
#include "player/kalmanFilter/kalmanFilter.h"

int main() {
    auto kinConfig = load_configs(KINCONFIG_PATH);
    std::cout << "configs loaded\n";

    /* --instantiate and such-- */
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

    std::array<double, DOFS> home_qs = kin.doIK(home_pose.pos, home_pose.ori.n(), {0,0,0}, {0,0,0}).qs;
    doHoming_presetPos(*teensy, home_qs);


    Pose target;
    
    target.pos = {home_pose.pos[0]+20._cm, TABLE_LENGTH-50._cm, home_pose.pos[2]+50._cm};
    target.ori = {0, 0};
    mp.setTarget(target, Pose0vels, 2);

    /* --start code-- */
    waitInput("begin");
    mp.begin();

    waitInput();

    cv::Mat im(100, 100, CV_32FC1);
    while (true){
        cv::imshow("in", im);
        int k = cv::waitKey();

        if      (k=='w') {target.pos[1]+=0.1;}
        else if (k=='s') {target.pos[1]-=0.1;}
        else if (k=='a') {target.pos[0]+=0.1;}
        else if (k=='d') {target.pos[0]-=0.1;}
        else if (k=='q') {target.pos[2]+=0.1;}
        else if (k=='e') {target.pos[2]-=0.1;}

        else if (k=='j') {target.ori.theta+=0.1;}
        else if (k=='l') {target.ori.theta-=0.1;}
        else if (k=='i') {target.ori.phi+=0.1;}
        else if (k=='k') {target.ori.phi-=0.1;}

        else if (k==27) {break;}
        for (float i: target.to5vec()) {std::cout << i << ", ";} std::cout << "\n";

        //waitInput();
        mp.setTarget(target, Pose0vels);
    }
    cv::destroyAllWindows();

    waitInput("home");
    mp.setTarget(home_pose, Pose0vels, 2.5);
    sleep(3);
    mp.stop();
    return 0;
}