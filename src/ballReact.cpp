//meant to react to the ball. not terribly usefu but a testing area

#include <iostream>
#include <atomic>

#include "kinematics/inverseK/inverseKin.hpp"
#include "kinematics/motionPather/motionPather.hpp"

#include "vision/ballDet/ballDet.h"
#include "vision/stereo/multiStereo.h"

#include "gaussianBlob.h"
#include "player/kalmanFilter/kalmanFilter.h"

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
    BallDetector balldet(DETCONFIG_PATH, camBL.capName); //need to fix

    TriStereo vis(camBL, camBR, camMR, balldet);

    //manual setup
    doHoming_presetPos(*teensy, kin.doIK(home_pose.pos, home_pose.ori.n(), {0,0,0}, {0,0,0}).qs);

    /* --start code-- */
    waitInput("begin");
    camBL.beginLoop();
    camBR.beginLoop();
    camMR.beginLoop();
    mp.begin();
    std::signal(SIGINT, signal_handler); 
    //code VV
    
    kalmanFilter filter(process_noise, dt);
    Eigen::Matrix<double, 6, 1>& ball_state = filter.state;

    //main loop
    while (mainLooping.load(std::memory_order_relaxed)) {
        filter.predict();
        if (filter.state.z < 0) filter.apply_mat(table_bounce_mat);
        
        GaussBlob<3> measurement; 
        if (vis.getMeasurement(measurement)) filter.combine(measurement);

        //do something smart with ball_state
    } std::cout << "exited \n"

    //code ^^
    waitInput("end"); //cleanup VV
    mp.setTarget(home_pose, Pose0vels, 2.5); sleep(3);
    mp.stop(); //ends mp loop

    camBL.release(); //end cam loops
    camBR.release();
    camMR.release();

    std::cout << "all done! :)\n";
    return 0;
}