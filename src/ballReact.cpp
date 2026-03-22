//meant to react to the ball. not terribly usefu but a testing area

#include <iostream>

#include "kinematics/inverseK/inverseKin.hpp"
#include "kinematics/motionPather/motionPather.hpp"

#include "vision/ballDet/ballDet.h"
#include "vision/stereo/multiStereo.h"

#include "config/config.h"
#include "utils.h"

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

    Camera camBL("/dev/cam_BL", CONF_PATH + "vision/cam_BL-intrinsics.yml", 1280, 800, 120, 35);
    Camera camBR("/dev/cam_BR", CONF_PATH + "vision/cam_BR-intrinsics.yml", 1280, 800, 120, 35);
    Camera camMR("/dev/cam_MR", CONF_PATH + "vision/cam_MR-intrinsics.yml", );
    BallDetector balldet(DETCONFIG_PATH, camBL.capName); //need to fix

    TriStereo vis(camBL, camBR, camMR, balldet);

    class PlayerModel {} smarts;

    //code V

    /* --start code-- */
    waitInput("begin");
    mp.begin();

    GaussBlob<3> currentState;

    bool running = true;
    while (running) {
        GaussBlob<3> measurement = vis.getMeasurement(currentState);
        currentState *= measurement; //pretend combine with easurement

        target = smarts.think(currentState.mu); //mu is 6 dims

        currentState = timeStep * currentState; //timestep forward
        waitUntilNext();
    }

    waitInput("home");
    mp.setTarget(home_pose, Pose0vels, 2.5);
    sleep(3);

    mp.stop();
    camBL.release();
    camBR.release();
    camMR.release();

    return 0;
}