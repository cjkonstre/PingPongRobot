//meant to react to the ball. not terribly usefu but a testing area

#include <iostream>

#include "kinematics/inverseK/inverseKin.hpp"
#include "kinematics/motionPather/motionPather.hpp"
#include "config/config.h"
#include "utils.h"

#include "kinematics/motionPather/motionScheduler.h"

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
    

    MotionScheduler schedule;
    schedule.loop = false;
    MotionScheduler::Frame frame;

    double angle = 3*PI/8;
    for (int i=0; i<50; i++){
        frame.spat.pos = {TABLE_WIDTH/2, 0.25, 0.25};
        frame.spat.ori = {0, angle};
        frame.vels.pos = {0, cos(angle), sin(angle)};
        frame.vels.ori = {0, 0};
        schedule.add(frame);

        frame.spat.pos = {TABLE_WIDTH/2, 0.25+cos(angle), 0.25+cos(angle)};
        schedule.add(frame);
    }
    frame.vels = Pose0vels;
    schedule.add(frame);

    mp.attachSchedule(schedule);

    /* --start code-- */
    waitInput("begin");
    //sleep(10);
    mp.begin();



    waitInput("home");
    mp.setTarget(home_pose, Pose0vels, 2.5);
    sleep(3);
    mp.stop();
    return 0;
}