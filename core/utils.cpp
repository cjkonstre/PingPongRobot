#include "core/utils.h"
#include "kinematics/SCComms/motorController.h"

KinematicsSolver<DOFS> make_kinSolver() {return KinematicsSolver<DOFS>(frame_pulleyPoss, paddle_anchorOffsets, pulley_anchorOffsets_refOri);}

void doHoming_presetPos(MotorController& controller, KinematicsSolver<DOFS> kin, std::array<double, 3> presetpos, std::array<double, 3> presetOri,
double post_taught) {

    std::array<double, DOFS> presetQs=kin.doIK(presetpos, presetOri, {0, 0, 0})

    for (int i=0; i<DOFS; i++) {controller.sendCommand(MotorController::MOTOR_DISABLE, i);} //disable all motors
    for (int i=0; i<DOFS; i++){
        double input;
        std::cout << "Move axis " << i << " till taught. enter 1 to continue\n";
        std::cin >> input;

        controller.sendCommand(MotorController::MOTOR_ENABLE, i);
        controller.setOffset(i, -(presetQs[i]+post_taught));
    }
    //loop over and do micro adjustments to make cables nice and taught?
}