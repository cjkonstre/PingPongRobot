#include "utils.h"
#include <iostream>

KinematicsSolver<DOFS> make_kinSolver() {return KinematicsSolver<DOFS>(frame_pulleyPoss, paddle_anchorOffsets, pulley_anchorOffsets_refOri);}

void doHoming_presetPos(MotorController& controller, const std::array<double, DOFS> presetQs, const double post_taught)
{
    for (int i=0; i<DOFS; i++) {controller.sendCommand(MOTOR_DISABLE, i);} //disable all motors
    for (int i=0; i<DOFS; i++){
        double input;
        std::cout << "Move axis " << i << " till taught. enter 1 to continue\n";
        std::cin >> input;

        controller.sendCommand(MOTOR_ENABLE, i);
        controller.setOffset(i, -(presetQs[i]+post_taught));
    }
    //loop over and do micro adjustments to make cables nice and taught?
}