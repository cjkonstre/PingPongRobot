#include "utils.h"
#include <iostream>

KinematicsSolver<DOFS> make_kinSolver() {return KinematicsSolver<DOFS>(frame_pulleyPoss, paddle_anchorOffsets, pulley_anchorOffsets_refOri);}


void doHoming_presetPos(MotorController& controller, const std::array<double, DOFS> presetQs) {
    for (int i=0; i<DOFS; i++) {controller.sendCommand(MOTOR_DISABLE, i);} //disable all motors
    waitInput("Move axes till taught. Enter to continue\n");
    for (int i=0; i<DOFS; i++) {controller.sendCommand(MOTOR_ENABLE, i);} //enable all motors

    controller.sendCommand(MOTOR_SETSTP, 0, presetQs);
}

void waitInput(const char* message) {
    std::cout << message;
    std::cin.get();
}

void waitInput() {
    std::cout << "Enter to Continue...";
    std::cin.get();
}