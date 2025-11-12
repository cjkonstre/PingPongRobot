#include "utils.h"
#include <iostream>

KinematicsSolver<DOFS> make_kinSolver() {return KinematicsSolver<DOFS>(frame_pulleyPoss, paddle_anchorOffsets, pulley_anchorOffsets_refOri);}

const Packet actionPacket(const Frame action) {
    Packet packet;
    packet.frames[0] = action;
    packet.packetLength = 1;
    return packet;
}

void doHoming_presetPos(MotorController& controller, const std::array<double, DOFS> presetQs, const double post_taught)
{
    for (int i=0; i<DOFS; i++) {controller.sendCommand(MOTOR_DISABLE, i);} //disable all motors
    waitInput("Move axes till taught. Enter to continue\n");
    for (int i=0; i<DOFS; i++) {controller.sendCommand(MOTOR_ENABLE, i);} //enable all motors
    controller.setCurrentState(presetQs, {0, 0, 0});

    Frame post_taught_f;
    post_taught_f.dt=1.0;
    post_taught_f.dq_new.fill(0);
    post_taught_f.q_new = presetQs;
    for (int i=0; i<DOFS; i++) {post_taught_f.q_new[i] -= post_taught;}

    controller.sendPacket(actionPacket(post_taught_f));
    //controller.setCurrentState(presetQs, {0, 0, 0});
}

void waitInput(const char* message) {
    std::cout << message;
    std::cin.get();
}

void waitInput() {
    std::cout << "Enter to Continue...";
    std::cin.get();
}