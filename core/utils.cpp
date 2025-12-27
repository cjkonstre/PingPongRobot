#include "utils.h"
#include <iostream>

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

std::array<double, 3> randVector(std::array<double, 3> mins, std::array<double, 3> maxs) {
    std::array<double, 3> vect;
    for (int i=0; i<3; i++){
        vect[i] = mins[i] + static_cast <float> (rand()) /( static_cast <float> (RAND_MAX/(maxs[i]-mins[i])));
    }
    return vect;
}