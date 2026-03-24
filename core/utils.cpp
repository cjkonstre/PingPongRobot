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

void synchCamrecsToNow(CameraRec& cam1, CameraRec& cam2, CameraRec& cam3){
    cam1.tsReader >> cam1.first_ts;
    cam2.tsReader >> cam2.first_ts;
    cam3.tsReader >> cam3.first_ts;
    uint64_t earlieststart = std::min({cam1.first_ts,  cam2.first_ts, cam3.first_ts});

    cam1.offset = cam1.first_ts - earlieststart;
    cam2.offset = cam2.first_ts - earlieststart;
    cam3.offset = cam3.first_ts - earlieststart;

    auto global_start_time = std::chrono::steady_clock::now();
    cam1.start_time = global_start_time;
    cam2.start_time = global_start_time;
    cam3.start_time = global_start_time;
}