//just unspools a single motor and keeps track of the unspooled amount

#include <iostream>
#include "kinematics/inverseK/inverseKin.hpp"
#include "kinematics/motionPather/motionPather.hpp"
#include "config/config.h"
#include <chrono>

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

    const auto cycle = std::chrono::microseconds((int)kinConfig.control_cycle);
    using clock_type = std::chrono::steady_clock;

    std::cout << "motor to spool: ";
    int motorI;
    std::cin >> motorI;
    for (int i=0; i<7; i++) {teensy->sendCommand(MOTOR_DISABLE, i);}
    teensy->sendCommand(MOTOR_ENABLE, motorI);
    teensy->sendCommand(MOTOR_SETSTP, 0, {0, 0, 0, 0, 0, 0, 0});

    PosFrame currentLen{};
    currentLen.index = 1;
    for (int i = 0; i < 7; i++) currentLen.poss[i] = 0.0f;

    float dx;
    float targetPos=0;
    float vel = 0.1; //m/s

    while (true) {
        std::cout << "cm to move (q to quit): ";

        if (!(std::cin >> dx)) {
            if (std::cin.eof()) break;

            std::cin.clear();
            std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');

            std::cout << "Invalid numeric input.\n";
            continue;
        } 

        targetPos += dx/100;

        auto next_tick = clock_type::now();

        while (std::abs(currentLen.poss[motorI] - targetPos) > 0.0001) {

            next_tick += cycle;

            double error = targetPos - currentLen.poss[motorI];
            double step  = vel * (kinConfig.control_cycle / 1e6); // cm per cycle

            if (std::abs(error) <= step) {
                currentLen.poss[motorI] = targetPos;
            } else {
                currentLen.poss[motorI] += step * (error > 0 ? 1.0 : -1.0);
            }

            teensy->sendFrame(currentLen);

            std::this_thread::sleep_until(next_tick);
        }


        std::cout << currentLen.poss[motorI] <<"m \n";
    }
return 0;

}