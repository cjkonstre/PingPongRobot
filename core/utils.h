#pragma once

#include "config.h"
#include "kinematics/inverseK/inverseKin.hpp"
#include "measurements/dimensions.h"
#include <array>
#include <iostream>

KinematicsSolver<DOFS> make_kinSolver();

const Packet actionPacket(const Frame action);

void waitInput(const char* message);
void waitInput();

void doHoming_presetPos(MotorController& controller, const std::array<double, DOFS> presetQs, const double post_taught);