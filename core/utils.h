#pragma once

#include "config/config.h"
#include "kinematics/inverseK/inverseKin.hpp"
#include "measurements/dimensions.h"
#include <array>
#include <iostream>

KinematicsSolver<DOFS> make_kinSolver();

void waitInput(const char* message);
void waitInput();

void doHoming_presetPos(MotorController& controller, const std::array<double, DOFS> presetQs);