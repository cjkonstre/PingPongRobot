#pragma once

#include "config.h"
#include "kinematics/inverseK/inverseKin.hpp"
#include "measurements/dimensions.h"
#include <array>

//hello
KinematicsSolver<DOFS> make_kinSolver();

void doHoming_presetPos(MotorController& controller, const std::array<double, DOFS> presetQs, const double post_taught);