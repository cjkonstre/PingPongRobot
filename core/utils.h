#pragma once

#include "core/config.h"
#include "kinematics/inverseK/inverseKin.hpp"
#include "core/measurements/dimensions.h"
#include "kinematics/SCComms/motorController.h"

KinematicsSolver<DOFS> make_kinSolver();

void doHoming_presetPos(MotorController& controller, KinematicsSolver<DOFS> kin, std::array<double, DOFS> presetpos, std::array<double, 3> presetOri,double post_taught)