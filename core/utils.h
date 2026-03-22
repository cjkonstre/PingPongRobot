#pragma once

#include "config/config.h"
#include "kinematics/inverseK/inverseKin.hpp"
#include <array>
#include <iostream>

void waitInput(const char* message);
void waitInput();

void doHoming_presetPos(MotorController& controller, const std::array<double, DOFS> presetQs);

std::array<double, 3> randVector(std::array<double, 3> mins, std::array<double, 3> maxs);

constexpr Pose Pose0vels{{0, 0, 0}, {0, 0}};