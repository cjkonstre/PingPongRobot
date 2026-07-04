#pragma once

#include "config/config.h"
#include "kinematics/inverseK/inverseKin.hpp"
#include <array>
#include <iostream>
#include "vision/camera/camera.h"

int waitInput(const char* message);
int waitInput();

void doHoming_presetPos(MotorController& controller, const std::array<double, DOFS> presetQs);

std::array<double, 3> randVector(std::array<double, 3> mins, std::array<double, 3> maxs);

constexpr Pose Pose0vels{{0, 0, 0}, {0, 0}};


void synchCamrecsToNow(CameraRec& cam1, CameraRec& cam2, CameraRec& cam3);
