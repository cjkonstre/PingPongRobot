//both settings and config, as its pretty small rn
//should include form lib, but not util

#pragma once

#include "measurements/dimensions.h"
#include "kinematics/SCComms/packet.h"
#include "kinematics/SCComms/motorController.hpp"
#include "kinematics/motionPather/motionPather.hpp"
#include <array>
#include "config/config_loader.h" //so can include config and be good

//config settings
#define PI 3.14159
#define DOFS 7

//aliases
using PosFrame = PosFrameD<DOFS>;
using MotorController = MotorControllerD<DOFS, PosFrame>;

//constant positions. sp mean spherical
#define ORI_sp_FORWARD {0, 0}
#define ORI_sp_UPWARD {0, PI/2} 

constexpr std::array<double, 3> home_pos = {TABLE_WIDTH/2, PADDLE_HEIGHT/2, 8._mm};
constexpr std::array<double, 2> home_ori_sp = ORI_sp_UPWARD;
constexpr Pose home_pose{home_pos, home_ori_sp};
constexpr std::array<double, 3> idle_pos = {TABLE_WIDTH/2, 50._cm, 0.4_m};
constexpr std::array<double, 2> idle_ori = ORI_sp_UPWARD;
constexpr Pose idle_pose{idle_pos, idle_ori};