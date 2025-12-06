//both settings and config, as its pretty small rn
//should include form lib, but not util

#pragma once

#include "measurements/dimensions.h"
#include "kinematics/SCComms/packet.h"
#include "kinematics/SCComms/motorController.hpp"
#include "kinematics/motionPather/motionPather.hpp"
#include <array>

//config settings
#define PI 3.14159
#define DOFS 7
#define CONTROL_CYCLE_us 500. // 500 microseconds. limit seems to be on the computer side. too unreliable at 250us
constexpr double control_cycle = CONTROL_CYCLE_us/1.e6; //make SURE this aligns with what the mc expects


//aliases
using VelFrame = VelFrameD<DOFS>;
using MotorController = MotorControllerD<DOFS, VelFrame>;

//constant positions. sp mean spherical
#define ORI_sp_FORWARD {0, 0}
#define ORI_sp_UPWARD {0, PI/2} 

constexpr std::array<double, 3> home_pos = {TABLE_WIDTH/2, PADDLE_HEIGHT/2, 8._mm};
constexpr std::array<double, 2> home_ori_sp = ORI_sp_UPWARD;
constexpr Pose home_pose{home_pos, home_ori_sp};
constexpr std::array<double, 3> idle_pos = {TABLE_WIDTH/2, 50._cm, 0.4_m};
constexpr std::array<double, 2> idle_ori = ORI_sp_UPWARD;
constexpr Pose idle_pose{idle_pos, idle_ori};

//speeds. units in {m, m, m, rad, rad}. speeds for theta and phi arent physical
constexpr std::array<double, 5> maxVels = {4, 4, 4, 100, 100}; 
constexpr std::array<double, 5> maxAccels = {40, 40, 40, 100, 100}; //25 is too much acc for higher speeds, starts to skip. at least @ 1.4A
constexpr std::array<double, 5> maxJerks = {400, 400, 400, 500, 500}; 
constexpr SpeedsConfig<5> maxSpeeds{maxVels, maxAccels, maxJerks};