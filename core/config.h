//both settings and config, as its pretty small rn
//should include form lib, but not util

#pragma once

#include "kinematics/SCComms/packet.h"
#include "measurements/dimensions.h"
#include "kinematics/SCComms/motorController.hpp"
#include <array>

//config settings
#define DOFS 7
#define PACKET_FRAME_N 5

//aliases
using Frame = MotionFrameD<DOFS>;
using Packet = MotionPacketD<DOFS, PACKET_FRAME_N>;
using MotorController = MotorControllerD<DOFS, Packet>;

//constant positions
#define ORI_FORWARD {0, 1, 0}
#define ORI_UPWARD {0, 0, 1}
constexpr std::array<double, 3> home_pos = {TABLE_WIDTH/2, PADDLE_HEIGHT/2, 8._mm};
constexpr std::array<double, 3> home_ori = ORI_UPWARD;
constexpr std::array<double, 3> idle_pos = {TABLE_WIDTH/2, 0, 0.4_m};
constexpr std::array<double, 3> idle_ori = ORI_FORWARD;

//speeds
constexpr std::array<double, 3> spatial_maxVels = {10, 10, 10}; 
constexpr std::array<double, 3> spatial_maxAccels = {20, 20, 20}; //25 is too much acc for higher speeds, starts to skip. at least @ 1.4A
constexpr std::array<double, 3> spatial_maxJerks = {400, 400, 400}; 