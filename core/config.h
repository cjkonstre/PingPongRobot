//both settings and config, as its pretty small rn
//should include form lib, but not util

#pragma once

#include "kinematics/SCComms/packet.h"
#include "measurements/dimensions.h"
#include "kinematics/SCComms/motorController.hpp"
#include <array>


#define DOFS 7


#define PACKET_FRAME_N 5
using Frame = MotionFrameD<DOFS>;
using Packet = MotionPacketD<DOFS, PACKET_FRAME_N>;

using MotorController = MotorControllerD<DOFS, Packet>;

constexpr std::array<double, 3> home_pos = {TABLE_WIDTH/2, TABLE_LENGTH-PADDLE_HEIGHT/2, 4._mm};
constexpr std::array<double, 3> home_ori = {0, 0, 1};