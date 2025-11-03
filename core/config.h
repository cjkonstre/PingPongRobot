//both settings and config, as its pretty small rn

#pragma once
#include "kinematics/SCComms/packet.h"

#define DOFS 7

#define PACKET_FRAME_N 5
using Frame = MotionFrameD<DOFS>;
using Packet = MotionPacketD<DOFS, PACKET_FRAME_N>;

