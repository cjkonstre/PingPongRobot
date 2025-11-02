#pragma once

#include <kinematics/kinUtil.h>
#include <cstdint>

#pragma pack(push, 1)
//frame, for sending to stepper controller. pretty much just a state+extraneous information. state will be target future state
template <int DoFs>
struct MotionFrameD {
    std::array<double, DoFs> q_new{};
    std::array<double, DoFs> dq_new{};
    int32_t frameIdx = 0;
    double dt = 1.0;
};
#pragma pack(pop)

#pragma pack(push, 1) //no padding
//packet to send over to the stepper controller, basically a packet of frames. this overcomes any latency issues in transmittance
template <int DoFs, int frameCount>
struct MotionPacketD {
    int32_t packetId = 0;
    int32_t packetLength = frameCount; //eq to framecount if it wasnt stopped early
    MotionFrameD<DoFs> frames[frameCount];
};
#pragma pack(pop)