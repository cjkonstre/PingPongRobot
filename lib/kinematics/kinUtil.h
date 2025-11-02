#pragma once //try to phase out

#include <vector>
#include <array>

//internal include header?
#define PACKET_FRAMECOUNT 10 //here?

//instantaneous state/pos of motion
template <int DoFs>
struct MotionStateD {
    std::array<double, DoFs> qs;
    std::array<double, DoFs> dqs;

    MotionStateD(std::array<double, DoFs> qs, std::array<double, DoFs> dqs) : qs(qs), dqs(dqs) {}
};

//sequential motion schedule. does it in order
template <int DoFs>
struct MotionSchedule {
    std::vector<MotionStateD<DoFs>> states;
};