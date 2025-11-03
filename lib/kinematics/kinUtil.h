#pragma once

#include <array>

//instantaneous state/pos of motion
template <int DoFs>
struct MotionStateD {
    std::array<double, DoFs> qs;
    std::array<double, DoFs> dqs;

    MotionStateD(std::array<double, DoFs> qs, std::array<double, DoFs> dqs) : qs(qs), dqs(dqs) {}
};
