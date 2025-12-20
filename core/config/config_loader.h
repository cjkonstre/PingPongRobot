#pragma once

#include <array>
#include <string>
#include "kinematics/motionPather/motionPather.hpp"

struct KinConfig final {
    SpeedsConfig<5> speeds;
    float control_cycle; //in us, float so that whatever arithmiic done will be float
    std::array<std::array<double, 3>, 7> pulleyPoss;
};

KinConfig load_configs(const std::string& path);