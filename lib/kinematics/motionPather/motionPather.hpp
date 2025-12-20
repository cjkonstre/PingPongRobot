#pragma once

#include <thread>
#include <atomic>
#include <array>
#include <chrono>
#include <pthread.h>
#include <ruckig/ruckig.hpp>
#include <cmath>

//DIAG
//#define DOPATHERDIAGS
#ifdef DOPATHERDIAGS
#include "matplotlibcpp.h"
namespace plt = matplotlibcpp;
#endif

template <int DoFs>
struct SpeedsConfig {
    std::array<double, DoFs> max_vel;
    std::array<double, DoFs> max_acc;
    std::array<double, DoFs> max_jerk;
};

//ori is spherical rather than normal
struct Pose {
    std::array<double, 3> pos;
    std::array<double, 2> ori; //theta, phi. (0,0) is (1,0,0).
};

//internally uses 5 DOFS, 3 spatial and 2 for normal angles
template <typename MC, typename KS>
class MotionPather {
private:
    MC& mc;
    KS& kin;

    ruckig::Ruckig<5> otg;
    double control_cycle;
    ruckig::OutputParameter<5> output;

    Pose idlePose;

    std::thread worker;
    std::atomic<bool> running{false};

    // Pending target
    std::array<double, 5> pending_pose{};
    std::array<double, 5> pending_vel{};
    std::atomic<double> pendingMinDur{0.0};
    std::atomic<bool> pendingTarget{false};

    void loop();
    void pin_to_core(int core_id);
    void set_realtime_priority(int priority = 80);

public:
    ruckig::InputParameter<5> input;
    double velFactor;

    MotionPather(double control_cycle,
                 const SpeedsConfig<5>& speeds,
                 const Pose& idlePose,
                 const Pose& currentPose,
                 MC& mc,
                 KS& kin);

    void setTarget(const Pose& target_pose,
                   const std::array<double, 3>& target_vel, //final rot is assumed 0
                   double min_dur = 0.0);

    void begin();
    void stop();
};

#include "kinematics/motionPather/motionPather.tpp"