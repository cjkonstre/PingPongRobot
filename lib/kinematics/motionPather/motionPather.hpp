#pragma once

#include <thread>
#include <atomic>
#include <array>
#include <chrono>
#include <pthread.h>
#include <ruckig/ruckig.hpp>
#include <cmath>
#include "kinematics/Pose.h"
#include "kinematics/motionPather/motionScheduler.h"

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

    void blockingWait(auto& next_tick, const auto& waittime, const auto& frame); // waits for update

    bool scheduleAttached = false;
    MotionScheduler schedule;
    bool gotoIdleOnFinish;


public:
    struct MotionExpose{ //to expose the current state. for visualization, not much else
        std::array<double, 3> position  = {}; //yes ik this is a trainwreck of code, id redo all this is ROS if i had the time
        std::array<double, 3> velocity  = {};
        std::array<double, 3> acceleration  = {};
        std::array<double, 3> normal    = {};
        bool valid = false;
    };

private:
    std::atomic<MotionExpose*> snapFront{ new MotionExpose() };
    std::atomic<MotionExpose*> snapBack { new MotionExpose() };

public:

    inline MotionExpose getSnapshot() const {return *snapFront.load(std::memory_order_acquire);}

    ruckig::InputParameter<5> input;
    double velFactor;

    MotionPather(double control_cycle,
                 const SpeedsConfig<5>& speeds,
                 const Pose& idlePose, const Pose& currentPose,
                 MC& mc, KS& kin,
                 bool gotoIdleOnFinish);

    void setTarget(const Pose& target_pose,
                   const Pose& target_vel, //final rot is assumed 0
                   double min_dur = 0.0);
    inline void setTarget(const MotionScheduler::Frame& frame);

    void begin();
    void stop();

    void attachSchedule(MotionScheduler& sc);
};

#include "kinematics/motionPather/motionPather.tpp"