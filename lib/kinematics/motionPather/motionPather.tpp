#include <iostream>
#include "kinematics/motionPather/motionPather.hpp"
#include "kinematics/SCComms/packet.h"
#include "utils/timeLog/timeLog.hpp"

#define START_TIME using std::chrono::high_resolution_clock; \
                    using std::chrono::duration_cast; \
                    using std::chrono::duration; \
                    using std::chrono::milliseconds; \
                    auto timing_nonameconflict_t1 = high_resolution_clock::now(); \

#define PRINT_TIME auto timing_nonameconflict_t2 = high_resolution_clock::now(); \
                    auto ms_int = duration_cast<milliseconds>(timing_nonameconflict_t2 - timing_nonameconflict_t1); \
                    duration<double, std::milli> ms_double = timing_nonameconflict_t2 - timing_nonameconflict_t1; \
                    std::cout << ms_double.count() * 1000<< "us\n"; \


template <typename MC, typename KS>
MotionPather<MC, KS>::MotionPather(
        double control_cycle_us,
        const SpeedsConfig<5>& speeds,
        const Pose& idlePoseIn,
        const Pose& currentPose,
        MC& mc,
        KS& kin,
        bool gotoIdleOnFinish
) : mc(mc), kin(kin),
    otg(control_cycle_us/1.e6),
    control_cycle(control_cycle_us/1.e6),
    idlePose(idlePoseIn),
    gotoIdleOnFinish(gotoIdleOnFinish) {

    // Configure Ruckig limits
    input.max_velocity     = speeds.max_vel;
    input.max_acceleration = speeds.max_acc;
    input.max_jerk         = speeds.max_jerk;

    input.current_position = currentPose.to5vec();
    input.target_position = idlePose.to5vec();

    input.current_velocity.fill(0.0);
    input.current_acceleration.fill(0.0);

    input.target_velocity.fill(0.0);
    input.target_acceleration.fill(0.0);

}

template <typename MC, typename KS>
void MotionPather<MC, KS>::begin() {
    if (running.load()) return;

    running.store(true);
    worker = std::thread(&MotionPather::loop, this);
}

template <typename MC, typename KS>
void MotionPather<MC, KS>::stop() {
    running.store(false);
    if (worker.joinable())
        worker.join();
}

//not tots sure if this works
template <typename MC, typename KS>
void MotionPather<MC, KS>::pin_to_core(int core_id) {
    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    CPU_SET(core_id, &cpuset);
    pthread_setaffinity_np(pthread_self(), sizeof(cpu_set_t), &cpuset);
}

template <typename MC, typename KS>
void MotionPather<MC, KS>::set_realtime_priority(int priority) {
    sched_param sch;
    sch.sched_priority = priority;
    pthread_setschedparam(pthread_self(), SCHED_FIFO, &sch);
}

template <typename MC, typename KS>
void MotionPather<MC, KS>::blockingWait(auto& next_tick, const auto& waittime, const auto& frame){
    while (running.load(std::memory_order_acquire) && !pendingTarget.load(std::memory_order_acquire)) {
        next_tick += waittime;
        mc.sendFrame(frame); //do nothing until new target is given
        std::this_thread::sleep_until(next_tick);
    }
}

template <typename MC, typename KS>
void MotionPather<MC, KS>::loop() {

pin_to_core(2);             // change if desired
set_realtime_priority(80);  // needs CAP_SYS_NICE

auto next_tick = std::chrono::steady_clock::now();
const auto waittime = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
    std::chrono::duration<double>(control_cycle));

#ifdef DOPATHERDIAGS
std::array<std::vector<double>, 5> plists; //DIAG
std::array<std::vector<double>, 5> dplists; //DIAG
std::vector<double> ts; ts.push_back(0);
#endif

while (running.load(std::memory_order_acquire)) {
    //check for update target request
    if (pendingTarget.load(std::memory_order_acquire)) {
        input.target_position = pending_pose;
        input.target_velocity = pending_vel;
        input.minimum_duration = pendingMinDur.load(std::memory_order_relaxed);

        pendingTarget.store(false, std::memory_order_release);
    }

    next_tick += waittime;
    auto result = otg.update(input, output); //ruckig happens here
    output.pass_to_input(input);

    #ifdef DOPATHERDIAGS
    for (int i=0; i<5; i++) {plists[i].push_back(input.current_position[i]);}
    for (int i=0; i<5; i++) {dplists[i].push_back(input.current_velocity[i]);}
    ts.push_back(ts.back()+=control_cycle);
    #endif


    //START_TIME //for timing of the whole IK cycle
    //if these computations are limiting, can switch the IK to use spherical coords
    double theta = input.current_position[3]; double phi = input.current_position[4];
    double sin_theta = sin(theta);  double sin_phi = sin(phi); 
    double cos_theta = cos(theta); double cos_phi = cos(phi);

    double Dtheta = input.current_velocity[3]; double Dphi = input.current_velocity[4];

    std::array<double, 3> normal = {sin_theta*cos_phi,
                                    cos_theta*cos_phi,
                                    sin_phi};
    std::array<double, 3> Dnormal = {
        Dtheta * cos_theta * cos_phi - Dphi * sin_theta * sin_phi,
        -Dtheta * sin_theta * cos_phi - Dphi * cos_theta * sin_phi,
        Dphi * cos_phi
    };

    std::array<double, 7> poss = kin.doIK(
        {input.current_position[0], input.current_position[1], input.current_position[2]},
        normal,
        {input.current_velocity[0], input.current_velocity[1], input.current_velocity[2]},
        Dnormal
    ).qs;

    //PRINT_TIME //for timing of the whole IK cycle

    PosFrameD<7> frame;
    frame.index = 1;
    for (int i = 0; i < 7; i++) {frame.poss[i] = poss[i];}
    //TIMELOG << "sending frame...\n";
    mc.sendFrame(frame);
    //TIMELOG << "done\n";

    //TIMELOG; for (int i = 0; i < 7; i++) {std::cout << frame.poss[i] << ", ";} std::cout << "\n";

    if (result == ruckig::Result::Finished) {
        TIMELOG << "target reached\n";
        //DIAG
        #ifdef DOPATHERDIAGS
        ts.pop_back();
        plt::figure(1);
        plt::plot(ts, plists[0], {{"label", "X"}});
        plt::plot(ts, plists[1], {{"label", "Y"}});
        plt::plot(ts, plists[2], {{"label", "Z"}});
        plt::plot(ts, plists[3], {{"label", "Theta"}});
        plt::plot(ts, plists[4], {{"label", "Phi"}});

        plt::legend();
        plt::xlabel("s [s]");
        plt::ylabel("spatial");

        plt::figure(2);
        plt::plot(ts, dplists[0], {{"label", "dX"}});
        plt::plot(ts, dplists[1], {{"label", "dY"}});
        plt::plot(ts, dplists[2], {{"label", "dZ"}});
        plt::plot(ts, dplists[3], {{"label", "dTheta"}});
        plt::plot(ts, dplists[4], {{"label", "dPhi"}});

        plt::legend();
        plt::xlabel("s [s]");
        plt::ylabel("1st deriv");
        plt::show();

        for (int i=0; i<5; i++) {plists[i].clear();}
        for (int i=0; i<5; i++) {dplists[i].clear();}
        ts.clear(); ts.push_back(0);

        #endif

        if (scheduleAttached && !schedule.isFinished()) {
            setTarget(schedule.at()); schedule.inc(); //schedule func
        } else if (gotoIdleOnFinish) {
            setTarget(idlePose, Pose{{0, 0, 0}, {0, 0}}); //if go home on idle, set that as target. should loop this in
        } else {blockingWait(next_tick, waittime, frame);}

        }

    std::this_thread::sleep_until(next_tick); //will have a delay on one control cycle after the idle waiting. 1-5ms reaction delay isnt a huge deal
}

}

template <typename MC, typename KS>
void MotionPather<MC, KS>::setTarget(
        const Pose& target_pose,
        const Pose& target_vels,
        double min_dur) {

    pending_pose = target_pose.to5vec();
    pending_vel = target_vels.to5vec();

    pendingMinDur.store(min_dur, std::memory_order_release);
    pendingTarget.store(true, std::memory_order_release);
}

template <typename MC, typename KS>
inline void MotionPather<MC, KS>::setTarget(const MotionScheduler::Frame& frame) {
    setTarget(frame.spat, frame.vels, frame.minDur);
}

template <typename MC, typename KS>
void MotionPather<MC, KS>::attachSchedule(MotionScheduler& sc){
    scheduleAttached = true;
    schedule = sc;
    setTarget(schedule.at());
}