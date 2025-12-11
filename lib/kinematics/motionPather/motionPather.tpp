#include <iostream>
#include "kinematics/motionPather/motionPather.hpp"

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
        double control_cycle,
        const SpeedsConfig<5>& speeds,
        const Pose& idlePoseIn,
        const Pose& currentPose,
        MC& mc,
        KS& kin
) : mc(mc), kin(kin),
    otg(control_cycle),
    control_cycle(control_cycle),
    idlePose(idlePoseIn) {

    // Configure Ruckig limits
    input.max_velocity     = speeds.max_vel;
    input.max_acceleration = speeds.max_acc;
    input.max_jerk         = speeds.max_jerk;

    // Initial state
    for (int i = 0; i < 3; ++i) {
        input.current_position[i]   = currentPose.pos[i];
        input.current_position[3+i] = currentPose.ori[i];

        input.target_position[i]   = idlePose.pos[i];
        input.target_position[3+i] = idlePose.ori[i];
    }

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
void MotionPather<MC, KS>::loop() {

pin_to_core(2);             // change if desired
set_realtime_priority(80);  // needs CAP_SYS_NICE

auto next_tick = std::chrono::steady_clock::now();
const auto waittime = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
    std::chrono::duration<double>(control_cycle));

//std::vector<double> xlst, ylst, zlst; //DIAG

while (running.load(std::memory_order_acquire)) {
    //check for update target request
    if (pendingTarget.load(std::memory_order_acquire)) {
        input.target_position = pending_pose;
        input.target_velocity = pending_vel;
        input.minimum_duration = pendingMinDur.load(std::memory_order_relaxed);

        pendingTarget.store(false, std::memory_order_release);
    }

    next_tick += waittime;
    auto result = otg.update(input, output);
    output.pass_to_input(input);

    #ifdef DOPATHERDIAGS
    xlst.push_back(input.current_position[0]); //DIAG
    ylst.push_back(input.current_position[1]);
    zlst.push_back(input.current_position[2]);
    #endif

    //START_TIME //for timing of the whole IK cycle
    //if these computations are limiting, can switch the IK to use spherical coords
    double theta = input.current_position[3];
    double phi = input.current_position[4];
    double sin_theta = sin(theta);
    double cos_theta = cos(theta);
    double sin_phi = sin(phi);
    double cos_phi = cos(phi);

    double Dtheta = input.current_velocity[3];
    double Dphi = input.current_velocity[4];

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
    mc.sendFrame(frame);

    //for (int i = 0; i < 7; i++) {std::cout << frame.poss[i] << ", ";} std::cout << "\n";

    if (result == ruckig::Result::Finished) {

        //DIAG
        #ifdef DOPATHERDIAGS
        std::vector<double> tx = {0, TABLE_WIDTH,  TABLE_WIDTH,  0, 0};
        std::vector<double> ty = {0, 0,            TABLE_LENGTH, TABLE_LENGTH, 0};
        std::vector<double> tz = {0, 0, 0, 0, 0};

        plt::figure();

        xlst.insert(xlst.end(), tx.begin(), tx.end());
        ylst.insert(ylst.end(), ty.begin(), ty.end());
        zlst.insert(zlst.end(), tz.begin(), tz.end());

        plt::scatter(xlst, ylst, zlst, 30);

        plt::xlabel("X-axis");
        plt::ylabel("Y-axis");
        plt::set_zlabel("Z-axis");

        plt::show();

        xlst.clear(); ylst.clear(); zlst.clear(); //DIAG
        #endif

        if (pendingTarget.load()) { //if this was a new target
            pendingTarget.store(false);
            setTarget(idlePose, {0, 0, 0});
        } else { //else, this was getting to idlepos. ON IDLE CONDITION
            //frames are position now, just keep sending them
            while (running.load(std::memory_order_acquire) && !pendingTarget.load(std::memory_order_acquire)) {
                next_tick += waittime;
                mc.sendFrame(frame); //do nothing until new target is given
                std::this_thread::sleep_until(next_tick);
            }

        }
    }

    std::this_thread::sleep_until(next_tick); //will have a delay on one control cycle after the idle waiting. 1-5ms reaction delay isnt a huge deal
}

}

template <typename MC, typename KS>
void MotionPather<MC, KS>::setTarget(
        const Pose& target_pose,
        const std::array<double, 3>& target_vel,
        double min_dur) {

    pending_vel.fill(0);

    for (int i = 0; i < 3; ++i) {
        pending_pose[i]   = target_pose.pos[i];
        pending_vel[i]=target_vel[i];
    }

    pending_pose[3] = target_pose.ori[0];
    pending_pose[4] = target_pose.ori[1];

    pendingMinDur.store(min_dur, std::memory_order_release);
    pendingTarget.store(true, std::memory_order_release);
}