#include <iostream>

#include "kinematics/inverseK/inverseKin.hpp"
#include "kinematics/motionPather/motionPather.hpp"
#include "config/config.h"
#include "utils.h"

#include "misc/3dRenderer/3dRenderer.h"

inline Eigen::Vector3d toEigenVec(const std::array<double, 3>& arr) {return Eigen::Vector3d(arr[0], arr[1], arr[2]);}

#include <atomic>
#include <thread>
#include <sys/resource.h>
#include <sys/syscall.h>
#include <unistd.h>

std::atomic<bool> renderRunning{false};

template <typename MP>
void renderThreadFn(MP& mp, KinematicsSolver<DOFS>& kin) {
    pid_t tid = (pid_t)syscall(SYS_gettid);
    setpriority(PRIO_PROCESS, tid, 10);   // +10 → lower priority (range -20..+19)

    // optional: keep it off the RT core (you pin the control loop to core 2)
    // cpu_set_t set; CPU_ZERO(&set);
    // for (int c = 0; c < CPU_SETSIZE; ++c) if (c != 2) CPU_SET(c, &set);
    // pthread_setaffinity_np(pthread_self(), sizeof(set), &set);

    viz3d::init(1280, 800, "wibl");
    renderUtils::plot::TripleBuf posBuf, velBuf, accBuf;
    auto plotT0 = std::chrono::steady_clock::now();
    const float plotWindow = 10.0f;

    while (renderRunning.load(std::memory_order_acquire) && viz3d::open()) {
        viz3d::begin();
        renderUtils::vis3d::drawAxes();
        renderUtils::vis3d::drawTable();

        auto snap = mp.getSnapshot();
        if (snap.valid) {
            renderUtils::vis3d::drawPaddle(kin, snap.position, snap.normal, snap.velocity);

            float tNow = std::chrono::duration<float>(
                             std::chrono::steady_clock::now() - plotT0).count();
            posBuf.add(tNow, snap.position);
            velBuf.add(tNow, snap.velocity);
            accBuf.add(tNow, snap.acceleration);

            ImGui::SetNextWindowPos(ImVec2(1300, 100), ImGuiCond_FirstUseEver);
            ImGui::Begin("Motion");
            renderUtils::plot::plotTriple("Position",     "m",     posBuf, tNow, plotWindow);
            renderUtils::plot::plotTriple("Velocity",     "m/s",   velBuf, tNow, plotWindow);
            renderUtils::plot::plotTriple("Acceleration", "m/s^2", accBuf, tNow, plotWindow);
            ImGui::End();
        }
        viz3d::end();
    }

    viz3d::shutdown();
}

int main() {
    std::srand(21);
    
    auto kinConfig = load_configs(KINCONFIG_PATH);
    std::cout << "configs loaded\n";

    /* --instantiate and such-- */
    std::unique_ptr<MotorController> teensy; try {
        std::cout << "Trying connection at /dev/ttyACM0...\n";
        teensy = std::make_unique<MotorController>("/dev/ttyACM0");
    } catch (const boost::wrapexcept<boost::system::system_error>&) {
        std::cout << "Trying connection at /dev/ttyACM1...\n";
        teensy = std::make_unique<MotorController>("/dev/ttyACM1");
    } std::cout << "Connected\n";

    KinematicsSolver<DOFS> kin(kinConfig.pulleyPoss, paddle_anchorOffsets, pulley_anchorOffsets_refOri);

    MotionPather<MotorController, KinematicsSolver<DOFS>> mp(
        kinConfig.control_cycle, kinConfig.speeds,
        idle_pose, home_pose,
        *teensy, kin,
        false //go idlepos on idle
    );

    std::array<double, DOFS> home_qs = kin.doIK(home_pose.pos, home_pose.ori.n(), {0,0,0}, {0,0,0}).qs;
    doHoming_presetPos(*teensy, home_qs);
    
    /* --start code-- */
    waitInput("begin");

    //viz thread
    renderRunning.store(true);
    std::thread renderThread(renderThreadFn<decltype(mp)>, std::ref(mp), std::ref(kin));



    Pose target; 
    target.pos = idle_pose.pos;//{home_pose.pos[0], 100._cm, home_pose.pos[2]};
    target.ori = ORI_sp_FORWARD; //bounds of both at [-pi/2, pi/2]
    //mp.setTarget(target,  Pose0vels);
    
    mp.begin(); //idlepos by default once started 

    //   this is a rotation test
    waitInput();
    target.ori = {0, PI/8}; 
    mp.setTarget(target, Pose0vels);

    waitInput();
    target.ori = {0, -PI/8}; 
    mp.setTarget(target, Pose0vels);

    waitInput();
    target.ori = {PI/8, -0}; 
    mp.setTarget(target, Pose0vels);

    waitInput();
    target.ori = {-PI/8, 0}; 
    mp.setTarget(target, Pose0vels);

    //some movement
    target.ori={0, 0};
    for (;;) {
        if (waitInput()=='q') break;
        target.pos = randVector({0, 0, PADDLE_HEIGHT}, {TABLE_WIDTH, TABLE_LENGTH-25._cm, 0.8_m});
        mp.setTarget(target,  Pose0vels);
    }
    
    waitInput("home");
    mp.setTarget(home_pose, Pose0vels);
    sleep(3);

    renderRunning.store(false);
    if (renderThread.joinable()) renderThread.join();

    mp.stop();
    return 0;
}