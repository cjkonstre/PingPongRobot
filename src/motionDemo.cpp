#include <boost/asio.hpp>
#include <iostream>

#include "kinematics/inverseK/inverseKin.hpp"
#include "measurements/dimensions.h"
#include "config/config.h" //where a lot of type aliases live
#include "utils.h"
#include "ruckig/ruckig.hpp"
#include "kinematics/motionPather/motionPather.hpp"

std::array<double, 3> orisp_to_normal(std::array<double, 2> orisp) {
    double sin_theta = sin(orisp[0]);
    double cos_theta = cos(orisp[0]);
    double sin_phi = sin(orisp[1]);
    double cos_phi = cos(orisp[1]);

    std::array<double, 3> normal = {sin_theta*cos_phi, 
                                    cos_theta*cos_phi, 
                                    sin_phi};

    return normal;
}

#include "matplotlibcpp.h"
namespace plt = matplotlibcpp;

void plot3D(const Eigen::Matrix<double, 3, 7>& P) {
    std::vector<double> xs, ys, zs;

    for (int i = 0; i < 7; ++i) {
        xs.push_back(P(0, i));
        ys.push_back(P(1, i));
        zs.push_back(P(2, i));
    }

    std::vector<double> tx = {0, TABLE_WIDTH,  TABLE_WIDTH,  0, 0};
    std::vector<double> ty = {0, 0,            TABLE_LENGTH, TABLE_LENGTH, 0};
    std::vector<double> tz = {0, 0, 0, 0, 0};

    plt::figure();

    xs.insert(xs.end(), tx.begin(), tx.end());
    ys.insert(ys.end(), ty.begin(), ty.end());
    zs.insert(zs.end(), tz.begin(), tz.end());

    plt::scatter(xs, ys, zs, 30.0);

    plt::xlabel("X-axis");
    plt::ylabel("Y-axis");
    plt::set_zlabel("Z-axis");

    plt::show();
}

inline Eigen::Vector3d toEigenVec(const std::array<double, 3>& arr) {return Eigen::Vector3d(arr[0], arr[1], arr[2]);}

int main() { 
    auto kinConfig = load_configs("/home/connor/PingPongRobot/core/config/kin_conf.json");
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
        *teensy, kin
    );

    std::array<double, DOFS> home_qs = kin.doIK(home_pos, orisp_to_normal(home_ori_sp), {0,0,0}, {0,0,0}).qs;
    doHoming_presetPos(*teensy, home_qs);
    
    /* --start code-- */
    waitInput("begin");

    Pose target; 
    target.pos = {home_pos[0]+20._cm, TABLE_LENGTH-50._cm, home_pos[2]+50._cm};
    target.ori = {0, 0};
    mp.setTarget(target,  {0, 0, 0});
    //plot3D(kin.getAnchorPoints(toEigenVec(target.pos), toEigenVec(orisp_to_normal(target.ori))));

    mp.begin(); //idlepos by default once started

    for (int i=0; i<3; i++) {
        waitInput();
        target.pos = randVector({0, 0, PADDLE_HEIGHT}, {TABLE_WIDTH, TABLE_LENGTH, 0.8_m});
        //target.pos = {home_pos[0]-20._cm, TABLE_LENGTH-50._cm, home_pos[2]+50._cm};
        mp.setTarget(target,  {0, 0, 0});

        //waitInput();
        //target.pos = {home_pos[0]+20._cm, TABLE_LENGTH-50._cm, home_pos[2]+50._cm};
        //mp.setTarget(target,  {0, 0, 0});
    }
    
    waitInput("home");
    mp.setTarget(home_pose, {0, 0, 0});
    sleep(3);

    mp.stop();
    return 0;
}