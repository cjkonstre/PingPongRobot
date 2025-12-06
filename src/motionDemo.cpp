#include <boost/asio.hpp>
#include <iostream>

#include "kinematics/inverseK/inverseKin.hpp"
#include "measurements/dimensions.h"
#include "config.h" //where a lot of type aliases live
#include "utils.h"
#include "ruckig/ruckig.hpp"
#include "kinematics/motionPather/motionPather.hpp"

static inline std::array<double,3> lerp_array(const std::array<double,3>& a,
                                              const std::array<double,3>& b,
                                              double t) {
    return {
        a[0] + (b[0] - a[0]) * t,
        a[1] + (b[1] - a[1]) * t,
        a[2] + (b[2] - a[2]) * t
    };
}

std::array<double, 3> randVector(std::array<double, 3> mins, std::array<double, 3> maxs) {
    std::array<double, 3> vect;
    for (int i=0; i<3; i++){
        vect[i] = mins[i] + static_cast <float> (rand()) /( static_cast <float> (RAND_MAX/(maxs[i]-mins[i])));
    }
    return vect;
}

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

inline Eigen::Vector3d toEigenVec(const std::array<double, 3>& arr) {
    return Eigen::Vector3d(arr[0], arr[1], arr[2]);
}

int main() { 
    /* --instantiate and such-- */
    std::unique_ptr<MotorController> teensy; try {
        std::cout << "Trying connection at /dev/ttyACM0...\n";
        teensy = std::make_unique<MotorController>("/dev/ttyACM0");
    } catch (const boost::wrapexcept<boost::system::system_error>&) {
        std::cout << "Trying connection at /dev/ttyACM1...\n";
        teensy = std::make_unique<MotorController>("/dev/ttyACM1");
    } std::cout << "Connected\n";

    KinematicsSolver<DOFS> kin = make_kinSolver();
    std::array<double, DOFS> home_qs = kin.doIK(home_pos, orisp_to_normal(home_ori_sp), {0,0,0}, {0,0,0}).qs;
    doHoming_presetPos(*teensy, home_qs);

    MotionPather<MotorController, KinematicsSolver<DOFS>> mp(
        control_cycle, maxSpeeds,
        idle_pose, home_pose,
        1/(PI*pulley_diameter),
        *teensy, kin
    );

    /* --start code-- */
    waitInput("begin");

    Pose target; 
    target.pos = {home_pos[0]+20._cm, TABLE_LENGTH-50._cm, home_pos[2]+50._cm};
    target.ori = {0, 0};
    mp.setTarget(target,  {0, 0, 0});
    //plot3D(kin.getAnchorPoints(toEigenVec(target.pos), toEigenVec(orisp_to_normal(target.ori))));

    mp.begin(); //idlepos by default once started

    //waitInput("turn");
    //target.ori = {PI/4, 0};
    //mp.setTarget(target,  {0, 0, 0});
    //plot3D(kin.getAnchorPoints(toEigenVec(target.pos), toEigenVec(orisp_to_normal(target.ori))));

    waitInput("home");
    mp.setTarget(home_pose, {0, 0, 0});
    sleep(3);

    mp.stop();
    VelFrameD<7> endframe; endframe.index=1; endframe.vels.fill(0.);
    teensy->sendVel(endframe);
    return 0;
}