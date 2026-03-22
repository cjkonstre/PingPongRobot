//tracks hand or something and has the robo copy it


#include <iostream>

#include "kinematics/inverseK/inverseKin.hpp"
#include "kinematics/motionPather/motionPather.hpp"
#include "config/config.h"
#include "utils.h"
#include "opencv2/opencv.hpp"
#include <opencv2/aruco.hpp>
#include <chrono>

using namespace std::chrono;

cv::Mat cameraMatrix, distCoeffs;

void loadCamera(const std::string& path) {
    cv::FileStorage fs(path, cv::FileStorage::READ);
    fs["camera_matrix"] >> cameraMatrix;
    fs["dist_coeffs"]   >> distCoeffs;
    fs.release();
}

void rvecToYawPitchFromNormal(const cv::Vec3d& rvec,
                                double& yaw,
                                double& pitch) {
    cv::Mat R;
    cv::Rodrigues(rvec, R);

    double nx = R.at<double>(0,2);
    double ny = R.at<double>(1,2);
    double nz = R.at<double>(2,2);

    yaw = std::atan2(nx, std::abs(nz));

    pitch = std::atan2(-ny, std::sqrt(nx*nx + nz*nz));
}

inline std::array<double, 5> operator-(const std::array<double, 5>& a, const std::array<double, 5>& b) {
    std::array<double, 5> c;
    for (int i=0; i<5; i++) c[i]=a[i]-b[i]; 
    return c;
}

inline std::array<double, 5> operator/(const std::array<double, 5>& a, const double& b) {
    std::array<double, 5> c;
    for (int i=0; i<5; i++) c[i]=a[i]/b; 
    return c;
}

void print_state_table(
    const auto& p,
    const auto& v,
    const auto& a,
    const auto& j
) {
   std::cout << "pos : "; for (double pp: p) std::cout << pp << ", "; std::cout << "\n";
   std::cout << "vel : "; for (double pp: v) std::cout << pp << ", "; std::cout << "\n";
   std::cout << "acc : "; for (double pp: a) std::cout << pp << ", "; std::cout << "\n";
   std::cout << "jerk: "; for (double pp: j) std::cout << pp << ", "; std::cout << "\n";
   std::cout << "\n";
   
}

int main() {
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
    

    Pose target; 
    target.pos = {home_pose.pos[0], TABLE_LENGTH-50._cm, home_pose.pos[2]+50._cm};
    target.ori = {0, 0}; //bounds of both at [-pi/2, pi/2]
    mp.setTarget(target,  Pose0vels);
    Pose initPos = target;

    /* --start code-- */
    waitInput("begin");
    mp.begin(); //idlepos by default once started 

    double markerSizeMeters = 10.5/100.f;
    std::string cam_path =                                                "/dev/video0";
    std::string cam_intrinsics = "/home/connor/PingPongRobot/core/config/vision/video0-intrinsics.yml";
    loadCamera(cam_intrinsics);

    cv::VideoCapture cap(cam_path, cv::CAP_V4L2);
    cap.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M','J','P','G'));
    cap.set(cv::CAP_PROP_FPS, 30);
    if (!cap.isOpened()) return -1;

    auto dict = cv::aruco::getPredefinedDictionary(
        cv::aruco::DICT_5X5_250
    );

    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f>> corners;

    std::vector<Pose> prevposs;
    std::vector<double> times;

    milliseconds lastt;

    while (true){
        cv::Mat frame;
        cap.grab();        // drop frame
        cap.retrieve(frame);
        if (frame.empty()) break;

        cv::aruco::detectMarkers(frame, dict, corners, ids);

        if (!ids.empty()) {
            std::vector<cv::Vec3d> rvecs, tvecs;

            cv::aruco::estimatePoseSingleMarkers(
                corners,
                markerSizeMeters,
                cameraMatrix,
                distCoeffs,
                rvecs,
                tvecs
            );

            for (size_t i = 0; i < ids.size(); i++) {
                cv::aruco::drawAxis(
                    frame,
                    cameraMatrix,
                    distCoeffs,
                    rvecs[i],
                    tvecs[i],
                    markerSizeMeters * 0.5
                );

                Pose pose;
                pose.pos[0] = tvecs[0][0]+TABLE_WIDTH/2;
                pose.pos[1] = (1-tvecs[0][2])+0.25;
                pose.pos[2] = -tvecs[0][1]+0.4; 

                rvecToYawPitchFromNormal(rvecs[0], pose.ori.theta, pose.ori.phi);

                //for (double pos: pose.pos)std::cout << pos << ", ";
                //std::cout <<  "theta: " << pose.ori.theta << "  phi: " << pose.ori.phi << "\r";
                
                double now = std::chrono::duration<double>(
                    std::chrono::steady_clock::now().time_since_epoch()
                ).count();


                if (prevposs.size()==4) prevposs.pop_back();
                prevposs.insert(prevposs.begin(), pose);
                if (times.size()==4) times.pop_back();
                times.insert(times.begin(), now);

                if (prevposs.size()==4) {
                    double dt_v0 = times[0] - times[1];
                    double dt_v1 = times[1] - times[2];
                    double dt_v2 = times[2] - times[3];
                    auto v0 = (prevposs[0].to5vec() - prevposs[1].to5vec())/dt_v0;
                    auto v1 = (prevposs[1].to5vec() - prevposs[2].to5vec())/dt_v1;
                    auto v2 = (prevposs[2].to5vec() - prevposs[3].to5vec())/dt_v2;

                    double dt_a0 = (dt_v0+dt_v1)/2;
                    double dt_a1 = (dt_v1+dt_v2)/2;
                    auto a0 = (v0-v1)/dt_a0;
                    auto a1 = (v1-v2)/dt_a1;

                    double dt_j = (dt_a0+dt_a1)/2;
                    auto j = (a0-a1)/dt_j;

                    print_state_table(prevposs[0].to5vec(), v0, a0, j);
                }

                mp.setTarget(pose, Pose0vels);
            }

        }

        cv::imshow("pose", frame);
        auto k=cv::waitKey(5);
        if (k == 27) break;
    }

    waitInput("home");
    mp.setTarget(home_pose, Pose0vels, 2);
    sleep(3);
    mp.stop();
    return 0;

}