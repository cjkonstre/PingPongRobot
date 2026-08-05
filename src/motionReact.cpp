//tracks hand or something and has the robo copy it


#include <iostream>

#include "kinematics/inverseK/inverseKin.hpp"
#include "kinematics/motionPather/motionPather.hpp"
#include "config/config.h"
#include "utils.h"
#include "opencv2/opencv.hpp"
#include <opencv2/aruco.hpp>
#include <chrono>

#include "renderUtils.h"



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

#define DO_VIS_REC
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
    mp.setTarget(target,  Pose0vels, 5);
    Pose initPos = target;
    
    /* --start code-- */
    waitInput("begin");
    mp.begin(); //idlepos by default once started 
    viz3d::init(1280, 720, "pos");
    viz3d::startRec("/home/connor/PingPongRobot/docs/motionReact_01.mp4", 60.0);

    double markerSizeMeters = 10.5/100.f;
    std::string cam_path =                                                "/dev/video2";
    std::string cam_intrinsics = "/home/connor/PingPongRobot/core/config/vision/video2-conf.yml";
    loadCamera(cam_intrinsics);

    cv::VideoCapture cap(cam_path, cv::CAP_V4L2);
    cap.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M','J','P','G'));
    cap.set(cv::CAP_PROP_FRAME_WIDTH, 1280);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, 800);
    cap.set(cv::CAP_PROP_FPS, 120);
    if (!cap.isOpened()) return -1;

    auto dict = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);

    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f>> corners;

    plot::TripleBuf posBuf, velBuf, accBuf;
    auto plotT0 = std::chrono::steady_clock::now();
    const float plotWindow = 10.0f;   // seconds of history visible

    while (true){
        cv::Mat frame;
        cap.grab();        // drop frame
        cap.retrieve(frame);
        if (frame.empty()) continue;

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

                mp.setTarget(pose, Pose0vels);
            }

        }


        cv::imshow("pose", frame);
        auto k=cv::waitKey(5);
        if (k == 27) break;

        viz3d::begin();
        vis3d::drawAxes();
        vis3d::drawTable();

        if (mp.getSnapshot().valid) {
            auto snap = mp.getSnapshot();
            vis3d::drawPaddle(kin, snap.position, snap.normal, snap.velocity);

            float tNow = std::chrono::duration<float>(
                            std::chrono::steady_clock::now() - plotT0).count();
            posBuf.add(tNow, snap.position);
            velBuf.add(tNow, snap.velocity);
            accBuf.add(tNow, snap.acceleration);

            ImGui::Begin("Motion");
            plot::plotTriple("Position",     "m",     posBuf, tNow, plotWindow);
            plot::plotTriple("Velocity",     "m/s",   velBuf, tNow, plotWindow);
            plot::plotTriple("Acceleration", "m/s^2", accBuf, tNow, plotWindow);
            ImGui::End();
        }

        viz3d::end();
    }

    cv::destroyAllWindows();

    waitInput("home");
    mp.setTarget(home_pose, Pose0vels, 2);
    sleep(3);
    mp.stop();
    viz3d::end();
    return 0;

}