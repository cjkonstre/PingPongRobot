#include "vision/ballDet/ballDet.h"
#include "kinematics/inverseK/inverseKin.hpp"
#include "kinematics/motionPather/motionPather.hpp"
#include "config/config.h"
#include "utils.h"
#include <iostream>
#include <string>
#include <filesystem>
#include "vision/camera/camera.h"

using namespace cv;
using namespace std;

int main(int argc, char** argv) {
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
    //kin done ^^


    string cam1Name = filesystem::path(argv[1]).filename();
    string cam2Name = filesystem::path(argv[2]).filename();
    string cam1intrinsics = "/home/connor/PingPongRobot/core/config/vision/" + cam1Name + "-intrinsics.yml";
    string cam2intrinsics = "/home/connor/PingPongRobot/core/config/vision/" + cam2Name + "-intrinsics.yml";

    Camera cam1(argv[1], cam1intrinsics, 1280, 800, 120, 40);
    Camera cam2(argv[2], cam1intrinsics, 1280, 800, 120, 40);

    BallDetector balldet1(DETCONFIG_PATH, cam1Name);
    BallDetector balldet2(DETCONFIG_PATH, cam2Name);


    /* --start code-- */
    waitInput("begin");

    mp.begin(); //idlepos by default once started 

    vector<Point2f> cap1points;
    vector<Point2f> cap2points;
    vector<Point3f> IRLpoints;
    Size imsize;

    Pose target = idle_pose;
    while (true) {
        Mat im1, imcpy1, im2, imcpy2;
        /*cap1.grab();
        cap2.grab();
        cap1.retrieve(im1); if (im1.empty()) break; im1.copyTo(imcpy1);
        cap2.retrieve(im2); if (im2.empty()) break; im2.copyTo(imcpy2);
        imsize = im1.size();*/

        cam1.read(im1); if (im1.empty()) break; im1.copyTo(imcpy1);
        cam2.read(im2); if (im2.empty()) break; im2.copyTo(imcpy2);


        Point2f ppxpos1, ppxpos2; float rad;

        bool ret = balldet1.findBall(im1, ppxpos1, rad);
        if (ret) circle(imcpy1, ppxpos1, (int)rad, Scalar(0, 255, 0), 5);
        imshow(cam1Name, imcpy1);

        ret = balldet2.findBall(im2, ppxpos2, rad);
        if (ret) circle(imcpy2, ppxpos2, (int)rad, Scalar(0, 255, 0), 5);
        imshow(cam2Name, imcpy2);

        auto k = waitKey(10); if (k == 27) break; //esc

        //interactive target control
        //would be cool to make an interactive target control object or some easier interface class
        //could live in player
        const double mvspeed = 5._cm; 
        if      (k=='w') target.pos[1] += mvspeed;
        else if (k=='s') target.pos[1] -= mvspeed;
        else if (k=='a') target.pos[0] += mvspeed;
        else if (k=='d') target.pos[0] -= mvspeed;
        else if (k=='q') target.pos[2] += mvspeed;
        else if (k=='e') target.pos[2] -= mvspeed;
        mp.setTarget(target, Pose0vels);

        if (k=='c') {
            cap1points.push_back(ppxpos1);
            cap2points.push_back(ppxpos2);
            IRLpoints.push_back(Point3f(target.pos[0], 
                                        target.pos[1],
                                        target.pos[2]));
            cout << "captured!\n";
        }
    }

    if (IRLpoints.size()>5) {

    std::cout << "calinrating...";
    cv::Mat R, T, E, F;
    double rms = cv::stereoCalibrate(
        IRLpoints,
        cap1points,
        cap2points,
        cam1.intrinsics.K, cam1.intrinsics.D,
        cam2.intrinsics.K, cam2.intrinsics.D,
        imsize, //idk, set to 1st cam
        R, T, E, F,
        cv::CALIB_FIX_INTRINSIC,
        cv::TermCriteria(
            cv::TermCriteria::COUNT + cv::TermCriteria::EPS,
            100,
            1e-6
        )
    );

    std::cout << "rms: " << rms << "\n";
    cv::Mat rvec; cv::Rodrigues(R, rvec); double rotation_deg = cv::norm(rvec) * 180.0 / CV_PI;
    std::cout << "distance: " << cv::norm(T) << " (same units as objectPoints)\n";
    std::cout << "vector T: [" 
              << T.at<double>(0) << ", "
              << T.at<double>(1) << ", "
              << T.at<double>(2) << "]\n";
    std::cout << "relative rotation: " << rotation_deg << " deg\n";

    //should wrap this up into a stereo object. make the names a bit easier
    string filename = "/home/connor/PingPongRobot/core/config/vision/" + cam1Name + "+" + cam2Name + "-stereoExtrinsics";
    FileStorage fs(filename, FileStorage::WRITE);
    fs << "R" << R;
    fs << "T" << T;
    fs << "E" << E;
    fs << "F" << F;
    fs << "rms" << rms;
    fs.release();
    cout << "saved calibration to " << filename << endl;

    } else std::cout << "not enough data\n";

    waitInput("home");
    mp.setTarget(home_pose, Pose0vels);
    sleep(3);

    mp.stop();
}