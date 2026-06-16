#include "kinematics/inverseK/inverseKin.hpp"
#include "kinematics/motionPather/motionPather.hpp"
#include "config/config.h"
#include "utils.h"
#include <iostream>
#include <string>
#include <filesystem>
#include "vision/camera/camera.h"
#include "vision/stereo/stereoPair.h"

// i want whoever to know that despite this being ai, the code was inititaly written all by me. then after
// getting tired have some llm fill in the little stuff. dis is hard work, and is not viebcoded

using namespace cv;
using namespace std;

struct ClickState {
    Point2f pt;     // in displayed coordinates -- will rescale back
    bool valid = false;
};

ClickState cs1, cs2, cs3;

void onMouse1(int event, int x, int y, int, void*) {
    if (event == EVENT_LBUTTONDOWN) {
        cs1.pt = Point2f(x, y);
        cs1.valid = true;
        cout << "Cam1 updated\n";
    }
}

void onMouse2(int event, int x, int y, int, void*) {
    if (event == EVENT_LBUTTONDOWN) {
        cs2.pt = Point2f(x, y);
        cs2.valid = true;
        cout << "Cam2 updated\n";
    }
}

void onMouse3(int event, int x, int y, int, void*) {
    if (event == EVENT_LBUTTONDOWN) {
        cs3.pt = Point2f(x, y);
        cs3.valid = true;
        cout << "Cam3 updated\n";
    }
}

#define CONTROL_AUTO

int main(int argc, char** argv) {
    auto kinConfig = load_configs(KINCONFIG_PATH);
    cout << "configs loaded\n";

    unique_ptr<MotorController> teensy;
    try {
        teensy = make_unique<MotorController>("/dev/ttyACM0");
    } catch (...) {
        teensy = make_unique<MotorController>("/dev/ttyACM1");
    }

    KinematicsSolver<DOFS> kin(
        kinConfig.pulleyPoss,
        paddle_anchorOffsets,
        pulley_anchorOffsets_refOri
    );

    MotionPather<MotorController, KinematicsSolver<DOFS>> mp(
        kinConfig.control_cycle,
        kinConfig.speeds,
        idle_pose,
        home_pose,
        *teensy,
        kin,
        false
    );

    auto home_qs = kin.doIK(
        home_pose.pos,
        home_pose.ori.n(),
        {0,0,0},
        {0,0,0}
    ).qs;

    doHoming_presetPos(*teensy, home_qs);

    Camera cam_BL("/dev/cam_BL", CONF_PATH + "vision/cam_BL-intrinsics.yml", 1280, 800, 120, 35);
    Camera cam_BR("/dev/cam_BR", CONF_PATH + "vision/cam_BR-intrinsics.yml", 1280, 800, 120, 35);
    Camera cam_MR("/dev/cam_MR", CONF_PATH + "vision/cam_MR-intrinsics.yml", 1920, 1080, 120, 300);

#ifdef CONTROL_AUTO
    vector<array<double,3>> target_pos_list;

    const int ix_s = 5, iy_s = 4, iz_s = 3;
    for (int iz = 0; iz < iz_s; iz++) {
        for (int iy = 0; iy < iy_s; iy++) {
            for (int ix = 0; ix < ix_s; ix++) {
                double fx = (double)ix / (ix_s - 1);
                double fy = (double)iy / (iy_s - 1);
                double fz = (double)iz / (iz_s - 1);

                target_pos_list.push_back({
                    fx * TABLE_WIDTH,
                    fy * (TABLE_LENGTH - 20._cm - 10._cm) + 10._cm,
                    fz * (0.8_m - 25._cm) + 25._cm
                });
            }
        }
    }
#endif

    waitInput("begin");
    mp.begin();

    const float Zoffset = 29.365 / 1000.0f;

    // Pairwise calibration datasets
    vector<Point3f> objectPoints_12;
    vector<Point2f> imagePoints1_12, imagePoints2_12;

    vector<Point3f> objectPoints_23;
    vector<Point2f> imagePoints2_23, imagePoints3_23;

    vector<Point3f> objectPoints_31;
    vector<Point2f> imagePoints3_31, imagePoints1_31;

    // Track what the most recent accept actually added so undo works correctly
    struct AcceptRecord {
        bool added12 = false;
        bool added23 = false;
        bool added31 = false;
    };
    vector<AcceptRecord> acceptHistory;

    Pose target = idle_pose;

#ifdef CONTROL_AUTO
    int i = 0;
    target.pos = target_pos_list[0];
    mp.setTarget(target, Pose0vels);
#endif

    namedWindow("cam_BL");
    namedWindow("cam_BR");
    namedWindow("cam_MR");

    setMouseCallback("cam_BL", onMouse1);
    setMouseCallback("cam_BR", onMouse2);
    setMouseCallback("cam_MR", onMouse3);

    while (true) {
        Mat im1, im2, im3;
        cam_BL.read(im1); if (im1.empty()) break;
        cam_BR.read(im2); if (im2.empty()) break;
        cam_MR.read(im3); if (im3.empty()) break;

        // Display scale: set displayed height to 800 px, preserve aspect ratio
        float s1 = 800.0f / im1.rows;
        float s2 = 800.0f / im2.rows;
        float s3 = 800.0f / im3.rows;

        Mat imcpy1, imcpy2, imcpy3;
        resize(im1, imcpy1, Size(), s1, s1);
        resize(im2, imcpy2, Size(), s2, s2);
        resize(im3, imcpy3, Size(), s3, s3);

        // Draw selected points
        if (cs1.valid) circle(imcpy1, cs1.pt, 4, Scalar(0,255,0), 2);
        if (cs2.valid) circle(imcpy2, cs2.pt, 4, Scalar(0,255,0), 2);
        if (cs3.valid) circle(imcpy3, cs3.pt, 4, Scalar(0,255,0), 2);

        // UI
        putText(imcpy1, "Click to set/update", {20,30}, FONT_HERSHEY_SIMPLEX, 0.6, {255,255,255}, 1);
        putText(imcpy2, "ENTER/y=accept  n=skip  z=undo", {20,30}, FONT_HERSHEY_SIMPLEX, 0.6, {255,255,255}, 1);

        string counts = "12:" + to_string(objectPoints_12.size())
                      + "  23:" + to_string(objectPoints_23.size())
                      + "  31:" + to_string(objectPoints_31.size());
        putText(imcpy3, counts, {20,30}, FONT_HERSHEY_SIMPLEX, 0.6, {255,255,255}, 1);

        imshow("cam_BL", imcpy1);
        imshow("cam_BR", imcpy2);
        imshow("cam_MR", imcpy3);

        int k = waitKey(10);
        if (k == 27) break;

        // ACCEPT
        if (k == 13 || k == 'y') {
            AcceptRecord rec;

            // Rescale clicked points back to original image coordinates
            Point2f p1, p2, p3;
            if (cs1.valid) p1 = Point2f(cs1.pt.x / s1, cs1.pt.y / s1);
            if (cs2.valid) p2 = Point2f(cs2.pt.x / s2, cs2.pt.y / s2);
            if (cs3.valid) p3 = Point2f(cs3.pt.x / s3, cs3.pt.y / s3);

            Point3f obj(
                target.pos[0],
                target.pos[1],
                target.pos[2] - Zoffset
            );

            if (cs1.valid && cs2.valid) {
                imagePoints1_12.push_back(p1);
                imagePoints2_12.push_back(p2);
                objectPoints_12.push_back(obj);
                rec.added12 = true;
                cout << "Captured 12: " << objectPoints_12.size() << "\n";
            }

            if (cs2.valid && cs3.valid) {
                imagePoints2_23.push_back(p2);
                imagePoints3_23.push_back(p3);
                objectPoints_23.push_back(obj);
                rec.added23 = true;
                cout << "Captured 23: " << objectPoints_23.size() << "\n";
            }

            if (cs3.valid && cs1.valid) {
                imagePoints3_31.push_back(p3);
                imagePoints1_31.push_back(p1);
                objectPoints_31.push_back(obj);
                rec.added31 = true;
                cout << "Captured 31: " << objectPoints_31.size() << "\n";
            }

            if (!rec.added12 && !rec.added23 && !rec.added31) {
                cout << "Need at least 2 selected cameras to store a stereo point\n";
            } else {
                acceptHistory.push_back(rec);

#ifdef CONTROL_AUTO
                i = (i + 1) % target_pos_list.size();
                target.pos = target_pos_list[i];
                mp.setTarget(target, Pose0vels);
#endif
            }

            cs1.valid = cs2.valid = cs3.valid = false;
        }

        // SKIP
        if (k == 'n') {
            cs1.valid = cs2.valid = cs3.valid = false;

#ifdef CONTROL_AUTO
            i = (i + 1) % target_pos_list.size();
            target.pos = target_pos_list[i];
            mp.setTarget(target, Pose0vels);
#endif
        }

        // UNDO last accepted capture event
        if (k == 'z') {
            if (!acceptHistory.empty()) {
                AcceptRecord rec = acceptHistory.back();
                acceptHistory.pop_back();

                if (rec.added12) {
                    objectPoints_12.pop_back();
                    imagePoints1_12.pop_back();
                    imagePoints2_12.pop_back();
                }
                if (rec.added23) {
                    objectPoints_23.pop_back();
                    imagePoints2_23.pop_back();
                    imagePoints3_23.pop_back();
                }
                if (rec.added31) {
                    objectPoints_31.pop_back();
                    imagePoints3_31.pop_back();
                    imagePoints1_31.pop_back();
                }

                cout << "Undo."
                     << " 12:" << objectPoints_12.size()
                     << " 23:" << objectPoints_23.size()
                     << " 31:" << objectPoints_31.size()
                     << "\n";
            }
        }
    }

    mp.setTarget(home_pose, Pose0vels);
    sleep(3);

    cout << "calibration point counts:"
         << " 12=" << objectPoints_12.size()
         << " 23=" << objectPoints_23.size()
         << " 31=" << objectPoints_31.size()
         << "\n";

    if (objectPoints_12.size() >= 6) {
        cout << "calibrating BL-BR\n";
        StereoPair::calibrateToFile(
            objectPoints_12,
            imagePoints1_12,
            imagePoints2_12,
            cam_BL, cam_BR,
            CONF_PATH + "vision/camBL-cam_BR-stereoConf.yml"
        );
    } else {
        cout << "Not enough data for BL-BR\n";
    }

    if (objectPoints_23.size() >= 6) {
        cout << "calibrating BR-MR\n";
        StereoPair::calibrateToFile(
            objectPoints_23,
            imagePoints2_23,
            imagePoints3_23,
            cam_BR, cam_MR,
            CONF_PATH + "vision/camBR-cam_MR-stereoConf.yml"
        );
    } else {
        cout << "Not enough data for BR-MR\n";
    }

    if (objectPoints_31.size() >= 6) {
        cout << "calibrating MR-BL\n";
        StereoPair::calibrateToFile(
            objectPoints_31,
            imagePoints3_31,
            imagePoints1_31,
            cam_MR, cam_BL,
            CONF_PATH + "vision/camMR-cam_BL-stereoConf.yml"
        );
    } else {
        cout << "Not enough data for MR-BL\n";
    }

    mp.stop();
    return 0;
}