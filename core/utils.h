#pragma once

#include "config/config.h"
#include "kinematics/inverseK/inverseKin.hpp"
#include <array>
#include <iostream>
#include "vision/camera/camera.h"

#include "implot.h"


int waitInput(const char* message);
int waitInput();

void doHoming_presetPos(MotorController& controller, const std::array<double, DOFS> presetQs);

std::array<double, 3> randVector(std::array<double, 3> mins, std::array<double, 3> maxs);

constexpr Pose Pose0vels{{0, 0, 0}, {0, 0}};


void synchCamrecsToNow(CameraRec& cam1, CameraRec& cam2, CameraRec& cam3);

// this is mainly the 3d vis, but also just general visualization
namespace renderUtils {

//as opposd to viz3d, the lib. ik its a disaster
namespace vis3d {
    void drawAxes();
    void drawTable();
    void drawPaddle(const KinematicsSolver<DOFS>& kin,
                    const std::array<double, 3>& pos,
                    const std::array<double, 3>& norm,
                    const std::array<double, 3>& vel);
} // namespace vis3d

// ---- 2D time-series plotting ----
namespace plot {

// Fixed-size scrolling buffer for one scalar series.
struct ScrollBuf {
    int maxSize;
    int offset = 0;
    std::vector<float> t;   // timestamps
    std::vector<float> v;   // values

    ScrollBuf(int max = 4000) : maxSize(max) {
        t.reserve(maxSize);
        v.reserve(maxSize);
    }

    void add(float ts, float val) {
        if ((int)t.size() < maxSize) {
            t.push_back(ts);
            v.push_back(val);
        } else {
            t[offset] = ts;
            v[offset] = val;
            offset = (offset + 1) % maxSize;
        }
    }
};

// Holds X/Y/Z buffers for one quantity (pos / vel / acc).
struct TripleBuf {
    ScrollBuf x, y, z;
    TripleBuf(int max = 4000) : x(max), y(max), z(max) {}

    void add(float ts, const std::array<double, 3>& a) {
        x.add(ts, (float)a[0]);
        y.add(ts, (float)a[1]);
        z.add(ts, (float)a[2]);
    }
};

void plotTriple(const char* title, const char* yLabel,
                TripleBuf& buf, float tNow, float window);

} // namespace plot
} // namespace renderUtils