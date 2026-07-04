#pragma once

#include <array>
#include "implot.h"
#include "kinematics/inverseK/inverseKin.hpp"
#include "misc/3dRenderer/3dRenderer.h"
#include "misc/gaussianBlob.h"
#include "config/config.h"

//as opposd to viz3d, the lib. ik its a disaster
namespace vis3d {
    void drawAxes();
    void drawTable();
    void drawPaddle(const KinematicsSolver<DOFS>& kin,
                    const std::array<double, 3>& pos,
                    const std::array<double, 3>& norm,
                    const std::array<double, 3>& vel);
    void drawGaussblob(const GaussBlob<3>& blob,
                            float k,
                            float r, float g, float b,
                            int rings = 12,
                            int segments = 64);
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