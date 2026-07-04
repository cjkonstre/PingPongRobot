#include "renderUtils.h"
#include <iostream>


namespace vis3d {
void drawAxes() {
        float axlen = 0.25;
        viz3d::line(0, 0, 0, axlen, 0, 0, 1, 0 ,0);
        viz3d::line(0, 0, 0, 0, 0, axlen, 0, 1 ,0);
        viz3d::line(0, 0, 0, 0, axlen, 0, 0, 0 ,1);
    };

    void drawTable() {
        viz3d::grid(0, 0, TABLE_WIDTH, TABLE_LENGTH, 9, 8);
        viz3d::line(0, 0, TABLE_LENGTH, 0, 0, TABLE_LENGTH*2, 0.3f,0.3f,0.3f);
        viz3d::line(TABLE_WIDTH, 0, TABLE_LENGTH, TABLE_WIDTH, 0, TABLE_LENGTH*2, 0.3f,0.3f,0.3f);
        viz3d::line(0, 0, TABLE_LENGTH*2, TABLE_WIDTH, 0, TABLE_LENGTH*2, 0.3f,0.3f,0.3f);
        viz3d::line(0, 0, TABLE_LENGTH, 0, 0.1525, TABLE_LENGTH, 0.3f,0.3f,0.3f);
        viz3d::line(0, 0.1525, TABLE_LENGTH, TABLE_WIDTH, 0.1525, TABLE_LENGTH, 0.3f,0.3f,0.3f);
        viz3d::line(TABLE_WIDTH, 0, TABLE_LENGTH, TABLE_WIDTH, 0.1525, TABLE_LENGTH, 0.3f,0.3f,0.3f);
    }

    void drawPaddle(const KinematicsSolver<DOFS>& kin, 
                    const std::array<double, 3>& pos, 
                    const std::array<double, 3>& norm, 
                    const std::array<double, 3>& vel){

        Eigen::Vector3d p(pos[0], pos[1], pos[2]);
        Eigen::Vector3d n(norm[0], norm[1], norm[2]);
        auto anchors = kin.getAnchorPoints(p, n);

        constexpr int kOrder[] = {1, 6, 0, 3, 4, 2, 5}; 
        for (int i = 0; i < 7; i++) {
            int a  = kOrder[i];
            int b_ = kOrder[(i + 1) % 7];   // wraps 6→0, closes the loop
            Eigen::Vector3d p0 = anchors.col(a);
            Eigen::Vector3d p1 = anchors.col(b_);
            viz3d::line(p0.x(), p0.z(), p0.y(),
                        p1.x(), p1.z(), p1.y(),
                        0, 0.8, 0);
        }

        const float velscale = 0.1f;
        viz3d::line(pos[0], pos[2], pos[1], pos[0]+vel[0]*velscale, pos[2]+vel[2]*velscale, pos[1]+vel[1]*velscale,
                    0.8, 0, 0);
    }


 void drawGaussblob(const GaussBlob<3>& blob,
                   float k,
                   float r, float g, float b,
                   int rings,
                   int segments) {
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(blob.cov);

    Eigen::Vector3d evals = solver.eigenvalues();
    Eigen::Matrix3d evecs = solver.eigenvectors();

    Eigen::Vector3d radii;
    radii[0] = k * std::sqrt(std::max(0.0, evals[0]));
    radii[1] = k * std::sqrt(std::max(0.0, evals[1]));
    radii[2] = k * std::sqrt(std::max(0.0, evals[2]));

    auto ellipsoidPoint = [&](double x, double y, double z) {
        Eigen::Vector3d p;
        p << x * radii[0], y * radii[1], z * radii[2];

        // World-space point:
        // +x = left
        // +y = forward
        // +z = up
        p = evecs * p + blob.mu;

        return p;
    };

    auto drawLineWorld = [&](const Eigen::Vector3d& p0,
                         const Eigen::Vector3d& p1) {
        viz3d::line(
            (float)p0[0], (float)p0[2], (float)p0[1],
            (float)p1[0], (float)p1[2], (float)p1[1],
            r, g, b
        );
    };

    const float pi = (float)M_PI;

    // latitude rings
    for (int i = 1; i < rings; i++) {
        float v = -0.5f * pi + (float)i * pi / rings;

        float z = std::sin(v);
        float rr = std::cos(v);

        float step = 2.f * pi / segments;

        for (int j = 0; j < segments; j++) {
            float a0 = j * step;
            float a1 = a0 + step;

            Eigen::Vector3d p0 = ellipsoidPoint(rr * std::cos(a0),
                                                rr * std::sin(a0),
                                                z);

            Eigen::Vector3d p1 = ellipsoidPoint(rr * std::cos(a1),
                                                rr * std::sin(a1),
                                                z);

            drawLineWorld(p0, p1);
        }
    }

    // longitude rings
    for (int i = 0; i < rings; i++) {
        float a = (float)i * pi / rings;

        float ca = std::cos(a);
        float sa = std::sin(a);

        float step = 2.f * pi / segments;

        for (int j = 0; j < segments; j++) {
            float v0 = j * step;
            float v1 = v0 + step;

            Eigen::Vector3d p0 = ellipsoidPoint(std::cos(v0) * ca,
                                                std::cos(v0) * sa,
                                                std::sin(v0));

            Eigen::Vector3d p1 = ellipsoidPoint(std::cos(v1) * ca,
                                                std::cos(v1) * sa,
                                                std::sin(v1));

            drawLineWorld(p0, p1);
        }
    }
}

} // namespace vis3d

namespace plot {
void plotTriple(const char* title, const char* yLabel,
                TripleBuf& buf, float tNow, float window) {
    if (ImPlot::BeginPlot(title, ImVec2(-1, 220))) {
        ImPlot::SetupAxes("t [s]", yLabel,
                          ImPlotAxisFlags_NoTickLabels,
                          ImPlotAxisFlags_AutoFit);          // Y auto-fits to data
        ImPlot::SetupAxisLimits(ImAxis_X1, tNow - window, tNow, ImGuiCond_Always);

        auto line = [](const char* name, ScrollBuf& s) {
            if (s.t.empty()) return;
            ImPlot::PlotLine(name, s.t.data(), s.v.data(),
                             (int)s.t.size(), 0, s.offset, sizeof(float));
        };
        line("X", buf.x);
        line("Y", buf.y);
        line("Z", buf.z);
        ImPlot::EndPlot();
    }
}

} // namespace plot