#include "utils.h"
#include <iostream>
#include "misc/3dRenderer/3dRenderer.h"

void doHoming_presetPos(MotorController& controller, const std::array<double, DOFS> presetQs) {
    for (int i=0; i<DOFS; i++) {controller.sendCommand(MOTOR_DISABLE, i);} //disable all motors
    waitInput("Move axes till taught. Enter to continue\n");
    for (int i=0; i<DOFS; i++) {controller.sendCommand(MOTOR_ENABLE, i);} //enable all motors

    controller.sendCommand(MOTOR_SETSTP, 0, presetQs);
}

int waitInput(const char* message) {
    std::cout << message;
    return std::cin.get();
}

int waitInput() {
    std::cout << "Enter to Continue...";
    return std::cin.get();
}

std::array<double, 3> randVector(std::array<double, 3> mins, std::array<double, 3> maxs) {
    std::array<double, 3> vect;
    for (int i=0; i<3; i++){
        vect[i] = mins[i] + static_cast <float> (rand()) /( static_cast <float> (RAND_MAX/(maxs[i]-mins[i])));
    }
    return vect;
}

//may be at fault
void synchCamrecsToNow(CameraRec& cam1, CameraRec& cam2, CameraRec& cam3){
    cam1.tsReader >> cam1.first_ts;
    cam2.tsReader >> cam2.first_ts;
    cam3.tsReader >> cam3.first_ts;
    uint64_t earlieststart = std::min({cam1.first_ts,  cam2.first_ts, cam3.first_ts});

    cam1.offset = cam1.first_ts - earlieststart;
    cam2.offset = cam2.first_ts - earlieststart;
    cam3.offset = cam3.first_ts - earlieststart;

    auto global_start_time = std::chrono::steady_clock::now();
    cam1.start_time = global_start_time;
    cam2.start_time = global_start_time;
    cam3.start_time = global_start_time;
}


namespace renderUtils {

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
} // namespace renderUtils