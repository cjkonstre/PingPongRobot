//meant to react to the ball. not terribly usefu but a testing area

#include <iostream>
#include <atomic>
#include <algorithm>

#include "kinematics/inverseK/inverseKin.hpp"
#include "kinematics/motionPather/motionPather.hpp"

#include "vision/ballDet/ballDet.h"
#include "vision/informationSystem/informationSystem.h"
#include "vision/kalmanFilter/kalmanFilter.h"
#include "misc/gaussianBlob.h"

#include "config/config.h"
#include "utils.h"
#include "renderUtils.h"
#include "misc/timeLog/timeLog.hpp"

//for program halts
std::atomic<bool> mainLooping(true); void signal_handler(int signal) {if(signal==SIGINT)mainLooping=false;}
constexpr double g = 9.81;

inline GaussBlob<6> propagate(const GaussBlob<6>& s, double dt)
{
    Eigen::Matrix<double, 6, 6> F = Eigen::Matrix<double, 6, 6>::Identity();
    F.block<3, 3>(0, 3) = dt * Eigen::Matrix3d::Identity();

    GaussBlob<6> out = F * s;        
    out.mu(2) -= g/2 * dt*dt;  
    out.mu(5) -= g * dt;
    return out;
}

inline void bounce(GaussBlob<6>& s)
{
    Eigen::Matrix<double, 6, 6> R = Eigen::Matrix<double, 6, 6>::Identity();
    R(2, 2) = -1.0; R(5, 5) = -1.0;
    s = R * s;
    s.mu(2) = 0.0;
}


void computeFlightPath(const GaussBlob<6>& state,
                       std::array<GaussBlob<6>, 25>& flight_path,
                       double total_time)
{
    const double dt = total_time / 25.0;
    GaussBlob<6> s = state;
    for (int i = 0; i < 25; ++i) {
        flight_path[i] = s;
        s = propagate(s, dt);
        if (s.mu(2) <= 0.0) bounce(s);
    }
}

bool interpolateAtY(const std::array<GaussBlob<6>, 25>& flight_path,
                    double target_y,
                    double total_lookahead_time,
                    GaussBlob<6>& result,
                    double& time_to_point)
{
    constexpr size_t N = 25;
    const double dt = total_lookahead_time / static_cast<double>(N);  // matches computeFlightPath

    for (size_t i = 0; i + 1 < N; ++i) {
        const double y0 = flight_path[i].mu(1);
        const double y1 = flight_path[i + 1].mu(1);

        if ((target_y - y0) * (target_y - y1) > 0.0) continue;

        const double dy = y1 - y0;
        if (std::abs(dy) < 1e-9) return false;

        const double f  = (target_y - y0) / dy;
        const double tf = f * dt;

        result = propagate(flight_path[i], tf);  
        if (result.mu(2) <= 0.0) bounce(result); 
        result.mu(1) = target_y;

        time_to_point = (static_cast<double>(i) + f) * dt;
        return true;
    }
    return false;
}

//might be slow...
static bool fit_state(const std::vector<std::array<double, 6>>& path,
                      double dt,
                      bool& in_air,
                      GaussBlob<6>& state,        //at the NEWEST sample t = 0
                      int n = -1,
                      const double rms_max = 0.03,
                      const double xy_tol  = 4.0,
                      const double g_tol   = 4.0)
{
    if (n == -1) n = static_cast<int>(path.size()); 
    in_air=false;
    state.mu.setZero(); state.cov.setZero();

    if (n < 5) return false;

    Eigen::MatrixXd M(n, 3);
    Eigen::MatrixXd P(n, 3);
    for (int i = 0; i < n; ++i) {
        const double t = -i * dt;
        M(i, 0) = t * t; M(i, 1) = t; M(i, 2) = 1.0;
        P(i, 0) = path[i][0]; P(i, 1) = path[i][1]; P(i, 2) = path[i][2];
    }

    const Eigen::Matrix3d N    = M.transpose() * M;
    const Eigen::Matrix3d Ninv = N.ldlt().solve(Eigen::Matrix3d::Identity());
    const Eigen::Matrix3d coef = Ninv * (M.transpose() * P);  // rows: a,b,c per axis

    const Eigen::MatrixXd resid = P - M * coef;

    const double rms = std::sqrt(resid.squaredNorm() / (n * 3.0));
    const double ax = 2.0 * coef(0, 0), ay = 2.0 * coef(0, 1), az = 2.0 * coef(0, 2);
    in_air = (rms < rms_max) &&
             (std::abs(ax) < xy_tol) && (std::abs(ay) < xy_tol) &&
             (std::abs(az + g) < g_tol);

    state.mu.head<3>() = coef.row(2).transpose();
    state.mu.tail<3>() = coef.row(1).transpose();

    for (int ax_i = 0; ax_i < 3; ++ax_i) {
        const double s2 = resid.col(ax_i).squaredNorm() / (n - 3);
        state.cov(ax_i,     ax_i)     = s2 * Ninv(2, 2);   // σ_x2
        state.cov(ax_i + 3, ax_i + 3) = s2 * Ninv(1, 1);   // σ_v2
        state.cov(ax_i,     ax_i + 3) = s2 * Ninv(1, 2);   // σ_xv
        state.cov(ax_i + 3, ax_i)     = s2 * Ninv(1, 2);
    } return true;
}

//the time it takes for a probability mass to pass through plane p. returns false if fails. 
bool state_transit_time(double& T,
                        const GaussBlob<6>& state,
                         Eigen::Vector3d n, //should be normed, but dont think it matters?
                         double k = 4.0) //k is standard dev perp to the plane, but acts weird bc 3d. 3≈97.1 
{

    Eigen::Matrix<double, 2, 6> P = Eigen::Matrix<double, 2, 6>::Zero();
    P.block<1, 3>(0, 0) = n.transpose(); P.block<1, 3>(1, 3) = n.transpose();
    const GaussBlob<2> s = P * state;

    const double w   = -s.mu(1);      // into-speed  −n·μv  (must be > 0 to cross)
    const double sx2 =  s.cov(0, 0);  // n·Σxx·n
    const double sxv =  s.cov(0, 1);  // n·Σxv·n — invariant under n -> −n, do NOT flip
    const double sv2 =  s.cov(1, 1);  // n·Σvv·n

    const double k2=k*k;
    const double A = w*w - k2*sv2; if (w<= 0.0||A <= 0.0) return false;
    const double B  = -2*k2*sxv;
    const double C  = -k2*sx2;
    const double sq = std::sqrt(B*B - 4.0*A*C);  // ≥ 0 guaranteed

    T = sq / A;
    return true;
}

//viz controls
#define VIS_DO3D
#define VIS_DOCAMS
//#define VIS_DO3DREC 4 //if negative, no record

int main() {
    /* --instantiate and such-- */

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

    //vision
    
    const int exposure = 50;
    Camera camBL("/dev/cam_BL", CONF_PATH + "vision/cam_BL-conf.yml", 1280, 800, 120, exposure);
    Camera camBR("/dev/cam_BR", CONF_PATH + "vision/cam_BR-conf.yml", 1280, 800, 120, exposure);
    Camera camMR("/dev/cam_MR", CONF_PATH + "vision/cam_MR-conf.yml", 1280, 720, 90, exposure*exposure/2); //1920, 1080  -- calib was done @ 720p, fix this

    float dt = 0.01; //secs. doesnt make a ton of sense to go below 10ms
    KalmanFilter kf(dt);

    BallDetector bdet_BL(DETCONFIG_PATH, camBL.capName, kf); 
    BallDetector bdet_BR(DETCONFIG_PATH, camBR.capName, kf); 
    BallDetector bdet_MR(DETCONFIG_PATH, camMR.capName, kf); 
    
    InformationSystem<3> vision({&camBL, &camBR, &camMR}, 
                                {1., 1., 2.}); //px error in det

    //manual setup
    std::array<double, DOFS> home_qs = kin.doIK(home_pose.pos, home_pose.ori.n(), {0,0,0}, {0,0,0}).qs;
    doHoming_presetPos(*teensy, home_qs);


    /* --start code-- */
    waitInput("begin");
    camBL.beginLoop(); bdet_BL.beginLoop(camBL);
    camBR.beginLoop(); bdet_BR.beginLoop(camBR);
    camMR.beginLoop(); bdet_MR.beginLoop(camMR);

    mp.setTarget(Pose{{home_pose.pos[0], TABLE_LENGTH-50._cm, home_pose.pos[2]+50._cm}, ORI_sp_FORWARD}, Pose0vels, 2);
    mp.begin();

    std::signal(SIGINT, signal_handler); 
    //code VV
    
    BallDetection det_BL, det_BR, det_MR;
    GaussBlob<3> measurement;

    //ball future flight path points
    std::array<std::array<double, 6>, 25> flight_path;

    std::vector<std::array<double, 6>> prev_path(25);
    std::vector<bool> in_airs(25);

    //* visuals / data stuff */
    //disp thread to show dets.
    std::atomic<bool> display_running{true};
    std::thread display_thread([&]{
        #ifdef VIS_DO3D
        viz3d::init(1920, 1080, "pos");
        #ifdef VIS_DO3DREC
        viz3d::startRec("/home/connor/PingPongRobot/docs/3dballtrack_"+std::to_string(VIS_DO3DREC)+".mp4", 45.0);
        #endif
        #endif

        while(display_running){
            #ifdef VIS_DOCAMS
            auto im1 = camBL.frame_buffer.back().image.clone();
            if (!im1.empty()){
            if (det_BL.found) cv::circle(im1, det_BL.center, 5, cv::Scalar(0, 255, 0), -1);
            cv::imshow("cam_BL", im1);}

            auto im2 = camBR.frame_buffer.back().image.clone();
            if (!im2.empty()){
            if (det_BR.found) cv::circle(im2, det_BR.center, 5, cv::Scalar(0, 255, 0), -1);
            cv::imshow("cam_BR", im2);}

            auto im3 = camMR.frame_buffer.back().image.clone();
            if (!im3.empty()){
            if (det_MR.found) cv::circle(im3, det_MR.center, 5, cv::Scalar(0, 255, 0), -1);
            cv::imshow("cam_MR", im3);}

            cv::waitKey(1);
            #endif
            #ifdef VIS_DO3D
            vis3d::drawGaussblob(GaussBlob<3>{kf.state().cov.topLeftCorner<3,3>(), 
                                             kf.state().mu.head<3>()}, 3.5, 1.f, 0.f, 0.f);
            viz3d::begin();
            vis3d::drawAxes();
            vis3d::drawTable();

            //viz3d::sphere(measurement.mu[0], measurement.mu[2], measurement.mu[1], 2._cm, 0.75f, 0.25f, 0., 
            //                6, 24);
            vis3d::drawGaussblob(measurement, 3.5, 1.f, 1.f, 1.f, 
                                8, 12);
            vis3d::drawGaussblob(GaussBlob<3>{kf.state().cov.topLeftCorner<3,3>(), 
                                             kf.state().mu.head<3>()}, 3.5, 1.f, 0.f, 0.f, 
                                             8, 12);

            for (int i=1; i<flight_path.size(); i++) {
                viz3d::line(flight_path[i-1][0], flight_path[i-1][2], flight_path[i-1][1],
                            flight_path[i][0], flight_path[i][2], flight_path[i][1], 0.75f, 0.25f, 0.);
            }

            for (int i=1; i<prev_path.size(); i++) {
                viz3d::line(prev_path[i-1][0], prev_path[i-1][2], prev_path[i-1][1],
                            prev_path[i][0], prev_path[i][2], prev_path[i][1], in_airs[i]?0.25f:0.75f, in_airs[i]?0.75f:0.25f, 0.);
            }

            auto snap = mp.getSnapshot();
            vis3d::drawPaddle(kin, snap.position, snap.normal, snap.velocity);

            viz3d::end();
            #endif
        }
    });
    uint64_t prevts_BLD = 0, prevts_BRD = 0, prevts_MRD = 0; //dets, get detspeed
    int newDets = 0;
    //* ^^ visuals / data stuff ^^ */

    bool in_air;
    Eigen::Vector3d vel;

    float lookahead_time = 1.5; //s 
    float latency = 0.5; //s, latency in reaction times. rn thsi si due to poor calibration and some other stuffs

    uint64_t cycleCount = 0; //for timekeeping
    auto next_tick = std::chrono::steady_clock::now();
    const auto waittime = std::chrono::duration_cast<std::chrono::steady_clock::duration>(std::chrono::duration<double>(dt));
    while (mainLooping.load(std::memory_order_relaxed)) { cycleCount++;
        //TIMELOGDT << "start\n";
        next_tick += waittime;

        bdet_BL.latestDetection(det_BL);
        bdet_BR.latestDetection(det_BR);
        bdet_MR.latestDetection(det_MR);
        //only grabs ^^

        uint64_t ts; //mayhem right here. golf is fun.
        //if new update, increment if found
        ts = det_BL.timestamp_us; newDets+=ts!=prevts_BLD?prevts_BLD=ts,det_BL.found:0;
        ts = det_BR.timestamp_us; newDets+=ts!=prevts_BRD?prevts_BRD=ts,det_BR.found:0;
        ts = det_MR.timestamp_us; newDets+=ts!=prevts_MRD?prevts_MRD=ts,det_MR.found:0;

        if (cycleCount%(int)(0.5/dt) == 0) {
            std::cout << "~F/s: " << camBL.getFPS() <<", "<<camBR.getFPS()<<", "<<camMR.getFPS()<<
            " | ~D/s: " << newDets*1/(0.5)<< "\n";
            newDets = 0;
        }
        //TIMELOGDT << "kinda real start\n";        

        //TIMELOGDT << "measurements combined\n";                                        
        kf.predict(); 
        kf.update(measurement); //no rejection
        //TIMELOGDT << "kf updated\n";

        prev_path.pop_back();
        std::array<double, 6> pos; std::copy(kf.state().mu.data(), kf.state().mu.data() + 6, pos.begin());
        prev_path.insert(prev_path.begin(), pos);

        fit_velocity(prev_path, dt, in_air, vel, 15);

        in_airs.pop_back(); in_airs.insert(in_airs.begin(), in_air);

        //TIMELOGDT << "vel fitted\n";
        computeFlightPath(kf.state().mu, vel, flight_path, lookahead_time);
        //TIMELOGDT << "flight pathed\n";


        double targetY = TABLE_LENGTH/2-20._cm;
        std::array<double, 6> target; double time2target;
        bool ret = interpolateAtY(flight_path, targetY, lookahead_time, target, time2target);
        if (ret && (target[0]>20._cm && target[0]<TABLE_WIDTH-20._cm) && (target[2]>20._cm && target[2] < 0.7)) {
            Pose target_pose = Pose{{target[0], target[1], target[2]}, ORI_sp_FORWARD};
            Pose target_vel  = Pose{{0, 0.5, 0}, ORI_sp_FORWARD};



            mp.setTarget(target_pose, target_vel, time2target-latency);
        }
        //TIMELOGDT << "done\n";

        //viz is being run in a diff thread

        if (std::chrono::duration_cast<std::chrono::microseconds>(next_tick-std::chrono::steady_clock::now()).count()<0)
            std::cout << "OVERTIME\n";
        std::this_thread::sleep_until(next_tick); //may not be accurate enough, well see
    } std::cout << "exited \n";

    /* --end code-- */

    camBL.release(); bdet_BL.endLoop();
    camBR.release(); bdet_BR.endLoop();
    camMR.release(); bdet_MR.endLoop();

    waitInput("home");
    mp.setTarget(home_pose, Pose0vels, 2); sleep(3);
    mp.stop();

    display_running = false; if(display_thread.joinable()) display_thread.join();
    viz3d::end();

    std::cout << "all done! :)\n";
    return 0;
}