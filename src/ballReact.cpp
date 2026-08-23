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

constexpr float g = 9.81f;
void computeFlightPath(
    const Eigen::Matrix<double, 6, 1>& mu,
    const Eigen::Vector3d& vel_in,
    std::array<std::array<double, 6>, 25>& flight_path,
    double total_time)
{
    Eigen::Vector3d pos = mu.head<3>();
    Eigen::Vector3d vel = vel_in;

    const double dt = total_time / 25.0;

    Eigen::Vector3d acc(0.0, 0.0, -g);

    for (int i = 0; i < 25; i++)
    {
        flight_path[i][0] = pos.x();
        flight_path[i][1] = pos.y();
        flight_path[i][2] = pos.z();

        flight_path[i][3] = vel.x();
        flight_path[i][4] = vel.y();
        flight_path[i][5] = vel.z();

        vel += acc * dt;
        pos += vel * dt;

        if (pos[2] <= 0) { vel[2] *= -1; pos[2] = 0; }
    }
}

bool interpolateAtY(
    const std::array<std::array<double, 6>, 25>& flight_path,
    double target_y,
    double total_lookahead_time,
    std::array<double, 6>& result,
    double& time_to_point)
{
    constexpr size_t N = 25;
    const double dt = total_lookahead_time / (N - 1);

    for (size_t i = 0; i + 1 < N; ++i)
    {
        const auto& a = flight_path[i];
        const auto& b = flight_path[i + 1];

        const double y0 = a[1];
        const double y1 = b[1];

        // Does this segment cross the requested y?
        if ((target_y >= y0 && target_y <= y1) ||
            (target_y >= y1 && target_y <= y0))
        {
            const double dy = y1 - y0;

            // Degenerate segment
            if (std::abs(dy) < 1e-9)
                return false;

            // Fraction through this segment
            const double t = (target_y - y0) / dy;

            // Interpolate state
            for (int j = 0; j < 6; ++j)
                result[j] = a[j] + t * (b[j] - a[j]);

            result[1] = target_y;

            // Time from the start of the trajectory
            time_to_point = (static_cast<double>(i) + t) * dt;

            return true;
        }
    }

    // Never reached target_y within the simulated trajectory.
    return false;
}

// chi-squared, 3 DOF
constexpr double GATE_95  =  7.81; //tight. 95% of actual measurements get spassed
constexpr double GATE_99  = 11.34;
constexpr double GATE_997 = 14.16;   // ~"3-sigma" tail mass -- permissize
constexpr double GATE_m1en2 = 21.11; //99.99
constexpr double GATE_m1en3 = 25.90; //99.999
constexpr double GATE_m1en4 = 30.67; //99.9999
static double mahalanobis2(const GaussBlob<6>& state, const GaussBlob<3>& meas)
{
    const Eigen::Vector3d y = meas.mu - state.mu.head<3>();     // innovation
    const Eigen::Matrix3d S = state.cov.topLeftCorner<3,3>()    // filter's pos cov
                            + meas.cov;                         // measurement cov
    return y.transpose() * S.ldlt().solve(y);                  // squared distance
}

//fits least squares
static bool fit_velocity(const std::vector<std::array<double,6>>& path,
                         double dt,
                         bool& in_air,
                         Eigen::Vector3d& vel, 
                         int n = -1, // -1 if use entire vec, 
                         const double rms_max = 0.03, 
                         const double xy_tol=4.,
                         const double g_tol = 4)
{
    if (n==-1) int n=path.size();
    in_air = false;
    vel.setZero();

    if (n < 4) return false;                 // need >3 points for a real quadratic

    Eigen::MatrixXd M(n, 3);
    Eigen::MatrixXd P(n, 3);                  // columns: x, y, z

    for (int i = 0; i < n; ++i) {
        const double t = -i * dt;           // i=0 newest -> t=0
        M(i,0) = t*t; M(i,1) = t; M(i,2) = 1.0;
        P(i,0) = path[i][0];
        P(i,1) = path[i][1];
        P(i,2) = path[i][2];
    }

    Eigen::Matrix3d N   = M.transpose() * M;
    Eigen::Matrix<double,3,3> coef = N.ldlt().solve(M.transpose() * P); 

    const Eigen::MatrixXd resid = P - M * coef;
    const double rms = std::sqrt(resid.squaredNorm() / (n * 3));

    const double ax = 2.0 * coef(0,0); const double ay = 2.0 * coef(0,1); const double az = 2.0 * coef(0,2);

    vel = Eigen::Vector3d(coef(1,0), coef(1,1), coef(1,2));


    const bool horiz_ok = std::abs(ax) < xy_tol &&
                          std::abs(ay) < xy_tol;
    const bool grav_ok  = std::abs(az + -9.81) < g_tol;   // az ~ -9.81
    const bool fit_ok   = rms < rms_max;

    in_air = fit_ok && horiz_ok && grav_ok;
    return true;
}

//viz controls
#define VIS_DO3D
#define VIS_DOCAMS
//#define VIS_DO3DREC 4 //if negative, no record

int main() {
    cv::setNumThreads(1);//TEMP
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
    std::array<std::array<double, 6>, 25> flight_path_mu;

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

            for (int i=1; i<flight_path_mu.size(); i++) {
                viz3d::line(flight_path_mu[i-1][0], flight_path_mu[i-1][2], flight_path_mu[i-1][1],
                            flight_path_mu[i][0], flight_path_mu[i][2], flight_path_mu[i][1], 0.75f, 0.25f, 0.75);
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

        measurement = vision.combineMeasurements({det_BL.center, det_BR.center, det_MR.center}, 
                                                 {det_BL.found,  det_BR.found,  det_MR.found});

        //TIMELOGDT << "measurements combined\n";                                        
        kf.predict(); 
        //if (mahalanobis2(kf.state(), measurement) < GATE_m1en3) 
            kf.update(measurement); //rejection
        //TIMELOGDT << "kf updated\n";

        prev_path.pop_back();
        std::array<double, 6> pos; std::copy(kf.state().mu.data(), kf.state().mu.data() + 6, pos.begin());
        prev_path.insert(prev_path.begin(), pos);

        fit_velocity(prev_path, dt, in_air, vel, 15);

        in_airs.pop_back(); in_airs.insert(in_airs.begin(), in_air);

        //TIMELOGDT << "vel fitted\n";
        computeFlightPath(kf.state().mu, vel, flight_path, lookahead_time);
        computeFlightPath(kf.state().mu, kf.state().mu.tail(3), flight_path_mu, lookahead_time);
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