//meant to react to the ball. not terribly usefu but a testing area

#include <iostream>
#include <atomic>

#include "kinematics/inverseK/inverseKin.hpp"
#include "kinematics/motionPather/motionPather.hpp"

#include "vision/ballDet/ballDet.h"
#include "vision/stereo/multiStereo.h"

#include "misc/gaussianBlob.h"
#include "player/kalmanFilter/kalmanFilter.h"
#include <algorithm>

#include "config/config.h"
#include "utils.h"

#include "misc/3dRenderer/3dRenderer.h"

//for program halts
std::atomic<bool> mainLooping(false); void signal_handler(int signal) {if(signal==SIGINT)mainLooping=true;}

int main() {
    /* --instantiate and such-- */

    //kinematics
    auto kinConfig = load_configs(KINCONFIG_PATH);
    std::cout << "kinconfigs loaded\n";

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
        false
    );

    //vision
    Camera camBL("/dev/cam_BL", CONF_PATH + "vision/cam_BL-intrinsics.yml", 1280, 800, 120, 35);
    Camera camBR("/dev/cam_BR", CONF_PATH + "vision/cam_BR-intrinsics.yml", 1280, 800, 120, 35);
    Camera camMR("/dev/cam_MR", CONF_PATH + "vision/cam_MR-intrinsics.yml", 1920, 1080, 120, 300);
    BallDetector balldet(DETCONFIG_PATH, camBL.capName); //need to fix

    TriStereo vis(camBL, camBR, camMR, balldet);

    //manual setup
    doHoming_presetPos(*teensy, kin.doIK(home_pose.pos, home_pose.ori.n(), {0,0,0}, {0,0,0}).qs);

    /* --start code-- */
    waitInput("begin");
    camBL.beginLoop();
    camBR.beginLoop();
    camMR.beginLoop();
    mp.begin();
    viz3d::begin();

    std::signal(SIGINT, signal_handler); 
    //code VV
    
    KalmanModel filter(process_noise, dt); //only use to incorporate measurements

    //main loop
    Pose target;
    Pose targetVels;
    while (mainLooping.load(std::memory_order_relaxed)) {
        filter.predict(); if (filter.state.z < 0) filter.apply_mat(table_bounce_mat);
        GaussBlob<3> measurement; if (vis.getMeasurement(measurement)) 
            filter.combine(measurement);
        
        //player code
        Eigen::Vector6f ball_state = filter.state; //copies it, no auto
        Eigen::Vector3f ball_pos = ball_state.segment(0, 3); Eigen::Vector3f ball_vel = ball_state.segment(3, 3); //'references'
        if (ball_vel(1) < 0){ //ball is coming towards the robot, needs to return it
            //loop through timesteps of the flight path and assign how good of a time to make that shot would be
            //could def do this in a single run, via a more complicated network
            Eigen::Vector3f best_u;
            Eigen::Vector3f best_p;
            float best_T;
            float best_score;
            std::vector<double> scores; //asign values to it
            double dt_forward = dt*2; // timestep accuracy
            for (int i = 0; i<100; i++){
                ball_state = player.dynamics.forward(ball_state, dt_forward);
                if (ball_pos(1) < -1._ft) break; //past back
                if (ball_pos(0) < -0.5_ft || ball_pos(0, 0) < TABLE_WIDTH+ 0.5_ft) break; //past sides

                //figure out best target/return from this angle
                //for now constant
                Eigen::Vector3f shot_target(TABLE_WIDTH/2, 3/2*TABLE_LENGTH, 0);
                float T_return = 1.5; //s

                Eigen::Vector3f v_prime = player.dynamics.backward_P0_T_P1(ball_pos, T_return, shot_target);
                
                Eigen::Vector3f u_vec = player.calc_u(ball_vel, v_prime); //reflection angle+vel

                float score = u_vec.mag(); //rated on velocity of paddle.
                if !(score<best_score) continue;
                //is better
                best_T = i*dt_forward; 
                best_p = ball_pos;
                best_u = u_vec;
            }

            std::copy(best_p.data(), best_p.data() + best_p.size(), target.pos.begin());
            target.ori = Pose::Orientation::from_normal(u_vec.normalized());
            std::copy(best_u.data(), best_u.data() + best_u.size(), targetVels.pos.begin());
            targetVels.ori = {0, 0};

            mp.setTarget(target, targetVels);
        } else { //ball is going away from robot, do something. go idepos?
            mp.setTarget(idle_pose, Pose0vels);
        }

        //viz
        viz3d::begin();

        renderUtils::drawAxes();
        renderUtils::drawTable();
        
        if (mp.getSnapshot().valid) renderUtils::drawPaddle(kin, mp.getSnapshot().position, 
                                                                 mp.getSnapshot().normal, 
                                                                 mp.getSnapshot().velocity);


    } std::cout << "exited \n";

    //code ^^
    waitInput("end"); //cleanup VV
    mp.setTarget(home_pose, Pose0vels, 2.5); sleep(3);
    mp.stop(); //ends mp loop

    viz3d::end();

    camBL.release(); //end cam loops
    camBR.release();
    camMR.release();

    std::cout << "all done! :)\n";
    return 0;
}