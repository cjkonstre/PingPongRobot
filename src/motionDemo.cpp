#include <boost/asio.hpp>
#include <iostream>

#include "kinematics/inverseK/inverseKin.hpp"
#include "measurements/dimensions.h"
#include "config.h" //where a lot of type aliases live
#include "utils.h"

static inline std::array<double,3> lerp_array(const std::array<double,3>& a,
                                              const std::array<double,3>& b,
                                              double t) {
    return {
        a[0] + (b[0] - a[0]) * t,
        a[1] + (b[1] - a[1]) * t,
        a[2] + (b[2] - a[2]) * t
    };
}

// Direction vector normalized
static inline std::array<double,3> direction_vec(const std::array<double,3>& from,
                                                 const std::array<double,3>& to) {
    std::array<double,3> dir = { to[0]-from[0], to[1]-from[1], to[2]-from[2] };
    double norm = std::sqrt(dir[0]*dir[0] + dir[1]*dir[1] + dir[2]*dir[2]);
    if (norm > 1e-9)
        for (double& v : dir) v /= norm;
    return dir;
}

// Main function
void sendInterp(MotorController& mc,
                const KinematicsSolver<DOFS>& kin,
                const std::array<double,3>& cur_pos, const std::array<double,3>& cur_ori,
                const std::array<double,3>& target_pos, const std::array<double,3>& target_ori,
                double totalTime, int steps)
{
    Packet packet;
    packet.packetId = 1; // assign non-zero id

    const double dt = totalTime / steps;
    const std::array<double,3> travel_dir = direction_vec(cur_pos, target_pos);
    const double total_dist = std::sqrt(
        std::pow(target_pos[0]-cur_pos[0],2) +
        std::pow(target_pos[1]-cur_pos[1],2) +
        std::pow(target_pos[2]-cur_pos[2],2)
    );
    const double avg_speed = total_dist / totalTime; // scalar speed

    int frame_in_packet = 0;

    for (int i = 0; i < steps; ++i) {
        const double alpha = static_cast<double>(i+1) / steps;

        // interpolate position and orientation
        std::array<double,3> interp_pos = lerp_array(cur_pos, target_pos, alpha);
        std::array<double,3> interp_ori = lerp_array(cur_ori, target_ori, alpha);

        // velocity: directed motion, zero at last frame
        std::array<double,3> vel = {0,0,0};
        if (i < steps - 1) {
            for (int j=0; j<3; ++j)
                vel[j] = travel_dir[j] * avg_speed;
        }

        MotionStateD<DOFS> ikResult = kin.doIK(interp_pos, interp_ori, vel);

        Frame frame;
        frame.dt       = dt;
        frame.q_new    = ikResult.qs;
        frame.dq_new   = ikResult.dqs;
        frame.frameIdx = i;

        // add frame to packet
        packet.frames[frame_in_packet++] = frame;

        // send packet when full
        if (frame_in_packet == 5) {
            mc.sendPacket(packet);
            frame_in_packet = 0;
            packet = Packet(); // reset packet
            packet.packetId = 1;
        }
    }

    // send remaining frames (if any)
    if (frame_in_packet > 0) {
        packet.packetLength = frame_in_packet;
        mc.sendPacket(packet);
    }
}

int main() { 
    //instantiate and such
    std::unique_ptr<MotorController> teensy;
    try {
        std::cout << "Trying connection at /dev/ttyACM0...\n";
        teensy = std::make_unique<MotorController>("/dev/ttyACM0");
    } catch (const boost::wrapexcept<boost::system::system_error>&) {
        std::cout << "Trying connection at /dev/ttyACM1...\n";
        teensy = std::make_unique<MotorController>("/dev/ttyACM1");
    }
    std::cout << "Connected\n";

    KinematicsSolver<DOFS> kin = make_kinSolver();
    std::array<double, DOFS> home_qs = kin.doIK(home_pos, home_ori, {0,0,0}).qs;

    doHoming_presetPos(*teensy, home_qs, 0._mm);

    std::array<double, 3> Tpos;
    std::array<double, 3> Tori = {0, 1, 0};
    std::array<double, 3> Cpos = home_pos;
    std::array<double, 3> Cori = home_ori;

    double T = 5;
    int interpN=6;

    double height_max = 0.7;
    double height_min = PADDLE_HEIGHT/2 + 4._mm;

    for (int j=0; j<10; j++) {
        Tpos[0] = static_cast <float> (rand()) / (static_cast <float> (RAND_MAX/TABLE_WIDTH));
        Tpos[1] = static_cast <float> (rand()) / (static_cast <float> (RAND_MAX/TABLE_LENGTH));
        Tpos[2] = height_min + static_cast <float> (rand()) /( static_cast <float> (RAND_MAX/(height_max-height_min)));\

        sendInterp(*teensy, kin,
                Cpos, Cori,
                Tpos, Tori,
                T, interpN);
        sleep(T);
        Cpos=Tpos;
        Cori=Tori;
        //int in;
        //std::cin>>in;
        //if (in==1) {break;}
    }

    /*//right side of table
    Tpos={TABLE_WIDTH, TABLE_LENGTH-50._cm, 50._cm};
    Tori={0,1,0};
    sendInterp(*teensy, kin,
                Cpos, Cori,
                Tpos, Tori,
                T, interpN);
    sleep(T);
    Cpos=Tpos;
    Cori=Tori;
    waitInput();


    //;eft side of table
    Tpos={0, TABLE_LENGTH-50._cm, 50._cm};
    Tori={0,1,0};
    sendInterp(*teensy, kin,
                Cpos, Cori,
                Tpos, Tori,
                T, interpN);
    sleep(T);
    Cpos=Tpos;
    Cori=Tori;
    waitInput();*/

    //home
    Tpos=home_pos;
    Tori=home_ori;
    sendInterp(*teensy, kin,
                Cpos, Cori,
                Tpos, Tori,
                T, interpN);
    sleep(T);
    Cpos=Tpos;
    Cori=Tori;

    return 0;
}