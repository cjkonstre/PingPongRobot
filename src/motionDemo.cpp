#include <boost/asio.hpp>
#include <iostream>

#include "kinematics/inverseK/inverseKin.hpp"
#include "core/measurements/dimensions.h"
#include "core/config.h"
#include "core/utils.h"

Packet actionPacket(Frame action) {
    Packet packet;
    packet.frames[0] = action; 
    packet.packetLength=1;
    return packet;
}

int main() {
    std::unique_ptr<MotorController> teensy;
    try {
        teensy = std::make_unique<MotorController>("/dev/ttyACM0");
    } catch (const boost::wrapexcept<boost::system::system_error>&) {
        teensy = std::make_unique<MotorController>("/dev/ttyACM1");
    }

    KinematicsSolver<DOFS> kin = make_kinSolver();

    //preset ori
    Eigen::Vector3d initPos(TABLE_WIDTH/2, TABLE_LENGTH-paddleHeight/2, 4._mm);
    Eigen::Vector3d initOri(0, 0, 1);

    doHoming_presetPos(*teensy, kin, initPos, initOri, 5._mm)


    //not sure if this will work -- not sure if the packet truncation functionaloty will be interrete docrrectly
    //also be interesting to see how the motorcontroller deals w/ this trncated packet 
    //  --will it be reversed? will it reverse and end up reading the null frames? or will it reverse just the truncated
    
    home_presetPos(*teensy, presetQs);
 
    std::array<double, 3> Ipos = {TABLE_WIDTH/2, TABLE_LENGTH-paddleHeight/2, 4._mm};
    std::array<double, 3> Iori = {0, 0, 1};
    std::array<double, 3> Tpos = {TABLE_WIDTH/2, TABLE_LENGTH-50._cm, 50._cm};
    std::array<double, 3> Tori = {0, 1, 0};
    const int N_STEPS = 2;
    Packet packet;
    int frameCount = 0;


    for (int i = 0; i < N_STEPS; i++) {
        double t = static_cast<double>(i) / (N_STEPS - 1); // 0 → 1

        std::array<double, 3> pos, ori;
        for (int j = 0; j < 3; j++) {
            pos[j] = Ipos[j] + (Tpos[j] - Ipos[j]) * t;
            ori[j] = Iori[j] + (Tori[j] - Iori[j]) * t;
        }
        Frame thisFrame;// &thisFrame = packet.frames[5-frameCount]; //reveersd no clue why
        thisFrame.dt = 4;
        thisFrame.q_new = kin.doIK({pos[0], pos[1], pos[2]}, {ori[0], ori[1], ori[2]}, {0, 0, 0}).qs;
        teensy->sendPacket(actionPacket(thisFrame));
        int iuerhi;
        std::cin>>iuerhi;

        //frameCount++;

        // send once 5 frames filled
        //if (frameCount == 5 || i == N_STEPS - 1) {
          //  teensy.sendPacket(packet);
            //packet = Packet(); // reset for next batch
            //frameCount = 0;
        //}
    }

    Frame thisFrame;
    thisFrame.q_new = presetQs;
    thisFrame.dt=5;
    teensy->sendPacket(actionPacket(thisFrame));

    //extends 1m and then goes back
    /*Frame thisFrame;
    thisFrame.q_new[0]=1;
    thisFrame.dt=2;
    teensy.sendPacket(actionPacket(thisFrame));
    sleep(1);
    thisFrame.q_new[0]=0;
    teensy.sendPacket(actionPacket(thisFrame));*/

    


    return 0;
}

/* 
terminate called after throwing an instance of 'boost::wrapexcept<boost::system::system_error>'
  what():  open: No such file or directory
*/