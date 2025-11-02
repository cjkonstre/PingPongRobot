#include <boost/asio.hpp>
#include <iostream>
#include "kinematics/SCComms/packet.h"
#include "kinematics/inverseK/inverseKin.hpp"
#include <array>
#include "matplotlibcpp.h"

#include "iostream"

#define DOFS 7
constexpr double PI  = 3.1415;

constexpr double operator ""_cm(long double val) {return val/100;}
constexpr double operator ""_mm(long double val) {return val/1000;}
constexpr double operator ""_ft(long double val) {return val*0.3048;}
constexpr double operator ""_in(long double val) {return val/12*0.3048;}

using Frame = MotionFrameD<DOFS>;
using Packet = MotionPacketD<DOFS, 5>;

Packet actionPacket(Frame action) {
    Packet packet;
    packet.frames[0] = action; 
    packet.packetLength=1;
    return packet;
}

class MotorController {
private:
    boost::asio::io_service io;
    boost::asio::serial_port port;
    std::array<double, DOFS> motorOffsets = {0}; //subtracts these

    Packet preprocessPacket(Packet packet) {
        for (int i=0; i<packet.packetLength; i++) {
            Frame& frame = packet.frames[i];
            for (int j=0; j<DOFS; j++) frame.q_new[j] += motorOffsets[j]; 
        }
        return packet;
    }

public:
    MotorController(const char* device): port(io, device) {
        port.set_option(boost::asio::serial_port_base::baud_rate(115200));
    }

    auto getOffsets(bool invert) {
        if (invert){
            auto mofsets = motorOffsets;
            for (int i=0; i<DOFS; i++) mofsets[i]=-mofsets[i];
            return mofsets;
        }
        return motorOffsets;
    }

    //preprocesses -- should be temporary. this shold happen on the board, not here
    auto sendPacket(Packet packet){
        Packet proscessedPacket = preprocessPacket(packet);
        return boost::asio::write(port, boost::asio::buffer(&proscessedPacket, sizeof(proscessedPacket)));
    }

    //if at q_i=20 and want it to be zerod there, set offset as 20 
    //if at q_i=0 and want osset to be at 30 there, set offset as -30
    void setOffset(int motori, double offset) {
        motorOffsets[motori] = offset;
    }

    enum Command {
        MOTOR_ENABLE = 0b1000,
        MOTOR_DISABLE = 0b1001,
    };
 
    void sendCommand(uint8_t command, uint8_t arg) {
        uint32_t comm = (1u << 16) | ((command & 0xF) << 4) | (arg & 0xF);
        Packet actionPacket;
        actionPacket.packetId = comm;
        sendPacket(actionPacket);
    }

};

//pseudo-manual home -- increment until set.
void home_pseudoManual(MotorController& controller, double toOffset){
    for (int i=0; i<DOFS; i++) { //i being motor thats being homed rn
        double offset = 0;
        for (;;){
            double am_cm;
            std::cout << "Moving axis " << i << "; Enter amount to move inwards by [cm]. input -1 to set as offset:\n";
            std::cin >> am_cm;
            if (am_cm < 0) break;
            double am = am_cm * 1/100;
            offset -= am;

            Frame mvframe;
            double mvtime = 1;
            mvframe.dt = mvtime; 
            mvframe.dq_new = {0};
            mvframe.q_new = controller.getOffsets(true); mvframe.q_new[i]=offset;
            controller.sendPacket(actionPacket(mvframe));
        }
        controller.setOffset(i, offset-30._cm);
        
        Frame unspool;
        unspool.dt=6;
        unspool.dq_new={0};
        unspool.q_new=controller.getOffsets(true);
        controller.sendPacket(actionPacket(unspool));
    }
}

//should be run immediately after startup. motorcontroller should think all motors are zeroed
void home_manual(MotorController& controller, double toOffset){
    double unspoolAm = 4;//m
    double subunspoolAm = 2._cm;//cm, small just to conteract the pre extension

    for (int i=0; i<DOFS; i++) {controller.sendCommand(MotorController::MOTOR_DISABLE, i); controller.setOffset(i, 0);} //disable all motors
    Frame rstf; //reset frame
    rstf.dt = 0.1;
    rstf.q_new.fill(0.0);; //tell motorcontroller that its motors should be at 0 rn. resets from previous runs
    controller.sendPacket(actionPacket(rstf));
    for (int i=0; i<DOFS; i++) { //i being motor thats being homed rn
        std::cout<<"Move motor " << i << " to offset (" << toOffset*100 << "cm). Enter `1` to continue:\n";

        int input;
        std::cin >> input;
        if (input != 1) {std::cout<<"home canceled\n"; break;}

        //set as 30
        controller.sendCommand(MotorController::MOTOR_ENABLE, i);
        controller.setOffset(i, -toOffset);

        Frame unspool;
        unspool.dt = 0.5;
        unspool.q_new.fill(toOffset+unspoolAm); unspool.q_new[i]=toOffset+subunspoolAm; //unspool a lil to account for pre rotation of full unspool. not sure why that happens
        controller.sendPacket(actionPacket(unspool));

        unspool.dt = 5;
        unspool.q_new.fill(toOffset+unspoolAm); //unspool all. if not homed yet they'll still be disabled so they wont do anything
        controller.sendPacket(actionPacket(unspool));
    }
}

void home_presetPos(MotorController& controller, std::array<double, DOFS> presetQs) {
    for (int i=0; i<DOFS; i++) {controller.sendCommand(MotorController::MOTOR_DISABLE, i);} //disable all motors
    for (int i=0; i<DOFS; i++){
        double input;
        std::cout << "Move axis " << i << " till taught. enter 1 to continue\n";
        std::cin >> input;

        controller.sendCommand(MotorController::MOTOR_ENABLE, i);

        controller.setOffset(i, -presetQs[i]);
    }
    //loop over and do micro adjustments to make cables nice and taught?
}

//units in m
//relative to table height
#define TABLE_HEIGHT 29.75_in //roughly
#define TABLE_WIDTH 5._ft //x, width of table
#define TABLE_LENGTH 4.5_ft //y, length of half table !!
constexpr std::array<std::array<double, 3>, 7> pulleyPoss = {{
    {{TABLE_WIDTH + 1.5_ft + 1._cm, TABLE_LENGTH-3._cm, 6._ft+3.7_cm - TABLE_HEIGHT}},
    {{TABLE_WIDTH+30._cm, TABLE_LENGTH-3._cm, 26.75_in+8.5_cm - TABLE_HEIGHT}},
    {{-18._in+2._cm, -2._ft+3.5_in+3._cm, 2.5_ft+3.7_cm-TABLE_HEIGHT}},
    {{TABLE_WIDTH/2-3.5_cm, -2._ft+3.5_in+3._cm, 64.75_in-TABLE_HEIGHT}},
    {{-1.5_ft -1._cm, TABLE_LENGTH-3._cm, 6._ft+3.7_cm - TABLE_HEIGHT}},
    {{-30._cm, TABLE_LENGTH-3._cm, 26.75_in+8.5_cm - TABLE_HEIGHT}},
    {{TABLE_WIDTH+18._in-2._cm, -2._ft+3.5_in+3._cm, 2.5_ft+3.7_cm-TABLE_HEIGHT}},
}};

//w/ ref to paddle facing forward
constexpr double paddleHeight = 157.49_mm;
constexpr std::array<std::array<double, 3>, 7> anchorOffsets = {{
    {{59.887_mm, -4._mm, 47.225_mm}},
    {{42.352_mm, -4._mm, -78.264_mm}},
    {{-74.917_mm, -4._mm, -19.694_mm}},
    {{0, -4._mm, paddleHeight/2}},
    {{-42.352_mm, -4._mm, -78.264_mm}},
    {{-59.887_mm, -4._mm, 47.225_mm}},
    {{74.917_mm, -4._mm, -19.694_mm}},
}};

int main() {
    MotorController teensy("/dev/ttyACM1");//teensy serial
    KinematicsSolver<DOFS> kin(pulleyPoss, anchorOffsets, {0, 1, 0});
    kin.cableZeroLens = {0,0,0,0,0,0,0};//make sure

    //preset ori
    Eigen::Vector3d initPos(TABLE_WIDTH/2, TABLE_LENGTH-paddleHeight/2, 4._mm);
    Eigen::Vector3d initOri(0, 0, 1);
    std::array<double, 7> presetQs = kin.doIK(initPos, initOri, Eigen::Vector3d::Zero()).qs;

    //not sure if this will work -- not sure if the packet truncation functionaloty will be interrete docrrectly
    //also be interesting to see how the motorcontroller deals w/ this trncated packet 
    //  --will it be reversed? will it reverse and end up reading the null frames? or will it reverse just the truncated

    //BELOW DOES SOME PLT VIZ FOR IK
    /*namespace plt = matplotlibcpp;
    Eigen::Matrix<double, 3, 7> points = kin.getAnchorPoints(Eigen::Vector3d(TABLE_WIDTH/2, TABLE_LENGTH-50._cm, 50._cm), 
                                                             Eigen::Vector3d(0, 1, 0));
    std::vector<double> x, y, z;
    for (const auto& pt : anchorOffsets) {
        x.push_back(pt[0]);
        y.push_back(pt[1]);
        z.push_back(pt[2]);
    }

    for (int i = 0; i < points.cols(); ++i) {
        x.push_back(points(0, i));
        y.push_back(points(1, i));
        z.push_back(points(2, i));
    }

    x.push_back(0);
    y.push_back(0);
    z.push_back(0);

    x.push_back(TABLE_WIDTH);
    y.push_back(0);
    z.push_back(0);

    x.push_back(0);
    y.push_back(TABLE_LENGTH);
    z.push_back(0);

    x.push_back(TABLE_WIDTH);
    y.push_back(TABLE_LENGTH);
    z.push_back(0);

    plt::figure();
    plt::scatter(x, y, z);  // 3D scatter using red dots
    plt::xlabel("X");
    plt::ylabel("Y");
    plt::set_zlabel("Z");
    plt::xlim(0, 2);
    plt::ylim(0, 2);
    PyRun_SimpleString("import matplotlib.pyplot as plt; "
                   "ax = plt.gca(projection='3d'); "
                   "ax.set_zlim(0, 2)");
    plt::show();*/
    
    home_presetPos(teensy, presetQs);
 
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
        teensy.sendPacket(actionPacket(thisFrame));
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
    teensy.sendPacket(actionPacket(thisFrame));

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

/* "/home/connor/PingPongRobot/out/build/GCC 14.2.0 x86_64-linux-gnu/bin/test_full_motionDemo"
terminate called after throwing an instance of 'boost::wrapexcept<boost::system::system_error>'
  what():  open: No such file or directory
Aborted (core dumped)*/