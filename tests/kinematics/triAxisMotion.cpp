#include <boost/asio.hpp>
#include <iostream>
#include "kinematics/SCComms/packet.h"
#include "kinematics/inverseK/inverseKin.hpp"

#include <chrono>
#include <thread>

#define DOFS 3
constexpr double PI  = 3.1415;

double operator ""_cm(long double val) {return val/100;}

#include <vector>
#include <cmath>
template <std::size_t N>
std::array<std::array<double, 3>, N> 
getRegpolygonVerts(long double rad) {
    std::array<std::array<double, 3>, N> points{};
    for (std::size_t i = 0; i < N; i++) {
        points[i][0] = static_cast<double>(rad * std::cos(i * 2 * PI / N));
        points[i][1] = static_cast<double>(rad * std::sin(i * 2 * PI / N));
        points[i][2] = 0.0;
    }
    return points;
}

using Frame = MotionFrameD<DOFS>;
using Packet = MotionPacketD<DOFS, 5>;

int main() {
    using boost::asio::serial_port;
    using boost::asio::io_service;

    io_service io;
    serial_port port(io, "/dev/ttyACM0");//teensy serial
    port.set_option(boost::asio::serial_port_base::baud_rate(115200));

    std::array<std::array<double, 3>, 3> objanch = getRegpolygonVerts<3>(3._cm);
    std::array<std::array<double, 3>, 3> ppoints = getRegpolygonVerts<3>(30._cm);
    KinematicsSolver<3> kin(ppoints, objanch, {0,0,1});

    kin.cableZeroLens = 
        kin.getCableLens(
            kin.getAnchorPoints(
                Eigen::Vector3d(0, 0, 0), //set to 000
                kin.get_objNormalRef()));

    Packet packet;
    packet.packetId=0;
    
    float T=5;
    int circN = 100;
    float dt_ms=100;
    double rad=10._cm;
    MotionStateD<3> kinout({0, 0, 0}, {0, 0, 0});

    double ang=0;
    int framn = 0;
    std::chrono::steady_clock::time_point lastEventTime = std::chrono::steady_clock::now();
    //const std::chrono::milliseconds delayDuration(dt_ms); // 1 second delay
    while (true) {

        //load up packet / get next 5 frames
        packet.packetId++;
        for (int framn=0; framn<5; framn++) {
            kinout = kin.doIK(
                {rad*cos(ang), rad*sin(ang), 0}, 
                {0, 0, 1}, 
                {0, 0, 0});

            //packet.frames[framn].dt=dt;
            packet.frames[framn].dq_new = kinout.dqs;
            packet.frames[framn].q_new = kinout.qs;

            ang += 2*PI/(float)circN;
        }

        std::chrono::steady_clock::time_point currentTime = std::chrono::steady_clock::now();

        // Check if the delay has passed
        //if (currentTime - lastEventTime >= delayDuration) {
            //performTask();
        //    lastEventTime = currentTime; // Reset the timer
        //}

        if (framn == 5) {
            boost::asio::write(port, boost::asio::buffer(&packet, sizeof(packet)));
            std::cout << "sent packet" << packet.packetId << "\n";
            //std::this_thread::sleep_for(std::chrono::duration<double>(5 * dt));
            framn=0;
        }
    }
}