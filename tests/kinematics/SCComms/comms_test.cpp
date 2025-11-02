#include <boost/asio.hpp>
#include <iostream>
#include "kinematics/SCComms/packet.h"

#define DOFS 3

using Frame = MotionFrameD<DOFS>;
using Packet = MotionPacketD<DOFS, 5>;

long double operator ""_cm(long double val) {return val/100;}

double DT=0.1;
double tensionOffset=-1._cm;
int main() {
    using boost::asio::serial_port;
    using boost::asio::io_service;

    io_service io;
    serial_port port(io, "/dev/ttyACM0");//teensy serial
    port.set_option(boost::asio::serial_port_base::baud_rate(115200));

    Packet packet;
    packet.packetId=0;
    
    packet.frames[0].dt=1;
    packet.frames[0].q_new= {0};
    packet.frames[0].dq_new = {0};
    packet.frames[1].dq_new = {0};
    packet.frames[2].dq_new = {0};
    packet.frames[3].dq_new = {0};

    packet.frames[0].dt=1;
    packet.frames[0].q_new[0]=0-tensionOffset;
    packet.frames[0].q_new[1]=0-tensionOffset;
    packet.frames[0].q_new[2]=0-tensionOffset;

    packet.frames[1].dt=DT;
    packet.frames[1].q_new[0]= -10._cm -tensionOffset;
    packet.frames[1].q_new[1]= 6._cm -tensionOffset;
    packet.frames[1].q_new[2]= 6._cm -tensionOffset;

    packet.frames[2].dt=DT;
    packet.frames[2].q_new[0]= 6._cm -tensionOffset;
    packet.frames[2].q_new[1]= -10._cm -tensionOffset;
    packet.frames[2].q_new[2]= 6._cm -tensionOffset;

    packet.frames[3].dt=DT;
    packet.frames[3].q_new[0]= 6._cm -tensionOffset;
    packet.frames[3].q_new[1]= 6._cm -tensionOffset;
    packet.frames[3].q_new[2]= -10._cm -tensionOffset;

    packet.frames[4].dt=DT;
    packet.frames[4].q_new[0]= 0 -tensionOffset;
    packet.frames[4].q_new[1]=0 -tensionOffset;
    packet.frames[4].q_new[2]=0 -tensionOffset;

    while (true) {
        packet.packetId++;
        boost::asio::write(port, boost::asio::buffer(&packet, sizeof(packet)));
        sleep(2+(int)(5*DT));
    }
}