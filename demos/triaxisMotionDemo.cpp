#include <boost/asio.hpp>
#include "kinematics/SCComms/packet.h"

using Frame = MotionFrameD<3>;
using Packet = MotionPacketD<3, 5>;

int main() {
    using boost::asio::serial_port;
    using boost::asio::io_service;

    io_service io;
    serial_port port(io, "/dev/ttyACM1");//teensy serial
    port.set_option(boost::asio::serial_port_base::baud_rate(115200));

    Packet packet;
    
    packet.frames[0].dt=2;
    packet.frames[0].q_new= {0};
    packet.frames[0].dq_new = {0};

    packet.frames[1].dt=1;
    packet.frames[1].q_new= {0};
    packet.frames[1].dq_new = {0};

    packet.frames[2].dt=2;
    packet.frames[2].q_new= {0.1};
    packet.frames[2].dq_new = {0};

    packet.frames[3].dt=0.5;
    packet.frames[3].q_new= {0.1};
    packet.frames[3].dq_new = {0};

    packet.frames[4].dt=1;
    packet.frames[4].q_new= {0.2};
    packet.frames[4].dq_new = {0};

    while (true) {
        packet.packetId++;
        boost::asio::write(port, boost::asio::buffer(&packet, sizeof(packet)));
        sleep(3);
    }
}