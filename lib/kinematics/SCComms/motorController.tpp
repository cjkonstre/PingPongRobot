#pragma once

#include <boost/asio/write.hpp>
#include <boost/asio/serial_port_base.hpp>


template <int DOFS, typename Packet>
MotorControllerD<DOFS, Packet>::MotorControllerD(const char* device)
    : port(io, device) {
    port.set_option(boost::asio::serial_port_base::baud_rate(2000000));
}

template <int DOFS, typename Packet>
std::size_t MotorControllerD<DOFS, Packet>::sendPacket(Packet packet) {
    return boost::asio::write(port, boost::asio::buffer(&packet, sizeof(packet)));
}

template <int DOFS, typename Packet>
void MotorControllerD<DOFS, Packet>::setCurrentState(std::array<double, DOFS> qs, std::array<double, DOFS> dqs) {
    Packet packet;
    packet.frames[0].dt = -1;
    packet.frames[0].q_new = qs;
    packet.frames[0].dq_new = dqs;
    packet.packetLength = 1;
    sendPacket(packet);
}

template <int DOFS, typename Packet>
void MotorControllerD<DOFS, Packet>::sendCommand(uint8_t command, uint8_t arg) {
    uint32_t comm = (1u << 16) | ((command & 0xF) << 4) | (arg & 0xF);
    Packet actionPacket;
    actionPacket.packetId = comm; //overrides movment. should use frame for this but doesnt matter
    sendPacket(actionPacket);
}
