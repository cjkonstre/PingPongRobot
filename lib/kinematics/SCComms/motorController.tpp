#pragma once

#include <boost/asio/write.hpp>
#include <boost/asio/serial_port_base.hpp>

template <int DOFS, typename Packet>
Packet MotorControllerD<DOFS, Packet>::preprocessPacket(Packet packet) {
    for (int i = 0; i < packet.packetLength; i++) {
        auto& frame = packet.frames[i];
        for (int j = 0; j < DOFS; j++) {
            frame.q_new[j] += motorOffsets[j];
        }
    }
    return packet;
}

template <int DOFS, typename Packet>
MotorControllerD<DOFS, Packet>::MotorControllerD(const char* device)
    : port(io, device) {
    port.set_option(boost::asio::serial_port_base::baud_rate(115200));
}

template <int DOFS, typename Packet>
std::size_t MotorControllerD<DOFS, Packet>::sendPacket(Packet packet) {
    Packet processedPacket = preprocessPacket(packet);
    return boost::asio::write(port, boost::asio::buffer(&processedPacket, sizeof(processedPacket)));
}

template <int DOFS, typename Packet>
void MotorControllerD<DOFS, Packet>::setOffset(int motori, double offset) {
    motorOffsets[motori] = offset;
}

template <int DOFS, typename Packet>
void MotorControllerD<DOFS, Packet>::sendCommand(uint8_t command, uint8_t arg) {
    uint32_t comm = (1u << 16) | ((command & 0xF) << 4) | (arg & 0xF);
    Packet actionPacket;
    actionPacket.packetId = comm;
    sendPacket(actionPacket);
}
