#pragma once

#include <boost/asio/write.hpp>
#include <boost/asio/serial_port_base.hpp>

//temp
#include <iostream>


template <int DOFS, typename Frame>
MotorControllerD<DOFS, Frame>::MotorControllerD(const char* device)
    : port(io, device) {
    port.set_option(boost::asio::serial_port_base::baud_rate(2000000));
}

template <int DOFS, typename Frame>
inline std::size_t MotorControllerD<DOFS, Frame>::sendFrame(const Frame& frame) {
    return boost::asio::write(port, boost::asio::buffer(&frame, sizeof(frame))); //may need to be improved
}

template <int DOFS, typename Frame>
void MotorControllerD<DOFS, Frame>::sendCommand(uint8_t command, uint8_t arg) {
    uint32_t comm = (1u << 16) | ((command & 0xF) << 4) | (arg & 0xF);
    Frame frame;
    frame.index = comm; //overrides movment. should use frame for this but doesnt matter
    sendFrame(frame);
}

template <int DOFS, typename Frame>
void MotorControllerD<DOFS, Frame>::sendCommand(uint8_t command, uint8_t arg, std::array<double, 7> poss) {
    uint32_t comm = (1u << 16) | ((command & 0xF) << 4) | (arg & 0xF);
    Frame frame;
    for (int i=0; i<7; i++) {frame.poss[i] = poss[i];}
    frame.index = comm; //overrides movment. should use frame for this but doesnt matter
    sendFrame(frame);
}