#pragma once

#include <boost/asio/write.hpp>
#include <boost/asio/serial_port_base.hpp>

//temp
#include <iostream>


template <int DOFS, typename VelFrame>
MotorControllerD<DOFS, VelFrame>::MotorControllerD(const char* device)
    : port(io, device) {
    port.set_option(boost::asio::serial_port_base::baud_rate(2000000));
}

template <int DOFS, typename VelFrame>
inline std::size_t MotorControllerD<DOFS, VelFrame>::sendVel(VelFrame frame) {
    return boost::asio::write(port, boost::asio::buffer(&frame, sizeof(frame))); //may need to be improved
}

template <int DOFS, typename VelFrame>
void MotorControllerD<DOFS, VelFrame>::sendCommand(uint8_t command, uint8_t arg) {
    uint32_t comm = (1u << 16) | ((command & 0xF) << 4) | (arg & 0xF);
    VelFrame frame;
    frame.index = comm; //overrides movment. should use frame for this but doesnt matter
    sendVel(frame);
}
