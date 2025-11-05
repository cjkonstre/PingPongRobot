#pragma once

#include <boost/asio.hpp>
#include <array>
#include <cstdint>

#define MOTOR_ENABLE  0b1000
#define MOTOR_DISABLE 0b1001

//interface for motorcontroller
template <int DOFS, typename Packet>
class MotorControllerD {
private:
    boost::asio::io_service io;
    boost::asio::serial_port port;
    std::array<double, DOFS> motorOffsets = {0}; // subtracts these

    Packet preprocessPacket(Packet packet);

public:
    MotorControllerD(const char* device);
    MotorControllerD() = default;

    std::size_t sendPacket(Packet packet);

    void setOffset(int motori, double offset);

    void sendCommand(uint8_t command, uint8_t arg);
};

#include "motorController.tpp"
