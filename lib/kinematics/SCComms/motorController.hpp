#pragma once

#include <boost/asio.hpp>
#include <array>
#include <cstdint>

#define MOTOR_ENABLE  0b1000
#define MOTOR_DISABLE 0b1001

//interface for motorcontroller
template <int DOFS, typename VelFrame>
class MotorControllerD {
private:
    boost::asio::io_service io;
    boost::asio::serial_port port;

public:
    MotorControllerD(const char* device);
    MotorControllerD() = default;

    inline std::size_t sendVel(VelFrame frame);

    void sendCommand(uint8_t command, uint8_t arg);
};

#include "motorController.tpp"
