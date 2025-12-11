#pragma once

#include <boost/asio.hpp>
#include <array>
#include <cstdint>

#define MOTOR_ENABLE  0b1000
#define MOTOR_DISABLE 0b1001
#define MOTOR_SETSTP 0b1010

//interface for motorcontroller
template <int DOFS, typename Frame>
class MotorControllerD {
private:
    boost::asio::io_service io;
    boost::asio::serial_port port;

public:
    MotorControllerD(const char* device);
    MotorControllerD() = default;

    inline std::size_t sendFrame(Frame frame);

    void sendCommand(uint8_t command, uint8_t arg);
    void sendCommand(uint8_t command, uint8_t arg, std::array<double, 7> poss);
};

#include "motorController.tpp"
