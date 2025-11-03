#pragma once

#include <boost/asio.hpp>

//need to completely revamp, this is very temp

template <typename Packet> //VERY temp
class MotorController {
private:
    boost::asio::io_service io;
    boost::asio::serial_port port;
    std::array<double, DOFS> motorOffsets = {0}; //subtracts these

    Packet preprocessPacket(Packet packet);

public:
    MotorController(const char* device);

    MotorController() = default;

    auto getOffsets(bool invert);

    //preprocesses -- should be temporary. this shold happen on the board, not here
    auto sendPacket(Packet packet);

    //if at q_i=20 and want it to be zerod there, set offset as 20 
    //if at q_i=0 and want osset to be at 30 there, set offset as -30
    void setOffset(int motori, double offset);

    enum Command;

    void sendCommand(uint8_t command, uint8_t arg);
};