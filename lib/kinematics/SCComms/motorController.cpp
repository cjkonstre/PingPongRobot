#include "kinematics/SCComms/motorController.h"

Packet MotorController::preprocessPacket(Packet packet) {
    for (int i=0; i<packet.packetLength; i++) {
        Frame& frame = packet.frames[i];
        for (int j=0; j<DOFS; j++) frame.q_new[j] += motorOffsets[j]; 
    }
    return packet;
}

MotorController::MotorController(const char* device): port(io, device) {
    port.set_option(boost::asio::serial_port_base::baud_rate(115200));
}

auto MotorController::getOffsets(bool invert) {
    if (invert){
        auto mofsets = motorOffsets;
        for (int i=0; i<DOFS; i++) mofsets[i]=-mofsets[i];
        return mofsets;
    }
    return motorOffsets;
}

//preprocesses -- should be temporary. this shold happen on the board, not here
auto MotorController::sendPacket(Packet packet){
    Packet proscessedPacket = preprocessPacket(packet);
    return boost::asio::write(port, boost::asio::buffer(&proscessedPacket, sizeof(proscessedPacket)));
}

//if at q_i=20 and want it to be zerod there, set offset as 20 
//if at q_i=0 and want osset to be at 30 there, set offset as -30
void MotorController::setOffset(int motori, double offset) {
    motorOffsets[motori] = offset;
}

enum MotorController::Command {
    MOTOR_ENABLE = 0b1000,
    MOTOR_DISABLE = 0b1001,
};

void MotorController::sendCommand(uint8_t command, uint8_t arg) {
    uint32_t comm = (1u << 16) | ((command & 0xF) << 4) | (arg & 0xF);
    Packet actionPacket;
    actionPacket.packetId = comm;
    sendPacket(actionPacket);
}
