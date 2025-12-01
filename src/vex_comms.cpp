#include "robot_hardware/vex_comms.hpp"
#include <cstring>
#include <cstdio>

VEXComms::VEXComms()
: serial_conn()
{}
bool VEXComms::setup(const std::string& serial_device, uint32_t baud_rate)
{
    try {
        serial_conn.setPort(serial_device);
        serial_conn.setBaudrate(baud_rate);
        serial::Timeout timeout = serial::Timeout::simpleTimeout(50); // 50 ms 
        serial_conn.open();
        return serial_conn.isOpen();
    } catch (const std::exception& e) {
        printf("Serial Exception on setup: %s\n", e.what());
        return false;
    }
}

bool VEXComms::connected() const
{
    return serial_conn.isOpen();
}

void VEXComms::send_motors(serial::Serial &serial_conn, const MotorVelocities& mv) {
    serial_conn.write(reinterpret_cast<const uint8_t*>(&mv), sizeof(MotorVelocities));
}

std::string VEXComms::sendMsg(const std::string &msg_to_send, bool print_output)
{
    if (!connected()) return "";
    serial_conn.write(reinterpret_cast<const uint8_t *>(msg_to_send.data()), msg_to_send.size());
    if (print_output) printf("Sent %zu bytes\n", msg_to_send.size());

    std::string rx = serial_conn.read(1024); 
    if (print_output) printf("Received %zu bytes\n", rx.size());

    return rx;
}