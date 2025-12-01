// robot_hardware/vex_comms.h
#ifndef ROBOT_HARDWARE_VEX_COMMS_H
#define ROBOT_HARDWARE_VEX_COMMS_H

#include <serial/serial.h>
#include <string>
#include <vector>

struct MotorVelocities {
    uint16_t magic;
    double fl;
    double fr;
    double bl;
    double br;
};

class VEXComms
{
public:
  VEXComms();

  bool setup(const std::string& serial_device, uint32_t baud_rate);
  bool connected() const;

  void send_motors(serial::Serial &serial_conn, const MotorVelocities& mv);
  void send_motors(const MotorVelocities& mv);

  std::string sendMsg(const std::string &msg_to_send, bool print_output = false);

private:
  serial::Serial serial_conn;
};

#endif // ROBOT_HARDWARE_VEX_COMMS_H