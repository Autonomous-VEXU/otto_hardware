// robot_hardware/vex_comms.h
#ifndef OTTO_HARDWARE_VEX_COMMS_H
#define OTTO_HARDWARE_VEX_COMMS_H

#include <libserial/SerialPort.h>
#include <libserial/SerialStream.h>
#include <string>
#include <vector>

using namespace LibSerial;

struct MotorVelocities {
  uint16_t magic;
  double fl;
  double fr;
  double bl;
  double br;
};

class VEXComms{
  
  public:

    bool initalize(const std::string &serial_device); //maybe add baud rate later
    bool is_connected() const;

    void writeVelocities();
    void readVelocities();

  private:
};

#endif // OTTO_HARDWARE_VEX_COMMS_H