#include "otto_hardware/vex_comms.hpp"
#include <cstring>
#include <cstdio>
#include <libserial/SerialPort.h>
#include <libserial/SerialStream.h>

using namespace LibSerial;

SerialPort serial_port;
SerialStream serial_stream;

bool VEXComms::initalize(const std::string& serial_device)
{
    try {
        serial_port.Open(serial_device);
        serial_port.SetBaudRate(BaudRate::BAUD_1152000);
    } catch (const std::exception& e) {
        printf("Serial Exception on setup: %s\n", e.what());
        return false;
    }
}

bool VEXComms::is_connected() const{
    return serial_port.IsOpen();
}

void VEXComms::writeVelocities(){

}

void VEXComms::readVelocities(){
    
}