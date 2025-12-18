# otto_hardware
Hardware interface for connecting ros2_control controllers to a VEX V5 brain + motors.

URDF Plugin Name:`otto_hardware/OmniWheelhardware`
## Dependencies / Included Modules
- LibSerial
    - `<libserialSerialPort.h>`
    - `<libserialSerialStream.h>`

## vex_comms
Serial communication framework for communicating with the VEX brain.

### Methods:
- `initalize()`
    - Initiates the serial connection to the VEX V5 Brain by setting the port and baud rate.
- `is_connected()`
    - Returns a boolean. True means the serial port is open, false means it is not open.
- `writeVelocities()`
    - Not written yet
- `readVelocities()`
    - Not written yet

### Structs:
```cpp
struct MotorVelocities {
  uint16_t magic; // 0xFEFA
  double fl; // forward left wheel
  double fr; // forward right wheel
  double bl; // back left wheel
  double br; // back right wheel
};
```

## omni_wheel_hardware
Hardware interface system component.

### CallBack Return Methods:
- `on_init()`
    - Initialize hardware

### State Interface Methods:
- `export_state_interfaces()`

### Command Interface Methods:
- `export_command_interfaces()`

### Return Type Methods:
- `read()`
    - Read data from connected hardware
- `write()`
    - Write data to connected hardware
