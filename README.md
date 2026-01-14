# otto_hardware
Hardware interface for connecting ros2_control controllers to a VEX V5 brain + motors.

URDF Plugin Name:`otto_hardware/OmniWheelhardware`

## omni_wheel_hardware
Hardware interface system component.

### CallBack Return Methods:
- **`on_init()`**
    - runs **before** the hardware is actually connected to
    - loads parameters from URDF `ros2_control` hardware tag
        - serial_port (default: '/dev/ttyUSB0' )
        - baud (default: 115200)
    - checks configuration per joint against expected configuration in order to catch early errors:
        - number and type of state interfaces
        - number and type of command interfaces
- **`on_activate()`**
    - activates HW by opening a serial connection
- **`on_deactivate()`**
    - deactivates HW

### State Interface Methods:
- **`export_state_interfaces()`**
    - position of the wheels using encoders
    - velocity of the wheels as reported by the hardware

### Command Interface Methods:
- **`export_command_interfaces()`**
    - velocity of the motors

### Return Type Methods:
- **`read()`**
    - Read both position and velocity state interfaces from the connected hardware and update both state interfaces
- **`write()`**
    - Write velocity commands to the connected hardware in the form of the `MotorVelocity` struct shown below:

```cpp
struct MotorVelocities {
  uint16_t magic; // 0xFEFA
  double fl; // forward left wheel
  double fr; // forward right wheel
  double bl; // back left wheel
  double br; // back right wheel
};
```

## URDF/xacro configuration

Example `ros2_control` tag for this hardware interface + the omni wheel drive controller from `ros2_controllers`:

```xml
  <ros2_control name="DriveSystem" type="system">
        <hardware>
            <plugin>otto_hardware/OmniWheelHardware</plugin>
            <param name='serial_port'>/dev/ttyUSB0</param>
            <param name='baud_rate'>115200</param>
        </hardware>
        <joint name="wheel_1">
            <command_interface name="velocity"/>
            <state_interface name="position"/>
            <state_interface name="velocity"/>
        </joint>
        <joint name="wheel_2">
            <command_interface name="velocity"/>
            <state_interface name="position"/>
            <state_interface name="velocity"/>
        </joint>
        <joint name="wheel_3">
            <command_interface name="velocity"/>
            <state_interface name="position"/>
            <state_interface name="velocity"/>
        </joint>
        <joint name="wheel_4">
            <command_interface name="velocity"/>
            <state_interface name="position"/>
            <state_interface name="velocity"/>
        </joint>
    </ros2_control>
```

## Documentation + References
- [ros2_control API](http://docs.ros.org/en/jazzy/p/hardware_interface/)
- [Serialib GitHub Repository](https://github.com/imabot2/serialib/tree/master)
