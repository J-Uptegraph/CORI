# CORI Hardware Interface

Physical hardware integration for CORI robot head movement using Arduino and 9G servo motors.

## Overview

This package bridges the gap between CORI's Gazebo simulation and physical hardware, enabling real-time head movement synchronization. When CORI detects colors or moves its head in simulation, the physical hardware mirrors these movements exactly.

## Hardware Requirements

### Electronics
- **Arduino Nano/Uno** (1x)
- **9G Servo Motors** (1-2x)
  - Pan servo (required): Left/right head movement
  - Tilt servo (optional): Up/down head movement
- **External 5V Power Supply** (1x) - for servos
- **Breadboard and jumper wires**

### Mechanical
- **3D Printed Head Assembly** (optional)
- **Servo mounting brackets**
- **Base platform**

## Wiring Diagram

```
Arduino Nano/Uno:
                    ┌─────────────────┐
                    │    ARDUINO      │
                    │                 │
    Pan Servo ──────│ Pin D11 (PWM)   │  ← Left/Right movement
   Tilt Servo ──────│ Pin D12 (PWM)   │  ← Up/Down movement (nodding)
      LED     ──────│ Pin 13          │
                    │                 │
                    │ 5V  ────────────┼──── External 5V Power (+)
                    │ GND ────────────┼──── External 5V Power (-)
                    │                 │         │
                    │                 │         └─── Servo Power (+)
                    │ GND ────────────┼──────────── Servo Power (-)
                    │                 │
                    └─────────────────┘
                            │
                        USB Cable
                            │
                    ┌─────────────────┐
                    │   COMPUTER      │
                    │   (ROS2)        │
                    └─────────────────┘

Servo Connection (each servo):
    Red Wire    → External 5V Power (+)
    Brown Wire  → Arduino GND
    Orange Wire → Arduino Digital Pin (D11 for pan, D12 for tilt)
```

## Software Architecture

```
┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐
│  CORI Vision    │    │  Arduino Bridge │    │    Arduino      │
│                 │    │                 │    │                 │
│ Color Detection ├───►│ /cori/color_    ├───►│ Serial Commands │
│                 │    │  detected       │    │                 │
└─────────────────┘    └─────────────────┘    └─────────────────┘
                              │                         │
┌─────────────────┐           │                         │
│ Gazebo Sim      │           │                         │
│                 │           │                         │
│ Head Joint      ├───────────┘                         │
│ /model/cori/    │                                     │
│ joint/head_joint│                                     │
│ /cmd_pos        │                                     │
└─────────────────┘                                     │
                                                        │
                                              ┌─────────▼─────────┐
                                              │   Physical Head   │
                                              │                   │
                                              │  D11 Pan: ±90°   │
                                              │  D12 Tilt: ±90°  │
                                              │  🔴 RED → NOD!    │
                                              │                   │
                                              └───────────────────┘
```

## Installation

### 1. Arduino Setup

1. **Install Arduino IDE** or use Arduino CLI
2. **Install Servo Library** (usually pre-installed)
3. **Upload the sketch**:
   ```bash
   # Copy Arduino code to your Arduino IDE
   cp arduino/cori_head_controller/cori_head_controller.ino ~/Arduino/cori_head_controller/
   
   # Or use Arduino CLI
   arduino-cli compile --fqbn arduino:avr:nano arduino/cori_head_controller/
   arduino-cli upload -p /dev/ttyUSB0 --fqbn arduino:avr:nano arduino/cori_head_controller/
   ```

### 2. ROS2 Package Setup

1. **Build the package**:
   ```bash
   cd ~/your_workspace
   colcon build --packages-select cori_hardware
   source install/setup.bash
   ```

2. **Install dependencies**:
   ```bash
   sudo apt install python3-serial
   pip3 install pyserial
   ```

3. **Set permissions for serial port**:
   ```bash
   sudo usermod -a -G dialout $USER
   sudo chmod 666 /dev/ttyUSB0  # or your Arduino port
   ```

## Usage

### Quick Start

1. **Connect Arduino** via USB
2. **Upload Arduino code**
3. **Launch the hardware bridge**:
   ```bash
   ros2 launch cori_hardware cori_hardware.launch.py
   ```

### Manual Testing

Test the hardware with direct commands:

```bash
# Test basic functionality
ros2 run cori_hardware hardware_test basic

# Test color commands
ros2 run cori_hardware hardware_test color

# Test joint angles
ros2 run cori_hardware hardware_test joint

# Interactive testing
ros2 run cori_hardware hardware_test interactive
```

### Integration with CORI Vision

The hardware automatically responds to CORI's color detection:

```bash
# Start CORI vision system
ros2 launch cori_vision laundry_color_detector.launch.py

# Start hardware bridge
ros2 launch cori_hardware cori_hardware.launch.py

# Now when CORI detects colors, the physical head will move!
```

## Color-to-Angle Mapping

The hardware uses the exact same mappings as the CORI simulation:

| Color        | Angle (rad) | Servo Position | Direction     |
|-------------|-------------|----------------|---------------|
| **Black**   | 0.76        | ~155°          | Far Right     |
| **Grey**    | 0.62        | ~145°          | Right         |
| **Purple**  | 0.45        | ~130°          | Mid-Right     |
| **Blue**    | 0.23        | ~110°          | Slight Right  |
| **Green**   | 0.0         | 90°            | **Center**    |
| **Yellow**  | -0.23       | ~70°           | Slight Left   |
| **Orange**  | -0.45       | ~50°           | Mid-Left      |
| **Red**     | -0.62       | ~35°           | Left + **NOD**|

## Commands

### ROS Topics

**Subscriptions:**
- `/model/cori/joint/head_joint/cmd_pos` - Direct angle commands from Gazebo
- `/cori/color_detected` - Color detection commands
- `/cori/hardware_command` - Direct hardware commands

**Publications:**
- `/cori/hardware_status` - Arduino status messages
- `/cori/hardware_feedback` - Arduino feedback and responses

### Arduino Commands

Send commands via serial or ROS topics:

```bash
# Direct hardware commands
ros2 topic pub /cori/hardware_command std_msgs/String '{data: "CENTER"}'
ros2 topic pub /cori/hardware_command std_msgs/String '{data: "TEST"}'
ros2 topic pub /cori/hardware_command std_msgs/String '{data: "STATUS"}'
ros2 topic pub /cori/hardware_command std_msgs/String '{data: "NOD"}'     # Manual nodding

# Color commands (RED will automatically trigger nodding!)
ros2 topic pub /cori/color_detected std_msgs/String '{data: "red"}'      # Moves left + nods
ros2 topic pub /cori/color_detected std_msgs/String '{data: "blue"}'     # Moves right only
ros2 topic pub /cori/color_detected std_msgs/String '{data: "green"}'    # Centers

# Direct angle commands
ros2 topic pub /model/cori/joint/head_joint/cmd_pos std_msgs/Float64 '{data: 0.5}'
```

## Troubleshooting

### Common Issues

1. **Arduino not detected**:
   ```bash
   # Check available ports
   ls /dev/tty*
   
   # Check permissions
   sudo chmod 666 /dev/ttyUSB0
   
   # Check if Arduino is responding
   screen /dev/ttyUSB0 115200
   ```

2. **Servo not moving**:
   - Check power connections (servos need external 5V)
   - Verify servo signal wires are connected to D11 (pan) and D12 (tilt)
   - Test with simple Arduino servo sweep example
   - Use `NOD` command to test tilt servo specifically

3. **No ROS communication**:
   ```bash
   # Check ROS topics
   ros2 topic list | grep cori
   
   # Check node status
   ros2 node info /cori_arduino_bridge
   
   # Monitor feedback
   ros2 topic echo /cori/hardware_feedback
   ```

### Debug Mode

Enable debug output:

```bash
# Launch with debug logging
ros2 launch cori_hardware cori_hardware.launch.py --ros-args --log-level debug
```

## Customization

### Servo Calibration

Adjust servo limits in Arduino code:

```cpp
// In cori_head_controller.ino
const int SERVO_MIN = 0;      // Minimum angle
const int SERVO_MAX = 180;    // Maximum angle  
const int SERVO_CENTER = 90;  // Center position

// Pin assignments
const int PAN_SERVO_PIN = 11;  // D11 - Left/Right movement
const int TILT_SERVO_PIN = 12; // D12 - Up/Down movement (nodding)
```

### Speed Control

Modify movement speed:

```cpp
// In smoothMove() function (pan movement)
int stepDelay = max(5, 20 - steps); // Adjust delay

// In smoothTiltMove() function (nodding movement)
int stepDelay = max(3, 15 - steps); // Faster for nodding
```

### Add More Servos

Extend for tilt movement:

```cpp
// Tilt servo is already implemented on D12!
// For additional servos, use other pins:
const int EXTRA_SERVO_PIN = 4;  // D4 for additional movement
Servo extraServo;

// In setup()
extraServo.attach(EXTRA_SERVO_PIN);
```

## Performance Notes

- **Latency**: ~100ms from ROS command to physical movement
- **Accuracy**: ±2° servo positioning accuracy
- **Speed**: 0.1s/60° movement speed (typical 9G servo)
- **Range**: 180° physical range mapped to CORI's ±103° simulation range

## Special Features

### 🔴 RED Color Nodding
When the vision system detects **RED**, the head will:
1. **Move left** to the red position (-0.62 radians)
2. **Automatically nod** 3 times using the D12 tilt servo
3. **Return to neutral** tilt position

This creates an engaging "yes" response when CORI sees red objects!

### Manual Nodding
You can trigger nodding manually:
```bash
ros2 topic pub /cori/hardware_command std_msgs/String '{data: "NOD"}'
```

## Future Enhancements

- **Feedback Sensors**: Add encoders for position feedback
- **Force Control**: Add force/torque sensing  
- **Wireless**: ESP32 for wireless communication
- **Vision**: Add camera mounted on physical head
- **Multi-DOF**: Add more axes (roll, extra servos)
- **Emotion System**: Different gestures for different colors

## Support

For issues or questions:
1. Check the troubleshooting section
2. Monitor ROS topics for debug information
3. Use the hardware test utility for diagnosis
4. Check Arduino serial monitor for low-level debugging

---

**Created for CORI Robot Project**  
*Bridging simulation and reality, one servo at a time* 🤖