# CORI Manual Control Guide

Simple commands to control CORI's movements in real-time.

## Quick Start

1. Launch manual control mode:
   ```bash
   ./build.bash
   # Select option 5: Manual Robot Control
   ```

2. In a **new terminal**, source the workspace:
   ```bash
   cd ~/Workspaces/Robotics/Projects/CORI/cori_ws
   source install/setup.bash
   ```

3. Use simple commands!

## Three Ways to Control CORI

### Option 1: Quick Shortcuts (Easiest)
```bash
source cori_control_shortcuts.sh

cori-look-left
cori-look-right
cori-wave-left
cori-sit
cori-stand
```

### Option 2: Simple Commands
```bash
# Head control
ros2 run cori_control manual_control look left
ros2 run cori_control manual_control look right
ros2 run cori_control manual_control look center

# Arm movements
ros2 run cori_control manual_control wave left
ros2 run cori_control manual_control wave right
ros2 run cori_control manual_control reach forward

# Body posture
ros2 run cori_control manual_control sit
ros2 run cori_control manual_control stand
ros2 run cori_control manual_control reset
```

### Option 3: Direct Joint Control (Advanced)
```bash
# Precise control with values (-1.8 to 1.8 for head)
ros2 run cori_control manual_control head 0.5
ros2 run cori_control manual_control left_shoulder 1.0
ros2 run cori_control manual_control right_elbow -0.8
```

## Available Joints

All joints can be controlled with direct commands:

**Head:**
- `head` (-1.8 to 1.8)

**Arms:**
- `left_shoulder`, `right_shoulder` (-2.0 to 2.0)
- `left_elbow`, `right_elbow` (-1.5 to 1.5)
- `left_wrist`, `right_wrist` (-1.0 to 1.0)

**Legs:**
- `left_hip`, `right_hip` (-1.0 to 1.0)
- `left_knee`, `right_knee` (-1.5 to 0.1)
- `left_ankle`, `right_ankle` (-0.5 to 0.5)

## Interactive Control

You can also **click and drag** CORI directly in the Gazebo window!

## Example Sequence

```bash
# Source shortcuts for easier typing
source cori_control_shortcuts.sh

# Make CORI wave hello
cori-look-left
cori-wave-left
sleep 2
cori-look-right
cori-wave-right
sleep 2
cori-look-center
cori-reset

# Make CORI sit down
cori-sit
sleep 2
cori-stand
```

## Troubleshooting

**Commands not working?**
- Make sure manual control mode (option 5) is running
- Source the workspace: `source install/setup.bash`
- Check that Gazebo window is open

**Need to rebuild?**
```bash
colcon build --packages-select cori_control
source install/setup.bash
```

## Tips

- Use Tab completion after typing `ros2 run cori_control manual_control `
- Commands execute immediately (no need to hold or repeat)
- Combine with Gazebo's click-and-drag for complex poses
- Use `cori-reset` to return to neutral position anytime
