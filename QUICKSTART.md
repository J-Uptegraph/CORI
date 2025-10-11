# CORI Quickstart Guide

Get C.O.R.I. up and running in minutes! This guide covers system requirements, installation, and first-time setup.

---

## System Requirements

- **Operating System:** Ubuntu 22.04 LTS
- **ROS Version:** ROS 2 Humble
- **Simulator:** Gazebo Harmonic (gz-sim)
- **Python:** 3.10+
- **Hardware (Optional):** ESP32 microcontroller with servos

---

## Prerequisites

Before starting, ensure you have the following installed:

### 1. ROS 2 Humble

```bash
# Add ROS 2 repository
sudo apt update && sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS 2 Humble
sudo apt update
sudo apt install ros-humble-desktop -y
sudo apt install ros-dev-tools -y

# Source ROS 2 environment
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### 2. Gazebo Harmonic

```bash
# Install Gazebo Harmonic
sudo apt-get update
sudo apt-get install ros-humble-ros-gz -y
```

### 3. System Dependencies

```bash
# Install required system packages
sudo apt install -y \
    python3-pip \
    python3-colcon-common-extensions \
    python3-opencv \
    git \
    bc \
    curl \
    wget \
    net-tools \
    nginx
```

### 4. Python Dependencies

```bash
# Install Python packages
pip3 install \
    opencv-python \
    numpy \
    fastapi \
    uvicorn[standard] \
    pyserial \
    requests \
    pydantic \
    python-multipart \
    websockets
```

---

## Installation

### Step 1: Clone the Repository

```bash
cd ~/
mkdir -p Workspaces/Robotics/Projects
cd Workspaces/Robotics/Projects
git clone https://github.com/yourusername/CORI.git
cd CORI/cori_ws
```

### Step 2: Build the Workspace

The automated build script handles everything:

```bash
chmod +x build.bash
./build.bash
```

This script will:
- Clean previous builds
- Build all ROS 2 packages using `colcon`
- Source the workspace
- Add CORI to your `.bashrc` for persistent access
- Display the interactive program menu

---

## First Run

After building, the script automatically presents a menu with options:

```
╭──────────────────────────────────────────────────────────────────────╮
│                        SELECT A PROGRAM:                             │
│                                                                      │
│ 1) 🚀 Full System                                                    │
│ 2) 🎮 Gazebo Simulation                                              │
│ 3) 🧺 Laundry Sorting Assistant                                      │
│ 4) 📷 Webcam Color Detection                                         │
│ 5) 💬 Terminal Motion Control                                        │
│ 6) 🔗 ESP32 Hardware Bridge                                          │
│ 7) 🌐 Real-time Web Control (Legacy)                                 │
│ 8) 🧹 Kill All ROS Processes                                         │
│ 9) 🔗 System Test                                                    │
│ 10) 🚪 Exit                                                          │
│ 11) 🔐 Secure Web Control (Nginx)                                    │
│                                                                      │
╰──────────────────────────────────────────────────────────────────────╯
```

### Recommended First Steps

#### Option 2: Gazebo Simulation Only
Perfect for verifying installation:
```bash
# Enter choice: 2
```
This launches CORI in Gazebo Harmonic. You should see the robot model with the simulated environment.

#### Option 9: System Test
Tests camera + simulation integration:
```bash
# Enter choice: 9
```
Launches Gazebo, starts webcam, and tests color detection. Hold colored objects in front of your camera to test!

#### Option 5: Terminal Motion Control
Interactive terminal control:
```bash
# Enter choice: 5
```
Type colors (`red`, `blue`, `green`) to make CORI look at corresponding objects in the simulation.

---

## Running Modes Explained

### 1. Full System
Complete integration with Gazebo simulation, camera detection, and adaptive learning database.

**Use case:** Full CORI experience with camera-driven head movement.

### 2. Gazebo Simulation
Launches only the Gazebo environment with CORI's robot model.

**Use case:** Visualization and environment testing.

### 3. Laundry Sorting Assistant
Interactive terminal-based laundry sorting with adaptive learning.

**Use case:** Test CORI's learning algorithms without hardware/simulation overhead.

**Example interaction:**
```
You: red shirt
CORI: I think this goes in COLORS (75% confident). Do you agree? (yes/no)
```

### 4. Webcam Color Detection
Real-time color detection using your physical webcam with live GUI display.

**Use case:** Computer vision testing and calibration.

### 5. Terminal Motion Control
Control CORI's head position by typing color names or movement commands in the terminal.

**Commands:**
- `red`, `blue`, `green`, `yellow` - Look at colored objects
- `left`, `right` - Manual head positioning
- `quit` - Exit

### 6. ESP32 Hardware Bridge
Connects physical ESP32 hardware to ROS 2 for real servo control synchronized with simulation.

**Requirements:** ESP32 with servos connected.

### 7. Real-time Web Control (Legacy)
Web-based control interface with WebSocket support (direct port access).

**Access:** `http://localhost:8091`

### 8. Kill All ROS Processes
Emergency cleanup - stops all running ROS 2 processes, Gazebo, and web servers.

### 9. System Test
Comprehensive test: Gazebo + webcam + color detection + GUI display.

### 10. Exit
Safely exits the build script.

### 11. Secure Web Control (Nginx)
Production-ready web interface with nginx reverse proxy and authentication.

**Setup required:** Configure nginx and set environment variables:
```bash
export CORI_USERNAME="your_username"
export CORI_PASSWORD="your_password"
```

---

## Quick Commands Reference

### Manual Launch (after initial build)

```bash
# Source workspace
cd ~/Workspaces/Robotics/Projects/CORI/cori_ws
source install/setup.bash

# Launch Gazebo simulation
ros2 launch cori_description spawn_cori_ignition.launch.py

# Launch webcam color detection
ros2 launch cori_vision camera_only.launch.py

# Run terminal motion control
python3 install/cori_control/lib/python3.10/site-packages/cori_control/interactive_control.py
```

### Rebuild Workspace

```bash
cd ~/Workspaces/Robotics/Projects/CORI/cori_ws
./build.bash
```

### Kill Stuck Processes

```bash
# Run the build script and select option 8
./build.bash
# Then select: 8
```

---

## Troubleshooting

### Issue: Gazebo won't start

**Solution:**
```bash
# Check if Gazebo processes are running
ps aux | grep gz

# Kill existing processes
pkill -f "gz sim"
pkill -f "ign gazebo"

# Try again
./build.bash
```

### Issue: Webcam not detected

**Solution:**
```bash
# List available camera devices
ls /dev/video*

# Check permissions
sudo usermod -a -G video $USER
# Log out and back in for changes to take effect
```

### Issue: Build fails with missing dependencies

**Solution:**
```bash
# Install all ROS 2 dependencies
cd ~/Workspaces/Robotics/Projects/CORI/cori_ws
rosdep install --from-paths src --ignore-src -r -y

# Rebuild
./build.bash
```

### Issue: Python module not found

**Solution:**
```bash
# Ensure workspace is sourced
source install/setup.bash

# Verify PYTHONPATH includes workspace
echo $PYTHONPATH

# If missing, manually add to ~/.bashrc:
echo "source ~/Workspaces/Robotics/Projects/CORI/cori_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### Issue: Port already in use (8000, 8091, 8767)

**Solution:**
```bash
# Find and kill process using the port
sudo lsof -i :8091
sudo kill -9 <PID>

# Or use the build script's cleanup option
./build.bash
# Select option: 8
```

---

## Next Steps

Once you have CORI running:

1. **Explore the demo modes** - Try each numbered option to understand capabilities
2. **Review the documentation** - Check `docs/` for detailed technical documentation
3. **Read update logs** - See `docs/project_updates/` for version histories
4. **Modify parameters** - Customize CORI's behavior in `src/` package configurations
5. **Add your own code** - Extend CORI with new packages in `src/`

---

## Support

For issues, questions, or contributions:

- **Documentation:** `/docs` directory
- **GitHub Issues:** [Report bugs and request features]
- **Email:** jwuptegraph@gmail.com
- **Portfolio:** [juptegraph.dev](https://juptegraph.dev)

---

## Quick Reference Card

| Task | Command |
|------|---------|
| Build workspace | `./build.bash` |
| Launch Gazebo only | Select option `2` |
| Test camera + vision | Select option `9` |
| Interactive terminal control | Select option `5` |
| Kill all processes | Select option `8` |
| Web control (secure) | Select option `11` |
| Source workspace | `source install/setup.bash` |

---

**Welcome to C.O.R.I.** - Built to function, designed to matter.
