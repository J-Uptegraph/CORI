# CORI Hardware Web API

A RESTful web API for controlling your CH341A USB adapter and ESP32 hardware remotely via HTTP requests.

## Features

- 🌐 **Web-based control interface** - Full GUI accessible via browser
- 🔌 **Auto-detection** of CH341A/ESP32 devices
- 🎨 **Color commands** with predefined angle mappings
- 📐 **Precise angle control** in radians
- 🕹️ **Manual controls** (left, right, up, down, center, nod)
- 📡 **ROS2 integration** via bridge service
- 📊 **Real-time status** monitoring
- 📝 **Command history** tracking
- 🚨 **Emergency stop** functionality

## Quick Start

### 1. Install Dependencies

```bash
cd cori_ws/src/cori_hardware
pip install -r requirements.txt
```

### 2. Start the Web API

```bash
# Direct Python execution
python3 cori_hardware/web_api.py

# Or via ROS2 (if integrated)
ros2 run cori_hardware web_api
```

### 3. Access the Web Interface

Open your browser to: **http://localhost:8000**

You'll see a full control panel with:
- Color command buttons
- Manual control buttons  
- Custom command input
- Angle control slider
- Real-time status display
- Command history

### 4. API Documentation

Interactive API docs: **http://localhost:8000/docs**

## API Endpoints

### Device Management

- `GET /api/status` - Get connection status
- `POST /api/connect` - Auto-connect to device
- `GET /api/devices` - List available devices
- `POST /api/emergency_stop` - Emergency stop

### Control Commands

- `POST /api/command` - Send raw command
- `POST /api/angle` - Send angle in radians
- `POST /api/color` - Send color command
- `GET /api/colors` - List available colors

### Monitoring

- `GET /api/history` - Get command history

## Example API Usage

### Send Color Command
```bash
curl -X POST "http://localhost:8000/api/color" \
     -H "Content-Type: application/json" \
     -d '{"color": "red"}'
```

### Send Angle Command
```bash
curl -X POST "http://localhost:8000/api/angle" \
     -H "Content-Type: application/json" \
     -d '{"angle_rad": 0.5}'
```

### Send Raw Command
```bash
curl -X POST "http://localhost:8000/api/command" \
     -H "Content-Type: application/json" \
     -d '{"command": "NOD"}'
```

### Check Status
```bash
curl "http://localhost:8000/api/status"
```

## ROS2 Integration

### Start ROS-Web Bridge

The bridge allows ROS2 topics to control the web API:

```bash
# Start the web API first
python3 cori_hardware/web_api.py

# In another terminal, start the ROS bridge
python3 cori_hardware/ros_web_bridge.py

# Or specify custom API URL
python3 cori_hardware/ros_web_bridge.py http://192.168.1.100:8000
```

### ROS Topics

The bridge subscribes to:
- `/model/cori/joint/head_joint/cmd_pos` (Float64) - Gazebo joint commands
- `/cori/color_detected` (String) - Color detection results
- `/cori/hardware_command` (String) - Direct hardware commands

The bridge publishes:
- `/cori/web_api_status` (String) - API connection status
- `/cori/web_api_feedback` (String) - Command responses

### Test ROS Integration

```bash
# Send color command via ROS
ros2 topic pub /cori/color_detected std_msgs/String "{data: 'red'}"

# Send hardware command via ROS  
ros2 topic pub /cori/hardware_command std_msgs/String "{data: 'NOD'}"

# Monitor status
ros2 topic echo /cori/web_api_status
```

## Supported Colors

The API supports these predefined colors with angle mappings:

- **red** (-0.62 rad) - Left
- **orange** (-0.45 rad) - Mid-left  
- **yellow** (-0.23 rad) - Slight left
- **green** (0.0 rad) - Center
- **blue** (0.23 rad) - Slight right
- **purple** (0.45 rad) - Mid-right
- **grey/gray** (0.62 rad) - Right
- **black** (0.76 rad) - Far right
- **white** (0.0 rad) - Center

## Hardware Commands

### ESP32 Commands (from your Arduino code)

- `STATUS` - Get device status
- `CENTER` - Move to center position
- `NOD` - Perform nodding sequence
- `TEST` - Run test sequence through all colors
- `ANGLE:<radians>` - Move to specific angle
- `COLOR:<color>` - Move to color position
- `LEFT`, `RIGHT`, `UP`, `DOWN` - Manual movement
- `X:<degrees>`, `Y:<degrees>` - Set specific servo angles

## Configuration

### Change API Port

```python
# In web_api.py, modify the last line:
uvicorn.run(app, host="0.0.0.0", port=8080)  # Use port 8080
```

### Change Device Settings

The API auto-detects CH341A devices, but you can modify:

```python
# In web_api.py, adjust these globals:
device_config = DeviceConfig(
    port="/dev/ttyUSB0",  # Specific port
    baudrate=115200,      # Baud rate
    timeout=1.0          # Timeout
)
```

## Troubleshooting

### No Devices Found

1. Check USB connection
2. Verify device permissions:
   ```bash
   sudo chmod 666 /dev/ttyUSB*
   # Or add user to dialout group:
   sudo usermod -a -G dialout $USER
   ```
3. Install CH340/CH341 drivers if needed

### Connection Issues

1. Check if device is already in use by another program
2. Try different USB ports
3. Restart the API server
4. Check device logs: `dmesg | grep tty`

### API Not Responding

1. Check if port 8000 is available: `netstat -tulpn | grep 8000`
2. Try different port in web_api.py
3. Check firewall settings
4. Ensure FastAPI dependencies are installed

### ROS Bridge Issues

1. Make sure web API is running first
2. Check ROS2 environment is sourced
3. Verify topic names: `ros2 topic list`
4. Check bridge logs for connection errors

## Development

### Add New Endpoints

```python
@app.post("/api/my_command")
async def my_command(data: MyModel):
    # Your custom logic here
    return {"success": True}
```

### Modify Web Interface

Edit the HTML content in the `root()` function in `web_api.py`.

### Testing

```bash
# Install test dependencies
pip install pytest httpx

# Run tests (create test files as needed)
pytest tests/
```

## Security Notes

⚠️ **This API is intended for local/trusted network use**

- No authentication implemented
- Allows direct hardware control
- Use firewall rules to restrict access
- Consider adding API keys for production use

## Integration with Your Existing Setup

This web API works alongside your existing:
- ✅ ESP32 Arduino code (connects via serial)
- ✅ ROS2 Arduino bridge (via ros_web_bridge.py)  
- ✅ Gazebo simulation (through ROS topics)
- ✅ Manual serial control (both can coexist)

The API provides an additional control method without replacing your current workflow!