#!/usr/bin/env python3
"""
CORI Hardware Web API
RESTful API for controlling CH341A/ESP32 via web interface
"""

from fastapi import FastAPI, HTTPException, BackgroundTasks
from fastapi.middleware.cors import CORSMiddleware
from fastapi.staticfiles import StaticFiles
from fastapi.responses import HTMLResponse, JSONResponse
from pydantic import BaseModel
import serial
import serial.tools.list_ports
import asyncio
import threading
import time
import json
import glob
from typing import Optional, Dict, Any, List
import logging
from datetime import datetime

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# FastAPI app
app = FastAPI(
    title="CORI Hardware API",
    description="RESTful API for controlling CH341A USB adapter and ESP32 hardware",
    version="1.0.0"
)

# Enable CORS for web frontend
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Pydantic models for API requests
class SerialCommand(BaseModel):
    command: str
    timeout: Optional[float] = 1.0

class AngleCommand(BaseModel):
    angle_rad: float

class ColorCommand(BaseModel):
    color: str

class DeviceConfig(BaseModel):
    port: str
    baudrate: int = 115200
    timeout: float = 1.0

# Global variables
serial_connection: Optional[serial.Serial] = None
connection_lock = threading.Lock()
device_config = DeviceConfig(port="/dev/ttyUSB0", baudrate=115200)
command_history: List[Dict] = []
device_status = {
    "connected": False,
    "port": None,
    "last_command": None,
    "last_response": None,
    "uptime": time.time()
}

# Color angle mappings (from your ESP32 code)
COLOR_ANGLES = {
    'black': 0.76,
    'grey': 0.62,
    'gray': 0.62,
    'purple': 0.45,
    'blue': 0.23,
    'green': 0.0,
    'yellow': -0.23,
    'orange': -0.45,
    'red': -0.62,
    'white': 0.0,
    'light_gray': 0.62,
    'navy': 0.23,
    'cyan': 0.23,
    'pink': -0.62
}

def find_ch341_devices() -> List[str]:
    """Find CH341A USB devices"""
    devices = []
    
    # Common patterns for CH341A devices
    patterns = ['/dev/ttyUSB*', '/dev/ttyACM*']
    
    for pattern in patterns:
        devices.extend(glob.glob(pattern))
    
    # Also check via pyserial
    ports = list(serial.tools.list_ports.comports())
    for port in ports:
        if any(keyword in port.description.lower() for keyword in ['ch340', 'ch341', 'usb']):
            if port.device not in devices:
                devices.append(port.device)
    
    return sorted(devices)

def connect_to_device(port: str, baudrate: int = 115200) -> bool:
    """Connect to CH341A/ESP32 device"""
    global serial_connection, device_status
    
    try:
        with connection_lock:
            if serial_connection and serial_connection.is_open:
                serial_connection.close()
            
            serial_connection = serial.Serial(port, baudrate, timeout=1.0)
            time.sleep(2)  # Wait for device initialization
            
            # Test connection
            test_response = send_command_sync("STATUS")
            
            device_status.update({
                "connected": True,
                "port": port,
                "last_command": "STATUS",
                "last_response": test_response
            })
            
            logger.info(f"Connected to device at {port}")
            return True
            
    except Exception as e:
        logger.error(f"Failed to connect to {port}: {e}")
        device_status["connected"] = False
        return False

def send_command_sync(command: str, timeout: float = 1.0) -> str:
    """Send command synchronously and return response"""
    global serial_connection, command_history
    
    if not serial_connection or not serial_connection.is_open:
        raise HTTPException(status_code=503, detail="No device connected")
    
    try:
        with connection_lock:
            # Send command
            command_bytes = (command + '\n').encode('utf-8')
            serial_connection.write(command_bytes)
            serial_connection.flush()
            
            # Read response
            start_time = time.time()
            response = ""
            
            while time.time() - start_time < timeout:
                if serial_connection.in_waiting > 0:
                    line = serial_connection.readline().decode('utf-8', errors='ignore').strip()
                    if line:
                        response += line + "\n"
                        # Stop reading if we get an OK or ERROR response
                        if line.startswith(("OK:", "ERROR:", "STATUS:")):
                            break
                time.sleep(0.01)
            
            # Log command history
            command_history.append({
                "timestamp": datetime.now().isoformat(),
                "command": command,
                "response": response.strip(),
                "duration_ms": int((time.time() - start_time) * 1000)
            })
            
            # Keep only last 100 commands
            command_history = command_history[-100:]
            
            device_status.update({
                "last_command": command,
                "last_response": response.strip()
            })
            
            return response.strip()
            
    except Exception as e:
        logger.error(f"Error sending command '{command}': {e}")
        raise HTTPException(status_code=500, detail=f"Command failed: {str(e)}")

# API Endpoints

@app.get("/", response_class=HTMLResponse)
async def root():
    """Serve main control interface"""
    html_content = """
    <!DOCTYPE html>
    <html>
    <head>
        <title>CORI Hardware API Control</title>
        <meta name="viewport" content="width=device-width, initial-scale=1">
        <style>
            body { font-family: Arial, sans-serif; margin: 20px; background: #f5f5f5; }
            .container { max-width: 1200px; margin: 0 auto; }
            .card { background: white; padding: 20px; margin: 10px 0; border-radius: 8px; box-shadow: 0 2px 4px rgba(0,0,0,0.1); }
            .grid { display: grid; grid-template-columns: repeat(auto-fit, minmax(250px, 1fr)); gap: 20px; }
            button { padding: 12px 20px; margin: 5px; border: none; border-radius: 4px; cursor: pointer; font-size: 14px; }
            .btn-primary { background: #007bff; color: white; }
            .btn-success { background: #28a745; color: white; }
            .btn-warning { background: #ffc107; color: black; }
            .btn-danger { background: #dc3545; color: white; }
            .btn-info { background: #17a2b8; color: white; }
            .color-grid { display: grid; grid-template-columns: repeat(4, 1fr); gap: 10px; }
            input, select { padding: 8px; margin: 5px; border: 1px solid #ddd; border-radius: 4px; width: 100%; }
            .status { padding: 10px; border-radius: 4px; margin: 10px 0; }
            .status.connected { background: #d4edda; border: 1px solid #c3e6cb; color: #155724; }
            .status.disconnected { background: #f8d7da; border: 1px solid #f5c6cb; color: #721c24; }
            .response { background: #f8f9fa; border: 1px solid #dee2e6; padding: 10px; border-radius: 4px; white-space: pre-wrap; font-family: monospace; max-height: 200px; overflow-y: auto; }
            h1, h2 { color: #333; }
        </style>
    </head>
    <body>
        <div class="container">
            <h1>🤖 CORI Hardware API Control Panel</h1>
            
            <div class="card">
                <h2>📡 Connection Status</h2>
                <div id="status" class="status disconnected">Checking connection...</div>
                <button class="btn-primary" onclick="checkConnection()">🔄 Refresh Status</button>
                <button class="btn-success" onclick="autoConnect()">🔗 Auto Connect</button>
            </div>

            <div class="grid">
                <div class="card">
                    <h2>🎨 Color Commands</h2>
                    <div class="color-grid">
                        <button class="btn-danger" onclick="sendColor('red')">🔴 Red</button>
                        <button class="btn-warning" onclick="sendColor('orange')">🟠 Orange</button>
                        <button class="btn-warning" onclick="sendColor('yellow')">🟡 Yellow</button>
                        <button class="btn-success" onclick="sendColor('green')">🟢 Green</button>
                        <button class="btn-primary" onclick="sendColor('blue')">🔵 Blue</button>
                        <button class="btn-primary" onclick="sendColor('purple')">🟣 Purple</button>
                        <button class="btn-info" onclick="sendColor('grey')">⚫ Grey</button>
                        <button class="btn-info" onclick="sendColor('black')">⚫ Black</button>
                    </div>
                </div>

                <div class="card">
                    <h2>🕹️ Manual Controls</h2>
                    <button class="btn-primary" onclick="sendCommand('LEFT')">⬅️ Left</button>
                    <button class="btn-primary" onclick="sendCommand('RIGHT')">➡️ Right</button>
                    <button class="btn-primary" onclick="sendCommand('UP')">⬆️ Up</button>
                    <button class="btn-primary" onclick="sendCommand('DOWN')">⬇️ Down</button>
                    <button class="btn-success" onclick="sendCommand('CENTER')">🎯 Center</button>
                    <button class="btn-info" onclick="sendCommand('NOD')">👋 Nod</button>
                </div>

                <div class="card">
                    <h2>🎛️ Custom Commands</h2>
                    <input type="text" id="customCommand" placeholder="Enter command (e.g., ANGLE:0.5)">
                    <button class="btn-primary" onclick="sendCustomCommand()">📤 Send Command</button>
                    <br><br>
                    <label>Angle (radians):</label>
                    <input type="number" id="angleInput" step="0.1" min="-1.8" max="1.8" value="0">
                    <button class="btn-primary" onclick="sendAngle()">📐 Send Angle</button>
                </div>

                <div class="card">
                    <h2>🔧 System Commands</h2>
                    <button class="btn-info" onclick="sendCommand('STATUS')">📊 Status</button>
                    <button class="btn-warning" onclick="sendCommand('TEST')">🧪 Test Sequence</button>
                    <button class="btn-danger" onclick="sendCommand('MENU')">📋 Menu Mode</button>
                </div>
            </div>

            <div class="card">
                <h2>📝 Response</h2>
                <div id="response" class="response">Ready...</div>
            </div>

            <div class="card">
                <h2>📈 Command History</h2>
                <button class="btn-info" onclick="loadHistory()">🔄 Load History</button>
                <div id="history" class="response"></div>
            </div>
        </div>

        <script>
            async function apiCall(endpoint, options = {}) {
                try {
                    const response = await fetch(endpoint, {
                        headers: { 'Content-Type': 'application/json' },
                        ...options
                    });
                    const data = await response.json();
                    document.getElementById('response').textContent = JSON.stringify(data, null, 2);
                    return data;
                } catch (error) {
                    document.getElementById('response').textContent = 'Error: ' + error.message;
                    console.error('API call failed:', error);
                }
            }

            async function checkConnection() {
                const data = await apiCall('/api/status');
                const statusDiv = document.getElementById('status');
                if (data && data.connected) {
                    statusDiv.className = 'status connected';
                    statusDiv.textContent = `✅ Connected to ${data.port}`;
                } else {
                    statusDiv.className = 'status disconnected';
                    statusDiv.textContent = '❌ Not connected';
                }
            }

            async function autoConnect() {
                await apiCall('/api/connect', { method: 'POST' });
                checkConnection();
            }

            async function sendCommand(cmd) {
                await apiCall('/api/command', {
                    method: 'POST',
                    body: JSON.stringify({ command: cmd })
                });
            }

            async function sendColor(color) {
                await apiCall('/api/color', {
                    method: 'POST',
                    body: JSON.stringify({ color: color })
                });
            }

            async function sendAngle() {
                const angle = parseFloat(document.getElementById('angleInput').value);
                await apiCall('/api/angle', {
                    method: 'POST',
                    body: JSON.stringify({ angle_rad: angle })
                });
            }

            async function sendCustomCommand() {
                const cmd = document.getElementById('customCommand').value;
                if (cmd) {
                    await sendCommand(cmd);
                }
            }

            async function loadHistory() {
                const data = await apiCall('/api/history');
                document.getElementById('history').textContent = JSON.stringify(data, null, 2);
            }

            // Auto-refresh status every 5 seconds
            setInterval(checkConnection, 5000);
            
            // Check connection on page load
            checkConnection();
        </script>
    </body>
    </html>
    """
    return HTMLResponse(content=html_content)

@app.get("/api/status")
async def get_status():
    """Get device connection status"""
    return {
        "connected": device_status["connected"],
        "port": device_status["port"],
        "uptime_seconds": int(time.time() - device_status["uptime"]),
        "last_command": device_status["last_command"],
        "last_response": device_status["last_response"],
        "available_devices": find_ch341_devices(),
        "timestamp": datetime.now().isoformat()
    }

@app.post("/api/connect")
async def connect_device():
    """Auto-connect to first available CH341A device"""
    devices = find_ch341_devices()
    
    if not devices:
        raise HTTPException(status_code=404, detail="No CH341A devices found")
    
    for device in devices:
        if connect_to_device(device, device_config.baudrate):
            return {
                "success": True,
                "port": device,
                "message": f"Connected to {device}"
            }
    
    raise HTTPException(status_code=503, detail="Failed to connect to any device")

@app.post("/api/command")
async def send_command(cmd: SerialCommand):
    """Send raw command to device"""
    try:
        response = send_command_sync(cmd.command, cmd.timeout)
        return {
            "success": True,
            "command": cmd.command,
            "response": response,
            "timestamp": datetime.now().isoformat()
        }
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))

@app.post("/api/angle")
async def send_angle(angle_cmd: AngleCommand):
    """Send angle command to device"""
    command = f"ANGLE:{angle_cmd.angle_rad:.3f}"
    response = send_command_sync(command)
    return {
        "success": True,
        "angle_rad": angle_cmd.angle_rad,
        "command": command,
        "response": response,
        "timestamp": datetime.now().isoformat()
    }

@app.post("/api/color")
async def send_color(color_cmd: ColorCommand):
    """Send color command to device"""
    color = color_cmd.color.lower()
    
    if color not in COLOR_ANGLES:
        raise HTTPException(status_code=400, detail=f"Unknown color: {color}")
    
    # Send both color and equivalent angle
    command = f"COLOR:{color}"
    response = send_command_sync(command)
    
    return {
        "success": True,
        "color": color,
        "angle_rad": COLOR_ANGLES[color],
        "command": command,
        "response": response,
        "timestamp": datetime.now().isoformat()
    }

@app.get("/api/colors")
async def get_available_colors():
    """Get list of available colors and their angles"""
    return {
        "colors": COLOR_ANGLES,
        "count": len(COLOR_ANGLES)
    }

@app.get("/api/history")
async def get_command_history():
    """Get command history"""
    return {
        "history": command_history[-20:],  # Last 20 commands
        "total_commands": len(command_history)
    }

@app.post("/api/emergency_stop")
async def emergency_stop():
    """Emergency stop - center device and disconnect"""
    try:
        if serial_connection and serial_connection.is_open:
            send_command_sync("CENTER", timeout=0.5)
            with connection_lock:
                serial_connection.close()
        
        device_status["connected"] = False
        return {"success": True, "message": "Emergency stop executed"}
        
    except Exception as e:
        return {"success": False, "error": str(e)}

@app.get("/api/devices")
async def list_devices():
    """List available CH341A devices"""
    devices = find_ch341_devices()
    
    detailed_devices = []
    ports = list(serial.tools.list_ports.comports())
    
    for device in devices:
        device_info = {"port": device, "description": "Unknown"}
        for port in ports:
            if port.device == device:
                device_info.update({
                    "description": port.description,
                    "manufacturer": port.manufacturer,
                    "hardware_id": port.hwid
                })
                break
        detailed_devices.append(device_info)
    
    return {"devices": detailed_devices, "count": len(detailed_devices)}

# Startup/Shutdown events
@app.on_event("startup")
async def startup_event():
    """Auto-connect on startup"""
    logger.info("CORI Hardware API starting up...")
    
    # Try to auto-connect
    devices = find_ch341_devices()
    if devices:
        logger.info(f"Found devices: {devices}")
        for device in devices:
            if connect_to_device(device):
                logger.info(f"Auto-connected to {device}")
                break
    else:
        logger.warning("No CH341A devices found on startup")

@app.on_event("shutdown")
async def shutdown_event():
    """Clean shutdown"""
    logger.info("CORI Hardware API shutting down...")
    
    global serial_connection
    if serial_connection and serial_connection.is_open:
        try:
            send_command_sync("CENTER", timeout=1.0)
            serial_connection.close()
            logger.info("Device centered and disconnected")
        except:
            pass

if __name__ == "__main__":
    import uvicorn
    print("🚀 Starting CORI Hardware API...")
    print("🌐 Web interface: http://localhost:8000")
    print("📖 API docs: http://localhost:8000/docs")
    
    uvicorn.run(app, host="0.0.0.0", port=8000, log_level="info")