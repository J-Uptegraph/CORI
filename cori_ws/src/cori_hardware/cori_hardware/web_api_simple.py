#!/usr/bin/env python3
"""
CORI Simple Web Interface - 1X Family Edition
Just movement controls and the famous 1X question button!
"""

from fastapi import FastAPI, HTTPException
from fastapi.middleware.cors import CORSMiddleware
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
    title="CORI 1X Interface",
    description="Simple interface for CORI to join the 1X family!",
    version="1.0.0"
)

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Pydantic models
class SerialCommand(BaseModel):
    command: str
    timeout: Optional[float] = 1.0

# Global variables
serial_connection: Optional[serial.Serial] = None
connection_lock = threading.Lock()
device_status = {
    "connected": False,
    "port": None,
    "last_command": None,
    "last_response": None
}

def find_ch341_devices() -> List[str]:
    """Find CH341A USB devices"""
    devices = []
    patterns = ['/dev/ttyUSB*', '/dev/ttyACM*']
    
    for pattern in patterns:
        devices.extend(glob.glob(pattern))
    
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
            time.sleep(2)
            
            device_status.update({
                "connected": True,
                "port": port
            })
            
            logger.info(f"Connected to device at {port}")
            return True
            
    except Exception as e:
        logger.error(f"Failed to connect to {port}: {e}")
        device_status["connected"] = False
        return False

def send_command_sync(command: str, timeout: float = 1.0) -> str:
    """Send command synchronously and return response"""
    global serial_connection
    
    if not serial_connection or not serial_connection.is_open:
        raise HTTPException(status_code=503, detail="No device connected")
    
    try:
        with connection_lock:
            command_bytes = (command + '\n').encode('utf-8')
            serial_connection.write(command_bytes)
            serial_connection.flush()
            
            start_time = time.time()
            response = ""
            
            while time.time() - start_time < timeout:
                if serial_connection.in_waiting > 0:
                    line = serial_connection.readline().decode('utf-8', errors='ignore').strip()
                    if line:
                        response += line + "\n"
                        if line.startswith(("OK:", "ERROR:", "STATUS:")):
                            break
                time.sleep(0.01)
            
            device_status.update({
                "last_command": command,
                "last_response": response.strip()
            })
            
            return response.strip()
            
    except Exception as e:
        logger.error(f"Error sending command '{command}': {e}")
        raise HTTPException(status_code=500, detail=f"Command failed: {str(e)}")

# Main interface
@app.get("/", response_class=HTMLResponse)
async def root():
    """Serve the simple 1X family interface"""
    html_content = """
    <!DOCTYPE html>
    <html>
    <head>
        <title>CORI & John - Ready for 1X?</title>
        <meta name="viewport" content="width=device-width, initial-scale=1">
        <style>
            body { 
                font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif; 
                margin: 0; 
                padding: 20px;
                background: #171717;
                color: white;
                min-height: 100vh;
                display: flex;
                align-items: center;
                justify-content: center;
            }
            .container { 
                max-width: 600px; 
                text-align: center;
                background: rgba(35, 35, 35, 0.8);
                backdrop-filter: blur(15px);
                padding: 40px;
                border-radius: 25px;
                box-shadow: 0 20px 40px rgba(0,0,0,0.5);
                border: 1px solid #73aa2d;
            }
            
            h1 { 
                font-size: 42px; 
                margin-bottom: 20px;
                color: #73aa2d;
                text-shadow: 2px 2px 4px rgba(0,0,0,0.5);
            }
            
            .status { 
                padding: 15px; 
                border-radius: 15px; 
                margin: 25px 0; 
                font-size: 18px;
                font-weight: bold;
                transition: all 0.3s ease;
            }
            .status.connected { 
                background: #73aa2d; 
                animation: pulse 2s infinite;
            }
            .status.disconnected { 
                background: #444444; 
                border: 1px solid #666666;
            }
            
            @keyframes pulse {
                0% { transform: scale(1); box-shadow: 0 0 0 0 rgba(115, 170, 45, 0.7); }
                70% { transform: scale(1.02); box-shadow: 0 0 0 10px rgba(115, 170, 45, 0); }
                100% { transform: scale(1); box-shadow: 0 0 0 0 rgba(115, 170, 45, 0); }
            }
            
            button { 
                padding: 18px 25px; 
                margin: 12px; 
                border: none; 
                border-radius: 15px; 
                cursor: pointer; 
                font-size: 16px; 
                font-weight: bold;
                transition: all 0.3s ease;
                box-shadow: 0 8px 16px rgba(0,0,0,0.2);
                text-transform: uppercase;
                letter-spacing: 1px;
            }
            
            button:hover { 
                transform: translateY(-3px); 
                box-shadow: 0 12px 24px rgba(0,0,0,0.3); 
            }
            
            button:active {
                transform: translateY(-1px);
            }
            
            .btn-movement { 
                background: #73aa2d; 
                color: white; 
                min-width: 100px; 
            }
            
            .btn-1x { 
                background: #73aa2d; 
                color: white; 
                font-size: 22px;
                padding: 25px 35px;
                margin: 35px auto;
                display: block;
                max-width: 100%;
                text-shadow: 2px 2px 4px rgba(0,0,0,0.4);
                animation: glow 2s ease-in-out infinite alternate;
                border: 3px solid rgba(115, 170, 45, 0.5);
                box-shadow: 0 0 20px rgba(115, 170, 45, 0.4);
            }
            
            @keyframes glow {
                from { box-shadow: 0 0 20px rgba(115, 170, 45, 0.4); }
                to { box-shadow: 0 0 30px rgba(115, 170, 45, 0.8); }
            }
            
            .btn-connect { 
                background: #73aa2d; 
                color: white; 
                font-size: 18px;
                padding: 20px 30px;
            }
            
            .btn-center { 
                background: #73aa2d; 
                color: white; 
                min-width: 100px;
            }
            
            .control-grid { 
                display: grid; 
                grid-template-columns: 1fr 120px 1fr; 
                grid-template-rows: 120px 120px 120px;
                gap: 15px; 
                max-width: 350px; 
                margin: 30px auto;
                align-items: center;
                justify-items: center;
            }
            
            .response { 
                background: rgba(0, 0, 0, 0.6); 
                border: 1px solid #73aa2d; 
                padding: 20px; 
                border-radius: 15px; 
                font-family: 'Courier New', monospace; 
                margin: 25px 0;
                min-height: 60px;
                display: flex;
                align-items: center;
                justify-content: center;
                font-size: 16px;
                color: white;
            }
            
            .emoji { font-size: 2em; margin: 0 10px; }
        </style>
    </head>
    <body>
        <div class="container">
            <h1>🤖 CORI Control Interface</h1>
            
            <div id="status" class="status disconnected">
                <span class="emoji">🔍</span>Checking CORI connection...<span class="emoji">🔍</span>
            </div>
            
            <button class="btn-connect" onclick="autoConnect()">
                🚀 Connect to CORI
            </button>

            <h2 style="margin: 40px 0 20px 0; font-size: 24px;">Head Movement Controls</h2>
            <div class="control-grid">
                <!-- Row 1 -->
                <div></div>
                <button class="btn-movement" onclick="sendCommand('UP')">⬆️<br>UP</button>
                <div></div>
                
                <!-- Row 2 -->
                <button class="btn-movement" onclick="sendCommand('LEFT')">⬅️<br>LEFT</button>
                <button class="btn-center" onclick="sendCommand('CENTER')">🎯<br>CENTER</button>
                <button class="btn-movement" onclick="sendCommand('RIGHT')">➡️<br>RIGHT</button>
                
                <!-- Row 3 -->
                <div></div>
                <button class="btn-movement" onclick="sendCommand('DOWN')">⬇️<br>DOWN</button>
                <div></div>
            </div>

            <button class="btn-1x" onclick="join1XFamily()">
                🤝 Are John and CORI ready to join the 1X family? 🤖
            </button>

            <div class="response" id="response">
                <span class="emoji">🤖</span>Ready to control CORI...<span class="emoji">🤖</span>
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
                    
                    const responseDiv = document.getElementById('response');
                    if (data.success) {
                        responseDiv.innerHTML = `<span class="emoji">✅</span>${data.response || 'Command sent successfully!'}<span class="emoji">✅</span>`;
                    } else {
                        responseDiv.innerHTML = `<span class="emoji">❌</span>Error: ${data.error || 'Unknown error'}<span class="emoji">❌</span>`;
                    }
                    return data;
                } catch (error) {
                    document.getElementById('response').innerHTML = `<span class="emoji">⚠️</span>Connection Error: ${error.message}<span class="emoji">⚠️</span>`;
                    console.error('API call failed:', error);
                }
            }

            async function checkConnection() {
                try {
                    const data = await fetch('/api/status').then(r => r.json());
                    const statusDiv = document.getElementById('status');
                    if (data && data.connected) {
                        statusDiv.className = 'status connected';
                        statusDiv.innerHTML = `<span class="emoji">✅</span>CORI Connected (${data.port})<span class="emoji">✅</span>`;
                    } else {
                        statusDiv.className = 'status disconnected';
                        statusDiv.innerHTML = `<span class="emoji">❌</span>CORI Not Connected<span class="emoji">❌</span>`;
                    }
                } catch (error) {
                    const statusDiv = document.getElementById('status');
                    statusDiv.className = 'status disconnected';
                    statusDiv.innerHTML = `<span class="emoji">⚠️</span>Connection Check Failed<span class="emoji">⚠️</span>`;
                }
            }

            async function autoConnect() {
                document.getElementById('response').innerHTML = '<span class="emoji">🔄</span>Connecting to CORI...<span class="emoji">🔄</span>';
                await apiCall('/api/connect', { method: 'POST' });
                setTimeout(checkConnection, 1000);
            }

            async function sendCommand(cmd) {
                document.getElementById('response').innerHTML = `<span class="emoji">📡</span>Sending: ${cmd}<span class="emoji">📡</span>`;
                await apiCall('/api/command', {
                    method: 'POST',
                    body: JSON.stringify({ command: cmd })
                });
            }

            async function join1XFamily() {
                document.getElementById('response').innerHTML = '<span class="emoji">🤔</span>CORI is thinking about joining 1X...<span class="emoji">🤔</span>';
                
                // Send the thinking and nodding sequence - CORI will think and then nod YES!
                await apiCall('/api/headshake', { method: 'POST' });
            }

            // Auto-refresh status every 3 seconds
            setInterval(checkConnection, 3000);
            
            // Check connection on page load
            checkConnection();
        </script>
    </body>
    </html>
    """
    return HTMLResponse(content=html_content)

# API Endpoints
@app.get("/api/status")
async def get_status():
    """Get device connection status"""
    return {
        "connected": device_status["connected"],
        "port": device_status["port"],
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
        return {"success": False, "error": "No CH341A devices found"}
    
    for device in devices:
        if connect_to_device(device):
            return {
                "success": True,
                "port": device,
                "message": f"Connected to {device}"
            }
    
    return {"success": False, "error": "Failed to connect to any device"}

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
        return {"success": False, "error": str(e)}

@app.post("/api/headshake")
async def headshake_sequence():
    """Make CORI think and then nod YES!"""
    try:
        # Thinking and nodding sequence - CORI considers joining 1X family!
        responses = []
        
        # Step 1: Tilt head back (thinking pose)
        response = send_command_sync("UP", 1.0)
        responses.append(f"Step 1: Tilting back to think -> {response}")
        time.sleep(0.8)  # Hold the thinking pose
        
        # Step 2: Look off to the left slightly (contemplating)
        response = send_command_sync("LEFT", 1.0)
        responses.append(f"Step 2: Looking left to contemplate -> {response}")
        time.sleep(1.2)  # Take time to think
        
        # Step 3: Look back forward to center
        response = send_command_sync("CENTER", 1.0)
        responses.append(f"Step 3: Looking forward -> {response}")
        time.sleep(0.5)
        
        # Step 4: First nod down (YES!)
        response = send_command_sync("DOWN", 1.0)
        responses.append(f"Step 4: First nod down -> {response}")
        time.sleep(0.4)
        
        # Step 5: Back to center
        response = send_command_sync("CENTER", 1.0)
        responses.append(f"Step 5: Back to center -> {response}")
        time.sleep(0.3)
        
        # Step 6: Second nod down (emphatic YES!)
        response = send_command_sync("DOWN", 1.0)
        responses.append(f"Step 6: Second nod down -> {response}")
        time.sleep(0.4)
        
        # Step 7: Return to neutral center position
        response = send_command_sync("CENTER", 1.0)
        responses.append(f"Step 7: Return to center -> {response}")
        
        commands = ["UP (think)", "LEFT (contemplate)", "CENTER", "DOWN (nod 1)", "CENTER", "DOWN (nod 2)", "CENTER"]
        
        return {
            "success": True,
            "message": "CORI thought about it and nodded YES! Welcome to the 1X family! 🤖✨",
            "sequence": commands,
            "responses": responses,
            "timestamp": datetime.now().isoformat()
        }
        
    except Exception as e:
        return {"success": False, "error": str(e)}

# Startup/Shutdown events
@app.on_event("startup")
async def startup_event():
    """Auto-connect on startup"""
    logger.info("CORI 1X Interface starting up...")
    devices = find_ch341_devices()
    if devices:
        for device in devices:
            if connect_to_device(device):
                logger.info(f"Auto-connected to {device}")
                break

@app.on_event("shutdown")
async def shutdown_event():
    """Clean shutdown"""
    logger.info("CORI 1X Interface shutting down...")
    global serial_connection
    if serial_connection and serial_connection.is_open:
        try:
            send_command_sync("CENTER", timeout=1.0)
            serial_connection.close()
        except:
            pass

if __name__ == "__main__":
    import uvicorn
    print("🚀 Starting CORI 1X Interface...")
    print("🌐 Interface: http://localhost:8001")
    print("🤖 Ready to ask the big question!")
    
    uvicorn.run(app, host="0.0.0.0", port=8001, log_level="info")