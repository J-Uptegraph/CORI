#!/usr/bin/env python3
"""
CORI Real-time Web Control
Zero-latency WebSocket-based robot control
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, String
import asyncio
import websockets
import json
import threading
from typing import Dict
import time
import struct

class CORIRealtimeWebControl(Node):
    def __init__(self):
        super().__init__('cori_realtime_web_control')
        
        # WebSocket clients tracking
        self.websocket_clients = set()
        
        # ROS Publishers for direct hardware control
        self.joint_publisher = self.create_publisher(
            Float64,
            '/model/cori/joint/head_joint/cmd_pos',
            10
        )
        
        self.hardware_publisher = self.create_publisher(
            String,
            '/cori/hardware_command',
            10
        )
        
        self.color_publisher = self.create_publisher(
            String,
            '/cori/color_detected',
            10
        )
        
        # Status subscriber for feedback
        self.status_subscriber = self.create_subscription(
            String,
            '/cori/hardware_status',
            self.status_callback,
            10
        )
        
        # Command statistics
        self.command_count = 0
        self.last_command_time = time.time()
        
        # Color angle mappings for quick lookup
        self.color_angles = {
            'red': -0.62,
            'orange': -0.45,
            'yellow': -0.23,
            'green': 0.0,
            'blue': 0.23,
            'purple': 0.45,
            'grey': 0.62,
            'black': 0.76,
            'white': 0.0
        }
        
        self.get_logger().info("🚀 CORI Real-time Web Control initialized")
        
    def status_callback(self, msg: String):
        """Forward hardware status to all connected web clients"""
        asyncio.run_coroutine_threadsafe(
            self.broadcast_status(msg.data),
            self.loop
        )
    
    async def broadcast_status(self, status: str):
        """Broadcast status to all connected clients"""
        if self.websocket_clients:
            message = json.dumps({"type": "status", "data": status})
            await asyncio.gather(
                *[client.send(message) for client in self.websocket_clients.copy()],
                return_exceptions=True
            )
    
    async def broadcast_feedback(self, feedback: dict):
        """Broadcast command feedback to all clients"""
        if self.websocket_clients:
            message = json.dumps({"type": "feedback", "data": feedback})
            await asyncio.gather(
                *[client.send(message) for client in self.websocket_clients.copy()],
                return_exceptions=True
            )
    
    async def handle_client(self, websocket):
        """Handle WebSocket client connections"""
        self.websocket_clients.add(websocket)
        client_addr = websocket.remote_address
        self.get_logger().info(f"🌐 Client connected: {client_addr}")
        
        try:
            await websocket.send(json.dumps({
                "type": "welcome",
                "data": "Connected to CORI Real-time Control"
            }))
            
            async for message in websocket:
                await self.process_command(message, websocket)
                
        except websockets.exceptions.ConnectionClosed:
            self.get_logger().info(f"🔌 Client disconnected: {client_addr}")
        except Exception as e:
            self.get_logger().error(f"❌ Client error: {e}")
        finally:
            self.websocket_clients.discard(websocket)
    
    async def process_command(self, message: str, websocket):
        """Process incoming WebSocket commands with minimal latency"""
        try:
            start_time = time.time()
            
            # Parse command
            cmd = json.loads(message)
            command_type = cmd.get('type')
            data = cmd.get('data', {})
            
            self.command_count += 1
            
            # Direct ROS publishing based on command type
            if command_type == 'angle':
                angle = float(data.get('angle', 0.0))
                self.publish_angle(angle)
                
            elif command_type == 'color':
                color = data.get('color', 'green').lower()
                self.publish_color(color)
                
            elif command_type == 'hardware':
                command = data.get('command', '')
                self.publish_hardware_command(command)
                
            elif command_type == 'emergency_stop':
                self.publish_hardware_command('STOP')
                
            elif command_type == 'status':
                self.publish_hardware_command('STATUS')
                
            else:
                await websocket.send(json.dumps({
                    "type": "error",
                    "data": f"Unknown command type: {command_type}"
                }))
                return
            
            # Calculate and send latency feedback
            processing_time = (time.time() - start_time) * 1000  # ms
            
            await self.broadcast_feedback({
                "type": "command_executed",
                "command": command_type,
                "command_data": data,
                "processing_time_ms": round(processing_time, 2),
                "command_count": self.command_count,
                "timestamp": time.time(),
                "total_users": len(self.websocket_clients)
            })
            
            self.last_command_time = time.time()
            
        except json.JSONDecodeError:
            await websocket.send(json.dumps({
                "type": "error",
                "data": "Invalid JSON format"
            }))
        except Exception as e:
            self.get_logger().error(f"❌ Command processing error: {e}")
            await websocket.send(json.dumps({
                "type": "error",
                "data": str(e)
            }))
    
    def publish_angle(self, angle: float):
        """Publish angle command directly to ROS"""
        msg = Float64()
        msg.data = angle
        self.joint_publisher.publish(msg)
        self.get_logger().info(f"📡 Publishing angle: {angle:.3f} rad to /model/cori/joint/head_joint/cmd_pos")
    
    def publish_color(self, color: str):
        """Publish color command and convert to angle"""
        msg = String()
        msg.data = color
        self.color_publisher.publish(msg)
        
        # Also publish direct angle for immediate response
        if color in self.color_angles:
            self.publish_angle(self.color_angles[color])
            
        self.get_logger().debug(f"🎨 Color: {color}")
    
    def publish_hardware_command(self, command: str):
        """Publish hardware command directly"""
        msg = String()
        msg.data = command
        self.hardware_publisher.publish(msg)
        self.get_logger().debug(f"🔧 Hardware: {command}")
    
    def run_websocket_server(self):
        """Run WebSocket server in separate thread"""
        self.loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self.loop)
        
        self.get_logger().info("🌐 WebSocket server starting on ws://localhost:8767")
        
        async def start_server():
            async with websockets.serve(
                self.handle_client,
                "0.0.0.0",
                8767,
                ping_interval=None,  # Disable ping for lower latency
                ping_timeout=None,
                origins=None  # Allow all origins for GitHub Pages
            ):
                await asyncio.Future()  # Run forever
        
        self.loop.run_until_complete(start_server())
    
    def start_server(self):
        """Start WebSocket server in background thread"""
        server_thread = threading.Thread(target=self.run_websocket_server, daemon=True)
        server_thread.start()
        return server_thread

def main(args=None):
    rclpy.init(args=args)
    
    try:
        controller = CORIRealtimeWebControl()
        
        # Start WebSocket server
        server_thread = controller.start_server()
        
        controller.get_logger().info("🤖 CORI Real-time Web Control Running")
        controller.get_logger().info("🌐 WebSocket server: ws://localhost:8767")
        controller.get_logger().info("📡 Publishing to:")
        controller.get_logger().info("   /model/cori/joint/head_joint/cmd_pos")
        controller.get_logger().info("   /cori/hardware_command")
        controller.get_logger().info("   /cori/color_detected")
        controller.get_logger().info("🎮 Ready for real-time control commands")
        
        # Run ROS node
        rclpy.spin(controller)
        
    except KeyboardInterrupt:
        controller.get_logger().info("🛑 Interrupted by user")
    except Exception as e:
        if 'controller' in locals():
            controller.get_logger().error(f"❌ Controller error: {e}")
        else:
            print(f"❌ Controller error: {e}")
    finally:
        if 'controller' in locals():
            controller.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()