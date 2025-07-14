#!/usr/bin/env python3
"""
CORI Hardware Arduino Bridge
Bridges ROS topics to Arduino serial communication for physical head movement
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, String
import serial
import threading
import time
import json
from typing import Optional, Dict, Any

class CORIArduinoBridge(Node):
    def __init__(self):
        super().__init__('cori_arduino_bridge')
        
        # Serial connection
        self.serial_port: Optional[serial.Serial] = None
        self.serial_lock = threading.Lock()
        
        # Parameters
        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('serial_baudrate', 115200)
        self.declare_parameter('serial_timeout', 1.0)
        self.declare_parameter('auto_detect_port', True)
        
        self.serial_port_name = self.get_parameter('serial_port').value
        self.serial_baudrate = self.get_parameter('serial_baudrate').value
        self.serial_timeout = self.get_parameter('serial_timeout').value
        self.auto_detect_port = self.get_parameter('auto_detect_port').value
        
        # Color angle mappings (matching CORI simulation exactly)
        self.color_angles = {
            'black': 0.76,      # Far right
            'grey': 0.62,       # Right
            'gray': 0.62,       # Support both spellings
            'purple': 0.45,     # Mid-right
            'blue': 0.23,       # Slight right
            'green': 0.0,       # Center
            'yellow': -0.23,    # Slight left
            'orange': -0.45,    # Mid-left
            'red': -0.62,       # Left
            'white': 0.0,       # Center
            'light_gray': 0.62, # Same as gray
            'navy': 0.23,       # Same as blue
            'cyan': 0.23,       # Same as blue
            'pink': -0.62       # Same as red
        }
        
        # Setup ROS interfaces
        self.setup_ros_interfaces()
        
        # Connect to Arduino
        self.connect_to_arduino()
        
        # Start serial monitor thread
        self.serial_thread = threading.Thread(target=self.serial_monitor, daemon=True)
        self.serial_thread.start()
        
        self.get_logger().info("🤖 CORI Arduino Bridge initialized")
        
    def setup_ros_interfaces(self):
        """Setup ROS subscribers and publishers"""
        
        # Subscribe to Gazebo head joint commands
        self.joint_subscriber = self.create_subscription(
            Float64,
            '/model/cori/joint/head_joint/cmd_pos',
            self.joint_command_callback,
            10
        )
        
        # Subscribe to color detection commands
        self.color_subscriber = self.create_subscription(
            String,
            '/cori/color_detected',
            self.color_command_callback,
            10
        )
        
        # Subscribe to direct hardware commands
        self.hardware_subscriber = self.create_subscription(
            String,
            '/cori/hardware_command',
            self.hardware_command_callback,
            10
        )
        
        # Publisher for Arduino status
        self.status_publisher = self.create_publisher(
            String,
            '/cori/hardware_status',
            10
        )
        
        # Publisher for hardware feedback
        self.feedback_publisher = self.create_publisher(
            String,
            '/cori/hardware_feedback',
            10
        )
        
        self.get_logger().info("✅ ROS interfaces setup complete")
    
    def connect_to_arduino(self):
        """Connect to Arduino via serial"""
        
        # Try to auto-detect Arduino port
        if self.auto_detect_port:
            detected_port = self.detect_arduino_port()
            if detected_port:
                self.serial_port_name = detected_port
                self.get_logger().info(f"🔍 Auto-detected Arduino at: {detected_port}")
        
        try:
            self.serial_port = serial.Serial(
                self.serial_port_name,
                self.serial_baudrate,
                timeout=self.serial_timeout
            )
            
            # Wait for Arduino to initialize
            time.sleep(2)
            
            # Send test command to verify connection
            self.send_arduino_command("STATUS")
            
            self.get_logger().info(f"✅ Connected to Arduino at {self.serial_port_name}")
            
        except Exception as e:
            self.get_logger().error(f"❌ Failed to connect to Arduino: {e}")
            self.get_logger().error(f"   Port: {self.serial_port_name}")
            self.get_logger().error(f"   Try: sudo chmod 666 {self.serial_port_name}")
            self.serial_port = None
    
    def detect_arduino_port(self) -> Optional[str]:
        """Auto-detect Arduino serial port"""
        import glob
        
        # Common Arduino port patterns
        port_patterns = [
            '/dev/ttyUSB*',
            '/dev/ttyACM*',
            '/dev/cu.usbmodem*',
            '/dev/cu.usbserial*'
        ]
        
        for pattern in port_patterns:
            ports = glob.glob(pattern)
            for port in ports:
                try:
                    # Try to open port briefly
                    test_serial = serial.Serial(port, self.serial_baudrate, timeout=0.5)
                    test_serial.close()
                    return port
                except:
                    continue
        
        return None
    
    def joint_command_callback(self, msg: Float64):
        """Handle joint position commands from Gazebo"""
        angle_rad = msg.data
        self.get_logger().info(f"📡 Joint command: {angle_rad:.3f} rad")
        
        # Send angle command to Arduino
        command = f"ANGLE:{angle_rad:.3f}"
        self.send_arduino_command(command)
    
    def color_command_callback(self, msg: String):
        """Handle color detection commands"""
        color = msg.data.lower().strip()
        self.get_logger().info(f"🎨 Color command: {color}")
        
        # Send color command to Arduino
        command = f"COLOR:{color}"
        self.send_arduino_command(command)
        
        # Also send equivalent angle for verification
        if color in self.color_angles:
            angle = self.color_angles[color]
            self.get_logger().info(f"   → Angle: {angle:.3f} rad")
    
    def hardware_command_callback(self, msg: String):
        """Handle direct hardware commands"""
        command = msg.data.strip()
        self.get_logger().info(f"🔧 Hardware command: {command}")
        
        # Send command directly to Arduino
        self.send_arduino_command(command)
    
    def send_arduino_command(self, command: str):
        """Send command to Arduino via serial"""
        if not self.serial_port:
            self.get_logger().warn("⚠️  No Arduino connection - command ignored")
            return
        
        try:
            with self.serial_lock:
                command_bytes = (command + '\n').encode('utf-8')
                self.serial_port.write(command_bytes)
                self.serial_port.flush()
                
            self.get_logger().debug(f"📤 Sent: {command}")
            
        except Exception as e:
            self.get_logger().error(f"❌ Serial send error: {e}")
            self.reconnect_arduino()
    
    def serial_monitor(self):
        """Monitor serial port for Arduino responses"""
        while rclpy.ok():
            if not self.serial_port:
                time.sleep(1)
                continue
            
            try:
                with self.serial_lock:
                    if self.serial_port.in_waiting > 0:
                        line = self.serial_port.readline().decode('utf-8').strip()
                        if line:
                            self.process_arduino_response(line)
                            
            except Exception as e:
                self.get_logger().error(f"❌ Serial read error: {e}")
                self.reconnect_arduino()
                time.sleep(1)
            
            time.sleep(0.01)  # Small delay to prevent CPU overload
    
    def process_arduino_response(self, response: str):
        """Process responses from Arduino"""
        self.get_logger().debug(f"📥 Arduino: {response}")
        
        # Publish all responses as feedback
        feedback_msg = String()
        feedback_msg.data = response
        self.feedback_publisher.publish(feedback_msg)
        
        # Handle specific response types
        if response.startswith("STATUS:"):
            self.handle_status_response(response)
        elif response.startswith("OK:"):
            self.handle_ok_response(response)
        elif response.startswith("ERROR:"):
            self.handle_error_response(response)
        elif response.startswith("DEBUG:"):
            self.handle_debug_response(response)
        elif response == "CORI_HARDWARE_READY":
            self.get_logger().info("✅ Arduino hardware ready!")
            self.publish_status("READY")
    
    def handle_status_response(self, response: str):
        """Handle status responses from Arduino"""
        self.publish_status(response)
    
    def handle_ok_response(self, response: str):
        """Handle OK responses from Arduino"""
        self.get_logger().info(f"✅ Arduino OK: {response[3:]}")
    
    def handle_error_response(self, response: str):
        """Handle error responses from Arduino"""
        self.get_logger().error(f"❌ Arduino Error: {response[6:]}")
    
    def handle_debug_response(self, response: str):
        """Handle debug responses from Arduino"""
        self.get_logger().debug(f"🐛 Arduino Debug: {response[6:]}")
    
    def publish_status(self, status: str):
        """Publish hardware status"""
        status_msg = String()
        status_msg.data = status
        self.status_publisher.publish(status_msg)
    
    def reconnect_arduino(self):
        """Attempt to reconnect to Arduino"""
        if self.serial_port:
            try:
                self.serial_port.close()
            except:
                pass
            self.serial_port = None
        
        self.get_logger().warn("🔄 Attempting to reconnect to Arduino...")
        time.sleep(2)
        self.connect_to_arduino()
    
    def destroy_node(self):
        """Clean shutdown"""
        self.get_logger().info("🔄 Shutting down Arduino bridge...")
        
        # Send center command before shutdown
        if self.serial_port:
            self.send_arduino_command("CENTER")
            time.sleep(0.5)
        
        # Close serial port
        if self.serial_port:
            try:
                self.serial_port.close()
                self.get_logger().info("✅ Serial port closed")
            except:
                pass
        
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    
    try:
        bridge = CORIArduinoBridge()
        
        # Print helpful information
        bridge.get_logger().info("🤖 CORI Arduino Bridge Running")
        bridge.get_logger().info("📡 Subscribed to:")
        bridge.get_logger().info("   /model/cori/joint/head_joint/cmd_pos")
        bridge.get_logger().info("   /cori/color_detected")
        bridge.get_logger().info("   /cori/hardware_command")
        bridge.get_logger().info("📤 Publishing to:")
        bridge.get_logger().info("   /cori/hardware_status")
        bridge.get_logger().info("   /cori/hardware_feedback")
        bridge.get_logger().info("🔧 Test commands:")
        bridge.get_logger().info("   ros2 topic pub /cori/hardware_command std_msgs/String '{data: \"TEST\"}'")
        bridge.get_logger().info("   ros2 topic pub /cori/hardware_command std_msgs/String '{data: \"CENTER\"}'")
        bridge.get_logger().info("   ros2 topic pub /cori/color_detected std_msgs/String '{data: \"red\"}'")
        
        rclpy.spin(bridge)
        
    except KeyboardInterrupt:
        bridge.get_logger().info("🛑 Interrupted by user")
    except Exception as e:
        bridge.get_logger().error(f"❌ Bridge error: {e}")
    finally:
        if 'bridge' in locals():
            bridge.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()