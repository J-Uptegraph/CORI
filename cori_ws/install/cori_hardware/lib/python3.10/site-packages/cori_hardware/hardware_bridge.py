#!/usr/bin/env python3
"""
CORI Hardware Bridge - ESP32 to ROS/Gazebo Integration
Bridges commands between ROS topics and ESP32 hardware via serial
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String, Float64
import serial
import threading
import time
import math

class CORIHardwareBridge(Node):
    def __init__(self):
        super().__init__('cori_hardware_bridge')
        
        # Serial connection to ESP32
        self.serial_port = None
        self.connect_to_esp32()
        
        # ROS subscriptions - listen to joint commands
        self.joint_subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_callback,
            10
        )
        
        # ROS subscriptions - listen to head joint commands
        self.head_cmd_subscription = self.create_subscription(
            Float64,
            '/model/cori/joint/head_joint/cmd_pos',
            self.head_cmd_callback,
            10
        )
        
        # ROS subscriptions - listen to tilt joint commands
        self.tilt_cmd_subscription = self.create_subscription(
            Float64,
            '/model/cori/tilt_joint/cmd_pos',
            self.tilt_cmd_callback,
            10
        )
        
        # ROS subscriptions - listen to color commands  
        self.color_subscription = self.create_subscription(
            String,
            '/detected_color',
            self.color_callback,
            10
        )
        
        # ROS subscriptions - listen to direct hardware commands
        self.hardware_cmd_subscription = self.create_subscription(
            String,
            '/cori/hardware_command',
            self.hardware_command_callback,
            10
        )
        
        # Track current head positions
        self.current_head_angle = 0.0  # Pan angle
        self.current_tilt_angle = 0.0  # Tilt angle
        
        # Start serial reading thread
        self.serial_thread = threading.Thread(target=self.read_serial)
        self.serial_thread.daemon = True
        self.serial_thread.start()
        
        self.get_logger().info("🔗 CORI Hardware Bridge Active!")
        self.get_logger().info("🤖 ESP32 connected - Ready for Gazebo integration!")
        
    def connect_to_esp32(self):
        """Connect to ESP32 via serial"""
        try:
            # Try common ESP32 ports
            ports = ['/dev/ttyUSB0', '/dev/ttyACM0', '/dev/ttyUSB1']
            for port in ports:
                try:
                    # Close any existing connection first
                    if self.serial_port and self.serial_port.is_open:
                        self.serial_port.close()
                        time.sleep(0.5)
                    
                    self.serial_port = serial.Serial(
                        port, 
                        115200, 
                        timeout=1,
                        rtscts=False,  # Disable hardware flow control
                        dsrdtr=False   # Disable DTR/DSR flow control
                    )
                    time.sleep(3)  # Wait longer for ESP32 to initialize
                    self.get_logger().info(f"✅ Connected to ESP32 on {port}")
                    
                    # Clear any pending data
                    self.serial_port.flushInput()
                    self.serial_port.flushOutput()
                    
                    # Set ESP32 to Gazebo mode - send twice to ensure it's received
                    time.sleep(1)  # Wait for ESP32 menu to display
                    self.send_command("1")  # Enter Gazebo mode
                    time.sleep(1)
                    self.send_command("1")  # Send again in case first was missed
                    time.sleep(1)
                    self.get_logger().info("📟 Sent Gazebo mode command to ESP32")
                    
                    # Send servo test sequence to verify hardware
                    time.sleep(2)
                    self.get_logger().info("🔧 Testing servo with center position...")
                    self.send_command("ANGLE:0.000")  # Center position
                    time.sleep(2)
                    self.get_logger().info("🔧 Testing servo with left position...")
                    self.send_command("ANGLE:-0.500")  # Left position
                    time.sleep(2)
                    self.get_logger().info("🔧 Testing servo with right position...")
                    self.send_command("ANGLE:0.500")  # Right position
                    time.sleep(2)
                    self.get_logger().info("🔧 Returning servo to center...")
                    self.send_command("ANGLE:0.000")  # Back to center
                    self.get_logger().info("✅ Servo test sequence complete - check for physical movement!")
                    return
                except serial.SerialException as e:
                    self.get_logger().debug(f"Failed to connect to {port}: {e}")
                    continue
            
            self.get_logger().error("❌ Could not connect to ESP32")
            
        except Exception as e:
            self.get_logger().error(f"❌ ESP32 connection error: {e}")
    
    def send_command(self, command):
        """Send command to ESP32"""
        if self.serial_port and self.serial_port.is_open:
            try:
                self.serial_port.write(f"{command}\n".encode())
                self.serial_port.flush()  # Ensure data is sent immediately
                # Only log important commands, not routine angle updates
                if not command.startswith("ANGLE:") or abs(float(command.split(":")[1])) > 0.1:
                    self.get_logger().info(f"📤 Sent to ESP32: '{command}'")
            except Exception as e:
                self.get_logger().error(f"❌ Send error: {e}")
                # Try to reconnect if sending fails
                self.connect_to_esp32()
        else:
            self.get_logger().error("❌ Serial port not available, attempting reconnection...")
            self.connect_to_esp32()
    
    def read_serial(self):
        """Read responses from ESP32"""
        while rclpy.ok():
            if self.serial_port and self.serial_port.is_open:
                try:
                    if self.serial_port.in_waiting > 0:  # Only read if data available
                        response = self.serial_port.readline().decode().strip()
                        if response:
                            # Filter out routine status messages to reduce spam
                            if (not response.startswith("GLE:") and 
                                not response.startswith("STATUS:") and
                                not response.startswith("OK") and
                                len(response) > 3):  # Only log meaningful messages
                                self.get_logger().info(f"📥 ESP32 says: {response}")
                            
                            # Check if ESP32 is asking for menu choice
                            if "Enter choice" in response or "1. Gazebo Mode" in response:
                                self.get_logger().warn("⚠️ ESP32 is in MENU mode, sending '1' to enter Gazebo mode")
                                time.sleep(0.1)
                                self.send_command("1")
                except Exception as e:
                    self.get_logger().error(f"❌ Read error: {e}")
                    # Try to reconnect if reading fails
                    try:
                        if self.serial_port:
                            self.serial_port.close()
                    except:
                        pass
                    self.connect_to_esp32()
            time.sleep(0.01)
    
    def joint_callback(self, msg):
        """Handle joint state updates from Gazebo"""
        try:
            # Find head joint in the message
            if 'head_joint' in msg.name:
                idx = msg.name.index('head_joint')
                new_angle = msg.position[idx]
                
                # Only send if angle changed significantly
                if abs(new_angle - self.current_head_angle) > 0.05:
                    self.current_head_angle = new_angle
                    
                    # Send angle command to ESP32
                    self.send_command(f"ANGLE:{new_angle:.3f}")
                    # Only log significant movements to reduce spam
                    if abs(new_angle) > 0.1:
                        self.get_logger().info(f"🎯 Head angle: {new_angle:.3f} rad")
                    
        except Exception as e:
            self.get_logger().error(f"❌ Joint callback error: {e}")
    
    def head_cmd_callback(self, msg):
        """Handle head joint position commands from Gazebo"""
        try:
            angle = msg.data
            
            # Only send if angle changed significantly
            if abs(angle - self.current_head_angle) > 0.05:
                self.current_head_angle = angle
                
                # Convert radians to degrees for ESP32
                angle_deg = math.degrees(angle)
                
                # Send angle command directly to ESP32
                self.send_command(f"ANGLE:{angle:.3f}")
                # Only log significant movements to reduce spam
                if abs(angle) > 0.1:
                    self.get_logger().info(f"🎯 Head command: {angle:.3f} rad ({angle_deg:.1f}°) → ESP32")
                
        except Exception as e:
            self.get_logger().error(f"❌ Head command callback error: {e}")
    
    def tilt_cmd_callback(self, msg):
        """Handle tilt joint position commands"""
        try:
            angle = msg.data
            
            # Only send if angle changed significantly
            if abs(angle - self.current_tilt_angle) > 0.05:
                self.current_tilt_angle = angle
                
                # Convert radians to degrees for ESP32
                angle_deg = math.degrees(angle)
                
                # Send tilt command directly to ESP32
                self.send_command(f"TILT:{angle:.3f}")
                self.get_logger().info(f"🎯 Tilt command: {angle:.3f} rad ({angle_deg:.1f}°) → ESP32")
                
        except Exception as e:
            self.get_logger().error(f"❌ Tilt command callback error: {e}")
    
    def hardware_command_callback(self, msg):
        """Handle direct hardware commands from web interface"""
        try:
            command = msg.data.strip()
            self.get_logger().warn(f"🔥 DEBUG: Hardware bridge received: '{command}'")
            
            # Send command directly to ESP32
            self.send_command(command)
            self.get_logger().warn(f"🔥 DEBUG: Sent to ESP32: '{command}'")
            
        except Exception as e:
            self.get_logger().error(f"❌ Hardware command callback error: {e}")
    
    def color_callback(self, msg):
        """Handle color detection from vision system"""
        try:
            color = msg.data.lower()
            
            # Map colors to ESP32 commands
            color_map = {
                'red': 'red',
                'orange': 'orange', 
                'yellow': 'yellow',
                'green': 'green',
                'blue': 'blue',
                'purple': 'purple',
                'grey': 'grey',
                'gray': 'grey',
                'black': 'black',
                'white': 'white'
            }
            
            if color in color_map:
                esp32_color = color_map[color]
                self.send_command(esp32_color)
                self.get_logger().info(f"🎨 Color detected: {color} -> sending '{esp32_color}' to ESP32")
            
        except Exception as e:
            self.get_logger().error(f"❌ Color callback error: {e}")

def main(args=None):
    rclpy.init(args=args)
    
    try:
        bridge = CORIHardwareBridge()
        rclpy.spin(bridge)
    except KeyboardInterrupt:
        pass
    finally:
        if 'bridge' in locals() and bridge.serial_port:
            bridge.serial_port.close()
        try:
            rclpy.shutdown()
        except Exception:
            pass  # Ignore shutdown errors

if __name__ == '__main__':
    main()