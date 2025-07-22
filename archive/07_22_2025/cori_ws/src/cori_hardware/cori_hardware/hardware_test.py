#!/usr/bin/env python3
"""
CORI Hardware Test Utility
Test the Arduino hardware bridge with various commands
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float64
import time
import sys
import threading
from typing import List

class CORIHardwareTest(Node):
    def __init__(self):
        super().__init__('cori_hardware_test')
        
        # Publishers
        self.hardware_cmd_pub = self.create_publisher(
            String, '/cori/hardware_command', 10
        )
        self.color_cmd_pub = self.create_publisher(
            String, '/cori/color_detected', 10
        )
        self.joint_cmd_pub = self.create_publisher(
            Float64, '/model/cori/joint/head_joint/cmd_pos', 10
        )
        
        # Subscribers for feedback
        self.status_sub = self.create_subscription(
            String, '/cori/hardware_status', self.status_callback, 10
        )
        self.feedback_sub = self.create_subscription(
            String, '/cori/hardware_feedback', self.feedback_callback, 10
        )
        
        # State
        self.last_feedback = []
        self.arduino_ready = False
        
        self.get_logger().info("🧪 CORI Hardware Test initialized")
        
    def status_callback(self, msg: String):
        """Handle hardware status messages"""
        status = msg.data
        self.get_logger().info(f"📊 Status: {status}")
        
        if "READY" in status:
            self.arduino_ready = True
    
    def feedback_callback(self, msg: String):
        """Handle hardware feedback messages"""
        feedback = msg.data
        self.last_feedback.append(feedback)
        
        # Keep only last 10 feedback messages
        if len(self.last_feedback) > 10:
            self.last_feedback = self.last_feedback[-10:]
        
        self.get_logger().info(f"💬 Feedback: {feedback}")
    
    def send_hardware_command(self, command: str):
        """Send hardware command"""
        msg = String()
        msg.data = command
        self.hardware_cmd_pub.publish(msg)
        self.get_logger().info(f"📤 Sent hardware command: {command}")
    
    def send_color_command(self, color: str):
        """Send color command"""
        msg = String()
        msg.data = color
        self.color_cmd_pub.publish(msg)
        self.get_logger().info(f"🎨 Sent color command: {color}")
    
    def send_joint_command(self, angle: float):
        """Send joint angle command"""
        msg = Float64()
        msg.data = angle
        self.joint_cmd_pub.publish(msg)
        self.get_logger().info(f"📐 Sent joint command: {angle:.3f} rad")
    
    def wait_for_arduino(self, timeout: float = 10.0):
        """Wait for Arduino to be ready"""
        self.get_logger().info("⏳ Waiting for Arduino to be ready...")
        
        start_time = time.time()
        while not self.arduino_ready and (time.time() - start_time) < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
        
        if self.arduino_ready:
            self.get_logger().info("✅ Arduino is ready!")
            return True
        else:
            self.get_logger().error("❌ Arduino not ready after timeout")
            return False
    
    def run_basic_test(self):
        """Run basic hardware test"""
        self.get_logger().info("🧪 Starting basic hardware test...")
        
        # Wait for Arduino
        if not self.wait_for_arduino():
            return False
        
        # Test sequence
        tests = [
            ("STATUS", "Get Arduino status"),
            ("CENTER", "Move to center position"),
            ("TEST", "Run Arduino test sequence"),
            ("CENTER", "Return to center"),
        ]
        
        for command, description in tests:
            self.get_logger().info(f"🔧 Test: {description}")
            self.send_hardware_command(command)
            time.sleep(2)  # Wait for command to execute
            rclpy.spin_once(self, timeout_sec=0.1)
        
        self.get_logger().info("✅ Basic test complete")
        return True
    
    def run_color_test(self):
        """Run color command test"""
        self.get_logger().info("🎨 Starting color command test...")
        
        if not self.wait_for_arduino():
            return False
        
        # Test all colors from the mapping
        colors = ['red', 'orange', 'yellow', 'green', 'blue', 'purple', 'grey', 'black']
        
        for color in colors:
            self.get_logger().info(f"🎨 Testing color: {color}")
            self.send_color_command(color)
            time.sleep(1.5)  # Wait for movement
            rclpy.spin_once(self, timeout_sec=0.1)
        
        # Return to center
        self.send_color_command('green')
        time.sleep(1)
        
        self.get_logger().info("✅ Color test complete")
        return True
    
    def run_joint_test(self):
        """Run joint angle test"""
        self.get_logger().info("📐 Starting joint angle test...")
        
        if not self.wait_for_arduino():
            return False
        
        # Test various angles
        angles = [-1.5, -1.0, -0.5, 0.0, 0.5, 1.0, 1.5, 0.0]
        
        for angle in angles:
            self.get_logger().info(f"📐 Testing angle: {angle:.1f} rad")
            self.send_joint_command(angle)
            time.sleep(1.5)  # Wait for movement
            rclpy.spin_once(self, timeout_sec=0.1)
        
        self.get_logger().info("✅ Joint test complete")
        return True
    
    def run_interactive_test(self):
        """Run interactive test mode"""
        self.get_logger().info("🎮 Interactive test mode")
        self.get_logger().info("Commands: color:<name>, angle:<radians>, cmd:<command>, quit")
        
        if not self.wait_for_arduino():
            return False
        
        while True:
            try:
                command = input("Test> ").strip().lower()
                
                if command == 'quit':
                    break
                elif command.startswith('color:'):
                    color = command.split(':', 1)[1]
                    self.send_color_command(color)
                elif command.startswith('angle:'):
                    try:
                        angle = float(command.split(':', 1)[1])
                        self.send_joint_command(angle)
                    except ValueError:
                        print("Invalid angle format")
                elif command.startswith('cmd:'):
                    cmd = command.split(':', 1)[1]
                    self.send_hardware_command(cmd)
                elif command == 'help':
                    print("Available commands:")
                    print("  color:red, color:blue, etc.")
                    print("  angle:0.5, angle:-1.0, etc.")
                    print("  cmd:TEST, cmd:CENTER, etc.")
                    print("  quit")
                else:
                    print("Unknown command. Type 'help' for available commands.")
                
                # Process any incoming messages
                rclpy.spin_once(self, timeout_sec=0.1)
                time.sleep(0.1)
                
            except KeyboardInterrupt:
                break
            except EOFError:
                break
        
        self.get_logger().info("✅ Interactive test complete")
        return True

def main(args=None):
    rclpy.init(args=args)
    
    try:
        test_node = CORIHardwareTest()
        
        # Start spinning in background thread
        spin_thread = threading.Thread(target=lambda: rclpy.spin(test_node), daemon=True)
        spin_thread.start()
        
        # Give time for connections to establish
        time.sleep(2)
        
        # Check command line arguments
        if len(sys.argv) > 1:
            test_type = sys.argv[1].lower()
            
            if test_type == 'basic':
                test_node.run_basic_test()
            elif test_type == 'color':
                test_node.run_color_test()
            elif test_type == 'joint':
                test_node.run_joint_test()
            elif test_type == 'interactive':
                test_node.run_interactive_test()
            else:
                print(f"Unknown test type: {test_type}")
                print("Available tests: basic, color, joint, interactive")
        else:
            # Default to interactive mode
            test_node.run_interactive_test()
        
    except KeyboardInterrupt:
        print("\n🛑 Test interrupted by user")
    except Exception as e:
        print(f"❌ Test error: {e}")
    finally:
        if 'test_node' in locals():
            test_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()