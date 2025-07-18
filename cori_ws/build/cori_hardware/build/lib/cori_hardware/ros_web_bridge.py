#!/usr/bin/env python3
"""
CORI ROS-Web Bridge
Integrates the Web API with existing ROS2 Arduino bridge
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, String
import requests
import threading
import time
import json
from typing import Optional

class CORIRosWebBridge(Node):
    def __init__(self, web_api_url: str = "http://localhost:8000"):
        super().__init__('cori_ros_web_bridge')
        
        self.web_api_url = web_api_url
        self.last_command_time = time.time()
        
        # Setup ROS subscribers (same as arduino_bridge.py)
        self.joint_subscriber = self.create_subscription(
            Float64,
            '/model/cori/joint/head_joint/cmd_pos',
            self.joint_command_callback,
            10
        )
        
        self.color_subscriber = self.create_subscription(
            String,
            '/cori/color_detected',
            self.color_command_callback,
            10
        )
        
        self.hardware_subscriber = self.create_subscription(
            String,
            '/cori/hardware_command',
            self.hardware_command_callback,
            10
        )
        
        # Publishers for status/feedback
        self.status_publisher = self.create_publisher(
            String,
            '/cori/web_api_status',
            10
        )
        
        self.feedback_publisher = self.create_publisher(
            String,
            '/cori/web_api_feedback',
            10
        )
        
        # Status monitoring timer
        self.status_timer = self.create_timer(5.0, self.check_api_status)
        
        self.get_logger().info(f"🌐 ROS-Web Bridge initialized, connecting to API at {web_api_url}")
        
    def call_web_api(self, endpoint: str, method: str = "GET", data: dict = None) -> Optional[dict]:
        """Make HTTP call to web API"""
        try:
            url = f"{self.web_api_url}{endpoint}"
            
            if method == "GET":
                response = requests.get(url, timeout=5.0)
            elif method == "POST":
                response = requests.post(url, json=data, timeout=5.0)
            else:
                self.get_logger().error(f"Unsupported HTTP method: {method}")
                return None
            
            if response.status_code == 200:
                return response.json()
            else:
                self.get_logger().error(f"API call failed: {response.status_code} - {response.text}")
                return None
                
        except requests.exceptions.RequestException as e:
            self.get_logger().error(f"API connection error: {e}")
            return None
        except Exception as e:
            self.get_logger().error(f"Unexpected error in API call: {e}")
            return None
    
    def joint_command_callback(self, msg: Float64):
        """Handle joint position commands from Gazebo"""
        angle_rad = msg.data
        self.get_logger().info(f"📡 Joint command: {angle_rad:.3f} rad")
        
        # Send to web API
        result = self.call_web_api("/api/angle", "POST", {"angle_rad": angle_rad})
        
        if result:
            self.get_logger().info(f"✅ Web API response: {result.get('response', 'No response')}")
            self.publish_feedback(f"ANGLE_OK:{angle_rad:.3f}:{result.get('response', '')}")
        else:
            self.get_logger().error(f"❌ Failed to send angle command via web API")
            self.publish_feedback(f"ANGLE_ERROR:{angle_rad:.3f}")
        
        self.last_command_time = time.time()
    
    def color_command_callback(self, msg: String):
        """Handle color detection commands"""
        color = msg.data.lower().strip()
        self.get_logger().info(f"🎨 Color command: {color}")
        
        # Send to web API
        result = self.call_web_api("/api/color", "POST", {"color": color})
        
        if result:
            self.get_logger().info(f"✅ Web API response: {result.get('response', 'No response')}")
            self.publish_feedback(f"COLOR_OK:{color}:{result.get('response', '')}")
        else:
            self.get_logger().error(f"❌ Failed to send color command via web API")
            self.publish_feedback(f"COLOR_ERROR:{color}")
        
        self.last_command_time = time.time()
    
    def hardware_command_callback(self, msg: String):
        """Handle direct hardware commands"""
        command = msg.data.strip()
        self.get_logger().info(f"🔧 Hardware command: {command}")
        
        # Send to web API
        result = self.call_web_api("/api/command", "POST", {"command": command})
        
        if result:
            self.get_logger().info(f"✅ Web API response: {result.get('response', 'No response')}")
            self.publish_feedback(f"COMMAND_OK:{command}:{result.get('response', '')}")
        else:
            self.get_logger().error(f"❌ Failed to send hardware command via web API")
            self.publish_feedback(f"COMMAND_ERROR:{command}")
        
        self.last_command_time = time.time()
    
    def check_api_status(self):
        """Periodically check API status"""
        status = self.call_web_api("/api/status")
        
        if status:
            # Publish status
            status_msg = String()
            if status.get('connected', False):
                status_msg.data = f"WEB_API_CONNECTED:{status.get('port', 'unknown')}"
                self.get_logger().debug(f"📊 API Status: Connected to {status.get('port', 'unknown')}")
            else:
                status_msg.data = "WEB_API_DISCONNECTED"
                self.get_logger().warn("⚠️  API Status: Device not connected")
            
            self.status_publisher.publish(status_msg)
        else:
            # API unreachable
            status_msg = String()
            status_msg.data = "WEB_API_UNREACHABLE"
            self.status_publisher.publish(status_msg)
            self.get_logger().error("❌ Web API unreachable")
    
    def publish_feedback(self, feedback: str):
        """Publish feedback message"""
        feedback_msg = String()
        feedback_msg.data = feedback
        self.feedback_publisher.publish(feedback_msg)

def main(args=None):
    rclpy.init(args=args)
    
    try:
        # Allow custom API URL via command line
        import sys
        api_url = "http://localhost:8000"
        if len(sys.argv) > 1:
            api_url = sys.argv[1]
        
        bridge = CORIRosWebBridge(api_url)
        
        bridge.get_logger().info("🤖 CORI ROS-Web Bridge Running")
        bridge.get_logger().info(f"🌐 Web API URL: {api_url}")
        bridge.get_logger().info("📡 Subscribed to:")
        bridge.get_logger().info("   /model/cori/joint/head_joint/cmd_pos")
        bridge.get_logger().info("   /cori/color_detected")
        bridge.get_logger().info("   /cori/hardware_command")
        bridge.get_logger().info("📤 Publishing to:")
        bridge.get_logger().info("   /cori/web_api_status")
        bridge.get_logger().info("   /cori/web_api_feedback")
        bridge.get_logger().info("🔧 Test commands:")
        bridge.get_logger().info("   ros2 topic pub /cori/hardware_command std_msgs/String '{data: \"STATUS\"}'")
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