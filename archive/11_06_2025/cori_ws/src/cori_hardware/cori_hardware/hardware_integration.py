#!/usr/bin/env python3
"""
CORI Hardware Integration Module
Provides easy integration with existing CORI vision and control systems
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, String
from sensor_msgs.msg import JointState
import threading
import time
from typing import Dict, Optional, Callable

class CORIHardwareIntegration(Node):
    """
    High-level hardware integration for CORI
    Automatically bridges vision system to physical hardware
    """
    
    def __init__(self, node_name: str = 'cori_hardware_integration'):
        super().__init__(node_name)
        
        # Integration parameters
        self.declare_parameter('enable_hardware', True)
        self.declare_parameter('mirror_gazebo', True)
        self.declare_parameter('mirror_vision', True)
        self.declare_parameter('enable_feedback', True)
        
        self.enable_hardware = self.get_parameter('enable_hardware').value
        self.mirror_gazebo = self.get_parameter('mirror_gazebo').value
        self.mirror_vision = self.get_parameter('mirror_vision').value
        self.enable_feedback = self.get_parameter('enable_feedback').value
        
        # Color angle mappings (from CORI vision system)
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
        
        # State tracking
        self.last_angle = 0.0
        self.last_color = "unknown"
        self.hardware_ready = False
        
        # Callback registration
        self.color_callbacks = []
        self.angle_callbacks = []
        
        # Setup ROS interfaces
        self.setup_ros_interfaces()
        
        self.get_logger().info(f"🔗 CORI Hardware Integration initialized")
        self.get_logger().info(f"   Hardware enabled: {self.enable_hardware}")
        self.get_logger().info(f"   Mirror Gazebo: {self.mirror_gazebo}")
        self.get_logger().info(f"   Mirror Vision: {self.mirror_vision}")
    
    def setup_ros_interfaces(self):
        """Setup ROS publishers and subscribers"""
        
        # Hardware command publisher
        self.hardware_cmd_pub = self.create_publisher(
            String, '/cori/hardware_command', 10
        )
        
        # Color detection publisher (for hardware)
        self.color_cmd_pub = self.create_publisher(
            String, '/cori/color_detected', 10
        )
        
        # Joint command publisher (for hardware)
        self.joint_cmd_pub = self.create_publisher(
            Float64, '/model/cori/joint/head_joint/cmd_pos', 10
        )
        
        # Subscribe to existing CORI topics
        if self.mirror_gazebo:
            self.gazebo_sub = self.create_subscription(
                Float64, '/model/cori/joint/head_joint/cmd_pos',
                self.gazebo_joint_callback, 10
            )
        
        if self.mirror_vision:
            # Subscribe to color detection from vision system
            self.vision_color_sub = self.create_subscription(
                String, '/cori/color_detected',
                self.vision_color_callback, 10
            )
            
            # Subscribe to joint states if available
            self.joint_state_sub = self.create_subscription(
                JointState, '/joint_states',
                self.joint_state_callback, 10
            )
        
        # Hardware feedback subscription
        if self.enable_feedback:
            self.hardware_status_sub = self.create_subscription(
                String, '/cori/hardware_status',
                self.hardware_status_callback, 10
            )
            
            self.hardware_feedback_sub = self.create_subscription(
                String, '/cori/hardware_feedback',
                self.hardware_feedback_callback, 10
            )
    
    def gazebo_joint_callback(self, msg: Float64):
        """Handle Gazebo joint commands"""
        if not self.enable_hardware:
            return
        
        angle = msg.data
        self.last_angle = angle
        
        # Forward to hardware if it's a new angle
        if abs(angle - self.last_angle) > 0.01:  # 0.01 rad threshold
            self.get_logger().debug(f"🎯 Mirroring Gazebo angle: {angle:.3f} rad")
            self.send_angle_to_hardware(angle)
            
            # Notify callbacks
            for callback in self.angle_callbacks:
                callback(angle)
    
    def vision_color_callback(self, msg: String):
        """Handle vision system color detection"""
        if not self.enable_hardware:
            return
        
        color = msg.data.lower().strip()
        
        # Only process if it's a new color
        if color != self.last_color:
            self.last_color = color
            self.get_logger().info(f"🎨 Mirroring vision color: {color}")
            
            # Send color to hardware
            self.send_color_to_hardware(color)
            
            # Notify callbacks
            for callback in self.color_callbacks:
                callback(color)
    
    def joint_state_callback(self, msg: JointState):
        """Handle joint state messages"""
        if not self.enable_hardware:
            return
        
        # Look for head joint in joint states
        if 'head_joint' in msg.name:
            idx = msg.name.index('head_joint')
            if idx < len(msg.position):
                angle = msg.position[idx]
                
                # Forward to hardware if significantly different
                if abs(angle - self.last_angle) > 0.02:  # 0.02 rad threshold
                    self.last_angle = angle
                    self.get_logger().debug(f"📐 Joint state angle: {angle:.3f} rad")
                    self.send_angle_to_hardware(angle)
    
    def hardware_status_callback(self, msg: String):
        """Handle hardware status updates"""
        status = msg.data
        
        if "READY" in status:
            if not self.hardware_ready:
                self.hardware_ready = True
                self.get_logger().info("✅ Hardware is ready!")
                
                # Send initial position
                self.send_angle_to_hardware(0.0)  # Center position
        
        self.get_logger().debug(f"📊 Hardware status: {status}")
    
    def hardware_feedback_callback(self, msg: String):
        """Handle hardware feedback"""
        feedback = msg.data
        self.get_logger().debug(f"💬 Hardware feedback: {feedback}")
        
        # Parse feedback for useful information
        if feedback.startswith("OK:"):
            # Hardware confirmed command
            pass
        elif feedback.startswith("ERROR:"):
            # Hardware error
            self.get_logger().warn(f"⚠️ Hardware error: {feedback}")
    
    def send_angle_to_hardware(self, angle: float):
        """Send angle command to hardware"""
        if not self.enable_hardware:
            return
        
        # Send via joint command topic (hardware bridge will pick this up)
        msg = Float64()
        msg.data = angle
        self.joint_cmd_pub.publish(msg)
        
        self.get_logger().debug(f"📡 Sent angle to hardware: {angle:.3f} rad")
    
    def send_color_to_hardware(self, color: str):
        """Send color command to hardware"""
        if not self.enable_hardware:
            return
        
        # Send via color detection topic
        msg = String()
        msg.data = color
        self.color_cmd_pub.publish(msg)
        
        self.get_logger().debug(f"🎨 Sent color to hardware: {color}")
    
    def send_hardware_command(self, command: str):
        """Send direct hardware command"""
        if not self.enable_hardware:
            return
        
        msg = String()
        msg.data = command
        self.hardware_cmd_pub.publish(msg)
        
        self.get_logger().info(f"🔧 Sent hardware command: {command}")
    
    def move_to_color(self, color: str):
        """Move head to color position (for manual control)"""
        if color in self.color_angles:
            angle = self.color_angles[color]
            self.send_angle_to_hardware(angle)
            self.get_logger().info(f"🎯 Moving to {color} position: {angle:.3f} rad")
        else:
            self.get_logger().warn(f"⚠️ Unknown color: {color}")
    
    def move_to_angle(self, angle: float):
        """Move head to specific angle (for manual control)"""
        # Clamp angle to safe range
        angle = max(-1.8, min(1.8, angle))
        self.send_angle_to_hardware(angle)
        self.get_logger().info(f"📐 Moving to angle: {angle:.3f} rad")
    
    def center_head(self):
        """Center the head"""
        self.move_to_angle(0.0)
    
    def test_hardware(self):
        """Test hardware functionality"""
        self.get_logger().info("🧪 Testing hardware...")
        self.send_hardware_command("TEST")
    
    def get_hardware_status(self):
        """Request hardware status"""
        self.send_hardware_command("STATUS")
    
    def register_color_callback(self, callback: Callable[[str], None]):
        """Register callback for color detection events"""
        self.color_callbacks.append(callback)
    
    def register_angle_callback(self, callback: Callable[[float], None]):
        """Register callback for angle change events"""
        self.angle_callbacks.append(callback)
    
    def wait_for_hardware(self, timeout: float = 10.0) -> bool:
        """Wait for hardware to be ready"""
        start_time = time.time()
        
        while not self.hardware_ready and (time.time() - start_time) < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
        
        return self.hardware_ready

# Convenience functions for easy integration

def create_hardware_integration(
    enable_hardware: bool = True,
    mirror_gazebo: bool = True,
    mirror_vision: bool = True
) -> CORIHardwareIntegration:
    """Create and configure hardware integration"""
    
    integration = CORIHardwareIntegration()
    
    # Override parameters
    integration.enable_hardware = enable_hardware
    integration.mirror_gazebo = mirror_gazebo
    integration.mirror_vision = mirror_vision
    
    return integration

def add_hardware_to_existing_node(node: Node) -> CORIHardwareIntegration:
    """Add hardware integration to existing ROS node"""
    
    # Create integration instance
    integration = CORIHardwareIntegration('hardware_integration_addon')
    
    # Start spinning in background thread
    def spin_integration():
        while rclpy.ok():
            rclpy.spin_once(integration, timeout_sec=0.1)
    
    spin_thread = threading.Thread(target=spin_integration, daemon=True)
    spin_thread.start()
    
    return integration

def main(args=None):
    """Standalone hardware integration node"""
    rclpy.init(args=args)
    
    try:
        integration = CORIHardwareIntegration()
        
        # Wait for hardware to be ready
        integration.get_logger().info("⏳ Waiting for hardware to be ready...")
        if integration.wait_for_hardware():
            integration.get_logger().info("✅ Hardware integration active!")
        else:
            integration.get_logger().warn("⚠️ Hardware not ready, but continuing...")
        
        # Spin node
        rclpy.spin(integration)
        
    except KeyboardInterrupt:
        integration.get_logger().info("🛑 Hardware integration stopped by user")
    except Exception as e:
        integration.get_logger().error(f"❌ Integration error: {e}")
    finally:
        if 'integration' in locals():
            integration.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()