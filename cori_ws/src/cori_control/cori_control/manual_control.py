#!/usr/bin/env python3
"""
CORI Manual Control - Simple command interface for controlling CORI's joints
Usage: ros2 run cori_control manual_control <command> [value]

Commands:
  look left/right/center       - Turn head left/right/center
  wave left/right              - Wave left or right arm
  reach left/right/forward     - Reach with arms
  sit/stand                    - Sit down or stand up
  reset                        - Reset all joints to neutral

Examples:
  ros2 run cori_control manual_control look left
  ros2 run cori_control manual_control wave right
  ros2 run cori_control manual_control head 0.5
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import sys
import time

class ManualController(Node):
    def __init__(self):
        super().__init__('manual_controller')

        # Create publishers for all joints
        self.joint_publishers = {
            'head': self.create_publisher(Float64, '/model/cori/joint/head_joint/cmd_pos', 10),
            'left_shoulder': self.create_publisher(Float64, '/model/cori/joint/left_shoulder_joint/cmd_pos', 10),
            'left_elbow': self.create_publisher(Float64, '/model/cori/joint/left_elbow_joint/cmd_pos', 10),
            'left_wrist': self.create_publisher(Float64, '/model/cori/joint/left_wrist_joint/cmd_pos', 10),
            'right_shoulder': self.create_publisher(Float64, '/model/cori/joint/right_shoulder_joint/cmd_pos', 10),
            'right_elbow': self.create_publisher(Float64, '/model/cori/joint/right_elbow_joint/cmd_pos', 10),
            'right_wrist': self.create_publisher(Float64, '/model/cori/joint/right_wrist_joint/cmd_pos', 10),
            'left_hip': self.create_publisher(Float64, '/model/cori/joint/left_hip_joint/cmd_pos', 10),
            'left_knee': self.create_publisher(Float64, '/model/cori/joint/left_knee_joint/cmd_pos', 10),
            'left_ankle': self.create_publisher(Float64, '/model/cori/joint/left_ankle_joint/cmd_pos', 10),
            'right_hip': self.create_publisher(Float64, '/model/cori/joint/right_hip_joint/cmd_pos', 10),
            'right_knee': self.create_publisher(Float64, '/model/cori/joint/right_knee_joint/cmd_pos', 10),
            'right_ankle': self.create_publisher(Float64, '/model/cori/joint/right_ankle_joint/cmd_pos', 10),
        }

        # Predefined poses
        self.poses = {
            'reset': {
                'head': 0.0,
                'left_shoulder': 0.0, 'left_elbow': 0.0, 'left_wrist': 0.0,
                'right_shoulder': 0.0, 'right_elbow': 0.0, 'right_wrist': 0.0,
                'left_hip': 0.0, 'left_knee': 0.0, 'left_ankle': 0.0,
                'right_hip': 0.0, 'right_knee': 0.0, 'right_ankle': 0.0,
            },
            'wave_left': {
                'left_shoulder': 1.5,
                'left_elbow': -1.2,
                'left_wrist': 0.5,
            },
            'wave_right': {
                'right_shoulder': 1.5,
                'right_elbow': -1.2,
                'right_wrist': 0.5,
            },
            'reach_forward': {
                'left_shoulder': 1.0,
                'right_shoulder': 1.0,
                'left_elbow': 0.0,
                'right_elbow': 0.0,
            },
            'sit': {
                'left_hip': -0.8,
                'right_hip': -0.8,
                'left_knee': -1.3,
                'right_knee': -1.3,
            },
            'stand': {
                'left_hip': 0.0,
                'right_hip': 0.0,
                'left_knee': 0.0,
                'right_knee': 0.0,
            }
        }

    def set_joint(self, joint_name, value):
        """Set a single joint to a specific value"""
        if joint_name in self.joint_publishers:
            msg = Float64()
            msg.data = float(value)
            self.joint_publishers[joint_name].publish(msg)
            self.get_logger().info(f'Setting {joint_name} to {value}')
            return True
        return False

    def set_pose(self, pose_dict):
        """Set multiple joints from a pose dictionary"""
        for joint_name, value in pose_dict.items():
            self.set_joint(joint_name, value)
            time.sleep(0.05)  # Small delay between commands

    def execute_command(self, args):
        """Execute a command from command line arguments"""
        if not args:
            self.print_usage()
            return

        command = args[0].lower()

        # Head commands
        if command in ['look', 'head']:
            if len(args) < 2:
                self.get_logger().error('Usage: look <left/right/center> or head <value>')
                return

            direction = args[1].lower()
            if direction == 'left':
                self.set_joint('head', 1.5)
            elif direction == 'right':
                self.set_joint('head', -1.5)
            elif direction == 'center':
                self.set_joint('head', 0.0)
            else:
                try:
                    value = float(direction)
                    self.set_joint('head', value)
                except ValueError:
                    self.get_logger().error(f'Invalid direction: {direction}')

        # Wave commands
        elif command == 'wave':
            if len(args) < 2:
                self.get_logger().error('Usage: wave <left/right>')
                return

            side = args[1].lower()
            if side == 'left':
                self.set_pose(self.poses['wave_left'])
            elif side == 'right':
                self.set_pose(self.poses['wave_right'])
            else:
                self.get_logger().error(f'Invalid side: {side}')

        # Reach commands
        elif command == 'reach':
            if len(args) < 2:
                self.get_logger().error('Usage: reach <left/right/forward>')
                return

            direction = args[1].lower()
            if direction == 'forward':
                self.set_pose(self.poses['reach_forward'])
            elif direction == 'left':
                self.set_pose({'left_shoulder': 1.5, 'left_elbow': 0.0})
            elif direction == 'right':
                self.set_pose({'right_shoulder': 1.5, 'right_elbow': 0.0})
            else:
                self.get_logger().error(f'Invalid direction: {direction}')

        # Posture commands
        elif command == 'sit':
            self.set_pose(self.poses['sit'])

        elif command == 'stand':
            self.set_pose(self.poses['stand'])

        elif command == 'reset':
            self.set_pose(self.poses['reset'])

        # Direct joint control
        elif command in self.joint_publishers:
            if len(args) < 2:
                self.get_logger().error(f'Usage: {command} <value>')
                return
            try:
                value = float(args[1])
                self.set_joint(command, value)
            except ValueError:
                self.get_logger().error(f'Invalid value: {args[1]}')

        else:
            self.get_logger().error(f'Unknown command: {command}')
            self.print_usage()

    def print_usage(self):
        """Print usage information"""
        print(__doc__)
        print("\nAvailable joints for direct control:")
        for joint in self.joint_publishers.keys():
            print(f"  {joint}")

def main(args=None):
    rclpy.init(args=args)
    controller = ManualController()

    # Get command line arguments (skip the first one which is the script name)
    import sys
    cmd_args = sys.argv[1:]

    if not cmd_args:
        controller.print_usage()
    else:
        controller.execute_command(cmd_args)
        # Spin briefly to ensure messages are sent
        rclpy.spin_once(controller, timeout_sec=0.5)

    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
