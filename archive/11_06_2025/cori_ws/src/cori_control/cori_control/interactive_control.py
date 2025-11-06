#!/usr/bin/env python3
"""
CORI Interactive Control Shell
Just type commands in plain English!
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import sys
import threading
import time
import math

class InteractiveController(Node):
    def __init__(self):
        super().__init__('interactive_controller')

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

        # Color positions on table (Y coordinates from laundry_world.sdf)
        # CORI is at (0, 0), table is at (1.5, 0), rotated 90 degrees
        # Colors are arranged along the Y axis
        self.color_positions = {
            'red': -1.071,
            'orange': -0.714,
            'yellow': -0.357,
            'green': 0.0,
            'blue': 0.357,
            'purple': 0.714,
            'grey': 1.071,
            'gray': 1.071,  # Alternate spelling
            'black': 1.428,
        }

        self.running = True

    def set_joint(self, joint_name, value):
        """Set a single joint to a specific value"""
        if joint_name in self.joint_publishers:
            msg = Float64()
            msg.data = float(value)
            self.joint_publishers[joint_name].publish(msg)
            return True
        return False

    def set_pose(self, pose_dict):
        """Set multiple joints from a pose dictionary"""
        for joint_name, value in pose_dict.items():
            self.set_joint(joint_name, value)
            time.sleep(0.02)

    def look_at_color(self, color_name):
        """Calculate angle to look at a specific color on the table"""
        color_name = color_name.lower()
        if color_name not in self.color_positions:
            return None

        # Get color Y position
        color_y = self.color_positions[color_name]

        # Table is at X=1.5m, color is at Y=color_y
        # CORI's head is approximately at (0, 0, 0.15)
        table_x = 1.5

        # Calculate angle (atan2 gives angle from X axis to target)
        # Negative because head joint is inverted (positive rotates left, negative rotates right)
        angle = math.atan2(color_y, table_x)

        # Clamp to joint limits (-1.8 to 1.8 radians)
        angle = max(-1.8, min(1.8, angle))

        return angle

    def parse_command(self, text):
        """Parse natural language commands"""
        text = text.lower().strip()

        if not text or text in ['exit', 'quit', 'q']:
            self.running = False
            print("👋 Goodbye!")
            return

        if text in ['help', 'h', '?']:
            self.show_help()
            return

        # Parse the command
        words = text.split()

        # Check if it's a color command first
        for color in self.color_positions.keys():
            if color in words or text == color:
                angle = self.look_at_color(color)
                if angle is not None:
                    self.set_joint('head', angle)
                    print(f"👀 Looking at {color} ({angle:.2f} radians)")
                    return
                break

        # Look commands
        if 'look' in words or 'head' in words:
            if 'left' in words:
                self.set_joint('head', 1.5)
                print("👈 Looking left")
            elif 'right' in words:
                self.set_joint('head', -1.5)
                print("👉 Looking right")
            elif 'center' in words or 'middle' in words or 'forward' in words:
                self.set_joint('head', 0.0)
                print("👀 Looking center")
            else:
                print("❓ Look where? (left/right/center)")

        # Wave commands
        elif 'wave' in words or 'hi' in words or 'hello' in words:
            if 'left' in words:
                self.set_pose(self.poses['wave_left'])
                print("👋 Waving left arm")
            elif 'right' in words:
                self.set_pose(self.poses['wave_right'])
                print("👋 Waving right arm")
            else:
                # Default to right wave
                self.set_pose(self.poses['wave_right'])
                print("👋 Waving right arm")

        # Reach commands
        elif 'reach' in words or 'arms' in words and 'forward' in words:
            if 'left' in words:
                self.set_pose({'left_shoulder': 1.5, 'left_elbow': 0.0})
                print("🤚 Reaching left")
            elif 'right' in words:
                self.set_pose({'right_shoulder': 1.5, 'right_elbow': 0.0})
                print("🤚 Reaching right")
            elif 'forward' in words or 'out' in words:
                self.set_pose(self.poses['reach_forward'])
                print("🙌 Reaching forward")
            else:
                self.set_pose(self.poses['reach_forward'])
                print("🙌 Reaching forward")

        # Posture commands
        elif 'sit' in words or 'down' in words:
            self.set_pose(self.poses['sit'])
            print("🪑 Sitting down")

        elif 'stand' in words or 'up' in words:
            self.set_pose(self.poses['stand'])
            print("🧍 Standing up")

        elif 'reset' in words or 'neutral' in words or 'home' in words:
            self.set_pose(self.poses['reset'])
            print("🔄 Resetting to neutral pose")

        # Greetings
        elif text in ['hi', 'hello', 'hey']:
            self.set_pose(self.poses['wave_right'])
            print("👋 Hello! *waves*")

        # Left/Right only commands
        elif words == ['left']:
            self.set_joint('head', 1.5)
            print("👈 Looking left")

        elif words == ['right']:
            self.set_joint('head', -1.5)
            print("👉 Looking right")

        elif words == ['center'] or words == ['middle']:
            self.set_joint('head', 0.0)
            print("👀 Looking center")

        # Combo commands
        elif text in ['down', 'sit down']:
            self.set_pose(self.poses['sit'])
            print("🪑 Sitting down")

        elif text in ['up', 'stand up', 'get up']:
            self.set_pose(self.poses['stand'])
            print("🧍 Standing up")

        # Fun commands
        elif 'dance' in words or 'move' in words:
            print("💃 Let's move!")
            self.set_joint('head', 1.0)
            time.sleep(0.3)
            self.set_joint('head', -1.0)
            time.sleep(0.3)
            self.set_joint('head', 0.0)
            print("✨ Dance move complete!")

        else:
            print(f"❓ I don't understand '{text}'. Type 'help' for commands.")

    def show_help(self):
        """Show available commands"""
        print("\n╭─────────────────────────────────────────────────╮")
        print("│  🤖 CORI INTERACTIVE CONTROL                    │")
        print("├─────────────────────────────────────────────────┤")
        print("│  Just type naturally! Examples:                │")
        print("│                                                 │")
        print("│  🎨 Look at Colors on Table:                    │")
        print("│    red / orange / yellow / green               │")
        print("│    blue / purple / grey / black                │")
        print("│    (CORI will automatically look at that color)│")
        print("│                                                 │")
        print("│  🔄 Head Control:                               │")
        print("│    look left / look right / look center        │")
        print("│    left / right / center (shortcuts)           │")
        print("│                                                 │")
        print("│  👋 Arm Control:                                │")
        print("│    wave / wave left / wave right               │")
        print("│    reach forward / reach left / reach right    │")
        print("│                                                 │")
        print("│  🧍 Body Control:                               │")
        print("│    sit / stand / reset                         │")
        print("│                                                 │")
        print("│  🎉 Fun Commands:                               │")
        print("│    hi / hello / hey    - Wave hello            │")
        print("│    dance / move        - Do a little dance     │")
        print("│                                                 │")
        print("│  ℹ️  Other:                                      │")
        print("│    help - Show this help                       │")
        print("│    quit - Exit                                 │")
        print("╰─────────────────────────────────────────────────╯\n")

    def run_interactive(self):
        """Run the interactive command loop"""
        print("\n╭─────────────────────────────────────────────────╮")
        print("│  🤖 CORI INTERACTIVE CONTROL                    │")
        print("├─────────────────────────────────────────────────┤")
        print("│  Type commands in plain English!                │")
        print("│  Try: 'red', 'blue', 'wave', 'sit'             │")
        print("│  🎨 Type any color name to look at it!          │")
        print("│  Type 'help' for all commands                  │")
        print("╰─────────────────────────────────────────────────╯\n")

        # Start ROS2 spinning in a separate thread
        spin_thread = threading.Thread(target=self.spin_ros, daemon=True)
        spin_thread.start()

        # Main command loop
        while self.running:
            try:
                command = input("cori> ").strip()
                if command:
                    self.parse_command(command)
                    # Small delay to allow message to be sent
                    time.sleep(0.1)
            except KeyboardInterrupt:
                print("\n\n👋 Goodbye!")
                self.running = False
                break
            except (EOFError, OSError):
                # Handle EOF or broken pipe gracefully
                print("\n\n👋 Goodbye!")
                self.running = False
                break
            except Exception as e:
                print(f"\n❌ Error: {e}")
                self.running = False
                break

        # Give spin thread time to finish
        time.sleep(0.2)

    def spin_ros(self):
        """Spin ROS2 in background thread"""
        while self.running and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)

def main(args=None):
    rclpy.init(args=args)
    controller = InteractiveController()

    try:
        controller.run_interactive()
    except KeyboardInterrupt:
        print("\n👋 Goodbye!")
    except Exception as e:
        print(f"\n❌ Error: {e}")
    finally:
        controller.running = False
        try:
            controller.destroy_node()
        except:
            pass
        try:
            rclpy.shutdown()
        except:
            pass

if __name__ == '__main__':
    main()
