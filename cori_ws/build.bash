#!/bin/bash
# CORI Robot Complete Build and Run Script
echo "🤖 CORI Robot - Complete Build and Run Script"
echo "=============================================="

# Navigate to workspace
cd /home/juptegraph/Workspaces/Robotics/Projects/CORI/cori_ws
echo "📁 Current directory: $(pwd)"

# Clean previous build (optional)
echo "🧹 Cleaning previous build..."
rm -rf build/ devel/ install/

# Build both packages
echo "🔨 Building workspace..."
colcon build --packages-select cori_description cori_cv

# Check if build was successful
if [ $? -eq 0 ]; then
    echo "✅ Build successful!"
else
    echo "❌ Build failed! Check the output above."
    exit 1
fi

# Source the workspace
echo "📦 Sourcing workspace..."
source install/setup.bash

echo ""
echo "🎯 Choose what to run:"
echo "1) 🚀 Full system (Gazebo + Webcam + Color Detection)"
echo "2) 🎮 Just Gazebo simulation"
echo "3) 🦾 Manual Control Mode (No Input Lag!)"
echo "4) 📷 Just webcam color detection"
echo "5) 🧹 Kill all ROS processes and exit"
echo "6) 🚪 Exit"
echo ""
read -p "Enter choice [1-6]: " choice

case $choice in
    1)
        echo ""
        echo "🚀 Starting complete CORI system..."
        echo "📋 This will open multiple processes:"
        echo "   🎮 Gazebo simulation with CORI"
        echo "   📷 Webcam feed"
        echo "   🎨 Color detection"
        echo "   🖥️  Terminal color display"
        echo ""
        echo "⚠️  Press Ctrl+C in any terminal to stop"
        echo ""
        
        # Function to kill all background processes
        cleanup() {
            echo ""
            echo "🛑 Stopping all processes..."
            
            # Kill specific ROS processes
            pkill -f "ros2 launch cori_description"
            pkill -f "ros2 launch cori_cv"
            pkill -f "ros2 run cori_cv"
            pkill -f "v4l2_camera_node"
            pkill -f "ign gazebo"
            pkill -f "gz sim"
            
            # Kill any remaining ROS processes
            pkill -f "robot_state_publisher"
            pkill -f "laundry_color_detector"
            pkill -f "simple_color_detector"
            pkill -f "color_display"
            
            # Wait a moment for clean shutdown
            sleep 2
            
            echo "✅ Cleanup complete"
            exit 0
        }
        
        # Clean up any existing processes first
        echo "🧹 Cleaning up any existing processes..."
        pkill -f "ros2 launch" 2>/dev/null || true
        pkill -f "ros2 run" 2>/dev/null || true
        pkill -f "v4l2_camera" 2>/dev/null || true
        pkill -f "ign gazebo" 2>/dev/null || true
        sleep 3
        
        # Set trap to cleanup on Ctrl+C
        trap cleanup SIGINT
        
        # Start Gazebo in background
        echo "🎮 Starting Gazebo simulation..."
        ros2 launch cori_description spawn_cori_ignition.launch.py &
        GAZEBO_PID=$!
        
        echo "⏳ Waiting for Gazebo to fully load..."
        sleep 8
        
        # Check if Gazebo started properly
        if ! ps -p $GAZEBO_PID > /dev/null; then
            echo "❌ Gazebo failed to start!"
            exit 1
        fi
        
        # Start webcam in background
        echo "📷 Starting webcam..."
        ros2 launch cori_cv laundry_color_detector.launch.py &
        WEBCAM_PID=$!
        
        echo "⏳ Waiting for webcam to initialize..."
        sleep 5
        
        # Check if webcam started
        if ! ps -p $WEBCAM_PID > /dev/null; then
            echo "❌ Webcam failed to start!"
            kill $GAZEBO_PID 2>/dev/null
            exit 1
        fi
        
        # Start color detection bridge in background
        echo "🔗 Starting color detection bridge..."
        ros2 run cori_cv simple_color_detector &
        BRIDGE_PID=$!
        
        echo "⏳ Waiting for color detection to connect..."
        sleep 3
        
        # Start color display in foreground (this will show the output)
        echo "🎨 Starting color display..."
        echo "👋 Hold colored objects in front of your webcam!"
        echo "📺 Check that your webcam permissions are enabled"
        ros2 run cori_cv color_display
        
        # If we get here, user stopped the color display
        cleanup
        ;;
        
    2)
        echo "🎮 Launching CORI robot in Ignition Gazebo..."
        echo "⚠️  Press Ctrl+C to stop the simulation"
        echo ""
        ros2 launch cori_description spawn_cori_ignition.launch.py
        ;;
        
    3)
        echo "🦾 Manual Control Mode - No Input Lag!"
        echo "📋 This launches CORI for manual manipulation:"
        echo "   🎮 Gazebo with CORI"
        echo "   🚫 NO automatic joint control"
        echo "   ✋ Click and drag CORI freely!"
        echo ""
        echo "⚠️  Press Ctrl+C to stop"
        echo ""
        
        # Function to kill manual control processes
        cleanup_manual() {
            echo ""
            echo "🛑 Stopping manual control..."
            pkill -f "ign gazebo" 2>/dev/null || true
            pkill -f "robot_state_publisher" 2>/dev/null || true
            pkill -f "ros_gz_sim" 2>/dev/null || true
            sleep 2
            echo "✅ Manual control stopped"
            exit 0
        }
        
        # Kill any existing joint state publishers first
        echo "🧹 Killing any existing joint state publishers..."
        sudo pkill -f joint_state_publisher 2>/dev/null || true
        sleep 1
        
        # Set trap for cleanup
        trap cleanup_manual SIGINT
        
        # Start Gazebo in background
        echo "🎮 Starting Gazebo..."
        ign gazebo /home/juptegraph/Workspaces/Robotics/Projects/CORI/cori_ws/src/cori_description/worlds/laundry_world.sdf &
        GAZEBO_PID=$!
        
        echo "⏳ Waiting for Gazebo to load..."
        sleep 6
        
        # Start robot state publisher in background
        echo "🤖 Starting robot state publisher..."
        ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:="$(xacro src/cori_description/urdf/cori.urdf.xacro)" &
        RSP_PID=$!
        
        echo "⏳ Waiting for robot description..."
        sleep 3
        
        # Spawn CORI
        echo "🚀 Spawning CORI..."
        ros2 run ros_gz_sim create -name cori -topic robot_description
        
        echo ""
        echo "🎉 CORI is ready for manual control!"
        echo "✋ Click and drag any part of CORI in Gazebo"
        echo "🚫 No input lag - he stays exactly where you put him!"
        echo "⚠️  Press Ctrl+C to stop"
        echo ""
        
        # Keep the script running and wait for Ctrl+C
        while true; do
            sleep 1
            # Check if Gazebo is still running
            if ! ps -p $GAZEBO_PID > /dev/null 2>&1; then
                echo "🛑 Gazebo closed, shutting down..."
                cleanup_manual
            fi
        done
        ;;
        
    4)
        echo "📷 Starting webcam color detection system..."
        echo ""
        
        # Start webcam
        echo "📷 Starting webcam..."
        ros2 launch cori_cv laundry_color_detector.launch.py &
        WEBCAM_PID=$!
        sleep 3
        
        # Start color detection bridge
        echo "🎨 Starting color detection..."
        ros2 run cori_cv simple_color_detector &
        BRIDGE_PID=$!
        sleep 2
        
        # Start color display
        echo "👋 Hold colored objects in front of your webcam!"
        ros2 run cori_cv color_display
        
        # Cleanup
        echo "🛑 Stopping webcam processes..."
        kill $WEBCAM_PID $BRIDGE_PID 2>/dev/null
        ;;
        
    5)
        echo "🧹 Killing all ROS processes and resetting camera..."
        
        # Comprehensive cleanup
        pkill -f "ros2" 2>/dev/null || true
        pkill -f "ign gazebo" 2>/dev/null || true
        pkill -f "gz sim" 2>/dev/null || true
        pkill -f "v4l2_camera" 2>/dev/null || true
        pkill -f "robot_state_publisher" 2>/dev/null || true
        pkill -f "cori_cv" 2>/dev/null || true
        
        # Wait for processes to die
        sleep 3
        
        # Reset USB camera driver (simulates unplug/replug)
        echo "🔄 Resetting USB camera driver..."
        sudo modprobe -r uvcvideo 2>/dev/null || true
        sleep 2
        sudo modprobe uvcvideo 2>/dev/null || true
        sleep 2
        
        echo "✅ All ROS processes killed and camera reset"
        echo "💡 Camera should be available for restart"
        echo "📷 Camera devices: $(ls /dev/video* 2>/dev/null | tr '\n' ' ')"
        exit 0
        ;;
        
    6)
        echo "👋 Exiting..."
        exit 0
        ;;
        
    *)
        echo "❌ Invalid choice. Exiting..."
        exit 1
        ;;
esac

echo "🏁 CORI system ended."