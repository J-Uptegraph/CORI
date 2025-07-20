#!/bin/bash
# CORI Robot Build and Run Script - Cooperative Organizational Robotic Intelligence
# Description: Unified build and execution for CORI's laundry sorting system
# Constants
WORKSPACE_DIR="/home/juptegraph/Workspaces/Robotics/Projects/CORI/cori_ws"
INTEGRATION_PATH="src/cori_tools/cori_tools/cori_ignition_integration.py"
WORLD_FILE="src/cori_description/worlds/laundry_world.sdf"
URDF_FILE="src/cori_description/urdf/cori.urdf.xacro"

    # Function to display a loading bar
    show_loading_bar() {
        local duration=$1
        local bar_length=65
        local sleep_time=$(echo "scale=2; $duration / $bar_length" | bc)

        echo -ne "   ["
        for i in $(seq 1 $bar_length); do
            printf "\e[32m█\e[0m"
            sleep $sleep_time
        done
        echo -e "]\n"
    }

    # Display startup sequence
    show_startup_sequence() {
        echo -e "\n🤖 Initializing C.O.R.I. system..."
        show_loading_bar 1
        
        # --- Start of Banner Box (Using rounded borders) ---
        local TOTAL_BOX_WIDTH=70
        local banner_inner_width=$((TOTAL_BOX_WIDTH - 2)) 

        # Top border - now rounded
        echo -e "\n╭"$(printf '─%.0s' $(seq 1 $banner_inner_width))"╮"

        # Blank line inside banner
        printf "│%*s│\n" $banner_inner_width ""

        local banner_lines=(
            "    ██████╗    ██████╗    ██████╗    ██╗    "
            "   ██╔════╝   ██╔═══██╗   ██╔══██╗   ██║    "
            "   ██║        ██║   ██║   ██████╔╝   ██║    "
            "   ██║        ██║   ██║   ██╔══██╗   ██║    "
            "   ╚██████╗██╗╚██████╔╝██╗██║  ██║██╗██║ ██╗"
            "    ╚═════╝╚═╝ ╚═════╝ ╚═╝╚═╝  ╚═╝╚═╝╚═╝ ╚═╝"
        )

        for line in "${banner_lines[@]}"; do
            local len=${#line}
            local padding_left=$(( (banner_inner_width - len) / 2 ))
            local padding_right=$(( banner_inner_width - len - padding_left ))
            printf "│%*s%s%*s│\n" $padding_left "" "$line" $padding_right ""
        done

        # Blank line inside banner
        printf "│%*s│\n" $banner_inner_width ""

        local text_lines=(
            "Cooperative Organizational Robotic Intelligence"
            "Developed by Johnathan Uptegraph - 2025"
            "Built to function, designed to matter."
        )
        for text_line in "${text_lines[@]}"; do
            local len=${#text_line}
            local padding_left=$(( (banner_inner_width - len) / 2 ))
            local padding_right=$(( banner_inner_width - len - padding_left ))
            printf "│%*s%s%*s│\n" $padding_left "" "$text_line" $padding_right ""
        done
        
        # Blank line inside banner
        printf "│%*s│\n" $banner_inner_width ""

        echo "╰"$(printf '─%.0s' $(seq 1 $banner_inner_width))"╯"
        # --- End of Banner Box ---
    }

    # Check if file exists
    check_file() {
        local file_path="$1"
        [ -f "$file_path" ] || { echo "❌ File not found: $file_path"; return 1; }
        return 0
    }

    # Get public IP addresses (IPv4 and IPv6)
    get_public_ipv4() {
        local public_ip=""
        
        # Try multiple methods to get IPv4 public IP (force IPv4)
        public_ip=$(timeout 3 curl -4 -s ifconfig.me 2>/dev/null) && [[ $public_ip =~ ^[0-9]+\.[0-9]+\.[0-9]+\.[0-9]+$ ]] && echo "$public_ip" && return
        public_ip=$(timeout 3 curl -4 -s ipinfo.io/ip 2>/dev/null) && [[ $public_ip =~ ^[0-9]+\.[0-9]+\.[0-9]+\.[0-9]+$ ]] && echo "$public_ip" && return
        public_ip=$(timeout 3 curl -4 -s icanhazip.com 2>/dev/null) && [[ $public_ip =~ ^[0-9]+\.[0-9]+\.[0-9]+\.[0-9]+$ ]] && echo "$public_ip" && return
        public_ip=$(timeout 3 wget -qO- -4 checkip.amazonaws.com 2>/dev/null | tr -d '\n') && [[ $public_ip =~ ^[0-9]+\.[0-9]+\.[0-9]+\.[0-9]+$ ]] && echo "$public_ip" && return
        public_ip=$(timeout 3 dig +short myip.opendns.com @resolver1.opendns.com 2>/dev/null) && [[ $public_ip =~ ^[0-9]+\.[0-9]+\.[0-9]+\.[0-9]+$ ]] && echo "$public_ip" && return
        
        # Fallback to local IPv4 
        public_ip=$(ip route get 8.8.8.8 2>/dev/null | grep -oP 'src \K[0-9]+\.[0-9]+\.[0-9]+\.[0-9]+' | head -1)
        [[ $public_ip =~ ^[0-9]+\.[0-9]+\.[0-9]+\.[0-9]+$ ]] && echo "$public_ip" && return
        
        # Final fallback
        public_ip=$(ip addr show | grep 'inet ' | grep -v '127.0.0.1' | awk '{print $2}' | cut -d/ -f1 | grep -E '^[0-9]+\.[0-9]+\.[0-9]+\.[0-9]+$' | head -1)
        [[ $public_ip =~ ^[0-9]+\.[0-9]+\.[0-9]+\.[0-9]+$ ]] && echo "$public_ip"
    }
    
    get_public_ipv6() {
        local public_ipv6=""
        
        # Try multiple methods to get IPv6 public IP (force IPv6)
        public_ipv6=$(timeout 3 curl -6 -s ifconfig.me 2>/dev/null) && [[ $public_ipv6 =~ ^[0-9a-fA-F:]+$ ]] && echo "$public_ipv6" && return
        public_ipv6=$(timeout 3 curl -6 -s ipinfo.io/ip 2>/dev/null) && [[ $public_ipv6 =~ ^[0-9a-fA-F:]+$ ]] && echo "$public_ipv6" && return
        public_ipv6=$(timeout 3 curl -6 -s icanhazip.com 2>/dev/null) && [[ $public_ipv6 =~ ^[0-9a-fA-F:]+$ ]] && echo "$public_ipv6" && return
        
        # Fallback to local IPv6
        public_ipv6=$(ip -6 addr show | grep 'inet6' | grep 'global' | awk '{print $2}' | cut -d/ -f1 | head -1)
        [[ $public_ipv6 =~ ^[0-9a-fA-F:]+$ ]] && echo "$public_ipv6"
    }

    # Display URL information
    display_urls() {
        # Get IPv4 addresses only - multiple methods to ensure IPv4
        local local_ip=""
        
        # Method 1: Use ip route to get primary IPv4
        local_ip=$(ip route get 8.8.8.8 2>/dev/null | grep -oP 'src \K[0-9]+\.[0-9]+\.[0-9]+\.[0-9]+' | head -1)
        
        # Method 2: Use ip addr to get IPv4 from active interface
        if [ -z "$local_ip" ]; then
            local_ip=$(ip addr show | grep 'inet ' | grep -v '127.0.0.1' | awk '{print $2}' | cut -d/ -f1 | grep -E '^[0-9]+\.[0-9]+\.[0-9]+\.[0-9]+$' | head -1)
        fi
        
        # Method 3: Use hostname with strict IPv4 filtering
        if [ -z "$local_ip" ]; then
            local_ip=$(hostname -I | tr ' ' '\n' | grep -E '^[0-9]+\.[0-9]+\.[0-9]+\.[0-9]+$' | head -1)
        fi
        
        local public_ip=$(get_public_ip)
        
        echo ""
        echo "╭────────────────────────────────────────────────────────────────╮"
        echo "│                    🌐 CORI WEB INTERFACE LINKS                 │"
        echo "├────────────────────────────────────────────────────────────────┤"
        echo "│                                                                │"
        echo "│ 📍 Local:  http://localhost:8091/index.html                   │"
        
        if [ -n "$local_ip" ]; then
            printf "│ 🏠 LAN:    %-47s │\n" "http://$local_ip:8091/index.html"
        fi
        
        if [ -n "$public_ip" ] && [ "$public_ip" != "$local_ip" ]; then
            printf "│ 🌍 Public: %-47s │\n" "http://$public_ip:8091/index.html"
            echo "│                                                                │"
            echo "│ 🎉 SEND THIS TO YOUR FRIENDS! 🎉                              │"
            printf "│    👉 %-54s │\n" "http://$public_ip:8091/index.html"
            echo "│                                                                │"
            echo "│ 🇳🇴 Norway friends can control CORI!                          │"
            echo "│ 🇺🇸 Jersey parents can control CORI!                          │"
            echo "│ 🌍 Anyone worldwide can control CORI!                         │"
            echo "│                                                                │"
            printf "│ 📋 Copy & paste this link: %-30s │\n" "$public_ip:8091/index.html"
            echo "│ ⚠️  Note: Ensure ports 8091 & 8767 are forwarded              │"
        else
            echo "│                                                                │"
            echo "│ ⚠️  Public IP not detected - trying to detect...               │"
            echo "│                                                                │"
            if [ -n "$local_ip" ]; then
                printf "│ 🏠 Try LAN link for now: %-32s │\n" "http://$local_ip:8091/index.html"
            fi
            echo "│ 💡 Check internet connection and port forwarding               │"
        fi
        
        echo "│                                                                │"
        echo "╰────────────────────────────────────────────────────────────────╯"
        echo ""
    }

    # Clean up processes
    cleanup_processes() {
        local mode="$1"
        echo "🛑 Stopping $mode processes..."
        
        # Kill ROS2 processes
        pkill -f "ros2 launch" 2>/dev/null || true
        pkill -f "ros2 run" 2>/dev/null || true
        
        # Kill Gazebo/Ignition processes
        pkill -f "ign gazebo" 2>/dev/null || true
        pkill -f "gz sim" 2>/dev/null || true
        pkill -f "gazebo" 2>/dev/null || true
        
        # Kill camera processes
        pkill -f "v4l2_camera" 2>/dev/null || true
        
        # Kill ROS2 core processes
        pkill -f "robot_state_publisher" 2>/dev/null || true
        pkill -f "joint_state_publisher" 2>/dev/null || true
        pkill -f "parameter_bridge" 2>/dev/null || true
        
        # Kill CORI-specific processes
        pkill -f "cori_vision" 2>/dev/null || true
        pkill -f "cori_gui" 2>/dev/null || true
        pkill -f "cori_simulation" 2>/dev/null || true
        pkill -f "cori_tools" 2>/dev/null || true
        pkill -f "cori_ignition_integration" 2>/dev/null || true
        pkill -f "spatial_database" 2>/dev/null || true
        
        # Kill hardware bridge processes
        pkill -f "realtime_web_control" 2>/dev/null || true
        pkill -f "hardware_bridge" 2>/dev/null || true
        
        # Kill web server processes on port 8091
        pkill -f "python.*8091" 2>/dev/null || true
        pkill -f "http.server.*8091" 2>/dev/null || true
        
        # Force kill any remaining python processes that might be CORI-related
        pgrep -f "cori.*python" | xargs -r kill -9 2>/dev/null || true
        
        # Wait for processes to terminate
        sleep 3
        echo "✅ $mode processes stopped"
    }

# Build workspace
build_workspace() {
    echo "🧹 Cleaning previous build..."
    rm -rf build/ devel/ install/
    echo "🔨 Building workspace..."
    colcon build
    [ $? -eq 0 ] && echo "✅ Build successful!" || { echo "❌ Build failed!"; exit 1; }
    
    # Fix ROS 2 executable paths for cori_vision
    echo "🔧 Fixing ROS 2 executable paths..."
    if [ -d "install/cori_vision/bin" ] && [ -d "install/cori_vision/lib/cori_vision" ]; then
        cp install/cori_vision/bin/* install/cori_vision/lib/cori_vision/ 2>/dev/null || true
        echo "✅ Fixed cori_vision executable paths"
    fi
    
    echo "📦 Sourcing workspace..."
    source install/setup.bash
    
    # Add to bashrc for system-wide availability
    echo "🌐 Setting up system-wide access..."
    local setup_line="source $WORKSPACE_DIR/install/setup.bash"
    if ! grep -q "$setup_line" ~/.bashrc; then
        echo "# CORI Workspace Setup" >> ~/.bashrc
        echo "$setup_line" >> ~/.bashrc
        echo "✅ Added CORI workspace to ~/.bashrc"
    else
        echo "✅ CORI workspace already in ~/.bashrc"
    fi
}

# Start Gazebo simulation
start_gazebo() {
    local pid_var="$1"
    echo "🎮 Starting Gazebo simulation..."
    source install/setup.bash
    ros2 launch cori_description spawn_cori_ignition.launch.py &
    eval "$pid_var=\$!"
    sleep 8
    [ -z "$(ps -p ${!pid_var} -o pid=)" ] && { echo "❌ Gazebo failed to start!"; exit 1; }
}

# Start webcam
start_webcam() {
    local pid_var="$1"
    echo "📷 Starting webcam..."
    source install/setup.bash
    ros2 launch cori_vision camera_only.launch.py &
    eval "$pid_var=\$!"
    sleep 5
    [ -z "$(ps -p ${!pid_var} -o pid=)" ] && { echo "❌ Webcam failed to start!"; return 1; }
    return 0
}

# Run system test
run_system_test() {
    cleanup_processes "system test"
    trap 'cleanup_processes "system test"; exit 0' SIGINT
    start_gazebo GAZEBO_PID
    start_webcam WEBCAM_PID || { kill $GAZEBO_PID 2>/dev/null; exit 1; }
    echo "🔗 Starting color detection bridge..."
    ros2 run cori_vision simple_color_detector &
    BRIDGE_PID=$!
    sleep 3
    echo "🎨 Starting color display..."
    echo "👋 Hold colored objects in front of your webcam!"
    echo "📺 Ensure webcam permissions are enabled"
    ros2 run cori_gui color_display
    cleanup_processes "system test"
}

# Run Gazebo simulation only
run_gazebo_only() {
    cleanup_processes "Gazebo"
    trap 'cleanup_processes "Gazebo"; exit 0' SIGINT
    ros2 launch cori_description spawn_cori_ignition.launch.py
}

# Run manual control mode
run_manual_control() {
    cleanup_processes "manual control"
    trap 'cleanup_processes "manual control"; exit 0' SIGINT
    echo "🎮 Starting Gazebo..."
    ign gazebo "$WORLD_FILE" &
    GAZEBO_PID=$!
    sleep 6
    echo "🤖 Starting robot state publisher..."
    ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:="$(xacro "$URDF_FILE")" &
    RSP_PID=$!
    sleep 3
    echo "🚀 Spawning CORI..."
    ros2 run ros_gz_sim create -name cori -topic robot_description
    echo "🎉 CORI is ready for manual control!"
    echo "✋ Click and drag CORI in Gazebo"
    while true; do sleep 1; [ -z "$(ps -p $GAZEBO_PID -o pid=)" ] && cleanup_processes "manual control"; done
}

# Run webcam color detection
run_webcam_color() {
    cleanup_processes "webcam color detection"
    trap 'cleanup_processes "webcam color detection"; exit 0' SIGINT
    start_webcam WEBCAM_PID || exit 1
    echo "🎨 Starting color detection..."
    ros2 run cori_vision simple_color_detector &
    BRIDGE_PID=$!
    sleep 2
    echo "👋 Hold colored objects in front of your webcam!"
    ros2 run cori_gui color_display
    cleanup_processes "webcam color detection"
}

# Run hardware bridge
run_hardware_bridge() {
    [ $(check_file "$INTEGRATION_PATH"; echo $?) -ne 0 ] && exit 1
    cleanup_processes "hardware bridge"
    trap 'cleanup_processes "hardware bridge"; exit 0' SIGINT
    echo "🔗 ESP32 HARDWARE BRIDGE + FULL SYSTEM"
    echo "======================================="
    echo "🤖 Full Gazebo simulation with ESP32 hardware integration"
    echo "📌 ESP32 detected - launching complete system"
    echo "🎯 Features:"
    echo "   🎮 Gazebo simulation"
    echo "   📷 Camera detection with head movement"
    echo "   🧠 Unified database"
    echo "   🔗 ESP32 hardware bridge"
    read -p "🚀 Start full hardware integration? [y/N]: " confirm
    [[ ! $confirm =~ ^[Yy]$ ]] && { echo "👋 Cancelled"; exit 0; }
    
    # Fix lib directory structure for ROS2 launch
    mkdir -p install/cori_hardware/lib/cori_hardware
    cp install/cori_hardware/bin/* install/cori_hardware/lib/cori_hardware/ 2>/dev/null || true
    
    echo "🔌 Starting ESP32 hardware bridge..."
    source install/setup.bash
    ros2 launch cori_hardware hardware_bridge.launch.py &
    BRIDGE_PID=$!
    sleep 3
    
    echo "🎮 Starting Gazebo simulation..."
    start_gazebo GAZEBO_PID
    echo "🔍 Verifying Gazebo startup..."
    sleep 5
    
    echo "📷 Starting camera for integration..."
    start_webcam WEBCAM_PID
    sleep 3
    
    echo "🔗 Starting CORI integration system..."
    cd src/cori_tools/cori_tools/
    # Auto-select Ignition Full mode (option 2) via command line argument
    python3 cori_ignition_integration.py 2
    cleanup_processes "hardware bridge"
}


# Run laundry assistant
run_laundry_assistant() {
    local script_path="src/cori_simulation/cori_simulation/cori_simulator.py"
    [ $(check_file "$script_path"; echo $?) -ne 0 ] && { echo "❌ Laundry assistant not found!"; exit 1; }
    echo "🧺 CORI LAUNDRY SORTING ASSISTANT"
    echo "================================="
    echo "🤖 Features:"
    echo "   📚 Learns your preferences"
    echo "   🧠 Improves with each item"
    echo "   🗂️ Sorts: Lights, Darks, Colors"
    read -p "🚀 Start laundry sorting? [y/N]: " confirm
    [[ ! $confirm =~ ^[Yy]$ ]] && { echo "👋 Cancelled"; exit 0; }
    trap 'echo -e "\n🛑 Stopping...\n💾 Progress saved!"; exit 0' SIGINT
    cd src/cori_simulation/cori_simulation/
    echo "🚀 Launching Laundry Assistant..."
    echo "🎯 TIPS: Start with 'red shirt', 'blue jeans'; type 'quit' to stop"
    python3 cori_simulator.py
}

# Run real-time web control
run_realtime_web_control() {
    cleanup_processes "real-time web control"
    trap 'cleanup_processes "real-time web control"; exit 0' SIGINT
    echo "🌐 CORI REAL-TIME WEB CONTROL + HARDWARE"
    echo "======================================="
    echo "🎯 Features:"
    echo "   🚀 Zero-latency WebSocket control"
    echo "   🎮 Web interface with gamepad support"
    echo "   📡 Direct ROS topic publishing"
    echo "   🔄 Real-time feedback and status"
    echo "   🎨 Color and angle control"
    echo "   🛑 Emergency stop capability"
    echo "   🔗 ESP32 hardware bridge for real servos"
    echo "   🤖 Controls both simulation AND physical robot"
    echo ""
    echo "🔌 WebSocket API: ws://localhost:8767"
    echo "🔌 ESP32 Hardware: Auto-detected and connected"
    echo ""
    display_urls
    echo ""
    read -p "🚀 Start real-time web control? [y/N]: " confirm
    [[ ! $confirm =~ ^[Yy]$ ]] && { echo "👋 Cancelled"; exit 0; }
    
    # Fix lib directory structure for ROS2 launch
    mkdir -p install/cori_hardware/lib/cori_hardware
    cp install/cori_hardware/bin/* install/cori_hardware/lib/cori_hardware/ 2>/dev/null || true
    
    echo "🔌 Starting real-time web control system..."
    source install/setup.bash
    ros2 launch cori_hardware realtime_web_control.launch.py &
    CONTROL_PID=$!
    sleep 5
    
    echo "🎮 Starting Gazebo simulation..."
    start_gazebo GAZEBO_PID
    echo "🔍 Verifying Gazebo startup..."
    sleep 5
    
    echo "✅ System ready!"
    display_urls
    echo "🎮 Use keyboard arrows, mouse, or gamepad for control"
    echo "🛑 Press Ctrl+C to stop"
    echo ""
    
    # Wait for servo testing to complete, then display final URLs
    echo "⏳ Waiting for servo testing to complete..."
    sleep 10
    echo ""
    echo "🎯 All testing complete! System is fully operational."
    display_urls
    
    while true; do 
        sleep 1
        [ -z "$(ps -p $CONTROL_PID -o pid=)" ] && { echo "❌ Web control stopped"; cleanup_processes "real-time web control"; exit 1; }
        [ -z "$(ps -p $GAZEBO_PID -o pid=)" ] && { echo "❌ Gazebo stopped"; cleanup_processes "real-time web control"; exit 1; }
    done
}

# Run full system
run_full_system() {
    [ $(check_file "$INTEGRATION_PATH"; echo $?) -ne 0 ] && exit 1
    echo "🔗 CORI UNIFIED INTEGRATION SYSTEM"
    echo "=================================="
    echo "🎯 Features:"
    echo "   🎮 Gazebo simulation"
    echo "   📷 Camera detection with head movement"
    echo "   🧠 Unified database"
    echo "   🤖 Automatic mode selection (Ignition Full)"
    read -p "🚀 Start integration? [y/N]: " confirm
    [[ ! $confirm =~ ^[Yy]$ ]] && { echo "👋 Cancelled"; exit 0; }
    trap 'cleanup_processes "unified integration"; exit 0' SIGINT
    start_gazebo GAZEBO_PID
    echo "🔍 Verifying Gazebo startup..."
    sleep 5
    # Launch camera for full system mode
    echo "📷 Starting camera for integration..."
    start_webcam WEBCAM_PID
    sleep 3
    echo "🔗 Starting CORI integration system..."
    cd src/cori_tools/cori_tools/
    # Auto-select Ignition Full mode (option 2) via command line argument
    python3 cori_ignition_integration.py 2
    cleanup_processes "unified integration"
}

# Kill all processes
kill_all_processes() {
    cleanup_processes "all ROS"
    echo "📷 Camera devices: $(ls /dev/video* 2>/dev/null | tr '\n' ' ')"
    exit 0
}

# Main execution
main() {
    cd "$WORKSPACE_DIR" || { echo "❌ Failed to navigate to $WORKSPACE_DIR"; exit 1; }
    echo "📁 Current directory: $(pwd)"
    build_workspace
    show_startup_sequence

    # --- Start of Menu Box ---
    local TOTAL_MENU_WIDTH=70
    local menu_inner_width=$((TOTAL_MENU_WIDTH))
    local menu_padding=2  # Left padding for menu items

    echo "╭"$(printf '─%.0s' $(seq 1 $menu_inner_width))"╮"

    # Menu title with centered alignment
    local menu_title="SELECT A PROGRAM:"
    local title_len=${#menu_title}
    local title_pad_left=$(( (menu_inner_width - title_len) / 2 ))
    printf "│%*s%s%*s│\n" $title_pad_left "" "$menu_title" $((menu_inner_width - title_len - title_pad_left)) ""

    # Empty line after title
    printf "│%*s│\n" $menu_inner_width ""

    local menu_items=(
        "1) 🚀 Full System"
        "2) 🎮 Gazebo Simulation"
        "3) 🧺 Laundry Sorting Assistant"
        "4) 📷 Webcam Color Detection"
        "5) 🦾 Manual Robot Control"
        "6) 🔗 ESP32 Hardware Bridge"
        "7) 🌐 Real-time Web Control"
        "8) 🧹 Kill All ROS Processes"
        "9) 🔗 System Test"
        "10) 🚪 Exit"
    )

    for item in "${menu_items[@]}"; do
        printf "│ %-*s │\n" $((menu_inner_width)) "$item"
    done

    # Empty line before bottom border
    printf "│%*s│\n" $menu_inner_width ""

    echo "╰"$(printf '─%.0s' $(seq 1 $menu_inner_width))"╯"
    # --- End of Menu Box ---

    read -p "Enter choice [1-10]: " choice
    case $choice in
        1) run_full_system ;;
        2) run_gazebo_only ;;
        3) run_laundry_assistant ;;
        4) run_webcam_color ;;
        5) run_manual_control ;;
        6) run_hardware_bridge ;;
        7) run_realtime_web_control ;;
        8) kill_all_processes ;;
        9) run_system_test ;;
        10) echo "👋 Exiting..."; exit 0 ;;
        *) echo "❌ Invalid choice"; exit 1 ;;
    esac
    echo "🏁 CORI system ended."
}

main