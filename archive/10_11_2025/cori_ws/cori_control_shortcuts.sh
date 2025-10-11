#!/bin/bash
# CORI Manual Control Shortcuts
# Source this file for quick access: source cori_control_shortcuts.sh

# Head controls
alias cori-look-left='ros2 run cori_control manual_control look left'
alias cori-look-right='ros2 run cori_control manual_control look right'
alias cori-look-center='ros2 run cori_control manual_control look center'

# Arm controls
alias cori-wave-left='ros2 run cori_control manual_control wave left'
alias cori-wave-right='ros2 run cori_control manual_control wave right'
alias cori-reach-forward='ros2 run cori_control manual_control reach forward'

# Posture
alias cori-sit='ros2 run cori_control manual_control sit'
alias cori-stand='ros2 run cori_control manual_control stand'
alias cori-reset='ros2 run cori_control manual_control reset'

# Function for direct control
cori() {
    ros2 run cori_control manual_control "$@"
}

echo "🤖 CORI shortcuts loaded!"
echo ""
echo "Quick commands:"
echo "  cori-look-left       Turn head left"
echo "  cori-look-right      Turn head right"
echo "  cori-look-center     Center head"
echo "  cori-wave-left       Wave left arm"
echo "  cori-wave-right      Wave right arm"
echo "  cori-reach-forward   Reach forward with both arms"
echo "  cori-sit             Sit down"
echo "  cori-stand           Stand up"
echo "  cori-reset           Reset to neutral pose"
echo ""
echo "Or use: cori <command> [args]"
echo "  Example: cori look left"
echo "  Example: cori head 0.5"
