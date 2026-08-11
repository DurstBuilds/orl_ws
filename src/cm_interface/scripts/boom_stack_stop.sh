#!/bin/bash
# Stop all boom_stack.launch.py processes. C++ nodes often outlive ros2 launch under systemd.
set -euo pipefail

stop_boom_stack() {
  local signal="$1"
  pkill "-${signal}" -f 'ros2 launch cm_interface boom_stack' 2>/dev/null || true
  pkill "-${signal}" -x can_gateway_node 2>/dev/null || true
  pkill "-${signal}" -x joint_translator_node 2>/dev/null || true
  pkill "-${signal}" -x motor_unwrapper_node 2>/dev/null || true
  pkill "-${signal}" -f boom_joystick_control 2>/dev/null || true
  pkill "-${signal}" -f joint_position_sequence 2>/dev/null || true
  pkill "-${signal}" -x joy_node 2>/dev/null || true
}

stop_boom_stack TERM
sleep 2
stop_boom_stack KILL
