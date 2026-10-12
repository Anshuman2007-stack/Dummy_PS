#!/usr/bin/env bash
# Kill leftover Gazebo / bridge / node processes after a crashed or Ctrl-C'd run.
pkill -f "ign gazebo"           2>/dev/null
pkill -f "ruby.*ign"          2>/dev/null
pkill -f parameter_bridge     2>/dev/null
pkill -f true_pose_gps        2>/dev/null
pkill -f ekf_node             2>/dev/null
pkill -f navigation_pid_node  2>/dev/null
pkill -f goal_pose_node       2>/dev/null
sleep 1
echo "Remaining Gazebo/ROS processes (should be empty):"
pgrep -af "ign gazebo|parameter_bridge|ekf_node|navigation_pid_node|true_pose_gps" || echo "  none"
