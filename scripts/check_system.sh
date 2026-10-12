#!/usr/bin/env bash
# Quick health check of a RUNNING system: nodes, topics, and that data is flowing.
# Usage:  source install/setup.bash && ./scripts/check_system.sh
set -u

if ! command -v ros2 >/dev/null; then
  echo "ros2 not found - source /opt/ros/<distro>/setup.bash and install/setup.bash first." >&2
  exit 1
fi

pass=0; fail=0
ok()   { echo "  [ OK ] $*"; pass=$((pass+1)); }
bad()  { echo "  [FAIL] $*"; fail=$((fail+1)); }

echo "== Nodes =="
nodes="$(ros2 node list 2>/dev/null)"
for n in /true_pose_gps /ekf_node /navigation_pid_node /goal_input /ros_gz_bridge; do
  if echo "$nodes" | grep -q "^${n}$"; then ok "$n"; else bad "$n not running"; fi
done
echo "  (goal_input is optional if you started it manually; the bridge name may differ - see 'ros2 node list')"

echo "== Topics (one message within 3 s) =="
for t in /odom/wheels /imu/data /gnss/fix /true_odom /ekf/odom; do
  if timeout 3 ros2 topic echo --once "$t" >/dev/null 2>&1; then ok "$t"; else bad "$t no data"; fi
done

echo "== Rates (4 s sample) =="
for t in /odom/wheels /imu/data /gnss/fix /ekf/odom; do
  r="$(timeout 4 ros2 topic hz "$t" 2>/dev/null | grep 'average rate' | tail -1 | awk '{print $3}')"
  echo "  $t : ${r:-no data} Hz"
done
echo "  expected: /odom/wheels ~50, /imu/data ~100, /gnss/fix ~5, /ekf/odom ~50"

echo
echo "Result: ${pass} ok, ${fail} failed"
[ "$fail" -eq 0 ]
