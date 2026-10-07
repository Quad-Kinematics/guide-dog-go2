#!/bin/bash
# Starts the complete guide dog system from this workspace (robot.launch.py):
# Hesai LiDAR, Go2 base (TF, cmd_vel bridge), AMCL + Nav2, then the app layer.
# Only /opt/ros/foxy and ~/cyclonedds_ws (CycloneDDS 0.10 RMW) are used from
# outside. NAV2_CONTROLLER=rpp selects Regulated Pure Pursuit (default dwb).
# Extra arguments go to the launch file, e.g. camera:=true (RealSense +
# person detection), app:=false or map:=<yaml>.

# Colors for output
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m' # No Color

echo -e "${GREEN}======================================${NC}"
echo -e "${GREEN}  Starting Complete Guide Dog System  ${NC}"
echo -e "${GREEN}======================================${NC}"

# Installed at <ws>/install/guide_dog_bringup/lib/guide_dog_bringup/
WS="$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")/../../../.." && pwd)"
if [[ ! -f "$WS/install/setup.bash" ]]; then
    WS="$HOME/unitree_ros_ws"
fi

# 1. Source ROS 2 and the workspace
echo -e "${YELLOW}[1/3] Sourcing ROS 2 workspaces...${NC}"
# ROS 2 Foxy's Python tools need the system Python 3.8, not pyenv's
export PYENV_VERSION=system
unset PYTHONHOME
source /opt/ros/foxy/setup.bash
source "$HOME/cyclonedds_ws/install/setup.bash"
source "$WS/install/setup.bash"
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI="file://$(ros2 pkg prefix guide_dog_bringup)/share/guide_dog_bringup/config/cyclonedds_eth.xml"

# 2. Make sure nothing from the old setup is still running
echo -e "${YELLOW}[2/3] Checking for conflicting processes...${NC}"
if pgrep -f "slam_toolbox|controller_server|hesai_ros_driver_node" >/dev/null; then
    echo -e "${RED}SLAM, Nav2 or the Hesai driver is already running.${NC}"
    echo -e "${YELLOW}Stop it first (Ctrl+C in its terminal, or ~/SLAM/stop_all.sh for the old setup).${NC}"
    exit 1
fi
# The cmd_vel bridge now runs in the launch file; a second one (the old
# systemd service or a manual run) would also drive the robot.
for svc in go2-cmdvel-bridge.service go2-odom-to-tf.service; do
    state="$(systemctl --user is-active "$svc" 2>/dev/null)"
    if [[ "$state" == "active" || "$state" == "activating" ]]; then
        echo -e "${YELLOW}Stopping $svc (replaced by this launch)${NC}"
        systemctl --user stop "$svc"
    fi
done
if pgrep -f "cmd_vel_unitree_bridge_node|lowstate_to_joint_states.py|go2_mapping/odom_to_tf" >/dev/null; then
    echo -e "${YELLOW}Stopping old cmd_vel bridge / lowstate / odom_to_tf processes${NC}"
    pkill -f "cmd_vel_unitree_bridge_node|lowstate_to_joint_states.py|go2_mapping/odom_to_tf"
fi

# RealSense sometimes ignores SIGINT and keeps the camera busy
trap "echo -e '${YELLOW}Force quitting Guide Dog System...${NC}'; killall -9 realsense2_camera_node 2>/dev/null" EXIT

# 3. Launch everything (blocking). The app layer starts 10 s after Nav2.
# viewer_host must be an address of this Jetson (eth0 on the robot network), not the
# viewing laptop's. Open http://192.168.123.18:8000 from the laptop.
echo -e "${GREEN}[3/3] Launching (controller: ${NAV2_CONTROLLER:-dwb})...${NC}"
ros2 launch guide_dog_bringup robot.launch.py viewer_host:="192.168.123.18" viewer_port:=8000 "$@"
