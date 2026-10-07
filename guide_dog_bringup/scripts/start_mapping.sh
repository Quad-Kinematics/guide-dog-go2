#!/bin/bash
# Builds a new map from this workspace (mapping.launch.py): Hesai LiDAR, Go2
# base (TF, cmd_vel bridge) and SLAM Toolbox. Replaces ~/SLAM/start_slam.sh.
# Walk the robot through the floor, then save from another shell:
#   cd ~/unitree_ros_ws/src/guide_dog_navigation/maps
#   ros2 run guide_dog_navigation save_map <name>

GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m' # No Color

# Installed at <ws>/install/guide_dog_bringup/lib/guide_dog_bringup/
WS="$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")/../../../.." && pwd)"
if [[ ! -f "$WS/install/setup.bash" ]]; then
    WS="$HOME/unitree_ros_ws"
fi

export PYENV_VERSION=system
unset PYTHONHOME
source /opt/ros/foxy/setup.bash
source "$HOME/cyclonedds_ws/install/setup.bash"
source "$WS/install/setup.bash"
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI="file://$(ros2 pkg prefix guide_dog_bringup)/share/guide_dog_bringup/config/cyclonedds_eth.xml"

# SLAM and AMCL both publish map -> odom
if pgrep -f "slam_toolbox|amcl|hesai_ros_driver_node" >/dev/null; then
    echo -e "${RED}SLAM, AMCL/Nav2 or the Hesai driver is already running. Stop it first.${NC}"
    exit 1
fi
for svc in go2-cmdvel-bridge.service go2-odom-to-tf.service; do
    state="$(systemctl --user is-active "$svc" 2>/dev/null)"
    if [[ "$state" == "active" || "$state" == "activating" ]]; then
        echo -e "${YELLOW}Stopping $svc (replaced by this launch)${NC}"
        systemctl --user stop "$svc"
    fi
done

echo -e "${GREEN}Starting mapping. Save with: ros2 run guide_dog_navigation save_map <name>${NC}"
ros2 launch guide_dog_bringup mapping.launch.py "$@"
