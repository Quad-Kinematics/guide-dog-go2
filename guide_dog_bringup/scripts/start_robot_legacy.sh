#!/bin/bash
# OLD startup path, kept as a fallback until start_robot.sh is verified on the
# robot: Nav2 from ~/SLAM/start_navigation.sh, the cmd_vel bridge from
# ~/go2_bringup_ws, unitree msgs from ~/unitree_ros2. Delete once unused.

# Colors for output
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo -e "${GREEN}======================================${NC}"
echo -e "${GREEN}  Starting Complete Guide Dog System  ${NC}"
echo -e "${GREEN}======================================${NC}"

# 1. Source ROS 2 and Workspaces
echo -e "${YELLOW}[1/4] Sourcing ROS 2 workspaces...${NC}"
source /opt/ros/foxy/setup.bash
source $HOME/unitree_ros2/cyclonedds_ws/install/setup.bash
source $HOME/unitree_ros_ws/install/setup.bash

# 2. Start the Navigation Stack in the Background
echo -e "${YELLOW}[2/4] Booting Navigation Stack...${NC}"
cd $HOME/SLAM
bash start_navigation.sh $HOME/unitree_ros_ws/floor_15_map/my_go2_map.yaml &
NAV_PID=$!

# 3. Setup Cleanup Trap (Catches Ctrl+C)
trap "echo -e '${YELLOW}Force quitting Guide Dog System...${NC}'; killall -9 realsense2_camera_node 2>/dev/null; kill -TERM -$NAV_PID 2>/dev/null; exit" INT TERM EXIT

# Wait for Nav2, CycloneDDS, and the Bridge to settle
echo -e "${YELLOW}[3/4] Waiting 10 seconds for Nav2 to initialize...${NC}"
sleep 10

# 4. Launch the Guide Dog Application Layer (Blocking call)
echo -e "${GREEN}[4/4] Launching Guide Dog Application Layer...${NC}"

# viewer_host must be an address of this Jetson (eth0 on the robot network), not the
# viewing laptop's. Open http://192.168.123.18:8000 from the laptop.
ros2 launch guide_dog_bringup guide_dog.launch.py viewer_host:="192.168.123.18" viewer_port:=8000
