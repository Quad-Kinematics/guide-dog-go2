#!/bin/bash

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

# You can change 0.0.0.0 and 8000 here to whatever your network needs
ros2 launch guide_dog_bringup guide_dog.launch.py viewer_host:="192.168.123.100" viewer_port:=8000
