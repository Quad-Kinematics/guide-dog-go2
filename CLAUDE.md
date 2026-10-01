# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Overview

ROS 2 Jazzy + Gazebo Harmonic workspace for a Unitree Go2 "guide dog" robot. The robot patrols waypoints, detects a person with YOLOv8, turns to face them, greets them via TTS, and guides them to a goal using Nav2.

This directory (`src/`) is the git repo; the colcon workspace root is the parent directory (`~/dog_v1_ws`), where `build/`, `install/`, `log/` and `sim_results/` live. Branches: `simulation` (current, sim-only packages), `hardware` (real robot; sim-specific files removed), `main`, `demo_01`.

## Build and run

Run from the workspace root (`~/dog_v1_ws`), not `src/`:

```bash
source /opt/ros/jazzy/setup.bash
colcon build                                        # all packages
colcon build --packages-select guide_dog_mission    # single package
source install/setup.bash
```

`src/guide_dog_perception/venv/` is a Python venv (gitignored) that holds `ultralytics`. It has no `COLCON_IGNORE`, so add one if colcon starts picking it up.

Tests are the ament lint defaults only (flake8, pep257, copyright for Python packages; `ament_lint_auto` for C++ with copyright and cpplint disabled):

```bash
colcon test --packages-select guide_dog_audio && colcon test-result --verbose
```

Bringup (each in its own sourced terminal):

```bash
ros2 launch unitree_go2_slam slam.launch.py          # Gazebo + CHAMP + EKF + Nav2 (with saved map) + RViz
                                                     # args: world:=, map:=, params_file:=, headless:=true (no GUI/RViz)
ros2 run guide_dog_perception perception_node        # needs ultralytics; loads yolov8n.pt from the CWD
ros2 run guide_dog_audio tts_node                    # pyttsx3
ros2 run guide_dog_mission main_fsm
ros2 service call /start_mission std_srvs/srv/Trigger   # IDLE -> PATROL
```

AMCL starts at the spawn pose (map origin) via `set_initial_pose` in the Nav2 params, so no "2D Pose Estimate" click is needed unless the robot is moved. Do not run `unitree_go2_sim` and `slam.launch.py` together (or leave nodes from one running): the plain sim's CHAMP EKF also publishes `/odom` and `odom -> base_footprint`. `unitree_go2_sim` (`unitree_go2_launch.py`, args `rviz:=true`, `world:=...`) is the plain sim without SLAM/Nav2. To make a new map, use `teleop_twist_keyboard`, then `ros2 run nav2_map_server map_saver_cli -f src/unitree_go2_slam/maps/arena_map --ros-args -p use_sim_time:=true`.

## Architecture

**Locomotion / sim stack** (`unitree_go2_description`, `unitree_go2_sim`, `unitree_go2_slam`, `champ`, `champ_base`, `champ_msgs`):
- `champ`/`champ_base` are the vendored CHAMP quadruped controller. `quadruped_controller_node` turns `/cmd_vel` into joint efforts via `gz_ros2_control`.
- `slam.launch.py` is the main bringup. It uses staged `TimerAction` delays (controller spawners at 20 s and 25 s, Nav2 at 30 s), so expect about 30 s before navigation is ready.
- Odometry is Gazebo ground truth (`/odom`, zero covariance) fed into a `robot_localization` EKF (`footprint_to_odom_ekf`) that fuses x, y **and yaw**. Without absolute yaw the EKF's heading variance grows without bound and gait sway flipped the heading by 100+ degrees. `/odom` must have exactly one publisher (the bridge).
- `body_tilt_tf` publishes `base_footprint -> base_link` from the bridged IMU: the trunk's real height (0.241 m standing, not the 0.375 m spawn height) plus roll and pitch. `pointcloud_to_laserscan` cuts `/scan` in the level `base_footprint` frame (0.15-1.5 m above the floor, `range_min` 0.45 m to drop the robot's own legs). Cutting it relative to a static, level `base_link` filled scans with floor hits whenever the trunk tilted.
- The VLP-16 sees nothing within 0.5 m of itself, so the costmaps never clear within 0.7 m of the robot centre (`raytrace_min_range`) and never treat empty beams as free.
- Launch args `world` (default `worlds/room_human_aligned.world`, a standing person at (3.5, 0); `room.world` has a walking one), `map` (default this package's `maps/arena_map.yaml`) and `params_file` (default `config/nav2/nav2_params.yaml`).
- `config/nav2/nav2_params.yaml` is tuned from the Nav2 Jazzy defaults (changed lines are marked `Go2:`). CHAMP silently caps commands at 0.3 m/s forward, 0.25 m/s sideways and 0.5 rad/s (`config/gait/gait.yaml`), and the smoother, behaviors and controller all respect those caps. The controller is a rotation shim plus regulated pure pursuit at 10 Hz. MPPI lost in trials because trot sway in the velocity feedback made it weave. AMCL uses the omni motion model even for forward-only control, because the diff model turned gait sway into heading noise.
- Navigate-to-pose runs `config/nav2/navigate_to_pose_go2.xml`. It walks to the goal with any heading (`FollowPath` + `general_goal_checker`), then turns on the spot to the goal heading (`TurnToGoal` + `goal_heading_checker`), and stops replanning within `goal_reached_tol` (0.8 m) of the goal. Every new path resets the goal checker's and the rotation shim's separate "position reached" latches. Doing both in one `FollowPath` stalled whenever the two latches disagreed at the edge of the tolerance. With two goal checkers loaded, every `FollowPath` in a tree must set `goal_checker_id`; `navigate_through_poses_go2.xml` does, and ends with any heading. The mission's goals use yaw 0 (`patrol_state.cpp` leaves the orientation unset), so the robot turns to face +x at each waypoint.
- The physical robot runs ROS 2 Foxy, whose Nav2 has none of the rotation shim, velocity smoother, collision monitor, `IsPathValid`/`GlobalUpdatedGoal` BT nodes or plugin-style AMCL motion models used here; this params file needs a Foxy port before it can run on the robot.

**Mission layer** (`guide_dog_*`):
- `guide_dog_mission`: C++ YASMIN state machine in `main_fsm.cpp`. The transition table is defined in `setup_state_machine()`. One state class per file in `src/states/` plus a header in `include/guide_dog_mission/states/`; new states must also be added to `add_executable` in `CMakeLists.txt`. All states share one node (a non-owning `shared_ptr` to `this`). `main()` spins a `MultiThreadedExecutor` on a separate thread while `run()` blocks on `sm->execute()`, so state subscriptions receive callbacks during blocking execution. Monitor with `yasmin_viewer` (FSM name `Guide_Dog_FSM`).
- Flow: `IDLE` (`/start_mission` Trigger service) → `PATROL` (Nav2 `NavigateToPose` over hardcoded waypoints in `patrol_state.cpp`; a `/detected_face` message cancels the goal, giving `CANCEL` → `ALIGN`) → `SCAN` (360° spin on `/cmd_vel`, using `/odom` yaw) → `ALIGN` (P-controller on `center_offset_x`) → `GREET` (`/speak`) → `GUIDE` (Nav2 to the goal) → `ARRIVE` → `IDLE`.
- State subscriptions are created in constructors and stay alive for the whole run. `PatrolState` uses an `is_active_` flag so `/detected_face` is ignored when it is not executing; other states do not yet, so they still receive callbacks while inactive.
- `guide_dog_interfaces`: `msg/DetectedFace` (name, confidence, normalized `center_offset_x/y` in [-0.5, 0.5]) and `srv/Speak` (text → success).
- `guide_dog_perception`: subscribes `/d455/image` and publishes `/detected_face` for the highest-confidence COCO "person" (class 0, conf > 0.5). Despite the message name, it detects people, not faces.
- `guide_dog_audio`: `/speak` service backed by `pyttsx3`.

Some state dependencies (`std_srvs`, `nav_msgs`) are used in code but not declared in `guide_dog_mission`'s `package.xml`/`CMakeLists.txt`. Declare them if you touch those files.

## Simulation results

`../sim_results/` (outside the repo) holds per-run outputs: `csv/run_<ts>.csv` (`timestamp,x,y,event_type,event_label`) and `plots/run_<ts>/` (`trajectory_map.png`, `state_transitions.md`, `audio_log.txt`). The script that generates them is not in this repo.

To debug pose jumps, run `ros2 run unitree_go2_slam pose_logger` next to the sim (Ctrl+C prints a summary that names the layer responsible: AMCL, odometry/EKF, Gazebo physics, TF stalls or TF conflicts), then `ros2 run unitree_go2_slam plot_pose_log`. Output goes to `pose_logs/<timestamp>/` under the current directory.
