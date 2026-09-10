# guide_bringup

One launch file that starts the guide dog packages together instead of
running each `ros2 launch` / `ros2 run` command by hand.

## What it starts

| Argument | Default | Starts |
|---|---|---|
| `enable_perception` | `true` | `guide_perception_extended`'s RealSense camera + `face_recognition_node` (via its own `face_recognition.launch.py`) |
| `enable_cognition` | `true` | `guide_cognition`'s `greeting_node` |

Each argument is independent, so a subsystem can be switched off for testing
without editing the launch file:

```bash
ros2 launch guide_bringup bringup.launch.py
ros2 launch guide_bringup bringup.launch.py enable_cognition:=false
```

## What is not in here yet

- **`guide_actuation`** — not included. As of this file being written it does
  not exist in this repo (see the top-level `CLAUDE.md`); it needs to be
  rebuilt/recovered and given its own `enable_actuation` block here before
  this launch file can drive the real robot.
- **`human_detector`** (Janith's), **`guide_control`** (the shared FSM), and
  **`guide_navigation`** — not built yet. When they are, they get their own
  `enable_<package>` argument and their own block in
  `launch/bringup.launch.py`, following the pattern the two blocks already
  there use.

## Why one package instead of putting this in guide_perception_extended or guide_cognition

Neither Randil's nor Janith's packages should have to depend on the other's
launch file to be plugged in. `guide_bringup` sits above both and only knows
about turning packages on/off — it doesn't own any topic, node, or config
that belongs to another package.
