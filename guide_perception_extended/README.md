# guide_perception_extended

Face recognition package for the Unitree Go2 guide robot. Identifies who the
robot's camera is looking at, once an upstream LiDAR + head-pointing system
has already turned the head/camera toward the closest detected person.

## Pipeline position

```
/camera/image_raw --> [face_recognition_node] --> /recognized_person
```

This package only does detection + identification. Downstream nodes (not
part of this package) subscribe to `/recognized_person` to decide what to do
with the identity (memory lookup, greeting, etc).

## Setup

```bash
cd <your_ros2_ws>/src
# (this package should already be here as guide_perception_extended/)

pip install -r guide_perception_extended/requirements.txt --break-system-packages

cd <your_ros2_ws>
colcon build --packages-select guide_perception_extended
source install/setup.bash
```

## Usage

The typical workflow is three steps:

### 1. Enroll known faces (once)

Organize reference photos as one subfolder per person:

```
known_faces/
  alice/*.jpg|jpeg|png
  bob/*.jpg|jpeg|png
```

Then run:

```bash
ros2 run guide_perception_extended enroll_faces --images-dir ./known_faces --out ~/oxy_face_db.pkl
```

or directly with Python:

```bash
python3 guide_perception_extended/guide_perception_extended/enroll_faces.py --images-dir ./known_faces --out ~/oxy_face_db.pkl --ctx-id -1
```

This builds a `{name: embedding}` pickle used by `face_recognition_node`.
Since production faces will be seen at an off-axis pitch angle from a
low-mounted camera, include at least one non-frontal enrollment photo per
person, not just straight-on headshots.

### 2. Start the camera

Frames come from the Intel RealSense driver, which publishes the colour stream
on `/camera/color/image_raw`. The bundled launch file brings up the camera and
the face recogniser together:

```bash
ros2 launch guide_perception_extended face_recognition.launch.py
```

### 3. Run face recognition

```bash
ros2 run guide_perception_extended face_recognition_node
```

Key parameters (all overridable via `--ros-args -p <name>:=<value>`):

| Parameter                 | Default                 | Description                                    |
|---------------------------|--------------------------|-------------------------------------------------|
| `camera_topic`             | `/camera/image_raw`      | Input image topic                               |
| `output_topic`             | `/recognized_person`     | Output identity topic                           |
| `debug_image_topic`        | `/face_recognition/debug_image` | Annotated debug image topic              |
| `face_db_path`             | `~/oxy_face_db.pkl`      | Path to enrolled face DB pickle                 |
| `detector_ctx_id`          | `-1`                     | InsightFace context (-1 = CPU, >=0 = GPU index) |
| `process_every_n_frames`   | `3`                      | Throttle factor to bound CPU load               |
| `similarity_threshold`     | `0.45`                   | Minimum cosine similarity to accept a match      |
| `publish_debug_image`      | `true`                   | Whether to publish the annotated debug image    |

### Example output

```bash
$ ros2 topic echo /recognized_person
data: '{"timestamp": 1720454123.912, "face_detected": true, "name": "alice", "confidence": 0.62, "bbox": [412.0, 133.0, 561.0, 312.0]}'
---
```

### Visual sanity check

```bash
ros2 run rqt_image_view rqt_image_view /face_recognition/debug_image
```

This shows the live camera feed with the selected face's bounding box and
name/confidence label drawn on it.

## Known limitations

1. **Dev testing != deployment accuracy.** A laptop or eye-level camera does
   not reproduce the low-camera-mount pitch degradation that the accompanying
   research is about. Treat such accuracy numbers as evidence that "the
   pipeline works end to end," not as an estimate of deployment accuracy.
2. **The 0.45 similarity threshold is an unvalidated starting guess.** It
   needs to be tuned later against real false-accept/false-reject
   tradeoffs once representative enrollment and test data are available.
3. **No multi-frame fusion yet.** Every frame is matched independently;
   there is no temporal smoothing or voting across frames. This is a
   planned future mitigation for noisy single-frame matches.
4. **Output is JSON-in-String, not a custom message type.** `/recognized_person`
   publishes `std_msgs/String` containing a JSON payload rather than a
   dedicated `.msg` type. This is fine for now but may be worth revisiting
   if downstream consumers grow in number or complexity.
