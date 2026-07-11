# Target Recognition Package

## Overview
The `target_recognition` package is a core component of the autonomous payload delivery system designed for the **Kontes Robot Terbang Indonesia (KRTI)** competition. It performs real-time visual target detection, tracking, and geometric georeferencing to determine the global WGS84 coordinates of ground-based drop zones.

This package utilizes a modular architecture to strictly isolate image processing pipelines from flight state tracking. Once a target is visually locked, the node acts as a **ROS 2 Action Client**, passing high-confidence coordinate maps to the `airdrop_planning` flight orchestrator.

---

## System Architecture & Data Flow


```

+-----------------------+
|   /camera/image_raw   |
+-----------+-----------+
| (sensor_msgs/msg/Image)
v
+------------+------------+      +------------------------------+
|    processImage()       | ---> |  calculateTargetEstimate()   |
|  (HSV Mask & Contours)  |      |  (Georeferencing & Kalman)   |
+------------+------------+      +--------------+---------------+
|                                  |
| (Valid targets)                  | (GPSCoordinate)
v                                  v
+------------+------------+      +--------------+---------------+
|   updateMissionState()  | ---> |       performAction()        |
| (Finite State Machine)  |      |    (ROS 2 Action Client)     |
+-------------------------+      +--------------+---------------+
|
| (interfaces/action/TargetAirdrop)
v
[ To: airdrop_planning ]

```

### Vision & Georeferencing Pipeline
1. **Color & Geometry Isolation:** Camera frames are ingested and converted from BGR to HSV color space to maintain color-thresholding consistency under variable ambient sunlight. Contours are filtered dynamically based on pixel area and distance from the frame boundary.
2. **Homography Projection:** Using an instantaneous telemetry snapshot (latitude, longitude, altitude, roll, pitch, and yaw), a ray trace is cast from the 2D pixel center to the ground terrain under a local flat-earth assumption.
3. **Temporal Filtering:** Raw local coordinates pass through a 2D stationary Kalman Filter (`TargetFilter2D`) to reject pixel scatter caused by engine and airframe vibrations. The target frame must be consistently tracked across a predefined frame threshold before locking.

---

## State Machine Model

The node operates a Finite State Machine (FSM) to protect the payload against false-positive launches:
* **`STATE_SEARCHING`:** Actively filtering incoming video streams. Automatically transitions to the gathering phase upon first target detection.
* **`STATE_GATHERING`:** Activating the georeferencing and tracking pipelines. Accumulates observation passes across the frame threshold (e.g., 75 consecutive frames). If visual contact is lost before completion, it resets cleanly back to the searching pool.
* **`STATE_LOCKED`:** The coordinate map is finalized. The visual tracking pipeline suspends to preserve CPU cycles for navigation tasks.
* **`STATE_WAITING`:** The ROS 2 Action Goal is successfully dispatched to the flight planner. The node continuously listens to feedback updates describing the aircraft approach profile.
* **`STATE_FAILED`:** Triggered if action links fail or the planner aborts the flight vector. Safely clears internal tracking logs and returns to scanning patterns.

---

## Directory Configuration

Ensure your package reflects the following workspace layout for proper compilation:

```text
target_recognition/
├── CMakeLists.txt
├── package.xml
├── config/
│   └── target_recognition_params.yaml
├── include/
│   └── target_recognition/
│       ├── coordinates_conversion.hpp
│       └── targetdetection.hpp
├── launch/
│   └── target_recognition.launch.py
└── src/
    └── targetdetection.cpp

```

---

## Installation & Workspace Generation

### 1. Build Dependencies

Ensure your workspace includes your custom message stack package named `interfaces` and that the default ROS 2 Action libraries are installed on your environment.

Add the missing transaction framework layer onto your global system path if required:

```bash
sudo apt-get update
sudo apt-get install ros-${ROS_DISTRO}-rclcpp-action ros-${ROS_DISTRO}-unique-identifier-msgs

```

### 2. Compilation

Navigate to your primary workspace source route and clean-compile the targeted interfaces and vision packages:

```bash
cd ~/FW_RSC_2026
rm -rf build/target_recognition/ build/interfaces/
colcon build --packages-select interfaces target_recognition
source install/setup.bash

```

---

## Runtime Execution

Launch the standalone target recognition pipeline along with its parameter constraints file:

```bash
ros2 launch target_recognition target_recognition.launch.py

```

### Parameter Tuning

Modify values inside `config/target_recognition_params.yaml` directly to adapt to varied conditions on the airfield without recompiling your binary code:

* **`thresholding`:** Modify `low_H` and `high_H` values to map target tarps under morning, noon, or evening light profiles.
* **`camera` / `distortion`:** Update matrix elements whenever replacing lenses or matching changed capture resolutions.
* **`topics`:** Redirect topic links to switch between simulation strings (`/gazebo`) and live hardware paths (`/mavros`).

