# Parking Rat

Parking Rat is a ROS 2 robot project for autonomous navigation and parking on a Jetson/Raspberry Pi-style differential-drive robot platform. The project has two important phases with different goals:

- `phase_1/` focuses on coordinate-based goal navigation and front ToF obstacle avoidance.
- `phase_2/` focuses on line following, color recognition, and ArUco marker reading/parking behavior.

Both phases matter. They share the same broad robot stack, but they are not simply old/new copies of each other.

## What the robot does

- Captures compressed camera frames from a Jetson camera.
- Undistorts and preprocesses camera images for different vision tasks.
- Navigates to coordinate goals in the `odom` frame.
- Uses a front ToF distance sensor to detect and avoid obstacles.
- Tracks a dark line and publishes lateral error for steering.
- Detects colored objects/signals for go/stop behavior.
- Detects ArUco markers and publishes marker ID, distance, angle error, and heading.
- Converts `/cmd_vel` velocity commands into left/right motor control.
- Estimates odometry from commanded velocity, encoder constants, and optional IMU heading.
- Runs behavior-level navigation logic for coordinate goals, obstacle avoidance, line following, color detection, and ArUco parking.

## Repository layout

```text
.
|-- phase_1/                  Coordinate navigation and obstacle avoidance phase
|-- phase_2/                  Line following, color recognition, and ArUco phase
|   |-- behaviors/            High-level robot behaviors
|   |-- calibration/          Camera and wheel calibration scripts/data
|   |-- config/               Shared motion and vision YAML configs
|   |-- docs/                 Source inventory and dependency notes
|   |-- drivers/              Low-level hardware drivers and examples
|   |-- launch/               Convenience launch files grouped by domain
|   |-- motion/               Motion-control ROS packages
|   |-- vision/               Camera, image processing, and interfaces
|   |-- build/                Generated colcon build output
|   |-- install/              Generated colcon install output
|   `-- log/                  Generated colcon/ROS logs
|-- docker_viewer/            Laptop-side debug-feed viewer tools
`-- succesful_run_example.txt Example output from a successful mission launch
```

The `build/`, `install/`, and `log/` folders are generated artifacts. They are useful as evidence of past builds, but source changes should normally happen in `behaviors/`, `motion/`, `vision/`, `drivers/`, `calibration/`, `config/`, and `launch/`.

## Main ROS packages

### `jetson_camera`

Locations:

- `phase_1/vision/camera/jetson_camera/`
- `phase_2/vision/camera/jetson_camera/`

Camera capture and processing package.

Important executables:

- `publisher_node`: captures frames and publishes `/camera/image_raw`.
- `processing_node`: consumes raw images and publishes processed camera streams.
- `subscriber_node`: simple image subscriber/viewer.
- `pair_subscriber_node`: views raw and undistorted image pairs.

Important topics:

- `/camera/image_raw`
- `/camera/image_line`
- `/camera/image_undistorted`
- `/camera/image_pair`

### `jetson_camera_interfaces`

Locations:

- `phase_1/vision/interfaces/jetson_camera_interfaces/`
- `phase_2/vision/interfaces/jetson_camera_interfaces/`

Defines the custom `ProcessedImagePair` message:

```text
std_msgs/Header header
sensor_msgs/CompressedImage raw_image
sensor_msgs/CompressedImage undistorted_image
```

### `motor_control`

Locations:

- `phase_1/motion/control/motor_control/`
- `phase_2/motion/control/motor_control/`

Differential-drive control and navigation support.

Important executables:

- `kinematics_node`: subscribes to `/cmd_vel` and drives the motors.
- `odometry_node`: publishes `/odom`.
- `pid_controller`: drives toward `/goal_pose` and publishes `/goal_reached`.
- `movement_node`: publishes waypoint goals.
- `hand_follower`: experimental camera/hand-following controller.
- Phase 1 also includes `tof_reader_node` and `imu_reader_node` in the `motor_control` package.

Important config:

- `phase_2/motion/control/motor_control/config/kinematics.yaml`
- `phase_2/config/motion/kinematics.yaml`

### Phase 1 `goal_navigation`

Location: `phase_1/behaviors/goal_navigation/goal_navigation/`

Coordinate-based navigation with front ToF obstacle avoidance.

Important executables:

- `go_to_goal_node`: drives toward a requested `(goal_x, goal_y)` coordinate using odometry.
- `telemetry_overlay_node`: overlays current pose, goal, distance left, heading, and ToF obstacle distance on the camera stream.

Important topics:

- `/cmd_vel`
- `/odom`
- `/tof/front`
- `/goal_reached`
- `/goal_navigation/debug/compressed`

Algorithm summary:

- Drive toward the goal in the `odom` frame.
- Use cardinal/orthogonal headings when enabled: `0`, `90`, `180`, and `-90` degrees.
- Stop when the front ToF distance is below `obstacle_stop_distance` for enough consecutive readings.
- Check the right and left side paths.
- Choose a usable side, drive around the obstacle, and resume navigation to the coordinate goal.

Coordinate convention:

- Start pose is `(0.0, 0.0, 0.0)`.
- `+x` points forward from the robot starting direction.
- `+y` points left.
- Units are meters.
- Frame is `odom`.

### Phase 2 `line_following`

Location: `phase_2/behaviors/line_following/line_following/`

Line detection, steering, and mission navigation behavior.

Important executables:

- `line_tracker`: detects the line and publishes `/vision_error_x`.
- `angular_tracker`: turns `/vision_error_x` into `/cmd_vel`.
- `navigator`: mission state machine that coordinates line tracking, ArUco detection, color detection, and goals.

Important topics:

- `/vision_error_x`
- `/line_tracker/lost`
- `/line_tracker/enabled`
- `/linefollower/enabled`
- `/navigator/state`
- `/aruco/target_id`
- `/cmd_vel`
- `/goal_pose`

### Phase 2 `color_detection_mode`

Location: `phase_2/behaviors/color_detection/color_detection_mode/`

Color-based behavior package.

Important executables:

- `go_stop_node`: detects color from a camera image and publishes `/color_detection/result`.
- `color_detection_node`: uses raw/undistorted image pairs and can publish motion commands.

Behavior:

- Pink means stop.
- Blue means move forward.
- No target color means stop.

Important topics:

- `/color_detection/result`
- `/robot/debug/compressed`
- `/cmd_vel`

### Phase 2 `aruco_parking`

Location: `phase_2/behaviors/aruco_parking/aruco_parking/`

ArUco marker detection for parking and alignment.

Important executable:

- `aruco_detector_node`

Important topics:

- `/aruco/detected`
- `/aruco/id`
- `/aruco/angle_error`
- `/aruco/distance`
- `/aruco/heading`
- `/aruco/target_id`
- `/robot/debug/compressed`

### Phase 2 `goal_navigation`

Location: `phase_2/behaviors/goal_navigation/goal_navigation/`

Publishes start-relative coordinate goals for testing odometry/PID navigation in the Phase 2 stack. The fuller coordinate-navigation and obstacle-avoidance behavior lives in Phase 1.

Important executable:

- `goal_publisher_node`

Important topic:

- `/goal_pose`

Coordinate convention:

- Start pose is `(0.0, 0.0, 0.0)`.
- `+x` points forward from the robot starting direction.
- `+y` points left.
- Units are meters and radians.
- Frame is `odom`.

## System flow

### Phase 1 coordinate navigation and obstacle avoidance

Phase 1 is built around coordinate goals, odometry, and a front ToF sensor:

```text
goal_navigation launch
  -> starts kinematics_node
  -> starts odometry_node
  -> starts tof_reader_node after a short delay
  -> optionally starts imu_reader_node
  -> optionally starts camera_publisher and telemetry_overlay_node

tof_reader_node
  -> /tof/front

go_to_goal_node
  -> reads /odom and /tof/front
  -> drives toward goal_x/goal_y
  -> stops when an obstacle is close
  -> checks right and left paths
  -> drives around the obstacle
  -> resumes coordinate navigation
  -> publishes /cmd_vel and /goal_reached

telemetry_overlay_node
  -> overlays pose, goal, heading, distance left, and ToF readings
  -> publishes /goal_navigation/debug/compressed
```

### Phase 2 line, color, and ArUco behavior

Phase 2 is built around camera-driven perception and behavior switching:

```text
Jetson camera
  -> /camera/image_raw
  -> processing_node
      -> /camera/image_line          for line tracking
      -> /camera/image_undistorted   for ArUco and color detection
      -> /camera/image_pair          for raw-vs-undistorted color behavior

line_tracker
  -> /vision_error_x
  -> angular_tracker
  -> /cmd_vel

aruco_detector_node
  -> /aruco/detected, /aruco/id, /aruco/angle_error, /aruco/distance, /aruco/heading

go_stop_node
  -> /color_detection/result

navigator
  -> enables/disables line following
  -> publishes target ArUco IDs and goals
  -> publishes /cmd_vel when needed

motor_control
  -> kinematics_node drives motors from /cmd_vel
  -> odometry_node publishes /odom
  -> pid_controller drives to /goal_pose
```

## Dependencies

This project expects a ROS 2 Python workspace on the robot. The generated paths and logs suggest ROS 2 with Python 3.8, commonly ROS 2 Foxy on Jetson Nano/Ubuntu 20.04.

ROS/package dependencies include:

- `rclpy`
- `std_msgs`
- `sensor_msgs`
- `geometry_msgs`
- `nav_msgs`
- `launch`
- `launch_ros`
- `ament_index_python`
- `rosidl_default_generators`
- `rosidl_default_runtime`

Python/hardware dependencies used by drivers and scripts include:

- `opencv-python` / OpenCV with GStreamer support
- `numpy`
- `setuptools`
- `spidev`
- `Adafruit-SSD1306`
- `pillow`
- `mpu6050-raspberrypi`

The short dependency note copied from the original project is in:

```text
phase_2/docs/dependencies.txt
```

## Build

Build each phase from its own workspace folder.

### Phase 1

```bash
cd phase_1
colcon build
source install/setup.bash
```

To build only the main Phase 1 packages:

```bash
cd phase_1
colcon build --packages-select \
  jetson_camera_interfaces \
  jetson_camera \
  motor_control \
  goal_navigation
source install/setup.bash
```

### Phase 2

```bash
cd phase_2
colcon build --packages-select \
  jetson_camera_interfaces \
  jetson_camera \
  motor_control \
  line_following \
  color_detection_mode \
  aruco_parking \
  goal_navigation
source install/setup.bash
```

If you change message definitions in `jetson_camera_interfaces`, rebuild that package and any package that imports it.

## Run commands

Run these from the matching phase folder after building and sourcing `install/setup.bash`.

### Phase 1 coordinate navigation with obstacle avoidance

From `phase_1/`:

```bash
ros2 launch goal_navigation goal_navigation.launch.py goal_x:=1.0 goal_y:=0.0
```

This starts the coordinate navigator, motor kinematics, odometry, front ToF reader, and optional telemetry camera overlay. The robot tries to reach the requested coordinate, stops for obstacles seen on `/tof/front`, checks right and left, avoids the obstacle, then continues toward the goal.

Useful Phase 1 options:

```bash
ros2 launch goal_navigation goal_navigation.launch.py \
  goal_x:=1.0 goal_y:=0.0 \
  show_overlay_window:=false

ros2 launch goal_navigation goal_navigation.launch.py \
  goal_x:=1.0 goal_y:=0.0 \
  enable_camera_overlay:=false

ros2 launch goal_navigation goal_navigation.launch.py \
  goal_x:=1.0 goal_y:=0.0 \
  obstacle_stop_distance:=0.25 \
  obstacle_confirm_count:=3 \
  path_clear_distance:=0.30 \
  avoid_forward_distance:=0.25
```

To check the front ToF sensor directly:

```bash
ros2 topic echo /tof/front
```

### Phase 2 camera pipeline

From `phase_2/`:


```bash
ros2 launch jetson_camera camera_pipeline.launch.py
```

This starts camera publishing, processing, and pair viewing.

### Phase 2 line following

```bash
ros2 launch line_following line_following.launch.py
```

Pipeline:

```text
publisher_node -> /camera/image_raw
line_tracker -> /vision_error_x
angular_tracker -> /cmd_vel
kinematics_node -> motors
```

### Phase 2 color detection mode

```bash
ros2 launch color_detection_mode color_detection_mode.launch.py
```

For the display-only go/stop detector:

```bash
ros2 launch color_detection_mode go_stop.launch.py
```

### Phase 2 ArUco parking detector

```bash
ros2 launch aruco_parking aruco_parking.launch.py
```

### Phase 2 motion control only

```bash
ros2 launch motor_control motor_control.launch.py
```

### Phase 2 goal publisher test

```bash
ros2 launch goal_navigation goal_navigation.launch.py
```

Example goal overrides:

```bash
ros2 launch goal_navigation goal_navigation.launch.py goal_x:=0.5 goal_y:=0.0
ros2 launch goal_navigation goal_navigation.launch.py goal_x:=0.0 goal_y:=0.5
ros2 launch goal_navigation goal_navigation.launch.py goal_x:=0.0 goal_y:=-0.5
```

### Phase 2 full mission

The successful run example used:

```bash
ros2 launch line_following full_mission.launch.py
```

That package-local launch file starts:

- `jetson_camera/publisher_node`
- `jetson_camera/processing_node`
- `line_following/line_tracker`
- `aruco_parking/aruco_detector_node`
- `color_detection_mode/go_stop_node`
- `line_following/angular_tracker`
- `motor_control/kinematics_node`
- `motor_control/odometry_node`
- `motor_control/pid_controller`
- `line_following/navigator`

See `succesful_run_example.txt` for a captured successful startup log.

## Laptop debug viewer

`docker_viewer/` contains laptop-side tools for viewing the robot debug feed.

Setup:

```bash
cd docker_viewer
chmod +x view.sh
```

Run:

```bash
./view.sh
```

The viewer is intended for topics such as `/robot/debug/compressed`.

## Calibration

### Camera calibration

Camera calibration files live under both phase folders, for example:

```text
phase_1/calibration/camera/
phase_2/calibration/camera/
```

Important paths:

- `scripts/camera_calibration.py`
- `images/`
- `output/camera_calibration.yaml`
- `vision/camera/jetson_camera/config/camera_calibration.yaml`

The `processing_node` uses calibration data to publish undistorted images.

### Wheel and motor calibration

Wheel calibration scripts live under both phase folders, for example:

```text
phase_1/calibration/wheel/
phase_2/calibration/wheel/
```

Important scripts/configs:

- `calib_gain.py`
- `calib_trim.py`
- `calibrate_final.py`
- `drive_1m.py`
- `kinematics.yaml`

The motion stack uses `kinematics.yaml` values such as baseline, wheel radius, gain, trim, encoder ticks, and IMU options.

## Useful topic checks

After launching a pipeline, these commands are helpful:

```bash
ros2 topic list
ros2 topic echo /tof/front
ros2 topic echo /vision_error_x
ros2 topic echo /color_detection/result
ros2 topic echo /aruco/detected
ros2 topic echo /aruco/id
ros2 topic echo /odom
ros2 topic echo /goal_reached
```

For manual velocity testing:

```bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.1}, angular: {z: 0.0}}"
```

Stop the robot:

```bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0}, angular: {z: 0.0}}"
```

## Notes and known issues

- `phase_1/` and `phase_2/` are both important, but they solve different parts of the project. Phase 1 is for coordinate navigation and obstacle avoidance; Phase 2 is for line following, color recognition, and ArUco reading.
- The repository includes generated `build/`, `install/`, and `log/` folders. If builds behave strangely, clean those generated folders and rebuild.
- Some launch files are duplicated in package folders and in phase-level `launch/` folders. Prefer package launch files when using `ros2 launch package_name file.launch.py`.
- `phase_2/launch/full_mission.launch.py` references `pid_controller_node`, but the `motor_control` package entry point is named `pid_controller`. The package-local `line_following/full_mission.launch.py` uses the correct executable name.
- Some comments/logs in launch files show mojibake characters from an encoding mismatch. The code is still readable, but those comments can be cleaned later.
- `succesful_run_example.txt` is misspelled in the repository name but kept as-is to avoid breaking references.
- Several packages still have placeholder metadata such as `version='0.0.0'`, TODO license fields, and `jetson@todo.todo` maintainer information.

## Source history

The project was reorganized from earlier assignment and test-script folders. See:

```text
phase_1/docs/source_inventory.md
phase_2/docs/source_inventory.md
```

Those files record how vision, camera calibration, motion control, low-level drivers, wheel calibration, and behavior packages were copied into the current structures.
