# Final Project Structure

This folder is a clean starting point for the final robot project.

It is organized by responsibility so you can bring in working code from your
existing assignments and test scripts without mixing everything together.

## Suggested layout

- `calibration/camera/`
  Camera calibration scripts, images, and calibration outputs.
- `calibration/wheel/`
  Wheel, encoder, trim, and kinematics calibration code.
- `vision/camera/`
  Camera capture, undistortion, and image-pair processing.
- `vision/detection/`
  Vision algorithms such as ArUco, lane, color, or object detection.
- `motion/control/`
  Forward motion, speed control, and motor command logic.
- `motion/turning/`
  Turn actions, heading-based movement, and motion primitives.
- `drivers/button/`
  Button input drivers and helpers.
- `drivers/display/`
  OLED or screen display drivers and simple display utilities.
- `drivers/imu/`
  IMU sensor drivers and reader utilities.
- `drivers/tof/`
  ToF distance sensor drivers and helpers.
- `drivers/motor/`
  Low-level motor and HAT driver code.
- `drivers/encoder/`
  Encoder drivers and wheel feedback helpers.
- `behaviors/obstacle_avoidance/`
  Placeholder for future high-level behavior code.
- `behaviors/color_detection/`
  Color-driven behavior code that can use the calibrated camera feed to stop or move the robot.
- `config/`
  YAML files, calibration constants, and shared settings.
- `launch/`
  Launch files once you turn this structure into ROS packages.
- `docs/`
  Notes, integration plans, and architecture decisions.

## Source mapping

You said these are the main working sources to reorganize later:

- `asg2_vision_group/` -> mostly `calibration/camera/`, `vision/camera/`, and `vision/detection/`
- `asg3_motion_group/` -> mostly `motion/` and shared motor-related pieces
- `testScripts/calibration/` -> mostly `calibration/wheel/`
- `testScripts/imuScripts/` -> `drivers/imu/`
- `testScripts/tofScripts/` -> `drivers/tof/`
- `testScripts/displayScripts/` -> `drivers/display/`
- `testScripts/buttonScripts/` -> `drivers/button/`
- `testScripts/motorScripts/` -> `drivers/motor/`
- `testScripts/encoderScripts/` -> `drivers/encoder/`

## What is already populated

The structure is now populated with the current working code from your existing
assignment folders and driver test folders.

- `calibration/camera/`
  Contains the camera calibration script, saved calibration YAML, and checkerboard images.
- `vision/camera/jetson_camera/`
  Preserves the camera ROS package layout from `asg2_vision_group/` inside the new vision area.
- `vision/interfaces/jetson_camera_interfaces/`
  Preserves the custom message definition package for processed image pairs.
- `motion/control/motor_control/`
  Preserves the motion ROS package layout from `asg3_motion_group/`.
- `drivers/`
  Contains the standalone button, display, IMU, ToF, motor, and encoder code copied from `testScripts/`.
- `behaviors/color_detection/color_detection_mode/`
  Contains a new ROS behavior package that subscribes to the calibrated image pair,
  stops on pink, moves forward on blue, and shows a raw-vs-calibrated detection window.
- `launch/vision/` and `launch/motion/`
  Contain the copied launch files in one easy-to-find place.
- `config/vision/` and `config/motion/`
  Contain copied shared YAML settings for quick access.

## Notes

- Generated folders such as `build/`, `install/`, and `log/` were intentionally not copied.
- Python cache files and compiled artifacts were intentionally not copied.
- The sample camera video output file was intentionally not copied because it is a generated artifact.
- Some copied scripts still use their original imports or legacy paths.
  That is expected for this stage.
  The next cleanup step would be to normalize imports and decide which pieces become final ROS packages.

See `docs/source_inventory.md` for the migration summary.

---

## Line follower integration (added)

The line-following logic from `asg3_individual_ale` has been integrated into
this project structure.

### New files

| Path | Description |
|------|-------------|
| `behaviors/line_following/line_following/` | New ROS2 package — `line_following` |
| `behaviors/line_following/line_following/line_following/line_tracker_node.py` | Vision node: detects line, publishes `/vision_error_x` |
| `behaviors/line_following/line_following/line_following/angular_tracker_node.py` | PID controller: subscribes `/vision_error_x`, publishes `/cmd_vel` |
| `behaviors/line_following/line_following/launch/line_following.launch.py` | Full 4-node pipeline launch file |
| `behaviors/line_following/line_following/config/line_following.yaml` | Config for the line_following nodes |
| `vision/detection/line_tracker/line_tracker_node.py` | Reference copy of the detection node alongside other detectors |
| `launch/vision/line_following.launch.py` | Convenience copy of the launch file |

### How to build and run

```bash
# From the final_project root
colcon build --packages-select jetson_camera motor_control line_following
source install/setup.bash
ros2 launch line_following line_following.launch.py
```

### Pipeline

```
publisher_node  (jetson_camera)   →  /camera/image_raw
line_tracker    (line_following)  →  /vision_error_x
angular_tracker (line_following)  →  /cmd_vel
kinematics_node (motor_control)   →  motors
```

`publisher_node` and `kinematics_node` are **reused from existing packages** —
no changes were made to them.
