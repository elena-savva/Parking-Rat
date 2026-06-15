# Source Inventory

This file records what was copied into `final_project/` during the first
population pass.

## Vision

- Source: `asg2_vision_group/src/jetson_camera/`
- Destination: `vision/camera/jetson_camera/`
- Copied:
  `jetson_camera/` Python package, `config/`, `launch/`, `resource/`,
  `package.xml`, `setup.py`, `setup.cfg`, and test files

- Source: `asg2_vision_group/src/jetson_camera_interfaces/`
- Destination: `vision/interfaces/jetson_camera_interfaces/`
- Copied:
  `msg/ProcessedImagePair.msg`, `CMakeLists.txt`, and `package.xml`

## Camera Calibration

- Source: `asg2_vision_group/src/jetson_camera/jetson_camera/camera_calibration.py`
- Destination: `calibration/camera/scripts/camera_calibration.py`

- Source: `asg2_vision_group/src/jetson_camera/config/camera_calibration.yaml`
- Destination: `calibration/camera/output/camera_calibration.yaml`

- Source: `asg2_vision_group/src/camera_calib_images/`
- Destination: `calibration/camera/images/`

## Motion

- Source: `asg3_motion_group/src/motor_control/`
- Destination: `motion/control/motor_control/`
- Copied:
  `motor_control/` Python package, `config/`, `launch/`, `resource/`,
  `package.xml`, `setup.py`, `setup.cfg`, and test files

- Extra extracted motion files:
  `motion/visualization/visualizer_node.py`
  `behaviors/hand_following/hand_follower_node.py`
  `config/motion/kinematics.yaml`
  `launch/motion/*.launch.py`

## Drivers

- Source: `testScripts/buttonScripts/`
- Destination: `drivers/button/`
- Copied:
  `buttonClass.py`, `ledClass.py`, `main.py`

- Source: `testScripts/displayScripts/`
- Destination: `drivers/display/`
- Copied:
  `Adafruit_GPIO/`, `Adafruit_Python_SSD1306/`, `main.py`,
  `show_battery.py`, `README.md`

- Source: `testScripts/imuScripts/src/`
- Destination: `drivers/imu/`
- Copied:
  `imuDriver.py`, `imu_reader_node.py`, `imu_printer_node.py`

- Source: `testScripts/tofScripts/`
- Destination: `drivers/tof/`
- Copied:
  `tofDriver.py`, `main.py`, `assets/speed_vs_voltage.png`

- Source: `asg3_motion_group/src/motor_control/motor_control/`
- Destination: `drivers/motor/` and `drivers/encoder/`
- Copied:
  `Adafruit_I2C.py`, `Adafruit_PWM_Servo_Driver.py`, `hat.py`,
  `motorDriver.py`, `encoderDriver.py`

- Extra copied examples:
  `drivers/motor/examples/main.py`
  `drivers/encoder/examples/main.py`

## Wheel Calibration

- Source: `testScripts/calibration/`
- Destination: `calibration/wheel/`
- Copied:
  `Adafruit_I2C.py`, `Adafruit_PWM_Servo_Driver.py`, `encoderDriver.py`,
  `hat.py`, `motorDriver.py`, `calib_gain.py`, `calib_trim.py`,
  `calibrate_final.py`, `calibration_general.py`,
  `calibration_general_updated.py`, `drive_1m.py`, `kinematics.yaml`

- Source: `testScripts/calibration.py`
- Destination: `calibration/wheel/calibration.py`

## Shared Docs

- Source: `testScripts/dependencies.txt`
- Destination: `docs/dependencies.txt`

## New behavior added in final_project

- Native addition:
  `behaviors/color_detection/color_detection_mode/`

- Added:
  `color_detection_node.py`, `color_detection.yaml`,
  `color_detection_mode.launch.py`, `package.xml`, `setup.py`, and
  the resource/setup support files

- Purpose:
  Subscribe to the processed raw + calibrated image pair, detect pink and blue
  within a configurable ROI, publish `/cmd_vel` commands, and display the
  raw image next to the calibrated detection view.

## Intentionally skipped

- `build/`, `install/`, and `log/` directories
- `__pycache__/` folders and `*.pyc` files
- generated camera output video files
- `testScripts/speed_measure.py`
  It was left out for now because it still points at the old legacy folder
  layout and should be migrated deliberately in a later cleanup step.

## Important note

This first pass focused on clean grouping, not final import cleanup.
Some ROS nodes and scripts still preserve their original package imports or
legacy assumptions.
That is okay for now and gives you a clearer base to refactor from.
