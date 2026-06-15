# Goal Navigation

Full goal navigation with obstacle avoidance:

```text
orthogonal goal path -> detect obstacle -> check right/left -> avoid -> continue to goal
```

Coordinate convention:

- Start pose is `(0.0, 0.0, 0.0)`.
- `+x` points forward from the robot's starting direction.
- `+y` points left from the robot's starting direction.
- Units are meters.
- The navigation frame is `odom`.

Build:

```bash
cd final_project_v5
colcon build --packages-select jetson_camera motor_control goal_navigation
source install/setup.bash
```

Run:

```bash
ros2 launch goal_navigation goal_navigation.launch.py goal_x:=1.0 goal_y:=0.0
```

This launch also starts the camera display overlay. The overlay labels show:

- front ToF obstacle distance
- current robot pose `x`, `y`, `z`
- current heading
- goal coordinate and remaining distance

The labeled camera stream is also published on:

```text
/goal_navigation/debug/compressed
```

If you are using SSH or no monitor is attached, keep the overlay topic but stop
the OpenCV window:

```bash
ros2 launch goal_navigation goal_navigation.launch.py \
  goal_x:=1.0 goal_y:=0.0 \
  show_overlay_window:=false
```

To run navigation without the camera overlay:

```bash
ros2 launch goal_navigation goal_navigation.launch.py \
  goal_x:=1.0 goal_y:=0.0 \
  enable_camera_overlay:=false
```

Algorithm:

- Drive toward the goal using odometry.
- Goal travel uses only cardinal headings: `0`, `90`, `180`, and `-90` degrees.
- Stop when the front ToF distance is below `obstacle_stop_distance` for
  `obstacle_confirm_count` readings in a row.
- Turn `90` degrees right first and measure the path.
- Turn `90` degrees left from the original obstacle heading and measure the path.
- Choose the usable side that best moves toward the goal; use clearance as the tie breaker.
- Turn to the selected `90` degree path and drive forward `avoid_forward_distance`.
- Resume orthogonal goal navigation using updated odometry.
- Stop when the robot reaches the goal.

Useful tuning:

```bash
ros2 launch goal_navigation goal_navigation.launch.py \
  goal_x:=1.0 goal_y:=0.0 \
  orthogonal_navigation_enabled:=true \
  obstacle_stop_distance:=0.25 \
  obstacle_confirm_count:=3 \
  path_clear_distance:=0.30 \
  path_check_turn_angle_deg:=90.0 \
  avoid_forward_distance:=0.25 \
  avoid_linear_speed:=0.10 \
  avoid_stop_distance:=0.15
```

Expected log lines:

- `Goal navigation ready`
- `Obstacle detection active`
- `Obstacle avoidance active`
- `Obstacle detected`
- `Checking right side first`
- `Checking left side next`
- `Path choice`
- `Starting obstacle avoidance`
- `avoid_forward`
- `Avoidance complete`
- `Goal reached`

After `Goal reached`, the robot publishes stop commands, sends `/goal_reached`,
and stops the periodic navigation and sensor debug logs.

The node also prints one-line telemetry while it runs:

```text
[GOAL_NAV] event=MOVING_TO_GOAL | mode=GOAL | current=(x=0.12, y=0.00, z=0.00) m | heading=0.0 deg | goal=(x=1.00, y=0.00, z=0.00) m | distance_left=0.88 m | tof_front=0.65 m | cmd=(linear=0.12, angular=0.00)
```

To print faster:

```bash
ros2 launch goal_navigation goal_navigation.launch.py \
  goal_x:=1.0 goal_y:=0.0 \
  print_telemetry_period_sec:=0.5
```

Stop the robot if the log says `Obstacle detection is disabled.` The full
navigation launch should always print `Obstacle detection active` before it
moves.

The launch starts with `use_imu_heading:=false` by default to keep the I2C bus
focused on the ToF sensor. If turns are inaccurate, try:

```bash
ros2 launch goal_navigation goal_navigation.launch.py \
  goal_x:=1.0 goal_y:=0.0 \
  use_imu_heading:=true
```

To check the ToF directly:

```bash
ros2 topic echo /tof/front
```

Put an object about 10 cm in front of the sensor. The `range` value should be
finite. Move the object to about 25 cm or closer so `range <= 0.25` to trigger
the default obstacle stop.

The navigation obstacle check still uses the front ToF sensor. The camera is
only for live viewing with labels.
