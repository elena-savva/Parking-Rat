# Goal Navigation

This package is the first no-ArUco coordinate-navigation step.

Coordinate convention:

- Start pose is `(0.0, 0.0, 0.0)`.
- `+x` points forward from the robot's starting direction.
- `+y` points left from the robot's starting direction.
- Units are meters and radians.
- The frame is `odom` for now.

Edit `config/goal_navigation.yaml` to choose the test goal.

Build:

```bash
cd final_project/final_project_v3
colcon build --packages-select motor_control goal_navigation
source install/setup.bash
```

Test goal publishing only:

```bash
ros2 launch goal_navigation goal_navigation.launch.py
ros2 topic echo /goal_pose
```

Robot movement test:

```bash
ros2 launch motor_control motor_control.launch.py
ros2 launch goal_navigation goal_navigation.launch.py
```

Start with `goal_x: 0.50` and `goal_y: 0.00` before trying larger goals.

Y-axis tests:

```bash
# 50 cm left from the start pose
ros2 launch goal_navigation goal_navigation.launch.py goal_x:=0.0 goal_y:=0.5

# 50 cm right from the start pose
ros2 launch goal_navigation goal_navigation.launch.py goal_x:=0.0 goal_y:=-0.5
```

For the y-axis test, the robot should first rotate toward the target and then
move. Positive `goal_y` means left; negative `goal_y` means right.
