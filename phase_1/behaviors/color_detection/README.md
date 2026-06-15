# Color Detection Mode

This behavior uses the calibrated camera pipeline and the motion stack:

- subscribes to `/camera/image_pair`
- publishes motion commands on `/cmd_vel`
- publishes detection status on `/color_detection/result`

Behavior:

- `pink` -> stop
- `blue` -> move forward
- no target color -> stop

Display:

- left panel shows the raw image
- right panel shows the calibrated image with detection overlay

Package:

- `color_detection_mode`

Launch:

```bash
ros2 launch color_detection_mode color_detection_mode.launch.py
```
