# vision/detection/line_tracker

Contains `line_tracker_node.py`, the vision detection node for line following.

This file is the **reference copy** kept here for visibility alongside other
detection algorithms. The live ROS entry point copy lives at:

    behaviors/line_following/line_following/line_following/line_tracker_node.py

If you edit the logic, update **both** copies, or consolidate by making the
behavior package import from this location.

## What it does

1. Subscribes to `/camera/image_raw` (CompressedImage from jetson_camera)
2. Applies a horizontal scanline slit at 80% down the image
3. Thresholds for dark pixels to find the line
4. Computes the centroid of the largest blob in the scanline
5. Publishes `error_x = target_x - centroid_x` on `/vision_error_x` (Float32)
6. Publishes a debug image with overlays on `/camera/tracking_debug/compressed`

## Key tuning parameters (in node source)

| Parameter         | Default | Effect                                      |
|-------------------|---------|---------------------------------------------|
| `threshold_val`   | 75      | Brightness cutoff for dark-line detection   |
| `target_ratio`    | 0.5     | Horizontal target (0=left, 0.5=center, 1=right) |
| `scanline_y_ratio`| 0.80    | How far ahead to look (lower = further ahead) |
| `scanline_thickness` | 20   | Height of the detection slit in pixels      |
