#!/bin/bash
#
# view.sh
#
# Run this on your laptop to start the robot debug viewer.
# It handles xhost, Docker, ROS domain, and the viewer script automatically.
#
# Usage:
#   chmod +x view.sh    (only needed once)
#   ./view.sh

set -e

DOMAIN_ID=5
VIEWER_SCRIPT="$(cd "$(dirname "$0")" && pwd)/ratcam.py"

echo ""
echo "=========================================="
echo "  Starting Robot Debug Viewer"
echo "  Listening on /robot/debug/compressed"
echo "  ROS_DOMAIN_ID=$DOMAIN_ID"
echo "=========================================="
echo ""

# Allow Docker to use the local display
xhost +local:root

# Start Docker, mount the viewer script, and run it automatically
docker run -it --rm \
  --net=host \
  --env="DISPLAY=$DISPLAY" \
  --env="LIBGL_ALWAYS_SOFTWARE=1" \
  --env="ROS_DOMAIN_ID=$DOMAIN_ID" \
  --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
  --volume="$VIEWER_SCRIPT:/viewer/ratcam.py:ro" \
  osrf/ros:foxy-desktop \
  bash -c "
    source /opt/ros/foxy/setup.bash && \
    apt-get update -qq && \
    apt-get install -y -qq python3-pip python3-opencv && \
    echo '' && \
    echo 'Viewer starting — press Ctrl+C to stop.' && \
    echo '' && \
    python3 /viewer/ratcam.py
  "
