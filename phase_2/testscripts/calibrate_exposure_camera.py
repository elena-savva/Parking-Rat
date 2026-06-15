#!/usr/bin/env python3
"""
Interactive calibration tool for Jetson camera exposure and LUT threshold.

Run this script directly on the Jetson to find the best EXPOSURE_TIME and
DARK_CUTOFF values for your current lighting environment, then copy them into
publisher_node.py.

Controls
--------
  W / S   — increase / decrease exposure time  (+/- 1000 µs, hold for fast)
  A / D   — decrease / increase dark_cutoff     (+/- 5, hold for fast)
  R       — reset to defaults
  P       — print current values (ready to copy into publisher_node.py)
  Q / ESC — quit

Usage
-----
    python3 calibrate_camera_exposure.py [sensor_id] [width] [height] [fps]

    sensor_id 0 → CSI Jetson camera (nvarguscamerasrc)
    sensor_id 1 → USB/V4L2 camera   (v4l2src /dev/video1)

Examples
--------
    python3 calibrate_camera_exposure.py 0 1280 720 30
    python3 calibrate_camera_exposure.py 1 640 480 30
"""

import sys
import time

import cv2
import numpy as np

# ─────────────────────────────────────────────────────────────────────────────
# DEFAULT STARTING VALUES  ←  start here and tune with keys
# ─────────────────────────────────────────────────────────────────────────────
DEFAULT_EXPOSURE_TIME = 20000   # µs  — used for sensor_id == 0 (CSI / nvargus)
DEFAULT_DARK_CUTOFF   = 100     # 0–255 — pixels below this stay dark; above → white
# ─────────────────────────────────────────────────────────────────────────────

EXPOSURE_STEP_SMALL  = 1_000   # µs per W/S press
EXPOSURE_MIN         = 13_000  # µs — hardware minimum for IMX219 (from sensor modes)
EXPOSURE_MAX         = 300_000 # µs

CUTOFF_STEP_SMALL    = 1       # per A/D press
CUTOFF_MIN           = 0
CUTOFF_MAX           = 254


# ─── GStreamer pipeline builders ─────────────────────────────────────────────

def pipeline_csi(sensor_id, width, height, fps, exposure_time):
    """nvarguscamerasrc with manual exposure (sensor_id 0 or similar CSI)."""
    return (
        f"nvarguscamerasrc sensor-id={sensor_id} "
        f"exposuretimerange=\"{exposure_time} {exposure_time}\" "
        f"gainrange=\"1 1\" "
        f"ispdigitalgainrange=\"1 1\" "
        f"aelock=true wbmode=1 ! "
        f"video/x-raw(memory:NVMM),width={width},height={height},"
        f"framerate={fps}/1,format=NV12 ! "
        f"nvvidconv ! video/x-raw,format=BGRx ! "
        f"videoconvert ! video/x-raw,format=BGR ! "
        f"appsink drop=true sync=false max-buffers=1"
    )


def pipeline_v4l2(device, width, height, fps):
    """V4L2 USB camera pipeline (sensor_id 1); exposure set via v4l2-ctl."""
    return (
        f"v4l2src device={device} ! "
        f"video/x-raw,format=YUY2,width={width},height={height},"
        f"framerate={fps}/1 ! "
        f"videoconvert ! video/x-raw,format=BGR ! "
        f"appsink sync=false max-buffers=1 drop=true"
    )


def set_v4l2_exposure(device, exposure_time_us):
    """Disable auto-exposure and set manual exposure on V4L2 device."""
    import subprocess
    # exposure_absolute is in 100µs units for most UVC cameras
    exposure_100us = max(1, exposure_time_us // 100)
    subprocess.run(
        ["v4l2-ctl", f"--device={device}",
         "--set-ctrl=auto_exposure=1"],          # 1 = manual
        capture_output=True,
    )
    subprocess.run(
        ["v4l2-ctl", f"--device={device}",
         f"--set-ctrl=exposure_time_absolute={exposure_100us}"],
        capture_output=True,
    )


# ─── LUT builder ─────────────────────────────────────────────────────────────

def build_lut(dark_cutoff: int) -> np.ndarray:
    """Pixels < dark_cutoff stay as-is; pixels >= dark_cutoff → 255."""
    table = np.zeros(256, dtype=np.uint8)
    for i in range(256):
        table[i] = i if i < dark_cutoff else 255
    return table


# ─── Camera opener ───────────────────────────────────────────────────────────

def open_camera(sensor_id, width, height, fps, exposure_time):
    if sensor_id == 0:
        gst = pipeline_csi(sensor_id, width, height, fps, exposure_time)
        cap = cv2.VideoCapture(gst, cv2.CAP_GSTREAMER)
    elif sensor_id == 1:
        device = "/dev/video1"
        set_v4l2_exposure(device, exposure_time)
        gst = pipeline_v4l2(device, width, height, fps)
        cap = cv2.VideoCapture(gst, cv2.CAP_GSTREAMER)
    else:
        raise ValueError(f"Unknown sensor_id {sensor_id}")

    if not cap.isOpened():
        raise RuntimeError(
            f"Could not open camera sensor_id={sensor_id}. "
            "Check device connection and GStreamer plugins."
        )
    return cap


# ─── OSD overlay ─────────────────────────────────────────────────────────────

def draw_overlay(frame, exposure_time, dark_cutoff, sensor_id):
    h, w = frame.shape[:2]
    overlay = frame.copy()

    # Semi-transparent background bar
    cv2.rectangle(overlay, (0, 0), (w, 110), (0, 0, 0), -1)
    cv2.addWeighted(overlay, 0.55, frame, 0.45, 0, frame)

    font      = cv2.FONT_HERSHEY_SIMPLEX
    color_val = (0, 230, 255)
    color_key = (200, 200, 200)
    lh        = 26   # line height

    lines = [
        (f"[W/S]  Exposure : {exposure_time:>7} µs", color_val),
        (f"[A/D]  Cutoff   : {dark_cutoff:>7}",       color_val),
        (f"[P] print  [R] reset  [Q/ESC] quit   sensor={sensor_id}", color_key),
        (f"Left=raw   Right=LUT filtered", color_key),
    ]
    for i, (text, col) in enumerate(lines):
        cv2.putText(frame, text, (10, 24 + i * lh), font, 0.62, col, 1,
                    cv2.LINE_AA)
    return frame


# ─── Main loop ───────────────────────────────────────────────────────────────

def run(sensor_id=0, width=1280, height=720, fps=30):
    exposure_time = DEFAULT_EXPOSURE_TIME
    dark_cutoff   = DEFAULT_DARK_CUTOFF

    print(__doc__)
    print(f"\nStarting with sensor_id={sensor_id}  {width}x{height}@{fps}")
    print(f"  exposure_time = {exposure_time} µs")
    print(f"  dark_cutoff   = {dark_cutoff}\n")

    cap = open_camera(sensor_id, width, height, fps, exposure_time)
    lut = build_lut(dark_cutoff)

    # need_reopen: CSI cameras require pipeline rebuild to change exposure
    need_reopen      = False
    last_exposure    = exposure_time
    key_repeat_delay = 0.08   # seconds between repeated adjustments while held

    cv2.namedWindow("Calibration", cv2.WINDOW_NORMAL)

    prev_key_time = 0.0
    w, h = width, height
    window_sized = False

    while True:
        ret, raw = cap.read()
        if not ret:
            print("[WARNING] Frame read failed, retrying...")
            time.sleep(0.05)
            continue

        h, w = raw.shape[:2]

        if not window_sized:
            cv2.resizeWindow("Calibration", min(w * 2, 1600), min(h, 800))
            window_sized = True

        # Apply LUT and build side-by-side view
        filtered = cv2.LUT(raw, lut)
        display  = np.hstack([raw, filtered])
        display  = draw_overlay(display, exposure_time, dark_cutoff, sensor_id)

        cv2.imshow("Calibration", display)
        key = cv2.waitKey(1) & 0xFF

        now = time.time()
        if key == 0xFF:
            # No key pressed — allow fast-repeat if last key was recent
            key = cv2.waitKey(1) & 0xFF

        if key == 0xFF:
            continue   # still nothing

        changed_exposure = False
        changed_cutoff   = False

        if key in (ord('w'), ord('W')):
            exposure_time = min(EXPOSURE_MAX, exposure_time + EXPOSURE_STEP_SMALL)
            changed_exposure = True
        elif key in (ord('s'), ord('S')):
            exposure_time = max(EXPOSURE_MIN, exposure_time - EXPOSURE_STEP_SMALL)
            changed_exposure = True
        elif key in (ord('d'), ord('D')):
            dark_cutoff = min(CUTOFF_MAX, dark_cutoff + CUTOFF_STEP_SMALL)
            changed_cutoff = True
        elif key in (ord('a'), ord('A')):
            dark_cutoff = max(CUTOFF_MIN, dark_cutoff - CUTOFF_STEP_SMALL)
            changed_cutoff = True
        elif key in (ord('r'), ord('R')):
            exposure_time    = DEFAULT_EXPOSURE_TIME
            dark_cutoff      = DEFAULT_DARK_CUTOFF
            changed_exposure = True
            changed_cutoff   = True
            print("[RESET] Back to defaults.")
        elif key in (ord('p'), ord('P')):
            _print_values(exposure_time, dark_cutoff)
        elif key in (ord('q'), ord('Q'), 27):   # q or ESC
            break

        if changed_cutoff:
            lut = build_lut(dark_cutoff)

        # CSI exposure requires reopening the GStreamer pipeline
        if changed_exposure and sensor_id == 0:
            cap.release()
            cap = open_camera(sensor_id, w, h, fps, exposure_time)
        elif changed_exposure and sensor_id == 1:
            set_v4l2_exposure("/dev/video1", exposure_time)

    cap.release()
    cv2.destroyAllWindows()
    print("\nFinal values:")
    _print_values(exposure_time, dark_cutoff)


def _print_values(exposure_time, dark_cutoff):
    print("\n" + "=" * 60)
    print("  Copy these values into publisher_node.py:")
    print("=" * 60)
    print(f"  EXPOSURE_TIME = {exposure_time}   # µs — set in GStreamer pipeline")
    print(f"  DARK_CUTOFF   = {dark_cutoff}       # LUT threshold")
    print("=" * 60 + "\n")


# ─── Entry point ─────────────────────────────────────────────────────────────

if __name__ == "__main__":
    args = sys.argv[1:]
    sensor_id = int(args[0]) if len(args) > 0 else 0
    width     = int(args[1]) if len(args) > 1 else 1280
    height    = int(args[2]) if len(args) > 2 else 720
    fps       = int(args[3]) if len(args) > 3 else 30
    run(sensor_id, width, height, fps)