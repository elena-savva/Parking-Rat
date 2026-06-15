#!/usr/bin/env python3
"""
Interactive Gain & Trim Calibration
=====================================
Run this directly on the Jetson (no ROS needed).

Usage:
    python3 calibrate.py [--yaml path/to/kinematics.yaml]

Workflow:
    1. Trim phase  — drive 1 m straight, you rate the result,
                     the script adjusts trim and repeats.
    2. Gain phase  — drive 1 m straight (with trim fixed),
                     you measure the actual distance,
                     the script adjusts gain and repeats.

The final values are saved back to the YAML file.
"""

import argparse
import math
import os
import sys
import time

# ── Optional YAML support ─────────────────────────────────────────────────────
try:
    import yaml
    YAML_AVAILABLE = True
except ImportError:
    YAML_AVAILABLE = False

# ── Hardware imports ──────────────────────────────────────────────────────────
try:
    from motorDriver import DaguWheelsDriver
    HARDWARE_AVAILABLE = True
except ImportError:
    HARDWARE_AVAILABLE = False
    print("[WARNING] motorDriver not found — running in DRY-RUN mode (no motors).")


# ═════════════════════════════════════════════════════════════════════════════
# Configuration
# ═════════════════════════════════════════════════════════════════════════════

# Distance the robot drives on each trial (metres)
TRIAL_DISTANCE_M = 1.0

# Cruise speed in m/s (keep low to avoid wheel slip)
CRUISE_SPEED_MS  = 0.15

# Physical parameters (must match your kinematics.yaml)
MAX_PHYSICAL_VELOCITY = 0.5   # m/s — the denominator when converting to motor cmd

# How much to nudge trim per rating step
TRIM_STEP = {
    "hl": +0.06,   # hard left  → increase trim to boost right / reduce left
    "sl": +0.02,   # slight left
    "s":   0.00,   # straight   → done!
    "sr": -0.02,   # slight right
    "hr": -0.06,   # hard right
}

# Maximum allowed trim magnitude (sanity clamp)
TRIM_MAX = 0.5

# Rating prompt shown to the user
RATING_PROMPT = (
    "\nHow did the robot drive?\n"
    "  hl = hard left   sl = slight left   s = straight\n"
    "  sr = slight right   hr = hard right\n"
    "  q  = quit and save current values\n"
    "Your rating: "
)


# ═════════════════════════════════════════════════════════════════════════════
# YAML helpers
# ═════════════════════════════════════════════════════════════════════════════

def load_yaml(path):
    """Load kinematics.yaml and return (gain, trim, full_dict)."""
    if not YAML_AVAILABLE:
        print("[INFO] PyYAML not installed — using default gain=1.0, trim=0.0")
        return 1.0, 0.0, None

    if not os.path.exists(path):
        print(f"[INFO] YAML not found at '{path}' — using defaults.")
        return 1.0, 0.0, None

    with open(path, "r") as f:
        data = yaml.safe_load(f)

    params = data.get("kinematics_node", {}).get("ros__parameters", {})
    gain   = float(params.get("gain", 1.0))
    trim   = float(params.get("trim", 0.0))
    print(f"[INFO] Loaded from YAML — gain={gain:.4f}  trim={trim:.4f}")
    return gain, trim, data


def save_yaml(path, data, gain, trim):
    """Write updated gain and trim back to the YAML file."""
    if not YAML_AVAILABLE or data is None:
        print("\n[INFO] Cannot save YAML. Final values:")
        print(f"  gain: {gain:.4f}")
        print(f"  trim: {trim:.4f}")
        return

    data["kinematics_node"]["ros__parameters"]["gain"] = round(gain, 4)
    data["kinematics_node"]["ros__parameters"]["trim"] = round(trim, 4)

    with open(path, "w") as f:
        yaml.dump(data, f, default_flow_style=False)

    print(f"\n[SAVED] {path}")
    print(f"  gain: {gain:.4f}")
    print(f"  trim: {trim:.4f}")


# ═════════════════════════════════════════════════════════════════════════════
# Motor helpers
# ═════════════════════════════════════════════════════════════════════════════

def speed_to_motor_cmd(speed_ms, gain, trim_left, trim_right, max_vel):
    """
    Convert a target m/s to left/right motor commands [-1, 1],
    applying gain and per-side trim exactly as kinematics_node does.
    """
    # No rotation — both wheels same ideal speed
    v_l_ideal = speed_ms
    v_r_ideal = speed_ms

    v_l_calib = v_l_ideal * gain * (1.0 - trim_left)
    v_r_calib = v_r_ideal * gain * (1.0 + trim_right)

    left_cmd  = max(min(v_l_calib / max_vel,  1.0), -1.0)
    right_cmd = max(min(v_r_calib / max_vel,  1.0), -1.0)
    return left_cmd, right_cmd


def drive(motors, left_cmd, right_cmd, duration_s):
    """Send motor commands, wait, then stop."""
    if motors is not None:
        motors.set_wheels_speed(left=left_cmd, right=right_cmd)
    else:
        print(f"  [DRY-RUN] left={left_cmd:.3f}  right={right_cmd:.3f}  for {duration_s:.1f}s")
    time.sleep(duration_s)
    if motors is not None:
        motors.set_wheels_speed(left=0.0, right=0.0)


def trial_duration(distance_m, speed_ms):
    """Seconds needed to cover distance_m at speed_ms."""
    return distance_m / speed_ms


# ═════════════════════════════════════════════════════════════════════════════
# Calibration phases
# ═════════════════════════════════════════════════════════════════════════════

def phase_trim(motors, gain, trim, max_vel):
    """
    Interactively adjust trim until the robot drives straight.
    Returns the final trim value.
    """
    print("\n" + "═" * 55)
    print("  PHASE 1 — TRIM CALIBRATION")
    print("  Goal: robot drives straight for 1 m.")
    print("  Current trim: {:.4f}".format(trim))
    print("═" * 55)

    duration = trial_duration(TRIAL_DISTANCE_M, CRUISE_SPEED_MS)

    while True:
        left_cmd, right_cmd = speed_to_motor_cmd(
            CRUISE_SPEED_MS, gain, trim, trim, max_vel
        )

        print(f"\n  trim={trim:+.4f}   left_cmd={left_cmd:.3f}   right_cmd={right_cmd:.3f}")
        input("  Place robot on start line and press ENTER to drive...")

        drive(motors, left_cmd, right_cmd, duration)

        rating = input(RATING_PROMPT).strip().lower()

        if rating == "q":
            print("  Trim calibration aborted — keeping current value.")
            break

        if rating not in TRIM_STEP:
            print("  Unknown rating — please use: hl, sl, s, sr, hr, q")
            continue

        if rating == "s":
            print(f"\n  ✓ Straight! Trim locked at {trim:+.4f}")
            break

        adjustment = TRIM_STEP[rating]
        trim = max(min(trim + adjustment, TRIM_MAX), -TRIM_MAX)
        print(f"  → Adjusting trim by {adjustment:+.4f}  →  new trim = {trim:+.4f}")

    return trim


def phase_gain(motors, gain, trim, max_vel):
    """
    Interactively adjust gain until the robot covers exactly 1 m.
    Returns the final gain value.
    """
    print("\n" + "═" * 55)
    print("  PHASE 2 — GAIN CALIBRATION")
    print("  Goal: robot travels exactly {:.1f} m.".format(TRIAL_DISTANCE_M))
    print("  Current gain: {:.4f}".format(gain))
    print("═" * 55)

    duration = trial_duration(TRIAL_DISTANCE_M, CRUISE_SPEED_MS)

    while True:
        left_cmd, right_cmd = speed_to_motor_cmd(
            CRUISE_SPEED_MS, gain, trim, trim, max_vel
        )

        print(f"\n  gain={gain:.4f}   left_cmd={left_cmd:.3f}   right_cmd={right_cmd:.3f}")
        input("  Mark the start, place robot there, press ENTER to drive...")

        drive(motors, left_cmd, right_cmd, duration)

        actual_str = input(
            f"\n  Measure the actual distance traveled (target={TRIAL_DISTANCE_M:.2f} m).\n"
            "  Enter meters (or 'q' to quit): "
        ).strip().lower()

        if actual_str == "q":
            print("  Gain calibration aborted — keeping current value.")
            break

        try:
            actual = float(actual_str)
        except ValueError:
            print("  Please enter a number in metres (e.g. 0.87).")
            continue

        if actual <= 0:
            print("  Distance must be positive.")
            continue

        # Scale gain proportionally: if robot went short, gain needs to go up
        gain = gain * (TRIAL_DISTANCE_M / actual)
        print(f"  → New gain = {gain:.4f}")

        redo = input("  Run again to verify? [y/n]: ").strip().lower()
        if redo != "y":
            print(f"\n  ✓ Gain locked at {gain:.4f}")
            break

    return gain


# ═════════════════════════════════════════════════════════════════════════════
# Main
# ═════════════════════════════════════════════════════════════════════════════

def main():
    parser = argparse.ArgumentParser(description="Interactive gain/trim calibration.")
    parser.add_argument(
        "--yaml",
        default="kinematics.yaml",
        help="Path to kinematics.yaml (default: ./kinematics.yaml)",
    )
    parser.add_argument(
        "--gain-only", action="store_true",
        help="Skip trim phase and go straight to gain calibration."
    )
    parser.add_argument(
        "--trim-only", action="store_true",
        help="Skip gain phase and only calibrate trim."
    )
    args = parser.parse_args()

    # ── Load YAML ─────────────────────────────────────────────────────────────
    gain, trim, yaml_data = load_yaml(args.yaml)

    # ── Init hardware ─────────────────────────────────────────────────────────
    motors = None
    if HARDWARE_AVAILABLE:
        try:
            motors = DaguWheelsDriver()
            print("[INFO] Motors initialized.")
        except Exception as e:
            print(f"[WARNING] Motor init failed: {e} — running dry-run.")
    
    print(f"\n  TRIAL_DISTANCE : {TRIAL_DISTANCE_M} m")
    print(f"  CRUISE_SPEED   : {CRUISE_SPEED_MS} m/s  ({CRUISE_SPEED_MS/MAX_PHYSICAL_VELOCITY:.2f} motor scale)")
    print(f"  Starting gain  : {gain:.4f}")
    print(f"  Starting trim  : {trim:+.4f}")

    try:
        # ── Phase 1: Trim ──────────────────────────────────────────────────────
        if not args.gain_only:
            trim = phase_trim(motors, gain, trim, MAX_PHYSICAL_VELOCITY)

        # ── Phase 2: Gain ──────────────────────────────────────────────────────
        if not args.trim_only:
            gain = phase_gain(motors, gain, trim, MAX_PHYSICAL_VELOCITY)

    except KeyboardInterrupt:
        print("\n\n  Calibration interrupted.")

    finally:
        if motors is not None:
            motors.set_wheels_speed(left=0.0, right=0.0)
            motors.close()

    # ── Save results ───────────────────────────────────────────────────────────
    print("\n" + "═" * 55)
    print("  CALIBRATION COMPLETE")
    save_yaml(args.yaml, yaml_data, gain, trim)
    print("═" * 55)
    print("\n  Remember to rebuild and relaunch your ROS nodes:")
    print("    colcon build && source install/setup.bash")
    print("    ros2 launch motor_control motor_control.launch.py")


if __name__ == "__main__":
    main()
