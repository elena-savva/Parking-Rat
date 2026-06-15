#!/usr/bin/env python3
"""
Drive 1 Metre Test
==================
Reads gain and trim from a YAML file (or uses defaults),
then drives the robot straight for 1 metre.

Usage:
    python3 drive_1m.py
    python3 drive_1m.py --yaml /path/to/kinematics.yaml
    python3 drive_1m.py --yaml /path/to/kinematics.yaml --distance 0.5
"""

import argparse
import os
import time

# ── Optional YAML support ─────────────────────────────────────────────────────
try:
    import yaml
    YAML_AVAILABLE = True
except ImportError:
    YAML_AVAILABLE = False

# ── Hardware ──────────────────────────────────────────────────────────────────
try:
    from motorDriver import DaguWheelsDriver
    HARDWARE_AVAILABLE = True
except ImportError:
    HARDWARE_AVAILABLE = False
    print("[WARNING] motorDriver not found — dry-run mode.")


# ── Constants ─────────────────────────────────────────────────────────────────
CRUISE_SPEED_MS       = 0.15   # m/s — match whatever you used during calibration
MAX_PHYSICAL_VELOCITY = 0.5    # m/s — must match kinematics.yaml


# ── YAML ──────────────────────────────────────────────────────────────────────
def load_yaml(path):
    if not YAML_AVAILABLE:
        print("[INFO] PyYAML not installed — using gain=1.0, trim=0.0")
        return 1.0, 0.0

    if not os.path.exists(path):
        print(f"[INFO] YAML not found at '{path}' — using gain=1.0, trim=0.0")
        return 1.0, 0.0

    with open(path, "r") as f:
        data = yaml.safe_load(f)

    params = data.get("kinematics_node", {}).get("ros__parameters", {})
    gain   = float(params.get("gain", 1.0))
    trim   = float(params.get("trim", 0.0))
    print(f"[INFO] Loaded — gain={gain:.4f}  trim={trim:+.4f}")
    return gain, trim


# ── Motor command ─────────────────────────────────────────────────────────────
def compute_motor_cmds(speed_ms, gain, trim):
    """Apply gain and trim exactly as kinematics_node does."""
    v_l = speed_ms * gain * (1.0 - trim)
    v_r = speed_ms * gain * (1.0 + trim)
    left_cmd  = max(min(v_l / MAX_PHYSICAL_VELOCITY,  1.0), -1.0)
    right_cmd = max(min(v_r / MAX_PHYSICAL_VELOCITY,  1.0), -1.0)
    return left_cmd, right_cmd


# ── Main ──────────────────────────────────────────────────────────────────────
def main():
    parser = argparse.ArgumentParser(description="Drive straight for a set distance.")
    parser.add_argument("--yaml",     default="kinematics.yaml",
                        help="Path to kinematics.yaml")
    parser.add_argument("--distance", type=float, default=1.0,
                        help="Distance to drive in metres (default: 1.0)")
    args = parser.parse_args()

    gain, trim = load_yaml(args.yaml)

    duration   = args.distance / CRUISE_SPEED_MS
    left_cmd, right_cmd = compute_motor_cmds(CRUISE_SPEED_MS, gain, trim)

    print(f"\n  Distance   : {args.distance:.2f} m")
    print(f"  Speed      : {CRUISE_SPEED_MS} m/s")
    print(f"  Duration   : {duration:.2f} s")
    print(f"  left_cmd   : {left_cmd:.4f}")
    print(f"  right_cmd  : {right_cmd:.4f}")

    motors = None
    if HARDWARE_AVAILABLE:
        try:
            motors = DaguWheelsDriver()
            print("[INFO] Motors initialized.\n")
        except Exception as e:
            print(f"[ERROR] Motor init failed: {e}")
            return

    input("  Press ENTER to drive...")

    try:
        if motors:
            motors.set_wheels_speed(left=left_cmd, right=right_cmd)
        else:
            print(f"  [DRY-RUN] Driving for {duration:.2f}s...")

        time.sleep(duration)

    except KeyboardInterrupt:
        print("\n  Interrupted.")

    finally:
        if motors:
            motors.set_wheels_speed(left=0.0, right=0.0)
            motors.close()
        print("  Stopped.")


if __name__ == "__main__":
    main()
