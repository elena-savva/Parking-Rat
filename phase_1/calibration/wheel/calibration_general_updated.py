#!/usr/bin/env python3
"""
Diagnostic Motor Calibration — Ground-Truth Edition
====================================================
Drives both motors at equal PWM, then asks the operator for tape-measure
ground-truth measurements of where the robot ended up. From three simple
numbers (forward, lateral, drift side) it derives:

  - The actual arc the robot traveled (heading change, radius of curvature)
  - The ground distance each wheel ACTUALLY traveled (not the encoder lie)
  - The slip ratio per wheel (how much the encoder over-reports vs reality)
  - Suggested gain values calibrated to ground distance, not encoder ticks

Why this matters: encoders measure rotation, not ground motion. On
low-friction surfaces, wheel slip means the encoder can underreport the
real left/right asymmetry by 10× or more. The original encoder-only
calibration script gave wrong-sign gains because of this.

Geometry (assuming constant-curvature arc starting at origin facing +x):
    forward = R · sin(θ)
    lateral = R · (1 − cos(θ))
    →  θ = 2 · atan2(lateral, forward)
    →  R = forward / sin(θ)
    outer wheel arc = (|R| + b/2) · |θ|
    inner wheel arc = (|R| − b/2) · |θ|
"""

import time
import math
import yaml
import os

from motorDriver import DaguWheelsDriver
from encoderDriver import WheelEncoderDriver

# ── Settings (match the rest of the stack) ──────────────────────────────────
TEST_VELOCITY    = 0.25
TEST_DURATION    = 8.0
WHEEL_RADIUS     = 0.0325
TICKS_PER_REV    = 137.0
MAX_PHYSICAL_VEL = 0.5
BASELINE         = 0.185

GPIO_LEFT_ENC    = 12
GPIO_RIGHT_ENC   = 35

YAML_PATH = os.path.expanduser(
    "~/EVC/workshops/asg3_v2_maxxing/src/motor_control/config/kinematics.yaml"
)


# ── Helpers ──────────────────────────────────────────────────────────────────

def ticks_to_distance(ticks: int) -> float:
    return (ticks / TICKS_PER_REV) * 2.0 * math.pi * WHEEL_RADIUS


def ask_float(prompt: str) -> float:
    while True:
        try:
            return float(input(prompt).strip())
        except ValueError:
            print("  → please enter a number")


def ask_side(prompt: str) -> int:
    """Return +1 for left, −1 for right, 0 for straight."""
    while True:
        s = input(prompt).strip().lower()
        if s in ("l", "left"):    return +1
        if s in ("r", "right"):   return -1
        if s in ("s", "straight", ""): return 0
        print("  → please type 'left', 'right', or 'straight'")


def compute_arc_geometry(forward_m, lateral_m, baseline):
    """
    Returns (theta_rad, radius_m, d_center, d_left, d_right).

    Sign convention (ROS standard):
      + lateral = drifted LEFT
      + theta   = turned LEFT  (CCW)
    """
    if abs(lateral_m) < 1e-4:
        # Straight line
        return 0.0, float('inf'), forward_m, forward_m, forward_m

    # Heading change derived from end position (geometric identity for arcs)
    theta = 2.0 * math.atan2(lateral_m, forward_m)
    abs_theta = abs(theta)

    # Radius of curvature
    abs_R = abs(forward_m / math.sin(theta))
    d_center = abs_R * abs_theta

    if theta > 0:                       # left turn → right wheel is outer
        d_left  = (abs_R - baseline / 2.0) * abs_theta
        d_right = (abs_R + baseline / 2.0) * abs_theta
    else:                               # right turn → left wheel is outer
        d_left  = (abs_R + baseline / 2.0) * abs_theta
        d_right = (abs_R - baseline / 2.0) * abs_theta

    return theta, abs_R, d_center, d_left, d_right


def update_yaml(gain_left, gain_right):
    with open(YAML_PATH, "r") as f:
        config = yaml.safe_load(f)
    p = config["kinematics_node"]["ros__parameters"]
    p["gain_left"]  = round(gain_left,  4)
    p["gain_right"] = round(gain_right, 4)
    p["gain"]       = 1.0
    p["trim"]       = 0.0
    with open(YAML_PATH, "w") as f:
        yaml.dump(config, f, default_flow_style=False)


# ── Main ─────────────────────────────────────────────────────────────────────

def main():
    print("=" * 66)
    print("  DIAGNOSTIC MOTOR CALIBRATION  (Ground-Truth Edition)")
    print("=" * 66)
    print(f"  PWM={TEST_VELOCITY} for {TEST_DURATION}s on both wheels.")
    print(f"  Ideal forward distance: "
          f"{TEST_VELOCITY * MAX_PHYSICAL_VEL * TEST_DURATION:.2f} m")
    print()
    print("  Setup:")
    print("    1. Flat surface, robot at a marked start position")
    print("    2. Mark a STRAIGHT LINE on the floor under the wheel axle")
    print("    3. Note which direction the robot is pointing (so you can")
    print("       measure forward/lateral relative to the START heading)")
    print("    4. Have a tape measure ready")
    print("=" * 66)

    try:
        left_enc  = WheelEncoderDriver(GPIO_LEFT_ENC)
        right_enc = WheelEncoderDriver(GPIO_RIGHT_ENC)
        motors    = DaguWheelsDriver()
    except Exception as e:
        print(f"\n  Hardware init failed: {e}")
        return

    input("\n  Press ENTER to drive ...")

    try:
        left_enc._ticks  = 0
        right_enc._ticks = 0

        print(f"\n  Driving for {TEST_DURATION} s ...")
        motors.set_wheels_speed(left=TEST_VELOCITY, right=TEST_VELOCITY)
        time.sleep(TEST_DURATION)
        motors.set_wheels_speed(left=0.0, right=0.0)
        print("  Stopped.\n")

        # ── Encoder data ──────────────────────────────────────────────────────
        ticks_l    = left_enc._ticks
        ticks_r    = right_enc._ticks
        dist_l_enc = ticks_to_distance(ticks_l)
        dist_r_enc = ticks_to_distance(ticks_r)
        expected   = TEST_VELOCITY * MAX_PHYSICAL_VEL * TEST_DURATION

        print("─" * 66)
        print("  ENCODER READINGS   (what the wheels rotated, ignoring slip)")
        print("─" * 66)
        print(f"  Left  : {ticks_l:5d} ticks  →  {dist_l_enc:6.3f} m")
        print(f"  Right : {ticks_r:5d} ticks  →  {dist_r_enc:6.3f} m")
        print(f"  Diff  : {dist_l_enc - dist_r_enc:+.3f} m  (L − R)")

        # ── Ground-truth prompts ──────────────────────────────────────────────
        print("\n" + "─" * 66)
        print("  GROUND-TRUTH MEASUREMENT")
        print("─" * 66)
        print("  Measure from the robot's start mark to where it ended up.")
        print("  'Forward' is in the direction the robot was originally facing.")
        print("  'Lateral' is perpendicular drift (just the magnitude, in cm).")
        print()
        forward_cm = ask_float("  Forward distance from start (cm): ")
        lateral_cm = ask_float("  Lateral drift magnitude (cm): ")
        side       = ask_side("  Drifted to which side? (left/right/straight): ")

        forward_m = forward_cm / 100.0
        lateral_m = (side * lateral_cm) / 100.0   # + = left, − = right

        theta, R, d_center, d_left_gnd, d_right_gnd = compute_arc_geometry(
            forward_m, lateral_m, BASELINE
        )

        # ── Slip ratios ───────────────────────────────────────────────────────
        slip_l = dist_l_enc / d_left_gnd  if d_left_gnd  > 1e-3 else float('inf')
        slip_r = dist_r_enc / d_right_gnd if d_right_gnd > 1e-3 else float('inf')

        # ── Gain calculations ────────────────────────────────────────────────
        gain_l_gt  = expected / d_left_gnd  if d_left_gnd  > 1e-3 else 1.0
        gain_r_gt  = expected / d_right_gnd if d_right_gnd > 1e-3 else 1.0
        gain_l_enc = expected / dist_l_enc if dist_l_enc > 1e-3 else 1.0
        gain_r_enc = expected / dist_r_enc if dist_r_enc > 1e-3 else 1.0

        # ── Report ────────────────────────────────────────────────────────────
        print("\n" + "=" * 66)
        print("  DIAGNOSTIC RESULTS")
        print("=" * 66)

        print(f"\n  Path geometry (from your measurements):")
        side_label = "LEFT" if side > 0 else "RIGHT" if side < 0 else "straight"
        print(f"    End position     : forward={forward_m:.3f} m  "
              f"lateral={lateral_m:+.3f} m  ({side_label})")
        print(f"    Heading change   : {math.degrees(theta):+6.1f}°")
        if math.isfinite(R):
            print(f"    Arc radius       : {R:.3f} m")
        else:
            print(f"    Arc radius       : ∞ (straight line)")
        print(f"    Center traveled  : {d_center:.3f} m")

        print(f"\n  Per-wheel distances:")
        print(f"    {'':14} {'Encoder':>10}  {'Ground':>10}   {'Slip':>6}")
        print(f"    Left  wheel : {dist_l_enc:8.3f} m  {d_left_gnd:8.3f} m   "
              f"{slip_l:5.2f}×")
        print(f"    Right wheel : {dist_r_enc:8.3f} m  {d_right_gnd:8.3f} m   "
              f"{slip_r:5.2f}×")
        print(f"\n  Slip ratio = encoder / ground.  1.0 = no slip.")
        print(f"  > 1.0 = wheel spun more than it actually moved on the floor.")

        if abs(slip_l - slip_r) < 0.1:
            verdict = "Wheels have similar slip; drift is from motor asymmetry."
        elif slip_l > slip_r:
            verdict = ("LEFT wheel slips more → robot drifts LEFT under equal "
                       "PWM (right pushes more effectively).")
        else:
            verdict = ("RIGHT wheel slips more → robot drifts RIGHT under "
                       "equal PWM (left pushes more effectively).")
        print(f"\n  → {verdict}")

        print(f"\n  Suggested gains (calibrated to GROUND distance):")
        print(f"    gain_left  : {gain_l_gt:.4f}")
        print(f"    gain_right : {gain_r_gt:.4f}")
        print(f"    Ratio L/R  : {gain_l_gt / gain_r_gt:.4f}")
        print(f"    (Higher gain on a wheel = more PWM to that wheel)")

        print(f"\n  For comparison, the OLD encoder-only method would suggest:")
        print(f"    gain_left  : {gain_l_enc:.4f}   ← wrong-sign on slippery floor")
        print(f"    gain_right : {gain_r_enc:.4f}")
        print("=" * 66)

        # ── Save? ─────────────────────────────────────────────────────────────
        save = input("\n  Save ground-truth gains to kinematics.yaml? [y/n]: ") \
                  .strip().lower()
        if save == "y":
            update_yaml(gain_l_gt, gain_r_gt)
            print(f"\n  Saved to {YAML_PATH}")
            print("  Next: colcon build && source install/setup.bash")
        else:
            print("\n  Not saved.  Apply manually if you want:")
            print(f"    gain_left:  {gain_l_gt:.4f}")
            print(f"    gain_right: {gain_r_gt:.4f}")

    except KeyboardInterrupt:
        print("\n  Interrupted.")
    finally:
        motors.set_wheels_speed(left=0.0, right=0.0)
        motors.close()
        print("\n  Motors shut down.\n")


if __name__ == "__main__":
    main()