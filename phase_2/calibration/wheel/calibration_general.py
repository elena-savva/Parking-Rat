
#!/usr/bin/env python3
"""
Individual Motor Calibration Script (Option 3)
================================================
Both motors run simultaneously (robot drives forward), but we read each
encoder separately to calculate individual gain_left and gain_right.

This correctly handles motors with different physical characteristics
without needing to lift the robot or risk stalling from friction.
"""

import time
import math
import yaml
import os

from motorDriver import DaguWheelsDriver
from encoderDriver import WheelEncoderDriver

# ── Settings ─────────────────────────────────────────────────────────────────
TEST_VELOCITY    = 0.25   # Match V_CRUISE in pid_controller_node.py
TEST_DURATION    = 15.0   # 20s at 0.15 → ~3m of travel
WHEEL_RADIUS     = 0.0325 # meters — must match kinematics.yaml
TICKS_PER_REV    = 137.0
MAX_PHYSICAL_VEL = 0.5    # m/s at PWM=1.0 — must match kinematics.yaml

GPIO_LEFT_ENC    = 12
GPIO_RIGHT_ENC   = 35

YAML_PATH = os.path.expanduser(
    "~/EVC/workshops/asg3_v2_maxxing/src/motor_control/config/kinematics.yaml"
)
# ─────────────────────────────────────────────────────────────────────────────


def ticks_to_distance(ticks: int) -> float:
    """Convert encoder ticks to distance traveled (meters)."""
    return (ticks / TICKS_PER_REV) * 2.0 * math.pi * WHEEL_RADIUS


def update_yaml(gain_left: float, gain_right: float):
    """Update kinematics.yaml with individual motor gains."""
    with open(YAML_PATH, "r") as f:
        config = yaml.safe_load(f)

    params = config["kinematics_node"]["ros__parameters"]
    params["gain_left"]  = round(gain_left,  4)
    params["gain_right"] = round(gain_right, 4)
    # Reset shared gain/trim to neutral since individual gains take over
    params["gain"] = 1.0
    params["trim"] = 0.0

    with open(YAML_PATH, "w") as f:
        yaml.dump(config, f, default_flow_style=False)

    print(f"\nYAML updated at: {YAML_PATH}")


def main():
    print("=" * 55)
    print("  INDIVIDUAL MOTOR CALIBRATION")
    print("=" * 55)
    print(f"Both motors run at PWM={TEST_VELOCITY} for {TEST_DURATION}s.")
    print(f"Expected distance: {TEST_VELOCITY * MAX_PHYSICAL_VEL * TEST_DURATION:.2f}m")
    print("Each encoder is read separately to get individual gains.")
    print("\nPlace the robot on the floor and mark its starting line.")

    # ── Initialize hardware ───────────────────────────────────────────────────
    try:
        left_enc  = WheelEncoderDriver(GPIO_LEFT_ENC)
        right_enc = WheelEncoderDriver(GPIO_RIGHT_ENC)
        motors    = DaguWheelsDriver()
        print("Hardware initialized.")
    except Exception as e:
        print(f"Hardware init failed: {e}")
        return

    input("\nPress ENTER when ready...")

    try:
        # ── Reset encoders ────────────────────────────────────────────────────
        left_enc._ticks  = 0
        right_enc._ticks = 0

        # ── Drive both motors ─────────────────────────────────────────────────
        print(f"\nDriving for {TEST_DURATION} seconds...")
        motors.set_wheels_speed(left=TEST_VELOCITY, right=TEST_VELOCITY)
        time.sleep(TEST_DURATION)
        motors.set_wheels_speed(left=0.0, right=0.0)
        print("Stopped.")

        # ── Read encoders ─────────────────────────────────────────────────────
        ticks_l = left_enc._ticks
        ticks_r = right_enc._ticks
        print(f"\nEncoder ticks  →  Left: {ticks_l}   Right: {ticks_r}")

        if ticks_l == 0 or ticks_r == 0:
            print("WARNING: One or both encoders recorded zero ticks!")
            print("Check wiring before proceeding.")
            return

        # ── Calculate actual distances ────────────────────────────────────────
        dist_l = ticks_to_distance(ticks_l)
        dist_r = ticks_to_distance(ticks_r)

        print(f"Actual distance  →  Left: {dist_l:.4f}m   Right: {dist_r:.4f}m")

        # ── Expected distance ─────────────────────────────────────────────────
        expected_dist = TEST_VELOCITY * MAX_PHYSICAL_VEL * TEST_DURATION

        # ── Calculate individual gains ────────────────────────────────────────
        gain_left  = expected_dist / dist_l
        gain_right = expected_dist / dist_r

        # ── Motor asymmetry ratio ─────────────────────────────────────────────
        ratio = dist_l / dist_r if dist_r != 0 else 0

        # ── Print results ─────────────────────────────────────────────────────
        print("\n" + "=" * 55)
        print("  CALIBRATION RESULTS")
        print("=" * 55)
        print(f"  Expected distance : {expected_dist:.4f} m")
        print(f"  Left  traveled    : {dist_l:.4f} m  → gain_left  = {gain_left:.4f}")
        print(f"  Right traveled    : {dist_r:.4f} m  → gain_right = {gain_right:.4f}")
        print(f"  L/R ratio         : {ratio:.4f}  (1.0 = perfectly matched)")
        if ratio > 1.02:
            print("  → Left wheel traveled more → robot drifts LEFT")
            print("    gain_left < gain_right will slow left down to compensate")
        elif ratio < 0.98:
            print("  → Right wheel traveled more → robot drifts RIGHT")
            print("    gain_right < gain_left will slow right down to compensate")
        else:
            print("  → Motors are well matched!")
        print("=" * 55)

        # ── Save to YAML ──────────────────────────────────────────────────────
        save = input("\nSave to kinematics.yaml? [y/n]: ").strip().lower()
        if save == "y":
            update_yaml(gain_left, gain_right)
            print("\nValues saved successfully.")
            print("\n" + "=" * 55)
            print("  NEXT STEPS")
            print("=" * 55)
            print("  1. Rebuild:")
            print("       cd ~/EVC/workshops/asg3_v2_maxxing")
            print("       colcon build")
            print("       source install/setup.bash")
            print()
            print("  2. Launch:")
            print("       ros2 launch motor_control motor_control.launch.py")
            print()
            print("  3. Movement selector:")
            print("       ros2 run motor_control movement_node")
            print("=" * 55)
        else:
            print("Values NOT saved. Update kinematics.yaml manually:")
            print(f"  gain_left:  {gain_left:.4f}")
            print(f"  gain_right: {gain_right:.4f}")

    except KeyboardInterrupt:
        print("\nCalibration interrupted.")
    finally:
        motors.set_wheels_speed(left=0.0, right=0.0)
        motors.close()
        print("\nMotors shut down safely.")


if __name__ == "__main__":
    main()