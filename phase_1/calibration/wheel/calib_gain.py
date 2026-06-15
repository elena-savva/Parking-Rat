#!/usr/bin/env python3
import time

from motorDriver import DaguWheelsDriver

# --- CALIBRATION SETTINGS ---
TEST_VELOCITY = 0.5    # Target speed in m/s
TEST_DURATION = 2.0    # How long to drive (seconds)

# Calculate what the math says the distance SHOULD be
EXPECTED_DISTANCE = TEST_VELOCITY * TEST_DURATION 

def main():
    print("--- Gain Calibration ---")
    print(f"The robot will attempt to drive at {TEST_VELOCITY} m/s for {TEST_DURATION} seconds.")
    print(f"Mathematically, it should travel exactly {EXPECTED_DISTANCE:.2f} meters.")
    
    try:
        motors = DaguWheelsDriver()
    except Exception as e:
        print(f"Hardware initialization failed: {e}")
        return

    try:
        input("\nPlace the robot on the floor, mark its STARTING line, and press ENTER...")
        
        print("\nDriving...")
        # Since trim is handled in ROS, we just apply the raw velocity here
        motors.set_wheels_speed(left=TEST_VELOCITY, right=TEST_VELOCITY)
        time.sleep(TEST_DURATION)
        motors.set_wheels_speed(left=0.0, right=0.0)
        
        print("\nRobot stopped!")
        actual_dist_str = input("Measure the ACTUAL distance traveled in meters (e.g., 0.85): ")
        actual_dist = float(actual_dist_str)
        
        # Calculate the new Gain
        if actual_dist <= 0:
            print("Distance must be greater than zero.")
        else:
            gain = EXPECTED_DISTANCE / actual_dist
            print(f"\n>>> Calculated Gain Multiplier: {gain:.4f} <<<")
            print(f"Update your kinematics.yaml with: gain: {gain:.4f}")
            
    except ValueError:
        print("Invalid input. Please enter a number for the distance.")
    except KeyboardInterrupt:
        print("\nCalibration interrupted.")
    finally:
        motors.set_wheels_speed(left=0.0, right=0.0)
        motors.close()

if __name__ == '__main__':
    main()