#!/usr/bin/env python3
import time

from motorDriver import DaguWheelsDriver
from encoderDriver import WheelEncoderDriver

GPIO_LEFT_ENCODER = 12
GPIO_RIGHT_ENCODER = 35

# Set a moderate speed to prevent wheel slip during calibration
TEST_VELOCITY = 0.2    
# Run for 10 seconds to gather a "considerable amount" of samples
TEST_DURATION = 10.0   

def main():
    print("--- Starting Automated Trim Calibration ---")
    
    # 1. Initialize Hardware
    try:
        left_encoder = WheelEncoderDriver(GPIO_LEFT_ENCODER)
        right_encoder = WheelEncoderDriver(GPIO_RIGHT_ENCODER)
        motors = DaguWheelsDriver()
        print("Hardware initialized successfully.")
    except Exception as e:
        print(f"Hardware initialization failed: {e}")
        return

    try:
        # 2. Reset the tick counters to zero before starting
        left_encoder._ticks = 0
        right_encoder._ticks = 0
        
        print(f"Running motors at {TEST_VELOCITY} speed for {TEST_DURATION} seconds...")
        
        # 3. Start the motors
        motors.set_wheels_speed(left=TEST_VELOCITY, right=TEST_VELOCITY)
        
        # 4. Wait while the robot drives and the encoders collect data
        time.sleep(TEST_DURATION)
        
        # 5. Stop the motors immediately after the timer finishes
        motors.set_wheels_speed(left=0.0, right=0.0)
        
        # 6. Read the final sample sizes
        e_l = left_encoder._ticks
        e_r = right_encoder._ticks
        
        print("\n--- Calibration Results ---")
        print(f"Left Encoder Ticks:  {e_l}")
        print(f"Right Encoder Ticks: {e_r}")
        
        # 7. Calculate the Trim Value
        if (e_l + e_r) == 0:
            print("Error: No ticks registered. Check your encoder wiring and pin numbers.")
        else:
            # Formula: (Left - Right) / (Left + Right)
            trim = (e_l - e_r) / (e_l + e_r)
            print(f"\n>>> Calculated Trim Value: {trim:.4f} <<<")
            print(f"Update your kinematics.yaml with: trim: {trim:.4f}")
            
    except KeyboardInterrupt:
        print("\nCalibration manually interrupted by user.")
    finally:
        # Safety catch: Always ensure motors are stopped on exit
        motors.set_wheels_speed(left=0.0, right=0.0)
        motors.close()
        print("Motors safely shut down.")

if __name__ == '__main__':
    main()