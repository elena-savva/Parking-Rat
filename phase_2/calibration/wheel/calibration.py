#!/usr/bin/env python2
# coding: utf-8

import sys
import os

# Add current directory to path to allow local imports
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from motorScripts.motorDriver import *
from encoderScripts.encoderDriver import WheelEncoderDriver
from multiprocessing import Process, Manager

from time import sleep
import numpy as np

def run_encoder(shared_dict, dt, pin_r=18, pin_l=19):
    encoder_r = WheelEncoderDriver(pin_r)
    encoder_l = WheelEncoderDriver(pin_l)

    while True:
        shared_dict['ticks_r'] = encoder_r._ticks
        shared_dict['ticks_l'] = encoder_l._ticks
        sleep(dt)


def clamp(value, min_value=-1.0, max_value=1.0):
    return max(min_value, min(value, max_value))


class PIDController:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral = 0
        self.prev_error = 0

    def reset(self):
        self.integral = 0
        self.prev_error = 0

    def update(self, error, dt):
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt if dt > 0 else 0
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        self.prev_error = error
        return output

if __name__ == '__main__':
    dt = 0.1

    manager = Manager()
    shared_data = manager.dict()
    shared_data['ticks_r'] = 0
    shared_data['ticks_l'] = 0

    encoder_process = Process(target=run_encoder, args=(shared_data, dt))
    encoder_process.start()

    motor = DaguWheelsDriver()
    pid = PIDController(kp=0.0002, ki=0.001, kd=0.00)
    
    correction = 0

    try:
        print("Running motors with PID correction to stay straight...")

        base_speed = 0.6
        left_speed = base_speed
        right_speed = base_speed - correction
        # Set initial speeds
        motor.set_wheels_speed(left=left_speed, right=right_speed)

        prev_ticks_r = shared_data['ticks_r']
        prev_ticks_l = shared_data['ticks_l']

        while True:
            sleep(dt)

            curr_ticks_r = shared_data['ticks_r']
            curr_ticks_l = shared_data['ticks_l']

            delta_r = curr_ticks_r - prev_ticks_r
            delta_l = curr_ticks_l - prev_ticks_l

            error = delta_r - delta_l  # Want this to be 0

            correction = pid.update(error, dt)

            # Adjust speeds based on correction
            right_speed = clamp(base_speed - correction)
            left_speed = clamp(base_speed)

            motor.set_wheels_speed(left=left_speed, right=right_speed)

            print("ΔR: %s | ΔL: %s | error: %.4f | correction: %.4f | L: %.4f, R: %.4f" % (delta_r, delta_l, error, correction, left_speed, right_speed))
            
            prev_ticks_r = curr_ticks_r
            prev_ticks_l = curr_ticks_l

        motor.set_wheels_speed(left=0, right=0)

    except KeyboardInterrupt:
        print("Stopped by user.")

    finally:
        motor.close()
        encoder_process.terminate()
        encoder_process.join()