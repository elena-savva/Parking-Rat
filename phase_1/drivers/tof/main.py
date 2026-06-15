from tofDriver import VL53L0X
import time

sensor = VL53L0X()

try:
    while True:
        avg = 0

        # Take 10 measurements
        for i in range(10):
            distance = sensor.read_distance()

            if distance is not None:
                avg += distance

            time.sleep(0.01)

        # Calculate average
        avg = avg / 10

        print("Average Distance: {:.2f} mm".format(avg))

        time.sleep(0.1)

except KeyboardInterrupt:
    print("Measurement stopped by User")

finally:
    sensor.close()