#!/usr/bin/env python2

import Adafruit_SSD1306
from PIL import Image, ImageDraw, ImageFont
from time import sleep
import serial
import time
import json

INFO_COMMAND = b"HH"

def info_duckiebattery(serial_port="/dev/ttyACM0", baud_rate=9600):
    """
    Reads info about the Duckiebattery via USB serial connection.
    """
    command = INFO_COMMAND
    try:
        ser = serial.Serial(port=serial_port, baudrate=baud_rate, timeout=1)
        print(
            "Connected to Duckiebattery on {} at {} baud.".format(
                serial_port, baud_rate
            )
        )

        ser.write(command)

        response = ser.readline().decode("utf-8").replace('\x00', '').strip()

        data_dict = json.loads(response)

        ser.close()

        return data_dict

    except serial.SerialException as e:
        print("Serial exception occurred: {}".format(e))
    except Exception as e:
        print("An error occurred while sending the shutdown command: {}".format(e))
    return None  # Explicitly return None if an exception occurred

# Initialize the display (I2C address: 0x3C)
disp = Adafruit_SSD1306.SSD1306_128_64(rst=None, i2c_bus=1, i2c_address=0x3C)

disp.begin()
disp.clear()
disp.display()

width = disp.width
height = disp.height
image = Image.new('1', (width, height))

draw = ImageDraw.Draw(image)

font = ImageFont.load_default()

# Attempt to get battery info, handle None gracefully
battery_info = info_duckiebattery()
if battery_info is not None:
    battery_percentage = battery_info.get("SOC(%)", "N/A")
else:
    battery_percentage = "N/A"

draw.rectangle((0, 0, width, height), outline=0, fill=0)  # Clear the image
draw.text((10, 5), "Battery: {}%".format(battery_percentage), font=font, fill=255)
draw.text((10, 20), "Connection: {}".format("OK" if battery_percentage != "N/A" else "Failed"), font=font, fill=255)
draw.text((10, 35), "ToF: Active", font=font, fill=255)

disp.image(image)
disp.display()

while True:
    sleep(5)
