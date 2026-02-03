#!/usr/bin/env python3

"""
This test is used to collect data from the color sensor.
It must be run on the robot.
"""

# Add your imports here, if any
from utils.brick import EV3ColorSensor, wait_ready_sensors, TouchSensor, reset_brick
from time import sleep


COLOR_SENSOR_DATA_FILE = "../data_analysis/color_sensor_red.csv"
DELAY_SEC = 0.01  # seconds of delay between measurements

# complete this based on your hardware setup
COLOR_SENSOR = EV3ColorSensor(2)
TOUCH_SENSOR = TouchSensor(1)

wait_ready_sensors(True) # Input True to see what the robot is trying to initialize! False to be silent.


def collect_color_sensor_data():
    "Collect color sensor data."
    pressed = False # boolean for only taking the value once upon press or hold
    try:
        output_file = open(COLOR_SENSOR_DATA_FILE, "w")
        # loop for intaking data
        while True:
            if not TOUCH_SENSOR.is_pressed():
                pressed = False
            if TOUCH_SENSOR.is_pressed():
                l_rgb = COLOR_SENSOR.get_rgb()
                if pressed == False:
                    print(f"raw_value: {l_rgb}")
                    if l_rgb is not None:
                        output_file.write(f"{l_rgb}\n")
                    pressed = True
            sleep(DELAY_SEC)


    except BaseException:  # capture all exceptions including KeyboardInterrupt (Ctrl-C)
        pass
    finally:
        print("Done collecting US distance samples")
        output_file.close()
        reset_brick() # Turn off everything on the brick's hardware, and reset it
        exit()

if __name__ == "__main__":
    collect_color_sensor_data()
