# from utils.brick import Color, wait_ready_sensors
# KILL_SWITCH = TouchSensor(4)

from utils.brick import EV3UltrasonicSensor, wait_ready_sensors, TouchSensor, reset_brick
from time import sleep

DELAY_SEC = 0.1  # seconds of delay between measurements

# complete this based on your hardware setup
US_SENSOR = EV3UltrasonicSensor(3)

wait_ready_sensors(True) # Input True to see what the robot is trying to initialize! False to be silent.


def collect_color_sensor_data():
    "Collect color sensor data."
    try:
        # loop for intaking data
        while True:
            data = US_SENSOR.get_value()
            print(f"raw_value: {data}")
            sleep(DELAY_SEC)


    except BaseException:  # capture all exceptions including KeyboardInterrupt (Ctrl-C)
        pass
    finally:
        print("Done collecting US distance samples")
        reset_brick() # Turn off everything on the brick's hardware, and reset it
        exit()

if __name__ == "__main__":
    collect_color_sensor_data()


