from utils.brick import wait_ready_sensors, reset_brick, Motor
from controllers.color_sensor_controller import ColorSensor
from pathlib import Path


#Sensor declaration

left_motor = Motor("C")
right_motor = Motor("B")
color_sensor = ColorSensor(2)

wait_ready_sensors(True)

try:
    while True:
        print(color_sensor.read_colour())
except KeyboardInterrupt:
    print("Interrupted via keyboard...")
except Exception as err:
    print(err)
finally:
    reset_brick() # Turn off everything on the brick's hardware, and reset it
    exit()