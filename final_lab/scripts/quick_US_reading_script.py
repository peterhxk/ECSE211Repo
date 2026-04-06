# just a script to read the US distances i need to set for room traversal

from utils.brick import EV3UltrasonicSensor, wait_ready_sensors
import time
from data.ports_brick import Ports

US_SENSOR = EV3UltrasonicSensor(Ports.ULTRASONIC_SENSOR.value)
wait_ready_sensors(True)

while True:
    print(f"US distance: {US_SENSOR.get_value()} cm")
    time.sleep(0.1)