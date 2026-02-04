import sys

sys.path.append("../")

from utils.brick import TouchSensor, wait_ready_sensors
KILL_SWITCH = TouchSensor(3)

wait_ready_sensors(True)