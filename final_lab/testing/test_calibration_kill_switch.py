from controllers.grabber_controller import GrabberController
from data.ports_brick import Ports
from utils.brick import wait_ready_sensors, reset_brick, Motor
import threading

kill_switch_flag = threading.Event()
grabber_controller = GrabberController(kill_switch_flag,Ports.GRABBER_MOTOR.value, Ports.GYRO_SENSOR.value, calibration=True, debug=False)

try:
    while True:
        print("waiting until ")
except KeyboardInterrupt:
    reset_brick()

