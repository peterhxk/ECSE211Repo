from controllers.grabber_controller import GrabberController
from data.ports_brick import Ports
from utils.brick import wait_ready_sensors, reset_brick, Motor
import threading
import time

kill_switch_flag = threading.Event()
grabber_controller = GrabberController(kill_switch_flag,Ports.GRABBER_MOTOR.value, Ports.GYRO_SENSOR.value, calibration=True, debug=True)
wait_ready_sensors(True)
grabber_controller.initialize_sensors()

try:
    start = time.time()
    while True:
        if time.time() - start > 1:
            print("Waiting for kill switch...")
        if kill_switch_flag.is_set():
            print("kill switch flag was activated")
            break

except KeyboardInterrupt:
    reset_brick()
#
