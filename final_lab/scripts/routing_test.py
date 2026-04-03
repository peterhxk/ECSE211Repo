import sys
import time

from impl.kill_switch import KillSwitchInterrupt
from impl.room_traverser import RoomTraverser, RoomType
sys.path.append(".")

import traceback
from utils.brick import wait_ready_sensors, reset_brick, Motor
from controllers.color_sensor_controller import ColourSensorController
from controllers.driving_controller import DrivingController
from impl.routing import Routing
from data.ports_brick import Ports
from controllers.grabber_controller import GrabberController
from controllers.sound_controller import SoundController
import threading

# Sensor declaration

kill_switch_flag = threading.Event()

left_motor = Motor(Ports.LEFT_MOTOR.value)
right_motor = Motor(Ports.RIGHT_MOTOR.value)
color_sensor_motor = Motor(Ports.COLOR_MOTOR.value)

driving_controller = DrivingController(left_motor, right_motor, 200)
color_sensor_controller = ColourSensorController(Ports.COLOR_SENSOR.value, color_sensor_motor, calibrate=False)
grabber_controller = GrabberController(kill_switch_flag,Ports.GRABBER_MOTOR.value, Ports.GYRO_SENSOR.value, calibration=False, debug=False)
routing = Routing(color_sensor_controller, driving_controller)
sound_controller = SoundController()
room_traverser = RoomTraverser(driving_controller, color_sensor_controller, sound_controller=sound_controller)

wait_ready_sensors(True)
grabber_controller.initialize_sensors()
print("Initialized grabber sensors")

try:
    def worker():
        routing.room2_to_room3A()

    main_worker = threading.Thread(target=worker, daemon=True)

    main_worker.start()

    kill_switch_flag.wait()
    print("[INFO] Kill switch activated. Terminating script...")

    raise KillSwitchInterrupt()

except KeyboardInterrupt:
    print("Interrupted via keyboard...")

except KillSwitchInterrupt:
    pass

except Exception as err:
    print(err)
    traceback.print_exc()
finally:
    reset_brick() # Turn off everything on the brick's hardware, and reset it
    exit()
