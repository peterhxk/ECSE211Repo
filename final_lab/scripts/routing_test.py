import sys
import time

from impl.room_traverser import RoomTraverser, RoomType
sys.path.append("./")

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

# Kill switch flag
kill_switch_flag = threading.Event()

left_motor = Motor(Ports.LEFT_MOTOR.value)
right_motor = Motor(Ports.RIGHT_MOTOR.value)
color_sensor_motor = Motor(Ports.COLOR_MOTOR.value)

driving_controller = DrivingController(left_motor, right_motor, 200)
color_sensor_controller = ColourSensorController(Ports.COLOR_SENSOR.value, color_sensor_motor, calibrate=False)
grabber_controller = GrabberController(kill_switch_flag,Ports.GRABBER_MOTOR.value, Ports.GYRO_SENSOR.value, calibration=False, debug=False)
routing = Routing(color_sensor_controller, driving_controller)
sound_controller = SoundController()
room_traverser = RoomTraverser(left_motor, right_motor, 160, color_sensor_controller, sound_controller=sound_controller)

wait_ready_sensors(True)
grabber_controller.initialize_sensors()
print("Initialized grabber sensors")

try:
     routing.pharmacy_to_grab_block_position()
     routing.grab_block(grabber_controller)
     routing.pharmacy_to_room1()
    # routing.room1_to_room2()
    #routing.room2_to_room3A()
    # routing.room3A_to_room3B()
    # routing.room3B_to_pharmacy()

except KeyboardInterrupt:
    print("Interrupted via keyboard...")

except Exception as err:
    print(err)
    traceback.print_exc()
finally:
    reset_brick() # Turn off everything on the brick's hardware, and reset it
    exit()
