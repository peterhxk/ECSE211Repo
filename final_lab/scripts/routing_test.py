import sys

from impl.room_traverser import RoomTraverser, RoomType
sys.path.append("./")

import traceback
from utils.brick import wait_ready_sensors, reset_brick, Motor
from controllers.color_sensor_controller import ColourSensorController
from controllers.driving_controller import DrivingController
from impl.routing import Routing
from data.ports_brick import Ports
from controllers.grabber_controller import GrabberController

# Sensor declaration

left_motor = Motor(Ports.LEFT_MOTOR.value)
right_motor = Motor(Ports.RIGHT_MOTOR.value)
color_sensor_motor = Motor(Ports.COLOR_MOTOR.value)

driving_controller = DrivingController(left_motor, right_motor, 200)
color_sensor_controller = ColourSensorController(Ports.COLOR_SENSOR.value, color_sensor_motor)
grabber_controller = GrabberController(Ports.GRABBER_MOTOR.value, Ports.GYRO_SENSOR.value, calibration=False, debug=False)
routing = Routing(color_sensor_controller, driving_controller)
room_traverser = RoomTraverser(left_motor, right_motor, 160, color_sensor_controller)
sound_controller = SoundController()

wait_ready_sensors(True)
grabber_controller.initialize_sensors()
print("Initialized grabber sensors")

try:
    routing.pharmacy_to_grab_block_position()
    routing.grab_block(grabber_controller)
    routing.pharmacy_to_room1()
    room_traverser.traverse_a_room(room_type=RoomType.CORNER_SINGLE, grabber_controller=grabber_controller)
    routing.room1_to_pharmacy()

except KeyboardInterrupt:
    print("Interrupted via keyboard...")

except Exception as err:
    print(err)
    traceback.print_exc()
finally:
    reset_brick() # Turn off everything on the brick's hardware, and reset it
    exit()
