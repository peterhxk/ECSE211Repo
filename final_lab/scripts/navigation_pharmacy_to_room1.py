import sys
sys.path.append(".")

import traceback
from utils.brick import wait_ready_sensors, reset_brick, Motor
from controllers.color_sensor_controller import ColourSensorController
from controllers.driving_controller import DrivingController
from impl.routing import Routing
from data.ports_brick import Ports

# Sensor declaration

left_motor = Motor(Ports.LEFT_MOTOR)
right_motor = Motor(Ports.RIGHT_MOTOR)
color_sensor_motor = Motor(Ports.COLOR_MOTOR)

driving_controller = DrivingController(left_motor, right_motor, 150)
color_sensor_controller = ColourSensorController(Ports.COLOR_SENSOR, color_sensor_motor)

routing = Routing(color_sensor_controller, driving_controller)

wait_ready_sensors(True)

try:
    routing.pharmacy_to_room1()
except KeyboardInterrupt:
    print("Interrupted via keyboard...")

except Exception as err:
    print(err)
    traceback.print_exc()
finally:
    reset_brick() # Turn off everything on the brick's hardware, and reset it
    exit()
