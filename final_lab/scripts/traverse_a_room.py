import time
from utils.brick import wait_ready_sensors, reset_brick, Motor
from controllers.color_sensor_controller import ColourSensorController
from impl.room_traverser import RoomTraverser, RoomType
from data.ports_brick import Ports
from controllers.grabber_controller import GrabberController
from controllers.driving_controller import DrivingController
from impl.routing import Routing


MOTOR_POWER = 170

# sensor declaration
left_motor = Motor(Ports.LEFT_MOTOR.value)
right_motor = Motor(Ports.RIGHT_MOTOR.value)
color_motor = Motor(Ports.COLOR_MOTOR.value)

driving_controller = DrivingController(left_motor, right_motor, 150)

color_sensor_controller = ColourSensorController(Ports.COLOR_SENSOR.value, color_motor)
room_traverser = RoomTraverser(left_motor, right_motor, MOTOR_POWER, color_sensor_controller)
grabber_controller = GrabberController(Ports.GRABBER_MOTOR.value, Ports.GYRO_SENSOR.value, False, False)
routing = Routing(color_sensor_controller, driving_controller)

print("Waiting for sensors to be ready...")

wait_ready_sensors(True)

try:
    grabber_controller.initialize_sensors()
    print("Initialized grabber sensors")
    time.sleep(2)
except Exception as err:
    print(err)
    

if __name__ == "__main__":
    try:
        # Example usage: traverse a corner room while detecting for a bed
        room_traverser.traverse_a_room(room_type=RoomType.CORNER_SINGLE, grabber_controller=grabber_controller)
    except Exception as err:
        print(err)
    finally:
        reset_brick()