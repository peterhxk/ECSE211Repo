from time import time
from utils.brick import wait_ready_sensors, reset_brick, Motor
from controllers.color_sensor_controller import ColourSensorController
from controllers.driving_controller import DrivingController
from impl.room_traverser import RoomTraverser
from data.ports_brick import Ports

MOTOR_POWER = 80

# sensor declaration
left_motor = Motor(Ports.LEFT_MOTOR)
right_motor = Motor(Ports.RIGHT_MOTOR)
color_motor = Motor(Ports.COLOR_MOTOR)

driving_controller = DrivingController(left_motor, right_motor, MOTOR_POWER)
color_sensor_controller = ColourSensorController(Ports.COLOR_SENSOR, color_motor)
room_traverser = RoomTraverser(driving_controller, color_sensor_controller)

print("Waiting for sensors to be ready...")

wait_ready_sensors(True)

if __name__ == "__main__":
    try:
        # Example usage: traverse a corner room while detecting for a bed
        room_traverser.traverse_a_room(room_type=RoomTraverser.RoomType.CORNER_SINGLE)
    except Exception as err:
        print(err)
    finally:
        reset_brick()