import time
from utils.brick import wait_ready_sensors, reset_brick, Motor
from controllers.color_sensor_controller import ColourSensorController
from impl.room_traverser import RoomTraverser, RoomType
from data.ports_brick import Ports
from controllers.grabber_controller import GrabberController
from controllers.driving_controller import DrivingController
from impl.routing import Routing
from controllers.sound_controller import SoundController


MOTOR_POWER = 260

# sensor declaration
left_motor = Motor(Ports.LEFT_MOTOR.value)
right_motor = Motor(Ports.RIGHT_MOTOR.value)
color_motor = Motor(Ports.COLOR_MOTOR.value)

driving_controller = DrivingController(left_motor, right_motor, 120)

color_sensor_controller = ColourSensorController(Ports.COLOR_SENSOR.value, color_motor)
grabber_controller = GrabberController(Ports.GRABBER_MOTOR.value, Ports.GYRO_SENSOR.value, calibration=False, debug=False)
sound_controller = SoundController()
room_traverser = RoomTraverser(driving_controller, color_sensor_controller, grabber_controller, sound_controller)
# grabber_controller = GrabberController(Ports.GRABBER_MOTOR.value, Ports.GYRO_SENSOR.value, calibration=False, debug=False)
routing = Routing(color_sensor_controller, driving_controller)

print("Waiting for sensors to be ready...")

wait_ready_sensors(True)

# grabber_controller.initialize_sensors()
# print("Initialized grabber sensors")
# time.sleep(2)
    

if __name__ == "__main__":
    try:
        # Example usage: traverse a corner room while detecting for a bed
        # room_traverser.traverse_a_room(room_type=RoomType.WALL_SINGLE, grabber_controller=grabber_controller)
        # room_traverser.traverse_a_room(room_type=RoomType.CORNER_SINGLE)
        # routing.room1_to_room2()
        # room_traverser.traverse_a_room(room_type=RoomType.WALL_SINGLE)
        room_traverser.align_to_wall()
        # room_traverser.run_double_room(room_type=RoomType.CORNER_DOUBLE_FIRST_HALF)
    except Exception as err:
        print(err)
    finally:
        reset_brick()