"""
Main control logic running the robot.

"""

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

# Sensor declaration

left_motor = Motor(Ports.LEFT_MOTOR.value)
right_motor = Motor(Ports.RIGHT_MOTOR.value)
color_sensor_motor = Motor(Ports.COLOR_MOTOR.value)

driving_controller = DrivingController(left_motor, right_motor, 200)
color_sensor_controller = ColourSensorController(Ports.COLOR_SENSOR.value, color_sensor_motor)
grabber_controller = GrabberController(Ports.GRABBER_MOTOR.value, Ports.GYRO_SENSOR.value, calibration=False, debug=False)
routing = Routing(color_sensor_controller, driving_controller)
sound_controller = SoundController()
room_traverser = RoomTraverser(left_motor, right_motor, 160, color_sensor_controller, sound_controller=sound_controller)

wait_ready_sensors(True)
grabber_controller.initialize_sensors()
print("Initialized grabber sensors")

'''
State of the robot needed in case the robot deviates
Block stored is stored separatly 
0 - before pickup
1 - after pickup
2 - after 1st room 
3 - after 2nd room 
4 - after room 3A 
5 - after all rooms (go to pharmacy)
'''
robot_state = 0

'''
Number of blocks in the robot. 
'''
blocks_stored = 0

# Exception in case routing fails at some point
class RoutingError(Exception):
    pass


try:
    # Initial routing
    routing.pharmacy_to_grab_block_position()
    routing.grab_block(grabber_controller)
    robot_state = 1
    blocks_stored = grabber_controller.bloc_count

    # Room 1
    routing.pharmacy_to_room1()
    room_traverser.traverse_a_room(room_type=RoomType.CORNER_SINGLE, grabber_controller=grabber_controller)
    robot_state = 2
    # UPDATE BLOCK_STORED 
    blocks_stored = grabber_controller.bloc_count

    # Room 2
    routing.room1_to_room2()
    room_traverser.traverse_a_room(room_type=RoomType.WALL_SINGLE, grabber_controller=grabber_controller)
    robot_state = 3
    # UPDATE BLOCK_STORED 
    blocks_stored = grabber_controller.bloc_count

    # Go back early 
    if (blocks_stored == 0):
        robot_state = 5
        routing.room2_to_pharmacy()
        sound_controller.play_mission_complete()
        time.sleep(1)
        reset_brick() # Turn off everything on the brick's hardware, and reset it
        exit()
        
    # Room 3A
    routing.room2_to_room3A()
    room_traverser.traverse_a_room(room_type=RoomType.CORNER_DOUBLE_FIRST_HALF, grabber_controller=grabber_controller)
    robot_state = 4
    # UPDATE BLOCK_STORED 
    blocks_stored = grabber_controller.bloc_count

    # Go back early 
    if (blocks_stored == 0):
        robot_state = 5
        routing.room3A_to_pharmacy()
        sound_controller.play_mission_complete()
        time.sleep(1)
        reset_brick() # Turn off everything on the brick's hardware, and reset it
        exit()

    # Room 3B
    routing.room3A_to_room3B() # Week Demo
    room_traverser.traverse_a_room(room_type=RoomType.CORNER_DOUBLE_FIRST_HALF, grabber_controller=grabber_controller)
    robot_state = 5
    # UPDATE BLOCK_STORED 
    blocks_stored = grabber_controller.bloc_count
    routing.room3B_to_pharmacy()

    sound_controller.play_mission_complete()
    time.sleep(1)

except RoutingError:
    # Use robot state and blocks stored to go to specific code segment
    pass

except KeyboardInterrupt:
    print("Interrupted via keyboard...")

except Exception as err:
    print(err)
    traceback.print_exc()
finally:
    reset_brick() # Turn off everything on the brick's hardware, and reset it
    exit()