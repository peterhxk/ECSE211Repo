"""
Main control logic running the robot.

"""

import sys
import threading
import time
import traceback

sys.path.append("./")

from impl.kill_switch import KillSwitchInterrupt
from impl.room_traverser import RoomTraverser, RoomType
from utils.brick import wait_ready_sensors, reset_brick, Motor
from controllers.color_sensor_controller import ColourSensorController
from controllers.driving_controller import DrivingController
from impl.routing import Routing
from data.ports_brick import Ports
from controllers.grabber_controller import GrabberController
from controllers.sound_controller import SoundController

# Sensor declaration

kill_switch_flag = threading.Event()

left_motor = Motor(Ports.LEFT_MOTOR.value)
right_motor = Motor(Ports.RIGHT_MOTOR.value)
color_sensor_motor = Motor(Ports.COLOR_MOTOR.value)

driving_controller = DrivingController(left_motor, right_motor, 200)
color_sensor_controller = ColourSensorController(Ports.COLOR_SENSOR.value, color_sensor_motor)
grabber_controller = GrabberController(kill_switch_flag, Ports.GRABBER_MOTOR.value, Ports.GYRO_SENSOR.value, calibration=False, debug=False)
routing = Routing(color_sensor_controller, driving_controller)
sound_controller = SoundController()
room_traverser = RoomTraverser(driving_controller, color_sensor_controller, grabber_controller, sound_controller)

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
    def state_machine():
        global robot_state
        global blocks_stored

        # Initial routing
        routing.pharmacy_to_grab_block_position()
        routing.grab_block(grabber_controller)
        robot_state = 1
        blocks_stored = grabber_controller.cube_count

        # Room 1
        routing.pharmacy_to_room1()
        room_traverser.traverse_a_room(room_type=RoomType.CORNER_SINGLE, grabber_controller=grabber_controller)
        robot_state = 2
        # UPDATE BLOCK_STORED 
        blocks_stored = grabber_controller.cube_count

        # Room 2
        routing.room1_to_room2()
        room_traverser.traverse_a_room(room_type=RoomType.WALL_SINGLE, grabber_controller=grabber_controller)
        robot_state = 3
        # UPDATE BLOCK_STORED 
        blocks_stored = grabber_controller.cube_count

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
        blocks_stored = grabber_controller.cube_count

        # Go back early 
        if (blocks_stored == 0):
            robot_state = 5
            routing.room3A_to_pharmacy()
            sound_controller.play_mission_complete()
            time.sleep(1)
            reset_brick() # Turn off everything on the brick's hardware, and reset it
            exit()

        # Room 3B
        routing.room3A_to_room3C() # Week Demo
        room_traverser.traverse_a_room(room_type=RoomType.CORNER_DOUBLE_FIRST_HALF, grabber_controller=grabber_controller)
        robot_state = 5
        # UPDATE BLOCK_STORED 
        blocks_stored = grabber_controller.cube_count
        routing.room3C_to_pharmacy()

        sound_controller.play_mission_complete()
        time.sleep(1)
    
    def worker():
        # routing.pharmacy_to_grab_block_position()
        # routing.grab_block(grabber_controller)

        # routing.pharmacy_to_room1()
        # room_traverser.traverse_a_room(RoomType.CORNER_SINGLE)

        # routing.room1_to_room2()
        # room_traverser.traverse_a_room(RoomType.WALL_SINGLE)

        # routing.room2_to_room3A()
        room_traverser.traverse_a_room(RoomType.CORNER_DOUBLE_FIRST_HALF)

        routing.room3A_to_room3C()
        room_traverser.traverse_a_room(RoomType.CORNER_DOUBLE_SECOND_HALF)

        routing.room3C_to_room3B()
        room_traverser.traverse_a_room(RoomType.CORNER_DOUBLE_MIDDLE)

        # routing.room3A_to_pharmacy()

        # sound_controller.play_mission_complete()
        # sound_controller.wait_done()

        # # Check if we got lucky and found the 2 green beds right away
        # if grabber_controller.cube_count == 0:
        #     routing.room2_to_pharmacy()
        # else:
        #     routing.room2_to_room3A()
        #     room_traverser.traverse_a_room(RoomType.CORNER_DOUBLE_FIRST_HALF)

        #     if grabber_controller.cube_count == 0:
        #         routing.room3A_to_pharmacy()
        #     else:
        #         routing.room3A_to_room3B()
        #         room_traverser.traverse_a_room(RoomType.CORNER_DOUBLE_SECOND_HALF)
        #         routing.room3B_to_pharmacy()

        # sound_controller.play_mission_complete()
        # sound_controller.wait_done()

    control_thread = threading.Thread(target=worker, daemon=True)

    control_thread.start()

    kill_switch_flag.wait()

    print("[INFO] Kill switch activated. Terminating script...")

    raise KillSwitchInterrupt()

except RoutingError:
    # Use robot state and blocks stored to go to specific code segment
    pass

except KillSwitchInterrupt:
    pass

except KeyboardInterrupt:
    print("Interrupted via keyboard...")

except Exception as err:
    print(err)
    traceback.print_exc()
finally:
    reset_brick() # Turn off everything on the brick's hardware, and reset it
    exit()