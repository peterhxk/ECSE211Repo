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

stop_event = threading.Event()

try:
    def worker():
        routing.pharmacy_to_grab_block_position()
        routing.grab_block(grabber_controller)
        routing.pharmacy_to_room1()
        room_traverser.traverse_a_room(room_type=RoomType.CORNER_SINGLE, grabber_controller=grabber_controller)
        # routing.room1_to_pharmacy() # Week Demo
        # sound_controller.play_mission_complete()
        time.sleep(1)

        routing.room1_to_room2()
        room_traverser.traverse_a_room(room_type=RoomType.WALL_SINGLE, grabber_controller=grabber_controller)
        sound_controller.play_mission_complete()
    
    def check_stop_event():
        while True:
            user_input = input("Enter k to kill the process: ")
            if user_input == 'k':
                stop_event.set()
    
    main_worker = threading.Thread(target=worker, daemon=True)
    stop_event_worker = threading.Thread(target=check_stop_event, daemon=True)

    main_worker.start()
    stop_event_worker.start()

    stop_event.wait()
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
