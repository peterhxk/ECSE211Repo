import sys
sys.path.append("../")

import traceback
from utils.brick import wait_ready_sensors, reset_brick, Motor, TouchSensor
from controllers.color_sensor_controller import ColourSensorController
from controllers.driving_controller import DrivingController
from impl.sweep_line_follower import SweepLineFollower
import time
import data.measurements as consts
from controllers.grabber_controller import GrabberController
import threading

# Sensor declaration

left_motor = Motor("C")
right_motor = Motor("B")
color_sensor_motor = Motor("A")


driving_controller = DrivingController(left_motor, right_motor, 150)
color_sensor_controller = ColourSensorController(2, color_sensor_motor)
grabber_controller = GrabberController("D", 4, False, True)
touch_sensor = TouchSensor(1)

line_follower = SweepLineFollower(color_sensor_controller, driving_controller)

wait_ready_sensors(True)

grabber_controller.initialize_sensors()

def slow_decrease():
    global init_power
    while init_power > 30:
        init_power = max(29, init_power/2)
        time.sleep(1)
try:
    color_sensor_controller.set_arm_at_angle(85)
    line_follower.move_forward(100,force=True)
    init_power = 20
    driving_controller.move_forward_by_power(init_power)
    grabber_controller.start_loading()
    #threading.Thread(target=slow_decrease).start()
    while not grabber_controller.isBlockCaught() and not grabber_controller.get_loading_status() == "FAILED":
        pass
    print(f"[DEBUG] Status of loader is {grabber_controller.get_loading_status()}")
    if grabber_controller.isBlockCaught():
        print("[DEBUG] Block Caught ! Stopping drive")
    driving_controller.stop_motors()
    while grabber_controller.get_loading_status() == "SUCCESS":
        pass
    print("[DEBUG] Loading first block complete")

    #grabber_controller._end_operation_recording()

    time.sleep(10)
    # stop for now

    line_follower.move_forward(20,force=True)
    init_power = 60
    grabber_controller.load_bloc()
    driving_controller.move_forward_by_power(init_power)
    #threading.Thread(target=slow_decrease).start()
    while not grabber_controller.isBlockCaught() and not grabber_controller.get_loading_status() == "FAILED":
        pass
    driving_controller.stop_motors()
    color_sensor_controller.set_arm_at_angle(0)

except KeyboardInterrupt:
    color_sensor_controller.set_arm_at_angle(0)
    print("Interrupted via keyboard...")

except Exception as err:
    print(err)
    traceback.print_exc()
finally:
    reset_brick() # Turn off everything on the brick's hardware, and reset it
    exit()
