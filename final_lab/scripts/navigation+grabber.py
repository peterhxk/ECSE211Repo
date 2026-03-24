import sys
sys.path.append("../")

import traceback
from utils.brick import wait_ready_sensors, reset_brick, Motor, TouchSensor
from controllers.color_sensor_controller import ColourSensorController
from controllers.driving_controller import DrivingController
from impl.sweep_line_follower import SweepLineFollower
import time
from controllers.grabber_controller import GrabberController
from data.ports_brick import Ports

# Sensor declaration

left_motor = Motor(Ports.LEFT_MOTOR.value)
right_motor = Motor(Ports.RIGHT_MOTOR.value)
color_sensor_motor = Motor(Ports.COLOR_SENSOR.value)


driving_controller = DrivingController(left_motor, right_motor, 150)
color_sensor_controller = ColourSensorController(Ports.COLOR_SENSOR.value, color_sensor_motor)
grabber_controller = GrabberController(Ports.GRABBER_MOTOR.value, Ports.GYRO_SENSOR.value,True, True)
#touch_sensor = TouchSensor(1)

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
    init_power = 15
    driving_controller.move_forward_by_power(init_power)
    grabber_controller.start_loading()
    print("moved past start loading")
    #threading.Thread(target=slow_decrease).start()
    starter = time.time()
    while not grabber_controller.isBlockCaught() and grabber_controller.get_loading_status() == "LOADING":
        if time.time() - starter > 0.6:
            print("Still loading")
            print(grabber_controller.get_loading_status())
            starter = time.time()

    print(f"[DEBUG] Status of loader is {grabber_controller.get_loading_status()}")
    if grabber_controller.isBlockCaught():
        print("[DEBUG] Block Caught ! Stopping drive")
    driving_controller.stop_motors()
    grabber_controller.clearBlockCaught()
    start = time.time()
    watchdog = time.time()
    while grabber_controller.is_loading():
        if time.time() - start > 1:
            print("[DEBUG] Status of loader is {}".format(grabber_controller.get_loading_status()))
            start = time.time()
        if grabber_controller.get_loading_status() == "FAILED":
            raise RuntimeError("Failure to catch block. Should we back up the robot and start over ?")
#        if time.time() - watchdog > 0.6:
#            grabber_controller.increase_on_velocity_drop()
#            watchdog = time.time()
    print("[DEBUG] Loading first block complete")

    #grabber_controller._end_operation_recording()
    #grabber_controller._end_operation_recording()
    print("[DEBUG] Loading first block complete. Waiting 5s before loading second block")
    if grabber_controller.bloc_count != 1:
        raise RuntimeError("Failure to catch block.First block not registered")
    time.sleep(5)
    # stop for now

    line_follower.move_forward(20,force=True)
    init_power = 15
    grabber_controller.load_bloc()
    driving_controller.move_forward_by_power(init_power)
    #threading.Thread(target=slow_decrease).start()
    starter = time.time()
    while not grabber_controller.isBlockCaught() and not grabber_controller.get_loading_status() == "FAILED":
        if time.time() - starter > 1: 
          print("waiting for block to get caught")
          starter = time.time()
    driving_controller.stop_motors()
    grabber_controller.clearBlockCaught()
    while grabber_controller.is_loading():
        print("[DEBUG] Status of loader is {}".format(grabber_controller.get_loading_status()))
        if  grabber_controller.get_loading_status() == "FAILED":
            raise RuntimeError("Failure to catch block. Should we back up the robot and start over ?")
    print("[DEBUG] Loading second block complete")

    color_sensor_controller.set_arm_at_angle(0)

    time.sleep(5)
    # stop for now
    init_power = 15
    line_follower.move_forward(20,force=True)
    driving_controller.stop_motors()
    grabber_controller.start_unloading()
    starter = time.time()
    while grabber_controller.get_unloading_status() == "UNLOADING":
        if time.time() - starter > 1:
            print("waiting for block to unload")
            starter = time.time()
    if grabber_controller.get_unloading_status() == "FAILED" or grabber_controller.bloc_count != 1:
        print(grabber_controller.get_unloading_status())
        print(grabber_controller.bloc_count)
        raise RuntimeError("Failure to unload block. Should we back up the robot and start over ?")
    print("Successfully unloaded block!")

    time.sleep(5)
    init_power = 15
    line_follower.move_forward(20,force=True)
    driving_controller.stop_motors()
    grabber_controller.start_unloading()
    starter = time.time()
    while grabber_controller.get_unloading_status() == "UNLOADING":
        if time.time() - starter > 1:
            print("waiting for block to unload")
            starter = time.time()
    if grabber_controller.get_unloading_status() == "FAILED" or grabber_controller.bloc_count != 0:
        print(grabber_controller.get_unloading_status())
        print(grabber_controller.bloc_count)
        raise RuntimeError("Failure to unload last block. Should we back up the robot and start over ?")
    print("Successfully unloaded block!")

except KeyboardInterrupt:
    color_sensor_controller.set_arm_at_angle(0)
    print("Interrupted via keyboard...")

except Exception as err:
    print(err)
    traceback.print_exc()
finally:
    reset_brick() # Turn off everything on the brick's hardware, and reset it
    exit()
