
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
import threading

# Sensor declaration

left_motor = Motor(Ports.LEFT_MOTOR.value)
right_motor = Motor(Ports.RIGHT_MOTOR.value)
color_sensor_motor = Motor(Ports.COLOR_MOTOR.value)


driving_controller = DrivingController(left_motor, right_motor, 100)
color_sensor_controller = ColourSensorController(Ports.COLOR_SENSOR.value, color_sensor_motor)
grabber_controller = GrabberController(Ports.GRABBER_MOTOR.value, Ports.GYRO_SENSOR.value,False, True)
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
    driving_controller.set_speed(100)
    driving_controller.move_distance(100) # move 100 mm before the first block
    left_motor.wait_is_stopped(0.1)
    print("moved past start loading")
    grabber_controller.start_loading()
    #threading.Thread(target=slow_decrease).start()
    starter = time.time()
    driving_controller.stop_motors()
    #while not grabber_controller.isBlockCaught():
    #    if time.time() - starter > 1.0:
    #        print("[DEBUG] Still waiting for block detection at Gate")
    #        starter = time.time()
    print("[DEBUG] Block caught at gate, slowing down motor drive")
    while not grabber_controller.loading_finished():
        if time.time() - starter > 1.0:
            print("Still loading")
            print(grabber_controller.get_loading_status())
            #threading.Thread(target=grabber_controller.increase_on_velocity_drop, daemon=True).start()
            starter = time.time()

    print(f"[DEBUG] Status of loader is {grabber_controller.get_loading_status()}")
    if grabber_controller.isBlockCaught():
        print("[DEBUG] Block Caught ! Stopping drive")
    print("WAITING")
    grabber_controller.clearBlockCaught()
    time.sleep(4)
    #grabber_controller._end_operation_recording()
    #grabber_controller._end_operation_recording()
    print("[DEBUG] Loading first block complete. Waiting 5s before loading second block")
    if grabber_controller.bloc_count != 1:
        raise RuntimeError("Failure to catch block.First block not registered")
    # stop for now
    driving_controller.move_distance(100)
    #threading.Thread(target=slow_decrease).start()
    starter = time.time()
    left_motor.wait_is_stopped(0.1)
    driving_controller.stop_motors()
    while not grabber_controller.isBlockCaught() and not grabber_controller.loading_finished():
        if time.time() - starter > 1: 
          print("waiting for block to get caught")
          starter = time.time()
    grabber_controller.clearBlockCaught()
    while grabber_controller.is_loading():
        print("[DEBUG] Status of loader is {}".format(grabber_controller.get_loading_status()))
        if  grabber_controller.get_loading_status() == "FAILED":
            raise RuntimeError("Failure to catch block. Should we back up the robot and start over ?")
    print("[DEBUG] Loading second block complete")

    color_sensor_controller.set_arm_at_angle(0)

    time.sleep(5)
    # stop for now
    driving_controller.move_distance(20)
    left_motor.wait_is_stopped(0.1)
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
    driving_controller.move_distance(20)
    left_motor.wait_is_stopped(0.1)
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
