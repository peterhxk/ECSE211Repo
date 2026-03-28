import sys
sys.path.append("../")

import time
import traceback

from utils.brick import wait_ready_sensors, reset_brick, Motor
from controllers.color_sensor_controller import ColourSensorController
from controllers.driving_controller import DrivingController
from impl.sweep_line_follower import SweepLineFollower
from controllers.grabber_controller import GrabberController
from data.ports_brick import Ports


# ----------------------------
# Hardware setup
# ----------------------------
left_motor = Motor(Ports.LEFT_MOTOR.value)
right_motor = Motor(Ports.RIGHT_MOTOR.value)
color_sensor_motor = Motor(Ports.COLOR_MOTOR.value)

driving_controller = DrivingController(left_motor, right_motor, 100)
color_sensor_controller = ColourSensorController(
    Ports.COLOR_SENSOR.value,
    color_sensor_motor,
)
grabber_controller = GrabberController(
    Ports.GRABBER_MOTOR.value,
    Ports.GYRO_SENSOR.value,
    False,
    False,
)
line_follower = SweepLineFollower(color_sensor_controller, driving_controller)


# ----------------------------
# Helpers
# ----------------------------
def wait_for_loading(expected_bloc_count, timeout=15.0, label="block"):
    start = time.time()
    last_log = start

    while True:
        status = grabber_controller.get_loading_status()

        if status == "FAILED":
            raise RuntimeError(f"Failed while loading {label}")

        if grabber_controller.loading_finished():
            break

        now = time.time()
        if now - last_log > 1.0:
            print(f"[DEBUG] Waiting for loading of {label}... status={status}")
            last_log = now

        if now - start > timeout:
            raise TimeoutError(f"Timeout while loading {label}")

        time.sleep(0.05)

    final_status = grabber_controller.get_loading_status()
    print(f"[DEBUG] Loading finished for {label} with status={final_status}")

    if grabber_controller.bloc_count != expected_bloc_count:
        raise RuntimeError(
            f"Unexpected bloc_count after loading {label}: "
            f"expected {expected_bloc_count}, got {grabber_controller.bloc_count}"
        )


def wait_for_unloading(expected_bloc_count, timeout=15.0, label="block"):
    start = time.time()
    last_log = start

    while True:
        status = grabber_controller.get_unloading_status()

        if status == "FAILED":
            raise RuntimeError(f"Failed while unloading {label}")

        if status != "UNLOADING":
            break

        now = time.time()
        if now - last_log > 1.0:
            print(f"[DEBUG] Waiting for unloading of {label}... status={status}")
            last_log = now

        if now - start > timeout:
            raise TimeoutError(f"Timeout while unloading {label}")

        time.sleep(0.05)

    final_status = grabber_controller.get_unloading_status()
    print(f"[DEBUG] Unloading finished for {label} with status={final_status}")

    if grabber_controller.bloc_count != expected_bloc_count:
        raise RuntimeError(
            f"Unexpected bloc_count after unloading {label}: "
            f"expected {expected_bloc_count}, got {grabber_controller.bloc_count}"
        )

def move_and_stop(distance_mm):
    driving_controller.move_distance(distance_mm)
    left_motor.wait_is_stopped(0.1)
    driving_controller.stop_motors()


def load_one_block(move_distance_mm, expected_bloc_count, label):
    print(f"[DEBUG] Moving to {label}")
    move_and_stop(move_distance_mm)

    print(f"[DEBUG] Starting loading for {label}")
    grabber_controller.start_loading()
    wait_for_loading(expected_bloc_count=expected_bloc_count, label=label)

    print(f"[DEBUG] Successfully loaded {label}")


def unload_one_block(move_distance_mm, expected_bloc_count, label):
    print(f"[DEBUG] Moving to unload position for {label}")
    move_and_stop(move_distance_mm)

    print(f"[DEBUG] Starting unloading for {label}")
    grabber_controller.start_unloading()
    wait_for_unloading(expected_bloc_count=expected_bloc_count, label=label)

    print(f"[DEBUG] Successfully unloaded {label}")


# ----------------------------
# Main sequence
# ----------------------------
try:
    wait_ready_sensors(True)
    grabber_controller.initialize_sensors()

    color_sensor_controller.set_arm_at_angle(0)
    time.sleep(1)
    color_sensor_controller.set_arm_at_angle(90)
    driving_controller.set_speed(100)

    # Load block 1
    load_one_block(
        move_distance_mm=135,
        expected_bloc_count=1,
        label="block 1",
    )

    time.sleep(2)

    # Load block 2
    load_one_block(
        move_distance_mm=83,
        expected_bloc_count=2,
        label="block 2",
    )

    # Prepare for unloading
    color_sensor_controller.set_arm_at_angle(95)
    time.sleep(1)

    # Unload first carried block
    unload_one_block(
        move_distance_mm=0,
        expected_bloc_count=1,
        label="first unload",
    )

    time.sleep(2)

    # Unload second carried block
    unload_one_block(
        move_distance_mm=20,
        expected_bloc_count=0,
        label="second unload",
    )

    print("[DEBUG] Full sequence complete: loaded 2 blocks, unloaded 2 blocks")

except KeyboardInterrupt:
    color_sensor_controller.set_arm_at_angle(0)
    print("Interrupted via keyboard...")

except Exception as err:
    print(err)
    traceback.print_exc()

finally:
    grabber_controller.stop_gyro_monitor() # stop gyro before reseting brickpi
    reset_brick()
    exit()