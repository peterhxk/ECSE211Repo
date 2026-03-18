from utils.brick import wait_ready_sensors, reset_brick, Motor, UltrasonicSensor
from controllers.driving_controller import DrivingController
import time

left_motor = Motor("B")
right_motor = Motor("C")
ultrasonic_sensor = UltrasonicSensor(1)

driving_controller = DrivingController(left_motor, right_motor, 20)
print("waiting for sensors to be ready...")
wait_ready_sensors(True)
print("sensors ready!")


try:
    while True:
        driving_controller.set_left_motor(20)
        driving_controller.set_right_motor(-20)
        time.sleep(0.01)
        driving_controller.set_left_motor(0)
        driving_controller.set_right_motor(0)
        time.sleep(0.01)
        driving_controller.set_left_motor(-40)
        driving_controller.set_right_motor(40)
        time.sleep(0.01)
        driving_controller.set_left_motor(0)
        driving_controller.set_right_motor(0)
        time.sleep(0.01)
        driving_controller.set_left_motor(20)
        driving_controller.set_right_motor(-20)
        time.sleep(0.01)
        driving_controller.set_left_motor(0)
        driving_controller.set_right_motor(0)

except KeyboardInterrupt:
    print("Interrupted via keyboard...")
