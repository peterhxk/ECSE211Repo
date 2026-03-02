from time import sleep, time
from utils.brick import wait_ready_sensors, reset_brick, Motor, TouchSensor, EV3UltrasonicSensor
from concurrent import futures
from controllers import driving_controller
print("done importing...")

DC = driving_controller.DrivingController(Motor("B"), Motor("C"))

if __name__ == "__main__":
    while True:
        DC.go_left()
        DC.go_right()
