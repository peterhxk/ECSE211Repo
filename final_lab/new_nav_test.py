from time import sleep, time
from utils.brick import wait_ready_sensors, reset_brick, Motor, TouchSensor, EV3UltrasonicSensor
from concurrent import futures
from controllers import driving_controller
print("done importing...")

DC = driving_controller.DrivingController(Motor("B"), Motor("C"))

if __name__ == "__main__":
    while True:
        dir = input("T (turn) or S (straight)?")
        x = float(input("x: "))
        power = (input("power: "))
        if dir == "T":
            DC.turn_angle(x,power)
        elif dir == "S":
            DC.move_straight(x, power)
        else:
            print("no work")