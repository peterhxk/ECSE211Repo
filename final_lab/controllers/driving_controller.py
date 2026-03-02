import time
from time import sleep, time
from utils.brick import wait_ready_sensors, reset_brick, Motor, TouchSensor, EV3UltrasonicSensor
from concurrent import futures

class DrivingController():
    def __init__(self, left_motor, right_motor, motor_power=10):
        self.left_motor = left_motor
        self.right_motor = right_motor
        self.motor_power = motor_power

    def set_power(self, power):
        self.left_motor.set_power(power)
        self.right_motor.set_power(power)

    def go_right(self):
        self.left_motor.set_dps(80)
        self.right_motor.set_dps(30)

    def go_left(self):
        self.left_motor.set_dps(30)
        self.right_motor.set_dps(80)

    def move_straight(self, x, dps=10):
        """x: angle in degrees?
        power: value -100 to 100"""
        print(f"Move straight by {x} at {dps} degrees per second ")
        self.left_motor.set_dps(dps)
        self.right_motor.set_dps(dps)
        sleep(abs(x/dps)) # CHANGE THIS
        self.left_motor.set_dps(0)
        self.right_motor.set_dps(0)


    def turn_angle(self, x, dps=10):
        """x: angle in degrees
        dps: degrees per second (can range positive to negative)
        positive -> turn _
        negative -> turn _"""
        self.left_motor.set_dps(dps)
        self.right_motor.set_dps(-dps)
        sleep(abs(x/dps)) 
        self.left_motor.set_dps(0)
        self.right_motor.set_dps(0)


    def turn_right(self, dps=10):
        self.turn_angle(90, dps)

    def turn_left(self, dps=10):
        self.turn_angle(-90,dps)

    
    def drive(self, color_sensor):
        direction = 'l'
        past_black = False
        cur_colour = None
        timer = None
        self.go_left()
        while True:
            colour = color_sensor.read_colour()
            if timer is None:
                timer = time.time()
                cur_colour = colour
            else:
                if colour != cur_colour:
                    timer = time.time()
                    cur_colour = colour
                elif time.time() - timer >= 0.1:
                    if colour == "b":
                        past_black = True
                    elif past_black and colour == "w":
                        if direction == 'l':
                            self.go_right()
                            direction = 'r'
                        else:
                            self.go_left()
                            direction = 'l'
                        past_black = False
            time.sleep(0.05)
