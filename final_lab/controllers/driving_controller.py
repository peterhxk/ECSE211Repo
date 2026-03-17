from typing import Literal
from utils.brick import Motor
import time
class DrivingController:
    """Controller to drive the robot."""

    WHEEL_DIAMETER_MM = 42.6
    WHEEL_DISTANCE_MM = 159 
    ORIENTTODEG = WHEEL_DISTANCE_MM/WHEEL_DIAMETER_MM
    ANGLE_TURN_90 = ORIENTTODEG*90 # angle for wheel to turn to turn right angle

    def __init__(self, left_motor: Motor, right_motor: Motor, base_power: int):
        """
        Initializes the driving controller.

        Params: `right_motor`, `left_motor`: BrickPi `Motor` objects.
        """
        self.left_motor = left_motor 
        self.right_motor = right_motor
        self.base_power = base_power
    
    def move_forward(self):
        """
        Sets the left and right motors to move the robot forward at a rate of `self.base_power`.
        """
        # print(f"moving forward with {self.base_power}")

        self.right_motor.set_dps(self.base_power)
        self.left_motor.set_dps(self.base_power)

    def stop_motors(self):
        """Stops the motors."""
        self.right_motor.set_dps(0)
        self.left_motor.set_dps(0)
    
    def reset_encoders(self):
        """Resets the motor encoder values of both motors."""
        self.left_motor.reset_encoder()
        self.right_motor.reset_encoder()

    def get_avg_encoder_value(self):
        """Reads the average encoder value of the left and right motors."""
        return (self.left_motor.get_encoder() + self.right_motor.get_encoder()) / 2.0

    def pivot(self, direction: Literal["left", "right"]):
        """
        Pivots the robot with one of the wheels as a pivot point.
        
        Args:
            power (int): The power at which to rotate the moving wheel.
            direction (left | right): The direction in which to turn the robot.
        """
        print(f"pivoting {direction}")

        if direction == "left":
            self.right_motor.set_dps(self.base_power)
            self.left_motor.set_dps(0)
        elif direction == "right":
            self.left_motor.set_dps(self.base_power)
            self.right_motor.set_dps(0)
        
        # try rotate in place (works)
        # if direction == "left":
        #     self.right_motor.set_dps(self.base_power/2)
        #     self.left_motor.set_dps(-self.base_power/2)
        # elif direction == "right":
        #     self.left_motor.set_dps(self.base_power/2)
        #     self.right_motor.set_dps(-self.base_power/2)
        else:
            raise Exception(f"Invalid option for parameter direction in DrivingController.pivot: {direction}.\nValid options are: left, right")

    def pivot_90(self, direction: Literal["left", "right"]):
        """
        Pivots the robot with one of the wheels as a pivot point (exactly 90 degrees).
        
        Args:
            power (int): The power at which to rotate the moving wheel.
            direction (left | right): The direction in which to turn the robot.
        """
        print(f"pivoting {direction}")

        if direction == "left":
            self.right_motor.set_limits(0, self.base_power)
            self.right_motor.set_position_relative(90*self.ORIENTTODEG)
        elif direction == "right":
            self.left_motor.set_limits(0, self.base_power)
            self.left_motor.set_position_relative(90*self.ORIENTTODEG)

    def adjust_direction(self, adjustment: int):
        """
        Adjusts the robots direction by changing the left and right motor powers by a fixed amount `adjustment`.

        - A positive value of `adjustment` will turn the robot to the right.
        - A larger magnitude of `adjustment` will make the robot turn with a smaller turn radius.
        """
        self.right_motor.set_dps(self.base_power + adjustment)
        self.left_motor.set_dps(self.base_power - adjustment)

    def turn_in_place(self, angle: float):
        """
        Based on class notes
        angle: in degrees
        dps: degrees per second (can range positive to negative)
        positive -> turn left?
        negative -> turn right?""" 
        
        
        if (angle > 0):
            self.right_motor.set_dps(-self.base_power)
            self.left_motor.set_dps(self.base_power)
        else:
            self.right_motor.set_dps(self.base_power)
            self.left_motor.set_dps(-self.base_power)
        new_angle = abs(angle)
        time.sleep(self.base_power/new_angle)
        self.right_motor.set_dps(0)
        self.left_motor.set_dps(0)
        time.sleep(2)
        
        print("start of other 2nd test")
        self.left_motor.set_limits(0, self.base_power)
        self.right_motor.set_limits(0, self.base_power)
        self.left_motor.set_position_relative(angle*self.ORIENTTODEG)
        self.right_motor.set_position_relative(-angle*self.ORIENTTODEG)
        

        
    def turn_90(self, direction: Literal["left", "right"]):
        """
        Turns in place to the right or left"""
        if direction == "left":
            self.turn_in_place(90)
        else:
            self.turn_in_place(-90)
        