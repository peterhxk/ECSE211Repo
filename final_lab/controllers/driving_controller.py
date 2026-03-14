from typing import Literal
from utils.brick import Motor

class DrivingController:
    """Controller to drive the robot."""

    WHEEL_DIAMETER_MM = 42.6

    def __init__(self, left_motor: Motor, right_motor: Motor, base_power: int):
        """
        Initializes the driving controller.

        Params: `right_motor`, `left_motor`: BrickPi `Motor` objects.
        """

        self.right_motor = right_motor
        self.left_motor = left_motor 
        self.base_power = base_power
    
    def move_forward(self):
        """
        Sets the left and right motors to move the robot forward at a rate of `self.base_power`.
        """
        print(f"moving forward with {self.base_power}")

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

    def pivot(self, power: int, direction: Literal["left", "right"]):
        """
        Pivots the robot with one of the wheels as a pivot point.
        
        Args:
            power (int): The power at which to rotate the moving wheel.
            direction (left | right): The direction in which to turn the robot.
        """

        if direction == "left":
            self.right_motor.set_dps(power)
            self.left_motor.set_dps(0)
        elif direction == "right":
            self.left_motor.set_dps(power)
            self.right_motor.set_dps(0)
        else:
            raise Exception(f"Invalid option for parameter direction in DrivingController.pivot: {direction}.\nValid options are: left, right")

    def adjust_direction(self, adjustment: int):
        """
        Adjusts the robots direction by changing the left and right motor powers by a fixed amount `adjustment`.

        - A positive value of `adjustment` will turn the robot to the right.
        - A larger magnitude of `adjustment` will make the robot turn with a smaller turn radius.
        """
        self.right_motor.set_dps(self.base_power + adjustment)
        self.left_motor.set_dps(self.base_power - adjustment)

