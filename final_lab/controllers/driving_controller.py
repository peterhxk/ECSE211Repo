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
        self.left_motor = left_motor 
        self.right_motor = right_motor
        self.base_power = base_power
    
    def move_forward(self, power: int = None):
        """
        Sets the left and right motors to move the robot forward at a rate of `self.base_power`.
        """
        # print(f"moving forward with {self.base_power}")

        print(f"moving forward with {'base power' if power is None else power}")
        self.right_motor.set_dps(self.base_power if power is None else power)
        self.left_motor.set_dps(self.base_power if power is None else power)

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

    def adjust_direction(self, adjustment: int):
        """
        Adjusts the robots direction by changing the left and right motor powers by a fixed amount `adjustment`.

        - A positive value of `adjustment` will turn the robot to the right.
        - A larger magnitude of `adjustment` will make the robot turn with a smaller turn radius.
        """
        self.right_motor.set_dps(self.base_power + adjustment)
        self.left_motor.set_dps(self.base_power - adjustment)

    def set_left_motor(self, power: int):
        """Sets the left motor to a specific power."""
        self.left_motor.set_dps(power)
    
    def set_right_motor(self, power: int):
        """Sets the right motor to a specific power."""
        self.right_motor.set_dps(power)

    def get_base_power(self):
        """Returns the base power of the motors."""
        return self.base_power