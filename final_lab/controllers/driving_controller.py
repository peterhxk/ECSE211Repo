from utils.brick import Motor

class DrivingController:
    """Controller to drive the robot."""

    def __init__(self, right_motor: Motor, left_motor: Motor, base_power: int):
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
        self.right_motor.set_dps(self.base_power)
        self.left_motor.set_dps(self.base_power)

    def adjust_direction(self, adjustment: int):
        """
        Adjusts the robots direction by changing the left and right motor powers by a fixed amount `adjustment`.

        - A positive value of `adjustment` will turn the robot to the right.
        - A larger magnitude of `adjustment` will make the robot turn with a smaller turn radius.
        """
        self.right_motor.set_dps(self.base_power + adjustment)
        self.left_motor.set_dps(self.base_power - adjustment)