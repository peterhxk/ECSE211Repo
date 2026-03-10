from utils.brick import EV3ColorSensor, Motor
import time
import json
from enum import Enum
from typing import Literal, List, Tuple, Union, NamedTuple, Optional

def normalize_colours(r, g, b):
    total = r+g+b
    if total == 0:
        return 0,0,0
    return r/total, g/total, b/total

class SweepDirection(Enum):
    RIGHT=1
    LEFT=-1

class ColourSensorController (EV3ColorSensor):

    # Angles to the right of normal are positive
    MAX_SWEEP_ANGLE = {
        "left": {
            SweepDirection.RIGHT: 0,
            SweepDirection.LEFT: -45
        },
        "right": {
            SweepDirection.RIGHT: 45,
            SweepDirection.LEFT: 0
        },
        "full": {
            SweepDirection.RIGHT: 45,
            SweepDirection.LEFT: -45
        }
    }

    # Angle tolerance at which the line is still considered the same one
    THETA_TOLERANCE = 10

    MOTOR_POWER_DPS = 50

    def __init__(self, port, motor: Motor, mode="component", bp=None):
        super().__init__(port)

        self.motor = motor
        self.sweep_dir = None
        self.colours_found = []

        # Reset the value of the motor encoder.
        self.motor.reset_encoder()

    def _is_at_boundary(self, range: Literal["left", "right", "full"]) -> bool:
        """
        Given the current `sweep_dir`, this helper function checks if the current motor encoder position
        of the colour sensor is beyond the pre-defined threshold. If `True`, then we need to switch the direction
        """
        return self.sweep_dir.value * self.motor.get_position() >= self.sweep_dir.value * self.MAX_SWEEP_ANGLE[range][self.sweep_dir]

    def sweep(self, colour="b", range: Literal["left", "right", "full"] = "left", stop_on_match: bool = False) -> List[Tuple[int, str, int]]:
        """Sweeps the colour sensor arm and checks the colour.

        Args:
            colour (str): The colour to search for (e.g., "b" for black).
            range (str): "left", "right" or "full". Determines the sweep arc.
            stop_on_match (bool): If True, exits early on the first detection.

        Returns:
            List[Tuple]: A list of detections where each 
            tuple is (angle, colour, timestamp).
        """

        duration = 0

        if self.sweep_dir == None:
            # Not currently sweeping. Start sweeping to the right.
            self.sweep_dir = SweepDirection.RIGHT

        while True:
            # Check if a black line is found
            if self.read_colour() == colour:
                # Check to make sure we are not "finding" the same black line as previously
                theta = self.motor.get_position()
                
                for line_angle, _ in self.colours_found:
                    if abs(theta - line_angle) <= self.THETA_TOLERANCE:
                        break
                else:
                    # Add theta to the list of matches once the colour is found
                    self.colours_found.append((self.motor.get_position(), colour, duration))

                    if stop_on_match:
                        results = self.colours_found
                        self.colours_found = []
                        return results
            
            # Check if we are beyond the maximum sweep angle
            if self._is_at_boundary(range):
                # Reverse the direction
                self.sweep_dir = SweepDirection.LEFT if self.sweep_dir == SweepDirection.RIGHT else SweepDirection.RIGHT
                
                # At this point, we've completed one sweep, so we must return the colours found.
                results = self.colours_found
                self.colours_found = []
                return results
            
            # Check if the motor is operating in the current direction
            if (self.motor.get_speed() * self.sweep_dir.value <= 0):
                # Reverse the direction of the motor
                self.motor.set_dps(self.MOTOR_POWER_DPS * self.sweep_dir.value)

            # Otherwise, wait a small bit more before sweeping and checking again
            time.sleep(0.1)
            duration += 0.1

    def read_colour(self):
        cur_rgb = self.get_rgb()
        cur_r = cur_rgb[0]
        cur_g = cur_rgb[1]
        cur_b = cur_rgb[2]
        # nor_r, nor_g, nor_b = normalize_colours(cur_r,cur_g,cur_b)
        # print((nor_r, nor_g, nor_b))
        # print(cur_rgb)
        with open("../data/colours.json", "r") as f:
            colours = json.load(f)
        colour_to_distance = {}
        for colour, rgb in colours.items():
            r = int(rgb[0])
            g = int(rgb[1])
            b = int(rgb[2])

            distance = ((cur_r - r)**2+(cur_g - g)**2+(cur_b - b)**2)**(1/2)
            colour_to_distance[colour] = distance
        
        shortest_d = None
        closest_colour = None
        for colour, d in colour_to_distance.items():
            if not closest_colour or d < shortest_d:
                shortest_d = d
                closest_colour = colour
        return closest_colour
    
    def save_colour_data(file):
        with open(file, mode="w", encoding="utf-8") as outfile:
            count = 0
            while count < 50:
                outfile.write(f"{ColourSensorController.get_rgb()}\n")
                time.sleep(0.1)

