from utils.brick import EV3ColorSensor, Motor
import time
import json
from enum import Enum
from typing import Literal, List, Tuple, Union, NamedTuple, Optional
import math
import data.measurements as consts
from collections import deque


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
            SweepDirection.LEFT: -60
        },
        "right": {
            SweepDirection.RIGHT: 60,
            SweepDirection.LEFT: 0
        },
        "full": {
            SweepDirection.RIGHT: 60,
            SweepDirection.LEFT: -60
        }
    }

    # Angle tolerance at which the line is still considered the same one
    THETA_TOLERANCE = 10

    MOTOR_POWER_DPS = 250

    def __init__(self, port, motor: Motor, mode="component", bp=None):
        super().__init__(port)

        self.motor = motor
        self.sweep_dir = None
        self.colours_found = []
        self._bw_history = deque(maxlen=5)

        # Reset the value of the motor encoder.
        self.motor.reset_encoder()
        # Sets maximum speed of motor.
        motor.set_limits(0, self.MOTOR_POWER_DPS)

    def _is_at_boundary(self, range: Literal["left", "right", "full"]) -> bool:
        """
        Given the current `sweep_dir`, this helper function checks if the current motor encoder position
        of the colour sensor is beyond the pre-defined threshold. If `True`, then we need to switch the direction
        """
        return self.sweep_dir.value * self.motor.get_position() >= self.sweep_dir.value * self.MAX_SWEEP_ANGLE[range][self.sweep_dir]

    def set_arm_at_angle(self, angle: float):
        """
        ISSUE: currently the arm is moving too fast
        Sets the arm at {angle} degrees off the origin (straight ahead) 
        
        angle > 0 -> right?
        angle < 0 -> left?
        """
        self.motor.set_limits(0, self.MOTOR_POWER_DPS)
        self.motor.set_position(angle)
        self.motor.wait_is_stopped(0.01)

    def read_current_angle(self):
        pass


    def sweep(self, colours=[], range: Literal["left", "right", "full"] = "left", stop_on_match: bool = False, starting_on_right: bool = True, black_white = False) -> List[Tuple[int, str, int]]:
        """Sweeps the colour sensor arm and checks the colour.

        Args:
            colours (List[str]): A list of the colours to search for (e.g., "b" for black, "w" for white, and "colour" for all other colours).
            range (str): "left", "right" or "full". Determines the sweep arc.
            stop_on_match (bool): If True, exits early on the first detection.
            starting_on_right (bool): If True, colour arm starts sweeping towards the right
            black_white (bool): If True, the reading will be grayscale (only black or white). If False, reads all other colours other than black/white.

        Returns:
            List[Tuple]: A list of detections where each 
            tuple is (angle, colour, timestamp).
        """

        duration = 0

        #initial colour sensor position (should be straight)
        init_position = 0 #self.motor.get_position()
        print("Setting arm to initial position...")
        self.set_arm_at_angle(init_position)
        print("Setting arm to initial position completed...")

        
        print("------------")
        print(init_position)
        if starting_on_right:
            self.sweep_dir = SweepDirection.RIGHT
        else:
            self.sweep_dir = SweepDirection.LEFT

        prev_reversed = False
        consecutive_count = 0
        consecutive_positions = []

        while True:
            # Check if the motor is operating in the current direction
            if (self.motor.get_speed() * self.sweep_dir.value <= 0):
                # Reverse the direction of the motor
                self.motor.set_dps(self.MOTOR_POWER_DPS * self.sweep_dir.value)

            if black_white:
                detected_colour = self.read_black_white()
            else:   
                detected_colour = self.read_colour()
            # Check if a black line is found
            if detected_colour in colours:
                consecutive_count += 1
                consecutive_positions.append(self.motor.get_position())
                if consecutive_count >= 5:
                    avg_pos = sum(consecutive_positions) / len(consecutive_positions)
                    self.colours_found.append((avg_pos, detected_colour, duration))
                    print(self.colours_found)
                    consecutive_count = 0
                    consecutive_positions = []
                    if stop_on_match:
                        results = self.colours_found
                        self.colours_found = []
                        self.motor.set_dps(0)
                        self.set_arm_at_angle(init_position)
                        return results
            else:
                consecutive_count = 0
                consecutive_positions = []
            
            # Check if we are beyond the maximum sweep angle
            if self._is_at_boundary(range):
                # Reverse the direction
                self.sweep_dir = SweepDirection.LEFT if self.sweep_dir == SweepDirection.RIGHT else SweepDirection.RIGHT

                if prev_reversed:
                    # At this point, we've completed one sweep, so we must return the colours found.
                    results = self.colours_found
                    self.colours_found = []
                    self.motor.set_dps(0)
                    self.set_arm_at_angle(init_position)
                    print(f"force return angle result")
                    return results
                prev_reversed = True
    
        
            

            # Otherwise, wait a small bit more before sweeping and checking again
            time.sleep(0.01)
            duration += 0.1


    def read_black_white(self):
        """
        Uses ambient light to classify surface as black or white.
        Returns "b" or "w" only if the last 5 readings all agree, None otherwise.
        """
        ambient = self.read_ambient()
        self.read_colour()
        print(ambient)
        if ambient <= 1:
            reading = "b"
        else:
            reading = "w"

        return reading

    def ingest_colours(self, colour_name: str):
        """
        Reads the current sensor RGB, normalizes it, and updates the running
        average for colour_name in colours.json.
        Call this while the sensor is positioned over the target colour to calibrate.
        """
        with open("data/colours.json", "r") as f:
            colours = json.load(f)

        raw = self.get_rgb()
        r_n, g_n, b_n = normalize_colours(raw[0], raw[1], raw[2])

        if colour_name in colours:
            old = colours[colour_name]
            colours[colour_name] = [
                (old[0] + r_n) / 2,
                (old[1] + g_n) / 2,
                (old[2] + b_n) / 2,
            ]
        else:
            colours[colour_name] = [r_n, g_n, b_n]

        with open("data/colours.json", "w") as f:
            json.dump(colours, f, indent=2)

        print(f"Updated '{colour_name}': {colours[colour_name]}")

    def read_colour(self):
        '''
        Reads the current colour of the colour sensor (not black and white).
        '''
        cur_rgb = self.get_rgb()
        cur_r, cur_g, cur_b = normalize_colours(cur_rgb[0], cur_rgb[1], cur_rgb[2])

        with open("data/colours.json", "r") as f:
            colours = json.load(f)

        colour_to_distance = {}
        for colour, rgb in colours.items():
            r, g, b = float(rgb[0]), float(rgb[1]), float(rgb[2])
            distance = ((cur_r - r)**2 + (cur_g - g)**2 + (cur_b - b)**2) ** 0.5
            colour_to_distance[colour] = distance

        return min(colour_to_distance, key=colour_to_distance.get)
    
    def read_colour_continuous(self, time_interval=0.1, amount=1000, rgb=False):
        """
        Reads the color every time interval, until amount. 
        rbg flag returns rbg value instead of color.
        """
        if (rgb == False):
            for _ in range(amount):
                print(self.read_colour())
                time.sleep(time_interval)
        else:
            for _ in range(amount):
                print(self.get_rgb())
                time.sleep(time_interval)



    def read_colour_pivot(self):
        """
        This function is no longer useful due to updated hardware. 

        Reads colour, but intended to be used when sensor is tilted at 81 degrees (angle to pivot on intersection).
        Uses pivot_colours.json instead
        """
        cur_rgb = self.get_rgb()
        cur_r = cur_rgb[0]
        cur_g = cur_rgb[1]
        cur_b = cur_rgb[2]
        # Uncomment for rgb data adjustment
        # print(cur_rgb)
        with open("data/pivot_colours.json", "r") as f:
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

    def read_colour_print(self):
        print(self.get_rgb())

    def read_ambient(self):
        return self.get_ambient()
    
    def read_rgb(self):
        return self.get_rgb()
