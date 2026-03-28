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
        self._bw_history = deque(maxlen=8)
        self.BW_DROP_THRESHOLD = 1.5   # drop from baseline → black
        self.BW_RISE_THRESHOLD = 1.5   # rise from baseline → white

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


    def sweep_start(self, colours: List[str] = [], range: Literal["left", "right", "full"] = "full", starting_on_right: bool = True, black_white: bool = False):
        """Begin a non-blocking sweep. Call sweep_step() each loop iteration."""
        self._sweep_colours = colours
        self._sweep_range = range
        self._sweep_black_white = black_white
        self._sweep_consecutive_count = 0
        self._sweep_consecutive_positions = []
        self._sweep_prev_reversed = False
        self._sweep_active = True
        self.colours_found = []

        if starting_on_right:
            self.sweep_dir = SweepDirection.RIGHT
        else:
            self.sweep_dir = SweepDirection.LEFT

        self.motor.set_dps(self.MOTOR_POWER_DPS * self.sweep_dir.value)

    def sweep_step(self) -> Optional[List[Tuple[int, str, int]]]:
        """One iteration of the sweep. Returns None if still sweeping,
        or a list of detections if a match is found or the sweep completes."""
        if not getattr(self, '_sweep_active', False):
            return None

        if self.motor.get_speed() * self.sweep_dir.value <= 0:
            self.motor.set_dps(self.MOTOR_POWER_DPS * self.sweep_dir.value)

        if self._sweep_black_white:
            detected_colour = self.read_black_white()
        else:
            detected_colour = self.read_colour()

        if detected_colour in self._sweep_colours:
            self._sweep_consecutive_count += 1
            self._sweep_consecutive_positions.append(self.motor.get_position())
            if self._sweep_consecutive_count >= 5:
                avg_pos = sum(self._sweep_consecutive_positions) / len(self._sweep_consecutive_positions)
                self.colours_found.append((avg_pos, detected_colour, 0))
                print(self.colours_found)
                self._sweep_consecutive_count = 0
                self._sweep_consecutive_positions = []
                self._sweep_active = False
                results = self.colours_found
                self.colours_found = []
                self.motor.set_dps(0)
                self.set_arm_at_angle(0)
                return results
        else:
            self._sweep_consecutive_count = 0
            self._sweep_consecutive_positions = []

        if self._is_at_boundary(self._sweep_range):
            self.sweep_dir = SweepDirection.LEFT if self.sweep_dir == SweepDirection.RIGHT else SweepDirection.RIGHT
            if self._sweep_prev_reversed:
                self._sweep_active = False
                results = self.colours_found
                self.colours_found = []
                self.motor.set_dps(0)
                self.set_arm_at_angle(0)
                print("force return angle result")
                return results
            self._sweep_prev_reversed = True

        return None

    def sweep_stop(self):
        """Cancel an in-progress non-blocking sweep."""
        self._sweep_active = False
        self.motor.set_dps(0)

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
        Reads the current raw RGB, accumulates it into a rolling average,
        then classifies as "b" or "w" by Euclidean distance to calibrated raw RGB
        values in black_white.json. Raw values preserve brightness, which is the
        key distinguishing feature between black and white.
        Returns None if the file is missing or lacks both entries.
        """
        raw = self.get_rgb()

        try:
            with open("data/black_white.json", "r") as f:
                bw = json.load(f)
        except (FileNotFoundError, json.JSONDecodeError):
            return None

        if "b" not in bw or "w" not in bw:
            return None

        def euclid(a, b):
            return math.sqrt(sum((x - y) ** 2 for x, y in zip(a, b)))

        dist_b = euclid(raw, bw["b"])
        dist_w = euclid(raw, bw["w"])
        return "b" if dist_b < dist_w else "w"

    def ingest_bw(self, label: str):
        """
        Reads the current raw RGB and updates the running average for
        label ("b" or "w") in data/black_white.json.
        Raw values preserve brightness — the key signal for black vs white.
        Call this while the sensor is positioned over the target surface to calibrate.
        """
        if label not in ("b", "w"):
            raise ValueError(f"label must be 'b' or 'w', got '{label}'")

        try:
            with open("data/black_white.json", "r") as f:
                bw = json.load(f)
        except (FileNotFoundError, json.JSONDecodeError):
            bw = {}

        raw = self.get_rgb()
        r, g, b = float(raw[0]), float(raw[1]), float(raw[2])

        old = bw.get(label)
        if isinstance(old, list) and len(old) == 3:
            bw[label] = [(old[0] + r) / 2, (old[1] + g) / 2, (old[2] + b) / 2]
        else:
            bw[label] = [r, g, b]

        with open("data/black_white.json", "w") as f:
            json.dump(bw, f, indent=2)

        print(f"Updated '{label}': {bw[label]}  magnitude={math.sqrt(r**2+g**2+b**2):.1f}")

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