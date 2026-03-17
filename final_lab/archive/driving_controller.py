import time
import math
from utils.brick import wait_ready_sensors, reset_brick, Motor, TouchSensor, EV3UltrasonicSensor
from concurrent import futures

class DrivingController:
    """Controller for line following with intersection detection."""
    
    # Configuration constants
    WHEEL_RADIUS = 3.0  # cm
    LINE_THICKNESS = 1.5  # cm
    LEFT_TURN_DPS = (-100, 200)  # (left_motor, right_motor)
    RIGHT_TURN_DPS = (200, -100)  # (left_motor, right_motor)
    
    # Timing thresholds
    COLOR_CHANGE_THRESHOLD = 0.1  # seconds to confirm color change
    INTERSECTION_TIME_MULTIPLIER = 1.05  # multiplier for intersection detection
    LOOP_DELAY = 0.05  # seconds

    # Path preset
    path = ["right"]
    
    def __init__(self, left_motor, right_motor, motor_power=10):
        """Initialize the driving controller.
        
        Args:
            left_motor: Left motor controller
            right_motor: Right motor controller
        """
        self.left_motor = left_motor
        self.right_motor = right_motor
        self.motor_power = motor_power
        
        # State tracking
        self.direction = 'l'  # current turning direction ('l' or 'r')
        self.current_colour = None
        self.color_change_timer = None
        self.black_start_time = None
        self.on_black = False
        self.intersection_handled = False  # flag to avoid handling same intersection multiple times
        self.path_position = 0  # current position in path array
        
        # Calculate expected line crossing time
        self.expected_crossing_time = self._calculate_crossing_time()
        self.intersection_threshold = self.expected_crossing_time * self.INTERSECTION_TIME_MULTIPLIER
    
    def _calculate_crossing_time(self):
        """Calculate expected time to cross a line based on wheel speeds.
        
        Returns:
            float: Expected crossing time in seconds
        """
        # Average forward speed component (using average of both wheels)
        avg_left_dps = abs(sum(self.LEFT_TURN_DPS) / 2)
        avg_right_dps = abs(sum(self.RIGHT_TURN_DPS) / 2)
        avg_dps = (avg_left_dps + avg_right_dps) / 2
        
        # Convert DPS to linear velocity (cm/s)
        # velocity = (degrees/second) * (2*pi*radius) / 360
        linear_velocity = (avg_dps * 2 * math.pi * self.WHEEL_RADIUS) / 360
        
        # Time to cross line = line_thickness / velocity
        if linear_velocity > 0:
            crossing_time = self.LINE_THICKNESS / linear_velocity
        else:
            crossing_time = 0.1  # fallback value
        
        return crossing_time
    
    def set_power(self, power):
        """Set power for both motors.
        
        Args:
            power: Power value to set
        """
        self.left_motor.set_power(power)
        self.right_motor.set_power(power)
    
    def go_right(self):
        """Turn right by setting appropriate motor speeds."""
        self.left_motor.set_dps(self.RIGHT_TURN_DPS[0])
        self.right_motor.set_dps(self.RIGHT_TURN_DPS[1])
        self.direction = 'r'
    
    def go_left(self):
        """Turn left by setting appropriate motor speeds."""
        self.left_motor.set_dps(self.LEFT_TURN_DPS[0])
        self.right_motor.set_dps(self.LEFT_TURN_DPS[1])
        self.direction = 'l'

    def move_straight(self, x, dps=10):
        """x: angle in degrees?
        power: value -100 to 100"""
        print(f"Move straight by {x} at {dps} degrees per second ")
        self.left_motor.set_dps(dps)
        self.right_motor.set_dps(dps)
        time.sleep(abs(x/dps)) # CHANGE THIS
        self.left_motor.set_dps(0)
        self.right_motor.set_dps(0)


    def turn_angle(self, x, dps=10):
        """x: angle in degrees
        dps: degrees per second (can range positive to negative)
        positive -> turn _
        negative -> turn _"""
        self.left_motor.set_dps(dps)
        self.right_motor.set_dps(-dps)
        time.sleep(abs(x/dps)) 
        self.left_motor.set_dps(0)
        self.right_motor.set_dps(0)


    def turn_right(self, dps=10):
        self.turn_angle(90, dps)

    def turn_left(self, dps=10):
        self.turn_angle(-90,dps)
    
    def _handle_color_change(self, colour):
        """Handle color change detection with debouncing.
        
        Args:
            colour: Current color reading
            
        Returns:
            bool: True if color has been stable for threshold time
        """
        if colour != self.current_colour:
            # Color changed, reset timer
            self.color_change_timer = time.time()
            self.current_colour = colour
            return False
        else:
            # Color is same, check if enough time has passed
            if self.color_change_timer is not None:
                elapsed = time.time() - self.color_change_timer
                return elapsed >= self.COLOR_CHANGE_THRESHOLD
        return False
    
    def _check_intersection(self):
        """Check if robot is on an intersection based on time on black.
        
        Returns:
            bool: True if intersection detected
        """
        if self.on_black and self.black_start_time is not None:
            time_on_black = time.time() - self.black_start_time
            print(f"time_on_black: {time_on_black}, threshold: {self.intersection_threshold}")
            return time_on_black >= self.intersection_threshold
        return False
    
    def _update_black_timer(self, colour):
        """Update the timer tracking time on black surface.
        
        Args:
            colour: Current color reading
        """
        if colour == "b":
            if not self.on_black:
                # Just entered black
                self.black_start_time = time.time()
                self.on_black = True
        else:
            # Not on black
            self.black_start_time = None
            self.on_black = False
            self.intersection_handled = False  # Reset when leaving black
    
    def get_time_on_black(self):
        """Get the current time spent on black surface.
        
        Returns:
            float: Time in seconds, or 0 if not on black
        """
        if self.on_black and self.black_start_time is not None:
            return time.time() - self.black_start_time
        return 0.0
    
    def is_intersection(self):
        """Check if currently on an intersection.
        
        Returns:
            bool: True if intersection detected
        """
        return self._check_intersection()
    
    def _navigate_intersection(self, time_on_black):
        """Navigate intersection based on current path position.
        
        Args:
            time_on_black: Time spent on black surface in seconds
        """
        if self.path_position >= len(self.path):
            print("Path completed!")
            return
        
        action = self.path[self.path_position].lower()
        print(f"Intersection {self.path_position}: {action} (time_on_black: {time_on_black:.3f}s), current_direction: {self.direction}")
        
        if action == "straight":
            # Go straight by turning opposite direction for time_on_black
            if self.direction == 'l':
                self.go_right()
                print(f"Going straight: turning right for {time_on_black:.3f}s")
            else:
                self.go_left()
                print(f"Going straight: turning left for {time_on_black:.3f}s")
            time.sleep(time_on_black)
        
        elif action == "left":
            if self.direction == "l":
                return
            # Spin left until seeing second black line
            print("Turning left")
            self.go_left()
            
            # Wait to leave current black line (with 0.05s confirmation)
            left_current_black = True
            non_black_start = None
            while left_current_black:
                colour = self.color_sensor.read_colour()
                if colour != "b":
                    if non_black_start is None:
                        non_black_start = time.time()
                    elif time.time() - non_black_start >= 0.05:
                        left_current_black = False
                else:
                    non_black_start = None
                time.sleep(0.01)
            print("past current black")

            # Wait to find next black line (with 0.05s confirmation)
            found_next_black = False
            black_start = None
            while not found_next_black:
                colour = self.color_sensor.read_colour()
                if colour == "b":
                    if black_start is None:
                        black_start = time.time()
                    elif time.time() - black_start >= 0.05:
                        found_next_black = True
                        print("Found next black line after left turn")
                else:
                    black_start = None
                time.sleep(0.01)
            print("past next black")
        
        elif action == "right":
            if self.direction == "r":
                return
            # Spin right until seeing second black line
            print("Turning right")
            self.go_right()
            
            # Wait to leave current black line (with 0.05s confirmation)
            left_current_black = True
            non_black_start = None
            while left_current_black:
                colour = self.color_sensor.read_colour()
                if colour != "b":
                    if non_black_start is None:
                        non_black_start = time.time()
                    elif time.time() - non_black_start >= 0.05:
                        left_current_black = False
                else:
                    non_black_start = None
                time.sleep(0.01)
            print("past current black")
            
            # Wait to find next black line (with 0.05s confirmation)
            found_next_black = False
            black_start = None
            while not found_next_black:
                colour = self.color_sensor.read_colour()
                if colour == "b":
                    if black_start is None:
                        black_start = time.time()
                    elif time.time() - black_start >= 0.05:
                        found_next_black = True
                        print("Found next black line after right turn")
                else:
                    black_start = None
                time.sleep(0.01)
            print("past next black")
        
        elif action == "reverse":
            # Reverse by turning in current direction and at same spot until finding next black
            print(f"Reversing: continuing {self.direction} to find next black")
            if self.direction == "l":
                self.left_motor.set_dps(-150)
                self.right_motor.set_dps(150)
            else:
                self.left_motor.set_dps(150)
                self.right_motor.set_dps(-150)
            # Wait to leave current black line
            left_current_black = True
            while left_current_black:
                colour = self.color_sensor.read_colour()
                if colour != "b":
                    left_current_black = False
                time.sleep(0.01)

            found_next_black = False
            while not found_next_black:
                colour = self.color_sensor.read_colour()
                if colour == "b":
                    found_next_black = True
                    print("Found next black line after right turn")
                time.sleep(0.01)
        
        # Move to next position in path
        self.path_position += 1
    
    def drive(self, color_sensor):
        """Main driving loop for line following with intersection detection.
        
        Args:
            color_sensor: Color sensor controller for reading colors
        """
        self.color_sensor = color_sensor  # Store reference for intersection navigation
        past_black = False
        self.go_left()
        
        while True:
            colour = color_sensor.read_colour()
            
            # Initialize timer on first reading
            if self.color_change_timer is None:
                self.color_change_timer = time.time()
                self.current_colour = colour
                continue
            
            # Update black surface timer
            self._update_black_timer(colour)
            
            # Check for intersection
            if self.is_intersection() and not self.intersection_handled:
                time_on_black = self.get_time_on_black()
                
                # Navigate intersection based on path
                self._navigate_intersection(time_on_black)
                
                # Mark intersection as handled
                self.intersection_handled = True
                past_black = False  # Reset to resume normal line following

            
            # Handle color changes with debouncing
            if self._handle_color_change(colour):
                if colour == "b":
                    past_black = True
                elif past_black and colour == "w":
                    # Switch direction when leaving black line
                    if self.direction == 'l':
                        self.go_right()
                    else:
                        self.go_left()
                    past_black = False
            
            time.sleep(self.LOOP_DELAY)
