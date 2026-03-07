import time
import math


class DrivingController:
    """Controller for line following with intersection detection."""
    
    # Configuration constants
    WHEEL_RADIUS = 3.0  # cm
    LINE_THICKNESS = 1.5  # cm
    LEFT_TURN_DPS = (-100, 300)  # (left_motor, right_motor)
    RIGHT_TURN_DPS = (300, -100)  # (left_motor, right_motor)
    
    # Timing thresholds
    COLOR_CHANGE_THRESHOLD = 0.05  # seconds to confirm color change
    INTERSECTION_TIME_MULTIPLIER = 1.5  # multiplier for intersection detection
    LOOP_DELAY = 0.05  # seconds

    # Path preset
    path = ["Straight", "Right", "Right", "Reverse", "Left", "Straight"]
    
    def __init__(self, left_motor, right_motor):
        """Initialize the driving controller.
        
        Args:
            left_motor: Left motor controller
            right_motor: Right motor controller
        """
        self.left_motor = left_motor
        self.right_motor = right_motor
        
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
        
        action = self.path[self.path_position]
        print(f"Intersection {self.path_position}: {action} (time_on_black: {time_on_black:.3f}s)")
        
        if action == "Straight":
            # Go straight by turning opposite direction for time_on_black
            if self.direction == 'l':
                self.go_right()
                print(f"Going straight: turning right for {time_on_black:.3f}s")
            else:
                self.go_left()
                print(f"Going straight: turning left for {time_on_black:.3f}s")
            time.sleep(time_on_black)
        
        elif action == "Left":
            if self.direction == 'l':
                # Already turning left, continue
                print(f"Turning left: continuing left for {time_on_black:.3f}s")
                time.sleep(time_on_black)
            else:
                # Was turning right, need to turn back left for 2x time
                self.go_left()
                turn_time = 2 * time_on_black
                print(f"Turning left: turning back left for {turn_time:.3f}s")
                time.sleep(turn_time)
        
        elif action == "Right":
            if self.direction == 'r':
                # Already turning right, continue
                print(f"Turning right: continuing right for {time_on_black:.3f}s")
                time.sleep(time_on_black)
            else:
                # Was turning left, need to turn back right for 2x time
                self.go_right()
                turn_time = 2 * time_on_black
                print(f"Turning right: turning back right for {turn_time:.3f}s")
                time.sleep(turn_time)
        
        elif action == "Reverse":
            # Reverse by continuing current direction for 1x time_on_black
            print(f"Reversing: continuing {self.direction} for {time_on_black:.3f}s")
            time.sleep(time_on_black)
        
        # Move to next position in path
        self.path_position += 1
    
    def drive(self, color_sensor):
        """Main driving loop for line following with intersection detection.
        
        Args:
            color_sensor: Color sensor controller for reading colors
        """
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
