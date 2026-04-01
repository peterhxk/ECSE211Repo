from controllers.color_sensor_controller import ColourSensorController
from controllers.driving_controller import DrivingController
import math
import time 
from typing import Literal
import data.measurements as consts

class SweepLineFollower:

    THETA_TOLERANCE = 3
    MOTOR_BASE_POWER_DPS = 150
    K_P = 1   # @TODO: Choose appropriate values of K_P and K_D.
    K_D = 0.5
    LAMBDA = 0
    K_R = 5

    def __init__(self, sensor: ColourSensorController, pilot: DrivingController):
        self.sensor = sensor 
        self.pilot = pilot
        self.last_theta = 0
    
    def move_forward(self, max_distance, stop_on_colour = None):
        """Moves the robot forward as long as the colour sensor detects the black line.
        The robot will stop moving forward if max_distance is reached, or if the colour stop_on_colour is detected.
        
        Args:
            max_distance (int): The distance to move forward for, in millimetres.
            stop_on_colour (str | None): The colour that, upon detection, will halt the motors.
        """

        if stop_on_colour in ['b', 'w']:
            print("\033[30;103m" + "WARNING: stop_on_colour is set to " + stop_on_colour + ". This may cause unexpected line following behaviour.")

        # Reset encoder values
        self.pilot.reset_encoders()

        # Keep moving forward as long as the black line is detected
        # Keep track of the distance
        # @TODO Distance does not account for any adjustments (wiggling)
        distance = 0
        last_encoder_value = 0
        moving_forward = False
        starting_direction_right = True

        while distance < max_distance and self.sensor.read_colour() != stop_on_colour:
            # Moving forward
            if not moving_forward:
                self.pilot.move_forward()
                moving_forward=True

            distance += (math.pi/180) * (self.pilot.get_avg_encoder_value() - last_encoder_value) * (self.pilot.WHEEL_DIAMETER_MM/2)
            last_encoder_value = self.pilot.get_avg_encoder_value()

            # If the colour sensor has detected white, apply corrective feedback
            if self.sensor.read_black_white() == "w":
                if moving_forward:
                    moving_forward = False
                    self.pilot.stop_motors()

                results = self.sensor.sweep(colours=["b"], range="full", stop_on_match=True, starting_on_right=starting_direction_right, black_white=True)
                starting_direction_right = not starting_direction_right
                print(f"sweeping result {results}")
                if not results:
                    results = self.sensor.sweep(colours=["b"], range="full", stop_on_match=True, starting_on_right=starting_direction_right, black_white=True)
                    starting_direction_right = not starting_direction_right
                    print(f"sweeping result 2 {results}")
                angle = results[0][0]

                # Positive angles -> line is detected on the left
                self.pilot.pivot("right" if angle > 0 else "left")
                
                # Rotate for an amount of time based on theta
                # time t as a function of angle theta
                t = abs(self.f_theta(angle, 150))
                print(f"sleeping for {t}")
                time.sleep(t/2)
                #self.move_forward(50, force=True)
                last_encoder_value = 0 
                self.pilot.reset_encoders()
            time.sleep(0.1)
        print(f"advancement of {max_distance} distance completed")
        return
    
    # Helper
    def f_theta(self, angle, dps):

        # Calculation
        # phi: turning angle for robot wheels

        x = 180-abs(angle)
        c = math.sqrt(consts.l_rod_mm**2 + consts.l_axle_rod_mm**2 + 2 * consts.l_rod_mm * consts.l_axle_rod_mm * \
                            abs(math.cos(math.radians(x))))
        phi = math.degrees(math.asin(consts.l_rod_mm*math.sin(math.radians(x))/c))
        print(f"turning angle for robot {phi}")
        
        # For one wheel stationary rotation

        # circumference of wheel
        cir = consts.dia_wheel_mm*math.pi
        # total d needed for wheel to travel
        d = consts.dis_wheels_mm * phi * math.pi/180
        print(f"total_d to travel {d}")


        # distance/s from degrees/s
        d_per_s = dps * cir / 360
        print(f"distance per second {d_per_s}")

        # time t (in sec) for wheel to turn for
        t = d / d_per_s

        ## For center in-place rotation (works)
        
        # total_ang_for_wheel = phi * dis_wheels_mm / dia_wheel_mm

        # t = total_ang_for_wheel/dps

        
        return t


    def turn_logic(self, direction: Literal["left", "right"], heading: Literal["front", "back"] = "front", additional_dist=0, arm_avoid_block = False):
        """
        Detects the intersecting line, and performs robot turning logic to be aligned with the next line.

        Args:
            direction ("left" | "right"): The direction in which to turn.
            heading ("front" | "back"): Whether the robot needs to advance ("front") or back up ("back") to find the intersecting line.
        """
        self.pilot.stop_motors()

        if direction == "right":
            self.sensor.set_arm_at_angle(consts.turn_angle)
        elif direction == "left":
            self.sensor.set_arm_at_angle(-consts.turn_angle)
        else:
            raise Exception("Wrong direction input")
        

        # Moves 30 mm forward to find intersection
        if heading == "front":
            self.pilot.move_forward()
        elif heading == "back":
            self.pilot.move_backward()
        else:
            raise Exception(f"Invalid parameter heading for method SweepLineFollower.turn_logic: {heading}. Valid options are front, back")
        print(f"[DEBUG] turn_logic: The colors detected are : {self.sensor.read_black_white()}")

        consecutive_black = 0
        while consecutive_black < 3:
            reading = self.sensor.read_black_white()
            print(reading)
            if reading == "b":
                consecutive_black += 1
            else:
                consecutive_black = 0
            time.sleep(0.01)

        print("[DEBUG] turn_logic: stopping motors")
        self.pilot.stop_motors()

        print("[DEBUG] turn_logic: move straight to align")
        self.pilot.move_distance(consts.dist_move_mm+additional_dist)
        self.pilot.left_motor.wait_is_stopped()
        
        if arm_avoid_block:
            self.sensor.set_arm_at_angle(-110)

        print(f"[DEBUG] turn_logic: pivoting {direction}")
        self.pilot.pivot_90(direction)
        print("[DEBUG] turn_logic: done pivoting.")

        # Wait for pivot to complete before proceeding
        time.sleep(0.1)
        getattr(self.pilot, f"{'right' if direction == 'left' else 'left'}_motor").wait_is_stopped()

        # Set colour arm back to the normal position
        if not arm_avoid_block:
            self.sensor.set_arm_at_angle(0)
        time.sleep(0.1)
        self.sensor.motor.wait_is_stopped()
        time.sleep(0.1)

    def reset_arm(self):
        self.sensor.set_arm_at_angle(0)

    
