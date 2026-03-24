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

    def begin(self):
        self.pilot.move_forward()
    
    def move_forward(self, max_distance, force = False):
        """Moves the robot forward as long as the colour sensor detects the black line.
        
        Args:
            max_distance (int): The distance to move forward for, in millimetres.
            force (bool): If true, the robot will move forward for the max_distance regardless of the colour sensor readings.
        """


        # Reset encoder values
        self.pilot.reset_encoders()

        # Keep moving forward as long as the black line is detected
        # Keep track of the distance 
        # @TODO Distance does not account for any adjustments (wiggling)
        distance = 0
        last_encoder_value = 0
        moving_forward = False
        starting_direction_right = True

        while distance < max_distance:
            # Moving forward
            if not moving_forward:
                self.pilot.move_forward()
                moving_forward=True

            # print(self.pilot.get_avg_encoder_value())
            # print(self.sensor.read_colour())
            print(self.pilot.get_avg_encoder_value())
            distance += (math.pi/180) * (self.pilot.get_avg_encoder_value() - last_encoder_value) * (self.pilot.WHEEL_DIAMETER_MM/2)
            last_encoder_value = self.pilot.get_avg_encoder_value()

            print(self.sensor.read_colour())
            if not force and self.sensor.read_colour() == "w":
                if moving_forward:
                    moving_forward = False
                    self.pilot.stop_motors()

                results = self.sensor.sweep(range="full", stop_on_match=True, starting_on_right=starting_direction_right)
                starting_direction_right = not starting_direction_right
                print(f"sweeping result {results}")
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


    def turn_logic(self, direction: Literal["left", "right"], heading: Literal["front", "back"] = "front"):
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
        
        print("Moving arm")
        time.sleep(0.1)
        # Wait for sensor arm to be stopped before proceeding
        self.sensor.motor.wait_is_stopped()

        # Moves 30 mm forward to find intersection
        if heading == "front":
            self.pilot.move_forward()
        elif heading == "back":
            self.pilot.move_backward()
        else:
            raise Exception(f"Invalid parameter heading for method SweepLineFollower.turn_logic: {heading}. Valid options are front, back")
        print(f"The colors detected are : {self.sensor.get_rgb()}")

        while (self.sensor.read_colour() == "w"):
            print("reading white")

        print("stopping motors")
        self.pilot.stop_motors()

        print("move straight to align")
        self.pilot.move_distance(consts.dist_move_mm)
        self.pilot.left_motor.wait_is_stopped()
        
        print(f"pivoting {direction}")
        self.pilot.pivot_90(direction)

        # Wait for pivot to complete before proceeding
        time.sleep(0.1)
        getattr(self.pilot, f"{'right' if direction == 'left' else 'left'}_motor").wait_is_stopped()

        # Set colour arm back to the normal position
        self.sensor.set_arm_at_angle(0)
        time.sleep(0.1)
        self.sensor.motor.wait_is_stopped()
        time.sleep(0.1)

    def run(self):
        self.turn_logic("right")
        '''
        distance_mm = 20000
        self.move_forward(distance_mm)
        '''
        return
        while True:
            # Sweep the colour sensor once
            sweep_result = self.sensor.sweep(range="full")
            print("Swept sensor", sweep_result)

            if len(sweep_result) == 0:
                # Could not find the black line. Sweep again.
                print("Did not find black line")
                continue 
            
            if len(sweep_result) == 1:
                # We've found the black line. Figure out if we need to correct
                print("Found black line.", sweep_result[0])
                theta, *_ = sweep_result[0]

                if -self.THETA_TOLERANCE <= theta <= self.THETA_TOLERANCE:
                    # We are close enough to center that we can continue driving forward.
                    print("close to center. theta: ", theta)

                    self.pilot.move_forward()
                else:
                    # Adjust the motors to correct for the deviation
                    print("too far off. theta: ", theta)

                    adjustment = self.K_P * theta - self.K_D * (theta - self.last_theta)
                    print(adjustment, self.pilot.base_power)
                    self.pilot.adjust_direction(adjustment)

                    print("right motor: ", self.pilot.right_motor.get_dps())
                    print("left motor: ", self.pilot.left_motor.get_dps())

                self.last_theta = theta

            elif len(sweep_result) == 2 or len(sweep_result) == 3:
                # We've reached an intersection.
                print("found an intersection, ", len(sweep_result), " lines detected")

                continue

                theta, _ = sweep_result[1]

                # Turn right
                self.pilot.left_motor.set_dps(self.K_R * theta)

                # @TODO: Complete logic
                
                # 1st idea: make the run method sleep until we turned enough (we need to turn at specifc angle and power)
                
                pass
