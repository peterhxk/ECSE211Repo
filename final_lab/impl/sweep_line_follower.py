from controllers.color_sensor_controller import ColourSensorController
from controllers.driving_controller import DrivingController
import math
import time 

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
    
    def move_forward(self, max_distance):
        """Moves the robot forward as long as the colour sensor detects the black line.
        
        Args:
            max_distance (int): The distance to move forward for, in millimetres.
        """

        # Reset encoder values
        self.pilot.reset_encoders()

        # Keep moving forward as long as the black line is detected
        # Keep track of the distance 
        # @TODO Distance does not account for any adjustments (wiggling)
        distance = 0
        last_encoder_value = 0
        moving_forward = False

        while distance < max_distance:
            # Moving forward
            if not moving_forward:
                self.pilot.move_forward()
                moving_forward=True

            # print(self.pilot.get_avg_encoder_value())
            # print(self.sensor.read_colour())
            
            distance += (math.pi/180) * (self.pilot.get_avg_encoder_value() - last_encoder_value) * (self.pilot.WHEEL_DIAMETER_MM/2)
            time.sleep(0.1)
            if self.sensor.read_colour() == "w":
                if moving_forward:
                    moving_forward = False
                    self.pilot.stop_motors()

                results = self.sensor.sweep(range="full", stop_on_match=True)
                print(f"sweeping result {results}")
                angle = results[0][0]

                # Positive angles -> line is detected on the left
                self.pilot.pivot("right" if angle > 0 else "left")
                
                # Rotate for an amount of time based on theta
                # time t as a function of angle theta
                t = abs(self.f_theta(angle, 150))
                print(f"sleeping for {t*10}")
                time.sleep(t*10)

        print(f"advancement of {max_distance} distance completed")
        return
    
    # Helper
    def f_theta(self, angle, dps):

        # Define constants
        dis_wheels_mm = 159
        l_rod_mm = 65
        l_axle_rod_mm = 69
        dia_wheel_mm = 42.6

        # Calculation
        # phi: turning angle for robot wheels
        phi = math.asin(l_rod_mm*math.sin(math.degrees(180-angle))/ \
                        math.sqrt(l_rod_mm**2+l_axle_rod_mm**2+2*l_rod_mm*l_axle_rod_mm* \
                                  math.cos(math.degrees(180-angle))))
        print(f"turning angle for robot {phi}")
        # circumference of wheel
        cir = dia_wheel_mm*math.pi

        # total d needed for wheel to travel
        d = dis_wheels_mm*phi*math.pi/180
        print(f"total_d to travel {d}")


        # distance/s from degrees/s
        d_per_s = dps * cir / 360
        print(f"distance per second {d_per_s}")

        # time t (in sec) for wheel to turn for
        t = d / d_per_s
        
        return t


    def run(self):
        distance_mm = 10000
        self.move_forward(distance_mm)
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
