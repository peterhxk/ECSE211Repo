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

        # Begin moving forward
        self.pilot.move_forward()

        # Keep moving forward as long as the black line is detected
        # Keep track of the distance 
        # @TODO Distance does not account for any adjustments (wiggling)
        distance = 0
        last_encoder_value = 0
        while self.sensor.read_colour() == "b" and distance < max_distance:
            distance += (math.pi/180) * (self.pilot.get_avg_encoder_value() - last_encoder_value) * (self.pilot.WHEEL_DIAMETER_MM/2)
            time.sleep(0.1)

        if distance < max_distance:
            # Stopped due to the black line no longer being detected.
            # Sweep the colour arm to find the line once again

            while True:
                results = self.sensor.sweep(range="full", stop_on_match=True)
                angle = results[0][0]

                # Positive angles -> line is detected on the left
                self.pilot.pivot(20, "left" if angle > 0 else "right")
                
                # Rotate for an amount of time based on theta
                # @TODO calculate time based on phi rather than theta
                time.sleep(angle / 10)

                if -self.THETA_TOLERANCE <= angle <= self.THETA_TOLERANCE:
                    # Robot is straight
                    # @TODO: Advance slightly to ensure that robot actually is straight
                    # If that's the case, return to move_forward and advance the remaining distance
                    pass
                
                # @TODO: Otherwise, repeat the process to determine the next correction to perform


            return

    def run(self):
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
