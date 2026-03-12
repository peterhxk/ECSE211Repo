from controllers.color_sensor_controller import ColourSensorController
from controllers.driving_controller import DrivingController

class SweepLineFollower:

    THETA_TOLERANCE = 3
    MOTOR_BASE_POWER_DPS = 150
    K_P = 1   # @TODO: Choose appropriate values of K_P and K_D.
    K_D = 0
    LAMBDA = 0
    K_R = 5

    def __init__(self, sensor: ColourSensorController, pilot: DrivingController):
        self.sensor = sensor 
        self.pilot = pilot
        self.last_theta = 0

    def begin(self):
        self.pilot.move_forward()

    def run(self):
        while True:
            # Sweep the colour sensor once
            sweep_result = self.sensor.sweep(range="full")
            print("Swept sensor")

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
                    print("left motor: ", self.pilot.left.get_dps())

                self.last_theta = theta

            elif len(sweep_result) == 3:
                # We've reached an intersection.
                print("found an intersection")

                continue

                theta, _ = sweep_result[1]

                # Turn right
                self.pilot.left_motor.set_dps(self.K_R * theta)

                # @TODO: Complete logic
