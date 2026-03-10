from controllers.color_sensor_controller import ColourSensorController
from controllers.driving_controller import DrivingController

class SweepLineFollower:

    THETA_TOLERANCE = 3
    MOTOR_BASE_POWER_DPS = 150
    K_P = 1   # @TODO: Choose appropriate values of K_P and K_D.
    K_D = 0

    def __init__(self, sensor: ColourSensorController, pilot: DrivingController):
        self.sensor = sensor 
        self.pilot = pilot
        self.last_theta = 0

    def test_sweep(self):
        while True:
            # Sweep the colour sensor once
            sweep_result = self.sensor.sweep()

            if len(sweep_result) == 0:
                # Could not find the black line. Sweep again.
                continue 
            
            if len(sweep_result) == 1:
                # We've found the black line. Figure out if we need to correct
                theta, = sweep_result[0]

                if -self.THETA_TOLERANCE <= theta <= self.THETA_TOLERANCE:
                    # We are close enough to center that we can continue driving forward.
                    print("close to center. theta: ", theta)
                else:
                    # Adjust the motors to correct for the deviation
                    print("too far off. theta: ", theta)

                self.last_theta = theta
            elif len(sweep_result) == 3:
                # We've reached an intersection.
                # @TODO: Complete logic
                print("found an intersection")

    def begin(self):
        self.pilot.move_forward()

    def run(self):
        while True:
            # Sweep the colour sensor once
            sweep_result = self.sensor.sweep()

            if len(sweep_result) == 0:
                # Could not find the black line. Sweep again.
                continue 
            
            if len(sweep_result) == 1:
                # We've found the black line. Figure out if we need to correct
                theta, = sweep_result[0]

                if -self.THETA_TOLERANCE <= theta <= self.THETA_TOLERANCE:
                    # We are close enough to center that we can continue driving forward.
                    self.pilot.move_forward()
                else:
                    # Adjust the motors to correct for the deviation
                    adjustment = self.K_P * theta - self.K_D * (theta - self.last_theta)
                    self.pilot.adjust_direction(adjustment)

                self.last_theta = theta
            elif len(sweep_result) == 3:
                # We've reached an intersection.
                # @TODO: Complete logic
                pass
