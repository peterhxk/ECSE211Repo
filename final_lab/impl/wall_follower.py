from utils.brick import wait_ready_sensors, EV3UltrasonicSensor
from controllers.driving_controller import DrivingController
from collections import deque
import statistics
import time

class WallFollower:
    WHEEL_DIAMETER_MM = 42.6
    WHEEL_BASE_MM = 159

    def __init__(self, motor_controller: DrivingController, ultrasonic_sensor: EV3UltrasonicSensor):
        self.motor_controller = motor_controller
        self.ultrasonic_sensor = ultrasonic_sensor
        print("initializing wall follower...", end="\r", flush=True)
        wait_ready_sensors(True)
        print("Wall follower ready.         ")

    """ 
        Designed to make the robot follow a straight line
        (indirectly achieved by left-following a wall)
            - wall_distance: the distance from the wall that the robot should try to maintain
            - straight_distance: the distance (in cm) that the robot will move forward by before stopping
    """

    def run(self, wall_distance: float, straight_distance: float):
        # reset motor encoders
        self.motor_controller.reset_encoders()

        # initiate filter window for the stupid ultrasonic sensor
        median_window = deque(maxlen=5)

        # filter parameters
        ema_estimate = None
        ema_alpha = 0.4

        # PID stuff for wall following
        integral = 0.0
        prev_error = 0.0
        prev_time = time.time()

        Kp = 0.5
        Ki = 0.01
        Kd = 0.2

        target_degrees = (straight_distance / (self.WHEEL_DIAMETER_MM * 3.14159)) * 360

        self.motor_controller.move_forward()

        while True:
            # check travelled distance
            travelled_deg = self.motor_controller.get_avg_encoder_value()

            if travelled_deg >= target_degrees: 
                break

            now = time.time()
            dt = max(now - prev_time, 0.001)
            prev_time = now

            # outlier filtering step
            raw = self.ultrasonic_sensor.get_cm()
            print(raw)
            median_window.append(raw)
            denoised = statistics.median(median_window)

            # EMA smoothing
            if ema_estimate is None:
                ema_estimate = denoised
            else:
                ema_estimate = ema_alpha * denoised + (1 - ema_alpha) * ema_estimate

            distance_cm = ema_estimate

            # PID control part
            error = wall_distance - distance_cm
            integral = max(-50.0, min(50.0, integral + error * dt))
            derivative = (error - prev_error) / dt
            prev_error = error

            adjustment = int(Kp * error + Ki * integral + Kd * derivative)

            self.motor_controller.set_left_motor(self.motor_controller.base_power - adjustment)
            self.motor_controller.set_right_motor(self.motor_controller.base_power + adjustment)

        self.motor_controller.stop_motors()