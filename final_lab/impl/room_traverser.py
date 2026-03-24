from utils.brick import Motor, wait_ready_sensors, EV3UltrasonicSensor
from controllers.driving_controller import DrivingController
from controllers.color_sensor_controller import ColourSensorController
from collections import deque
import statistics
import time
from data.ports_brick import Ports

class RoomTraverser:
    WHEEL_DIAMETER_MM = 42.6
    WHEEL_BASE_MM = 159
    
    # PID constants
    Kp = 0.265
    Ki = 0
    Kd = 0.6

    # filter parameter
    ema_alpha = 0.4

    def __init__(self, motor_controller: DrivingController, ultrasonic_sensor: EV3UltrasonicSensor):
        self.motor_controller = motor_controller
        self.ultrasonic_sensor = ultrasonic_sensor
        self.color_sensor_motor = Motor(Ports.COLOR_SENSOR_MOTOR_PORT)
        self.color_sensor_controller = ColourSensorController(Ports.COLOR_SENSOR, self.color_sensor_motor)
        wait_ready_sensors(True)
        print("Room Traverser ready.")

    """ 
    Designed to make a robot move in a straight line until it detects a bed OR until it reaches a certain distance.
    Every call to the run() method is responsible for one straight forward or backwards path.
    
    Inputs:
    - wall_distance: The left distance from the wall to maintain while moving (in CM).
    - straight_distance: The MAXIMUM straight distance that the robot moves until the method returns.
    - detect_bed: Whether the robot should be detecting a bed or not (indicates whether the color sensor arm should be looking for a bed)

    Returns:
    - Tuple of (bed color detected or None, distance travelled in CM)
    """
    def run(self, wall_distance: float, straight_distance: float, detect_bed: bool = False) -> tuple[str | None, float]:
        # reset motor encoders
        self.motor_controller.reset_encoders()

        # initiate filter window for the stupid ultrasonic sensor
        median_window = deque(maxlen=5)

        # filter parameters
        ema_estimate = None

        # PID stuff for wall following
        integral = 0.0
        prev_error = 0.0
        prev_time = time.time()

        bed_detected = False

        target_degrees = (straight_distance * 10 / (self.WHEEL_DIAMETER_MM * 3.14159)) * 360

        self.motor_controller.move_forward()

        while True:
            if detect_bed: color_sensor_sweep_result = self.color_sensor_controller.sweep(colour="green", sweep_range="full", stop_on_match=True)
            if color_sensor_sweep_result and color_sensor_sweep_result[0][1] == "green":
                # if color detected, stop moving.
                bed_detected = True
                break

            # check travelled distance
            travelled_deg = self.motor_controller.get_avg_encoder_value()

            if travelled_deg >= target_degrees: 
                break

            now = time.time()
            dt = max(now - prev_time, 0.001)
            prev_time = now

            # outlier filtering step
            raw = self.ultrasonic_sensor.get_cm()
            median_window.append(raw)
            denoised = statistics.median(median_window)

            # EMA smoothing
            if ema_estimate is None:
                ema_estimate = denoised
            else:
                ema_estimate = self.ema_alpha * denoised + (1 - self.ema_alpha) * ema_estimate

            distance_cm = ema_estimate

            # PID control part
            print(distance_cm)
            error = wall_distance - distance_cm
            integral = max(-50.0, min(50.0, integral + error * dt))
            derivative = (error - prev_error) / dt
            prev_error = error

            adjustment = int(self.Kp * error + self.Ki * integral + self.Kd * derivative)

            self.motor_controller.set_left_motor(self.motor_controller.base_power - adjustment)
            self.motor_controller.set_right_motor(self.motor_controller.base_power + adjustment)

        self.motor_controller.stop_motors()
        if bed_detected:
            return ("green", (travelled_deg / 360) * (self.WHEEL_DIAMETER_MM * 3.14159) / 10)
        else:
            return (None, (travelled_deg / 360) * (self.WHEEL_DIAMETER_MM * 3.14159) / 10)