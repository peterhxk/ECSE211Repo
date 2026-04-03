from utils.brick import reset_brick, wait_ready_sensors, EV3UltrasonicSensor, Motor
from controllers.color_sensor_controller import ColourSensorController
import time
from data.ports_brick import Ports
from enum import Enum
from controllers.grabber_controller import GrabberController
from controllers.driving_controller import DrivingController
from controllers.sound_controller import SoundController
import math
import threading

class RoomType(Enum):
    CORNER_SINGLE = 1
    WALL_SINGLE = 2
    CORNER_DOUBLE_FIRST_HALF = 3
    CORNER_DOUBLE_SECOND_HALF = 4

room_type_to_wall_distance_map = {
    RoomType.CORNER_SINGLE: 4.8,
    RoomType.WALL_SINGLE: 54.4,
    RoomType.CORNER_DOUBLE_FIRST_HALF: 30,
    RoomType.CORNER_DOUBLE_SECOND_HALF: 4
}

def _kill_switch_callback():
    print("Kill switch pressed, resetting brick...")
    reset_brick()
    exit()

class RoomTraverser:
    WHEEL_DIAMETER_MM = 42.6
    WHEEL_BASE_MM = 159
    
    # PID constants
    parameters = {
        "forward": {
            RoomType.CORNER_SINGLE: {
                "Kp": 100,
                "Ki": 0,
                "Kd": 140,
                "steer_scale": 1.3,
                "ramp": 0.0,
                "straight_distance": 45,
            },
            RoomType.WALL_SINGLE: {
                "Kp": 205,
                "Ki": 0,
                "Kd": 170,
                "steer_scale": 1.3,
                "ramp": 0.0,
                "straight_distance": 40,
            },
            RoomType.CORNER_DOUBLE_FIRST_HALF: {
                "Kp": 100,
                "Ki": 0,
                "Kd": 140,
                "steer_scale": 1.3,
                "ramp": 0.0,
                "straight_distance": 40,
            },
            RoomType.CORNER_DOUBLE_SECOND_HALF: {
                "Kp": 100,
                "Ki": 0,
                "Kd": 140,
                "steer_scale": 1.3,
                "ramp": 0.0,
                "straight_distance": 40,
            },
        }, 
        "backward": {
            RoomType.CORNER_SINGLE: {
                "Kp": 1,
                "Ki": 0,
                "Kd": 180,
                "steer_scale": 0.7,
                "ramp": 0.0,
                "straight_distance": -80,
            },
            RoomType.WALL_SINGLE: {
                "Kp": 1,
                "Ki": 0,
                "Kd": 180,
                "steer_scale": 0.7,
                "ramp": 0.0,
                "straight_distance": -80,
            },
            RoomType.CORNER_DOUBLE_FIRST_HALF: {
                "Kp": 1,
                "Ki": 0,
                "Kd": 180,
                "steer_scale": 0.7,
                "ramp": 0.0,
                "straight_distance": -80,
            },
            RoomType.CORNER_DOUBLE_SECOND_HALF: {
                "Kp": 1,
                "Ki": 0,
                "Kd": 180,
                "steer_scale": 0.7,
                "ramp": 0.0,
                "straight_distance": -80,
            },
        },
        }
    

    # Low-pass the *steering* command so motor DPS does not jump every loop
    adjustment_smooth_alpha = 0.7

    def __init__(self, driving_controller: DrivingController, color_sensor_controller: ColourSensorController, grabber_controller: GrabberController, sound_controller: SoundController = None):
        self.left_motor = driving_controller.left_motor
        self.right_motor = driving_controller.right_motor
        self.base_power = driving_controller.base_power
        self.driving_controller = driving_controller
        self.color_sensor_controller = color_sensor_controller
        self.grabber_controller = grabber_controller
        self.sound_controller = sound_controller
        self.ultrasonic_sensor = EV3UltrasonicSensor(Ports.ULTRASONIC_SENSOR.value)
        wait_ready_sensors(True)
        print("Room Traverser ready.")

    """ 
    Designed to make a robot move in a straight line until it detects a bed OR until it reaches a certain distance.
    Every call to the run() method is responsible for one straight forward or backwards path.
    
    Inputs:
    - wall_distance: The left distance from the wall to maintain while moving (in CM).
    - straight_distance: The MAXIMUM straight distance that the robot moves until the method returns. Defaults to 25cm.
    - detect_bed: Whether the robot should be detecting a bed or not (indicates whether the color sensor arm should be looking for a bed)
    - detect_start: Whether the robot should be detecting the starting point or not (indicates whether the color sensor arm should be looking for the starting point)

    Returns:
    - Tuple of (bed color detected or None, distance travelled in CM)
    """
    def run(self, room_type: RoomType = RoomType.CORNER_SINGLE, detect_bed: bool = False, detect_start: bool = False):
        # Reset encoders first, then immediately float so the BrickPi3 position
        # controller (briefly engaged by reset_encoder) cannot lock the wheels
        # before set_dps takes over.
        self.left_motor.reset_encoder()
        self.right_motor.reset_encoder()
        self.left_motor.float_motor()
        self.right_motor.float_motor()

        # BrickPi3 reset_all() sets every port's SensorType to NONE. Without
        # re-applying modes, get_sensor raises "Sensor not configured or not supported."
        # (e.g. second run() after reset_brick, or any prior reset_all).
        self.ultrasonic_sensor.set_mode(self.ultrasonic_sensor.Mode.CM)
        self.ultrasonic_sensor.wait_ready()
        self.color_sensor_controller.set_mode("component")
        self.color_sensor_controller.wait_ready()

        # reject spikes: ignore readings that jump more than this from the last accepted value
        OUTLIER_THRESHOLD_CM = 8

        last_distance_cm = None

        # PID state for wall following
        integral = 0.0
        prev_error = 0.0
        prev_time = time.time()
        smoothed_adjustment = None

        bed_detected = False
        start_detected = False

        # parameters are keyed by direction then room type (not room_type at top level)
        direction_key = "backward" if detect_start else "forward"
        pid_cfg = self.parameters[direction_key][room_type]
        straight_distance = pid_cfg["straight_distance"]
        target_degrees = (straight_distance * 10 / (self.WHEEL_DIAMETER_MM * math.pi)) * 360

        motor_base_power = self.base_power
        start_dps = motor_base_power if straight_distance > 0 else -motor_base_power
        t0 = time.time()
        self.left_motor.set_dps(start_dps)
        self.right_motor.set_dps(start_dps)

        travelled_deg = 0.0
        try:
            if detect_bed:
                self.color_sensor_controller.sweep_start(colours=["red", "green"], range="full")

            while True:
                if detect_bed:
                    color_sensor_sweep_result = self.color_sensor_controller.sweep_get_result()
                    if color_sensor_sweep_result and color_sensor_sweep_result[0][1] in ["green", "red"]:
                        bed_detected = True
                        break

                if detect_start:
                    detected = self.color_sensor_controller.read_colour_interval()
                    if detected == "orange":
                        print("Orange detected — arrived back at starting point.")
                        start_detected = True
                        break

                    # check travelled distance
                    travelled_deg = (self.left_motor.get_encoder() + self.right_motor.get_encoder()) / 2.0

                    if target_degrees < 0 and travelled_deg <= target_degrees / 5.5:
                        print(f"[ARM] Halfway reached ({travelled_deg:.0f}/{target_degrees:.0f} deg), resetting arm to 0°")
                        self.color_sensor_controller.set_arm_at_angle(0)
                    if target_degrees > 0 and travelled_deg >= target_degrees:
                        break
                    elif target_degrees < 0 and travelled_deg <= target_degrees:
                        break

                now = time.time()
                dt = max(now - prev_time, 0.001)
                prev_time = now

                raw = self.ultrasonic_sensor.get_cm()
                if last_distance_cm is None or abs(raw - last_distance_cm) <= OUTLIER_THRESHOLD_CM:
                    last_distance_cm = raw

                distance_cm = last_distance_cm

                error = room_type_to_wall_distance_map[room_type] - distance_cm
                integral = max(-50.0, min(50.0, integral + error * dt))
                derivative = (error - prev_error) / dt
                prev_error = error

                g_p = self._wall_gain_scale(error, self.wall_gain_schedule_kp)
                g_d = self._wall_gain_scale(error, self.wall_gain_schedule_kd)
                raw_adjustment = (
                    pid_cfg["Kp"] * g_p * error
                    + pid_cfg["Ki"] * integral
                    + pid_cfg["Kd"] * g_d * derivative
                )

                if pid_cfg["ramp"] > 0:
                    ramp_factor = min(1.0, (now - t0) / pid_cfg["ramp"])
                    raw_adjustment *= ramp_factor

                base_abs = abs(motor_base_power)
                max_steer = max(12, min(int(base_abs * 0.42), base_abs - 15))
                raw_adjustment = max(-max_steer, min(max_steer, raw_adjustment))

                a = self.adjustment_smooth_alpha
                if smoothed_adjustment is None:
                    smoothed_adjustment = raw_adjustment
                else:
                    smoothed_adjustment = a * raw_adjustment + (1.0 - a) * smoothed_adjustment

                adjustment = int(round(smoothed_adjustment))
                s = pid_cfg["steer_scale"]

                if straight_distance >= 0:
                    left_dps  = self.base_power + adjustment * s
                    right_dps = self.base_power - adjustment * s
                else:
                    left_dps  = -self.base_power - adjustment * s
                    right_dps = -self.base_power + adjustment * s

                print(f"Adjustment: {adjustment}, Left DPS: {left_dps}, Right DPS: {right_dps}")
                self.left_motor.set_dps(left_dps)
                self.right_motor.set_dps(right_dps)

                time.sleep(0.04)
        finally:
            self.driving_controller.stop_motors()
            if detect_bed:
                self.color_sensor_controller.sweep_stop()

        dist_cm = (abs(travelled_deg) / 360) * (self.WHEEL_DIAMETER_MM * math.pi) / 10
        if bed_detected:
            detected_color = color_sensor_sweep_result[0][1]
            detected_angle = color_sensor_sweep_result[0][0]
            return (detected_color, dist_cm, detected_angle)
        elif start_detected:
            return ("orange", dist_cm, None)
        else:
            return (None, dist_cm, None)
    
    def _wait_for_loading(self, expected_bloc_count: int, timeout: float = 15.0, label: str = "block"):
        start = time.time()
        last_log = start

        while True:
            status = self.grabber_controller.get_loading_status()

            if status == "FAILED":
                raise RuntimeError(f"Failed while loading {label}")

            if self.grabber_controller.loading_finished():
                break

            now = time.time()
            if now - last_log > 1.0:
                print(f"[DEBUG] Waiting for loading of {label}... status={status}")
                last_log = now

            if now - start > timeout:
                raise TimeoutError(f"Timeout while loading {label}")

            time.sleep(0.05)

        final_status = self.grabber_controller.get_loading_status()
        print(f"[DEBUG] Loading finished for {label} with status={final_status}")

        if self.grabber_controller.bloc_count != expected_bloc_count:
            raise RuntimeError(
                f"Unexpected bloc_count after loading {label}: "
                f"expected {expected_bloc_count}, got {self.grabber_controller.bloc_count}"
            )

    def _wait_for_unloading(self, expected_bloc_count: int, timeout: float = 15.0, label: str = "block"):
        start = time.time()
        last_log = start

        while True:
            status = self.grabber_controller.get_unloading_status()

            if status == "FAILED":
                raise RuntimeError(f"Failed while unloading {label}")

            if status != "UNLOADING":
                break

            now = time.time()
            if now - last_log > 1.0:
                print(f"[DEBUG] Waiting for unloading of {label}... status={status}")
                last_log = now

            if now - start > timeout:
                raise TimeoutError(f"Timeout while unloading {label}")

            time.sleep(0.05)

        final_status = self.grabber_controller.get_unloading_status()
        print(f"[DEBUG] Unloading finished for {label} with status={final_status}")

        if self.grabber_controller.bloc_count != expected_bloc_count:
            raise RuntimeError(
                f"Unexpected bloc_count after unloading {label}: "
                f"expected {expected_bloc_count}, got {self.grabber_controller.bloc_count}"
            )

    """
    The main function to traverse a room depending on the room type. Everything is self contained here.
    This is the only function to call when you are in front of a room.

    Input:
    - room_type: The type of the room to traverse. This determines the path taken by the robot.
    """
    def traverse_a_room(self, room_type: RoomType) -> None:
        # here, we appear at the entry of the CORNER SINGLE ROOM, move straight up to 25CM or until detect bed.
        traverse_results = self.run(room_type, 45, detect_bed=True)
        bed_found = traverse_results[0] is not None
        bed_color_found = traverse_results[0]
        distance_travelled = traverse_results[1]
        bed_angle = traverse_results[2]
        
        if bed_found:
            print(f"Detected bed color: {bed_color_found}, Distance travelled: {distance_travelled:.2f} cm, Angle: {bed_angle}")
            if bed_color_found == "green":
                APPROACH_MM = 80

                robot_turn_mag = self.driving_controller.convert_sensor_angle_to_robot_angle(bed_angle)
                if bed_angle > 0:
                    turn_angle = robot_turn_mag
                elif bed_angle < 0:
                    turn_angle = -robot_turn_mag
                else:
                    turn_angle = 0

                self.color_sensor_controller.set_arm_at_angle(math.copysign(90, -1 * bed_angle))

                # Turn towards green bed
                print(f"Turning {turn_angle:.1f}° towards green bed (sensor arm was at {bed_angle:.1f}°)")
                self.driving_controller.turn_in_place(turn_angle)
                self.left_motor.wait_is_stopped()
                self.right_motor.wait_is_stopped()

                # Drive towards bed
                print(f"Driving {APPROACH_MM}mm towards green bed")
                self.driving_controller.move_distance(APPROACH_MM)
                self.left_motor.wait_is_stopped()
                self.right_motor.wait_is_stopped()

                # Drop one block (2 -> 1)
                print("Dropping block")

                self.grabber_controller.start_unloading()
                self._wait_for_unloading(expected_bloc_count=1, label="unload")

                self.sound_controller.play_success()
                self.sound_controller.wait_done()

                # Drive back to where the bed was detected
                print(f"Reversing {APPROACH_MM}mm from green bed")
                self.driving_controller.move_distance(-APPROACH_MM)
                self.left_motor.wait_is_stopped()
                self.right_motor.wait_is_stopped()

                # Turn back to original heading
                print(f"Turning back {-turn_angle:.1f}° to restore heading")
                self.driving_controller.turn_in_place(-turn_angle)
                self.left_motor.wait_is_stopped()
                self.right_motor.wait_is_stopped()

                print("Back at original position, ready for backwards pass")
            else:
                print("Red bed detected, skipping unloading")
            time.sleep(0.1)
        else:
            print(f"No bed detected, Distance travelled: {distance_travelled:.2f} cm")
        
        print("Going back.")
        back_result = self.run(room_type, -2 * distance_travelled, detect_bed=False, detect_start=True)
        if back_result[0] == "orange":
            print(f"Returned to starting point (orange detected) after {back_result[1]:.2f} cm.")
        else:
            print(f"Back run completed without detecting orange after {back_result[1]:.2f} cm.")

    # Ultrasonic: settle after motion, then median over samples (spike-resistant).
    _US_ENTRY_SETTLE_S = 0.15
    _US_ENTRY_SAMPLES = 9
    _US_ENTRY_INTERVAL_S = 0.04
    _US_CM_MIN = 2.0
    _US_CM_MAX = 250.0

    def _ultrasonic_cm_robust(self) -> float:
        """
        One stable reading in cm after the robot has stopped: brief settle, then
        median of several samples (EV3 ultrasonic often throws outliers).
        """
        time.sleep(self._US_ENTRY_SETTLE_S)
        buf: list[float] = []
        for _ in range(self._US_ENTRY_SAMPLES):
            v = self.ultrasonic_sensor.get_cm()
            if v is not None:
                try:
                    fv = float(v)
                    if self._US_CM_MIN <= fv <= self._US_CM_MAX:
                        buf.append(fv)
                except (TypeError, ValueError):
                    pass
            time.sleep(self._US_ENTRY_INTERVAL_S)
        if not buf:
            v = self.ultrasonic_sensor.get_cm()
            return float(v) if v is not None else 0.0
        buf.sort()
        mid = len(buf) // 2
        if len(buf) % 2:
            return buf[mid]
        return (buf[mid - 1] + buf[mid]) / 2.0

    # Sweep alignment: how many samples to take at each step (fewer than _US_ENTRY_SAMPLES
    # is fine — noise averages out across all the sweep steps).
    _US_SWEEP_SETTLE_S = 0.08
    _US_SWEEP_SAMPLES = 5

    def _ultrasonic_cm_sweep(self) -> float:
        """Faster variant of _ultrasonic_cm_robust for use in alignment sweeps.
        Uses fewer samples; individual noise is tolerable since the whole sweep
        curve is evaluated and only the argmin matters."""
        time.sleep(self._US_SWEEP_SETTLE_S)
        buf: list[float] = []
        for _ in range(self._US_SWEEP_SAMPLES):
            v = self.ultrasonic_sensor.get_cm()
            if v is not None:
                try:
                    fv = float(v)
                    if self._US_CM_MIN <= fv <= self._US_CM_MAX:
                        buf.append(fv)
                except (TypeError, ValueError):
                    pass
            time.sleep(self._US_ENTRY_INTERVAL_S)
        if not buf:
            v = self.ultrasonic_sensor.get_cm()
            return float(v) if v is not None else 0.0
        buf.sort()
        mid = len(buf) // 2
        return buf[mid] if len(buf) % 2 else (buf[mid - 1] + buf[mid]) / 2.0

    def _enc_to_robot_deg(self, dl: float, dr: float) -> float:
        """Convert left/right encoder deltas (motor degrees) to robot heading change.

        For turn_in_place(+θ): left goes +θ·ORIENTTODEG, right goes -θ·ORIENTTODEG.
        So: robot_deg = (dl - dr) / (2 * ORIENTTODEG).
        """
        return (dl - dr) / (2.0 * self.driving_controller.ORIENTTODEG)

    # Fraction of the computed correction to actually command.
    # < 1.0 deliberately under-turns to compensate for motor coast that carries
    # the chassis a few degrees past wait_is_stopped().
    # Tune empirically: if the robot still overshoots slightly, lower toward 0.80;
    # if it now undershoots, raise toward 1.0.
    ALIGN_CORRECTION_GAIN = 0.88

    def align_to_wall(self, search_range_deg: float = 50.0, step_deg: float = 4.0):
        """
        Align the robot so the ultrasonic sensor is pointing directly at the
        wall (minimum reading = robot parallel to the wall).

        Strategy: sweep through a fixed arc, take a filtered reading at each
        step, find the angle that gave the minimum, then turn to it.

        - Encoder feedback tracks *actual* rotation at every step, eliminating
          dead-reckoning drift that builds up over many small commanded turns.
        - Parabolic sub-step refinement uses the corrected vertex formula.
        - 3-point moving average smooths the curve before argmin.
        - ALIGN_CORRECTION_GAIN < 1 compensates for motor coast past stop.

        Args:
            search_range_deg: Total arc to sweep, centred on the current heading (default ±25°).
            step_deg:         Angular increment between readings (default 4°).
        """
        half = search_range_deg / 2.0
        n_steps = max(3, int(round(search_range_deg / step_deg)) + 1)

        def snap_enc():
            return self.left_motor.get_encoder(), self.right_motor.get_encoder()

        # --- Step 1: move to the start of the sweep arc, measure actual rotation ---
        l0, r0 = snap_enc()
        self.driving_controller.turn_in_place(-half)
        self.left_motor.wait_is_stopped()
        self.right_motor.wait_is_stopped()
        l1, r1 = snap_enc()
        cumulative = self._enc_to_robot_deg(l1 - l0, r1 - r0)
        print(f"[align] start turn: commanded {-half:.1f}°, actual {cumulative:.1f}°")

        # --- Step 2: sweep forward, recording actual angle at each step ---
        angles: list[float] = []
        readings: list[float] = []

        for i in range(n_steps):
            r = self._ultrasonic_cm_sweep()
            angles.append(cumulative)
            readings.append(r)
            print(f"[align] {cumulative:+.1f}°  →  {r:.1f} cm")
            if i < n_steps - 1:
                l0, r0 = snap_enc()
                self.driving_controller.turn_in_place(step_deg)
                self.left_motor.wait_is_stopped()
                self.right_motor.wait_is_stopped()
                l1, r1 = snap_enc()
                cumulative += self._enc_to_robot_deg(l1 - l0, r1 - r0)

        # --- Step 3: smooth with a 3-point moving average, then find coarse minimum ---
        smoothed = list(readings)
        for i in range(1, len(readings) - 1):
            smoothed[i] = (readings[i - 1] + readings[i] + readings[i + 1]) / 3.0

        min_idx = smoothed.index(min(smoothed))
        best_angle = angles[min_idx]

        # --- Step 4: parabolic sub-step refinement ---
        # Parabola through (−1, yl), (0, ym), (+1, yr): vertex at x = −(yr−yl) / (2*(yl+yr−2*ym))
        if 0 < min_idx < len(smoothed) - 1:
            yl, ym, yr = smoothed[min_idx - 1], smoothed[min_idx], smoothed[min_idx + 1]
            denom = yl + yr - 2.0 * ym  # positive for a true minimum
            # Require meaningful curvature to avoid amplifying noise on flat curves.
            if denom > 0.3:
                sub_step = -(yr - yl) / (2.0 * denom)  # in units of one step
                actual_step = (angles[min_idx + 1] - angles[min_idx - 1]) / 2.0
                best_angle += sub_step * actual_step
                print(f"[align] parabolic: sub-step {sub_step:+.3f} × {actual_step:.1f}° = {sub_step * actual_step:+.2f}°")

        # --- Step 5: turn from end of sweep to best_angle, dampened by gain ---
        raw_correction = best_angle - cumulative
        correction = raw_correction * self.ALIGN_CORRECTION_GAIN
        print(f"[align] raw correction {raw_correction:+.1f}°  ×  gain {self.ALIGN_CORRECTION_GAIN} = commanded {correction:+.1f}°")
        self.driving_controller.turn_in_place(correction)
        self.left_motor.wait_is_stopped()
        self.right_motor.wait_is_stopped()
        time.sleep(0.05)  # let chassis fully settle

        print(f"[align] done — min reading {readings[min_idx]:.1f} cm at sweep angle {angles[min_idx]:+.1f}°")


    def run_double_room(self, room_type: RoomType):
        NUM_ITERATIONS = 2
        turn_angles = {
            RoomType.CORNER_DOUBLE_FIRST_HALF: {
                "smaller_radius_deg": -30,
                "larger_radius_deg": 180,
            },
            RoomType.CORNER_DOUBLE_SECOND_HALF: {
                "smaller_radius_deg": -15,
                "larger_radius_deg": 150,
            },
        }


        self.left_motor.reset_encoder()
        self.right_motor.reset_encoder()

        for _ in range(NUM_ITERATIONS):
            self.left_motor.set_position(turn_angles[room_type]["smaller_radius_deg"])
            self.right_motor.set_position(turn_angles[room_type]["larger_radius_deg"])
            self.left_motor.wait_is_stopped()
            self.right_motor.wait_is_stopped()

            self.left_motor.reset_encoder()
            self.right_motor.reset_encoder()

            self.left_motor.set_position(turn_angles[room_type]["larger_radius_deg"])
            self.right_motor.set_position(turn_angles[room_type]["smaller_radius_deg"])
            self.left_motor.wait_is_stopped()
            self.right_motor.wait_is_stopped()

            self.left_motor.reset_encoder()
            self.right_motor.reset_encoder()

        self.right_motor.set_position(turn_angles[room_type]["larger_radius_deg"] - turn_angles[room_type]["smaller_radius_deg"])
        self.left_motor.wait_is_stopped()
        self.right_motor.wait_is_stopped()

        self.run(room_type=room_type, detect_bed=False, detect_start=True)
    