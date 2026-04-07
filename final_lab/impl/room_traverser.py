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
from typing import Optional

class RoomType(Enum):
    CORNER_SINGLE = 1
    WALL_SINGLE = 2
    CORNER_DOUBLE_FIRST_HALF = 3
    CORNER_DOUBLE_SECOND_HALF = 4
    CORNER_DOUBLE_MIDDLE = 5

room_type_to_wall_distance_map = {
    RoomType.CORNER_SINGLE: 4.8,
    RoomType.WALL_SINGLE: 54.4,
    RoomType.CORNER_DOUBLE_FIRST_HALF: 29.7,
    RoomType.CORNER_DOUBLE_SECOND_HALF: 4.7,
    RoomType.CORNER_DOUBLE_MIDDLE: 17.7
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
                "straight_distance": 37,
            },
            RoomType.CORNER_DOUBLE_SECOND_HALF: {
                "Kp": 100,
                "Ki": 0,
                "Kd": 140,
                "steer_scale": 1.3,
                "ramp": 0.0,
                "straight_distance": 37,
            },
            RoomType.CORNER_DOUBLE_MIDDLE: {
                "Kp": 100,
                "Ki": 0,
                "Kd": 140,
                "steer_scale": 1.3,
                "ramp": 0.0,
                "straight_distance": 37,
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
                "Kp": 2,
                "Ki": 0,
                "Kd": 220,
                "steer_scale": 0.7,
                "ramp": 0.0,
                "straight_distance": -80,
            },
            RoomType.CORNER_DOUBLE_FIRST_HALF: {
                "Kp": 4,
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
            RoomType.CORNER_DOUBLE_MIDDLE: {
                "Kp": 1,
                "Ki": 0,
                "Kd": 180,
                "steer_scale": 0.7,
                "ramp": 0.0,
                "straight_distance": -80,
            }
        }
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
    Wall-follow in a straight line, optionally detecting beds or the start marker.

    For single room types the loop stops as soon as a bed is found (or max distance
    is reached). For double room types the sweep looks for green only; the loop runs
    until max distance or until the first green bed is seen, then stops so the caller
    can run a full sweep to distinguish one bed from two adjacent greens.

    An optional override_distance_cm parameter replaces the straight_distance that
    would normally come from the PID config table. Pass a negative value to travel
    backward without detect_start.

    Returns a 7-tuple:
        (first_bed_color, first_bed_dist_cm, first_bed_angle,
         second_bed_color, second_bed_dist_cm, second_bed_angle,
         total_dist_cm)

    For single rooms, second_bed_* entries are always None.
    When detect_start=True and orange is found, first_bed_color is "orange" and
    all other fields except total_dist_cm are None.
    """
    def _run(self, room_type: RoomType, detect_bed: bool = False,
             detect_start: bool = False, override_distance_cm: float = None):
        self.left_motor.reset_encoder()
        self.right_motor.reset_encoder()

        self.ultrasonic_sensor.wait_ready()
        self.color_sensor_controller.set_mode("component")
        self.color_sensor_controller.wait_ready()

        OUTLIER_THRESHOLD_CM = 8

        last_distance_cm = None

        integral = 0.0
        prev_error = 0.0
        prev_time = time.time()
        smoothed_adjustment = None

        first_bed_detected = False
        first_bed_color = None
        first_bed_angle = None
        first_bed_dist_cm = 0.0

        second_bed_detected = False
        second_bed_color = None
        second_bed_angle = None
        second_bed_dist_cm = 0.0

        start_detected = False

        is_double = room_type not in [RoomType.CORNER_SINGLE, RoomType.WALL_SINGLE]
        print(f"is_double: {is_double}")

        # Choose PID direction: backward when detect_start or when an explicit
        # negative distance override is given.
        going_backward = detect_start or (
            override_distance_cm is not None and override_distance_cm < 0
        )
        direction_key = "backward" if going_backward else "forward"
        pid_cfg = self.parameters[direction_key][room_type]

        straight_distance = (
            override_distance_cm
            if override_distance_cm is not None
            else pid_cfg["straight_distance"]
        )
        target_degrees = (straight_distance * 10 / (self.WHEEL_DIAMETER_MM * math.pi)) * 360

        motor_base_power = self.base_power
        start_dps = motor_base_power if straight_distance > 0 else -motor_base_power
        t0 = time.time()
        self.left_motor.set_dps(start_dps)
        self.right_motor.set_dps(start_dps)

        travelled_deg = 0.0
        try:
            bed_colours = ["green"] if is_double else ["red", "green"]
            if detect_bed:
                self.color_sensor_controller.sweep_start(colours=bed_colours, range="full")

            while True:
                travelled_deg = (
                    self.left_motor.get_encoder() + self.right_motor.get_encoder()
                ) / 2.0
                current_dist_cm = (
                    (abs(travelled_deg) / 360) * (self.WHEEL_DIAMETER_MM * math.pi) / 10
                )

                # Universal max-distance stop condition.
                if target_degrees > 0 and travelled_deg >= target_degrees:
                    break
                if target_degrees < 0 and travelled_deg <= target_degrees:
                    break

                if detect_bed:
                    sweep_result = self.color_sensor_controller.sweep_get_result()
                    if sweep_result and sweep_result[0][1] in bed_colours:
                        detected_color = sweep_result[0][1]
                        detected_angle = sweep_result[0][0]

                        if not is_double:
                            # Single room: stop immediately at the first bed.
                            print("SINGLE ROOM: stopping at first bed")
                            first_bed_detected = True
                            first_bed_color = detected_color
                            first_bed_angle = detected_angle
                            first_bed_dist_cm = current_dist_cm
                            break
                        else:
                            # Double room: green only — stop forward pass; caller
                            # runs a full sweep to see one vs two adjacent beds.
                            print("DOUBLE ROOM: green seen — end forward pass for full sweep")
                            first_bed_detected = True
                            first_bed_color = detected_color
                            first_bed_angle = detected_angle
                            first_bed_dist_cm = current_dist_cm
                            break

                if detect_start:
                    detected = self.color_sensor_controller.read_colour()
                    print(detected)
                    if detected == "orange":
                        self.left_motor.set_dps(-motor_base_power)
                        self.right_motor.set_dps(-motor_base_power)
                        print("Moving back to starting point")
                        
                        if detected in ["red", "green"] or detected == "yellow":
                            print("Yellow detected — continuing")
                            continue

                        detected = self.color_sensor_controller.read_colour()
                        while detected != "white/black":
                            detected = self.color_sensor_controller.read_colour()
                            time.sleep(0.01)
                            print("waiting to detect white/black")

                        print("Orange detected — arrived back at starting point.")
                        start_detected = True
                        self.left_motor.set_dps(0)
                        self.right_motor.set_dps(0)
                        break

                    if target_degrees < 0 and travelled_deg <= target_degrees / 10:
                        print(
                            f"[ARM] Halfway reached ({travelled_deg:.0f}/{target_degrees:.0f} deg),"
                            " resetting arm to 0°"
                        )
                        self.color_sensor_controller.set_arm_at_angle(0)

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

                raw_adjustment = (
                    pid_cfg["Kp"] * error
                    + pid_cfg["Ki"] * integral
                    + pid_cfg["Kd"] * derivative
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

                self.left_motor.set_dps(left_dps)
                self.right_motor.set_dps(right_dps)

                time.sleep(0.01)
        finally:
            self.driving_controller.stop_motors()
            if detect_bed:
                self.color_sensor_controller.sweep_stop()

        total_dist_cm = (abs(travelled_deg) / 360) * (self.WHEEL_DIAMETER_MM * math.pi) / 10

        if start_detected:
            return ("orange", total_dist_cm, None, None, None, None, total_dist_cm)

        return (
            first_bed_color,
            first_bed_dist_cm,
            first_bed_angle,
            second_bed_color,
            second_bed_dist_cm,
            second_bed_angle,
            total_dist_cm,
        )
    
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

        if self.grabber_controller.cube_count != expected_bloc_count:
            raise RuntimeError(
                f"Unexpected bloc_count after loading {label}: "
                f"expected {expected_bloc_count}, got {self.grabber_controller.cube_count}"
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

        if self.grabber_controller.cube_count != expected_bloc_count:
            raise RuntimeError(
                f"Unexpected bloc_count after unloading {label}: "
                f"expected {expected_bloc_count}, got {self.grabber_controller.cube_count}"
            )

    """
    The main function to traverse a room depending on the room type. Everything is self contained here.
    This is the only function to call when you are in front of a room.

    Input:
    - room_type: The type of the room to traverse. This determines the path taken by the robot.
    """
    def _approach_and_unload(self, bed_angle: float, bed_colour: str, approach_mm: int = 80) -> None:
        """
        Turn towards the detected bed, drive up to it, drop a block, then
        reverse and restore the original heading.

        The robot MUST already be wall-parallel (heading 0°) before this is
        called — callers are responsible for straightening first.

        Args:
            bed_angle:   Arm angle at detection (degrees, relative to robot body).
                         Positive = right.
            bed_colour:  Detected colour string ("green" or "red").
            approach_mm: How far to drive toward the bed (mm).
        """
        if bed_colour != "green":
            print(f"Bed colour is '{bed_colour}' — skipping unload.")
            return

        robot_turn_mag = self.driving_controller.convert_sensor_angle_to_robot_angle(bed_angle)
        turn_angle = math.copysign(robot_turn_mag, bed_angle) if bed_angle != 0 else 0

        self.color_sensor_controller.set_arm_at_angle(math.copysign(90, -1 * bed_angle))

        print(f"Turning {turn_angle:.1f}° towards green bed (arm at {bed_angle:.1f}°)")
        self.driving_controller.turn_in_place(turn_angle)
        self.left_motor.wait_is_stopped()
        self.right_motor.wait_is_stopped()

        print(f"Driving {approach_mm}mm towards green bed")
        self.driving_controller.move_distance(approach_mm)
        self.left_motor.wait_is_stopped()
        self.right_motor.wait_is_stopped()

        print("Dropping block")
        expected_after_unload = self.grabber_controller.cube_count - 1
        self.grabber_controller.start_unloading()
        self._wait_for_unloading(expected_bloc_count=expected_after_unload, label="unload")

        if self.sound_controller:
            self.sound_controller.play_success()
            self.sound_controller.wait_done()

        print(f"Reversing {approach_mm}mm from green bed")
        self.driving_controller.move_distance(-approach_mm)
        self.left_motor.wait_is_stopped()
        self.right_motor.wait_is_stopped()

        print(f"Turning back {-turn_angle:.1f}° to restore heading")
        self.driving_controller.turn_in_place(-turn_angle)
        self.left_motor.wait_is_stopped()
        self.right_motor.wait_is_stopped()

        print("Restored to wall-parallel heading.")
        time.sleep(0.1)

    def _double_room_full_sweep_bed_angles(self, ordered_detections: list) -> list[float]:
        """
        Parse hits from a single max-left → max-right arm pass (time order).

        Two separate green beds are only assumed if the sequence contains a yellow
        detection between the first contiguous green region and a later green region.
        Otherwise all greens in the sweep are treated as one bed (mean arm angle).
        """
        seq: list[tuple[float, str]] = []
        for t in ordered_detections:
            if len(t) < 2 or t[1] not in ("green", "yellow"):
                continue
            try:
                seq.append((float(t[0]), t[1]))
            except (TypeError, ValueError):
                continue
        if not seq:
            return []

        i = 0
        while i < len(seq) and seq[i][1] != "green":
            i += 1
        if i >= len(seq):
            return []

        j = i
        while j < len(seq) and seq[j][1] == "green":
            j += 1
        first_run_angles = [seq[k][0] for k in range(i, j)]

        k = j
        while k < len(seq):
            if seq[k][1] == "yellow":
                break
            if seq[k][1] == "green":
                # More green before any yellow — still the first bed (e.g. sweep re-hit).
                start_g = k
                while k < len(seq) and seq[k][1] == "green":
                    k += 1
                first_run_angles.extend(seq[t][0] for t in range(start_g, k))
                continue
            k += 1

        bed1 = sum(first_run_angles) / len(first_run_angles)

        if k >= len(seq) or seq[k][1] != "yellow":
            return [bed1]

        k += 1
        while k < len(seq) and seq[k][1] != "green":
            k += 1
        if k >= len(seq):
            return [bed1]

        m = k
        while m < len(seq) and seq[m][1] == "green":
            m += 1
        second_run_angles = [seq[t][0] for t in range(k, m)]
        bed2 = sum(second_run_angles) / len(second_run_angles)
        return [bed1, bed2]

    """
    The main function to traverse a room depending on the room type. Everything is
    self-contained here. This is the only function to call when you are in front of
    a room.

    Single room  (CORNER_SINGLE / WALL_SINGLE):
      1. Wall-follow forward; stop at first bed or max distance.
      2. If green bed: unload. If red bed: skip.
      3. Wall-follow backward to start (orange detection).

    Double room  (all other RoomType values) — one straight pass per call:
      1. Wall-follow forward while sweeping for green only (red ignored).
      2. If green is seen, stop, move arm to max left, sweep once left→max right
         logging green and yellow. Two beds only if yellow lies between green
         regions in that pass; then unload twice, else once.
      3. Wall-follow backward to start (orange detection).

    Input:
    - room_type: The type of the room to traverse.
    """
    def traverse_a_room(self, room_type: RoomType) -> None:
        is_double = room_type not in [RoomType.CORNER_SINGLE, RoomType.WALL_SINGLE]

        # --- Forward pass ---
        (
            first_color, first_dist, first_angle,
            second_color, second_dist, second_angle,
            total_dist,
        ) = self._run(room_type, detect_bed=True)

        if not is_double:
            # ------------------------------------------------------------------
            # Single room
            # ------------------------------------------------------------------
            if first_color is not None:
                print(
                    f"Bed detected: {first_color} at {first_dist:.2f} cm,"
                    f" angle {first_angle:.1f}°"
                )
                if first_color == "green":
                    self._approach_and_unload(first_angle, first_color)
                    print("UNLOADING")
                    if self.sound_controller:
                        self.sound_controller.play_success()
                        self.sound_controller.wait_done()
                    time.sleep(1)
                else:
                    print("Red bed — skipping unload (no sound).")
            else:
                print(f"No bed detected. Distance travelled: {total_dist:.2f} cm")

            print("Going back to start.")
            back = self._run(room_type, detect_start=True)
            if back[0] == "orange":
                print(f"Returned to start after {back[1]:.2f} cm.")
            else:
                print(f"Back run finished without orange after {back[1]:.2f} cm.")

        else:
            # ------------------------------------------------------------------
            # Double room — green only while driving; full sweep after first hit.
            # ------------------------------------------------------------------
            if first_color == "green":
                print(
                    f"Green bed hit while sweeping forward at {first_dist:.2f} cm,"
                    f" arm ~{first_angle:.1f}° — max-left then left→right sweep for beds"
                )
                raw_hits = self.color_sensor_controller.sweep_colours_max_left_to_right(
                    ["green", "yellow"],
                )
                bed_angles = self._double_room_full_sweep_bed_angles(raw_hits)
                if len(bed_angles) >= 2:
                    print(
                        "Left→right sweep: yellow between greens — two beds, unloading twice"
                    )
                    for i, ang in enumerate(bed_angles):
                        print(f"  Bed {i + 1} at mean angle {ang:.1f}°")
                        self._approach_and_unload(ang, "green")
                        print("UNLOADING")
                        time.sleep(1)
                elif len(bed_angles) == 1:
                    print(
                        f"Left→right sweep: single green region (no yellow between beds) "
                        f"at {bed_angles[0]:.1f}°"
                    )
                    self._approach_and_unload(bed_angles[0], "green")
                    print("UNLOADING")
                    time.sleep(1)
                else:
                    print(
                        "Left→right sweep found no green/yellow — using initial hit angle"
                    )
                    # self._approach_and_unload(first_angle, "green")
                    print("UNLOADING")
                    if self.sound_controller:
                        self.sound_controller.play_success()
                        self.sound_controller.wait_done()
                    time.sleep(1)
            elif first_color is not None:
                # Should not occur (double pass sweeps green only).
                print(f"Unexpected bed colour in double pass: {first_color}")
            else:
                print(f"No green bed on this pass. Travelled {total_dist:.2f} cm")

            print("Going back to start.")
            back = self._run(room_type, detect_start=True)
            if back[0] == "orange":
                print(f"Returned to start after {back[1]:.2f} cm.")
            else:
                print(f"Back run finished without orange after {back[1]:.2f} cm.")

    """
    --- UNDER THIS LINE IS LEGACY DOUBLE ROOM TRAVERSAL CODE ---
    """

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

    def align_to_wall_continuous(
        self,
        turn_dps: int = 50,
        sample_dt: float = 0.01,
        reversal_limit: int = 3,
        max_iters: int = 2000,
        offset: int = -1.5,
        window_size: int = 5,
    ):
        """
        Align the robot to the wall using continuous closed-loop turning.

        Strategy:
        - Turn slowly in one direction, sampling the ultrasonic sensor each tick.
        - Raw readings are smoothed with a sliding window median to remove noise.
        - As long as the smoothed reading keeps decreasing, continue in the same direction.
        - The moment the smoothed reading increases (minimum was passed), reverse direction.
        - Stop after reversal_limit reversals — the robot is oscillating around
          the minimum and is considered aligned.
        """

        def _get_us() -> Optional[float]:
            v = self.ultrasonic_sensor.get_cm()
            try:
                fv = float(v)
                if self._US_CM_MIN <= fv <= self._US_CM_MAX:
                    return fv
            except (TypeError, ValueError):
                pass
            return None

        window = []

        def _smoothed_us() -> Optional[float]:
            v = _get_us()
            if v is None:
                return None
            window.append(v)
            if len(window) > window_size:
                window.pop(0)
            sorted_w = sorted(window)
            mid = len(sorted_w) // 2
            return sorted_w[mid] if len(sorted_w) % 2 else (sorted_w[mid - 1] + sorted_w[mid]) / 2.0

        # Prime the window before starting to move.
        prev = None
        for _ in range(window_size):
            v = _smoothed_us()
            if v is not None:
                prev = v
            time.sleep(sample_dt)
        if prev is None:
            print("[align_continuous] no US reading — aborting")
            return

        direction = 1   # +1 = left, -1 = right (matches turn_in_place sign convention)
        reversals = 0

        self.left_motor.set_dps(direction * turn_dps)
        self.right_motor.set_dps(-direction * turn_dps)
        print(f"[align_continuous] starting — initial US={prev:.1f} cm")

        time.sleep(0.1)  # brief initial delay to let the robot start moving before sampling
        for _ in range(max_iters):
            time.sleep(sample_dt)

            cur = _smoothed_us()
            if cur is None:
                continue

            print(f"[align_continuous] US={cur:.1f} cm  dir={'+' if direction > 0 else '-'}  reversals={reversals}")

            if cur > prev:
                # Reading got bigger — we passed the minimum, reverse.
                direction = -direction
                reversals += 1
                if reversals >= reversal_limit:
                    break
                self.left_motor.set_dps(direction * turn_dps)
                self.right_motor.set_dps(-direction * turn_dps)

            prev = cur

        self.left_motor.set_dps(0)
        self.right_motor.set_dps(0)
        print(f"[align_continuous] done — final US={prev:.1f} cm")

        self.left_motor.set_dps(offset * turn_dps)
        self.right_motor.set_dps(-offset * turn_dps)
        time.sleep(0.5)
        self.left_motor.set_dps(0)
        self.right_motor.set_dps(0)
        print(f"[align_continuous] done — calibration done")




    def align_to_wall(self, search_range_deg: float = 40.0, step_deg: float = 4.0):
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

    def move_differential(self, left_dist_mm: float, right_dist_mm: float, power: int = None):
        """
        Move each wheel an independent distance (mm) at synchronized speeds so
        both finish at the same time.

        Args:
            left_dist_mm:  Distance for the left wheel (negative = backward).
            right_dist_mm: Distance for the right wheel (negative = backward).
            power:         Peak DPS for the faster wheel. Defaults to self.base_power.
                           Pass a higher value here to make the move execute faster
                           without changing the global base_power.
        """
        left_deg  = left_dist_mm  * 360 / self.WHEEL_DIAMETER_MM / math.pi
        right_deg = right_dist_mm * 360 / self.WHEEL_DIAMETER_MM / math.pi

        max_deg = max(abs(left_deg), abs(right_deg))
        if max_deg == 0:
            return

        peak = power if power is not None else self.base_power
        left_dps  = int(peak * abs(left_deg)  / max_deg)
        right_dps = int(peak * abs(right_deg) / max_deg)

        self.left_motor.set_limits(100, left_dps)
        self.right_motor.set_limits(100, right_dps)

        self.left_motor.set_position_relative(left_deg)
        self.right_motor.set_position_relative(right_deg)

        self.left_motor.wait_is_stopped()
        self.right_motor.wait_is_stopped()

    def run_double_room(self, room_type: RoomType):
        NUM_ITERATIONS = 4
        turn_angles = {
            RoomType.CORNER_DOUBLE_FIRST_HALF: {
                "smaller_radius_deg": -50,
                "larger_radius_deg": 140,
            },
            RoomType.CORNER_DOUBLE_SECOND_HALF: {
                "smaller_radius_deg": -30,
                "larger_radius_deg": 100,
            },
        }


        self.left_motor.reset_encoder()
        self.right_motor.reset_encoder()

        POWER = 320

        print("MOVING FORWARD, SWEEPING")

        smaller_radius_deg = turn_angles[room_type]["smaller_radius_deg"]
        larger_radius_deg = turn_angles[room_type]["larger_radius_deg"]

        EXCLUSION_THRESHOLD = 15.0  # degrees — beds within this angle are considered the same bed
        excluded_angles: list = []
        heading_at_detection = 0.0
        cumulative_heading = 0.0

        def _check_sweep():
            result = self.color_sensor_controller.sweep_get_result()
            if result:
                potential_combined = result[0][0] + cumulative_heading
                if any(abs(potential_combined - ea) < EXCLUSION_THRESHOLD for ea in excluded_angles):
                    # Already visited this bed — restart sweep and ignore
                    self.color_sensor_controller.sweep_stop()
                    self.color_sensor_controller.sweep_start(colours=["red", "green"], range="full")
                    return None
                nonlocal heading_at_detection
                heading_at_detection = cumulative_heading
            return result

        moves = [
            (10, 10),
            (0, smaller_radius_deg / 2),
            (larger_radius_deg / 2, 0),
        ]
        for _ in range(NUM_ITERATIONS):
            moves += [
                (smaller_radius_deg, 0),
                (0, larger_radius_deg),
                (0, smaller_radius_deg),
                (larger_radius_deg, 0),
            ]
        moves += [
            (smaller_radius_deg / 2, 0),
            (0, larger_radius_deg / 2),
        ]

        move_idx = 0

        while move_idx < len(moves) and self.grabber_controller.cube_count > 0:
            self.color_sensor_controller.sweep_start(colours=["red", "green"], range="full")
            bed_result = None

            while move_idx < len(moves):
                left_mm, right_mm = moves[move_idx]
                self.move_differential(left_mm, right_mm, POWER)
                cumulative_heading += (left_mm - right_mm) * 180.0 / (math.pi * self.WHEEL_BASE_MM)
                move_idx += 1
                bed_result = _check_sweep()
                if bed_result:
                    print(f"Bed detected during sweep: {bed_result}")
                    break

            self.color_sensor_controller.sweep_stop()
            self.color_sensor_controller.set_arm_at_angle(0)

            # Straighten back to wall-parallel before any approach.
            print(f"Straightening: turning {-cumulative_heading:.1f}° (accumulated {cumulative_heading:.1f}°)")
            self.driving_controller.turn_in_place(-cumulative_heading)
            self.left_motor.wait_is_stopped()
            self.right_motor.wait_is_stopped()
            cumulative_heading = 0.0

            if bed_result:
                angle, colour, _ = bed_result[0]
                combined_angle = angle + heading_at_detection
                print(f"  colour={colour}  arm={angle:.1f}°  heading={heading_at_detection:.1f}°  combined={combined_angle:.1f}°")
                self._approach_and_unload(combined_angle, colour)
                excluded_angles.append(combined_angle)
                heading_at_detection = 0.0

        print("Moving back.")
        self._run(room_type=room_type, detect_bed=False, detect_start=True)
    