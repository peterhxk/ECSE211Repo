from utils.brick import reset_brick, wait_ready_sensors, EV3UltrasonicSensor, Motor
from controllers.color_sensor_controller import ColourSensorController
import time
from data.ports_brick import Ports
from enum import Enum
from controllers.grabber_controller import GrabberController
from controllers.driving_controller import DrivingController
from controllers.sound_controller import SoundController

class RoomType(Enum):
    CORNER_SINGLE = 1
    WALL_SINGLE = 2
    CORNER_DOUBLE_FIRST_HALF = 3
    CORNER_DOUBLE_SECOND_HALF = 4

room_type_to_wall_distance_map = {
    RoomType.CORNER_SINGLE: 4.8,
    RoomType.WALL_SINGLE: 53.4,
    RoomType.CORNER_DOUBLE_FIRST_HALF: 30,
    RoomType.CORNER_DOUBLE_SECOND_HALF: 4
}

class RoomTraverser:
    WHEEL_DIAMETER_MM = 42.6
    WHEEL_BASE_MM = 159
    
    # PID constants
    pid_constants = {
        "forward": {
            "Kp": 11,
            "Ki": 0,
            "Kd": 0
        },
        "backward": {
            "Kp": 1,
            "Ki": 0,
            "Kd": 0
        }
    }

    # Low-pass the *steering* command so motor DPS does not jump every loop
    adjustment_smooth_alpha = 0.7

    def __init__(self, left_motor: Motor, right_motor: Motor, base_power: int, color_sensor_controller: ColourSensorController):
        self.left_motor = left_motor
        self.right_motor = right_motor
        self.base_power = base_power
        self.ultrasonic_sensor = EV3UltrasonicSensor(Ports.ULTRASONIC_SENSOR.value)
        self.color_sensor_controller = color_sensor_controller
        self.driving_controller = DrivingController(left_motor, right_motor, base_power)
        self.sound_controller = SoundController()
        wait_ready_sensors(True)
        print("Room Traverser ready.")

    """ 
    Designed to make a robot move in a straight line until it detects a bed OR until it reaches a certain distance.
    Every call to the run() method is responsible for one straight forward or backwards path.
    
    Inputs:
    - wall_distance: The left distance from the wall to maintain while moving (in CM).
    - straight_distance: The MAXIMUM straight distance that the robot moves until the method returns. Defaults to 25cm.
    - detect_bed: Whether the robot should be detecting a bed or not (indicates whether the color sensor arm should be looking for a bed)

    Returns:
    - Tuple of (bed color detected or None, distance travelled in CM)
    """
    def run(self, room_type: RoomType = RoomType.CORNER_SINGLE, straight_distance: float = 25, detect_bed: bool = False, detect_start: bool = False):
        # Reset encoders first, then immediately float so the BrickPi3 position
        # controller (briefly engaged by reset_encoder) cannot lock the wheels
        # before set_dps takes over.
        self.left_motor.reset_encoder()
        self.right_motor.reset_encoder()
        self.left_motor.float_motor()
        self.right_motor.float_motor()

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
        orange_count = 0
        ORANGE_CONFIRM_THRESHOLD = 5

        target_degrees = (straight_distance * 10 / (self.WHEEL_DIAMETER_MM * 3.14159)) * 360

        motor_base_power = self.base_power
        start_dps = motor_base_power if straight_distance > 0 else -motor_base_power
        print(f"Starting with {start_dps} dps")
        self.left_motor.set_dps(start_dps)
        self.right_motor.set_dps(start_dps)

        while True:
            color_sensor_sweep_result = None
            if detect_bed:
                color_sensor_sweep_result = self.color_sensor_controller.sweep(colours=["red", "green"], range="full", stop_on_match=True)
            if color_sensor_sweep_result and color_sensor_sweep_result[0][1] in ["green", "red"]:
                # if color detected, stop moving.
                bed_detected = True
                break

            if detect_start:
                if self.color_sensor_controller.read_colour() == "orange":
                    orange_count += 1
                    print(f"Orange reading {orange_count}/{ORANGE_CONFIRM_THRESHOLD}")
                    if orange_count >= ORANGE_CONFIRM_THRESHOLD:
                        print("Orange confirmed — arrived back at starting point.")
                        start_detected = True
                        break
                else:
                    orange_count = 0

            # check travelled distance
            travelled_deg = (self.left_motor.get_encoder() + self.right_motor.get_encoder()) / 2.0

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

            # P (optional I/D later)
            error = room_type_to_wall_distance_map[room_type] - distance_cm
            integral = max(-50.0, min(50.0, integral + error * dt))
            derivative = (error - prev_error) / dt
            prev_error = error

            if straight_distance >= 0:
                pid_constants = self.pid_constants["forward"]
            else:
                pid_constants = self.pid_constants["backward"]
            raw_adjustment = pid_constants["Kp"] * error + pid_constants["Ki"] * integral + pid_constants["Kd"] * derivative

            # Keep differential steering bounded so one wheel does not reverse / stall abruptly
            base_abs = abs(motor_base_power)
            max_steer = max(12, min(int(base_abs * 0.42), base_abs - 15))
            raw_adjustment = max(-max_steer, min(max_steer, raw_adjustment))

            a = self.adjustment_smooth_alpha
            if smoothed_adjustment is None:
                smoothed_adjustment = raw_adjustment
            else:
                smoothed_adjustment = a * raw_adjustment + (1.0 - a) * smoothed_adjustment

            adjustment = int(round(smoothed_adjustment))

            # Sensor is LEFT-mounted.
            # Positive error = too close to left wall → steer RIGHT (away from wall).
            # Left wheel faster = steer right, so left gets +adj, right gets -adj.
            if straight_distance >= 0:
                left_dps  = self.base_power + adjustment * 1.3
                right_dps = self.base_power - adjustment * 1.3
            else:
                left_dps  = -self.base_power - adjustment * 0.8
                right_dps = -self.base_power + adjustment * 0.8


            print(f"Adjustment: {adjustment}, Left DPS: {left_dps}, Right DPS: {right_dps}")
            self.left_motor.set_dps(left_dps)
            self.right_motor.set_dps(right_dps)

            # Pace the loop so the ultrasonic sensor has time to produce a fresh
            # sample and the motor controller is not overwhelmed with commands.
            # When a sweep is active it naturally throttles the loop; only sleep
            # when running with no sweep at all (~25 Hz target).
            if not detect_bed and not detect_start:
                time.sleep(0.04)

        self.driving_controller.stop_motors()
        dist_cm = (abs(travelled_deg) / 360) * (self.WHEEL_DIAMETER_MM * 3.14159) / 10
        if bed_detected:
            detected_color = color_sensor_sweep_result[0][1]
            detected_angle = color_sensor_sweep_result[0][0]
            return (detected_color, dist_cm, detected_angle)
        elif start_detected:
            return ("orange", dist_cm, None)
        else:
            return (None, dist_cm, None)
        
    """
    The main function to traverse a room depending on the room type. Everything is self contained here.
    This is the only function to call when you are in front of a room.

    Input:
    - room_type: The type of the room to traverse. This determines the path taken by the robot.
    """
    def traverse_a_room(self, room_type: RoomType) -> None:
        try:
            # grabber_controller.start_loading()
            # timer = time.time()
            # print("Started inward belt movement")
            # while not grabber_controller.loading_finished():
            #     if time.time() - timer > 1:
            #         print("still loading")
            #         timer = time.time()
            #     time.sleep(0.01)
            
            # if grabber_controller.get_loading_result():
            #     print("Loading successful")
            # else:
            #     print("Loading failed")
            
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

                    # Turn towards green bed
                    print(f"Turning {turn_angle:.1f}° towards green bed (sensor arm was at {bed_angle:.1f}°)")
                    self.driving_controller.turn_in_place(turn_angle)
                    self.left_motor.wait_is_stopped()
                    self.right_motor.wait_is_stopped()
                    time.sleep(0.5)

                    # Drive towards bed
                    print(f"Driving {APPROACH_MM}mm towards green bed")
                    self.driving_controller.move_distance(APPROACH_MM)
                    self.left_motor.wait_is_stopped()
                    self.right_motor.wait_is_stopped()
                    time.sleep(0.5)

                    # Drop block
                    print("Dropping block")
                    self.sound_controller.play_success()
                    time.sleep(2)

                    # Drive back to where the bed was detected
                    print(f"Reversing {APPROACH_MM}mm from green bed")
                    self.driving_controller.move_distance(-APPROACH_MM)
                    self.left_motor.wait_is_stopped()
                    self.right_motor.wait_is_stopped()
                    time.sleep(0.5)

                    # Turn back to original heading
                    print(f"Turning back {-turn_angle:.1f}° to restore heading")
                    self.driving_controller.turn_in_place(-turn_angle)
                    self.left_motor.wait_is_stopped()
                    self.right_motor.wait_is_stopped()
                    time.sleep(0.5)

                    print("Back at original position, ready for backwards pass")
                else:
                    print("Red bed detected, skipping unloading")
                time.sleep(2)
            else:
                print(f"No bed detected, Distance travelled: {distance_travelled:.2f} cm")
            
            print("Going back.")
            back_result = self.run(room_type, -1.15 * distance_travelled, detect_bed=False, detect_start=True)
            if back_result[0] == "orange":
                print(f"Returned to starting point (orange detected) after {back_result[1]:.2f} cm.")
            else:
                print(f"Back run completed without detecting orange after {back_result[1]:.2f} cm.")
            
            self.sound_controller.play_mission_complete()
        except KeyboardInterrupt:
            print("Interrupted via keyboard...")
        except Exception as err:
            print(err)  
        finally:  
            reset_brick()  