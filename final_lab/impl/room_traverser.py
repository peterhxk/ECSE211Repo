from utils.brick import reset_brick, wait_ready_sensors, EV3UltrasonicSensor, Motor
from controllers.color_sensor_controller import ColourSensorController
import time
from data.ports_brick import Ports
from enum import Enum
from controllers.grabber_controller import GrabberController

class RoomType(Enum):
    CORNER_SINGLE = 1
    WALL_SINGLE = 2
    CORNER_DOUBLE_FIRST_HALF = 3
    CORNER_DOUBLE_SECOND_HALF = 4

room_type_to_wall_distance_map = {
    RoomType.CORNER_SINGLE: 5.3,
    RoomType.WALL_SINGLE: 53.4,
    RoomType.CORNER_DOUBLE_FIRST_HALF: 30,
    RoomType.CORNER_DOUBLE_SECOND_HALF: 4
}

MAX_UPDATE_COUNT = 100

class RoomTraverser:
    WHEEL_DIAMETER_MM = 42.6
    WHEEL_BASE_MM = 159
    MOTOR_POWER = 160
    
    # PID constants
    Kp = 8
    Ki = 2
    Kd = 3

    # Low-pass the *steering* command so motor DPS does not jump every loop
    adjustment_smooth_alpha = 0.7

    def __init__(self, left_motor: Motor, right_motor: Motor, base_power: int, color_sensor_controller: ColourSensorController):
        self.left_motor = left_motor
        self.right_motor = right_motor
        self.base_power = base_power
        self.ultrasonic_sensor = EV3UltrasonicSensor(Ports.ULTRASONIC_SENSOR.value)
        self.color_sensor_controller = color_sensor_controller
        # self.grabber_controller = GrabberController(Ports.GRABBER_MOTOR.value, Ports.GYRO_SENSOR.value)
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
    def run(self, room_type: RoomType = RoomType.CORNER_SINGLE, straight_distance: float = 25, detect_bed: bool = False):
        update_count = 0
        # reset motor encoders
        self.left_motor.reset_encoder()
        self.right_motor.reset_encoder()

        # reject spikes: ignore readings that jump more than this from the last accepted value
        OUTLIER_THRESHOLD_CM = 8

        last_distance_cm = None

        # PID state for wall following
        integral = 0.0
        prev_error = 0.0
        prev_time = time.time()
        smoothed_adjustment = None

        bed_detected = False

        target_degrees = (straight_distance * 10 / (self.WHEEL_DIAMETER_MM * 3.14159)) * 360

        motor_base_power = self.base_power
        start_dps = motor_base_power if straight_distance > 0 else -motor_base_power
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

            raw_adjustment = self.Kp * error + self.Ki * integral + self.Kd * derivative

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
                left_dps  = -self.base_power + adjustment * 1.3
                right_dps = -self.base_power - adjustment * 1.3

            if update_count < MAX_UPDATE_COUNT:
                self.left_motor.set_dps(left_dps)
                self.right_motor.set_dps(right_dps)
                update_count += 1
            else:
                # After a certain number of updates, stop adjusting and just go straight to avoid jitter.
                self.left_motor.set_dps(start_dps)
                self.right_motor.set_dps(start_dps)

            time.sleep(0.02)

        self.left_motor.set_dps(0)
        self.right_motor.set_dps(0)
        if bed_detected:
            detected_color = color_sensor_sweep_result[0][1]
            
            return (detected_color, (travelled_deg / 360) * (self.WHEEL_DIAMETER_MM * 3.14159) / 10)
        else:
            return (None, (travelled_deg / 360) * (self.WHEEL_DIAMETER_MM * 3.14159) / 10)
        
    """
    The main function to traverse a room depending on the room type. Everything is self contained here.
    This is the only function to call when you are in front of a room.

    Input:
    - room_type: The type of the room to traverse. This determines the path taken by the robot.
    """
    def traverse_a_room(self, room_type: RoomType) -> None:
        try:
            # here, we appear at the entry of the CORNER SINGLE ROOM, move straight up to 25CM or until detect bed.
            traverse_results = self.run(room_type, 35, detect_bed=True)
            bed_found = traverse_results[0] is not None
            bed_color_found = traverse_results[0]
            distance_travelled = traverse_results[1]
            
            if bed_found:
                print(f"Detected bed color: {bed_color_found}, Distance travelled: {distance_travelled:.2f} cm")
                if bed_color_found == "green":
                    print("DROPPING BLOCK!!!")
                time.sleep(2)
                # self.grabber_controller.start_unloading()
                # while not self.grabber_controller.unloading_finished():
                #     time.sleep(0.01)
                # if self.grabber_controller.get_unloading_result():
                #     print("Unloading successful")
                # else:
                #     print("Unloading failed")

            else:
                print(f"No bed detected, Distance travelled: {distance_travelled:.2f} cm")
            
            print("Going back.")
            self.run(room_type, -1.5 * distance_travelled, detect_bed=False)

        except KeyboardInterrupt:
            print("Interrupted via keyboard...")
        except Exception as err:
            print(err)  
        finally:  
            reset_brick()  