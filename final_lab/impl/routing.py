from controllers.color_sensor_controller import ColourSensorController
from controllers.driving_controller import DrivingController
from impl.sweep_line_follower import SweepLineFollower
import data.measurements as consts
from controllers.grabber_controller import GrabberController
import threading

import time

class Routing(SweepLineFollower):
    """Class defines fixed routing commands to move the robot between rooms."""

    def __init__(self, sensor: ColourSensorController, pilot: DrivingController):
        super().__init__(sensor, pilot)
        self.thread: threading.Thread = None

    def _begin_routing(self, worker):
        """
        Helper to begin routing in a separate thread.
        
        Args:
            worker (function): The worker to run in a separate thread
        
        Returns:
            success (bool): `False` if another worker is already being run in the thread,
                `True` otherwise
        """

        if self.thread and self.thread.is_active():
            return False
        else:
            self.thread = threading.Thread(target=worker, daemon=True)
            return True
        
    def kill(self):
        """Abruptly terminates all actions and routing commands."""
        self.kill_event.set()
        self.pilot.stop_motors()

    def pharmacy_to_room1(self):
        self.pilot.move_distance(-50)
        self.pilot.right_motor.wait_is_stopped()
        self.turn_logic("right", "back")
        self.turn_logic("right", "front")
        self.move_forward(200, "orange")
    
    def room1_to_pharmacy(self):
        self.pilot.move_distance(-435)
        self.pilot.right_motor.wait_is_stopped()

    
    def room1_to_room2(self):
        self.turn_logic("right", "back")
        self.move_forward(200)
        self.turn_logic("left", "front")
        self.move_forward(50, "orange")
    
    def room2_to_room3A(self):
        self.turn_logic("right", "back")
        self.turn_logic("right", "front")
        self.move_forward(50, "orange")

    def room3A_to_room3B(self):
        self.turn_logic("left", "back")
        self.turn_logic("right", "front")
        self.move_forward(50, "orange") 

    def pharmacy_to_grab_block_position(self):
        self.pilot.move_distance(50)
        self.pilot.right_motor.wait_is_stopped(0.1)
        self.turn_logic("right", "front")
        self.turn_logic("right", "front", consts.dist_blocks_mm, arm_avoid_block=True)

    def grab_block(self,grabberInstance:GrabberController):
        self.pilot.move_distance(110)
        self.pilot.right_motor.wait_is_stopped(0.1)
        attempts = 0
        EXTRA_DISTANCE = 10
        while attempts < 3:
            grabberInstance.start_loading(timeout=5.0)
            time.sleep(0.5)
            if grabberInstance.wait_for_loading(0, label="block 1"):
                print(f"Successfully loaded block 1 after {attempts} attempts")
                break
            else:
                self.pilot.move_distance(EXTRA_DISTANCE)
                self.pilot.right_motor.wait_is_stopped(0.1)
                attempts+=1
        if attempts == 3:
            raise RuntimeError
        time.sleep(0.5)
        self.pilot.move_distance(79-attempts*EXTRA_DISTANCE)
        self.pilot.right_motor.wait_is_stopped(0.1)
        self.pilot.stop_motors()
        time.sleep(0.5)
        attempts = 0
        EXTRA_DISTANCE = 5
        while attempts < 2:
            grabberInstance.start_loading(timeout=5.0)
            time.sleep(0.5)
            if grabberInstance.wait_for_loading(0, label="block 2"):
                print(f"Successfully loaded block 2 after {attempts} attempts")
                break
            else:
                self.pilot.move_distance(EXTRA_DISTANCE)
                self.pilot.right_motor.wait_is_stopped(0.1)
                attempts += 1
        if attempts == 3:
            raise RuntimeError
        self.pilot.move_distance(-100-attempts*EXTRA_DISTANCE) # move extra if had to move forward
        self.pilot.right_motor.wait_is_stopped(0.1)

        #reset arm angle at the end
        self.reset_arm()
        self.pilot.move_backward()
        while self.sensor.read_colour() != "orange":
            time.sleep(0.01)
        self.pilot.stop_motors()
