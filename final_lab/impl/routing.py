from impl.sweep_line_follower import SweepLineFollower
import data.measurements as consts
from controllers.grabber_controller import GrabberController
import threading
import time

class Routing(SweepLineFollower):
    """Class defines fixed routing commands to move the robot between rooms."""

    ROOM_EXIT_DST = -50
    """Distance (mm) we must back up when exiting the room
    so that the colour sensor is not on the orange line"""

    def _exit_room(self):
        """Backs off the orange line and waits for motors to stop."""
        self.pilot.move_distance(self.ROOM_EXIT_DST)
        self.pilot.right_motor.wait_is_stopped()

    def _advance_into_pharmacy(self):
        """From the main horizontal black line, advances into the pharmacy"""
        res = self.move_forward(300, "orange") 
        if res == "colour":
            self.pilot.move_distance(150)

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
        # Back off of orange line
        print('[DEBUG] routing: Exiting room')
        self._exit_room()

        print('[DEBUG] routing: Beginning left turn')
        self.turn_logic("right", "back")

        print('[DEBUG] routing: Moving forward 200mm')
        self.move_forward(230)

        print('[DEBUG] routing: Beginning second left turn')
        self.turn_logic("left", "front")
    
    def room2_to_room3A(self):
        self._exit_room()

        self.turn_logic("right", "back")
        self.turn_logic("right", "front")

    def room3A_to_room3C(self):
        self._exit_room()

        self.turn_logic("left", "back")
        self.turn_logic("right", "front", additional_dist=15)

    def room3C_to_room3B(self):
        self._exit_room()

        self.turn_logic("right", "back")
        self.move_forward(30)
        self.pilot.turn_90("left")

        self.pilot.right_motor.wait_is_stopped()

    def room2_to_pharmacy(self):
        self._exit_room()

        self.turn_logic("left", "back")
        self.turn_logic("left", "front")

        self._advance_into_pharmacy()

    def room3A_to_pharmacy(self):
        self._exit_room()

        self.turn_logic("right", "back")
        self.move_forward(220)
        self.turn_logic("left", "front")

        self._advance_into_pharmacy()

    def room3C_to_pharmacy(self):
        self._exit_room()

        self.turn_logic("right", "back")
        self.move_forward(500)
        self.turn_logic("left", "front")

        self._advance_into_pharmacy()

    def stop_on_orange(self):
        while self.sensor.read_colour() != "orange":
            time.sleep(0.01)
        print("FOUND ORANGE LINE : stopping motors")
        self.pilot.stop_motors() # stop both motors : this will set "wait_is_stopped()" to True
        return

    def pharmacy_to_grab_block_position(self):
        self.pilot.move_distance(50)
        self.pilot.right_motor.wait_is_stopped(0.1)
        self.turn_logic("right", "front")
        threading.Thread(target=self.stop_on_orange).start()
        self.turn_logic("right", "front",consts.dist_blocks_mm, arm_avoid_block=True) # horizontal distance right?
        # self.turn_logic("right", "front", consts.dist_blocks_mm, arm_avoid_block=True)xw
        print("Finished turning right")
        # Edit :
        # 1. stop moving once the color sensor sees orange
        # 2. Rotate the colour sweeper arm 180 degrees
        # 3. Rotate the wheel opposite to the arm forward until the colour sensor sees orange.
        # 4. The robot should then be lined up with the virtual wall and the blocks
        self.sensor.set_arm_at_angle(90)
        time.sleep(0.1)
        self.pilot.pivot_angle("left",-90)
        threading.Thread(target=self.stop_on_orange).start()
        self.pilot.stop_motors()
        self.reset_arm()

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