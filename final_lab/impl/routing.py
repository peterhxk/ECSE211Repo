from impl.sweep_line_follower import SweepLineFollower
import data.measurements as consts
import time

class Routing(SweepLineFollower):

    """Class defines fixed routing commands to move the robot between rooms."""

    def pharmacy_to_room1(self):
        print("Reversing 50mm")
        self.pilot.move_distance(-50)
        self.pilot.right_motor.wait_is_stopped()
        print("Motors stopped")
        
        print("Turning right")
        self.turn_logic("right", "back")
        
        print("Turning right")
        self.turn_logic("right", "front")
        
        print("Moving forward, searching for orange line")
        self.move_forward(200, "orange")
        
        print("Found orange line.")
    
    def room1_to_pharmacy(self):
        self.pilot.move_distance(-430)
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
        self.pilot.right_motor.wait_is_stopped()
        self.turn_logic("right", "front")
        self.turn_logic("right", "front", consts.dist_blocks_mm-10, arm_avoid_block=True)

    def grab_block(self):
        self.reset_arm()