from impl.sweep_line_follower import SweepLineFollower
import data.measurements as consts
from controllers.grabber_controller import GrabberController

import time
from scripts.navigation_grabber import wait_for_loading

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
        self.pilot.right_motor.wait_is_stopped(0.1)

    
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
        self.pilot.move_distance(108)
        self.pilot.right_motor.wait_is_stopped(0.1)
        grabberInstance.start_loading()
        time.sleep(0.5)
        wait_for_loading(0,timeout=15.0,label="block 1") #loading first block 
        self.pilot.move_distance(80)
        self.pilot.right_motor.wait_is_stopped(0.1)
        self.pilot.stop_motors()
        time.sleep(0.5)
        # print("Waiting for 2 seconds before loading second block...")
        # time.sleep(2)
        grabberInstance.start_loading()
        time.sleep(0.5)
        wait_for_loading(1,timeout=15.0,label="block 2") #loading second block
        self.pilot.move_distance(-100)
        self.pilot.right_motor.wait_is_stopped(0.1)

        #reset arm angle at the end
        self.reset_arm()
        self.pilot.move_backward()
        while self.sensor.read_colour() != "orange":
            time.sleep(0.01)
        self.pilot.stop_motors()