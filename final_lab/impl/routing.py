from sweep_line_follower import SweepLineFollower

class Routing(SweepLineFollower):

    """Class defines fixed routing commands to move the robot between rooms."""

    def pharmacy_to_room1(self):
        self.pilot.move_distance(-50)
        self.pilot.right_motor.wait_is_stopped()
        self.turn_logic("right", "back")
        self.turn_logic("right", "front")
        self.move_forward(200, "o")
    
    def room1_to_pharmacy(self):
        self.pilot.move_backward(430)
    
    def room1_to_room2(self):
        self.turn_logic("right", "back")
        self.move_forward(200)
        self.turn_logic("left", "front")
        self.move_forward(50, "o")
    
    def room2_to_room3A(self):
        self.turn_logic("right", "back")
        self.turn_logic("right", "front")
        self.move_forward(50, "o")

    def room3A_to_room3B(self):
        self.turn_logic("left", "back")
        self.turn_logic("right", "front")
        self.move_forward(50, "o") 
