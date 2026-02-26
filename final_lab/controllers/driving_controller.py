import time

class DrivingController():
    def __init__(self, left_motor, right_motor):
        self.left_motor = left_motor
        self.right_motor = right_motor

    def set_power(self, power):
        self.left_motor.set_power(power)
        self.right_motor.set_power(power)

    def go_right(self):
        self.left_motor.set_dps(50)
        self.right_motor.set_dps(20)

    def go_left(self):
        self.left_motor.set_dps(20)
        self.right_motor.set_dps(50)
    
    def drive(self, color_sensor):
        direction = 'l'
        past_black = False
        cur_colour = None
        timer = None
        self.go_left()
        while True:
            colour = color_sensor.read_colour()
            if not timer:
                timer = time()
                cur_colour = colour
            else:
                if colour != cur_colour:
                    timer = time()
                    cur_colour = colour
                elif time() - timer >= 0.5:
                    if colour == "b":
                        past_black = True
                    elif past_black and colour == "w":
                        if direction == 'l':
                            self.go_right()
                            direction = 'r'
                        else:
                            self.go_left()
                            direction = 'l'
                        past_black = False
            time.sleep(0.05)
