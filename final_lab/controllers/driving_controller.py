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
        self.go_left()
        while True:
            colour = color_sensor.read_colour()
            print(colour)
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
