import time

class DrivingController():
    def __init__(self, left_motor, right_motor):
        self.left_motor = left_motor
        self.right_motor = right_motor

    def set_power(self, power):
        self.left_motor.set_power(power)
        self.right_motor.set_power(power)

    def go_right(self):
        self.left_motor.set_dps(80)
        self.right_motor.set_dps(30)

    def go_left(self):
        self.left_motor.set_dps(30)
        self.right_motor.set_dps(80)

    def move_straight(self, x, power):
        
        print(f"Move straight by {x} with {power}% power")
        left_motor.set_power(power)
        self.right_motor.set_power(power)
        sleep(x) # CHANGE THIS
        self.left_motor.set_power(0)
        self.right_motor.set_power(0)

    def move_straight(x):
        """x: distance in cm"""
        move_straight(x, self.MOTOR_POWER)

    def turn_angle(x, power):
        """x: angle in degrees?
        power: value -100 to 100"""
        left_motor.set_power(power)
        right_motor.set_power(-power)
        sleep(x) # CHANGE THIS
        left_motor.set_power(0)
        right_motor.set_power(0)

    def turn_angle(x):
        turn_angle(x, MOTOR_POWER)

    def turn_right(power):
        turn_angle(90, power)

    def turn_right():
        turn_angle(90)

    def turn_left(power):
        turn_angle(-90,power)

    def turn_left():
        turn_angle(-90)
    
    def drive(self, color_sensor):
        direction = 'l'
        past_black = False
        cur_colour = None
        timer = None
        self.go_left()
        while True:
            colour = color_sensor.read_colour()
            if timer is None:
                timer = time.time()
                cur_colour = colour
            else:
                if colour != cur_colour:
                    timer = time.time()
                    cur_colour = colour
                elif time.time() - timer >= 0.1:
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
