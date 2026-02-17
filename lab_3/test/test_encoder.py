from time import sleep
import time
"""
This code performs the testing of the calibration process for the motor encoder.
An automatic calibration is done at the beginning to find the maximum angle of the beam, which is used to map the angles to musical notes.
The calibration works as follows : the motor is run in one direction (arbitrary) until it stalls (using live velocity feedback). Once stalling, the motor stops and resets the encoder position. 
"""
from utils.brick import Motor, reset_brick,wait_ready_sensors,TouchSensor
from utils import sound

motor = Motor("B")
button = TouchSensor("2")

POWER = 10              # adjust
STALL_WINDOW = 0.35     # seconds to observe
STALL_DELTA = 3         # encoder ticks considered "no movement"
PAUSE_AT_END = 0.2      # pause before reversing

angles = []


def motor_command(direction: int):
    """
    direction: +1 or -1
    Replace this with your library's way of running the motor continuously.
    Examples in other libs: motor.set_power(direction*POWER)
    """
    motor.set_power(direction * POWER)

def motor_stop():
    """
    Replace with your library's stop/brake call.
    """
    motor.set_power(0)
    # may need to add a soft brake so the beam fully gets in position (currently I think it does a fast stop)

def is_stalling(window_s=STALL_WINDOW, delta_ticks=STALL_DELTA):
    p0 = motor.get_position()
    t0 = time.time()
    while time.time() - t0 < window_s:
        sleep(0.02)
    p1 = motor.get_position()
    return abs(p1 - p0) <= delta_ticks

wait_ready_sensors(True)

direction = +1

# SETUP : calibrate motor
motor_command(direction)
timeout = 4 # set a timeout in case the stalling is not detected and thus prevent damage on the motor
t0 = time.time()

def reset_encoder():
    while True:
        if time.time() - t0 > timeout:
            raise RuntimeError("The BrickPi timed out to detect stalling. Please check connections")
        if is_stalling():
            motor_stop()
            sleep(PAUSE_AT_END)
            sleep(0.5) # let the motor stop
            motor.reset_encoder()
            print("MOTOR STALLED : RESETTING ENCODER")
            break

# now play the tones


print("Getting into loop")

if __name__ == "__main__":
    reset_encoder()
    while True :
        # get a value from reading the sensor (replace with your library's way of reading the sensor) and map it to even intervals to play the corresponding note. For example, if the sensor value is between 0 and 8, play note 0. If it's between 8 and 16, play note 1, etc. You can adjust the intervals as needed.
        sensor_value = (-1)*motor.get_position()  # multiply by -1 because it goes in the negative angles
        # only play the tone if the button is pressed

        print("Sensor value: (degrees)", sensor_value)

        sleep(1) # wait for a second

        # motor.get_position() returns the encoder value of the motor, which is set to 0 when initializing the motor.
        # use Motor.reset_encoder() to reset the position
        #offset_encoder also helps manually offsetting the position
