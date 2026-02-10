import sys
from time import sleep, time
import csv
"""
This code uses a large motor's encoder to play musical notes
An automatic calibration is done at the beginning to find the maximum angle of the beam, which is used to map the angles to musical notes.
The calibration works as follows : the motor is run in one direction (arbitrary) until it stalls (using live velocity feedback). Once stalling, the motor stops and resets the encoder position. 
"""

sys.path.append("../")
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
    t0 = time()
    while time() - t0 < window_s:
        sleep(0.02)
    p1 = motor.get_position()
    return abs(p1 - p0) <= delta_ticks

wait_ready_sensors(True)

direction = +1

# SETUP : calibrate motor
motor_command(direction)
while True:
    if is_stalling():
        motor_stop()
        sleep(PAUSE_AT_END)
        motor.reset_encoder()
        print("MOTOR STALLED : RESETTING ENCODER")
        break

# now play the tones

DELAY_SEC = 0.01  # seconds of delay between measurements

NOTES = ["A4", "B4", "C5", "D5", "E5", "F5", "G5"]   # The notes to be played
SOUNDS = [sound.Sound(duration=0.3, pitch=note, volume=80) for note in NOTES]   # An array of Sounds corresponding to the musical notes
NOTE_DST = 15        # The physical distance threshold between each note
#TEMPO = 0.05                        # The tempo of the music (sec/note)

def play_sound(note):
    "Play a single note."
    note.play()
    note.wait_done()

print("Getting into loop")

while True :
    # get a value from reading the sensor (replace with your library's way of reading the sensor) and map it to even intervals to play the corresponding note. For example, if the sensor value is between 0 and 8, play note 0. If it's between 8 and 16, play note 1, etc. You can adjust the intervals as needed.
    sensor_value = (-1)*motor.get_position()  # multiply by -1 because it goes in the negative angles
    # only play the tone if the button is pressed
    if button.is_pressed():
        print("button pressed, sensor value: ", sensor_value)
        for i in range(len(NOTES)):
            if sensor_value < NOTE_DST * (i+1):
                # play note i
                print(f"note {i}")
                play_sound(SOUNDS[i])
                break;
        else:
            # do nothing
            print("no note")
"""
    while True:
        try:
            # Wait until stall is detected
            while True:
                if is_stalling():
                    motor_stop()
                    sleep(PAUSE_AT_END)
                    direction *= -1
                    angles.append(motor.get_position())
                    break
    
                sleep(0.01)
    
        except KeyboardInterrupt:
            print("Interrupting")
            with open("angles.csv", "w", newline="") as f:
                writer = csv.writer(f)
                writer.writerow(["angle"])
                for a in angles:
                    writer.writerow([a])
            print("Saved angles:", angles)
            reset_brick()
            break
"""

    # motor.get_position() returns the encoder value of the motor, which is set to 0 when initializing the motor.
    # use Motor.reset_encoder() to reset the position
    #offset_encoder also helps manually offsetting the position
