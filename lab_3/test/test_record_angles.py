# imports
import sys

sys.path.append("../")
direction = 1

from time import sleep, time
from utils import sound
from utils.brick import wait_ready_sensors, reset_brick, Motor, TouchSensor, EV3UltrasonicSensor

print("done importing...")

# initializing ports
drum_toggle_button = TouchSensor(1)
flute_play_button = TouchSensor(2)
US_sensor = EV3UltrasonicSensor(3)
note_selector_motor = Motor("A")
drum_motor = Motor("D")
print("done initializing ports...")

# declaring constants
MOTOR_POWER = 10
STALL_WINDOW = 0.35
STALL_DELTA = 3
PAUSE_AT_END = 0.2
NOTES = ["A4", "B4", "C5", "D5", "E5", "F5", "G5"]
SOUNDS = [sound.Sound(duration=0.3, pitch=note, volume=80) for note in NOTES]
NOTE_ANG = 350 / len(NOTES)
NOTE_ANG_LIST = [i * NOTE_ANG for i in range(len(NOTES))]
NOTE_ANG = 10
TEMPO = 0.5
SOUND_DURATION = 0.5

# boolean helpers
kill_switch_active = False
kill_switch_timer = None

# drumming FSM
drum_sensor_pressed = False  # state
drumming_active = False

# flute note state
flute_sensor_pressed = False
sound_playing_now = False
sound_timer = None


# helper functions
def play_sound(note):
    global sound_playing_now, sound_timer, SOUND_DURATION
    if not sound_playing_now:
        note.play()
        sound_playing_now = True
        sound_timer = time()
    else:
        if sound_timer and time() - sound_timer > SOUND_DURATION:
            sound_timer = None
            sound_playing_now = False

    # note.wait_done()
    print(f"playing note {note}")


def motor_command(direction: int):
    """Commands the motor"""
    note_selector_motor.set_power(direction * MOTOR_POWER)


def motor_stop():
    """Stops the motor"""
    note_selector_motor.set_power(0)


def is_stalling(window_s=STALL_WINDOW, delta_ticks=STALL_DELTA):
    """Determine if the motor is immobile - stalling"""
    p0 = note_selector_motor.get_position()
    t0 = time()
    while time() - t0 < window_s:
        sleep(0.02)
    p1 = note_selector_motor.get_position()
    return abs(p1 - p0) <= delta_ticks


# wait for sensors to be ready
wait_ready_sensors(True)
print("sensors ready")

# SETUP : calibrate motor
motor_command(1)
while True:
    if is_stalling():
        motor_stop()
        sleep(PAUSE_AT_END)
        note_selector_motor.reset_encoder()
        print("MOTOR STALLED : RESETTING ENCODER")

        # Now unblock the motor to run in read-only mode
        note_selector_motor.float_motor()
        break

# main loop
try:
    while True:
        direction *= -1
        motor_command(direction)
        note_selector_motor.set_power(direction * MOTOR_POWER)
        while not is_stalling():
            sleep(0.2)
        print("stalled")

except KeyboardInterrupt:
    print("Interrupted via keyboard...")
except Exception as err:
    print(err)
finally:
    reset_brick()  # Turn off everything on the brick's hardware, and reset it
    exit()
