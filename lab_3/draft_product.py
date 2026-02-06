# this is a draft of the final code for the entire thing (drum + flute)

# imports
import sys
sys.path.append("../")
from time import sleep, time
from utils import sound
from utils.brick import wait_ready_sensors, Motor, TouchSensor, EV3ColorSensor
print("done importing...")

# initializing ports
drum_toggle_button = TouchSensor(1)
flute_play_button = TouchSensor(2)
note_selector_motor = Motor("A")
drum_motor = Motor("B")
#kill_switch_sensor = EV3ColorSensor(2)
print("done initializing ports...")

# declaring constants
MOTOR_POWER = 10
STALL_WINDOW = 0.35
STALL_DELTA = 3
PAUSE_AT_END = 0.2
NOTES = ["A4", "B4", "C5", "D5", "E5", "F5", "G5"]
SOUNDS = [sound.Sound(duration=0.3, pitch=note, volume=80) for note in NOTES]
NOTE_ANG = 10
TEMPO = 0.5

# wait for sensors to be ready
wait_ready_sensors()
print("sensors ready")

# boolean helpers
drumming_active = False
kill_switch_active = False

# helper functions
def play_sound(note):
    note.play()
    note.wait_done()
    print(f"playing note {note}")


# main loop
while not kill_switch_active:
    # checking all toggle buttons here...
    if drum_toggle_button.is_pressed():
        drumming_active = not drumming_active

    if flute_play_button.is_pressed():
        play_sound(SOUNDS[(-1 * int(note_selector_motor.get_position())) % 6])

    # asserting state of motors given the boolean helpers
    if drumming_active:
        drum_motor.set_power(30)
    else:
        drum_motor.set_power(0)

print("kill switch activated...")
