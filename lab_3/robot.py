# imports
from time import sleep, time
from utils import sound
from utils.brick import wait_ready_sensors, reset_brick, Motor, TouchSensor, EV3UltrasonicSensor
print("done importing...")

# initializing ports
drum_toggle_button = TouchSensor(1)
flute_play_button = TouchSensor(2)
US_sensor = EV3UltrasonicSensor(3)
note_selector_motor = Motor("A")
drum_motor = Motor("B")
print("done initializing ports...")

# declaring constants
MOTOR_POWER = 10
STALL_WINDOW = 0.35
STALL_DELTA = 3
PAUSE_AT_END = 0.2
NOTES = ["A4", "B4", "C5", "D5", "E5", "F5", "G5"]
SOUNDS = [sound.Sound(duration=0.3, pitch=note, volume=80) for note in NOTES]
NOTE_ANG = 20
NOTE_ANG_LIST = [i * NOTE_ANG for i in range(len(NOTES))]
NOTE_ANG = 10
TEMPO = 0.5

# boolean helpers
drumming_active = False
kill_switch_active = False
kill_switch_timer = None

# helper functions
def play_sound(note):
    note.play()
    note.wait_done()
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
        break

# main loop
try:
    while not kill_switch_active:
        sensor_value = note_selector_motor.get_position()
        # checking drum toggle button
        if drum_toggle_button.is_pressed():
            drumming_active = not drumming_active
            
        # asserting state of motors given the boolean helpers
        if drumming_active:
            drum_motor.set_power(30)
        else:
            drum_motor.set_power(0)

    # checking flute toggle button
    if flute_play_button.is_pressed():
        for ang, sound in zip(NOTE_ANG_LIST, SOUNDS):
            if abs(sensor_value) < ang:
                play_sound(sound)
                sleep(TEMPO)
                break

        

        # kill switch logic
        # US Sensor value will need to be under 10 for 2 seconds to enable kill_switch
        if US_sensor.get_value() <= 10:
            if not kill_switch_timer:
                kill_switch_timer = time()
            if time() - kill_switch_timer > 2:
                kill_switch_activate = True
                print("kill switch activated...")
        else:
            kill_switch_timer = None
except KeyboardInterrupt:
    print("Interrupted via keyboard...")
except Exception as err:
    print(err)
finally:
    reset_brick() # Turn off everything on the brick's hardware, and reset it
    exit()
