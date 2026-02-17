#!/usr/bin/env python3

"""
This test is used to collect data from the ultrasonic sensor.
It must be run on the robot.
"""

from utils import sound
from utils.brick import TouchSensor, EV3UltrasonicSensor, wait_ready_sensors, reset_brick
from time import sleep


DELAY_SEC = 0.01  # seconds of delay between measurements

NOTES = ["A4", "B4", "C5", "D5", "E5", "F5", "G5"]   # The notes to be played 
SOUNDS = [sound.Sound(duration=0.3, pitch=note, volume=80) for note in NOTES]   # An array of Sounds corresponding to the musical notes
NOTE_DST = 8        # The physical distance threshold between each note
TEMPO = 0.5                        # The tempo of the music (sec/note)

def play_sound(note):
    "Play a single note."
    note.play()
    note.wait_done()

print("Program start.\nWaiting for sensors to turn on...")

US_SENSOR = EV3UltrasonicSensor(2)
KILL_SWITCH = TouchSensor(3)

wait_ready_sensors(True) # Input True to see what the robot is trying to initialize! False to be silent.
print("Done waiting.")


def collect_us():
    "Collect continuous data from the ultrasonic sensor and play the corresponding note."
    try:
        while not KILL_SWITCH.is_pressed():
            us_data = US_SENSOR.get_value()  # Float value in centimeters 0, capped to 255 cm
            if us_data is not None: # If None is given, then data collection failed that time
                print(us_data)
                
                for i in range(len(NOTES)):
                    if us_data < NOTE_DST * (i+1):
                        # play note i
                        print(f"note {i}")
                        play_sound(SOUNDS[i])
                        break;
                else:
                    # do nothing
                    print("no note")
                        
                
            sleep(TEMPO)
    except BaseException as error:  # capture all exceptions including KeyboardInterrupt (Ctrl-C)
        print("There was an error: ", error)
    finally:
        print("Done collecting US distance samples")
        reset_brick() # Turn off everything on the brick's hardware, and reset it
        exit()

if __name__ == "__main__":
    collect_us()
