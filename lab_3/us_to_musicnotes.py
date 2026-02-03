#!/usr/bin/env python3

"""
This test is used to collect data from the ultrasonic sensor.
It must be run on the robot.
"""

from utils import sound
from utils.brick import TouchSensor, EV3UltrasonicSensor, wait_ready_sensors, reset_brick
from time import sleep


DELAY_SEC = 0.01  # seconds of delay between measurements
SOUND1 = sound.Sound(duration=0.3, pitch="A4", volume=60)
SOUND2 = sound.Sound(duration=0.3, pitch="B4", volume=60)
SOUND3 = sound.Sound(duration=0.3, pitch="C4", volume=60)
SOUND4 = sound.Sound(duration=0.3, pitch="D4", volume=60)

def play_sound(note):
    "Play a single note."
    note.play()
    note.wait_done()

NOTE1DISTANCE = 10
NOTE2DISTANCE = 20
NOTE3DISTANCE = 30
NOTE4DISTANCE = 40
# How often flute plays notes
MUSICTEMPO = 2 

print("Program start.\nWaiting for sensors to turn on...")

TOUCH_SENSOR = TouchSensor(1)
US_SENSOR = EV3UltrasonicSensor(2)


wait_ready_sensors(True) # Input True to see what the robot is trying to initialize! False to be silent.
print("Done waiting.")


def collect_us():
    "Collect continuous data from the ultrasonic sensor between two button presses."
    try:
        while not TOUCH_SENSOR.is_pressed():
            pass  # do nothing while waiting for first button press
        print("Touch sensor pressed")
        sleep(1)
        print("Starting to collect US distance samples")
        while not TOUCH_SENSOR.is_pressed():
            us_data = US_SENSOR.get_value()  # Float value in centimeters 0, capped to 255 cm
            if us_data is not None: # If None is given, then data collection failed that time
                print(us_data)
                if (us_data < NOTE1DISTANCE):
                    #play note 1
                    print("note 1")
                    play_sound(SOUND1)

                elif (us_data < NOTE2DISTANCE):
                    #play note 2
                    print("note 2")
                    play_sound(SOUND2)
                elif (us_data < NOTE3DISTANCE):
                    #play note 3
                    print("note 3")
                    play_sound(SOUND3)
                elif (us_data < NOTE4DISTANCE):
                    #play note 4
                    print("note 4")
                    play_sound(SOUND4)
                else:
                    #do nothing
                    print("no note")
            sleep(MUSICTEMPO)
    except BaseException:  # capture all exceptions including KeyboardInterrupt (Ctrl-C)
        pass
    finally:
        print("Done collecting US distance samples")
        reset_brick() # Turn off everything on the brick's hardware, and reset it
        exit()

if __name__ == "__main__":
    collect_us()
