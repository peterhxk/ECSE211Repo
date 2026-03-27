import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from time import sleep
from controllers.sound_controller import SoundController

sc = SoundController()

print("Playing mission complete...")
sc.play_mission_complete()
sleep(7.0)  # wait for sequence to finish (~6.0s total)

print("Playing success...")
sc.play_success()
sleep(1.0)  # wait for sequence to finish (~0.37s total)

print("Done.")
