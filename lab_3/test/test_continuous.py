import sys

import os, sys
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))
import time

sys.path.append("../../")
from utils import sound

NOTES = ["A4", "B4", "C5", "D5", "E5", "F5", "G5"]
SOUNDS = [sound.Sound(duration=2, pitch=note, volume=80) for note in NOTES]

PERIOD = 1.0  # seconds between note starts

def sleep_until(t_deadline, spin=0.002):
    """
    Sleep until an absolute perf_counter() deadline.
    spin: busy-wait in last 'spin' seconds for tighter timing.
    """
    while True:
        now = time.perf_counter()
        remaining = t_deadline - now
        if remaining <= 0:
            return
        if remaining > spin:
            time.sleep(remaining - spin)
        else:
            # busy-wait the last couple ms
            pass

t0 = time.perf_counter()
for i, s in enumerate(SOUNDS):
    t_i = t0 + i * PERIOD
    sleep_until(t_i)
    s.play()

# optional: keep process alive long enough to hear last sound
sleep_until(t0 + len(SOUNDS) * PERIOD + 2.0)

