import os
import sys
import math
import time

# --- Make sure we import *your* project utils/, not the pip "utils" package ---
# If this file is in ECSE211Repo/lab_3/, ".." points to ECSE211Repo/
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from utils.brick import Motor, reset_brick, wait_ready_sensors, TouchSensor
from utils import sound


"""
Continuous pitch (no discrete note mapping) using deterministic timers.

Key ideas:
- Use perf_counter() + sleep_until() for deterministic update timing.
- Map motor encoder position -> frequency (Hz) continuously (exponential mapping).
- Avoid overlapping playbacks by reusing ONE Sound and updating its pitch.
  (Still depends on how utils/sound.py is implemented; this is the "best effort"
  approach within that API.)
"""

# -------------------- Hardware --------------------
motor = Motor("B")
button = TouchSensor("2")

# -------------------- Motor calibration params --------------------
POWER = 10
STALL_WINDOW = 0.35
STALL_DELTA = 3
PAUSE_AT_END = 0.2

# -------------------- Timing params --------------------
UPDATE_HZ = 20.0                  # how often to update pitch (Hz)
PERIOD = 1.0 / UPDATE_HZ
SPIN = 0.002                      # busy-wait in last 2ms for tighter timing

# -------------------- Continuous pitch mapping --------------------
ENCODER_SIGN = -1                 # matches your earlier convention

# Choose a pitch range (Hz). Example: 2 octaves A3 -> A5
FREQ_MIN = 220.0                  # A3
FREQ_MAX = 880.0                  # A5

# Choose encoder span that maps to that range (tune this!)
# If your travel is larger/smaller, adjust POS_MAX.
NOTE_DST = 15
POS_MIN = 0
POS_MAX = 7 * NOTE_DST            # ~your earlier 7-step span, but now continuous

# Gate sound with button?
REQUIRE_BUTTON = False            # set True to require button press

VOLUME = 80


# -------------------- Helpers --------------------
def motor_command(direction: int):
    motor.set_power(direction * POWER)

def motor_stop():
    motor.set_power(0)

def is_stalling(window_s=STALL_WINDOW, delta_ticks=STALL_DELTA):
    p0 = motor.get_position()
    t0 = time.time()
    while time.time() - t0 < window_s:
        time.sleep(0.02)
    p1 = motor.get_position()
    return abs(p1 - p0) <= delta_ticks

def clamp(x, lo, hi):
    return lo if x < lo else hi if x > hi else x

def pos_to_freq(pos):
    """
    Smooth exponential mapping (musical):
      f = fmin * (fmax/fmin)^t , where t in [0,1]
    """
    pos = clamp(pos, POS_MIN, POS_MAX)
    t = (pos - POS_MIN) / (POS_MAX - POS_MIN + 1e-9)
    return FREQ_MIN * ((FREQ_MAX / FREQ_MIN) ** t)

def sleep_until(t_deadline, spin=SPIN):
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
        # else: busy-wait


# -------------------- Main --------------------
wait_ready_sensors(True)

# Calibrate: run until stall, then set encoder to 0
direction = +1
motor_command(direction)
while True:
    if is_stalling():
        motor_stop()
        time.sleep(PAUSE_AT_END)
        motor.reset_encoder()
        print("MOTOR STALLED: encoder reset to 0")
        break

print("Entering continuous pitch loop (deterministic timing)")

# Reuse ONE Sound and update its pitch continuously
# duration should be "long enough" so it keeps sounding while we update pitch
carrier = sound.Sound(duration=60, pitch=FREQ_MIN, volume=VOLUME)

playing = False
last_freq = None

t0 = time.perf_counter()
tick = 0

try:
    while True:
        # deterministic tick schedule
        t_tick = t0 + tick * PERIOD
        sleep_until(t_tick)
        tick += 1

        pos = ENCODER_SIGN * motor.get_position()
        gate = True if not REQUIRE_BUTTON else button.is_pressed()

        if not gate:
            if playing:
                try:
                    carrier.stop()
                except Exception:
                    pass
                playing = False
                last_freq = None
            continue

        # compute continuous frequency
        freq = pos_to_freq(pos)

        # start if not already playing
        if not playing:
            carrier.set_pitch(freq)
            carrier.update_audio()
            carrier.play()
            playing = True
            last_freq = freq
            continue

        # update pitch (only if changed enough to matter)
        if last_freq is None or abs(freq - last_freq) >= 1.0:
            carrier.set_pitch(freq)
            carrier.update_audio()
            last_freq = freq

except KeyboardInterrupt:
    print("\nStopping.")
    try:
        carrier.stop()
    except Exception:
        pass
    motor_stop()
    reset_brick()
