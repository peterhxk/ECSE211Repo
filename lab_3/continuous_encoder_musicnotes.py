import sys
from time import sleep, time
import math

"""
Continuous-pitch version (no note presets).

How it works:
- Calibrates one hard-stop to define encoder zero (same as your original).
- Maps current motor position continuously to a frequency (Hz) over a chosen range.
- Plays very short tone "chunks" repeatedly (non-blocking) so the pitch feels continuous.
IMPORTANT:
- This assumes your utils.sound.Sound can accept pitch as a FREQUENCY in Hz (float/int).
  If your Sound() only accepts note names like "A4", you’ll need the Hz-capable tone API.
"""

sys.path.append("../")
from utils.brick import Motor, reset_brick, wait_ready_sensors, TouchSensor
from utils import sound

# -------------------- Hardware --------------------
motor = Motor("B")
button = TouchSensor("2")  # set to your port; you can keep the "always on" mode below

# -------------------- Motor calibration params --------------------
POWER = 10              # adjust motor power
STALL_WINDOW = 0.35     # seconds to observe for stall
STALL_DELTA = 3         # encoder ticks considered "no movement"
PAUSE_AT_END = 0.2      # pause before resetting encoder

# -------------------- Continuous sound params --------------------
VOLUME = 80
CHUNK_SEC = 0.03        # 20–60ms works well; smaller = smoother but more CPU
LOOP_SLEEP = 0.005      # loop sleep

# Map encoder position -> frequency.
# Choose a musical range (example: A3=220 Hz to A5=880 Hz, 2 octaves)
FREQ_MIN = 220.0
FREQ_MAX = 880.0

# Choose the encoder span to cover that frequency range.
# You can tune this. If you know your physical travel, set POS_MAX accordingly.
# Here we reuse your idea of 7 steps * 15 ticks = ~105 ticks span.
NOTE_DST = 15
POS_MIN = 0
POS_MAX = 7 * NOTE_DST

# If your encoder goes negative in the direction you use, keep this.
ENCODER_SIGN = -1  # multiply motor.get_position() by this

# If you want to require button press to produce sound, set to True.
REQUIRE_BUTTON = False


# -------------------- Helpers --------------------
def motor_command(direction: int):
    motor.set_power(direction * POWER)

def motor_stop():
    motor.set_power(0)

def is_stalling(window_s=STALL_WINDOW, delta_ticks=STALL_DELTA):
    p0 = motor.get_position()
    t0 = time()
    while time() - t0 < window_s:
        sleep(0.02)
    p1 = motor.get_position()
    return abs(p1 - p0) <= delta_ticks

def clamp(x, lo, hi):
    return lo if x < lo else hi if x > hi else x

def pos_to_freq(pos):
    """
    Map encoder position -> frequency (Hz), smoothly.
    Uses exponential mapping so equal distances feel like equal pitch steps (musical).
    """
    pos = clamp(pos, POS_MIN, POS_MAX)
    t = (pos - POS_MIN) / (POS_MAX - POS_MIN + 1e-9)  # 0..1
    return FREQ_MIN * ((FREQ_MAX / FREQ_MIN) ** t)


# -------------------- Main --------------------
wait_ready_sensors(True)

# 1) Calibrate: run until stall, then set encoder to 0
direction = +1
motor_command(direction)

while True:
    if is_stalling():
        motor_stop()
        sleep(PAUSE_AT_END)
        motor.reset_encoder()
        print("MOTOR STALLED: encoder reset to 0")
        break

print("Entering continuous tone loop")

last_play_t = 0.0

try:
    while True:
        # Read encoder, apply sign if needed
        pos = ENCODER_SIGN * motor.get_position()

        # Check button gate
        gate = True
        if REQUIRE_BUTTON:
            gate = button.is_pressed()

        if gate:
            freq = pos_to_freq(pos)

            # Play short chunks repeatedly (non-blocking)
            now = time()
            if (now - last_play_t) >= CHUNK_SEC:
                last_play_t = now

                # Assumes pitch can be Hz. If not, see note below.
                s = sound.Sound(duration=CHUNK_SEC, pitch=freq, volume=VOLUME)
                s.play()  # DO NOT call wait_done()

        sleep(LOOP_SLEEP)

except KeyboardInterrupt:
    print("\nStopping.")
    motor_stop()
    reset_brick()
