import sys
from time import sleep, time
import math

"""
Stable continuous-ish pitch WITHOUT streaming libs (no numpy, no sounddevice).

Uses a bank of prebuilt tones and switches between them.
This avoids simpleaudio/ALSA mmap crashes caused by starting new playbacks too fast.

Tune:
- N_STEPS: more steps = smoother pitch but more memory
- UPDATE_HZ: how often we check encoder and maybe switch
- POS_MAX: encoder span mapped to the pitch range
"""

sys.path.append("../")
from utils.brick import Motor, reset_brick, wait_ready_sensors, TouchSensor
from utils import sound

# -------------------- Hardware --------------------
motor = Motor("B")
button = TouchSensor("2")

# -------------------- Motor calibration params --------------------
POWER = 10
STALL_WINDOW = 0.35
STALL_DELTA = 3
PAUSE_AT_END = 0.2

# -------------------- Pitch mapping --------------------
ENCODER_SIGN = -1

FREQ_MIN = 220.0     # A3
FREQ_MAX = 880.0     # A5

NOTE_DST = 15
POS_MIN = 0
POS_MAX = 7 * NOTE_DST   # tune to your travel

REQUIRE_BUTTON = False   # set True to gate with button

# -------------------- Quantized “continuous” sound --------------------
N_STEPS = 24             # 12/24/36/48; 24 is a good start
VOLUME = 80
TONE_DUR = 0.25          # seconds; longer reduces retriggers
UPDATE_HZ = 20           # checks/sec (NOT play/sec)
UPDATE_PERIOD = 1.0 / UPDATE_HZ

# Only switch tone if bin changes (prevents rapid restarting)
# Optional "deadband" to reduce chatter near boundaries:
DEADBAND_BINS = 0        # set to 1 if it chatters

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

def pos_to_bin(pos):
    pos = clamp(pos, POS_MIN, POS_MAX)
    t = (pos - POS_MIN) / (POS_MAX - POS_MIN + 1e-9)  # 0..1
    b = int(t * (N_STEPS - 1) + 0.5)  # nearest bin
    return clamp(b, 0, N_STEPS - 1)

def bin_to_freq(b):
    t = b / (N_STEPS - 1 + 1e-9)
    return FREQ_MIN * ((FREQ_MAX / FREQ_MIN) ** t)

# ---- Build tone bank ONCE (important!) ----
print("Building tone bank...")
TONE_BANK = []
for i in range(N_STEPS):
    freq = bin_to_freq(i)
    # Your Sound() previously accepted numeric pitch (we tried that),
    # so we keep it. If it doesn't, you must change Sound() to accept Hz.
    TONE_BANK.append(sound.Sound(duration=TONE_DUR, pitch=freq, volume=VOLUME))
print("Tone bank ready.")

wait_ready_sensors(True)

# 1) Calibrate
direction = +1
motor_command(direction)
while True:
    if is_stalling():
        motor_stop()
        sleep(PAUSE_AT_END)
        motor.reset_encoder()
        print("MOTOR STALLED: encoder reset to 0")
        break

print("Entering stable quantized pitch loop")

current_bin = None
current_sound = None
last_check = 0.0

try:
    while True:
        now = time()
        if (now - last_check) < UPDATE_PERIOD:
            sleep(0.002)
            continue
        last_check = now

        pos = ENCODER_SIGN * motor.get_position()
        gate = True if not REQUIRE_BUTTON else button.is_pressed()

        if not gate:
            if current_sound is not None:
                try:
                    current_sound.stop()
                except Exception:
                    pass
            current_sound = None
            current_bin = None
            continue

        b = pos_to_bin(pos)

        # deadband to prevent chatter: only accept changes beyond +/- DEADBAND_BINS
        if current_bin is not None and abs(b - current_bin) <= DEADBAND_BINS:
            continue

        if b != current_bin:
            current_bin = b

            # stop previous tone before starting new
            if current_sound is not None:
                try:
                    current_sound.stop()
                except Exception:
                    pass

            current_sound = TONE_BANK[b]
            try:
                current_sound.play()   # non-blocking
            except Exception as e:
                print("Audio error:", e)
                current_sound = None
                current_bin = None
                sleep(0.2)

except KeyboardInterrupt:
    print("\nStopping.")
    try:
        if current_sound is not None:
            current_sound.stop()
    except Exception:
        pass
    motor_stop()
    reset_brick()
