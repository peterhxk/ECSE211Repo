import sys
from time import sleep, time

"""
Continuous pitch from motor encoder without presets.

Key fixes vs your previous attempt:
- Only ONE sound plays at a time (stop previous before starting new)
- Rate-limit pitch updates (UPDATE_HZ)
- Only update when pitch changes enough (FREQ_EPS_HZ)
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

# -------------------- Continuous sound params --------------------
VOLUME = 80

UPDATE_HZ = 15.0                 # how often we allow pitch updates (10–20 is usually safe)
UPDATE_PERIOD = 1.0 / UPDATE_HZ
TONE_DURATION = 0.15             # play slightly longer than update period to avoid gaps
FREQ_EPS_HZ = 3.0                # don't restart audio unless freq changed by at least this much

# Pitch range
FREQ_MIN = 220.0                 # A3
FREQ_MAX = 880.0                 # A5

# Encoder mapping range
NOTE_DST = 15
POS_MIN = 0
POS_MAX = 7 * NOTE_DST           # tune this to match your travel

ENCODER_SIGN = -1                # keep your sign convention

# Gate with button?
REQUIRE_BUTTON = False           # set True to require button press


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
    Smooth, musical mapping (exponential):
      f = fmin * (fmax/fmin)^t, t in [0,1]
    """
    pos = clamp(pos, POS_MIN, POS_MAX)
    t = (pos - POS_MIN) / (POS_MAX - POS_MIN + 1e-9)
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

# We keep track of the currently playing sound (and stop it before starting a new one)
current_sound = None
current_freq = None
last_update_t = 0.0

try:
    while True:
        pos = ENCODER_SIGN * motor.get_position()

        gate = True
        if REQUIRE_BUTTON:
            gate = button.is_pressed()

        now = time()

        if gate:
            # rate limit updates
            if (now - last_update_t) >= UPDATE_PERIOD:
                last_update_t = now
                freq = pos_to_freq(pos)

                # only restart audio if frequency changed enough
                if (current_freq is None) or (abs(freq - current_freq) >= FREQ_EPS_HZ):
                    current_freq = freq

                    # STOP previous playback to avoid ALSA mmap exhaustion
                    try:
                        if current_sound is not None:
                            current_sound.stop()   # utils.sound.Sound has stop() in your stack trace context
                    except Exception:
                        # if stop() isn't available or fails, ignore and continue
                        pass

                    # Create and play new tone
                    try:
                        current_sound = sound.Sound(duration=TONE_DURATION, pitch=freq, volume=VOLUME)
                        current_sound.play()  # DO NOT wait_done()
                    except Exception as e:
                        # If audio device glitches, back off briefly so ALSA can recover
                        print("Audio error:", e)
                        sleep(0.2)
                        current_sound = None
                        current_freq = None

        else:
            # if button not pressed, stop sound once
            if current_sound is not None:
                try:
                    current_sound.stop()
                except Exception:
                    pass
                current_sound = None
                current_freq = None

        sleep(0.005)

except KeyboardInterrupt:
    print("\nStopping.")
    try:
        if current_sound is not None:
            current_sound.stop()
    except Exception:
        pass
    motor_stop()
    reset_brick()
