import time
from utils.brick import Motor, reset_brick, wait_ready_sensors, TouchSensor
from utils import sound

# -------------------- Hardware --------------------
motor = Motor("B")
button = TouchSensor("2")

# -------------------- Motor calibration --------------------
POWER = 10
STALL_WINDOW = 0.35
STALL_DELTA = 3
PAUSE_AT_END = 0.2

# -------------------- Deterministic timing --------------------
UPDATE_HZ = 8.0                 # KEEP LOW to avoid simpleaudio/ALSA mmap errors
PERIOD = 1.0 / UPDATE_HZ
SPIN = 0.002

# -------------------- Continuous pitch mapping --------------------
ENCODER_SIGN = -1

FREQ_MIN = 220.0                # A3
FREQ_MAX = 880.0                # A5

POS_MIN = 0
POS_MAX = 7 * 15                # tune to your travel span (encoder ticks)

VOLUME = 80
CHUNK_DUR = 0.12                # chunk duration (>= PERIOD helps prevent gaps)

REQUIRE_BUTTON = False          # set True if you want button gating

def sleep_until(t_deadline, spin=SPIN):
    while True:
        now = time.perf_counter()
        remaining = t_deadline - now
        if remaining <= 0:
            return
        if remaining > spin:
            time.sleep(remaining - spin)
        # else busy-wait

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
    # exponential mapping
    pos = clamp(pos, POS_MIN, POS_MAX)
    t = (pos - POS_MIN) / (POS_MAX - POS_MIN + 1e-9)
    return FREQ_MIN * ((FREQ_MAX / FREQ_MIN) ** t)

# -------------------- Main --------------------
wait_ready_sensors(True)

# Calibrate
motor_command(+1)
while True:
    if is_stalling():
        motor_stop()
        time.sleep(PAUSE_AT_END)
        motor.reset_encoder()
        print("MOTOR STALLED: encoder reset to 0")
        break

print("Entering continuous-ish pitch loop")

tick = 0
t0 = time.perf_counter()

# Optional: start with a known audible tone once to confirm audio works in THIS script
test = sound.Sound(duration=0.2, pitch="C5", volume=VOLUME)
test.play()

try:
    while True:
        # deterministic schedule
        t_tick = t0 + tick * PERIOD
        sleep_until(t_tick)
        tick += 1

        pos = ENCODER_SIGN * motor.get_position()
        gate = True if not REQUIRE_BUTTON else button.is_pressed()

        if not gate:
            continue

        freq = pos_to_freq(pos)

        # Play short chunk at this freq
        # If your Sound() doesn't accept Hz, replace `freq` with a note string mapping.
        s = sound.Sound(duration=CHUNK_DUR, pitch=freq, volume=VOLUME)
        s.play()

except KeyboardInterrupt:
    print("\nStopping.")
    motor_stop()
    reset_brick()

