import sys
from time import sleep, time
import math
import threading

import numpy as np
import sounddevice as sd

sys.path.append("../")
from utils.brick import Motor, reset_brick, wait_ready_sensors, TouchSensor

# -------------------- Hardware --------------------
motor = Motor("B")
button = TouchSensor("2")

# -------------------- Motor calibration --------------------
POWER = 10
STALL_WINDOW = 0.35
STALL_DELTA = 3
PAUSE_AT_END = 0.2

# -------------------- Pitch mapping --------------------
ENCODER_SIGN = -1

FREQ_MIN = 220.0   # A3
FREQ_MAX = 880.0   # A5

NOTE_DST = 15
POS_MIN = 0
POS_MAX = 7 * NOTE_DST  # tune this to match your travel

REQUIRE_BUTTON = False  # set True if you want gating

# -------------------- Audio stream params --------------------
FS = 44100
VOLUME = 0.2  # 0.0..1.0

# Shared state for audio callback
state_lock = threading.Lock()
target_freq = 440.0
enabled = True
phase = 0.0

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
    pos = clamp(pos, POS_MIN, POS_MAX)
    t = (pos - POS_MIN) / (POS_MAX - POS_MIN + 1e-9)
    return FREQ_MIN * ((FREQ_MAX / FREQ_MIN) ** t)

def audio_callback(outdata, frames, time_info, status):
    global phase
    if status:
        # You can print(status) if debugging
        pass

    with state_lock:
        freq = target_freq
        on = enabled

    if not on:
        outdata[:] = 0
        return

    # Generate sine wave chunk with continuous phase
    t = (np.arange(frames) / FS).astype(np.float32)
    omega = 2.0 * math.pi * float(freq)
    chunk = np.sin(omega * t + phase).astype(np.float32)

    # Update phase so waveform is continuous across callbacks
    phase = (phase + omega * (frames / FS)) % (2.0 * math.pi)

    outdata[:] = (VOLUME * chunk).reshape(-1, 1)

def main():
    global target_freq, enabled

    wait_ready_sensors(True)

    # Calibrate: run until stall -> reset encoder to 0
    motor_command(+1)
    while True:
        if is_stalling():
            motor_stop()
            sleep(PAUSE_AT_END)
            motor.reset_encoder()
            print("MOTOR STALLED: encoder reset to 0")
            break

    print("Starting audio stream (continuous pitch)")

    with sd.OutputStream(
        samplerate=FS,
        channels=1,
        dtype="float32",
        callback=audio_callback,
        blocksize=0,   # let sounddevice choose
    ):
        try:
            while True:
                pos = ENCODER_SIGN * motor.get_position()
                gate = True if not REQUIRE_BUTTON else button.is_pressed()
                freq = pos_to_freq(pos)

                with state_lock:
                    target_freq = freq
                    enabled = gate

                sleep(0.01)

        except KeyboardInterrupt:
            print("\nStopping.")
        finally:
            motor_stop()
            reset_brick()

if __name__ == "__main__":
    main()
