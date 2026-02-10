import sys
from time import sleep, time
import math
import threading
import array

import sounddevice as sd

sys.path.append("../")
from utils.brick import Motor, reset_brick, wait_ready_sensors, TouchSensor

motor = Motor("B")
button = TouchSensor("2")

POWER = 10
STALL_WINDOW = 0.35
STALL_DELTA = 3
PAUSE_AT_END = 0.2

ENCODER_SIGN = -1

FREQ_MIN = 220.0   # A3
FREQ_MAX = 880.0   # A5

NOTE_DST = 15
POS_MIN = 0
POS_MAX = 7 * NOTE_DST

REQUIRE_BUTTON = False

FS = 44100
VOLUME = 0.2  # 0..1

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
        # print(status)  # uncomment if you want diagnostics
        pass

    with state_lock:
        freq = float(target_freq)
        on = bool(enabled)

    if not on:
        outdata[:] = 0
        return

    omega = 2.0 * math.pi * freq / FS

    # Build mono float32 samples without numpy
    buf = array.array('f', [0.0]) * frames
    ph = phase
    amp = VOLUME
    for i in range(frames):
        buf[i] = amp * math.sin(ph)
        ph += omega
        if ph > 2.0 * math.pi:
            ph -= 2.0 * math.pi

    phase = ph

    # Write into outdata (frames x channels)
    # outdata is a numpy-like buffer provided by sounddevice; assignment from list/array works.
    outdata[:, 0] = buf

def main():
    global target_freq, enabled

    wait_ready_sensors(True)

    motor_command(+1)
    while True:
        if is_stalling():
            motor_stop()
            sleep(PAUSE_AT_END)
            motor.reset_encoder()
            print("MOTOR STALLED: encoder reset to 0")
            break

    print("Starting audio stream (no numpy)")

    with sd.OutputStream(
        samplerate=FS,
        channels=1,
        dtype="float32",
        callback=audio_callback,
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
