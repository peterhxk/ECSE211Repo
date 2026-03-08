import time
import math
from enum import Enum

# Headless-safe plotting on EV3
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from utils.brick import EV3GyroSensor, TouchSensor, Motor


class BlocLocation(Enum):
    BACK_STORAGE = 1
    UNDER_GYRO = 2
    UNLOADED = 3


class GyroTilt(Enum):
    SLOW_TILT = 1
    FAST_TILT = 2


class GrabberController:
    """
    Responsibilities:
      1) Run the conveyor belt inwards / outwards (via motor power)
      2) Detect events with the gyroscope (DPS thresholds based on calibrated mu/sigma)
      3) Update internal state (how many blocs are loaded and where they are)
      4) Load / unload blocs (load implemented; unload can be added similarly)

    NEW:
      - Logs motor speed while motor is running during load/unload actions
      - Saves a PNG plot automatically when motor stops
    """

    def __init__(self):
        # Internal state
        self.bloc_count = 0
        self.bloc_positions = [BlocLocation.UNLOADED, BlocLocation.UNLOADED]

        # Hardware
        self.gyro = EV3GyroSensor(port=1, mode=EV3GyroSensor.Mode.DPS)
        self.loading_button = TouchSensor(port=2)
        self.unloading_button = TouchSensor(port=3)
        self.motor = Motor("B")

        # Gaussian parameters for idle gyro DPS (will be calibrated)
        self.mu = 0.0
        self.sigma = 1.0

        # Detection / timing knobs
        self.sample_hz = 50
        self.consecutive_needed = 15
        self.running_power = 45

    # ----------------------------
    # Setup / calibration
    # ----------------------------
    def initialize_sensors(self):
        print("Initializing sensors... Put the gyro in vertical (rest) position.")
        self.gyro.wait_ready()
        self.loading_button.wait_ready()
        self.unloading_button.wait_ready()

        self.gyro.reset_measure()
        self.motor.set_dps(0)
        print("Reset measures; motor stopped.")

        self.calibrate_gyro_noise(seconds=15.0, sample_hz=self.sample_hz, warmup=0.5)
        print("Calibrated mu and sigma.")

    def calibrate_gyro_noise(self, seconds=10.0, sample_hz=50, warmup=0.5):
        dt = 1.0 / float(sample_hz)

        # Warmup / discard
        t0 = time.time()
        while time.time() - t0 < warmup:
            _ = self.gyro.get_dps_measure()
            time.sleep(dt)

        # Collect samples
        samples = []
        t1 = time.time()
        while time.time() - t1 < seconds:
            v = self.gyro.get_dps_measure()
            if v is not None:
                samples.append(float(v))
            time.sleep(dt)

        if len(samples) < 10:
            raise RuntimeError(f"Not enough gyro samples collected: {len(samples)}")

        mu = sum(samples) / len(samples)
        var = sum((x - mu) ** 2 for x in samples) / (len(samples) - 1)  # ddof=1
        sigma = math.sqrt(var)
        sigma = max(sigma, 1e-6)

        self.mu = mu
        self.sigma = sigma
        print(f"[Gyro calibration] n={len(samples)} mu={mu:.3f} dps, sigma={sigma:.3f} dps")
        return mu, sigma

    # ----------------------------
    # Gyro event detection
    # ----------------------------
    def _k_for_tilt(self, tilt: GyroTilt) -> float:
        return 8.0 if tilt == GyroTilt.SLOW_TILT else 12.0

    def _floor_for_inwards(self, tilt: GyroTilt) -> float:
        return -10.0 if tilt == GyroTilt.SLOW_TILT else -18.0

    def _floor_for_outwards(self, tilt: GyroTilt) -> float:
        return 10.0 if tilt == GyroTilt.SLOW_TILT else 18.0

    def is_tilted_inwards(self, tilt: GyroTilt) -> bool:
        dps = self.gyro.get_dps_measure()
        if dps is None:
            return False
        dps = float(dps)
        k = self._k_for_tilt(tilt)
        thr = self.mu - k * self.sigma
        return dps < min(thr, self._floor_for_inwards(tilt))

    def is_tilted_outwards(self, tilt: GyroTilt) -> bool:
        dps = self.gyro.get_dps_measure()
        if dps is None:
            return False
        dps = float(dps)
        k = self._k_for_tilt(tilt)
        thr = self.mu + k * self.sigma
        return dps > max(thr, self._floor_for_outwards(tilt))

    # ----------------------------
    # Motor logging helpers
    # ----------------------------
    def _motor_speed_dps(self):
        """
        Read motor speed in dps.
        Adjust this if your Motor API differs.
        """
        v = self.motor.get_dps()  # <-- if your API name differs, change here
        if v is None:
            return None
        try:
            return float(v)
        except Exception:
            return None

    def _run_motor_with_logging(self, power, run_fn, label="run"):
        """
        Turns motor on, runs `run_fn(log_sample, dt)` while collecting speed samples,
        then turns motor off and saves a PNG plot.
        """
        speeds = []
        times = []
        t0 = time.time()
        dt = 1.0 / float(self.sample_hz)

        def log_sample():
            v = self._motor_speed_dps()
            if v is not None:
                speeds.append(v)
                times.append(time.time() - t0)

        self.motor.set_power(power)
        try:
            result = run_fn(log_sample, dt)
            return result
        finally:
            self.motor.set_power(0)

            # Save plot
            if len(speeds) >= 3:
                ts = time.strftime("%Y%m%d_%H%M%S")
                fname = f"{label}_{ts}.png"

                plt.figure()
                plt.plot(times, speeds, label="Speed (dps)")
                plt.xlabel("Time (s)")
                plt.ylabel("Motor speed (dps)")
                plt.title(f"Motor speed trace: {label}")
                plt.grid(True)
                plt.legend()
                plt.tight_layout()
                plt.savefig(fname, dpi=150)
                plt.close()

                print(f"[log] Saved motor speed plot: {fname}")
            else:
                print("[log] Not enough speed samples to save plot.")

    # ----------------------------
    # High-level actions
    # ----------------------------
    def load_bloc(self) -> bool:
        """
        Runs motor inwards and uses gyro events to load blocs.
        Logs motor speed while motor is running and saves a PNG when motor stops.
        """
        if self.bloc_count >= 2:
            print("Already loaded 2 blocs, aborting.")
            return False

        # ignore the first fraction of a second (startup from rest)
        STARTUP_IGNORE_S = 0.30

        def do_load(log_sample, dt):
            t_ignore_end = time.time() + STARTUP_IGNORE_S

            def wait_consecutive(predicate, needed, timeout_s):
                hits = 0
                t_end = time.time() + timeout_s
                while time.time() < t_end and hits < needed:
                    log_sample()

                    # ignore startup transient
                    if time.time() < t_ignore_end:
                        time.sleep(dt)
                        continue

                    if predicate():
                        hits += 1
                    else:
                        hits = 0

                    time.sleep(dt)
                return hits >= needed

            if self.bloc_count == 0:
                # Wait for first bloc to arrive (inwards slow)
                ok = wait_consecutive(
                    lambda: self.is_tilted_inwards(GyroTilt.SLOW_TILT),
                    needed=3,
                    timeout_s=5.0,
                )
                if not ok:
                    print("Timeout waiting for first bloc to arrive.")
                    return False

                time.sleep(0.3)
                self.bloc_positions[0] = BlocLocation.UNDER_GYRO
                self.bloc_count = 1
                return True

            # bloc_count == 1
            if self.bloc_positions[0] == BlocLocation.UNLOADED:
                raise ValueError("First bloc not yet unloaded, aborting.")

            # First bloc leaving (outwards fast)
            ok = wait_consecutive(
                lambda: self.is_tilted_outwards(GyroTilt.FAST_TILT),
                needed=2,
                timeout_s=3.0,
            )
            time.sleep(0.3)  # refractory

            if not ok:
                raise RuntimeError("Gyro did not detect first bloc leaving the position.")
            self.bloc_positions[0] = BlocLocation.BACK_STORAGE

            # Second bloc arriving (inwards slow)
            ok = wait_consecutive(
                lambda: self.is_tilted_inwards(GyroTilt.SLOW_TILT),
                needed=3,
                timeout_s=5.0,
            )
            if not ok:
                raise RuntimeError("Timed out detecting bloc being loaded. Check connections and thresholds.")

            self.bloc_positions[1] = BlocLocation.UNDER_GYRO
            self.bloc_count = 2
            print("Successfully loaded second bloc.")
            return True

        label = "load_first" if self.bloc_count == 0 else "load_second"
        try:
            return bool(self._run_motor_with_logging(self.running_power, do_load, label=label))
        finally:
            self.motor.set_power(0)

    def unload_bloc(self) -> bool:
        """
        Unloads blocs (your previous logic) and logs motor speed while motor is running.
        Saves a PNG when motor stops.
        """
        if self.bloc_count <= 0:
            print("No blocs to unload.")
            return False

        STARTUP_IGNORE_S = 0.20

        def do_unload(log_sample, dt):
            t_ignore_end = time.time() + STARTUP_IGNORE_S

            def wait_consecutive(predicate, needed, timeout_s):
                hits = 0
                t_end = time.time() + timeout_s
                while time.time() < t_end and hits < needed:
                    log_sample()

                    if time.time() < t_ignore_end:
                        time.sleep(dt)
                        continue

                    if predicate():
                        hits += 1
                    else:
                        hits = 0

                    time.sleep(dt)
                return hits >= needed

            if self.bloc_count == 2:
                # Run motor outwards (negative power in your code) and wait for outwards slow tilt
                ready = wait_consecutive(
                    lambda: self.is_tilted_outwards(GyroTilt.SLOW_TILT),
                    needed=3,
                    timeout_s=3.0,
                )
                if not ready:
                    print("Timeout waiting for initial outwards SLOW tilt (ready).")
                    return False

                time.sleep(0.3)

                unloaded = wait_consecutive(
                    lambda: self.is_tilted_outwards(GyroTilt.SLOW_TILT),
                    needed=3,
                    timeout_s=5.0,
                )
                if not unloaded:
                    print("Timeout unloading (no outwards SLOW tilt detected).")
                    return False

                # Update state
                if self.bloc_positions[0] != BlocLocation.UNLOADED:
                    self.bloc_positions[0] = BlocLocation.UNLOADED
                elif self.bloc_positions[1] != BlocLocation.UNLOADED:
                    self.bloc_positions[1] = BlocLocation.UNLOADED

                self.bloc_count = max(0, self.bloc_count - 1)
                print("Unloaded one bloc (2->1).")
                return True

            # Exactly 1 bloc remaining: run outwards for ~2 seconds
            t_end = time.time() + 2.0
            while time.time() < t_end:
                log_sample()
                time.sleep(dt)

            self.bloc_positions = [BlocLocation.UNLOADED, BlocLocation.UNLOADED]
            self.bloc_count = 0
            print("Unloaded last bloc (1->0).")
            return True

        # In your previous code, unload used negative power
        power = -1.6 * self.running_power if self.bloc_count == 2 else -1.5 * self.running_power
        label = "unload_two" if self.bloc_count == 2 else "unload_one"
        try:
            return bool(self._run_motor_with_logging(power, do_unload, label=label))
        finally:
            self.motor.set_power(0)

    # ----------------------------
    # Main loop
    # ----------------------------
    def loop(self):
        """
        Main loop of the grabber controller.
        Press button to attempt to load/unload a bloc.
        """
        # optional: eject all blocs on boot
        self.motor.set_power(-self.running_power)
        time.sleep(1)
        self.motor.set_power(0)
        print("Ejected all blocs")

        try:
            while True:
                if self.loading_button.get_value():
                    print("Loading Button pressed, attempting to load bloc...")
                    success = self.load_bloc()
                    if success:
                        print(f"Successfully loaded bloc. Count={self.bloc_count}, positions={self.bloc_positions}")
                    else:
                        print("Failed to load bloc.")
                    time.sleep(0.2)  # debounce

                elif self.unloading_button.get_value():
                    print("Unloading Button pressed, attempting to unload bloc...")
                    success = self.unload_bloc()
                    if success:
                        print(f"Successfully unloaded bloc. Count={self.bloc_count}, positions={self.bloc_positions}")
                    else:
                        print("Failed to unload bloc.")
                    time.sleep(0.2)  # debounce

                time.sleep(0.01)

        except KeyboardInterrupt:
            print("Stopping grabber controller loop.")
            self.motor.set_power(0)


if __name__ == "__main__":
    controller = GrabberController()
    controller.initialize_sensors()
    controller.loop()