import time
import math
from enum import Enum

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
    """

    def __init__(self):
        # Internal state
        self.bloc_count = 0
        self.bloc_positions = [BlocLocation.UNLOADED, BlocLocation.UNLOADED]

        # Hardware
        self.gyro = EV3GyroSensor(port=1, mode=EV3GyroSensor.Mode.DPS)
        self.loading_button = TouchSensor(port=3)
        self.unloading_button = TouchSensor(port=4)
        self.motor = Motor("A")

        # Gaussian parameters for idle gyro DPS (will be calibrated)
        self.mu = 0.0
        self.sigma = 1.0

        # Detection / timing knobs
        self.sample_hz = 50
        self.consecutive_needed = 15  # require N consecutive samples beyond threshold

        self.running_power = 60
        if self.running_power > 70:
            print("Warning: running power too high. Pick a lower value")

    # ----------------------------
    # Setup / calibration
    # ----------------------------
    def initialize_sensors(self):
        print("Initializing sensors... Put the gyro in vertical (rest) position.")
        self.gyro.wait_ready()
        self.loading_button.wait_ready()

        self.gyro.reset_measure()
        self.motor.set_dps(0)
        print("Reset measures; motor stopped.")

        self.calibrate_gyro_noise(seconds=15.0, sample_hz=self.sample_hz, warmup=0.5)
        print("Calibrated mu and sigma.")

    def calibrate_gyro_noise(self, seconds=10.0, sample_hz=50, warmup=0.5):
        """
        Estimate mu and sigma for idle gyro DPS.

        - warmup: discard early readings (can be unstable right after reset)
        - seconds: collection time after warmup
        - sample_hz: sampling frequency for calibration
        """
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
        var = sum((x - mu) ** 2 for x in samples) / (len(samples) - 1)  # sample variance (ddof=1)
        sigma = math.sqrt(var)

        # Avoid sigma=0 causing thresholds to collapse
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
        # negative DPS = inwards
        return -10.0 if tilt == GyroTilt.SLOW_TILT else -18.0  # tune

    def _floor_for_outwards(self, tilt: GyroTilt) -> float:
        # positive DPS = outwards
        return 10.0 if tilt == GyroTilt.SLOW_TILT else 18.0  # tune

    def is_tilted_inwards(self, tilt: GyroTilt) -> bool:
        dps = self.gyro.get_dps_measure()
        if dps is None:
            return False
        dps = float(dps)
        k = self._k_for_tilt(tilt)
        thr = self.mu - k * self.sigma
        # must be below BOTH the gaussian threshold and a hard negative floor
        return dps < min(thr, self._floor_for_inwards(tilt))

    def is_tilted_outwards(self, tilt: GyroTilt) -> bool:
        dps = self.gyro.get_dps_measure()
        if dps is None:
            return False
        dps = float(dps)
        k = self._k_for_tilt(tilt)
        thr = self.mu + k * self.sigma
        # must be above BOTH the gaussian threshold and a hard positive floor
        return dps > max(thr, self._floor_for_outwards(tilt))

    def _consecutive_trigger(self, predicate, needed=None, sample_hz=None, timeout=None) -> bool:
        """
        Returns True once `predicate()` is True for `needed` consecutive samples.
        Optional timeout (seconds): return False if not triggered in time.
        """
        needed = self.consecutive_needed if needed is None else int(needed)
        sample_hz = self.sample_hz if sample_hz is None else float(sample_hz)
        dt = 1.0 / sample_hz

        hits = 0
        start = time.time()
        while hits < needed:
            if timeout is not None and (time.time() - start) >= timeout:
                return False

            if predicate():
                hits += 1
            else:
                hits = 0

            time.sleep(dt)

        return True

    # ----------------------------
    # High-level actions
    # ----------------------------
    def load_bloc(self) -> bool:
        """
        Returns True on success, False on failure (timeout or already loaded 2 blocs).

        Assumptions (based on your original flow):
        - Loading the first bloc: motor runs until an "inwards" tilt is detected.
        - Loading the second bloc: first detect first bloc leaving (outwards FAST),
          then detect second bloc arriving (inwards SLOW).
        """
        if self.bloc_count >= 2:
            print("Already loaded 2 blocs, aborting.")
            return False

        self.motor.set_power(self.running_power)
        start = time.time()

        try:
            if self.bloc_count == 0:
                # Wait for first bloc to arrive under gyro (inwards event)

                # debug only : measure DPS before doing anything then compute threshold measurements
                dps = self.gyro.get_dps_measure()
                print(f"dps={dps},mu={self.mu},sigma={self.sigma},threshold={self.mu - 3 * self.sigma}")

                ok = self._consecutive_trigger(
                    lambda: self.is_tilted_inwards(GyroTilt.SLOW_TILT),
                    needed=3,
                    timeout=5.0,
                )
                if not ok:
                    print("Timeout waiting for first bloc to arrive.")
                    return False

                self.motor.set_power(0)
                self.bloc_positions[0] = BlocLocation.UNDER_GYRO
                self.bloc_count = 1
                return True

            # bloc_count == 1
            if self.bloc_positions[0] == BlocLocation.UNLOADED:
                raise ValueError("First bloc not yet unloaded, aborting.")

            # First bloc leaving position (outwards fast) - short timeout
            ok = self._consecutive_trigger(
                lambda: self.is_tilted_outwards(GyroTilt.FAST_TILT),
                needed=2,
                timeout=3.0,
            )
            # REFRACTORY PERIOD: ignore ring-down oscillation
            time.sleep(0.3)

            if not ok:
                raise RuntimeError("Gyro did not detect first bloc leaving the position.")
            self.bloc_positions[0] = BlocLocation.BACK_STORAGE  # the first block was successfully moved

            # Second bloc arriving (inwards slow) - longer timeout
            ok = self._consecutive_trigger(
                lambda: self.is_tilted_inwards(GyroTilt.SLOW_TILT),
                needed=3,
                timeout=5.0,
            )
            if not ok:
                raise RuntimeError(
                    "Timed out detecting bloc being loaded. Check connections and thresholds."
                )

            self.motor.set_power(0)
            self.bloc_positions[1] = BlocLocation.UNDER_GYRO
            self.bloc_count = 2
            print("Successfully loaded second bloc.")
            return True

        finally:
            # Safety stop if something goes wrong
            # (don’t stop if success already stopped motor, but stopping twice is fine)
            if time.time() - start > 0:
                self.motor.set_power(0)

    def unload_bloc(self) -> bool:
        """
        Unload logic (as requested):

        If 2 blocs loaded:
          1) Wait for SLOW outwards tilt (gyro back to rest / ready)
          2) Run motor outwards until SLOW outwards tilt, then stop motor
          3) Update internal state (one bloc unloaded)

        If 1 bloc remaining:
          - Run motor outwards for ~5 seconds, then stop
          - Update internal state (all unloaded)

        Returns True on success, False on timeout / invalid state.
        """

        # Sanity
        if self.bloc_count <= 0:
            print("No blocs to unload.")
            return False

        # NOTE: Your original "tilt outwards" detector is a one-sided threshold:
        #   dps > mu + k*sigma
        # so we use it as an "event" gate.

        try:
            if self.bloc_count == 2:
                # 1) Wait until system is "ready" / gyro returns to an outwards-slow event
                # (Using a timeout so we don't block forever.)
                self.motor.set_power(
                    -1.6 * self.running_power)  # increase power because blocs may not be fully aligned adding extra stress
                ready = self._consecutive_trigger(
                    lambda: self.is_tilted_outwards(GyroTilt.SLOW_TILT),
                    needed=3,
                    timeout=3.0,
                )
                if not ready:
                    print("Timeout waiting for initial outwards SLOW tilt (ready).")
                    return False

                time.sleep(0.3)  # rebound

                # 2) Run motor outwards until we detect outwards SLOW tilt again, then stop.
                unloaded = self._consecutive_trigger(
                    lambda: self.is_tilted_outwards(GyroTilt.SLOW_TILT),
                    needed=3,
                    timeout=5.0,
                )
                self.motor.set_power(0)

                if not unloaded:
                    print("Timeout unloading (no outwards SLOW tilt detected).")
                    return False

                # 3) Update internal state: unload the front-most/active bloc
                # We'll unload slot 0 first, then slot 1.
                if self.bloc_positions[0] != BlocLocation.UNLOADED:
                    self.bloc_positions[0] = BlocLocation.UNLOADED
                elif self.bloc_positions[1] != BlocLocation.UNLOADED:
                    self.bloc_positions[1] = BlocLocation.UNLOADED

                self.bloc_count = max(0, self.bloc_count - 1)
                print("Unloaded one bloc (2->1).")
                return True

            # If exactly 1 bloc remaining:
            self.motor.set_power(-1.5 * self.running_power)
            time.sleep(2.0)
            self.motor.set_power(0)

            # Update internal state: mark everything unloaded
            self.bloc_positions = [BlocLocation.UNLOADED, BlocLocation.UNLOADED]
            self.bloc_count = 0
            print("Unloaded last bloc (1->0).")
            return True

        finally:
            # Safety stop
            self.motor.set_power(0)

    # ----------------------------
    # Main loop
    # ----------------------------
    def loop(self):
        """
        Main loop of the grabber controller.
        Press button to attempt to load a bloc.
        # NEGATIVE DPS : inward tilt
        # POSITIVE DPS : outwards tilt
        """
        self.motor.set_power(-self.running_power)
        time.sleep(1)
        self.motor.set_power(0)
        print("Ejected all blocs")
        try:
            while True:
                # print(self.gyro.get_dps_measure())
                if self.loading_button.get_value():
                    print("Loading Button pressed, attempting to load bloc...")
                    success = self.load_bloc()
                    if success:
                        print(
                            f"Successfully loaded bloc. Count={self.bloc_count}, "
                            f"positions={self.bloc_positions}"
                        )
                    else:
                        print("Failed to load bloc.")
                    # crude debounce
                    time.sleep(0.2)
                elif self.unloading_button.get_value():
                    print("Unloading Button pressed, attempting to unload bloc...")
                    success = self.unload_bloc()
                    if success:
                        print(
                            f"Successfully unloaded bloc. Count={self.bloc_count}, "
                            f"positions={self.bloc_positions}"
                        )
                    else:
                        print("Failed to unload bloc.")
                    # crude debounce
                    time.sleep(0.2)

                time.sleep(0.01)

        except KeyboardInterrupt:
            print("Stopping grabber controller loop.")
            self.motor.set_power(0)


if __name__ == "__main__":
    controller = GrabberController()
    controller.initialize_sensors()
    controller.loop()