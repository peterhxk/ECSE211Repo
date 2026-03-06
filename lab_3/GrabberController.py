import time
import math
from enum import Enum
from collections import deque

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
        self.loading_button = TouchSensor(port=2)
        self.unloading_button = TouchSensor(port=3)
        self.motor = Motor("B")

        # Gaussian parameters for idle gyro DPS (will be calibrated)
        self.mu = 0.0
        self.sigma = 1.0

        # Detection / timing knobs
        self.sample_hz = 50
        self.consecutive_needed = 15  # require N consecutive samples beyond threshold
        
        self.running_power = 45

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

    def _consecutive_trigger(self, predicate, needed=None, sample_hz=None, timeout=None, on_sample=None) -> bool:
        """
        Returns True once `predicate()` is True for `needed` consecutive samples.
        Optional timeout (seconds): return False if not triggered in time.
        Optional on_sample(now_t): called each sample tick (useful for velocity-drop monitoring).
        """
        needed = self.consecutive_needed if needed is None else int(needed)
        sample_hz = self.sample_hz if sample_hz is None else float(sample_hz)
        dt = 1.0 / sample_hz

        hits = 0
        start = time.time()
        while hits < needed:
            now = time.time()

            if timeout is not None and (now - start) >= timeout:
                return False

            if on_sample is not None:
                try:
                    on_sample(now)
                except Exception as e:
                    print(f"[drop-monitor] error: {e}")

            if predicate():
                hits += 1
            else:
                hits = 0

            time.sleep(dt)

        return True

    # ----------------------------
    # High-level actions
    # ----------------------------

    from collections import deque

    def _motor_speed_dps(self):
        """
        Read motor speed in dps (or an equivalent 'velocity' unit).
        Adjust this if your Motor API differs.
        """
        v = self.motor.get_dps()  # <-- change here if needed
        if v is None:
            return None
        try:
            return float(v)
        except Exception:
            return None

    def _velocity_drop_monitor_init(
            self,
            window_size=500,
            k=2.0,
            ignore_first_n=1000,
            start_moving_dps=30.0,
            startup_grace_s=0.30,
    ):
        """
        Initializes state for rolling drop detection.
        - ignore_first_n: ignore first N samples after motor command (your request)
        - start_moving_dps + startup_grace_s: ignore 'starts from rest' transient
        """
        return {
            "window_size": int(window_size),
            "k": float(k),
            "ignore_first_n": int(ignore_first_n),
            "start_moving_dps": float(start_moving_dps),
            "startup_grace_s": float(startup_grace_s),
            "buf": deque(maxlen=int(window_size)),
            "n_total": 0,
            "moving_seen": False,
            "moving_since": None,
            "last_alert_t": 0.0,
            "drop_dps": 60.0,  # tune: try 50–100
            "drop_needed": 3,  # require 3 consecutive samples
            "drop_hits": 0,
        }

    def _velocity_drop_check_simple(self, mon_state, now_t):
        v = self._motor_speed_dps()
        mon_state["n_total"] += 1
        if v is None:
            return False, None

        # ignore first N samples
        if mon_state["n_total"] <= mon_state["ignore_first_n"]:
            mon_state["buf"].append(v)
            return False, None

        # ignore "start from rest" moment
        if not mon_state["moving_seen"]:
            if abs(v) >= mon_state["start_moving_dps"]:
                mon_state["moving_seen"] = True
                mon_state["moving_since"] = now_t
            mon_state["buf"].append(v)
            return False, None

        if (now_t - mon_state["moving_since"]) < mon_state["startup_grace_s"]:
            mon_state["buf"].append(v)
            return False, None

        buf = mon_state["buf"]
        if len(buf) < mon_state["window_size"]:
            buf.append(v)
            return False, None

        # baseline from PREVIOUS samples (don’t include current v)
        mean = sum(buf) / len(buf)

        # simple absolute drop threshold (tune this)
        DROP_DPS = mon_state["drop_dps"]  # e.g. 40–80

        is_drop = (v < (mean - DROP_DPS))

        # consecutive requirement
        if is_drop:
            mon_state["drop_hits"] += 1
        else:
            mon_state["drop_hits"] = 0

        buf.append(v)

        info = {"speed": v, "mean": mean, "thr": mean - DROP_DPS, "hits": mon_state["drop_hits"]}
        return (mon_state["drop_hits"] >= mon_state["drop_needed"]), info

    def load_bloc(self) -> bool:
        if self.bloc_count >= 2:
            print("Already loaded 2 blocs, aborting.")
            return False

        # ---- Drop detection config (your tuned values) ----
        WINDOW_SIZE = 25
        K = 1
        IGNORE_FIRST = 5  # ignore first 1000 samples (your request)

        # Also ignore "start from rest" moment:
        START_MOVING_DPS = 30.0  # tune if needed
        STARTUP_GRACE_S = 0.30  # tune if needed
        ALERT_COOLDOWN_S = 0.50  # avoid spamming prints

        mon = self._velocity_drop_monitor_init(
            window_size=WINDOW_SIZE,
            k=K,
            ignore_first_n=IGNORE_FIRST,
            start_moving_dps=START_MOVING_DPS,
            startup_grace_s=STARTUP_GRACE_S,
        )

        def monitor_tick(now_t):
            drop, info = self._velocity_drop_check_simple(mon, now_t)
            if info and mon["n_total"] % 25 == 0:
                print(
                    f"[debug] v={info['speed']:.1f} mean={info['mean']:.1f} thr={info['thr']:.1f} hits={info['hits']}")

            if drop and (now_t - mon["last_alert_t"]) >= ALERT_COOLDOWN_S:
                mon["last_alert_t"] = now_t
                print(f"⚠️ DROP: v={info['speed']:.1f} < {info['thr']:.1f} (mean={info['mean']:.1f})")
            v = self._motor_speed_dps()
            if mon["n_total"] % 25 == 0:
                print(
                    f"[debug] n={mon['n_total']} motor_v={v} buf_len={len(mon['buf'])} moving_seen={mon['moving_seen']}")

        self.motor.set_power(self.running_power)
        start = time.time()

        try:
            if self.bloc_count == 0:
                dps = self.gyro.get_dps_measure()
                print(f"dps={dps},mu={self.mu},sigma={self.sigma},threshold={self.mu - 3 * self.sigma}")

                ok = self._consecutive_trigger(
                    lambda: self.is_tilted_inwards(GyroTilt.SLOW_TILT),
                    needed=3,
                    timeout=5.0,
                    on_sample=monitor_tick,  # <-- run drop detection while waiting
                )
                if not ok:
                    print("Timeout waiting for first bloc to arrive.")
                    return False

                time.sleep(0.3)
                self.motor.set_power(0)
                self.bloc_positions[0] = BlocLocation.UNDER_GYRO
                self.bloc_count = 1
                return True

            # bloc_count == 1
            if self.bloc_positions[0] == BlocLocation.UNLOADED:
                raise ValueError("First bloc not yet unloaded, aborting.")

            ok = self._consecutive_trigger(
                lambda: self.is_tilted_outwards(GyroTilt.FAST_TILT),
                needed=2,
                timeout=3.0,
                on_sample=monitor_tick,  # <-- monitor while waiting
            )
            time.sleep(0.3)

            if not ok:
                raise RuntimeError("Gyro did not detect first bloc leaving the position.")
            self.bloc_positions[0] = BlocLocation.BACK_STORAGE

            ok = self._consecutive_trigger(
                lambda: self.is_tilted_inwards(GyroTilt.SLOW_TILT),
                needed=3,
                timeout=5.0,
                on_sample=monitor_tick,  # <-- monitor while waiting
            )
            if not ok:
                raise RuntimeError("Timed out detecting bloc being loaded. Check connections and thresholds.")

            self.motor.set_power(0)
            self.bloc_positions[1] = BlocLocation.UNDER_GYRO
            self.bloc_count = 2
            print("Successfully loaded second bloc.")
            return True

        finally:
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
                self.motor.set_power(-1.6*self.running_power) # increase power because blocs may not be fully aligned adding extra stress 
                ready = self._consecutive_trigger(
                    lambda: self.is_tilted_outwards(GyroTilt.SLOW_TILT),
                    needed=3,
                    timeout=3.0,
                )
                if not ready:
                    print("Timeout waiting for initial outwards SLOW tilt (ready).")
                    return False
                
                time.sleep(0.3) # rebound
                
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
            self.motor.set_power(-1.5*self.running_power)
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
                #print(self.gyro.get_dps_measure())
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