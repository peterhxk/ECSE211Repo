#!/usr/bin/env python3
import math
import statistics
import threading
from enum import Enum
from collections import deque
import sys
from tqdm import tqdm
sys.path.append("../utils")
from brick import EV3GyroSensor, Motor
import os
import glob
import time
import pickle
from datetime import datetime
from collections import deque


class BlocLocation(Enum):
    BACK_STORAGE = 1      # in the back storage area
    UNDER_GYRO = 2        # under the gyro arm
    AT_GATE = 3           # closest to the end of the conveyor belts
    UNLOADED = 4          # not yet loaded, or already unloaded


class GyroTilt(Enum):
    SLOW_TILT = 1
    FAST_TILT = 2


class GrabberController:
    """
    Responsibilities:
      1) Run the conveyor belt inwards / outwards
      2) Detect events with the gyroscope using calibrated mu/sigma
      3) Detect second-bloc loading by motor velocity drop
      4) Update internal state
      5) Record gyro traces for empirical calibration
    """

    def __init__(self, motor_port="A", gyro_port=3, calibration=True, debug=True):
        # Internal state
        self.bloc_count = 0
        self.bloc_positions = [BlocLocation.UNLOADED, BlocLocation.UNLOADED]
        self.calibration = calibration
        self.debug = debug

        # Thread-safe state / async-like control
        self.state_lock = threading.Lock()

        self._block_caught_event = threading.Event()

        self._load_thread = None
        self._load_done_event = threading.Event()
        self._load_cancel_event = threading.Event()
        self._load_result = None

        # Hardware
        self.gyro = EV3GyroSensor(port=gyro_port, mode=EV3GyroSensor.Mode.DPS)
        self.motor = Motor(motor_port)

        # Idle gyro calibration
        self.mu = 0.0
        self.sigma = 1.0

        # Motor inward speed calibration
        self.motor_mu_in = 0.0
        self.motor_sigma_in = 1.0

        # Gyro detection knobs
        self.sample_hz = 50
        self.consecutive_needed = 15
        self.running_dps = 250

        # Motor-drop detection knobs
        self.motor_calibration_dps = 250
        self.motor_sample_sleep = 0.02
        self.motor_eject_time = 2.0

        self.motor_dry_run_time = 5.0
        self.motor_drop_k = 4.0
        self.motor_drop_consecutive_needed = 5
        self.use_motor_moving_average = True
        self.motor_moving_avg_window = 5
        self.motor_drop_warm_up_delay = 0.5

        if self.running_dps > 400:
            print("Warning: running power too high. Pick a lower value")

        # Empirical gyro trace recording
        self.recordings_dir = "gyro_operation_recordings"
        os.makedirs(self.recordings_dir, exist_ok=True)

        self._current_record = None
        self._recording_lock = threading.Lock()

        self._command_queue = deque()
        self._input_thread = threading.Thread(target=self._input_worker, daemon=True)
        self._input_thread.start()

    # ----------------------------
    # Setup / calibration
    # ----------------------------
    def _load_bloc_worker(self):
        try:
            self._load_result = self.load_bloc(stop_event=self._load_cancel_event)
        finally:
            self._load_done_event.set()

    def start_loading(self) -> bool:
        """
        Start load_bloc() in a background thread and return immediately.

        Returns False if a loading operation is already in progress, True otherwise.
        """
        if self._load_thread is not None and self._load_thread.is_alive():
            print("[INFO] Loading already in progress.")
            return False

        self._load_done_event.clear()
        self._load_cancel_event.clear()
        self._block_caught_event.clear()
        self._load_result = None

        self._load_thread = threading.Thread(target=self._load_bloc_worker, daemon=True)
        self._load_thread.start()
        return True

    def is_loading(self) -> bool:
        return self._load_thread is not None and self._load_thread.is_alive()

    def loading_finished(self) -> bool:
        return self._load_done_event.is_set()

    def get_loading_result(self):
        return self._load_result

    def cancel_loading(self):
        self._load_cancel_event.set()

    def initialize_sensors(self):
        print("[Calibration] Initializing sensors... Don't move the Gyro sensor")
        self.gyro.wait_ready()
        self.gyro.reset_measure()
        self.motor.set_dps(0)
        print("Reset measures; motor stopped.")

        self.calibrate_motor_drop_detection(calibration=self.calibration)
        if self.calibration:
            print("Calibrated motor inward speed-drop mu and sigma.")
        else:
            print("Skipped motor calibration")

        self.calibrate_gyro_noise(
            seconds=15.0,
            sample_hz=self.sample_hz,
            warmup=0.5,
            calibration=self.calibration,
        )
        if self.calibration:
            print("Calibrated gyroscope")
        else:
            print("Skipped gyro calibration")

    def calibrate_gyro_noise(self, seconds=10.0, sample_hz=50, warmup=0.5, calibration=True):
        """
        If calibration=True:
            - Collect gyro samples
            - Compute mu/sigma
            - Save to gyro_calibrations/*.pkl

        If calibration=False:
            - Load most recent calibration
        """
        calibration_dir = getattr(self, "gyro_calibration_dir", "gyro_calibrations")
        os.makedirs(calibration_dir, exist_ok=True)

        if calibration:
            print(f"[Gyro calibration] Calibrating... Don't move sensor ({seconds}s)")
            dt = 1.0 / float(sample_hz)

            t0 = time.time()
            while time.time() - t0 < warmup:
                _ = self.gyro.get_dps_measure()
                time.sleep(dt)

            samples = []
            n_samples = int(seconds * sample_hz)

            for _ in tqdm(range(n_samples), desc="[Gyro calibration]", unit="sample"):
                v = self.gyro.get_dps_measure()
                if v is not None:
                    samples.append(float(v))
                time.sleep(dt)

            if len(samples) < 10:
                raise RuntimeError(f"Not enough gyro samples collected: {len(samples)}")

            mu = sum(samples) / len(samples)
            var = sum((x - mu) ** 2 for x in samples) / (len(samples) - 1)
            sigma = math.sqrt(var)
            sigma = max(sigma, 1e-6)

            self.mu = mu
            self.sigma = sigma

            calibration_data = {
                "date": datetime.now().isoformat(),
                "mu": self.mu,
                "sigma": self.sigma,
                "seconds": seconds,
                "sample_hz": sample_hz,
            }

            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            calibration_file = os.path.join(
                calibration_dir, f"gyro_calibration_{timestamp}.pkl"
            )

            with open(calibration_file, "wb") as f:
                pickle.dump(calibration_data, f)

            if self.debug:
                print(f"[Gyro calibration] Saved to {calibration_file}")

        else:
            calibration_files = sorted(
                glob.glob(os.path.join(calibration_dir, "gyro_calibration_*.pkl"))
            )

            if not calibration_files:
                raise FileNotFoundError(
                    f"No gyro calibration file found in '{calibration_dir}'."
                )

            latest_file = calibration_files[-1]

            with open(latest_file, "rb") as f:
                calibration_data = pickle.load(f)

            self.mu = calibration_data["mu"]
            self.sigma = max(calibration_data["sigma"], 1e-6)

            if self.debug:
                print(f"[Gyro calibration] Loaded latest calibration from {latest_file}")
                print(f"[Gyro calibration] Calibration date: {calibration_data['date']}")

        print(f"[Gyro calibration] mu={self.mu:.3f}, sigma={self.sigma:.3f}")
        return self.mu, self.sigma

    def _sample_motor_speeds(self, dps, duration, sleep_time):
        """
        Run motor at constant dps for duration seconds and collect raw signed speed samples.
        """
        samples = []
        self.motor.set_dps(dps)
        start = time.time()

        try:
            while time.time() - start < duration:
                speed = self.motor.get_speed()
                if speed is not None:
                    samples.append(float(speed))
                time.sleep(sleep_time)
        finally:
            self.motor.set_dps(0)

        return samples

    def calibrate_motor_drop_detection(self, calibration=True):
        """
        If calibration=True:
            1) Eject outward
            2) Dry run both ways at power=45
            3) Keep inward mu/sigma for drop detection
            4) Save calibration data

        If calibration=False:
            - Load the latest saved calibration
        """
        calibration_dir = getattr(self, "motor_calibration_dir", "motor_calibrations")
        os.makedirs(calibration_dir, exist_ok=True)

        if calibration:
            if not self.debug:
                raise RuntimeError("Calibration requested but debug mode is disabled.")

            print("[Motor calibration] Ejecting outward first...")
            self.motor.set_dps(-self.motor_calibration_dps)
            time.sleep(self.motor_eject_time)
            self.motor.set_dps(0)
            time.sleep(0.5)

            print("[Motor calibration] Dry run outward...")
            outward_samples = self._sample_motor_speeds(
                dps=-self.motor_calibration_dps,
                duration=self.motor_dry_run_time,
                sleep_time=self.motor_sample_sleep,
            )
            if len(outward_samples) < 2:
                raise RuntimeError("Not enough outward motor samples collected.")

            mu_out = statistics.mean(outward_samples)
            sigma_out = max(statistics.stdev(outward_samples), 1e-6)

            print(
                f"[Motor calibration] outward n={len(outward_samples)} "
                f"mu={mu_out:.3f}, sigma={sigma_out:.3f}"
            )

            time.sleep(0.5)

            print("[Motor calibration] Dry run inward...")
            inward_samples = self._sample_motor_speeds(
                dps=self.motor_calibration_dps,
                duration=self.motor_dry_run_time,
                sleep_time=self.motor_sample_sleep,
            )
            if len(inward_samples) < 2:
                raise RuntimeError("Not enough inward motor samples collected.")

            self.motor_mu_in = statistics.mean(inward_samples)
            self.motor_sigma_in = max(statistics.stdev(inward_samples), 1e-6)
            inward_n = len(inward_samples)

            calibration_data = {
                "date": datetime.now().isoformat(),
                "motor_mu_in": self.motor_mu_in,
                "motor_sigma_in": self.motor_sigma_in,
                "motor_calibration_dps": self.motor_calibration_dps,
                "motor_dry_run_time": self.motor_dry_run_time,
                "motor_sample_sleep": self.motor_sample_sleep,
                "motor_drop_k": self.motor_drop_k,
            }

            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            calibration_file = os.path.join(
                calibration_dir, f"motor_calibration_{timestamp}.pkl"
            )

            with open(calibration_file, "wb") as f:
                pickle.dump(calibration_data, f)

            if self.debug:
                print(f"[Motor calibration] Saved calibration to {calibration_file}")

        else:
            calibration_files = sorted(
                glob.glob(os.path.join(calibration_dir, "motor_calibration_*.pkl"))
            )
            if not calibration_files:
                raise FileNotFoundError(
                    f"No calibration file found in '{calibration_dir}'."
                )

            latest_file = calibration_files[-1]
            with open(latest_file, "rb") as f:
                calibration_data = pickle.load(f)

            self.motor_mu_in = calibration_data["motor_mu_in"]
            self.motor_sigma_in = max(calibration_data["motor_sigma_in"], 1e-6)
            inward_n = "loaded"

            if self.debug:
                print(f"[Motor calibration] Loaded latest calibration from {latest_file}")
                print(f"[Motor calibration] Calibration date: {calibration_data['date']}")

        threshold = self.motor_mu_in - self.motor_drop_k * self.motor_sigma_in
        print(
            f"[Motor calibration] inward n={inward_n} "
            f"mu={self.motor_mu_in:.3f}, sigma={self.motor_sigma_in:.3f}, "
            f"threshold={threshold:.3f}"
        )

        return threshold

    # ----------------------------
    # Gyro thresholds / predicates
    # ----------------------------
    def _k_for_tilt(self, tilt: GyroTilt) -> float:
        return 8.0 if tilt == GyroTilt.SLOW_TILT else 12.0

    def _floor_for_inwards(self, tilt: GyroTilt) -> float:
        return -12.0 if tilt == GyroTilt.SLOW_TILT else -8.0

    def _floor_for_outwards(self, tilt: GyroTilt) -> float:
        return 6.0 if tilt == GyroTilt.SLOW_TILT else 8.0

    def _gyro_threshold_inwards(self, tilt: GyroTilt) -> float:
        k = self._k_for_tilt(tilt)
        thr = self.mu - k * self.sigma
        return min(thr, self._floor_for_inwards(tilt))

    def _gyro_threshold_outwards(self, tilt: GyroTilt) -> float:
        k = self._k_for_tilt(tilt)
        thr = self.mu + k * self.sigma
        return max(thr, self._floor_for_outwards(tilt))

    def _is_tilted_inwards_sample(self, dps: float, tilt: GyroTilt) -> bool:
        return dps < self._gyro_threshold_inwards(tilt)

    def _is_tilted_outwards_sample(self, dps: float, tilt: GyroTilt) -> bool:
        return dps > self._gyro_threshold_outwards(tilt)

    # ----------------------------
    # Gyro recording helpers
    # ----------------------------
    def _begin_operation_recording(self, operation_name, timeout=None, extra=None):
        if self._current_record is not None:
            raise RuntimeError("A gyro operation record is already active.")

        self._current_record = {
            "operation": operation_name,
            "start_datetime": datetime.now().isoformat(),
            "end_datetime": None,
            "mu": self.mu,
            "sigma": self.sigma,
            "sample_hz": self.sample_hz,
            "timeout": timeout,
            "extra": extra or {},
            "samples": [],
            "outcome": None,
        }

    def _append_gyro_sample_to_record(
        self,
        elapsed_s: float,
        dps: float,
        z: float,
        detectors=None,
    ):
        if self._current_record is None:
            return

        sample = {
            "t": elapsed_s,
            "dps": dps,
            "z": z,
        }

        if detectors is not None:
            sample["detectors"] = detectors

        with self._recording_lock:
            self._current_record["samples"].append(sample)

    def _end_operation_recording(self, outcome="completed"):
        if self._current_record is None:
            return

        with self._recording_lock:
            self._current_record["outcome"] = outcome
            self._current_record["end_datetime"] = datetime.now().isoformat()

            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            op = self._current_record["operation"]
            filename = os.path.join(self.recordings_dir, f"{op}_{timestamp}.pkl")

            with open(filename, "wb") as f:
                pickle.dump(self._current_record, f)

            if self.debug:
                print(f"[Gyro recording] Saved operation trace to {filename}")

            self._current_record = None

    # ----------------------------
    # Unified gyro stream:
    # detector + recorder use same samples
    # ----------------------------
    def _consecutive_gyro_trigger(
            self,
            operation_name: str,
            detector_name: str,
            predicate_from_dps,
            needed=None,
            sample_hz=None,
            timeout=None,
            extra=None,
            stop_event=None,
    ) -> bool:
        """
        Read the gyro ONCE per loop.
        Use the same sample for:
          1) detection
          2) recording

        Returns True when predicate_from_dps(dps) is true for `needed` consecutive samples.
        """

        needed = self.consecutive_needed if needed is None else int(needed)
        sample_hz = self.sample_hz if sample_hz is None else float(sample_hz)
        dt = 1.0 / sample_hz

        self._begin_operation_recording(
            operation_name=operation_name,
            timeout=timeout,
            extra={
                **(extra or {}),
                "detector": detector_name,
                "needed": needed,
            },
        )

        hits = 0
        start = time.time()

        try:
            while hits < needed:

                if stop_event is not None and stop_event.is_set():
                    self._end_operation_recording(outcome="cancelled")
                    return False

                elapsed = time.time() - start
                if timeout is not None and elapsed >= timeout:
                    self._end_operation_recording(outcome="timeout")
                    return False

                raw = self.gyro.get_dps_measure()
                if raw is None:
                    time.sleep(dt)
                    continue

                dps = float(raw)
                z = (dps - self.mu) / max(self.sigma, 1e-6)

                detector_flags = {
                    "slow_in": self._is_tilted_inwards_sample(dps, GyroTilt.SLOW_TILT),
                    "fast_in": self._is_tilted_inwards_sample(dps, GyroTilt.FAST_TILT),
                    "slow_out": self._is_tilted_outwards_sample(dps, GyroTilt.SLOW_TILT),
                    "fast_out": self._is_tilted_outwards_sample(dps, GyroTilt.FAST_TILT),
                }

                cond = bool(predicate_from_dps(dps))

                detector_flags["active_detector"] = cond

                self._append_gyro_sample_to_record(
                    elapsed_s=elapsed,
                    dps=dps,
                    z=z,
                    detectors=detector_flags,
                )

                if cond:
                    hits += 1
                else:
                    hits = 0

                time.sleep(dt)

            self._end_operation_recording(outcome="success")
            return True

        except Exception:
            self._end_operation_recording(outcome="error")
            raise

    # ----------------------------
    # Motor drop detection
    # ----------------------------
    def increase_on_velocity_drop(self, duration=0.6):
        start = time.time()
        cooldown = time.time()
        timeout = 1.0

        while (time.time() - start) < timeout:
            # stop thread if loading finished
            if self.bloc_count != 0:
                return

            if (time.time() - cooldown) < duration:
                time.sleep(0.01)
                continue

            if self._is_motor_drop_detected(timeout=5.0,warmup_delay=0.1):
                print(f"[Motor drop detected] Temporarily increasing velocity to prevent blocking. Speed set to {self.running_dps * 1.5}")

                cooldown = time.time()

                boosted_power = self.running_dps * 1.5 #min(1.2 * self.motor.get_power(), self.running_dps * 1.3)
                self.motor.set_dps(boosted_power)

                time.sleep(duration)

                if self.bloc_count == 0:
                    self.motor.set_dps(self.running_dps)

    def _is_motor_drop_detected(self, timeout=5.0, warmup_delay=0.5, stop_event=None) -> bool:
        threshold = self.motor_mu_in - self.motor_drop_k * self.motor_sigma_in
        hits = 0
        start = time.time()
        ma_window = deque(maxlen=max(1, self.motor_moving_avg_window))

        print(
            f"[Drop detect] Waiting for inward speed drop: "
            f"threshold={threshold:.3f}, "
            f"k={self.motor_drop_k}, needed={self.motor_drop_consecutive_needed}"
        )

        while True:
            if stop_event is not None and stop_event.is_set():
                return False

            elapsed = time.time() - start
            if timeout is not None and elapsed >= timeout:
                return False

            raw_speed = self.motor.get_speed()
            if raw_speed is None:
                time.sleep(self.motor_sample_sleep)
                continue

            raw_speed = float(raw_speed)

            if elapsed < warmup_delay:
                time.sleep(self.motor_sample_sleep)
                continue

            if self.use_motor_moving_average and self.motor_moving_avg_window > 1:
                ma_window.append(raw_speed)
                speed_for_detection = sum(ma_window) / len(ma_window)
            else:
                speed_for_detection = raw_speed

            condition = speed_for_detection < threshold

            if condition:
                hits += 1
            else:
                hits = 0

            if hits >= self.motor_drop_consecutive_needed:
                print(
                    f"[Drop detect] Triggered: raw_speed={raw_speed:.3f}, "
                    f"filtered_speed={speed_for_detection:.3f}, "
                    f"threshold={threshold:.3f}"
                )
                self._block_caught_event.set()
                return True

            time.sleep(self.motor_sample_sleep)

    # ----------------------------
    # High-level actions
    # ----------------------------
    def load_bloc(self, stop_event=None) -> bool:
        """
        Returns True on success, False on failure.

        Logic:
        - First bloc: use gyro inward detection
        - Second bloc: use motor-speed drop detection
        """
        if self.bloc_count >= 2:
            print("Already loaded 2 blocs, aborting.")
            return False

        self.motor.set_dps(self.running_dps)

        try:
            if stop_event is not None and stop_event.is_set():
                print("[INFO] Loading cancelled before start.")
                return False

            # First bloc: gyro-based detection with unified sample stream
            if self.bloc_count == 0:
                #threading.Thread(target=self.increase_on_velocity_drop, daemon=True).start()
                #print("[INFO] Started velocity drop detection thread")

                ok = self._consecutive_gyro_trigger(
                    operation_name="load_first_bloc",
                    detector_name="gyro_inward_slow",
                    predicate_from_dps=lambda dps: self._is_tilted_inwards_sample(dps, GyroTilt.SLOW_TILT),
                    needed=2,
                    timeout=5.0,
                    extra={
                        "bloc_count_before": self.bloc_count,
                        "running_dps": self.running_dps,
                        "direction": "inward",
                    },
                    stop_event=stop_event,
                )

                if not ok:
                    print("Timeout or cancellation waiting for first bloc to arrive.")
                    return False

                if stop_event is not None and stop_event.is_set():
                    print("[INFO] Loading cancelled after detection.")
                    return False

                print("[INFO] First bloc gyro event triggered")
                self.motor.set_dps(0)
                with self.state_lock:
                    self.bloc_positions[0] = BlocLocation.UNDER_GYRO
                    self.bloc_count = 1

                print("Successfully loaded first bloc.")
                return True

            # Second bloc: motor-drop detection
            if self.bloc_positions[0] == BlocLocation.UNLOADED:
                print("Error: internal state inconsistency: bloc count is 1 but no bloc is loaded.")
                return False

            self.motor.set_dps(self.motor_calibration_dps)
            print("Loading second bloc: using motor velocity drop detection...")
            #time.sleep(self.motor_drop_warm_up_delay)

            if stop_event is not None and stop_event.is_set():
                print("[INFO] Loading cancelled during second-bloc warm-up.")
                return False

            ok = self._is_motor_drop_detected(timeout=5.0,warmup_delay=0.6,stop_event=stop_event)
            #TODO : problem is that if the first block is only after the gyro sensor it might temporarily jam the grabber thus triggering the velocity drop event
            if not ok:
                print("Timed out or cancelled waiting for motor velocity drop while loading second bloc.")
                return False

            with self.state_lock:
                self.bloc_positions[0] = BlocLocation.BACK_STORAGE
                self.bloc_positions[1] = BlocLocation.AT_GATE
                self.bloc_count = 2

            print("Successfully loaded second bloc.")
            return True

        finally:
            self.motor.set_dps(0)

    def unload_bloc(self) -> bool:
        """
        Unload logic:

        If 2 blocs loaded:
          - Run outward until slow outward gyro tilt is detected

        If 1 bloc remaining:
          - Use motor velocity drop detection
        """
        if self.bloc_count <= 0:
            print("No blocs to unload.")
            return False

        try:
            if self.bloc_count == 2:
                self.motor.set_dps(-1.2 * self.running_dps)

                unloaded = self._consecutive_gyro_trigger(
                    operation_name="unload_from_2_to_1",
                    detector_name="gyro_outward_slow",
                    predicate_from_dps=lambda dps: self._is_tilted_outwards_sample(dps, GyroTilt.SLOW_TILT),
                    needed=3,
                    timeout=5.0,
                    extra={
                        "bloc_count_before": self.bloc_count,
                        "running_dps": self.running_dps,
                        "direction": "outward",
                    },
                )

                self.motor.set_dps(0)

                if not unloaded:
                    print("Timeout unloading (no outward SLOW tilt detected).")
                    return False

                self.bloc_positions[0] = BlocLocation.AT_GATE
                self.bloc_positions[1] = BlocLocation.UNLOADED
                self.bloc_count = max(0, self.bloc_count - 1)

                print("Unloaded one bloc (2->1).")
                return True

            # only one bloc left
            self.motor.set_dps(-self.running_dps)
            print("Unloading last bloc: using motor velocity drop detection...")
            time.sleep(0.6)

            ok = self._is_motor_drop_detected(timeout=5.0,warmup_delay=0)
            if not ok:
                print("Timed out waiting for motor velocity drop while unloading last bloc.")
                return False

            self.motor.set_dps(-self.running_dps)
            time.sleep(0.1)
            self.motor.set_dps(0)

            self.bloc_positions = [BlocLocation.UNLOADED, BlocLocation.UNLOADED]
            self.bloc_count = 0

            print("Unloaded last bloc (1->0).")
            return True

        finally:
            self.motor.set_dps(0)

    # ----------------------------
    # Main loop
    # ----------------------------
    def loop(self):
        self.motor.set_dps(-self.running_dps)
        time.sleep(1)
        self.motor.set_dps(0)
        print("Ejected all blocs")

        try:
            while True:
                # -------------------------
                # 1. Handle user commands
                # -------------------------
                if self._command_queue:
                    cmd = self._command_queue.popleft()

                    if cmd == "l":
                        print("Starting non-blocking load...")
                        self.start_loading()

                    elif cmd == "u":
                        print("Unloading...")
                        self.unload_bloc()

                    elif cmd == "q":
                        print("Exiting...")
                        raise KeyboardInterrupt

                # -------------------------
                # 2. Periodic checks
                # -------------------------
                if self.is_loading():
                    if self.isBlockCaught():
                        print("[INFO] Block caught (velocity drop detected)")

                    if self.loading_finished():
                        print(f"[INFO] Loading finished: {self.get_loading_result()}")

                # -------------------------
                time.sleep(0.01)

        except KeyboardInterrupt:
            print("Stopping grabber controller loop.")
            self.cancel_loading()
            self.motor.set_dps(0)

    def kill(self):
        raise NotImplementedError()

    def isBlockCaught(self) -> bool:
        return self._block_caught_event.is_set()

    def clearBlockCaught(self):
        self._block_caught_event.clear()

    def _input_worker(self):
        while True:
            try:
                cmd = input("Enter command (l=load, u=unload, q=quit): ").lower()
                self._command_queue.append(cmd)
            except EOFError:
                break

    def get_loading_status(self) -> str:
        if self._load_thread is not None and self._load_thread.is_alive():
            if self._block_caught_event.is_set():

                return "BLOCK_CAUGHT"
            return "LOADING"

        if self._load_done_event.is_set():
            if self._load_result is True:
                return "SUCCESS"
            return "FAILED"

        return "IDLE"


if __name__ == "__main__":
    controller = GrabberController("D", 4, False , True)
    controller.initialize_sensors()
    controller.loop()
