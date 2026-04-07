#!/usr/bin/env python3
import math
import statistics
import threading
from enum import Enum
from collections import deque
import sys
from tqdm import tqdm

sys.path.append("../utils")
from brick import EV3GyroSensor, Motor,reset_brick
import os
import glob
import time
import pickle
from datetime import datetime
from collections import deque


class BlocLocation(Enum):
    """
    An Enum object to describe the cube locations inside the Grabber subsystem
    BACK_STORAGE : When the cube is at the rear of the grabber
    UNDER_GYRO : When the cube has tilted the gyro inwards / outwards, it remains under the gyro
    AT_GATE : When the cube is just at the entrance of the grabber
    """
    BACK_STORAGE = 1      # in the back storage area
    UNDER_GYRO = 2        # under the gyro arm
    AT_GATE = 3           # closest to the end of the conveyor belts
    UNLOADED = 4          # not yet loaded, or already unloaded


class GyroTilt(Enum):
    """
    An Enum object to describe the gyro tilt thresholds used in k-sigma normalization
    The following values are used as our ks for comparing the dps of the gyro against mu - k * sigma
    """
    SLOW_TILT = 1
    FAST_TILT = 2
    FAST_TILT_KILL = 3


class GrabberController:
    """
    Responsibilities:
      1) Run the conveyor belt inwards / outwards
      2) Detect events with the gyroscope using calibrated mu/sigma
      3) Detect second-cube loading by motor velocity drop
      4) Update internal state
      5) Record gyro traces for empirical calibration
    """

    def __init__(self, kill_switch_flag : threading.Event = None, motor_port="A", gyro_port=3, calibration=True, debug=True):
        """

        :param kill_switch_flag: The kill switch flag passed on from main code which will be set by _gyro_worker once kill switch is activated
        :param motor_port:
        :param gyro_port:
        :param calibration: if set to True, the GrabberController will proceed to calibrating both the Gyro sensor and the Belt motor. It will take approximately 20 seconds, and calibration data will be saved to working directory
        Note : if the grabber starts acting weird (trigger events missed or not inexistent) set calibration to True.
        :param debug: Activates verbose logging
        """
        self.cube_count = 0
        self.cube_positions = [BlocLocation.UNLOADED, BlocLocation.UNLOADED]
        self.calibration = calibration
        self.debug = debug

        # Thread-safe state / async-like control
        self.state_lock = threading.Lock()

        self._cube_caught_event = threading.Event()

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
        self.running_dps = 300

        # Motor-drop detection knobs
        self.motor_calibration_dps = 300
        self.motor_sample_sleep = 0.02
        self.motor_eject_time = 2.0

        self.motor_dry_run_time = 5.0
        self.motor_drop_k = 0.5
        self.motor_drop_consecutive_needed = 1
        self.use_motor_moving_average = True
        self.motor_moving_avg_window = 5
        self.motor_drop_warm_up_delay = 0.8

        self.gyro_calibration_file = "" # used to keep track of what calibration was used for the gyro

        self._unload_thread = None
        self._unload_done_event = threading.Event()
        self._unload_cancel_event = threading.Event()
        self._unload_result = None
        self.kill_switch = False

        if self.running_dps > 400:
            print("Warning: running power too high. Pick a lower value")

        # Empirical gyro trace recording
        self.recordings_dir = "data/gyro_operation_recordings"
        os.makedirs(self.recordings_dir, exist_ok=True)

        self._current_record = None
        self._recording_lock = threading.Lock()

        self._command_queue = deque()
        if __name__ == "__main__":
            print("Starting Gyro controller input thread")
            self._input_thread = threading.Thread(target=self._input_worker, daemon=True)
            self._input_thread.start()

        self._gyro_thread = None
        self._gyro_stop = threading.Event()
        self._gyro_sample = None
        self._gyro_sample_lock = threading.Lock()

        self._kill_event = kill_switch_flag if kill_switch_flag is not None else threading.Event()
        self.kill_switch = False
        self._kill_lock = threading.Lock()
        self._active_gyro_detector = None
        self._active_gyro_detector_lock = threading.Lock()

        self._gyro_sample_seq = 0
        self._gyro_sample_cond = threading.Condition(self._gyro_sample_lock)



    def _gyro_worker(self):
        """
        Threaded loop to poll Gyro DPS and publish dps to the kill switch monitor and to any gyro event detection event
        :return:
        """
        dt = 1.0 / float(self.sample_hz)
        print("[GYRO] worker started")

        try:
            kill_hits = 0
            kill_needed = 5
            while not self._gyro_stop.is_set():
                raw = self.gyro.get_dps_measure()
                now = time.time()

                if raw is None:
                    time.sleep(dt)
                    continue

                dps = float(raw)
                z = (dps - self.mu) / max(self.sigma, 1e-6)

                # internal kill trigger
                if self._is_tilted_outwards_sample(dps, GyroTilt.FAST_TILT_KILL):
                    print("[KILL SWITCH] Initial outwards press detected")
                    """
                    threading.Thread(
                        target=self._record_after_kill_trigger,
                        args=(5.0,),
                        daemon=True
                    ).start()
                    if self._is_tilted_inwards_sample(dps, GyroTilt.FAST_TILT):
                        print("[KILL SWITCH] Secondary inwards press detected")
                        self.kill(reason=f"FAST_TILT_KILL detected: dps={dps:.3f}")
                        return
                    """
                    if self._wait_for_inwards_after_kill(window_s=1.0, needed=2):
                        print("[KILL SWITCH] Secondary inwards press detected")
                        self.kill(reason="FAST_TILT_KILL: 1 slow inwards within 2s")
                        return

                with self._gyro_sample_cond:
                    self._gyro_sample = (now, dps, z)
                    self._gyro_sample_seq += 1
                    self._gyro_sample_cond.notify_all()

                time.sleep(dt)

        except Exception as e:
            print(f"[GYRO] worker crashed: {e}")
            raise

    def start_gyro_monitor(self):
        """
        Starts gyro monitor : MUST be called
        :return:
        """
        if self._gyro_thread and self._gyro_thread.is_alive():
            return
        self._gyro_stop.clear()
        self._gyro_thread = threading.Thread(target=self._gyro_worker, daemon=True)
        self._gyro_thread.start()

        # only needed if you still want external event-setting to work too
        threading.Thread(target=self.monitor_kill, daemon=True).start()
    # ----------------------------
    # Setup / calibration
    # ----------------------------
    def stop_gyro_monitor(self):
        self._gyro_stop.set()

        if self._gyro_thread is not None:
            self._gyro_thread.join(timeout=1.0)

        self._gyro_thread = None

    def _load_cube_worker(self,timeout=5.0):
        try:
            self._load_result = self.load_cube(timeout=timeout,stop_event=self._load_cancel_event)
        finally:
            self._load_done_event.set()

    def start_loading(self, timeout=5.0) -> bool:
        """
        Starts loading a cube (regardless of current count)
        :param timeout: past this timeout the function returns False
        :return: the status of loading (true/false)
        """
        if self._load_thread is not None and self._load_thread.is_alive():
            print("[INFO] Loading already in progress.")
            return False

        if self._unload_thread is not None and self._unload_thread.is_alive():
            print("[INFO] Cannot start loading while unloading is in progress.")
            return False

        self._load_done_event.clear()
        self._load_cancel_event.clear()
        self._cube_caught_event.clear()
        self._load_result = None

        self._load_thread = threading.Thread(
            target=self._load_cube_worker,
            kwargs={"timeout": timeout},
            daemon=True,
        )
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

    def _unload_cube(self, stop_event=None) -> bool:
        """
        This function is meant to be used INTERNALLY only
        Unload logic:

        If 2 cubes loaded:
          - Run outward until slow outward gyro tilt is detected

        If 1 cube remaining:
          - Use motor velocity drop detection
        """
        if self.cube_count <= 0:
            print("No cubes to unload.")
            return False

        try:
            if stop_event is not None and stop_event.is_set():
                print("[INFO] Unloading cancelled before start.")
                return False

            if self.cube_count == 2:
                self.motor.set_dps(-1.3 * self.running_dps)
                print("[INFO] Unloading first cube")
                unloaded = self._consecutive_gyro_trigger(
                    operation_name="unload_from_2_to_1",
                    detector_name="gyro_inwards_slow",
                    predicate_from_dps=lambda dps: self._is_tilted_inwards_sample(dps, GyroTilt.SLOW_TILT),
                    needed=1,
                    timeout=5.0,
                    extra={
                        "bloc_count_before": self.cube_count,
                        "running_dps": self.running_dps,
                        "direction": "outward",
                    },
                    stop_event=stop_event,
                )

                self.motor.set_dps(0)

                if not unloaded:
                    print("Timeout or cancellation unloading (no outward SLOW tilt detected).")
                    return False

                if stop_event is not None and stop_event.is_set():
                    print("[INFO] Unloading cancelled after detection.")
                    return False

                with self.state_lock:
                    self.cube_positions[0] = BlocLocation.AT_GATE
                    self.cube_positions[1] = BlocLocation.UNLOADED
                    self.cube_count = max(0, self.cube_count - 1)

                print("Unloaded one cube (2->1).")
                return True

            # only one cube left
            self.motor.set_dps(-self.running_dps)
            print("Unloading last cube: using motor velocity drop detection...")

            ok = self._is_motor_drop_detected(
                timeout=5.0,
                warmup_delay=0.9,
                stop_event=stop_event,
                operation_name="unload_last_bloc",
                extra={
                    "bloc_count_before": self.cube_count,
                    "running_dps": self.running_dps,
                    "direction": "outward",
                },
            )

            if not ok:
                print("Timed out or cancelled waiting for motor velocity drop while unloading last cube.")
                return False

            if stop_event is not None and stop_event.is_set():
                print("[INFO] Unloading cancelled after drop detection.")
                return False

            self.motor.set_dps(-self.running_dps)
            time.sleep(0.1)
            self.motor.set_dps(0)

            with self.state_lock:
                self.cube_positions = [BlocLocation.UNLOADED, BlocLocation.UNLOADED]
                self.cube_count = 0

            print("Unloaded last cube (1->0).")
            return True

        finally:
            self.motor.set_dps(0)

    def _unload_cube_worker(self):
        try:
            self._unload_result = self._unload_cube(stop_event=self._unload_cancel_event)
        finally:
            self._unload_done_event.set()

    def start_unloading(self) -> bool:
        """
        Start unload_cube() in a background thread and return immediately.

        Returns False if an unloading operation is already in progress, or if
        a loading operation is currently running. True otherwise.
        """
        if self._unload_thread is not None and self._unload_thread.is_alive():
            print("[INFO] Unloading already in progress.")
            return False

        if self._load_thread is not None and self._load_thread.is_alive():
            print("[INFO] Cannot start unloading while loading is in progress.")
            return False

        self._unload_done_event.clear()
        self._unload_cancel_event.clear()
        self._unload_result = None

        self._unload_thread = threading.Thread(target=self._unload_cube_worker, daemon=True)
        self._unload_thread.start()
        return True

    def is_unloading(self) -> bool:
        return self._unload_thread is not None and self._unload_thread.is_alive()

    def unloading_finished(self) -> bool:
        return self._unload_done_event.is_set()

    def get_unloading_result(self):
        return self._unload_result

    def cancel_unloading(self):
        self._unload_cancel_event.set()

    def get_unloading_status(self) -> str:
        if self._unload_thread is not None and self._unload_thread.is_alive():
            return "UNLOADING"

        if self._unload_done_event.is_set():
            if self._unload_result is True:
                return "SUCCESS"
            return "FAILED"

        return "IDLE"

    def initialize_sensors(self):
        print("[Calibration] Initializing sensors... Don't move the Gyro sensor")
        self.gyro.wait_ready()
        self.gyro.reset_measure()
        self.motor.set_dps(0)
        print("Reset measures; motor stopped.")
        self.start_gyro_monitor()  # THIS IS KEY
        print("started gyro monitor")

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
        calibration_dir = getattr(self, "gyro_calibration_dir", "data/gyro_calibrations")
        os.makedirs(calibration_dir, exist_ok=True)

        if calibration:
            self.motor.set_dps(self.motor_calibration_dps)
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
                "samples": samples,  # <-- added samples
            }

            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            calibration_file = os.path.join(
                calibration_dir, f"gyro_calibration_{timestamp}.pkl"
            )
            self.gyro_calibration_file = f"gyro_calibration_{timestamp}.pkl"

            with open(calibration_file, "wb") as f:
                pickle.dump(calibration_data, f)

            if self.debug:
                print(f"[Gyro calibration] Saved to {calibration_file}")

            self.motor.set_dps(0)

        else:
            calibration_files = sorted(
                glob.glob(os.path.join(calibration_dir, "gyro_calibration_*.pkl"))
            )

            if not calibration_files:
                raise FileNotFoundError(
                    f"No gyro calibration file found in '{calibration_dir}'."
                )

            latest_file = calibration_files[-1]
            self.gyro_calibration_file = latest_file

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

    def eject_existing_blocs(self,duration=5):
        print("[Eject] Ejecting any existing blocs...")
        self.motor.set_dps(-self.motor_calibration_dps)
        time.sleep(duration)
        self.motor.set_dps(0)
        time.sleep(0.5)


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
        calibration_dir = getattr(self, "motor_calibration_dir", "data/motor_calibrations")
        os.makedirs(calibration_dir, exist_ok=True)

        if calibration:

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
        """
        Returns the k value for different GyroTilt events
        :param tilt:
        :return: float

        Note : in general the higher the K, the lower the sensitivity (and vice versa)
        """
        if tilt == GyroTilt.SLOW_TILT:
            return 10.0
        elif tilt == GyroTilt.FAST_TILT_KILL :
            return 40.0
            # change this value if needed for kill switch
        else:
            return 16.0

    def _floor_for_inwards(self, tilt: GyroTilt) -> float:
        """
        Floor for inwards detection
        :param tilt:
        :return:
        """
        return -self.sigma*2 if tilt == GyroTilt.SLOW_TILT else -self.sigma*3

    def _floor_for_outwards(self, tilt: GyroTilt) -> float:
        """
        Floor for outwards detection
        :param tilt:
        :return:
        """
        return self.sigma*2 if tilt == GyroTilt.SLOW_TILT else self.sigma*3

    def _gyro_threshold_inwards(self, tilt: GyroTilt) -> float:
        """
        Returns the MINIMUM between 1) mu-ksigma and 2) the floor preset
        :param tilt:
        :return:
        """
        k = self._k_for_tilt(tilt)
        thr = self.mu - k * self.sigma
        return min(thr, self._floor_for_inwards(tilt))

    def _gyro_threshold_outwards(self, tilt: GyroTilt) -> float:
        """
        Returns the MINIMUM between 1) mu-ksigma and 2) the floor preset
        :param tilt:
        :return:
        """
        k = self._k_for_tilt(tilt)
        thr = self.mu + k * self.sigma
        return max(thr, self._floor_for_outwards(tilt))

    def _is_tilted_inwards_sample(self, dps: float, tilt: GyroTilt) -> bool:
        """
        performs statistical threshold detection k-mu*sigma
        """
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
            "motor_samples": [],  # <-- add this
            "outcome": None,
            "gyro_calibration": self.gyro_calibration_file,
        }

    def _append_motor_sample_to_record(self, elapsed_s: float, motor_dps: float):
        if self._current_record is None:
            return

        sample = {
            "t": elapsed_s,
            "dps": motor_dps,
        }

        with self._recording_lock:
            self._current_record["motor_samples"].append(sample)

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
        if not self.debug:
            self._current_record = None
            return
        if self._current_record is None:
            return

        with self._recording_lock:
            self._current_record["outcome"] = outcome
            self._current_record["end_datetime"] = datetime.now().isoformat()

            if not self._current_record["samples"] and not self._current_record["motor_samples"]:
                print("[Recording] WARNING: no samples collected, not saving empty recording")
                self._current_record = None
                return

            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            op = self._current_record["operation"]
            filename = os.path.join(self.recordings_dir, f"{op}_{timestamp}.pkl")

            with open(filename, "wb") as f:
                pickle.dump(self._current_record, f)

            print(f"[Recording] Saved operation trace to {filename}")
            self._current_record = None

    def _read_gyro_dps(self, last_seq=None, timeout=None):
        """
        Wait for a fresh gyro sample from the gyro worker.

        Returns:
            (seq, timestamp, dps, z) on success
            None on timeout
        """

        with self._gyro_sample_cond:
            if last_seq is None:
                if self._gyro_sample is None:
                    got = self._gyro_sample_cond.wait(timeout=timeout)
                    if not got and self._gyro_sample_seq == last_seq:
                        print("[GYRO] timed out waiting for fresh sample")
                        return None
                    if not got or self._gyro_sample is None:
                        return None
                ts, dps, z = self._gyro_sample
                return self._gyro_sample_seq, ts, dps, z

            end_time = None if timeout is None else (time.time() + timeout)

            while self._gyro_sample_seq == last_seq:
                remaining = None if end_time is None else (end_time - time.time())
                if remaining is not None and remaining <= 0:
                    return None
                got = self._gyro_sample_cond.wait(timeout=remaining)
                if not got and self._gyro_sample_seq == last_seq:
                    return None

            ts, dps, z = self._gyro_sample
            return self._gyro_sample_seq, ts, dps, z
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
            Listen to gyro samples and apply the given predicate to detect events. Return True if the predicate is satisfied for 'needed' consecutive samples, False if timeout or stop_event is set.
        :param operation_name:
        :param detector_name:
        :param predicate_from_dps:
        :param needed:
        :param sample_hz:
        :param timeout:
        :param extra:
        :param stop_event:
        :return:
        """
        if not self._gyro_thread or not self._gyro_thread.is_alive():
            raise RuntimeError("You must start the gyro thread before listening for gyro events")

        needed = self.consecutive_needed if needed is None else int(needed)
        record_enabled = self.debug and bool(detector_name) and bool(operation_name)

        detector_threshold = None
        detector_direction = None
        detector_floor = None

        if detector_name == "gyro_outwards_fast":
            detector_floor = self._floor_for_outwards(GyroTilt.FAST_TILT)
            k = self._k_for_tilt(GyroTilt.FAST_TILT)
            detector_threshold = self.mu + k * self.sigma
            detector_direction = "outwards"
        elif detector_name == "gyro_outward_slow":
            detector_floor = self._floor_for_outwards(GyroTilt.SLOW_TILT)
            k = self._k_for_tilt(GyroTilt.SLOW_TILT)
            detector_threshold = self.mu + k * self.sigma
            detector_direction = "outwards"
        elif detector_name == "gyro_inwards_fast":
            detector_floor = self._floor_for_inwards(GyroTilt.FAST_TILT)
            k = self._k_for_tilt(GyroTilt.FAST_TILT)
            detector_threshold = self.mu - k * self.sigma
            detector_direction = "inwards"
        elif detector_name == "gyro_inwards_slow":
            detector_floor = self._floor_for_inwards(GyroTilt.SLOW_TILT)
            k = self._k_for_tilt(GyroTilt.SLOW_TILT)
            detector_threshold = self.mu - k * self.sigma
            detector_direction = "inwards"

        if record_enabled:
            self._begin_operation_recording(
                operation_name=operation_name,
                timeout=timeout,
                extra={
                    **(extra or {}),
                    "detector": detector_name,
                    "needed": needed,
                    "detector_threshold": detector_threshold,
                    "detector_direction": detector_direction,
                    "detector_floor": detector_floor,
                },
            )

        hits = 0
        start = time.time()
        last_seq = None

        try:
            while hits < needed:
                if self.kill_switch or (self._kill_event is not None and self._kill_event.is_set()):
                    if record_enabled:
                        self._end_operation_recording(outcome="killed")
                    return False

                if stop_event is not None and stop_event.is_set():
                    if record_enabled:
                        self._end_operation_recording(outcome="cancelled")
                    return False

                elapsed = time.time() - start
                if timeout is not None and elapsed >= timeout:
                    if record_enabled:
                        self._end_operation_recording(outcome="timeout")
                    return False

                remaining = None if timeout is None else max(0.0, timeout - elapsed)
                sample = self._read_gyro_dps(last_seq=last_seq, timeout=remaining)

                if sample is None:
                    if record_enabled:
                        self._end_operation_recording(outcome="timeout")
                    return False

                last_seq, ts, dps, z = sample
                elapsed = ts - start
                motor_speed = self.motor.get_speed()
                if motor_speed is not None and record_enabled:
                    self._append_motor_sample_to_record(
                        elapsed_s=elapsed,
                        motor_dps=float(motor_speed),
                    )

                detector_flags = {
                    "slow_in": self._is_tilted_inwards_sample(dps, GyroTilt.SLOW_TILT),
                    "fast_in": self._is_tilted_inwards_sample(dps, GyroTilt.FAST_TILT),
                    "slow_out": self._is_tilted_outwards_sample(dps, GyroTilt.SLOW_TILT),
                    "fast_out": self._is_tilted_outwards_sample(dps, GyroTilt.FAST_TILT),
                }

                cond = bool(predicate_from_dps(dps))
                detector_flags["active_detector"] = cond

                if record_enabled:
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

            if record_enabled:
                self._end_operation_recording(outcome="success")
            return True

        except Exception:
            if record_enabled:
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
            if self.cube_count != 0:
                return

            if (time.time() - cooldown) < duration:
                time.sleep(0.01)
                continue

            if self.motor.get_dps() < self.running_dps * 2 / 3:
                print(f"[Motor drop detected] Temporarily increasing velocity to prevent cubeking. Speed set to {self.running_dps * 5}")

                cooldown = time.time()

                boosted_power = self.running_dps * 5 #min(1.2 * self.motor.get_power(), self.running_dps * 1.3)
                self.motor.set_dps(boosted_power)

                time.sleep(duration)

                if self.get_loading_result() == "IDLE":
                    self.motor.set_dps(0)
                    print("Operation finished, terminating")


    def _is_motor_drop_detected(
            self,
            timeout=5.0,
            warmup_delay=0.5,
            stop_event=None,
            operation_name=None,
            extra=None,
    ) -> bool:
        record_enabled = self.debug and bool(operation_name)

        if record_enabled:
            self._begin_operation_recording(
                operation_name=operation_name,
                timeout=timeout,
                extra={
                    **(extra or {}),
                    "detector": "motor_drop",
                    "motor_mu_in": self.motor_mu_in,
                    "motor_sigma_in": self.motor_sigma_in,
                    "motor_drop_k": self.motor_drop_k,
                    "motor_threshold": self.motor_mu_in - self.motor_drop_k * self.motor_sigma_in,
                },
            )
        #Looks at the derivative of the velocity (acceleration) because sometimes the drop might be too subtle to be observed by looking at constant velocity threshold
        threshold = self.motor_mu_in - self.motor_drop_k * self.motor_sigma_in
        hits = 0
        start = time.time()
        ma_window = deque(maxlen=max(1, self.motor_moving_avg_window))

        print(
            f"[Drop detect] Waiting for inward speed drop: "
            f"threshold={threshold:.3f}, "
            f"k={self.motor_drop_k}, needed={self.motor_drop_consecutive_needed}"
        )
        try :
            while True:
                if self.kill_switch or (self._kill_event is not None and self._kill_event.is_set()):
                        if record_enabled:
                            self._end_operation_recording(outcome="killed")
                        return False
                if stop_event is not None and stop_event.is_set():
                    if record_enabled:
                        self._end_operation_recording(outcome="cancelled")
                    return False

                elapsed = time.time() - start
                if timeout is not None and elapsed >= timeout:
                    if record_enabled:
                        self._end_operation_recording(outcome="timeout")
                    return False

                raw_speed = self.motor.get_speed()
                if raw_speed is None:
                    time.sleep(self.motor_sample_sleep)
                    continue

                raw_speed = float(raw_speed)

                if record_enabled:
                    self._append_motor_sample_to_record(
                        elapsed_s=elapsed,
                        motor_dps=raw_speed,
                    )

                    # also store current gyro value if available, so both traces exist
                    latest = self._read_gyro_dps(last_seq=None, timeout=0.0)
                    if latest is not None:
                        _, ts, dps, z = latest
                        detector_flags = {
                            "motor_drop_condition": speed_for_detection < threshold if 'speed_for_detection' in locals() else False
                        }
                        self._append_gyro_sample_to_record(
                            elapsed_s=elapsed,
                            dps=dps,
                            z=z,
                            detectors=detector_flags,
                        )

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
                    print("Setting BLOCK CAUGHT flag")
                    self._cube_caught_event.set()
                    if record_enabled:
                        self._end_operation_recording(outcome="success")
                    return True

                time.sleep(self.motor_sample_sleep)
        except Exception:
            if record_enabled:
                self._end_operation_recording(outcome="error")
            raise

    # ----------------------------
    # High-level actions
    # ----------------------------
    def load_cube(self, timeout=5.0,stop_event=None) -> bool:
        """
        Returns True on success, False on failure.

        Logic:
        - First cube: use gyro inward detection
        - Second cube: use motor-speed drop detection
        """
        if self.cube_count >= 2:
            print("Already loaded 2 cubes, aborting.")
            return False

        self.motor.set_dps(1.8*self.running_dps)

        try:
            if stop_event is not None and stop_event.is_set():
                print("[INFO] Loading cancelled before start.")
                return False

            # First cube: gyro-based detection with unified sample stream
            if self.cube_count == 0:
                print("[INFO] Started velocity drop detection thread")

                ok = self._consecutive_gyro_trigger(
                    operation_name="load_first_bloc",
                    detector_name="gyro_outwards_fast", #TODO fix the inconsistency
                    predicate_from_dps=lambda dps: self._is_tilted_outwards_sample(dps, GyroTilt.FAST_TILT),
                    needed=1,
                    timeout=timeout,
                    extra={
                        "bloc_count_before": self.cube_count,
                        "running_dps": self.running_dps,
                        "direction": "outwards",
                    },
                    stop_event=stop_event,
                )

                if not ok:
                    print("Timeout or cancellation waiting for first cube to arrive.")
                    return False

                if stop_event is not None and stop_event.is_set():
                    print("[INFO] Loading cancelled after detection.")
                    return False

                print("[INFO] First cube gyro event triggered")
                #time.sleep(2) # make sure cube goes to back
                self.motor.set_dps(0)
                with self.state_lock:
                    self.cube_positions[0] = BlocLocation.UNDER_GYRO
                    self.cube_count = 1

                print("Successfully loaded first cube.")
                return True

            # Second cube: motor-drop detection
            if self.cube_positions[0] == BlocLocation.UNLOADED:
                print("Error: internal state inconsistency: cube count is 1 but no cube is loaded.")
                return False

            self.motor.set_dps(self.running_dps)
            print("Loading second cube: using motor velocity drop detection...")
            #time.sleep(self.motor_drop_warm_up_delay)

            if stop_event is not None and stop_event.is_set():
                print("[INFO] Loading cancelled during second-cube warm-up.")
                return False

            ok = self._is_motor_drop_detected(
                timeout=5.0,
                warmup_delay=0.3,
                stop_event=stop_event,
                operation_name="load_second_bloc",
                extra={
                    "bloc_count_before": self.cube_count,
                    "running_dps": self.running_dps,
                    "direction": "inward",
                },
            )
            #TODO : problem is that if the first block is only after the gyro sensor it might temporarily jam the grabber thus triggering the velocity drop event
            if not ok:
                print("Timed out or cancelled waiting for motor velocity drop while loading second bloc.")
                return False

            with self.state_lock:
                self.cube_positions[0] = BlocLocation.BACK_STORAGE
                self.cube_positions[1] = BlocLocation.AT_GATE
                self.cube_count = 2

            print("Successfully loaded second cube.")
            return True

        finally:
            self.motor.set_dps(0)


    # ----------------------------
    # Main loop
    # ----------------------------

    def monitor_kill(self):
        self._kill_event.wait()
        #self.kill(reason="event set") OPTIONAL ONLY FOR DEBUGGING

    def loop(self):
        self.start_gyro_monitor()
        #threading.Thread(target=self.monitor_kill, daemon=True).start()
        self.motor.set_dps(-self.running_dps)
        time.sleep(1)
        self.motor.set_dps(0)
        print("Ejected all cubes")

        try:
            while True:
                # -------------------------
                # 1. Handle user commands
                # -------------------------
                if self.kill_switch:
                    print("[INFO] KILL SWITCH ACTIVATED")
                    raise KeyboardInterrupt
                if self._command_queue:
                    cmd = self._command_queue.popleft()

                    if cmd == "l":
                        print("Starting non-blocking load...")
                        self.start_loading()


                    elif cmd == "u":
                        print("Starting non-blocking unload...")
                        self.start_unloading()

                    elif cmd == "q":
                        print("Exiting...")
                        raise KeyboardInterrupt

                # -------------------------
                # 2. Periodic checks
                # -------------------------
                if self.is_loading():
                    if self.loading_finished() and not self.is_loading():
                        print(f"[INFO] Loading finished: {self.get_loading_result()}")

                    if self.unloading_finished() and not self.is_unloading():
                        print(f"[INFO] Unloading finished: {self.get_unloading_result()}")

                # -------------------------
                time.sleep(0.01)

        except KeyboardInterrupt:
            print("Stopping grabber controller loop.")
            self.cancel_loading()
            self.cancel_unloading()
            self.motor.set_dps(0)
            self._gyro_stop.set()

    def kill(self, reason="internal trigger"):
        with self._kill_lock:
            if self.kill_switch:
                return

            print(f"[INFO] KILL SWITCH ACTIVATED: {reason}")
            self.kill_switch = True

            # publish the kill state to the shared event
            self._kill_event.set()

            # cancel cooperative operations
            self.cancel_loading()
            self.cancel_unloading()

            # stop hardware
            self.motor.set_dps(0)
            self._gyro_stop.set()

            """
            try:
                reset_brick()
            except Exception as e:
                print(f"[WARN] reset_brick failed during kill: {e}")
            """

    def monitor_kill(self):
        self._kill_event.wait()
        self.kill(reason="event set")

    def isCubeCaught(self) -> bool:
        return self._cube_caught_event.is_set()

    def clearCubeCaught(self):
        self._cube_caught_event.clear()

    def _input_worker(self):
        while True:
            try:
                cmd = input("Enter command (l=load, u=unload, q=quit): ").lower()
                self._command_queue.append(cmd)
            except EOFError:
                break

    def get_loading_status(self) -> str:
        if self._load_thread is not None and self._load_thread.is_alive():
            if self._cube_caught_event.is_set():

                return "BLOCK_CAUGHT"
            return "LOADING"

        if self._load_done_event.is_set():
            if self._load_result is True:
                return "SUCCESS"
            return "FAILED"

        return "IDLE"

    def wait_for_loading(self,expected_bloc_count=0, label="block")->bool:
        start = time.time()
        last_log = start
        counter = 0
        while True:
            status = self.get_loading_status()

            if status == "FAILED" or not status:
                return False

            if (self.loading_finished() or status == "IDLE") and not self.is_loading():
                break

            now = time.time()
            if now - last_log > 1.0:
                print(f"[DEBUG] Waiting for loading of {label}... status={status}")
                last_log = now
                counter +=1
            if counter > 1:
                self.motor.set_dps(6*self.running_dps)
                print("Temporary BOOST")
                counter = 0
                time.sleep(1.0)
                #threading.Timer(1.8,lambda:self.motor.set_dps(1.8*self.running_dps)).start()

            time.sleep(0.05)

        final_status = self.get_loading_status()
        print(f"[DEBUG] Loading finished for {label} with status={final_status}")
        self.motor.set_dps(0)
        return True

    def eject_existing_blocs(self,duration=5):
        print("Ejecting all cubes inside the Grabber")
        time.sleep(0.1)
        self.motor.set_dps(-self.motor_calibration_dps)
        time.sleep(duration)
        self.motor.set_dps(0)
        time.sleep(0.5)

    def _record_after_kill_trigger(self, duration=5.0):
        """
        Records gyro data for a fixed duration after kill trigger
        """
        print("[RECORD] Starting post-kill recording")

        self._begin_operation_recording(
            operation_name="kill_switch_post_record",
            timeout=duration,
            extra={"type": "post_kill"}
        )

        start = time.time()
        last_seq = None

        while time.time() - start < duration:
            sample = self._read_gyro_dps(last_seq=last_seq, timeout=0.1)
            if sample is None:
                continue

            last_seq, ts, dps, z = sample
            elapsed = ts - start

            self._append_gyro_sample_to_record(
                elapsed_s=elapsed,
                dps=dps,
                z=z,
                detectors={"post_kill": True},
            )

        self._end_operation_recording(outcome="completed")
        print("[RECORD] Finished post-kill recording")

    def _wait_for_inwards_after_kill(self, window_s=2.0, needed=2) -> bool:
        """
        After the initial outward kill-switch trigger, listen for up to window_s
        seconds and return True only if we observe `needed` consecutive
        FAST_TILT inwards samples.
        """
        start = time.time()
        hits = 0
        last_seq = None

        while time.time() - start < window_s:
            remaining = max(0.0, window_s - (time.time() - start))
            sample = self._read_gyro_dps(last_seq=last_seq, timeout=remaining)

            if sample is None:
                return False

            last_seq, ts, dps, z = sample

            if self._is_tilted_inwards_sample(dps, GyroTilt.SLOW_TILT):
                hits += 1
                if hits >= needed:
                    return True
            else:
                hits = 0

        return False

if __name__ == "__main__":
    kill_event = threading.Event()
    controller = GrabberController(kill_event, "D", 4, False, True)
    controller.initialize_sensors()
    controller.loop()
