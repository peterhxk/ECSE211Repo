#!/usr/bin/env python3
import math
import statistics
import threading
from enum import Enum
from collections import deque
import sys
from tqdm import tqdm
sys.path.append("../utils")
from brick import EV3GyroSensor, TouchSensor, Motor
import os
import glob
import time
import pickle
from datetime import datetime
import threading


class BlocLocation(Enum):
    BACK_STORAGE = 1 # in the back storage area
    UNDER_GYRO = 2 # under the gyro arm, either pushing inwards or outwards
    AT_GATE = 2 # the closest to the end of the parallel conveyor belts
    UNLOADED = 3 # not yet loaded, or already unloaded


class GyroTilt(Enum):
    SLOW_TILT = 1
    FAST_TILT = 2


class GrabberController:
    """
    Responsibilities:
      1) Run the conveyor belt inwards / outwards (via motor power)
      2) Detect events with the gyroscope (DPS thresholds based on calibrated mu/sigma)
      3) Detect second-bloc loading by motor velocity drop
      4) Update internal state (how many blocs are loaded and where they are)
      5) Load / unload blocs
    """

    def __init__(self,motor_port="A",gyro_port=3,calibration=True,debug=True):

        # Internal state variables

        self.bloc_count = 0
        self.bloc_positions = [BlocLocation.UNLOADED, BlocLocation.UNLOADED]
        self.calibration = calibration
        self.debug = debug

        # Hardware definition
        self.gyro = EV3GyroSensor(port=gyro_port, mode=EV3GyroSensor.Mode.DPS)
        self.motor = Motor(motor_port)

        # Gaussian parameters for idle gyro DPS
        self.mu = 0.0
        self.sigma = 1.0

        # Gaussian parameters for inward motor speed at power=45
        self.motor_mu_in = 0.0
        self.motor_sigma_in = 1.0

        # Detection / timing knobs
        self.sample_hz = 50
        self.consecutive_needed = 15  # gyro detection default
        self.running_power = 60

        # Motor-drop detection knobs
        self.motor_calibration_power = 45
        self.motor_sample_sleep = 0.02
        self.motor_eject_time = 2.0
        self.motor_dry_run_time = 5.0
        self.motor_drop_k = 3.0
        self.motor_drop_consecutive_needed = 5
        self.use_motor_moving_average = True
        self.motor_moving_avg_window = 5

        if self.running_power > 70:
            print("Warning: running power too high. Pick a lower value")

    # ----------------------------
    # Setup / calibration#
    # ----------------------------
    def initialize_sensors(self):
        print("[Calibration] Initializing sensors... Don't move the Gyro sensor")
        self.gyro.wait_ready()
        #self.loading_button.wait_ready()
        #self.unloading_button.wait_ready()

        self.gyro.reset_measure()
        self.motor.set_power(0)
        print("Reset measures; motor stopped.")

        self.calibrate_motor_drop_detection()
        print("Calibrated motor inward speed-drop mu and sigma.")

        self.calibrate_gyro_noise(seconds=15.0, sample_hz=self.sample_hz, warmup=0.5)
        print("Calibrated gyro mu and sigma.")


    def calibrate_gyro_noise(self, seconds=10.0, sample_hz=50, warmup=0.5):
        """
        Estimate mu and sigma for idle gyro DPS.
        """
        print(f"[Calibration] Calibrating gyro noise... Don't move the sensor during this period ( {seconds} seconds)")
        dt = 1.0 / float(sample_hz)

        t0 = time.time()
        while time.time() - t0 < warmup:
            _ = self.gyro.get_dps_measure()
            time.sleep(dt)

        samples = []
        t1 = time.time()

        n_samples = int(seconds * sample_hz)

        for _ in tqdm(range(n_samples), desc="[Calibration]", unit="sample"):
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
        print(f"[Gyro calibration] n={len(samples)} mu={mu:.3f} dps, sigma={sigma:.3f} dps")
        return mu, sigma

    def _sample_motor_speeds(self, power, duration, sleep_time):
        """
        Run motor at constant power for `duration` seconds and collect raw signed speed samples.
        """
        samples = []
        self.motor.set_power(power)
        start = time.time()

        try:
            while time.time() - start < duration:
                speed = self.motor.get_speed()
                if speed is not None:
                    samples.append(float(speed))
                time.sleep(sleep_time)
        finally:
            self.motor.set_power(0)

        return samples

    def calibrate_motor_drop_detection(self, calibration=True):
        """
        If calibration=True:
            1) Eject outward
            2) Dry run both ways at power=45
            3) Keep inward mu/sigma for drop detection while loading second bloc
            4) Save calibration data to a timestamped serialized object

        If calibration=False:
            - Load the latest saved calibration object
            - Do not overwrite anything
        """
        calibration_dir = getattr(self, "motor_calibration_dir", "motor_calibrations")
        os.makedirs(calibration_dir, exist_ok=True)

        if calibration:
            if self.debug:
                print("[Motor calibration] Ejecting outward first...")
                self.motor.set_power(-self.motor_calibration_power)
                time.sleep(self.motor_eject_time)
                self.motor.set_power(0)
                time.sleep(0.5)

                print("[Motor calibration] Dry run outward...")
                outward_samples = self._sample_motor_speeds(
                    power=-self.motor_calibration_power,
                    duration=self.motor_dry_run_time,
                    sleep_time=self.motor_sample_sleep,
                )
                if len(outward_samples) < 2:
                    raise RuntimeError("Not enough outward motor samples collected.")

                mu_out = statistics.mean(outward_samples)
                sigma_out = statistics.stdev(outward_samples)
                sigma_out = max(sigma_out, 1e-6)

                print(
                    f"[Motor calibration] outward n={len(outward_samples)} "
                    f"mu={mu_out:.3f}, sigma={sigma_out:.3f}"
                )

                time.sleep(0.5)

                print("[Motor calibration] Dry run inward...")
                inward_samples = self._sample_motor_speeds(
                    power=self.motor_calibration_power,
                    duration=self.motor_dry_run_time,
                    sleep_time=self.motor_sample_sleep,
                )
                if len(inward_samples) < 2:
                    raise RuntimeError("Not enough inward motor samples collected.")

                self.motor_mu_in = statistics.mean(inward_samples)
                self.motor_sigma_in = statistics.stdev(inward_samples)
                self.motor_sigma_in = max(self.motor_sigma_in, 1e-6)
                inward_n = len(inward_samples)

            else:
                raise RuntimeError("Calibration requested but debug mode is disabled.")

            calibration_data = {
                "date": datetime.now().isoformat(),
                "motor_mu_in": self.motor_mu_in,
                "motor_sigma_in": self.motor_sigma_in,
                "motor_calibration_power": self.motor_calibration_power,
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
    # Motor drop detection
    # ----------------------------

    def increase_on_velocity_drop(self, duration=0.5):
        start = time.time()
        cooldown = False

        while time.time() - start < duration and not cooldown:

            # stop thread if loading finished
            if self.bloc_count != 0:
                return

            if self._is_motor_drop_detected(timeout=5.0):
                print("[Motor drop detected] Temporarily increasing velocity to prevent blocking")

                cooldown = True

                boosted_power = min(1.5 * self.motor.get_power(), self.running_power * 1.4)
                self.motor.set_power(boosted_power)

                # restore only if still loading
                if self.bloc_count == 0:
                    self.motor.set_power(self.running_power)
        time.sleep(1.0)

    def _is_motor_drop_detected(self, timeout=5.0) -> bool:
        """
        While motor is already running inward, detect a sustained drop in motor speed:
            speed < mu_in - k * sigma_in
        for N consecutive samples.
        """
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
            if timeout is not None and (time.time() - start) >= timeout:
                return False

            raw_speed = self.motor.get_speed()
            if raw_speed is None:
                time.sleep(self.motor_sample_sleep)
                continue

            raw_speed = float(raw_speed)

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
                return True

            time.sleep(self.motor_sample_sleep)

    # ----------------------------
    # High-level actions
    # ----------------------------
    def load_bloc(self) -> bool:
        """
        Returns True on success, False on failure.

        Logic:
        - First bloc: use gyro inwards detection
        - Second bloc: run inward until motor-speed drop is detected
        """
        if self.bloc_count >= 2:
            print("Already loaded 2 blocs, aborting.")
            return False

        self.motor.set_power(self.running_power)
        start = time.time()

        try:
            if self.bloc_count == 0:
                dps = self.gyro.get_dps_measure()
                print(
                    f"dps={dps}, mu={self.mu}, sigma={self.sigma}, "
                    f"threshold={self.mu - 3 * self.sigma}"
                )
                # start a thread
                threading.Thread(target=self.increase_on_velocity_drop, daemon=True).start()

                print("Started THREAD")

                ok = self._consecutive_trigger(
                    lambda: self.is_tilted_inwards(GyroTilt.SLOW_TILT),
                    needed=3, #TODO make a script to calibrate this : as a general rule INCREASE this is it triggers by itself, DECREASE it if it has false triggers
                    timeout=5.0,
                )
                if not ok:
                    print("Timeout waiting for first bloc to arrive.")
                    return False

                self.motor.set_power(0)
                self.bloc_positions[0] = BlocLocation.UNDER_GYRO
                self.bloc_count = 1
                print("Successfully loaded first bloc.")
                return True

            # bloc_count == 1
            if self.bloc_positions[0] == BlocLocation.UNLOADED:
                print("Error: internal state inconsistency: bloc count is 1 but no bloc is loaded.")
                return False

            self.motor.set_power(45) # use calibration power
            print("Loading second bloc: using motor velocity drop detection...")
            time.sleep(1) # give a moment for the first bloc to reach the storage zone. NEEDS FINE TUNE
            #TODO make this more dynamic
            ok = self._is_motor_drop_detected(timeout=5.0)
            if not ok:
                print("Timed out waiting for motor velocity drop while loading second bloc.")
                return False

            self.motor.set_power(60)
            time.sleep(0.1)
            # to make sure it gets through
            self.motor.set_power(0)
            self.bloc_positions[0] = BlocLocation.BACK_STORAGE
            self.bloc_positions[1] = BlocLocation.AT_GATE
            self.bloc_count = 2
            print("Successfully loaded second bloc.")
            return True

        finally:
            if time.time() - start > 0:
                self.motor.set_power(0)

    def unload_bloc(self) -> bool:
        """
        Unload logic:

        If 2 blocs loaded:
          1) Wait for SLOW outwards tilt
          2) Run motor outwards until SLOW outwards tilt, then stop motor
          3) Update internal state

        If 1 bloc remaining:
          - Run motor outwards for ~2 seconds, then stop
          - Update internal state
        """
        if self.bloc_count <= 0:
            print("No blocs to unload.")
            return False

        try:
            if self.bloc_count == 2:
                self.motor.set_power(-1.2 * self.running_power)
                """
                unloaded = self._consecutive_trigger(
                    lambda: self.is_tilted_inwards(GyroTilt.FAST_TILT),
                    needed=3,
                    timeout=5.0,
                )
                """
                # alternative detection : we wait for the gyro to "spring back" which will give us more accuracy
                unloaded = self._consecutive_trigger(
                    lambda: self.is_tilted_outwards(GyroTilt.SLOW_TILT),
                    needed=3,
                    timeout=5.0,
                )
                self.motor.set_power(0)

                if not unloaded:
                    print("Timeout unloading (no outwards SLOW tilt detected).")
                    return False

                self.bloc_positions[0] = BlocLocation.AT_GATE
                # the first bloc should move to at gate position while the second one is ejected
                self.bloc_positions[1] = BlocLocation.UNLOADED

                self.bloc_count = max(0, self.bloc_count - 1)
                print("Unloaded one bloc (2->1).")
                return True

            # only one bloc left
            self.motor.set_power(-45)  # use calibration power
            print("Unloading first bloc: using motor velocity drop detection...")
            time.sleep(0.6)
            ok = self._is_motor_drop_detected(timeout=5.0)
            if not ok:
                print("Timed out waiting for motor velocity drop while unloading first bloc.")
                return False

            self.motor.set_power(-60)
            time.sleep(0.1)
            # to make sure it gets through
            self.motor.set_power(0)

            self.bloc_positions = [BlocLocation.UNLOADED, BlocLocation.UNLOADED]
            self.bloc_count = 0
            print("Unloaded last bloc (1->0).")
            return True

        finally:
            self.motor.set_power(0)

    # ----------------------------
    # Main loop
    # ----------------------------
    def loop(self):
        """
        Main loop of the grabber controller.
        NEGATIVE DPS : inward tilt
        POSITIVE DPS : outwards tilt
        """
        self.motor.set_power(-self.running_power)
        time.sleep(1)
        self.motor.set_power(0)
        print("Ejected all blocs")

        try:
            while True:
                user_command = input("Enter command (l=load, u=unload, q=quit): ").lower() # ideally later use MATCH statement for better handling
                #if self.loading_button.get_value():
                if user_command == 'l':
                    print("Loading a package into the Grabber")
                    success = self.load_bloc()
                    if success:
                        print(
                            f"Successfully loaded bloc. Count={self.bloc_count}, "
                            f"positions={self.bloc_positions}"
                        )
                    else:
                        print("Failed to load bloc.")
                    time.sleep(0.2)

                elif user_command == 'u':
                #elif self.unloading_button.get_value():
                    print("Unloading a package from the Grabber")
                    success = self.unload_bloc()
                    if success:
                        print(
                            f"Successfully unloaded bloc. Count={self.bloc_count}, "
                            f"positions={self.bloc_positions}"
                        )
                    else:
                        print("Failed to unload bloc.")
                    time.sleep(0.2)

                elif user_command == 'q':
                    time.sleep(0.2)
                    print("Gently excitting program")
                    raise KeyboardInterrupt

                time.sleep(0.01)

        except KeyboardInterrupt:
            print("Stopping grabber controller loop.")
            self.motor.set_power(0)

    def kill(self):
        raise NotImplementedError()


if __name__ == "__main__":
    controller = GrabberController("A",2,True,True)
    controller.initialize_sensors()
    controller.loop()