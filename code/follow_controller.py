# Comments in English only
import time
import math
import threading
import csv
import os
import collections
from dataclasses import dataclass
from typing import Any, Dict, Optional


@dataclass
class FollowConfig:
    # --- Loop / speed limits ---
    FOLLOW_DT: float = 0.02          # Internal loop period (overridden by AppConfig at runtime)
    MAX_VX_FOLLOW: float = 2.0       # Absolute forward ceiling (overridden by AppConfig at runtime)
    MAX_VY_FOLLOW: float = 0.8       # Overall lateral clamp

    # --- Cartesian targets (robot body frame) ---
    TARGET_Y: float = 1.1            # Desired LATERAL gap (m). This is the distance that is kept.
    TARGET_X: float = -0.6           # Desired forward offset (m). Human slightly behind robot.

    # --- Forward control ---
    KP_VX: float = 0.8
    DEADBAND_X: float = 0.05
    KI_X: float = 0.15               # Small integral (closes residual forward gap while walking)
    I_CLAMP: float = 0.30            # Anti-windup clamp

    # --- LATERAL control: asymmetric (gentle when far, STRONG when too close -> sideways evasion) ---
    DEADBAND_Y_CLOSE: float = 0.08   # small deadband when too close -> responsive
    DEADBAND_Y_FAR: float = 0.20     # larger deadband when too far -> relaxed
    KP_VY_CLOSE: float = 1.3         # strong push AWAY when the gap shrinks
    KP_VY_FAR: float = 0.6           # gentle pull in when the gap is too wide
    MAX_VY_CLOSE: float = 0.8        # fast sideways evasion
    MAX_VY_FAR: float = 0.5

    # --- Feed-forward (match the human's velocity) ---
    FF_GAIN: float = 1.0
    FF_ALPHA: float = 0.2

    # --- "Walk at the human's speed": dynamic forward cap ---
    CORRECTION_HEADROOM: float = 0.8   # m/s the robot may exceed the human's speed (gap closing)

    # --- Stop / start detection ---
    STOP_SPEED: float = 0.15        # m/s; smoothed speed below this -> stopped
    MOVE_SPEED: float = 0.25         # m/s; smoothed speed above this -> moving (also fast-start threshold)

    # --- Emergency proximity safety ---
    EMERGENCY_DIST: float = 0.65     # m; below this, hard move-away (last resort)
    RETREAT_SPEED: float = 0.30

    # --- Yaw: align robot heading with the human's direction of travel ---
    KP_WZ: float = 0.8
    DEADBAND_WZ: float = 0.08
    MAX_WZ_CORRECTION: float = 0.6
    WZ_SIGN: float = 1.0             # Flip to -1.0 if the robot yaws the wrong way
    YAW_COMP_SIGN: float = 1.0       # Rotation-compensation sign. Flip if it spins WHILE MOVING.

    # --- Output shaping (asymmetric: slow to speed up, fast to stop) ---
    SMOOTH_ALPHA: float = 0.2        # Output EMA (overridden by AppConfig at runtime)
    MAX_ACCEL_X: float = 0.8
    MAX_DECEL_X: float = 2.5
    MAX_ACCEL_Y: float = 2.0         # lateral evasion should be quick
    MAX_DECEL_Y: float = 3.0

    # --- Compatibility fields required by system_init.py (do not remove) ---
    LPF_ALPHA: float = 0.2
    POS_WINDOW_SIZE: int = 5
    HEADING_WINDOW_SIZE: int = 5
    YAW_WINDOW_SIZE: int = 5
    MAX_VX: float = 1.0
    MAX_VY: float = 0.40
    MAX_WZ: float = 0.96
    MAX_WZ_FOLLOW: float = 0.96
    DEAD_BAND_D: float = 1.2
    DEAD_BAND_O: float = 0.20
    DIST_SLOWDOWN: float = 1.0
    SLOWDOWN_ANGLE: float = 1.0
    TARGET_ORI: float = math.radians(90)


class FollowController:
    """
    UWB follow controller (polar -> body-frame Cartesian).
    Logs experiment data to a specific subject folder.
    """

    def __init__(
        self,
        state_manager: Any,
        avoid_client: Any,
        behavior: Dict[str, Any],
        config: Optional[FollowConfig] = None,
    ):
        self.state_manager = state_manager
        self.avoid_client = avoid_client
        self.behavior = behavior
        self.cfg = config or FollowConfig()
        self._thread: Optional[threading.Thread] = None

        # --- Experiment Setup ---
        self.subject_num = self.behavior.get("subject_num", "default")

        self.experiment_dir = os.path.join("Experiments", self.subject_num)
        os.makedirs(self.experiment_dir, exist_ok=True)
        
        self._total_distance = 0.0
        self._start_time_real = None

        # --- Speed estimation variables ---
        self._last_speed_calc_time = time.time()
        self._prev_human_x = None
        self._prev_human_y = None
        self._total_human_speed = 0.0
        self._speed_samples_count = 0
        self._last_avg_print_time = time.time()

        # --- Buffers ---
        self._speed_buffer = collections.deque(maxlen=5)   # smoothed speed (CSV / steady state)
        self._start_buf = collections.deque(maxlen=2)      # fast start trigger

        # --- Feed-forward velocity (filtered, body frame) ---
        self._ff_vx = 0.0
        self._ff_vy = 0.0

        # --- Integral state ---
        self._i_x = 0.0

        # --- Stop detection state ---
        self._human_speed_smooth = 0.0
        self._is_moving = False

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------
    @staticmethod
    def _soft_deadband(error, band):
        """Return a zero output inside the deadband, otherwise reduce the error by band."""
        if abs(error) <= band:
            return 0.0
        return math.copysign(abs(error) - band, error)

    @staticmethod
    def _clamp(v, lim):
        """Clamp a value to the symmetric range [-lim, lim]."""
        return max(-lim, min(lim, v))

    @staticmethod
    def _slew_asym(target, current, accel_delta, decel_delta):
        """Apply asymmetric rate limiting to move current toward target.

        Uses the deceleration limit when reversing direction or slowing down,
        otherwise uses the acceleration limit.
        """
        if (target * current < 0.0) or (abs(target) < abs(current)):
            max_delta = decel_delta
        else:
            max_delta = accel_delta
        delta = target - current
        if delta > max_delta:
            delta = max_delta
        elif delta < -max_delta:
            delta = -max_delta
        return current + delta

    def _lateral_cmd(self, human_y):
        """Compute lateral velocity command to maintain the desired side gap.

        Returns a strong evasion command when the human is too close, a gentle
        pull-in command when too far, and zero within the acceptable deadband.
        """
        target_gap = abs(self.cfg.TARGET_Y)
        gap = abs(human_y)
        side = 1.0 if human_y >= 0.0 else -1.0

        if gap < target_gap - self.cfg.DEADBAND_Y_CLOSE:
            mag = self.cfg.KP_VY_CLOSE * ((target_gap - self.cfg.DEADBAND_Y_CLOSE) - gap)
            return -side * min(mag, self.cfg.MAX_VY_CLOSE)
        elif gap > target_gap + self.cfg.DEADBAND_Y_FAR:
            mag = self.cfg.KP_VY_FAR * (gap - (target_gap + self.cfg.DEADBAND_Y_FAR))
            return side * min(mag, self.cfg.MAX_VY_FAR)
        return 0.0

    # ------------------------------------------------------------------
    # Thread management
    # ------------------------------------------------------------------
    # Start the controller thread if not already running.
    # stop_event: Event used to request loop termination.
    # daemon: Whether the thread should run as a daemon.
    def start(self, stop_event: threading.Event, daemon: bool = True) -> None:
        if self._thread and self._thread.is_alive():
            return
        self._thread = threading.Thread(
            target=self._run_loop, args=(stop_event,), daemon=daemon
        )
        self._thread.start()

    def join(self, timeout: Optional[float] = None) -> None:
        if self._thread:
            self._thread.join(timeout=timeout)

   # ------------------------------------------------------------------
    # Main loop
    # ------------------------------------------------------------------
    # Main controller loop that runs until stop_evt is set.
    # Updates follow commands, logs experiment data, performs human speed
    # estimation, lateral gap control, emergency retreat, and output shaping.
    def _run_loop(self, stop_evt: threading.Event):
        """
        Executes the main control loop for the robot's FOLLOW behavior.

        This function continuously monitors the human target's estimated distance
        and orientation, calculates relative velocities, and applies PI control
        with feed-forward to maintain a target position. It also includes safety
        mechanisms like emergency retreat, and logs telemetry data to a CSV file
        at a fixed 10Hz rate.

        Args:
            stop_evt (threading.Event): Event flag used to gracefully terminate the loop.
        """
        print("[FOLLOW] Controller started: lateral distance-keeping (sideways evasion) + "
              "fast-start detection + human-paced cap + rotation-compensated speed.")

        # Initialize previous velocity commands for acceleration slewing
        last_vx, last_vy, last_wz = 0.0, 0.0, 0.0

        # Setup logging metadata and construct the CSV file path
        timestamp_str = time.strftime("%Y%m%d_%H%M%S")
        start_time_str = time.strftime("%Y-%m-%d %H:%M:%S")
        csv_filename = os.path.join(self.experiment_dir, f"human_speed_log_{timestamp_str}.csv")

        # Track experiment duration and total estimated human travel distance
        self._start_time_real = time.time()
        self._total_distance = 0.0

        try:
            # Open CSV file for logging experiment data
            with open(csv_filename, mode='w', newline='') as csv_file:
                csv_writer = csv.writer(csv_file)
                # Write CSV headers
                csv_writer.writerow(["timestamp", "speed_mps", "elapsed_seconds", "total_distance_m"])
                print(f"[FOLLOW] Logging EXPERIMENT data (10Hz) to: {csv_filename}")

                # Main execution loop
                while not stop_evt.is_set():
                    # Only execute follow logic if the current behavior mode is "FOLLOW"
                    if self.behavior.get("mode") == "FOLLOW":
                        # Retrieve human target state from the state manager
                        dis = getattr(self.state_manager.remote_state, "distance_est", None)
                        ori = getattr(self.state_manager.remote_state, "orientation_est", None)

                        # Check for invalid tracking data or extreme proximity (target lost/too close)
                        if dis is None or ori is None or dis <= 0.1:
                            raw_vx, raw_vy, raw_wz = 0.0, 0.0, 0.0
                            self._i_x = 0.0
                            self._ff_vx = 0.0
                            self._ff_vy = 0.0
                            self._is_moving = False
                        else:
                            # Convert polar coordinates (distance, orientation) to Cartesian (x, y) relative to robot
                            human_x = dis * math.cos(ori)
                            human_y = dis * math.sin(ori)

                            # ----------------------------------------------------
                            # Speed / feed-forward estimation (strictly 10 Hz)
                            # ----------------------------------------------------
                            current_time = time.time()
                            dt_calc = current_time - self._last_speed_calc_time

                            # Execute velocity estimation logic roughly every 100ms
                            if dt_calc >= 0.10:
                                if self._prev_human_x is not None and self._prev_human_y is not None:
                                    # Get current robot velocities
                                    robot_vx = self.behavior.get("vx", 0.0)
                                    robot_vy = self.behavior.get("vy", 0.0)
                                    robot_wz = self.behavior.get("wz", 0.0)

                                    # Compensate for robot's own ego-motion to calculate apparent target velocity
                                    app_vx = -robot_vx + self.cfg.YAW_COMP_SIGN * robot_wz * self._prev_human_y
                                    app_vy = -robot_vy - self.cfg.YAW_COMP_SIGN * robot_wz * self._prev_human_x

                                    # Predict where the human should be if they were stationary
                                    expected_human_x = self._prev_human_x + app_vx * dt_calc
                                    expected_human_y = self._prev_human_y + app_vy * dt_calc

                                    # Difference between actual and expected position gives human's true movement
                                    dx_human = human_x - expected_human_x
                                    dy_human = human_y - expected_human_y

                                    # Calculate instantaneous human velocities
                                    human_vel_x = dx_human / dt_calc
                                    human_vel_y = dy_human / dt_calc

                                    human_speed_raw = math.hypot(human_vel_x, human_vel_y)
                                    human_speed_smooth = self._human_speed_smooth

                                    # Filter out unrealistic speed spikes
                                    if human_speed_raw < 5.0:
                                        # Update moving average buffers
                                        self._speed_buffer.append(human_speed_raw)
                                        self._start_buf.append(human_speed_raw)
                                        human_speed_smooth = sum(self._speed_buffer) / len(self._speed_buffer)
                                        self._human_speed_smooth = human_speed_smooth

                                        # Apply exponential moving average to feed-forward velocities
                                        a = self.cfg.FF_ALPHA
                                        self._ff_vx = a * human_vel_x + (1.0 - a) * self._ff_vx
                                        self._ff_vy = a * human_vel_y + (1.0 - a) * self._ff_vy

                                        # Update moving state based on speed thresholds
                                        if self._human_speed_smooth > self.cfg.MOVE_SPEED:
                                            self._is_moving = True
                                        elif self._human_speed_smooth < self.cfg.STOP_SPEED:
                                            self._is_moving = False

                                        # Fast-start detection: if buffer is full of high speeds, force moving state
                                        if (len(self._start_buf) == self._start_buf.maxlen and
                                                all(s > self.cfg.MOVE_SPEED for s in self._start_buf)):
                                            self._is_moving = True

                                        # Accumulate totals for calculating overall averages
                                        self._total_human_speed += human_speed_smooth
                                        self._speed_samples_count += 1
                                        
                                        # Update Distance and Time for CSV logging
                                        elapsed_seconds = current_time - self._start_time_real
                                        step_distance = human_speed_smooth * dt_calc
                                        self._total_distance += step_distance

                                        # Write instantaneous metrics to log file
                                        csv_writer.writerow([
                                            current_time, 
                                            human_speed_smooth, 
                                            f"{elapsed_seconds:.2f}", 
                                            f"{self._total_distance:.2f}"
                                        ])
                                        csv_file.flush()

                                    # Print debugging statistics every 5 seconds
                                    if current_time - self._last_avg_print_time >= 5.0:
                                        if self._speed_samples_count > 0:
                                            avg_speed = self._total_human_speed / self._speed_samples_count
                                            state = "MOVING" if self._is_moving else "STOPPED"
                                            print(f"\n[DEBUG] {state} | Smooth Speed: {human_speed_smooth:.2f} m/s "
                                                  f"| 5sec Avg: {avg_speed:.2f} m/s | gap={abs(human_y):.2f} m\n")
                                        self._last_avg_print_time = current_time

                                # Update previous state variables for the next iteration
                                self._prev_human_x = human_x
                                self._prev_human_y = human_y
                                self._last_speed_calc_time = current_time
                            # ----------------------------------------------------

                            # Calculate forward error with soft deadband to prevent micro-oscillations
                            err_x = self._soft_deadband(human_x - self.cfg.TARGET_X, self.cfg.DEADBAND_X)

                            if self._is_moving:
                                # Update integral term for forward velocity with anti-windup clamping
                                self._i_x = self._clamp(self._i_x + err_x * self.cfg.FOLLOW_DT, self.cfg.I_CLAMP)

                                # Apply feed-forward gains
                                ff_vx = self.cfg.FF_GAIN * self._ff_vx
                                ff_vy = self.cfg.FF_GAIN * self._ff_vy

                                # Calculate raw forward velocity (Feed-forward + Proportional + Integral)
                                raw_vx = ff_vx + self.cfg.KP_VX * err_x + self.cfg.KI_X * self._i_x
                                
                                # Cap forward velocity based on human speed to prevent overshooting
                                forward_cap = min(self.cfg.MAX_VX_FOLLOW,
                                                  self._human_speed_smooth + self.cfg.CORRECTION_HEADROOM)
                                raw_vx = self._clamp(raw_vx, forward_cap)

                                # Calculate lateral evasion velocity and clamp to max limits
                                raw_vy = self._clamp(ff_vy + self._lateral_cmd(human_y), self.cfg.MAX_VY_FOLLOW)

                                # Calculate desired travel heading relative to current feed-forward velocities
                                travel_dir = math.atan2(self._ff_vy, self._ff_vx)
                                travel_dir = (travel_dir + math.pi) % (2 * math.pi) - math.pi
                                
                                # Proportional control for rotational velocity (yaw)
                                heading_err = self._soft_deadband(travel_dir, self.cfg.DEADBAND_WZ)
                                wz_calc = self.cfg.WZ_SIGN * heading_err * self.cfg.KP_WZ
                                raw_wz = math.copysign(
                                    min(abs(wz_calc), self.cfg.MAX_WZ_CORRECTION), wz_calc
                                )
                            else:
                                # Decay feed-forward values and reset integral when target stops moving
                                self._ff_vx *= 0.5
                                self._ff_vy *= 0.5
                                self._i_x = 0.0
                                raw_vx = 0.0
                                raw_wz = 0.0

                                # Allow lateral evasion even if target is stopped
                                vy_cmd = self._lateral_cmd(human_y)
                                side = 1.0 if human_y >= 0.0 else -1.0
                                # Prevent moving towards the human if already on the correct side
                                if vy_cmd * side > 0.0:
                                    vy_cmd = 0.0
                                raw_vy = vy_cmd

                            # Emergency override: Retreat if the target breaches safety distance
                            if dis < self.cfg.EMERGENCY_DIST:
                                norm = max(dis, 1e-3)
                                raw_vx = -self.cfg.RETREAT_SPEED * (human_x / norm)
                                raw_vy = -self.cfg.RETREAT_SPEED * (human_y / norm)
                                raw_wz = 0.0
                                self._i_x = 0.0

                        # Define max acceleration and deceleration steps per control tick
                        accel_dx = self.cfg.MAX_ACCEL_X * self.cfg.FOLLOW_DT
                        decel_dx = self.cfg.MAX_DECEL_X * self.cfg.FOLLOW_DT
                        accel_dy = self.cfg.MAX_ACCEL_Y * self.cfg.FOLLOW_DT
                        decel_dy = self.cfg.MAX_DECEL_Y * self.cfg.FOLLOW_DT

                        # Apply low-pass smoothing to raw commands
                        blend_vx = self.cfg.SMOOTH_ALPHA * raw_vx + (1.0 - self.cfg.SMOOTH_ALPHA) * last_vx
                        blend_vy = self.cfg.SMOOTH_ALPHA * raw_vy + (1.0 - self.cfg.SMOOTH_ALPHA) * last_vy
                        blend_wz = self.cfg.SMOOTH_ALPHA * raw_wz + (1.0 - self.cfg.SMOOTH_ALPHA) * last_wz

                        # Apply asymmetric slew rate limiting to respect mechanical acceleration constraints
                        last_vx = self._slew_asym(blend_vx, last_vx, accel_dx, decel_dx)
                        last_vy = self._slew_asym(blend_vy, last_vy, accel_dy, decel_dy)
                        last_wz = blend_wz

                        # Commit smoothed, limited velocities to behavior state
                        self.behavior["vx"] = last_vx
                        self.behavior["vy"] = last_vy
                        self.behavior["wz"] = last_wz

                    else:
                        # If not in FOLLOW mode, retain current velocities but reset follow state variables
                        last_vx = self.behavior.get("vx", 0.0)
                        last_vy = self.behavior.get("vy", 0.0)
                        last_wz = self.behavior.get("wz", 0.0)
                        self._i_x = 0.0
                        self._ff_vx = 0.0
                        self._ff_vy = 0.0
                        self._is_moving = False

                    # Sleep to maintain the control loop frequency
                    time.sleep(self.cfg.FOLLOW_DT)
                    
                # --- After the while loop stops: Write the Summary ---
                end_time_str = time.strftime("%Y-%m-%d %H:%M:%S")
                end_time_real = time.time()
                total_duration = end_time_real - self._start_time_real

                # Append final experiment summary block to the CSV
                csv_writer.writerow([])
                csv_writer.writerow(["--- EXPERIMENT SUMMARY ---", "", "", ""])
                csv_writer.writerow(["Start Time:", start_time_str, "", ""])
                csv_writer.writerow(["End Time:", end_time_str, "", ""])
                csv_writer.writerow(["Total Duration (sec):", f"{total_duration:.2f}", "", ""])
                csv_writer.writerow(["Total Distance (m):", f"{self._total_distance:.2f}", "", ""])

        except Exception as e:
            # Catch and report any runtime errors during the loop
            print(f"[FOLLOW] Exception in run loop: {e}")
        finally:
            # Ensure the user knows the file handler closed appropriately
            print("[FOLLOW] Controller stopped. CSV file closed safely.")