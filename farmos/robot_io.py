"""RobotIO — the actuation interface the executor drives, with two implementations:

  SimRobot    — pure-Python dead-reckoning simulator (for tests + the offline demo).
                Models optional wheel slip / heading error so "executed" positions can
                differ from "planned", which is exactly what the report visualises.
  BridgeRobot — real UNO Q adapter: forward/turn become timed setMotors calls, plant
                calls the plantSeed RPC. Untested on hardware yet (no robot in this env);
                the Bridge import is lazy so this module stays importable off-device.

Heading is in degrees, math convention: 0 = +x, 90 = +y (CCW positive).
"""
from __future__ import annotations

import math
import time
from typing import Protocol, runtime_checkable


@runtime_checkable
class RobotIO(Protocol):
    def turn_to(self, heading_deg: float) -> None: ...
    def forward(self, distance_m: float) -> None: ...
    def plant(self) -> None: ...
    def stop(self) -> None: ...


class SimRobot:
    """Dead-reckoning simulator. With slip=0 and heading_err=0 it's exact (ideal DR)."""

    def __init__(self, start=(0.0, 0.0, 90.0), *, slip: float = 0.0,
                 heading_err_deg: float = 0.0, seed: int = 0):
        import random
        self.x, self.y, self.heading = start
        self.slip = slip                      # fractional distance error, e.g. 0.03 = 3%
        self.heading_err_deg = heading_err_deg
        self._rng = random.Random(seed)
        self.planted: list[tuple[float, float]] = []
        self.path_trace: list[tuple[float, float]] = [(self.x, self.y)]
        self.distance_travelled = 0.0

    def turn_to(self, heading_deg: float) -> None:
        err = self._rng.uniform(-self.heading_err_deg, self.heading_err_deg)
        self.heading = heading_deg + err

    def forward(self, distance_m: float) -> None:
        d = distance_m * (1.0 + self._rng.uniform(-self.slip, self.slip))
        rad = math.radians(self.heading)
        self.x += d * math.cos(rad)
        self.y += d * math.sin(rad)
        self.distance_travelled += abs(d)
        self.path_trace.append((self.x, self.y))

    def plant(self) -> None:
        self.planted.append((self.x, self.y))

    def plant_cross(self, angles=(0, 90), dwell_s: float = 0.0) -> int:
        """Mirror of BridgeRobot.plant_cross: 2 seeds per arm position, same spot."""
        for _ in angles:
            self.planted.append((self.x, self.y))
        return 2 * len(tuple(angles))

    def stop(self) -> None:
        pass


class BridgeRobot:
    """Real UNO Q adapter — open-loop timed dead reckoning over RouterBridge RPCs.

    Tracks the *commanded* pose (no encoders, so this is the intended path, not measured),
    which is what the open-loop report shows. Fires setMotors/stop/plantSeed on the MCU.
    Calibrate `speed_mps` and `turn_deg_per_s` on the actual surface first (field_test.py).
    Set plant_enabled=False to drive the path without firing the seeder (dry run).

    Battery compensation: a sagging pack drives the same command a shorter distance (we
    measured ~half distance at 46%), so every move re-reads the pack and scales the duty
    to hold the *calibrated* speed. See `_gain()`.
    """

    def __init__(self, *, speed_mps: float, pwm: int = 120, turn_deg_per_s: float = 90.0,
                 start=(0.0, 0.0, 90.0), plant_enabled: bool = True, settle_s: float = 0.3,
                 left_trim: float = 0.91, right_trim: float = 1.00, startup_s: float = 0.5,
                 turn_startup_s: float = 0.75, turn_pwm: int | None = None,
                 turn_ramp_s: float = 0.0, turn_ramp_steps: int = 3,
                 turn_ramp_floor: float = 0.6,
                 batt_comp: bool = True, nominal_volts: float | None = None,
                 max_pwm: int = 255, max_gain: float = 1.5, min_gain: float = 0.75,
                 volts_ttl_s: float = 3.0, volts_samples: int = 3, diag: bool = True):
        self.speed_mps, self.pwm, self.turn_deg_per_s = speed_mps, pwm, turn_deg_per_s
        # dead-time model: a short hop wastes ~startup_s accelerating from a stop, so
        # forward time = startup_s + distance/cruise_speed. Without this, short seeding hops
        # (0.3-0.5 m) barely translate and the robot just spins through its turns.
        self.startup_s = startup_s
        # Turn timing offset. MAY BE NEGATIVE: a skid-steer turn is low-friction, so the
        # chassis keeps spinning after the brake — measured ~30 deg of coast at PWM 120
        # (and ~75 deg at PWM 180, which is why turns use a lower duty). A coast is a
        # negative offset, the opposite sign to forward's dead time.
        self.turn_startup_s = turn_startup_s
        # Turns get their own duty: high duty builds angular momentum the brake can't
        # absorb, so turning slower is what makes the stop angle repeatable.
        self.turn_pwm = pwm if turn_pwm is None else turn_pwm
        # Deceleration ramp at the END of a turn. Coast scales with how fast the chassis
        # is spinning when we brake, so easing off first shrinks both the coast and its
        # run-to-run scatter. There is still only ONE stop, and it happens slowly — which
        # is the point: stopping twice would just add a second coast to guess at.
        # Calibration is unchanged (tsolve on total commanded time) as long as the ramp
        # is fixed, since it just folds into the measured offset.
        self.turn_ramp_s = turn_ramp_s          # 0 = disabled (hard brake, old behaviour)
        self.turn_ramp_steps = max(1, turn_ramp_steps)
        self.turn_ramp_floor = turn_ramp_floor  # final duty as a fraction of turn_pwm
        self.left_trim, self.right_trim = left_trim, right_trim   # straighten drive (console values)
        self.x, self.y, self.heading = start
        self.plant_enabled = plant_enabled
        self.settle_s = settle_s                 # brief pause after each move (reduce overshoot)
        self.batt_comp = batt_comp
        self.nominal_volts = nominal_volts       # pack volts the speed/turn calibration was done at
        self.max_pwm, self.max_gain, self.min_gain = max_pwm, max_gain, min_gain
        self.volts_ttl_s = volts_ttl_s           # reuse one reading across a turn+forward pair
        self.volts_samples = volts_samples       # median-filter the ADC (see volts())
        self.planted: list[tuple[float, float]] = []
        self._volts_cache: tuple[float, float] | None = None   # (monotonic_t, volts)
        self._bridge = None
        self.diag = diag                     # capture an MCU snapshot after each move
        self.diag_log: list[dict] = []
        self.warnings: list[str] = []        # anomalies found by _check_diag
        self._sent: tuple[int, int] = (0, 0)  # last duty we asked for
        self._prev: dict | None = None       # previous snapshot (counter deltas)

    def _b(self):
        if self._bridge is None:
            from arduino.app_utils import Bridge  # lazy: only exists on the board
            self._bridge = Bridge
        return self._bridge

    def volts(self) -> float | None:
        """Resting pack voltage, read between moves (motors off = comparable to calibration).

        Median of a few samples: the reading is normally stable to ~0.04 V but we have seen
        a lone ~0.7 V outlier, and one bad sample would skew that hop's duty by ~6%.
        Cached for volts_ttl_s. Returns None if the RPC is unavailable — compensation then
        stays neutral rather than taking the run down with it.
        """
        now = time.monotonic()
        if self._volts_cache and now - self._volts_cache[0] < self.volts_ttl_s:
            return self._volts_cache[1]
        xs = []
        for _ in range(self.volts_samples):
            try:
                r = self._b().call("getBattery")
                if isinstance(r, (str, bytes)):
                    import json
                    r = json.loads(r)
                v = float(r["volts"])
            except Exception:
                continue
            if v > 0:
                xs.append(v)
        if not xs:
            return self._volts_cache[1] if self._volts_cache else None
        v = sorted(xs)[len(xs) // 2]
        self._volts_cache = (now, v)
        return v

    def _gain(self) -> float:
        """Duty multiplier that cancels the pack's drift from the calibration voltage.

        Motor speed at a fixed duty tracks supply volts, so scaling duty by V_cal/V holds
        the calibrated speed. Symmetric on purpose: a *sagged* pack gets boosted, but a
        pack fresher than V_cal gets backed off, otherwise calibrating at 76% and running
        on a full pack would overshoot every hop. Clamped both ways so one bad ADC read
        can't bolt the robot or stall it below the motors' deadband.
        """
        if not self.batt_comp:
            return 1.0
        v = self.volts()
        if v is None:
            return 1.0
        if self.nominal_volts is None:
            self.nominal_volts = v      # no explicit calibration volts: assume "as of now"
        return min(max(self.nominal_volts / v, self.min_gain), self.max_gain)

    def _apply(self, left: float, right: float) -> float:
        """setMotors(left, right) scaled by the battery gain; returns the time-stretch factor.

        If the boosted duty clips at max_pwm we can't restore the speed, so whatever gain
        got clipped comes back as extra drive time instead (stretch > 1).
        """
        gain = self._gain()
        left, right = left * gain, right * gain
        clip = max(abs(left), abs(right)) / self.max_pwm
        if clip > 1.0:
            left, right = left / clip, right / clip
        self._sent = (int(round(left)), int(round(right)))
        self._b().call("setMotors", *self._sent)
        return max(clip, 1.0)

    def _snap(self, tag: str) -> None:
        """Record + check an MCU snapshot. Call only AFTER a move has stopped.

        Polling mid-move would add a bridge round-trip inside the interval we time
        the move by, so every check here works off values the MCU latched itself.
        """
        if not self.diag:
            return
        try:
            r = self._b().call("getDiag")
            if isinstance(r, (str, bytes)):
                import json
                r = json.loads(r)
        except Exception as e:                       # noqa: BLE001
            self.warnings.append(f"{tag}: getDiag unavailable ({e})")
            self.diag = False                        # old firmware — stop retrying
            return
        if not isinstance(r, dict):                  # firmware answered, but not usably
            self.warnings.append(f"{tag}: getDiag returned {type(r).__name__}, not an object")
            self.diag = False
            return
        r["_tag"], r["_sent"] = tag, list(self._sent)
        self.diag_log.append(r)
        self._check_diag(tag, r)
        self._prev = r

    def _check_diag(self, tag: str, d: dict) -> None:
        """Compare what we asked for against what the MCU saw, and flag mismatches."""
        mv, prev = d.get("move", {}), self._prev
        sl, sr = self._sent

        if mv and (mv.get("req_l"), mv.get("req_r")) != (sl, sr):
            self.warnings.append(
                f"{tag}: MCU received {mv.get('req_l')}/{mv.get('req_r')} "
                f"but we sent {sl}/{sr}")
        # pins must mirror the applied duty, split by direction (forward on RPWM)
        if mv:
            want = (max(0, mv.get("app_l", 0)), max(0, -mv.get("app_l", 0)),
                    max(0, mv.get("app_r", 0)), max(0, -mv.get("app_r", 0)))
            got = (mv.get("l_rpwm"), mv.get("l_lpwm"), mv.get("r_rpwm"), mv.get("r_lpwm"))
            if got != want:
                self.warnings.append(f"{tag}: driver pins {got} != applied duty {want}")
        if prev and d.get("up_ms", 0) < prev.get("up_ms", 0):
            self.warnings.append(f"{tag}: MCU RESET mid-run (uptime went backwards)")
        if prev and d.get("cmd", {}).get("n", 0) == prev.get("cmd", {}).get("n", 0):
            self.warnings.append(f"{tag}: setMotors never reached the MCU (counter frozen)")

        a = d.get("amps")                            # only if CURRENT_SENSE is wired
        if a and a.get("n", 0) > 0:
            for side, avg, duty in (("LEFT", a.get("l_avg", 0), mv.get("app_l", 0)),
                                    ("RIGHT", a.get("r_avg", 0), mv.get("app_r", 0))):
                if duty and avg < 0.05:
                    self.warnings.append(
                        f"{tag}: {side} commanded {duty} but drew {avg}A "
                        f"— drive never reached the motor (check IBT-2 wiring)")

    def turn_to(self, heading_deg: float) -> None:
        delta = ((heading_deg - self.heading + 180) % 360) - 180  # shortest signed turn
        if abs(delta) > 0.5:
            # setMotors(-pwm, +pwm) swings CCW (heading increases); (+pwm, -pwm) is CW.
            p = self.turn_pwm
            left, right = (-p, p) if delta > 0 else (p, -p)
            stretch = self._apply(left, right)
            # clamp at 0: with a coast offset (negative turn_startup_s) any turn smaller
            # than the coast angle computes negative — it is simply not achievable at this
            # duty, so drive for zero rather than crashing on a negative sleep.
            secs = max(0.0, stretch * (self.turn_startup_s + abs(delta) / self.turn_deg_per_s))
            ramp = min(self.turn_ramp_s, secs)
            time.sleep(secs - ramp)
            if ramp > 0:                              # ease off before braking
                n = self.turn_ramp_steps
                for i in range(n):
                    f = 1.0 - (1.0 - self.turn_ramp_floor) * (i + 1) / n
                    self._apply(left * f, right * f)
                    time.sleep(ramp / n)
            self._b().call("stop")
            time.sleep(self.settle_s)
            self._snap(f"turn {delta:+.0f}deg")
        self.heading = heading_deg

    def forward(self, distance_m: float) -> None:
        stretch = self._apply(self.pwm * self.left_trim, self.pwm * self.right_trim)
        # dead-time model (see __init__); stretch covers sag we couldn't out-duty
        time.sleep(max(0.0, stretch * (self.startup_s + distance_m / self.speed_mps)))
        self._b().call("stop")
        time.sleep(self.settle_s)
        self._snap(f"fwd {distance_m:.2f}m")
        rad = math.radians(self.heading)          # integrate the commanded move
        self.x += distance_m * math.cos(rad)
        self.y += distance_m * math.sin(rad)

    def plant(self) -> None:
        if self.plant_enabled:
            self._b().call("plantSeed")
        self.planted.append((self.x, self.y))

    def plant_cross(self, angles=(0, 90), dwell_s: float = 0.6) -> int:
        """Plant at several ARM POSITIONS from one stop; returns the seed count.

        The arm carries 2 outlets 180 deg apart and ONE solenoid fires both, so each
        plantSeed drops 2 seeds. angles=(0, 90) therefore lays a 4-seed cross:
        0/180, rotate, 90/270. Angles are arm positions mod 180 — the S3003 only
        travels 0..180 and the opposite outlet supplies the +180 half.

        The arm is left back at 0 (flat) so it can't foul the ground while driving.
        """
        seeds = 0
        B = self._b()
        for a in angles:
            B.call("indexSpool", int(a) % 180)
            time.sleep(dwell_s)               # servo must ARRIVE before the drop
            if self.plant_enabled:
                B.call("plantSeed")           # punch -> drop (both outlets) -> retract
            seeds += 2
            self.planted.append((self.x, self.y))
        B.call("indexSpool", 0)               # flat for driving
        time.sleep(dwell_s)
        return seeds

    def stop(self) -> None:
        self._b().call("stop")
