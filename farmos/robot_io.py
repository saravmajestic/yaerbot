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

    def stop(self) -> None:
        pass


class BridgeRobot:
    """Real UNO Q adapter — timed dead reckoning over RouterBridge RPCs.

    NOTE: untested on hardware (built alongside the offline modules). Calibrate
    `pwm`, `turn_deg_per_s`, and speed on the actual plot before a real run.
    """

    def __init__(self, *, speed_mps: float, pwm: int = 120,
                 turn_deg_per_s: float = 90.0):
        self.speed_mps = speed_mps
        self.pwm = pwm
        self.turn_deg_per_s = turn_deg_per_s
        self.heading = 90.0
        self._bridge = None

    def _b(self):
        if self._bridge is None:
            from arduino.app_utils import Bridge  # lazy: only exists on the board
            self._bridge = Bridge
        return self._bridge

    def turn_to(self, heading_deg: float) -> None:
        import time
        delta = ((heading_deg - self.heading + 180) % 360) - 180  # shortest signed turn
        dur = abs(delta) / self.turn_deg_per_s
        left, right = (-self.pwm, self.pwm) if delta > 0 else (self.pwm, -self.pwm)
        self._b().call("setMotors", left, right)
        time.sleep(dur)
        self._b().call("stop")
        self.heading = heading_deg

    def forward(self, distance_m: float) -> None:
        import time
        self._b().call("setMotors", self.pwm, self.pwm)
        time.sleep(distance_m / self.speed_mps)
        self._b().call("stop")

    def plant(self) -> None:
        self._b().call("plantSeed")

    def stop(self) -> None:
        self._b().call("stop")
