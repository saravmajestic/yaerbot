"""BridgeRobot self-diagnosis — a field run must flag when the robot didn't obey.

The MCU's getDiag latches each move, so these checks run AFTER the move stops
(polling mid-move would add bridge traffic inside the interval being timed).
Faults here are the ones actually seen on the bench: a loose IBT-2 lead, a
command arriving mangled, and firmware too old to answer at all.
"""
import json
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from farmos.robot_io import BridgeRobot


class DiagBridge:
    """Fake MCU. `fault` injects one specific failure into the latched move block."""

    def __init__(self, fault=None, amps=True):
        self.fault, self.amps = fault, amps
        self.n = self.stops = 0
        self.last = (0, 0)
        self.up_ms = 10_000

    def call(self, name, *args):
        if name == "getBattery":
            return '{"volts":11.90,"pct":75}'
        if name == "setMotors":
            self.n += 1
            self.last = args
            return None
        if name == "stop":
            self.stops += 1
            return None
        if name == "getDiag":
            if self.fault == "absent":
                raise RuntimeError("method not available")
            if self.fault == "notjson":
                return None
            l, r = self.last
            if self.fault == "mangled":
                l -= 40                       # bridge delivered something else
            pins = [max(0, l), max(0, -l), max(0, r), max(0, -r)]
            if self.fault == "pins":
                pins[0] = 0                   # firmware wrote the wrong pin
            if self.fault == "reset":
                self.up_ms = 5                # MCU rebooted mid-run
            if self.fault == "frozen":
                self.n = 1                    # setMotors never got through
            d = {
                "up_ms": self.up_ms,
                "cmd": {"n": self.n, "ms_ago": 300},
                "move": {"req_l": l, "req_r": r, "app_l": l, "app_r": r,
                         "l_rpwm": pins[0], "l_lpwm": pins[1],
                         "r_rpwm": pins[2], "r_lpwm": pins[3]},
                "stops": self.stops,
                "batt": {"raw": 7412.0, "volts": 11.90},
            }
            if self.amps:
                d["amps"] = {"l_avg": 0.01 if self.fault == "deadleft" else 1.28,
                             "l_max": 2.10, "r_avg": 1.31, "r_max": 2.22, "n": 840}
            return json.dumps(d)
        return None


def run(fault=None, amps=True):
    """One forward hop; returns (robot, warnings)."""
    fb = DiagBridge(fault, amps)
    robot = BridgeRobot(speed_mps=0.82, pwm=180, turn_deg_per_s=123.0,
                        settle_s=0.0, nominal_volts=11.9)
    robot._bridge = fb
    real_sleep = time.sleep
    time.sleep = lambda _s: None
    try:
        robot.forward(0.4)
    finally:
        time.sleep = real_sleep
    return robot, robot.warnings


def test_healthy_run_raises_nothing():
    robot, warns = run()
    assert warns == []
    assert len(robot.diag_log) == 1
    assert robot.diag_log[0]["_sent"] == [164, 180]   # round(180*0.91), 180*1.0


def test_dead_side_is_caught():
    """The bench fault: duty commanded, no current drawn — a loose IBT-2 lead."""
    _, warns = run("deadleft")
    assert len(warns) == 1
    assert "LEFT commanded 164 but drew 0.01A" in warns[0]
    assert "IBT-2" in warns[0]


def test_command_arriving_mangled_is_caught():
    _, warns = run("mangled")
    assert any("MCU received 124/180 but we sent 164/180" in w for w in warns)


def test_pins_disagreeing_with_duty_is_caught():
    _, warns = run("pins")
    assert any("driver pins" in w and "!= applied duty" in w for w in warns)


def test_missing_getdiag_warns_once_then_stops_retrying():
    robot, warns = run("absent")
    assert len(warns) == 1 and "unavailable" in warns[0]
    assert robot.diag is False        # old firmware: don't spam every hop
    assert robot.diag_log == []


def test_unusable_payload_does_not_crash_the_run():
    """A firmware answering with None must degrade, not take the field run down."""
    robot, warns = run("notjson")
    assert robot.diag is False
    assert any("not an object" in w for w in warns)


def test_mcu_reset_midrun_is_caught():
    fb = DiagBridge()
    robot = BridgeRobot(speed_mps=0.82, pwm=180, settle_s=0.0, nominal_volts=11.9)
    robot._bridge = fb
    real_sleep = time.sleep
    time.sleep = lambda _s: None
    try:
        robot.forward(0.4)
        fb.fault = "reset"            # uptime jumps backwards on the second hop
        robot.forward(0.4)
    finally:
        time.sleep = real_sleep
    assert any("MCU RESET" in w for w in robot.warnings)


def test_no_current_sense_means_no_false_alarm():
    """Un-wired IS pins must stay silent, not report every side as dead."""
    _, warns = run(amps=False)
    assert warns == []
