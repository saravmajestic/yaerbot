"""BridgeRobot battery compensation — a sagging pack must not shrink our distances.

No hardware here: a FakeBridge stands in for the RouterBridge and time.sleep is stubbed,
so we assert on the duty actually commanded and the drive time actually requested.
"""
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from farmos.robot_io import BridgeRobot

NOMINAL_FWD_S = 0.5 + 0.3 / 0.5     # startup_s + dist/cruise, for forward(0.3) @ 0.5 m/s
NOMINAL_TURN_S = 0.0 + 90 / 45.0    # turn_startup_s + angle/rate, for 90 deg @ 45 deg/s


class FakeBridge:
    def __init__(self, volts, fail=False):
        self.volts, self.fail, self.calls = volts, fail, []

    def call(self, name, *args):
        self.calls.append((name, *args))
        if name == "getBattery":
            if self.fail:
                raise RuntimeError("method not available")
            return {"volts": self.volts, "pct": 50}
        return None


def drive(volts, *, pwm=180, nominal=12.6, fail=False, comp=True):
    """forward(0.3) then a 90 deg turn; returns (setMotors calls, sleep durations)."""
    fb = FakeBridge(volts, fail)
    robot = BridgeRobot(speed_mps=0.5, pwm=pwm, turn_deg_per_s=45.0, startup_s=0.5,
                        turn_startup_s=0.0,   # isolated in test_turn_dead_time_is_applied
                        settle_s=0.0, nominal_volts=nominal, batt_comp=comp)
    robot._bridge = fb
    slept, real_sleep = [], time.sleep
    time.sleep = slept.append
    try:
        robot.forward(0.3)
        robot.turn_to(0.0)          # start heading is 90 -> a 90 deg turn
    finally:
        time.sleep = real_sleep
    return [c for c in fb.calls if c[0] == "setMotors"], [s for s in slept if s > 0]


def test_full_pack_is_untouched():
    motors, times = drive(12.6)
    assert motors[0] == ("setMotors", 164, 180)      # pwm * left_trim 0.91, right_trim 1.0
    assert times == [NOMINAL_FWD_S, NOMINAL_TURN_S]


def test_sag_raises_duty_and_keeps_timing():
    motors, times = drive(11.0)
    gain = 12.6 / 11.0
    assert motors[0] == ("setMotors", round(180 * 0.91 * gain), round(180 * gain))
    # turn_to(0) from heading 90 is a CW swing -> (+pwm, -pwm)
    assert motors[1] == ("setMotors", round(180 * gain), -round(180 * gain))
    # duty had headroom, so the calibrated speed is restored and time must NOT change
    assert times == [NOMINAL_FWD_S, NOMINAL_TURN_S]


def test_clipped_duty_falls_back_to_stretching_time():
    motors, times = drive(9.0, pwm=200)              # 200 * 1.4 = 280 -> over the 255 ceiling
    assert motors[0][2] == 255
    clip = (200 * 1.4) / 255
    assert abs(times[0] - clip * NOMINAL_FWD_S) < 1e-3
    assert abs(times[1] - clip * NOMINAL_TURN_S) < 1e-3


def test_gain_is_capped_so_a_bad_reading_cannot_bolt():
    motors, times = drive(6.0)                       # would ask 2.1x; max_gain clamps to 1.5
    assert motors[0][2] == 255
    assert times[0] < 1.5 * NOMINAL_FWD_S


def test_fresher_than_calibration_backs_the_duty_off():
    """Calibrating at 76% then running on a full pack must not overshoot every hop."""
    motors, times = drive(14.0)
    gain = 12.6 / 14.0                               # 0.9 -> slow down, don't run long
    assert gain < 1.0
    assert motors[0] == ("setMotors", round(180 * 0.91 * gain), round(180 * gain))
    assert times == [NOMINAL_FWD_S, NOMINAL_TURN_S]  # speed held -> timing unchanged


def test_gain_has_a_floor_so_it_cannot_stall():
    motors, _ = drive(30.0)                          # absurd high reading; min_gain 0.75
    assert motors[0] == ("setMotors", round(180 * 0.91 * 0.75), round(180 * 0.75))


def test_missing_battery_rpc_degrades_to_neutral():
    motors, times = drive(11.0, fail=True)
    assert motors[0] == ("setMotors", 164, 180)      # a dead RPC must not take the run down
    assert times == [NOMINAL_FWD_S, NOMINAL_TURN_S]


def test_comp_can_be_disabled():
    motors, times = drive(11.0, comp=False)
    assert motors[0] == ("setMotors", 164, 180)
    assert times == [NOMINAL_FWD_S, NOMINAL_TURN_S]


def test_unset_nominal_volts_is_captured_once_and_cached():
    fb = FakeBridge(11.0)
    robot = BridgeRobot(speed_mps=0.5, pwm=180, settle_s=0.0, nominal_volts=None)
    robot._bridge = fb
    real_sleep = time.sleep
    time.sleep = lambda _s: None
    try:
        robot.forward(0.3)
        robot.forward(0.3)
    finally:
        time.sleep = real_sleep
    assert robot.nominal_volts == 11.0               # "calibrated as of now"
    assert [c for c in fb.calls if c[0] == "setMotors"][0] == ("setMotors", 164, 180)
    # one TTL-cached read, median-filtered over volts_samples -> 3 RPCs total, not 6
    assert len([c for c in fb.calls if c[0] == "getBattery"]) == 3


def test_turn_dead_time_is_applied():
    """Breaking a skid-steer loose costs ~0.75s; without it a 90 deg turn barely rotates.

    Measured on the robot: 3.0s -> 270 deg and 2.0s -> 150 deg, i.e. 120 deg/s after a
    0.75s dead time. Commanding just 90/120 = 0.75s would be pure dead time -> ~0 rotation.
    """
    fb = FakeBridge(12.6)
    robot = BridgeRobot(speed_mps=0.75, pwm=180, turn_deg_per_s=120.0, turn_startup_s=0.75,
                        settle_s=0.0, nominal_volts=12.6)
    robot._bridge = fb
    slept, real_sleep = [], time.sleep
    time.sleep = slept.append
    try:
        robot.turn_to(0.0)                           # 90 deg CW from the default heading 90
    finally:
        time.sleep = real_sleep
    assert [s for s in slept if s > 0] == [0.75 + 90 / 120.0]   # 1.5s, not the naive 0.75s
    assert robot.heading == 0.0


def test_median_rejects_a_single_bad_sample():
    """A lone outlier (we saw one ~0.7V low) must not skew that hop's duty."""
    class Flaky(FakeBridge):
        def call(self, name, *args):
            if name == "getBattery":
                self.calls.append((name,))
                seq = [11.22, 11.91, 11.90]          # first sample is the bad one
                return {"volts": seq[len([c for c in self.calls if c[0] == name]) - 1]}
            return super().call(name, *args)

    fb = Flaky(0)
    robot = BridgeRobot(speed_mps=0.5, pwm=180, nominal_volts=12.6, settle_s=0.0)
    robot._bridge = fb
    assert robot.volts() == 11.90                    # median, not the 11.22 outlier


def test_json_string_payload_is_parsed():
    """getBattery answers with a JSON string over the real bridge, not a dict."""
    class StringBridge(FakeBridge):
        def call(self, name, *args):
            if name == "getBattery":
                return '{"volts":11.91,"pct":76}'
            return super().call(name, *args)

    robot = BridgeRobot(speed_mps=0.5, pwm=180, nominal_volts=12.6)
    robot._bridge = StringBridge(0)
    assert robot.volts() == 11.91


def test_turn_ramp_eases_off_before_braking():
    """Coast scales with spin speed at brake time, so duty must step down first.

    Total commanded time must be unchanged (the ramp is inside it, not added to it),
    otherwise the tsolve calibration would no longer describe the motion.
    """
    fb = FakeBridge(12.6)
    robot = BridgeRobot(speed_mps=0.616, pwm=180, turn_pwm=120, turn_deg_per_s=51.0,
                        turn_startup_s=-0.75, turn_ramp_s=0.3, turn_ramp_steps=3,
                        turn_ramp_floor=0.6, settle_s=0.0, nominal_volts=12.6,
                        diag=False)
    robot._bridge = fb
    slept, real_sleep = [], time.sleep
    time.sleep = slept.append
    try:
        robot.turn_to(0.0)                       # 90 deg CW from heading 90
    finally:
        time.sleep = real_sleep

    total_expected = -0.75 + 90 / 51.0           # 1.0147 s
    assert abs(sum(s for s in slept if s > 0) - total_expected) < 1e-6

    duties = [c[1] for c in fb.calls if c[0] == "setMotors"]
    assert duties[0] == 120                      # full turn duty while spinning up
    assert duties == sorted(duties, reverse=True)  # monotonically eased off
    assert duties[-1] == round(120 * 0.6)        # ends at the floor, then brakes
    assert fb.calls[-1] == ("stop",)


def test_turn_ramp_off_by_default():
    """Existing calibrations must not change behaviour until the ramp is opted into."""
    fb = FakeBridge(12.6)
    robot = BridgeRobot(speed_mps=0.616, pwm=180, turn_pwm=120, turn_deg_per_s=51.0,
                        turn_startup_s=-0.75, settle_s=0.0, nominal_volts=12.6)
    robot._bridge = fb
    real_sleep = time.sleep
    time.sleep = lambda _s: None
    try:
        robot.turn_to(0.0)
    finally:
        time.sleep = real_sleep
    assert [c[1] for c in fb.calls if c[0] == "setMotors"] == [120]   # one duty, no ramp
