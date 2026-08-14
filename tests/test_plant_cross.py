"""plant_cross — the 4-seed cross fired from ONE stop (Act 3, and the `cycle` test).

The arm carries 2 outlets 180 deg apart and a SINGLE solenoid fires both, so each
plantSeed drops 2 seeds and angles=(0, 90) gives 0/180 then 90/270. What matters and
is easy to get wrong: the spool must ARRIVE before the drop (index, dwell, then plant —
never plant then index), and the arm must be left flat at 0 or it fouls the ground on
the next hop.
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from farmos.robot_io import BridgeRobot, SimRobot


class SpyBridge:
    """Records the RPC sequence so ordering can be asserted, not just the count."""

    def __init__(self):
        self.calls: list[tuple] = []

    def call(self, name, *args):
        self.calls.append((name,) + args)
        if name == "getBattery":
            return '{"volts":12.50,"pct":95}'
        return None

    def names(self, *want):
        return [c for c in self.calls if c[0] in want]


def robot(bridge, **kw):
    r = BridgeRobot(speed_mps=0.6, turn_deg_per_s=50, batt_comp=False, diag=False,
                    settle_s=0.0, **kw)
    r._bridge = bridge
    return r


def test_four_seeds_from_two_arm_positions():
    b = SpyBridge()
    assert robot(b).plant_cross((0, 90), dwell_s=0.0) == 4
    assert len(b.names("plantSeed")) == 2       # 2 drops x 2 outlets = 4 seeds


def test_spool_is_indexed_before_every_drop():
    b = SpyBridge()
    robot(b).plant_cross((0, 90), dwell_s=0.0)
    seq = [c[0] for c in b.names("indexSpool", "plantSeed")]
    # index, plant, index, plant, index(home) — a plant must never precede its index
    assert seq == ["indexSpool", "plantSeed", "indexSpool", "plantSeed", "indexSpool"]
    assert [c[1] for c in b.names("indexSpool")] == [0, 90, 0]


def test_arm_left_flat_for_driving():
    b = SpyBridge()
    robot(b).plant_cross((90, 0), dwell_s=0.0)
    assert b.names("indexSpool")[-1] == ("indexSpool", 0)


def test_angles_wrap_into_the_servo_range():
    """270 is reached by the OPPOSITE outlet at 90 — the S3003 only travels 0..180."""
    b = SpyBridge()
    robot(b).plant_cross((180, 270), dwell_s=0.0)
    assert [c[1] for c in b.names("indexSpool")] == [0, 90, 0]


def test_disarmed_moves_the_spool_but_never_fires_the_solenoid():
    b = SpyBridge()
    seeds = robot(b, plant_enabled=False).plant_cross((0, 90), dwell_s=0.0)
    assert b.names("plantSeed") == []
    assert seeds == 4                    # still reports the intended count for the dry run
    assert len(b.names("indexSpool")) == 3


def test_records_one_planting_per_arm_position_at_the_stop():
    b = SpyBridge()
    r = robot(b)
    r.x, r.y = 1.0, 2.0
    r.plant_cross((0, 90), dwell_s=0.0)
    assert r.planted == [(1.0, 2.0), (1.0, 2.0)]   # same spot, cross around it


def test_sim_matches_the_bridge_seed_count():
    """The offline demo must report the same seeds as the hardware would drop."""
    assert SimRobot().plant_cross((0, 90)) == 4
