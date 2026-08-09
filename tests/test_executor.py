import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from farmos import SeedPlan, plan_boustrophedon, execute, SimRobot


def test_ideal_execution_matches_plan_exactly():
    cfg = SeedPlan(plot_w_m=1.2, plot_l_m=2.4, row_gap_m=0.3, seed_gap_m=0.15)
    path = plan_boustrophedon(cfg)
    robot = SimRobot(slip=0.0, heading_err_deg=0.0)   # perfect DR
    log = execute(cfg, path, robot)

    assert len(log.executed) == len(log.planned) == cfg_spots(cfg)
    # with zero slip, executed ≈ planned
    assert log.stats["max_position_error_m"] < 1e-6


def test_planted_count_and_seeds_per_spot():
    cfg = SeedPlan(plot_w_m=0.9, plot_l_m=0.9, row_gap_m=0.3, seed_gap_m=0.3, seeds_per_spot=3)
    path = plan_boustrophedon(cfg)
    robot = SimRobot()
    log = execute(cfg, path, robot)
    # executed collapses seeds_per_spot repeats -> one position per spot
    assert len(log.executed) == log.summary["spots"]
    assert log.summary["seeds_total"] == log.summary["spots"] * 3


def test_slip_creates_bounded_drift():
    cfg = SeedPlan()
    path = plan_boustrophedon(cfg)
    robot = SimRobot(slip=0.03, heading_err_deg=1.5, seed=1)
    log = execute(cfg, path, robot)
    # drift is present but small (cm-scale), not exploding
    assert 0.0 < log.stats["max_position_error_m"] < 0.5


def test_distance_and_time_positive():
    cfg = SeedPlan(speed_mps=0.1)
    log = execute(cfg, plan_boustrophedon(cfg), SimRobot())
    assert log.stats["distance_m"] > 0
    assert log.stats["est_run_time_s"] > 0


def cfg_spots(cfg):
    from farmos.path import plan_summary
    return plan_summary(cfg)["spots"]


if __name__ == "__main__":
    for name, fn in sorted(globals().items()):
        if name.startswith("test_") and callable(fn):
            fn(); print(f"ok  {name}")
    print("all executor tests passed")
