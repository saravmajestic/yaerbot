import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from farmos import SeedPlan, plan_boustrophedon
from farmos.path import row_xs, seed_ys, plan_summary


def test_counts_and_bounds():
    cfg = SeedPlan(plot_w_m=1.2, plot_l_m=2.4, row_gap_m=0.3, seed_gap_m=0.15)
    xs, ys = row_xs(cfg), seed_ys(cfg)
    # 1.2 m wide, 0.3 gap, half-inset -> rows at 0.15,0.45,0.75,1.05 -> 4 rows
    assert len(xs) == 4
    # 2.4 m long, 0.15 gap, half-inset -> 16 seeds per row
    assert len(ys) == 16
    for x in xs:
        assert 0 < x < cfg.plot_w_m
    for y in ys:
        assert 0 < y < cfg.plot_l_m


def test_boustrophedon_order():
    cfg = SeedPlan(plot_w_m=0.9, plot_l_m=0.9, row_gap_m=0.3, seed_gap_m=0.3)
    path = plan_boustrophedon(cfg)
    ys = [round(y, 4) for y in seed_ys(cfg)]   # match the waypoints' rounding
    n = len(ys)
    row0 = [w.y for w in path[:n]]
    row1 = [w.y for w in path[n:2 * n]]
    assert row0 == ys                 # row 0 ascending
    assert row1 == list(reversed(ys)) # row 1 descending (snake)


def test_summary_matches_grid():
    cfg = SeedPlan(plot_w_m=1.2, plot_l_m=2.4, row_gap_m=0.3, seed_gap_m=0.15, seeds_per_spot=2)
    s = plan_summary(cfg)
    assert s["spots"] == 4 * 16
    assert s["seeds_total"] == 4 * 16 * 2


def test_all_waypoints_are_plant_points():
    cfg = SeedPlan()
    path = plan_boustrophedon(cfg)
    assert path and all(w.plant for w in path)


if __name__ == "__main__":
    for name, fn in sorted(globals().items()):
        if name.startswith("test_") and callable(fn):
            fn(); print(f"ok  {name}")
    print("all path tests passed")


def test_count_never_overruns_the_plot():
    """Rounding UP put the last point past the plot edge — in the field that means
    driving into the headland with no room to turn. Short is the safe error."""
    # 5.0 m at 0.3 m: round() gave 18 points ending at 5.10 m, outside the plot
    cfg = SeedPlan(plot_w_m=5.0, plot_l_m=5.0, row_gap_m=0.3, seed_gap_m=0.3,
                   edge_margin_m=0.0)
    for y in seed_ys(cfg):
        assert y <= cfg.plot_l_m, f"seed at {y} is outside a {cfg.plot_l_m} m plot"
    for x in row_xs(cfg):
        assert x <= cfg.plot_w_m, f"row at {x} is outside a {cfg.plot_w_m} m plot"


def test_exact_multiples_are_not_shortchanged():
    """The float guard: 4.8/0.4 is 11.9999... in binary, so a bare int() would drop
    the last legitimate seed off a row whose length IS an exact multiple of the gap."""
    cfg = SeedPlan(plot_w_m=1.2, plot_l_m=4.8, row_gap_m=0.4, seed_gap_m=0.4,
                   edge_margin_m=0.0)
    ys = seed_ys(cfg)
    assert len(ys) == 13          # 0.0 .. 4.8 inclusive
    assert abs(ys[-1] - 4.8) < 1e-9, f"last seed at {ys[-1]}, expected the edge at 4.8"
