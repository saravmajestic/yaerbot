"""All four acts, offline, end to end:

  Act 1 (plan): crop -> reconcile panchangam + biodynamic -> recommended date + SeedPlan
  Act 2 (seed): SeedPlan -> boustrophedon path -> SimRobot timed-DR execution -> RunLog
  Act 4 (report): RunLog -> SVG farm map (carrying the Act-1 date + rationale)

Run:  python examples/full_demo.py
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from farmos.planner import recommend
from farmos import plan_boustrophedon, execute, SimRobot
from farmos.report import save_report

OUT = os.path.dirname(os.path.abspath(__file__))


def main() -> None:
    # ── Act 1: planner reconciles both calendars ──
    rec = recommend("groundnut", after="2026-08-10")
    print("=== Act 1: Planner ===")
    print(f"crop: {rec.crop}")
    print(f"recommended date: {rec.recommended_date}  (both systems agree: {rec.both_agree})")
    print(f"  panchangam: {rec.nakshatra} ({rec.nakshatra_tamil}) -> {rec.nokku} nokku")
    print(f"  biodynamic: {rec.biodynamic} day")
    print(f"  alternatives: {rec.alternatives}")
    print(f"rationale: {rec.rationale}\n")

    # ── Act 2: plan + execute the seeding ──
    cfg = rec.to_seed_plan(plot_w_m=1.2, plot_l_m=2.4, speed_mps=0.10)
    path = plan_boustrophedon(cfg)
    robot = SimRobot(slip=0.03, heading_err_deg=1.5, seed=7)
    log = execute(cfg, path, robot)
    print("=== Act 2: Seeding (simulated) ===")
    print(f"spots={log.summary['spots']}  seeds={log.summary['seeds_total']}  "
          f"distance={log.stats['distance_m']} m  avg spacing="
          f"{log.stats['executed_spacing']['mean_gap_m']*100:.1f} cm\n")

    # ── Act 4: report ──
    svg = save_report(log, os.path.join(OUT, "full_report.svg"))
    print("=== Act 4: Report ===")
    print(f"report -> {svg}")


if __name__ == "__main__":
    main()
