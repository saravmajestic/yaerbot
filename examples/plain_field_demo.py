"""Act 2 + Act 4, end to end, no hardware:

  SeedPlan -> boustrophedon path -> SimRobot executes (timed DR) -> RunLog -> SVG report.

Run:  python examples/plain_field_demo.py
Outputs runlog.json + report.svg next to this script.
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from farmos import SeedPlan, plan_boustrophedon, execute, SimRobot
from farmos.report import save_report

OUT = os.path.dirname(os.path.abspath(__file__))


def main() -> None:
    # A ~4x8 ft plot, 30 cm rows, 15 cm seed spacing — groundnut.
    cfg = SeedPlan.from_feet(
        plot_w_ft=4, plot_l_ft=8, row_gap_in=12, seed_gap_in=6,
        crop="groundnut", speed_mps=0.10,
        recommended_date="2026-08-18 (Keel Nokku Naal window)",
        rationale="Groundnut sown on the waning-moon root day per the biodynamic "
                  "calendar; soil temp and the 10-day forecast both favourable.",
    )

    path = plan_boustrophedon(cfg)
    # Slight slip + heading error so 'planted' differs from 'planned' (realistic).
    robot = SimRobot(start=(0.0, 0.0, 90.0), slip=0.03, heading_err_deg=1.5, seed=7)
    log = execute(cfg, path, robot)

    log.save(os.path.join(OUT, "runlog.json"))
    svg = save_report(log, os.path.join(OUT, "report.svg"))

    print(f"crop={log.crop}  spots={log.summary['spots']}  seeds={log.summary['seeds_total']}")
    print(f"distance={log.stats['distance_m']} m  est_run={log.stats['est_run_time_s']} s")
    print(f"planted avg spacing={log.stats['executed_spacing']['mean_gap_m']*100:.1f} cm  "
          f"max drift={log.stats['max_position_error_m']*100:.1f} cm")
    print(f"report -> {svg}")


if __name__ == "__main__":
    main()
