import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from farmos import SeedPlan, plan_boustrophedon, execute, SimRobot
from farmos.report import render_svg, save_report


def _log():
    cfg = SeedPlan(crop="groundnut", recommended_date="2026-08-18",
                   rationale="test rationale that is long enough to wrap across lines nicely")
    return execute(cfg, plan_boustrophedon(cfg), SimRobot(slip=0.02, seed=3))


def test_svg_is_wellformed_and_sized():
    svg = render_svg(_log())
    assert svg.startswith("<svg") and svg.rstrip().endswith("</svg>")
    assert 'xmlns="http://www.w3.org/2000/svg"' in svg


def test_svg_has_one_circle_per_planted_and_planned():
    log = _log()
    svg = render_svg(log)
    # planted (filled) + planned (hollow) + a couple legend swatches
    filled = svg.count(f'fill="#3f7d3a"')
    assert filled >= len(log.executed)          # planted dots + legend swatch
    assert svg.count("<circle") >= len(log.executed) + len(log.planned)


def test_header_shows_crop_and_counts():
    log = _log()
    svg = render_svg(log)
    assert "Seeding Report" in svg and "groundnut" in svg
    assert str(log.summary["spots"]) in svg


def test_save_writes_file(tmp_path=None):
    import tempfile
    d = tempfile.mkdtemp()
    p = os.path.join(d, "report.svg")
    save_report(_log(), p)
    assert os.path.getsize(p) > 500


if __name__ == "__main__":
    for name, fn in sorted(globals().items()):
        if name.startswith("test_") and callable(fn):
            fn(); print(f"ok  {name}")
    print("all report tests passed")
