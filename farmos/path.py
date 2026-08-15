"""Boustrophedon (serpentine) path planner for plain-land fixed-spacing seeding.

Given a SeedPlan (plot rectangle + row/seed spacing), compute the ordered list of seed
positions the robot visits, snaking row by row. Consecutive waypoints are always
axis-aligned: down a row (y changes) or across to the next row (x changes), so the
timed dead-reckoning executor only ever drives straight then turns 90 degrees.

Coordinate frame: origin (0,0) at a plot corner. x = across rows (width),
y = along rows (length). Units: metres.
"""
from __future__ import annotations

from dataclasses import dataclass

from .config import SeedPlan


@dataclass
class Waypoint:
    x: float
    y: float
    plant: bool = True   # every seed position is a plant point; row-transition corners are not


def _count(span: float, inset: float, gap: float) -> int:
    """How many points fit at `gap` spacing within `span`, inset from both edges.

    ALWAYS ROUNDS DOWN. Rounding up put the last point past the usable span — in the
    field that means driving into the headland with no room left to turn around, so
    a short row is always the safe error. (span=5, gap=0.3 rounded up to 18 points,
    the last at 5.1m; flooring gives 17, the last at 4.8m.)

    The 1e-6 is not cosmetic: 5.0/0.4 evaluates to 12.4999... and 4.8/0.4 to
    11.9999... in binary floating point, so a bare int() would silently drop a
    legitimate hop off any row whose length is an exact multiple of the gap.
    """
    usable = span - 2 * inset
    if usable < 0:
        return 0
    return int(usable / gap + 1e-6) + 1


def row_xs(cfg: SeedPlan) -> list[float]:
    n = _count(cfg.plot_w_m, cfg.row_inset_m, cfg.row_gap_m)
    return [cfg.row_inset_m + i * cfg.row_gap_m for i in range(n)]


def seed_ys(cfg: SeedPlan) -> list[float]:
    n = _count(cfg.plot_l_m, cfg.seed_inset_m, cfg.seed_gap_m)
    return [cfg.seed_inset_m + j * cfg.seed_gap_m for j in range(n)]


def plan_boustrophedon(cfg: SeedPlan) -> list[Waypoint]:
    """Return seed waypoints in serpentine order (row 0 up, row 1 down, ...)."""
    xs = row_xs(cfg)
    ys = seed_ys(cfg)
    path: list[Waypoint] = []
    for i, x in enumerate(xs):
        row = ys if i % 2 == 0 else list(reversed(ys))
        for y in row:
            path.append(Waypoint(x=round(x, 4), y=round(y, 4), plant=True))
    return path


def plan_summary(cfg: SeedPlan) -> dict:
    xs, ys = row_xs(cfg), seed_ys(cfg)
    spots = len(xs) * len(ys)
    return {
        "rows": len(xs),
        "seeds_per_row": len(ys),
        "spots": spots,
        "seeds_total": spots * cfg.seeds_per_spot,
    }
