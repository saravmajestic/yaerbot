"""SeedPlan — the config that drives a plain-land seeding run.

All distances are in metres (canonical). Use SeedPlan.from_feet(...) if you prefer to
think in feet. The planner (Act 1) can fill in crop / recommended_date / rationale; the
path planner and executor only need the geometry + speed.
"""
from __future__ import annotations

from dataclasses import dataclass, asdict

FEET_TO_M = 0.3048


@dataclass
class SeedPlan:
    crop: str = "groundnut"

    # Plot geometry (a rectangle). Rows run along the LENGTH; the robot snakes across
    # the WIDTH, one row per row_gap.
    plot_w_m: float = 1.20     # width  — across rows
    plot_l_m: float = 2.40     # length — along rows
    row_gap_m: float = 0.30    # spacing between adjacent rows
    seed_gap_m: float = 0.15   # spacing between seeds within a row

    # Inset from the plot edge to the first row / first seed. If None, defaults to half
    # the relevant gap (so seeds sit centred in their cells, never on the boundary).
    edge_margin_m: float | None = None

    # Drive speed used to convert distance -> time for the timed dead-reckoning executor.
    # Calibrate this on the actual plot at demo speed (seconds per metre = 1/speed_mps).
    speed_mps: float = 0.10

    seeds_per_spot: int = 1

    # Optional context handed down from the planner (Act 1) -> shown on the report.
    recommended_date: str = ""
    rationale: str = ""

    @classmethod
    def from_feet(cls, *, plot_w_ft: float, plot_l_ft: float,
                  row_gap_in: float, seed_gap_in: float, **kw) -> "SeedPlan":
        """Convenience: build a plan from feet (plot) + inches (spacings)."""
        return cls(
            plot_w_m=plot_w_ft * FEET_TO_M,
            plot_l_m=plot_l_ft * FEET_TO_M,
            row_gap_m=row_gap_in * 0.0254,
            seed_gap_m=seed_gap_in * 0.0254,
            **kw,
        )

    def __post_init__(self) -> None:
        for name in ("plot_w_m", "plot_l_m", "row_gap_m", "seed_gap_m", "speed_mps"):
            if getattr(self, name) <= 0:
                raise ValueError(f"{name} must be > 0 (got {getattr(self, name)})")
        if self.seeds_per_spot < 1:
            raise ValueError("seeds_per_spot must be >= 1")

    @property
    def row_inset_m(self) -> float:
        return self.edge_margin_m if self.edge_margin_m is not None else self.row_gap_m / 2

    @property
    def seed_inset_m(self) -> float:
        return self.edge_margin_m if self.edge_margin_m is not None else self.seed_gap_m / 2

    def to_dict(self) -> dict:
        return asdict(self)
