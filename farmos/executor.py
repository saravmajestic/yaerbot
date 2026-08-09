"""Executor — drive a boustrophedon path over a RobotIO, logging planned vs executed
seed positions into a RunLog (which Act 4's report renders).

Timed dead reckoning: for each waypoint the robot turns to face it, then drives straight
the exact distance. The RobotIO implementation decides how (SimRobot integrates a pose;
BridgeRobot converts distance -> setMotors time). Planting happens at each plant waypoint.
"""
from __future__ import annotations

import json
import math
import time
from dataclasses import dataclass, field

from .config import SeedPlan
from .path import Waypoint, plan_summary
from .robot_io import RobotIO


@dataclass
class RunLog:
    config: dict
    planned: list[tuple[float, float]]        # intended seed positions
    executed: list[tuple[float, float]]       # where the robot actually planted
    summary: dict                             # rows / spots / seeds_total
    stats: dict                               # spacing accuracy, distance, est. time
    crop: str = ""
    recommended_date: str = ""
    rationale: str = ""

    def to_json(self, indent: int = 2) -> str:
        return json.dumps(self.__dict__, indent=indent)

    def save(self, path: str) -> None:
        with open(path, "w") as f:
            f.write(self.to_json())


def _heading_deg(x0, y0, x1, y1) -> float:
    return math.degrees(math.atan2(y1 - y0, x1 - x0))


def _spacing_stats(pts: list[tuple[float, float]]) -> dict:
    """Nearest-consecutive spacing (mean/min/max) — a proxy for spacing accuracy."""
    if len(pts) < 2:
        return {"mean_gap_m": 0.0, "min_gap_m": 0.0, "max_gap_m": 0.0}
    gaps = [math.dist(pts[i], pts[i + 1]) for i in range(len(pts) - 1)]
    return {
        "mean_gap_m": round(sum(gaps) / len(gaps), 4),
        "min_gap_m": round(min(gaps), 4),
        "max_gap_m": round(max(gaps), 4),
    }


def execute(cfg: SeedPlan, path: list[Waypoint], robot: RobotIO) -> RunLog:
    planned = [(w.x, w.y) for w in path if w.plant]

    # Position the robot at the first waypoint's cell start without "planting" phantom
    # seeds: we simply drive to each waypoint in turn from the robot's current pose.
    total_distance = 0.0
    prev = _robot_pose(robot)
    for w in path:
        heading = _heading_deg(prev[0], prev[1], w.x, w.y)
        dist = math.dist(prev, (w.x, w.y))
        if dist > 1e-9:
            robot.turn_to(heading)
            robot.forward(dist)
            total_distance += dist
        if w.plant:
            for _ in range(cfg.seeds_per_spot):
                robot.plant()
        prev = (w.x, w.y)
    robot.stop()

    executed = list(getattr(robot, "planted", []))
    # collapse seeds_per_spot repeats to one point per spot for the position log
    if cfg.seeds_per_spot > 1 and executed:
        executed = executed[:: cfg.seeds_per_spot]

    est_time_s = round(total_distance / cfg.speed_mps, 1) if cfg.speed_mps else 0.0
    stats = {
        "distance_m": round(total_distance, 3),
        "est_run_time_s": est_time_s,
        "planned_spacing": _spacing_stats(planned),
        "executed_spacing": _spacing_stats(executed),
        "max_position_error_m": round(_max_error(planned, executed), 4),
    }
    return RunLog(
        config=cfg.to_dict(),
        planned=[(round(x, 4), round(y, 4)) for x, y in planned],
        executed=[(round(x, 4), round(y, 4)) for x, y in executed],
        summary=plan_summary(cfg),
        stats=stats,
        crop=cfg.crop,
        recommended_date=cfg.recommended_date,
        rationale=cfg.rationale,
    )


def _robot_pose(robot: RobotIO) -> tuple[float, float]:
    return (getattr(robot, "x", 0.0), getattr(robot, "y", 0.0))


def _max_error(planned, executed) -> float:
    if not planned or not executed:
        return 0.0
    n = min(len(planned), len(executed))
    return max(math.dist(planned[i], executed[i]) for i in range(n))
