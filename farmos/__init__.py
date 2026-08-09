"""farmos — the software brain for the yaerbot seeding robot.

Modules are built act-by-act (see docs/demo-plan.md):
  Act 2 — plain-land fixed-spacing seeding:  config -> path -> executor -> run log
  Act 4 — report:                            run log -> SVG farm map

Everything here is hardware-free and testable off the robot: actuation goes through
the RobotIO interface (robot_io.py), with a dead-reckoning SimRobot for tests/demos and
a BridgeRobot adapter for the real UNO Q.
"""

from .config import SeedPlan
from .path import Waypoint, plan_boustrophedon
from .executor import RunLog, execute
from .robot_io import RobotIO, SimRobot

__all__ = [
    "SeedPlan",
    "Waypoint",
    "plan_boustrophedon",
    "RunLog",
    "execute",
    "RobotIO",
    "SimRobot",
]
