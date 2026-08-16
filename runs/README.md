# Run artifacts — real data off the robot

Everything here was produced by the robot itself and copied off the board unmodified.
Nothing is synthetic. They are kept in the repo so the system's behaviour can be
inspected without the hardware.

| File | What it is |
|---|---|
| `field_run.json` | A `RunLog` from an executed plot run — the planner's **planned** waypoints alongside the positions the robot **actually** seeded, plus the run config. Written by `farmos/executor.py`. |
| `field_report.svg` | The Act-4 farm map generated from that run log by `farmos/report.py`. Self-contained SVG — open it in a browser. |
| `field_diag.json` | Per-move MCU snapshots from a field run, captured via the `getDiag` RPC. For each move it records what the console **sent**, what the MCU **latched**, and the duty that reached the **driver pins** — so a discrepancy anywhere in that chain is visible after the fact. |
| `battery-log.csv` | 4,400+ samples of the pack voltage logged every 2 s during real runs, with the motor command and run state alongside. |

## Why these exist

`field_diag.json` and `battery-log.csv` are debugging instrumentation, not demo material.
They were built because two faults were otherwise invisible:

- **The drive chain** — a command could look correct in the console while a loose driver
  lead meant something different reached the motors. `getDiag` closes that loop by
  reporting from the MCU side, so "the robot did not do what we asked" is detectable
  rather than inferred.
- **The battery sense line** — an intermittent divider connection produced readings that
  were wrong *and wandering*. A terminal can't catch an intermittent fault, so the
  console logs the reading continuously with the motor state next to it: dips that track
  driving are load, dips that don't are a bad contact.

Both are described in [`../docs/troubleshooting.md`](../docs/troubleshooting.md).

## Reproducing

```bash
# on the robot, inside the app container
python3 /app/python/field_test.py row total=5 hop=0.4 rows=2 rowgap=0.4 \
  turn=90 dir=right cross speed=0.628 startup=0.099 tdps=45.2 \
  tstartup=-0.80 tpwm=120 ltrim=0.83 nobatt plant
```

Off the robot, with no hardware at all:

```bash
python examples/plain_field_demo.py     # config -> path -> sim -> log -> report.svg
```
