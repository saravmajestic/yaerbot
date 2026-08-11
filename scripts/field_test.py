#!/usr/bin/env python3
"""On-device Act-2 field test — runs INSIDE the console container (has the RouterBridge):

  docker exec motor-control-main-1 python3 /app/python/field_test.py <mode> ...

Calibration (do this first, on the pack you'll run on):
  batt                        pack voltage — write it down, it's v_cal for the runs below
  diag                        MCU snapshot: commands received, duty written to the IBT-2
                              pins, raw battery ADC, and per-move motor current if the
                              firmware has CURRENT_SENSE wired. Text log: arduino-app-cli monitor
  fwd  <pwm> <secs> [ltrim]   drive straight → measure distance (m)
  turn <pwm> <secs> [dir]     turn in place  → measure angle (deg); dir = left|right
  solve  <t1> <d1> <t2> <d2>  two fwd runs (long + short)  → prints speed= and startup=
  tsolve <t1> <a1> <t2> <a2>  two turn runs (long + short) → prints tdps= and tstartup=
                              (measure turns against a floor tape strip; a 3/4 or full
                               circle is far easier to call accurately than 90 deg)

Runs (key=value args, any order):
  row   drive one row as stop-and-go hops, then a row-end turn
        total=1.2 hop=0.4 turn=90 dir=right speed=<m/s> tdps=<deg/s>
        [pwm=180] [startup=<s>] [tstartup=<s>] [ltrim=] [v=<v_cal>] [nobatt] [plant]
  uturn the ROW CHANGE as one primitive: leg -> 90 -> gap -> 90 -> leg.
        leg=1.0 gap=0.4 turn=90 dir=right + the same robot params.
        Measure whether the two legs are PARALLEL and how far apart — both turns go
        the same way, so per-turn overshoot doubles here.
  Turn tuning: [tpwm=120] lower duty = less coast · [tramp=0.3] ease off before braking
        (cuts terminal coast and its scatter; recalibrate tdps/tstartup after changing it)
  plan  full boustrophedon over a plot + run log + SVG report to /app/python/
        w= l= rowgap= seedgap= speed= tdps= [pwm=180] [startup=] [v=] [nobatt] [plant]

The seeder is DISARMED unless you pass `plant`. `v=` is the pack voltage the speed/tdps
numbers were calibrated at — the run then raises duty to hold that speed as the pack sags.

Robot moves — keep it clear. Uses the same setMotors/stop/plantSeed RPCs as the console.
"""
import sys
import time

sys.path.insert(0, "/app/python")


def bridge():
    from arduino.app_utils import Bridge
    return Bridge


def volts(B) -> str:
    """Pack voltage as a printable string ('?' if the RPC isn't answering).

    getBattery answers with a JSON *string* over the bridge, not a dict.
    """
    try:
        r = B.call("getBattery")
        if isinstance(r, (str, bytes)):
            import json
            r = json.loads(r)
        return f"{float(r['volts']):.2f}V ({r.get('pct', '?')}%)"
    except Exception as e:
        return f"? ({e})"


def diag(B):
    """MCU-side snapshot via getDiag. None if the firmware predates it (not yet flashed)."""
    try:
        r = B.call("getDiag")
        if isinstance(r, (str, bytes)):
            import json
            r = json.loads(r)
        return r
    except Exception:
        return None


def diag_line(d) -> str:
    """One-line summary of a move: what we sent vs what reached the IBT-2 pins.

    Reads the MCU's latched `move` block, which survives the stop that ended the
    move (the live cmd/pins fields are zero by then).
    """
    if not d:
        return "diag: unavailable (flash the firmware with getDiag)"
    m = d.get("move", {})
    sent = d.get("_sent") or []
    s = (f"diag: sent={'/'.join(map(str, sent))} "
         f"mcu={m.get('req_l')}/{m.get('req_r')} "
         f"pins L[{m.get('l_rpwm')},{m.get('l_lpwm')}] R[{m.get('r_rpwm')},{m.get('r_lpwm')}] "
         f"n={d.get('cmd', {}).get('n')} "
         f"batt={d.get('batt', {}).get('volts')}V(raw {d.get('batt', {}).get('raw')})")
    a = d.get("amps")
    if a:      # only present when the firmware has CURRENT_SENSE enabled
        s += (f" amps L={a.get('l_avg')}avg/{a.get('l_max')}pk "
              f"R={a.get('r_avg')}avg/{a.get('r_max')}pk n={a.get('n')}")
    return s


def _tag(d, sent):
    """Attach what we sent, so diag_line can show it beside the MCU's view."""
    if d:
        d["_sent"] = list(sent)
    return d


def report_warnings(robot, path=None) -> None:
    """Print anything the MCU snapshots contradicted, and save the raw samples."""
    if getattr(robot, "warnings", None):
        print("\n" + "!" * 66)
        print(f"!! {len(robot.warnings)} DIAGNOSTIC WARNING(S) — the robot did not do "
              f"what we asked:")
        for w in robot.warnings:
            print(f"!!   {w}")
        print("!" * 66)
    elif getattr(robot, "diag_log", None):
        print(f"diag: {len(robot.diag_log)} snapshots, no anomalies "
              f"(commands + pins verified at the MCU)")
    if path and getattr(robot, "diag_log", None):
        import json
        try:
            with open(path, "w") as f:
                json.dump(robot.diag_log, f, indent=2)
            print("saved", path)
        except OSError as e:      # never lose a completed run to a bad save path
            print(f"(could not save diag to {path}: {e})")


def kv(args, **defaults):
    """Parse `key=value` / bare-flag arguments into a dict, over the given defaults."""
    opts = dict(defaults)
    for a in args:
        k, sep, v = a.partition("=")
        opts[k] = v if sep else True
    return opts


def need(opts, key, hint):
    if key not in opts or opts[key] is True:
        sys.exit(f"missing {key}= — {hint}")
    return float(opts[key])


def build_robot(opts, start=(0.0, 0.0, 90.0)):
    """BridgeRobot from key=value opts, with the seeder disarmed unless `plant` is given."""
    from farmos.robot_io import BridgeRobot
    v = opts.get("v")
    robot = BridgeRobot(speed_mps=need(opts, "speed", "cruise m/s from `solve`"),
                        turn_deg_per_s=need(opts, "tdps", "turn deg/s from the `turn` run"),
                        pwm=int(float(opts.get("pwm", 180))),
                        startup_s=float(opts.get("startup", 0.5)),
                        turn_startup_s=float(opts.get("tstartup", 0.75)),
                        turn_pwm=int(float(opts["tpwm"])) if opts.get("tpwm") else None,
                        turn_ramp_s=float(opts.get("tramp", 0.0)),
                        left_trim=float(opts.get("ltrim", 0.91)),
                        right_trim=float(opts.get("rtrim", 1.0)),
                        plant_enabled="plant" in opts,
                        batt_comp="nobatt" not in opts,
                        nominal_volts=float(v) if v and v is not True else None,
                        start=start)
    print(f"robot: pwm={robot.pwm} speed={robot.speed_mps} m/s startup={robot.startup_s}s "
          f"turn_pwm={robot.turn_pwm} tdps={robot.turn_deg_per_s} "
          f"tstartup={robot.turn_startup_s}s "
          f"trim={robot.left_trim}/{robot.right_trim} "
          f"batt_comp={robot.batt_comp} v_cal={robot.nominal_volts or 'as-of-now'} "
          f"seeder={'ARMED' if robot.plant_enabled else 'off'}")
    return robot


def main() -> None:
    m = sys.argv[1] if len(sys.argv) > 1 else "help"
    args = sys.argv[2:]

    if m == "batt":
        print("battery:", volts(bridge()))

    elif m == "diag":
        import json
        d = diag(bridge())
        print(json.dumps(d, indent=2) if d else diag_line(d))

    elif m == "fwd":
        pwm, secs = int(args[0]), float(args[1])
        lt = float(args[2]) if len(args) > 2 else 0.91   # left trim (tune for straight)
        B = bridge()
        print("battery before:", volts(B))
        sent = (int(pwm * lt), pwm)
        B.call("setMotors", *sent); time.sleep(secs); B.call("stop")
        print(f"fwd pwm={pwm} L*{lt} for {secs}s — measure the distance (m)")
        print(diag_line(_tag(diag(B), sent)))

    elif m == "turn":
        pwm, secs = int(args[0]), float(args[1])
        d = args[2] if len(args) > 2 else "right"
        left, right = (pwm, -pwm) if d == "right" else (-pwm, pwm)   # right = CW
        B = bridge()
        print("battery before:", volts(B))
        B.call("setMotors", left, right); time.sleep(secs); B.call("stop")
        print(f"turn {d} pwm={pwm} for {secs}s — measure the angle (deg); tdps = angle/{secs}")
        print(diag_line(_tag(diag(B), (left, right))))

    elif m in ("solve", "tsolve"):
        # x = rate * (t - dead_time) for both runs -> solve the two unknowns.
        # solve:  x = metres  -> speed= / startup=      tsolve: x = degrees -> tdps= / tstartup=
        t1, x1, t2, x2 = (float(v) for v in args[:4])
        rate = (x1 - x2) / (t1 - t2)
        dead = t1 - x1 / rate
        unit, keys, check = (("m", ("speed", "startup"), 0.4) if m == "solve"
                             else ("deg", ("tdps", "tstartup"), 90.0))
        print(f"long {x1}{unit} in {t1}s, short {x2}{unit} in {t2}s")
        print(f"  {keys[0]}={rate:.3f}  {keys[1]}={dead:.3f}")
        print(f"  (a {check:g}{unit} move will therefore be commanded for "
              f"{dead + check / rate:.2f}s)")

    elif m == "row":
        opts = kv(args, total="1.2", hop="0.4", turn="90", dir="right")
        total, hop = float(opts["total"]), float(opts["hop"])
        turn_deg = float(opts["turn"])
        robot = build_robot(opts)
        hops = int(round(total / hop))
        print(f"row: {hops} x {hop}m = {hops * hop:.2f}m, then {turn_deg:.0f}deg {opts['dir']}")
        print("battery at start:", volts(bridge()))
        try:
            for i in range(hops):
                robot.forward(hop)
                if robot.plant_enabled:
                    robot.plant()
                print(f"  hop {i + 1}/{hops}: commanded y={robot.y:.2f}m  "
                      f"pack={robot.volts()}  duty gain={robot._gain():.2f}x")
                print("   ", diag_line(robot.diag_log[-1] if robot.diag_log else None))
            sign = -1 if opts["dir"] == "right" else 1      # right = CW = heading decreases
            robot.turn_to(robot.heading + sign * turn_deg)
        finally:
            robot.stop()
        print(f"done. commanded pose x={robot.x:.2f} y={robot.y:.2f} heading={robot.heading:.0f}")
        print("battery at end:", volts(bridge()))
        report_warnings(robot, "/app/python/field_diag.json")
        print(f"MEASURE: distance travelled (expect {hops * hop:.2f}m) and the turn "
              f"(expect {turn_deg:.0f}deg)")

    elif m == "uturn":
        # The row change: A->B (row), B->B1 (90 deg, row gap, 90 deg), B1->C (next row).
        # BOTH turns go the same way, so a systematic per-turn overshoot DOUBLES here —
        # which is why this is calibrated as one primitive rather than as two 90s.
        # Its two outputs are independently measurable:
        #   heading error -> is leg C parallel to leg A (should be, but reversed)?
        #   lateral error -> is the gap between the leg centre-lines == gap?
        opts = kv(args, leg="1.0", gap="0.4", turn="90", dir="right")
        leg, gap, turn_deg = float(opts["leg"]), float(opts["gap"]), float(opts["turn"])
        robot = build_robot(opts)
        sign = -1 if opts["dir"] == "right" else 1
        print(f"uturn: leg {leg}m -> {turn_deg:.0f}deg {opts['dir']} -> gap {gap}m "
              f"-> {turn_deg:.0f}deg {opts['dir']} -> leg {leg}m")
        print("battery at start:", volts(bridge()))
        try:
            robot.forward(leg)                                   # A -> B
            robot.turn_to(robot.heading + sign * turn_deg)        # B -> B1 (first 90)
            robot.forward(gap)                                    # the row gap
            robot.turn_to(robot.heading + sign * turn_deg)        # B1 (second 90)
            robot.forward(leg)                                    # B1 -> C
        finally:
            robot.stop()
        print(f"done. commanded pose x={robot.x:.2f} y={robot.y:.2f} "
              f"heading={robot.heading:.0f} (started 90)")
        report_warnings(robot, "/app/python/field_diag.json")
        print("\nMEASURE two things:")
        print(f"  1. HEADING  — is the last leg parallel to the first? "
              f"(off by how many deg, and which way)")
        print(f"  2. LATERAL  — perpendicular distance between the two legs "
              f"(expect {gap:.2f}m)")
        print("  'parallel or not' is far easier to judge by eye than '90 deg or not',")
        print("  which is why we tune the pair, not each turn.")

    elif m == "plan":
        from farmos import SeedPlan, plan_boustrophedon, execute
        from farmos.report import save_report
        opts = kv(args)
        cfg = SeedPlan(plot_w_m=need(opts, "w", "plot width m"),
                       plot_l_m=need(opts, "l", "plot length m"),
                       row_gap_m=need(opts, "rowgap", "row spacing m"),
                       seed_gap_m=need(opts, "seedgap", "seed spacing m"),
                       speed_mps=need(opts, "speed", "cruise m/s from `solve`"),
                       crop="groundnut")
        path = plan_boustrophedon(cfg)
        robot = build_robot(opts)
        print(f"plan: {len(path)} spots over {cfg.plot_w_m}x{cfg.plot_l_m} m, "
              f"rows @ {cfg.row_gap_m}m, seeds @ {cfg.seed_gap_m}m")
        print("battery at start:", volts(bridge()))
        try:
            log = execute(cfg, path, robot)
        finally:
            robot.stop()
        log.save("/app/python/field_run.json")
        save_report(log, "/app/python/field_report.svg")
        print("stats:", log.stats)
        print("battery at end:", volts(bridge()), "| final duty gain:", f"{robot._gain():.2f}x")
        print("saved /app/python/field_run.json + field_report.svg")
        report_warnings(robot, "/app/python/field_diag.json")

    else:
        print(__doc__)


if __name__ == "__main__":
    main()
