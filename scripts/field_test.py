#!/usr/bin/env python3
"""On-device Act-2 field test — runs INSIDE the console container (has the RouterBridge):

  docker exec motor-control-main-1 python3 /app/python/field_test.py <mode> ...

Calibration (do this first, on the pack you'll run on):
  batt                        pack voltage — write it down, it's v_cal for the runs below
  battlog [n]                 summarise the console's continuous A4/battery log
                              (last n samples). Judge the sensor on the IDLE spread —
                              sag while driving is real. Log: ~motor-control/battery.csv
  diag                        MCU snapshot: commands received, duty written to the IBT-2
                              pins, raw battery ADC, and per-move motor current if the
                              firmware has CURRENT_SENSE wired. Text log: arduino-app-cli monitor
  fwd  <pwm> <secs> [ltrim]   drive straight → measure distance (m)
  turn <pwm> <secs> [dir]     turn in place  → measure angle (deg); dir = left|right
  gyro [n]                    MPU-6050 check: alive? and how much RESIDUAL rate is left
                              after removing bias — that times the turn length is the
                              error integration will accumulate. Robot must be STILL.
  gspin <secs> [actual_deg]   SCALE CHECK, motors off: integrate while YOU turn the robot
                              by hand through a known angle (use 360 — far easier to call
                              than 90). Pass the actual angle and it prints the corrected
                              lsb_per_dps. Isolates sensitivity from skid and coast.
  gscale <lsb_x1000>          apply a scale factor at runtime (lost on reboot; bake the
                              confirmed value into the firmware afterwards)
  gturn <deg> [n] [dir] [pwm] CLOSED-LOOP turn: pivot until the gyro says it got there,
                              then print commanded vs achieved. Repeat n times to see the
                              spread — a steady offset is coast, a wide spread is SKID.
  solve  <t1> <d1> <t2> <d2>  two fwd runs (long + short)  → prints speed= and startup=
  tsolve <t1> <a1> <t2> <a2>  two turn runs (long + short) → prints tdps= and tstartup=
                              (measure turns against a floor tape strip; a 3/4 or full
                               circle is far easier to call accurately than 90 deg)

Runs (key=value args, any order):
  row   drive one row as stop-and-go hops, then a row-end turn
        total=1.2 hop=0.4 turn=90 dir=right speed=<m/s> tdps=<deg/s>
        [pwm=180] [startup=<s>] [tstartup=<s>] [ltrim=] [v=<v_cal>] [nobatt] [plant]
        The seeder flags are INDEPENDENT and both default OFF — four combinations:
          (neither)    drive only, nothing planted
          plant        one 2-seed drop per stop (spool never moves)
          cross        spool sweeps at each stop, solenoid stays OFF (rehearsal)
          cross plant  the 4-spot pattern: 0/90/180/270, 4 seeds per stop
        [cross=0,90] to pick the arm positions.  [dwell=0.6] arm settle before a drop.
        Every run prints a "SEEDER:" line saying which of the four you got.
        [rows=3] [rowgap=0.4] — serpentine over N rows. Between rows it does a real
        ROW CHANGE (turn + rowgap + turn, alternating), NOT a 180 spin-in-place, so
        each row is offset into fresh ground. With rows=1 you get the old single
        end-of-row turn instead.
        Hop count ALWAYS ROUNDS DOWN: a long row leaves no headland to turn in.
  spool S3003 arm calibration — SEEDER ONLY, never touches the drive motors:
        spool sweep [from=0] [to=180] [step=15] [dwell=3]   step + hold, you measure
        spool go <deg>                                      park at one commanded angle
        spool solve <cmd1> <phys1> <cmd2> <phys2>           -> SPOOL_A / SPOOL_B
  cycle FULL ACTUATOR LOOP x n — the one that exercises everything end to end:
        hop -> stop -> 4-seed cross (spool 0/180 then 90/270, solenoid each time)
        -> body turn -> repeat.
        n=3 hop=0.4 turn=90 dir=right angles=0,90 dwell=0.6 speed= tdps=
        [pwm=180] [startup=] [tstartup=] [tpwm=] [ltrim=] [v=] [nobatt] [plant]
        WITHOUT `plant` the spool + turns still run but the solenoid never fires —
        do that first, on the bench, with the hopper empty.
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

# Stream the log as it happens. stdout is BLOCK-buffered when it isn't a terminal, and
# these runs are always piped (ssh -> docker exec), so a 20-hop row printed nothing at
# all until it finished — useless when the whole point is watching the robot against
# the log, and worse, a run you abort shows you nothing about where it got to.
try:
    sys.stdout.reconfigure(line_buffering=True)
except (AttributeError, OSError):        # odd stream; `python3 -u` is the fallback
    pass


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


def _jsonrpc(B, name, *a, timeout=None):
    """Call an RPC that answers with a JSON *string* over the bridge, return a dict.

    `timeout` matters for the BLOCKING RPCs. Bridge.call defaults to 10s, and gyroIntegrate
    holds the MCU for its whole window (plus a bias sample), so a 12s spin raised
    TimeoutError while the turn itself was running perfectly well.
    """
    import json
    r = B.call(name, *a, timeout=timeout) if timeout else B.call(name, *a)
    if isinstance(r, (bytes, bytearray)):
        r = r.decode()
    return json.loads(r) if isinstance(r, str) else r


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

    elif m == "gyro":
        # Is the MPU-6050 alive, and is its residual small enough to integrate?
        #   bias_dps  — zero-rate offset, removed before integration (any value is fine)
        #   mean_dps  — what is LEFT after removing it. This is the number that matters:
        #               it is the drift rate, so mean x turn_seconds = degrees of error.
        #   peak_dps  — worst single sample; vibration shows up here, not in the mean.
        B = bridge()
        for i in range(int(args[0]) if args else 1):
            r = _jsonrpc(B, "getGyro")
            if not r.get("ok"):
                print("gyro NOT responding: whoami=%s  %s" % (r.get("whoami"), r.get("err", "")))
                print("  check: module on +3V3 (not 5V), SDA->D20, SCL->D21, AD0->GND")
                break
            drift = r["mean_dps"] * 2.0          # a pivot lasts about two seconds
            print("gyro ok (whoami=0x%02X)  bias %+.3f  residual %+.3f dps  peak %+.3f  "
                  "-> ~%.1f deg of drift over a 2s turn"
                  % (r["whoami"], r["bias_dps"], r["mean_dps"], r["peak_dps"], abs(drift)))
            if abs(drift) > 5:
                print("  WARNING residual is large — is the robot completely still?")
            if i:
                time.sleep(0.5)

    elif m == "gspin":
        # SCALE FACTOR, isolated. Motors never run — you turn the robot by hand through a
        # known angle. A full 360 against a floor mark is far easier to call accurately
        # than 90 deg (same reasoning as tsolve), and it removes skid, coast and PWM from
        # the measurement entirely, leaving only sensitivity.
        secs = float(args[0]) if args else 10.0
        actual = float(args[1]) if len(args) > 1 else None
        B = bridge()
        print("hold the robot STILL — sampling bias...")
        # +8s of headroom: the MCU blocks for the whole window and cannot service
        # the bridge until it returns.
        r = _jsonrpc(B, "gyroIntegrate", int(secs * 1000), timeout=secs + 8)
        if not r.get("ok"):
            print("  FAILED:", r.get("err")); return
        print("  integrated %+.1f deg (|%.1f|) over %.1fs, peak %+.1f dps, %d samples"
              % (r["signed_deg"], r["abs_deg"], secs, r["peak_dps"], r["samples"]))
        if actual:
            ratio = abs(r["signed_deg"]) / actual
            new = r["lsb_per_dps"] * ratio
            print("  reported %.1f vs actual %.1f -> reads %.3fx reality" % (
                abs(r["signed_deg"]), actual, ratio))
            print("  lsb_per_dps should be %.1f (currently %.1f)" % (new, r["lsb_per_dps"]))
            print("  apply it now (until reboot):  gscale %.0f" % (new * 1000))
        else:
            print("  now re-run with the angle you actually turned it through, e.g.")
            print("    gspin %g 360" % secs)

    elif m == "gscale":
        # Apply a scale factor without reflashing. Bake the confirmed value into the
        # firmware afterwards — this does not survive a reboot.
        B = bridge()
        print(_jsonrpc(B, "setGyroScale", int(float(args[0]))))

    elif m == "gturn":
        # CLOSED-LOOP turn: pivot until the gyro says we got there, and report the skid.
        #   gturn 90            one 90 deg right turn
        #   gturn 90 4          four of them, to see run-to-run spread
        #   gturn 90 4 left
        deg = float(args[0]) if args else 90.0
        n = int(args[1]) if len(args) > 1 else 1
        d = args[2] if len(args) > 2 else "right"
        pwm = int(args[3]) if len(args) > 3 else 120
        B = bridge()
        print("battery before:", volts(B))
        print(f"pivotDeg {deg:g} deg {d} at pwm {pwm}, x{n} — closed on the gyro")
        errs = []
        for i in range(n):
            r = _jsonrpc(B, "pivotDeg", int(deg), pwm,
                         1 if d == "right" else 0, timeout=15)
            if not r.get("ok"):
                print("  FAILED: %s" % r.get("err")); break
            errs.append(r["achieved"] - deg)
            print("  %d: target %5.1f  achieved %5.1f  err %+5.1f  %4dms  avg %5.1f dps  "
                  "coast used %4.1f seen %4.1f -> next %4.1f  judder %+4.1f%s"
                  % (i + 1, r["target"], r["achieved"], r["err_deg"], r["ms"],
                     r.get("avg_dps", 0), r.get("coast_used", 0), r.get("coast_seen", 0),
                     r.get("coast_next", 0), r.get("judder_deg", 0),
                     "  TIMEOUT" if r.get("timeout") in (True, "true") else ""))
            time.sleep(1.0)
        if errs:
            print("  spread: mean %+.1f deg, worst %+.1f deg over %d turn(s)"
                  % (sum(errs) / len(errs), max(errs, key=abs), len(errs)))
            print("  (a consistent OFFSET is coast to trim in firmware; a wide SPREAD")
            print("   is skid, which is exactly what the closed loop is for)")
            print("  judder = |rate| integral minus signed integral. Large means the pivot")
            print("  is shaking, so the chassis is oscillating rather than turning cleanly.")
            print("  coast is LEARNED: it releases at target-coast, measures the real coast")
            print("  through the stop, and carries it forward — so err should shrink over")
            print("  the first turn or two and then stay put on that surface.")

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
        opts = kv(args, total="1.2", hop="0.4", turn="90", dir="right", dwell="0.6")
        total, hop = float(opts["total"]), float(opts["hop"])
        turn_deg, dwell = float(opts["turn"]), float(opts["dwell"])
        # `cross` (bare flag, or cross=0,90) plants the 4-spot pattern at every stop
        # instead of a single 2-seed drop. Arm positions, not seed positions: each one
        # drops 2 seeds 180 deg apart, so 0,90 puts seeds at 0/90/180/270.
        cx = opts.get("cross")
        angles = ([0, 90] if cx is True else
                  [int(float(a)) % 180 for a in str(cx).split(",") if a != ""] if cx
                  else None)
        robot = build_robot(opts)
        # ALWAYS ROUND DOWN — a long row runs out of headland to turn around in, so a
        # short row is the safe error. The 1e-6 is required, not cosmetic: 5.0/0.4 is
        # 12.4999... and 4.8/0.4 is 11.9999... in binary floating point, so a bare
        # int() would drop a legitimate hop off any row that IS an exact multiple.
        hops = int(total / hop + 1e-6)
        if hops * hop < total - 1e-9:
            print(f"note: {total}m / {hop}m doesn't divide evenly — rounding DOWN to "
                  f"{hops} hops = {hops * hop:.2f}m (short, so the headland is safe)")
        rows = int(float(opts.get("rows", 1)))
        rowgap = float(opts.get("rowgap", hop))
        sign = -1 if opts["dir"] == "right" else 1      # right = CW = heading decreases
        spots = ", ".join(f"{a}/{a + 180}" for a in angles) if angles else "single (2 seeds)"
        print(f"row: {rows} row(s) x {hops} x {hop}m = {hops * hop:.2f}m each, "
              f"planting {spots}")
        if rows > 1:
            print(f"  row change: {turn_deg:.0f}deg + {rowgap}m + {turn_deg:.0f}deg, "
                  f"first one {opts['dir']}, alternating (serpentine)")
        else:
            print(f"  then a single {turn_deg:.0f}deg {opts['dir']} at the end")
        # Say the seeder state in words. `cross` and `plant` are INDEPENDENT and both
        # default off, which gives four legitimate combinations — spelling out which one
        # you got beats discovering it from the seed count afterwards.
        #   (neither)     drive only
        #   plant         one 2-seed drop per stop, spool never moves
        #   cross         spool sweeps, solenoid stays off  (rehearse the arm, no seeds)
        #   cross plant   the 4-spot pattern
        stops = hops * rows
        if angles is None and not robot.plant_enabled:
            print("  SEEDER: OFF — drive only. Add `plant` for a single drop per stop, "
                  "or `cross plant` for 4 spots.")
        elif angles is None:
            print(f"  SEEDER: ARMED, SINGLE DROP — 2 seeds at each of {stops} stops = "
                  f"{2 * stops} seeds. The spool does NOT move; add `cross` for 4 spots.")
        elif not robot.plant_enabled:
            print("  SEEDER: DRY CROSS — the spool sweeps at every stop, solenoid stays "
                  "OFF. Add `plant` to fire it.")
        else:
            print(f"  SEEDER: ARMED, 4-SPOT CROSS — {2 * len(angles)} seeds at each of "
                  f"{stops} stops = {2 * len(angles) * stops} seeds.")
        print("battery at start:", volts(bridge()))
        seeds = 0
        try:
            for r in range(rows):
                for i in range(hops):
                    robot.forward(hop)
                    if angles is not None:
                        seeds += robot.plant_cross(angles, dwell_s=dwell)
                    elif robot.plant_enabled:
                        robot.plant(); seeds += 2
                    print(f"  row {r + 1}/{rows} hop {i + 1}/{hops}: "
                          f"commanded x={robot.x:.2f} y={robot.y:.2f}  seeds={seeds}  "
                          f"pack={robot.volts()}  duty gain={robot._gain():.2f}x")
                    print("   ", diag_line(robot.diag_log[-1] if robot.diag_log else None))
                if rows == 1:
                    robot.turn_to(robot.heading + sign * turn_deg)   # legacy single turn
                elif r < rows - 1:
                    # ROW CHANGE, not a U-turn: turn, cross the row gap, turn again. The
                    # two turns go the SAME way (which is why per-turn overshoot doubles
                    # here — see `uturn`), and the pair ALTERNATES direction each row, so
                    # the path serpentines and every row is driven in the opposite sense.
                    s = sign if r % 2 == 0 else -sign
                    way = "right" if s < 0 else "left"
                    robot.turn_to(robot.heading + s * turn_deg)
                    robot.forward(rowgap)
                    robot.turn_to(robot.heading + s * turn_deg)
                    print(f"  -- row change {r + 1}->{r + 2} ({way}): now at "
                          f"x={robot.x:.2f} y={robot.y:.2f} heading={robot.heading:.0f}")
        finally:
            robot.stop()
        print(f"done. {seeds} seeds. commanded pose x={robot.x:.2f} y={robot.y:.2f} "
              f"heading={robot.heading:.0f}")
        print("battery at end:", volts(bridge()))
        report_warnings(robot, "/app/python/field_diag.json")
        print(f"MEASURE: distance travelled (expect {hops * hop:.2f}m) and the turn "
              f"(expect {turn_deg:.0f}deg)")

    elif m == "spool":
        # S3003 arm calibration. DELIBERATELY MOTOR-FREE: this mode only ever calls
        # indexSpool, so the robot cannot drive off while you are holding a protractor
        # against the arm. Bench or ground, doesn't matter — nothing rolls.
        #
        # Why this is needed: indexSpool passes the angle straight to Servo::write(),
        # but an S3003 does not sweep a true 180 deg over a 500-2500us pulse.
        # firmware/s3003_cycle.ino:35 says write(90) reached only ~60 deg of travel,
        # yet the same sketch dialled in goTo(80) for a physical quarter turn — those
        # two notes contradict each other and nobody has measured it since the RPC
        # replaced that sketch. It matters because the drip cross plants at arm 0 and
        # 90: if commanded != physical, the "4-seed cross" is a skewed X.
        #
        # Mark the arm (tape flag on one outlet) and measure against a fixed reference.
        # Measure the SAME outlet every time — the other one reads 180 deg away.
        sub = args[0] if args else "sweep"
        B = bridge()

        if sub == "go":
            deg = int(float(args[1]))
            B.call("indexSpool", deg)
            print(f"spool commanded {deg} deg — measure the PHYSICAL arm angle now")

        elif sub == "solve":
            # Fit commanded = A*physical + B from two (commanded, physical) pairs.
            c1, p1, c2, p2 = (float(v) for v in args[1:5])
            if p1 == p2:
                sys.exit("the two physical angles must differ")
            a = (c2 - c1) / (p2 - p1)
            b = c1 - a * p1
            print(f"measured: cmd {c1:g} -> phys {p1:g} deg,  cmd {c2:g} -> phys {p2:g} deg")
            print(f"  travel ratio: {1 / a:.3f} deg physical per deg commanded")
            print(f"  to get PHYSICAL p, command:  {a:.4f} * p + {b:.2f}")
            print(f"\nfirmware (farm_os.ino):")
            print(f"  const float SPOOL_A = {a:.4f}f;")
            print(f"  const float SPOOL_B = {b:.2f}f;")
            # what the drip cross actually needs, and whether the servo can reach it
            lo, hi = (0 - b) / a, (180 - b) / a
            lo, hi = min(lo, hi), max(lo, hi)
            print(f"\nreachable physical range: {lo:.0f}..{hi:.0f} deg")
            for want in (0, 45, 90):
                cmd = a * want + b
                ok = "" if 0 <= cmd <= 180 else "   <-- OUT OF RANGE, unreachable"
                print(f"  physical {want:3d} deg  ->  command {cmd:6.1f}{ok}")
            if hi < 90:
                print("\n  !! the arm cannot reach a physical 90 deg — the 4-seed cross")
                print("     cannot be a true cross with this servo/linkage as geared.")

        else:  # sweep
            opts = kv(args[1:], step="15", dwell="3")
            lo = float(opts.get("from", 0)); hi = float(opts.get("to", 180))
            step, dwell = float(opts["step"]), float(opts["dwell"])
            n = int(abs(hi - lo) / step) + 1
            print(f"spool sweep {lo:g}..{hi:g} step {step:g}, holding {dwell:g}s each "
                  f"({n} positions, ~{n * dwell:.0f}s)")
            print("MOTORS ARE NOT TOUCHED. Mark one outlet and read each position.\n")
            print("  commanded   physical (write it down)")
            for i in range(n):
                d = int(round(lo + i * step * (1 if hi >= lo else -1)))
                B.call("indexSpool", d)
                print(f"  {d:9d}   ______")
                time.sleep(dwell)
            B.call("indexSpool", 0)
            print("\nparked at 0. Now feed the two most accurate readings to:")
            print("  field_test.py spool solve <cmd1> <phys1> <cmd2> <phys2>")
            print("Pick two WIDELY SEPARATED points (e.g. 0 and 180) — a short baseline")
            print("multiplies your reading error into the fit.")

    elif m == "cycle":
        # The whole actuator chain in one loop: drive, seeder (spool + solenoid), body turn.
        # Deliberately NOT a row: each iteration ends with a turn, so n=4 x turn=90 walks
        # a square and returns to the start — an integration test whose error you can see
        # on the ground, not just a distance to tape-measure.
        opts = kv(args, n="3", hop="0.4", turn="90", dir="right",
                  angles="0,90", dwell="0.6")
        n = int(float(opts["n"]))
        hop, turn_deg = float(opts["hop"]), float(opts["turn"])
        dwell = float(opts["dwell"])
        angles = [int(float(a)) % 180 for a in str(opts["angles"]).split(",") if a != ""]
        robot = build_robot(opts)
        sign = -1 if opts["dir"] == "right" else 1        # right = CW = heading decreases
        spots = ", ".join(f"{a}/{a + 180}" for a in angles)
        print(f"cycle: {n} x [ {hop}m -> plant {spots} ({2 * len(angles)} seeds) -> "
              f"{turn_deg:.0f}deg {opts['dir']} ]")
        print("seeder: " + ("ARMED — solenoid WILL fire" if robot.plant_enabled
                            else "DISARMED — spool moves, solenoid stays off"))
        print("battery at start:", volts(bridge()))
        seeds = 0
        try:
            for i in range(n):
                robot.forward(hop)
                print(f"  [{i + 1}/{n}] hop {hop}m  pack={robot.volts()}  "
                      f"gain={robot._gain():.2f}x")
                print("     ", diag_line(robot.diag_log[-1] if robot.diag_log else None))
                seeds += robot.plant_cross(angles, dwell_s=dwell)
                print(f"     planted {spots} — {seeds} seeds so far")
                robot.turn_to(robot.heading + sign * turn_deg)
                print(f"     turned {turn_deg:.0f}deg {opts['dir']} -> "
                      f"commanded heading {robot.heading:.0f}")
                print("     ", diag_line(robot.diag_log[-1] if robot.diag_log else None))
        finally:
            robot.stop()
        print(f"\ndone. {n} cycles, {seeds} seeds. commanded pose "
              f"x={robot.x:.2f} y={robot.y:.2f} heading={robot.heading:.0f} (started 90)")
        print("battery at end:", volts(bridge()))
        report_warnings(robot, "/app/python/field_diag.json")
        print("\nMEASURE:")
        print(f"  · hop length each time (expect {hop:.2f}m) — does it drift over the 3?")
        print(f"  · turn angle each time (expect {turn_deg:.0f}deg) — same question")
        print(f"  · the seed holes: {2 * len(angles)} per stop, in a cross around the spot")
        if abs(n * turn_deg % 360) < 1e-6:
            print("  · the turns close a full circle — the robot should end where it "
                  "started, facing the same way. Any gap is the ACCUMULATED error.")

    elif m == "battlog":
        # Summarise the continuous A4 log the console writes. The point is to catch
        # an INTERMITTENT fault: a reading that is low *and* unstable is a wiring
        # problem, and whether the dips coincide with driving says which kind.
        import csv
        n = int(args[0]) if args else 200
        path = "/app/battery.csv"
        try:
            rows = list(csv.DictReader(open(path)))
        except OSError as e:
            return print(f"no log yet at {path} ({e}) — is the console app running?")
        if not rows:
            return print("log is empty")
        rows = rows[-n:]
        vals = [float(r["volts"]) for r in rows if r["volts"]]
        moving = [r for r in rows if r["left"] not in ("", "0", "None")]
        idle = [float(r["volts"]) for r in rows if r["left"] in ("", "0", "None") and r["volts"]]
        print(f"{len(rows)} samples  {rows[0]['utc']} .. {rows[-1]['utc']}")
        print(f"  all    : min {min(vals):.2f}  max {max(vals):.2f}  "
              f"spread {max(vals) - min(vals):.2f}V")
        if idle:
            print(f"  IDLE   : min {min(idle):.2f}  max {max(idle):.2f}  "
                  f"spread {max(idle) - min(idle):.2f}V   <- judge the sensor on THIS")
        print(f"  moving : {len(moving)} samples (sag here is real, not a fault)")
        # the tell: big swings while idle
        if idle and max(idle) - min(idle) > 0.3:
            print("  !! idle spread > 0.3V — unstable sensor, suspect the A4 connection")
        elif idle:
            print("  idle reading looks stable")
        print("  worst dips:")
        for r in sorted(rows, key=lambda r: float(r["volts"] or 99))[:5]:
            print(f"    {r['utc']}  {r['volts']}V  raw={r['raw']}  "
                  f"motors={r['left']}/{r['right']}  src={r['src']}  run={r['run_state']}")

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
