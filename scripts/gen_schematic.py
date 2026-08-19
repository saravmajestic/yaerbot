#!/usr/bin/env python3
"""Generate the robot's circuit schematics as SVG, from code.

    pip install schemdraw
    python3 scripts/gen_schematic.py          # -> docs/schematic/*.svg

WHY THIS IS A SCRIPT AND NOT A DRAWING. The schematic has to stay true to the firmware, and
the firmware moves: A4 was the battery divider until the gyro took it, EC drive moved off D10
when the seeder spool claimed it, and A5 has been three different things. A drawing made in a
GUI goes stale silently. This one lives beside the code, diffs like code, and every pin below
is traceable to a #define in firmware/farm_os/farm_os.ino.

Four sheets, because one sheet holding all of it would be unreadable:

    1. power       LiPo -> fuses -> buck -> the 5V and 12V rails
    2. drive       UNO Q -> 2x IBT-2 -> 4 gear motors
    3. seeder      2 servos + the solenoid low-side driver
    4. gyro        the MPU-6050 on Wire2 -- the only sensor actually fitted

Three conventions, each of which fixed a class of unreadable output:

* THE SVGs CARRY ONLY THE CIRCUIT -- no titles, no prose. Captions and warnings live in
  docs/schematic/index.html, generated alongside. Prose on a schemdraw canvas fights the
  auto-sized viewBox and lands on top of the wires.
* PIN NAMES GO ON THE WIRES, not inside the blocks. Ic draws pin labels *inside* the box,
  where they collide with the block's own name however wide you make it.
* LAYOUT IS BY EXPLICIT COORDINATES. Relative chaining kept walking branches into each other.

PIN SOURCE OF TRUTH: firmware/farm_os/farm_os.ino. Change a pin there, change it here.
"""
import os

import schemdraw
import schemdraw.elements as elm

schemdraw.use("svg")

OUT = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                   "docs", "schematic")

# An explicit font matters: schemdraw defaults to font-family="sans", which browsers
# resolve but most SVG rasterizers (ImageMagick included) cannot, so the file renders
# fine on screen and fails the moment anyone converts it for a slide or a PDF.
STYLE = dict(unit=2.2, fontsize=13, lw=1.9, font="Helvetica")
SMALL = 11

# Colour carries MEANING here, one hue per net class, so a wire's job is readable without
# tracing it: nothing is coloured for decoration. The legend in index.html repeats this map.
INK   = "#2B333C"   # signal / logic wiring, and the default
V12   = "#C0392B"   # the 12V battery side
V5    = "#C98A1E"   # the 5V rail out of the buck
GND   = "#5C6875"   # ground, and the ground symbols
SIG   = "#2E6FB0"   # I2C / data
DIM    = "#5C6875"  # annotation text
BODY  = "#F4F6F8"   # block fill
PART  = "#E8D8A8"   # passive component fill (resistors, fuses, coil)
PEDGE = "#A6844A"   # passive component stroke


def _drawing(name):
    d = schemdraw.Drawing(file=os.path.join(OUT, name + ".svg"), show=False)
    d.config(**STYLE)
    return d


def _block(label, left=(), right=(), top=(), bottom=(), w=3.2, h=3.0, show_pins=False):
    """A block whose pins are named anchors.

    RBox exposes only start/end, so every block here is an Ic even when it is not a chip.

    Ic draws `name` INSIDE the body, where it lands on top of the block's own label however
    wide the body is made -- so `name` is always empty here. When a pin label is wanted it goes
    in `pin`, which schemdraw renders just OUTSIDE the body, which is where a schematic puts it
    anyway. Blocks with a single pass-through pin pass show_pins=False and carry the signal name
    on the wire instead.
    """
    pins = []
    for side, names in (("left", left), ("right", right), ("top", top), ("bottom", bottom)):
        for i, n in enumerate(names):
            pins.append(elm.IcPin(name="", pin=n if show_pins else None, anchorname=n,
                                  side=side, slot="%d/%d" % (len(names) - i, len(names))))
    # size=(w, h) is the REAL parameter. Ic's signature is (size, pins, slant, **kwargs), so
    # passing w=/h= puts them in kwargs where they are silently ignored -- the body keeps its
    # default size and every pin sits at the fixed 0.6 default pitch. That is what made blocks
    # land on top of each other while the code claimed to space them.
    #
    # Pins are distributed down the body: for n pins they span y = 0.5 .. h-0.5, so the pitch is
    # (h - 1) / (n - 1). Size a block's h from the pitch you need, not from how big it should look.
    #
    # .right() is NOT decoration either: an Ic inherits the drawing's current direction, so a
    # block placed after a vertical Line comes out rotated 90 degrees with its pins on the top
    # and bottom edges. Pinning theta=0 makes placement independent of draw order.
    return elm.Ic(pins=pins, size=(w, h)).right().fill(BODY).color(INK).label(label)


def _note(d, text, at, color=DIM):
    d.add(elm.Label().label(text, fontsize=SMALL, color=color, halign="center").at(at))


# --------------------------------------------------------------------- 1. power
def sheet_power():
    """One lane per branch -- the point of this sheet is that each branch has its OWN fuse
    sized to its OWN wire."""
    BUS, GND_Y = 5.0, -7.0
    with _drawing("power") as d:
        # --- pack -> main fuse -> 12V distribution block
        d += elm.Battery().up().at((0, 0)).length(2.2).color(V12).label(
            "3S LiPo\n12.6V full", loc="left", color=INK)
        d += elm.Line().at((0, 2.2)).to((1.4, 2.2)).color(V12)
        d += elm.Fuse().at((1.4, 2.2)).right().length(2.2).color(V12).fill(PART).label(
            "MAIN\n20A", color=INK)
        d += elm.Line().at((3.6, 2.2)).to((BUS, 2.2)).color(V12)
        d += elm.Dot().at((BUS, 2.2)).color(V12)
        _note(d, "12V distribution block", (BUS + 2.4, 3.2))

        # --- the 12V bus
        d += elm.Line().at((BUS, 5.2)).to((BUS, -4.0)).color(V12)

        # branch A: 3A -> buck -> the 5V rail
        d += elm.Dot().at((BUS, 5.2)).color(V12)
        d += elm.Fuse().at((BUS, 5.2)).right().length(2.2).color(V12).fill(PART).label(
            "3A", color=INK)
        d += elm.Line().at((BUS + 2.2, 5.2)).to((BUS + 3.2, 5.2)).color(V12)
        d += _block("BUCK\n12V - 5V", left=["IN"], right=["OUT"], w=3.4, h=1.8).at(
            (BUS + 3.2, 5.2)).anchor("IN")
        # everything from here right is the 5V rail
        d += elm.Line().at((BUS + 6.6, 5.2)).to((BUS + 8.4, 5.2)).color(V5)
        d += elm.Dot().at((BUS + 8.4, 5.2)).color(V5)
        _note(d, "+5V", (BUS + 7.5, 6.0), color=V5)
        # NO bulk capacitor: as built the buck feeds the UNO Q's 5V header pin directly.
        unoq = d.add(_block("UNO Q\n5V header pin", left=["5V"], bottom=["GND"],
                            w=4.0, h=1.8).at((BUS + 8.4, 5.2)).anchor("5V"))
        d += elm.Line().at((BUS + 8.4, 5.2)).to(unoq.__getattr__("5V")).color(V5)
        d += elm.Line().at((BUS + 8.4, 5.2)).to((BUS + 8.4, 8.2)).color(V5)
        srv = d.add(_block("servos\nS3003 + SG90", left=["V+"], bottom=["GND"],
                           w=4.0, h=1.6).at((BUS + 11.0, 8.2)).anchor("V+"))
        d += elm.Line().at((BUS + 8.4, 8.2)).to(srv.__getattr__("V+")).color(V5)
        # each load returns to the same node; ground symbols ARE that node
        for blk in (unoq, srv):
            g = blk.__getattr__("GND")
            d += elm.Line().at(g).to((g[0], g[1] - 0.7)).color(GND)
            d += elm.Ground().at((g[0], g[1] - 0.7)).color(GND)

        # branch B: 2A -> solenoid rail
        d += elm.Dot().at((BUS, 0.0)).color(V12)
        d += elm.Fuse().at((BUS, 0.0)).right().length(2.2).color(V12).fill(PART).label(
            "2A", color=INK)
        d += elm.Line().at((BUS + 2.2, 0.0)).to((BUS + 3.2, 0.0)).color(V12)
        d += _block("+12V to the solenoid\nsheet 3", left=["V+"], w=5.4, h=1.7).at(
            (BUS + 3.2, 0.0)).anchor("V+")

        # branch C: motor power, straight off the block
        d += elm.Dot().at((BUS, -4.0)).color(V12)
        d += elm.Line().at((BUS, -4.0)).to((BUS + 3.2, -4.0)).color(V12)
        d += _block("IBT-2 x2   B+\nsheet 2", left=["B+"], w=5.4, h=1.7).at(
            (BUS + 3.2, -4.0)).anchor("B+")

        # --- the pack's own return, and the star point every ground symbol refers to
        d += elm.Line().at((0, 0)).to((0, GND_Y)).color(GND)
        d += elm.Line().at((0, GND_Y)).to((BUS + 6.0, GND_Y)).color(GND)
        d += elm.Ground().at((BUS + 2.0, GND_Y)).color(GND)
        _note(d, "common GND - one star point at the block", (BUS + 6.4, GND_Y - 0.75), color=GND)


# --------------------------------------------------------------------- 2. drive
def sheet_drive():
    """One IBT-2 per side, two gear motors in PARALLEL ACROSS its two outputs.

    Note the motors sit between M+ and M-, not between M+ and ground: a BTS7960 is an
    H-bridge, so both of its outputs are driven and neither is a ground reference. Grounding
    M- would be a short through the low-side switch whenever the bridge drove that leg high.
    """
    MOT_X, RAIL_A, RAIL_B = 16.5, 16.5, 21.3
    with _drawing("drive") as d:
        mcu = d.add(_block("UNO Q\nSTM32U585\n(MCU side)",
                           right=["D3", "D5", "D7", "D8", "D6", "D9", "D4", "D12"],
                           w=4.4, h=8.7, show_pins=True).at((0, 0)).anchor("center"))

        for side, pins, cy in (("LEFT", ("D3", "D5", "D7", "D8"), 3.6),
                               ("RIGHT", ("D6", "D9", "D4", "D12"), -3.6)):
            drv = d.add(_block("IBT-2 %s\nBTS7960\n%s" % ("#1" if side == "LEFT" else "#2", side),
                               left=["RPWM", "LPWM", "R_EN", "L_EN"], right=["M+", "M-"],
                               w=5.2, h=4.4, show_pins=True).at((10.6, cy)).anchor("center"))
            # 'z' routes horizontal -> vertical -> horizontal, so the wire ARRIVES horizontally
            # at the pin. '-|' arrives vertically and cuts straight through the body.
            for pin, ctrl in zip(pins, ("RPWM", "LPWM", "R_EN", "L_EN")):
                d += elm.Wire("z", k=1.6).at(getattr(mcu, pin)).to(getattr(drv, ctrl)).color(SIG)
            _note(d, "B+ 12V from sheet 1", (10.6, cy - 3.0))

            # two motors in parallel across M+ / M-
            mp, mm = getattr(drv, "M+"), getattr(drv, "M-")
            ya, yb = mp[1], mm[1]                  # motor rows sit ON the driver's output pins
            d += elm.Line().at(mp).to((RAIL_A, ya)).color(V12)
            d += elm.Line().at((RAIL_A, ya)).to((RAIL_A, yb)).color(V12)     # the M+ rail
            d += elm.Dot().at((RAIL_A, ya)).color(V12)
            for y, which in ((ya, "front"), (yb, "rear")):
                d += elm.Motor().right().at((MOT_X, y)).length(RAIL_B - RAIL_A).color(
                    INK).label("%s %s" % (side[0], which), fontsize=SMALL, color=INK)
            d += elm.Line().at((RAIL_B, ya)).to((RAIL_B, yb)).color(V12)     # the M- rail
            # M- routed clear of the M+ rail, below the motors
            d += elm.Line().at(mm).to((mm[0] + 0.8, mm[1])).color(V12)
            d += elm.Line().at((mm[0] + 0.8, mm[1])).to((mm[0] + 0.8, cy - 3.4)).color(V12)
            d += elm.Line().at((mm[0] + 0.8, cy - 3.4)).to((RAIL_B, cy - 3.4)).color(V12)
            d += elm.Line().at((RAIL_B, cy - 3.4)).to((RAIL_B, yb)).color(V12)
            d += elm.Dot().at((RAIL_A, yb)).color(V12)
            d += elm.Dot().at((RAIL_B, yb))


# --------------------------------------------------------------------- 3. seeder
def sheet_seeder():
    """The punch is a low-side switch: coil to +12V, MOSFET to ground, flyback diode across
    the coil with its BAND (cathode) at the +12V end. Backwards, it is a dead short.

    Geometry note: the MCU body height sets the pin PITCH, so it has to exceed the height of
    whatever hangs off each pin or the blocks overlap. h=8.0 over 3 pins gives a 2.0 pitch for
    1.6-high blocks. The solenoid driver is pushed out past x=10.5 for the same reason -- the
    coil rises through the rows the servos occupy.
    """
    with _drawing("seeder") as d:
        mcu = d.add(_block("UNO Q\nSTM32U585", right=["D10", "D11", "A3"],
                           w=4.0, h=5.0, show_pins=True).at((0, 0)).anchor("center"))

        for pin, name in (("D10", "S3003 servo\nspool / arm rotate"),
                          ("D11", "SG90 servo\nmetering drum")):
            p = getattr(mcu, pin)
            d += elm.Line().at(p).to((p[0] + 3.0, p[1])).color(SIG).label(
                "PWM", loc="top", fontsize=SMALL, color=INK)
            d += _block(name, left=["sig"], w=5.0, h=1.6).at((p[0] + 3.0, p[1])).anchor("sig")

        # --- the solenoid low-side driver, clear of the servo rows above it
        #
        # MEASURED, because guessing here drew the FET on top of the resistor: with
        # .anchor("gate").at((x, y)), NFet puts its gate lead at x but its BODY 1.37 units to the
        # LEFT, so drain and source land at x - 1.37. The gate therefore has to sit at least
        # 1.37 + clearance to the right of whatever precedes it.
        GATE_X, COIL_X = 17.6, 20.8
        a3 = mcu.A3
        y = a3[1]
        d += elm.Line().at(a3).to((13.5, y)).color(SIG)
        d += elm.Resistor().at((13.5, y)).right().length(2.2).color(PEDGE).fill(PART).label(
            "100R", fontsize=SMALL, color=INK)
        d += elm.Line().at((15.7, y)).to((GATE_X, y)).color(SIG)
        d += elm.Dot().at((GATE_X, y)).color(SIG)
        fet = d.add(elm.NFet(bulk=False).anchor("gate").at((GATE_X, y)).label(
            "IRLZ44N\nlogic-level N-ch", loc="right", ofst=(0.9, -1.0), fontsize=SMALL))
        d += elm.Ground().at(fet.source).color(GND)
        # gate -> GND pulldown, so the coil cannot twitch while A3 floats during MCU reset
        d += elm.Resistor().at((GATE_X, y)).down().length(2.0).color(PEDGE).fill(PART).label(
            "10k", fontsize=SMALL, color=INK)
        d += elm.Ground().at((GATE_X, y - 2.0)).color(GND)

        # drain routed out to the coil's own column
        drain = fet.drain
        d += elm.Line().at(drain).to((drain[0], drain[1] + 1.1)).color(V12)
        d += elm.Line().at((drain[0], drain[1] + 1.1)).to((COIL_X, drain[1] + 1.1)).color(V12)
        low = (COIL_X, drain[1] + 1.1)
        d += elm.Dot().at(low).color(V12)
        d += elm.Inductor2(loops=4).at(low).up().length(3.0).color(PEDGE).label(
            "JF-0530B solenoid\n12V  5N  10mm", loc="left", fontsize=SMALL, color=INK)
        high = (low[0], low[1] + 3.0)
        d += elm.Dot().at(high).color(V12)
        # flyback diode beside the coil; the band (cathode) is the +12V end
        d += elm.Line().at(low).to((low[0] + 3.2, low[1])).color(V12)
        d += elm.Diode().at((low[0] + 3.2, low[1])).up().length(3.0).color(PEDGE).fill(
            PART).label("1N4007", loc="right", fontsize=SMALL, color=INK)
        d += elm.Line().at((high[0] + 3.2, high[1])).to(high).color(V12)
        d += elm.Line().at(high).to((high[0], high[1] + 1.1)).color(V12)
        d += elm.Vdd().at((high[0], high[1] + 1.1)).color(V12).label(
            "+12V   2A fuse, sheet 1", fontsize=SMALL, color=INK)


# ----------------------------------------------------------------------- 4. gyro
def sheet_gyro():
    """The gyro, on Wire2 (i2c3) at A4/A5 -- the only I2C the board actually exposes.

    VCC is the 5V rail from the buck. The module runs its own regulator, so its SDA/SCL idle at
    3.3V rather than 5V, which matters because the UNO Q's analog pins are NOT 5V tolerant.

    No GPS, and that is a decision rather than an omission: consumer GPS is +/-2-5 m across a
    plot 3-6 m wide, so it would be less accurate than dead reckoning and gives no heading at
    rest. The problem was always plot-scale RELATIVE odometry.
    """
    with _drawing("gyro") as d:
        mcu = d.add(_block("UNO Q\nSTM32U585\n14-bit ADC\nVREF+ = 3.3V",
                           right=["A4", "A5"], w=4.6, h=3.6,
                           show_pins=True).at((0, 0)).anchor("center"))

        gy = d.add(_block("MPU-6050   0x68\nWire2 / i2c3\ngyro Z axis only",
                          left=["SDA", "SCL"], top=["VCC"], bottom=["GND"],
                          w=6.0, h=3.6, show_pins=True)
                   .at((mcu.A4[0] + 3.4, mcu.A4[1])).anchor("SDA"))
        d += elm.Line().at(mcu.A4).to(gy.SDA).color(SIG)
        d += elm.Line().at(mcu.A5).to(gy.SCL).color(SIG)

        # VCC and GND are the module's OWN pins -- not taps off the data lines
        d += elm.Line().at(gy.VCC).to((gy.VCC[0], gy.VCC[1] + 1.4)).color(V5)
        d += elm.Vdd().at((gy.VCC[0], gy.VCC[1] + 1.4)).color(V5).label(
            "5V from the buck (sheet 1)", fontsize=SMALL, color=INK)
        d += elm.Line().at(gy.GND).to((gy.GND[0], gy.GND[1] - 1.4)).color(GND)
        d += elm.Ground().at((gy.GND[0], gy.GND[1] - 1.4)).color(GND)


def main():
    os.makedirs(OUT, exist_ok=True)
    for fn in (sheet_power, sheet_drive, sheet_seeder, sheet_gyro):
        fn()
        print("wrote", fn.__name__)
    print("-> %s" % OUT)


if __name__ == "__main__":
    main()
