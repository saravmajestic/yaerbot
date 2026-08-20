#!/usr/bin/env python3
"""Generate the system architecture diagram as SVG.

    python3 scripts/gen_arch.py          # -> docs/architecture.svg

WHY IT LOOKS LIKE THIS. The organising axis is TIME-CRITICALITY, not a component inventory.
The split between the two halves of the UNO Q exists for one reason: the solenoid punch must
hold for exactly 500 ms while a neural network runs on the same board. On a single processor
those two compete; here they do not, because the punch is one RPC the STM32 executes
atomically. So the Linux side is drawn as the half with no deadlines, and the MCU as the half that
is timing to the millisecond -- and the RouterBridge is the boundary between those two
worlds, labelled with what crosses it in each direction.

DELIBERATELY ABSENT: soil probes and GPS (not fitted), the ESP32-CAM (superseded by USB), the
battery monitor (disabled -- the gyro took its pin), and every pin number (that is what
docs/schematic/ is for). The omissions are what keep this readable at six seconds on screen.

CANVAS IS 1280x720 -- 16:9, so the PNG drops into the video edit with no letterboxing. Type
sizes are floored at 13px in that space so the smallest label survives 1080p.

Palette is shared with scripts/gen_schematic.py on purpose: the two documents should read as
one set.
"""
import os

OUT = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                   "docs", "architecture.svg")

W, H = 1280, 720

PAPER = "#EDF1F5"
PLATE = "#FFFFFF"
INK = "#1F2A36"
MUTED = "#5C6875"
HAIR = "#C4CDD6"
V12 = "#C0392B"      # battery / power
SIG = "#2E6FB0"      # data, logic, RPCs
ACCENT = "#2E8B57"   # the two AI workloads
GND = "#5C6875"
PART = "#E8D8A8"     # passive / peripheral fill
PEDGE = "#A6844A"
LANE_SLOW = "#F2F6FA"   # the no-deadline half
LANE_FAST = "#FBF4EC"   # the deadline-bound half

FONT = ("ui-sans-serif,-apple-system,'Segoe UI',Roboto,'Helvetica Neue',"
        "Helvetica,Arial,sans-serif")
MONO = "ui-monospace,'SF Mono',Menlo,Consolas,monospace"

# ---- board geometry
BX, BY, BW, BH = 290, 64, 660, 526
LX, LW = BX + 16, BW - 32                 # lane x / width
LIN_Y, LIN_H = 104, 182                   # Linux lane
BR_Y, BR_H = 318, 54                      # RouterBridge band
MCU_Y, MCU_H = 386, 162                   # STM32 lane

COL_L, COL_LW = 28, 222                   # left column
COL_R, COL_RW = 990, 262                  # right column

_p = []


def esc(s):
    return (s.replace("&", "&amp;").replace("<", "&lt;").replace(">", "&gt;"))


def rect(x, y, w, h, fill, stroke=None, rx=8, sw=1.5, dash=None):
    d = ' stroke-dasharray="%s"' % dash if dash else ""
    st = ' stroke="%s" stroke-width="%s"' % (stroke, sw) if stroke else ""
    _p.append('<rect x="%g" y="%g" width="%g" height="%g" rx="%g" fill="%s"%s%s/>'
              % (x, y, w, h, rx, fill, st, d))


def text(x, y, s, size=15, fill=INK, weight="400", anchor="start", family=FONT,
         spacing=None, style=None):
    extra = ""
    if spacing:
        extra += ' letter-spacing="%s"' % spacing
    if style:
        extra += ' font-style="%s"' % style
    _p.append('<text x="%g" y="%g" font-family="%s" font-size="%g" font-weight="%s" '
              'fill="%s" text-anchor="%s"%s>%s</text>'
              % (x, y, family, size, weight, fill, anchor, extra, esc(s)))


def arrow(x1, y1, x2, y2, color=SIG, sw=2.2, marker="end", dash=None):
    d = ' stroke-dasharray="%s"' % dash if dash else ""
    mid = ""
    if marker in ("end", "both"):
        mid += ' marker-end="url(#a-%s)"' % color.lstrip("#")
    if marker in ("start", "both"):
        mid += ' marker-start="url(#s-%s)"' % color.lstrip("#")
    _p.append('<line x1="%g" y1="%g" x2="%g" y2="%g" stroke="%s" stroke-width="%g" '
              'stroke-linecap="round"%s%s/>' % (x1, y1, x2, y2, color, sw, mid, d))


def chip(x, y, w, h, label, sub=None, ai=False):
    """A unit of work inside a lane. `ai` badges the two neural workloads."""
    rect(x, y, w, h, PLATE, ACCENT if ai else HAIR, rx=7, sw=2 if ai else 1.3)
    cy = y + (h / 2 + 5 if not sub else h / 2 - 3)
    text(x + w / 2, cy, label, 15, INK, "600", "middle")
    if sub:
        text(x + w / 2, y + h / 2 + 16, sub, 12.5, MUTED, "400", "middle")
    if ai:
        rect(x + w - 30, y - 9, 26, 18, ACCENT, None, rx=9)
        text(x + w - 17, y + 4, "AI", 11, "#FFFFFF", "700", "middle")


def build():
    _p.append('<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 %d %d" '
              'width="%d" height="%d" role="img" '
              'aria-label="Farm OS system architecture: one Arduino UNO Q, a Qualcomm Linux '
              'side running the console, OpenCV tube detection, a FOMO emitter model and an '
              'LLM planner, joined over RouterBridge to an STM32U585 that drives the motors, '
              'servos, solenoid punch and gyro.">' % (W, H, W, H))
    # arrow markers, one per colour used
    _p.append("<defs>")
    for c in (SIG, V12, ACCENT, MUTED):
        for pre, refx, path in (("a", 8, "M0,0 L9,3.5 L0,7 Z"), ("s", 1, "M9,0 L0,3.5 L9,7 Z")):
            _p.append('<marker id="%s-%s" viewBox="0 0 10 8" refX="%g" refY="3.5" '
                      'markerWidth="7" markerHeight="6" orient="auto-start-reverse">'
                      '<path d="%s" fill="%s"/></marker>'
                      % (pre, c.lstrip("#"), refx, path, c))
    _p.append("</defs>")
    rect(0, 0, W, H, PAPER, rx=0)

    # ---------------------------------------------------------------- the board
    rect(BX, BY, BW, BH, PLATE, INK, rx=14, sw=2.5)
    text(BX + 20, BY + 34, "Arduino UNO Q", 25, INK, "700")
    # right-aligned: guessing the title's rendered width put this on top of it
    text(BX + BW - 20, BY + 34, "one board · nothing else", 14, MUTED, "400",
         anchor="end")

    # ---- Linux half: nothing here has a deadline
    rect(LX, LIN_Y, LW, LIN_H, LANE_SLOW, HAIR, rx=10, sw=1.3)
    text(LX + 16, LIN_Y + 26, "Qualcomm QRB2210 · Linux", 18, INK, "700")
    text(LX + 16, LIN_Y + 45, "thinking — no deadlines", 13, MUTED, "400",
         style="italic")
    cw, gap = (LW - 32 - 3 * 12) / 4.0, 12
    for i, (lbl, sub, ai) in enumerate([
            ("Web console", "operator UI", False),
            ("Tube detection", "OpenCV", False),
            ("Emitter model", "Edge Impulse FOMO", True),
            ("Crop planner", "on-device LLM", True)]):
        chip(LX + 16 + i * (cw + gap), LIN_Y + 62, cw, 62, lbl, sub, ai)
    text(LX + 16, LIN_Y + 158, "path planner · run log · farm-map report",
         13, MUTED, "400", family=MONO)

    # ---- the boundary
    rect(LX, BR_Y, LW, BR_H, PLATE, SIG, rx=9, sw=2)
    text(LX + 16, BR_Y + 33, "RouterBridge", 17, SIG, "700")
    arrow(LX + 200, BR_Y - 16, LX + 200, BR_Y + 8, SIG)
    text(LX + 212, BR_Y - 4, "15 RPCs — every actuator command", 13, INK, "500")
    arrow(LX + 200, BR_Y + BR_H + 16, LX + 200, BR_Y + BR_H - 8, MUTED)
    text(LX + 212, BR_Y + BR_H + 12,
         "getDiag — what actually reached the driver pins", 13, MUTED, "400")

    # ---- MCU half: every one of these is timed
    rect(LX, MCU_Y, LW, MCU_H, LANE_FAST, PEDGE, rx=10, sw=1.3)
    text(LX + 16, MCU_Y + 26, "STM32U585 · microcontroller", 18, INK, "700")
    text(LX + 16, MCU_Y + 45, "acting — to the millisecond", 13, MUTED, "400",
         style="italic")
    for i, (lbl, sub) in enumerate([
            ("Motor PWM", "4WD skid-steer"),
            ("Two servos", "spool + drum"),
            ("Solenoid punch", "500 ms, atomic"),
            ("Gyro I²C", "closed-loop heading")]):
        chip(LX + 16 + i * (cw + gap), MCU_Y + 62, cw, 62, lbl, sub)

    # ---------------------------------------------------------------- left column
    # no cloud
    rect(COL_L, 96, COL_LW, 58, PLATE, ACCENT, rx=10, sw=2)
    text(COL_L + COL_LW / 2, 122, "No cloud", 17, ACCENT, "700", "middle")
    text(COL_L + COL_LW / 2, 141, "both models run on the board", 12.5, MUTED, "400", "middle")

    # operator
    rect(COL_L, 196, COL_LW, 74, PLATE, HAIR, rx=10, sw=1.3)
    text(COL_L + COL_LW / 2, 222, "Operator's browser", 15, INK, "600", "middle")
    text(COL_L + COL_LW / 2, 241, "phone or laptop", 12.5, MUTED, "400", "middle")
    text(COL_L + COL_LW / 2, 258, "own hotspot, or your network", 12.5, MUTED, "400", "middle")
    arrow(COL_L + COL_LW, 233, BX - 6, 233, SIG)
    text((COL_L + COL_LW + BX) / 2, 224, "WiFi", 12.5, SIG, "600", "middle")

    # power
    rect(COL_L, 420, COL_LW, 108, PLATE, V12, rx=10, sw=1.6)
    text(COL_L + COL_LW / 2, 446, "3S LiPo", 15, INK, "600", "middle")
    text(COL_L + COL_LW / 2, 468, "20 A main → branch fuses", 12.5, MUTED, "400", "middle")
    text(COL_L + COL_LW / 2, 486, "12 V → 5 V buck", 12.5, MUTED, "400", "middle")
    text(COL_L + COL_LW / 2, 508, "a fuse per branch, per wire", 12, V12, "600", "middle")
    arrow(COL_L + COL_LW, 474, BX - 6, 474, V12)

    # ---------------------------------------------------------------- right column
    # camera -> Linux
    rect(COL_R, 116, COL_RW, 74, PART, PEDGE, rx=10, sw=1.5)
    text(COL_R + COL_RW / 2, 142, "USB camera", 15, INK, "600", "middle")
    text(COL_R + COL_RW / 2, 161, "320×240 at 30 fps", 12.5, MUTED, "400", "middle")
    text(COL_R + COL_RW / 2, 178, "focus locked by udev", 12.5, MUTED, "400", "middle")
    arrow(COL_R, 153, BX + BW + 6, 153, SIG, marker="start")

    # gyro <-> MCU
    rect(COL_R, 300, COL_RW, 62, PART, PEDGE, rx=10, sw=1.5)
    text(COL_R + COL_RW / 2, 326, "MPU-6050 gyro", 15, INK, "600", "middle")
    text(COL_R + COL_RW / 2, 345, "Wire2 on A4/A5", 12.5, MUTED, "400", "middle")
    arrow(COL_R, 331, BX + BW + 6, 331, SIG, marker="both")

    # drive
    rect(COL_R, 390, COL_RW, 74, PART, PEDGE, rx=10, sw=1.5)
    text(COL_R + COL_RW / 2, 416, "2× IBT-2 drivers", 15, INK, "600", "middle")
    text(COL_R + COL_RW / 2, 435, "4 gear motors", 12.5, MUTED, "400", "middle")
    text(COL_R + COL_RW / 2, 452, "one per side, 2 in parallel", 12.5, MUTED, "400", "middle")
    arrow(BX + BW + 6, 427, COL_R - 6, 427, V12)

    # seeder
    rect(COL_R, 486, COL_RW, 74, PART, PEDGE, rx=10, sw=1.5)
    text(COL_R + COL_RW / 2, 512, "Seeder", 15, INK, "600", "middle")
    text(COL_R + COL_RW / 2, 531, "S3003 spool · SG90 drum", 12.5, MUTED, "400", "middle")
    text(COL_R + COL_RW / 2, 548, "solenoid punch", 12.5, MUTED, "400", "middle")
    arrow(BX + BW + 6, 523, COL_R - 6, 523, V12)

    # ---------------------------------------------------------------- the two claims
    y0 = 622
    rect(COL_L, y0, W - 2 * COL_L, 72, PLATE, HAIR, rx=10, sw=1.3)
    text(COL_L + 20, y0 + 27,
         "The punch holds for 500 ms, timed by the microcontroller — a "
         "garbage-collection pause on the Linux side cannot stretch it.", 14, INK, "500")
    text(COL_L + 20, y0 + 52,
         "Both AI models run here, staggered and never concurrent: about 1 GB is free with a "
         "model resident. Nothing leaves the robot.", 14, MUTED, "400")

    _p.append("</svg>")
    return "\n".join(_p)


def main():
    os.makedirs(os.path.dirname(OUT), exist_ok=True)
    open(OUT, "w", encoding="utf-8").write(build())
    print("wrote %s (%.1f KB)" % (OUT, os.path.getsize(OUT) / 1024))


if __name__ == "__main__":
    main()
