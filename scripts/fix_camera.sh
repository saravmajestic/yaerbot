#!/bin/sh
# Unstick the USB webcam when it goes BLANK WHITE. Runs on the board.
#
#   ssh unoq 'bash -s' < scripts/fix_camera.sh          # from the Mac, no install
#   ./fix_camera.sh                                     # on the board
#   ./fix_camera.sh --show                              # just print the controls
#   ./fix_camera.sh --pin                               # stop it recurring: fix WB+exposure
#
# WHY THIS WORKS. The Logitech C310's AUTOMATIC WHITE BALANCE latches on Linux and the sensor
# output saturates to white. It is a known uvcvideo/C310 quirk, not a fault in the robot's code
# and not the `Failed to resubmit video URB` message in dmesg (that is a separate, mostly
# benign USB-controller scheduling complaint). The fix every Linux thread lands on is to turn
# the auto-WB control off and back on, which forces the driver to re-run the algorithm. No
# replug, no restart, and the app can keep streaming throughout.
#
# WHY IT FINDS THE DEVICE INSTEAD OF TAKING A NUMBER. /dev/video indices are NOT stable on this
# board — the camera has been video0 and video2 on different boots of the same hardware, with
# the Qualcomm Venus codec taking the other pair. A hardcoded number silently targets the codec,
# where these controls do not exist. Same sysfs-by-name rule as find_uvc_camera() in
# vision/camera.py: skip anything that is a codec, and prefer `index` 0, because a UVC camera
# exposes two nodes with the SAME name (capture + metadata) and only the capture node carries
# the controls.

set -e

NOT_A_CAMERA='venus\|encoder\|decoder\|codec\|m2m\|jpeg'

find_cam() {
    fallback=""
    for d in /sys/class/video4linux/video*; do
        [ -e "$d/name" ] || continue
        name=$(tr 'A-Z' 'a-z' < "$d/name")
        echo "$name" | grep -q "$NOT_A_CAMERA" && continue
        dev="/dev/$(basename "$d")"
        idx=$(cat "$d/index" 2>/dev/null || echo 0)
        if [ "$idx" = "0" ]; then
            echo "$dev"
            return 0
        fi
        [ -z "$fallback" ] && fallback="$dev"
    done
    # No index-0 node (shouldn't happen on a UVC cam, but don't fail silently)
    [ -n "$fallback" ] && { echo "$fallback"; return 0; }
    return 1
}

# Control names moved between kernel versions: white_balance_temperature_auto ->
# white_balance_automatic, exposure_auto -> auto_exposure. Ask the driver which it has rather
# than guessing, or this script "succeeds" while setting nothing.
ctrl_exists() { echo "$CTRLS" | grep -q "^ *$1 "; }

pick_ctrl() {
    for c in "$@"; do
        ctrl_exists "$c" && { echo "$c"; return 0; }
    done
    return 1
}

DEV=$(find_cam) || { echo "ERROR: no UVC camera found under /sys/class/video4linux"; exit 1; }
NAME=$(cat "/sys/class/video4linux/$(basename "$DEV")/name")
echo "camera: $DEV  ($NAME)"

command -v v4l2-ctl >/dev/null || { echo "ERROR: v4l2-ctl not installed"; exit 1; }
CTRLS=$(v4l2-ctl -d "$DEV" --list-ctrls 2>/dev/null)
[ -n "$CTRLS" ] || { echo "ERROR: $DEV exposes no controls — wrong node?"; exit 1; }

AWB=$(pick_ctrl white_balance_automatic white_balance_temperature_auto) || {
    echo "ERROR: no auto-white-balance control on this camera"; exit 1; }
WBT=$(pick_ctrl white_balance_temperature || true)
AE=$(pick_ctrl auto_exposure exposure_auto || true)
ET=$(pick_ctrl exposure_time_absolute exposure_absolute || true)
# GAIN IS AUTO-CONTROLLED IN APERTURE PRIORITY, AND IT DOES NOT SAY SO.
#
# It carries no `inactive` flag, so it looks independently settable, and a write appears to
# succeed. It does not. Verified on the board:
#     v4l2-ctl -c gain=64        -> exit 0
#     v4l2-ctl --get-ctrl gain   -> 64     <- the DRIVER's cached value
#     v4l2-ctl -C gain           -> 192    <- what the DEVICE actually has
# The camera's own auto-exposure algorithm drives it straight back. So a high gain reading under
# auto exposure is the camera responding to the scene, NOT a value left frozen by a previous --pin,
# and there is nothing here to reset.
#
# THE TRAP WORTH REMEMBERING: `--get-ctrl` reports what the driver cached and will happily echo a
# value the hardware ignored. `-C` (capital) queries the device. Use `-C` to verify anything you
# set on this camera, or you will confirm your own write and learn nothing.
#
# Gain is therefore only worth pinning alongside MANUAL exposure, where the auto algorithm is not
# fighting you.
GAIN=$(pick_ctrl gain || true)
GAIN_DEFAULT=64

show() {
    for c in "$AWB" "$WBT" "$AE" "$ET" gain brightness; do
        [ -n "$c" ] && v4l2-ctl -d "$DEV" --list-ctrls 2>/dev/null | grep "^ *$c " || true
    done
}

case "${1:-}" in
--show)
    show
    exit 0
    ;;
--pin)
    # Deterministic image: stops the latch recurring, at the cost of adapting to changing
    # light. Good for a demo run in steady conditions, bad if the sun moves during it.
    echo "pinning white balance and exposure (run with no arguments to go back to auto)"
    v4l2-ctl -d "$DEV" -c "$AWB"=0
    [ -n "$WBT" ] && v4l2-ctl -d "$DEV" -c "$WBT"=4000
    if [ -n "$AE" ]; then
        # menu value 1 = Manual Mode on uvcvideo; 3 = Aperture Priority (the auto default)
        v4l2-ctl -d "$DEV" -c "$AE"=1 || echo "  (could not set manual exposure)"
        [ -n "$ET" ] && v4l2-ctl -d "$DEV" -c "$ET"=166
    fi
    # Pin gain too, or the value the auto algorithm had ramped to stays frozen in place.
    [ -n "$GAIN" ] && v4l2-ctl -d "$DEV" -c "$GAIN"=$GAIN_DEFAULT
    echo "after:"
    show
    exit 0
    ;;
esac

echo "before:"
show
echo "toggling $AWB off -> on ..."
v4l2-ctl -d "$DEV" -c "$AWB"=0
sleep 1
v4l2-ctl -d "$DEV" -c "$AWB"=1
sleep 1
# Also hand exposure back to auto, in case a previous --pin left it manual and that is what
# is making the picture wrong now.
[ -n "$AE" ] && v4l2-ctl -d "$DEV" -c "$AE"=3 2>/dev/null || true
# Gain is deliberately NOT touched here: under Aperture Priority the camera owns it and drives
# any write straight back (see the note above). Setting it would look like it worked and change
# nothing.
echo "after:"
show
echo
echo "Look at the Camera tab. If it is still white, replug the camera's USB and re-run;"
echo "if it is white every time, the cable or the 5V feed is the next thing to swap."
