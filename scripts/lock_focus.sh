#!/bin/sh
# Lock the webcam's focus. Run with the camera MOUNTED and looking at the ground it will follow.
#
#   ssh unoq 'bash -s' < scripts/lock_focus.sh            # find the optimum and lock it
#   ssh unoq 'bash -s' < scripts/lock_focus.sh -- 264     # just set this value, no sweep
#   ssh unoq 'bash -s' < scripts/lock_focus.sh -- --show   # print focus state and exit
#   ssh unoq 'bash -s' < scripts/lock_focus.sh -- --odo     # hand-push odometer calibration
#
# --odo honours PUSH_S (push window, seconds) and REAL_M (the distance you push), e.g.
#   ssh unoq 'PUSH_S=40 REAL_M=1.0 bash -s' < scripts/lock_focus.sh -- --odo
#
# WHY A WRAPPER. The sweep needs OpenCV, which only exists inside the app's venv in the
# container; the V4L2 ioctls need the device, which the running app holds open (UVC allows one
# opener). So this stops the app, runs the sweep in a throwaway container built from the app's
# own image and venv, and starts the app again — including on failure, via the trap below. An
# earlier version left the app stopped when the sweep errored, which looks exactly like the
# board having died.
#
# The container flags are not decoration: --group-add 44 is the `video` group, without which
# OpenCV reports "can't open camera by index" even under --privileged (the nodes are root:video
# 0660). That cost a while to find, so it is spelled out here.
#
# It uses the IMAGE's python (/usr/local/bin/python3), not the app's venv. cv2 ships in the image
# at /usr/local/lib/python3.13/site-packages/cv2 — the venv under .cache is for the app's own
# dependencies and does not carry OpenCV.
#
# NOTE ON PERSISTENCE: focus and autofocus are driver-side controls with NO persistence. They
# reset when the camera is replugged or the board reboots, and the camera powers up with
# autofocus ENABLED. That is handled by scripts/99-farmcam-focus.rules, a udev rule that
# re-applies the value on every plug event — install it once and this script becomes a
# re-measuring tool rather than something to remember at start of day. Verified on the board:
# the settings survive the app closing and reopening the camera, so a plug-time rule is enough.

set -e

IMAGE=ghcr.io/arduino/app-bricks/python-apps-base:0.12.0
APP=/home/arduino/ArduinoApps/motor-control
CONTAINER=motor-control-main-1

NOT_A_CAMERA='venus\|encoder\|decoder\|codec\|m2m\|jpeg'

# Same sysfs-by-name rule as find_uvc_camera() in vision/camera.py: /dev/video indices are NOT
# stable on this board, and a UVC camera exposes two nodes with the same name — only `index` 0
# is the capture node that carries the controls.
find_cam() {
    fallback=""
    for d in /sys/class/video4linux/video*; do
        [ -e "$d/name" ] || continue
        name=$(tr 'A-Z' 'a-z' < "$d/name")
        echo "$name" | grep -q "$NOT_A_CAMERA" && continue
        idx=$(cat "$d/index" 2>/dev/null || echo 0)
        if [ "$idx" = "0" ]; then echo "/dev/$(basename "$d")"; return 0; fi
        [ -z "$fallback" ] && fallback="/dev/$(basename "$d")"
    done
    [ -n "$fallback" ] && { echo "$fallback"; return 0; }
    return 1
}

DEV=$(find_cam) || { echo "ERROR: no UVC camera found"; exit 1; }
echo "camera: $DEV  ($(cat "/sys/class/video4linux/$(basename "$DEV")/name"))"

ODO=0
case "${1:-}" in
--show)
    v4l2-ctl -d "$DEV" -C focus_automatic_continuous,focus_absolute
    exit 0
    ;;
--odo)
    # Handled below. Consumed HERE so it cannot fall through to the "bare number means set this
    # focus" branch, which would reject it as "not a focus value" — it did exactly that once.
    ODO=1
    shift
    ;;
esac

# A bare number means "just set it" — no sweep, no stopping the app. Controls are settable
# while another process is streaming, so this is safe to run mid-session.
if [ -n "${1:-}" ]; then
    case "$1" in
    ''|*[!0-9]*) echo "ERROR: '$1' is not a focus value (0-1023)"; exit 1 ;;
    esac
    v4l2-ctl -d "$DEV" -c focus_automatic_continuous=0
    v4l2-ctl -d "$DEV" -c focus_absolute="$1"
    echo "set, device reports:"
    v4l2-ctl -d "$DEV" -C focus_automatic_continuous,focus_absolute
    exit 0
fi

PAYLOAD=/probe/lock_focus.py
if [ "$ODO" = "1" ]; then
    PAYLOAD=/probe/calib_odometer.py
    [ -f /tmp/calib_odometer.py ] || { echo "ERROR: scp scripts/calib_odometer.py unoq:/tmp/ first"; exit 1; }
fi

command -v docker >/dev/null || { echo "ERROR: docker not found"; exit 1; }
if [ "$PAYLOAD" = "/probe/lock_focus.py" ]; then
    [ -f /tmp/lock_focus.py ] || { echo "ERROR: copy scripts/lock_focus.py to /tmp first:"; \
        echo "  scp scripts/lock_focus.py unoq:/tmp/"; exit 1; }
fi

# Restart the app WHATEVER happens — a sweep that dies must not leave the robot headless.
restart_app() {
    echo "--- restarting the console app ---"
    docker start "$CONTAINER" >/dev/null 2>&1 || echo "WARNING: could not restart $CONTAINER"
}
trap restart_app EXIT INT TERM

echo "stopping the app so the camera is free ..."
docker stop "$CONTAINER" >/dev/null

# WAIT FOR THE DEVICE TO ACTUALLY BE FREE, do not sleep and hope. `docker stop` returns when the
# container is gone, but the UVC node can still be held for a moment afterwards, and a UVC device
# allows exactly one opener — so the payload died with "could not open camera index 0" on a run
# launched shortly after an app restart, while the identical command had worked minutes earlier.
# A fixed `sleep 5` makes that failure random, which is the worst kind.
printf "waiting for %s to be released" "$DEV"
i=0
while fuser "$DEV" >/dev/null 2>&1; do
    i=$((i + 1))
    if [ "$i" -gt 30 ]; then
        echo
        echo "ERROR: $DEV is still held after 15s. Who has it:"
        fuser -v "$DEV" 2>&1 | head -5
        exit 1
    fi
    printf "."
    sleep 0.5
done
echo " free."
sleep 1                 # a beat for the driver to settle after the last close

# NO -it. This is normally invoked as `ssh unoq 'bash -s' < scripts/lock_focus.sh`, where stdin
# is the script itself, so there is no TTY and docker fails outright with "the input device is
# not a TTY". calib_odometer.py therefore uses a timed push window instead of waiting for Ctrl-C.
# EVERY VARIABLE THE PAYLOAD READS IS PASSED EXPLICITLY, with an explicit value, and its default
# resolved HERE rather than in the payload.
#
# PUSH_S=40 was ignored and the window ran for 25s with no error anywhere. The cause was dull:
# the -e flags were simply absent from this docker run — an edit meant to add them matched
# nothing and reported success. OUT_DIR was missing the same way, which is why the focus sweep's
# proof frame kept landing in the container's own /tmp and dying with --rm.
#
# Passing `-e VAR="$VAR"` with the value spelled out, and defaulting here, makes both failures
# impossible to repeat silently: if the variable is unset the default is visible in this file,
# and the value the container receives is the one this script decided on rather than whatever
# happened to be exported.
: "${PUSH_S:=25}"
: "${REAL_M:=1.0}"

docker run --rm --privileged --user root --group-add 44 \
    -e CAM_DEV="$DEV" -e OUT_DIR=/probe \
    -e PUSH_S="$PUSH_S" -e REAL_M="$REAL_M" \
    -v "$APP":/app -v /tmp:/probe \
    --entrypoint /usr/local/bin/python3 "$IMAGE" "$PAYLOAD" \
    2>&1 | grep --line-buffered -viE "gstreamer|INFO -|^\[ WARN" || true
