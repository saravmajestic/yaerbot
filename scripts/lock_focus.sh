#!/bin/sh
# Lock the webcam's focus. Run with the camera MOUNTED and looking at the ground it will follow.
#
#   ssh unoq 'bash -s' < scripts/lock_focus.sh            # find the optimum and lock it
#   ssh unoq 'bash -s' < scripts/lock_focus.sh -- 264     # just set this value, no sweep
#   ssh unoq 'bash -s' < scripts/lock_focus.sh -- --show   # print focus state and exit
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
# NOTE ON PERSISTENCE: focus and autofocus are driver-side controls with NO persistence. They
# reset when the camera is replugged or the board reboots, and the camera powers up with
# autofocus ENABLED. Re-run this after either. If it becomes routine, a udev rule is the fix.

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

case "${1:-}" in
--show)
    v4l2-ctl -d "$DEV" -C focus_automatic_continuous,focus_absolute
    exit 0
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

command -v docker >/dev/null || { echo "ERROR: docker not found"; exit 1; }
[ -x "$APP/.cache/.venv/bin/python3" ] || { echo "ERROR: app venv missing at $APP"; exit 1; }
[ -f /tmp/lock_focus.py ] || { echo "ERROR: copy scripts/lock_focus.py to /tmp first:"; \
    echo "  scp scripts/lock_focus.py unoq:/tmp/"; exit 1; }

# Restart the app WHATEVER happens — a sweep that dies must not leave the robot headless.
restart_app() {
    echo "--- restarting the console app ---"
    docker start "$CONTAINER" >/dev/null 2>&1 || echo "WARNING: could not restart $CONTAINER"
}
trap restart_app EXIT INT TERM

echo "stopping the app so the camera is free ..."
docker stop "$CONTAINER" >/dev/null
sleep 5

CAM_DEV="$DEV" docker run --rm --privileged --user root --group-add 44 \
    -e CAM_DEV="$DEV" \
    -v "$APP":/app -v /tmp:/probe \
    --entrypoint "$APP/.cache/.venv/bin/python3" "$IMAGE" /probe/lock_focus.py \
    2>&1 | grep -viE "gstreamer|INFO -|^\[ WARN"
