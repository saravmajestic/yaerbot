#!/usr/bin/env bash
# Install the yaerbot on-board services so they survive reboots.
# Run ON THE UNO Q with sudo:  sudo bash ~/yaerbot/deploy/install-services.sh
#
# Installs:
#   ollama.service          — the on-device LLM server (:11434)
#   yaerbot-planner.service — the Act-1 planner + chat backend (:8765), After=ollama
# The operator console autostarts already (App Lab docker app), so it's not included here.
set -euo pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"

# free the ports if the nohup dev processes are still holding them
fuser -k 11434/tcp 2>/dev/null || true
fuser -k 8765/tcp  2>/dev/null || true

install -m 644 "$HERE/systemd/ollama.service"          /etc/systemd/system/ollama.service
install -m 644 "$HERE/systemd/yaerbot-planner.service" /etc/systemd/system/yaerbot-planner.service

systemctl daemon-reload
systemctl enable --now ollama.service yaerbot-planner.service

sleep 4
systemctl --no-pager --plain status ollama.service yaerbot-planner.service | grep -E "●|Active:" || true
echo "done — services enabled (start on boot) and running."
