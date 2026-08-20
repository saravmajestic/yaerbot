# yaerbot — notes for Claude

Contest submission repo for the **Arduino Physical AI Challenge India 2026**: a farm
**seeding robot** on an Arduino UNO Q. The 4-act demo plan and build status live in the
ai-labs repo: `ai-labs/apps/farm-robot/docs/farm-os/demo-plan.md`.
This file documents how the on-board pieces fit together and how to run/deploy them.

## Repo layout
- `farmos/` — the software brain (hardware-free, unit-tested):
  - `planner/` — **Act 1**: reconciles the Tamil panchangam (Nokku Naal) + biodynamic
    calendar → a sowing date + alternatives; real cached calendars, mock prices, live weather.
  - `path.py` / `executor.py` / `report.py` — **Act 2 + 4**: boustrophedon path, timed
    dead-reckoning executor, SVG farm-map report.
- `webchat/` — the **planner service** (server + chat UI); backend for the console's Plan tab.
- `console/` — the **operator console** App Lab app (`assets/` = tabbed web UI incl. Plan;
  `python/main.py` = Socket.IO + RouterBridge RPCs; `app.yaml`, `sketch/sketch.yaml`).
- `docs/`, `scripts/` (`bench_llm.py`, `gen_mock_prices.py`), `tests/`, `examples/`.

## The board (Arduino UNO Q, `ssh unoq` → `farm-os.local`, user `arduino`)
- 4 GB RAM, 32 GB eMMC **partitioned**: `/` root ≈ 10 GB (tight, ~3 GB free) · **`/home/arduino`
  ≈ 18 GB (roomy)**. Keep large things (Ollama, models) on `/home`, never on `/`.
- Two networks it may be on: home WiFi or the field hotspot. Reach it at `farm-os.local`.

## Three runtime pieces (start in this order)

### 1) Ollama service — the on-device LLM
- Installed under **`~/ollama-dist/`** (binary + libs, on `/home`; NOT the tiny root partition).
- Model: **`qwen2.5:1.5b`** in `~/.ollama/models` (chosen over Gemma 2B — ~2× faster on this
  CPU with acceptable quality; agentic tool-calling at ≤2B is unreliable so we only use it to
  *present* pre-computed data). Speed reality: **~2 tok/s** (A53 CPU-bound) — pre-warm it and
  rely on streaming + the instant card. RAM: model resident ≈ 1 GB, fits fine.
- Start (env flags tune KV + keep it warm):
  ```bash
  ssh unoq 'export OLLAMA_MODELS=$HOME/.ollama/models OLLAMA_FLASH_ATTENTION=1 \
    OLLAMA_KV_CACHE_TYPE=q8_0 OLLAMA_KEEP_ALIVE=30m; \
    nohup ~/ollama-dist/bin/ollama serve >~/ollama.log 2>&1 </dev/null &'
  # check: curl -s localhost:11434/api/tags ; ~/ollama-dist/bin/ollama list
  ```
- Listens on `localhost:11434`. Models are portable GGUF blobs — pull on a fast machine and
  `scp ~/.ollama/models` across if the board's link is slow (it is).

### 2) Planner service (`webchat/`) — Act 1 backend + chat
- Pure Python (stdlib + `farmos`); calls Ollama at `localhost:11434` and the `farmos.planner`.
- Deploy + run on the **host** (not the container):
  ```bash
  scp -r farmos webchat unoq:~/yaerbot/
  ssh unoq 'cd ~/yaerbot && PLANNER_HOST=0.0.0.0 \
    nohup python3 webchat/server.py 8765 >~/webchat.log 2>&1 </dev/null &'
  ```
- Endpoints (CORS enabled so the console at `:7000` can call it): `POST /api/plan` (instant
  deterministic card data, no LLM), `POST /api/narrate` (streams the LLM prose token-by-token),
  `GET /api/crops`. Standalone UI at `http://farm-os.local:8765`.
- Env: `PLANNER_HOST` (0.0.0.0 for LAN), `PLANNER_MODEL` (default `qwen2.5:1.5b`),
  `PLANNER_AFTER` (demo date), `PLANNER_LOCATION` (weather; default `salem`).

### 3) Operator console (`console/`) — the tabbed UI
- App Lab app on the board at **`~/ArduinoApps/motor-control/`** (mounted into docker
  `motor-control-main-1` as `/app`), served at **`http://farm-os.local:7000`**.
  `~/motor-control` is a **symlink** to it, so the paths below still work — App Lab's apps
  directory moved to `~/ArduinoApps/` in the Aug-2026 runtime upgrade and it only discovers
  apps there. The trained emitter model is bound in **`console/app.yaml`**
  (`arduino:object_detection: {model: ei-model-1088852-1}`), not in the App Lab UI —
  see `../ai-labs/apps/farm-robot/docs/farm-os/ml-emitter-model.md`. Tabs: Drive / Seed / Soil / Cam /
  **Plan** / Settings. Drive/Seed/Soil/Cam/Settings talk to the STM32 via RouterBridge RPCs
  (`python/main.py`). The **Plan tab** is pure frontend that calls the planner service at
  `http://<host>:8765` (`/api/plan` + `/api/narrate`).
- Deploy a UI change (fast, no MCU reflash):
  ```bash
  scp console/assets/* unoq:/home/arduino/motor-control/assets/
  ssh unoq 'docker restart motor-control-main-1'
  ```

## Start-up (autostart on boot)
- Order: **Ollama → planner → console**. The Plan tab needs the planner (:8765); the planner
  needs Ollama (:11434).
- **Ollama + planner run as systemd services** (`deploy/systemd/*.service`), enabled to start on
  boot. Install/reinstall on the board:
  ```bash
  scp -r deploy unoq:~/yaerbot/ && ssh unoq 'sudo bash ~/yaerbot/deploy/install-services.sh'
  ```
  Manage: `systemctl {status,restart} ollama yaerbot-planner` · logs: `journalctl -u <svc> -f`.
- The **console** autostarts already (App Lab docker app, restart policy).
- So the whole stack recovers after a power-cycle. Still **pre-warm the model** before recording
  (one request) so the first on-camera answer isn't the ~15 s cold-load.

## References
- Demo storyboard + build status: `ai-labs/apps/farm-robot/docs/farm-os/demo-plan.md` (**ai-labs repo**, not this one)
- On-device LLM findings (speed, lean prompt, model choice): `scripts/bench_llm.py`, and
  the demo plan's §B1 in the ai-labs repo
- Prices are **MOCK** (`farmos/planner/market.py`, loudly flagged) — real data.gov.in Agmarknet
  path is stubbed for a one-line swap. Weather is **real** (Open-Meteo, no key).
