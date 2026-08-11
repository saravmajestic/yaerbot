const socket = io(`http://${window.location.host}`);

const dot        = document.getElementById('dot');
const statusText = document.getElementById('status-text');
const speedSlider = document.getElementById('speed-slider');
const speedVal   = document.getElementById('speed-val');

// ── Connection status ──────────────────────────────────────────────────────
socket.on('connect', () => {
  dot.className = 'status-dot connected';
  statusText.textContent = 'Connected';
  socket.emit('get_stats', {});          // prime the stat chips immediately
  const camUrl = (document.getElementById('cam-url').value || '').trim();
  if (camUrl) socket.emit('set_camera', { url: camUrl });  // start the cam loop (Drive + Cam show video)
});

socket.on('disconnect', () => {
  dot.className = 'status-dot disconnected';
  statusText.textContent = 'Disconnected';
  socket.emit('motor_stop', {});
});

// ── Tabs ─────────────────────────────────────────────────────────────────────
let currentTab = 'drive';

// Drive-tab live cam view: reuses the backend's pushed annotated frames (the same
// 'cam_frame' the Cam tab uses), NOT a 2nd direct stream — the ESP32-CAM serves
// only one MJPEG client and the backend already holds it. Frames arrive while the
// Drive tab polls get_vision (below); here we just clear the image when leaving.
function setDriveCam(on) {
  const dc = document.getElementById('drive-cam');
  if (dc && !on) dc.src = '';            // blank it when off Drive (CSS hides empty src)
}

document.querySelectorAll('.tab').forEach(tab => {
  tab.addEventListener('click', () => {
    const name = tab.dataset.tab;
    currentTab = name;
    document.querySelectorAll('.tab').forEach(t => t.classList.toggle('active', t === tab));
    document.querySelectorAll('.panel').forEach(p =>
      p.classList.toggle('active', p.id === `panel-${name}`));
    socket.emit('motor_stop', {});        // safety: leaving Drive stops the robot
    setDriveCam(name === 'drive');        // start/stop the Drive-tab direct stream
    if (name === 'soil')   socket.emit('get_soil', {});    // prime readings immediately
    if (name === 'diag')   socket.emit('get_diag', {});
    if (name === 'seed')   socket.emit('get_plot', {});
    if (name === 'camera') { socket.emit('get_vision', {}); socket.emit('get_capture', {}); }
  });
});
setDriveCam(true);                        // Drive is the default active tab on load

// ── D-pad buttons ──────────────────────────────────────────────────────────
document.querySelectorAll('.btn-dir').forEach(btn => {
  const dir = btn.dataset.dir;
  const start = (e) => { e.preventDefault(); btn.classList.add('pressed'); socket.emit('motor_cmd', { direction: dir }); };
  const stop  = (e) => { e.preventDefault(); btn.classList.remove('pressed'); socket.emit('motor_stop', {}); };
  btn.addEventListener('touchstart', start, { passive: false });
  btn.addEventListener('touchend',   stop,  { passive: false });
  btn.addEventListener('mousedown',  start);
  btn.addEventListener('mouseup',    stop);
  btn.addEventListener('mouseleave', stop);
});

document.getElementById('btn-stop').addEventListener('click', () => socket.emit('motor_stop', {}));

// ── E-STOP (status bar) ──────────────────────────────────────────────────────
document.getElementById('btn-estop').addEventListener('click', () => {
  socket.emit('motor_stop', {});
  const b = document.getElementById('btn-estop');
  b.classList.add('flash'); setTimeout(() => b.classList.remove('flash'), 250);
});

// ── Speed slider ───────────────────────────────────────────────────────────
speedSlider.addEventListener('input', () => {
  const p = parseInt(speedSlider.value);
  speedVal.textContent = p;
  socket.emit('set_speed', { speed: Math.round(p * 2.55) });  // 0-100% → 0-255
});

// ── Seeder panel ─────────────────────────────────────────────────────────────
const seedGap   = document.getElementById('seed-gap');
const seedSpot  = document.getElementById('seed-spot');
const seedSpeed = document.getElementById('seed-speed');
const bind = (el, out) => { const f = () => document.getElementById(out).textContent = el.value; el.addEventListener('input', f); f(); };
bind(seedGap, 'seed-gap-val'); bind(seedSpot, 'seed-spot-val'); bind(seedSpeed, 'seed-speed-val');

const seedParams = () => ({
  gap_ms: Math.round(parseFloat(seedGap.value) * 1000),
  seeds_per_spot: parseInt(seedSpot.value),
  drive_speed: Math.round(parseInt(seedSpeed.value) * 2.55),   // % → 0-255
});

document.getElementById('seed-start').addEventListener('click', () => socket.emit('seed_start', seedParams()));
document.getElementById('seed-pause').addEventListener('click', () => socket.emit('seed_pause', {}));
document.getElementById('seed-stop').addEventListener('click',  () => socket.emit('seed_stop', {}));
document.getElementById('seed-plant1').addEventListener('click',() => socket.emit('plant_once', {}));

function renderSeed(s) {
  if (!s) return;
  const el = document.getElementById('seed-status');
  const line = {
    idle:    'Idle',
    running: `Running — ${s.phase} · planted ${s.planted} (${s.seeds} seeds)`,
    paused:  `Paused · planted ${s.planted} (${s.seeds} seeds)`,
  }[s.state] || s.state;
  el.textContent = line;
  el.className = 'seed-status ' + s.state;
}

// ── Soil panel ───────────────────────────────────────────────────────────────
const surveyInt = document.getElementById('survey-int');
bind(surveyInt, 'survey-int-val');

document.getElementById('soil-sample').addEventListener('click', () => socket.emit('soil_sample', {}));
document.getElementById('survey-start').addEventListener('click', () =>
  socket.emit('survey_start', { interval_s: parseInt(surveyInt.value) }));
document.getElementById('survey-stop').addEventListener('click', () => socket.emit('survey_stop', {}));

socket.on('soil', (d) => {
  const mo = (pct, raw) => (pct === null || pct === undefined) ? '—' : `${pct}% (${raw})`;
  document.getElementById('so-m0').textContent   = mo(d.moist_pct_0, d.moisture_a0);
  document.getElementById('so-m1').textContent   = mo(d.moist_pct_1, d.moisture_a1);
  document.getElementById('so-temp').textContent = (d.temp_c === null || d.temp_c === undefined) ? '—' : `${d.temp_c.toFixed(1)} °C`;
  document.getElementById('so-ec').textContent   = (d.ec_raw === null || d.ec_raw === undefined) ? '—' : d.ec_raw;
  if (typeof d.logged === 'number')
    document.getElementById('soil-logged').textContent = `${d.logged} sample${d.logged === 1 ? '' : 's'} logged`;
  const surv = document.getElementById('survey-state');
  surv.textContent = d.surveying
    ? `Surveying — logging every ${surveyInt.value}s…`
    : 'Not surveying — drive the robot and it logs a reading each interval.';
  surv.classList.toggle('active', !!d.surveying);
});
// poll soil only while its tab is open (temp read is slow)
setInterval(() => { if (socket.connected && currentTab === 'soil') socket.emit('get_soil', {}); }, 3000);

// ── Camera panel (Stage 5 — drip vision) ─────────────────────────────────────
// The UNO Q backend is the SOLE consumer of the ESP32-CAM stream; it runs
// detection and pushes annotated frames here ('cam_frame'). The browser never
// hits the cam directly — that removes the two-clients choke that caused the lag.
document.getElementById('cam-set').addEventListener('click', () => {
  const v = document.getElementById('cam-url').value.trim();
  socket.emit('set_camera', { url: v });   // backend opens the stream + runs detection
  document.getElementById('cam-status').textContent = v ? 'Connecting to camera…' : 'Enter the camera address (e.g. farmcam.local)';
  socket.emit('get_vision', {});           // begins the annotated-frame push
});

document.getElementById('drip-start').addEventListener('click', () => socket.emit('drip_start', {}));
document.getElementById('drip-stop').addEventListener('click',  () => socket.emit('drip_stop', {}));

// ── Dataset capture (collect the emitter training set while driving) ──────────
const capInt = document.getElementById('cap-int');
bind(capInt, 'cap-int-val');
document.getElementById('cap-start').addEventListener('click', () =>
  socket.emit('capture_start', { interval: parseFloat(capInt.value) }));
document.getElementById('cap-stop').addEventListener('click', () => socket.emit('capture_stop', {}));
document.getElementById('cap-clear').addEventListener('click', () => {
  if (confirm('Delete ALL captured images on the robot? This cannot be undone.')) socket.emit('capture_clear', {});
});
socket.on('capture_cleared', () => {
  document.getElementById('cap-strip').innerHTML = '';
  document.getElementById('cap-count').textContent = '0';
});

socket.on('capture_status', (d) => {
  document.getElementById('cap-count').textContent = d.count;
  const st = document.getElementById('cap-status');
  if (!d.cam_connected) {
    st.textContent = 'Camera not connected — Connect it first, then Start.';
    st.className = 'cap-status';
  } else if (d.on) {
    st.textContent = `Capturing every ${d.interval}s — ${d.count} saved. Drive the drip line.`;
    st.className = 'cap-status on';
  } else {
    st.textContent = `Stopped — ${d.count} saved to ${d.dir}`;
    st.className = 'cap-status';
  }
});

socket.on('capture_saved', (d) => {
  document.getElementById('cap-count').textContent = d.count;
  if (!d.thumb) return;
  const strip = document.getElementById('cap-strip');
  const img = document.createElement('img');
  img.className = 'cap-thumb';
  img.src = 'data:image/jpeg;base64,' + d.thumb;
  img.title = d.name;
  strip.prepend(img);                                  // newest first
  while (strip.childElementCount > 24) strip.lastElementChild.remove();
});

function renderVisionStatus(d) {
  const st = document.getElementById('cam-status');
  if (!d.vision_ok) {
    st.textContent = 'Vision offline — install OpenCV on the board';
    st.className = 'seed-status';
    return;
  }
  const tube = d.tube || {}, em = d.emitter || {};
  const bits = [];
  bits.push(tube.found ? `tube corr ${tube.correction >= 0 ? '+' : ''}${tube.correction}` : 'no tube');
  if (em.detected) bits.push(`emitter ${em.confidence}`);
  if (d.drip === 'following') bits.push('· FOLLOWING');
  st.textContent = bits.join(' ').trim() || '—';
  st.className = 'seed-status ' + (d.drip === 'following' ? 'running' : '');
}

// annotated frames from the UNO Q (tube/emitter overlay already drawn in)
socket.on('cam_frame', (d) => {
  const url = 'data:image/jpeg;base64,' + d.jpeg;
  document.getElementById('cam-feed').src = url;
  document.getElementById('cam-hint').style.display = 'none';
  if (currentTab === 'drive') document.getElementById('drive-cam').src = url;  // same feed on Drive
  renderVisionStatus(d);
});

// status + heartbeat: the poll keeps the backend pushing while this tab is open
socket.on('vision', renderVisionStatus);
setInterval(() => { if (socket.connected && (currentTab === 'camera' || currentTab === 'drive')) socket.emit('get_vision', {}); }, 500);

// ── Live stats: poll every 2s, render chips + seeder status ─────────────────
const asPct = v => (v === null || v === undefined) ? '—' : `${v}%`;
socket.on('stats', (d) => {
  // Battery: live %, voltage in the tooltip, colour when low/critical
  document.getElementById('st-batt').textContent =
    (d.battery === null || d.battery === undefined) ? '—'
    : `${d.battery}%` + (typeof d.battery_v === 'number' ? ` ${d.battery_v.toFixed(2)}V` : '');
  const chipBatt = document.getElementById('chip-batt');
  chipBatt.title = (typeof d.battery_v === 'number')
    ? `Battery ${d.battery ?? '?'}% · ${d.battery_v.toFixed(2)}V (3S)`
    : 'Battery (no sensor)';
  chipBatt.classList.toggle('crit', typeof d.battery === 'number' && d.battery <= 10);
  chipBatt.classList.toggle('low',  typeof d.battery === 'number' && d.battery > 10 && d.battery <= 20);
  document.getElementById('st-ram').textContent  = asPct(d.ram);
  document.getElementById('st-cpu').textContent  = asPct(d.cpu);
  document.getElementById('st-up').textContent   = d.uptime || '—';
  if (typeof d.speed === 'number') {           // keep slider synced to the robot
    speedSlider.value = d.speed; speedVal.textContent = d.speed;
  }
  renderSeed(d.seed);
});
setInterval(() => { if (socket.connected) socket.emit('get_stats', {}); }, 2000);

// ── Plot seeding (Act 2): mark the plot, preview the serpentine, run it ─────
(function initPlot() {
  const svg   = document.getElementById('pt-svg');
  const stats = document.getElementById('pt-stats');
  const marks = document.getElementById('pt-marks');
  if (!svg) return;

  const cfgEls = { w: 'pt-w', l: 'pt-l', row_gap: 'pt-rowgap',
                   seed_gap: 'pt-seedgap', seeds_per_spot: 'pt-spot' };
  const sendCfg = () => socket.emit('plot_config', {
    w: +document.getElementById('pt-w').value,
    l: +document.getElementById('pt-l').value,
    row_gap: +document.getElementById('pt-rowgap').value / 100,   // cm in UI, m on the wire
    seed_gap: +document.getElementById('pt-seedgap').value / 100,
    seeds_per_spot: +document.getElementById('pt-spot').value,
    dry: document.getElementById('pt-dry').checked,
  });
  Object.values(cfgEls).forEach(id =>
    document.getElementById(id).addEventListener('change', sendCfg));
  document.getElementById('pt-spot').addEventListener('input', () => {
    document.getElementById('pt-spot-val').textContent = document.getElementById('pt-spot').value;
  });
  document.getElementById('pt-dry').addEventListener('change', sendCfg);

  marks.querySelectorAll('.pt-mark').forEach(b =>
    b.addEventListener('click', () => socket.emit('plot_mark', { corner: +b.dataset.c })));
  document.getElementById('pt-clear').addEventListener('click',
    () => socket.emit('plot_clear', {}));
  document.getElementById('pt-start').addEventListener('click', () => {
    const dry = document.getElementById('pt-dry').checked;
    // it drives itself across the plot — make that explicit, and louder if armed
    if (!confirm(dry ? 'Start the dry run?\n\nThe robot will drive the whole plot. Keep the area clear.'
                     : '⚠ SEEDER ARMED\n\nThe robot will drive the whole plot and plant at every spot. Continue?')) return;
    socket.emit('plot_start', {});
  });
  document.getElementById('pt-pause').addEventListener('click', () => socket.emit('plot_pause', {}));
  document.getElementById('pt-stop').addEventListener('click',  () => socket.emit('plot_stop', {}));

  const mmss = s => `${Math.floor(s / 60)}m ${String(s % 60).padStart(2, '0')}s`;

  function draw(d) {
    const w = d.w || 5, l = d.l || 5, pad = 6;
    // plot x = across rows, y = along rows; SVG y is inverted so row 1 reads at the bottom
    const sx = v => pad + (v / w) * (100 - 2 * pad);
    const sy = v => 100 - pad - (v / l) * (100 - 2 * pad);
    const pts = d.planned || [];
    const done = d.spot || 0, per = Math.max(1, d.seeds_per_spot || 1);
    const doneSpots = Math.floor(done / per);

    const path = pts.map((p, i) => `${i ? 'L' : 'M'}${sx(p[0]).toFixed(2)},${sy(p[1]).toFixed(2)}`).join(' ');
    const dots = pts.map((p, i) =>
      `<circle cx="${sx(p[0]).toFixed(2)}" cy="${sy(p[1]).toFixed(2)}" r="${i < doneSpots ? 1.5 : 1}"
        class="${i < doneSpots ? 'pt-done' : 'pt-todo'}"/>`).join('');
    const corners = (d.corners || []).length;
    // Walk order matches the drive: 1 = start, 2 = far end of the FIRST row, then
    // across to 3 and back to 4. So "1 -> 2" is the first straight run.
    const cmark = [[0, 0], [0, l], [w, l], [w, 0]].slice(0, corners).map(([x, y], i) =>
      `<circle cx="${sx(x).toFixed(2)}" cy="${sy(y).toFixed(2)}" r="2.6" class="pt-corner"/>
       <text x="${sx(x).toFixed(2)}" y="${(sy(y) + 1.2).toFixed(2)}" class="pt-cnum">${i + 1}</text>`).join('');

    svg.innerHTML = `
      <rect x="${pad}" y="${pad}" width="${100 - 2 * pad}" height="${100 - 2 * pad}" class="pt-field"/>
      ${pts.length ? `<path d="${path}" class="pt-path"/>` : ''}
      ${dots}${cmark}`;

    const s = d.summary || {};
    if (!pts.length) { stats.textContent = 'Set the plot size and spacing to see the plan.'; return; }
    stats.innerHTML =
      `<b>${s.rows || 0}</b> rows × <b>${s.seeds_per_row || 0}</b> spots ` +
      `= <b>${pts.length}</b> spots · <b>${pts.length * per}</b> seeds · ` +
      `~<b>${mmss(d.est_s || 0)}</b> · ${corners}/4 corners marked` +
      (d.state === 'running' || d.state === 'paused'
        ? ` · <span class="pt-live">spot ${done}/${d.total}</span>` : '');
  }

  socket.on('plot', d => {
    draw(d);
    const st = document.getElementById('seed-status');
    const label = { idle: 'Idle', running: 'Running', paused: 'Paused',
                    done: 'Done', stopped: 'Stopped', error: 'Error' }[d.state] || d.state;
    st.textContent = label + (d.msg ? ' — ' + d.msg : '');
    st.className = 'seed-status ' + (d.state === 'running' ? 'running' : '');
    marks.querySelectorAll('.pt-mark').forEach((b, i) =>
      b.classList.toggle('marked', i < (d.corners || []).length));
  });

  socket.on('plot_report', d => {
    const box = document.getElementById('pt-report');
    document.getElementById('pt-report-svg').innerHTML = d.svg || '';
    box.hidden = !d.svg;
  });
})();

// ── Diag: trace one command browser → console → MCU → pins → motor current ──
(function initDiag() {
  const chainEl = document.getElementById('dg-chain');
  const rawEl   = document.getElementById('dg-raw-json');
  if (!chainEl) return;
  const pair = (a, b) => `${a ?? '—'} / ${b ?? '—'}`;

  socket.on('diag', d => {
    const sent = d.ui || {};
    const mcu  = d.mcu;
    const cmd  = (mcu && mcu.cmd)  || {};
    const pins = (mcu && mcu.pins) || {};
    const amps = mcu && mcu.amps;             // absent unless CURRENT_SENSE is wired
    const moving = !!(sent.left || sent.right);

    // Stage 3+ can only be judged when the MCU answered at all.
    const noDiag = !mcu;
    const gotCmd = !noDiag && cmd.req_l === sent.left && cmd.req_r === sent.right;
    // pins must reflect the applied duty, split by direction (forward on RPWM)
    const pinsOk = !noDiag &&
      pins.l_rpwm === Math.max(0, cmd.app_l) && pins.l_lpwm === Math.max(0, -cmd.app_l) &&
      pins.r_rpwm === Math.max(0, cmd.app_r) && pins.r_lpwm === Math.max(0, -cmd.app_r);
    // a side commanded but drawing no current = the drive never reached the motor
    const dead = amps && ((cmd.app_l && amps.l_avg < 0.05) || (cmd.app_r && amps.r_avg < 0.05));

    const stages = [
      { n: 'Browser', v: sent.direction ? `${sent.direction} @ ${sent.speed ?? '—'}`
                                        : 'no command yet',
        s: sent.direction ? 'ok' : 'idle',
        note: d.age_ms != null ? `${d.age_ms} ms ago` : '' },
      { n: 'Console (after trim)', v: pair(sent.left, sent.right),
        s: sent.direction ? 'ok' : 'idle',
        note: d.trim ? `trim ${d.trim[0]} / ${d.trim[1]}` : '' },
      { n: 'MCU received', v: noDiag ? '—' : pair(cmd.req_l, cmd.req_r),
        s: noDiag ? 'na' : (gotCmd ? 'ok' : 'bad'),
        note: noDiag ? 'no getDiag — flash the firmware'
                     : (gotCmd ? `n=${cmd.n}, ${cmd.ms_ago} ms ago`
                               : 'MISMATCH vs console — command did not arrive intact') },
      { n: 'Driver pins (IBT-2)', v: noDiag ? '—'
            : `L[${pins.l_rpwm},${pins.l_lpwm}] R[${pins.r_rpwm},${pins.r_lpwm}]`,
        s: noDiag ? 'na' : (pinsOk ? 'ok' : 'bad'),
        note: noDiag ? '' : (pinsOk ? 'RPWM/LPWM match the applied duty'
                                    : 'pins disagree with the applied duty') },
      { n: 'Motor current', v: amps ? `${amps.l_avg} A / ${amps.r_avg} A`
                                    : 'not wired',
        s: !amps ? 'na' : (dead ? 'bad' : (moving ? 'ok' : 'idle')),
        note: !amps ? 'wire IBT-2 IS pins + set CURRENT_SENSE 1'
                    : (dead ? 'COMMANDED BUT NO CURRENT — check IBT-2 wiring'
                            : `peak ${amps.l_max} / ${amps.r_max} A over ${amps.n} samples`) },
    ];

    chainEl.innerHTML = stages.map(st => `
      <li class="dg-stage ${st.s}">
        <span class="dg-dot"></span>
        <span class="dg-name">${st.n}</span>
        <span class="dg-val">${st.v}</span>
        <span class="dg-note">${st.note || ''}</span>
      </li>`).join('');

    rawEl.textContent = d.error ? `getDiag failed: ${d.error}`
                                : JSON.stringify(mcu, null, 2) || '—';
  });

  // 400ms: fast enough to catch a button hold, light enough to leave running
  setInterval(() => {
    if (socket.connected && currentTab === 'diag') socket.emit('get_diag', {});
  }, 400);
})();

// ── Network mode: ask the host helper (:7999) for the real mode ─────────────
(function initNetMode() {
  const statusEl = document.getElementById('net-status');
  const actionEl = document.getElementById('net-action');
  const netChip  = document.getElementById('st-net');

  const setHotspot = () => {
    netChip.textContent = 'AP';
    statusEl.textContent = '📡 Hotspot ON';
    statusEl.classList.add('active');
    actionEl.textContent = '📶 Connect WiFi';
    actionEl.onclick = () => {
      if (!confirm('Switch the robot to home WiFi?\n\nThis hotspot will drop — reconnect your phone to home WiFi, then open farm-os.local:7000. (If WiFi is unreachable, the robot returns to this hotspot.)')) return;
      socket.emit('connect_wifi', {});
      statusText.textContent = 'Switching to home WiFi…';
      dot.className = 'status-dot disconnected';
    };
  };
  const setWifi = () => {
    netChip.textContent = 'WiFi';
    statusEl.textContent = '📶 WiFi Connected';
    statusEl.classList.add('active');
    actionEl.textContent = '📡 Start Hotspot';
    actionEl.onclick = () => {
      if (!confirm('Start the FarmOS-AP hotspot?\n\nThe robot will leave home WiFi. Connect your phone to "FarmOS-AP", then open farm-os.local:7000 (or 192.168.4.1:7000).')) return;
      socket.emit('start_hotspot', {});
      statusText.textContent = 'Starting hotspot (FarmOS-AP)…';
      dot.className = 'status-dot disconnected';
    };
  };

  fetch(`http://${location.hostname}:7999/status`, { cache: 'no-store' })
    .then(r => r.json())
    .then(d => (d.mode === 'hotspot' ? setHotspot() : setWifi()))
    .catch(() => (location.hostname.startsWith('192.168.4.') ? setHotspot() : setWifi()));
})();

// ── Power: reboot / shutdown (with confirm) ─────────────────────────────────
document.getElementById('btn-reboot').addEventListener('click', () => {
  if (!confirm('Reboot the robot? It will be back in ~1 minute.')) return;
  socket.emit('reboot', {});
  statusText.textContent = 'Rebooting… reconnect in ~1 min';
  dot.className = 'status-dot disconnected';
});

document.getElementById('btn-shutdown').addEventListener('click', () => {
  if (!confirm('Halt the robot?\n\nThe UNO Q will shut down and stay halted. Once its LED goes off, cut the LiPo power (switch). Then power back on by hand.')) return;
  socket.emit('shutdown', {});
  statusText.textContent = 'Halting… when the LED is off, cut power (LiPo switch)';
  dot.className = 'status-dot disconnected';
});

// ── Plan tab (Act 1 — sowing advisor). Talks to the on-host planner service (:8765),
//    which reconciles the Tamil panchangam + biodynamic calendar and streams the LLM prose. ──
(function initPlan() {
  const PLANNER = `http://${location.hostname}:8765`;
  const chat = document.getElementById('plan-chat'),
        input = document.getElementById('plan-text'),
        sendBtn = document.getElementById('plan-send'),
        quick = document.getElementById('plan-quick');
  if (!chat) return;
  ["groundnut", "corn", "sesame"].forEach(c => {
    const b = document.createElement('button'); b.textContent = c;
    b.onclick = () => { input.value = c; planSend(); }; quick.appendChild(b);
  });
  function pb(cls) { const d = document.createElement('div'); d.className = 'pl-msg ' + cls; chat.appendChild(d); chat.scrollTop = chat.scrollHeight; return d; }
  function chips(a, cls) { return a && a.length ? `<div class="pl-chips">${a.map(d => `<span class="pl-chip ${cls}">${d}</span>`).join('')}</div>` : `<div class="pl-chips"><span class="pl-chip" style="color:var(--muted)">none in window</span></div>`; }
  function card(p) {
    const nok = p.needs.nokku, bio = p.needs.biodynamic;
    const head = p.recommended_date
      ? `<span class="pl-date">${p.recommended_date}</span><span class="pl-badge agree">✓ both calendars agree</span>`
      : `<span class="pl-date" style="color:var(--warn);font-size:15px">No single date agrees in this window</span><span class="pl-badge single">pick an alternative</span>`;
    const panch = (p.recommended_date && p.nakshatra)
      ? `<div class="pl-row"><span class="pl-ico">🕉</span><span>Panchangam: <b>${p.nakshatra} (${p.nakshatra_tamil || ''})</b> → ${nok} nokku</span></div>`
      : `<div class="pl-row"><span class="pl-ico">🕉</span><span>Needs a <b>${nok} nokku</b> day</span></div>`;
    const pr = p.price ? `${p.price.current.price.toLocaleString()} ₹/qtl (${p.price.recent_trend}, YoY ${p.price.yoy_change_pct}%)` : '—';
    let wx = '—'; if (p.weather) { wx = `${p.weather.avg_tmax_c}/${p.weather.avg_tmin_c}°C, ${p.weather.total_rain_mm}mm over ${p.weather.rainy_days} rainy days` + (p.weather.recommended_in_horizon === false ? ` (date beyond 16-day forecast)` : ''); }
    return `<div class="pl-card"><div>${head}</div>${panch}
      <div class="pl-row"><span class="pl-ico">🌿</span><span>Biodynamic: needs a <b>${bio} day</b></span></div>
      <div class="pl-divider"></div>
      <div class="pl-label">Alternatives — panchangam only (${nok})</div>${chips(p.alternatives.panchangam, 'pan')}
      <div class="pl-label" style="margin-top:8px">Alternatives — biodynamic only (${bio})</div>${chips(p.alternatives.biodynamic, 'bio')}
      <div class="pl-label" style="margin-top:8px">Avoid — கரி நாள்</div>${chips(p.avoid_days, 'avoid')}
      <div class="pl-divider"></div>
      <div class="pl-row"><span class="pl-ico">💰</span><span>Price <span class="pl-tag mock">MOCK</span> <b>${pr}</b></span></div>
      <div class="pl-row"><span class="pl-ico">🌦</span><span>Weather <span class="pl-tag real">LIVE</span> <b>${wx}</b></span></div></div>`;
  }
  async function planSend() {
    const q = input.value.trim(); if (!q) return;
    input.value = ''; sendBtn.disabled = true;
    pb('user').textContent = q;
    const b = pb('bot');
    const prose = document.createElement('div'); prose.className = 'pl-prose';
    prose.innerHTML = `<span class="pl-dots"><i></i><i></i><i></i></span>`;
    const host = document.createElement('div'); b.appendChild(prose); b.appendChild(host);
    try {
      const r = await fetch(PLANNER + '/api/plan', { method: 'POST', headers: { 'Content-Type': 'application/json' }, body: JSON.stringify({ message: q }) });
      const p = await r.json();
      if (p.error === 'unknown_crop') { prose.textContent = `I can advise on: ${p.known_crops.join(', ')}. Which one?`; sendBtn.disabled = false; return; }
      host.innerHTML = card(p); chat.scrollTop = chat.scrollHeight;
      prose.innerHTML = ''; const cur = document.createElement('span'); cur.className = 'pl-cursor'; prose.appendChild(cur);
      const resp = await fetch(PLANNER + '/api/narrate', { method: 'POST', headers: { 'Content-Type': 'application/json' }, body: JSON.stringify({ crop: p.crop }) });
      const reader = resp.body.getReader(), dec = new TextDecoder(); let txt = '';
      while (true) { const { value, done } = await reader.read(); if (done) break; txt += dec.decode(value, { stream: true }); prose.textContent = txt; chat.scrollTop = chat.scrollHeight; }
    } catch (e) { prose.textContent = 'Planner unavailable: ' + e; }
    sendBtn.disabled = false; input.focus();
  }
  sendBtn.addEventListener('click', planSend);
  input.addEventListener('keydown', e => { if (e.key === 'Enter') planSend(); });
  pb('bot').innerHTML = `<div class="pl-prose">Tell me what you want to plant and I'll check the Tamil panchangam (Nokku Naal) and the biodynamic calendar, then recommend a sowing date with alternatives. Try "groundnut".</div>`;
})();
