'use strict';

const $ = id => document.getElementById(id);
const cam = $('cam'), overlay = $('overlay'), ctx = overlay.getContext('2d');
let latest = null;      // newest answer from the server
let shown = null;       // the answer whose frame is on screen
let paused = false;
let loading = 0;        // sequence number of the frame being loaded

const layers = () => ({
  rect: $('l-rect').checked, raw: $('l-raw').checked, chosen: $('l-chosen').checked,
  lines: $('l-lines').checked, tags: $('l-tags').checked, left: $('l-left').checked, grid: $('l-grid').checked, robot: $('l-robot').checked,
});

// ------------------------------------------------------------------ table coordinates (the locked rectangle is 20 x 20 cm)
function mul3(M, x, y) { const w = M[2][0] * x + M[2][1] * y + M[2][2]; return [(M[0][0] * x + M[0][1] * y + M[0][2]) / w, (M[1][0] * x + M[1][1] * y + M[1][2]) / w]; }
function inv3(M) {
  const [a, b, c, d, e, f, g, h, i] = M.flat(), det = a * (e * i - f * h) - b * (d * i - f * g) + c * (d * h - e * g);
  return [[(e * i - f * h) / det, (c * h - b * i) / det, (b * f - c * e) / det], [(f * g - d * i) / det, (a * i - c * g) / det, (c * d - a * f) / det], [(d * h - e * g) / det, (b * g - a * h) / det, (a * e - b * d) / det]];
}
const lockedMatrix = () => (latest && latest.calibration && latest.calibration.state === 'locked') ? latest.calibration.px_to_cm : null;

// ------------------------------------------------------------------ drawing on the picture
function label(text, x, y, color, size) {
  ctx.font = `600 ${size}px system-ui, sans-serif`;
  const w = ctx.measureText(text).width + 8, h = size + 6;
  ctx.fillStyle = 'rgba(10,14,22,.8)'; ctx.fillRect(x, y - h, w, h);
  ctx.fillStyle = color; ctx.fillText(text, x + 4, y - 5);
}
function segment(p, q, color, width, dash) {
  ctx.strokeStyle = color; ctx.lineWidth = width; ctx.setLineDash(dash || []);
  ctx.beginPath(); ctx.moveTo(p[0], p[1]); ctx.lineTo(q[0], q[1]); ctx.stroke(); ctx.setLineDash([]);
}
function polygonPath(points) { ctx.beginPath(); points.forEach((p, i) => (i ? ctx.lineTo(p[0], p[1]) : ctx.moveTo(p[0], p[1]))); ctx.closePath(); }

function draw() {
  const rect = overlay.getBoundingClientRect(), dpr = window.devicePixelRatio || 1;
  overlay.width = Math.round(rect.width * dpr); overlay.height = Math.round(rect.height * dpr);
  ctx.clearRect(0, 0, overlay.width, overlay.height);
  if (!shown || !shown.detection) return;
  const det = shown.detection, k = overlay.width / det.width, ui = dpr * 1.05;
  const S = p => [p[0] * k, p[1] * k];
  const on = layers();

  if (on.lines) for (const l of det.lines) if (!l.name) segment(S(l.p0), S(l.p1), 'rgba(52,198,255,.8)', 1.2 * ui);
  const smooth = shown.polygon_shown;   // the locked rectangle once calibrated, otherwise the smoothed live one
  if (on.rect && smooth) {
    const pts = smooth.map(S);
    polygonPath(pts); ctx.fillStyle = 'rgba(40,211,107,.14)'; ctx.fill();
    polygonPath(pts); ctx.strokeStyle = '#28d36b'; ctx.lineWidth = 3 * ui; ctx.stroke();
  }
  const M = lockedMatrix();
  if (on.grid && M) {                    // lines every 5 cm: straight and square if the right-angle model is right
    const back = inv3(M), P = (x, y) => S(mul3(back, x, y));
    for (const v of [5, 10, 15]) { segment(P(v, 0), P(v, 20), 'rgba(255,255,255,.55)', 1 * ui, [4 * ui, 4 * ui]); segment(P(0, v), P(20, v), 'rgba(255,255,255,.55)', 1 * ui, [4 * ui, 4 * ui]); }
    for (const v of [5, 10, 15]) { const a = P(v, 20), b = P(20, v); label(String(v), a[0] - 6 * ui, a[1] + 18 * ui, '#fff', 10 * ui); label(String(v), b[0] + 6 * ui, b[1] + 4 * ui, '#fff', 10 * ui); }
  }
  if (on.raw && det.polygon && shown.locked) { polygonPath(det.polygon.map(S)); ctx.strokeStyle = '#fff'; ctx.lineWidth = 1.5 * ui; ctx.setLineDash([6 * ui, 5 * ui]); ctx.stroke(); ctx.setLineDash([]); }
  if (on.chosen) for (const [name, l] of Object.entries(det.chosen)) {
    segment(S(l.p0), S(l.p1), '#ff9a2e', 2.5 * ui);
    const mid = S([(l.p0[0] + l.p1[0]) / 2, (l.p0[1] + l.p1[1]) / 2]);
    label(name, mid[0] + 4 * ui, mid[1] - 4 * ui, '#ffbf73', 11 * ui);
  }
  if (on.left) {
    if (det.left_estimate) { segment(S(det.left_estimate[0]), S(det.left_estimate[1]), '#e56bff', 2.5 * ui, [7 * ui, 5 * ui]); const [a, b] = det.left_estimate.map(S), p = [a[0] + 0.45 * (b[0] - a[0]), a[1] + 0.45 * (b[1] - a[1])]; label('left edge (estimated)', p[0] + 8 * ui, p[1], '#f0a5ff', 11 * ui); }
    if (det.left_detected) { segment(S(det.left_detected[0]), S(det.left_detected[1]), '#fff', 1.5 * ui, [3 * ui, 5 * ui]); const [a, b] = det.left_detected.map(S), p = [a[0] + 0.7 * (b[0] - a[0]), a[1] + 0.7 * (b[1] - a[1])]; label('line seen at the left', p[0] + 8 * ui, p[1], '#fff', 11 * ui); }
  }
  if (on.robot && drive && drive.enabled && M) {                 // robot calibration markers, drawn through the camera model
    for (const c of drive.corners) if (c.px) {
      const p = S(c.px); ctx.strokeStyle = '#3fe0b5'; ctx.lineWidth = 2 * ui; ctx.beginPath(); ctx.arc(p[0], p[1], 9 * ui, 0, 7); ctx.stroke();
      label(`${c.index + 1} ${c.name}`, p[0] + 12 * ui, p[1] + (c.v > 0.5 ? 16 : -4) * ui, '#7dffd9', 11 * ui);
    }
    if (target) { const p = S([target.px, target.py]); ctx.strokeStyle = '#34c6ff'; ctx.lineWidth = 3 * ui; ctx.beginPath(); ctx.arc(p[0], p[1], 14 * ui, 0, 7); ctx.moveTo(p[0] - 20 * ui, p[1]); ctx.lineTo(p[0] + 20 * ui, p[1]); ctx.moveTo(p[0], p[1] - 20 * ui); ctx.lineTo(p[0], p[1] + 20 * ui); ctx.stroke(); label('target', p[0] + 16 * ui, p[1] - 12 * ui, '#9fe3ff', 11 * ui); }
    const rt = drive.real && drive.real.tip_px; if (rt) { const p = S(rt); ctx.strokeStyle = '#ff3b30'; ctx.lineWidth = 3 * ui; ctx.beginPath(); ctx.arc(p[0], p[1], 11 * ui, 0, 7); ctx.moveTo(p[0] - 16 * ui, p[1]); ctx.lineTo(p[0] + 16 * ui, p[1]); ctx.moveTo(p[0], p[1] - 16 * ui); ctx.lineTo(p[0], p[1] + 16 * ui); ctx.stroke(); label('real tip (arm\'s own estimate)', p[0] + 16 * ui, p[1] + 26 * ui, '#ffb3ad', 11 * ui); }
    const tip = drive.sim && drive.sim.tip_px; if (tip) { const p = S(tip); ctx.strokeStyle = '#ff5cf0'; ctx.lineWidth = 3 * ui; ctx.beginPath(); ctx.moveTo(p[0] - 10 * ui, p[1] - 10 * ui); ctx.lineTo(p[0] + 10 * ui, p[1] + 10 * ui); ctx.moveTo(p[0] + 10 * ui, p[1] - 10 * ui); ctx.lineTo(p[0] - 10 * ui, p[1] + 10 * ui); ctx.stroke(); label('simulated tip', p[0] + 12 * ui, p[1] + 20 * ui, '#ffb3f7', 11 * ui); }
  }
  if (on.tags) for (const t of det.tags) {
    polygonPath(t.corners.map(S)); ctx.strokeStyle = '#ffe14a'; ctx.lineWidth = 3 * ui; ctx.stroke();
    const c = S(t.center); label(`tag ${t.id}`, c[0] + 6 * ui, c[1] - 6 * ui, '#ffe14a', 12 * ui);
  }
  if (on.rect && smooth) smooth.map(S).forEach((p, i) => {
    ctx.fillStyle = '#28d36b'; ctx.beginPath(); ctx.arc(p[0], p[1], 5 * ui, 0, 7); ctx.fill();
    label(['TL', 'TR', 'BR', 'BL'][i], p[0] + (i === 1 || i === 2 ? -30 * ui : 8 * ui), p[1] + (i < 2 ? -6 * ui : 22 * ui), '#7dffb0', 12 * ui);
  });
}
new ResizeObserver(draw).observe(overlay);
for (const id of ['l-rect', 'l-raw', 'l-chosen', 'l-lines', 'l-tags', 'l-left', 'l-grid', 'l-robot']) $(id).addEventListener('change', draw);

// ------------------------------------------------------------------ the text panels
function badge(state, text) { const b = document.createElement('span'); b.className = 'badge'; b.dataset.s = state; b.textContent = text; return b; }
function row(state, badgeText, what, extra) {
  const li = document.createElement('li'); li.append(badge(state, badgeText));
  const w = document.createElement('span'); w.className = 'what'; w.textContent = what;
  if (extra) { const e = document.createElement('span'); e.className = 'extra'; e.textContent = ' ' + extra; w.append(e); }
  li.append(w); return li;
}

function renderPanels(data) {
  const det = data.detection, items = [], cal = data.calibration || {};
  const ids = det.tags.map(t => t.id).sort((a, b) => a - b);
  items.push(row(ids.length >= 2 ? 'ok' : 'bad', ids.length ? `${ids.length} seen` : 'none', 'AprilTags', ids.length ? `ids ${ids.join(', ')}; the arm may hide the others` : 'no tag visible'));
  for (const [key, text] of [['top', 'Top line'], ['bottom', 'Bottom line'], ['mid', 'Middle line'], ['right', 'Right line']])
    items.push(det.chosen[key] ? row('ok', 'found', text) : row('bad', 'missing', text));
  const estimated = det.method && det.method.includes('estimated'), detected = det.chosen.left;
  items.push(detected ? row('ok', 'found', 'Left line', 'seen and consistent with the other lines')
    : estimated ? row('warn', 'estimated', 'Left line', 'hidden, worked out from the middle and right lines')
    : row('bad', 'missing', 'Left line'));
  if (cal.state === 'locked') {
    items.unshift(row(cal.drift_warn ? 'bad' : cal.drift_px > 3 ? 'warn' : 'ok', 'locked', 'Calibration', `from ${cal.n_frames} of ${cal.of} frames at ${new Date(cal.locked_at * 1000).toLocaleTimeString()}; live check ${cal.drift_px} px from the lock`));
  } else if (cal.state === 'countdown') items.unshift(row('warn', `${Math.ceil(cal.remaining)} s`, 'Calibration', 'measuring…'));
  else if (cal.state === 'failed') items.unshift(row('bad', 'failed', 'Calibration', cal.reason));
  if (det.grid && det.grid.rms_mm != null) items.push(row(det.grid.rms_mm < 3 ? 'ok' : 'warn', `${det.grid.rms_mm} mm`, 'Right-angle grid fit', 'how well the lines fit one right-angled grid seen in perspective'));
  if (det.grid && det.grid.right_width_cm) items.push(row('ok', `${det.grid.right_width_cm} cm`, 'Right square width', 'implied by the grid (assumed 21.1)'));
  if (det.aspect) items.push(row(det.aspect > 0.8 && det.aspect < 1.25 ? 'ok' : 'warn', det.aspect.toFixed(2), 'Width / height in the picture', 'about 0.9 to 1.1: the camera looks at the table at an angle, so it is not exactly 1 (the crop is straightened to a square)'));
  $('checks').replaceChildren(...items);

  const issues = det.issues.length ? det.issues.map(t => { const li = document.createElement('li'); li.textContent = t; return li; })
    : (() => { const li = document.createElement('li'); li.className = 'none'; li.textContent = 'Nothing to report.'; return [li]; })();
  $('issues').replaceChildren(...issues);
  $('timing').textContent = `Detection took ${data.detect_ms} ms · picture ${(data.now - data.received).toFixed(1)} s old`;

  const v = $('verdict'), title = $('verdict-title'), detail = $('verdict-detail');
  if (cal.state === 'countdown') { v.dataset.tone = 'idle'; title.textContent = `Measuring the rectangle… ${Math.ceil(cal.remaining)} s`; detail.textContent = `Keep the table and camera still (${cal.frames} usable frames so far).`; }
  else if (cal.state === 'failed') { v.dataset.tone = 'bad'; title.textContent = 'Could not lock the rectangle'; detail.textContent = `${cal.reason}. Press Recalibrate to try again.`; }
  else if (cal.state === 'locked' && cal.drift_warn) { v.dataset.tone = 'bad'; title.textContent = 'Something moved: the live picture no longer matches the locked rectangle'; detail.textContent = `The tags or lines are ${cal.drift_px} px from where they were. Press Recalibrate if the camera or the table was moved.`; }
  else if (cal.state === 'locked' && cal.method.includes('estimated')) { v.dataset.tone = 'warn'; title.textContent = 'Rectangle locked, left edge estimated'; detail.textContent = `Measured over ${cal.n_frames} frames (jitter ±${cal.spread_px} px). The left line was hidden; move the arm away and recalibrate to see it.`; }
  else if (cal.state === 'locked') { v.dataset.tone = 'ok'; title.textContent = 'Rectangle locked'; detail.textContent = `All edges come from lines seen in the picture, measured over ${cal.n_frames} frames (jitter ±${cal.spread_px} px).`; }
  else if (!det.polygon) { v.dataset.tone = 'bad'; title.textContent = 'Rectangle not found'; detail.textContent = det.issues[0] || ''; }
  else if (!det.ok) { v.dataset.tone = 'bad'; title.textContent = 'A rectangle was found, but it looks wrong'; detail.textContent = det.issues.find(i => /square|convex|outside/.test(i)) || ''; }
  else if (estimated) { v.dataset.tone = 'warn'; title.textContent = 'Rectangle found, left edge estimated'; detail.textContent = 'The left line is hidden (probably by the arm). Check the green outline against the paper.'; }
  else { v.dataset.tone = 'ok'; title.textContent = 'Rectangle found'; detail.textContent = 'All four edges come from lines seen in the picture.'; }

  const crop = $('crop');
  if (data.has_crop) { crop.hidden = false; $('crop-empty').hidden = true; crop.src = `/crop/${data.seq}.jpg`; }
  else { crop.hidden = true; $('crop-empty').hidden = false; }
}

function renderCountdown(data) {
  const cal = data.calibration || {};
  $('countdown').hidden = cal.state !== 'countdown';
  if (cal.state === 'countdown') $('count-num').textContent = Math.max(1, Math.ceil(cal.remaining));
}

function renderConnection(data) {
  const chip = $('conn'), src = data.source || {};
  if (paused) { chip.dataset.tone = 'warn'; chip.textContent = 'Paused'; return; }
  if (src.status !== 'live') { chip.dataset.tone = 'bad'; chip.textContent = src.error ? `Camera: ${src.status} (${src.error})` : `Camera: ${src.status}`; return; }
  const age = data.received ? data.now - data.received : null;
  if (age !== null && age > 6) { chip.dataset.tone = 'warn'; chip.textContent = `Camera stalled (${age.toFixed(0)} s old)`; }
  else { chip.dataset.tone = 'ok'; chip.textContent = `Live · ${age === null ? '' : age.toFixed(1) + ' s old'}`; }
}

// ------------------------------------------------------------------ loop
cam.addEventListener('load', () => { if (latest && latest.seq === loading) { shown = latest; renderPanels(shown); draw(); } });
async function poll() {
  try {
    const data = await (await fetch('/api/detect', {cache: 'no-store'})).json();
    latest = data; renderConnection(data); renderCountdown(data);
    if (!paused && data.seq && data.seq !== loading) { loading = data.seq; cam.src = `/frame/${data.seq}.jpg`; }
  } catch (_) { $('conn').dataset.tone = 'bad'; $('conn').textContent = 'Lost connection to the page server'; }
  setTimeout(poll, 350);
}
$('pause').addEventListener('click', () => { paused = !paused; $('pause').setAttribute('aria-pressed', paused); $('pause').textContent = paused ? 'Resume' : 'Pause'; if (latest) renderConnection(latest); });
$('recal').addEventListener('click', async () => { try { latest = await (await fetch('/api/recalibrate', {method: 'POST'})).json(); renderCountdown(latest); } catch (_) { /* the poll loop shows connection problems */ } });
overlay.addEventListener('pointermove', e => {
  const M = lockedMatrix(), rect = overlay.getBoundingClientRect();
  if (!M || !shown) return;
  const k = shown.detection.width / rect.width, [x, y] = mul3(M, (e.clientX - rect.left) * k, (e.clientY - rect.top) * k);
  $('hover').textContent = (x >= -0.5 && x <= 20.5 && y >= -0.5 && y <= 20.5) ? `Table position: x ${x.toFixed(1)} cm, y ${y.toFixed(1)} cm (0, 0 = top-left corner of the rectangle, 20, 20 = bottom-right)` : `Outside the rectangle: x ${x.toFixed(1)} cm, y ${y.toFixed(1)} cm`;
});
overlay.addEventListener('pointerleave', () => { $('hover').textContent = 'Move the pointer over the picture to read table positions (cm) once the rectangle is locked.'; });

// ------------------------------------------------------------------ driving the simulator (small intents; the server plans and executes)
let drive = null, target = null, tourIndex = -1;
const clock = {offset: 0, rtt: null, bestRtt: Infinity};
const serverNow = () => Date.now() + clock.offset;            // the server's clock, as best estimated from the quickest round trip
const imageUnitToUv = (xu, yu, f) => ({up: [xu, 1 - yu], right: [yu, xu], down: [1 - xu, yu], left: [1 - yu, 1 - xu]})[f];

function uvOfPixel(px, py) {                                 // picture pixel -> (u, v) with the locked rectangle's matrix
  const M = lockedMatrix(); if (!M || !drive) return null;
  const [xc, yc] = mul3(M, px, py); return imageUnitToUv(xc / 20, yc / 20, drive.faces);
}
function xyOfUv(u, v) {                                      // the robot calibration is a rectangle: corner 1 + u * (right edge) + v * (far edge)
  const c = drive.corners; if (!c || c.length < 4) return null;
  return [c[0].x + u * (c[1].x - c[0].x) + v * (c[3].x - c[0].x), c[0].y + u * (c[1].y - c[0].y) + v * (c[3].y - c[0].y)];
}
const taskBusy = t => Boolean(t && ['planning', 'running'].includes(t.state));
const driveBusy = () => Boolean(drive && (taskBusy(drive.sim && drive.sim.task) || taskBusy(drive.real && drive.real.task)));
const realSelected = () => $('arm-real').checked;
const status = (text, tone) => { const e = $('drive-status'); e.textContent = text; e.style.color = tone === 'bad' ? 'var(--bad)' : tone === 'ok' ? 'var(--ok)' : ''; };

function renderDrive() {
  const panel = $('drive');
  if (!drive || !drive.enabled) { panel.hidden = true; return; }
  panel.hidden = false;
  const rm = drive.robot_map;
  $('drive-cal').textContent = rm ? `Calibrated from ${rm.n_points} points recorded with the real arm (${rm.points.join(', ')}). They imply a rectangle of ${rm.size_cm.u} × ${rm.size_cm.v} cm (not assumed to be 20 × 20); fit error ${Math.max(...Object.values(rm.residual_mm)).toFixed(1)} mm.`
    : 'No robot calibration loaded: start the server with --robot-points.';
  $('drive-warn').replaceChildren(...((rm && rm.warnings) || []).map(t => { const li = document.createElement('li'); li.textContent = t; return li; }));
  const faces = $('faces'); if (!faces.options.length) { for (const f of drive.faces_options) faces.add(new Option(f, f)); }
  faces.value = drive.faces;
  const link = $('link'), lag = clock.rtt;
  link.textContent = lag === null ? 'Link…' : `Link ${Math.round(lag)} ms`;
  link.dataset.tone = lag === null ? 'idle' : lag < 300 ? 'ok' : lag < 800 ? 'warn' : 'bad';
  const locked = Boolean(lockedMatrix()), sim = drive.sim || {};
  const sv = drive.servo || {available: false}, busy = driveBusy() || sv.running;
  $('go-target').disabled = !(target && locked && sim.connected) || busy;
  $('tour-next').disabled = !(locked && sim.connected) || busy;
  $('tour-prev').disabled = !(locked && sim.connected) || busy || tourIndex <= 0;
  const cl = $('closed-loop'); cl.disabled = !sv.available || busy || realSelected();
  renderReal(locked, busy);
  $('servo-note').textContent = sv.available ? (sv.demo_error_mm && (sv.demo_error_mm[0] || sv.demo_error_mm[1]) ? `Demo: the simulated board is ${sv.demo_error_mm[0]}, ${sv.demo_error_mm[1]} mm off the arm's coordinates (a stand-in for the camera). Without the correction the tip lands that far from the target.` : 'Camera feedback is available.') : 'Closed-loop correction needs a tip observer (a marker on the gripper tracked by the camera). None is configured yet.';
  const items = [];
  if (sv.running) { const li = document.createElement('li'); li.textContent = `Closed loop running: ${sv.label}…`; items.push(li); }
  if (sv.last) {
    for (const st of sv.last.steps) { const li = document.createElement('li'); li.textContent = `${st.iteration === 0 ? 'First move' : 'Correction ' + st.iteration}: ${st.error_mm === null ? 'tip not seen' : st.error_mm + ' mm from the target'}${st.note ? ' — ' + st.note : ''}`; items.push(li); }
    const f = document.createElement('li'); f.className = 'final'; f.textContent = `${sv.last.ok ? 'Done' : 'Ended'}: ${sv.last.reason}${sv.last.final_error_mm !== null ? ` (final ${sv.last.final_error_mm} mm)` : ''}`; items.push(f);
  }
  $('drive-servo').replaceChildren(...items);
  $('drive-log').replaceChildren(...drive.log.slice().reverse().map(t => { const li = document.createElement('li'); li.textContent = t; return li; }));
  if (!sim.connected) status(`Simulator not connected${sim.error ? ': ' + sim.error : ''}`, 'bad');
  else if (!locked) status('Waiting for the rectangle to lock.');
  else if (sim.task) status(`Simulator: ${sim.task.state}${sim.task.message ? ' — ' + sim.task.message : ''}${sim.task.state === 'stopped' ? ' (you can keep driving)' : ''}`, sim.task.state === 'error' ? 'bad' : '');
  draw();
}

const SHORT = {bottom: 'bottom', top: 'top', left: 'left', right: 'right'};
function pointLabels() {                                       // name each corner by where it is in the PICTURE, so the buttons match what you see
  const cs = drive.corners || [], placed = cs.filter(c => c.px);
  if (placed.length < 4) return cs.map(c => `${c.index + 1} · ${c.name}`);
  const cx = placed.reduce((a, c) => a + c.px[0], 0) / 4, cy = placed.reduce((a, c) => a + c.px[1], 0) / 4;
  return cs.map(c => `${c.index + 1} · ${c.px[1] < cy ? 'top' : 'bottom'}-${c.px[0] < cx ? 'left' : 'right'}`);
}
function renderReal(locked, busy) {
  const real = drive.real || {available: false}, radio = $('arm-real');
  radio.disabled = !(real.available && real.connected);
  $('arm-real-label').title = !real.available ? 'The real arm is off: start the server with --real-arm.' : !real.connected ? 'The real arm is not answering.' : '';
  if (radio.disabled && radio.checked) document.querySelector('input[name=arm][value=sim]').checked = true;   // the real arm went away: back to the simulator
  const twinOn = Boolean(drive.sim && drive.sim.twin && drive.sim.twin.available && drive.sim.twin.enabled);     // the simulator mirrors the real arm
  const simRadio = document.querySelector('input[name=arm][value=sim]');
  simRadio.disabled = twinOn;
  simRadio.parentElement.title = twinOn ? 'The simulator is a digital twin: it follows the real arm, so moves go to the real arm.' : '';
  if (twinOn && radio.disabled === false && !radio.checked) radio.checked = true;
  const isReal = realSelected();
  $('real-warn').hidden = !isReal; $('real-status').hidden = !isReal;
  if (isReal) $('real-status').textContent = real.connected ? `Real arm: ${real.owner ? 'controlled by ' + real.owner : 'nobody holds control'}; gripper ${Math.round(real.gripper)} ${real.gripper > 25 ? '(OPEN: close it first, the move will be refused)' : '(closed)'}; ${real.emergency_stop ? 'EMERGENCY STOP LATCHED' : 'emergency stop clear'}; ${real.task ? real.task.state + (real.task.message ? ' — ' + real.task.message : '') : ''}` : 'Real arm: not answering.';
  $('height').disabled = isReal; if (isReal) $('height').value = '3';
  const labels = pointLabels(), box = $('points'), ok = locked && (isReal ? real.connected : drive.sim && drive.sim.connected) && !busy;
  if (box.children.length !== 5) { box.replaceChildren(...[0, 1, 2, 3, 4].map(i => { const b = document.createElement('button'); b.dataset.i = i; b.onclick = () => goPoint(i); return b; })); }
  [...box.children].forEach((b, i) => {
    const name = i < 4 ? labels[i] || `Corner ${i + 1}` : 'Park';
    b.replaceChildren(document.createTextNode(name)); const small = document.createElement('small'); small.textContent = i < 4 ? '3 cm above' : '3 cm above ' + (labels[0] || 'corner 1').replace(/^1 · /, ''); b.append(small);
    b.disabled = !ok; b.classList.toggle('on-real', isReal);
  });
  $('tour-next').disabled = $('tour-next').disabled || isReal; $('tour-prev').disabled = $('tour-prev').disabled || isReal;
  $('go-target').disabled = !(target && locked && (isReal ? real.connected : drive.sim && drive.sim.connected)) || busy;
  $('go-target').textContent = isReal ? 'Move REAL arm here' : 'Move simulator here'; $('go-target').classList.toggle('on-real', isReal);
}
async function goPoint(i) {
  const index = i < 4 ? i : 0, name = i < 4 ? pointLabels()[i] : 'Park (3 cm above the bottom-left corner)';
  if (realSelected()) {
    if (!confirm(`Move the REAL arm to ${name}?\n\nIt will rise, glide at 5 cm and stop 3 cm above the board, slowly (20 to 40 seconds).\nWatch the arm and keep Stop (Esc) ready.`)) return;
    const r = await drivePost('/api/drive/corner', {index, height_cm: 3, closed_loop: false, target: 'real', confirm_real: true}, `REAL arm: ${name}`);
    if (r) tourIndex = index;
  } else {
    const r = await drivePost('/api/drive/corner', {index, height_cm: i < 4 ? heightCm() : 3, closed_loop: i < 4 && closedLoop()}, `Simulator: ${name}`);
    if (r) tourIndex = index;
  }
}
document.querySelectorAll('input[name=arm]').forEach(r => r.addEventListener('change', () => { target = null; renderDrive(); }));

async function pollDrive() {
  const wall0 = Date.now(), t0 = performance.now();
  try {
    const data = await (await fetch('/api/drive', {cache: 'no-store'})).json();
    const rtt = performance.now() - t0;
    clock.rtt = rtt;
    if (data.enabled && rtt <= clock.bestRtt * 1.3) { clock.bestRtt = Math.min(clock.bestRtt, rtt); clock.offset = data.server_ms - (wall0 + rtt / 2); }
    clock.bestRtt *= 1.01;                                   // forget slowly, so the offset follows clock drift
    drive = data;
  } catch (_) { clock.rtt = null; if (drive) drive.sim = {connected: false, error: 'no answer from the page server'}; }
  renderDrive();
  setTimeout(pollDrive, 400);
}

async function sendCommand(path, body) {
  const cid = (crypto.randomUUID ? crypto.randomUUID() : String(Math.random()).slice(2) + Date.now());
  const envelope = {cid, sent_ms: serverNow(), gen: drive.generation, ...body};
  for (let attempt = 0; attempt < 2; attempt++) {            // a lost reply is retried with the SAME id: the server never moves the arm twice
    try {
      const r = await fetch(path, {method: 'POST', headers: {'Content-Type': 'application/json'}, body: JSON.stringify(envelope), signal: AbortSignal.timeout(6000)});
      const data = await r.json().catch(() => ({}));
      if (!r.ok) throw Object.assign(new Error(data.error || `Request failed (${r.status})`), {code: data.code});
      return data;
    } catch (e) { if (e.code || attempt === 1) throw e; await new Promise(res => setTimeout(res, 300)); }
  }
}
async function drivePost(path, body, what) {
  status(`${what}…`);
  try { const r = await sendCommand(path, body); status(r.duplicate ? `${what}: already received` : `${what}: sent`, 'ok'); return r; }
  catch (e) {
    const hint = e.code === 'stale' ? ' The link is lagging: nothing was moved. Try again.' : e.code === 'old_generation' ? ' (you pressed Stop after sending it).' : '';
    status(`${what} refused: ${e.message}${hint}`, 'bad'); return null;
  } finally { pollDriveSoon(); }
}
const pollDriveSoon = () => setTimeout(async () => { try { drive = await (await fetch('/api/drive', {cache: 'no-store'})).json(); renderDrive(); } catch (_) {} }, 150);
const heightCm = () => Number($('height').value);

const closedLoop = () => $('closed-loop').checked && !$('closed-loop').disabled;
$('go-target').addEventListener('click', async () => {
  if (!target) return;
  if (realSelected()) {
    const t = $('target-text').textContent;
    if (!confirm(`Move the REAL arm to the point you clicked?\n\n${t}\n\nIt will rise, glide at 5 cm and stop 3 cm above the board.\nWatch the arm and keep Stop (Esc) ready.`)) return;
    await drivePost('/api/drive/go', {px: target.px, py: target.py, height_cm: 3, closed_loop: false, target: 'real', confirm_real: true}, 'REAL arm: move to the clicked point');
    return;
  }
  drivePost('/api/drive/go', {px: target.px, py: target.py, height_cm: heightCm(), closed_loop: closedLoop()}, closedLoop() ? 'Move to the target (closed loop)' : 'Move to the target');
});
async function tourTo(index) {
  const r = await drivePost('/api/drive/corner', {index, height_cm: heightCm(), closed_loop: closedLoop()}, `Move to corner ${index + 1}${closedLoop() ? ' (closed loop)' : ''}`);
  if (r) { tourIndex = index; const c = drive.corners[index]; $('tour-text').textContent = `Corner ${index + 1} of 4, ${c.name} (u ${c.u}, v ${c.v}): the arm goes to x ${(c.x * 100).toFixed(1)}, y ${(c.y * 100).toFixed(1)} cm from its base.`; }
}
$('tour-next').addEventListener('click', () => tourTo((tourIndex + 1) % 4));
$('tour-prev').addEventListener('click', () => { if (tourIndex > 0) tourTo(tourIndex - 1); });
$('faces').addEventListener('change', async () => { await fetch('/api/drive/faces', {method: 'POST', headers: {'Content-Type': 'application/json'}, body: JSON.stringify({faces: $('faces').value})}); target = null; pollDriveSoon(); });

async function stopMotion() {                                // a soft stop: always accepted, whatever the lag or generation
  status('Stopping…', 'bad');
  try {
    const r = await (await fetch('/api/drive/stop', {method: 'POST', headers: {'Content-Type': 'application/json'}, body: '{}', signal: AbortSignal.timeout(6000)})).json();
    if (drive) drive.generation = r.generation;
    status(r.held ? 'Stopped. The arm is held where it is; you can send the next move.' : 'Stop sent, but the simulator did not answer.', r.held ? 'ok' : 'bad');
  } catch (_) { status('The stop did not get through (no answer from the page server). Try again.', 'bad'); }
  pollDriveSoon();
}
$('stop-motion').addEventListener('click', stopMotion);
document.addEventListener('keydown', e => { if (e.key === 'Escape') { e.preventDefault(); stopMotion(); } });

overlay.addEventListener('pointerup', e => {                 // a click on the picture chooses a target (it moves nothing by itself)
  if (!shown || !drive || !drive.enabled || !lockedMatrix() || !downPoint || Math.hypot(e.clientX - downPoint[0], e.clientY - downPoint[1]) > 8) return;
  const rect = overlay.getBoundingClientRect(), k = shown.detection.width / rect.width;
  const px = (e.clientX - rect.left) * k, py = (e.clientY - rect.top) * k, uv = uvOfPixel(px, py);
  if (!uv || uv[0] < -0.03 || uv[0] > 1.03 || uv[1] < -0.03 || uv[1] > 1.03) { status('That click is outside the rectangle.', 'bad'); return; }
  target = {px, py}; const xy = xyOfUv(uv[0], uv[1]);
  $('target-text').textContent = `u ${uv[0].toFixed(2)}, v ${uv[1].toFixed(2)}${xy ? ` → x ${(xy[0] * 100).toFixed(1)}, y ${(xy[1] * 100).toFixed(1)} cm from the arm base` : ''}`;
  renderDrive();
});
let downPoint = null;
overlay.addEventListener('pointerdown', e => { downPoint = [e.clientX, e.clientY]; });
pollDrive();
poll();


// Gripper (first-person) camera: a live stream through this server; the checkbox stops it to save bandwidth on a laggy link.
(function () {
  const img = $('pov'), box = $('pov-on'), off = $('pov-off');
  const apply = () => {
    try { localStorage.setItem('pov-on', box.checked ? '1' : '0'); } catch (e) { /* storage may be blocked */ }
    off.hidden = box.checked; img.hidden = !box.checked;
    if (box.checked) img.src = '/wrist.mjpg?' + Date.now(); else img.removeAttribute('src');
  };
  try { if (localStorage.getItem('pov-on') === '0') box.checked = false; } catch (e) { /* ignore */ }
  box.addEventListener('change', apply);
  img.addEventListener('error', () => { if (box.checked) setTimeout(() => { if (box.checked) img.src = '/wrist.mjpg?' + Date.now(); }, 3000); });
  apply();
})();


// The simulator's own page, embedded in the right column (same machine; the address comes from the server, set at start-up).
(function () {
  const frame = $('sim-view'), box = $('sim-on'), off = $('sim-off');
  let url = '';
  const apply = () => {
    try { localStorage.setItem('sim-on', box.checked ? '1' : '0'); } catch (e) { /* ignore */ }
    off.hidden = box.checked || !url; frame.hidden = !box.checked || !url;
    if (box.checked && url && frame.getAttribute('src') !== url) frame.src = url;
    if (!box.checked) frame.removeAttribute('src');
  };
  try { if (localStorage.getItem('sim-on') === '0') box.checked = false; } catch (e) { /* ignore */ }
  box.addEventListener('change', apply);
  fetch('/api/drive').then(r => r.json()).then(d => { url = d.sim_page || ''; if (!url) off.textContent = 'No simulator address configured.'; apply(); }).catch(() => { off.hidden = false; });
})();


// The big progress bar at the top: what the selected arm is doing right now.
const PHASES = {rise: 'Lifting the gripper', glide: 'Travelling over the board', hover: 'Lowering to 3 cm above the board', up: 'Lifting', down: 'Lowering',
                trim: 'Fine-tuning the height (gravity sag)', done: 'Arrived'};
function renderProgress() {
  if (!drive || !drive.enabled) return;
  const real = realSelected(), arm = real ? (drive.real || {}) : (drive.sim || {}), task = arm.task || {};
  const bar = $('progress'), name = real ? 'REAL arm' : 'Simulator';
  $('pg-arm').textContent = name; $('pg-arm').classList.toggle('real', real);
  let tone = 'idle', state = 'Ready', detail = 'Waiting for a command.', pct = 0;
  if (!arm.connected && !(arm.task)) { state = 'Not connected'; detail = real ? 'The real arm is not answering.' : 'The simulator is not answering.'; tone = 'bad'; }
  else if (real && arm.emergency_stop) { state = 'EMERGENCY STOP latched'; detail = 'Reset it in the manual page before moving.'; tone = 'bad'; }
  else if (task.state === 'planning') { state = 'Planning the path'; detail = 'Working out the joint angles…'; tone = 'busy'; pct = 2; }
  else if (task.state === 'running') {
    const total = Number(task.total) || 0, index = Number(real ? (task.index ?? 0) + 1 : task.step) || 0;
    pct = total ? Math.min(99, Math.round(100 * index / total)) : 5;
    state = 'Moving'; tone = 'busy';
    detail = (real ? (PHASES[task.step] || task.step || '') : (task.message || '')) + (total ? `  (step ${Math.min(index, total)} of ${total})` : '');
  } else if (task.state === 'done') { state = 'Arrived'; detail = real ? 'Move finished; control released.' : (task.message || 'Move finished.'); tone = 'ok'; pct = 100; }
  else if (task.state === 'stopped') { state = 'Stopped'; detail = 'The arm holds where it is. You can send the next move.'; tone = 'bad'; pct = 0; }
  else if (task.state === 'error') { state = 'Stopped'; detail = task.message || 'The move could not be completed.'; tone = 'bad'; }
  else if (real && arm.owner) { state = 'Busy'; detail = `Controlled by ${arm.owner}.`; tone = 'busy'; }
  $('pg-state').textContent = state; $('pg-detail').textContent = detail; $('pg-pct').textContent = `${pct}%`;
  $('pg-fill').style.width = `${pct}%`; bar.dataset.tone = tone;
  const t = arm.tip_base; $('pg-tip').textContent = t ? `Tip (the arm's own estimate): x ${(t.x * 100).toFixed(1)}, y ${(t.y * 100).toFixed(1)}, height ${(t.z * 100).toFixed(1)} cm` + (real && arm.gripper != null ? ` · gripper ${Math.round(arm.gripper)}` : '') : '';
}
const _renderDrive = renderDrive;
renderDrive = function () { _renderDrive(); renderProgress(); };
renderProgress();


// The staged real grasp: each button is one step; the agent says which steps are allowed next.
const GRASP_TEXT = {
  open: 'Open the gripper to 80 (a 73 mm gap). The arm does not move.',
  hover: 'Move the gripper to about 6 cm above the block you clicked, with the open jaws straddling it.',
  lower: 'Lower the open gripper straight down to the height of the block\'s centre, in 1 cm steps. It stops if anything resists.',
  close: 'Close the gripper slowly until it stalls on the block.',
  lift: 'Lift the block 6 cm straight up.',
  putback: 'Lower the block back, open the gripper and rise clear.',
};
function renderGrasp() {
  const card = $('grasp-card');
  const real = (drive && drive.real) || {}, g = real.grasp;
  card.hidden = !(realSelected() && g);
  if (card.hidden) return;
  const busy = driveBusy(), allowed = new Set(g.next || []);
  $('grasp-text').textContent = `Stage: ${g.stage}.` + (g.hold_value ? ` Jaws stalled at ${g.hold_value.toFixed(1)}.` : '') + (g.last ? ` ${g.last}` : '') + (g.block ? '' : ' Click the block on the picture first.');
  card.querySelectorAll('button[data-grasp]').forEach(b => {
    const a = b.dataset.grasp;
    b.disabled = busy || !allowed.has(a) || (a === 'hover' && !target) || !real.connected;
    b.classList.toggle('next', !b.disabled && a !== 'open');
  });
}
document.querySelectorAll('#grasp-card button[data-grasp]').forEach(b => b.addEventListener('click', async () => {
  const action = b.dataset.grasp;
  if (!confirm(`REAL ARM: ${GRASP_TEXT[action]}\n\nWatch the gripper camera and keep Stop (Esc) ready. Continue?`)) return;
  const body = {action, confirm_real: true};
  if (action === 'hover') { body.px = target.px; body.py = target.py; }
  await drivePost('/api/drive/grasp', body, `Grasp step: ${action}`);
}));
const _renderDrive2 = renderDrive;
renderDrive = function () { _renderDrive2(); renderGrasp(); };
renderGrasp();
