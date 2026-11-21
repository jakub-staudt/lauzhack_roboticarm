'use strict';

const $ = id => document.getElementById(id);
const cam = $('cam'), overlay = $('overlay'), ctx = overlay.getContext('2d');
let latest = null;      // newest answer from the server
let shown = null;       // the answer whose frame is on screen
let paused = false;
let loading = 0;        // sequence number of the frame being loaded

const layers = () => ({
  rect: $('l-rect').checked, raw: $('l-raw').checked, chosen: $('l-chosen').checked,
  lines: $('l-lines').checked, tags: $('l-tags').checked, left: $('l-left').checked, grid: $('l-grid').checked,
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
for (const id of ['l-rect', 'l-raw', 'l-chosen', 'l-lines', 'l-tags', 'l-left', 'l-grid']) $(id).addEventListener('change', draw);

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
poll();
