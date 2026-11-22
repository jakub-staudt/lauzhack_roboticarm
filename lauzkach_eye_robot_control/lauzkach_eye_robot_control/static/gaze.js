'use strict';
// Full-screen overhead camera, driven by the eye tracker (gaze_service) or by the mouse.
// Look at a block for 2 s to pick it; look at a free spot in the rectangle for 2 s to choose where it goes; then "Select" moves it.
const $ = id => document.getElementById(id);
const cam = $('cam'), overlay = $('overlay'), ctx = overlay.getContext('2d');
const DWELL_S = 2.0, BUTTON_DWELL_S = 1.5, DECAY = 1.5, GRACE_S = 0.8, GAZE_HZ = 30;
const BLOCK_CSS = {red: '#e5383b', blue: '#3a6ff7', green: '#2fb344', yellow: '#f2c200'};

// ------------------------------------------------------------------ state
let det = null;                    // the latest overhead detection: {width, height, polygon, blocks}
let drive = null;                  // the arm side: /api/drive
let trackerUrl = '';
let gaze = {x: null, y: null, phase: 'idle', face: false, calibrated: false, target: null, camera: false, error: '', mock: false};
let pick = null, dest = null;      // pick: a detected block; dest: {px, py} in picture pixels
let dwell = {key: '', t: 0, point: null}, grace = 0, mouse = null, confirming = false, status = null;
const clock = {offset: 0};
const serverNow = () => Date.now() + clock.offset;

// ------------------------------------------------------------------ the picture <-> screen
function view() {                                              // where the picture sits on screen (object-fit: contain)
  const W = overlay.clientWidth, H = overlay.clientHeight, iw = (det && det.width) || 640, ih = (det && det.height) || 480;
  const s = Math.min(W / iw, H / ih);
  return {s, ox: (W - iw * s) / 2, oy: (H - ih * s) / 2, W, H};
}
const toScreen = (p, v = view()) => [v.ox + p[0] * v.s, v.oy + p[1] * v.s];
const toImage = (x, y, v = view()) => [(x - v.ox) / v.s, (y - v.oy) / v.s];
function insidePolygon(p, poly) {
  let c = false;
  for (let i = 0, j = poly.length - 1; i < poly.length; j = i++) {
    const [xi, yi] = poly[i], [xj, yj] = poly[j];
    if ((yi > p[1]) !== (yj > p[1]) && p[0] < (xj - xi) * (p[1] - yi) / (yj - yi) + xi) c = !c;
  }
  return c;
}
const blocks = () => (det && det.blocks) || [];
const sameBlock = (a, b) => a && b && Math.hypot(a.px[0] - b.px[0], a.px[1] - b.px[1]) < 40;       // the detected centre jitters: match by place
const holding = () => Boolean(drive && drive.real && drive.real.grasp && ['closed', 'lifted', 'carried'].includes(drive.real.grasp.stage));
const seq = () => (drive && drive.real && drive.real.sequence) || null;
const busy = () => Boolean(seq() && seq().state === 'running');

// ------------------------------------------------------------------ talking to the page server and the eye tracker
async function getJson(url, ms = 3000) { const r = await fetch(url, {cache: 'no-store', signal: AbortSignal.timeout(ms)}); return r.json(); }
async function pollDetect() {
  try { const d = await getJson('/api/detect'); const r = d.result || d; if (r.detection) det = {width: r.detection.width, height: r.detection.height, polygon: r.polygon_shown, blocks: r.blocks || [], locked: r.locked}; } catch (_) { /* keep the last */ }
  setTimeout(pollDetect, 400);
}
async function pollDrive() {
  try {
    const t0 = Date.now(); drive = await getJson('/api/drive'); trackerUrl = drive.tracker_url || trackerUrl;
    const rtt = Date.now() - t0; if (drive.server_ms) clock.offset = drive.server_ms - (t0 + rtt / 2);
  } catch (_) { drive = null; }
  setTimeout(pollDrive, 600);
}
async function pollGaze() {
  if (!trackerUrl) { setTimeout(pollGaze, 300); return; }
  try { gaze = await getJson(trackerUrl + '/api/gaze', 1200); } catch (_) { gaze = {...gaze, camera: false, error: 'The eye tracker is not running.', x: null, y: null}; }
  setTimeout(pollGaze, 1000 / GAZE_HZ);
}
async function trackerPost(path) {                           // text/plain: a simple request, so the tracker needs no CORS preflight
  try { await fetch(trackerUrl + path, {method: 'POST', headers: {'Content-Type': 'text/plain'}, body: JSON.stringify({screen: [window.innerWidth, window.innerHeight]}), signal: AbortSignal.timeout(4000)}); } catch (_) { say('Could not reach the eye tracker.', 'Start it with: python -m gaze_service.server --port 8830', 'bad'); }
}
async function command(path, body) {
  const cid = crypto.randomUUID ? crypto.randomUUID() : String(Math.random()).slice(2) + Date.now();
  const r = await fetch(path, {method: 'POST', headers: {'Content-Type': 'application/json'}, body: JSON.stringify({cid, sent_ms: serverNow(), gen: drive ? drive.generation : 0, ...body}), signal: AbortSignal.timeout(6000)});
  const data = await r.json().catch(() => ({}));
  if (!r.ok) throw new Error(data.error || `Request failed (${r.status})`);
  return data;
}

// ------------------------------------------------------------------ messages
function say(title, detail = '', tone = '') { status = {title, detail, tone, until: Date.now() + 6000}; }
function headline() {
  const s = seq();
  if (status && Date.now() < status.until) return status;
  if (s && s.state === 'running') return {title: 'Moving the block', detail: `Step ${s.step} of ${s.total}: ${s.name}`};
  if (s && s.state === 'done' && !pick && !dest) return {title: 'Block moved', detail: s.message};
  if (s && (s.state === 'error' || s.state === 'stopped') && !pick && !dest) return {title: 'Stopped', detail: s.message, tone: 'bad'};
  if (holding() && !dest) return {title: 'The arm holds a block', detail: 'Look at a free spot for 2 seconds to choose where it goes.'};
  if (!pick && !holding()) return {title: 'Look at a block', detail: 'Look at it for 2 seconds to pick it.'};
  if (!dest) return {title: `${pick.colour} block chosen`, detail: 'Now look at a free spot inside the rectangle for 2 seconds.'};
  return {title: 'Ready', detail: 'Look at "Select" for 1.5 seconds to move it, or "Restart selection".'};
}

// ------------------------------------------------------------------ choosing
function choosePick(b) { pick = {...b}; dest = null; say(`${b.colour} block chosen`, 'Now look at a free spot inside the rectangle.'); }
function chooseDest(px) { dest = {px: px[0], py: px[1]}; say('Destination chosen', 'Look at "Select" to move it.'); }
function restart() { pick = null; dest = null; confirming = false; say('Selection cleared', 'Look at a block to start again.'); }
function canSelect() { return (pick || holding()) && dest && !busy() && drive && drive.real && drive.real.connected; }
function ask() { if (!canSelect()) { say('Not ready', !dest ? 'Choose a spot first.' : 'Choose a block first.', 'bad'); return; } confirming = true; $('confirm-text').textContent = holding() ? 'Carry the block the arm is holding to the chosen spot?' : `Move the ${pick.colour} block to the chosen spot?`; }
async function go() {
  confirming = false;
  try {
    await command('/api/drive/pickplace', {pick: pick ? {px: pick.px[0], py: pick.px[1], box: pick.box_px, colour: pick.colour} : null, dest, others: blocks().filter(b => !sameBlock(b, pick)).map(b => b.px), park_after: true, confirm_real: true});
    say('Moving the block', 'The arm runs every step by itself. "Cancel" stops it.'); pick = null; dest = null;
  } catch (e) { say('The arm did not start', e.message, 'bad'); }
}
async function stop() {
  confirming = false;
  try { await fetch('/api/drive/stop', {method: 'POST', headers: {'Content-Type': 'application/json'}, body: '{}', signal: AbortSignal.timeout(5000)}); say('Stopped', 'The arm holds where it is.', 'bad'); } catch (_) { say('The stop did not get through', 'Try again (Esc).', 'bad'); }
}
const ACTIONS = {
  select: () => ask(), restart: () => restart(), stop: () => stop(), yes: () => go(), no: () => { confirming = false; say('Not moved', ''); },
  calibrate: () => { if (gaze.phase === 'calibrating' || gaze.phase === 'correcting') trackerPost('/api/cancel'); else trackerPost(gaze.savedCalibration && gaze.calibrated ? '/api/correct' : '/api/calibrate'); },
};

// ------------------------------------------------------------------ the pointer: gaze (or mouse) and dwell
function pointer() {
  if (gaze.x != null && gaze.face && gaze.phase === 'tracking') return {x: gaze.x * overlay.clientWidth, y: gaze.y * overlay.clientHeight, src: 'gaze'};
  if (mouse && Date.now() - mouse.t < 15000) return {x: mouse.x, y: mouse.y, src: 'mouse'};
  return null;
}
function buttonAt(p) {
  for (const el of document.querySelectorAll('.gbtn:not(:disabled)')) {
    if (el.closest('[hidden]')) continue;
    const r = el.getBoundingClientRect(); if (p.x >= r.left - 12 && p.x <= r.right + 12 && p.y >= r.top - 12 && p.y <= r.bottom + 12) return el;
  }
  return null;
}
function targetAt(p, v) {
  const el = buttonAt(p); if (el) return {key: 'btn:' + el.dataset.key, kind: 'button', el};
  if (confirming || !det) return null;
  let best = null;
  for (const b of blocks()) {
    const c = toScreen(b.px, v), size = Math.max(...(b.size_px || [40, 40])) * v.s, R = Math.max(58, size * 0.9), d = Math.hypot(p.x - c[0], p.y - c[1]);
    if (d < R && (!best || d < best.d)) best = {key: 'blk:' + b.colour, kind: 'block', block: b, d};
  }
  if (best) return best;
  if (det.polygon && (pick || holding())) { const ip = toImage(p.x, p.y, v); if (insidePolygon(ip, det.polygon)) return {key: 'spot', kind: 'spot', point: ip}; }
  return null;
}
function fire(t) {
  if (t.kind === 'button') { ACTIONS[t.el.dataset.key] && ACTIONS[t.el.dataset.key](); return; }
  if (busy()) { say('The arm is busy', 'Wait, or look at "Cancel".', 'bad'); return; }
  if (t.kind === 'block') { if (holding()) { say('The arm already holds a block', 'Choose a free spot.', 'bad'); return; } choosePick(t.block); return; }
  if (t.kind === 'spot') {
    if (blocks().some(b => !sameBlock(b, pick) && Math.hypot(b.px[0] - t.point[0], b.px[1] - t.point[1]) < 36)) { say('That spot is on a block', 'Choose a free spot.', 'bad'); return; }
    chooseDest(t.point);
  }
}
let last = performance.now();
function step(now) {
  const dt = Math.min(0.2, (now - last) / 1000); last = now;
  const v = view(), p = pointer(), calibrating = gaze.phase === 'calibrating' || gaze.phase === 'correcting';
  grace = Math.max(0, grace - dt);
  const t = (!calibrating && p && grace === 0) ? targetAt(p, v) : null;
  const need = t && t.kind === 'button' ? BUTTON_DWELL_S : DWELL_S;
  if (t && p.src === 'gaze') {                                  // mouse users click instead of dwelling
    if (t.key === dwell.key) { dwell.t += dt; if (t.kind === 'spot') dwell.point = dwell.point ? [dwell.point[0] * .85 + t.point[0] * .15, dwell.point[1] * .85 + t.point[1] * .15] : t.point; }
    else { dwell.t = Math.max(0, dwell.t - DECAY * dt); if (dwell.t === 0) dwell = {key: t.key, t: 0, point: t.point || null}; }
    if (dwell.t >= need) { if (t.kind === 'spot') t.point = dwell.point || t.point; fire(t); dwell = {key: '', t: 0, point: null}; grace = GRACE_S; }
  } else { dwell.t = Math.max(0, dwell.t - DECAY * dt); if (dwell.t === 0) dwell.key = ''; }
  draw(v, p, t, need);
  requestAnimationFrame(step);
}

// ------------------------------------------------------------------ drawing
function fitCanvas() { const d = window.devicePixelRatio || 1; overlay.width = overlay.clientWidth * d; overlay.height = overlay.clientHeight * d; ctx.setTransform(d, 0, 0, d, 0, 0); }
function poly(points, close = true) { ctx.beginPath(); points.forEach((p, i) => (i ? ctx.lineTo(p[0], p[1]) : ctx.moveTo(p[0], p[1]))); if (close) ctx.closePath(); }
function outline(points, color, w) { ctx.lineJoin = 'round'; ctx.strokeStyle = '#fff'; ctx.lineWidth = w + 3; poly(points); ctx.stroke(); ctx.strokeStyle = color; ctx.lineWidth = w; poly(points); ctx.stroke(); }
function draw(v, p, t, need) {
  ctx.clearRect(0, 0, overlay.clientWidth, overlay.clientHeight);
  if (det && det.polygon) { ctx.setLineDash([10, 8]); outline(det.polygon.map(q => toScreen(q, v)), '#35c759', 2); ctx.setLineDash([]); }
  for (const b of blocks()) {
    const box = (b.box_px || []).map(q => toScreen(q, v)), picked = sameBlock(b, pick), c = toScreen(b.px, v);
    if (box.length === 4) outline(box, BLOCK_CSS[b.colour] || '#fff', picked ? 6 : 3);
    if (picked) { ctx.strokeStyle = '#ff3b30'; ctx.lineWidth = 4; ctx.beginPath(); ctx.moveTo(c[0] - 16, c[1]); ctx.lineTo(c[0] + 16, c[1]); ctx.moveTo(c[0], c[1] - 16); ctx.lineTo(c[0], c[1] + 16); ctx.stroke(); }
    label(picked ? `${b.colour} (picked)` : b.colour, c[0] + 30, c[1] - 24, picked ? '#ffd2cf' : '#fff');
  }
  if (pick && dest && pick.box_px) {                              // the destination: a square like the block, and a line to it
    const dx = dest.px - pick.px[0], dy = dest.py - pick.px[1], sq = pick.box_px.map(q => toScreen([q[0] + dx, q[1] + dy], v)), a = toScreen(pick.px, v), z = toScreen([dest.px, dest.py], v);
    ctx.setLineDash([12, 8]); ctx.strokeStyle = '#fff'; ctx.lineWidth = 7; ctx.beginPath(); ctx.moveTo(a[0], a[1]); ctx.lineTo(z[0], z[1]); ctx.stroke(); poly(sq); ctx.stroke();
    ctx.strokeStyle = '#1f8a3b'; ctx.lineWidth = 3.5; ctx.beginPath(); ctx.moveTo(a[0], a[1]); ctx.lineTo(z[0], z[1]); ctx.stroke(); poly(sq); ctx.stroke(); ctx.setLineDash([]);
    label('destination', z[0] + 32, z[1] - 6, '#b8f0c6');
  } else if (dest) { const z = toScreen([dest.px, dest.py], v); ctx.strokeStyle = '#1f8a3b'; ctx.lineWidth = 5; ctx.beginPath(); ctx.arc(z[0], z[1], 26, 0, 7); ctx.stroke(); label('destination', z[0] + 32, z[1] - 6, '#b8f0c6'); }
  if (t && t.kind !== 'button' && dwell.key === t.key && dwell.t > 0) {                 // the dwell ring on what you are looking at
    const c = t.kind === 'block' ? toScreen(t.block.px, v) : (dwell.point ? toScreen(dwell.point, v) : [p.x, p.y]), f = Math.min(1, dwell.t / need);
    ctx.lineWidth = 9; ctx.strokeStyle = '#ffffff55'; ctx.beginPath(); ctx.arc(c[0], c[1], 44, 0, 7); ctx.stroke();
    ctx.strokeStyle = '#ffd23f'; ctx.beginPath(); ctx.arc(c[0], c[1], 44, -Math.PI / 2, -Math.PI / 2 + 2 * Math.PI * f); ctx.stroke();
  }
  document.querySelectorAll('.gbtn').forEach(el => {              // button dwell: a fill that grows while you look at it
    let fill = el.querySelector('.fill'); if (!fill) { fill = document.createElement('i'); fill.className = 'fill'; el.prepend(fill); }
    fill.style.width = t && t.kind === 'button' && t.el === el && dwell.key === t.key ? `${Math.min(100, 100 * dwell.t / need)}%` : '0%';
  });
  if (p && p.src === 'gaze') { ctx.fillStyle = '#ffd23f'; ctx.strokeStyle = '#000'; ctx.lineWidth = 2; ctx.beginPath(); ctx.arc(p.x, p.y, 13, 0, 7); ctx.fill(); ctx.stroke(); }
  syncUi();
}
function label(text, x, y, color) { ctx.font = '600 15px system-ui'; ctx.lineWidth = 4; ctx.strokeStyle = '#000'; ctx.strokeText(text, x, y); ctx.fillStyle = color; ctx.fillText(text, x, y); }

// ------------------------------------------------------------------ the page around the picture
let lastUi = '';
function syncUi() {
  const h = headline(), calib = gaze.phase === 'calibrating' || gaze.phase === 'correcting';
  $('msg-title').textContent = h.title; $('msg-detail').textContent = h.detail || ''; $('msg').style.outline = h.tone === 'bad' ? '3px solid var(--bad)' : 'none';
  const chip = $('chip-tracker');
  chip.className = 'chip ' + (!gaze.camera ? 'bad' : !gaze.calibrated ? 'warn' : gaze.face ? 'ok' : 'warn');
  chip.textContent = !gaze.camera ? (gaze.error || 'Eye tracker not running') : calib ? `Calibrating ${gaze.target ? gaze.target.index + 1 + '/' + gaze.target.total : ''}` : !gaze.calibrated ? 'Eye tracker: calibrate' : gaze.face ? 'Eye tracker: tracking' + (gaze.mock ? ' (mock)' : '') : 'Eye tracker: no face';
  const arm = $('chip-arm'), real = drive && drive.real;
  arm.className = 'chip ' + (!drive ? 'bad' : real && real.connected ? 'ok' : 'warn'); arm.textContent = !drive ? 'Page server not answering' : real && real.connected ? 'Real arm connected' : 'Real arm not available';
  const s = seq(), bar = $('progress');
  bar.style.display = s && s.state === 'running' ? 'block' : 'none';
  if (s && s.state === 'running') { $('progress-fill').style.width = `${Math.round(100 * (s.step - 0.5) / s.total)}%`; $('progress-text').textContent = `Step ${s.step} of ${s.total}: ${s.name}`; }
  $('b-select').disabled = !canSelect(); $('b-select').classList.toggle('ready', Boolean(canSelect()));
  $('b-restart').disabled = busy(); $('b-calib').querySelector('span').textContent = calib ? 'Cancel calibration' : 'Calibrate eyes';
  $('calib-hint').textContent = gaze.calibrated ? 'recalibrate' : 'look at the dots';
  $('confirm').hidden = !confirming; $('calib-layer').hidden = !calib;
  if (calib && gaze.target) { const d = $('calib-dot'); d.style.left = gaze.target.x + 'px'; d.style.top = gaze.target.y + 'px'; d.style.transform = `scale(${gaze.target.recording ? 0.7 : 1})`; $('calib-text').textContent = gaze.phase === 'calibrating' ? 'Follow the dot with your eyes. Keep your head still.' : 'Look at each cross.'; }
}

// ------------------------------------------------------------------ mouse and keys (for testing and as a fallback)
overlay.addEventListener('mousemove', e => { mouse = {x: e.clientX, y: e.clientY, t: Date.now()}; });
overlay.addEventListener('click', e => {
  const v = view(), p = {x: e.clientX, y: e.clientY}, t = targetAt(p, v); if (t) fire(t);
});
document.querySelectorAll('.gbtn').forEach(el => el.addEventListener('click', () => { const a = ACTIONS[el.dataset.key]; if (a) a(); }));
document.addEventListener('keydown', e => {
  if (e.key === 'Escape') { e.preventDefault(); stop(); } else if (e.key === 'f' || e.key === 'F') toggleFull(); else if (e.key === 'r' || e.key === 'R') restart();
  else if (e.key === 'k' || e.key === 'K') ACTIONS.calibrate();
});
function toggleFull() { if (document.fullscreenElement) document.exitFullscreen(); else document.documentElement.requestFullscreen().catch(() => {}); }
$('btn-full').addEventListener('click', toggleFull);
window.addEventListener('resize', fitCanvas);

// ------------------------------------------------------------------ go
cam.src = '/overhead.mjpg?' + Date.now();
cam.addEventListener('error', () => setTimeout(() => { cam.src = '/overhead.mjpg?' + Date.now(); }, 2500));
fitCanvas(); pollDetect(); pollDrive(); pollGaze(); requestAnimationFrame(step);
