'use strict';
// Same protocol as the original dashboard: claim -> heartbeat; hold-jog = cartesian/start, renew every 100 ms, stop on release.
const $ = id => document.getElementById(id);
const KEYS = {w:'forward', s:'back', a:'left', d:'right', r:'up', f:'down', q:'open', e:'close',
              i:'tilt_up', k:'tilt_down', j:'roll_left', l:'roll_right'};
const JOINTS = ['shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex', 'wrist_roll', 'gripper'];

// Name shown to others while this page holds control. ?name=Jakub in the URL sets it; otherwise a random tag per tab.
const CLIENT_LABEL = (() => {
  const given = new URLSearchParams(location.search).get('name');
  if (given) return 'Eye-control UI (' + given.slice(0, 24) + ')';
  try { let t = sessionStorage.getItem('tag'); if (!t) { t = Math.random().toString(36).slice(2, 6); sessionStorage.setItem('tag', t); } return 'Eye-control UI #' + t; }
  catch (_) { return 'Eye-control UI #' + Math.random().toString(36).slice(2, 6); }
})();
let state = null, online = false, token = null, ownerId = null, claiming = false;
let hold = null, pending = false, seq = 0, stopping = false;

function note(text) { $('notice').textContent = text || ''; }
const own = () => token !== null && state?.control?.owner_id === ownerId;
const stopped = () => Boolean(state?.emergency_stop?.latched);

async function post(route, body = {}, ms = 1800) {
  const r = await fetch('/api/' + route, {method: 'POST', headers: {'Content-Type': 'application/json'},
                                          body: JSON.stringify(body), signal: AbortSignal.timeout(ms)});
  let data = {}; try { data = await r.json(); } catch (_) {}
  return {ok: r.ok, status: r.status, data};
}

// ------------------------------------------------------------ state polling and rendering
// The link to the server can be a slow relay: a single missed reply is not an outage. Show "unreachable" and stop
// holds only after several in a row; a held move is protected anyway by the controller's own renew watchdog.
let failures = 0, ownerMisses = 0, polling = false;
async function poll() {
  if (polling) return;  // never queue polls behind a slow one
  polling = true; let fresh = false;
  try {
    const r = await fetch('/api/state', {cache: 'no-store', signal: AbortSignal.timeout(2500)});
    if (!r.ok) throw new Error(r.status);
    state = await r.json(); online = true; failures = 0; fresh = true;
  } catch (_) { if (++failures >= 3) { online = false; if (hold) cancelHold(); } }
  finally { polling = false; }
  if (fresh && token !== null) {
    // Only an owner mismatch seen twice in a row counts, so one odd reply cannot drop control.
    if (state.control?.owner_id !== ownerId) { if (++ownerMisses >= 2) { ownerMisses = 0; loseControl('Control expired (heartbeats did not reach the arm in time). Take control again.'); } }
    else ownerMisses = 0;
  }
  render();
}
async function freshSequence() {
  // The controller accepts a hold-jog start only against an observation under 0.75 s old, so ask for one right now.
  try {
    const r = await fetch('/api/state', {cache: 'no-store', signal: AbortSignal.timeout(1200)});
    if (r.ok) { const s = await r.json(); state = s; return s.sequence; }
  } catch (_) {}
  return state?.sequence;
}
var render = function () {
  const pill = $('status');
  if (!online) { pill.textContent = 'controller unreachable'; pill.className = 'pill bad'; }
  else if (stopped()) { pill.textContent = 'STOPPED (latched)'; pill.className = 'pill bad'; }
  else if (!state.connected) { pill.textContent = 'arm disconnected / preview'; pill.className = 'pill warn'; }
  else {
    const owner = state.control?.owner_id, label = state.control?.label || 'someone';
    pill.textContent = own() ? 'you have control'
      : !owner ? 'ready'
      : label === CLIENT_LABEL ? 'held by an older session of this tab (wait ~2 s)'
      : label.startsWith('Eye-control UI') ? 'held by another Eye-control UI: ' + label + ' (close it, or wait if it just ended)'
      : 'in use by: ' + label;
    pill.className = 'pill ' + (own() ? 'ok' : '');
  }
  $('estop-reset').hidden = !stopped();
  $('claim').hidden = own(); $('release').hidden = !own();
  $('claim').disabled = !online || stopped() || claiming || !state?.connected || Boolean(state?.control?.owner_id && !own());
  const k = $('kbd-state');
  k.textContent = own() ? 'on' : 'needs control'; k.className = 'pill small ' + (own() ? 'ok' : '');
  document.querySelectorAll('.pad button').forEach(b => { b.disabled = !own() || stopped(); b.classList.toggle('active', hold?.direction === b.dataset.dir); });
  if (state?.measured) {
    $('joints').innerHTML = JOINTS.map(j => `<tr><td>${j}</td><td>${Number(state.measured[j]).toFixed(1)}</td></tr>`).join('');
    $('extra').textContent = `torque ${state.torque ? 'on' : 'OFF'} · feedback age ${(state.age_s ?? NaN).toFixed?.(2) ?? '?'} s`;
  }
}
setInterval(poll, 250); poll();

// ------------------------------------------------------------ control ownership
async function claim() {
  if (claiming || own() || stopped()) return;
  claiming = true; render(); note('');
  try {
    const {ok, data} = await post('control/claim', {label: CLIENT_LABEL, emergency_stop_generation: state.emergency_stop.generation});
    if (!ok) { note(data.error || 'Could not take control.'); return; }
    token = data.token; ownerId = data.owner_id; heartbeat();
  } catch (_) { note('Connection lost.'); } finally { claiming = false; poll(); }
}
function loseControl(msg) { cancelHold(); token = null; ownerId = null; if (msg) note(msg); }
var release = async function () {
  cancelHold(); const t = token; loseControl();
  if (t) try { await post('control/release', {token: t}); } catch (_) {}
  poll();
}
async function heartbeat() {
  // Overlapping requests are fine and keep the lease renewed when one is slow (the lease is 1.5 s).
  // Only an explicit refusal means control is gone; a timeout is just a slow link.
  const t = token; if (t === null) return;
  try {
    const {ok, status} = await post('control/heartbeat', {token: t}, 2500);
    if (!ok && token === t && [401, 403, 409].includes(status)) loseControl('Control lost (the controller no longer recognises this session). Take control again.');
  } catch (_) {}
}
setInterval(heartbeat, 250);
$('claim').onclick = claim; $('release').onclick = release;

// ------------------------------------------------------------ hold-to-move
function cancelHold() {
  const h = hold; if (!h) return;
  hold = null; clearTimeout(h.timer);
  const body = {token: h.token, sequence: ++seq}; if (h.motionId) body.motion_id = h.motionId;
  // A higher sequence also cancels a start whose reply has not arrived yet.
  fetch('/api/cartesian/stop', {method: 'POST', headers: {'Content-Type': 'application/json'}, body: JSON.stringify(body), keepalive: true}).catch(() => {});
  render();
}
async function step(h, route) {
  if (pending || hold !== h) return;
  pending = true;
  let retry = false;
  try {
    const body = {token: h.token};
    if (route === 'start') {
      const mode = $('recovery').checked && ['forward', 'back', 'up', 'down'].includes(h.direction) ? 'recovery' : 'normal';
      const observation = await freshSequence();
      if (hold !== h) return;
      Object.assign(body, {direction: h.direction, profile: $('profile').value, mode, observation_sequence: observation});
    } else body.motion_id = h.motionId;
    h.seq = body.sequence = ++seq;
    const {ok, data} = await post('cartesian/' + route, body, route === 'start' ? 1800 : 800);
    if (hold !== h) return;
    if (!ok && route === 'start' && /fresh observation/i.test(data.error || '') && (h.retries = (h.retries || 0) + 1) <= 3) { retry = true; return; }
    if (!ok || !data.cartesian?.active || !data.cartesian.motion_id) { cancelHold(); note(data.error || 'Motion ended. Release and hold again.'); return; }
    h.motionId = data.cartesian.motion_id; note('');
    h.timer = setTimeout(() => step(h, 'renew'), 100);
  } catch (_) { if (hold === h) { cancelHold(); note('Motion connection lost. Release and hold again.'); } }
  finally { pending = false; if (retry && hold === h) step(h, 'start'); }  // key still held: try again with a newer observation
}
function begin(direction, source) {
  if (hold || pending || !own() || stopped() || !online) return false;
  hold = {direction, source, token, motionId: null, seq: 0, timer: null};
  render(); step(hold, 'start'); return true;
}
const typing = t => t?.closest?.('input,textarea,select,[contenteditable]:not([contenteditable="false"])');
document.addEventListener('keydown', e => {
  if (e.key === 'Escape') { e.preventDefault(); softStop(); return; }
  const dir = KEYS[e.key?.toLowerCase()];
  if (!dir || e.repeat || e.altKey || e.ctrlKey || e.metaKey || e.shiftKey || typing(e.target)) return;
  if (begin(dir, 'key:' + e.key.toLowerCase())) e.preventDefault();
});
document.addEventListener('keyup', e => { if (hold?.source === 'key:' + e.key?.toLowerCase()) { e.preventDefault(); cancelHold(); } });
window.addEventListener('blur', cancelHold);
document.addEventListener('visibilitychange', () => { if (document.hidden) cancelHold(); });
$('profile').onchange = cancelHold;
$('recovery').onchange = cancelHold;  // changing mode ends any hold, as in the original panel
document.querySelectorAll('.pad button').forEach(b => {
  b.addEventListener('pointerdown', e => { if (e.button === 0 && begin(b.dataset.dir, 'pointer:' + e.pointerId)) { e.preventDefault(); try { b.setPointerCapture(e.pointerId); } catch (_) { cancelHold(); } } });
  for (const n of ['pointerup', 'pointercancel', 'lostpointercapture']) b.addEventListener(n, e => { if (hold?.source === 'pointer:' + e.pointerId) cancelHold(); });
  b.addEventListener('contextmenu', e => e.preventDefault());
});

// ------------------------------------------------------------ emergency stop
var emergencyStop = async function () {
  if (stopping) return; stopping = true;
  cancelHold(); token = null; ownerId = null;
  try { await post('emergency-stop', {}, 3000); note('STOP latched.'); } catch (_) { note('STOP request failed: check the arm and use the physical stop.'); }
  stopping = false; poll();
}
$('estop').onclick = emergencyStop;
$('estop-reset').onclick = async () => {
  try { const {ok, data} = await post('emergency-stop/reset', {emergency_stop_generation: state.emergency_stop.generation}, 3000); note(ok ? '' : (data.error || 'Reset refused.')); } catch (_) { note('Reset failed.'); }
  poll();
};

// Closing or reloading the page frees control immediately, instead of leaving a lease that blocks the next load for ~1.5 s.
window.addEventListener('pagehide', () => {
  cancelHold();
  if (token !== null) fetch('/api/control/release', {method: 'POST', headers: {'Content-Type': 'application/json'}, body: JSON.stringify({token}), keepalive: true}).catch(() => {});
});

// Cameras: live MJPEG streams (the dashboard streams 2 fps unless asked). Video shares the connection with the controls, so the frame
// rate and the wrist camera can be changed live here (or by ?fps=2|10|15|20 and ?wrist=0 in the address) when the link is laggy.
const _q = new URLSearchParams(location.search);
$('cam-fps').value = ['2', '10', '15', '20'].includes(_q.get('fps')) ? _q.get('fps') : '20';
$('wrist-on').checked = _q.get('wrist') !== '0';
const _streams = [];
function liveStream(imgId, urlFn, enabledFn = () => true) {
  const img = $(imgId); let timer = null;
  const connect = () => {
    clearTimeout(timer);
    if (!enabledFn() || document.hidden) { img.removeAttribute('src'); return; }   // dropping the src closes the stream
    img.src = urlFn() + '&t=' + Date.now();
  };
  img.addEventListener('error', () => { if (enabledFn()) timer = setTimeout(connect, 2000); });
  document.addEventListener('visibilitychange', connect);
  _streams.push(connect); connect();
}
const _camFps = () => $('cam-fps').value;
liveStream('overhead', () => `/cameras/overhead.mjpg?fps=${_camFps()}&profile=preview`);
liveStream('wrist', () => `/cameras/wrist.mjpg?fps=${_camFps()}&profile=preview`, () => $('wrist-on').checked);
$('cam-fps').onchange = () => _streams.forEach(c => c());
$('wrist-on').onchange = () => { $('wrist').closest('figure').hidden = !$('wrist-on').checked; _streams.forEach(c => c()); };
$('wrist').closest('figure').hidden = !$('wrist-on').checked;

// Wrist-roll nudge: a plain joint move, allowed even when the roll sits at its end stop (Cartesian moves are refused there).
async function nudgeRoll(delta) {
  if (!own() || stopped() || hold) return;
  try {
    const {ok, data} = await post('jog', {token, joint: 'wrist_roll', delta}, 2500);
    note(ok ? '' : (data.error || 'Nudge refused.'));
  } catch (_) { note('Nudge request failed (connection).'); }
}
$('roll-minus').onclick = () => nudgeRoll(-10);
$('roll-plus').onclick = () => nudgeRoll(10);
const _render = render;
render = function () { _render(); for (const id of ['roll-minus', 'roll-plus']) $(id).disabled = !own() || stopped(); };

// 3D arm view: drawn locally from the measured joints (the table plane comes from the controller's workspace setting).
const posePreview = PosePreview.create($('pose'));
const _render2 = render;
render = function () {
  _render2();
  if (state?.measured) posePreview.draw(state.measured, state.cartesian?.workspace?.table_z_m ?? 0);
};

// ------------------------------------------------------------ click to move
// Calibrate: put the gripper tip on a spot you can see in the overhead image, click that spot, and the tip's 3D position is stored.
// After 4+ spots, Move mode turns a click into: 25 mm above that point, down, and back up (gripper closed).
let mvp = {available: true, move: {state: 'idle'}, plane: {count: 0, pairs: [], ready: false}}, lastClick = null;
const mode = () => document.querySelector('input[name=mode]:checked').value;
function imgRect() {  // where the picture really is inside the <img> box (object-fit: contain)
  const el = $('overhead'), r = el.getBoundingClientRect(), nw = el.naturalWidth || 4, nh = el.naturalHeight || 3;
  const k = Math.min(r.width / nw, r.height / nh), w = nw * k, h = nh * k;
  return {x: r.left + (r.width - w) / 2, y: r.top + (r.height - h) / 2, w, h, box: r};
}
async function mvpPost(route, body = {}) {
  const r = await fetch('/api/mvp/' + route, {method: 'POST', headers: {'Content-Type': 'application/json'}, body: JSON.stringify(body), signal: AbortSignal.timeout(5000)});
  let data = {}; try { data = await r.json(); } catch (_) {}
  return {ok: r.ok, data};
}
async function mvpPoll() {
  try { const r = await fetch('/api/mvp/status', {cache: 'no-store', signal: AbortSignal.timeout(2500)}); if (r.ok) mvp = await r.json(); } catch (_) {}
  renderMvp(); drawMarks();
}
setInterval(mvpPoll, 700); mvpPoll();
function renderMvp() {
  const p = mvp.plane || {count: 0, pairs: []}, mv = mvp.move || {state: 'idle'}, el = $('mvp-status');
  $('mode-hint').textContent = mode() === 'calibrate'
    ? 'Put the gripper tip on a spot you can see in the camera, hold still, then click that spot in the image. Record 4+ spread-out spots (5+ to see an error check).'
    : 'Click a point on the table: the arm goes 25 mm above it, lowers, then rises. Gripper must be closed.';
  if (mvp.available === false) { el.textContent = 'Click-to-move unavailable: ' + mvp.error; el.className = 'mvp-status error'; }
  else if (mv.state === 'moving') { el.textContent = `Moving: ${mv.step} (${(mv.index ?? 0) + 1}/${mv.total})`; el.className = 'mvp-status moving'; }
  else if (mv.state === 'error') { el.textContent = mv.message; el.className = 'mvp-status error'; }
  else if (mv.state === 'done') { el.textContent = mv.message; el.className = 'mvp-status done'; }
  else { el.textContent = p.ready ? `Calibrated with ${p.count} spots` + (p.checked ? ` · fit error rms ${p.rms_mm.toFixed(1)} mm, worst ${p.max_mm.toFixed(1)} mm` : ' (add a 5th spot to check the fit)') : `${p.count} of at least 4 spots recorded`; el.className = 'mvp-status'; }
  $('cal-list').innerHTML = (p.pairs || []).map((q, i) => `<li>spot ${i + 1}: tip x ${(q.xyz[0] * 1000).toFixed(0)} y ${(q.xyz[1] * 1000).toFixed(0)} z ${(q.xyz[2] * 1000).toFixed(0)} mm${q.residual_mm != null && p.checked ? ' · err ' + q.residual_mm.toFixed(1) + ' mm' : ''}</li>`).join('');
  $('mv-cancel').disabled = mv.state !== 'moving';
}
function drawMarks() {
  const cv = $('marks'), box = cv.getBoundingClientRect(), dpr = Math.min(window.devicePixelRatio || 1, 2);
  cv.width = Math.round(box.width * dpr); cv.height = Math.round(box.height * dpr);
  const g = cv.getContext('2d'); g.setTransform(dpr, 0, 0, dpr, 0, 0); g.clearRect(0, 0, box.width, box.height);
  const r = imgRect(), at = (px, py) => [r.x - box.left + px * r.w, r.y - box.top + py * r.h];
  g.font = '12px sans-serif'; g.textBaseline = 'middle';
  (mvp.plane?.pairs || []).forEach((q, i) => {
    const [x, y] = at(q.px, q.py); g.fillStyle = 'rgba(40,200,90,.9)'; g.strokeStyle = '#000'; g.lineWidth = 2;
    g.beginPath(); g.arc(x, y, 6, 0, 7); g.fill(); g.stroke(); g.fillStyle = '#fff'; g.strokeText(String(i + 1), x + 9, y); g.fillText(String(i + 1), x + 9, y);
  });
  if (lastClick) { const [x, y] = at(lastClick.px, lastClick.py); g.strokeStyle = '#ff9800'; g.lineWidth = 2.5; g.beginPath(); g.moveTo(x - 10, y); g.lineTo(x + 10, y); g.moveTo(x, y - 10); g.lineTo(x, y + 10); g.stroke(); }
}
window.addEventListener('resize', drawMarks);
$('overhead').addEventListener('load', drawMarks);
document.querySelectorAll('input[name=mode]').forEach(el => el.onchange = () => { renderMvp(); });

$('overhead').addEventListener('click', async e => {
  const r = imgRect(), px = (e.clientX - r.x) / r.w, py = (e.clientY - r.y) / r.h;
  if (px < 0 || px > 1 || py < 0 || py > 1) return;
  lastClick = {px, py}; drawMarks();
  try {
    if (mode() === 'calibrate') {
      const {ok, data} = await mvpPost('calpoint', {px, py});
      note(ok ? '' : (data.error || 'Could not record the spot.'));
    } else {
      if (!$('allow-move').checked) { note('Tick "Allow the arm to move when I click" first.'); return; }
      if (!own()) { note('Take control first.'); return; }
      const {ok, data} = await mvpPost('click', {px, py, token});
      note(ok ? '' : (data.error || 'The click was refused.'));
    }
  } catch (_) { note('Request failed (connection).'); }
  mvpPoll();
});
$('cal-undo').onclick = async () => { await mvpPost('calundo'); mvpPoll(); };
$('cal-clear').onclick = async () => { if (confirm('Delete all calibration spots?')) { await mvpPost('calclear'); lastClick = null; mvpPoll(); } };
$('mv-cancel').onclick = async () => { await mvpPost('cancel'); mvpPoll(); };
// STOP and releasing control also cancel any running click-move.
const _stop = emergencyStop; emergencyStop = async function () { mvpPost('cancel').catch(() => {}); return _stop(); };
const _release = release; release = async function () { mvpPost('cancel').catch(() => {}); return _release(); };
$('release').onclick = release; $('estop').onclick = emergencyStop;  // the buttons captured the originals before wrapping

// Up / Down: one request each. The backend computes a joint target for the tip moved straight up or down (keeping tilt/roll),
// refuses to go below the controller's tip floor, and waits for the arm to arrive.
async function nudgeTip(sign) {
  if (!own() || stopped() || hold) return;
  const dz = sign * Number($('nudge-step').value) / 1000;
  try {
    const {ok, data} = await mvpPost('nudge', {dz, token});
    note(ok ? '' : (data.error || 'Move refused.'));
  } catch (_) { note('Request failed (connection).'); }
  mvpPoll();
}
$('nudge-up').onclick = () => nudgeTip(+1);
$('nudge-down').onclick = () => nudgeTip(-1);
const _render3 = render;
render = function () { _render3(); for (const id of ['nudge-up', 'nudge-down']) $(id).disabled = !own() || stopped() || mvp.move?.state === 'moving'; };

// Soft stop: halt the arm where it is, end a held key and cancel a click-move. NOT the emergency stop (that one latches the
// controller): afterwards the control lease is released, so click "Take control" to carry on, remotely.
async function softStop() {
  const t = token;
  cancelHold();
  note('Stopping…');
  await Promise.allSettled([mvpPost('cancel'), t ? post('stop', {token: t}, 2500) : Promise.resolve()]);
  note('Motion stopped; the arm is holding where it is. This was not the emergency stop: click Take control to carry on.');
}
$('softstop').onclick = softStop;
