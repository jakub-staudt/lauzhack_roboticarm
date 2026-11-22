'use strict';

const $ = id => document.getElementById(id);
const SWATCH = {red: '#c8081c', blue: '#0a40c8', yellow: '#e09c09'};
const SNAP_M = 0.035;
const BLOCKS = ['left'];           // only the left block is used

let cfg = null, blocks = [], twin = null, task = {state: 'idle', message: 'Connecting…'};
let mode = 'pp';                 // 'pp' = move a block, 'go' = move the gripper
let src = null, dst = null;      // src: {x, y, color}; dst: {x, y}
let notice = {text: '', kind: '', until: 0};

// ------------------------------------------------------------------ geometry (mirrors simulator/frame.py)
const toWorld = (px, py) => {
  const depth = cfg.cam_pos[2] - cfg.table_z;
  return [cfg.cam_pos[0] + (px - (cfg.cam_size[0] - 1) / 2) / cfg.cam_focal * depth,
          cfg.cam_pos[1] - (py - (cfg.cam_size[1] - 1) / 2) / cfg.cam_focal * depth];
};
const toPixel = (x, y) => {
  const depth = cfg.cam_pos[2] - cfg.table_z;
  return [(x - cfg.cam_pos[0]) / depth * cfg.cam_focal + (cfg.cam_size[0] - 1) / 2,
          -(y - cfg.cam_pos[1]) / depth * cfg.cam_focal + (cfg.cam_size[1] - 1) / 2];
};
const blockCorner = (name, cx, cy) => {                              // block cm (0..20) -> world metres; the block is a rotated, stretched rectangle
  const L = cfg.left, u = cx / cfg.side_cm * L.su_m, v = cy / cfg.side_cm * L.sv_m;
  return [L.origin[0] + v * Math.cos(L.theta) + u * Math.sin(L.theta), L.origin[1] + v * Math.sin(L.theta) - u * Math.cos(L.theta)];
};
function blockAt(x, y, margin = 0) {
  for (const name of BLOCKS) {
    const L = cfg.left, dx = x - L.origin[0], dy = y - L.origin[1];
    const cx = (dx * Math.sin(L.theta) - dy * Math.cos(L.theta)) / L.su_m * cfg.side_cm, cy = (dx * Math.cos(L.theta) + dy * Math.sin(L.theta)) / L.sv_m * cfg.side_cm;
    if (cx >= -margin && cx <= cfg.side_cm + margin && cy >= -margin && cy <= cfg.side_cm + margin) return {name, cx, cy};
  }
  return null;
}
function describe(x, y) {
  const hit = blockAt(x, y, 0.5);
  return hit ? `(${hit.cx.toFixed(1)}, ${hit.cy.toFixed(1)})` : 'outside the block';
}

// ------------------------------------------------------------------ talking to the server
async function api(path, body) {
  const options = body === undefined ? {cache: 'no-store'}
    : {method: 'POST', headers: {'Content-Type': 'application/json'}, body: JSON.stringify(body)};
  const response = await fetch(path, options);
  const data = await response.json().catch(() => ({}));
  if (!response.ok) throw new Error(data.error || `Request failed (${response.status})`);
  return data;
}
function say(text, kind = 'error', ms = 5000) { notice = {text, kind, until: performance.now() + ms}; renderStatus(); }

async function run(path, body) {
  try { task = await api(path, body); notice.until = 0; renderStatus(); }
  catch (error) { say(error.message); }
}

// ------------------------------------------------------------------ camera
const cam = $('cam');
function nextFrame() { cam.src = '/frame.png?t=' + Date.now(); }
cam.addEventListener('load', () => setTimeout(nextFrame, 60));
cam.addEventListener('error', () => setTimeout(nextFrame, 1000));

// ------------------------------------------------------------------ overlay
const overlay = $('overlay'), ctx = overlay.getContext('2d');
function sizeOverlay() {
  const rect = overlay.getBoundingClientRect(), ratio = window.devicePixelRatio || 1;
  overlay.width = Math.round(rect.width * ratio); overlay.height = Math.round(rect.height * ratio);
  draw();
}
new ResizeObserver(sizeOverlay).observe(overlay);

function label(text, x, y, color) {
  ctx.font = `600 ${12 * ctx.scale_}px system-ui, sans-serif`;
  const w = ctx.measureText(text).width + 10 * ctx.scale_, h = 18 * ctx.scale_;
  ctx.fillStyle = 'rgba(15,20,30,.78)'; ctx.fillRect(x, y - h, w, h);
  ctx.fillStyle = color; ctx.fillText(text, x + 5 * ctx.scale_, y - 5 * ctx.scale_);
}
function ring(x, y, color, text) {
  const [px, py] = toPixel(x, y), s = ctx.scale_;
  ctx.lineWidth = 3 * s; ctx.strokeStyle = color; ctx.beginPath(); ctx.arc(px * ctx.k, py * ctx.k, 18 * s, 0, 7); ctx.stroke();
  ctx.fillStyle = color; ctx.beginPath(); ctx.arc(px * ctx.k, py * ctx.k, 3.5 * s, 0, 7); ctx.fill();
  if (text) label(text, px * ctx.k + 22 * s, py * ctx.k - 8 * s, '#fff');
}
function draw() {
  if (!cfg) return;
  ctx.clearRect(0, 0, overlay.width, overlay.height);
  ctx.k = overlay.width / cfg.cam_size[0];                        // image pixels -> canvas pixels
  ctx.scale_ = (window.devicePixelRatio || 1) * 1.05;                // labels and rings keep a constant on-screen size
  const s = ctx.scale_, k = ctx.k;

  for (const name of BLOCKS) {                                    // block outline and corner labels
    const corners = [[0, 0], [20, 0], [20, 20], [0, 20]].map(([cx, cy]) => toPixel(...blockCorner(name, cx, cy)));
    ctx.lineWidth = 2 * s; ctx.strokeStyle = 'rgba(25,70,190,.9)'; ctx.setLineDash([8 * s, 5 * s]);
    ctx.beginPath(); corners.forEach(([x, y], i) => (i ? ctx.lineTo(x * k, y * k) : ctx.moveTo(x * k, y * k))); ctx.closePath(); ctx.stroke();
    ctx.setLineDash([]);
    const [ox, oy] = corners[0], [fx, fy] = corners[2];
    label('(0, 0)', ox * k + 2 * s, oy * k + 22 * s, '#9fc0ff');
    label('(20, 20)', fx * k - 66 * s, fy * k - 2 * s, '#9fc0ff');
    // spots the arm cannot reach with a top-down grasp
    if (cfg.reach) {
      const ok = new Set(cfg.reach.blocks[name].map(p => p.join(',')));
      ctx.fillStyle = 'rgba(200,40,50,.55)';
      for (let cy = 0; cy <= 20; cy += cfg.reach.step_cm) for (let cx = 0; cx <= 20; cx += cfg.reach.step_cm) {
        if (ok.has(`${cx},${cy}`)) continue;
        const [px, py] = toPixel(...blockCorner(name, cx, cy));
        ctx.beginPath(); ctx.arc(px * k, py * k, 2.6 * s, 0, 7); ctx.fill();
      }
    }
  }

  if (src && dst && mode === 'pp') {                              // arrow from block to destination
    const [a, b] = [toPixel(src.x, src.y), toPixel(dst.x, dst.y)];
    ctx.strokeStyle = 'rgba(255,255,255,.85)'; ctx.lineWidth = 2 * s; ctx.setLineDash([6 * s, 6 * s]);
    ctx.beginPath(); ctx.moveTo(a[0] * k, a[1] * k); ctx.lineTo(b[0] * k, b[1] * k); ctx.stroke(); ctx.setLineDash([]);
  }
  if (src && mode === 'pp') ring(src.x, src.y, '#2fd27a', describe(src.x, src.y));
  if (dst) ring(dst.x, dst.y, '#ffa733', describe(dst.x, dst.y));
}

// ------------------------------------------------------------------ taps
function tapWorld(event) {
  const rect = overlay.getBoundingClientRect();
  return toWorld((event.clientX - rect.left) / rect.width * cfg.cam_size[0], (event.clientY - rect.top) / rect.height * cfg.cam_size[1]);
}
let downAt = null;
overlay.addEventListener('pointerdown', e => { downAt = [e.clientX, e.clientY]; });
overlay.addEventListener('pointermove', e => {
  if (!cfg) return;
  const [x, y] = tapWorld(e);
  $('hover').textContent = describe(x, y);
});
overlay.addEventListener('pointerleave', () => { $('hover').textContent = 'Tap the picture'; });
overlay.addEventListener('pointerup', e => {
  if (!cfg || !downAt || Math.hypot(e.clientX - downAt[0], e.clientY - downAt[1]) > 10) return;
  onTap(...tapWorld(e));
});

function onTap(x, y) {
  if (mode === 'go') {
    dst = {x, y}; draw(); updatePicks();
    if (!$('confirm').checked) execute();
    return;
  }
  if (!src || (src && dst)) {                                     // choose the block (starting over if both were set)
    const near = blocks.filter(b => !b.held).sort((a, b) => Math.hypot(a.x - x, a.y - y) - Math.hypot(b.x - x, b.y - y))[0];
    if (!near || Math.hypot(near.x - x, near.y - y) > SNAP_M) { say('No block there. Tap one of the coloured blocks.', 'error', 3000); return; }
    src = {x: near.x, y: near.y, color: near.color}; dst = null;
  } else {                                                        // choose the destination
    if (!blockAt(x, y, 1)) { say('Tap the destination inside the block.', 'error', 3500); return; }
    dst = {x, y};
    if (!$('confirm').checked) { draw(); updatePicks(); execute(); return; }
  }
  draw(); updatePicks();
}

function execute() {
  if (task.state === 'running' || task.state === 'planning') { say('The arm is busy. Press Stop first.'); return; }
  if (mode === 'go') { if (dst) run('/api/go_to', {at: toPixel(dst.x, dst.y)}); }
  else if (src && dst) run('/api/pick_place', {src: toPixel(src.x, src.y), dst: toPixel(dst.x, dst.y)});
}

// ------------------------------------------------------------------ controls
function setMode(next) {
  mode = next; src = dst = null;
  $('mode-pp').setAttribute('aria-selected', next === 'pp'); $('mode-go').setAttribute('aria-selected', next === 'go');
  $('hint').textContent = next === 'pp' ? 'Tap a block, then tap where it should go.' : 'Tap a spot and the gripper moves over it.';
  $('src-box').hidden = next === 'go';
  draw(); updatePicks();
}
$('mode-pp').onclick = () => setMode('pp');
$('mode-go').onclick = () => setMode('go');
$('go').onclick = execute;
$('clear').onclick = () => { src = dst = null; draw(); updatePicks(); };
$('stop').onclick = () => run('/api/stop', {});
$('home').onclick = () => run('/api/home', {});
$('park').onclick = () => run('/api/park', {});
$('reset').onclick = () => { src = dst = null; draw(); updatePicks(); run('/api/reset_blocks', {}); };
document.addEventListener('keydown', e => { if (e.key === 'Escape') run('/api/stop', {}); });

function updatePicks() {
  $('src-text').textContent = src ? `${src.color} at ${describe(src.x, src.y)}` : 'not chosen';
  $('dst-text').textContent = dst ? describe(dst.x, dst.y) : 'not chosen';
  const busy = task.state === 'running' || task.state === 'planning';
  const twinOn = Boolean(twin && twin.available && twin.enabled);       // a digital twin cannot move on its own
  $('go').disabled = busy || twinOn || !dst || (mode === 'pp' && !src);
  for (const id of ['home', 'park', 'reset']) $(id).disabled = twinOn;
}

function renderStatus() {
  const chip = $('status');
  if (notice.until > performance.now()) { chip.textContent = notice.text; chip.dataset.state = notice.kind || 'error'; }
  else {
    const progress = task.state === 'running' && task.total ? ` · ${task.label} ${Math.round(100 * task.step / task.total)}%` : '';
    chip.textContent = (task.message || '') + progress; chip.dataset.state = task.state || 'idle';
  }
  updatePicks();
}

function renderBlocks() {
  $('blocks').replaceChildren(...blocks.map(b => {
    const li = document.createElement('li');
    const swatch = document.createElement('span'); swatch.className = 'swatch'; swatch.style.background = SWATCH[b.color];
    li.append(swatch, `${b.color[0].toUpperCase() + b.color.slice(1)}: ${b.held ? 'in the gripper' : b.label}`);
    return li;
  }));
}

async function poll() {
  try {
    const state = await api('/api/state');
    task = state.task; blocks = state.blocks; twin = state.twin; renderTwin(); renderStatus(); renderBlocks(); draw();
  } catch (_) { task = {state: 'error', message: 'Lost connection to the simulator server.'}; renderStatus(); }
  setTimeout(poll, 300);
}

(async () => {
  cfg = await api('/api/config');
  nextFrame(); sizeOverlay(); poll();
})();


function renderTwin() {
  const box = $('twin');
  if (!twin || !twin.available) { box.hidden = true; return; }
  box.hidden = false;
  const real = twin.real || {};
  const bad = !twin.ok || real.connected === false;
  box.dataset.tone = bad ? 'bad' : 'ok';
  $('twin-text').textContent = !twin.ok
    ? `The real arm is not answering${twin.age_s != null ? ` (last seen ${twin.age_s} s ago)` : ''}; the simulated arm holds its last pose.`
    : `Mirroring the real arm, ${twin.age_s} s old${twin.max_diff_deg != null ? `, joints within ${twin.max_diff_deg}°` : ''}${real.owner ? `, controlled by ${real.owner}` : ''}${real.emergency_stop ? ', EMERGENCY STOP latched' : ''}. This view cannot move on its own.`;
}
