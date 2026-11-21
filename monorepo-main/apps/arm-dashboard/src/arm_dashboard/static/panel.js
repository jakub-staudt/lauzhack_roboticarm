'use strict';
const $ = id => document.getElementById(id);
let state = null, online = false, token = null, ownerId = null, selected = null;
let primary = 'overhead', mobileView = 'overhead', claimPending = false, heartbeatRequest = null;
let builtSignature = '', dragging = new Set(), viewerURL = '', cameraSignature = '';
let safetyPending = '', safetyFeedback = '', safetyActionEpoch = 0, controlRequestEpoch = 0;
let manualMode = 'cartesian', cartesianSequence = 0, cartesianHold = null, cartesianPending = false;
let previewFps = 2, previewOverride = false, previewDeliveryInitialized = false;
let noticeScope = '';
let snapshotPreview = false;
const cameraJobs = new Map();
const compact = matchMedia('(max-width: 760px), (max-width: 1000px) and (max-height: 500px)');
const title = name => String(name).replaceAll('_', ' ').replace(/^./, c => c.toUpperCase());
const fmt = value => Number.isFinite(value) ? value.toFixed(1) : '—';
const torqueText = value => value === true ? 'on' : value === false ? 'off' : 'unknown';
const unit = name => name === state?.gripper ? '%' : '°';
const own = () => Boolean(token && online && state?.control?.owner_id === ownerId);
const healthy = () => Boolean(online && state && state.connected !== false && state.loop_ok !== false && Number.isFinite(state.age_s) && state.age_s <= 1.5 && !(state.faults || []).length);
const stopStateAvailable = () => typeof state?.emergency_stop?.latched === 'boolean' && Number.isInteger(state.emergency_stop.generation);
function supportsCommands(...routes) {
  const commands = state?.operator_commands;
  return Array.isArray(commands) && routes.every(route => commands.includes('/api/' + route));
}
const leaseCommandsAvailable = () => supportsCommands('control/claim', 'control/heartbeat', 'control/release', 'emergency-stop');
const cartesianCommandsAvailable = () => supportsCommands('cartesian/start', 'cartesian/renew', 'cartesian/stop');
const movable = () => own() && healthy() && leaseCommandsAvailable() && stopStateAvailable() && !state.emergency_stop.latched && !safetyPending && state.torque;
function notice(message = '', scope = 'general') {
  const text = String(message), host = $('notice'), body = $('notice-message');
  noticeScope = text ? scope : '';
  if (body.textContent !== text) body.textContent = text;
  if (host.hidden !== !text) host.hidden = !text;
}
function clearNoticeForAction(scope) {
  if (noticeScope === scope || noticeScope === 'general' || (scope === 'control' && noticeScope === 'connection')) notice();
}
$('notice-dismiss').onclick = () => notice();
function loseControl() { cancelCartesian(); token = null; ownerId = null; enable(); }
function markOffline() {
  for (const slot of ['primary','secondary','observer']) { setText(slot + '-status','Connection lost'); const overlay = $(slot + '-media').querySelector('.camera-stale'); if (overlay) { overlay.hidden = $(slot + '-image').hidden; overlay.textContent = 'Connection lost · last frame'; } }
  setText('viewer-status','Stale · server unavailable'); $('viewer-status').classList.add('bad');
}
async function post(route, data = {}, useToken = false) {
  const scope = route === 'cameras' ? 'camera' : route.startsWith('control/') || route.startsWith('emergency-stop') ? 'control' : useToken ? 'motion' : 'general';
  if (route !== 'emergency-stop' && !supportsCommands(route)) {
    notice('This command is unavailable through the dashboard.',scope);
    return null;
  }
  if (useToken && !own()) { notice('Take control before changing the arm.','control'); return null; }
  if (route !== 'control/heartbeat' && route !== 'view') clearNoticeForAction(scope);
  const requestToken = useToken ? token : data.token, requestEpoch = controlRequestEpoch;
  const stale = () => (requestToken != null && requestToken !== token) || ((requestToken != null || route === 'control/claim') && requestEpoch !== controlRequestEpoch);
  try {
    const response = await fetch('/api/' + route, {method:'POST', headers:{'Content-Type':'application/json'}, body:JSON.stringify(useToken ? {...data, token} : data), signal:AbortSignal.timeout(1800)});
    const result = await response.json();
    if (stale()) return null;
    if (!response.ok) {
      notice(result.error || 'The request was not accepted. Check diagnostics.',scope);
      if (response.status === 401 || response.status === 403 || (response.status === 409 && useToken)) loseControl();
      return null;
    }
    return result;
  } catch (_) {
    if (stale()) return null;
    online = false; loseControl(); markOffline(); notice('Connection lost. Motion controls are disabled. Reconnect and take control again.','connection'); return null;
  }
}
function setText(id, text) { $(id).textContent = text; }
function renderEmergency() {
  const status = $('emergency-status'), reset = $('emergency-reset');
  if (safetyPending) { status.textContent = safetyPending === 'stop' ? 'Requesting software stop...' : 'Resetting stop latch...'; status.dataset.state = 'pending'; reset.disabled = true; return; }
  if (!online) { status.textContent = 'Controller unreachable | stop status unconfirmed'; status.dataset.state = 'unknown'; reset.hidden = !state?.emergency_stop?.latched; reset.disabled = true; return; }
  if (!stopStateAvailable()) { status.textContent = 'Stop latch unavailable | motion locked'; status.dataset.state = 'unknown'; reset.hidden = true; reset.disabled = true; return; }
  const stop = state.emergency_stop;
  if (stop.latched) {
    status.dataset.state = 'latched';
    status.textContent = stop.hold_error ? 'Software stop latched | hold error: ' + stop.hold_error : stop.hold_status === 'sent' ? 'Software stop latched | hold command sent' : 'Software stop latched | hold status pending';
    reset.hidden = false; reset.disabled = !supportsCommands('emergency-stop/reset'); return;
  }
  status.dataset.state = 'clear'; status.textContent = safetyFeedback || 'Software stop clear | controller state available'; reset.hidden = true; reset.disabled = true;
}
function enable() {
  renderEmergency();
  const stopAvailable = stopStateAvailable(), stopped = stopAvailable && state.emergency_stop.latched;
  document.querySelectorAll('.motion').forEach(el => el.disabled = !movable() || !supportsCommands(el.dataset.command));
  renderCartesian();
  const ownerActionsReady = own() && healthy() && leaseCommandsAvailable() && stopAvailable && !stopped && !safetyPending;
  document.querySelectorAll('.owner-action').forEach(el => el.disabled = !ownerActionsReady || !supportsCommands(el.dataset.command));
  $('claim').disabled = !healthy() || !leaseCommandsAvailable() || !stopAvailable || stopped || Boolean(safetyPending) || claimPending || Boolean(state?.control?.owner_id && !own());
  $('claim').hidden = own(); $('release').hidden = !own();
  const owner = state?.control;
  const controlMessage = stopped ? 'Software stop latched. Reset before taking control.' : !stopAvailable ? 'Stop status unavailable. Motion is locked.' : online && !leaseCommandsAvailable() ? 'Control commands are unavailable through this dashboard.' : own() ? state.torque ? 'Manual movement is enabled.' : 'Enable torque in Session to move.' : owner?.owner_id ? (owner.label || 'Another browser') + ' owns the arm.' : healthy() ? 'Take control to move the arm.' : 'Connect the arm to enable manual controls.';
  setText('owner', stopped ? 'Software stop latched' : !stopAvailable ? 'Stop status unavailable' : own() ? 'You have control' : owner?.owner_id ? 'Controlled by ' + (owner.label || 'another session') : healthy() && leaseCommandsAvailable() ? 'Read only | control available' : 'Read only | arm unavailable');
  setText('control-title', stopped ? 'Software stop active' : own() ? 'You have control' : owner?.owner_id ? 'Another session has control' : 'Observe the workspace');
  setText('control-copy', controlMessage);
  setText('camera-help', own() ? 'Camera changes apply to everyone viewing this arm.' : state?.preview_only ? 'Camera assignments are fixed in disconnected preview.' : 'Take control to change camera assignment.');
  $('link').className = 'status' + (healthy() ? ' live' : online ? ' bad' : '');
  setText('link', !online ? 'Server unavailable' : state?.connected === false ? 'Arm disconnected' : (state?.loop_ok === false || !Number.isFinite(state?.age_s) || state.age_s > 1.5) ? 'Robot feedback stale' : state?.faults?.length ? 'Arm needs attention' : 'Arm connected');
}
function bounds(name) { const limits = state?.limits?.[name]; return Array.isArray(limits) ? limits : [0, 100]; }
function bounded(name, value) { const [low, high] = bounds(name); return Math.max(low, Math.min(high, value)); }
function makeControl(name) {
  const root = document.createElement('div'); root.className = 'servo-control'; root.dataset.joint = name;
  root.innerHTML = '<div class="servo-head"><button type="button" class="servo-name" aria-pressed="false"></button><output class="measured">-</output></div><div class="servo-row"><button class="motion minus" disabled></button><input class="motion target-range" type="range" step="0.5" disabled><button class="motion plus" disabled></button></div>';
  const selectButton = root.querySelector('.servo-name'); selectButton.textContent = title(name); selectButton.onclick = () => selectJoint(name);
  const slider = root.querySelector('input'), [low, high] = bounds(name);
  slider.dataset.command = 'target';
  for (const selector of ['.minus','.plus']) root.querySelector(selector).dataset.command = 'jog';
  slider.min = low; slider.max = high; slider.setAttribute('aria-label', title(name) + ' target');
  const step = () => name === state.gripper ? 10 : Number($('step').value);
  const updateLabels = () => { root.querySelector('.minus').textContent = '−' + step() + unit(name); root.querySelector('.plus').textContent = '+' + step() + unit(name); };
  updateLabels();
  root.querySelector('.minus').onclick = () => jog(name, -step()); root.querySelector('.plus').onclick = () => jog(name, step());
  slider.addEventListener('input', () => dragging.add(name));
  slider.addEventListener('change', () => { dragging.delete(name); if (movable()) post('target', {joint:name, value:bounded(name, Number(slider.value))}, true); });
  slider.addEventListener('pointercancel', () => dragging.delete(name));
  slider.addEventListener('blur', () => dragging.delete(name));
  if (name === state.gripper) {
    root.classList.add('gripper-control');
    root.querySelector('.servo-row').querySelector('.minus').hidden = true;
    root.querySelector('.servo-row').querySelector('.plus').hidden = true;
    const row = document.createElement('div'); row.className = 'jog-row gripper-actions';
    for (const [text, value] of [['Close', low], ['Open', high]]) { const b = document.createElement('button'); b.className = 'motion'; b.dataset.command = 'target'; b.textContent = text; b.disabled = true; b.onclick = () => movable() && post('target', {joint:name, value}, true); row.append(b); }
    root.append(row);
  }
  return root;
}
function jog(name, delta) {
  if (!movable()) return;
  const current = Number(state.target?.[name]);
  if (!Number.isFinite(current)) return;
  const adjusted = bounded(name, current + delta) - current;
  if (adjusted) post('jog', {joint:name, delta:adjusted}, true);
}
function selectJoint(name, send = true) {
  selected = name;
  document.querySelectorAll('[data-joint]').forEach(root => root.querySelector('.servo-name').setAttribute('aria-pressed', String(root.dataset.joint === name)));
  if (send && supportsCommands('view')) post('view', {joint:name});
  enable();
}
function build(s) {
  const names = s.joints || [];
  $('joint-list').replaceChildren(...names.map(name => makeControl(name)));
  if (names.length) {
    const initial = names.includes(selected) ? selected : names.find(name => name !== s.gripper) || names[0];
    selectJoint(initial, false);
  }
}
function updateControls() {
  document.querySelectorAll('[data-joint]').forEach(root => {
    const name = root.dataset.joint, suffix = unit(name);
    root.querySelector('.measured').textContent = fmt(state.measured?.[name]) + suffix;
    if (!dragging.has(name)) { const value = state.target?.[name]; if (Number.isFinite(value)) root.querySelector('input').value = value; }
  });
}
function cameraDelivery() {
  const delivery = state?.camera_delivery;
  if (!Number.isFinite(delivery?.max_fps) || !Array.isArray(delivery.available_fps)) return null;
  const rates = delivery.available_fps.filter(fps => [2,10,15].includes(fps) && fps <= delivery.max_fps);
  return rates.includes(2) && delivery.default_fps === 2 ? {...delivery,rates} : null;
}
function renderCameraDelivery() {
  const delivery = cameraDelivery(), select = $('video-preview');
  $('video-preview-setting').hidden = !delivery; select.disabled = !delivery;
  if (!delivery) return;
  if (!previewDeliveryInitialized) {
    const directHost = Array.isArray(delivery.direct_hosts) && delivery.direct_hosts.some(host => typeof host === 'string' && host.toLowerCase() === location.hostname.toLowerCase());
    if (directHost && delivery.rates.includes(delivery.direct_default_fps)) {
      previewFps = delivery.direct_default_fps; previewOverride = true;
    }
    previewDeliveryInitialized = true;
  }
  if (!delivery.rates.includes(previewFps)) { previewFps = delivery.default_fps; previewOverride = false; }
  for (const option of Array.from(select.options)) option.disabled = !delivery.rates.includes(Number(option.value));
  select.value = String(previewFps);
}
function cameraDisplayed(slot) {
  const bench = document.querySelector('.workbench');
  if (bench.classList.contains('viewer-enlarged') && slot !== 'observer') return false;
  if (compact.matches) return mobileView === ({primary:'overhead',secondary:'wrist',observer:'observer'})[slot];
  return slot === 'primary' || !bench.classList.contains('enlarged');
}
function updateCameras() {
  updateCamera('primary',primary); updateCamera('secondary',primary === 'overhead' ? 'wrist' : 'overhead'); updateCamera('observer','observer');
}
// Finite requests release HTTP/1 slots when streaming previews starve state.
function stopCamera(slot) {
  const job = cameraJobs.get(slot);
  cameraJobs.delete(slot);
  if (job) { clearTimeout(job.timer); clearTimeout(job.deadline); job.controller?.abort(); if (job.objectURL) URL.revokeObjectURL(job.objectURL); }
  const img = $(slot + '-image'); img.removeAttribute('src'); delete img.dataset.feed;
}
function suspendMedia() {
  for (const slot of ['primary','secondary','observer']) stopCamera(slot);
  $('viewer').removeAttribute('src'); viewerURL = '';
}
function renderSnapshotFailure(slot) {
  if (!cameraJobs.get(slot)?.failed) return;
  const message = 'Preview interrupted \u00b7 retrying';
  setText(slot + '-status',message);
  const overlay = $(slot + '-media').querySelector('.camera-stale');
  if (overlay) { overlay.hidden = false; overlay.textContent = message; }
}
function startSnapshots(slot, url) {
  const existing = cameraJobs.get(slot);
  if (existing?.url === url && existing.fps === previewFps) return;
  stopCamera(slot);
  const img = $(slot + '-image'), parsed = new URL(url,window.location.href);
  parsed.pathname = parsed.pathname.replace(/\.mjpg$/, '.jpg');
  // Blob fetches require CORS: use the same dashboard for localhost snapshots.
  if (['localhost','127.0.0.1'].includes(location.hostname) && parsed.port === '8781') parsed.port = new URL(window.location.href).port;
  const job = {url,fps:previewFps,timer:null,deadline:null,controller:null,objectURL:null,failed:false};
  cameraJobs.set(slot,job); img.dataset.feed = url;
  const current = () => cameraJobs.get(slot) === job && online && !document.hidden;
  async function frame() {
    if (!current()) return;
    const started = Date.now(); let failed = false;
    job.controller = new AbortController();
    job.deadline = setTimeout(() => job.controller?.abort(),1200);
    try {
      const response = await fetch(parsed.href,{cache:'no-store',signal:job.controller.signal});
      if (!response.ok) throw new Error('Camera snapshot unavailable');
      const blob = await response.blob();
      if (!current()) return;
      const nextURL = URL.createObjectURL(blob), previousURL = job.objectURL;
      job.objectURL = nextURL; img.src = nextURL; img.hidden = false; img.nextElementSibling.hidden = true;
      if (previousURL) URL.revokeObjectURL(previousURL);
      job.failed = false; updateCameras();
    } catch (_) { failed = true; if (current()) { job.failed = true; renderSnapshotFailure(slot); } }
    finally {
      clearTimeout(job.deadline); job.deadline = null; job.controller = null;
      if (current()) job.timer = setTimeout(frame,failed ? 1000 : Math.max(0,1000 / job.fps - (Date.now() - started)));
    }
  }
  frame();
}
function updateCamera(slot, name) {
  const meta = state?.cameras?.[name] || {status:'unconfigured'};
  const img = $(slot + '-image'), empty = img.nextElementSibling;
  let overlay = $(slot + '-media').querySelector('.camera-stale');
  if (!overlay) { overlay = document.createElement('span'); overlay.className = 'camera-stale'; overlay.hidden = true; $(slot + '-media').append(overlay); }
  overlay.hidden = meta.status !== 'stale'; overlay.textContent = 'Stale · last frame' + (Number.isFinite(meta.age_s) ? ' · ' + fmt(meta.age_s) + ' s old' : '');
  setText(slot + '-title', title(name)); img.alt = title(name) + ' camera feed';
  const status = meta.status || 'unconfigured';
  const delivery = cameraDelivery();
  setText(slot + '-status', status === 'live' ? delivery ? `Live · target ${previewFps} fps` : `Live${meta.fps ? ' · capture ' + Number(meta.fps).toFixed(0) + ' fps' : ''}` : status === 'unconfigured' ? 'Unassigned' : title(status));
  $(slot + '-status').title = [meta.fps ? `Capture: ${Number(meta.fps).toFixed(0)} fps.` : '',delivery ? `Preview target: ${previewFps} fps; delivered rate is not measured.` : ''].filter(Boolean).join(' ');
  const cameraPath = meta.url || `/cameras/${name}.mjpg`;
  const localCameraRelay = state?.viewer?.proxy_url && ['localhost','127.0.0.1'].includes(location.hostname);
  let url = localCameraRelay && cameraPath.startsWith('/') ? location.protocol + '//' + location.hostname + ':8781' + cameraPath : cameraPath;
  if (delivery && previewOverride) {
    const parsed = new URL(url,window.location.href); parsed.searchParams.set('fps',String(previewFps));
    url = url.startsWith('/') ? parsed.pathname + parsed.search + parsed.hash : parsed.href;
  }
  const show = online && !document.hidden && (status === 'live' || status === 'stale') && cameraDisplayed(slot);
  img.hidden = !show; empty.hidden = show;
  if (show && snapshotPreview) startSnapshots(slot,url);
  else if (show && img.dataset.feed !== url) { stopCamera(slot); img.dataset.feed = url; img.src = url; }
  if (!show) stopCamera(slot);
  if (show && snapshotPreview) renderSnapshotFailure(slot);
  setText(slot + '-empty-title', status === 'unconfigured' ? `Set up your ${name} view` : `${title(name)} camera ${status}`);
  setText(slot + '-empty-copy', meta.error || (status === 'unconfigured' ? 'Assign a camera in Session to see the workspace.' : status === 'connecting' ? 'Opening the camera stream.' : 'Check its connection and assignment in Session.'));
}
function resizeViewer() {
  const frame = $('viewer'), host = frame.parentElement;
  if (frame.hidden || !host.clientWidth || !host.clientHeight) return;
  const renderWidth = Math.max(640, host.clientWidth), scale = host.clientWidth / renderWidth;
  frame.style.width = renderWidth + 'px';
  frame.style.height = (host.clientHeight / scale) + 'px';
  frame.style.transform = 'scale(' + scale + ')';
}
const viewerResizeObserver = new ResizeObserver(resizeViewer);
viewerResizeObserver.observe($('viewer').parentElement);
function updateViewer() {
  const meta = state?.viewer;
  setText('viewer-status', state?.preview_only ? 'Waiting for arm' : meta?.available ? meta.stale ? 'Stale · last pose' : 'Live · read only' : 'Read only');
  $('viewer-status').classList.toggle('bad', Boolean(meta?.available && meta.stale));
  $('viewer-status').title = meta?.feed || '';
  const eligible = online && !document.hidden && (!compact.matches || mobileView === 'twin');
  if (!eligible) { if ($('viewer').hasAttribute('src')) $('viewer').removeAttribute('src'); viewerURL = ''; $('viewer').hidden = true; $('viewer-empty').hidden = false; return; }
  if (meta?.available && meta.proxy_url) {
    const url = new URL(meta.proxy_url, window.location.href);
    url.search = '?darkMode'; url.hash = '';
    if (viewerURL !== url.href) { viewerURL = url.href; $('viewer').src = viewerURL; }
    $('viewer').hidden = false; $('viewer-empty').hidden = true; resizeViewer();
  } else { $('viewer').hidden = true; $('viewer-empty').hidden = false; setText('viewer-message', meta?.error || 'Start the read-only viewer to see the arm.'); }
}
function renderDiagnostics() {
  const s = state;
  setText('diagnostic-summary', `${fmt(s.loop_hz)} Hz · torque ${torqueText(s.torque)} · ${(s.faults || []).length} faults`);
  if (!$('diagnostics').open) return;
  const entries = [['Robot',s.robot || 'SO-101'],['Port',s.port || '—'],['Loop',fmt(s.loop_hz) + ' Hz'],['State age',fmt(s.age_s) + ' s'],['Sequence',s.sequence ?? '—'],['Torque',title(torqueText(s.torque))],['Clipping',s.clip || 'None'],['Updated',new Date().toLocaleTimeString()]];
  $('health').replaceChildren(...entries.flatMap(([k,v]) => { const dt = document.createElement('dt'), dd = document.createElement('dd'); dt.textContent = k; dd.textContent = v; return [dt,dd]; }));
  $('fault-list').replaceChildren(...((s.faults || []).length ? s.faults : ['No reported faults']).map(message => {const li = document.createElement('li'); li.textContent = message; return li;}));
  $('tracking').replaceChildren(...(s.joints || []).map(name => { const tr = document.createElement('tr'); const measured = s.measured?.[name], target = s.target?.[name]; for (const val of [title(name),fmt(measured) + unit(name),fmt(target) + unit(name),fmt(Number.isFinite(measured) && Number.isFinite(target) ? target - measured : null) + unit(name)]) {const td = document.createElement('td'); td.textContent = val; tr.append(td);} return tr; }));
  $('events').replaceChildren(...((s.events || []).length ? s.events.slice(-40).reverse() : [{message:'No activity reported.'}]).map(event => {const li = document.createElement('li'); li.className = 'event-' + (event.level || 'info'); if (event.timestamp) {const time = document.createElement('time'); time.dateTime = new Date(event.timestamp * 1000).toISOString(); time.textContent = new Date(event.timestamp * 1000).toLocaleTimeString(); li.append(time);} li.append(document.createTextNode(event.message || '')); return li;}));
}
function render() {
  const signature = JSON.stringify([state.joints,state.gripper,state.limits]);
  if (signature !== builtSignature) { builtSignature = signature; build(state); }
  renderCameraDelivery(); updateControls(); updateCameras(); updateViewer(); renderDiagnostics();
  setText('torque-badge', 'Torque ' + torqueText(state.torque)); setText('torque', typeof state.torque === 'boolean' ? state.torque ? 'Turn torque off' : 'Turn torque on' : 'Torque unavailable');
  setText('session-info', `${state.robot || 'SO-101'} · ${state.port || 'No serial port'} · ${fmt(state.loop_hz)} Hz`);
  if (document.activeElement !== $('speed')) $('speed').value = String(state.speed);
  const cams = JSON.stringify([state.cameras?.overhead?.source,state.cameras?.wrist?.source,state.cameras?.observer?.source]);
  if (cams !== cameraSignature && !$('session').contains(document.activeElement)) {cameraSignature = cams; for (const name of ['overhead','wrist','observer']) {const source = state.cameras?.[name]?.source; const select = $('camera-' + name); if (source != null && !Array.from(select.options).some(o => o.value === String(source))) select.add(new Option(String(source), String(source))); select.value = source == null ? '' : String(source);}}
  enable();
}
async function poll() {
  const request = controlRequestEpoch;
  let next;
  try {
    const response = await fetch('/api/state', {cache:'no-store', signal:AbortSignal.timeout(1200)});
    if (!response.ok) throw new Error('State unavailable');
    next = await response.json();
  } catch (_) {
    snapshotPreview = true; suspendMedia();
    if (request === controlRequestEpoch) { online = false; loseControl(); markOffline(); enable(); }
    setTimeout(poll,250); return;
  }
  if (request === controlRequestEpoch) {
    try {
      const endedReason = token && next.control?.owner_id !== ownerId ? stoppedReasonForHold(next.cartesian,cartesianHold) : '';
      state = next; online = true;
      if (token && state.control?.owner_id !== ownerId) loseControl();
      else if (token && !leaseCommandsAvailable()) release();
      if (endedReason) notice(endedReason,'motion');
      render();
    } catch (_) {
      // Rendering errors fail closed without changing the camera transport.
      online = false; loseControl(); suspendMedia(); markOffline(); enable();
    }
  }
  setTimeout(poll,250);
}
$('claim').onclick = async () => {
  if (!healthy() || !leaseCommandsAvailable() || !stopStateAvailable() || state.emergency_stop.latched || safetyPending || claimPending || document.hidden) return;
  const request = ++controlRequestEpoch, generation = state.emergency_stop.generation;
  claimPending = true; enable();
  const result = await post('control/claim', {label:compact.matches ? 'Phone browser' : 'Laptop browser', emergency_stop_generation:generation});
  if (request !== controlRequestEpoch) return;
  if (result?.token) { token = result.token; ownerId = result.owner_id; if (document.hidden) release(); else { if (state) state.control = {owner_id:ownerId,label:'This browser'}; heartbeat(); } }
  claimPending = false; enable();
};
function release() {
  controlRequestEpoch++; claimPending = false;
  const old = token; loseControl();
  if (old) fetch('/api/control/release', {method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({token:old}),keepalive:true}).catch(() => {});
}
async function heartbeat() {
  if (!own() || document.hidden) return;
  if (heartbeatRequest?.token === token && heartbeatRequest.epoch === controlRequestEpoch) return;
  const request = {token,epoch:controlRequestEpoch};
  heartbeatRequest = request;
  try { await post('control/heartbeat', {token:request.token}); }
  finally { if (heartbeatRequest === request) heartbeatRequest = null; }
}
setInterval(heartbeat, 400);
$('release').onclick = release;
$('step').addEventListener('change', () => document.querySelectorAll('[data-joint]').forEach(root => {
  const name = root.dataset.joint, step = name === state?.gripper ? 10 : Number($('step').value);
  root.querySelector('.minus').textContent = '−' + step + unit(name);
  root.querySelector('.plus').textContent = '+' + step + unit(name);
}));
function selectMode(mode) {
  cancelCartesian();
  for (const name of ['manual','model']) { const active = name === mode; $(name).hidden = !active; $(name + '-tab').setAttribute('aria-selected', String(active)); $(name + '-tab').tabIndex = active ? 0 : -1; }
}
for (const mode of ['manual','model']) {
  $(mode + '-tab').onclick = () => selectMode(mode);
  $(mode + '-tab').addEventListener('keydown', event => {
    if (['ArrowLeft','ArrowRight','Home','End'].includes(event.key)) { event.preventDefault(); const next = event.key === 'Home' ? 'manual' : event.key === 'End' ? 'model' : mode === 'manual' ? 'model' : 'manual'; selectMode(next); $(next + '-tab').focus(); }
  });
}
selectMode('manual');
const cartesianKeys = {w:'forward',s:'back',a:'left',d:'right',r:'up',f:'down',q:'open',e:'close',i:'tilt_up',k:'tilt_down',j:'roll_left',l:'roll_right'};
const cartesianButtons = () => document.querySelectorAll('[data-direction]');
function cartesianAllowed(direction) {
  const capability = state?.cartesian;
  if (!(movable() && cartesianCommandsAvailable() && manualMode === 'cartesian' && !$('manual').hidden && !document.hidden && capability?.available)) return false;
  const orientation = capability.orientation_controls;
  const group = ['tilt_up','tilt_down'].includes(direction) ? 'tilt' : ['roll_left','roll_right'].includes(direction) ? 'roll' : null;
  if (group) return orientation?.supported === true && orientation[group]?.supported === true && orientation[group].available === true;
  return ['open','close'].includes(direction) || capability.spatial_available;
}
function renderCartesian() {
  const capability = state?.cartesian, profile = capability?.profiles?.[$('cartesian-speed').value || 'slow'];
  renderRecovery(capability);
  renderOrientation(capability,profile);
  renderCartesianRanges(capability?.ranges);
  cartesianButtons().forEach(button => { button.disabled = !cartesianAllowed(button.dataset.direction); button.setAttribute('aria-pressed',String(cartesianHold?.direction === button.dataset.direction)); });
  setText('cartesian-rates', profile ? `${Number(profile.linear_m_s * 1000).toFixed(0)} mm/s · yaw ${profile.angular_deg_s}°/s · gripper ${profile.gripper_units_s} units/s` : 'Motion rates unavailable');
  setText('cartesian-status', cartesianHold ? 'Moving ' + cartesianHold.direction.replaceAll('_',' ') + ' · release to stop' : !cartesianCommandsAvailable() ? 'Cartesian commands are unavailable through this dashboard.' : !capability?.available ? capability?.reason || 'Cartesian control unavailable.' : !capability.spatial_available ? capability.reason || 'Configure the workspace floor to enable arm movement. Gripper is available.' : 'Hold one direction. Release to stop.');
  if (cartesianHold && !cartesianAllowed(cartesianHold.direction)) cancelCartesian();
  // Polls can predate a start. Only an equally new backend sequence can end it.
  if (cartesianHold?.motionId && capability?.sequence >= cartesianHold.sequence && (!capability.active || capability.motion_id !== cartesianHold.motionId)) {
    const reason = stoppedReasonForHold(capability,cartesianHold);
    cancelCartesian(); if (reason) notice(reason,'motion');
  }
}
function stoppedReasonForHold(snapshot,hold) {
  return hold?.motionId && snapshot?.sequence >= hold.sequence && !snapshot.active && snapshot.stopped_motion_id === hold.motionId && typeof snapshot.stop_reason === 'string' ? snapshot.stop_reason : '';
}
function renderOrientation(capability, profile) {
  const orientation = capability?.orientation_controls;
  const messages = [];
  if (orientation?.supported !== true) messages.push('Orientation controls are unavailable on this controller.');
  else for (const group of ['tilt','roll']) {
    const control = orientation[group];
    if (control?.supported !== true || control.available !== true) messages.push(title(group) + ': ' + (control?.reason || 'Unavailable on this controller.'));
  }
  setText('cartesian-orientation-status',messages.join(' '));
  $('cartesian-orientation-status').hidden = !messages.length;
  const tiltRate = profile?.[orientation?.angular_rate_key], rollRate = profile?.[orientation?.angular_rate_key];
  setText('cartesian-orientation-rates',orientation?.supported === true && Number.isFinite(tiltRate) && Number.isFinite(rollRate) ? `Tilt ${tiltRate}°/s · roll ${rollRate}°/s` : 'Uses selected movement speed.');
}
function recoverySupported(capability = state?.cartesian) {
  const recovery = capability?.recovery;
  return cartesianCommandsAvailable() && recovery?.supported === true && [recovery.max_soft_violation_deg,recovery.max_tip_drift_m,recovery.max_pitch_change_deg].every(value => Number.isFinite(value) && value > 0);
}
const recoverySelected = () => $('cartesian-recovery').getAttribute('aria-checked') === 'true';
function renderRecovery(capability) {
  const supported = recoverySupported(capability), button = $('cartesian-recovery');
  $('cartesian-recovery-setting').hidden = !supported; button.disabled = !supported;
  if (!supported) {
    button.setAttribute('aria-checked','false');
    if (cartesianHold?.mode === 'recovery') cancelCartesian();
  }
  const selected = recoverySelected(), recovery = capability?.recovery;
  setText('cartesian-recovery-selection',selected ? 'On' : 'Off');
  if (supported) setText('cartesian-recovery-help',`Up to ${recovery.max_pitch_change_deg}° total tilt change and ${Number((recovery.max_tip_drift_m * 1000).toFixed(1))} mm tip error per hold. Can recover ${recovery.max_soft_violation_deg}° joint overshoot. Uses selected speed.`);
  const phase = capability?.phase || recovery?.phase;
  const active = selected && cartesianHold?.mode === 'recovery' && capability?.active;
  const message = active ? phase === 'ingress' ? 'Returning joint inside limits · hold to continue.' : 'Recovery moving · release to stop.' : selected && recovery?.reason ? 'Recovery stopped: ' + recovery.reason : '';
  setText('cartesian-recovery-status',message); $('cartesian-recovery-status').hidden = !message;
  const progress = active && Number.isFinite(recovery?.pitch_change_deg) && Number.isFinite(recovery?.tip_drift_m);
  setText('cartesian-recovery-progress',progress ? `Tilt change ${fmt(recovery.pitch_change_deg)}° · tip error ${fmt(recovery.tip_drift_m * 1000)} mm` : '');
  $('cartesian-recovery-progress').hidden = !progress;
}
function renderCartesianRanges(ranges) {
  const fresh = healthy() && Number.isFinite(ranges?.age_s) && ranges.age_s >= 0 && ranges.age_s <= 0.75;
  const ready = fresh && ranges?.status === 'ready' && ranges.orientation === 'fixed_tilt';
  const sourcePose = ranges?.source_pose;
  const validSource = sourcePose && !Array.isArray(sourcePose) && (state?.joints || []).length > 0 && state.joints.every(joint => Number.isFinite(sourcePose[joint])) && Number.isInteger(ranges.source_observation_sequence) && ranges.source_observation_sequence >= 0;
  const previous = healthy() && ranges?.status === 'stale' && ranges.stale_reason === 'stale_pose' && ranges.orientation === 'fixed_tilt' && validSource && Number.isFinite(ranges.computed_age_s) && ranges.computed_age_s >= 0 && ranges.computed_age_s <= 3;
  const display = ready || previous;
  $('cartesian-ranges').dataset.state = previous ? 'previous' : ready ? 'current' : 'unavailable';
  setText('cartesian-range-context',recoverySelected() ? previous ? 'Normal mode at the sampled tilt · Tip travel estimate' : 'Normal mode at current tilt · Tip travel estimate' : previous ? 'At the sampled tilt · Tip travel estimate' : 'At current tilt · Tip travel estimate');
  $('cartesian-range-context').title = previous ? 'Source observation ' + ranges.source_observation_sequence + ': ' + state.joints.map(joint => title(joint) + ' ' + fmt(sourcePose[joint])).join(', ') : '';
  const horizon = ranges?.horizon_m;
  const usableScale = Number.isFinite(horizon) && horizon > 0;
  const reasons = {joint_limit:'Joint limit',tcp_floor:'Minimum tip height.',floor:'Minimum tip height.',local_solver_limit:'No further path found at this tilt.',ik_limit:'No further path found at this tilt.',horizon:'Estimate horizon',horizon_reached:'Estimate horizon','Local solver / position-orientation boundary.':'No further path found at this tilt.','Configured TCP floor.':'Minimum tip height.'};
  const values = {};
  for (const name of ['forward','back','up','down']) {
    const estimate = ranges?.directions?.[name];
    const usable = display && usableScale && typeof estimate?.allowed === 'boolean' && Number.isFinite(estimate.distance_m) && estimate.distance_m >= 0 && estimate.distance_m <= horizon;
    const mm = usable ? (estimate.distance_m * 1000).toFixed(1).replace(/\.0$/,'') : null;
    values[name] = usable ? (estimate.at_least ? '≥' : '~') + mm + ' mm' : '—';
    setText('range-' + name,values[name]);
    $('range-' + name + '-fill').style.width = usable ? Math.min(50,estimate.distance_m / horizon * 50) + '%' : '0%';
    $('range-' + name + '-side').dataset.boundary = ready && usable && estimate.allowed === false ? 'blocked' : 'clear';
    const reason = estimate?.limiting_reason;
    setText('range-' + name + '-reason',usable ? [reason ? reasons[reason] || reason : estimate.at_least ? 'Estimate horizon' : '',estimate.limiting_joint ? title(estimate.limiting_joint) : ''].filter(Boolean).join(' · ') || 'No boundary reported' : '—');
    $('range-' + name + '-reason').title = usable && reason ? reason : '';
  }
  for (const [axis,negative,positive] of [['reach','back','forward'],['height','down','up']]) {
    $('range-' + axis + '-track').setAttribute('aria-label',`${previous ? 'Previous pose: ' : ''}${title(negative)} ${values[negative]}; ${title(positive)} ${values[positive]}`);
  }
  setText('cartesian-range-status',!online ? 'Connection lost · estimates unavailable.' : !healthy() ? 'Fresh arm feedback is required for an estimate.' : previous && usableScale ? `Updating — estimate from previous pose, ${ranges.computed_age_s.toFixed(1)}s ago.` : ready && usableScale ? 'Distances from the current arm pose.' : ranges?.status === 'pending' ? 'Calculating travel from this pose…' : ranges?.status === 'stale' || ranges?.status === 'ready' && !fresh ? 'Updating estimate · previous values hidden.' : ranges?.status === 'unavailable' ? 'Travel estimate unavailable for this pose.' : 'Waiting for a travel estimate.');
  setText('cartesian-range-scale',display && usableScale ? `Each side shows up to ${Number((horizon * 1000).toFixed(1))} mm. Limits change with pose.` : 'Limits change with the arm pose.');
}
function cancelCartesian() {
  const hold = cartesianHold;
  if (!hold) return;
  cartesianHold = null;
  clearTimeout(hold.timer);
  const body = {token:hold.token,sequence:++cartesianSequence};
  if (hold.motionId) body.motion_id = hold.motionId;
  // Higher sequences cancel starts whose responses have not arrived yet.
  fetch('/api/cartesian/stop', {method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(body),keepalive:true}).catch(() => {});
  cartesianButtons().forEach(button => button.setAttribute('aria-pressed','false'));
  setText('cartesian-status','Stopped · hold a direction to move.');
}
async function cartesianRequest(hold, route) {
  if (cartesianPending || cartesianHold !== hold) return;
  cartesianPending = true;
  hold.sequence = ++cartesianSequence;
  const body = {token:hold.token,sequence:hold.sequence};
  if (route === 'start') Object.assign(body,{direction:hold.direction,profile:$('cartesian-speed').value || 'slow',mode:hold.mode,observation_sequence:state.sequence});
  else body.motion_id = hold.motionId;
  try {
    const response = await fetch('/api/cartesian/' + route,{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(body),signal:AbortSignal.timeout(route === 'start' ? 1200 : 250)});
    const result = await response.json();
    if (cartesianHold !== hold) return;
    if (!response.ok || !result.cartesian?.active || !result.cartesian.motion_id || (route === 'renew' && result.cartesian.motion_id !== hold.motionId) || !cartesianAllowed(hold.direction)) {
      const snapshot = result.cartesian?.sequence >= hold.sequence ? result.cartesian : state?.cartesian?.sequence >= hold.sequence ? state.cartesian : null;
      const genericEnded = result.error === 'Hold-jog has ended; press again to move.';
      const stoppedReason = stoppedReasonForHold(snapshot,hold);
      const reason = result.error && !genericEnded ? result.error : stoppedReason || result.error || 'Motion ended. Release and hold again when ready.';
      cancelCartesian(); notice(reason,'motion'); return;
    }
    hold.motionId = result.cartesian.motion_id;
    if (result.cartesian.recovery) renderRecovery(result.cartesian);
    hold.timer = setTimeout(() => cartesianRequest(hold,'renew'),100);
  } catch (_) {
    if (cartesianHold === hold) { cancelCartesian(); notice('Motion connection lost. Release and hold again when ready.','motion'); }
  } finally { cartesianPending = false; }
}
function beginCartesian(direction, source) {
  if (cartesianHold || cartesianPending || !cartesianAllowed(direction)) return false;
  clearNoticeForAction('motion');
  const mode = recoverySelected() && recoverySupported() && ['forward','back','up','down'].includes(direction) ? 'recovery' : 'normal';
  const hold = {direction,source,token,mode,motionId:null,sequence:0,timer:null};
  cartesianHold = hold; renderCartesian(); cartesianRequest(hold,'start'); return true;
}
function selectManualMode(mode) {
  cancelCartesian(); manualMode = mode;
  for (const name of ['cartesian','joint']) { const active = name === mode; $(name + '-controls').hidden = !active; $(name + '-mode').setAttribute('aria-pressed',String(active)); }
  enable();
}
for (const mode of ['cartesian','joint']) $(mode + '-mode').onclick = () => selectManualMode(mode);
$('cartesian-speed').onchange = cancelCartesian;
$('cartesian-keyboard').onchange = cancelCartesian;
$('cartesian-recovery').onclick = () => {
  if (!recoverySupported()) return;
  cancelCartesian();
  $('cartesian-recovery').setAttribute('aria-checked',String(!recoverySelected()));
  enable();
};
cartesianButtons().forEach(button => {
  button.addEventListener('pointerdown',event => {
    if (event.button !== 0 || !beginCartesian(button.dataset.direction,'pointer:' + event.pointerId)) return;
    event.preventDefault();
    try { button.setPointerCapture(event.pointerId); } catch (_) { cancelCartesian(); }
  });
  for (const name of ['pointerup','pointercancel','lostpointercapture']) button.addEventListener(name,event => { if (cartesianHold?.source === 'pointer:' + event.pointerId) cancelCartesian(); });
  button.addEventListener('contextmenu',event => event.preventDefault());
});
document.addEventListener('keydown',event => {
  const direction = cartesianKeys[event.key?.toLowerCase()];
  if (!direction || !$('cartesian-keyboard').checked || event.repeat || event.altKey || event.ctrlKey || event.metaKey || event.shiftKey || (event.target !== $('cartesian-keyboard') && event.target?.closest?.('input,textarea,select,[contenteditable]:not([contenteditable="false"])'))) return;
  if (beginCartesian(direction,'key:' + event.key.toLowerCase())) event.preventDefault();
});
document.addEventListener('keyup',event => { if (cartesianHold?.source === 'key:' + event.key?.toLowerCase()) { event.preventDefault(); cancelCartesian(); } });
window.addEventListener('blur',cancelCartesian);
window.addEventListener('pointerup',event => { if (cartesianHold?.source === 'pointer:' + event.pointerId) cancelCartesian(); });
document.addEventListener('visibilitychange', () => { if (document.hidden) { release(); suspendMedia(); } else if (state && online) { updateCameras(); updateViewer(); } });
window.addEventListener('pagehide', () => { release(); suspendMedia(); });
async function stop() {
  const request = ++safetyActionEpoch; controlRequestEpoch++; claimPending = false; safetyPending = 'stop'; safetyFeedback = '';
  dragging.clear(); loseControl(); enable();
  const result = await post('emergency-stop');
  if (request !== safetyActionEpoch) return;
  safetyPending = '';
  if (result?.emergency_stop) {
    if (state) { state.emergency_stop = result.emergency_stop; state.control = {owner_id:null}; }
    safetyFeedback = result.emergency_stop.hold_error ? 'Software stop latched | hold error: ' + result.emergency_stop.hold_error : result.emergency_stop.hold_status === 'sent' ? 'Software stop latched | hold command sent' : 'Software stop latched | hold status pending';
  } else safetyFeedback = 'Stop request not confirmed | use the physical power disconnect if needed';
  enable();
}
async function resetEmergencyStop() {
  if (!stopStateAvailable() || !state.emergency_stop.latched || safetyPending || !supportsCommands('emergency-stop/reset')) return;
  const request = ++safetyActionEpoch; controlRequestEpoch++; claimPending = false;
  const generation = state.emergency_stop.generation; safetyPending = 'reset'; safetyFeedback = '';
  loseControl(); enable();
  const result = await post('emergency-stop/reset', {emergency_stop_generation:generation});
  if (request !== safetyActionEpoch) return;
  safetyPending = '';
  if (result?.emergency_stop && !result.emergency_stop.latched) {
    state.emergency_stop = result.emergency_stop; state.control = {owner_id:null};
    safetyFeedback = 'Stop latch reset | take control again when ready';
  } else safetyFeedback = 'Reset request not confirmed | refresh state and retry';
  enable();
}
$('stop').onclick = stop;
$('emergency-reset').onclick = resetEmergencyStop;
document.addEventListener('keydown', event => { if (event.key === 'Escape') { event.preventDefault(); stop(); } });
$('home').onclick = () => movable() && post('home', {}, true);
$('speed').onchange = () => post('speed', {value:Number($('speed').value)}, true);
$('torque').onclick = () => { if (!own()) return; if (state.torque && !confirm('Support the arm before turning torque off. The arm can fall under its own weight. Turn torque off?')) return; post('torque', {on:!state.torque}, true); };

function drawer(open) { $('session').hidden = !open; $('session-toggle').setAttribute('aria-expanded', String(open)); if (open) $('session-close').focus(); else $('session-toggle').focus(); }
$('session-toggle').onclick = () => drawer($('session').hidden); $('session-close').onclick = () => drawer(false);
$('video-preview').onchange = () => {
  const delivery = cameraDelivery(), next = Number($('video-preview').value);
  if (!delivery?.rates.includes(next) || next === previewFps) return;
  previewFps = next; previewOverride = true; updateCameras();
};
for (const name of ['overhead','wrist','observer']) for (let index = 0; index < 10; index++) $('camera-' + name).add(new Option('Camera ' + index, String(index)));
$('camera-save').onclick = () => {
  const source = name => { const value = $('camera-' + name).value; return value === '' ? null : /^\d+$/.test(value) ? Number(value) : value; };
  const overhead = source('overhead'), wrist = source('wrist'), observer = source('observer');
  const assigned = [overhead,wrist,observer].filter(value => value !== null).map(String);
  if (new Set(assigned).size !== assigned.length) { notice('Assign a different camera to each view.'); return; }
  post('cameras', {overhead,wrist,observer}, true);
};
function swap() { if (compact.matches) { setMobileView(mobileView === 'overhead' ? 'wrist' : 'overhead'); return; } primary = primary === 'overhead' ? 'wrist' : 'overhead'; if (state) { updateCamera('primary',primary); updateCamera('secondary',primary === 'overhead' ? 'wrist' : 'overhead'); } }
$('swap').onclick = swap; $('primary-media').onclick = swap; $('secondary-media').onclick = swap;
$('enlarge').onclick = () => { document.querySelector('.workbench').classList.remove('viewer-enlarged'); $('enlarge-viewer').setAttribute('aria-pressed','false'); const expanded = document.querySelector('.workbench').classList.toggle('enlarged'); $('enlarge').setAttribute('aria-label', expanded ? 'Restore camera layout' : 'Enlarge primary camera'); $('enlarge').setAttribute('aria-pressed',String(expanded)); if (state) updateCameras(); };
$('enlarge-viewer').onclick = () => {
  const bench = document.querySelector('.workbench'); bench.classList.remove('enlarged'); $('enlarge').setAttribute('aria-pressed','false');
  const expanded = bench.classList.toggle('viewer-enlarged');
  $('enlarge-viewer').setAttribute('aria-pressed',String(expanded));
  $('enlarge-viewer').setAttribute('aria-label',expanded ? 'Restore 3D view layout' : 'Enlarge 3D view');
  $('enlarge-viewer').title = expanded ? 'Restore 3D view layout' : 'Enlarge 3D view';
  if (state) updateCameras();
};
function setMobileView(view) { document.querySelector('.workbench').classList.remove('viewer-enlarged'); $('enlarge-viewer').setAttribute('aria-pressed','false'); $('enlarge-viewer').setAttribute('aria-label','Enlarge 3D view'); mobileView = view; document.body.dataset.mobileView = view; document.querySelectorAll('[data-view]').forEach(b => b.setAttribute('aria-pressed',String(b.dataset.view === view))); if (compact.matches) primary = 'overhead'; if (state) updateCameras(); updateViewer(); }
document.querySelectorAll('[data-view]').forEach(b => b.onclick = () => setMobileView(b.dataset.view));
compact.addEventListener('change', () => setMobileView(mobileView));
$('diagnostics').addEventListener('toggle', () => {if (state) renderDiagnostics();});
for (const slot of ['primary','secondary','observer']) {
  const image = $(slot + '-image');
  image.addEventListener('error', () => {
    if (!snapshotPreview && online && image.dataset.feed && ['localhost','127.0.0.1'].includes(location.hostname)
        && new URL(image.dataset.feed, location.href).port === '8781') {
      snapshotPreview = true;
      updateCameras();
      return;
    }
    image.hidden = true;
    image.nextElementSibling.hidden = false;
    setText(slot + '-empty-title','Camera stream interrupted');
    setText(slot + '-empty-copy','Check camera connection in Session.');
  });
}
setMobileView('overhead'); poll();
