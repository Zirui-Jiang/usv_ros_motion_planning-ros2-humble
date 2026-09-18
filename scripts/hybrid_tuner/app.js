'use strict';
const $ = (id) => document.getElementById(id);
const TAU = Math.PI * 2;
const wrap = (a) => ((a + Math.PI) % TAU + TAU) % TAU - Math.PI;
const fmt = (x, n = 1) => Number.isFinite(x) ? x.toFixed(n) : '—';
const deg = (r) => r * 180 / Math.PI;

const state = {
  world: null,
  occupancy: null,
  costs: null,
  start: [8, 12, 0],
  goal: [54, 28, 0],
  path: [],
  trajectory: [],
  ghost: null,
  overlays: [],
  last: null,
  tool: 'brush',
  drag: null,
  rect: null,
  busy: false,
  inflateTimer: 0,
};

function idx(ix, iy) {
  const { width: cols, height: rows } = state.world.map;
  if (ix < 0 || iy < 0 || ix >= cols || iy >= rows) return -1;
  return iy * cols + ix;
}

function collectParams() {
  const params = {};
  document.querySelectorAll('[data-param]').forEach((el) => {
    const key = el.dataset.param;
    if (el.type === 'checkbox') params[key] = el.checked;
    else if (key === 'planner_name' || key === 'heuristic') params[key] = el.value;
    else params[key] = Number(el.value);
  });
  params.turning_radius_multipliers = [...document.querySelectorAll('#multipliers [data-mult]:checked')]
    .map((el) => Number(el.dataset.mult));
  params.with_runtime = $('with-runtime').checked;
  return params;
}

function syncOutputs() {
  document.querySelectorAll('[data-out]').forEach((out) => {
    const input = document.querySelector(`[data-param="${out.dataset.out}"]`);
    if (!input) return;
    const value = Number(input.value);
    out.textContent = Number.isInteger(value) || input.step === '1' ? String(value) : fmt(value, value < 2 ? 2 : 1);
    if (out.dataset.out === 'max_expansions') out.textContent = String(Math.round(value));
    if (out.dataset.out === 'angle_bins') out.textContent = String(Math.round(value));
  });
}

function syncPoseFields() {
  ['sx', 'sy', 'syaw', 'gx', 'gy', 'gyaw'].forEach((id, i) => {
    const src = i < 3 ? state.start : state.goal;
    $(id).value = fmt(src[i % 3], i % 3 === 2 ? 3 : 2);
  });
}

function readPoseFields() {
  state.start = [Number($('sx').value), Number($('sy').value), Number($('syaw').value)];
  state.goal = [Number($('gx').value), Number($('gy').value), Number($('gyaw').value)];
}

async function api(path, body) {
  const response = await fetch(path, {
    method: body ? 'POST' : 'GET',
    headers: body ? { 'Content-Type': 'application/json' } : undefined,
    body: body ? JSON.stringify(body) : undefined,
  });
  const data = await response.json();
  if (!response.ok) throw new Error(data.error || response.statusText);
  return data;
}

function layout() {
  const canvas = $('map');
  const pad = 36;
  const map = state.world.map;
  const worldW = map.width * map.resolution;
  const worldH = map.height * map.resolution;
  const scale = Math.min((canvas.width - pad * 2) / worldW, (canvas.height - pad * 2) / worldH);
  return { canvas, pad, scale, worldW, worldH, x0: pad, y0: canvas.height - pad, map };
}

function toCanvas(x, y, L) {
  return [L.x0 + x * L.scale, L.y0 - y * L.scale];
}

function toWorld(cx, cy, L) {
  return [(cx - L.x0) / L.scale, (L.y0 - cy) / L.scale];
}

function eventPos(ev) {
  const canvas = $('map');
  const box = canvas.getBoundingClientRect();
  return [(ev.clientX - box.left) * canvas.width / box.width,
          (ev.clientY - box.top) * canvas.height / box.height];
}

function costRGBA(cost, occ) {
  if (occ === 2 || cost === 255) return [148, 158, 162, 255];
  if (occ === 1 || cost === 254) return [22, 32, 44, 255];
  if (cost === 253) return [196, 154, 48, 230];
  if (!cost) return [244, 247, 244, 255];
  const t = cost / 252;
  return [Math.round(226 - t * 150), Math.round(236 - t * 110), Math.round(232 - t * 70), 255];
}

function draw() {
  if (!state.world) return;
  const L = layout();
  const ctx = L.canvas.getContext('2d');
  ctx.clearRect(0, 0, L.canvas.width, L.canvas.height);
  ctx.fillStyle = '#eef3ee';
  ctx.fillRect(0, 0, L.canvas.width, L.canvas.height);

  const { width: cols, height: rows } = L.map;
  const img = ctx.createImageData(cols, rows);
  const pix = img.data;
  for (let row = 0; row < rows; row++) {
    const iy = rows - 1 - row;
    for (let ix = 0; ix < cols; ix++) {
      const i = iy * cols + ix;
      const rgba = costRGBA(state.costs ? state.costs[i] : 0, state.occupancy[i]);
      const p = (row * cols + ix) * 4;
      pix[p] = rgba[0]; pix[p + 1] = rgba[1]; pix[p + 2] = rgba[2]; pix[p + 3] = rgba[3];
    }
  }
  const off = draw._off || (draw._off = document.createElement('canvas'));
  off.width = cols;
  off.height = rows;
  off.getContext('2d').putImageData(img, 0, 0);
  ctx.imageSmoothingEnabled = false;
  const [x0, yTop] = [L.x0, L.y0 - L.worldH * L.scale];
  ctx.drawImage(off, x0, yTop, L.worldW * L.scale, L.worldH * L.scale);

  ctx.save();
  ctx.beginPath();
  ctx.rect(x0, yTop, L.worldW * L.scale, L.worldH * L.scale);
  ctx.clip();
  ctx.strokeStyle = '#b7c6bb';
  ctx.lineWidth = 1;
  ctx.globalAlpha = 0.45;
  for (let x = 0; x <= L.worldW; x += 5) {
    const [cx] = toCanvas(x, 0, L);
    ctx.beginPath(); ctx.moveTo(cx, yTop); ctx.lineTo(cx, L.y0); ctx.stroke();
  }
  for (let y = 0; y <= L.worldH; y += 5) {
    const [, cy] = toCanvas(0, y, L);
    ctx.beginPath(); ctx.moveTo(x0, cy); ctx.lineTo(x0 + L.worldW * L.scale, cy); ctx.stroke();
  }
  ctx.globalAlpha = 1;

  if (state.rect) {
    const [ax, ay] = toCanvas(state.rect[0], state.rect[1], L);
    const [bx, by] = toCanvas(state.rect[2], state.rect[3], L);
    ctx.fillStyle = '#c34d4e33';
    ctx.strokeStyle = '#c34d4e';
    ctx.fillRect(Math.min(ax, bx), Math.min(ay, by), Math.abs(bx - ax), Math.abs(by - ay));
    ctx.strokeRect(Math.min(ax, bx), Math.min(ay, by), Math.abs(bx - ax), Math.abs(by - ay));
  }

  state.overlays.forEach((layer) => drawPath(ctx, L, layer.path, layer.color, layer.width, layer.dash));
  if (state.ghost) drawPath(ctx, L, state.ghost.path, '#c4a35a', 3, [8, 5]);
  if (state.path.length) {
    drawPath(ctx, L, state.path, '#087d70', 3);
    drawHulls(ctx, L, state.path);
  }
  if (state.trajectory.length) drawPath(ctx, L, state.trajectory.map((p) => [p[2], p[3], p[4]]), '#327bc0', 1.5, [3, 3]);
  drawArrow(ctx, L, state.start, '#163b50', '起');
  drawArrow(ctx, L, state.goal, '#c34d4e', '终');
  ctx.restore();

  ctx.fillStyle = '#617c8a';
  ctx.font = '11px Consolas, monospace';
  ctx.fillText('0', x0 - 4, L.y0 + 14);
  ctx.fillText(`${L.worldW.toFixed(0)} m`, x0 + L.worldW * L.scale - 28, L.y0 + 14);
  ctx.fillText(`${L.worldH.toFixed(0)} m`, x0 - 32, yTop + 10);
}

function drawPath(ctx, L, path, color, width, dash) {
  if (!path || path.length < 2) return;
  ctx.beginPath();
  path.forEach((p, i) => {
    const [x, y] = toCanvas(p[0], p[1], L);
    if (i) ctx.lineTo(x, y); else ctx.moveTo(x, y);
  });
  ctx.strokeStyle = color;
  ctx.lineWidth = width;
  ctx.setLineDash(dash || []);
  ctx.lineJoin = 'round';
  ctx.stroke();
  ctx.setLineDash([]);
}

function drawHulls(ctx, L, path) {
  const params = collectParams();
  const len = params.footprint_length, wid = params.footprint_width;
  let acc = 0;
  ctx.strokeStyle = '#087d7088';
  ctx.lineWidth = 1;
  for (let i = 1; i < path.length; i++) {
    acc += Math.hypot(path[i][0] - path[i - 1][0], path[i][1] - path[i - 1][1]);
    if (acc < 4 && i !== path.length - 1) continue;
    acc = 0;
    const p = path[i], c = Math.cos(p[2]), s = Math.sin(p[2]);
    const hx = len / 2, hy = wid / 2;
    const corners = [[hx, hy], [hx, -hy], [-hx, -hy], [-hx, hy], [hx, hy]];
    ctx.beginPath();
    corners.forEach((q, k) => {
      const [x, y] = toCanvas(p[0] + q[0] * c - q[1] * s, p[1] + q[0] * s + q[1] * c, L);
      if (k) ctx.lineTo(x, y); else ctx.moveTo(x, y);
    });
    ctx.stroke();
  }
}

function drawArrow(ctx, L, pose, color, label) {
  const [x, y] = toCanvas(pose[0], pose[1], L);
  const tip = toCanvas(pose[0] + 3.2 * Math.cos(pose[2]), pose[1] + 3.2 * Math.sin(pose[2]), L);
  ctx.strokeStyle = color;
  ctx.fillStyle = color;
  ctx.lineWidth = 2.4;
  ctx.beginPath();
  ctx.moveTo(x, y);
  ctx.lineTo(tip[0], tip[1]);
  ctx.stroke();
  ctx.beginPath();
  ctx.arc(x, y, 7, 0, TAU);
  ctx.fill();
  const ang = Math.atan2(tip[1] - y, tip[0] - x);
  ctx.beginPath();
  ctx.moveTo(tip[0], tip[1]);
  ctx.lineTo(tip[0] - 12 * Math.cos(ang - 0.4), tip[1] - 12 * Math.sin(ang - 0.4));
  ctx.lineTo(tip[0] - 12 * Math.cos(ang + 0.4), tip[1] - 12 * Math.sin(ang + 0.4));
  ctx.closePath();
  ctx.fill();
  ctx.font = '12px Microsoft YaHei UI, sans-serif';
  ctx.fillText(label, x + 10, y - 10);
}

function paintCell(ix, iy, value) {
  const i = idx(ix, iy);
  if (i < 0) return;
  state.occupancy[i] = value;
}

function worldToCell(x, y) {
  const res = state.world.map.resolution;
  return [Math.floor(x / res), Math.floor(y / res)];
}

function applyRect(x0, y0, x1, y1, value) {
  const a = worldToCell(x0, y0), b = worldToCell(x1, y1);
  const xA = Math.min(a[0], b[0]), xB = Math.max(a[0], b[0]);
  const yA = Math.min(a[1], b[1]), yB = Math.max(a[1], b[1]);
  for (let iy = yA; iy <= yB; iy++) for (let ix = xA; ix <= xB; ix++) paintCell(ix, iy, value);
}

function hitPose(wx, wy) {
  const tests = [['start', state.start], ['goal', state.goal]];
  for (const [name, pose] of tests) {
    const hx = pose[0] + 3.2 * Math.cos(pose[2]);
    const hy = pose[1] + 3.2 * Math.sin(pose[2]);
    if (Math.hypot(wx - hx, wy - hy) < 1.6) return { name, mode: 'yaw' };
    if (Math.hypot(wx - pose[0], wy - pose[1]) < 1.8) return { name, mode: 'move' };
  }
  return null;
}

function clampPose(pose) {
  pose[0] = Math.min(Math.max(pose[0], 0.4), state.world.world.width - 0.4);
  pose[1] = Math.min(Math.max(pose[1], 0.4), state.world.world.height - 0.4);
}

async function refreshCosts() {
  const data = await api('/api/inflate', { occupancy: Array.from(state.occupancy), params: collectParams() });
  state.costs = Uint8Array.from(data.costs);
  draw();
}

function scheduleInflate() {
  clearTimeout(state.inflateTimer);
  state.inflateTimer = setTimeout(() => refreshCosts().catch((err) => setMessage(err.message, true)), 140);
}

function setBusy(on, text) {
  state.busy = on;
  $('busy').classList.toggle('on', on);
  $('busy').textContent = text || '规划中…';
  $('plan').disabled = on;
}

function setMessage(text, bad) {
  $('message').textContent = text;
  $('message').style.borderLeftColor = bad ? '#ffa5a0' : '#63cfbd';
}

function showResult(result) {
  state.last = result;
  state.path = result.path || [];
  state.trajectory = (result.runtime && result.runtime.trajectory) || [];
  if (result.costs) state.costs = Uint8Array.from(result.costs);
  const ok = result.status === 'success' || result.status === 'goal_reached';
  $('m-status').textContent = result.status || '—';
  $('m-status').classList.toggle('bad', !ok);
  $('m-length').textContent = ok ? `${fmt(result.length, 2)} m` : '—';
  $('m-expanded').textContent = result.expanded == null ? '—' : String(result.expanded);
  $('m-elapsed').textContent = result.elapsed == null ? '—' : `${fmt(result.elapsed * 1000, 0)} ms`;
  $('m-turns').textContent = result.turn_switches == null ? '—' : String(result.turn_switches);
  $('m-clear').textContent = result.min_clearance == null ? '—' : `${fmt(result.min_clearance, 2)} m`;
  const extra = result.runtime ? ` 决策链 ${result.runtime.status}: ${result.runtime.message || 'OK'}` : '';
  setMessage((result.message || result.status) + extra, !ok);
  if (state.ghost && ok) {
    const dL = result.length - state.ghost.length;
    const dT = result.elapsed - state.ghost.elapsed;
    $('delta').textContent = `对照 A  Δ长度 ${dL >= 0 ? '+' : ''}${fmt(dL, 2)} m · Δ耗时 ${dT >= 0 ? '+' : ''}${fmt(dT * 1000, 0)} ms`;
  } else if (!state.ghost) $('delta').textContent = '';
  draw();
}

function requestBody() {
  return { occupancy: Array.from(state.occupancy), start: state.start, goal: state.goal, params: collectParams() };
}

async function runPlan() {
  if (state.busy) return;
  setBusy(true);
  try {
    const result = await api('/api/plan', requestBody());
    state.overlays = [];
    $('panel').innerHTML = '';
    showResult(result);
  } catch (error) {
    setMessage(error.message, true);
  } finally {
    setBusy(false);
  }
}

function table(caption, headers, rows) {
  const html = [`<table><caption>${caption}</caption><thead><tr>${headers.map((h) => `<th>${h}</th>`).join('')}</tr></thead><tbody>`];
  rows.forEach((row) => html.push(`<tr>${row.map((c) => `<td>${c}</td>`).join('')}</tr>`));
  html.push('</tbody></table>');
  $('panel').innerHTML = html.join('');
}

async function runExperiment(kind) {
  if (state.busy) return;
  setBusy(true, '实验运行中…');
  try {
    if (kind === 'heading') {
      const data = await api('/api/experiment/heading', { ...requestBody(), samples: 12 });
      state.overlays = (data.variants || [])
        .filter((item) => item.status === 'success')
        .map((item) => ({ path: item.path, color: '#be741b55', width: 1.5 }));
      if (data.best) state.overlays.push({ path: data.best.path, color: '#087d70', width: 3 });
      const fixedLen = data.fixed && data.fixed.status === 'success' ? data.fixed.length : null;
      const bestLen = data.best ? data.best.length : null;
      const tax = (fixedLen != null && bestLen != null) ? fixedLen - bestLen : null;
      table('终点艏向（12 个方向）', ['艏向', '状态', '长度 m', '展开'], [
        ['固定', data.fixed.status, fmt(data.fixed.length, 2), data.fixed.expanded],
        ...(data.variants || []).map((item) => [`${fmt(deg(item.yaw), 0)}°`, item.status, fmt(item.length, 2), item.expanded]),
      ]);
      showResult(data.best ? { ...data.best, costs: data.fixed.costs, message: `最短自由艏向 ${fmt(bestLen, 2)} m；固定艏向 ${fmt(fixedLen, 2)} m；艏向税 ${fmt(tax, 2)} m` }
        : data.fixed);
    } else if (kind === 'heuristic') {
      const data = await api('/api/experiment/heuristic', requestBody());
      const colors = { euclidean: '#087d70', dubins: '#327bc0', max_dubins_dijkstra: '#be741b' };
      state.overlays = Object.entries(data).filter(([, r]) => r.status === 'success')
        .map(([mode, r]) => ({ path: r.path, color: colors[mode], width: mode === 'euclidean' ? 3 : 2, dash: mode === 'euclidean' ? [] : [5, 4] }));
      table('启发式', ['模式', '状态', '长度 m', '展开', 'ms'],
        Object.entries(data).map(([mode, r]) => [mode, r.status, fmt(r.length, 2), r.expanded, fmt(r.elapsed * 1000, 0)]));
      showResult(data.euclidean || Object.values(data)[0]);
    } else if (kind === 'cost') {
      const data = await api('/api/experiment/cost-grid', requestBody());
      const rows = data.cells.map((row) => [fmt(row[0].obstacle_factor, 2),
        ...row.map((cell) => (cell.status === 'success' ? `${fmt(cell.length, 1)}/${fmt(cell.min_clearance || 0, 1)}` : cell.status.slice(0, 7)))]);
      table('长度 m / 净空 m', ['factor \\ penalty', ...data.penalties.map((v) => fmt(v, 1))], rows);
      const flat = data.cells.flat().filter((cell) => cell.status === 'success');
      const best = flat.sort((a, b) => a.length - b.length)[0];
      if (best) {
        state.overlays = [{ path: best.path, color: '#087d70', width: 3 }];
        showResult({ ...best, costs: state.costs && Array.from(state.costs), message: `网格最短：factor=${best.obstacle_factor} penalty=${best.cost_penalty}` });
      }
    } else if (kind === 'sweep') {
      const param = $('sweep-param').value;
      const current = collectParams()[param];
      const span = Math.max(Math.abs(current) * 0.5, param === 'heuristic_weight' ? 0.5 : 0.2);
      const values = [];
      for (let i = 0; i < 5; i++) values.push(Number((current - span + (2 * span * i) / 4).toFixed(3)));
      if (param === 'heuristic_weight') values.forEach((v, i) => { values[i] = Math.max(1, v); });
      const data = await api('/api/sweep', { ...requestBody(), param, values });
      table(`扫描 ${param}`, ['值', '状态', '长度 m', '展开', 'ms'],
        data.points.map((p) => [fmt(p.value, 3), p.status, fmt(p.length, 2), p.expanded, fmt(p.elapsed * 1000, 0)]));
      state.overlays = data.points.filter((p) => p.status === 'success')
        .map((p, i) => ({ path: p.path, color: `hsla(${160 - i * 20}, 55%, 35%, .7)`, width: 2 }));
      const mid = data.points[Math.floor(data.points.length / 2)];
      if (mid) showResult({ ...mid, costs: state.costs && Array.from(state.costs), message: `扫描 ${param}` });
    }
  } catch (error) {
    setMessage(error.message, true);
  } finally {
    setBusy(false);
  }
}

function loadPreset(key) {
  const preset = state.world.presets[key];
  state.occupancy = Uint8Array.from(preset.occupancy);
  state.start = preset.start.slice();
  state.goal = preset.goal.slice();
  state.path = [];
  state.overlays = [];
  $('preset-note').textContent = preset.note;
  document.querySelectorAll('#presets button').forEach((btn) => btn.setAttribute('aria-pressed', String(btn.dataset.preset === key)));
  syncPoseFields();
  refreshCosts().catch((err) => setMessage(err.message, true));
}

function onPointerDown(ev) {
  if (state.busy) return;
  const L = layout();
  const [cx, cy] = eventPos(ev);
  const [wx, wy] = toWorld(cx, cy, L);
  const handle = hitPose(wx, wy);
  if (handle) {
    state.drag = handle;
    $('map').setPointerCapture(ev.pointerId);
    return;
  }
  if (wx < 0 || wy < 0 || wx > L.worldW || wy > L.worldH) return;
  const value = state.tool === 'erase' ? 0 : state.tool === 'unknown' ? 2 : 1;
  if (state.tool === 'rect') {
    state.rect = [wx, wy, wx, wy];
    state.drag = { name: 'rect', value };
  } else {
    const [ix, iy] = worldToCell(wx, wy);
    paintCell(ix, iy, value);
    state.drag = { name: 'paint', value };
    scheduleInflate();
  }
  $('map').setPointerCapture(ev.pointerId);
  draw();
}

function onPointerMove(ev) {
  if (!state.drag) return;
  const L = layout();
  const [wx, wy] = toWorld(...eventPos(ev), L);
  if (state.drag.name === 'paint') {
    const [ix, iy] = worldToCell(wx, wy);
    paintCell(ix, iy, state.drag.value);
    draw();
    scheduleInflate();
    return;
  }
  if (state.drag.name === 'rect') {
    state.rect[2] = wx;
    state.rect[3] = wy;
    draw();
    return;
  }
  const pose = state[state.drag.name];
  if (state.drag.mode === 'move') {
    pose[0] = wx;
    pose[1] = wy;
    clampPose(pose);
  } else {
    pose[2] = wrap(Math.atan2(wy - pose[1], wx - pose[0]));
  }
  syncPoseFields();
  draw();
}

function onPointerUp() {
  if (state.drag && state.drag.name === 'rect' && state.rect) {
    applyRect(...state.rect, state.drag.value);
    state.rect = null;
    scheduleInflate();
  }
  state.drag = null;
  draw();
}

function bind() {
  document.querySelectorAll('.tools [data-tool]').forEach((btn) => {
    btn.addEventListener('click', () => {
      state.tool = btn.dataset.tool;
      document.querySelectorAll('.tools [data-tool]').forEach((other) => other.setAttribute('aria-pressed', String(other === btn)));
    });
  });
  $('clear-map').addEventListener('click', () => {
    state.occupancy.fill(0);
    state.path = [];
    state.overlays = [];
    scheduleInflate();
  });
  $('plan').addEventListener('click', runPlan);
  $('pin').addEventListener('click', () => {
    if (!state.last || !state.path.length) return;
    state.ghost = { path: state.path, length: state.last.length, elapsed: state.last.elapsed };
    $('delta').textContent = '已钉住对照 A。再规划一次看 Δ。';
    draw();
  });
  $('unpin').addEventListener('click', () => { state.ghost = null; $('delta').textContent = ''; draw(); });
  $('ex-heading').addEventListener('click', () => runExperiment('heading'));
  $('ex-heuristic').addEventListener('click', () => runExperiment('heuristic'));
  $('ex-cost').addEventListener('click', () => runExperiment('cost'));
  $('sweep').addEventListener('click', () => runExperiment('sweep'));
  ['sx', 'sy', 'syaw', 'gx', 'gy', 'gyaw'].forEach((id) => $(id).addEventListener('change', () => { readPoseFields(); draw(); }));
  document.querySelectorAll('[data-param], #multipliers input').forEach((el) => {
    el.addEventListener('input', () => {
      syncOutputs();
      if (['inscribed_radius', 'inflation_radius', 'cost_scaling_factor'].includes(el.dataset.param)) scheduleInflate();
    });
  });
  const canvas = $('map');
  canvas.addEventListener('pointerdown', onPointerDown);
  canvas.addEventListener('pointermove', onPointerMove);
  canvas.addEventListener('pointerup', onPointerUp);
  canvas.addEventListener('pointercancel', onPointerUp);
  window.addEventListener('keydown', (ev) => {
    if (ev.key === 'Enter' && ev.target.tagName !== 'TEXTAREA' && ev.target.tagName !== 'INPUT') {
      ev.preventDefault();
      runPlan();
    }
  });
}

async function init() {
  if (location.protocol === 'file:') {
    $('file-warning').style.display = 'block';
    return;
  }
  try {
    state.world = await api('/api/world');
  } catch (error) {
    $('file-warning').style.display = 'block';
    $('file-warning').textContent = `无法连接规划服务：${error.message}`;
    return;
  }
  const presets = $('presets');
  Object.entries(state.world.presets).forEach(([key, item]) => {
    const btn = document.createElement('button');
    btn.type = 'button';
    btn.dataset.preset = key;
    btn.textContent = item.name;
    btn.addEventListener('click', () => loadPreset(key));
    presets.appendChild(btn);
  });
  bind();
  syncOutputs();
  loadPreset('dock');
}

init();
