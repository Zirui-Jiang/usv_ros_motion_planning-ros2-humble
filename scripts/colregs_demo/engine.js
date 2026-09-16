/* Local educational prototype: A* global route -> rule gate -> ordinary LOS.
 * Coordinates: metres, yaw east=0 and counter-clockwise positive.
 * Vessel response is a bounded 2-D approximation, not the ROS PID/Gazebo model.
 */
(function (root, factory) {
  const api = factory();
  if (typeof module === 'object' && module.exports) module.exports = api;
  if (root) root.ColregsDemo = api;
})(typeof globalThis !== 'undefined' ? globalThis : this, function () {
  'use strict';
  const PI = Math.PI;
  const clamp = (n, a, b) => Math.max(a, Math.min(b, n));
  const wrap = a => ((a + PI) % (2 * PI) + 2 * PI) % (2 * PI) - PI;
  const dist = (a, b) => Math.hypot(a.x - b.x, a.y - b.y);
  const point = p => Array.isArray(p) ? {x: Number(p[0]), y: Number(p[1])} : {x: Number(p.x), y: Number(p.y)};
  const copyRoute = route => route.map(point);
  const velocity = s => ({x: s.speed * Math.cos(s.yaw), y: s.speed * Math.sin(s.yaw)});
  const DEFAULTS = Object.freeze({ownSpeed: 3, targetSpeed: 3, safetyDistance: 18,
    lookahead: 1, horizon: 20, rulesEnabled: true, targetCooperative: false,
    riskWindow: 90, predictionHorizon: 60, gridSize: 5, replanInterval: 0.5,
    routeDeviationWeight: 0.025, timeWeight: 0.25, searchCooldown: 5});
  const LOS_DEFAULTS = Object.freeze({trajectory_time_interval: 0.5, trajectory_time: 20,
    switch_dist: 5, goal_tolerance: 1, lookahead_distance: 5,
    yaw_rate_max: 0.3, kp_yaw: 1, slowdown_distance: 3});
  const SCENARIOS = [
    {id: 'head-on', title: '对遇', subtitle: '双方应向右转向', description: '两船近乎相反航向接近。规则区和方向边约束进入 A* 搜索，本船保持让清责任。', watch: '观察规则区、网格搜索航路、责任锁定及驶过后的接回。', rule: 'Rule 14 / 对遇'},
    {id: 'crossing-starboard', title: '右舷交叉', subtitle: '本船让路，避免横越船首', description: '目标船从本船右舷接近。A* 在预测冲突安全域和非对称规则约束下搜索船尾侧通路。', watch: '对照原物理 A* 航路与规则约束搜索航路，以及目标通过后的接回。', rule: 'Rule 15、16 / 交叉让路'},
    {id: 'crossing-port', title: '左舷交叉', subtitle: '先直航，必要时协助避碰', description: '本船起初为直航船。目标不配合时，按风险升级进入协助行动和紧迫危险阶段。', watch: '先保持，再降速；比较目标配合与不配合。', rule: 'Rule 17 / 直航船行动'},
    {id: 'overtaking', title: '本船追越', subtitle: '让清之前责任持续', defaults: {targetSpeed: 1.4}, description: '目标船默认速度为 1.4 m/s，形成可见的追越过程；追越方向采用右侧工程偏好。', watch: '航向改变不会取消追越责任，驶过并让清后回归。', rule: 'Rule 13 / 追越'},
    {id: 'being-overtaken', title: '本船被追越', subtitle: '保持航向航速并监测', defaults: {targetSpeed: 4.8}, description: '目标船默认速度为 4.8 m/s。其不采取让路行动时，本船在后期协助避碰。', watch: '对比目标船主动右侧追越和本船末期协助。', rule: 'Rule 13、17 / 被追越'},
    {id: 'clear', title: '无会遇风险', subtitle: '沿用原 A* 与 LOS', defaults: {targetSpeed: 2.4}, description: '目标船与本船保持足够横向间隔，不激活规则区和局部规则搜索。', watch: '全局航路与活动航路重合，LOS 输出连续参考轨迹。', rule: '无触发 / 常规航行'},
    {id: 'obstacle-crossing', title: '岸障与交叉会遇', subtitle: '物理约束与规则约束共同搜索', description: '八邻域 A* 同时检查静态岛障与规则区；LOS 输出再次接受岸障、规则、动态三项回验。', watch: '比较图搜索航路与 LOS 轨迹；无可行移动候选时明确制动及失败原因。', rule: 'Rule 15、16 + 静态约束'}
  ];

  /** Port of LosTrajectoryPlanner.plan's ordinary (non-circle) branch.
   * losPlan(route, state, cruisingSpeed, snakeCaseOptions) -> sample objects.
   * Internal forward Euler steps, waypoint switching and speed profile mirror
   * the Python implementation. Demo command=0 is handled outside this function.
   */
  function losPlan(path, state, cruisingSpeed, options = {}) {
    const cfg = {...LOS_DEFAULTS};
    if (options.lookahead !== undefined) cfg.lookahead_distance = options.lookahead;
    if (options.horizon !== undefined) cfg.trajectory_time = options.horizon;
    for (const key of Object.keys(LOS_DEFAULTS)) if (options[key] !== undefined) cfg[key] = Number(options[key]);
    for (const n of Object.values(cfg)) if (!Number.isFinite(n) || n <= 0) throw new RangeError('LOS settings must be positive and finite');
    if (!Array.isArray(path) || !path.length) throw new RangeError('LOS path is empty');
    const route = path.map(point).filter((p, i, a) => !i || dist(p, a[i - 1]) > 1e-9);
    if (route.some(p => !Number.isFinite(p.x + p.y))) throw new RangeError('LOS path must be finite');
    let x = Number(state.x), y = Number(state.y), yaw = wrap(Number(state.yaw ?? state.psi));
    if (!Number.isFinite(x + y + yaw) || !Number.isFinite(cruisingSpeed) || cruisingSpeed < 0) throw new RangeError('Invalid LOS state or speed');
    const dt = cfg.trajectory_time_interval, horizon = cfg.trajectory_time;
    if (horizon / dt > 100000 || horizon / 0.1 > 1000000) throw new RangeError('LOS rollout limit exceeded');
    const goal = route.at(-1), tolerance = cfg.goal_tolerance;
    const row = (t, s, curvature, speed, acceleration) => ({t, s, x, y, yaw, curvature, speed, acceleration});
    if (dist({x, y}, goal) <= tolerance) return [row(0, 0, 0, 0, 0), row(Math.min(dt, horizon), 0, 0, 0, 0)];
    if (cruisingSpeed === 0) throw new RangeError('Positive cruising speed required to move');
    const suffix = new Array(route.length).fill(0);
    for (let i = route.length - 2; i >= 0; i--) suffix[i] = suffix[i + 1] + dist(route[i], route[i + 1]);
    let index = 0, travelled = 0, elapsed = 0;
    const initialDistance = Math.max(dist({x, y}, route[Math.min(1, route.length - 1)]) + (route.length > 1 ? suffix[1] : 0), tolerance);
    const denominator = Math.max(initialDistance, cfg.slowdown_distance);
    let speed = cruisingSpeed * Math.min(1, initialDistance / denominator), reached = false;
    const rows = [row(0, 0, 0, speed, 0)];
    for (let n = 1; n <= Math.ceil(horizon / dt); n++) {
      const outputTime = Math.min(n * dt, horizon), previousSpeed = speed;
      let yawRate = 0, allowSwitch = true;
      while (elapsed < outputTime - 1e-12) {
        const h = Math.min(0.1, outputTime - elapsed);
        if (dist({x, y}, goal) <= tolerance) { reached = true; speed = 0; elapsed = outputTime; break; }
        let desiredYaw, remaining;
        if (route.length === 1) {
          desiredYaw = Math.atan2(goal.y - y, goal.x - x);
          remaining = dist({x, y}, goal);
        } else {
          const start = route[index], target = route[index + 1];
          const segmentYaw = Math.atan2(target.y - start.y, target.x - start.x);
          const crossTrack = -(x - start.x) * Math.sin(segmentYaw) + (y - start.y) * Math.cos(segmentYaw);
          desiredYaw = segmentYaw - Math.atan2(crossTrack, cfg.lookahead_distance);
          if (index === route.length - 2 && dist({x, y}, target) < cfg.switch_dist) desiredYaw = Math.atan2(target.y - y, target.x - x);
          remaining = dist({x, y}, target) + suffix[index + 1];
          if (allowSwitch && index < route.length - 2 && dist({x, y}, target) < cfg.switch_dist) index++;
        }
        allowSwitch = false;
        yawRate = clamp(cfg.kp_yaw * wrap(desiredYaw - yaw), -cfg.yaw_rate_max, cfg.yaw_rate_max);
        speed = cruisingSpeed * clamp(remaining / denominator, 0, 1);
        yaw = wrap(yaw + yawRate * h);
        const motion = speed * h;
        x += motion * Math.cos(yaw); y += motion * Math.sin(yaw);
        travelled += motion; elapsed += h;
      }
      rows.push(row(outputTime, travelled, speed > 1e-9 && !reached ? yawRate / speed : 0,
        speed, (speed - previousSpeed) / (outputTime - rows.at(-1).t)));
      if (reached) break;
    }
    return rows;
  }

  function segmentDistance(p, a, b) {
    const dx = b.x - a.x, dy = b.y - a.y;
    const t = clamp(((p.x - a.x) * dx + (p.y - a.y) * dy) / (dx * dx + dy * dy || 1), 0, 1);
    return dist(p, {x: a.x + t * dx, y: a.y + t * dy});
  }
  function freeSegment(a, b, obstacles, margin) {
    return obstacles.every(o => segmentDistance(o, a, b) > o.r + margin);
  }
  function insidePolygon(p, points) {
    let inside = false;
    for (let i = 0, j = points.length - 1; i < points.length; j = i++) {
      const a = points[i], b = points[j];
      if (segmentDistance(p, a, b) < 1e-8) return true;
      if ((a.y > p.y) !== (b.y > p.y) && p.x < (b.x - a.x) * (p.y - a.y) / (b.y - a.y) + a.x) inside = !inside;
    }
    return inside;
  }
  function segmentsIntersect(a, b, c, d) {
    const cross = (p, q, r) => (q.x - p.x) * (r.y - p.y) - (q.y - p.y) * (r.x - p.x);
    if (Math.max(a.x, b.x) < Math.min(c.x, d.x) - 1e-9 || Math.max(c.x, d.x) < Math.min(a.x, b.x) - 1e-9 ||
        Math.max(a.y, b.y) < Math.min(c.y, d.y) - 1e-9 || Math.max(c.y, d.y) < Math.min(a.y, b.y) - 1e-9) return false;
    return cross(a, b, c) * cross(a, b, d) <= 1e-9 && cross(c, d, a) * cross(c, d, b) <= 1e-9;
  }
  function zoneFree(a, b, zones, margin = 0) {
    return zones.every(zone => {
      const polygon = zone.points;
      if (insidePolygon(a, polygon) || insidePolygon(b, polygon)) return false;
      for (let i = 0; i < polygon.length; i++) {
        const c = polygon[i], d = polygon[(i + 1) % polygon.length];
        if (segmentsIntersect(a, b, c, d)) return false;
        if (margin > 0 && Math.min(segmentDistance(a, c, d), segmentDistance(b, c, d),
          segmentDistance(c, a, b), segmentDistance(d, a, b)) < margin) return false;
      }
      return true;
    });
  }
  function directionFree(a, b, directions = []) {
    return directions.every(rule => {
      const alongA = (a.x - rule.origin.x) * Math.cos(rule.yaw) + (a.y - rule.origin.y) * Math.sin(rule.yaw);
      const alongB = (b.x - rule.origin.x) * Math.cos(rule.yaw) + (b.y - rule.origin.y) * Math.sin(rule.yaw);
      const lateralA = -(a.x - rule.origin.x) * Math.sin(rule.yaw) + (a.y - rule.origin.y) * Math.cos(rule.yaw);
      const lateralB = -(b.x - rule.origin.x) * Math.sin(rule.yaw) + (b.y - rule.origin.y) * Math.cos(rule.yaw);
      const leftward = -(b.x - a.x) * Math.sin(rule.yaw) + (b.y - a.y) * Math.cos(rule.yaw);
      return Math.max(alongA, alongB) < rule.from || Math.min(alongA, alongB) > rule.until ||
        Math.max(lateralA, lateralB) < (rule.minLateral ?? -Infinity) || leftward <= 1e-8;
    });
  }
  function routeDistance(p, route) {
    return route.length < 2 ? 0 : Math.min(...route.slice(1).map((b, i) => segmentDistance(p, route[i], b)));
  }
  /** Nonnegative graph cost: ds + w[1/m] * deviation[m] * ds[m]. */
  function routeCost(route, globalRoute = [], deviationWeight = 0.025) {
    let length = 0, routeDeviation = 0;
    for (let i = 1; i < route.length; i++) {
      const a = route[i - 1], b = route[i], ds = dist(a, b);
      length += ds;
      routeDeviation += Math.max(0, deviationWeight) * routeDistance({x: (a.x + b.x) * 0.5, y: (a.y + b.y) * 0.5}, globalRoute) * ds;
    }
    return {length, routeDeviation, total: length + routeDeviation};
  }
  /** The optimality claim is restricted to this finite 8-neighbour graph.
   * Every edge is checked against physical obstacles, virtual polygons and
   * directed rule constraints. Euclidean h is admissible because g >= length.
   * Only collinear vertices are removed; no unconstrained shortcut smoothing.
   */
  function searchAstar(start, goal, options = {}) {
    const {obstacles = [], zones = [], directions = [], globalRoute = [], gridSize = 5,
      margin = 11, zoneMargin = 8, deviationWeight = 0.025} = options;
    const result = {route: [], rawRoute: [], expanded: 0, closed: [], costBreakdown: {length: 0, routeDeviation: 0, total: Infinity}, status: '无可行网格航路'};
    if (![start, goal].every(p => Number.isFinite(p.x + p.y) && Math.abs(p.x) <= 155 && Math.abs(p.y) <= 105)) return result;
    const nx = Math.floor(310 / gridSize) + 1, ny = Math.floor(210 / gridSize) + 1;
    const idAt = p => clamp(Math.round((p.y + 105) / gridSize), 0, ny - 1) * nx + clamp(Math.round((p.x + 155) / gridSize), 0, nx - 1);
    const begin = idAt(start), end = idAt(goal);
    const pos = id => id === begin ? point(start) : id === end ? point(goal) : {x: (id % nx) * gridSize - 155, y: Math.floor(id / nx) * gridSize - 105};
    const edgeFree = (a, b) => freeSegment(a, b, obstacles, margin) && zoneFree(a, b, zones, zoneMargin) && directionFree(a, b, directions);
    const blocked = new Uint8Array(nx * ny);
    for (let id = 0; id < blocked.length; id++) blocked[id] = edgeFree(pos(id), pos(id)) ? 0 : 1;
    if (blocked[begin] || blocked[end] || !edgeFree(goal, goal)) return result;
    if (begin === end) return edgeFree(start, goal) ? {...result, route: [point(start), point(goal)], rawRoute: [point(start), point(goal)], expanded: 1,
      closed: [point(start)], status: '搜索成功', costBreakdown: routeCost([start, goal], globalRoute, deviationWeight)} : result;
    const cost = new Float64Array(nx * ny).fill(Infinity), previous = new Int32Array(nx * ny).fill(-1);
    const open = new Set([begin]), closed = new Uint8Array(nx * ny);
    cost[begin] = 0;
    while (open.size) {
      let current = -1, score = Infinity;
      for (const id of open) {
        const f = cost[id] + dist(pos(id), goal);
        if (f < score - 1e-10) { score = f; current = id; }
      }
      open.delete(current); closed[current] = 1; result.expanded++;
      result.closed.push(pos(current));
      if (current === end) {
        for (let id = end; id !== -1; id = previous[id]) result.rawRoute.push(pos(id));
        result.rawRoute.reverse();
        result.route = [result.rawRoute[0]];
        for (let i = 1; i < result.rawRoute.length - 1; i++) {
          const a = result.rawRoute[i - 1], b = result.rawRoute[i], c = result.rawRoute[i + 1];
          const cross = (b.x - a.x) * (c.y - b.y) - (b.y - a.y) * (c.x - b.x);
          if (Math.abs(cross) > 1e-8 || !edgeFree(result.route.at(-1), c)) result.route.push(b);
        }
        result.route.push(result.rawRoute.at(-1));
        result.costBreakdown = routeCost(result.rawRoute, globalRoute, deviationWeight);
        result.costBreakdown.total = cost[end];
        result.status = '搜索成功';
        return result;
      }
      const cx = current % nx, cy = Math.floor(current / nx), a = pos(current);
      for (let dy = -1; dy <= 1; dy++) for (let dx = -1; dx <= 1; dx++) {
        if ((!dx && !dy) || cx + dx < 0 || cx + dx >= nx || cy + dy < 0 || cy + dy >= ny) continue;
        const next = (cy + dy) * nx + cx + dx;
        if (blocked[next] || closed[next]) continue;
        if (dx && dy && (blocked[cy * nx + cx + dx] || blocked[(cy + dy) * nx + cx])) continue;
        const b = pos(next);
        if (!edgeFree(a, b)) continue;
        const candidate = cost[current] + routeCost([a, b], globalRoute, deviationWeight).total;
        if (candidate < cost[next] - 1e-10) { cost[next] = candidate; previous[next] = current; open.add(next); }
      }
    }
    return result;
  }
  function astar(start, goal, obstacles = [], gridSize = 5, margin = 11) {
    return searchAstar(start, goal, {obstacles, gridSize, margin, globalRoute: [start, goal], deviationWeight: 0}).route;
  }
  function validateTrajectory(trajectory, options = {}) {
    const {obstacles = [], zones = [], target = null, safetyDistance = 18, directions = []} = options;
    if (!Array.isArray(trajectory) || !trajectory.length || trajectory.some((p, i) => !Number.isFinite(p.x + p.y + (p.t ?? 0)) ||
      (p.t ?? 0) < 0 || (i > 0 && (p.t ?? 0) < (trajectory[i - 1].t ?? 0)))) {
      return {staticSafe: false, ruleSafe: false, dynamicSafe: false, minDistance: NaN, reason: '轨迹坐标 / 时间无效或时间倒序'};
    }
    let staticSafe = trajectory.length > 0, ruleSafe = trajectory.length > 0, minDistance = Infinity;
    const tv = target ? velocity(target) : {x: 0, y: 0};
    for (let i = 0; i < trajectory.length; i++) {
      const b = trajectory[i], a = trajectory[Math.max(0, i - 1)];
      if (!freeSegment(a, b, obstacles, 7)) staticSafe = false;
      if (!zoneFree(a, b, zones) || !directionFree(a, b, directions)) ruleSafe = false;
      if (target) {
        const ta = a.t || 0, tb = b.t || 0;
        const ra = {x: a.x - target.x - tv.x * ta, y: a.y - target.y - tv.y * ta};
        const rb = {x: b.x - target.x - tv.x * tb, y: b.y - target.y - tv.y * tb};
        minDistance = Math.min(minDistance, segmentDistance({x: 0, y: 0}, ra, rb));
      }
    }
    const dynamicSafe = minDistance >= safetyDistance;
    return {staticSafe, ruleSafe, dynamicSafe, minDistance,
      reason: !staticSafe ? 'LOS 未通过物理岸障检查' : !ruleSafe ? 'LOS 未通过虚拟规则区 / 方向检查' : !dynamicSafe ? 'LOS 预测动态间距不足' : '岸障、规则与动态三项回验通过'};
  }
  function remainingRoute(route, own) {
    if (route.length < 2) return route;
    let best = Infinity, index = 0;
    for (let i = 0; i < route.length - 1; i++) {
      const d = segmentDistance(own, route[i], route[i + 1]);
      if (d < best - 1e-6) { best = d; index = i; }
    }
    if (index < route.length - 2 && dist(own, route[index + 1]) < 5) index++;
    return route.slice(index);
  }
  function cpa(own, target) {
    if (!target) return {distance: Infinity, dcpa: Infinity, tcpa: null, closing: false};
    const r = {x: target.x - own.x, y: target.y - own.y}, ov = velocity(own), tv = velocity(target);
    const v = {x: tv.x - ov.x, y: tv.y - ov.y}, v2 = v.x * v.x + v.y * v.y;
    const dot = r.x * v.x + r.y * v.y;
    const tcpa = v2 > 1e-8 ? -dot / v2 : null;
    return {distance: Math.hypot(r.x, r.y), dcpa: tcpa === null ? Math.hypot(r.x, r.y) : Math.hypot(r.x + v.x * tcpa, r.y + v.y * tcpa), tcpa, closing: dot < -0.05};
  }
  function classify(sim) {
    if (!sim.target) return 'clear';
    const a = sim.own, b = sim.target, bearing = wrap(Math.atan2(b.y - a.y, b.x - a.x) - a.yaw);
    const heading = Math.abs(wrap(a.yaw - b.yaw));
    const fromTarget = wrap(Math.atan2(a.y - b.y, a.x - b.x) - b.yaw);
    // Rule 13: approach from more than 22.5 degrees abaft the beam.
    // The speed projection is an engineering check that the rear vessel gains.
    const gainsOnTarget = a.speed * Math.cos(wrap(a.yaw - b.yaw)) > b.speed + 0.05;
    const targetGains = b.speed * Math.cos(wrap(b.yaw - a.yaw)) > a.speed + 0.05;
    if (Math.abs(fromTarget) > 112.5 * PI / 180 && gainsOnTarget) return 'overtaking';
    if (Math.abs(bearing) > 112.5 * PI / 180 && targetGains) return 'being-overtaken';
    if (heading > 5 * PI / 6 && Math.abs(bearing) < PI / 7) return 'head-on';
    return bearing < 0 ? 'crossing-starboard' : 'crossing-port';
  }
  function encounterText(kind) {
    return ({'head-on': '对遇', 'crossing-starboard': '右舷交叉', 'crossing-port': '左舷交叉', overtaking: '本船追越', 'being-overtaken': '本船被追越', clear: '无会遇风险'})[kind];
  }
  function role(kind) { return ['head-on', 'crossing-starboard', 'overtaking'].includes(kind) ? '让路船' : kind === 'clear' ? '常规航行' : '直航船'; }
  function ruleFor(kind) { return ({'head-on': 'Rule 14', 'crossing-starboard': 'Rule 15 / 16', 'crossing-port': 'Rule 17', overtaking: 'Rule 13', 'being-overtaken': 'Rule 13 / 17', clear: '常规航行'})[kind]; }
  function log(sim, text, type = 'info') { sim.logs.push({time: sim.time, text, type}); if (sim.logs.length > 80) sim.logs.shift(); }
  function released(sim, memory, m) {
    const own = sim.own, target = sim.target, safe = sim.config.safetyDistance;
    if (!target || sim.time - memory.since < 8 || m.distance < safe * 1.6) return false;
    const along = (own.x - target.x) * Math.cos(memory.ownYaw) + (own.y - target.y) * Math.sin(memory.ownYaw);
    if (memory.kind === 'overtaking') return along > safe * 2;
    if (memory.kind === 'being-overtaken') return along < -safe * 2;
    if (memory.kind === 'head-on') return along > safe && !m.closing;
    const targetCross = -(target.x - memory.origin.x) * Math.sin(memory.ownYaw) + (target.y - memory.origin.y) * Math.cos(memory.ownYaw);
    return (targetCross * memory.initialTargetSide < 0 && Math.abs(targetCross) > safe * 2) || (along > safe * 2 && !m.closing);
  }
  function rectangle(origin, yaw, front0, front1, left0, left1) {
    return [[front0, left0], [front1, left0], [front1, left1], [front0, left1]].map(([x, y]) =>
      ({x: origin.x + x * Math.cos(yaw) - y * Math.sin(yaw), y: origin.y + x * Math.sin(yaw) + y * Math.cos(yaw)}));
  }
  function buildRuleConstraints(sim) {
    const memory = sim._memory, own = sim.own, target = sim.target, safety = sim.config.safetyDistance;
    const origin = point(own), yaw = memory.ownYaw, metrics = cpa(own, target);
    const t = clamp(metrics.tcpa ?? 0, 0, sim.config.predictionHorizon);
    const ov = velocity(own), tv = velocity(target);
    const conflict = {x: (own.x + ov.x * t + target.x + tv.x * t) * 0.5,
      y: (own.y + ov.y * t + target.y + tv.y * t) * 0.5};
    const along = (conflict.x - origin.x) * Math.cos(yaw) + (conflict.y - origin.y) * Math.sin(yaw);
    const lateral = -(conflict.x - origin.x) * Math.sin(yaw) + (conflict.y - origin.y) * Math.cos(yaw);
    const begin = Math.max(25, along - safety * 2), end = Math.max(begin + safety * 2, along + safety * 2.5);
    const reason = '由当前相对位置、速度和 CPA 冲突点建立，随 safetyDistance 缩放；本轮冻结的有限时域二维工程约束。';
    sim.ruleZones = [
      {id: `safety-${sim._ruleVersion + 1}`, kind: 'safety', label: '预测冲突安全域',
        points: rectangle(origin, yaw, begin, end, lateral - safety * 1.8, lateral + safety * 1.8), reason},
      {id: `rule-${sim._ruleVersion + 1}`, kind: 'rule', label: memory.kind === 'overtaking' ? '右侧追越工程偏好' : memory.kind === 'crossing-starboard' ? '避免横越船首侧' : '抑制左绕的规则区',
        points: rectangle(origin, yaw, begin, end, lateral, lateral + safety * 3.5),
        reason: memory.kind === 'overtaking' ? 'Rule 13 要求追越让清；右侧是本演示的工程选择，矩形及方向不属于条文规定。' : '将本轮右转 / 避免船首通过意图转换为非对称禁止区；矩形形状和范围属于工程近似。'}
    ];
    // Keep the direction guard local to the predicted conflict lane. Ordinary
    // LOS can make small cross-track corrections before/after that lane.
    sim._directions = [{origin, yaw, from: begin, until: end + 5, minLateral: lateral - safety * 1.8,
      label: memory.kind === 'overtaking' ? '预测冲突航段限制左向边（右侧追越工程选择）' : '预测冲突航段限制左向边；已在安全域右侧允许 LOS 横向修正'}];
    sim._constraintEnd = {origin, yaw, end};
    sim._ruleVersion++;
  }
  function stoppedTrajectory(own, horizon) {
    const stopTime = own.speed / 0.7;
    const rows = [];
    for (let t = 0; t <= horizon; t += 0.5) {
      const a = Math.min(t, stopTime), s = own.speed * a - 0.35 * a * a;
      rows.push({t, x: own.x + s * Math.cos(own.yaw), y: own.y + s * Math.sin(own.yaw), yaw: own.yaw,
        speed: Math.max(0, own.speed - 0.7 * t), s, curvature: 0, acceleration: -0.7});
    }
    return rows;
  }
  function evaluate(sim, route, speed, search = null) {
    if (!route.length) return {route, trajectory: [], safe: false, chosen: false, speed, minDistance: 0, staticSafe: false, ruleSafe: false, dynamicSafe: false,
      reason: '规则约束图中没有可行航路', cost: Infinity, costBreakdown: {length: 0, routeDeviation: 0, total: Infinity, timePenalty: Infinity}, search};
    const trajectory = speed > 0 ? losPlan(remainingRoute(route, sim.own), sim.own, speed,
      {lookahead: sim.config.lookahead, horizon: sim.config.predictionHorizon}) : stoppedTrajectory(sim.own, sim.config.predictionHorizon);
    const validation = validateTrajectory(trajectory, {obstacles: sim.obstacles, zones: sim.ruleZones,
      directions: sim._directions, target: sim.target, safetyDistance: sim.config.safetyDistance});
    const costBreakdown = routeCost(route, sim.globalRoute, sim.config.routeDeviationWeight);
    costBreakdown.timePenalty = sim.config.timeWeight * (speed > 0 ? costBreakdown.length / speed : sim.config.riskWindow * 5);
    return {route, trajectory, safe: validation.staticSafe && validation.ruleSafe && validation.dynamicSafe, chosen: false, speed,
      ...validation, costBreakdown, cost: costBreakdown.total + costBreakdown.timePenalty, search};
  }
  function rejoinPlan(sim, forward = 40) {
    const route = remainingRoute(sim.globalRoute, sim.own);
    if (route.length < 2) return {point: point(sim.goal), suffix: []};
    const dx = route[1].x - route[0].x, dy = route[1].y - route[0].y;
    const t = clamp(((sim.own.x - route[0].x) * dx + (sim.own.y - route[0].y) * dy) / (dx * dx + dy * dy || 1), 0, 1);
    let rejoin = {x: route[0].x + t * dx, y: route[0].y + t * dy}, index = 0;
    while (index < route.length - 1) {
      const length = dist(rejoin, route[index + 1]);
      if (length > forward) {
        const f = forward / length;
        rejoin = {x: rejoin.x + f * (route[index + 1].x - rejoin.x), y: rejoin.y + f * (route[index + 1].y - rejoin.y)};
        break;
      }
      forward -= length; rejoin = point(route[++index]);
    }
    return {point: rejoin, suffix: copyRoute(route.slice(index + 1))};
  }
  function searchRoutes(sim, trigger, extraMargin = 0) {
    let forward = 40;
    if (sim._constraintEnd) {
      const c = sim._constraintEnd;
      const ownAlong = (sim.own.x - c.origin.x) * Math.cos(c.yaw) + (sim.own.y - c.origin.y) * Math.sin(c.yaw);
      forward = Math.max(60, c.end - ownAlong + 50);
    }
    let reconnect = rejoinPlan(sim, forward);
    // A reconnect inside a virtual region is not silently forced free.
    while (!zoneFree(reconnect.point, reconnect.point, sim.ruleZones, 25 + extraMargin) && forward < 500) {
      forward += 20; reconnect = rejoinPlan(sim, forward);
    }
    const options = [];
    for (const clearance of sim.ruleZones.length ? [12 + extraMargin, 20 + extraMargin] : [11]) {
      const search = searchAstar(sim.own, reconnect.point, {obstacles: sim.obstacles, zones: sim.ruleZones,
        directions: sim._directions, globalRoute: sim.globalRoute, gridSize: sim.config.gridSize,
        margin: Math.max(11, clearance), zoneMargin: clearance, deviationWeight: sim.config.routeDeviationWeight});
      sim._searchCount++;
      const route = search.route.length ? [...search.route, ...reconnect.suffix].filter((p, i, all) => !i || dist(p, all[i - 1]) > 1e-9) : [];
      options.push({route, search, clearance});
    }
    sim._searchCache = {options, time: sim.time, targetYaw: sim.target?.yaw, targetSpeed: sim.target?.speed,
      extraMargin, reconnect, trigger};
    sim.searchInfo = {algorithm: sim.ruleZones.length ? '规则约束 A*' : '原航路接回 A*', expanded: options.reduce((n, c) => n + c.search.expanded, 0),
      searchCount: sim._searchCount, ruleVersion: sim._ruleVersion, trigger, status: options.some(o => o.route.length) ? '搜索成功' : '无可行网格航路',
      rejoin: point(reconnect.point), gridSize: sim.config.gridSize, closed: options.flatMap(c => c.search.closed), rawRoute: options[0]?.search.rawRoute || [],
      costBreakdown: options[0]?.search.costBreakdown || {length: 0, routeDeviation: 0, total: Infinity},
      activeConstraints: [...sim.ruleZones.map(z => `${z.label}：${z.reason}`), ...sim._directions.map(d => d.label),
        `搜索额外净空 ${options.map(o => o.clearance).join('/')} m；LOS 对原规则边界独立回验`,
        `g = 距离 + ${sim.config.routeDeviationWeight} (1/m) × 原路偏离 × 边长；h = 欧氏距离` ]};
    log(sim, `${trigger}：执行 ${options.length} 次局部 A*，展开 ${sim.searchInfo.expanded} 个结点。`, 'search');
  }
  function candidatesFromCache(sim, desiredSpeed) {
    const speeds = [...new Set([desiredSpeed, desiredSpeed * 0.65, desiredSpeed * 0.35])];
    const candidates = [];
    for (const option of sim._searchCache.options) for (const speed of speeds) {
      candidates.push({...evaluate(sim, option.route, speed, option.search), label: `A* 净空 ${option.clearance} m / ${speed.toFixed(2)} m/s`});
    }
    return candidates;
  }
  function replan(sim) {
    if (sim.complete || sim.collision) return sim;
    const cfg = sim.config, m = cpa(sim.own, sim.target);
    sim.metrics = {...sim.metrics, ...m, crossTrack: routeDistance(sim.own, sim.globalRoute), minDistance: Math.min(sim.metrics.minDistance, m.distance)};
    const threat = m.tcpa !== null && m.tcpa >= 0 && m.tcpa <= cfg.riskWindow && m.dcpa < cfg.safetyDistance * 1.6;
    if (!sim._memory && threat && sim.target && !sim._finishedEncounter) {
      const kind = classify(sim);
      sim._memory = {kind, since: sim.time, ownYaw: sim.own.yaw, origin: point(sim.own),
        initialTargetSide: Math.sign(-(sim.target.x - sim.own.x) * Math.sin(sim.own.yaw) + (sim.target.y - sim.own.y) * Math.cos(sim.own.yaw)) || 1,
        acted: false};
      log(sim, `识别${encounterText(kind)}：责任保持至驶过并让清。`, 'rule');
    }
    if (sim._memory && released(sim, sim._memory, m)) {
      log(sim, '满足驶过与让清条件：清除本轮规则区，搜索前向原航路接回点。', 'success');
      sim._memory = null; sim._finishedEncounter = true; sim._recovering = cfg.rulesEnabled;
      sim.ruleZones = []; sim._directions = []; sim._constraintEnd = null; sim._searchCache = null; sim._ruleVersion++;
    }
    const memory = sim._memory, kind = memory ? memory.kind : 'clear';
    let phase = sim._recovering ? '恢复航路' : '常规航行', action = '沿原全局航路输出 LOS', reason = '当前未触发 CPA 风险预警。';
    let desiredSpeed = cfg.ownSpeed, mustAct = false, constrained = false;
    if (memory && cfg.rulesEnabled) {
      if (role(kind) === '直航船') {
        mustAct = m.distance < cfg.safetyDistance * 2.2 || (m.tcpa !== null && m.tcpa > 0 && m.tcpa < 10 && m.dcpa < cfg.safetyDistance);
        const mayAct = threat && ((m.tcpa !== null && m.tcpa < 22) || m.distance < cfg.safetyDistance * 3);
        if (memory.acted || mayAct || mustAct) {
          memory.acted = true; phase = mustAct ? '紧迫危险 · 协助避碰' : '直航船协助行动';
          action = '启用协助避碰约束并搜索可行航路';
          reason = '目标持续逼近，工程风险阈值触发 Rule 17 协助行动。';
          if (kind !== 'being-overtaken') desiredSpeed = Math.min(cfg.ownSpeed, 0.8);
        } else {
          phase = '直航监测'; action = '保持航向航速，监测让路船'; reason = '直航船尚未达到协助行动工程阈值。';
        }
      } else {
        memory.acted = true; phase = '保持避让责任'; action = '按规则区和方向边约束执行 A* 搜索航路';
        reason = kind === 'overtaking' ? '追越责任保持；右侧通行属于本演示工程偏好。' : '冻结本轮相对态势形成规则区；由搜索确定航路形状。';
      }
      constrained = memory.acted;
    }
    if (!cfg.rulesEnabled) {
      sim.ruleZones = []; sim._directions = []; sim._constraintEnd = null; sim._searchCache = null;
      phase = '基线 A* + LOS'; action = '沿原物理 A* 航路输出 LOS'; reason = '关闭规则约束；保留物理障碍与 CPA / 动态风险显示。';
    }
    let chosen;
    if (cfg.rulesEnabled && (constrained || sim._recovering)) {
      if (!sim._searchCache) {
        if (constrained) buildRuleConstraints(sim);
        searchRoutes(sim, constrained ? '首次避让约束激活' : '驶过让清：前向接回原航路');
      } else if (constrained && sim.time - sim._searchCache.time >= cfg.searchCooldown && sim.target &&
        (Math.abs(wrap(sim.target.yaw - sim._searchCache.targetYaw)) > 0.35 || Math.abs(sim.target.speed - sim._searchCache.targetSpeed) > 0.75)) {
        buildRuleConstraints(sim); searchRoutes(sim, '目标航向 / 航速显著改变');
      } else {
        sim.searchInfo.status = '沿用已搜索航路；每 0.5 s 独立 LOS 回验';
      }
      sim.candidates = candidatesFromCache(sim, desiredSpeed);
      chosen = sim.candidates.filter(c => c.safe && c.speed > 0).sort((a, b) => a.cost - b.cost)[0];
      if (!chosen && sim.time - sim._searchCache.time >= cfg.searchCooldown) {
        const extra = Math.min(24, sim._searchCache.extraMargin + 4);
        searchRoutes(sim, `LOS 回验失败：冷却 ${cfg.searchCooldown}s 后提高搜索净空`, extra);
        sim.candidates = candidatesFromCache(sim, desiredSpeed);
        chosen = sim.candidates.filter(c => c.safe && c.speed > 0).sort((a, b) => a.cost - b.cost)[0];
      }
      if (!chosen) {
        const stopped = evaluate(sim, sim.activeRoute.length ? sim.activeRoute : sim._searchCache.options.find(c => c.route.length)?.route || sim.globalRoute, 0);
        stopped.label = '无可行移动候选：制动等待'; sim.candidates.push(stopped); chosen = stopped;
        phase = '无可行候选 · 制动'; action = '制动并报告候选回验结果';
        reason = '移动候选未同时通过岸障、规则、动态回验；制动也不构成安全保证。';
        sim.searchInfo.status = 'LOS 回验无可行移动候选；等待事件 / 搜索冷却';
      }
      if (chosen.search) {
        sim.searchInfo.rawRoute = chosen.search.rawRoute;
        sim.searchInfo.costBreakdown = chosen.search.costBreakdown;
      }
    } else {
      chosen = evaluate(sim, sim.globalRoute, desiredSpeed);
      chosen.label = cfg.rulesEnabled ? '原全局航路（无需规则干预）' : '物理 A* 基线'; sim.candidates = [chosen];
      sim.searchInfo = {algorithm: '物理 A* 基线', expanded: sim._globalSearch.expanded, searchCount: sim._searchCount,
        ruleVersion: sim._ruleVersion, trigger: '无规则搜索事件', status: cfg.rulesEnabled ? '常规 / 直航监测：沿用原航路' : '规则关闭：物理 A* 基线',
        rejoin: null, gridSize: cfg.gridSize, closed: sim._globalSearch.closed, rawRoute: sim._globalSearch.rawRoute,
        costBreakdown: sim._globalSearch.costBreakdown, activeConstraints: ['物理障碍膨胀与八邻域可通航边']};
      if (!chosen.staticSafe && cfg.rulesEnabled) {
        chosen = {...evaluate(sim, sim.globalRoute, 0), label: '原 LOS 静态回验失败：制动'};
        sim.candidates.push(chosen); phase = '无可行候选 · 制动';
      }
    }
    chosen.chosen = true;
    if (cfg.rulesEnabled && memory?.kind === 'overtaking' && chosen.speed > 0 && chosen.speed <= sim.target.speed) {
      phase = '降速跟随 · 追越尚未完成'; action = '采用已通过回验的低速候选，继续保持追越责任';
      reason = '当前最小代价安全候选速度不高于目标船，不能取得追越净距；未认定驶过让清。';
    }
    sim.activeRoute = copyRoute(chosen.route); sim.speedLimit = chosen.speed;
    const remaining = remainingRoute(sim.activeRoute, sim.own);
    sim.trajectory = remaining.length && chosen.speed > 0 ? losPlan(remaining, sim.own, chosen.speed,
      {lookahead: cfg.lookahead, horizon: cfg.horizon}) : stoppedTrajectory(sim.own, cfg.horizon);
    sim.routeValidation = {staticSafe: chosen.staticSafe, ruleSafe: chosen.ruleSafe, dynamicSafe: chosen.dynamicSafe,
      reason: chosen.reason, minDistance: chosen.minDistance};
    sim.decision = {encounter: encounterText(kind), role: role(kind), phase, action, reason, rule: ruleFor(kind),
      risk: threat || !!memory, mustAct, feasible: chosen.safe};
    if (phase !== sim._lastPhase) { log(sim, `${phase}：${action}`, phase.includes('无可行') || mustAct ? 'warning' : 'info'); sim._lastPhase = phase; }
    sim._lastPlanTime = sim.time;
    return sim;
  }

  function createSimulation(scenarioId = 'head-on', params = {}, overrides = {}) {
    if (!SCENARIOS.some(s => s.id === scenarioId)) throw new RangeError(`Unknown scenario: ${scenarioId}`);
    const config = {...DEFAULTS, ...SCENARIOS.find(s => s.id === scenarioId).defaults, ...params};
    for (const key of ['ownSpeed', 'targetSpeed']) config[key] = clamp(Number(config[key]) || 0, 0, 8);
    config.safetyDistance = clamp(Number(config.safetyDistance) || DEFAULTS.safetyDistance, 8, 45);
    config.lookahead = clamp(Number(config.lookahead) || DEFAULTS.lookahead, 0.3, 30);
    config.horizon = clamp(Number(config.horizon) || DEFAULTS.horizon, 5, 90);
    config.riskWindow = clamp(Number(config.riskWindow) || DEFAULTS.riskWindow, 10, 180);
    config.predictionHorizon = clamp(Number(config.predictionHorizon) || DEFAULTS.predictionHorizon, 20, 90);
    config.gridSize = clamp(Number(config.gridSize) || DEFAULTS.gridSize, 2, 10);
    config.replanInterval = clamp(Number(config.replanInterval) || DEFAULTS.replanInterval, 0.1, 2);
    config.routeDeviationWeight = clamp(Number.isFinite(Number(config.routeDeviationWeight)) ? Number(config.routeDeviationWeight) : DEFAULTS.routeDeviationWeight, 0, 1);
    config.timeWeight = clamp(Number.isFinite(Number(config.timeWeight)) ? Number(config.timeWeight) : DEFAULTS.timeWeight, 0, 10);
    config.searchCooldown = clamp(Number(config.searchCooldown) || DEFAULTS.searchCooldown, 2, 30);
    const vessel = (x, y, yaw, speed) => ({x, y, yaw, speed, length: 10, width: 4, yawRate: 0});
    let own = vessel(-125, 0, 0, config.ownSpeed), target = vessel(105, 0, PI, config.targetSpeed);
    const obstacles = [];
    if (scenarioId === 'crossing-starboard') target = vessel(-20, -105, PI / 2, config.targetSpeed);
    if (scenarioId === 'crossing-port') target = vessel(-20, 105, -PI / 2, config.targetSpeed);
    if (scenarioId === 'overtaking') target = vessel(-80, 0, 0, config.targetSpeed);
    if (scenarioId === 'being-overtaken') { own = vessel(-75, 0, 0, config.ownSpeed); target = vessel(-145, 0, 0, config.targetSpeed); }
    if (scenarioId === 'clear') target = vessel(-100, 75, 0, config.targetSpeed);
    if (scenarioId === 'obstacle-crossing') { target = vessel(-20, -105, PI / 2, config.targetSpeed); obstacles.push({x: 5, y: 3, r: 23}); }
    own = {...own, ...(overrides.own || {})};
    target = overrides.target === null ? null : {...target, ...(overrides.target || {})};
    const goal = {...{x: 145, y: 0}, ...(overrides.goal || {})};
    const actualObstacles = overrides.obstacles ? overrides.obstacles.map(o => ({...o})) : obstacles;
    const globalSearch = searchAstar(own, goal, {obstacles: actualObstacles, gridSize: config.gridSize, deviationWeight: 0});
    const globalRoute = globalSearch.route;
    const sim = {scenarioId, config, time: 0, own, target, goal, obstacles: actualObstacles, globalRoute, activeRoute: [], trajectory: [], candidates: [],
      ruleZones: [], searchInfo: {}, routeValidation: {},
      decision: {}, metrics: {distance: Infinity, dcpa: Infinity, tcpa: null, minDistance: Infinity, crossTrack: 0}, logs: [],
      history: {own: [{...point(own), t: 0}], target: target ? [{...point(target), t: 0}] : []}, complete: false, collision: false,
      meta: {version: '2.0', effectiveOwnSpeed: own.speed, effectiveTargetSpeed: target?.speed ?? 0, controller: '有界二维速度 / 转向响应', globalPlanner: '8 邻域二值栅格 A*'},
      _initialOwn: {...own}, _initialTarget: target ? {...target} : null, _memory: null, _lastPlanTime: -Infinity, _lastHistoryTime: 0,
      _finishedEncounter: false, _recovery: null, _recovering: false, _ruleVersion: 0, _searchCount: 0,
      _searchCache: null, _directions: [], _constraintEnd: null, _globalSearch: globalSearch};
    log(sim, '已建立全局航路；规则区域和方向约束将真实作用于 A* 搜索，再由原 LOS 生成轨迹。');
    return replan(sim);
  }
  function advanceVessel(vessel, speedCommand, yawCommand, feedforward, dt) {
    const acceleration = clamp((speedCommand - vessel.speed) / 1.0, -0.7, 0.6);
    vessel.speed = Math.max(0, vessel.speed + acceleration * dt);
    const rateCommand = clamp(feedforward + 1.5 * wrap(yawCommand - vessel.yaw), -0.3, 0.3);
    vessel.yawRate = (vessel.yawRate || 0) + (rateCommand - (vessel.yawRate || 0)) * Math.min(1, dt / 0.3);
    const middleYaw = vessel.yaw + vessel.yawRate * dt * 0.5;
    vessel.x += vessel.speed * Math.cos(middleYaw) * dt;
    vessel.y += vessel.speed * Math.sin(middleYaw) * dt;
    vessel.yaw = wrap(vessel.yaw + vessel.yawRate * dt);
  }
  function advanceTarget(sim, dt) {
    if (!sim.target) return;
    let speed = sim._initialTarget.speed, yaw = sim._initialTarget.yaw;
    const m = sim._memory;
    if (sim.config.targetCooperative && m) {
      if (m.kind === 'head-on') yaw -= 0.42;
      if (m.kind === 'crossing-port') speed = 0;
      if (m.kind === 'being-overtaken') {
        const lateral = sim.target.y - sim._initialTarget.y;
        yaw = lateral > -sim.config.safetyDistance * 2.5 ? sim._initialTarget.yaw - 0.5 : sim._initialTarget.yaw;
      }
    }
    advanceVessel(sim.target, speed, yaw, 0, dt);
  }
  function step(sim, dt = 0.1) {
    if (!Number.isFinite(dt) || dt <= 0) return sim;
    const count = Math.ceil(Math.min(dt, 5) / 0.1), h = Math.min(dt, 5) / count;
    for (let n = 0; n < count && !sim.complete && !sim.collision; n++) {
      if (sim.time - sim._lastPlanTime >= sim.config.replanInterval - 1e-8) replan(sim);
      const elapsed = sim.time - sim._lastPlanTime;
      const reference = sim.trajectory.find(p => p.t >= elapsed + 0.2) || sim.trajectory.at(-1);
      const yaw = reference ? reference.yaw : sim.own.yaw;
      const feedforward = reference && sim.speedLimit > 0 ? reference.curvature * reference.speed : 0;
      const goalDistance = dist(sim.own, sim.goal);
      const terminalCap = Math.sqrt(2 * 0.45 * Math.max(0, goalDistance - 1));
      const terminalAlignment = goalDistance < 12 ? Math.max(0.15, Math.cos(wrap(Math.atan2(sim.goal.y - sim.own.y, sim.goal.x - sim.own.x) - sim.own.yaw))) : 1;
      const referenceSpeed = reference?.speed ?? 0;
      advanceVessel(sim.own, Math.min(sim.speedLimit, referenceSpeed, terminalCap * terminalAlignment), yaw, feedforward, h);
      advanceTarget(sim, h);
      sim.time += h;
      const m = cpa(sim.own, sim.target);
      sim.metrics = {...sim.metrics, ...m, minDistance: Math.min(sim.metrics.minDistance, m.distance)};
      const collisionRadius = sim.target ? (sim.own.length + sim.target.length) * 0.45 : 0;
      sim.collision = m.distance < collisionRadius || sim.obstacles.some(o => dist(sim.own, o) < o.r + sim.own.width / 2);
      if (sim.collision) { sim.decision.phase = '碰撞 / 约束失效'; log(sim, '检测到船体近似碰撞。此参数组合下策略失效。', 'danger'); }
      if (dist(sim.own, sim.goal) <= 1.3) {
        sim.complete = true; sim.own.speed = 0;
        sim.decision.phase = sim._memory ? '抵达目标 · 会遇责任未解除' : '抵达目标';
        log(sim, sim._memory ? '到达演示终点，但尚未满足驶过让清条件；不得据此认定追越 / 会遇已完成。' : '到达终点容差，停止演示。', sim._memory ? 'warning' : 'success');
      }
      if (sim.time - sim._lastHistoryTime >= 0.4 - 1e-8) {
        sim.history.own.push({...point(sim.own), t: sim.time}); if (sim.target) sim.history.target.push({...point(sim.target), t: sim.time});
        if (sim.history.own.length > 1200) sim.history.own.shift();
        if (sim.history.target.length > 1200) sim.history.target.shift();
        sim._lastHistoryTime = sim.time;
      }
    }
    return sim;
  }
  return {SCENARIOS, DEFAULTS, LOS_DEFAULTS, createSimulation, step, replan, losPlan, astar, cpa, searchAstar, validateTrajectory, routeCost};
});
