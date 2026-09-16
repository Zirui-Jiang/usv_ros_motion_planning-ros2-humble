/* Run: node scripts/colregs_demo/engine.test.cjs */
const assert = require('node:assert/strict');
const fs = require('node:fs');
assert.ok(fs.existsSync(__dirname + '/engine.js'), 'COLREGs / LOS demo engine must exist');
const E = require('./engine.js');
const almost = (actual, expected, tolerance = 1e-9) => assert.ok(Math.abs(actual - expected) < tolerance, `${actual} != ${expected}`);

// This catches using a different integrator or a constant cruise profile.
const straight = E.losPlan([[0, 0, 0], [100, 0, 0]], {x: 0, y: 0, yaw: 0}, 3, {trajectory_time: 1});
almost(straight.at(-1).x, 100 * (1 - Math.pow(0.997, 10)));
almost(straight.at(-1).y, 0);
assert.deepEqual(straight.map(p => p.t), [0, 0.5, 1]);
const near = E.losPlan([{x: 0, y: 0}, {x: 2, y: 0}], {x: 0, y: 0, yaw: 0}, 3);
almost(near[0].speed, 2);

// V2: virtual constraints must affect the graph, its simplification, and LOS.
assert.equal(typeof E.searchAstar, 'function', 'V2 requires a real constrained A* search');
const testZone = {id: 'test-rule', kind: 'rule', label: 'test', reason: 'test',
  points: [{x: -15, y: -12}, {x: 15, y: -12}, {x: 15, y: 35}, {x: -15, y: 35}]};
const testGlobal = [{x: -65, y: 0}, {x: 65, y: 0}];
const withoutZone = E.searchAstar(testGlobal[0], testGlobal[1], {globalRoute: testGlobal});
const constrained = E.searchAstar(testGlobal[0], testGlobal[1], {globalRoute: testGlobal, zones: [testZone]});
assert.ok(withoutZone.route.every(p => Math.abs(p.y) < 1e-9));
assert.ok(constrained.expanded > 0 && constrained.rawRoute.length > 2);
assert.ok(constrained.route.some(p => p.y < -12), 'Rule zone must force a different route');
assert.ok(E.validateTrajectory(constrained.route.map((p, i) => ({...p, t: i})), {zones: [testZone]}).ruleSafe,
  'Simplified A* route must still avoid virtual zones');
assert.equal(E.validateTrajectory([{x: -30, y: 0, t: 0}, {x: 30, y: 0, t: 1}], {zones: [testZone]}).ruleSafe, false,
  'A LOS segment crossing a rule zone must be rejected even when its endpoints are outside');
const unweighted = E.routeCost([{x: -65, y: 0}, {x: 0, y: -30}, {x: 65, y: 0}], testGlobal, 0);
const weighted = E.routeCost([{x: -65, y: 0}, {x: 0, y: -30}, {x: 65, y: 0}], testGlobal, 0.025);
assert.ok(weighted.routeDeviation > 0 && weighted.total > unweighted.total, 'Route deviation must contribute to cost');

// Rules-off is the A*+LOS baseline; rules-on must turn to starboard.
const on = E.createSimulation('head-on');
const off = E.createSimulation('head-on', {rulesEnabled: false});
assert.ok(on.ruleZones.length > 0);
assert.deepEqual(off.ruleZones, []);
const originalRoute = JSON.stringify(on.globalRoute), initialSearches = on.searchInfo.searchCount;
for (let i = 0; i < 50; i++) E.step(on, 0.1);
assert.equal(on.searchInfo.searchCount, initialSearches, 'Stable constraints should reuse the searched route');
assert.equal(JSON.stringify(on.globalRoute), originalRoute);
assert.ok(on.searchInfo.rejoin.x > -125, 'A* must reconnect forward along the original route');
assert.ok(on.activeRoute.some(p => p.y < -10), 'Head-on must make a starboard detour');
assert.ok(off.activeRoute.every(p => Math.abs(p.y) < 1e-8), 'Baseline must keep the original route');
for (let i = 0; i < 150; i++) E.step(on, 0.1);
assert.ok(on.own.y < -10);
assert.equal(on.decision.encounter, '对遇', 'Encounter responsibility survives heading changes');

for (const scenario of E.SCENARIOS) {
  const sim = E.createSimulation(scenario.id);
  for (let i = 0; i < 2400 && !sim.complete && !sim.collision; i++) E.step(sim, 0.1);
  assert.ok(Number.isFinite(sim.own.x + sim.own.y + sim.own.yaw + sim.own.speed));
  assert.ok(!sim.collision, `${scenario.id}: collision at ${sim.time.toFixed(1)}s`);
  assert.ok(sim.complete, `${scenario.id}: stalled at ${sim.time.toFixed(1)}s (${sim.decision.phase})`);
  if (scenario.id === 'overtaking' && sim.decision.encounter !== '无会遇风险') {
    assert.equal(sim.decision.phase, '抵达目标 · 会遇责任未解除', 'Following to the demo endpoint must not claim completed overtaking');
    assert.ok(sim.logs.some(event => event.text.includes('降速跟随')), 'A safe following choice must be made explicit to the operator');
  }
  console.log(`${scenario.id}: reached ${sim.time.toFixed(1)}s; minimum center distance ${sim.metrics.minDistance.toFixed(1)}m`);
}
const obstacle = E.createSimulation('obstacle-crossing');
assert.ok(obstacle.globalRoute.some(p => Math.abs(p.y) > 10), 'A* must route around the static obstacle');
const stopped = E.createSimulation('head-on', {ownSpeed: 0});
E.step(stopped, 0.1);
assert.ok(Number.isFinite(stopped.own.x));
// A zero speed cap must actually brake a currently moving vessel.
const braking = E.createSimulation('clear', {ownSpeed: 0}, {own: {speed: 3}});
for (let i = 0; i < 50; i++) E.step(braking, 0.1);
assert.ok(braking.own.speed < 0.5, `Zero speed cap did not brake: ${braking.own.speed}`);
const overtaken = E.createSimulation('being-overtaken', {targetCooperative: true});
assert.equal(overtaken.decision.phase, '直航监测', 'Stand-on vessel should first allow the overtaker to act');
for (let i = 0; i < 1500 && !overtaken.complete; i++) E.step(overtaken, 0.1);
assert.ok(overtaken.metrics.minDistance >= overtaken.config.safetyDistance);
assert.equal(obstacle.decision.encounter, '右舷交叉', 'The obstacle scenario should also exercise the rule layer');
const speedOverride = E.createSimulation('overtaking', {targetSpeed: 2});
almost(speedOverride.target.speed, 2);
almost(E.createSimulation('overtaking').target.speed, 1.4);
almost(E.createSimulation('being-overtaken').target.speed, 4.8);
const angledOvertake = E.createSimulation('overtaking', {ownSpeed: 4, targetSpeed: 1.35},
  {own: {x: -100, y: -20, yaw: Math.PI / 4}, target: {x: -70, y: 0, yaw: 0}});
assert.equal(angledOvertake.decision.encounter, '本船追越', 'Rule 13 recognition uses approach from abaft the target beam, including unequal headings');
const slanted = E.createSimulation('head-on', {}, {own: {y: 40}, target: {y: 40}, goal: {y: 10}});
const savedSlantedGlobal = JSON.stringify(slanted.globalRoute);
for (let i = 0; i < 1200 && slanted.decision.phase !== '恢复航路'; i++) E.step(slanted, 0.1);
assert.equal(slanted.decision.phase, '恢复航路');
assert.equal(JSON.stringify(slanted.globalRoute), savedSlantedGlobal);
const reconnectIndex = slanted.activeRoute.findIndex(p => Math.hypot(p.x - slanted.searchInfo.rejoin.x, p.y - slanted.searchInfo.rejoin.y) < 1e-8);
assert.ok(reconnectIndex >= 0, 'Searched connector must contain the advertised original-route rejoin');
for (const p of slanted.activeRoute.slice(reconnectIndex)) {
  assert.ok(slanted.globalRoute.slice(1).some((b, i) => {
    const a = slanted.globalRoute[i], dx = b.x - a.x, dy = b.y - a.y;
    const t = ((p.x - a.x) * dx + (p.y - a.y) * dy) / (dx * dx + dy * dy);
    return t >= -1e-8 && t <= 1 + 1e-8 && Math.abs((p.x - a.x) * dy - (p.y - a.y) * dx) < 1e-7;
  }), 'Rejoined suffix must stay on the actual grid-derived original route');
}
const sameCellBlocked = E.searchAstar({x: 0, y: 0}, {x: 1, y: 1}, {gridSize: 5, margin: 0, zoneMargin: 0, obstacles: [{x: 1, y: 1, r: 0.1}]});
assert.deepEqual(sameCellBlocked.route, [], 'Same-cell goal may not bypass its obstacle');
assert.equal(E.validateTrajectory([{x: -1, y: 0, t: 0}, {x: 0.5, y: 1, t: 0.5}],
  {directions: [{origin: {x: 0, y: 0}, yaw: 0, from: 0, until: 10}]}).ruleSafe, false, 'Direction-window crossing is checked across the whole edge');
for (const invalid of [[{x: NaN, y: 0, t: 0}], [{x: 0, y: 0, t: 1}, {x: 1, y: 1, t: 0}]]) {
  const check = E.validateTrajectory(invalid);
  assert.ok(!check.staticSafe && !check.ruleSafe && !check.dynamicSafe);
}
console.log('All engine checks passed.');
