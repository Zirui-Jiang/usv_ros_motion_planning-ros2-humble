/* Independent deterministic V2 scenario audit. No browser or ROS required.
 * Usage: node scripts/colregs_demo/run_scenarios.cjs [output-directory]
 * Baseline collisions are observations, not test failures: the baseline has no
 * dynamic avoidance. Rules-on defaults must complete without a collision.
 */
'use strict';
const assert = require('node:assert/strict');
const fs = require('node:fs');
const path = require('node:path');
const crypto = require('node:crypto');
const E = require('./engine.js');
const output = process.argv[2] ? path.resolve(process.argv[2]) : path.join(__dirname, 'audit-v2');
const report = {version: 2, passed: false, scenarios: [], failures: [],
  engineSha256: crypto.createHash('sha256').update(fs.readFileSync(path.join(__dirname,'engine.js'))).digest('hex')};
const initial = E.createSimulation('head-on');
assert.equal(initial.meta.version, '2.0', 'This audit must use the upgraded engine');

for (const scene of E.SCENARIOS) {
  for (const rulesEnabled of [true, false]) {
    const sim = E.createSimulation(scene.id, {rulesEnabled});
    const originalRoute = JSON.stringify(sim.globalRoute);
    const phases = [], acceptedPlans = [], started = performance.now();
    let lastSearchCount = -1, lastPhase = null, routeChanged = false;
    const capture = () => {
      if (sim.decision.phase !== lastPhase) {
        phases.push({time: sim.time, phase: sim.decision.phase, action: sim.decision.action});
        lastPhase = sim.decision.phase;
      }
      const count = sim.searchInfo?.searchCount ?? 0;
      if (count !== lastSearchCount) {
        const chosen = sim.candidates?.find(c => c.chosen);
        const feasible = (sim.candidates || []).filter(c => c.safe && Number.isFinite(c.cost));
        if (chosen && feasible.length && chosen.cost > Math.min(...feasible.map(c => c.cost)) + 1e-6) {
          report.failures.push(`${scene.id}/${rulesEnabled}: chosen candidate is not the least-cost feasible candidate at ${sim.time}`);
        }
        acceptedPlans.push({time: sim.time, searches: count, ruleVersion: sim.searchInfo?.ruleVersion,
          trigger: sim.searchInfo?.trigger, expanded: sim.searchInfo?.expanded,
          status: sim.searchInfo?.status, cost: sim.searchInfo?.costBreakdown,
          validation: sim.routeValidation ? {...sim.routeValidation} : null});
        lastSearchCount = count;
      }
      routeChanged ||= JSON.stringify(sim.globalRoute) !== originalRoute;
    };
    capture();
    for (let i=0; i<3000 && !sim.complete && !sim.collision; i++) {
      E.step(sim, .1); capture();
      if (![sim.own.x,sim.own.y,sim.own.yaw,sim.own.speed,sim.time].every(Number.isFinite)) {
        report.failures.push(`${scene.id}/${rulesEnabled}: non-finite vessel state`); break;
      }
    }
    const item = {id:scene.id, title:scene.title, rulesEnabled, complete:sim.complete, collision:sim.collision,
      elapsedSeconds:sim.time, wallMilliseconds:performance.now()-started,
      minCenterDistance:sim.metrics.minDistance, requestedCenterDistance:sim.config.safetyDistance,
      initialRoutePreserved:!routeChanged, finalPhase:sim.decision.phase,
      searches:sim.searchInfo?.searchCount, phases, acceptedPlans};
    report.scenarios.push(item);
    if (routeChanged) report.failures.push(`${scene.id}/${rulesEnabled}: nominal route mutated`);
    if (!rulesEnabled && sim.ruleZones?.length) report.failures.push(`${scene.id}: rules-off retained rule zones`);
    if (rulesEnabled && (!sim.complete || sim.collision)) report.failures.push(`${scene.id}: default rules-on did not complete safely (${sim.decision.phase})`);
    console.log(`${scene.id} ${rulesEnabled?'rules':'baseline'}: ${sim.complete?'arrived':sim.collision?'collision':'unfinished'}, ${sim.time.toFixed(1)} s, min ${sim.metrics.minDistance.toFixed(1)} m, searches ${sim.searchInfo?.searchCount}`);
  }
}
report.passed = report.failures.length === 0;
fs.mkdirSync(output, {recursive:true});
fs.writeFileSync(path.join(output,'scenario-report.json'),JSON.stringify(report,null,2));
console.log(`V2 scenario audit: ${report.passed?'PASS':'FAIL'}; ${path.join(output,'scenario-report.json')}`);
for (const failure of report.failures) console.error(failure);
if (!report.passed) process.exitCode=1;
