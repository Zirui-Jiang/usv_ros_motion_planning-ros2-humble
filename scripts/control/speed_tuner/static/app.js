"use strict";
const $ = id => document.getElementById(id);
let meta, response, pinned = null, searchResult = null;
let busy = false, dirty = true, revision = 0, rerun = false, timer;
const fmt = (value, digits = 2) => value == null ? "未达到" : Number(value).toFixed(digits);
const gainLabel = value => Number(Number(value).toFixed(6)).toString();
const current = () => ({kp: Number($("kp").value), ki: Number($("ki").value),
  target: Number($("target").value), duration: Number($("duration").value),
  scenario: document.querySelector('input[name="scenario"]:checked').value, source_id: meta.source_id});

function status(text, kind = "") {
  $("statusText").textContent = text;
  $("status").className = kind;
}
function buttons() {
  $("export").disabled = busy || dirty || !response;
  $("pin").disabled = busy || dirty || !response;
  $("sweep").disabled = busy || dirty || !response;
  $("run").disabled = busy;
}
function error(message) {
  $("error").textContent = message;
  $("error").hidden = false;
  status("计算未完成，请检查提示", "failed");
}
async function api(route, body) {
  const result = await fetch(`/api/${route}`, body === undefined ? {} : {
    method: "POST", headers: {"Content-Type": "application/json"}, body: JSON.stringify(body)
  });
  let data;
  try { data = await result.json(); } catch { throw new Error("服务未返回有效数据，请检查启动终端。"); }
  if (!result.ok) throw new Error(data.error || `请求失败 (${result.status})`);
  return data;
}
function settingsChanged(conditions = false) {
  revision++; dirty = true;
  if (conditions) {
    pinned = null; searchResult = null;
    $("candidateTable").hidden = true;
    $("sweepStatus").textContent = "试验条件已变化，请重新筛选候选参数。";
    updateScenario();
  }
  status("参数已改变，等待计算；图中暂为上一次结果", "busy");
  buttons();
  clearTimeout(timer);
  timer = setTimeout(analyze, 380);
}
async function analyze() {
  clearTimeout(timer);
  if (busy) { rerun = true; return; }
  if (!$("tuningForm").reportValidity()) { error("请检查参数输入范围。"); return; }
  busy = true; rerun = false; buttons();
  const runRevision = revision;
  const body = current();
  $("error").hidden = true;
  status("正在计算响应…", "busy");
  try {
    const data = await api("analyze", body);
    if (runRevision === revision) {
      response = data; dirty = false;
      render();
      const cost = data.candidate.runtime_ms + data.baseline.runtime_ms;
      $("runtime").textContent = `${fmt(cost, 0)} ms · ${body.duration} s 数值试验 × 2`;
      status("计算完成 · 当前参数尚未通过 Gazebo 验证");
    } else rerun = true;
  } catch (cause) {
    if (runRevision === revision) error(cause.message);
    else rerun = true;
  } finally {
    busy = false; buttons();
    if (rerun) { rerun = false; analyze(); }
  }
}
function updateScenario() {
  const scenario = current().scenario;
  $("scenarioHint").textContent = scenario === "step" ? "从静止开始，跟踪固定目标速度。" :
    scenario === "down" ? "前半程加速，中点将参考速度降至 30%。" :
      "前半程加速，中点切断推力并清积分；不主动倒桨制动。";
  $("t90Label").textContent = scenario === "step" ? "达到 90% 用时" : "切换后完成 90% 变化";
}
function metric(id, value, unit, digits) {
  const node = $(id);
  node.className = value == null ? "unreached" : "";
  node.replaceChildren();
  node.append(document.createTextNode(value == null ? "本次未达到" : fmt(value, digits)));
  if (value != null) { const label = document.createElement("small"); label.textContent = unit; node.append(label); }
}
function render() {
  const data = response.candidate, base = response.baseline;
  const m = data.metrics, bm = base.metrics;
  metric("metricT90", m.t90_s, "s", 1);
  metric("metricSettling", m.settling_s, "s", 1);
  metric("metricOvershoot", m.overshoot_pct, "%", 1);
  metric("metricError", m.tail_mae_mps, "m/s", 3);
  const difference = m.t90_s == null || bm.t90_s == null ? null : m.t90_s - bm.t90_s;
  $("deltaT90").textContent = difference == null ? `基准：${bm.t90_s == null ? "本次未达到" : fmt(bm.t90_s, 1) + " s"}` :
    `${difference > 0 ? "+" : ""}${fmt(difference, 1)} s 相比源配置基准`;
  $("settlingHint").textContent = `误差 ≤ ${fmt(m.band_mps, 3)} m/s，保持至结束`;
  $("peakForce").textContent = `${fmt(m.peak_force_n, 1)} N`;
  $("saturation").textContent = `${fmt(m.saturation_pct, 1)} %`;
  $("acceleration").textContent = `${fmt(m.peak_acceleration_mps2, 2)} m/s²`;
  $("comparisonNote").textContent = data.settings.kp === base.settings.kp && data.settings.ki === base.settings.ki ?
    "当前参数与源配置相同，曲线重合。拖动滑块或运行范围筛选，开始比较。" :
    `当前 Kp ${data.settings.kp} / Ki ${data.settings.ki}；基准 Kp ${base.settings.kp} / Ki ${base.settings.ki}。两组使用相同试验条件。`;
  $("pinLegend").hidden = !pinned; $("unpin").hidden = !pinned;
  if (pinned) $("pinLegend").textContent = `固定 Kp ${pinned.settings.kp} / Ki ${pinned.settings.ki}`;
  drawCharts();
}
function visibleEnd() {
  if (!response) return 0;
  const duration = response.candidate.settings.duration;
  return $("timeWindow").value === "full" ? duration : Math.min(duration, Number($("timeWindow").value));
}
function drawPlot(id, sets, includeLimits = false) {
  const svg = $(id), width = 900, height = id === "speedPlot" ? 310 : 235;
  const left = 55, right = 18, top = 16, bottom = 33, end = visibleEnd();
  let minimum = 0, maximum = 0;
  for (const set of sets) for (let i = 0; i < set.values.length && set.times[i] <= end + 1e-8; i++) {
    minimum = Math.min(minimum, set.values[i]); maximum = Math.max(maximum, set.values[i]);
  }
  if (includeLimits) {
    minimum = Math.min(minimum, meta.gains.force_min);
    maximum = Math.max(maximum, meta.gains.force_max);
  }
  const span = Math.max(maximum - minimum, id === "speedPlot" ? .1 : 1);
  const lo = minimum < 0 ? minimum - span*.1 : 0, hi = maximum + span*.12;
  const x = t => left + t/end*(width-left-right);
  const y = value => height-bottom-(value-lo)/(hi-lo)*(height-top-bottom);
  const lines = [`<title>${id === "speedPlot" ? "实际速度与参考速度" : "总推力与控制分量"}</title>`];
  for (let i = 0; i <= 5; i++) {
    const t = end*i/5, value = lo+(hi-lo)*i/5;
    lines.push(`<line class="grid-line" x1="${x(t)}" y1="${top}" x2="${x(t)}" y2="${height-bottom}"/>`);
    lines.push(`<text x="${x(t)}" y="${height-10}" text-anchor="middle">${fmt(t, 0)}</text>`);
    lines.push(`<line class="grid-line" x1="${left}" y1="${y(value)}" x2="${width-right}" y2="${y(value)}"/>`);
    lines.push(`<text x="${left-10}" y="${y(value)+4}" text-anchor="end">${fmt(value, id === "speedPlot" ? 1 : 0)}</text>`);
  }
  if (includeLimits) for (const limit of [meta.gains.force_min, meta.gains.force_max]) {
    lines.push(`<line class="zero-line" x1="${left}" y1="${y(limit)}" x2="${width-right}" y2="${y(limit)}"/>`);
    lines.push(`<text x="${width-right-4}" y="${y(limit)-5}" text-anchor="end">限幅 ${limit} N</text>`);
  }
  for (const set of sets) {
    let path = "";
    for (let i = 0; i < set.times.length && set.times[i] <= end + 1e-8; i++) {
      const px = x(set.times[i]).toFixed(2), py = y(set.values[i]).toFixed(2);
      path += i === 0 ? `M${px},${py}` : set.step ? `H${px}V${py}` : `L${px},${py}`;
    }
    lines.push(`<path class="${set.cls}" d="${path}"/>`);
  }
  lines.push(`<line class="cursor-line" x1="${left}" y1="${top}" x2="${left}" y2="${height-bottom}"/>`);
  lines.push(`<text x="${width-1}" y="${height-10}" text-anchor="end">s</text>`);
  svg.innerHTML = lines.join("");
}
function drawCharts() {
  if (!response) return;
  const a = response.candidate.series, b = response.baseline.series;
  const speed = [
    {times:a.time, values:a.reference, cls:"line-target", step:true},
    {times:b.time, values:b.speed, cls:"line-base"},
  ];
  if (pinned) speed.push({times:pinned.series.time, values:pinned.series.speed, cls:"line-pin"});
  speed.push({times:a.time, values:a.speed, cls:"line-current"});
  drawPlot("speedPlot", speed);
  const force = [{times:b.time, values:b.force, cls:"line-base", step:true}];
  if (pinned) force.push({times:pinned.series.time, values:pinned.series.force, cls:"line-pin", step:true});
  if ($("components").checked) {
    force.push({times:a.time, values:a.proportional, cls:"line-p", step:true});
    force.push({times:a.time, values:a.integral, cls:"line-i", step:true});
  }
  document.querySelectorAll(".component-legend").forEach(n => n.hidden = !$("components").checked);
  force.push({times:a.time, values:a.force, cls:"line-current", step:true});
  drawPlot("forcePlot", force, true);
  $("cursor").max = Math.round(visibleEnd()/meta.dt);
  readout(Math.min(Number($("cursor").value), Number($("cursor").max)));
}
function readout(index) {
  if (!response) return;
  const a = response.candidate.series;
  index = Math.max(0, Math.min(index, a.time.length-1, Number($("cursor").max)));
  $("cursor").value = index;
  $("readout").textContent = `${fmt(a.time[index],1)} s  ·  当前 ${fmt(a.speed[index],3)} / 参考 ${fmt(a.reference[index],2)} m/s  ·  推力 ${fmt(a.force[index],2)} N`;
  const x = 55+a.time[index]/visibleEnd()*(900-55-18);
  document.querySelectorAll(".cursor-line").forEach(line => { line.setAttribute("x1", x); line.setAttribute("x2", x); });
}
function lockSettings(lock) {
  document.querySelectorAll("#tuningForm input, #tuningForm select, #tuningForm button, #sweepForm input").forEach(n => n.disabled = lock);
  $("reload").disabled = lock;
}
async function screenCandidates(event) {
  event.preventDefault();
  if (busy || !$("sweepForm").reportValidity()) return;
  clearTimeout(timer); busy = true; buttons(); lockSettings(true); $("error").hidden = true;
  $("sweepStatus").textContent = "正在计算 25 组参数，请稍候…";
  status("正在进行网格筛选…", "busy");
  try {
    searchResult = await api("sweep", {...current(), kp_min:Number($("kpMin").value), kp_max:Number($("kpMax").value),
      ki_min:Number($("kiMin").value), ki_max:Number($("kiMax").value)});
    $("candidates").innerHTML = searchResult.candidates.slice(0,8).map((row, i) => `<tr><td>${String(i+1).padStart(2,"0")}</td><td>${gainLabel(row.kp)} / ${gainLabel(row.ki)}</td><td>${row.metrics.t90_s == null ? "未达到" : fmt(row.metrics.t90_s,1)+" s"}</td><td>${fmt(row.metrics.overshoot_pct,1)}%</td><td>${fmt(row.metrics.tail_mae_mps,3)}</td><td>${fmt(row.score,4)}</td><td><button type="button" class="button secondary" data-candidate="${i}" aria-label="采用第 ${i+1} 组参数">采用</button></td></tr>`).join("");
    $("candidateTable").hidden = false;
    $("sweepStatus").textContent = `已计算 25 组，展示前 8 组 · 用时 ${fmt(searchResult.runtime_ms/1000,2)} s。仅适用于本次模型和场景。`;
    status("筛选完成 · 点击候选参数查看响应");
  } catch (cause) { error(cause.message); $("sweepStatus").textContent = "筛选未完成，请检查参数范围。"; }
  finally { busy = false; lockSettings(false); buttons(); }
}
async function exportCandidate() {
  if (busy || dirty || !response) return;
  busy = true; buttons(); lockSettings(true); $("error").hidden = true;
  status("正在复算候选并保存 YAML、JSON 与 CSV…", "busy");
  try {
    const data = await api("export", response.candidate.settings);
    $("exportPath").textContent = data.path;
    $("exportEvidence").textContent = data.report_path + "\n" + data.csv_path;
    $("launchCommand").value = data.run_command;
    $("copyStatus").textContent = "";
    $("exportDialog").showModal();
    status("候选已导出，正式源配置保持不变");
  } catch (cause) { error(cause.message); }
  finally { busy = false; lockSettings(false); buttons(); }
}
async function initialize() {
  try {
    meta = await api("config");
    $("kp").value = $("kpRange").value = meta.default.kp;
    $("ki").value = $("kiRange").value = meta.default.ki;
    $("target").max = Math.min(3,meta.model.velocity_max);
    $("target").value = meta.default.target;
    $("baselineLabel").textContent = `Kp ${meta.default.kp}  /  Ki ${meta.default.ki}`;
    $("sourceFile").textContent = meta.config_path; $("sourceFile").title = meta.config_path;
    const modelRows = [
      ["质量 + 纵向附加质量",`${fmt(meta.model.mass+meta.model.x_dot_u,2)} kg`],
      ["纵向线性阻尼系数",`${meta.model.x_u} N/(m/s)`],
      ["控制周期",`${meta.dt} s`], ["总纵向力限幅",`${meta.gains.force_min} / ${meta.gains.force_max} N`],
      ["积分贡献限幅",`${meta.gains.speed_integral_min} / ${meta.gains.speed_integral_max} N`],
      ["单桨最大力",`${meta.model.max_force} N`]
    ];
    for (const [name,value] of modelRows) {
      const term=document.createElement("dt"), description=document.createElement("dd");
      term.textContent=name; description.textContent=value; $("modelValues").append(term,description);
    }
    for (const key of ["kp","ki"]) {
      $(key).addEventListener("input", () => { if ($(key).validity.valid) $(key+"Range").value=$(key).value; settingsChanged(); });
      $(key+"Range").addEventListener("input", () => { $(key).value=$(key+"Range").value; settingsChanged(); });
    }
    $("target").addEventListener("input", () => settingsChanged(true));
    $("duration").addEventListener("change", () => settingsChanged(true));
    document.querySelectorAll('input[name="scenario"]').forEach(n => n.addEventListener("change", () => settingsChanged(true)));
    $("reset").addEventListener("click", () => {
      $("kp").value=$("kpRange").value=meta.default.kp; $("ki").value=$("kiRange").value=meta.default.ki;
      settingsChanged();
    });
    $("tuningForm").addEventListener("submit", event => { event.preventDefault(); analyze(); });
    $("sweepForm").addEventListener("submit", screenCandidates);
    $("candidates").addEventListener("click", event => {
      const button=event.target.closest("[data-candidate]");
      if (!button || busy || !searchResult) return;
      const row=searchResult.candidates[Number(button.dataset.candidate)];
      $("kp").value=$("kpRange").value=Number(row.kp.toFixed(6));
      $("ki").value=$("kiRange").value=Number(row.ki.toFixed(6));
      settingsChanged();
    });
    $("pin").addEventListener("click", () => { if (!dirty && response) { pinned=response.candidate; render(); } });
    $("unpin").addEventListener("click", () => { pinned=null; render(); });
    $("timeWindow").addEventListener("change", drawCharts);
    $("components").addEventListener("change", drawCharts);
    $("cursor").addEventListener("input", () => readout(Number($("cursor").value)));
    for (const id of ["speedPlot","forcePlot"]) $(id).addEventListener("pointermove", event => {
      if (!response) return;
      const box=$(id).getBoundingClientRect();
      const t=((event.clientX-box.left)/box.width*900-55)/(900-55-18)*visibleEnd();
      readout(Math.round(t/meta.dt));
    });
    $("export").addEventListener("click", exportCandidate);
    updateScenario();
    await analyze();
  } catch (cause) { error(cause.message); }
}
$("reload").addEventListener("click", () => location.reload());
$("helpOpen").addEventListener("click", () => $("helpDialog").showModal());
document.querySelectorAll(".close-dialog").forEach(button => button.addEventListener("click", () => button.closest("dialog").close()));
$("copyCommand").addEventListener("click", async () => {
  try { await navigator.clipboard.writeText($("launchCommand").value); $("copyStatus").textContent="启动命令已复制"; }
  catch { $("launchCommand").focus(); $("launchCommand").select(); $("copyStatus").textContent="请按 Ctrl+C 复制选中的命令"; }
});
initialize();
