/* Presentation and controls only; all navigation decisions live in engine.js. */
(() => {
  'use strict';
  const E = window.ColregsDemo;
  const $ = id => document.getElementById(id);
  const canvas = $('chart'), ctx = canvas.getContext('2d');
  const colors = {own:'#087e8b', target:'#c56a46', route:'#879ba6', detour:'#c99143', rule:'#9670a6', safety:'#bc873e', search:'#6b91a7', grid:'#d3e1e4', text:'#58727e'};
  let params = {...E.DEFAULTS}, overrides = {}, scenarioId = 'head-on';
  let sim, running = false, playbackRate = 4, accumulator = 0, previousFrame = 0;
  let zoom = 1, width = 600, height = 480, scale = 1, drag = null, lastUiUpdate = 0;
  let lastLogSignature = '', lastSearchSignature = '', toastTimer;
  const rad = Math.PI / 180;
  const fmt = (n, digits=1) => Number.isFinite(n) ? n.toFixed(digits) : '—';
  const course = yaw => ((90-yaw/rad)%360+360)%360;
  const timeText = n => `${Math.floor(n/60).toString().padStart(2,'0')}:${(n%60).toFixed(1).padStart(4,'0')}`;
  const sceneMeta = () => E.SCENARIOS.find(s => s.id === scenarioId) || E.SCENARIOS[0];
  const setText = (id,value) => { const node=$(id), next=String(value ?? '—'); if(node.textContent!==next) node.textContent=next; };
  function toast(text) { clearTimeout(toastTimer); $('toast').textContent=text; $('toast').hidden=false; toastTimer=setTimeout(()=>$('toast').hidden=true,3000); }

  const sceneDrawings = {
    'head-on':'<path d="m14 13-10-4v8Z" fill="currentColor"/><path d="m20 13 10-4v8Z" fill="#c56a46"/>',
    'crossing-starboard':'<path d="m13 8-10-4v8Z" fill="currentColor"/><path d="m24 4-4 11h8Z" fill="#c56a46"/><path d="M13 8h11v11" stroke="currentColor" fill="none" stroke-dasharray="2 3" opacity=".45"/>',
    'crossing-port':'<path d="m13 19-10-4v8Z" fill="currentColor"/><path d="m24 24-4-11h8Z" fill="#c56a46"/><path d="M13 19h11V7" stroke="currentColor" fill="none" stroke-dasharray="2 3" opacity=".45"/>',
    'overtaking':'<path d="m14 17-10-4v8Z" fill="currentColor"/><path d="m30 9-10-4v8Z" fill="#c56a46"/>',
    'being-overtaken':'<path d="m30 9-10-4v8Z" fill="currentColor"/><path d="m14 17-10-4v8Z" fill="#c56a46"/>',
    'clear':'<path d="m14 14-11-4v8Z" fill="currentColor"/><path d="M18 14h12" stroke="currentColor" stroke-dasharray="2 3"/>',
    'obstacle-crossing':'<path d="m12 19-10-4v8Z" fill="currentColor"/><circle cx="20" cy="13" r="5" fill="#a6bba9"/><path d="m29 2-4 9h8Z" fill="#c56a46"/>'
  };
  for (const scene of E.SCENARIOS) {
    const button = document.createElement('button');
    button.type='button'; button.className='scene-btn'; button.dataset.scene=scene.id;
    const icon=document.createElementNS('http://www.w3.org/2000/svg','svg');
    icon.setAttribute('viewBox','0 0 34 28'); icon.setAttribute('aria-hidden','true'); icon.classList.add('scene-icon');
    icon.innerHTML=sceneDrawings[scene.id] || sceneDrawings.clear;
    const copy=document.createElement('span'), title=document.createElement('strong'), subtitle=document.createElement('small');
    title.textContent=scene.title; subtitle.textContent=scene.subtitle || scene.rule;
    copy.append(title,subtitle); const check=document.createElement('span'); check.className='scene-check'; check.textContent='✓';
    button.append(icon,copy,check); button.addEventListener('click',()=>setScenario(scene.id)); $('scene-list').append(button);
  }
  function reset(id) {
    if(id && id!==scenarioId) return setScenario(id);
    running=false; accumulator=0; drag=null;
    sim=E.createSimulation(scenarioId,params,overrides);
    params={...sim.config}; lastLogSignature=''; lastSearchSignature='';
    syncControls(); renderUi(); draw(); return sim;
  }
  function setScenario(id) {
    if(!E.SCENARIOS.some(s=>s.id===id)) throw new Error(`Unknown scenario: ${id}`);
    scenarioId=id; overrides={}; zoom=1;
    // The scenario chooses its initial speed relationship (important for overtaking).
    const switches={rulesEnabled:params.rulesEnabled,targetCooperative:params.targetCooperative};
    params={...E.DEFAULTS,...switches};
    const meta=sceneMeta();
    if(meta.params) Object.assign(params,meta.params);
    if(meta.defaults) Object.assign(params,meta.defaults);
    reset();
  }
  function syncControls() {
    const cfg=sim.config;
    for(const [id,key] of [['own-speed','ownSpeed'],['target-speed','targetSpeed'],['safety-distance','safetyDistance'],['lookahead','lookahead']]) {
      $(id).value=cfg[key];
    }
    $('rules-toggle').checked=cfg.rulesEnabled; $('cooperative-toggle').checked=cfg.targetCooperative;
    $('target-course').value=Math.round(sim.target ? course(sim.target.yaw) : 0);
    $('target-speed').disabled=!sim.target; $('target-course').disabled=!sim.target; $('cooperative-toggle').disabled=!sim.target;
    setText('own-speed-value',`${fmt(cfg.ownSpeed)} m/s`);
    setText('target-speed-value',`${fmt(cfg.targetSpeed)} m/s`);
    setText('target-course-value',sim.target ? `${Math.round(course(sim.target.yaw))}°` : '无目标');
    setText('safety-distance-value',`${fmt(cfg.safetyDistance,0)} m`);
    setText('lookahead-value',`${fmt(cfg.lookahead,1)} m`);
    for(const button of $('scene-list').children) {
      const selected=button.dataset.scene===scenarioId;
      button.classList.toggle('active',selected); button.setAttribute('aria-pressed',String(selected));
    }
    const meta=sceneMeta(); setText('scene-name',meta.title); setText('guide-title',meta.watch || meta.title); setText('guide-description',meta.description);
    setText('mode-label',cfg.rulesEnabled?'规则约束 A*':'原 A* + LOS 对照');
  }
  function renderUi() {
    const d=sim.decision || {}, m=sim.metrics || {};
    const ended=sim.complete || sim.collision;
    setText('run-state',sim.collision?'发生碰撞':sim.complete?'已到达':running?'仿真中':'已暂停');
    setText('sim-clock',timeText(sim.time));
    setText('play-icon',running?'Ⅱ':'▶'); setText('play-label',running?'暂停':ended?'重新开始':'开始仿真');
    $('step-btn').disabled=running||ended;
    $('observe-btn').disabled=running||ended;
    setText('decision-action',d.action); setText('decision-reason',d.reason);
    setText('rule-number',d.rule || (sim.config.rulesEnabled?'COLREGs':'BASELINE · A* + LOS'));
    setText('encounter-label',d.encounter); setText('role-label',d.role); setText('phase-label',d.phase);
    const danger=sim.collision || /紧急|无可行/.test(`${d.phase} ${d.action}`);
    const badge=$('risk-badge'); badge.className=`status-badge${danger?' danger':d.risk?' warn':''}`;
    badge.textContent=sim.collision?'碰撞':danger?'需处置':d.risk?'存在风险':'监测';
    setText('distance-value',fmt(m.distance)); setText('minimum-value',fmt(m.minDistance)); setText('dcpa-value',fmt(m.dcpa));
    setText('tcpa-value',fmt(m.tcpa)); setText('tcpa-unit',Number.isFinite(m.tcpa)&&m.tcpa<0?'s · 已过':'s');
    setText('own-status',`${fmt(sim.own.speed)} m/s · ${fmt(course(sim.own.yaw),0)}°`);
    setText('target-status',sim.target?`${fmt(sim.target.speed)} m/s · ${fmt(course(sim.target.yaw),0)}°`:'本场景无目标船');
    setText('cross-track',`${fmt(m.crossTrack)} m`);
    setText('reference-speed',`${fmt(sim.trajectory?.[0]?.speed)} m/s`);
    renderSearchUi();
    const logs=sim.logs || [], signature=`${logs.length}:${logs.at(-1)?.time}:${logs.at(-1)?.text}`;
    if(signature!==lastLogSignature) {
      lastLogSignature=signature; $('event-log').replaceChildren(); setText('log-count',`${logs.length} 条`);
      for(const entry of logs.slice(-12).reverse()) {
        const li=document.createElement('li'), time=document.createElement('time'), text=document.createElement('span');
        time.textContent=`T + ${fmt(entry.time)} s`; text.textContent=entry.text; li.append(time,text); $('event-log').append(li);
      }
    }
    const banner=$('finish-banner'); banner.hidden=!ended;
    if(ended){banner.replaceChildren(document.createTextNode(sim.collision?'发生碰撞':'已到达最终目标'));const small=document.createElement('small');small.textContent=`最小中心间距 ${fmt(m.minDistance)} m · 可导出记录后调整场景`;banner.append(small);}
  }
  function renderSearchUi() {
    const info=sim.searchInfo || {}, validation=sim.routeValidation || {};
    const candidates=sim.candidates || [];
    setText('search-algorithm',info.algorithm || '等待搜索信息');
    setText('search-trigger',info.trigger || (sim.config.rulesEnabled?'监测会遇，等待搜索触发':'规则关闭，跟踪原全局航路'));
    setText('search-count',info.searchCount ?? 0);
    setText('search-expanded',info.expanded ?? '—');
    setText('search-version',info.ruleVersion ?? '—');
    setText('search-grid',Number.isFinite(info.gridSize)?`${fmt(info.gridSize,1)} m`:'—');
    setText('search-rejoin',info.rejoin?`(${fmt(info.rejoin.x)}, ${fmt(info.rejoin.y)}) m`:'无活动接回点');
    const statuses={idle:'监测中',searching:'搜索中',success:'已找到航路',found:'已找到航路',valid:'回验通过',failed:'搜索失败',blocked:'无可行航路',baseline:'原航路对照',disabled:'规则已关闭',monitoring:'监测中',rejoined:'已接回原航路'};
    const status=$('search-status'); status.textContent=statuses[info.status] || info.status || '等待更新';
    status.className=`status-badge${/failed|blocked|无可行|失败/.test(info.status || '')?' danger':''}`;
    status.dataset.status=info.status || 'unknown';
    for(const [id,key] of [['check-static','staticSafe'],['check-rule','ruleSafe'],['check-dynamic','dynamicSafe']]) {
      const node=$(id), result=validation[key];
      node.textContent=result===true?'通过':result===false?'未通过':'未回验';
      node.dataset.result=result===true?'pass':result===false?'fail':'unknown';
    }
    setText('validation-reason',validation.reason || '等待引擎提供几何与预测回验结果。');
    setText('validation-distance',Number.isFinite(validation.minDistance)?`${fmt(validation.minDistance)} m`:'—');
    const constraintText=info.activeConstraints || [];
    const signature=JSON.stringify({constraintText,candidates:candidates.map(c=>({label:c.label,safe:c.safe,chosen:c.chosen,speed:c.speed,reason:c.reason,cost:c.cost,costBreakdown:c.costBreakdown}))});
    if(signature===lastSearchSignature)return;
    lastSearchSignature=signature;
    const constraints=$('active-constraints');constraints.replaceChildren();
    if(!constraintText.length){const item=document.createElement('span');item.className='constraint-empty';item.textContent=sim.config.rulesEnabled?'当前无活动规则约束':'规则约束已关闭';constraints.append(item);}
    for(const constraint of constraintText){const item=document.createElement('span');item.textContent=constraint;constraints.append(item);}
    const list=$('candidate-list');list.replaceChildren();
    setText('candidate-count',`${candidates.filter(c=>c.safe===true).length} / ${candidates.length} 可行`);
    if(!candidates.length){const empty=document.createElement('p');empty.className='candidate-empty';empty.textContent='当前沿原航路监测；触发搜索后显示各候选的回验结果与代价。';list.append(empty);return;}
    for(const candidate of candidates){
      const item=document.createElement('li');item.className=`candidate-item${candidate.chosen?' chosen':''}`;
      item.dataset.safe=String(candidate.safe ?? 'unknown');item.dataset.chosen=String(Boolean(candidate.chosen));
      const heading=document.createElement('div');heading.className='candidate-heading';
      const name=document.createElement('strong');name.textContent=candidate.label || '候选航路';
      const badge=document.createElement('span');badge.className='candidate-result';badge.textContent=candidate.chosen?'当前采用':candidate.safe===true?'可行':candidate.safe===false?'未通过':'待回验';heading.append(name,badge);
      const costs=document.createElement('div');costs.className='candidate-costs';
      const breakdown=candidate.costBreakdown || {};
      for(const [label,value] of [['航速',`${fmt(candidate.speed)} m/s`],['路径长度',`${fmt(breakdown.length)} m`],['偏离代价',`${fmt(breakdown.routeDeviation)} m`],['航路代价',`${fmt(breakdown.total)} m`],['时间惩罚',`${fmt(breakdown.timePenalty)} m`],['候选总代价',`${fmt(candidate.cost ?? breakdown.total)} m`]]){
        const pair=document.createElement('span'),key=document.createElement('small'),data=document.createElement('b');key.textContent=label;data.textContent=value;pair.append(key,data);costs.append(pair);
      }
      const reason=document.createElement('p');reason.textContent=candidate.reason || (candidate.safe===true?'在当前模型与预测时域内通过回验。':'等待回验结果。');
      item.append(heading,costs,reason);list.append(item);
    }
  }
  function advance(seconds) {
    running=false; accumulator=0;
    const steps=Math.max(0,Math.round(Math.min(seconds,600)*10));
    for(let i=0;i<steps&&!sim.complete&&!sim.collision;i++) E.step(sim,.1);
    renderUi();draw();return sim;
  }
  for(const [id,key] of [['own-speed','ownSpeed'],['target-speed','targetSpeed'],['safety-distance','safetyDistance'],['lookahead','lookahead']]) {
    $(id).addEventListener('input',()=>{params[key]=Number($(id).value);reset();});
  }
  $('target-course').addEventListener('input',()=>{
    overrides.target={...(overrides.target||{}),yaw:(90-Number($('target-course').value))*rad}; reset();
  });
  $('rules-toggle').addEventListener('change',()=>{params.rulesEnabled=$('rules-toggle').checked;reset();toast(params.rulesEnabled?'已启用规则约束 A*，场景重新开始':'原 A* + LOS 对照：场景重新开始');});
  $('cooperative-toggle').addEventListener('change',()=>{params.targetCooperative=$('cooperative-toggle').checked;reset();});
  $('play-btn').addEventListener('click',()=>{if(sim.complete||sim.collision)reset();running=!running;accumulator=0;renderUi();});
  $('step-btn').addEventListener('click',()=>advance(1)); $('observe-btn').addEventListener('click',()=>advance(10));
  $('reset-btn').addEventListener('click',()=>{reset();toast('已按当前参数重置场景');});
  $('compare-btn').addEventListener('click',()=>{params.rulesEnabled=!params.rulesEnabled;reset();toast(params.rulesEnabled?'规则约束 A* 已开启':'已切换为原航路对照');});
  $('speed-select').addEventListener('change',()=>{playbackRate=Number($('speed-select').value);});
  for(const id of ['show-trails','show-prediction','show-candidates','show-rules','show-search']) $(id).addEventListener('change',draw);
  $('fit-btn').addEventListener('click',()=>{zoom=1;draw();});
  function download(blob,name) { const a=document.createElement('a');const url=URL.createObjectURL(blob);a.href=url;a.download=name;document.body.append(a);a.click();a.remove();setTimeout(()=>URL.revokeObjectURL(url),5000); }
  $('export-btn').addEventListener('click',()=>{
    const closed=sim.searchInfo?.closed || [];
    const searchInfo=sim.searchInfo?{...sim.searchInfo,closed:closed.slice(0,1500),closedTotal:closed.length,closedTruncated:closed.length>1500,closedExportLimit:1500}:null;
    const candidates=(sim.candidates || []).map(candidate=>{
      if(!candidate.search)return {...candidate};
      const search=candidate.search;
      // Speed candidates can share one search; export its evidence without duplicating the closed set.
      return {...candidate,search:{algorithm:search.algorithm,status:search.status,expanded:search.expanded,rawRoute:search.rawRoute,costBreakdown:search.costBreakdown,closedOmitted:true,closedTotal:search.closed?.length || 0}};
    });
    const report={format:'colregs-los-demo-v2',meta:{...(E.meta || {}),...(sim.meta || {}),version:'2.0'},exportedAt:new Date().toISOString(),model:'Rule-constrained A* on a binary / nonnegative weighted demo grid + ported LOS + finite-horizon geometric validation + simplified 2D tracking; not ROS/Gazebo',scenario:sceneMeta(),parameters:sim.config,initialOverrides:overrides,time:sim.time,own:sim.own,target:sim.target,goal:sim.goal,globalRoute:sim.globalRoute,activeRoute:sim.activeRoute,trajectory:sim.trajectory,ruleZones:sim.ruleZones || [],searchInfo,routeValidation:sim.routeValidation || null,candidates,decision:sim.decision,metrics:sim.metrics,history:sim.history,logs:sim.logs,complete:sim.complete,collision:sim.collision};
    download(new Blob([JSON.stringify(report,null,2)],{type:'application/json'}),`colregs-v2-${scenarioId}-${sim.config.rulesEnabled?'rules':'baseline'}-${Math.round(sim.time)}s.json`);toast('已导出航路、规则区、搜索与回验记录');
  });
  $('capture-btn').addEventListener('click',()=>{canvas.toBlob(blob=>{if(blob)download(blob,`colregs-${scenarioId}-${Math.round(sim.time)}s.png`);});});
  const dialog=$('about-dialog');
  for(const id of ['about-btn','details-btn']) $(id).addEventListener('click',()=>{running=false;renderUi();dialog.showModal();});
  $('close-dialog').addEventListener('click',()=>dialog.close());
  dialog.addEventListener('click',event=>{if(event.target===dialog){const r=dialog.getBoundingClientRect();if(event.clientX<r.left||event.clientX>r.right||event.clientY<r.top||event.clientY>r.bottom)dialog.close();}});
  document.addEventListener('keydown',event=>{if(event.code==='Space'&&!/INPUT|BUTTON|SELECT|TEXTAREA/.test(event.target.tagName)&&!dialog.open){event.preventDefault();$('play-btn').click();}});
  document.addEventListener('visibilitychange',()=>{if(document.hidden&&running){running=false;renderUi();}});

  function screen(p){return {x:width/2+p.x*scale,y:height/2-p.y*scale};}
  function world(p){return {x:(p.x-width/2)/scale,y:(height/2-p.y)/scale};}
  function path(points,color,lineWidth=1,dash=[],opacity=1){
    if(!points||points.length<2)return;
    ctx.save();ctx.strokeStyle=color;ctx.lineWidth=lineWidth;ctx.globalAlpha=opacity;ctx.setLineDash(dash);ctx.lineJoin='round';ctx.lineCap='round';ctx.beginPath();
    points.forEach((p,i)=>{const q=screen(p);if(i===0)ctx.moveTo(q.x,q.y);else ctx.lineTo(q.x,q.y);});ctx.stroke();ctx.restore();
  }
  function circle(p,r,color,fill=null,dash=[]){const q=screen(p);ctx.save();ctx.beginPath();ctx.arc(q.x,q.y,r*scale,0,Math.PI*2);if(fill){ctx.fillStyle=fill;ctx.fill();}ctx.strokeStyle=color;ctx.lineWidth=1;ctx.setLineDash(dash);ctx.stroke();ctx.restore();}
  function label(text,x,y,color=colors.text,bg=true){
    ctx.save();ctx.font='10px "Microsoft YaHei", sans-serif';const w=ctx.measureText(text).width;
    if(bg){ctx.fillStyle='#f6faf9e8';ctx.fillRect(x-4,y-11,w+8,16);}ctx.fillStyle=color;ctx.fillText(text,x,y);ctx.restore();
  }
  function arrow(a,b,color,widthLine=1.4){const p=screen(a),q=screen(b),angle=Math.atan2(q.y-p.y,q.x-p.x);ctx.save();ctx.strokeStyle=color;ctx.lineWidth=widthLine;ctx.beginPath();ctx.moveTo(p.x,p.y);ctx.lineTo(q.x,q.y);ctx.lineTo(q.x-6*Math.cos(angle-.5),q.y-6*Math.sin(angle-.5));ctx.moveTo(q.x,q.y);ctx.lineTo(q.x-6*Math.cos(angle+.5),q.y-6*Math.sin(angle+.5));ctx.stroke();ctx.restore();}
  function drawRuleZones(){
    if(!$('show-rules').checked)return;
    for(const zone of sim.ruleZones || []){
      if(!zone.points || zone.points.length<3)continue;
      const safety=zone.kind==='safety', color=safety?colors.safety:colors.rule;
      ctx.save();ctx.fillStyle=safety?'#dfac5020':'#b28bc52b';ctx.strokeStyle=color;ctx.lineWidth=1;ctx.setLineDash(safety?[3,4]:[7,4]);ctx.beginPath();
      zone.points.forEach((point,i)=>{const p=screen(point);if(i===0)ctx.moveTo(p.x,p.y);else ctx.lineTo(p.x,p.y);});ctx.closePath();ctx.fill();ctx.stroke();ctx.restore();
      const center=zone.points.reduce((p,q)=>({x:p.x+q.x/zone.points.length,y:p.y+q.y/zone.points.length}),{x:0,y:0}),p=screen(center);
      label(`${safety?'安全区':'规则区'} · ${zone.label || zone.id || ''}`,Math.min(width-115,Math.max(10,p.x-36)),Math.max(70,Math.min(height-55,p.y)),color);
    }
  }
  function drawSearch(){
    if(!$('show-search').checked)return;
    const closed=sim.searchInfo?.closed || [];
    // Subsample only the display when large; search statistics remain exact.
    const stride=Math.max(1,Math.ceil(closed.length/8000));
    ctx.save();ctx.fillStyle=colors.search;ctx.globalAlpha=.33;
    for(let i=0;i<closed.length;i+=stride){const p=screen(closed[i]);ctx.fillRect(p.x-1.1,p.y-1.1,2.2,2.2);}
    ctx.restore();path(sim.searchInfo?.rawRoute,colors.search,1,[2,3],.7);
  }
  function drawRejoin(){
    const point=sim.searchInfo?.rejoin;if(!point)return;
    const p=screen(point);ctx.save();ctx.strokeStyle=colors.detour;ctx.fillStyle='#fffaf1';ctx.lineWidth=1.8;ctx.translate(p.x,p.y);ctx.rotate(Math.PI/4);ctx.fillRect(-4,-4,8,8);ctx.strokeRect(-4,-4,8,8);ctx.restore();
    label('接回点',Math.max(7,Math.min(width-48,p.x+10)),Math.max(70,Math.min(height-45,p.y-9)),colors.safety);
  }
  function ship(vessel,color,name){
    const q=screen(vessel);ctx.save();ctx.translate(q.x,q.y);ctx.rotate(-vessel.yaw);const len=Math.max(16,vessel.length*scale),beam=Math.max(8,vessel.width*scale);
    ctx.shadowBlur=5;ctx.shadowColor='#17364525';ctx.fillStyle=color;ctx.strokeStyle='#fff';ctx.lineWidth=1.4;ctx.beginPath();ctx.moveTo(len*.65,0);ctx.lineTo(len*.12,-beam*.5);ctx.lineTo(-len*.5,-beam*.5);ctx.lineTo(-len*.5,beam*.5);ctx.lineTo(len*.12,beam*.5);ctx.closePath();ctx.fill();ctx.stroke();ctx.shadowBlur=0;
    ctx.fillStyle='#ffffffb0';ctx.fillRect(-len*.22,-beam*.22,len*.29,beam*.44);ctx.restore();
    const end={x:vessel.x+Math.cos(vessel.yaw)*Math.max(8,vessel.speed*7),y:vessel.y+Math.sin(vessel.yaw)*Math.max(8,vessel.speed*7)};
    arrow(vessel,end,color,1.3);label(name,q.x+12,q.y-13,color);
  }
  function draw() {
    if(!sim)return;
    scale=Math.min((width-64)/320,(height-64)/220)*zoom;
    ctx.clearRect(0,0,width,height);ctx.fillStyle='#eaf2f3';ctx.fillRect(0,0,width,height);
    // A metric grid, with equal scaling on both axes and no fictitious coastline.
    const tick=zoom<.8?40:20;
    const xmin=-width/2/scale,xmax=width/2/scale,ymin=-height/2/scale,ymax=height/2/scale;
    ctx.save();ctx.strokeStyle=colors.grid;ctx.lineWidth=.7;ctx.font='9px Consolas, monospace';ctx.fillStyle='#96aeb8';
    for(let x=Math.ceil(xmin/tick)*tick;x<=xmax;x+=tick){const q=screen({x,y:0});ctx.beginPath();ctx.moveTo(q.x,0);ctx.lineTo(q.x,height);ctx.stroke();if(q.x>26&&q.x<width-35)ctx.fillText(String(x),q.x+3,height-30);}
    for(let y=Math.ceil(ymin/tick)*tick;y<=ymax;y+=tick){const q=screen({x:0,y});ctx.beginPath();ctx.moveTo(0,q.y);ctx.lineTo(width,q.y);ctx.stroke();if(q.y>55&&q.y<height-40)ctx.fillText(String(y),8,q.y-3);}
    ctx.restore();
    drawRuleZones();drawSearch();
    for(const ob of sim.obstacles||[]){circle(ob,ob.r,'#acbfae','#cedbd0');circle(ob,ob.r+3,'#bacbbd',null,[3,4]);const p=screen(ob);label('静态障碍',p.x-22,p.y+3,'#758e79',false);}
    path(sim.globalRoute,colors.route,1.6,[6,6],.85);
    if($('show-trails').checked){path(sim.history?.own,colors.own,2,[],.46);path(sim.history?.target,colors.target,1.8,[],.45);}
    if($('show-candidates').checked){for(const candidate of sim.candidates||[]){if(!candidate.chosen){path(candidate.route,candidate.safe?'#92b2b7':'#c69087',1,[2,5],.65);path(candidate.trajectory,candidate.safe?'#9ab9bc':'#cfaaa2',.8,[],.5);}}}
    if(sim.config.rulesEnabled&&sim.activeRoute?.length>1){path(sim.activeRoute,colors.detour,1.8,[],.85);const stride=Math.max(1,Math.ceil(sim.activeRoute.length/12));for(const p of sim.activeRoute.slice(1,-1).filter((_,i)=>i%stride===0)){const q=screen(p);ctx.save();ctx.fillStyle='#fff9ed';ctx.strokeStyle=colors.detour;ctx.beginPath();ctx.arc(q.x,q.y,3,0,Math.PI*2);ctx.fill();ctx.stroke();ctx.restore();}}
    path(sim.trajectory,colors.own,3,[],.95);
    for(const p of (sim.trajectory||[]).filter((_,i)=>i%10===0)){const q=screen(p);ctx.save();ctx.fillStyle='#fff';ctx.strokeStyle=colors.own;ctx.beginPath();ctx.arc(q.x,q.y,2.3,0,Math.PI*2);ctx.fill();ctx.stroke();ctx.restore();}
    const own=drag?.key==='own'?{...sim.own,...drag.point}:sim.own;
    const target=drag?.key==='target'?{...sim.target,...drag.point}:sim.target;
    if(target){
      circle(target,sim.config.safetyDistance,'#c68c6d55','#dbaa790a',[4,5]);
      if($('show-prediction').checked){const horizon=sim.config.horizon||20;const end={x:target.x+Math.cos(target.yaw)*target.speed*horizon,y:target.y+Math.sin(target.yaw)*target.speed*horizon};path([target,end],colors.target,1.4,[4,5],.65);const q=screen(end);label(`目标 +${horizon} s`,q.x+5,q.y-4,colors.target);
        const t=sim.metrics?.tcpa;
        if(Number.isFinite(t)&&t>0&&t<120){const a={x:own.x+Math.cos(own.yaw)*own.speed*t,y:own.y+Math.sin(own.yaw)*own.speed*t},b={x:target.x+Math.cos(target.yaw)*target.speed*t,y:target.y+Math.sin(target.yaw)*target.speed*t};path([a,b],'#8b83a4',1,[3,5],.8);for(const p of [a,b])circle(p,1.4,'#8b83a4','#f1edf7');const q=screen({x:(a.x+b.x)/2,y:(a.y+b.y)/2});label('匀速 CPA',q.x+7,q.y-6,'#8b83a4');}
      }
    }
    if(sim.goal){const p=screen(sim.goal);ctx.save();ctx.strokeStyle='#8097a2';ctx.lineWidth=1.3;ctx.beginPath();ctx.arc(p.x,p.y,7,0,Math.PI*2);ctx.moveTo(p.x-11,p.y);ctx.lineTo(p.x+11,p.y);ctx.moveTo(p.x,p.y-11);ctx.lineTo(p.x,p.y+11);ctx.stroke();ctx.restore();label('最终目标',p.x-18,p.y+23,'#738d99');}
    drawRejoin();ship(own,colors.own,'自船 OS');if(target)ship(target,colors.target,'目标 TS');
    ctx.save();ctx.translate(width-31,30);ctx.strokeStyle='#7e9ba6';ctx.fillStyle='#7e9ba6';ctx.lineWidth=1;ctx.beginPath();ctx.moveTo(0,18);ctx.lineTo(0,-3);ctx.moveTo(-3,3);ctx.lineTo(0,-3);ctx.lineTo(3,3);ctx.stroke();ctx.font='10px Consolas';ctx.fillText('N',-3,-9);ctx.restore();
    const bar=20*scale;ctx.save();ctx.strokeStyle='#7897a2';ctx.lineWidth=1.4;ctx.beginPath();ctx.moveTo(width-20-bar,height-20);ctx.lineTo(width-20,height-20);ctx.moveTo(width-20-bar,height-24);ctx.lineTo(width-20-bar,height-17);ctx.moveTo(width-20,height-24);ctx.lineTo(width-20,height-17);ctx.stroke();ctx.fillStyle='#7d98a2';ctx.font='9px Consolas';ctx.fillText('20 m',width-20-bar,height-29);ctx.restore();
  }
  function resize(){const rect=$('chart-wrap').getBoundingClientRect();width=rect.width;height=rect.height;const dpr=Math.min(window.devicePixelRatio||1,2);canvas.width=Math.round(width*dpr);canvas.height=Math.round(height*dpr);ctx.setTransform(dpr,0,0,dpr,0,0);draw();}
  new ResizeObserver(resize).observe($('chart-wrap'));
  function pointer(event){const r=canvas.getBoundingClientRect();return{x:event.clientX-r.left,y:event.clientY-r.top};}
  canvas.addEventListener('pointerdown',event=>{
    if(running){toast('先暂停，再拖动船位重设场景');return;}
    const p=pointer(event);let selected=null,min=22;
    for(const key of ['own','target']){if(!sim[key])continue;const q=screen(sim[key]),d=Math.hypot(q.x-p.x,q.y-p.y);if(d<min){selected=key;min=d;}}
    if(!selected)return;drag={key:selected,point:world(p)};canvas.setPointerCapture(event.pointerId);canvas.style.cursor='grabbing';
  });
  canvas.addEventListener('pointermove',event=>{if(!drag)return;const p=world(pointer(event));drag.point={x:Math.max(-145,Math.min(145,p.x)),y:Math.max(-95,Math.min(95,p.y))};draw();});
  function endDrag(){if(!drag)return;const key=drag.key,point=drag.point;const next={own:{x:sim.own.x,y:sim.own.y,yaw:sim.own.yaw}};if(sim.target)next.target={x:sim.target.x,y:sim.target.y,yaw:sim.target.yaw};next[key]={...next[key],...point};overrides=next;drag=null;canvas.style.cursor='default';reset();toast('已更新初始船位，仿真重新计时');}
  canvas.addEventListener('pointerup',endDrag);canvas.addEventListener('pointercancel',()=>{drag=null;canvas.style.cursor='default';draw();});
  canvas.addEventListener('wheel',event=>{event.preventDefault();zoom=Math.max(.6,Math.min(2.8,zoom*Math.exp(-event.deltaY*.001)));draw();},{passive:false});
  function frame(now){
    const elapsed=previousFrame?Math.min((now-previousFrame)/1000,.2):0;previousFrame=now;
    if(running){accumulator+=elapsed*playbackRate;let count=0;while(accumulator>=.1&&count<20){E.step(sim,.1);accumulator-=.1;count++;if(sim.complete||sim.collision){running=false;accumulator=0;break;}}draw();}
    if(now-lastUiUpdate>120){renderUi();lastUiUpdate=now;}
    requestAnimationFrame(frame);
  }
  window.demoApp={getState:()=>({...sim,running,playbackRate,layers:{rules:$('show-rules').checked,search:$('show-search').checked,trails:$('show-trails').checked,prediction:$('show-prediction').checked,candidates:$('show-candidates').checked}}),advance,reset,setScenario,screenPoint:screen};
  reset();resize();requestAnimationFrame(frame);
})();
