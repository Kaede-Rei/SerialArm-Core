/* Actual calibration overlay restored from read-only task evidence.
 * DISPLAY=:1 node_modules/.bin/electron --no-sandbox scripts/render-calibration-overlay.cjs \
 *   --record /path/to/model-calibration-task --clip-audit /path/to/clip-audit.js
 */
const {app,BrowserWindow}=require('electron');
const fs=require('node:fs'),path=require('node:path');
const {buildFixture}=require('./studio-ui-fixture.cjs');
const args=process.argv.slice(2),arg=(name,fallback)=>{const i=args.indexOf(name);return i<0?fallback:args[i+1];};
const record=arg('--record','');if(!record)throw Error('--record task directory is required');
const readabilityOnly=args.includes('--readability-only');
const motionDecision=args.includes('--motion-decision');
const progressFixture=args.includes('--motion-progress-fixture');
const out=path.resolve(arg('--out','/tmp/serialarm-calibration-overlay/screenshots'));
const fixture=buildFixture(path.dirname(out),{record});
const audit=arg('--clip-audit','')?fs.readFileSync(arg('--clip-audit',''),'utf8'):null;
const sources=['gain-candidate.json','result.json','metadata.json','frames.csv'].map(name=>({file:path.join(path.resolve(record),name),bytes:fs.readFileSync(path.join(path.resolve(record),name))}));
app.commandLine.appendSwitch('force-device-scale-factor','1');
function setupOverlay(theme,initializing=false,preview=true,motion=false){
 const data=window.studioUiData,r=data.overlayRecord,source=r.result,experiment=motion?r.candidate.candidate_comparisons.filter(x=>x.kind==='motion_establishment').at(-1):r.candidate.experiments.find(x=>x.kind==='independent_torque_disturbance')||r.candidate.experiments.filter(x=>x.kind==='reference_tracking').at(-1);
 const measured=r.candidate.experiments.find(x=>x.record_path===experiment.record_path&&Array.isArray(x.kp));
 const diagnostic=structuredClone(experiment);
 if(motion){diagnostic.current_kp??=measured?.kp;diagnostic.current_kd??=measured?.kd;}
 Object.assign(state,{page:'calibration',theme,language:'zh-CN',config:{profile:'tomato_picker（只读阶段还原）',profile_file:'',serial_port:'',baudrate:'',bus:'',resource_paths:''},info:structuredClone(data.info),profileEditor:structuredClone(data.editor),capabilityProfile:structuredClone(data.editor),session:{state:'running',mode:'workbench'},calibrationEntry:'gain',calibrationToolSelected:{gain:{directory:r.directory,gain_candidate:structuredClone(r.candidate)}},calibrationRuntimeCandidates:{},modelData:null,admittance:null,tuneDraft:null});
 const status={...structuredClone(source),phase:'gain_adaptation',error:'',gain_probe_stage:initializing?'model_initialization':motion?'motion_discovery':'disturbance_identification',gain_probe_kind:initializing?'model-initialization':motion?'motion_establishment':'independent_torque_disturbance',gain_probe_message:initializing?'model_seed_unmeasured':motion?'establishing_motion':'known_torque_disturbance',gain_probe_diagnostic:initializing?null:diagnostic,gain_candidate_index:initializing?0:source.gain_adaptation_revisions,gain_probe_kp:measured?.kp?.[0]||0,gain_probe_kd:measured?.kd?.[0]||0};
 const vector=key=>r.candidate.joint_names.map(name=>{const value=Number(r.frame[key+':'+name]);return Number.isFinite(value)?value:0;});
 state.telemetry={robot_state:'ACTIVE',valid:false,joint_names:r.candidate.joint_names,feedback_age_ms:Number(r.frame.feedback_age_ms)||0,model_calibration:status,joint:{pos:vector('q'),nominal_ref_pos:vector('nominal_q'),ref_pos:vector('ref_q')}};
 renderedPage='';shell();updateModelCalibrationTaskOverlay();
 if(!preview){restoreCalibrationTaskPreview();document.querySelector('#model-calibration-task-preview').hidden=true;}
 const note=document.createElement('span');note.className='hint';note.dataset.overlayFixtureNote='true';note.textContent='只读阶段还原；原任务已结束';document.querySelector('.task-window-head > div').append(note);
 return {originalPhase:source.phase,task:source.task_id,experiment:experiment.record_path};
}
function measure(){
 const win=document.querySelector('.model-calibration-task-window'),body=win.querySelector('.task-window-content'),wr=win.getBoundingClientRect(),br=body.getBoundingClientRect();
 const rect=n=>{const r=n.getBoundingClientRect();return {left:r.left,right:r.right,top:r.top,bottom:r.bottom,width:r.width,height:r.height};};
 const actions=[...win.querySelectorAll('.task-window-actions button:not([hidden])')].map(n=>({id:n.id,disabled:n.disabled,...rect(n)}));
 const tables=[...win.querySelectorAll('.table-scroll')].map(n=>({width:n.clientWidth,scrollWidth:n.scrollWidth,scrollLeft:n.scrollLeft,...rect(n)}));
 return {window:rect(win),body:{...rect(body),clientWidth:body.clientWidth,scrollWidth:body.scrollWidth,scrollHeight:body.scrollHeight,clientHeight:body.clientHeight,scrollTop:body.scrollTop},actions,tables,documentWidth:document.documentElement.scrollWidth,viewport:{width:innerWidth,height:innerHeight},bodyContained:br.left>=wr.left&&br.right<=wr.right};
}
function setupSyntheticProgress(blocked){
 const note=state.telemetry.model_calibration.gain_probe_diagnostic;
 note.blocked_axes=blocked?[2]:[];note.retry=!blocked;
 note.axes[2].input_response_progress={planned_input_nm:1.2,proposed_input_nm:1.21,observed_input_nm:note.axes[2].applied_pd_peak_nm,response_plateau_pending:!blocked,input_budget_plateau:true,blocked,reason:blocked?'axis_response_to_increased_effort_plateau_confirmed_by_slower_complete_set_measurement':'response_effort_plateau_requires_slower_complete_gain_set_confirmation'};
 updateModelCalibrationTaskOverlay();
 document.querySelector('[data-overlay-fixture-note]').textContent='合成接口状态（非实测）；原始任务数据未修改';
 const b=document.querySelector('.task-window-content');b.scrollTop=b.scrollHeight;for(const n of b.querySelectorAll('.table-scroll'))n.scrollLeft=n.scrollWidth;
 const d=document.querySelector('#gain-task-probe-diagnostic'),row=d.querySelectorAll('tbody tr')[2];
 return {text:d.innerText,cells:[...row.children].map(n=>({text:n.innerText,title:n.title}))};
}
app.whenReady().then(async()=>{
 fs.mkdirSync(out,{recursive:true});
 const win=new BrowserWindow({width:1280,height:1000,show:false,webPreferences:{preload:fixture.preload,sandbox:false,backgroundThrottling:false}});
 const errors=[];win.webContents.on('console-message',(_e,level,message)=>{if(level>=3){errors.push(message);console.error(message);}});
 const settle=()=>win.webContents.executeJavaScript('new Promise(resolve=>setTimeout(()=>requestAnimationFrame(()=>requestAnimationFrame(resolve)),230))');
 await win.loadFile(fixture.index);win.show();win.focus();win.webContents.focus();await settle();
 await win.webContents.executeJavaScript(`term?.dispose();term=null;fit=null;window.Terminal=undefined;window.overlayFixtureSetup=${setupOverlay};window.overlayFixtureMeasure=${measure};window.overlayFixtureProgress=${setupSyntheticProgress};void 0;`);
 const report={runtime:process.versions.electron,record:path.resolve(record),sources:sources.map(x=>({file:x.file,bytes:x.bytes.length})),description:'Readonly experiment; active stage reconstructed for layout, original terminal phase retained in report',checks:[],renders:[]};
 const assert=(ok,label)=>{if(!ok)throw Error(label);report.checks.push(label);};
 const faultCopy=await win.webContents.executeJavaScript(`(()=>{state.language='zh-CN';const zh=describeRuntimeFault({fault:{code:'CALIBRATION_RESPONSE_UNSTABLE'}});state.language='en';const en=describeRuntimeFault({fault:{code:'CALIBRATION_RESPONSE_UNSTABLE'}});return {zh,en};})()`);
 assert(faultCopy.zh.message.includes('非指令要求的持续位置振荡')&&faultCopy.zh.advice.includes('现有流程恢复')&&!faultCopy.zh.message.includes('共振'),'latched instability fault uses measured-position wording and existing recovery');
 assert(faultCopy.en.message.includes('not requested by the reference')&&faultCopy.en.advice.includes('existing fault-recovery flow'),'instability fault includes clear English guidance');
 const checkBounds=(m,label)=>{
  assert(m.window.left>=0&&m.window.right<=m.viewport.width&&m.window.top>=0&&m.window.bottom<=m.viewport.height,label+' window inside viewport');
  assert(m.bodyContained&&m.body.scrollWidth<=m.body.clientWidth+1,label+' evidence body has no horizontal overflow');
  assert(m.actions.every(n=>n.left>=m.window.left&&n.right<=m.window.right&&n.top>=0&&n.bottom<=m.viewport.height),label+' safety actions stay visible');
  assert(m.actions.filter(n=>['model-calibration-task-hold','model-calibration-task-cancel','model-calibration-task-disable'].includes(n.id)).length===3,label+' hold cancel disable remain present');
 };
 const runAudit=()=>win.webContents.executeJavaScript(`(${audit})({root:'body',viewport:true})`);
 const capture=async(item,state)=>{await settle();const m=await win.webContents.executeJavaScript('window.overlayFixtureMeasure()');checkBounds(m,item.width+' '+item.theme+' '+state);const file=path.join(out,`overlay-${item.width}-${item.theme}-${state}.png`);fs.writeFileSync(file,(await win.webContents.capturePage()).toPNG());const entry={state,file,metrics:m};if(audit){entry.clip=await runAudit();assert(entry.clip.cut===0,item.width+' '+item.theme+' '+state+' clip=0');}item.states.push(entry);};
 for(const [width,height] of (readabilityOnly?[[1168,1412]]:[[1280,1000],[1168,1412],[375,812]]))for(const theme of (readabilityOnly?['light']:['light','dark'])){
  win.setContentSize(width,height);const item={width,height,theme,states:[]};
  if(readabilityOnly){
   item.source=await win.webContents.executeJavaScript(`window.overlayFixtureSetup(${JSON.stringify(theme)},false,true,${motionDecision})`);
   await win.webContents.executeJavaScript(`(()=>{document.querySelector('#gain-task-probe .gain-probe-evidence').open=true;const b=document.querySelector('.task-window-content');b.scrollTop=b.scrollHeight;for(const n of b.querySelectorAll('.table-scroll'))n.scrollLeft=n.scrollWidth;})()`);
   const findings=await win.webContents.executeJavaScript(`(()=>{const d=document.querySelector('#gain-task-probe-diagnostic');return {text:d.innerText,cells:[...d.querySelectorAll('td[title]')].map(n=>({code:n.title,text:n.innerText}))};})()`);
   if(motionDecision){
    assert(findings.text.includes('本阶段已停止，尚未建立有效运动')&&findings.text.includes('joint3'),'recorded motion establishment stop remains readable');
    assert(findings.text.includes('2.7808')&&findings.text.includes('0.1093')&&findings.text.includes('0.0656')&&findings.text.includes('0.1311'),'recorded joint3 applied PD and bidirectional travel are displayed');
    assert(findings.cells.some(n=>n.code==='measured_insufficient_motion_bounded_stiffness_step_no_damping_inference'&&n.text.includes('实测双向行程不足')),'recorded motion reason keeps its raw code in the tooltip');
   }else for(const [code,label] of [['response_below_encoder_noise_floor','响应低于编码器与噪声门槛'],['closed_loop_model_not_identifiable','闭环模型尚不可辨识'],['independent_response_below_motion_floor_static_behavior_only','独立扰动位移不足，仅有静态证据'],['sustained_oscillation_after_release','释放后持续位置往复']])assert(findings.cells.some(n=>n.code===code&&n.text.includes(label))&&!findings.text.includes(code),'recorded reason is readable and keeps its original code in the tooltip: '+code);
   await capture(item,'readable-findings');
   if(motionDecision&&progressFixture){
    for(const blocked of [false,true]){
     const preview=await win.webContents.executeJavaScript(`window.overlayFixtureProgress(${blocked})`);
     assert(preview.cells[3].text==='2.7808'&&preview.cells[4].text==='本窗 1.2\n候选 1.21','synthetic interface keeps actual PD distinct from planned and proposed reference drive');
     assert(preview.text.includes(blocked?'受阻关节 joint3':'等待确认测量')&&preview.cells.at(-1).title.includes(blocked?'confirmed_by_slower':'requires_slower'),'synthetic pending and blocked states keep readable reasons and raw tooltips');
     await capture(item,blocked?'synthetic-blocked':'synthetic-pending');
    }
    const narrow={width:375,height:812,theme:'dark',source:{syntheticInterface:true},states:[]};win.setContentSize(narrow.width,narrow.height);
    await win.webContents.executeJavaScript(`state.theme='dark';applyTheme();const b=document.querySelector('.task-window-content');b.scrollTop=b.scrollHeight;for(const n of b.querySelectorAll('.table-scroll'))n.scrollLeft=n.scrollWidth;void 0;`);
    await capture(narrow,'synthetic-blocked');report.renders.push(narrow);
   }
   report.renders.push(item);continue;
  }
  item.source=await win.webContents.executeJavaScript(`window.overlayFixtureSetup(${JSON.stringify(theme)},true)`);await capture(item,'initializing');
  item.source=await win.webContents.executeJavaScript(`window.overlayFixtureSetup(${JSON.stringify(theme)})`);await capture(item,'default');
  const windowCount=await win.webContents.executeJavaScript(`(()=>{const s=state.telemetry.model_calibration,old=s.gain_adaptation_passes;s.gain_adaptation_passes=15;refreshGainProbeStatus(s);const text=document.querySelector('#gain-task-probe-windows').textContent;s.gain_adaptation_passes=old;refreshGainProbeStatus(s);return text;})()`);assert(windowCount==='15',width+' '+theme+' completed windows are not clamped or given the obsolete twelve-window ceiling');
  const stayedOpen=await win.webContents.executeJavaScript(`(()=>{const d=document.querySelector('#gain-task-probe .gain-probe-evidence');d.open=true;updateModelCalibrationTaskOverlay();return d.isConnected&&d.open;})()`);assert(stayedOpen,width+' '+theme+' measurement refresh keeps details expanded');await capture(item,'expanded');
  const tableCheck=await win.webContents.executeJavaScript(`(()=>{const d=document.querySelector('#gain-task-probe-diagnostic'),tables=d.querySelectorAll('table');return {rows:d.querySelectorAll('tbody tr').length,text:d.innerText,fits:[...tables[0].querySelectorAll('tbody tr')].every(row=>[3,4,5,6].every(i=>row.children[i].textContent==='—')),recovery:tables[1].textContent.includes('0.6044')};})()`);
  assert(tableCheck.rows===12,width+' '+theme+' all six FRF and six recovery rows exist');
  assert(!tableCheck.text.includes('NaN'),width+' '+theme+' missing evidence does not render NaN');
  assert(tableCheck.fits&&tableCheck.recovery,width+' '+theme+' invalid fits stay unknown while recorded disturbance measurements remain visible');
  const wheel=await win.webContents.executeJavaScript(`(()=>{const b=document.querySelector('.task-window-content'),r=b.getBoundingClientRect();b.scrollTop=0;return {x:Math.round(r.left+6),y:Math.round(r.top+16),needed:b.scrollHeight>b.clientHeight};})()`);
  win.webContents.sendInputEvent({type:'mouseMove',x:wheel.x,y:wheel.y});await settle();win.webContents.sendInputEvent({type:'mouseWheel',x:wheel.x,y:wheel.y,deltaY:-400,deltaX:0,canScroll:true});await settle();
  const wheelTop=await win.webContents.executeJavaScript('document.querySelector(".task-window-content").scrollTop');assert(!wheel.needed||wheelTop>0,width+' '+theme+' native wheel scrolls the evidence body '+JSON.stringify({wheel,wheelTop}));
  await win.webContents.executeJavaScript(`(()=>{const b=document.querySelector('.task-window-content');b.scrollTop=b.scrollHeight;for(const n of b.querySelectorAll('.table-scroll'))n.scrollLeft=n.scrollWidth;})()`);await capture(item,'scrolled-end');
  const local=await win.webContents.executeJavaScript(`(()=>{const list=[...document.querySelectorAll('#gain-task-probe-diagnostic .table-scroll')];return list.map(n=>{const r=n.getBoundingClientRect(),cell=n.querySelector('thead tr').lastElementChild.getBoundingClientRect();return {max:n.scrollWidth-n.clientWidth,scroll:n.scrollLeft,lastHeaderVisible:cell.right<=r.right+1&&cell.left>=r.left-1};});})()`);
  assert(local.every(n=>n.max<=1||n.scroll>0&&n.lastHeaderVisible),width+' '+theme+' final diagnostic column reachable by local horizontal scroll');
  const focus=await win.webContents.executeJavaScript(`(()=>{const ids=['model-calibration-task-hold','model-calibration-task-cancel','model-calibration-task-disable'];return ids.map(id=>{const n=document.getElementById(id);n.focus({preventScroll:true});return document.activeElement===n;});})()`);assert(focus.every(Boolean),width+' '+theme+' keyboard can reach all safety actions');
  const point=await win.webContents.executeJavaScript(`(()=>{const r=document.querySelector('#model-calibration-task-disable').getBoundingClientRect();return {x:Math.round(r.left+r.width/2),y:Math.round(r.top+r.height/2)};})()`);win.webContents.sendInputEvent({type:'mouseMove',...point});await capture(item,'hover');
  await win.webContents.executeJavaScript(`window.overlayFixtureSetup(${JSON.stringify(theme)},false,false);document.querySelector('#gain-task-probe .gain-probe-evidence').open=true;void 0;`);await capture(item,'without-preview');
  report.renders.push(item);
 }
 report.apiCalls=await win.webContents.executeJavaScript('window.serialArm.fixtureCalls()');report.consoleErrors=errors;
 assert(errors.length===0,'no renderer console errors');
 assert(sources.every(item=>fs.readFileSync(item.file).equals(item.bytes)),'original task sources remain byte-identical');
 fs.writeFileSync(path.join(out,'report.json'),JSON.stringify(report,null,2));console.log('CALIBRATION_OVERLAY_RENDER_PASS '+JSON.stringify({checks:report.checks.length,layouts:report.renders.length,captures:report.renders.reduce((n,x)=>n+x.states.length,0),report:path.join(out,'report.json')}));win.destroy();app.quit();
}).catch(error=>{console.error(error.stack);app.exit(1);});
