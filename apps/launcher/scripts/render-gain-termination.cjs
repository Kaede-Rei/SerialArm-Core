/* Production Electron task lifecycle, using only read-only saved evidence.
 * DISPLAY=:1 node_modules/.bin/electron --no-sandbox scripts/render-gain-termination.cjs
 *   --record /path/to/task --out /tmp/serialarm-gain-termination --clip-audit /path/to/clip-audit.js
 */
const {app,BrowserWindow}=require('electron');
const fs=require('node:fs'),path=require('node:path');
const {buildFixture}=require('./studio-ui-fixture.cjs');
const args=process.argv.slice(2),arg=(name,fallback)=>{const i=args.indexOf(name);return i<0?fallback:args[i+1];};
const record=arg('--record','');if(!record)throw Error('--record is required');
const out=path.resolve(arg('--out','/tmp/serialarm-gain-termination'));
const fixture=buildFixture(path.join(out,'fixture'),{record});
const audit=arg('--clip-audit','')?fs.readFileSync(arg('--clip-audit',''),'utf8'):null;
const sources=['gain-candidate.json','result.json','metadata.json','frames.csv'].map(name=>({file:path.join(path.resolve(record),name),bytes:fs.readFileSync(path.join(path.resolve(record),name))}));
app.commandLine.appendSwitch('force-device-scale-factor','1');
function setup(theme,identity){
 const data=window.studioUiData,r=data.overlayRecord;
 Object.assign(state,{page:'calibration',theme,language:'zh-CN',config:{profile:'tomato_picker（只读时序还原）',profile_file:'',serial_port:'',baudrate:'',bus:'',resource_paths:''},info:structuredClone(data.info),profileEditor:structuredClone(data.editor),capabilityProfile:structuredClone(data.editor),session:{state:'running',mode:'workbench',session_id:identity},calibrationEntry:'gain',calibrationToolSelected:{gain:{directory:r.directory,result:structuredClone(r.result),gain_candidate:structuredClone(r.candidate)}},calibrationRuntimeCandidates:{},modelData:null,admittance:null,tuneDraft:null});
 const status={...structuredClone(r.result),phase:'gain_adaptation',error:'',gain_warnings:[],gain_probe_stage:'motion_discovery',gain_probe_message:'establishing_motion',termination:{kind:'running',reason:'',blocked_joint_names:[]}};
 state.telemetry={robot_state:'ACTIVE',valid:false,joint_names:r.candidate.joint_names,model_calibration:status,joint:{pos:[],nominal_ref_pos:[]}};
 priorityErrorOpen=false;renderedPage='';shell();updateModelCalibrationTaskOverlay();
 const note=document.createElement('span');note.className='hint';note.textContent='只读时序还原；原任务与设备均未修改';document.querySelector('.task-window-head > div').append(note);
 return {originalPhase:r.result.phase,task:r.result.task_id};
}
function inspect(){
 const overlay=document.querySelector('#model-calibration-overlay'),win=overlay.querySelector('.model-calibration-task-window'),body=win.querySelector('.task-window-content');
 const rect=n=>{const r=n.getBoundingClientRect();return {left:r.left,right:r.right,top:r.top,bottom:r.bottom,width:r.width,height:r.height};};
 const tables=[...win.querySelectorAll('[id$="-diagnostic"] .table-scroll')].filter(n=>n.clientWidth).map(n=>({width:n.clientWidth,scrollWidth:n.scrollWidth,scrollLeft:n.scrollLeft,box:rect(n),first:rect(n.querySelector('th')),last:rect(n.querySelector('thead tr').lastElementChild)}));
 return {hidden:overlay.hidden,title:document.querySelector('#model-calibration-task-title').textContent,reason:document.querySelector('#model-calibration-task-conclusion').textContent,phase:state.telemetry?.model_calibration?.phase,window:rect(win),body:{...rect(body),width:body.clientWidth,scrollWidth:body.scrollWidth,scrollTop:body.scrollTop,height:body.clientHeight,scrollHeight:body.scrollHeight},tables,actions:[...win.querySelectorAll('.task-window-actions button')].filter(n=>!n.hidden).map(n=>({id:n.id,disabled:n.disabled,...rect(n)})),viewport:{width:innerWidth,height:innerHeight}};
}
app.whenReady().then(async()=>{
 fs.mkdirSync(out,{recursive:true});
 const win=new BrowserWindow({width:1555,height:1000,show:false,webPreferences:{preload:fixture.preload,sandbox:false,backgroundThrottling:false}});
 const errors=[];win.webContents.on('console-message',(_e,level,message)=>{if(level>=3)errors.push(message);});
 await win.loadFile(fixture.index);win.show();win.focus();win.webContents.focus();
 const js=code=>win.webContents.executeJavaScript(code),settle=()=>js('new Promise(resolve=>setTimeout(()=>requestAnimationFrame(()=>requestAnimationFrame(resolve)),150))');
 await settle();await js(`term?.dispose();term=null;fit=null;window.Terminal=undefined;window.terminationFixtureSetup=${setup};window.terminationFixtureInspect=${inspect};void 0;`);
 const report={runtime:process.versions.electron,record:path.resolve(record),checks:[],renders:[]};
 const assert=(ok,name)=>{if(!ok)throw Error(name);report.checks.push(name);};
 for(const [width,height] of [[1555,1000],[1280,1000],[375,812]])for(const theme of ['light','dark']){
  const label=`${width} ${theme}`;win.setContentSize(width,height);
  const source=await js(`window.terminationFixtureSetup(${JSON.stringify(theme)},${JSON.stringify(label)})`);await settle();
  const running=await js(`document.querySelector('#gain-task-probe details').open=true;window.terminationFixtureInspect()`);
  assert(!running.hidden,label+' running task is visible');
  if(width>=1280)assert(running.tables.length>0&&running.tables.every(n=>n.scrollWidth<=n.width+1&&n.first.left>=n.box.left-1&&n.last.right<=n.box.right+1),label+' running table includes its final column without horizontal scrolling');
  const ended=await js(`(()=>{state.telemetry.model_calibration=structuredClone(window.studioUiData.overlayRecord.result);updateModelCalibrationTaskOverlay();return window.terminationFixtureInspect();})()`);
  assert(!ended.hidden&&ended.reason.length>5,label+' non-success terminal state retains an explicit reason');
  assert(ended.phase===source.originalPhase,label+' historical terminal phase remains unchanged');
  if(source.originalPhase==='cancelled')assert(ended.title==='标定已停止'&&ended.reason.includes('短测响应试验已中断'),label+' historical cancelled result is not relabelled as a fault');
  else assert(ended.reason.includes('joint3')&&ended.reason.includes('低频激励与输入预算边界'),label+' blocked joint and stopping mechanism are visible');
  const stable=await js(`(()=>{const d=document.querySelector('#gain-task-probe details');d.open=false;for(let i=0;i<8;i++)updateModelCalibrationTaskOverlay();const retained=d.isConnected&&!d.open&&!document.querySelector('#model-calibration-overlay').hidden;d.open=true;const b=document.querySelector('.task-window-content');b.scrollTop=b.scrollHeight;return {retained,main:document.querySelector('#gain-probe-message').textContent};})()`);
  assert(stable.retained,label+' periodic refresh retains the result without reopening collapsed details');
  assert(stable.main.length>5&&!stable.main.includes('标定已完成'),label+' main page retains the actual terminal reason');
  await settle();const metrics=await js('window.terminationFixtureInspect()');
  assert(metrics.window.left>=0&&metrics.window.right<=width&&metrics.window.top>=0&&metrics.window.bottom<=height,label+' terminal window stays inside the viewport');
  assert(metrics.body.scrollWidth<=metrics.body.width+1,label+' body has no horizontal overflow');
  assert(metrics.actions.some(n=>n.id==='model-calibration-task-close')&&metrics.actions.every(n=>n.left>=metrics.window.left&&n.right<=metrics.window.right&&n.top>=0&&n.bottom<=height),label+' close and safety actions remain reachable');
  assert(metrics.actions.every(n=>n.id!=='model-calibration-task-cancel'),label+' ended task no longer offers cancellation');
  if(width>=1280)assert(metrics.tables.length>0&&metrics.tables.every(n=>n.scrollWidth<=n.width+1&&n.first.left>=n.box.left-1&&n.last.right<=n.box.right+1),label+' every measurement table includes its final column without horizontal scrolling');
  else {
   const local=await js(`(()=>{const tables=[...document.querySelectorAll('#gain-task-probe-diagnostic .table-scroll')];for(const n of tables)n.scrollLeft=n.scrollWidth;return tables.every(n=>{const a=n.getBoundingClientRect(),b=n.querySelector('thead tr').lastElementChild.getBoundingClientRect();return b.right<=a.right+1&&b.left>=a.left-1;});})()`);
   assert(local,label+' narrow diagnostic final column is reachable by local scroll');
  }
  await settle();
  const file=path.join(out,`termination-${width}-${theme}.png`);fs.writeFileSync(file,(await win.webContents.capturePage()).toPNG());
  const entry={width,height,theme,source,file,metrics:await js('window.terminationFixtureInspect()')};if(audit){entry.clip=await js(`(${audit})({root:'body',viewport:true})`);assert(entry.clip.cut===0,label+' clip=0');}report.renders.push(entry);
  const priority=await js(`(()=>{const previous=state.telemetry.robot_state;state.telemetry.robot_state='FAULT';priorityErrorOpen=true;updateModelCalibrationTaskOverlay();const hidden=document.querySelector('#model-calibration-overlay').hidden;priorityErrorOpen=false;updateModelCalibrationTaskOverlay();const retained=hidden&&!document.querySelector('#model-calibration-overlay').hidden;state.telemetry.robot_state=previous;updateModelCalibrationTaskOverlay();return retained;})()`);
  assert(priority,label+' terminal evidence returns after the fault alert is dismissed');
  const closed=await js(`(()=>{document.querySelector('#model-calibration-task-close').focus();const focused=document.activeElement.id==='model-calibration-task-close';document.querySelector('#model-calibration-task-close').click();for(let i=0;i<8;i++)updateModelCalibrationTaskOverlay();render();updateModelCalibrationTaskOverlay();return focused&&document.querySelector('#model-calibration-overlay').hidden&&![...document.querySelectorAll('.shell')].some(n=>n.inert);})()`);
  assert(closed,label+' closing is local and repeated refresh does not reopen the same task');
  const mainReason=await js(`(()=>{const n=document.querySelector('.gain-result-outcome');n.scrollIntoView({block:'center'});return n.innerText;})()`);
  assert(source.originalPhase==='cancelled'?mainReason.includes('标定已停止')&&mainReason.includes('短测响应试验已中断'):mainReason.includes('joint3')&&mainReason.includes('低频激励与输入预算边界'),label+' result page shows the main stopping reason after closing the popup');
  if(width===1280&&theme==='light'){await settle();const mainFile=path.join(out,'main-1280-light.png');fs.writeFileSync(mainFile,(await win.webContents.capturePage()).toPNG());report.mainPageCapture=mainFile;}
  const restarted=await js(`(()=>{const original=window.studioUiData.overlayRecord.result;state.telemetry.robot_state='ACTIVE';state.telemetry.model_calibration={...structuredClone(original),task_id:original.task_id+'-next-fixture',phase:'gain_adaptation',error:'',termination:{kind:'running'}};updateModelCalibrationTaskOverlay();const running=!document.querySelector('#model-calibration-overlay').hidden;state.telemetry.model_calibration={...state.telemetry.model_calibration,phase:'failed',error:'测试执行中断',termination:{kind:'execution_fault',reason:'测试执行中断',blocked_joint_names:[]}};updateModelCalibrationTaskOverlay();return running&&!document.querySelector('#model-calibration-overlay').hidden&&document.querySelector('#model-calibration-task-conclusion').textContent==='测试执行中断';})()`);
  assert(restarted,label+' a new failed task is not suppressed by the previous dismissal');
  await js(`dismissCalibrationTaskReview();void 0;`);
  const success=await js(`(()=>{const task=state.telemetry.model_calibration;state.telemetry.model_calibration={...task,task_id:task.task_id+'-qualified-fixture',phase:'gain_adaptation',termination:{kind:'running'}};updateModelCalibrationTaskOverlay();state.telemetry.model_calibration={...state.telemetry.model_calibration,phase:'complete',error:'',gain_available:true,termination:{kind:'qualified',reason:'',blocked_joint_names:[]}};updateModelCalibrationTaskOverlay();return document.querySelector('#model-calibration-overlay').hidden;})()`);
  assert(success,label+' an explicitly qualified task can finish normally');
 }
 report.apiCalls=await js('window.serialArm.fixtureCalls()');report.consoleErrors=errors;
 assert(errors.length===0,'no renderer console errors');
 assert(report.apiCalls.every(name=>['status','logs','profile_library'].includes(name)),'fixture makes no device or write requests');
 assert(sources.every(item=>fs.readFileSync(item.file).equals(item.bytes)),'original task sources remain byte-identical');
 fs.writeFileSync(path.join(out,'report.json'),JSON.stringify(report,null,2));
 console.log('GAIN_TERMINATION_RENDER_PASS '+JSON.stringify({checks:report.checks.length,captures:report.renders.length,report:path.join(out,'report.json')}));win.destroy();app.quit();
}).catch(error=>{console.error(error.stack);app.exit(1);});
