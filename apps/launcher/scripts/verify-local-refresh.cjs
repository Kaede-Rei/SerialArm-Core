/* DISPLAY=:1 node_modules/.bin/electron --no-sandbox scripts/verify-local-refresh.cjs
 * Complete production renderer; persistence and temporary application are simulated
 * only in this fixture's preload memory. No backend, device or Profile is opened. */
const {app,BrowserWindow}=require('electron');
const fs=require('node:fs'),path=require('node:path');
const {buildFixture}=require('./studio-ui-fixture.cjs');
const args=process.argv.slice(2),arg=(name,fallback)=>{const i=args.indexOf(name);return i<0?fallback:args[i+1];};
const out=path.resolve(arg('--out','/tmp/serialarm-local-refresh'));
const record=arg('--record','/home/kaede-rei/AgroTech/repos/ROS2-Workspace/temp-tomato-picker/src/tomato_picker/serial_arm_core_profile/tomato_picker/.serialarm/model-calibration/model-calibration-1794241207035');
const fixture=buildFixture(path.join(out,'fixture'),{record});
const fixtureDataPath=path.join(fixture.directory,'data.json'),fixtureData=JSON.parse(fs.readFileSync(fixtureDataPath,'utf8'));
fixtureData.editor.original_gravity_model_validated=fixtureData.editor.core.safety_policy.fault_recovery?.gravity_model_validated===true;
fs.writeFileSync(fixtureDataPath,JSON.stringify(fixtureData));
const sourceFiles=['gain-candidate.json','result.json','metadata.json','frames.csv'].map(name=>({file:path.join(record,name),bytes:fs.readFileSync(path.join(record,name))}));
const sceneSource=fs.readFileSync(path.join(__dirname,'render-studio-ui.cjs'),'utf8');
const scene=sceneSource.slice(sceneSource.indexOf('function setScene('),sceneSource.indexOf('\nfunction snapshot('));
fs.writeFileSync(fixture.preload,`const {contextBridge}=require('electron');
const data=require(${JSON.stringify(path.join(fixture.directory,'data.json'))});
const calls=[],readiness={state:'configuring',checks:{},evidence:{},control_available:true};let editor=structuredClone(data.editor),applied=false;
const fail=name=>{throw Error('Offline interaction fixture rejects request: '+name);};
contextBridge.exposeInMainWorld('studioUiData',data);
contextBridge.exposeInMainWorld('serialArm',{
 prefs:async()=>({theme:'light',language:'zh-CN'}),savePrefs:async()=>({}),
 request:async(name,params={})=>{calls.push({name,params});
  if(name==='status')return {installed:true,prefix:'/offline/install',session:{state:'idle'}};
  if(name==='logs')return '';
  if(name==='profile_library')return {profiles:data.library};
  if(name==='trajectory_list')return {items:Array.from({length:24},(_,index)=>({id:'offline-trajectory-'+index,name:'离线示教轨迹 '+index,samples:5000,duration_s:25,directory:'/offline/trajectory-'+index,compatible:true}))};
  if(name==='profiles')return {profiles:[{name:data.editor.profile}]};
  if(name==='inspect')return {...data.info,available:{...data.info.available,model:false}};
  if(name==='readiness')return readiness;
  if(name==='readiness_record'){const {stage,answers}=params;readiness.checks[stage]='recorded';readiness.evidence[stage]={note:answers.note,observations:answers};return readiness;}
  if(name==='profile_editor_load')return editor;
  if(name==='profile_editor_save'){editor={...editor,...params.payload};return editor;}
  if(name==='hardware_inspect'||name==='profile_hardware_inspect')return {plugin:'serial_arm_hardware_damiao',available:true,drivers:[{plugin:'serial_arm_hardware_damiao',available:true,label:'达妙 / Damiao'}]};
  if(name==='calibration_tool_load'&&params.kind==='gain'&&params.directory===data.overlayRecord.directory)return {directory:data.overlayRecord.directory,task_id:data.overlayRecord.result.task_id,result:data.overlayRecord.result,gain_candidate:data.overlayRecord.candidate};
  if(name==='calibration_candidate_apply'&&params.kind==='gain'){applied=true;return {gain_applied:true};}
  if(name==='calibration_candidate_restore'&&params.kind==='gain'){applied=false;return {gain_applied:false};}
  if(name==='workbench_request'&&params.method==='get_admittance')return null;
  return fail(name);
 },graphics:{status:async()=>({effectiveMode:'normal',requestedMode:'normal',requiresRestart:false,status:'ready'}),report:async()=>({}),setMode:async()=>fail('graphics.setMode')},
 onEvent:()=>{},onCloseRequest:()=>{},window:()=>{},clipboard:{write:async()=>{},read:async()=>''},selectProfile:async()=>null,selectProfilePackage:async()=>null,revealFile:async()=>{},fixtureCalls:async()=>calls
});`);
app.commandLine.appendSwitch('force-device-scale-factor','1');
function viewSnapshot(){
 const root=document.querySelector('#page'),body=root.querySelector('.studio-page-body');
 const active=document.activeElement;
 return {top:body?.scrollTop||0,outer:root.closest('.content').scrollTop,height:body?.scrollHeight,viewport:body?.clientHeight,focus:active?.id||active?.dataset.reviewStage||active?.dataset.profileField||active?.tagName,details:[...root.querySelectorAll('details')].map(n=>n.open),horizontal:[...root.querySelectorAll('.table-scroll')].map(n=>n.scrollLeft)};
}
function prepareReview(stage){
 const node=document.querySelector(`[data-lifecycle-stage="${stage}"] [data-review-stage]`);
 node.scrollIntoView({block:'center'});node.focus({preventScroll:true});return viewSnapshot();
}
function openRecordedGain(){
 setScene('calibration-result','light');
 const r=window.studioUiData.overlayRecord;
 state.calibrationToolSelected.gain={directory:r.directory,task_id:r.result.task_id,result:structuredClone(r.result),gain_candidate:structuredClone(r.candidate)};
 state.session={state:'running',mode:'workbench',session_id:'offline-apply-fixture'};
 state.telemetry={robot_state:'INACTIVE',valid:false,joint_names:r.candidate.joint_names,joint:{pos:[]},model_calibration:{...structuredClone(r.result),phase:'gain_adaptation',error:'',termination:{kind:'running'}}};
 render();updateModelCalibrationTaskOverlay();
 state.telemetry.model_calibration=structuredClone(r.result);updateModelCalibrationTaskOverlay();
 return {visible:!document.querySelector('#model-calibration-overlay').hidden,available:Object.keys(r.candidate.targets).filter(key=>gainCandidatePurposeAvailable(r.candidate,key)),validated:r.candidate.validated};
}
app.whenReady().then(async()=>{
 fs.mkdirSync(out,{recursive:true});
 const win=new BrowserWindow({width:1280,height:1000,show:false,webPreferences:{preload:fixture.preload,sandbox:false,backgroundThrottling:false}});
 const errors=[];win.webContents.on('console-message',(_e,level,message)=>{if(level>=3){errors.push(message);console.error('RENDERER',message);}});
 const js=async code=>{try{return await win.webContents.executeJavaScript(code);}catch(error){throw Error(error.message+'\nIn fixture interaction: '+code);}},settle=()=>js('new Promise(r=>setTimeout(()=>requestAnimationFrame(()=>requestAnimationFrame(r)),550))');
 await win.loadFile(fixture.index);win.showInactive();await settle();
 await js(`term?.dispose();term=null;fit=null;window.Terminal=undefined;window.setScene=${scene};window.viewSnapshot=${viewSnapshot};window.prepareReview=${prepareReview};window.openRecordedGain=${openRecordedGain};void 0;`);
 const report={runtime:process.versions.electron,source:'Production HTML/JS/CSS; fixture-only in-memory persistence and gain application',record,checks:[],captures:[]};
 const assert=(ok,name,details)=>{if(!ok)throw Error(name+' '+JSON.stringify(details||''));report.checks.push(name);};
 const sameView=(a,b,name)=>{assert(Math.abs(a.top-b.top)<=1&&Math.abs(a.outer-b.outer)<=1,name+' preserves actual inner and outer scroll',{a,b});assert(JSON.stringify(a.details)===JSON.stringify(b.details),name+' preserves expanded details');assert(a.focus===b.focus,name+' restores focus',{before:a,after:b});};
 for(const width of [1280,375]){
  win.setContentSize(width,1000);
  await js(`setScene('lifecycle','light');document.querySelectorAll('[data-readiness-section]').forEach(n=>n.open=true);void 0;`);await settle();
  for(const stage of ['mapping_calibration','geometry_calibration','validation']){
   const before=await js(`prepareReview('${stage}')`);assert(before.top>100,'long readiness page is actually scrolled '+width+' '+stage);
   await js(`document.activeElement.click();document.querySelectorAll('[data-review-key]').forEach(n=>n.checked=true);document.querySelector('#readiness-review-note').value='离线连续自查 '+${JSON.stringify(stage)};document.querySelector('#readiness-review-submit').click();void 0;`);await settle();
   const after=await js('viewSnapshot()');sameView(before,after,'readiness save '+width+' '+stage);
   assert(await js(`state.readiness.evidence['${stage}'].note==='离线连续自查 ${stage}'&&!document.querySelector('#confirm-dialog').open`),'real self-check Save handler persists the selected adjacent record '+width+' '+stage);
  }
  const file=path.join(out,`readiness-saved-${width}.png`);fs.writeFileSync(file,(await win.webContents.capturePage()).toPNG());report.captures.push(file);
  await js(`setScene('parameters-motion','light');document.querySelectorAll('#page details').forEach(n=>n.open=true);var fixtureScroller=document.querySelector('.studio-page-body');fixtureScroller.scrollTop=900;for(const n of document.querySelectorAll('.table-scroll'))n.scrollLeft=n.scrollWidth;document.querySelector('#profile-save').focus({preventScroll:true});void 0;`);await settle();
  const before=await js('viewSnapshot()');assert(before.top>100,'parameter page is actually scrolled '+width);
  await js(`document.querySelector('#profile-save').click();void 0;`);await settle();
  const after=await js('({...viewSnapshot(),error:state.profileError,activeField:document.activeElement?.dataset.profileField,dialog:document.querySelector("#confirm-dialog").open})');sameView(before,after,'parameter save '+width);
  assert(JSON.stringify(before.horizontal)===JSON.stringify(after.horizontal),'parameter save preserves nested horizontal tables '+width,{before,after});
  await js(`document.querySelector('#profile-reload').focus({preventScroll:true});void 0;`);
  const reloadBefore=await js('viewSnapshot()');await js(`document.querySelector('#profile-reload').click();void 0;`);await settle();sameView(reloadBefore,await js('viewSnapshot()'),'parameter reload '+width);
  const parameterFile=path.join(out,`parameters-saved-${width}.png`);fs.writeFileSync(parameterFile,(await win.webContents.capturePage()).toPNG());report.captures.push(parameterFile);
 }
 // Ordinary same-page refreshes preserve focus, selection and the real scroller.
 win.setContentSize(1280,1000);
 await js(`setScene('diagnostics','light');document.querySelectorAll('#page details').forEach(n=>n.open=true);var fixtureScroller=document.querySelector('.studio-page-body');fixtureScroller.scrollTop=fixtureScroller.scrollHeight;document.querySelector('#inspect-profile').focus({preventScroll:true});void 0;`);await settle();
 const diagBefore=await js('viewSnapshot()');await js(`document.querySelector('#inspect-profile').click();void 0;`);await settle();sameView(diagBefore,await js('viewSnapshot()'),'connection inspection');
 await js(`document.querySelector('#refresh-profiles').focus({preventScroll:true});void 0;`);const refreshBefore=await js('viewSnapshot()');await js(`document.querySelector('#refresh-profiles').click();void 0;`);await settle();sameView(refreshBefore,await js('viewSnapshot()'),'Profile list refresh');
 await js(`setScene('parameters-hardware','light');document.querySelectorAll('#page details').forEach(n=>n.open=true);var fixtureScroller=document.querySelector('.studio-page-body');fixtureScroller.scrollTop=fixtureScroller.scrollHeight;document.querySelector('#profile-driver-check').focus({preventScroll:true});void 0;`);await settle();
 const hardwareBefore=await js('viewSnapshot()');await js(`document.querySelector('#profile-driver-check').click();void 0;`);await settle();sameView(hardwareBefore,await js('viewSnapshot()'),'driver file check');
 await js(`setScene('parameters-model','light');delete state.profileEditor.core.model.gravity_correction_path;render();var fixtureScroller=document.querySelector('.studio-page-body');fixtureScroller.scrollTop=fixtureScroller.scrollHeight;document.querySelector('#profile-add-correction').focus({preventScroll:true});void 0;`);await settle();
 const draftBefore=await js('viewSnapshot()');await js(`document.querySelector('#profile-add-correction').click();void 0;`);await settle();const draftAfter=await js('viewSnapshot()');
 assert(Math.abs(draftBefore.top-draftAfter.top)<=1&&await js("state.profileDirty&&Object.hasOwn(state.profileEditor.core.model,'gravity_correction_path')"),'adding a parameter draft keeps its scrolled group');
 await js(`var precisionField=[...document.querySelectorAll('[data-profile-field]')].find(n=>JSON.parse(n.dataset.profileField).join('.')==='core.model.gravity');precisionField.focus({preventScroll:true});precisionField.setSelectionRange(2,6);render();void 0;`);assert(await js(`document.activeElement.dataset.profileField==='["core","model","gravity"]'&&document.activeElement.selectionStart===2&&document.activeElement.selectionEnd===6`),'same-group refresh preserves text-field focus and selection');
 await js(`document.querySelector('[data-profile-group="motion"]').click();void 0;`);await settle();assert((await js('viewSnapshot()')).top===0,'intentional parameter group navigation starts at the top');
 await js(`state.profileDirty=false;document.querySelector('[data-page="lifecycle"]').click();void 0;`);await settle();assert(await js("state.page==='lifecycle'&&document.querySelector('.studio-page-body').scrollTop===0"),'intentional page navigation starts at the top');
 await js(`(async()=>{setScene('workbench','light');state.workbenchTab='replay';await refreshTeachTrajectories();render();document.querySelector('#teach-refresh').scrollIntoView({block:'center'});document.querySelector('#teach-refresh').focus({preventScroll:true});})()`);await settle();
 const replayBefore=await js('viewSnapshot()');assert(replayBefore.top+replayBefore.outer>100,'saved-trajectory list creates a genuinely long replay page');await js(`document.querySelector('#teach-refresh').click();void 0;`);await settle();sameView(replayBefore,await js('viewSnapshot()'),'replay list refresh');
 await js(`setScene('geometry','light');document.querySelectorAll('#page details').forEach(n=>n.open=true);var fixtureScroller=document.querySelector('.studio-page-body');fixtureScroller.scrollTop=fixtureScroller.scrollHeight;document.querySelector('#geometry-rms')?.focus({preventScroll:true});void 0;`);await settle();const geometryBefore=await js('viewSnapshot()');await js('refreshGeometryPanel();void 0;');await settle();sameView(geometryBefore,await js('viewSnapshot()'),'geometry local-card refresh');
 // Live failure -> saved candidate -> existing apply / restore handlers.
 const gain=await js('openRecordedGain()');await settle();assert(gain.visible&&gain.available.join(',')==='rigid_tracking,servo_tracking','actual failed task retains precisely its completely measured parameter sets',gain);
 assert(await js("document.querySelector('#model-calibration-task-conclusion').textContent.includes('短测输入被拒绝')&&!document.querySelector('#model-calibration-task-conclusion').textContent.includes('short response input rejected')&&document.querySelector('#model-calibration-task-conclusion').title==='short response input rejected'"),'actual terminal reason is readable with its original diagnostic retained only as a tooltip');
 assert(await js("!document.querySelector('#model-calibration-task-candidate').hidden"),'terminal overlay exposes the existing candidate destination');
 const overlayFile=path.join(out,'terminal-candidate-link-1280.png');fs.writeFileSync(overlayFile,(await win.webContents.capturePage()).toPNG());report.captures.push(overlayFile);
 win.setContentSize(375,1000);await settle();assert(await js(`['model-calibration-task-gains','model-calibration-task-candidate','model-calibration-task-close'].every(id=>{const n=document.getElementById(id),r=n.getBoundingClientRect();return !n.hidden&&r.top>=0&&r.bottom<=innerHeight&&r.left>=0&&r.right<=innerWidth;})`),'narrow live-gain window, candidate and close actions remain reachable');const narrowOverlay=path.join(out,'terminal-candidate-link-375.png');fs.writeFileSync(narrowOverlay,(await win.webContents.capturePage()).toPNG());report.captures.push(narrowOverlay);win.setContentSize(1280,1000);await settle();
 await js(`document.querySelector('#model-calibration-task-candidate').click();void 0;`);await settle();
 assert(await js("document.querySelector('#model-calibration-overlay').hidden&&!document.querySelector('.shell').inert"),'candidate link closes only the terminal review');
 assert(await js("[...document.querySelectorAll('[data-gain-candidate-purpose]')].map(n=>n.dataset.gainCandidatePurpose).join(',')==='rigid_tracking,servo_tracking'"),'only measured rigid and servo uses are selectable');
 assert(await js(`document.querySelector('[data-gain-result="rigid_tracking"]').innerText.includes('保持扰动测量未完成')&&!document.querySelector('[data-gain-result="servo_tracking"]').innerText.includes('保持扰动测量未完成')`),'missing-evidence warnings follow each use and do not impose holding validation on servo');
 assert(await js("document.querySelector('[data-gain-candidate-purpose]').getBoundingClientRect().top>=document.querySelector('.workspace-operation-panel').getBoundingClientRect().bottom"),'candidate destination is visible below the sticky robot controls');
 const candidateFile=path.join(out,'measured-candidates-1280.png');fs.writeFileSync(candidateFile,(await win.webContents.capturePage()).toPNG());report.captures.push(candidateFile);
 assert(await js("!document.querySelector('.gain-result-actions').hidden&&!document.querySelector('#calibration-gain-temp-apply').disabled&&!document.querySelector('#confirm-dialog').open"),'warning result temporary apply is reachable without another approval');
 await js(`state.telemetry.robot_state='FAULT';refreshGainApplyControls();void 0;`);assert(await js("!document.querySelector('.gain-result-actions').hidden&&document.querySelectorAll('[data-gain-candidate-purpose]').length===2&&document.querySelector('#calibration-gain-temp-apply').disabled"),'a later fault retains measured candidate selection and the visible application entry while preserving INACTIVE gating');await js(`state.telemetry.robot_state='INACTIVE';refreshGainApplyControls();void 0;`);
 win.setContentSize(375,1000);await js(`document.querySelectorAll('#page details').forEach(n=>n.open=true);for(const n of document.querySelectorAll('#page .table-scroll'))n.scrollLeft=n.scrollWidth;document.querySelector('[data-gain-candidate-purpose]').focus({preventScroll:true});void 0;`);await settle();
 const nestedBefore=await js('viewSnapshot()');assert(nestedBefore.horizontal.some(n=>n>0),'actual narrow diagnostics exercise nested horizontal scrolling');await js('render();void 0;');await settle();const nestedAfter=await js('viewSnapshot()');sameView(nestedBefore,nestedAfter,'gain local-card refresh');assert(JSON.stringify(nestedBefore.horizontal)===JSON.stringify(nestedAfter.horizontal),'gain local-card refresh restores nested horizontal table positions');
 win.setContentSize(1280,1000);await js(`document.querySelectorAll('#page details').forEach(n=>n.open=false);void 0;`);await settle();
 await js(`document.querySelector('#calibration-gain-temp-apply').scrollIntoView({block:'center'});void 0;`);await settle();
 const gainFile=path.join(out,'measured-warning-apply-1280.png');fs.writeFileSync(gainFile,(await win.webContents.capturePage()).toPNG());report.captures.push(gainFile);
 await js(`document.querySelector('#calibration-gain-temp-apply').click();void 0;`);await settle();
 assert(await js("state.calibrationRuntimeCandidates.gain_applied&&!document.querySelector('#calibration-gain-temp-restore').disabled"),'existing temporary application handler keeps restore reachable');
 const appliedCalls=await js('window.serialArm.fixtureCalls()');const applied=appliedCalls.find(item=>item.name==='calibration_candidate_apply');assert(applied?.params?.gain_purposes.join(',')==='rigid_tracking,servo_tracking','temporary application submits exactly the selected measured uses');
 await js(`document.querySelector('#calibration-gain-temp-restore').click();void 0;`);await settle();assert(await js('!state.calibrationRuntimeCandidates.gain_applied'),'existing restore handler remains usable');
 await js(`state.calibrationToolSelected.gain=null;state.gainCandidateLoadPending='';state.gainCandidateLoadAttempted='';state.telemetry.model_calibration.phase='cancelled';syncCompletedGainCandidate();void 0;`);await settle();assert(await js("state.calibrationToolSelected.gain?.gain_candidate?.task_id===window.studioUiData.overlayRecord.candidate.task_id"),'cancelled terminal state automatically reads its saved candidate');
 assert(await js("!!document.querySelector('#calibration-gain-load-live')"),'cancelled task retains its manual candidate-load entry');
 for(let i=0;i<5;i++)await js('render();updateModelCalibrationTaskOverlay();void 0;');assert(await js("document.querySelector('#model-calibration-overlay').hidden"),'repeated refresh does not reopen a dismissed terminal review');
 report.apiCalls=await js('window.serialArm.fixtureCalls()');report.consoleErrors=errors;
 assert(errors.length===0,'no renderer errors',errors);
 assert(report.apiCalls.filter(n=>n.name==='workbench_request').every(n=>n.params.method==='get_admittance'),'no control command or hardware connection is requested');
 assert(sourceFiles.every(item=>fs.readFileSync(item.file).equals(item.bytes)),'actual task evidence remains byte-identical');
 fs.writeFileSync(path.join(out,'report.json'),JSON.stringify(report,null,2));console.log('LOCAL_REFRESH_PASS '+JSON.stringify({checks:report.checks.length,captures:report.captures.length,report:path.join(out,'report.json')}));win.destroy();app.quit();
}).catch(error=>{console.error(error.stack);app.exit(1);});
