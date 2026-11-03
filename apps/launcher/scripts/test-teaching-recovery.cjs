const test=require('node:test');
const assert=require('node:assert/strict');
const fs=require('node:fs');
const path=require('node:path');
const vm=require('node:vm');
const source=fs.readFileSync(path.join(__dirname,'../renderer/app.js'),'utf8');
function setup() {
  const state={workbenchEpoch:0,workbenchIdentity:null,session:{state:'idle'},language:'zh-CN',
    calibrationEntry:'teaching',calibrationToolSelected:{},telemetryHistory:[],modelCalibrationOffline:null};
  const errors=[];
  const ui={state,ghostGeneration:0,modelView:null,lastRuntimeFaultKey:'',lastTeachReplayErrorKey:'',reportedCalibrationErrors:new Set(),errors,
    workbenchConnected:()=>['running','stopping'].includes(state.session.state),
    modelCalibrationActive:s=>!['idle','failed','complete','cancelled'].includes(s.phase),
    gainCandidateDeployable:()=>false,calibrationToolHistoryHtml:()=>'',modelCalibrationPhaseText:s=>s,
    esc:s=>String(s),t:s=>s,busyText:(zh,en)=>state.language==='en'?en:zh,
    showPriorityError:(...args)=>errors.push(args),render:()=>{},updateModelCalibrationTaskOverlay:()=>{},
    updateModelCalibrationLive:()=>{},blockingOperation:async(label,run)=>run()};
  vm.createContext(ui);
  vm.runInContext(fs.readFileSync(path.join(__dirname,'../renderer/error-i18n.js'),'utf8'),ui);
  ui.localizedError=s=>ui.serialArmErrorI18n.translate(s,state.language);
  for(const name of ['synchronizeWorkbenchSession','currentModelCalibration','modelCalibrationTaskParams',
      'gainRequirementFieldsHtml','gainCalibrationOptionsHtml','calibrationStandalonePanelHtml','modelCalibrationQuick','finishTeachingSafely','describeRuntimeFault',
      'normalizeDialogError','checkRuntimeAlerts']) {
    const match=source.match(new RegExp('(?:async )?function '+name+'\\([^]*?\\n\\}'));
    assert.ok(match,name);vm.runInContext(match[0],ui);
  }
  return ui;
}
function session(ui,id) {ui.synchronizeWorkbenchSession({state:'running',mode:'workbench',session_id:id,pid:id});}
test('reconnect clears failed live state while keeping saved records readable',()=>{
  const ui=setup();session(ui,'first');
  const record={directory:'/saved',result:{phase:'failed',task_id:'saved',error:'OLD_FAILURE'}};
  ui.state.modelCalibrationOffline=record;
  ui.state.modelCalibrationDirectory='/saved';
  ui.state.telemetry={robot_state:'FAULT',model_calibration:{phase:'failed',error:'OLD_LIVE_FAILURE'}};
  ui.synchronizeWorkbenchSession({state:'exited',mode:'workbench',session_id:'first'});
  assert.equal(ui.state.telemetry,null);
  session(ui,'second');
  ui.state.telemetry={robot_state:'ACTIVE',model_calibration:{phase:'idle',task_id:'new'}};
  assert.equal(ui.state.modelCalibrationOffline,record);
  const html=ui.calibrationStandalonePanelHtml('teaching');
  assert.match(html,/id="model-calibration-teach-start"/);
  assert.doesNotMatch(html,/OLD_FAILURE|OLD_LIVE_FAILURE|class="button primary" disabled/);
  assert.equal(ui.modelCalibrationTaskParams().task_id,'new');
  assert.equal(ui.synchronizeWorkbenchSession({state:'exited',session_id:'first'}),false);
  assert.equal(ui.state.session.session_id,'second');
});
test('failed calibration request preserves its real error without a scope exception',async()=>{
  const ui=setup();session(ui,'first');
  ui.state.telemetry={model_calibration:{phase:'teaching',task_id:'current'}};
  ui.workbenchRequest=async method=>{if(method==='model_calibration_status')return {phase:'failed',task_id:'current'};throw new Error('ORIGINAL_ERROR');};
  const result=await ui.modelCalibrationQuick('model_calibration_cancel');
  assert.equal(result,null);assert.equal(ui.errors[0][0].message,'ORIGINAL_ERROR');
  assert.equal(ui.state.telemetry.model_calibration.phase,'failed');
});
test('late replies cannot overwrite a reconnected live task',async()=>{
  const ui=setup();session(ui,'first');
  ui.state.telemetry={model_calibration:{phase:'teaching',task_id:'old'}};
  let finish;ui.workbenchRequest=()=>new Promise(resolve=>{finish=resolve;});
  const pending=ui.modelCalibrationQuick('model_calibration_cancel');
  ui.synchronizeWorkbenchSession({state:'exited',session_id:'first'});session(ui,'second');
  ui.state.telemetry={model_calibration:{phase:'idle',task_id:'new'}};
  finish({phase:'failed',task_id:'old'});await pending;
  assert.equal(ui.state.telemetry.model_calibration.task_id,'new');assert.equal(ui.errors.length,0);
});
test('timeout dialog reports joint, source and milliseconds without losing the joint name',()=>{
  const ui=setup();
  const value=ui.describeRuntimeFault({fault:{code:'SAFETY_FAILED',safety_code:'STATE_TIMEOUT',
    joint_name:'joint3',joint_index:2,value:.0553,limit:.05,feedback_age_source:'host_transport_receive'}});
  const dialog=ui.normalizeDialogError(value,'fault');
  assert.match(dialog.summary,/joint3/);assert.match(dialog.summary,/55\.30 ms/);
  assert.match(dialog.summary,/50\.00 ms/);assert.match(dialog.summary,/主机接收/);
  assert.match(dialog.raw,/STATE_TIMEOUT/);
});
test('a safety-interrupted task shows the root safety fault without a second cleanup alarm',()=>{
  const ui=setup();ui.checkRuntimeAlerts({robot_state:'FAULT',fault:{code:'SAFETY_FAILED',safety_code:'STATE_TIMEOUT',value:.07,limit:.05},
    model_calibration:{phase:'failed',error:'same safety fault',fault_interrupted:true}});
  assert.equal(ui.errors.length,1);assert.equal(ui.errors[0][1],'fault');
});
test('gain workflow uses the live task while an unrelated gravity record stays selected',()=>{
  const ui=setup();session(ui,'gain-session');
  ui.state.calibrationEntry='gain';
  ui.state.modelCalibrationOffline={directory:'/saved-gravity',result:{phase:'complete',workflow_kind:'gravity'}};
  ui.state.calibrationToolSelected.gain={directory:'/saved-teaching'};
  ui.state.telemetry={robot_state:'ACTIVE',model_calibration:{phase:'idle'}};
  ui.calibrationTaskLoaded=()=>true;
  const html=ui.calibrationStandalonePanelHtml('gain');
  assert.match(html,/id="calibration-tool-import"/);
  assert.doesNotMatch(html,/id="calibration-tool-import"[^>]*disabled/);
  assert.equal(ui.state.modelCalibrationOffline.directory,'/saved-gravity');
});
test('park overlay provides stop actions that bypass the ordinary busy gate',async()=>{
  const ui=setup(),elements={};
  for(const id of ['operation-overlay','operation-message','operation-safety-actions','operation-hold','operation-disable','operation-force-disconnect'])
    elements[id]={hidden:true,disabled:false,setAttribute(){}};
  ui.$=id=>elements[id];ui.priorityErrorOpen=false;ui.performance=performance;ui.setTimeout=setTimeout;
  ui.workbenchRecoveryRequired=()=>false;ui.operation=fn=>fn();ui.toast=()=>{};
  for(const name of ['setOperationOverlay','blockingOperation','refreshParkingActions','interruptibleParking','interruptParking']) {
    const line=source.split('\n').find(s=>s.startsWith('function '+name+'(')||s.startsWith('async function '+name+'('));
    vm.runInContext(line.endsWith(' }')?line:source.match(new RegExp('(?:async )?function '+name+'\\([^]*?\\n\\}'))[0],ui);
  }
  session(ui,'parking-session');
  let release;const commands=[];
  ui.workbenchRequest=async method=>{commands.push(method);if(method==='park')return new Promise(resolve=>release=resolve);};
  const pending=ui.blockingOperation('Parking',()=>ui.interruptibleParking(()=>ui.workbenchRequest('park')),0);
  assert.equal(elements['operation-overlay'].hidden,false);
  assert.equal(elements['operation-safety-actions'].hidden,false);
  assert.equal(elements['operation-disable'].disabled,false);
  await ui.interruptParking('deactivate');
  assert.deepEqual(commands,['park','deactivate']);
  release();await pending;
  assert.equal(elements['operation-safety-actions'].hidden,true);
});
