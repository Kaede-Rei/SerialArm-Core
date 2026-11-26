/* R4 task stages, record details and scoped candidate-to-draft adoption.
 * DISPLAY=:1 node_modules/.bin/electron --no-sandbox scripts/verify-task-details.cjs
 * Uses the real offline URDF data and a deny-by-default API; no motors or files
 * are changed. Three.js rendering is covered by verify-model-lifecycle.cjs. */
const {app,BrowserWindow}=require('electron');
const fs=require('node:fs'),path=require('node:path');
const {buildFixture}=require('./studio-ui-fixture.cjs');
const output='/tmp/serialarm-task-details-verification';
const fixture=buildFixture(path.join(output,'fixture'),{model:true});
app.setPath('userData',path.join(output,'electron-userdata'));
function setScene(scene,theme,language='zh-CN'){
 const data=window.studioUiData;
 if(window.studioFixtureRescue)graphicsState.effectiveMode='rescue';
 Object.assign(state,{page:['lifecycle-path','lifecycle-task','lifecycle-observation'].includes(scene)?'lifecycle':scene.startsWith('parameters-')?'profile_setup':scene.startsWith('calibration-')?'calibration':scene.startsWith('workbench-')?'workbench':scene,theme,language,config:{profile:data.editor.profile,profile_file:'/offline/robot_profiles.yaml',serial_port:'',baudrate:'',bus:'',resource_paths:''},info:structuredClone(data.info),profiles:[{name:data.editor.profile}],library:structuredClone(data.library),profileEditor:structuredClone(data.editor),capabilityProfile:structuredClone(data.editor),profileGroup:scene.split('-')[1]||'model',profileDirty:false,profileParkingPath:null,profileParkingKey:'',profileRawInputs:{},profileFieldErrors:{},readiness:{state:'configuring',checks:{},evidence:{},control_available:true},commissioningDrafts:{},commissioningResults:{},commissioningContext:null,onboarding:null,modelData:data.model?structuredClone(data.model):null,modelPositions:data.model?.native?.state?.positions?.slice()||[],modelSelection:data.model?.model?.links?.[0]?.name||'',telemetry:null,session:{state:'idle'},admittance:null,calibrationEntry:scene==='calibration-teaching'?'teaching':'gain',calibrationToolSelected:{teaching:null,gain:null,gravity:null,dynamics:null},calibrationRuntimeCandidates:{},modelCalibrationRecords:[],gainMode:'all',gainTuningMode:'quick',gainFinePurpose:'rigid',gainRequirements:{},teachTrajectories:[],geometryTrajectories:[],driverInspection:{plugin:'serial_arm_hardware_damiao',available:true,drivers:[{plugin:'serial_arm_hardware_damiao',available:true,label:'达妙 / Damiao'}]}});
 if(scene==='calibration-result')state.calibrationToolSelected.gain={directory:'/offline/calibration-result',gain_candidate:structuredClone(data.candidate)};
 state.modelCalibrationOffline=null;state.idimReport=null;
 if(scene.startsWith('calibration-'))state.telemetry={joint_names:data.editor.joint_names,model_calibration:{workflow_kind:'gain',phase:scene==='calibration-result'?'complete':'idle',task_id:scene==='calibration-result'?data.candidate.task_id:'',gain_budget_source_duration_s:25,gain_target_budget_s:300,gain_time_budget_s:375,gain_elapsed_s:scene==='calibration-result'?239:0,gain_remaining_s:375}};
 if(['calibration-gravity','calibration-dynamics'].includes(scene)){
  state.calibrationEntry=scene.split('-')[1];state.session={state:'running',mode:'workbench'};
  state.telemetry.robot_state='INACTIVE';state.telemetry.valid=false;
  state.telemetry.model_calibration={phase:'idle',workflow_kind:state.calibrationEntry,gravity_result:{first_moments:[{link:'fixture-link',first_moment:[0,0,0]}]}};
  state.idimReport={physical_pass:true};
 }
 if(scene==='start'||scene==='library')state.config.profile='';
 if(['lifecycle','lifecycle-path','lifecycle-task','lifecycle-observation'].includes(scene)){
  state.page='lifecycle';state.config={...state.config,...data.commissioning.config};state.readiness=structuredClone(data.commissioning.MIT);
  state.profileEditor=structuredClone(data.commissioning.editor);state.capabilityProfile=structuredClone(data.commissioning.editor);
  if(scene==='lifecycle-observation'){state.readiness=structuredClone(data.commissioning.CSP);state.profileEditor=structuredClone(data.commissioning.editorCSP);state.capabilityProfile=structuredClone(data.commissioning.editorCSP);}
 }
 if(['calibration-observation','calibration-response'].includes(scene)){state.calibrationEntry=scene.split('-')[1];state.profileEditor.core.control.runtime.control_path='CSP';state.capabilityProfile.core.control.runtime.control_path='CSP';}
 if(scene==='workbench'||scene.startsWith('workbench-')){state.workbenchTab=scene.split('-')[1]||'control';state.controlSettingsOpen=false;}
 if(data.receipt){state.readiness.gain_initialization=structuredClone(data.receipt);state.config.profile=data.receiptLabel;}
 renderedPage='';renderedPageViewKey='';shell();
 if(scene==='lifecycle-observation')onboardingOpenTask('observation_validation','CSP');
 if(scene==='lifecycle-task')onboardingOpenTask('model_connection','MIT');
 if(scene==='lifecycle-path')onboardingOpenTask('observation_validation','CSP');
 if(scene.startsWith('parameters-shutdown-')){const path=scene.endsWith('csp')?'CSP':'MIT';state.profileParkingPath=path;if(scene.endsWith('disabled'))state.profileEditor.core.shutdown.paths[path].enabled=false;render();}
 if(scene==='parameters-csp'){state.profileEditor.core.control.runtime.control_path='CSP';state.profileGroup='motion';render();}
 if(scene==='parameters-motion')document.querySelector('[data-gain-purpose="RIGID"]').open=true;
}
app.whenReady().then(async()=>{
 fs.mkdirSync(output,{recursive:true});
 const win=new BrowserWindow({width:1440,height:900,show:false,webPreferences:{preload:fixture.preload,sandbox:false,backgroundThrottling:false}});
 const deadline=setTimeout(()=>{console.error('Task-detail fixture timed out');app.exit(1);},45000);
 win.webContents.on('render-process-gone',(_event,details)=>{console.error(details);app.exit(1);});
 await win.loadFile(fixture.index);
 await win.webContents.executeJavaScript(`(async()=>{await document.fonts.ready;await new Promise(resolve=>setTimeout(resolve,200));term?.dispose();term=null;fit=null;window.Terminal=undefined;window.studioFixtureRescue=true;window.studioFixtureSetScene=${setScene};})()`);
 const checks=await win.webContents.executeJavaScript(`(async()=>{
 const checks=[],assert=(condition,message)=>{if(!condition)throw Error(message);checks.push(message);};
 window.studioFixtureSetScene('model','light');
 state.modelSelection=state.modelData.model.links.find(link=>link.inertial)?.name;render();
 assert(document.querySelectorAll('.inspector-inertia-matrix td').length>=9,'Model inertia uses a structured 3×3 matrix');
 assert(document.querySelectorAll('.inspector-vector > div').length>=6,'XYZ and RPY have individual axis rows');
 window.studioFixtureSetScene('geometry','light');
 const geometry=document.querySelector('.geometry-task-flow');
 assert(geometry.querySelectorAll('.task-stage-button').length===5,'Fixed point has five task stages');
 const rms=document.getElementById('geometry-rms');
 geometry.querySelectorAll('.task-stage-button')[3].click();
 assert(rms.getClientRects().length>0,'Independent-validation controls appear in validation stage');
 await new Promise(resolve=>setTimeout(resolve,650));
 const stageNav=geometry.querySelector('.task-stage-navigation'),stageIndicator=stageNav.querySelector(':scope > .studio-segment-indicator'),stageActive=stageNav.querySelector('.task-stage-button.selected');
 assert(stageIndicator&&Math.abs(stageIndicator.getBoundingClientRect().top-stageActive.getBoundingClientRect().top)<2,'Spring indicator reaches the selected task stage, not a false aria-current stage');
 rms.value='4.2';geometry.querySelectorAll('.task-stage-button')[0].click();geometry.querySelectorAll('.task-stage-button')[3].click();
 assert(document.getElementById('geometry-rms')===rms&&rms.value==='4.2','Stage switching preserves the actual input node and draft');
 const geometryIds=[...geometry.querySelectorAll('[id]')].map(node=>node.id);assert(new Set(geometryIds).size===geometryIds.length,'Geometry stages never duplicate command or field IDs');
 state.geometryTab='repeat';render();
 assert(document.querySelectorAll('.geometry-task-flow .task-stage-button').length===3,'Repeatability uses teaching, saved path and validation stages');
 window.studioFixtureSetScene('calibration-dynamics','light');
 assert(document.querySelectorAll('#idim-workflow .task-stage-button').length===4,'Full IDIM uses data, settings, results and export stages');
 const idimInput=document.getElementById('idim-validation');document.querySelectorAll('#idim-workflow .task-stage-button')[1].click();
 idimInput.value='/offline/independent';document.querySelectorAll('#idim-workflow .task-stage-button')[2].click();document.querySelectorAll('#idim-workflow .task-stage-button')[1].click();
 assert(document.getElementById('idim-validation')===idimInput&&idimInput.value==='/offline/independent','IDIM phase navigation preserves settings');
 document.querySelector('.task-record-toolbar > button').click();
 assert(document.querySelector('.calibration-tool-history').getClientRects().length>0&&!document.getElementById('idim-workflow').getClientRects().length,'Record library is a separate view from current task');
 document.querySelectorAll('.task-record-toolbar > button')[1].click();
 assert(document.getElementById('idim-workflow').getClientRects().length>0,'Returning from records restores current task');
 window.studioFixtureSetScene('calibration-gain','light');
 assert(document.querySelectorAll('.gain-calibration-page .task-stage-button').length===3,'Gain calibration separates setup, acquisition and review');
 document.querySelectorAll('.gain-calibration-page .task-stage-button')[1].click();
 assert(document.querySelector('.task-progress-empty').getClientRects().length>0,'Idle acquisition stage explains how to begin');
 const idleActions=document.querySelector('.gain-task-actions');assert(idleActions.children.length>0||!idleActions.getClientRects().length,'An empty task action bar does not consume workspace height');
 window.studioFixtureSetScene('workbench-replay','light');
 state.teachTrajectories=[{id:'fixture-record',name:'Fixture trajectory',samples:100,duration_s:8,directory:'/offline/fixture-record',compatible:true}];state.teachSelectedId='fixture-record';render();
 const detail=document.querySelector('.teach-record-detail');
 assert(detail&&detail.hidden&&document.querySelector('.teach-record-preview .teach-timeline')&&document.getElementById('teach-play'),'Selected trajectory shows its real timeline alongside the record list');
 const open=[...document.querySelectorAll('.teach-replay-panel button')].find(button=>button.textContent==='打开所选记录详情');open.click();
 assert(!detail.hidden&&detail.querySelector('.teach-timeline')&&state.teachSelectedId==='fixture-record','Opening details moves the same timeline and retains the trajectory');
 detail.querySelector('.task-record-heading .button').click();
 assert(detail.hidden&&document.querySelector('.teach-record-preview .teach-timeline')&&state.teachSelectedId==='fixture-record','Returning to the library retains selection and the same preview');
 open.click();assert(!detail.hidden&&state.teachSelectedId==='fixture-record','Reopening details preserves the same trajectory');
 const ids=[...document.querySelectorAll('#page [id]')].map(node=>node.id);assert(new Set(ids).size===ids.length,'Trajectory details never duplicate command or field IDs');
 window.studioFixtureSetScene('calibration-result','light');
 const candidate=state.calibrationToolSelected.gain.gain_candidate;
 candidate.urdf_fingerprint=state.modelData.native.config.urdf_fingerprint;
 const purposes=Object.keys(gainCandidateGroups(candidate)).filter(key=>['rigid','compliant','drag'].includes(key)&&gainCandidatePurposeAvailable(candidate,key));
 assert(purposes.length>0,'The real fixture has measured gain uses for draft adoption');
 state.gainPurposeSelections={[gainSelectionKey(candidate)]:[purposes[0]]};
 const savedBefore=JSON.stringify(profileSavedValues()),runtimeBefore=JSON.stringify(state.telemetry),bus=Object.keys(state.profileEditor.hardware.buses)[0];
 state.profileEditor.hardware.buses[bus].device='/dev/unsaved-review';state.profileDirty=true;
 const callsBefore=(await api.fixtureCalls()).length;
 const changed=await studioAdoptGainDraft({review:false});
 assert(changed.length>0&&state.profileReviewSelection.every(path=>path.startsWith('core.control.')),'Gain adoption scopes the parameter review to selected gain fields');
 assert(state.profileEditor.hardware.buses[bus].device==='/dev/unsaved-review','Gain adoption preserves unrelated parameter drafts');
 assert(JSON.stringify(profileSavedValues())===savedBefore&&JSON.stringify(state.telemetry)===runtimeBefore,'Gain adoption changes neither saved defaults nor running values');
 assert((await api.fixtureCalls()).length===callsBefore,'Gain adoption sends no device command or disk-save request');
 candidate.joint_names=[...candidate.joint_names].reverse();let rejected=false;try{studioGainDraftChanges(candidate);}catch{rejected=true;}assert(rejected,'Candidate adoption rejects a mismatched joint order');
 return checks;
 })()`);

 await win.webContents.executeJavaScript(`(async()=>{await new Promise(resolve=>setTimeout(resolve,250));if(document.querySelector('#priority-error-overlay')?.open)throw Error(document.querySelector('#priority-error-raw')?.textContent||'Unexpected error dialog');})()`);
 fs.writeFileSync(path.join(output,'checks.json'),JSON.stringify(checks,null,2));
 console.log('TASK_DETAILS_PASS '+JSON.stringify({checks:checks.length,report:path.join(output,'checks.json')}));
 clearTimeout(deadline);win.destroy();app.quit();
}).catch(error=>{console.error(error.stack);app.exit(1);});
