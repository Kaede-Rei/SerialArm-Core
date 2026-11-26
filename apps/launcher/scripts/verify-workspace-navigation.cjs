/* Production workspace navigation with real read-only Profile/URDF fixtures.
 * node_modules/.bin/electron --no-sandbox scripts/verify-workspace-navigation.cjs
 * No hardware connection or robot command is available to this fixture.
 */
const {app,BrowserWindow}=require('electron');
const fs=require('node:fs'),path=require('node:path');
const {buildFixture}=require('./studio-ui-fixture.cjs');
const arg=process.argv.indexOf('--out'),output=path.resolve(arg<0?'/tmp/serialarm-workspace-navigation':process.argv[arg+1]);
if(!output.startsWith('/tmp/'))throw Error('Fixture output must be inside /tmp');
const fixture=buildFixture(path.join(output,'fixture'),{commissioning:true,model:true});
app.setPath('userData',path.join(output,'electron-userdata'));
function seedWorkspace(page='lifecycle',tab='control') {
 const data=window.studioUiData;
 Object.assign(state,{page,workbenchTab:tab,language:'zh-CN',theme:'light',config:{...data.commissioning.config},info:structuredClone(data.info),profileEditor:structuredClone(data.commissioning.editor),capabilityProfile:structuredClone(data.commissioning.editor),profileGroup:'hardware',profileDirty:false,profileCenterView:'edit',profileReviewSelection:null,profileRawInputs:{},profileFieldErrors:{},modelData:structuredClone(data.model),modelSelection:data.model.model.links[0].name,modelPositions:data.model.native.state.positions.slice(),readiness:structuredClone(data.commissioning.MIT),session:{state:'idle'},telemetry:null,telemetryHistory:[],admittance:null,calibrationEntry:'gain',calibrationToolSelected:{teaching:null,gain:null,gravity:null,dynamics:null},gainPurposeSelections:{},onboarding:null,commissioningDrafts:{},commissioningResults:{},commissioningContext:null,studioHardwareObject:'driver',studioEvents:[],studioEventId:'',studioEventFilter:'all',studioObservedJoint:'',studioObservedSignal:'pos',studioObservedTime:null,teachTrajectories:[],teachSelectedId:'',driverInspection:{plugin:'serial_arm_hardware_damiao',available:true,drivers:[{plugin:'serial_arm_hardware_damiao',available:true,label:'Damiao'}]}});
 graphicsState.effectiveMode='rescue';renderedPage='';renderedPageViewKey='';shell();
}
async function verifyWorkspaces(){
 const checks=[],assert=(ok,name,evidence)=>{if(!ok)throw Error(name+' '+JSON.stringify(evidence));checks.push({name,...(evidence?{evidence}:{})});};
 const tick=()=>new Promise(resolve=>requestAnimationFrame(()=>requestAnimationFrame(resolve)));
 window.seedWorkspace();await tick();
 const routes=studioSections().map(item=>item[4]);
 assert(JSON.stringify(routes)===JSON.stringify(['lifecycle','workbench:control','profile_setup','model','workbench:tuning','hardware','workbench:monitor'])&&document.querySelectorAll('[data-studio-section]').length===7,'Seven approved workspaces are rendered as live navigation controls');
 for(const route of routes){
  await studioNavigate(route);await tick();
  const active=[...document.querySelectorAll('[data-studio-section].active')];
  assert(studioRouteKey()===route&&active.length===1&&active[0].dataset.studioSection===studioSection(),'Workspace route and selected navigation agree: '+route,{actual:studioRouteKey()});
 }
 // Model rescue must not recurse through dependent-action/layout refresh.
 for(const page of ['profile_setup','hardware','lifecycle']){
  window.seedWorkspace(page);if(page==='lifecycle')onboardingOpenTask('model_connection');await tick();
  const host=document.querySelector('#workbench-viewport');assert(!!host,'Context workspace owns a real preview host: '+page);
  await mountWorkbenchView();const panel=host.querySelector('[data-graphics-panel]');
  assert(!!panel,'Rescue mode reports its state in the real model host: '+page);
  for(let i=0;i<4;i++){syncWorkspacePreviewVisibility();refreshGraphicsDependentActions();await mountWorkbenchView();}
  assert(host.querySelector('[data-graphics-panel]')===panel,'Repeated rescue refresh keeps one panel and terminates: '+page);
  const realUpdate=updateWorkbenchModel;let updates=0;
  try{updateWorkbenchModel=()=>{updates++;};updateWorkbenchLive();}finally{updateWorkbenchModel=realUpdate;}
  assert(updates===1,'Live updates reach context model: '+page,{updates});
 }
 window.seedWorkspace('profile_setup');await tick();
 const previewToggle=document.querySelector('[data-profile-toggle-model]'),profilePreview=document.querySelector('[data-workspace-preview]');
 assert(previewToggle&&profilePreview&&profilePreview.open,'Parameter heading exposes the reference model control');
 previewToggle.click();await tick();assert(!profilePreview.open&&previewToggle.textContent.includes('显示模型'),'Parameter heading collapses the model and updates its label');
 profilePreview.querySelector(':scope > summary').click();await tick();assert(profilePreview.open&&previewToggle.textContent.includes('收起模型'),'Opening from the model header updates the parameter heading control');
 window.seedWorkspace('workbench','monitor');await tick();
 const name=state.profileEditor.joint_names[0];
 state.session={state:'running',mode:'workbench'};
 state.telemetry={valid:true,robot_state:'INACTIVE',joint_names:[name],joint:{pos:[.1],vel:[.2],tor:[.3],ref_pos:[.4],residual_raw:[.5]},monotonic_ns:3e9,feedback_age_ms:0,observer_valid:true};
 state.telemetryReceivedAt=Date.now();state.telemetryHistory=[1,2,3].map(time=>({...structuredClone(state.telemetry),monotonic_ns:time*1e9}));
 state.studioObservedJoint=name;render();await tick();
 const draw=studioDrawObservationChart;let redraws=0;
 try{studioDrawObservationChart=()=>{redraws++;};updateWorkbenchLive();}finally{studioDrawObservationChart=draw;}
 assert(redraws===1,'Monitor telemetry triggers a chart repaint',{redraws});
 const signal=document.querySelector('#studio-observed-signal'),canvas=document.querySelector('#workbench-chart'),value=()=>document.querySelector('#studio-observation-value').textContent;
 signal.value='tor';signal.dispatchEvent(new Event('change',{bubbles:true}));
 assert(value().includes('0.30000'),'Torque chart reads joint.tor',{value:value()});
 signal.value='pos';signal.dispatchEvent(new Event('change',{bubbles:true}));
 assert(value().includes('0.10000')&&value().includes('0.40000'),'Position chart compares joint.pos with joint.ref_pos',{value:value()});
 canvas.dispatchEvent(new KeyboardEvent('keydown',{key:'ArrowLeft',bubbles:true,cancelable:true}));
 assert(state.studioObservedTime===2&&value().includes('1.000 s'),'Keyboard moves the shared time cursor by one recorded sample',{time:state.studioObservedTime,value:value()});
 signal.value='tor';signal.dispatchEvent(new Event('change',{bubbles:true}));
 assert(state.studioObservedTime===2&&value().includes('1.000 s')&&value().includes('0.30000'),'Changing signal preserves the selected timestamp');
 canvas.dispatchEvent(new KeyboardEvent('keydown',{key:'ArrowRight',bubbles:true,cancelable:true}));
 assert(state.studioObservedTime===3,'Right arrow advances the time cursor');
 canvas.dispatchEvent(new KeyboardEvent('keydown',{key:'End',bubbles:true,cancelable:true}));
 assert(state.studioObservedTime===null&&value().includes('2.000 s'),'End restores the newest-sample cursor');
 const box=canvas.getBoundingClientRect(),plot=canvas.studioHistory;
 canvas.dispatchEvent(new PointerEvent('pointermove',{clientX:box.left+plot.left,clientY:box.top+80,bubbles:true}));
 assert(state.studioObservedTime===1&&value().includes('0.000 s'),'Pointer movement reads the same time cursor as keyboard control');
 const config={...state.config};state.config={...config,profile_file:'/fixture/shared_profiles.yaml',profile:'armA'};const keyA=studioTaskViewKey('gravity');state.config.profile='armB';const keyB=studioTaskViewKey('gravity');state.config=config;
 assert(keyA!==keyB,'Task-view preferences distinguish profiles in the same YAML file',{keyA,keyB});
 // Follow the real hardware object and shared-draft review controls.
 window.seedWorkspace('hardware');await tick();
 const editor=state.profileEditor,bus=Object.keys(editor.hardware.buses)[0],devicePath=['hardware','buses',bus,'device'],deviceId=devicePath.join('.');
 const savedBefore=JSON.stringify(profileSavedValues()),otherPath=['core','model','gravity_scale',editor.joint_names[0]];
 profileAssignPath(editor,otherPath,1.234567);state.profileDirty=true;
 const busButton=[...document.querySelectorAll('[data-studio-hardware-object]')].find(node=>node.dataset.studioHardwareObject==='bus:'+bus);busButton.click();
 const deviceField=[...document.querySelectorAll('[data-profile-field]')].find(node=>node.dataset.profileField===JSON.stringify(devicePath));
 assert(!!deviceField&&state.studioHardwareObject==='bus:'+bus,'Selecting a hardware object opens its actual shared parameter field');
 deviceField.value='/dev/unsaved-workspace';deviceField.dispatchEvent(new Event('input',{bubbles:true}));
 assert(state.profileEditor===editor&&profileGet(editor,devicePath)==='/dev/unsaved-workspace'&&profileChanges().some(item=>item.id===deviceId),'Hardware field input updates the authoritative unsaved draft');
 await document.querySelector('[data-studio-review-scope]').onclick();await tick();
 assert(state.page==='profile_setup'&&state.profileCenterView==='review'&&JSON.stringify(state.profileReviewSelection)===JSON.stringify([deviceId]),'Hardware review opens parameter center with only the selected object change checked',{selection:state.profileReviewSelection});
 assert(profileGet(state.profileEditor,otherPath)===1.234567&&JSON.stringify(profileSavedValues())===savedBefore,'Object-scoped review preserves unrelated draft values and saved defaults');
 const unrelatedReview=[...document.querySelectorAll('[data-profile-review-field]')].find(node=>node.dataset.profileReviewField===otherPath.join('.'));
 assert(!!unrelatedReview&&!unrelatedReview.checked,'Unrelated changes remain visible and unchecked in the actual review controls');
 await document.querySelector('#profile-return-ready').onclick();await tick();
 assert(state.page==='hardware'&&state.studioHardwareObject==='bus:'+bus,'Returning from object review restores the originating hardware object');
 document.querySelector('[data-studio-hardware-object="driver"]').click();
 const frequencyId='core.control.runtime.ctrl_frequency_hz',frequencyField=[...document.querySelectorAll('[data-profile-field]')].find(node=>JSON.parse(node.dataset.profileField).join('.')===frequencyId);
 frequencyField.value=String(Number(frequencyField.value)+1);frequencyField.dispatchEvent(new Event('input',{bubbles:true}));
 await tick();
 const hardwareScroller=document.querySelector('.workbench-task');
 hardwareScroller.scrollTop=Math.min(120,hardwareScroller.scrollHeight-hardwareScroller.clientHeight);
 const hardwareScroll=hardwareScroller.scrollTop;
 assert(hardwareScroll>0,'Hardware review originates from a genuinely scrolled internal task pane',{scroll:hardwareScroll,height:hardwareScroller.clientHeight,contentHeight:hardwareScroller.scrollHeight});
 await document.querySelector('[data-studio-review-scope]').onclick();await tick();
 assert(JSON.stringify(state.profileReviewSelection)===JSON.stringify([frequencyId]),'Driver-object review excludes existing bus and model drafts',{selection:state.profileReviewSelection});
 await document.querySelector('#profile-return-ready').onclick();await tick();
 assert(state.page==='hardware'&&state.studioHardwareObject==='driver','Returning from driver review restores the driver task');
 assert(Math.abs(document.querySelector('.workbench-task').scrollTop-hardwareScroll)<2,'Returning from parameter review restores the hardware task internal scroll position',{before:hardwareScroll,after:document.querySelector('.workbench-task').scrollTop});
 window.seedWorkspace('workbench','tuning');await tick();
 await studioReviewScope(['core.control.controller']);await tick();
 await document.querySelector('#profile-return-ready').onclick();await tick();
 assert(studioRouteKey()==='workbench:tuning','Shared response review returns to the tuning subpage');
 // Event explanation consumes the same typed/string fault descriptions as alerts.
 window.seedWorkspace('workbench','monitor');await tick();
 state.session={state:'running',mode:'workbench',session_id:'fixture-events'};state.telemetry={robot_state:'INACTIVE'};
 studioRecordEvent({event:'telemetry',session_id:'fixture-events',data:{robot_state:'FAULT',joint_names:[name],fault:{code:'SAFETY_FAULT',safety_code:'JOINT_POS_LIMIT',joint_name:name,value:3.2,limit:2.8}}});
 const fault=state.studioEvents[0];assert(fault?.kind==='fault'&&typeof fault.detail.description==='string','Fault events retain the real runtime fault description');
 await studioNavigate('events');await tick();
 const facts=document.querySelector('.studio-event-facts'),explanation=localizedError(fault.detail.description);
 assert(facts?.textContent.includes(explanation)&&facts.textContent.includes(name),'Event detail displays the concrete limit cause outside raw diagnostics',{text:facts?.textContent});
 const observationLink=document.querySelector('.studio-record-detail [data-studio-route="workbench:monitor"]');
 assert(!!observationLink&&!!document.querySelector('.studio-record-detail [data-studio-route="diagnostics"]'),'Event detail offers real observation and diagnostic navigation controls');
 await observationLink.onclick();await tick();assert(studioRouteKey()==='workbench:monitor','Event observation control navigates to the real monitor');
 state.workbenchIdentity='fixture-events';state.lastSessionExit={error:'previous accepted exit',exit_code:0};
 const eventsBefore=state.studioEvents.length,lastExitBefore=JSON.stringify(state.lastSessionExit);
 onEvent({event:'session',state:'exited',mode:'workbench',session_id:'expired-session',reason:'stale exit',exit_code:1});
 assert(state.studioEvents.length===eventsBefore&&state.session.session_id==='fixture-events'&&state.session.state==='running'&&JSON.stringify(state.lastSessionExit)===lastExitBefore,'Expired session exits cannot add an event or overwrite the active session exit context');
 onEvent({event:'session',state:'exited',mode:'workbench',session_id:'fixture-events',reason:'accepted exit',exit_code:0});
 assert(state.studioEvents.length===eventsBefore+1&&state.studioEvents[0].detail.session_id==='fixture-events'&&state.session.state==='exited'&&state.lastSessionExit.error==='accepted exit','Accepted session exits update both the event list and exit context');
 // Adopting a measured gain candidate changes the shared draft, never the session.
 window.seedWorkspace();await tick();
 state.profileEditor=structuredClone(window.studioUiData.editor);state.telemetry={robot_state:'INACTIVE'};
 const candidate=structuredClone(window.studioUiData.candidate);candidate.urdf_fingerprint=state.modelData.native.config.urdf_fingerprint;
 state.calibrationToolSelected.gain={directory:'/offline/candidate',gain_candidate:candidate};
 const purposes=Object.keys(gainCandidateGroups(candidate)).filter(key=>['rigid','compliant','drag'].includes(key)&&gainCandidatePurposeAvailable(candidate,key));
 assert(purposes.length>0,'Fixture contains measured gain uses for adoption');
 state.gainPurposeSelections={[gainSelectionKey(candidate)]:[purposes[0]]};
 const gainSavedBefore=JSON.stringify(profileSavedValues()),runtimeBefore=JSON.stringify(state.telemetry),gainBus=Object.keys(state.profileEditor.hardware.buses)[0];
 state.profileEditor.hardware.buses[gainBus].device='/dev/unrelated-gain-draft';state.profileDirty=true;
 const callsBefore=(await api.fixtureCalls()).length,changed=await studioAdoptGainDraft({review:false});
 assert(changed.length>0&&state.profileReviewSelection.length>0&&state.profileReviewSelection.every(path=>path.startsWith('core.control.')),'Measured gain adoption scopes review to its selected control fields');
 assert(state.profileEditor.hardware.buses[gainBus].device==='/dev/unrelated-gain-draft'&&JSON.stringify(profileSavedValues())===gainSavedBefore&&JSON.stringify(state.telemetry)===runtimeBefore,'Gain adoption preserves unrelated drafts, saved defaults and runtime values');
 assert((await api.fixtureCalls()).length===callsBefore,'Gain adoption issues no device or disk-save request');
 // New R4 navigation and actions use the shared spring engine without delaying state.
 window.seedWorkspace('workbench','tuning');await tick();
 const sleep=ms=>new Promise(resolve=>setTimeout(resolve,ms));
 const side=document.querySelector('.sidebar'),tabGroup=document.querySelector('.response-workspace-tabs');
 await sleep(80);
 assert(!!side.querySelector(':scope > .studio-segment-indicator')&&!!tabGroup.querySelector(':scope > .studio-segment-indicator'),'Main navigation and response tabs own spring-driven selection indicators');
 const pressButton=document.querySelector('[data-response-view="parameters"]');
 pressButton.dispatchEvent(new KeyboardEvent('keydown',{key:' ',bubbles:true}));await sleep(70);
 const down=Number(getComputedStyle(pressButton).scale);
 pressButton.dispatchEvent(new KeyboardEvent('keyup',{key:' ',bubbles:true}));await sleep(300);
 assert(down<.999&&(Number.parseFloat(getComputedStyle(pressButton).scale)||1)>.999,'Secondary tab press springs and returns on keyboard release',{down});
 document.documentElement.dataset.reduceMotion='true';
 pressButton.dispatchEvent(new KeyboardEvent('keydown',{key:' ',bubbles:true}));await sleep(50);
 assert((Number.parseFloat(getComputedStyle(pressButton).scale)||1)>.999,'Reduced motion skips press movement while controls remain available');
 delete document.documentElement.dataset.reduceMotion;
 const surface=document.createElement('div');document.body.append(surface);window.serialArmStudio.enterSurface(surface);const start=Number(surface.style.opacity);await sleep(600);
 assert(start>=.85&&start<1&&!surface.hasAttribute('data-studio-spring-surface'),'Dialog and toast surface spring stays visible and settles',{start});surface.remove();
 while(priorityErrorOpen) acknowledgePriorityError();
 showPriorityError('Fixture diagnostic','operation');
 const priority=document.querySelector('#priority-error-overlay'),priorityCard=priority.querySelector('.priority-error-card');
 assert(priority.open&&priorityCard.hasAttribute('data-studio-spring-surface')&&Number(priorityCard.style.opacity)>=.85,'Priority error uses the same visible surface spring',{open:priority.open,present:priorityCard.hasAttribute('data-studio-spring-surface'),opacity:priorityCard.style.opacity,active:activePriorityError?.title,queued:priorityErrorQueue.length});
 acknowledgePriorityError();
 window.seedWorkspace('model');await tick();await sleep(750);
 const activeSidebar=document.querySelector('.sidebar .nav-item[data-studio-section].active'),sidebarMark=document.querySelector('.sidebar > .studio-segment-indicator');
 const sideTop=sidebarMark?.getBoundingClientRect().top,activeTop=activeSidebar?.getBoundingClientRect().top;
 assert(sidebarMark&&Math.abs(sideTop-activeTop)<2,'Sidebar spring indicator settles on the selected main navigation row',{sideTop,activeTop});
 const layer=document.querySelector('.model-toolbar label:has(input[type=checkbox])'),layerInput=layer?.querySelector('input');
 assert(!!layerInput,'Model layer control is available for keyboard motion');
 layerInput.dispatchEvent(new KeyboardEvent('keydown',{key:' ',bubbles:true}));await sleep(70);
 const layerDown=Number.parseFloat(getComputedStyle(layer).scale)||1;
 layerInput.dispatchEvent(new KeyboardEvent('keyup',{key:' ',bubbles:true}));await sleep(300);
 assert(layerDown<.999&&(Number.parseFloat(getComputedStyle(layer).scale)||1)>.999,'Layer checkbox uses the same spring press and release',{layerDown});
 await studioNavigate('hardware');await tick();await sleep(40);
 assert(!!document.querySelector('.studio-family-nav > .studio-segment-indicator')&&!!document.querySelector('.studio-device-directory > .studio-segment-indicator'),'Family tabs and device directory participate in spring selection');
 // Selection survives the full hardware directory rebuild and moves from
 // its previous screen position to the new object.
 await new Promise(resolve=>setTimeout(resolve,450));
 let directory=document.querySelector('.studio-device-directory');
 let marker=directory.querySelector(':scope > .studio-segment-indicator');
 const before=marker.getBoundingClientRect().top;
 const next=directory.querySelectorAll('[data-studio-hardware-object]')[4];
 assert(!!next,'Hardware fixture contains distant objects for spring continuity');
 next.click();
 await tick();
 directory=document.querySelector('.studio-device-directory');marker=directory.querySelector(':scope > .studio-segment-indicator');
 const immediate=marker.getBoundingClientRect().top;
 const destination=directory.querySelector('.selected').getBoundingClientRect().top;
 await new Promise(resolve=>setTimeout(resolve,65));
 const middle=marker.getBoundingClientRect().top;
 await new Promise(resolve=>setTimeout(resolve,550));
 const end=marker.getBoundingClientRect().top;
 assert(immediate<destination-1&&middle>immediate+1&&middle<destination-1&&Math.abs(end-destination)<2,
  'Rebuilt hardware directory retains the spring position and animates to the selected object',{before,immediate,middle,end,destination});
 // Retarget a moving selection after a full rerender and ensure reversal
 // starts from its current presentation rather than either endpoint.
 directory.querySelectorAll('[data-studio-hardware-object]')[0].click();
 await tick();await new Promise(resolve=>setTimeout(resolve,55));
 directory=document.querySelector('.studio-device-directory');
 marker=directory.querySelector(':scope > .studio-segment-indicator');
 const reversalStart=marker.getBoundingClientRect().top;
 const reverseTarget=directory.querySelectorAll('[data-studio-hardware-object]')[4];
 const reverseDestination=reverseTarget.getBoundingClientRect().top;
 reverseTarget.click();await tick();
 directory=document.querySelector('.studio-device-directory');
 marker=directory.querySelector(':scope > .studio-segment-indicator');
 const reversalImmediate=marker.getBoundingClientRect().top;
 await new Promise(resolve=>setTimeout(resolve,100));
 const reversalMiddle=marker.getBoundingClientRect().top;
 await new Promise(resolve=>setTimeout(resolve,500));
 const reversalEnd=marker.getBoundingClientRect().top;
 assert(reversalStart<reverseDestination-1&&Math.abs(reversalImmediate-reversalStart)<60&&reversalMiddle>reversalImmediate&&Math.abs(reversalEnd-reverseDestination)<2,
  'Rapid reversal keeps current spring presentation and reaches the new object',{reversalStart,reversalImmediate,reversalMiddle,reversalEnd,reverseDestination});
 // A stalled renderer must not catch up the entire pause in a single frame.
 directory.querySelectorAll('[data-studio-hardware-object]')[0].click();await tick();
 directory=document.querySelector('.studio-device-directory');marker=directory.querySelector(':scope > .studio-segment-indicator');
 const stallStart=marker.getBoundingClientRect().top,stallTarget=directory.querySelector('.selected').getBoundingClientRect().top;
 const deadline=performance.now()+220;while(performance.now()<deadline){};
 await new Promise(resolve=>requestAnimationFrame(resolve));
 const stallAfter=marker.getBoundingClientRect().top;
 assert(Math.abs(stallAfter-stallTarget)>2&&Math.abs(stallAfter-stallTarget)<Math.abs(stallStart-stallTarget),
  'A main-thread stall does not snap an active spring to its target',{stallStart,stallAfter,stallTarget});
 const calls=await api.fixtureCalls(),readOnly=new Set(['status','logs','profile_library','profiles','inspect','readiness','profile_editor_load','model','hardware_inspect','model_preview','trajectory_list','geometry_fixed_state','geometry_repeat_status']);
 assert(calls.every(name=>readOnly.has(name)),'All fixture requests are reads; no connection, motion or configuration write was issued',{calls});
 return checks;
}
app.whenReady().then(async()=>{
 fs.mkdirSync(output,{recursive:true});
 const win=new BrowserWindow({width:1440,height:900,show:false,webPreferences:{preload:fixture.preload,sandbox:false,backgroundThrottling:false}});
 const errors=[];win.webContents.on('console-message',(_event,level,message)=>{if(level>=3)errors.push(message);});
 win.webContents.on('render-process-gone',(_event,details)=>{console.error('Renderer exited',details);app.exit(1);});
 const deadline=setTimeout(()=>{console.error('Workspace fixture timed out');app.exit(1);},90000);
 await win.loadFile(fixture.index);win.show();
 await win.webContents.executeJavaScript(`(async()=>{await document.fonts.ready;await new Promise(resolve=>setTimeout(resolve,300));window.seedWorkspace=${seedWorkspace};return true;})()`);
 const checks=await win.webContents.executeJavaScript(`(${verifyWorkspaces})()`);
 win.setContentSize(900,720);
 const narrow=await win.webContents.executeJavaScript(`(async()=>{window.seedWorkspace('hardware');await new Promise(resolve=>setTimeout(resolve,700));const group=document.querySelector('.studio-device-directory'),marker=group.querySelector(':scope > .studio-segment-indicator'),selected=group.querySelector('.selected'),a=marker.getBoundingClientRect(),b=selected.getBoundingClientRect();return {position:getComputedStyle(group).position,dx:Math.abs(a.left-b.left),dy:Math.abs(a.top-b.top),dw:Math.abs(a.width-b.width),dh:Math.abs(a.height-b.height)};})()`);
 if(narrow.position!=='relative'||Math.max(narrow.dx,narrow.dy,narrow.dw,narrow.dh)>2)throw Error('Narrow hardware directory marker must align with the selected item: '+JSON.stringify(narrow));
 checks.push({name:'Narrow hardware directory retains its selection indicator positioning',evidence:narrow});
 if(errors.length)throw Error('Renderer errors: '+errors.join('\n'));
 const report={checks,source:'Production renderer and isolated read-only fixture; actual URDF loaded in rescue mode'};
 fs.writeFileSync(path.join(output,'report.json'),JSON.stringify(report,null,2));clearTimeout(deadline);
 console.log('WORKSPACE_NAVIGATION_PASS '+JSON.stringify({checks:checks.length,report:path.join(output,'report.json')}));app.exit(0);
}).catch(error=>{console.error(error);app.exit(1);});
