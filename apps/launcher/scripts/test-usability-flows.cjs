// Actual renderer and Chromium DOM with explicitly mocked IPC, without hardware.
// This verifies UI request contracts; backend file operations have separate tests.
const fs = require('node:fs'), path = require('node:path'), os = require('node:os');
const assert = require('node:assert/strict');
const {spawnSync, execFileSync} = require('node:child_process');
if (!process.versions.electron) {
    const electron = require('electron');
    const env = {...process.env}; delete env.ELECTRON_RUN_AS_NODE;
    if (!env.DISPLAY) throw Error('Run this UI regression with an available graphical display');
    const result = spawnSync(electron,[...(process.getuid?.() === 0 ? ['--no-sandbox'] : []),__filename],{env,stdio:'inherit'});
    if (result.error) throw result.error;
    process.exit(result.status ?? 1);
}
const {app,BrowserWindow,ipcMain} = require('electron');
const root=path.resolve(__dirname,'../../..'), launcher=path.join(root,'apps/launcher');
const output=fs.mkdtempSync(path.join(os.tmpdir(),'serialarm-usability-flows-'));
// Generate the ordinary built-in model through the real read-only backend;
// explicit fixture paths remain useful when diagnosing a particular robot.
const python=fs.existsSync(path.join(root,'.install/gui-venv/bin/python')) ? path.join(root,'.install/gui-venv/bin/python') : 'python3';
const generated=process.env.SERIALARM_MODEL_FIXTURE && process.env.SERIALARM_EDITOR_FIXTURE ? {} : JSON.parse(execFileSync(python,['-B','-c',`import sys,json
from pathlib import Path
root=Path(${JSON.stringify(root)})
sys.path.insert(0,str(root/'apps/launcher/backend'))
from profiles import Inspector
from profile_library import ProfileLibrary
from model_runtime import ModelRuntime
inspector=Inspector(root)
runtime=ModelRuntime()
try:
    config={'profile':'dm_arm_gray'}
    print(json.dumps({'model':runtime.model(inspector,config),'editor':ProfileLibrary(root,inspector).profile_editor_load(config)}))
finally:
    runtime.close()
`],{encoding:'utf8',env:{...process.env,PYTHONDONTWRITEBYTECODE:'1',PYTHONNOUSERSITE:'1',XDG_DATA_HOME:output}}));
const fixture=(key,fallback)=>process.env[key] ? JSON.parse(fs.readFileSync(process.env[key],'utf8')) : fallback;
const model=fixture('SERIALARM_MODEL_FIXTURE',generated.model);
const editor=fixture('SERIALARM_EDITOR_FIXTURE',generated.editor);
const names=model.native.config.joint_names, zero=names.map(()=>0);
const inertial=model.model.links.find(link=>link.inertial?.mass>0);
const candidate={static_pass:true,constraints_ok:true,first_moments:[{link_name:inertial.name,value:inertial.inertial.origin.xyz.map(value=>value*inertial.inertial.mass)}]};
const candidates=['new','old'].map((id,index)=>({id,source:'offline',path:'/tmp/ui-task/offline_candidates/'+id+'/recomputed-candidate.json',created_at:200-index,summary:{static_pass:true,recommendation:id==='old'?'prior_preferred':'candidate_preferred'}}));
const dynamicCandidates=[{id:'dyn-old',source:'idim',path:'/tmp/dynamic-task/idim_runs/dyn-old/identification-report.json',created_at:100,summary:{status:'review_passed_offline'}}];
const action={page:'profile_setup',group:'motion',field:'core.control.runtime.write_enabled'};
const readiness={state:'ready',control_available:true,checks:{profile_setup:'verified'},gain_purposes:{},
    modes:Object.fromEntries(['RIGID_HOLD','RIGID_TRACKING','COMPLIANT_HOLD','COMPLIANT_TRACKING','COMPLIANT_DRAG','SERVO_TRACKING'].map(key=>[key,'configured'])),
    setup_issues:[],connection_readiness:{structurally_valid:true,read_only_connectable:true,write_enabled:false,basic_calibration_ready:false,
        basic_calibration_prerequisites:[{id:'write_enabled',label:'执行器写入',ready:false,action}],next_actions:[action]}};
let trajectories=[{id:'saved-A',name:'已保存轨迹',directory:'/tmp/trajectory-A',samples:3,start:zero,end:zero.map((v,i)=>i===0?.1:v)}];
let control={robot_state:'ACTIVE',write_enabled:true,valid:true,sequence:10,monotonic_ns:2000000000,feedback_age_ms:1,
    impedance_mode:'RIGID_HOLD',gain_profile:'RIGID',gain_source:'CONFIGURED',reference_behavior:'NATIVE_IMPEDANCE',
    model_feedforward_mode:'GRAVITY',joint_names:names,effective_kp:zero.map(()=>20),effective_kd:zero.map(()=>1),
    joint:{pos:zero,vel:zero,tor:zero,nominal_ref_pos:zero,virtual_ref_pos:zero,accepted_ref_pos:zero},
    teach_replay:{phase:'idle'},model_calibration:{phase:'idle'}};
const restoredGeometry={active:true,parent_frame:'tool0',suggested_frame:'tip',frame_options:['tip','tool0'],purpose:'tcp',
    train:10,validation:5,candidate:{tcp_xyz_m:[.001,.002,.003]},report:{passed:true,rms_mm:.2,max_mm:.3,errors_mm:[.1,.2,.3,.2,.1],rms_limit_mm:2,max_limit_mm:4}};
const calls=[],checks=[],errors=[];
let exportGate=null;
let nextMotionId=41,library=Array.from({length:3},(_,i)=>({id:'robot-'+i,profile:i?'robot_'+i:'dm_arm_gray',source:'builtin',readiness}));
const dynamicReport={task_id:'dynamic-ui',report_path:dynamicCandidates[0].path,report:dynamicCandidates[0].path,run_id:'dyn-old',status:'review_passed_offline',validation_scope:'dynamic_inertia',full_dynamics_physical_pass:true,physical_pass:true,physical_review:[],exportable:true,review_exportable:true,metrics:{},candidate_urdf:'/tmp/dynamic-task/candidate.urdf'};
const record=directory=>directory==='/tmp/dynamic-task'?{task_id:'dynamic-ui',directory,has_result:true,metadata:{urdf_fingerprint:model.native.config.urdf_fingerprint},result:{task_id:'dynamic-ui',phase:'complete',workflow_kind:'dynamics'},idim_candidate:dynamicReport,dynamic_candidates:[dynamicReport]}:
    {task_id:'ui-task',directory,has_result:true,metadata:{urdf_fingerprint:model.native.config.urdf_fingerprint},result:{task_id:'ui-task',phase:'complete',workflow_kind:'gravity',gravity_result:candidate},recomputed_candidate:candidate};
const allowed=new Set((model.mesh_files || []).map(file=>path.resolve(file)));
const preload=path.join(output,'preload.cjs');
fs.writeFileSync(preload,"const {contextBridge,ipcRenderer}=require('electron');contextBridge.exposeInMainWorld('serialArm',{request:(method,params)=>ipcRenderer.invoke('usability:request',method,params),prefs:async()=>({language:'zh-CN',theme:'light'}),savePrefs:async()=>true,onEvent:()=>()=>{},window:()=>{},selectResources:async()=>'/tmp/ui-export',selectTrajectory:async()=>'/tmp/ui-trajectory.zip',selectGainCandidate:async()=>'/tmp/gain-candidate.json',selectProfile:async()=>null,selectProfileDirectory:async()=>null,clipboard:{write:async()=>{},read:async()=>''},modelResource:file=>ipcRenderer.invoke('usability:resource',file)});");
ipcMain.handle('usability:resource',(_event,file)=>{
    assert.ok(allowed.has(path.resolve(file)),'only fixture model assets may be read');
    return {extension:path.extname(file),data:fs.readFileSync(file).toString('base64')};
});
ipcMain.handle('usability:request',(_event,method,params={})=>{
    calls.push({method,params});
    if(method==='status')return {installed:true,session:{state:'idle'}};
    if(method==='logs')return '';
    if(method==='profile_library')return {profiles:library};
    if(method==='profile_editor_load')return structuredClone(editor);
    if(method==='profiles')return {profiles:[{name:params.profile || 'dm_arm_gray'}]};
    if(method==='readiness')return readiness;
    if(method==='inspect')return {profile:'dm_arm_gray',checks:[],available:{},resources:{},model_identity:{urdf:'A',sha:'A'}};
    if(method==='model_load')return model;
    if(method==='model_preview')return {...model.native,state:{...model.native.state,positions:params.positions}};
    if(method==='trajectory_list')return {items:trajectories,directory:'/tmp/teach-replay'};
    if(method==='trajectory_select')return {...trajectories.find(item=>item.id===params.id)};
    if(method==='trajectory_load')return {...trajectories.find(item=>item.id===params.id),phase:'ready',has_trajectory:true};
    if(method==='trajectory_export')return {path:'/tmp/exported-trajectory.zip'};
    if(method==='trajectory_rename'){trajectories=trajectories.map(item=>item.id===params.id?{...item,name:params.name}:item);return trajectories[0];}
    if(method==='trajectory_remove'){trajectories=trajectories.filter(item=>item.id!==params.id);return true;}
    if(method==='calibration_tool_list')return {items:[]};
    if(method==='calibration_tool_records_page')return {records:Array.from({length:params.offset?1:30},(_,i)=>({task_id:'task-'+(params.offset+i),directory:'/tmp/ui-task',record_kind:'gravity',trajectory:{samples:3}})),total:31,offset:params.offset,has_more:!params.offset};
    if(method==='calibration_candidates')return {directory:params.directory,candidates:params.kind==='dynamics'?dynamicCandidates:candidates};
    if(method==='model_calibration_recompute')return {candidate:{candidate_run_id:'old'},path:'/tmp/ui-task/recomputed-candidate.json'};
    if(method==='model_calibration_idim_export')return {candidate_urdf:'/tmp/ui-export/dynamic.urdf'};
    if(method==='calibration_tool_load'){
        const selected=(params.kind==='dynamics'?dynamicCandidates:candidates).find(item=>item.path===params.candidate_path);
        return {...record(params.directory),...(selected?{selected_candidate:{...selected,kind:params.kind}}:{})};
    }
    if(method==='calibration_tool_export')return exportGate || {path:'/tmp/ui-export/calibration.zip'};
    if(method==='calibration_candidate_apply')return {[params.kind+'_applied']:true};
    if(method==='calibration_candidate_status')return {};
    if(method==='geometry_fixed_state')return restoredGeometry;
    if(method==='geometry_fixed_preview_tcp')return {profile:'dm_arm_gray',source_urdf:'/tmp/source.urdf',source_core:'/tmp/core.yaml',source_profile:'/tmp/source-profile.yaml',parent_frame:'tool0',tool_frame:'dm_arm_gray_tcp',fixed_joint:'dm_arm_gray_tcp_fixed',tcp_xyz_m:[.001,.002,.003],source_hashes:{core:'core-sha',urdf:'urdf-sha',profile:'profile-sha'},change_summary:[{field:'core.model.tool_frame',before:'tool0',after:'dm_arm_gray_tcp'}],writes_source:false,orientation:'沿用父坐标系朝向'};
    if(method==='geometry_fixed_apply_tcp'){const entry={id:'tcp-entry',profile:'dm_arm_gray_tcp',source:'external',profile_file:'/tmp/ui-export/tcp-profile.yaml',readiness};library.push(entry);return {path:entry.profile_file,profile_file:entry.profile_file,urdf:'/tmp/ui-export/tcp.urdf',config:{profile:entry.profile,profile_file:entry.profile_file},adopted:true,applied:true,library_entry:entry};}
    if(method==='workbench_request'){
        if(params.method==='status')return control;
        if(params.method==='teach_replay_play')return {phase:'playing',has_trajectory:true,completed:0,start:zero,end:trajectories[0].end};
        if(params.method==='move_to')return {id:nextMotionId++,state:'running',target:params.params.positions,reference_complete:false,settled_s:0};
        throw Error('Native action not authorized by UI fixture: '+params.method);
    }
    throw Error('Unexpected or hardware request blocked: '+method);
});
app.setPath('userData',path.join(output,'profile'));
app.commandLine.appendSwitch('use-angle','swiftshader');app.commandLine.appendSwitch('enable-unsafe-swiftshader');
const delay=ms=>new Promise(resolve=>setTimeout(resolve,ms));
let win;
const run=(fn,...args)=>win.webContents.executeJavaScript('('+fn.toString()+')(...'+JSON.stringify(args)+')');
async function idle(){for(let i=0;i<100;i++){if(await run(()=>!state.blockingBusy))return;await delay(25);}throw Error('Renderer operation did not finish');}
async function click(selector){
    await idle();
    await run(selector=>{const node=document.querySelector(selector);if(!node)throw Error('Missing target '+selector);if(node.disabled)throw Error('Disabled target '+selector);for(let parent=node.parentElement;parent;parent=parent.parentElement)if(parent.tagName==='DETAILS')parent.open=true;node.scrollIntoView({block:'center',behavior:'instant'});node.click();},selector);
    await delay(80);await idle();await delay(30);
}
async function input(selector,value,type='input'){
    await idle();
    await run((selector,value,type)=>{const node=document.querySelector(selector);if(!node)throw Error('Missing input '+selector);node.value=value;node.dispatchEvent(new Event(type,{bubbles:true}));},selector,value,type);
    await delay(50);await idle();
}
async function mounted(){for(let i=0;i<100;i++){if(await run(()=>!!modelView?.hasFramedModel))return;await delay(30);}throw Error('Model did not mount');}
async function workspace(tab='control',connected=true){
    await run((model,editor,control,readiness,tab,connected)=>{
        state.config.profile='dm_arm_gray';state.page='workbench';state.workbenchTab=tab;state.profileDirty=false;
        state.modelData=model;state.capabilityProfile=editor;state.admittance=editor.core.capability.admittance;
        state.readiness={...readiness,connection_readiness:{...readiness.connection_readiness,write_enabled:true,basic_calibration_ready:true,basic_calibration_prerequisites:[]}};state.modelReloadPending=false;state.telemetry=connected?control:null;state.telemetryReceivedAt=Date.now();
        state.session=connected?{state:'running',mode:'workbench',session_id:'ui-session',pid:111}:{state:'idle'};
        state.workbenchIdentity=connected?'ui-session':null;state.workbenchTargets=control.joint.pos.slice();
        state.workbenchTargetEpoch=state.workbenchEpoch;state.workbenchTargetDirty=false;
        state.pendingTarget=null;state.pendingMotionId=null;state.commandState='idle';state.speedScale=.2;state.speedInputRaw='.2';state.speedInputError='';
        shell();
    },model,editor,control,readiness,tab,connected);
    await mounted();await delay(150);
}
const last=method=>calls.filter(call=>call.method===method).at(-1);
app.whenReady().then(async()=>{
    win=new BrowserWindow({show:true,width:1600,height:1000,webPreferences:{preload,contextIsolation:true,sandbox:false,backgroundThrottling:false}});
    win.webContents.on('console-message',(_e,...args)=>{const text=args.map(String).join(' ');if(text.includes('Uncaught'))errors.push(text);});
    try{
        await win.loadFile(path.join(launcher,'renderer/index.html'));await delay(500);
        for(const [width,height] of [[1300,900],[1900,1080]]){
            win.setSize(width,height);
            await run(library=>{state.page='start';state.library=library;state.config.profile='';state.session={state:'idle'};shell();},library);await delay(350);
            const layout=await run(()=>{
                const content=document.querySelector('.content'),r=content.getBoundingClientRect();
                return {right:r.right,window:innerWidth,outer:document.documentElement.scrollWidth,scroll:content.scrollHeight,height:content.clientHeight,
                    nested:[...content.querySelectorAll('*')].filter(node=>['auto','scroll'].includes(getComputedStyle(node).overflowY)&&node.scrollHeight>node.clientHeight+1).map(node=>node.className)};
            });
            assert.ok(Math.abs(layout.right-layout.window)<2,'home scroll surface reaches window edge '+JSON.stringify(layout));
            assert.ok(layout.outer<=layout.window+1,'home has no horizontal overflow');
            assert.deepEqual(layout.nested,[],'home has one scrolling surface');
            assert.equal(await run(()=>document.querySelectorAll('[data-library-row]').length),3);
            assert.equal(await run(()=>document.querySelector('#library-import-dir').textContent.trim()),'打开机械臂 Profile');
            assert.equal(await run(()=>document.querySelector('[data-start-intent="description"]').textContent.trim()),'新建机械臂 Profile');
            assert.match(await run(()=>document.querySelector('#library-import-file').textContent),/机械臂 Profile.*YAML/);
            fs.writeFileSync(path.join(output,'home-'+width+'.png'),(await win.webContents.capturePage()).toPNG());
        }
        checks.push('home scrolling and complete Profile entry labels at both requested sizes');

        await workspace();
        const before=calls.filter(call=>call.method==='workbench_request'&&call.params.method==='move_to').length;
        for(const value of ['0','','-0.1']){
            await input('#workbench-speed',value);
            assert.equal(await run(()=>document.querySelector('#workbench-speed').getAttribute('aria-invalid')),'true');
            assert.equal(await run(()=>document.querySelector('#target-move').disabled),true);
        }
        assert.equal(calls.filter(call=>call.method==='workbench_request'&&call.params.method==='move_to').length,before);
        checks.push('invalid zero, empty and negative speed cannot issue a move');
        await input('#workbench-speed','.15');await click('#target-move');
        assert.equal(last('workbench_request').params.method,'move_to');
        assert.equal(last('workbench_request').params.params.speed_scale,.15);
        const id=await run(()=>state.pendingMotionId);assert.equal(id,41);
        await run(id=>{updateMotionCompletion({motion:{id:id+1,state:'completed'}});},id);
        assert.notEqual(await run(()=>state.commandState),'complete','another command cannot finish this command');
        await run(control=>{applyWorkbenchSnapshot({...control,joint:{...control.joint,pos:control.joint.pos.map((v,i)=>i===0?.01:v)},motion:{id:41,state:'running'}});},control);
        assert.equal(await run(()=>state.commandState),'running','small stationary error is not completion');
        await run(()=>updateMotionCompletion({motion:{id:41,state:'completed'}}));
        assert.equal(await run(()=>state.commandState),'complete');
        checks.push('matching runtime motion completion replaces heuristic position completion');

        await run(()=>{state.pendingMotionId=91;state.pendingTarget=state.workbenchTargets.slice();state.commandState='running';});
        // The public helper is the same path used by range and numeric inputs.
        await run(()=>editWorkbenchTarget(0,.02));
        assert.equal(await run(()=>state.pendingMotionId),null);
        await run(()=>updateMotionCompletion({motion:{id:91,state:'completed'}}));
        assert.equal(await run(()=>state.commandState),'edited');
        checks.push('editing a new target prevents an old motion from completing the draft');

        await run(control=>applyWorkbenchSnapshot({...control,robot_state:'FAULT',reference_behavior:'MODEL_ADMITTANCE',impedance_mode:'COMPLIANT_DRAG',gain_profile:'SERVO_TRACKING',feedback_age_ms:87,fault:{code:'STATE_TIMEOUT'}}),control);
        assert.equal(await run(()=>document.querySelectorAll('.workspace-status>span').length),7);
        const status=await run(()=>Object.fromEntries([...document.querySelectorAll('[data-status-field]')].map(node=>[node.dataset.statusField,node.textContent])));
        assert.match(status.behavior,/导纳/);assert.match(status.task,/拖拽/);assert.match(status.age,/87 ms/);assert.equal(status.fault,'STATE_TIMEOUT');
        await run(()=>applyWorkbenchSnapshot({...state.telemetry,robot_state:'ACTIVE',reference_behavior:'NATIVE_IMPEDANCE',impedance_mode:'RIGID_HOLD',gain_profile:'RIGID',feedback_age_ms:2,fault:null}));
        assert.equal(await run(()=>document.querySelector('[data-status-field="fault"]').textContent),'OK');
        checks.push('live status fields update independently and clear recovered faults');

        await run(model=>{state.modelIdentity={sha:'A'};state.modelData=model;state.workbenchTargets=[.1];state.ghostRequestKey='old';window.identityView=modelView;adoptInspectionInfo({model_identity:{sha:'A'}});},model);
        assert.equal(await run(()=>state.modelData!==null&&modelView===window.identityView),true);
        await run(()=>adoptInspectionInfo({model_identity:{sha:'B'}}));
        assert.equal(await run(()=>state.modelData===null&&modelView===null&&state.workbenchTargets.length===0&&state.ghostRequestKey===''),true);
        checks.push('same model identity keeps preview while changed URDF clears stale model, ghost and draft');

        await workspace('replay',false);
        await run(async()=>{await refreshTeachTrajectories();refreshTeachReplayPanel();});
        const selectStart=calls.length;await click('[data-teach-select="saved-A"]');
        const selectionCalls=calls.slice(selectStart);
        assert.ok(selectionCalls.some(call=>call.method==='trajectory_select'));
        assert.ok(!selectionCalls.some(call=>call.method==='workbench_request'||call.method==='trajectory_load'));
        for(const selector of ['#teach-export','#teach-rename','#teach-delete'])assert.equal(await run(selector=>document.querySelector(selector).disabled,selector),false);
        await click('#teach-export');assert.equal(last('trajectory_export').params.id,'saved-A');
        await click('#teach-rename');await input('#teach-name-input','改名轨迹');await click('#teach-name-okay');assert.equal(last('trajectory_rename').params.name,'改名轨迹');
        await click('#teach-delete');await click('#teach-del-ok');assert.equal(last('trajectory_remove').params.id,'saved-A');
        checks.push('offline trajectory selection, export, rename and deletion never require Native connection');

        trajectories=[{id:'saved-A',name:'回放测试轨迹',directory:'/tmp/trajectory-A',samples:30,duration_s:.6,start:zero,end:zero.map((v,i)=>i===0?.1:v)}];
        await workspace('replay',true);
        await run(async()=>{await refreshTeachTrajectories();refreshTeachReplayPanel();});
        await click('[data-teach-select="saved-A"]');
        assert.equal(await run(()=>state.teachLoadedId),'','selection never loads into Native implicitly');
        await click('#teach-load');assert.equal(last('trajectory_load').params.id,'saved-A');
        await input('#teach-rate','.01');
        assert.equal(await run(()=>document.querySelector('#teach-rate').getAttribute('aria-invalid')),'true');
        assert.equal(await run(()=>document.querySelector('#teach-play').disabled),true);
        await input('#teach-rate','.2');
        for(const direction of ['forward','reverse','cycle']){
            const pose=direction==='reverse'?trajectories[0].end:zero;
            await run((control,pose)=>applyWorkbenchSnapshot({...control,joint:{...control.joint,pos:pose},teach_replay:{phase:'ready',has_trajectory:true}}),control,pose);
            await input('#teach-direction',direction,'change');
            if(direction==='cycle')await input('#teach-repeats','3');
            assert.equal(await run(()=>document.querySelector('#teach-repeats').closest('label').hidden),direction!=='cycle');
            await click('#teach-play');
            assert.ok(await run(()=>document.querySelector('#confirm-dialog').textContent.includes('单程')));
            await run(()=>{const released=document.querySelector('#teach-released');released.checked=true;released.dispatchEvent(new Event('change',{bubbles:true}));});
            await click('#teach-confirm-start');
            const request=last('workbench_request');assert.equal(request.params.method,'teach_replay_play');
            assert.equal(request.params.params.direction,direction);assert.equal(request.params.params.cycles,direction==='cycle'?3:1);
        }
        checks.push('explicit Native loading, invalid replay speed, single pass and complete round-trip contracts');
        await run(control=>applyWorkbenchSnapshot({...control,teach_replay:{phase:'ready',has_trajectory:true}}),control);
        await input('#teach-direction','reverse','change');
        const nativeBeforeDraft=calls.filter(call=>call.method==='workbench_request').length;
        await click('#teach-fill-start');
        assert.equal(await run(()=>state.workbenchTab),'control');
        assert.deepEqual(await run(()=>state.workbenchTargets),trajectories[0].end);
        assert.equal(await run(()=>state.commandState),'edited');
        assert.equal(calls.filter(call=>call.method==='workbench_request').length,nativeBeforeDraft,'endpoint draft cannot move or load Native');
        checks.push('endpoint target drafting changes the Control preview without sending a motion');

        await run((model,geometry)=>{state.page='geometry';state.geometryTab='fixed';state.modelData=model;state.geometryState=geometry;state.geometryFrame='tip';state.geometryPurpose='diagnostic';state.session={state:'idle'};shell();},model,restoredGeometry);await mounted();
        assert.equal(await run(()=>document.querySelector('#geometry-frame').value),'tool0');
        assert.equal(await run(()=>document.querySelector('#geometry-frame').disabled),true);
        assert.equal(await run(()=>document.querySelector('#geometry-purpose').value),'tcp');
        checks.push('restored active geometry displays its recorded parent frame and purpose');

        await run(readiness=>{state.page='lifecycle';state.readiness=readiness;state.profileEditor=null;shell();},readiness);await delay(200);
        const prerequisite=await run(()=>[...document.querySelectorAll('[data-setup-target]')].find(node=>JSON.parse(node.dataset.setupTarget).field==='core.control.runtime.write_enabled')?.dataset.setupTarget);
        assert.ok(prerequisite,'preparation exposes calibration prerequisites');
        await run(action=>openSetupTarget(action),JSON.parse(prerequisite));await delay(250);
        assert.equal(await run(()=>state.page),'profile_setup');assert.equal(await run(()=>state.profileGroup),'motion');
        assert.equal(await run(()=>JSON.parse(document.activeElement.dataset.profileField || '[]').join('.')),'core.control.runtime.write_enabled');
        checks.push('read-only connection and calibration prerequisites link to exact write-enable configuration');

        await run((model,editor)=>{state.page='calibration';state.calibrationEntry='gravity';state.modelData=model;state.capabilityProfile=editor;state.session={state:'idle'};state.modelCalibrationOffline=null;state.calibrationHistoryPages={};shell();},model,editor);await mounted();
        await input('[data-tool-query="gravity"]','payload-A','input');await click('[data-tool-scan="gravity"]');
        assert.equal(last('calibration_tool_records_page').params.query,'payload-A');assert.equal(last('calibration_tool_records_page').params.offset,0);
        await click('[data-tool-page="gravity"][data-offset="30"]');assert.equal(last('calibration_tool_records_page').params.offset,30);
        assert.equal(await run(()=>document.querySelectorAll('[data-tool-select="gravity"]').length),1);
        checks.push('history search and pagination preserve filters and show the requested page');

        await click('[data-tool-select="gravity"]');
        await input('[data-candidate-run="gravity"]','old','change');
        assert.equal(last('calibration_tool_load').params.candidate_path,candidates[1].path);
        await input('[data-candidate-compare="gravity"]','new','change');
        assert.equal(await run(()=>state.calibrationCandidateCompare.gravity),'new');
        let releaseExport;exportGate=new Promise(resolve=>{releaseExport=resolve;});
        await click('[data-tool-export="gravity"]');
        assert.equal(await run(()=>state.blockingBusy),false,'file export cannot block robot control');
        assert.equal(await run(()=>document.querySelector('[data-tool-export="gravity"]').dataset.exportBusy),'true');
        assert.equal(last('calibration_tool_export').params.candidate_path,candidates[1].path);
        releaseExport({path:'/tmp/ui-export/calibration.zip'});exportGate=null;await delay(100);
        checks.push('pending file export uses local busy state and leaves control actions unblocked');
        assert.equal(last('calibration_tool_export').params.candidate_source,'offline');
        checks.push('selected historical candidate and comparison drive exact export source');
        await run(control=>{state.session={state:'running',mode:'workbench',session_id:'ui-session',pid:111};state.telemetry={...control,robot_state:'INACTIVE'};render();},control);await delay(150);
        await click('#calibration-gravity-temp-apply');
        assert.equal(last('calibration_candidate_apply').params.candidate_path,candidates[1].path);
        checks.push('temporary apply passes the selected archived candidate rather than the latest run');

        await input('[data-candidate-run="gravity"]','new','change');
        await click('#model-calibration-recompute');
        assert.equal(await run(()=>state.calibrationCandidateSelection.gravity.id),'old');
        assert.equal(last('calibration_tool_load').params.candidate_path,candidates[1].path,'root recompute pointer resolves to the matching archived run');
        checks.push('recompute selects its run ID even when its latest pointer path differs from archive');
        await run(async()=>{await refreshCalibrationCandidates('dynamics','/tmp/dynamic-task','dyn-old');});
        await click('[data-calibration-entry="dynamics"]');
        assert.equal(await run(()=>state.modelCalibrationDirectory),'/tmp/dynamic-task');
        assert.equal(await run(()=>state.idimReport.report_path),dynamicCandidates[0].path);
        await click('#idim-export');
        assert.equal(last('model_calibration_idim_export').params.report_path,dynamicCandidates[0].path);
        await click('[data-calibration-entry="gravity"]');
        assert.equal(await run(()=>state.modelCalibrationDirectory),'/tmp/ui-task');
        await click('[data-tool-export="gravity"]');
        assert.equal(last('calibration_tool_export').params.directory,'/tmp/ui-task');
        assert.equal(last('calibration_tool_export').params.candidate_path,candidates[1].path);
        checks.push('switching gravity and dynamics restores each tool record and exports the displayed source');

        // The TCP adoption controls are deliberately exercised only after an
        // independently validated experiment; the IPC fixtures perform no file write.
        await run((model,geometry)=>{state.page='geometry';state.geometryTab='fixed';state.modelData=model;state.geometryState=geometry;state.session={state:'idle'};state.geometryTcpPreview=null;shell();},model,restoredGeometry);await mounted();
        await click('#geometry-preview-tcp');
        assert.ok(last('geometry_fixed_preview_tcp'));
        assert.ok(await run(()=>geometryTcpViewState().preview!==null));
        assert.ok(!calls.some(call=>call.method==='geometry_fixed_apply_tcp'),'preview cannot adopt implicitly');
        await click('#geometry-apply-tcp');
        await run(()=>{const dialog=document.querySelector('#confirm-dialog');if(dialog.open){for(const node of dialog.querySelectorAll('input[type="checkbox"]')){node.checked=true;node.dispatchEvent(new Event('change',{bubbles:true}));}}});
        const confirm=await run(()=>[...document.querySelectorAll('#confirm-dialog button')].find(node=>!node.disabled&&node.classList.contains('primary'))?.id);
        if(confirm)await click('#'+confirm);
        assert.ok(last('geometry_fixed_apply_tcp'));
        assert.equal(last('geometry_fixed_apply_tcp').params.confirmed,true);
        assert.deepEqual(last('geometry_fixed_apply_tcp').params.expected_hashes,{core:'core-sha',urdf:'urdf-sha',profile:'profile-sha'});
        assert.equal(await run(()=>state.activeLibraryId),'tcp-entry');
        assert.equal(await run(()=>state.config.profile),'dm_arm_gray_tcp');
        assert.ok(!calls.some(call=>['workbench_start','start'].includes(call.method)),'adoption never connects or launches the robot');
        checks.push('TCP preview is separate from explicit adoption into an independent Profile');

        assert.deepEqual(errors,[]);
        fs.writeFileSync(path.join(output,'verification.json'),JSON.stringify({checks,calls,errors,rendererAndIpcOnly:true,hardwareAccessed:false},null,2));
        console.log('USABILITY_RENDERER_FLOWS_PASS',checks.length,output);win.destroy();app.exit(0);
    }catch(error){
        console.error(error.stack);fs.writeFileSync(path.join(output,'failure.png'),(await win.webContents.capturePage()).toPNG());
        fs.writeFileSync(path.join(output,'failure.json'),JSON.stringify({error:String(error),checks,calls,errors},null,2));
        console.error(output);win.destroy();app.exit(1);
    }
});
