/* Actual renderer functions, preload IPC and Electron window lifecycle; no backend/device process.
 * DISPLAY=:1 node_modules/.bin/electron --no-sandbox scripts/verify-session-lifecycle.cjs
 */
const assert = require('node:assert/strict');
const fs = require('node:fs');
const os = require('node:os');
const path = require('node:path');
const vm = require('node:vm');
const {pathToFileURL} = require('node:url');
const {app, BrowserWindow, ipcMain} = require('electron');
const root = path.resolve(__dirname, '..');
const renderer = fs.readFileSync(path.join(root, 'renderer/app.js'), 'utf8');
const main = fs.readFileSync(path.join(root, 'desktop/main.cjs'), 'utf8');
function extract(source, name) {
    const match = new RegExp(`^(?:async )?function ${name}\\(`, 'm').exec(source);
    assert(match, `Missing production function ${name}`);
    const lineEnd = source.indexOf('\n', match.index);
    const firstLine = source.slice(match.index, lineEnd);
    const end = firstLine.endsWith('}') ? lineEnd : source.indexOf('\n}', lineEnd) + 2;
    assert(end > match.index, `Cannot extract ${name}`);
    return source.slice(match.index, end);
}
const checks = [];
function check(name, condition) { assert(condition, name); checks.push(name); }
const pause = ms => new Promise(resolve => setTimeout(resolve, ms));
async function waitUntil(predicate) {
    for (let n = 0; n < 100; n++) { if (predicate()) return; await pause(10); }
    throw Error('Timed out waiting for window lifecycle');
}
async function rendererChecks(win) {
    const names = ['active','profileSelectionLocked','profileRequestContext','profileContextCurrent','capture',
        'inspect','refreshProfiles','adoptInspectionInfo','openLibraryEntry','collectProfileEditor',
        'profileHasUnsavedChanges','updateMotionCompletion','updateTargetDraftState','editWorkbenchTarget',
        'fillReplayEndpointDraft','workbenchTargetMayMove','synchronizeWorkbenchSession','connectWorkbench','readinessConnectionState'];
    const targetStart = renderer.indexOf("        document.querySelectorAll('[data-target-joint]')");
    const targetEnd = renderer.indexOf('        setTimeout(() => refreshWorkbenchTargetFields(true)', targetStart);
    assert(targetStart >= 0 && targetEnd > targetStart);
    const result = await win.webContents.executeJavaScript(`(async()=>{
        const checks=[]; const check=(name,value)=>{if(!value)throw Error(name);checks.push(name);};
        let state={config:{profile:'A',profile_file:'',serial_port:'mock'},source:'builtin',session:{state:'idle'},inspecting:0,workbenchEpoch:1};
        let profileLoadGeneration=0, ghostGeneration=0, modelView=null, modelViewUnused=null;
        let pageModelMount=null,workspaceMount=null,lastRuntimeFaultKey='',lastTeachReplayErrorKey='';
        let reportedCalibrationErrors=new Set(),graphicsState={records:new Map(),current:null};
        let api={request:async()=>({})};
        const $=id=>document.getElementById(id), busyText=(zh,en)=>en, t=x=>x;
        const render=()=>{}, save=async()=>{}, statusUI=()=>{}, shell=()=>{}, clearProfileFormState=()=>{};
        const disposeModelPreview=()=>{}, resetProfileWorkspaceState=()=>{profileLoadGeneration++;};
        const refreshWorkbenchTargetFields=()=>{}, queueWorkbenchTargetGhost=()=>{}, jointInputToNative=(_name,x)=>Number(x);
        const commandStateText=x=>x,workbenchTargetFeedbackReady=()=>true,workbenchTargetProblem=()=>'',graphicsAvailabilityProblem=()=>'';
        const workbenchRecoveryRequired=()=>false,modelCalibrationActive=()=>false,gainModeConfigured=()=>true;
        const workbenchConnected=()=>state.session.state==='running'&&state.session.mode==='workbench';
        const refreshTeachTrajectories=async()=>{},loadAdmittance=async()=>{},refreshGeometryRepeatStatus=async()=>{};
        let replayPose=[.3,.4], workbenchRequest=async()=>({motion:{id:99,state:'running'}});
        const replayStartPose=()=>replayPose,updateWorkbenchSpeed=()=>{},blockingOperation=async(_message,fn)=>fn();
        let reads=0;const readProfileInput=input=>{reads++;if(input.value!=='unchanged'){state.profileEditor.core.value=Number(input.value);state.profileDirty=true;}};
        const profileGroupForField=()=>'',setTimeout=()=>{};
        ${names.map(name=>extract(renderer,name)).join('\n')}
        document.body.innerHTML='<select id="profile-select"><option>A</option><option>B</option></select><input id="serial_port" value="foreign"><input id="workbench-speed" value="0.2"><button id="target-sync"></button><button id="target-zero"></button><button id="target-move"></button><input data-target-joint="0"><input data-target-range="0"><span id="target-command-state"></span>';
        for(const sessionState of ['running','stopping']){
            state.session={state:sessionState,mode:'workbench',session_id:'live'};
            $('profile-select').value='B';capture();
            check('connected '+sessionState+' Profile locked',state.config.profile==='A'&&$('profile-select').value==='A'&&state.config.serial_port==='mock');
        }
        state.session={state:'idle'};state.runtimeStarting=true;$('profile-select').value='B';capture();
        check('connecting Profile locked',state.config.profile==='A');state.runtimeStarting=false;
        $('profile-select').value='A';$('serial_port').value='mock';
        let resolveInspect, calls=[];api.request=(method)=>{calls.push(method);return method==='inspect'?new Promise(resolve=>resolveInspect=resolve):Promise.resolve({session:{state:'idle'}});};
        const oldInspect=inspect();await Promise.resolve();
        state.session={state:'running',mode:'workbench',session_id:'new'};state.info={marker:'connected'};
        resolveInspect({marker:'stale'});await oldInspect;
        check('old inspect reply cannot replace connected info',state.info.marker==='connected');
        let profileCalls=0;api.request=async()=>{profileCalls++;return {profiles:[{name:'B'}]};};
        await refreshProfiles();check('connected refresh cannot change Profile',profileCalls===0&&state.config.profile==='A');
        state.library=[{id:'B',profile:'B',source:'builtin'}];let rejected=false;try{await openLibraryEntry('B');}catch{rejected=true;}
        check('direct Library open guarded',rejected&&state.config.profile==='A');
        state.session={state:'idle'};let resolveListing;api.request=()=>new Promise(resolve=>resolveListing=resolve);
        const refresh=refreshProfiles();state.session={state:'running',mode:'workbench',session_id:'next'};state.profiles=[{name:'A'}];resolveListing({profiles:[{name:'B'}]});await refresh;
        check('old catalogue cannot replace connected Profile',state.config.profile==='A'&&state.profiles[0].name==='A');
        state.session={state:'idle'};state.library=[{id:'B',profile:'B',source:'builtin'}];
        const requests=[];api.request=async(method,params)=>{requests.push([method,params]);if(method==='profiles')return {profiles:[{name:'B'}]};if(method==='inspect')return {marker:'B'};return {};};
        await openLibraryEntry('B');
        check('Library internal inspect uses selected Profile rather than old DOM',state.config.profile==='B'&&requests.find(x=>x[0]==='inspect')[1].profile==='B');
        $('profile-select').value='B';state.info={fingerprint:'same'};
        let resolveStart;api.request=(method)=>method==='readiness'?new Promise(resolve=>resolveStart=resolve):Promise.resolve(method==='workbench_start'?{state:'running',mode:'workbench',session_id:'start'}:{});
        const connecting=connectWorkbench(false);check('connection locks before readiness reply',profileSelectionLocked());
        $('profile-select').value='A';capture();check('pending connection retains selected Profile',state.config.profile==='B');resolveStart({});await connecting;
        check('connection leaves session bound and clears starting flag',state.session.session_id==='start'&&!state.runtimeStarting);
        state.telemetry={sequence:1,joint_names:['j1','j2'],joint:{pos:[.1,.2]},robot_state:'ACTIVE',impedance_mode:'RIGID_TRACKING'};
        state.workbenchTargets=[1,2];state.teachTrajectories=[];state.pendingMotionId=7;state.pendingTarget=[.7,.8];state.commandState='running';
        ${renderer.slice(targetStart,targetEnd)}
        $('target-sync').click();check('sync keeps executing motion',state.pendingMotionId===7&&state.pendingTarget[0]===.7&&state.commandState==='running');
        $('target-zero').click();check('zero keeps executing motion',state.pendingMotionId===7&&state.workbenchTargets[0]===0&&state.commandState==='running');
        const input=document.querySelector('[data-target-joint]');input.value='1.5';input.dispatchEvent(new Event('input'));
        check('editing keeps original accepted target',state.pendingMotionId===7&&state.pendingTarget[0]===.7&&state.workbenchTargets[0]===1.5);
        fillReplayEndpointDraft();check('replay endpoint only edits draft',state.pendingMotionId===7&&state.pendingTarget[0]===.7&&state.workbenchTargets[0]===.3);
        check('another move blocked until terminal event',!workbenchTargetMayMove());
        updateMotionCompletion({motion:{id:6,state:'completed'}});check('foreign completion ignored',state.pendingMotionId===7);
        for(const [terminal,expected] of [['completed','complete'],['failed','failed'],['interrupted','interrupted']]){
            state.pendingMotionId=7;state.pendingTarget=[.7,.8];state.commandState='running';editWorkbenchTarget(0,'1.6');
            updateMotionCompletion({motion:{id:7,state:terminal}});
            check('original motion '+terminal+' delivered after editing',state.pendingMotionId===null&&state.commandState===expected&&state.workbenchTargets[0]===1.6);
        }
        state.workbenchIdentity='start';state.pendingMotionId=8;state.pendingTarget=[1,2];state.commandState='running';
        synchronizeWorkbenchSession({state:'exited',mode:'workbench',session_id:'start'});
        check('disconnect marks original motion interrupted',state.commandState==='interrupted'&&state.pendingMotionId===null);
        document.body.innerHTML='<input data-profile-field="core.value" value="3.5">';
        state.page='profile_setup';state.profileEditor={core:{value:1}};state.profileDirty=false;state.profileFieldErrors={};
        check('close flush sees focused unsaved input',profileHasUnsavedChanges()&&reads===1&&state.profileEditor.core.value===3.5);
        const legacy={joint_names:['j1'],core:{model:{gravity:[0,0,-9.81],gravity_scale:{j1:1}},control:{runtime:{write_enabled:true},controller:{rigid_hold:{kp:{j1:10},kd:{j1:1}},compliant_drag:{kd:{j1:1}}}},safety_policy:{max_kp_override:{j1:.01},max_kd_override:{j1:.01}}}};
        check('legacy PD user caps do not affect readiness',readinessConnectionState(legacy).basic_calibration_prerequisites.find(x=>x.id==='hold_gains').ready);
        return checks;
    })().catch(error=>({fixture_error:String(error.stack||error)}))`);
    if (!Array.isArray(result)) throw Error(result.fixture_error || JSON.stringify(result));
    checks.push(...result);
}
let controller;
function createController(win, entry, directory, responses, session, options={}) {
    const log=[];
    const bridge={dead:false,pending:new Map(),session:{...session},
        request:async(method)=>{log.push(method);if(options.stopFails)throw Error('fixture stop failure');bridge.session={...bridge.session,state:'exited'};},
        close:async()=>log.push('bridge.close'),forceClose:async()=>log.push('bridge.forceClose')};
    const appFacade={relaunch:()=>log.push('relaunch'),quit:()=>log.push('quit')};
    const graphics={backendMode:'normal',requestedMode:'normal',compatibilityAvailable:true,
        select(mode){this.requestedMode=mode;log.push('graphics.select');},
        status(){return {requestedMode:this.requestedMode,requiresRestart:this.requestedMode==='compatibility'};}};
    const context=vm.createContext({win,bridge,app:appFacade,graphics,GRAPHICS_MODES:new Set(['normal','compatibility','rescue']),
        path,fs,process,setTimeout,clearTimeout,entry,pathToFileURL,
        dialog:{showMessageBox:async(_window,message)=>{log.push(message.message);assert(responses.length,'Unexpected native dialog');return {response:responses.shift()};},showErrorBox:(_title,message)=>{throw Error(message);}},
        readPrefs:()=>{try{return JSON.parse(fs.readFileSync(path.join(directory,'prefs.json'),'utf8'));}catch{return {graphicsUnsafeSoftware:true};}},
        prefsPath:()=>path.join(directory,'prefs.json'),savePrefs:()=>log.push('bounds.saved')});
    const functions=['trusted','windowDraftState','acceptCloseDraft','waitForSessionExit','closeWindow','canRestartForGraphics','selectGraphicsMode'];
    vm.runInContext('let closing=false,graphicsRestarting=false,runtimeTransitions=0,rendererCloseReady=false,closeRequestSerial=0,closeDraftRequest=null;\n'+functions.map(name=>extract(main,name)).join('\n'),context);
    const nativeClose=main.split('\n').find(line=>line.includes("win.on('close', event =>"));assert(nativeClose);
    vm.runInContext(nativeClose,context);
    const beforeQuit=main.slice(main.indexOf("app.on('before-quit'"),main.indexOf("app.on('window-all-closed'"));
    let quitHandler;context.app.on=(_name,handler)=>{quitHandler=handler;};vm.runInContext(beforeQuit,context);
    return {context,win,bridge,log,graphics,ready:()=>vm.runInContext('rendererCloseReady=true',context),
        draft:(event,result)=>context.acceptCloseDraft(event,result),
        close:()=>context.closeWindow(), graphicsMode:()=>context.selectGraphicsMode('compatibility',{restart:true}),
        quit:()=>{let prevented=false;quitHandler({preventDefault(){prevented=true;}});return prevented;},
        done:()=>vm.runInContext('!closing',context)};
}
async function closeChecks(directory,entry) {
    ipcMain.on('launcher:close-ready',event=>{if(controller?.win.webContents===event.sender)controller.ready();});
    ipcMain.on('launcher:close-draft',(event,result)=>controller.draft(event,result));
    ipcMain.handle('launcher:window',(event,action)=>{controller.context.trusted(event);assert.equal(action,'close');return controller.close();});
    const scenarios=[
        {name:'native close cancel dirty',dirty:true,responses:[1],entry:'native',destroyed:false},
        {name:'titlebar close discard dirty',dirty:true,responses:[0],entry:'ipc',destroyed:true},
        {name:'application quit cancel dirty',dirty:true,responses:[1],entry:'quit',destroyed:false},
        {name:'application quit discard dirty',dirty:true,responses:[0],entry:'quit',destroyed:true},
        {name:'session and draft cancel before stop',dirty:true,responses:[1],running:true,destroyed:false},
        {name:'discard then cancel session retains draft',dirty:true,responses:[0,1],running:true,destroyed:false},
        {name:'discard then stop exits',dirty:true,responses:[0,0],running:true,destroyed:true,stop:true},
        {name:'no draft and no session closes directly',dirty:false,responses:[],destroyed:true},
        {name:'no draft session cancel',dirty:false,responses:[1],running:true,destroyed:false},
        {name:'no draft session stop',dirty:false,responses:[0],running:true,destroyed:true,stop:true},
        {name:'failed stop force-cancel retains draft',dirty:true,responses:[0,0,1],running:true,destroyed:false,stop:true,stopFails:true},
        {name:'graphics cancel preserves mode and draft',dirty:true,responses:[1],entry:'graphics',destroyed:false},
        {name:'connected graphics restart rejected without discard',dirty:true,responses:[],entry:'graphics',running:true,destroyed:false,rejected:true},
        {name:'graphics discard commits after approval',dirty:true,responses:[0],entry:'graphics',destroyed:true},
        {name:'graphics no draft restarts',dirty:false,responses:[],entry:'graphics',destroyed:true},
    ];
    for(const [index,scenario] of scenarios.entries()) {
        const caseDir=path.join(directory,'case'+index);fs.mkdirSync(caseDir);
        const win=new BrowserWindow({show:false,webPreferences:{preload:path.join(root,'desktop/preload.cjs'),contextIsolation:true,nodeIntegration:false,sandbox:true}});
        controller=createController(win,entry,caseDir,scenario.responses.slice(),{state:scenario.running?'running':'idle',mode:'workbench',session_id:'fixture'},scenario);
        const current=controller;
        await win.loadFile(entry);
        await win.webContents.executeJavaScript(`window.fixtureDirty=${scenario.dirty};window.serialArm.onCloseRequest(()=>({dirty:window.fixtureDirty}));true;`);
        await pause(10);
        let result;
        if(scenario.entry==='native') {win.close();await waitUntil(current.done);}
        else if(scenario.entry==='quit') {check(scenario.name+' intercepts native quit',current.quit());await waitUntil(current.done);}
        else if(scenario.entry==='ipc') {
            const reply=win.webContents.executeJavaScript("window.serialArm.window('close')").catch(()=>null);
            await waitUntil(()=>win.isDestroyed() || current.log.length>0 && current.done());
            if(!win.isDestroyed())result=await reply;
        }
        else if(scenario.entry==='graphics') {
            try { result=await current.graphicsMode(); }
            catch(error) {assert(scenario.rejected,error.stack);result={restarting:false,canceled:true};check(scenario.name+' runtime refusal',error.message.includes('Disconnect'));}
        }
        else result=await current.close();
        check(scenario.name,win.isDestroyed()===scenario.destroyed);
        check(scenario.name+' stop permission',current.log.includes('workbench_stop')===!!scenario.stop);
        if(!scenario.destroyed) {
            check(scenario.name+' retains actual renderer draft',await win.webContents.executeJavaScript('window.fixtureDirty')===scenario.dirty);
            check(scenario.name+' leaves backend open',!current.log.includes('bridge.close'));
            if(scenario.running)check(scenario.name+' leaves session running',current.bridge.session.state==='running');
        }
        if(scenario.entry==='graphics') {
            const pref=path.join(caseDir,'prefs.json');
            check(scenario.name+' stages graphics preference',fs.existsSync(pref)===scenario.destroyed);
            check(scenario.name+' stages relaunch',current.log.includes('relaunch')===scenario.destroyed);
            check(scenario.name+' accurate restart reply',result.restarting===scenario.destroyed&&result.canceled===!scenario.destroyed);
            if(scenario.destroyed)check(scenario.name+' commit after backend close',current.log.indexOf('graphics.select')>current.log.indexOf('bridge.close'));
        }
        if(!win.isDestroyed())win.destroy();
    }
}
app.on('window-all-closed',()=>{});
app.whenReady().then(async()=>{
    const directory=fs.mkdtempSync(path.join(os.tmpdir(),'serialarm-session-lifecycle-'));
    const entry=path.join(directory,'index.html');fs.writeFileSync(entry,'<!doctype html><html><body></body></html>');
    try {
        const win=new BrowserWindow({show:false,webPreferences:{contextIsolation:true,nodeIntegration:false,sandbox:true}});
        await win.loadFile(entry);await rendererChecks(win);win.destroy();
        await closeChecks(directory,entry);
        console.log('LAUNCHER_SESSION_LIFECYCLE_PASS '+JSON.stringify({checks:checks.length,details:checks,no_device_backend:true}));
        app.exit(0);
    } finally {fs.rmSync(directory,{recursive:true,force:true});}
}).catch(error=>{console.error(error.stack);app.exit(1);});
