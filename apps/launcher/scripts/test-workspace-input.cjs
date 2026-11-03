// Real Chromium input regression: DOM .click() bypasses popover light-dismiss.
const fs = require('node:fs');
const path = require('node:path');
const os = require('node:os');
const assert = require('node:assert/strict');
if (!process.versions.electron) {
    const {spawnSync} = require('node:child_process');
    const env = {...process.env};
    delete env.ELECTRON_RUN_AS_NODE;
    const electron = require('electron');
    const args = [...(process.getuid?.() === 0 ? ['--no-sandbox'] : []), __filename];
    const result = spawnSync(env.DISPLAY ? electron : 'xvfb-run', env.DISPLAY ? args : ['-a', electron, ...args], {env, stdio: 'inherit'});
    if (result.error) throw result.error;
    process.exit(result.status ?? 1);
}
const {app, BrowserWindow, ipcMain} = require('electron');
const root = path.resolve(__dirname, '..');
const output = fs.mkdtempSync(path.join(os.tmpdir(), 'serialarm-workspace-input-'));
const preload = path.join(output, 'preload.cjs');
fs.writeFileSync(preload, `const {contextBridge,ipcRenderer}=require('electron');contextBridge.exposeInMainWorld('serialArm',{
request:(method,params)=>ipcRenderer.invoke('input-test:request',method,params),prefs:async()=>({language:'zh-CN',theme:'light'}),savePrefs:async()=>true,
onEvent:()=>()=>{},window:()=>{},clipboard:{write:async()=>{},read:async()=>''},modelResource:async()=>{throw Error('No external model resources in input regression')}});`);
const names = ['joint1'];
const model = {model: {robot_name: 'Input regression', links: [{name: 'base', visuals: [{geometry: {type: 'box', size: [.2,.1,.15]}}]}], joints: [{name:'joint1',type:'revolute',parent:'base',child:'base',axis:[0,0,1],controlled:true,limit:{lower:-2,upper:2}}]}, native: {config:{base_frame:'base',tool_frame:'base',joint_names:names},info:{joints_count:1,total_mass:1,reduced_total_mass:1,effective_inertias:[]},state:{positions:[0],frames:[{name:'base',position:[0,0,0],quaternion:[0,0,0,1]}]}}};
const calls = [], requestDetails = [], checks = [], rendererErrors = [];
let controlSnapshot = null;
ipcMain.handle('input-test:request', (_event, method, params) => {
    calls.push(method); requestDetails.push({method, params});
    if (method === 'status') return {installed:true,session:{state:'idle'}};
    if (method === 'logs') return '';
    if (method === 'profile_library') return {profiles:[]};
    if (method === 'model_preview') return {...model.native,state:{...model.native.state,positions:params.positions}};
    if (method === 'workbench_request' && params.method === 'set_impedance_mode') return {...controlSnapshot, impedance_mode:params.params.mode, gain_profile:params.params.gain_profile};
    if (method === 'workbench_request' && params.method === 'status') return controlSnapshot;
    throw Error('Hardware/unexpected request blocked by input regression: ' + method);
});
app.setPath('userData', path.join(output, 'profile'));
app.commandLine.appendSwitch('use-angle', 'swiftshader');
app.commandLine.appendSwitch('enable-unsafe-swiftshader');
const wait = ms => new Promise(resolve => setTimeout(resolve, ms));
let window;
const js = source => window.webContents.executeJavaScript(source);
async function pointerClick(selector) {
    // Native input needs a focused test window, and the same workspace now
    // retains its scroll position when changing panels.
    if (!window.isFocused()) {window.focus();window.webContents.focus();await wait(100);}
    const p = await js(`(async()=>{const node=document.querySelector(${JSON.stringify(selector)});if(!node)throw Error('Missing pointer target: '+${JSON.stringify(selector)});node.scrollIntoView({block:'center',behavior:'instant'});await new Promise(resolve=>requestAnimationFrame(()=>requestAnimationFrame(resolve)));const r=node.getBoundingClientRect();return {x:Math.round(r.left+Math.min(20,r.width/2)),y:Math.round(r.top+r.height/2)}})()`);
    window.webContents.sendInputEvent({type:'mouseMove',...p});
    window.webContents.sendInputEvent({type:'mouseDown',button:'left',clickCount:1,...p});
    await wait(20);
    window.webContents.sendInputEvent({type:'mouseUp',button:'left',clickCount:1,...p});
    await wait(100);
}
async function wheel(point, modifiers = [], count = 12, distance = -120) {
    if (!window.isFocused()) {window.focus();window.webContents.focus();await wait(100);}
    window.webContents.sendInputEvent({type:'mouseMove',...point});await wait(100);
    // Chromium input exercises default scrolling and camera handlers without
    // relying on the host window manager's wheel injection.
    if (!window.webContents.debugger.isAttached()) window.webContents.debugger.attach('1.3');
    const flags = modifiers.reduce((bits, key)=>bits | ({control:2,meta:4,shift:8,alt:1}[key] || 0),0);
    // One uninterrupted wheel transaction: no mouse move and no long gap.
    for (let i = 0; i < count; i++) {
        await window.webContents.debugger.sendCommand('Input.dispatchMouseEvent',{type:'mouseWheel',...point,deltaX:0,deltaY:-distance,modifiers:flags});
        await wait(12);
    }
    await wait(200);
}
async function modelReady() {
    for (let i = 0; i < 80; i++) {
        if (await js('!!modelView?.hasFramedModel')) return;
        await wait(50);
    }
    throw Error('Three.js model did not mount');
}
const layout = () => js(`(()=>{const nodes=['.content','.studio-page-body','.workbench-task'];return Object.fromEntries(nodes.map(s=>{const e=document.querySelector(s),r=e.getBoundingClientRect();return[s,{top:e.scrollTop,height:e.clientHeight,scroll:e.scrollHeight,overflow:getComputedStyle(e).overflowY,width:r.width}]}))})()`);
app.whenReady().then(async () => {
    window = new BrowserWindow({show:true,width:1600,height:1000,webPreferences:{preload,contextIsolation:true,sandbox:false,backgroundThrottling:false}});
    window.webContents.on('console-message', (_event, ...args) => {
        const text = args.map(String).join(' ');
        if (text.includes('Uncaught')) rendererErrors.push(text);
    });
    try {
        await window.loadFile(path.join(root, 'renderer/index.html'));
        await wait(450);
        await js(`state.config.profile='input-test';state.page='workbench';state.modelData=${JSON.stringify(model)};state.telemetry={valid:false,joint_names:['joint1'],joint:{pos:[0]},robot_state:'INACTIVE'};state.session={state:'idle'};shell();`);
        await modelReady();
        await js(`const field=document.createElement('label');field.id='input-select-label';field.innerHTML='Input<select id="input-select"><option value="a">A</option><option value="b">B</option></select>';document.querySelector('.workspace-panel').prepend(field);serialArmStudio.enhanceSelects();`);
        const trigger = '#input-select-label .ros-select-trigger';
        await pointerClick(trigger);
        assert.equal(await js(`!!document.querySelector('#input-select-label .ros-select-options:popover-open')`), true, 'first physical click opens');
        await pointerClick(trigger);
        assert.equal(await js(`!!document.querySelector('#input-select-label .ros-select-options:popover-open')`), false, 'second physical click closes');
        await pointerClick(trigger);
        await pointerClick('#input-select-label [data-value="b"]');
        assert.equal(await js(`document.querySelector('#input-select').value`), 'b');
        assert.equal(await js(`document.querySelector(${JSON.stringify(trigger)}).getAttribute('aria-expanded')`), 'false');
        await pointerClick(trigger);
        window.webContents.sendInputEvent({type:'keyDown',keyCode:'Escape'});
        window.webContents.sendInputEvent({type:'keyUp',keyCode:'Escape'});
        await wait(100);
        assert.equal(await js(`!!document.querySelector('#input-select-label .ros-select-options:popover-open')`), false);
        checks.push('physical pointer toggling, selection and Escape');

        assert.deepEqual(await js(`[...document.querySelectorAll('[data-gain-purpose]')].map(e=>e.dataset.gainPurpose)`), ['RIGID','COMPLIANT','DRAG','SERVO_TRACKING']);
        assert.equal(await js(`document.querySelectorAll('[data-impedance-mode]').length`), 0, 'legacy task modes are not gain choices');
        assert.equal(await js(`document.querySelectorAll('[data-reference-behavior]').length`), 2);
        await js(`state.capabilityProfile={core:{capability:{admittance:{enabled:true,reference_behavior:'NATIVE_IMPEDANCE'}}}};state.workbenchTab='replay';state.teachReplayBehavior='NATIVE_IMPEDANCE';render();`);
        await modelReady();
        assert.deepEqual(await js(`[...document.querySelectorAll('#teach-gain-purpose option')].map(e=>e.value)`), ['RIGID','COMPLIANT','SERVO_TRACKING']);
        await js(`document.querySelector('#teach-reference-behavior').scrollIntoView({block:'center',behavior:'instant'})`);
        await pointerClick('#teach-reference-behavior + .ros-select-trigger');
        await pointerClick('[id^="studio-options-"]:popover-open [data-value="MODEL_ADMITTANCE"]');
        await modelReady();
        assert.deepEqual(await js(`[...document.querySelectorAll('#teach-gain-purpose option')].map(e=>e.value)`), ['SERVO_TRACKING']);
        assert.equal(await js(`document.querySelector('#teach-gain-purpose').disabled`), true);
        await js(`state.workbenchTab='tuning';render()`);await modelReady();
        assert.equal(await js(`document.querySelectorAll('[data-reference-behavior],#preview-config,#capability-reference-body').length`), 0, 'interaction tuning does not duplicate operation or monitoring');
        assert.ok(await js(`!!document.querySelector('#response-start')`));
        await js(`state.workbenchTab='monitor';render()`);await modelReady();
        assert.ok(await js(`!!document.querySelector('#capability-reference-body')&&!!document.querySelector('#capability-torque-body')`));
        checks.push('four gain uses, replay strategy choices and distinct tuning/monitoring pages');

        // Actual operation controls use the new protocol without remounting Three.js
        controlSnapshot = {robot_state:'ACTIVE',valid:true,sequence:2,monotonic_ns:2000000000,feedback_age_ms:1,
            impedance_mode:'RIGID_HOLD',gain_profile:'RIGID',gain_source:'CONFIGURED',reference_behavior:'NATIVE_IMPEDANCE',
            model_feedforward_mode:'GRAVITY',joint_names:names,effective_kp:[20],effective_kd:[1],
            joint:{pos:[0],vel:[0],tor:[0],nominal_ref_pos:[0],virtual_ref_pos:[0],accepted_ref_pos:[0]},teach_replay:{phase:'ready',has_trajectory:true},model_calibration:{phase:'idle'}};
        await js(`state.session={state:'running',mode:'workbench',session_id:'input-session',pid:123};state.telemetry=${JSON.stringify(controlSnapshot)};state.admittance={enabled:true,reference_behavior:'NATIVE_IMPEDANCE'};state.readiness={modes:Object.fromEntries(['RIGID_HOLD','RIGID_TRACKING','COMPLIANT_HOLD','COMPLIANT_TRACKING','COMPLIANT_DRAG','SERVO_TRACKING'].map(key=>[key,'configured']))};state.workbenchTab='control';state.page='workbench';render();`);
        await modelReady();
        await js(`window.__inputControlView=modelView;document.querySelector('[data-gain-purpose="SERVO_TRACKING"]').scrollIntoView({block:'center',behavior:'instant'})`);
        await pointerClick('[data-gain-purpose="SERVO_TRACKING"]'); await wait(450);
        const gainRequest = requestDetails.find(item=>item.method==='workbench_request'&&item.params.method==='set_impedance_mode');
        assert.deepEqual(gainRequest.params.params,{mode:'RIGID_HOLD',gain_profile:'SERVO_TRACKING'});
        assert.equal(await js(`modelView===window.__inputControlView`),true,'gain use update keeps the Three.js instance');
        await js(`state.admittance.reference_behavior='MODEL_ADMITTANCE';state.telemetry.reference_behavior='MODEL_ADMITTANCE';state.telemetry.gain_profile='SERVO_TRACKING';updateWorkbenchControlState()`);
        assert.equal(await js(`document.querySelector('[data-gain-purpose="RIGID"]').disabled`),true);
        assert.equal(await js(`document.querySelector('[data-gain-purpose="COMPLIANT"]').disabled`),true);
        assert.equal(await js(`document.querySelector('[data-gain-purpose="SERVO_TRACKING"]').disabled`),false);
        checks.push('physical Native servo selection sends gain_profile; MODEL blocks contradictory gain uses without remounting Three.js');

        await js(`state.admittance.reference_behavior='NATIVE_IMPEDANCE';state.workbenchTab='replay';state.telemetry={...state.telemetry,robot_state:'INACTIVE',valid:false,teach_replay:{phase:'idle'}};render()`);
        await modelReady();
        assert.equal(await js(`document.querySelector('#teach-enable').disabled`),false);
        assert.equal(await js(`document.querySelector('#teach-park').disabled`),true);
        await js(`window.__inputReplayView=modelView;onEvent({event:'telemetry',session_id:'input-session',data:${JSON.stringify(controlSnapshot)}})`);
        assert.equal(await js(`document.querySelector('#teach-enable').disabled`),true);
        assert.equal(await js(`document.querySelector('#teach-park').disabled`),false);
        assert.equal(await js(`modelView===window.__inputReplayView`),true,'telemetry updates operation buttons without remounting');
        checks.push('INACTIVE to ACTIVE telemetry immediately refreshes replay enable and park');

        await js(`state.teachTrajectories=[{id:'input-path',name:'Asymmetric path',samples:20,duration_s:1,start:[0],end:[.6]}];state.teachSelectedId='input-path';state.teachLoadedId='input-path';state.teachDirection='forward';state.teachPreviewDirection=null;refreshTeachReplayPanel();updateStartGhost(true)`);
        await wait(150);
        assert.deepEqual(requestDetails.filter(item=>item.method==='model_preview').at(-1).params.positions,[0]);
        await js(`document.querySelector('#teach-direction').scrollIntoView({block:'center',behavior:'instant'})`);
        await pointerClick('#teach-direction + .ros-select-trigger');
        await pointerClick('[id^="studio-options-"]:popover-open [data-value="reverse"]');await wait(150);
        assert.deepEqual(requestDetails.filter(item=>item.method==='model_preview').at(-1).params.positions,[.6]);
        assert.equal(await js(`modelView===window.__inputReplayView`),true,'reverse updates ghost on the existing view');
        await pointerClick('#teach-direction + .ros-select-trigger');
        await pointerClick('[id^="studio-options-"]:popover-open [data-value="forward"]');await wait(150);
        await pointerClick('#teach-return');await wait(150);
        assert.deepEqual(requestDetails.filter(item=>item.method==='model_preview').at(-1).params.positions,[.6]);
        assert.equal(await js(`modelView===window.__inputReplayView`),true,'return updates ghost without remounting');
        assert.equal(await js(`document.querySelector('#confirm-dialog').open`),true);
        await pointerClick('#teach-confirm-cancel');
        assert.deepEqual(requestDetails.filter(item=>item.method==='model_preview').at(-1).params.positions,[0]);
        checks.push('physical direction and return actions preview the correct endpoint on the same Three.js instance');
        await js(`state.session={state:'idle'};state.telemetry={valid:false,joint_names:['joint1'],joint:{pos:[0]},robot_state:'INACTIVE'};state.admittance=null;state.teachTrajectories=[];state.teachSelectedId='';state.teachLoadedId='';`);

        for (const width of [1600, 800]) for (const page of ['workbench','geometry','calibration']) for (const runtime of [false,true]) {
            window.setSize(width, 1000);
            await js(`state.page=${JSON.stringify(page)};state.geometryTab='repeat';state.calibrationEntry='teaching';state.workbenchTab='control';render();document.querySelector('#runtime-panel').classList.toggle('is-hidden',${!runtime});document.querySelector('.workbench-task').insertAdjacentHTML('beforeend','<div class="input-scroll-tail" style="height:1800px;padding:24px">Scroll regression tail</div>');document.querySelector('.content').scrollTop=0;`);
            await modelReady(); await wait(80);
            const dimensions = await layout();
            assert.ok(dimensions['.content'].scroll > dimensions['.content'].height + 500, 'page has scroll range');
            for (const selector of ['.studio-page-body','.workbench-task']) {
                assert.equal(dimensions[selector].scroll, dimensions[selector].height, selector + ' is not a nested scroller');
            }
            assert.ok(await js(`document.documentElement.scrollWidth<=innerWidth+1`), 'no horizontal page overflow');
            assert.ok(await js(`document.querySelector('#workbench-viewport canvas').clientHeight<=720`), 'canvas is viewport bounded');
            const point = await js(`(()=>{const r=document.querySelector('.workbench-task').getBoundingClientRect();return{x:Math.round(r.left+25),y:Math.round(Math.min(Math.max(r.top+40,150),innerHeight-70))}})()`);
            // Crossing the former inner-panel limit in one gesture must keep moving.
            await wheel(point, [], 20);
            const scrolled = await js(`document.querySelector('.content').scrollTop`);
            assert.ok(scrolled > 780, 'continuous wheel crosses old inner panel boundary');
            assert.equal(await js(`document.querySelector('.workbench-task').scrollTop`), 0);
            assert.equal(await js(`document.querySelector('.studio-page-body').scrollTop`), 0);
            if (page === 'geometry') {
                const saved = await js(`(()=>{document.querySelector('details.geometry-step').open=true;document.querySelector('#geometry-rate').focus({preventScroll:true});document.querySelector('#geometry-rate').value='.27';const before=document.querySelector('.content').scrollTop;refreshGeometryPanel();return before})()`);
                assert.ok(Math.abs((await js(`document.querySelector('.content').scrollTop`)) - saved) <= 1, 'geometry refresh preserves page position');
                assert.equal(await js(`document.querySelector('details.geometry-step').open`), true, 'geometry refresh preserves disclosure');
                assert.equal(await js(`document.activeElement.id`), 'geometry-rate', 'geometry refresh preserves input focus');
                assert.equal(await js(`document.activeElement.value`), '.27', 'geometry refresh preserves in-progress number');
            }
            checks.push(`${width}px ${page} runtime=${runtime} single continuous page scroll`);
        }

        // Exercise actual camera handlers, not just the scrolling CSS.
        window.setSize(1600,1000);
        await js(`state.page='workbench';state.workbenchTab='control';render();document.querySelector('.workbench-task').insertAdjacentHTML('beforeend','<div style="height:2200px"></div>');document.querySelector('.content').scrollTop=0;`);
        await modelReady();await wait(80);
        const point = await js(`(()=>{const r=document.querySelector('#workbench-viewport canvas').getBoundingClientRect();return{x:Math.round(r.left+r.width/2),y:Math.round(Math.min(r.top+r.height/2,innerHeight-100))}})()`);
        const distance = () => js('modelView.camera.position.distanceTo(modelView.controls.target)');
        let before = await distance();
        await wheel(point, [], 8, -80);
        assert.ok((await js(`document.querySelector('.content').scrollTop`)) > 50, 'wheel over workspace model scrolls page');
        assert.ok(Math.abs((await distance()) - before) < 1e-8, 'ordinary wheel does not move workspace camera');
        await wait(600);
        for (const modifier of ['control','meta']) {
            await js(`document.querySelector('.content').scrollTo({top:0,behavior:'instant'})`);await wait(350);
            assert.equal(await js(`document.querySelector('.content').scrollTop`), 0, 'reset before '+modifier);
            before = await distance();
            await wheel(point,[modifier],3,80);await wait(400);
            assert.ok(Math.abs((await distance()) - before) > .01, modifier + ' wheel zooms actual camera');
            assert.ok((await js(`document.querySelector('.content').scrollTop`)) <= 1, modifier + ' zoom does not scroll page');
        }
        await js(`state.page='model';render()`);await modelReady();await wait(80);
        const viewerPoint=await js(`(()=>{const r=document.querySelector('#model-viewport canvas').getBoundingClientRect();return{x:Math.round(r.left+r.width/2),y:Math.round(Math.min(r.top+r.height/2,innerHeight-100))}})()`);
        before=await distance();await wheel(viewerPoint,[],3,80);await wait(400);
        assert.ok(Math.abs((await distance()) - before) > .01,'Model Viewer preserves ordinary wheel zoom');
        checks.push('workspace ordinary wheel scroll, Ctrl/Meta camera zoom, Model Viewer ordinary zoom');
        assert.deepEqual(rendererErrors, [], 'no uncaught renderer errors');
        fs.writeFileSync(path.join(output,'verification.json'),JSON.stringify({checks,calls,requestDetails,rendererErrors,hardwareAccessed:false},null,2));
        console.log('WORKSPACE_REAL_INPUT_PASS', checks.length, output);
        window.destroy();app.exit(0);
    } catch (error) {
        console.error(error.stack);
        fs.writeFileSync(path.join(output,'failure.json'),JSON.stringify({error:String(error),stack:error.stack,checks,calls,rendererErrors},null,2));
        try { fs.writeFileSync(path.join(output,'failure.png'),(await window.webContents.capturePage()).toPNG()); } catch {}
        console.error('Input regression artifacts:',output);
        window.destroy();app.exit(1);
    }
});
