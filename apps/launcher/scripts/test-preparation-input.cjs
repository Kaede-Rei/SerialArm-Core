// Real Chromium flow: preparation → exact field → validation → save → return.
const fs = require('node:fs');
const path = require('node:path');
const os = require('node:os');
const assert = require('node:assert/strict');
const {spawnSync, execFileSync} = require('node:child_process');
if (!process.versions.electron) {
  const env = {...process.env}; delete env.ELECTRON_RUN_AS_NODE;
  const electron = require('electron');
  const args = [...(process.getuid?.() === 0 ? ['--no-sandbox'] : []), __filename];
  const result = spawnSync(env.DISPLAY ? electron : 'xvfb-run', env.DISPLAY ? args : ['-a',electron,...args], {env,stdio:'inherit'});
  if (result.error) throw result.error;
  process.exit(result.status ?? 1);
}
const {app,BrowserWindow,ipcMain} = require('electron');
const root = path.resolve(__dirname,'../../..'), launcher = path.join(root,'apps/launcher');
const output = fs.mkdtempSync(path.join(os.tmpdir(),'serialarm-preparation-input-'));
const python = fs.existsSync(path.join(root,'.install/gui-venv/bin/python')) ? path.join(root,'.install/gui-venv/bin/python') : 'python3';
const editor = JSON.parse(execFileSync(python,['-B','-c',`import sys,json;from pathlib import Path;root=Path(${JSON.stringify(root)});sys.path.insert(0,str(root/'apps/launcher/backend'));from profiles import Inspector;from profile_library import ProfileLibrary;print(json.dumps(ProfileLibrary(root,Inspector(root)).profile_editor_load({'profile':'dm_arm_gray'})))`],{encoding:'utf8',env:{...process.env,PYTHONDONTWRITEBYTECODE:'1',PYTHONNOUSERSITE:'1',XDG_DATA_HOME:output}}));
const readiness = {state:'ready',control_available:true,checks:{profile_setup:'verified',description:'verified'},gain_purposes:Object.fromEntries(['RIGID','COMPLIANT','DRAG','SERVO_TRACKING'].map(key=>[key,{configured:true,evidence_status:'pending'}])),setup_issues:[]};
const calls=[], checks=[], rendererErrors=[];
const preload=path.join(output,'preload.cjs');
fs.writeFileSync(preload,`const {contextBridge,ipcRenderer}=require('electron');contextBridge.exposeInMainWorld('serialArm',{request:(method,params)=>ipcRenderer.invoke('test:request',method,params),prefs:async()=>({language:'zh-CN',theme:'light'}),savePrefs:async()=>true,onEvent:()=>()=>{},window:()=>{},clipboard:{write:async()=>{},read:async()=>''},selectGainCandidate:()=>ipcRenderer.invoke('test:candidate'),revealFile:file=>ipcRenderer.invoke('test:reveal',file)});`);
let saves=0,reveals=0;
ipcMain.handle('test:candidate',()=>'/tmp/gain-candidate.json');
ipcMain.handle('test:reveal',(_event,file)=>{assert.equal(file,editor.urdf_path);reveals++;return true;});
ipcMain.handle('test:request',(_event,method,params)=>{
  calls.push({method,params});
  if (method==='status') return {installed:true,session:{state:'idle'}};
  if (method==='logs') return '';
  if (method==='profile_library') return {profiles:[{id:'test',profile:'dm_arm_gray',source:'builtin',readiness}]};
  if (method==='profile_editor_load') return editor;
  if (method==='readiness' || method==='readiness_approve_mode') return readiness;
  if (method==='inspect') return {profile:'dm_arm_gray',checks:[],available:{},resources:{},hardware_plugin:'serial_arm_hardware_damiao'};
  if (method==='profile_editor_save') {saves++; assert.equal(params.payload.core.safety_policy.max_acc.joint1,1.37);return {...editor,core:params.payload.core};}
  if (method==='geometry_fixed_state') return {};
  if (method==='trajectory_list') return {items:[]};
  if (method==='calibration_tool_list') return {items:[]};
  throw Error('Unexpected request (hardware forbidden): '+method);
});
app.setPath('userData',path.join(output,'profile'));
app.commandLine.appendSwitch('use-angle','swiftshader');
app.commandLine.appendSwitch('enable-unsafe-swiftshader');
const delay=ms=>new Promise(resolve=>setTimeout(resolve,ms));
let win;
const js=source=>win.webContents.executeJavaScript(source);
const field=(name)=>`[data-profile-field='${JSON.stringify(name.split('.'))}']`;
async function click(selector) {
  await js(`(()=>{const node=document.querySelector(${JSON.stringify(selector)});if(!node)throw Error('Missing click target');node.scrollIntoView({block:'center'});node.click()})()`);
  await delay(420);
}
app.whenReady().then(async()=>{
  win=new BrowserWindow({show:true,width:1600,height:1000,webPreferences:{preload,contextIsolation:true,sandbox:false,backgroundThrottling:false}});
  win.webContents.on('console-message',(_e,...args)=>{const message=args.map(String).join(' ');if(message.includes('Uncaught'))rendererErrors.push(message);});
  try {
    await win.loadFile(path.join(launcher,'renderer/index.html'));await delay(450);
    assert.equal(await js(`document.querySelectorAll('[data-library-row]').length`),1,'home shows existing robot');
    await js(`state.config.profile='dm_arm_gray';state.readiness=${JSON.stringify(readiness)};state.page='lifecycle';shell();document.querySelector('.content').scrollTop=230;`);
    assert.equal(await js(`document.querySelectorAll('.mode-admission-row').length`),4,'four uses rather than six task enums');
    const labels=await js(`Array.from(document.querySelectorAll('.readiness-section h3')).map(e=>e.textContent)`);assert.deepEqual(labels,['连接前配置','运行检查','基础标定','专项标定']);
    const basics=await js(`Array.from(document.querySelectorAll('.readiness-section')[2].querySelectorAll('.lifecycle-step b')).map(e=>e.textContent)`);assert.deepEqual(basics,['重力补偿验证','执行器增益自动标定']);
    assert.equal(await js(`document.querySelectorAll('.readiness-section')[3].querySelectorAll('.lifecycle-step').length`),3,'all specialist tools are cards');
    await js(`openSetupTarget({page:'profile_setup',group:'safety',field:'core.safety_policy.max_acc.joint1'})`);await delay(160);
    assert.equal(await js('state.profileGroup'),'safety');
    assert.equal(await js('JSON.parse(document.activeElement.dataset.profileField).join(".")'),'core.safety_policy.max_acc.joint1');
    await js(`(()=>{const f=document.querySelector(${JSON.stringify(field('core.safety_policy.max_acc.joint1'))});f.value='';f.dispatchEvent(new Event('input',{bubbles:true}))})()`);
    await click('#profile-save');assert.equal(saves,0,'blank visible field cannot save old draft');
    assert.equal(await js('state.profileDirty'),true);
    await click('[data-profile-group="motion"]');
    await js(`(()=>{const f=document.querySelector('[data-profile-field][type="checkbox"]');f.checked=!f.checked;f.dispatchEvent(new Event('change',{bubbles:true}))})()`);
    assert.equal(await js(`Object.keys(state.profileFieldErrors).length`),1,'editing another field preserves invalid acceleration');
    await click('#profile-save');assert.equal(saves,0);
    assert.equal(await js('state.profileGroup'),'safety','save opens group containing first error');
    assert.equal(await js(`document.querySelector(${JSON.stringify(field('core.safety_policy.max_acc.joint1'))}).value`),'','invalid raw input survives tab change');
    await js(`(()=>{const f=document.querySelector(${JSON.stringify(field('core.safety_policy.max_acc.joint1'))});f.value='1.37';f.dispatchEvent(new Event('input',{bubbles:true}))})()`);
    await click('#profile-save');assert.equal(saves,1,'corrected visible value saves');
    await click('#profile-return-ready');assert.equal(await js('state.page'),'lifecycle');
    checks.push('invalid input blocks save, persists across groups, fixes before save and returns');
    await js(`openSetupTarget({page:'profile_setup',group:'motion',purpose:'SERVO_TRACKING'})`);await delay(160);
    assert.equal(await js(`document.querySelector('.profile-gain-purpose[data-gain-purpose="SERVO_TRACKING"]').open`),true,'deep purpose expands detail '+JSON.stringify(await js(`({page:state.page,group:state.profileGroup,focus:state.setupFocus,active:document.activeElement.outerHTML.slice(0,300),targets:[...document.querySelectorAll('[data-gain-purpose]')].map(e=>({tag:e.tagName,value:e.dataset.gainPurpose,open:e.open}))})`)));
    await js(`openSetupTarget({page:'profile_setup',group:'mapping',field:'core.calibration.joints.joint2.direction'})`);await delay(100);
    assert.equal(await js(`JSON.parse(document.activeElement.dataset.profileField).join('.')`),'core.calibration.joints.joint2.direction');
    await js(`openSetupTarget({page:'profile_setup',group:'hardware',field:'hardware.damiao.actuators.joint3.motor_type'})`);await delay(100);
    assert.equal(await js(`JSON.parse(document.querySelector('.setup-highlight [data-profile-field]').dataset.profileField).join('.')`),'hardware.damiao.actuators.joint3.motor_type');
    assert.equal(await js(`document.activeElement.classList.contains('ros-select-trigger')`),true,'focus visible select trigger');
    await js(`openSetupTarget({page:'profile_setup',group:'model',field:'core.model.tool_frame'})`);await delay(100);
    await click('[data-urdf-source]');assert.equal(reveals,1);
    checks.push('exact safety, gain-use, mapping, motor and source destinations');
    for (const [page,tool,key] of [['calibration','gravity_verify','calibrationEntry'],['calibration','gain','calibrationEntry'],['calibration','dynamics','calibrationEntry'],['geometry','fixed','geometryTab']]) {
      await js(`openSetupTarget(${JSON.stringify({page,tool})})`);await delay(160);
      assert.equal(await js(`state.${key}`),tool);assert.equal(await js('state.page'),page);
    }
    checks.push('calibration links select exact gravity, gain, dynamics and geometry tool');
    await js(`state.page='lifecycle';render();Array.from(document.querySelectorAll('[data-setup-target]')).find(e=>JSON.parse(e.dataset.setupTarget).anchor==='robot-operation').click()`);await delay(450);
    assert.equal(await js('state.page'),'workbench');assert.equal(await js('state.workbenchTab'),'control');assert.equal(await js(`document.querySelectorAll('.setup-highlight').length`),0,'generic navigation has no highlight');
    checks.push('motion and parking opens control actions without stray anchor outline');
    await js(`state.page='lifecycle';render()`);await click('#approve-gain-mode');
    assert.equal(calls.find(call=>call.method==='readiness_approve_mode').params.mode,'all');
    checks.push('file picker imports candidate uses without legacy enum selection');
    for (const language of ['zh-CN','en']) for(const width of [1600,800]) {
      win.setSize(width,1000);
      await js(`state.language=${JSON.stringify(language)};state.page='profile_setup';state.profileGroup='hardware';render();document.querySelector('.content').scrollTop=0;document.querySelector('#toasts').replaceChildren()`);await delay(500);
      assert.equal(await js('state.page'),'profile_setup');assert.equal(await js(`!!document.querySelector('.parameter-layout')`),true);
      assert.ok(await js(`document.documentElement.scrollWidth<=innerWidth+1`),'no outer horizontal overflow');
      if (language==='en') assert.doesNotMatch(await js(`document.querySelector('.parameter-card').innerText`),/[\u3400-\u9fff]/,'English parameter labels');
      fs.writeFileSync(path.join(output,`${language}-${width}-configuration.png`),(await win.webContents.capturePage()).toPNG());
    }
    await js(`state.language='zh-CN';state.page='lifecycle';render();document.querySelector('.content').scrollTop=0;document.querySelector('#toasts').replaceChildren()`);await delay(500);
    fs.writeFileSync(path.join(output,'preparation.png'),(await win.webContents.capturePage()).toPNG());
    await js(`state.page='profile_setup';state.profileGroup='hardware';delete state.profileEditor.hardware.damiao.actuators.joint2;state.profileEditor.hardware.damiao.actuators.joint1.test_extension=3;render()`);
    await click('[data-profile-complete="hardware"]');
    assert.equal(await js(`state.profileEditor.hardware.damiao.actuators.joint2.motor_id`),0,'missing motor ID remains unconfigured');
    assert.equal(await js(`!!document.querySelector(${JSON.stringify(field('hardware.damiao.actuators.joint2.motor_id'))})`),true,'missing actuator becomes editable');
    assert.equal(await js(`!!document.querySelector(${JSON.stringify(field('hardware.damiao.actuators.joint1.test_extension'))})`),true,'driver extension remains editable');
    await js(`state.profileGroup='mapping';delete state.profileEditor.core.calibration.joints.joint3.direction;render()`);
    await click('[data-profile-complete="mapping"]');
    assert.equal(await js(`!!document.querySelector(${JSON.stringify(field('core.calibration.joints.joint3.direction'))})`),true,'missing mapping becomes editable');
    await js(`state.profileGroup='safety';state.profileEditor.core.safety_policy.max_acc.joint2=null;render();const f=document.querySelector(${JSON.stringify(field('core.safety_policy.max_acc.joint2'))});f.value='1.5';f.dispatchEvent(new Event('input',{bubbles:true}));`);
    assert.equal(await js(`state.profileEditor.core.safety_policy.max_acc.joint2`),1.5,'null numeric YAML can be corrected');
    checks.push('incomplete actuator and mapping fields can be filled, extensions retained, null values repaired');
    await js(`state.modelCalibrationOffline={task_id:'robot-A'};state.modelCalibrationDirectory='/robot-A';state.idimReport={old:true};state.setupReturn={page:'calibration'};state.profileSavedForSession=true;resetProfileWorkspaceState();`);
    assert.equal(await js(`state.modelCalibrationOffline===null && state.modelCalibrationDirectory==='' && state.idimReport===null && state.setupReturn===null && state.profileSavedForSession===false`),true,'robot switch clears task and configuration context');
    checks.push('switching robots clears previous candidates and navigation state');
    assert.deepEqual(rendererErrors,[]);
    fs.writeFileSync(path.join(output,'verification.json'),JSON.stringify({checks,calls,saves,reveals,rendererErrors,hardwareAccessed:false},null,2));
    console.log('PREPARATION_REAL_INPUT_PASS',checks.length,output);win.destroy();app.exit(0);
  } catch(error) {
    console.error(error.stack);fs.writeFileSync(path.join(output,'failure.png'),(await win.webContents.capturePage()).toPNG());
    fs.writeFileSync(path.join(output,'failure.json'),JSON.stringify({error:String(error),checks,calls,rendererErrors},null,2));
    console.error(output);win.destroy();app.exit(1);
  }
});
