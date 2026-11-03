// Offline Electron/Three regression for task-only updates and camera lifetime.
const fs = require('node:fs'), path = require('node:path'), os = require('node:os'), assert = require('node:assert/strict');
if (!process.versions.electron) {
  const {spawnSync} = require('node:child_process'), electron = require('electron');
  const env = {...process.env}; delete env.ELECTRON_RUN_AS_NODE;
  const args = [...(process.getuid?.() === 0 ? ['--no-sandbox'] : []), __filename];
  const result = spawnSync(env.DISPLAY ? electron : 'xvfb-run', env.DISPLAY ? args : ['-a', electron, ...args], {env, stdio:'inherit'});
  if (result.error) throw result.error;
  process.exit(result.status ?? 1);
}
const {app, BrowserWindow, ipcMain} = require('electron');
const root = path.resolve(__dirname, '../../..');
const model = JSON.parse(fs.readFileSync(process.env.SERIALARM_MODEL_FIXTURE || '/tmp/serialarm-ui-unification/model.json', 'utf8'));
const editor = JSON.parse(fs.readFileSync(process.env.SERIALARM_EDITOR_FIXTURE || '/tmp/serialarm-motion-ui/editor.json', 'utf8'));
const output = fs.mkdtempSync(path.join(os.tmpdir(), 'serialarm-workspace-views-'));
const calls = [], errors = [], checks = [];
const allowed = new Set((model.mesh_files || model.model.mesh_files || []).map(file=>path.resolve(file)));
let resourceGate = null, previewGate = null;
ipcMain.handle('identification-test:resource', async (_event, file)=>{
  assert.ok(allowed.has(path.resolve(file)), 'resource outside fixture');
  const gate = resourceGate;
  if (gate) { resourceGate = null; gate.used = true; await gate.promise; }
  return {extension:path.extname(file),data:fs.readFileSync(file).toString('base64')};
});
ipcMain.handle('identification-test:request', async (_event, method, params)=>{
  calls.push({method,params});
  if (method==='status') return {installed:true,session:{state:'idle'}};
  if (method==='logs') return '';
  if (method==='profile_library') return {profiles:[]};
  if (method==='trajectory_list') return {items:[]};
  if (method==='profile_editor_load') return editor;
  if (method==='readiness') return {control_available:true,modes:{},checks:{}};
  if (method==='inspect') return {profile:'offline-test',available:{},checks:[]};
  if (method==='model_preview') {
    const gate=previewGate; if(gate){previewGate=null;gate.used=true;await gate.promise;}
    return {...model.native,state:{...model.native.state,positions:params.positions}};
  }
  throw Error('Unexpected RPC; hardware is forbidden: '+method);
});
app.setPath('userData', path.join(output,'profile'));
app.commandLine.appendSwitch('use-angle','swiftshader'); app.commandLine.appendSwitch('enable-unsafe-swiftshader');
const delay=ms=>new Promise(resolve=>setTimeout(resolve,ms));
let win; const js=source=>win.webContents.executeJavaScript(source);
async function mounted() {for(let i=0;i<80;i++){if(await js(`!!modelView?.hasFramedModel`))return;await delay(100);}throw Error('model mount did not finish');}
async function remember() {
  await delay(350);
  await js(`window.viewSaved=modelView;window.canvasSaved=modelView.renderer.domElement;window.cameraSaved=modelView.camera;window.controlsSaved=modelView.controls;window.poseSaved={position:modelView.camera.position.toArray(),target:modelView.controls.target.toArray(),zoom:modelView.camera.zoom};window.hostSaved=document.getElementById('workbench-viewport');`);
}
async function same(label) {
  await delay(180);
  const value=await js(`({view:modelView===window.viewSaved,canvas:modelView.renderer.domElement===window.canvasSaved,camera:modelView.camera===window.cameraSaved,controls:modelView.controls===window.controlsSaved,host:document.getElementById('workbench-viewport')===window.hostSaved,pose:modelView.camera.position.toArray().every((value,index)=>Math.abs(value-window.poseSaved.position[index])<1e-8)&&modelView.controls.target.toArray().every((value,index)=>Math.abs(value-window.poseSaved.target[index])<1e-8)&&modelView.camera.zoom===window.poseSaved.zoom})`);
  for(const [key,result] of Object.entries(value))assert.equal(result,true,label+': '+key);
  checks.push(label);
}
app.whenReady().then(async()=>{
 win=new BrowserWindow({show:true,width:1600,height:1080,webPreferences:{preload:path.join(__dirname,'identification-test-preload.cjs'),contextIsolation:true,sandbox:false,backgroundThrottling:false}});
 win.webContents.on('console-message',(_event,...args)=>{const text=args.join(' ');if(text.includes('Uncaught'))errors.push(text);});
 try {
  await win.loadFile(path.join(root,'apps/launcher/renderer/index.html')); await delay(400);
  await js(`state.config.profile='offline-test';state.page='calibration';state.calibrationEntry='teaching';state.modelData=${JSON.stringify(model)};state.capabilityProfile=${JSON.stringify(editor)};shell()`);
  await mounted(); await remember();
  for(const entry of ['gain','gravity','gravity_verify','dynamics','teaching']) {
   await js(`document.querySelector('[data-calibration-entry="${entry}"]').click()`);
   assert.equal(await js('state.calibrationEntry'),entry); await same('calibration click '+entry);
  }
  await js(`openSetupTarget({page:'calibration',tool:'gain'})`); await same('calibration same-page deep target');
  await js(`render()`); await same('calibration state refresh');
  // A gravity candidate belongs only to gravity/model tools, never to teaching or gain controls.
  const inertial=model.model.links.find(link=>link.inertial?.mass>0), mass=inertial.inertial.mass, com=inertial.inertial.origin.xyz;
  const candidate={task_id:'fixture',phase:'complete',workflow_kind:'gravity',urdf_fingerprint:model.native.config.urdf_fingerprint,joint_names:model.native.config.joint_names,gravity_result:{static_pass:false,first_moments:[{link_name:inertial.name,value:com.map((value,index)=>(value+(index===0?.01:0))*mass)}]}};
  await js(`state.modelCalibrationOffline={directory:'/tmp/fixture',metadata:{urdf_fingerprint:${JSON.stringify(model.native.config.urdf_fingerprint)}},result:${JSON.stringify(candidate)}};openSetupTarget({page:'calibration',tool:'gravity'})`);
  await same('gravity candidate refresh');
  assert.ok(await js(`modelView.scene.children.length>0 && state.modelCalibrationComparisonKey.length>0`));
  await js(`openSetupTarget({page:'calibration',tool:'teaching'})`); await same('teaching clears gravity comparison');
  assert.equal(await js(`state.modelCalibrationComparisonKey`),'');
  assert.equal(await js(`!!document.querySelector('.calibration-com-overlay')`),false);
  fs.writeFileSync(path.join(output,'calibration.png'),(await win.webContents.capturePage()).toPNG());
  await js(`state.modelCalibrationOffline=null;state.page='workbench';state.workbenchTab='control';render()`);
  await mounted(); await remember();
  for(const tab of ['replay','tuning','monitor','control']) {
   await js(`document.querySelector('[data-workbench-tab="${tab}"]').click()`);
   assert.equal(await js('state.workbenchTab'),tab); await same('workbench click '+tab);
   assert.equal(await js(`!!document.querySelector('.workbench-task [data-workbench-tab]')`),true);
  }
  await js(`openSetupTarget({page:'workbench',tab:'monitor'})`); await same('workbench same-page deep target');
  await js(`state.workbenchTab='replay';render();refreshTeachReplayPanel()`); await same('replay status refresh keeps right navigation');
  assert.equal(await js(`document.querySelectorAll('.workbench-task [data-workbench-tab]').length`),4);
  await js(`state.workbenchTab='control';render()`); await same('control after replay refresh');
  fs.writeFileSync(path.join(output,'workbench.png'),(await win.webContents.capturePage()).toPNG());
  await js(`state.page='model';render()`);await delay(200);
  assert.equal(await js(`!!document.getElementById('model-reset')`),false);
  await mounted();
  for (const page of ['model','description']) {
   let release;
   const gate={used:false,promise:new Promise(resolve=>{release=resolve})}; resourceGate=gate;
   const analysis={preview:model,links:model.model.links.map(link=>link.name),joints:model.model.joints,roots:[model.native.config.base_frame],urdf:model.urdf,package_name:'offline-test'};
   await js(`state.page=${JSON.stringify(page)};state.descriptionAnalysis=${JSON.stringify(analysis)};state.descriptionStep=2;state.modelLayers={visual:true,collision:false,linkFrames:true,jointAxes:true,com:true,inertia:true,labels:true};render();window.obsoleteMount=${page==='model'?'mountModelView()':'mountDescriptionView()'};void 0`);
   for(let i=0;i<80&&!gate.used;i++)await delay(25);
   assert.ok(gate.used,page+': delayed old mesh request started');
   await js(`state.page='workbench';state.workbenchTab='control';render()`);await mounted();await remember();
   const layers=await js(`JSON.stringify(modelView.layers)`);
   release();await js(`window.obsoleteMount`);await same(page+' late mount keeps workspace preview');
   assert.equal(await js(`JSON.stringify(modelView.layers)`),layers,page+': obsolete layers did not overwrite workspace');
  }
  let releasePreview;
  const pendingPreview={used:false,promise:new Promise(resolve=>{releasePreview=resolve})}; previewGate=pendingPreview;
  await js(`state.page='model';render();window.previewPayload=state.modelData.native;window.obsoletePreview=previewModel();void 0`);
  for(let i=0;i<80&&!pendingPreview.used;i++)await delay(25);
  assert.ok(pendingPreview.used,'delayed joint preview request started');
  await js(`state.page='workbench';state.workbenchTab='control';render()`);await mounted();await remember();
  await js(`window.poseUpdates=0;const update=modelView.updatePose.bind(modelView);modelView.updatePose=(...args)=>{window.poseUpdates++;return update(...args)};void 0`);
  releasePreview();await js(`window.obsoletePreview`);await same('late joint preview keeps workspace pose');
  assert.equal(await js(`state.modelData.native===window.previewPayload && window.poseUpdates===0`),true);
  assert.deepEqual(errors,[]);
  fs.writeFileSync(path.join(output,'verification.json'),JSON.stringify({checks,calls,errors,actualThreeView:true,hardwareAccessed:false},null,2));
  console.log('WORKSPACE_VIEW_IDENTITY_PASS',checks.length,output);win.destroy();app.exit(0);
 } catch(error) {
  console.error(error.stack);fs.writeFileSync(path.join(output,'failure.png'),(await win.webContents.capturePage()).toPNG());console.error(output);win.destroy();app.exit(1);
 }
});
