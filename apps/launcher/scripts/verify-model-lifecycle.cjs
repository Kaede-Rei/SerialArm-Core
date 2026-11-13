/* Actual production renderer/Three.js lifecycle with isolated read-only data.
 * DISPLAY=:1 node_modules/.bin/electron --no-sandbox --use-gl=angle \
 * --use-angle=swiftshader --enable-unsafe-swiftshader scripts/verify-model-lifecycle.cjs
 * --collision-geometry uses the same URDF's actual collision STL for visual rendering.
 */
const {app,BrowserWindow}=require('electron');
const fs=require('node:fs'),path=require('node:path'),{spawnSync}=require('node:child_process');
const {buildFixture}=require('./studio-ui-fixture.cjs');
const launcher=path.resolve(__dirname,'..'),repo=path.resolve(launcher,'../..');
const arg=process.argv.indexOf('--out'),output=path.resolve(arg<0?'/tmp/serialarm-model-lifecycle':process.argv[arg+1]);
if(!output.startsWith('/tmp/'))throw Error('Fixture output must be inside /tmp');
const fixture=buildFixture(path.join(output,'fixture'),{model:true});
const dataPath=path.join(fixture.directory,'data.json'),data=JSON.parse(fs.readFileSync(dataPath,'utf8'));
data.library=[{id:'first',profile:data.editor.profile,source:'builtin'},{id:'copy',profile:'fixture_model_copy',source:'builtin'}];
const {developmentBackend}=require('../desktop/native-runtime.cjs'),backend=developmentBackend(repo);
const manifest=JSON.parse(fs.readFileSync(path.join(repo,'.install/manifest.json'),'utf8'));
const positions=data.editor.joint_names.map(()=>.1);
const native=spawnSync(path.join(manifest.install_prefix,'bin/serial_arm_model_probe'),['--config',data.editor.core_path,'--positions',positions.join(',')],{env:backend.env,encoding:'utf8',timeout:10000});
if(native.status!==0)throw Error(native.stderr);
data.actualNative=JSON.parse(native.stdout);
if(process.argv.includes('--collision-geometry'))for(const link of data.model.model.links)link.visuals=structuredClone(link.collisions||[]);
fs.writeFileSync(dataPath,JSON.stringify(data));
function fixtureApi(){
 const {contextBridge}=require('electron'),fs=require('node:fs'),path=require('node:path');
 const data=require('__DATA_PATH__'),calls=[],reads={};let mode='normal',flags={};
 const fail=name=>{throw Error('Read-only lifecycle fixture rejects '+name);};
 contextBridge.exposeInMainWorld('studioUiData',data);
 contextBridge.exposeInMainWorld('serialArm',{
  prefs:async()=>({theme:'light',language:'en'}),savePrefs:async()=>({}),
  request:async(name,arg={})=>{calls.push({name,profile:arg.config?.profile});
   if(name==='status')return {installed:true,session:{state:'idle'}};
   if(name==='logs')return '';if(name==='profile_library')return {profiles:data.library};
   if(name==='profiles')return {profiles:data.library.map(p=>({name:p.profile}))};
   if(name==='inspect')return {...data.info,profile:arg.profile,available:{...data.info.available,model:!flags.unavailable},model_identity:{urdf:data.editor.urdf_path,profile:arg.profile,revision:flags.revision||0}};
   if(name==='readiness')return {state:'configuring',checks:{},evidence:{},control_available:true};
   if(name==='profile_editor_load')return {...structuredClone(data.editor),profile:arg.config.profile};
   if(name==='geometry_fixed_state')return {active:false,suggested_frame:''};
   if(name==='trajectory_list')return {items:[]};
   if(name==='geometry_repeat_status')return {supported:false,reason:'Read-only fixture'};
   if(name==='model'){if(flags.modelError)throw Error('Fixture native model load failure');return {...structuredClone(data.model),profile:arg.config.profile};}
   if(name==='model_preview')return structuredClone(data.model.native);
   if(name==='hardware_inspect')return {plugin:'serial_arm_hardware_damiao',drivers:[]};
   return fail(name);
  },
  modelResource:async(file)=>{const p=path.resolve(file);if(!data.model.mesh_files.includes(p))return fail('modelResource outside fixture');reads[p]=(reads[p]||0)+1;await new Promise(r=>setTimeout(r,12));if(flags.resourceError)throw Error('Fixture mesh read failure');return {path:p,extension:path.extname(p).toLowerCase(),data:fs.readFileSync(p).toString('base64')};},
  graphics:{status:async()=>({effectiveMode:mode,requestedMode:mode,requiresRestart:false,status:'ready'}),report:async()=>({}),setMode:async(value)=>{mode=value;return {};}},
  fixtureStats:async()=>({calls,reads}),fixtureFlags:async(value)=>{flags={...flags,...value};},
  onEvent:()=>{},onCloseRequest:()=>{},window:()=>{},selectProfile:async()=>null,selectProfilePackage:async()=>null,
  revealFile:async()=>{},clipboard:{write:async()=>{},read:async()=>''}
 });
}
fs.writeFileSync(fixture.preload,'('+fixtureApi.toString().replace("'__DATA_PATH__'",JSON.stringify(dataPath))+')()');
app.setPath('userData',path.join(output,'electron-userdata'));
async function verifyLifecycle(){
 window.lifecycleChecks=[];
 const check=(ok,name,evidence)=>{if(!ok)throw Error(name+' '+JSON.stringify(evidence));lifecycleChecks.push({name,...(evidence?{evidence}:{})});};
 const settle=()=>new Promise(r=>setTimeout(()=>requestAnimationFrame(()=>requestAnimationFrame(r)),100));
 const open=async id=>{await blockingOperation('Opening Profile...',()=>openLibraryEntry(id));await settle();};
 const goto=async page=>{await navigate(page);await mountCurrentPreview();await settle();};
 const total=s=>Object.values(s.reads).reduce((a,b)=>a+b,0);
 const promise=blockingOperation('Opening Profile...',()=>openLibraryEntry('first'));
 await new Promise(r=>setTimeout(r,45));
 let value={visible:!document.querySelector('#operation-overlay').hidden,text:document.querySelector('#operation-message').textContent};
 check(value.visible&&/Preloading/.test(value.text),'Opening blocks while real model/resources prepare',value);
 await promise;await settle();
 const firstView=modelView,firstCanvas=modelView.canvas,firstRenderer=modelView.renderer;
 value={page:state.page,loaded:modelView.modelLoaded,paused:modelView.paused,controls:modelView.controls.enabled,hidden:!!modelView.container.closest('[hidden]'),ghost:!!modelView.ghostRoot?.visible,blocking:state.blockingBusy};
 check(value.page==='lifecycle'&&value.loaded&&value.paused&&!value.controls&&value.hidden&&!value.ghost&&!value.blocking,'Preloaded hidden renderer is paused with no invented target',value);
 const initial=await serialArm.fixtureStats(),initialReads=total(initial);
 check(initialReads>0&&Object.values(initial.reads).every(n=>n===1),'Each actual mesh read once during preload',{initialReads,paths:Object.keys(initial.reads).length});
 const hiddenDrawCount=modelView.drawCount;modelView.requestRender();await settle();
 check(modelView.drawCount===hiddenDrawCount&&modelView.animationFrame===null&&modelView.frameTimer===null,'Hidden preview schedules no RAF/timer/draw');
 await goto('model');
 check(modelView===firstView&&modelView.canvas===firstCanvas&&modelView.renderer===firstRenderer&&!modelView.paused&&modelView.controls.enabled&&modelView.drawCount>hiddenDrawCount,'Model attaches and resumes exact prepared renderer/canvas');
 const originalPose=structuredClone(state.modelData.native);modelView.updatePose(originalPose);await settle();
 const idleDraw=modelView.drawCount;
 check(Array.from({length:12},()=>modelView.updatePose(structuredClone(originalPose))).every(changed=>changed===false),'Identical frame values skip redundant pose/COM updates');
 await settle();check(modelView.drawCount===idleDraw,'Identical feedback does not force redraw');
 const comObjects=[...modelView.comGroup.children],changedCom=structuredClone(originalPose);changedCom.state.center_of_mass[0]+=.001;modelView.updatePose(changedCom);
 check(comObjects.every((object,index)=>modelView.comGroup.children[index]===object)&&Math.abs(modelView.totalComVisual.point.position.x-changedCom.state.center_of_mass[0])<1e-12,'Moving COM reuses its geometry and label while updating position');
 const inPlace=structuredClone(changedCom),frame=inPlace.state.frames.find(frame=>modelView.linkGroups.has(frame.name));modelView.updatePose(inPlace);frame.position[0]+=.001;modelView.updatePose(inPlace);
 check(Math.abs(modelView.linkGroups.get(frame.name).matrix.elements[12]-frame.position[0])<1e-12,'In-place frame mutation still updates the measured pose');
 const missingCom=structuredClone(originalPose);delete missingCom.state.center_of_mass;modelView.updatePose(missingCom);
 check(modelView.comGroup.children.length===0,'Missing COM clears the old COM marker');modelView.updatePose(originalPose);
 check(modelView.comGroup.children.length===3,'Restored COM recreates its marker once');
 state.modelLayers.collision=true;modelView.setLayers(state.modelLayers);window.hiddenLink=state.modelData.model.links[0].name;state.modelVisibility.links[hiddenLink]=false;modelView.setObjectVisible('links',hiddenLink,false);modelView.camera.position.multiplyScalar(1.2);
 const savedCamera=modelView.camera.position.toArray();
 await goto('settings');
 check(modelView===firstView&&modelView.paused&&!modelView.controls.enabled,'Non-model page retains paused model');
 await goto('model');
 check(modelView===firstView&&modelView.layers.collision&&modelView.camera.position.toArray().every((v,i)=>Math.abs(v-savedCamera[i])<1e-9),'Model camera and layers restored per preview slot',{camera:modelView.camera.position.toArray(),expected:savedCamera,layers:modelView.layers});
 state.session={state:'running',mode:'workbench'};
 state.telemetry={valid:true,feedback_age_ms:1,sequence:1,robot_state:'INACTIVE',joint_names:studioUiData.editor.joint_names,joint:{pos:studioUiData.actualNative.state.positions,vel:positionsZero(),tor:positionsZero()},frames:studioUiData.actualNative.state.frames};
 state.telemetryReceivedAt=Date.now();window.fixtureFreshness=setInterval(()=>{state.telemetryReceivedAt=Date.now()},50);
 await goto('workbench');
 const pose=native=>{const moved=studioUiData.actualNative.state.frames.find(f=>modelView.linkGroups.has(f.name)&&JSON.stringify(f.position)!==JSON.stringify(studioUiData.model.native.state.frames.find(g=>g.name===f.name)?.position));const f=native.state.frames.find(f=>f.name===moved.name);return {actual:modelView.linkGroups.get(f.name).matrix.elements.slice(12,15),expected:f.position};};
 value=pose(studioUiData.actualNative);
 check(modelView.linkGroups.get(hiddenLink).visible,'Model-only hidden link is visible in workspace');
 check(modelView===firstView&&!modelView.layers.collision&&value.actual.every((v,i)=>Math.abs(v-value.expected[i])<1e-9),'Workspace uses own layers and latest real forward-kinematic pose',value);
 state.workbenchTab='monitor';render();await settle();
 check(modelView===firstView&&!modelView.paused,'Subpage refresh keeps mounted instance');
 await goto('geometry');check(modelView===firstView,'Geometry reuses instance');
 await goto('calibration');check(modelView===firstView,'Calibration reuses instance');
 state.session={state:'idle'};state.telemetry=null;await goto('model');check(!modelView.linkGroups.get(hiddenLink).visible,'Returning to model restores its hidden link');value=pose(state.modelData.native);
 check(value.actual.every((v,i)=>Math.abs(v-value.expected[i])<1e-9),'Model return restores draft pose rather than previous measured pose',value);
 state.config.serial_port='/dev/fixture-only';await inspect({captureInputs:false});
 check(modelView===firstView&&!!state.modelData,'Serial override leaves geometry cached');
 const reused=await serialArm.fixtureStats();
 check(JSON.stringify(reused.reads)===JSON.stringify(initial.reads)&&reused.calls.filter(c=>c.name==='model').length===1,'Main/subpage transitions reuse model data, mesh resources and parsed geometries');
 // A cached canvas must be in the new host before its first paint, not a later task.
 let beforePaint=false;state.page='geometry';render();await new Promise(r=>requestAnimationFrame(()=>{beforePaint=document.querySelector('#workbench-viewport canvas')===firstCanvas;r();}));
 check(beforePaint,'Cached canvas attached before first new-page paint');await settle();
 state.page='model';render();await mountCurrentPreview();await settle();
 await loadModelData();render();await mountCurrentPreview();await settle();
 check(firstView.disposed&&modelView!==firstView&&modelView.modelLoaded,'Explicit reload invalidates renderer and rereads resources');
 const beforeSwitch=modelView;await open('copy');
 check(beforeSwitch.disposed&&modelView!==beforeSwitch&&state.config.profile==='fixture_model_copy'&&modelView.payload.profile===state.config.profile&&modelView.paused,'Profile switch invalidates instance even with shared mesh paths');
 const beforeIdentity=modelView;adoptInspectionInfo({...state.info,model_identity:{...state.modelIdentity,revision:1}});
 check(beforeIdentity.disposed&&modelView===null&&state.modelData===null,'Model identity change invalidates data and renderer');
 await serialArm.fixtureFlags({resourceError:true});await open('first');
 value={page:state.page,profile:state.config.profile,blocking:state.blockingBusy,overlay:document.querySelector('#operation-overlay').hidden,view:modelView===null,fault:[...graphicsState.records.values()].some(r=>r.fault?.category==='model')};
 check(value.page==='lifecycle'&&value.profile===studioUiData.editor.profile&&!value.blocking&&value.overlay&&value.view&&value.fault,'Mesh failure closes blocking UI and keeps current Profile diagnosable',value);
 acknowledgePriorityError();await serialArm.fixtureFlags({resourceError:false,unavailable:true});
 const beforeUnavailable=await serialArm.fixtureStats();await open('copy');
 check(state.page==='lifecycle'&&state.modelData===null&&modelView===null&&!state.blockingBusy,'Incomplete Profile opens without a renderable model');
 const unavailable=await serialArm.fixtureStats();
 check(unavailable.calls.filter(c=>c.name==='model').length===beforeUnavailable.calls.filter(c=>c.name==='model').length,'Unavailable model is not loaded');
 await goto('profile_setup');check(!!state.profileEditor&&!state.modelData,'Incomplete Profile remains editable');
 await serialArm.fixtureFlags({unavailable:false});await changeGraphicsMode('rescue');
 const beforeRescue=await serialArm.fixtureStats();await open('first');
 check(state.modelData!==null&&modelView===null&&graphicsState.effectiveMode==='rescue'&&!state.blockingBusy,'Rescue mode preloads model data without WebGL');
 check(JSON.stringify((await serialArm.fixtureStats()).reads)===JSON.stringify(beforeRescue.reads),'Rescue mode reads no mesh resources');
 await goto('model');await changeGraphicsMode('normal');await mountCurrentPreview();await settle();
 check(modelView?.modelLoaded&&!modelView.paused,'Leaving rescue builds a fresh functioning preview');
 const beforeBackend=modelView;await changeGraphicsMode('compatibility');await mountCurrentPreview();await settle();
 check(beforeBackend.disposed&&modelView!==beforeBackend&&modelView.mode==='compatibility','Graphics backend change invalidates renderer resources');
 await goto('settings');const lostView=modelView;lostView.gl.getExtension('WEBGL_lose_context').loseContext();await settle();
 check(lostView.disposed&&modelView===null,'Context loss while hidden disposes the retained instance');
 await goto('geometry');check(modelView!==lostView&&modelView?.modelLoaded,'A different preview slot cannot attach the lost context');
 state.session={state:'idle'};await serialArm.fixtureFlags({modelError:true});await open('copy');
 check(state.page==='lifecycle'&&state.config.profile==='fixture_model_copy'&&state.modelData===null&&modelView===null&&!state.blockingBusy,'Native model failure keeps the new Profile and clears old geometry');
 acknowledgePriorityError();await serialArm.fixtureFlags({modelError:false});await open('first');
 check(modelView?.modelLoaded&&modelView.paused,'Failed loading can be retried normally');
 await goto('diagnostics');const previousLegacy=modelView,legacySelect=document.getElementById('profile-select');
 legacySelect.value='fixture_model_copy';await legacySelect.onchange();await settle();
 check(previousLegacy.disposed&&state.config.profile==='fixture_model_copy'&&modelView?.payload.profile===state.config.profile&&modelView.paused,'Legacy Profile selector also preloads the selected model');
 const beforeRefresh=modelView,beforeRefreshStats=await serialArm.fixtureStats();await refreshProfiles();await settle();
 check(modelView===beforeRefresh&&JSON.stringify((await serialArm.fixtureStats()).reads)===JSON.stringify(beforeRefreshStats.reads),'Legacy Profile refresh reuses unchanged preloaded geometry');
 const stats=await serialArm.fixtureStats(),allowed=['status','logs','profile_library','profiles','inspect','readiness','model','model_preview','profile_editor_load','hardware_inspect','geometry_fixed_state','trajectory_list','geometry_repeat_status'];
 check(stats.calls.every(c=>allowed.includes(c.name)),'Fixture makes only isolated read-only requests, no device command',stats.calls.filter(c=>!allowed.includes(c.name)));
 clearInterval(fixtureFreshness);
 return {checks:lifecycleChecks,initialReads,stats,graphics:'Actual Three.js, explicitly selected Electron SwiftShader'};
 function positionsZero(){return studioUiData.editor.joint_names.map(()=>0);}
}
app.whenReady().then(async()=>{
 const win=new BrowserWindow({width:1200,height:900,show:true,webPreferences:{preload:fixture.preload,sandbox:false,backgroundThrottling:false}});
 const deadline=setTimeout(()=>app.exit(1),30000);
 win.webContents.on('render-process-gone',(_event,details)=>{fs.writeFileSync(path.join(output,'renderer-crash.json'),JSON.stringify(details,null,2));console.error('MODEL_LIFECYCLE_RENDERER_GONE',details);app.exit(1);});
 win.webContents.on('console-message',(_event,level,message)=>{if(level>=3)console.error('MODEL_LIFECYCLE_CONSOLE',message);});
 try {
  await win.loadFile(fixture.index);
  await win.webContents.executeJavaScript('document.fonts.ready.then(()=>new Promise(r=>setTimeout(r,100)))');
  const report=await win.webContents.executeJavaScript('('+verifyLifecycle.toString()+')()');
  fs.writeFileSync(path.join(output,'model-lifecycle.png'),(await win.webContents.capturePage()).toPNG());
  await win.webContents.executeJavaScript('disposeModelPreview();void 0');
  report.geometrySource=process.argv.includes('--collision-geometry')?'Actual URDF collision STL rendered as visual; real native forward kinematics':'Actual URDF visual GLB and collision STL';
  fs.writeFileSync(path.join(output,'report.json'),JSON.stringify(report,null,2));
  console.log('MODEL_LIFECYCLE_PASS '+JSON.stringify({checks:report.checks.length,initialReads:report.initialReads,report:path.join(output,'report.json')}));
 } catch(error) {
  fs.writeFileSync(path.join(output,'failure.json'),JSON.stringify({error:error.stack,checks:await win.webContents.executeJavaScript('window.lifecycleChecks||[]')},null,2));throw error;
 } finally {clearTimeout(deadline);if(!win.isDestroyed())win.destroy();app.quit();}
}).catch(error=>{console.error(error.stack);app.exit(1);});
