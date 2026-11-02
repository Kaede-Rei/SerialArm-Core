/* Run with Electron, not node. All API requests are isolated mocks; no hardware backend. */
const {app,BrowserWindow,ipcMain}=require('electron');
const fs=require('node:fs'),path=require('node:path'),assert=require('node:assert/strict');
const fixturePath=process.env.SERIALARM_UI_FIXTURE;
if(!fixturePath)throw Error('Set SERIALARM_UI_FIXTURE to an offline model/record/report JSON fixture');
const fixture=JSON.parse(fs.readFileSync(fixturePath,'utf8'));
const output=process.env.SERIALARM_UI_OUTPUT||'/tmp/serialarm-identification-ui-test';fs.mkdirSync(output,{recursive:true});
app.setPath('userData',path.join(output,'electron-profile'));
app.commandLine.appendSwitch('use-angle','swiftshader');app.commandLine.appendSwitch('enable-unsafe-swiftshader');
const calls=[];const allowedMeshes=new Set((fixture.model.model?.mesh_files||[]).map(x=>path.resolve(x)));
ipcMain.handle('identification-test:resource',(_,file)=>{const p=path.resolve(file);assert.ok(allowedMeshes.has(p),'resource outside fixture');return{extension:path.extname(p).toLowerCase(),data:fs.readFileSync(p).toString('base64')};});
ipcMain.handle('identification-test:request',(_,method,params)=>{
 calls.push({method,params});
 if(method==='status')return{session:{state:'idle'}};
 if(method==='logs')return'';
 if(method==='profile_library')return{profiles:[],path:'/tmp/offline-test'};
 if(method==='model_calibration_idim_audit')return fixture.report.audit;
 if(method==='model_calibration_idim')return fixture.report;
 if(method==='model_calibration_idim_export')return{directory:'/tmp/test-only',candidate_urdf:params.report_only?null:'/tmp/test-only/robot.urdf',report:'/tmp/test-only/idim-report.json'};
 throw Error('Unexpected RPC (hardware routes are forbidden): '+method);
});
const wait=ms=>new Promise(r=>setTimeout(r,ms));
app.whenReady().then(async()=>{
 const win=new BrowserWindow({show:false,width:1600,height:1100,webPreferences:{preload:path.join(__dirname,'identification-test-preload.cjs'),contextIsolation:true,sandbox:false,backgroundThrottling:false}});
 const errors=[];win.webContents.on('console-message',(_event,...args)=>{const detail=args.map(String).join(' ');if(detail.includes('Uncaught'))errors.push(detail);});
 try{
  await win.loadFile(path.join(__dirname,'../renderer/index.html'));win.showInactive();await wait(700);
  await win.webContents.executeJavaScript(`state.config.profile='offline-review';state.page='calibration';state.calibrationEntry='dynamics';state.modelData=${JSON.stringify(fixture.model)};state.modelCalibrationOffline=${JSON.stringify(fixture.record)};state.modelCalibrationDirectory=${JSON.stringify(fixture.record.directory)};state.idimReport=${JSON.stringify(fixture.report)};state.idimAudit=state.idimReport.audit;state.idimScope='local';state.idimLocked=[];shell();`);
  let mounted=false;
  for(let i=0;i<80;++i){mounted=await win.webContents.executeJavaScript(`!!modelView?.hasFramedModel && !!document.querySelector('#workbench-viewport canvas')`);if(mounted)break;await wait(100);}
  assert.ok(mounted,'actual Three.js view did not finish loading');await wait(300);
  assert.ok(await win.webContents.executeJavaScript(`Array.from(modelView.linkGroups.values()).some(g=>g.children.some(x=>x.userData.layer==='visual'))`),'No robot visual geometry loaded');
  const interaction=await win.webContents.executeJavaScript(`(()=>{
   window.__idimView=modelView;window.__idimCanvas=document.querySelector('#workbench-viewport canvas');
   modelView.camera.position.set(1.1,-.8,.75);const camera=modelView.camera.position.toArray();
   const input=document.querySelector('[data-idim-locked]'),original=input;
   const field=document.getElementById('idim-validation');field.value='/tmp/my-independent-session';field.dispatchEvent(new Event('input',{bubbles:true}));
   const task=document.querySelector('.workbench-task');task.scrollTop=180;const before=task.scrollTop;
   for(let i=0;i<5;++i)input.click();
   return{viewSame:modelView===window.__idimView,canvasSame:document.querySelector('#workbench-viewport canvas')===window.__idimCanvas,inputSame:input===document.querySelector('[data-idim-locked]'),cameraSame:JSON.stringify(camera)===JSON.stringify(modelView.camera.position.toArray()),scrollSame:task.scrollTop===before,textKept:document.getElementById('idim-validation').value==='/tmp/my-independent-session',selected:state.idimLocked.length,draft:document.getElementById('idim-draft-state').textContent};})()`);
  for(const key of ['viewSame','canvasSame','inputSame','cameraSame','scrollSame','textKept'])assert.equal(interaction[key],true,key);
  assert.equal(interaction.selected,1);assert.ok(interaction.draft);
  await win.webContents.executeJavaScript(`document.getElementById('idim-audit').click()`);await wait(400);
  assert.equal(await win.webContents.executeJavaScript(`modelView===window.__idimView`),true,'audit remounted the model');
  await win.webContents.executeJavaScript(`document.getElementById('idim-run').click()`);await wait(500);
  assert.equal(await win.webContents.executeJavaScript(`modelView===window.__idimView`),true,'results remounted the model');
  const run=calls.find(x=>x.method==='model_calibration_idim');assert.ok(run);assert.equal(run.params.mode,'local');assert.equal(run.params.locked_links.length,1);assert.equal(run.params.validation_directory,'/tmp/my-independent-session');
  assert.equal(await win.webContents.executeJavaScript(`document.getElementById('idim-export').disabled`),true,'unvalidated export enabled');
  await win.webContents.executeJavaScript(`document.getElementById('idim-report-export').click()`);await wait(900);
  assert.equal(calls.find(x=>x.method==='model_calibration_idim_export').params.report_only,true);
  await win.webContents.executeJavaScript(`document.querySelector('.workbench-task').scrollTop=0`);await wait(100);
  fs.writeFileSync(path.join(output,'workflow.png'),(await win.webContents.capturePage()).toPNG());
  await win.webContents.executeJavaScript(`document.getElementById('idim-results').scrollIntoView({block:'start',behavior:'instant'});`);await wait(700);
  const resultPosition=await win.webContents.executeJavaScript(`(()=>{const task=document.querySelector('.workbench-task'),result=document.getElementById('idim-results');task.scrollTop+=result.getBoundingClientRect().top-task.getBoundingClientRect().top;return {top:result.getBoundingClientRect().top,taskTop:task.getBoundingClientRect().top,scroll:task.scrollTop};})()`);
  assert.ok(resultPosition.scroll>300,'result panel did not scroll');await wait(500);
  fs.writeFileSync(path.join(output,'results.png'),(await win.webContents.capturePage()).toPNG());
  assert.equal(errors.length,0,errors.join('\n'));
  fs.writeFileSync(path.join(output,'ui-verification.json'),JSON.stringify({interaction,resultPosition,calls,hardwareAccessed:false,actualThreeView:true},null,2));
  console.log('IDENTIFICATION_REAL_DOM_THREE_CAMERA_SCROLL_INPUT_EXPORT_PASS');win.destroy();app.exit(0);
 }catch(e){console.error(e.stack);win.destroy();app.exit(1);}
});
