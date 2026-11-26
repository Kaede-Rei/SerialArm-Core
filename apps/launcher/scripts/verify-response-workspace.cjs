/* Real response receipts and renderer; fixture transport rejects hardware calls. */
const {app,BrowserWindow}=require('electron');
const fs=require('node:fs'),path=require('node:path'),os=require('node:os'),assert=require('node:assert/strict');
const {spawnSync}=require('node:child_process');
const {buildFixture}=require('./studio-ui-fixture.cjs');
const out=fs.mkdtempSync(path.join(os.tmpdir(),'serialarm-response-workspace-'));
const fixture=buildFixture(path.join(out,'fixture'),{commissioning:true});
const result=spawnSync('python3',[path.join(__dirname,'commissioning-ui-fixture.py'),path.join(fixture.directory,'commissioning-api'),'response'],{input:'{"axis":0}',encoding:'utf8'});
if(result.status!==0)throw Error(result.stderr);
const record=JSON.parse(result.stdout),dataPath=path.join(fixture.directory,'data.json'),data=JSON.parse(fs.readFileSync(dataPath));
data.responseReceipt=record;data.responseWindow=JSON.parse(fs.readFileSync(record.receipt.path));data.info.available.model=false;
fs.writeFileSync(dataPath,JSON.stringify(data));
const renderSource=fs.readFileSync(path.join(__dirname,'render-studio-ui.cjs'),'utf8');
const scenes=renderSource.slice(renderSource.indexOf('function setScene('),renderSource.indexOf('function snapshot('));
app.setPath('userData',path.join(out,'userdata'));
app.whenReady().then(async()=>{
 const win=new BrowserWindow({width:1440,height:900,frame:false,show:false,webPreferences:{preload:fixture.preload,sandbox:false,offscreen:true,backgroundThrottling:false}});
 try{
  await win.loadFile(fixture.index);await win.webContents.executeJavaScript('document.fonts.ready');
  await win.webContents.executeJavaScript(scenes+`;setScene('workbench-tuning','light');state.profileEditor.core.control.runtime.control_path='CSP';state.capabilityProfile.core.control.runtime.control_path='CSP';state.readiness=structuredClone(studioUiData.responseReceipt.readiness);state.responseCapture=structuredClone(studioUiData.responseWindow);state.responseWorkspaceView='experiment';render();bindResponseWorkspace();drawResponseChart();`);
  await new Promise(resolve=>setTimeout(resolve,450));
  fs.writeFileSync(path.join(out,'experiment-1440.png'),(await win.capturePage()).toPNG());
  const value=await win.webContents.executeJavaScript(`(async()=>{
    const assert=(value,message)=>{if(!value)throw Error(message);};
    const capture=state.responseCapture,receipt=JSON.stringify(state.readiness.response_records),editor=state.profileEditor;
    assert(document.querySelector('#capability-response-chart'),'Experiment opens with response curve');
    assert(document.querySelector('#response-chart-empty').hidden,'Recorded response does not show the empty state');
    const surface=document.querySelector('#response-workspace-panel');window.serialArmStudio.enterSurface(surface);
    assert(Number(surface.style.opacity)>=.85,'Subview spring does not flash from transparent');
    document.documentElement.dataset.reduceMotion='true';window.serialArmStudio.enterSurface(surface);
    assert(!surface.style.opacity&&!surface.style.translate,'Reduced motion finishes the subview presentation immediately');
    delete document.documentElement.dataset.reduceMotion;
    assert(document.querySelectorAll('.response-quick-fields [data-profile-field]').length===3,'Three current-joint parameters share the experiment view');
    assert(!document.querySelector('#profile-apply-admittance'),'Full parameter editor is a separate view');
    const parameter=document.querySelector('.response-quick-fields [data-profile-field]'),field=JSON.parse(parameter.dataset.profileField),prior=Number(parameter.value);parameter.value=String(prior+.1);parameter.dispatchEvent(new Event('input',{bubbles:true}));
    document.querySelector('[data-response-view="parameters"]').click();bindResponseWorkspace();
    assert(state.profileEditor===editor&&profileGet(state.profileEditor,field)===prior+.1,'Shared parameter draft survives subview navigation');
    assert(document.querySelector('#profile-apply-admittance'),'Existing application control remains in parameter view');
    assert(!document.querySelector('#capability-response-chart'),'Parameter view does not duplicate the experiment canvas');
    document.querySelector('[data-response-view="records"]').click();bindResponseWorkspace();
    assert(document.querySelectorAll('[data-response-record]').length===state.readiness.response_records.length,'Saved receipts populate record list');
    document.querySelector('[data-response-record]').click();bindResponseWorkspace();
    assert(document.querySelector('.response-record-detail'),'Record opens a separate detail view');
    assert(document.querySelector('.response-record-detail').textContent.includes(studioUiData.responseReceipt.receipt.scope),'Detail preserves configuration binding scope');
    assert(JSON.stringify(state.readiness.response_records)===receipt,'Record navigation does not change validation or binding');
    assert(state.responseCapture===capture,'Tab and record navigation preserve capture identity');
    document.querySelector('[data-response-record-back]').click();bindResponseWorkspace();
    document.querySelector('[data-response-view="experiment"]').click();bindResponseWorkspace();
    assert(state.responseCapture===capture&&state.responseCapture.samples.length===41,'Returning to experiment retains recorded samples and release marker');
    assert(document.querySelector('#response-release').disabled&&!document.querySelector('#response-export').disabled,'Stopped capture keeps original control enablement');
    const telemetry=state.telemetry;state.responseCapture=null;state.telemetry=null;render();bindResponseWorkspace();drawResponseChart();
    assert(document.querySelectorAll('#response-joint option').length===editor.joint_names.length,'Offline observed-joint selector falls back to the configured joints');
    assert(!document.querySelector('#response-chart-empty').hidden,'Empty chart explains why there is no response curve');
    const primary=document.querySelector('#response-start'),secondary=document.querySelector('#response-stop');
    assert(primary.disabled&&secondary.disabled&&getComputedStyle(primary).backgroundColor===getComputedStyle(secondary).backgroundColor,'Disabled primary and secondary actions have consistent neutral backgrounds');
    const editable=document.querySelector('.response-quick-fields [data-profile-field]');
    assert(!editable.disabled&&getComputedStyle(editable).backgroundColor!==getComputedStyle(secondary).backgroundColor,'Editable quick parameter is visually distinct from disabled controls');
    state.responseCapture=capture;state.telemetry=telemetry;
    const originalSession=state.session, originalTelemetry=state.telemetry;
    for(const robotState of ['OFFLINE','INACTIVE','ACTIVE','FAULT']){
      state.session=robotState==='OFFLINE'?null:{state:'running',mode:'workbench'};
      state.telemetry={...(originalTelemetry||{}),robot_state:robotState};
      updateWorkbenchControlState();
      const dot=document.querySelector('.robot-operation-dot');
      assert(dot.classList.contains(robotState==='ACTIVE'?'on':robotState==='FAULT'?'fault':'idle'),'Robot state indicator matches '+robotState);
      assert([...document.querySelectorAll('[data-workbench-action]')].every(button=>button.disabled===!!controlOperationProblem(button.dataset.workbenchAction)),'Operation buttons preserve their existing availability for '+robotState);
      assert(document.querySelectorAll('.robot-operation-group').length===2,'Power and recovery actions keep distinct groups');
      assert(getComputedStyle(document.querySelector('.robot-operation-bar')).display==='grid','Operation controls use the previous two-row grid');
      const status=document.querySelector('.robot-operation-state').getBoundingClientRect(),hint=document.querySelector('.robot-operation-guidance').getBoundingClientRect();
      assert(!hint.width||status.bottom<=hint.top+1,'Status and guidance do not overlap for '+robotState);
    }
    for(const theme of ['light','dark']){
      state.theme=theme;applyTheme();const button=document.querySelector('[data-workbench-action="activate"]'),probe=document.createElement('i');
      document.body.append(probe);probe.style.backgroundColor='var(--primary)';
      assert(button.disabled&&getComputedStyle(button).backgroundColor===getComputedStyle(probe).backgroundColor,'Disabled activate retains its primary emphasis in '+theme);
      probe.remove();
    }
    state.theme='light';applyTheme();
    state.session=originalSession;state.telemetry=originalTelemetry;updateWorkbenchControlState();
    const calls=await api.fixtureCalls();assert(!calls.some(name=>/workbench_(request|start)|profile_editor_save/.test(name)),'Navigation sends no control or save request');
    return {sharedDraft:true,captureRetained:true,receiptBindingRetained:true,samples:capture.samples.length,records:state.readiness.response_records.length};
  })()`);
  await win.webContents.executeJavaScript(`state.responseWorkspaceView='records';state.responseRecordSelection=state.readiness.response_records[0].id;render();bindResponseWorkspace();`);
  await new Promise(resolve=>setTimeout(resolve,350));fs.writeFileSync(path.join(out,'record-detail-1440.png'),(await win.capturePage()).toPNG());
  win.setContentSize(1000,720);await win.webContents.executeJavaScript(`state.responseWorkspaceView='experiment';state.theme='dark';applyTheme();render();bindResponseWorkspace();drawResponseChart();`);
  await new Promise(resolve=>setTimeout(resolve,450));fs.writeFileSync(path.join(out,'experiment-1000-dark.png'),(await win.capturePage()).toPNG());
  const width=await win.webContents.executeJavaScript('({width:innerWidth,scroll:document.scrollingElement.scrollWidth})');assert(width.scroll<=width.width,'No root horizontal overflow');
  console.log(JSON.stringify({out,...value}));win.destroy();app.quit();
 }catch(error){console.error(error);win.destroy();app.exit(1);}
});
