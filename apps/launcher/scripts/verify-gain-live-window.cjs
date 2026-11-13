/* Real sandboxed read-only Electron child window; no backend, Profile or hardware.
 * node_modules/.bin/electron scripts/verify-gain-live-window.cjs
 * Optional external shadow audit: SERIAL_ARM_CLIP_AUDIT=/path/to/clip-audit.js */
const {app,BrowserWindow,ipcMain}=require('electron');
const {GainLiveWindow}=require('../desktop/gain-live-window.cjs');
const assert=require('node:assert/strict'),fs=require('node:fs'),path=require('node:path'),vm=require('node:vm');
const directory=path.resolve(__dirname,'../desktop'),root=path.resolve(__dirname,'..');
const source=fs.readFileSync(path.join(root,'renderer/app.js'),'utf8');
function extract(name){const a=source.indexOf('function '+name+'('),b=source.indexOf('\nfunction ',a+1);return source.slice(a,b);}
const state={language:'zh-CN',theme:'light',telemetryReceivedAt:Date.now(),telemetry:{valid:true,feedback_age_ms:5,sequence:12,
  robot_state:'ACTIVE',joint_names:Array.from({length:8},(_,i)=>'axis_'+i),joint:{kp:[1,2,3,4,5,6,7,8],kd:[.1,.2,.3,.4,.5,.6,.7,.8],pos:Array(8).fill(.1),ref_pos:Array(8).fill(.12),pd_torque:Array(8).fill(.2)},
  model_calibration:{task_id:'offline-ui-test',phase:'gain_adaptation',gain_probe_stage:'trajectory_frozen',gain_active_target:'rigid_tracking',gain_probe_kp:499,gain_probe_kd:4.9}}};
const context=vm.createContext({state,workbenchConnected:()=>true,gainProbeStageLabel:()=> '整轨测量',modelCalibrationPhaseText:()=> '已结束',gainPurposeLabel:()=> '刚性',jointDisplaySpec:()=>({factor:180/Math.PI,unit:'°'})});
vm.runInContext(extract('gainLiveSnapshot'),context);
const snapshot=JSON.parse(JSON.stringify(vm.runInContext('gainLiveSnapshot()',context)));
assert.equal(snapshot.rows[0].kp,1,'Actual cycle gains are used instead of probe/candidate gains');
assert.equal(snapshot.rows.length,8,'Axis count is model driven');
delete state.telemetry.joint.kp;
assert.equal(vm.runInContext('gainLiveSnapshot().rows[0].kp',context),null,'Missing actual gain remains unmeasured');
app.commandLine.appendSwitch('force-device-scale-factor','1');
app.whenReady().then(async()=>{
  const owner=new BrowserWindow({show:false,webPreferences:{sandbox:true}});
  const monitor=new GainLiveWindow({BrowserWindow,ipcMain,owner,trusted:()=>{},directory});
  await monitor.open({...snapshot,rows:[]});const child=monitor.window;
  const read=code=>child.webContents.executeJavaScript(code);
  await read('window.gainMonitor.current().then(paintGainMonitor)');
  assert((await read("document.querySelector('#gain-live-rows').textContent")).includes('等待实际控制周期参数'));
  monitor.update(snapshot);await read('window.gainMonitor.current().then(paintGainMonitor)');
  assert.equal(await read("document.querySelectorAll('#gain-live-rows tr').length"),8);
  assert.equal(await read("document.querySelector('#gain-live-rows tr td').textContent"),'1.000');
  assert.equal(await read('typeof window.serialArm'),'undefined','No control bridge is exposed in this window');
  const next=structuredClone(snapshot);next.rows[0].kp=1.25;next.received_at=Date.now();monitor.update(next);
  await read('window.gainMonitor.current().then(paintGainMonitor)');
  assert.equal(await read("document.querySelector('#gain-live-rows tr td').textContent"),'1.250');
  await read('paintGainMonitor(gainSnapshot,Date.now()+1000)');
  assert((await read("document.querySelector('#gain-live-status').textContent")).includes('已过期'));
  monitor.update({...next,connected:false,rows:[]});await read('window.gainMonitor.current().then(paintGainMonitor)');
  assert((await read("document.querySelector('#gain-live-status').textContent")).includes('已断开'));
  assert.equal(await read("document.querySelector('#gain-live-rows tr td').textContent"),'1.250','Disconnected clearing retains the last actual gain snapshot');
  await monitor.open(next);assert.equal(monitor.window,child,'Repeated open focuses the same child');
  const out='/tmp/serialarm-gain-live-ui';fs.mkdirSync(out,{recursive:true});
  const layouts=[];
  const auditSource=process.env.SERIAL_ARM_CLIP_AUDIT?fs.readFileSync(process.env.SERIAL_ARM_CLIP_AUDIT,'utf8'):null;
  for(const [width,height,theme] of [[1280,720,'light'],[375,760,'light'],[880,620,'dark'],[480,360,'light'],[480,360,'dark']]){
    child.setMinimumSize(320,360);child.setContentSize(width,height);
    monitor.update({...next,theme,received_at:Date.now()});await read('window.gainMonitor.current().then(paintGainMonitor)');
    await read('document.querySelector(".gain-live-shell").scrollTop=0');
    await new Promise(resolve=>setTimeout(resolve,100));
    const layout=await read(`(()=>({width:innerWidth,height:innerHeight,documentWidth:document.documentElement.scrollWidth,
        footerBottom:document.querySelector('footer').getBoundingClientRect().bottom,
        tableHeight:document.querySelector('.gain-live-table-wrap').getBoundingClientRect().height,
        visible:['header','.gain-live-meta','.gain-live-table-wrap'].every(selector=>{
            const rect=document.querySelector(selector).getBoundingClientRect();return rect.width>0&&rect.left>=0&&rect.right<=innerWidth;
        }),tableRows:document.querySelectorAll('#gain-live-rows tr').length}))()`);
    assert.equal(layout.width,width,'The actual content viewport matches the tested width');
    assert(layout.documentWidth<=width&&layout.visible,'Page and primary sections fit; wide data scrolls inside its table');
    assert(layout.tableHeight>0,'The table retains a scroll viewport');
    if(height>480)assert(layout.footerBottom<=height,'Footer remains visible at ordinary window heights');
    assert.equal(layout.tableRows,next.rows.length,'Every runtime axis remains available at each viewport size');
    if(auditSource){const audit=await read('('+auditSource+')({viewport:true})');assert.equal(audit.cut,0);}
    const image=await child.webContents.capturePage();assert(!image.isEmpty(),'Rendered evidence is nonempty');
    fs.writeFileSync(path.join(out,`gain-${width}-${theme}.png`),image.toPNG());
    if(height<=480){
        const reached=await read(`(()=>{const shell=document.querySelector('.gain-live-shell');shell.scrollTop=shell.scrollHeight;
            const footer=document.querySelector('footer').getBoundingClientRect();return {scrollTop:shell.scrollTop,top:footer.top,bottom:footer.bottom};})()`);
        assert(reached.top>=0&&reached.bottom<=height,'Footer remains reachable by native scrolling at the minimum window height');
        layout.footerAfterScroll=reached;
        fs.writeFileSync(path.join(out,`gain-${width}-${theme}-bottom.png`),(await child.webContents.capturePage()).toPNG());
    }
    if(width===375){
        const reached=await read(`(()=>{const wrapper=document.querySelector('.gain-live-table-wrap');wrapper.scrollLeft=wrapper.scrollWidth;
            const last=document.querySelector('#gain-live-rows tr').lastElementChild.getBoundingClientRect(),bounds=wrapper.getBoundingClientRect();
            return last.right<=bounds.right+1&&last.left>=bounds.left;})()`);
        assert(reached,'The last telemetry column is reachable by native table scrolling');
    }
    layouts.push({theme,...layout});
  }
  fs.writeFileSync(path.join(out,'report.json'),JSON.stringify({layouts},null,2));
  await new Promise(resolve=>{child.once('closed',resolve);child.close();});assert.equal(monitor.window,null);assert(!owner.isDestroyed(),'Closing monitor does not close the owner');
  await monitor.open({...next,active:false,phase:'标定已结束'});const reopened=monitor.window;
  await reopened.webContents.executeJavaScript('window.gainMonitor.current().then(paintGainMonitor)');
  assert((await reopened.webContents.executeJavaScript("document.querySelector('#gain-live-phase').textContent")).includes('已结束'));
  owner.destroy();assert(reopened.isDestroyed(),'Closing owner closes its monitor');
  console.log('GAIN_LIVE_WINDOW_PASS '+out);app.exit(0);
}).catch(error=>{console.error(error.stack);app.exit(1);});
