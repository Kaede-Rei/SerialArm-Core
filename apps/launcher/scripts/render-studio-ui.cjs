/* DISPLAY=:1 node_modules/.bin/electron --no-sandbox scripts/render-studio-ui.cjs
 * Loads the complete production HTML/JS/CSS with a read-only fixture API */
const {app,BrowserWindow}=require('electron');
const fs=require('node:fs');
const path=require('node:path');
const {buildFixture}=require('./studio-ui-fixture.cjs');
const args=process.argv.slice(2),arg=(name,fallback)=>{const i=args.indexOf(name);return i<0?fallback:args[i+1];};
const output=path.resolve(arg('--out','/tmp/serialarm-studio-ui/screenshots'));
const fixture=buildFixture(path.dirname(output),{receipt:arg('--receipt','')});
const auditFile=arg('--clip-audit','');
const audit=auditFile?fs.readFileSync(auditFile,'utf8'):null;
const scenes=arg('--scenes','start,library,lifecycle,parameters-model,parameters-mapping,parameters-motion,parameters-safety,parameters-shutdown,parameters-hardware,model,workbench,geometry,calibration-teaching,calibration-gain,calibration-result,diagnostics,run,settings').split(',');
const widths=arg('--widths','1280,375').split(',').map(Number),themes=arg('--themes','light,dark').split(',');
app.commandLine.appendSwitch('force-device-scale-factor','1');
function setScene(scene,theme){
 const data=window.studioUiData;
 Object.assign(state,{page:scene.startsWith('parameters-')?'profile_setup':scene.startsWith('calibration-')?'calibration':scene,theme,language:'zh-CN',config:{profile:data.editor.profile,profile_file:'/offline/robot_profiles.yaml',serial_port:'',baudrate:'',bus:'',resource_paths:''},info:structuredClone(data.info),profiles:[{name:data.editor.profile}],library:structuredClone(data.library),profileEditor:structuredClone(data.editor),capabilityProfile:structuredClone(data.editor),profileGroup:scene.split('-')[1]||'model',profileDirty:false,profileRawInputs:{},profileFieldErrors:{},readiness:{state:'configuring',checks:{},evidence:{},control_available:true},modelData:null,telemetry:null,session:{state:'idle'},admittance:null,tuneDraft:null,calibrationEntry:scene==='calibration-teaching'?'teaching':'gain',calibrationToolSelected:{teaching:null,gain:null,gravity:null,dynamics:null},calibrationRuntimeCandidates:{},modelCalibrationRecords:[],gainMode:'all',gainTuningMode:'quick',gainFinePurpose:'rigid',gainRequirements:{},teachTrajectories:[],geometryTrajectories:[],driverInspection:{plugin:'serial_arm_hardware_damiao',available:true,drivers:[{plugin:'serial_arm_hardware_damiao',available:true,label:'达妙 / Damiao'}]}});
 if(scene==='calibration-result')state.calibrationToolSelected.gain={directory:'/offline/calibration-result',gain_candidate:structuredClone(data.candidate)};
 if(scene.startsWith('calibration-'))state.telemetry={joint_names:data.editor.joint_names,model_calibration:{workflow_kind:'gain',phase:scene==='calibration-result'?'complete':'idle',task_id:scene==='calibration-result'?data.candidate.task_id:'',gain_budget_source_duration_s:25,gain_target_budget_s:300,gain_time_budget_s:375,gain_elapsed_s:scene==='calibration-result'?239:0,gain_remaining_s:375}};
 if(scene==='start'||scene==='library')state.config.profile='';
 if(data.receipt){state.readiness.gain_initialization=structuredClone(data.receipt);state.config.profile=data.receiptLabel;}
 renderedPage='';shell();
 if(scene==='parameters-motion')document.querySelector('[data-gain-purpose="RIGID"]').open=true;
}
function snapshot(){
 const page=document.querySelector('#page'),texts=[...page.querySelectorAll('h2,h3,h4,summary')].map(n=>n.textContent.trim());
 const sizes=[...new Set([...page.querySelectorAll('*')].filter(n=>n.textContent.trim()&&n.getClientRects().length&&getComputedStyle(n).display!=='none').map(n=>getComputedStyle(n).fontSize))];
 const outer=[...document.querySelectorAll('.content,.studio-page-body')].filter(n=>n.scrollWidth>n.clientWidth+1).map(n=>({class:n.className,width:n.clientWidth,scroll:n.scrollWidth}));
 const buttons=[...page.querySelectorAll('.button.primary')].filter(n=>n.getClientRects().length).map(n=>({id:n.id,text:n.textContent.trim(),color:getComputedStyle(n).color,background:getComputedStyle(n).backgroundColor}));
 const unusualFonts=[...page.querySelectorAll('*')].filter(n=>n.getClientRects().length&&n.textContent.trim()&&n.children.length===0&&![12,14,16,24].includes(parseFloat(getComputedStyle(n).fontSize))).map(n=>({tag:n.tagName,class:n.className,id:n.id,size:getComputedStyle(n).fontSize}));
 return {title:texts,fonts:sizes,unusualFonts,outerOverflow:outer,pageWidth:page.getBoundingClientRect().width,viewport:innerWidth,bodyWidth:document.documentElement.scrollWidth,buttons,middleDots:/\s·\s/.test(page.innerText),parameterKeys:page.querySelectorAll('.parameter-key').length};
}
function testParameters(){
 const checks=[],assert=(ok,name)=>{if(!ok)throw Error(name);checks.push(name);};
 setScene('parameters-model','light');
 const before=JSON.stringify(state.profileEditor.core),gravity=document.querySelector('[data-profile-field=\'["core","model","gravity"]\']');
 assert(gravity.value==='0.01234568, -0.001234568, -9.81','数值数组展示收敛到七位有效数字');
 collectProfileEditor();assert(JSON.stringify(state.profileEditor.core)===before&&!state.profileDirty,'未编辑整表收集保留全部原始精度');
 const gain=document.querySelector('[data-profile-field=\'["core","model","gravity_scale","joint1"]\']');
 assert(gain.value==='1.123457','数值参数不会展示十七位小数');
 gain.value='1.25';gain.dispatchEvent(new Event('input',{bubbles:true}));
 assert(state.profileEditor.core.model.gravity_scale.joint1===1.25&&state.profileDirty,'编辑数值使用实际输入');
 gain.value='1.123457';gain.dispatchEvent(new Event('input',{bubbles:true}));
 assert(state.profileEditor.core.model.gravity_scale.joint1===1.123457,'编辑后还原为短展示值按用户输入保存');
 const group=document.querySelector('[data-profile-group="motion"]');group.click();
 assert(document.querySelectorAll('[data-gain-purpose]').length===4,'真实参数分组切换显示四用途');
 const detail=document.querySelector('[data-gain-purpose="RIGID"]');detail.open=true;
 assert(detail.querySelectorAll('.profile-gain-table tbody tr').length===12,'保持与跟踪参数按关节表格展示');
 assert(document.querySelectorAll('.parameter-key').length===0,'参数表不重复显示配置键');
 setScene('calibration-gain','light');
 document.querySelector('#gain-tuning-mode').value='fine';document.querySelector('#gain-tuning-mode').dispatchEvent(new Event('change',{bubbles:true}));
 assert(!!document.querySelector('#gain-target')&&state.gainTuningMode==='fine','全应用内快标和精调切换生效');
 const receipt={source:'fixture',configuration_applied:false,runtime_baseline_unchanged:true,suggestion_applicable:true,kp:[1],kd:[.1]};
 const html=gainModelInitializationHtml(receipt,['joint1']);assert(html.includes('当前运行与恢复参数保持原值')&&!html.includes('选择受支持的硬件驱动后'),'纯建议回执不会误提示硬件缺失或已写运行参数');
 const staged=gainModelInitializationHtml(window.studioUiData.candidate.model_initialization,window.studioUiData.editor.joint_names);
 assert(staged.includes('需求 Kp')&&staged.includes('模态检查前 Kp')&&staged.includes('初测 Kp')&&staged.includes('初测增益较小不代表机械惯量较小')&&staged.includes('按惯量耦合模态分配阻尼'),'模型需求、执行约束和模态修正分层展示');
 if(window.studioUiData.receipt){const actual=gainModelInitializationHtml(window.studioUiData.receipt);assert(actual.includes('当前运行与恢复参数保持原值')&&actual.includes('尚无示教轨迹')&&actual.includes('需求 Kp')&&!actual.includes('NaN'),'真实Profile回执逐轴模型候选与无示教恢复目标正确展示');}
 return checks;
}
app.whenReady().then(async()=>{
 fs.mkdirSync(output,{recursive:true});
 const win=new BrowserWindow({width:1280,height:1000,show:false,webPreferences:{preload:fixture.preload,sandbox:false,backgroundThrottling:false}});
 const errors=[];win.webContents.on('console-message',(_e,level,message)=>{if(level>=3){errors.push(message);console.error('RENDERER',message);}});
 const settle=()=>win.webContents.executeJavaScript('new Promise(resolve=>setTimeout(()=>requestAnimationFrame(()=>requestAnimationFrame(resolve)),220))');
 await win.loadFile(fixture.index);win.showInactive();await settle();
 win.webContents.debugger.attach('1.3');await win.webContents.debugger.sendCommand('DOM.enable');await win.webContents.debugger.sendCommand('CSS.enable');
 // Terminal emulation is covered by the desktop smoke test. This fixture
 // replaces the shell repeatedly without starting a runtime or fitting xterm.
 await win.webContents.executeJavaScript('term?.dispose();term=null;fit=null;window.Terminal=undefined;void 0;');
 await win.webContents.executeJavaScript(`window.setScene=window.studioFixtureSetScene=${setScene};window.studioFixtureSnapshot=${snapshot};window.studioFixtureTestParameters=${testParameters};void 0;`);
 const checks=await win.webContents.executeJavaScript('(()=>{try{return window.studioFixtureTestParameters();}catch(error){return {error:error.stack};}})()');
 if(checks.error)throw Error(checks.error);
 const report={runtime:process.versions.electron,source:'complete production index.html and app.js, read-only fixture API',checks,renders:[]};
 const runAudit=()=>win.webContents.executeJavaScript(`(${audit})({root:'#app',viewport:true})`);
 for(const width of widths)for(const theme of themes)for(const scene of scenes){
  win.setContentSize(width,1000);await win.webContents.executeJavaScript(`window.studioFixtureSetScene(${JSON.stringify(scene)},${JSON.stringify(theme)})`);await settle();
  const dom=await win.webContents.executeJavaScript('window.studioFixtureSnapshot()');
  if(dom.bodyWidth>width||dom.outerOverflow.length)throw Error('Outer overflow '+JSON.stringify({scene,width,dom}));
  if(dom.buttons.some(b=>b.color!=='rgb(255, 255, 255)'||b.background==='rgb(166, 75, 0)'))throw Error('Primary button palette '+JSON.stringify({scene,theme,buttons:dom.buttons}));
  const tree=await win.webContents.debugger.sendCommand('DOM.getDocument');const fontNode=await win.webContents.debugger.sendCommand('DOM.querySelector',{nodeId:tree.root.nodeId,selector:'#page h2'});const platformFonts=fontNode.nodeId?(await win.webContents.debugger.sendCommand('CSS.getPlatformFontsForNode',{nodeId:fontNode.nodeId})).fonts:[];
  if(platformFonts.some(font=>/italic|oblique/i.test(font.postScriptName||'')))throw Error('UI heading resolved to an italic system alias '+JSON.stringify(platformFonts));
  const item={scene,width,theme,dom,platformFonts,captures:[]};
  if(audit){item.defaultClip=await runAudit();if(item.defaultClip.cut)throw Error('Default clip '+JSON.stringify({scene,width,clip:item.defaultClip}));}
  const scroller=await win.webContents.executeJavaScript(`(()=>{const nodes=[...document.querySelectorAll('.studio-page-body,.content')];const node=nodes.find(n=>n.scrollHeight>n.clientHeight+2)||nodes[0];node.dataset.fixtureScroll='true';return {height:node.scrollHeight,viewport:node.clientHeight};})()`);
  for(const position of [0,Math.max(0,scroller.height-scroller.viewport)]){
   if(position===0&&item.captures.length)continue;
   const scrollTop=await win.webContents.executeJavaScript(`(()=>{const n=document.querySelector('[data-fixture-scroll]');n.scrollTop=${position};return n.scrollTop;})()`);await settle();
   const shot=await win.webContents.capturePage(),png=shot.toPNG();if(shot.isEmpty()||png.length<10000)throw Error('Empty screenshot');
   const file=path.join(output,`${scene}-${width}-${theme}${position?'-bottom':''}.png`);fs.writeFileSync(file,png);item.captures.push({file,scrollTop,bytes:png.length});
  }
  await win.webContents.executeJavaScript(`document.querySelector('[data-fixture-scroll]').scrollTop=0`);
  const point=await win.webContents.executeJavaScript(`(()=>{const n=[...document.querySelectorAll('#page button:not(:disabled),#page summary')].find(n=>{const r=n.getBoundingClientRect();return r.top>100&&r.bottom<innerHeight;});if(!n)return null;const r=n.getBoundingClientRect();return {x:Math.round(r.left+r.width/2),y:Math.round(r.top+r.height/2)};})()`);
  if(point){win.webContents.sendInputEvent({type:'mouseMove',...point});await settle();}
  if(audit){item.hoverClip=await runAudit();if(item.hoverClip.cut)throw Error('Hover clip '+JSON.stringify({scene,width,clip:item.hoverClip}));}
  if(['parameters-motion','parameters-hardware','calibration-result','diagnostics'].includes(scene)||scene==='lifecycle'&&args.includes('--receipt')){
   await win.webContents.executeJavaScript(`document.querySelectorAll('#page details').forEach(n=>n.open=true)`);await settle();
   item.expandedDom=await win.webContents.executeJavaScript('window.studioFixtureSnapshot()');
   if(item.expandedDom.outerOverflow.length)throw Error('Expanded overflow '+scene+' '+width);
   if(audit){item.expandedClip=await runAudit();if(item.expandedClip.cut)throw Error('Expanded clip '+JSON.stringify({scene,width,clip:item.expandedClip}));}
   const file=path.join(output,`${scene}-${width}-${theme}-expanded.png`);fs.writeFileSync(file,(await win.webContents.capturePage()).toPNG());item.expandedFile=file;
   const target=scene==='calibration-result'||scene==='lifecycle'?'[data-gain-model-stages]':scene==='parameters-motion'?'.profile-gain-task':null;
   if(target){const position=await win.webContents.executeJavaScript(`(()=>{const n=document.querySelector(${JSON.stringify(target)}),s=document.querySelector('[data-fixture-scroll]');if(!n)return null;const top=s.getBoundingClientRect().top,occluded=Math.max(top,...[...document.querySelectorAll('#page > .page-head,.workspace-operation-panel')].map(node=>node.getBoundingClientRect().bottom));s.scrollTop+=n.getBoundingClientRect().top-occluded-16;return s.scrollTop;})()`);await settle();const detail=path.join(output,`${scene}-${width}-${theme}-detail.png`);fs.writeFileSync(detail,(await win.webContents.capturePage()).toPNG());item.expandedDetail={file:detail,scrollTop:position};}
  }
  report.renders.push(item);
 }
 report.apiCalls=await win.webContents.executeJavaScript('window.serialArm.fixtureCalls()');report.consoleErrors=errors;
 if(errors.length)throw Error('Renderer console errors '+JSON.stringify(errors));
 fs.writeFileSync(path.join(output,'report.json'),JSON.stringify(report,null,2));
 console.log('STUDIO_UI_RENDER_PASS '+JSON.stringify({checks:checks.length,renders:report.renders.length,report:path.join(output,'report.json')}));win.destroy();app.quit();
}).catch(error=>{console.error(error.stack);app.exit(1);});
