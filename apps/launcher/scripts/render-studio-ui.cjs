/* Whole production renderer with an isolated, deny-by-default fixture API.
 * DISPLAY=:1 node_modules/.bin/electron --no-sandbox scripts/render-studio-ui.cjs \
 *   --render-only --sizes 1300x900,1000x720 --languages zh-CN,en \
 *   --scenes start,lifecycle,parameters-motion,model,workbench,calibration-result,settings \
 *   --themes light,dark --out /tmp/serialarm-studio-ui/screenshots
 * --with-model reads builtin description through the offline model executable;
 * --rescue-model inspects its real structure without claiming 3D rendering.
 * --allow-findings captures all pages while retaining issues in report.json.
 * --software explicitly selects the bundled SwiftShader preview backend.
 */
const {app,BrowserWindow}=require('electron');
const fs=require('node:fs');
const path=require('node:path');
const {buildFixture}=require('./studio-ui-fixture.cjs');
const args=process.argv.slice(2),arg=(name,fallback)=>{const i=args.indexOf(name);return i<0?fallback:args[i+1];};
const output=path.resolve(arg('--out','/tmp/serialarm-studio-ui/screenshots'));
const fixture=buildFixture(path.dirname(output),{receipt:arg('--receipt',''),model:args.includes('--with-model')});
const auditFile=arg('--clip-audit','');
const audit=auditFile?fs.readFileSync(auditFile,'utf8'):null;
const scenes=arg('--scenes','start,library,lifecycle,parameters-model,parameters-mapping,parameters-motion,parameters-safety,parameters-shutdown,parameters-hardware,model,workbench,geometry,calibration-teaching,calibration-gain,calibration-gravity,calibration-dynamics,calibration-result,diagnostics,run,settings').split(',');
const widths=arg('--widths','1300,1000').split(',').map(Number),themes=arg('--themes','light,dark').split(',');
const sizes=arg('--sizes','').split(',').filter(Boolean).map(value=>value.split('x').map(Number));
if(!sizes.length)for(const width of widths)sizes.push([width,Number(arg('--height','900'))]);
if(sizes.some(size=>size.length!==2||size.some(value=>!Number.isFinite(value)||value<500)))throw Error('Expected desktop dimensions such as --sizes 1300x900,1000x720');
const languages=arg('--languages','zh-CN').split(',');
app.commandLine.appendSwitch('force-device-scale-factor','1');
app.setPath('userData',path.join(fixture.directory,'electron-userdata'));
if(args.includes('--software')){app.commandLine.appendSwitch('use-gl','angle');app.commandLine.appendSwitch('use-angle','swiftshader');app.commandLine.appendSwitch('enable-unsafe-swiftshader');}
function setScene(scene,theme,language='zh-CN'){
 const data=window.studioUiData;
 if(window.studioFixtureRescue)graphicsState.effectiveMode='rescue';
 Object.assign(state,{page:scene.startsWith('parameters-')?'profile_setup':scene.startsWith('calibration-')?'calibration':scene,theme,language,config:{profile:data.editor.profile,profile_file:'/offline/robot_profiles.yaml',serial_port:'',baudrate:'',bus:'',resource_paths:''},info:structuredClone(data.info),profiles:[{name:data.editor.profile}],library:structuredClone(data.library),profileEditor:structuredClone(data.editor),capabilityProfile:structuredClone(data.editor),profileGroup:scene.split('-')[1]||'model',profileDirty:false,profileRawInputs:{},profileFieldErrors:{},readiness:{state:'configuring',checks:{},evidence:{},control_available:true},modelData:data.model?structuredClone(data.model):null,modelPositions:data.model?.native?.state?.positions?.slice()||[],modelSelection:data.model?.model?.links?.[0]?.name||'',telemetry:null,session:{state:'idle'},admittance:null,tuneDraft:null,calibrationEntry:scene==='calibration-teaching'?'teaching':'gain',calibrationToolSelected:{teaching:null,gain:null,gravity:null,dynamics:null},calibrationRuntimeCandidates:{},modelCalibrationRecords:[],gainMode:'all',gainTuningMode:'quick',gainFinePurpose:'rigid',gainRequirements:{},teachTrajectories:[],geometryTrajectories:[],driverInspection:{plugin:'serial_arm_hardware_damiao',available:true,drivers:[{plugin:'serial_arm_hardware_damiao',available:true,label:'达妙 / Damiao'}]}});
 if(scene==='calibration-result')state.calibrationToolSelected.gain={directory:'/offline/calibration-result',gain_candidate:structuredClone(data.candidate)};
 state.modelCalibrationOffline=null;state.idimReport=null;
 if(scene.startsWith('calibration-'))state.telemetry={joint_names:data.editor.joint_names,model_calibration:{workflow_kind:'gain',phase:scene==='calibration-result'?'complete':'idle',task_id:scene==='calibration-result'?data.candidate.task_id:'',gain_budget_source_duration_s:25,gain_target_budget_s:300,gain_time_budget_s:375,gain_elapsed_s:scene==='calibration-result'?239:0,gain_remaining_s:375}};
 if(['calibration-gravity','calibration-dynamics'].includes(scene)){
  state.calibrationEntry=scene.split('-')[1];state.session={state:'running',mode:'workbench'};
  state.telemetry.robot_state='INACTIVE';state.telemetry.valid=false;
  state.telemetry.model_calibration={phase:'idle',workflow_kind:state.calibrationEntry,gravity_result:{first_moments:[{link:'fixture-link',first_moment:[0,0,0]}]}};
  state.idimReport={physical_pass:true};
 }
 if(scene==='start'||scene==='library')state.config.profile='';
 if(data.receipt){state.readiness.gain_initialization=structuredClone(data.receipt);state.config.profile=data.receiptLabel;}
 renderedPage='';shell();
 if(scene==='parameters-csp'){state.profileEditor.core.control.runtime.control_path='CSP';state.profileGroup='motion';render();}
 if(scene==='parameters-motion')document.querySelector('[data-gain-purpose="RIGID"]').open=true;
}
function snapshot(){
 const page=document.querySelector('#page'),texts=[...page.querySelectorAll('h2,h3,h4,summary')].map(n=>n.textContent.trim());
 const visibleText=[...page.querySelectorAll('*')].filter(n=>{const r=n.getBoundingClientRect(),style=getComputedStyle(n);return r.width&&r.height&&r.top<innerHeight&&r.bottom>0&&style.visibility!=='hidden'&&([...n.childNodes].some(child=>child.nodeType===3&&child.textContent.trim())||n.matches('input:not([type=checkbox]):not([type=radio]),textarea,select'));});
 const sizes=[...new Set(visibleText.map(n=>getComputedStyle(n).fontSize))];
 const outer=[...document.querySelectorAll('.content,.studio-page-body')].filter(n=>n.scrollWidth>n.clientWidth+1).map(n=>({class:n.className,width:n.clientWidth,scroll:n.scrollWidth}));
 const buttons=[...page.querySelectorAll('.button.primary')].filter(n=>n.getClientRects().length).map(n=>({id:n.id,text:n.textContent.trim(),color:getComputedStyle(n).color,background:getComputedStyle(n).backgroundColor}));
 const paletteProbe=document.createElement('i');paletteProbe.hidden=true;paletteProbe.style.cssText='color:var(--primary-on);background-color:var(--primary)';document.body.append(paletteProbe);
 const primaryPalette={color:getComputedStyle(paletteProbe).color,background:getComputedStyle(paletteProbe).backgroundColor};paletteProbe.remove();
 const unusualFonts=visibleText.filter(n=>parseFloat(getComputedStyle(n).fontSize)<12).map(n=>({tag:n.tagName,class:n.className,id:n.id,size:getComputedStyle(n).fontSize}));
 const parse=value=>{const parts=value.match(/[\d.]+/g)?.map(Number)||[];return parts.length>=3?[...parts.slice(0,3),parts[3]??1]:[0,0,0,0];};
 const blend=(front,back)=>front.slice(0,3).map((channel,index)=>channel*front[3]+back[index]*(1-front[3]));
 const luminance=rgb=>rgb.map(value=>{const c=value/255;return c<=.04045?c/12.92:((c+.055)/1.055)**2.4;}).reduce((v,c,i)=>v+c*[.2126,.7152,.0722][i],0);
 const contrast=[...page.querySelectorAll('h2,h3,label,.hint,.pill,.eyebrow,.gain-section-label,.parameter-key,.breadcrumb')].filter(node=>{const r=node.getBoundingClientRect();return node.innerText?.trim()&&r.width&&r.top<innerHeight&&r.bottom>0&&!node.closest(':disabled,[aria-disabled="true"]');}).slice(0,100).map(node=>{
  let background=document.documentElement.classList.contains('light')?[255,255,255]:[0,0,0];const ancestors=[];
  for(let parent=node;parent;parent=parent.parentElement)ancestors.push(parent);
  for(const parent of ancestors.reverse())background=blend(parse(getComputedStyle(parent).backgroundColor),background);
  const style=getComputedStyle(node),foreground=blend(parse(style.color),background),lum=[luminance(foreground),luminance(background)].sort((a,b)=>b-a);
  return {tag:node.tagName,class:node.className,text:node.innerText.trim().slice(0,60),size:style.fontSize,color:style.color,ratio:Math.round((lum[0]+.05)/(lum[1]+.05)*100)/100};
 });
 return {title:texts,fonts:sizes,smallText:unusualFonts,contrast,unusualFonts,outerOverflow:outer,pageWidth:page.getBoundingClientRect().width,viewport:innerWidth,bodyWidth:document.documentElement.scrollWidth,buttons,primaryPalette,middleDots:/\s·\s/.test(page.innerText),parameterKeys:page.querySelectorAll('.parameter-key').length};
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
 assert(document.querySelectorAll('[data-gain-purpose]').length===3&&document.querySelector('.profile-mode-note').textContent.includes('伺服跟踪'),'三组独立增益与伺服共享说明');
 const detail=document.querySelector('[data-gain-purpose="RIGID"]');detail.open=true;
 assert(detail.querySelectorAll('.profile-gain-table tbody tr').length===6,'统一增益每个关节只展示一行');
 const field=path=>document.querySelector('[data-profile-field='+JSON.stringify(JSON.stringify(path))+']');
 const execution=field(['core','control','runtime','control_path']);
 assert(document.querySelectorAll('[data-profile-field]').length===new Set([...document.querySelectorAll('[data-profile-field]')].map(n=>n.dataset.profileField)).size,'配置参数无重复编辑入口');
 execution.value='CSP';execution.dispatchEvent(new Event('change',{bubbles:true}));
 assert(!document.querySelector('[data-gain-purpose]')&&!!field(['core','capability','admittance','controller','mass','joint1']),'切换CSP显示MDK并隐藏MIT增益');
 assert(!field(['core','control','runtime','model_feedforward_mode'])&&!field(['core','capability','admittance','enabled']),'CSP不展示MIT前馈或废弃导纳开关');
 assert(!!document.querySelector('#profile-add-tracking-error')&&!state.profileEditor.core.safety_policy.max_tracking_error,'未配置的跟踪上限明确提示且不生成安全值');
 document.querySelector('[data-profile-group=observation]').click();
 assert(!!field(['core','capability','admittance','observer','momentum_gain','joint1']),'外力估计拥有独立参数分组');
 const observer=field(['core','capability','admittance','observer','momentum_gain','joint1']);observer.value='30';observer.dispatchEvent(new Event('input',{bubbles:true}));
 document.querySelector('[data-profile-group=motion]').click();
 assert(state.profileEditor.core.capability.admittance.observer.momentum_gain.joint1===30,'切换分组保留实际参数草稿');
 const back=field(['core','control','runtime','control_path']);back.value='MIT';back.dispatchEvent(new Event('change',{bubbles:true}));
 assert(!!document.querySelector('[data-gain-purpose=RIGID]')&&!field(['core','capability','admittance','controller','mass','joint1']),'切回MIT恢复独立增益编辑');
 assert(document.querySelectorAll('.parameter-key').length===0,'参数表不重复显示配置键');
 state.session={state:'running',mode:'workbench'};state.workbenchPending={kind:''};state.blockingBusy=false;
 state.telemetry={robot_state:'ACTIVE',control_path:'CSP',impedance_mode:'SERVO_TRACKING'};
 state.readiness={modes:{RIGID:'unconfigured'},modes_by_path:{MIT:{RIGID:'unconfigured',SERVO_TRACKING:'unconfigured'},CSP:{RIGID:'configured',SERVO_TRACKING:'configured'}}};
 assert(gainModeConfigured('RIGID')&&controlGainPurposeMayChange('SERVO_TRACKING'),'当前CSP准入不依赖MIT增益或已保存路径');
 state.telemetry.control_path='MIT';assert(!gainModeConfigured('RIGID'),'切回MIT仍检查MIT增益资格');
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
 win.webContents.on('render-process-gone',(_event,details)=>{console.error('Fixture renderer exited',JSON.stringify(details));app.exit(1);});
 const deadline=setTimeout(()=>{console.error('UI fixture exceeded its bounded render time');app.exit(1);},Number(arg('--timeout','120000')));
 const errors=[];win.webContents.on('console-message',(_e,level,message)=>{if(level>=3){errors.push(message);console.error('RENDERER',message);}});
 const settle=()=>win.webContents.executeJavaScript('document.fonts.ready.then(()=>new Promise(resolve=>setTimeout(()=>requestAnimationFrame(()=>requestAnimationFrame(resolve)),220)))');
 await win.loadFile(fixture.index);win.show();win.focus();win.webContents.focus();await settle();
 win.webContents.debugger.attach('1.3');await win.webContents.debugger.sendCommand('DOM.enable');await win.webContents.debugger.sendCommand('CSS.enable');
 // Terminal emulation is covered by the desktop smoke test. This fixture
 // replaces the shell repeatedly without starting a runtime or fitting xterm.
 await win.webContents.executeJavaScript('term?.dispose();term=null;fit=null;window.Terminal=undefined;void 0;');
 await win.webContents.executeJavaScript(`window.studioFixtureRescue=${args.includes('--rescue-model')};window.setScene=window.studioFixtureSetScene=${setScene};window.studioFixtureSnapshot=${snapshot};window.studioFixtureTestParameters=${testParameters};void 0;`);
 const checks=args.includes('--render-only')?[]:await win.webContents.executeJavaScript('(()=>{try{return window.studioFixtureTestParameters();}catch(error){return {error:error.stack};}})()');
 if(checks.error)throw Error(checks.error);
 const report={runtime:process.versions.electron,graphicsBackend:args.includes('--software')?'bundled SwiftShader':'normal',modelPreview:args.includes('--rescue-model')?'rescue without 3D':args.includes('--with-model')?'read-only native model':'not loaded',findings:[],source:'complete production index.html and app.js, read-only fixture API',checks,renders:[]};
 const runAudit=()=>win.webContents.executeJavaScript(`(${audit})({root:'#app',viewport:true})`);
 const finding=(name,evidence)=>{report.findings.push({name,...evidence});if(!args.includes('--allow-findings'))throw Error(name+' '+JSON.stringify(evidence));};
 for(const [width,height] of sizes)for(const theme of themes)for(const language of languages)for(const scene of scenes){
  win.setContentSize(width,height);await win.webContents.executeJavaScript(`window.studioFixtureSetScene(${JSON.stringify(scene)},${JSON.stringify(theme)},${JSON.stringify(language)})`);win.webContents.sendInputEvent({type:'mouseMove',x:0,y:0});await settle();
  console.log('STUDIO_SCENE '+JSON.stringify({scene,width,height,theme,language}));
  const dom=await win.webContents.executeJavaScript('window.studioFixtureSnapshot()');
  if(dom.bodyWidth>width||dom.outerOverflow.length)finding('Outer overflow',{scene,width,height,theme,language,dom});
  if(dom.buttons.some(b=>b.color!==dom.primaryPalette.color||b.background!==dom.primaryPalette.background))finding('Primary button palette',{scene,width,height,theme,language,buttons:dom.buttons,expected:dom.primaryPalette});
  const tree=await win.webContents.debugger.sendCommand('DOM.getDocument');const fontNode=await win.webContents.debugger.sendCommand('DOM.querySelector',{nodeId:tree.root.nodeId,selector:'#page .page-title,#page h1,#page h2,#page h3'});const platformFonts=fontNode.nodeId?(await win.webContents.debugger.sendCommand('CSS.getPlatformFontsForNode',{nodeId:fontNode.nodeId})).fonts:[];
  const fontState=await win.webContents.executeJavaScript(`({status:document.fonts.status,check:document.fonts.check('14px "SerialArm Maple Italic"'),faces:[...document.fonts].filter(face=>face.family.includes('SerialArm Maple Italic')).map(face=>({family:face.family,weight:face.weight,style:face.style,status:face.status}))})`);
  if(!fontState.check||!fontState.faces.some(face=>face.status==='loaded')||!platformFonts.some(font=>/MapleMono.*CN.*Italic/i.test(font.postScriptName||'')))finding('Requested Maple Italic font was not used',{scene,width,height,theme,language,platformFonts,fontState});
  const item={scene,width,height,theme,language,dom,platformFonts,fontState,captures:[]};
  const stem=`${scene}-${width}x${height}-${theme}-${language}`;
  if(dom.fonts.length>4||dom.smallText.length)finding('Visible text scale',{scene,width,height,theme,language,fonts:dom.fonts,smallText:dom.smallText});
  if(audit){item.defaultClip=await runAudit();if(item.defaultClip.cut)finding('Default shadow clip',{scene,width,height,theme,language,clip:item.defaultClip});}
  const scroller=await win.webContents.executeJavaScript(`(()=>{const nodes=[...document.querySelectorAll('.studio-page-body,.content')];const node=nodes.find(n=>n.scrollHeight>n.clientHeight+2)||nodes[0];node.dataset.fixtureScroll='true';return {height:node.scrollHeight,viewport:node.clientHeight};})()`);
  for(const position of [0,Math.max(0,scroller.height-scroller.viewport)]){
   if(position===0&&item.captures.length)continue;
   const scrollTop=await win.webContents.executeJavaScript(`(()=>{const n=document.querySelector('[data-fixture-scroll]');n.scrollTop=${position};return n.scrollTop;})()`);await settle();
   const shot=await win.webContents.capturePage(),png=shot.toPNG();if(shot.isEmpty()||png.length<10000)throw Error('Empty screenshot');
   const file=path.join(output,`${stem}${position?'-bottom':''}.png`);fs.writeFileSync(file,png);item.captures.push({file,scrollTop,bytes:png.length});
  }
  await win.webContents.executeJavaScript(`document.querySelector('[data-fixture-scroll]').scrollTop=0`);
  item.hover=[];
  for(const [kind,selector] of [['button','#page .button:not(:disabled)'],['navigation','.nav-item:not(.active):not(:disabled)'],['entry','.calibration-entry:not(.selected):not(:disabled),.mode-card:not(.active):not(:disabled)']]){
   win.focus();win.webContents.focus();
   win.webContents.sendInputEvent({type:'mouseMove',x:0,y:0});await settle();
   const target=await win.webContents.executeJavaScript(`(()=>{const n=[...document.querySelectorAll(${JSON.stringify(selector)})].find(n=>{const r=n.getBoundingClientRect();return r.width&&r.top>40&&r.bottom<innerHeight;});if(!n)return null;n.dataset.fixtureHover='true';const r=n.getBoundingClientRect(),s=getComputedStyle(n);return {x:Math.round(r.left+r.width/2),y:Math.round(r.top+r.height/2),text:n.innerText,background:s.backgroundColor,color:s.color};})()`);
   if(!target)continue;win.webContents.sendInputEvent({type:'mouseMove',x:target.x,y:target.y});await settle();
   const hover=await win.webContents.executeJavaScript(`(()=>{const n=document.querySelector('[data-fixture-hover]'),s=getComputedStyle(n);const result={active:n.matches(':hover'),background:s.backgroundColor,color:s.color};delete n.dataset.fixtureHover;return result;})()`);
   const entry={kind,before:target,after:hover};if(audit){entry.clip=await runAudit();if(entry.clip.cut)finding('Hover shadow clip',{scene,width,height,theme,language,kind,clip:entry.clip});}
   if(width===1300&&language==='zh-CN'&&['lifecycle','calibration-gain','run','settings'].includes(scene)){entry.file=path.join(output,`${stem}-hover-${kind}.png`);fs.writeFileSync(entry.file,(await win.webContents.capturePage()).toPNG());}
   item.hover.push(entry);
  }
  if(['parameters-motion','parameters-hardware','calibration-result','diagnostics'].includes(scene)||scene==='lifecycle'&&args.includes('--receipt')){
   await win.webContents.executeJavaScript(`document.querySelectorAll('#page details').forEach(n=>n.open=true)`);await settle();
   item.expandedDom=await win.webContents.executeJavaScript('window.studioFixtureSnapshot()');
   if(item.expandedDom.outerOverflow.length)finding('Expanded overflow',{scene,width,height,theme,language,dom:item.expandedDom});
   if(audit){item.expandedClip=await runAudit();if(item.expandedClip.cut)finding('Expanded shadow clip',{scene,width,height,theme,language,clip:item.expandedClip});}
   const file=path.join(output,`${stem}-expanded.png`);fs.writeFileSync(file,(await win.webContents.capturePage()).toPNG());item.expandedFile=file;
   const target=scene==='calibration-result'||scene==='lifecycle'?'[data-gain-model-stages]':scene==='parameters-motion'?'.profile-gain-task':null;
   if(target){const position=await win.webContents.executeJavaScript(`(()=>{const n=document.querySelector(${JSON.stringify(target)}),s=document.querySelector('[data-fixture-scroll]');if(!n)return null;const top=s.getBoundingClientRect().top,occluded=Math.max(top,...[...document.querySelectorAll('#page > .page-head,.workspace-operation-panel')].map(node=>node.getBoundingClientRect().bottom));s.scrollTop+=n.getBoundingClientRect().top-occluded-16;return s.scrollTop;})()`);await settle();const detail=path.join(output,`${stem}-detail.png`);fs.writeFileSync(detail,(await win.webContents.capturePage()).toPNG());item.expandedDetail={file:detail,scrollTop:position};}
  }
  if(audit){await settle();item.finalClip=await runAudit();if(item.finalClip.cut)finding('Final shadow clip',{scene,width,height,theme,language,clip:item.finalClip});}
  report.renders.push(item);
 }
 // Test keyboard focus and reduced-motion using Chromium's real input/media
 // machinery; no device API or production backend exists in this fixture.
 await win.webContents.executeJavaScript(`window.studioFixtureSetScene('settings','light','en')`);await settle();
 win.focus();win.webContents.focus();
 await win.webContents.executeJavaScript(`document.querySelector('#page button:not(:disabled)')?.focus()`);
 win.webContents.sendInputEvent({type:'keyDown',keyCode:'Tab'});win.webContents.sendInputEvent({type:'keyUp',keyCode:'Tab'});await settle();
 report.keyboardFocus=await win.webContents.executeJavaScript(`(()=>{const node=document.activeElement,style=getComputedStyle(node);return {tag:node.tagName,id:node.id,text:node.textContent.trim().slice(0,80),visible:node.matches(':focus-visible'),outline:style.outlineStyle,width:style.outlineWidth,shadow:style.boxShadow}})()`);
 report.explicitReducedMotion=await win.webContents.executeJavaScript(`(async()=>{window.studioFixtureSetScene('lifecycle','light','en');const button=document.querySelector('#page .button.primary');button.dispatchEvent(new PointerEvent('pointerdown',{bubbles:true,button:0}));await new Promise(resolve=>setTimeout(resolve,32));const before=button.style.scale;document.documentElement.dataset.reduceMotion='true';await new Promise(resolve=>requestAnimationFrame(()=>requestAnimationFrame(resolve)));const result={enabled:document.documentElement.dataset.reduceMotion==='true',before,after:button.style.scale,transition:getComputedStyle(button).transitionDuration};delete document.documentElement.dataset.reduceMotion;return result;})()`);
 await win.webContents.debugger.sendCommand('Emulation.setEmulatedMedia',{features:[{name:'prefers-reduced-motion',value:'reduce'}]});
 await win.webContents.executeJavaScript(`window.studioFixtureSetScene('lifecycle','light','en')`);await settle();
 report.reducedMotion=await win.webContents.executeJavaScript(`({enabled:matchMedia('(prefers-reduced-motion: reduce)').matches,animations:document.getAnimations().map(animation=>({target:animation.effect?.target?.className||'',duration:animation.effect?.getTiming().duration,iterations:animation.effect?.getTiming().iterations}))})`);
 report.apiCalls=await win.webContents.executeJavaScript('window.serialArm.fixtureCalls()');report.consoleErrors=errors;
 if(errors.length)finding('Renderer console errors',{errors});
 fs.writeFileSync(path.join(output,'report.json'),JSON.stringify(report,null,2));
 const htmlEsc=value=>String(value).replace(/[&<>"']/g,char=>({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;',"'":'&#39;'}[char]));
 fs.writeFileSync(path.join(output,'index.html'),`<!doctype html><meta charset="utf-8"><title>SerialArm renderer preview</title><style>body{margin:24px;background:#eef1f5;color:#192331;font:14px/1.5 system-ui}main{display:grid;grid-template-columns:repeat(auto-fit,minmax(420px,1fr));gap:20px}figure{margin:0}img{width:100%;display:block;border:1px solid #c8d0db}figcaption{padding:8px 0}a{color:inherit}</style><h1>SerialArm renderer preview</h1><p>Read-only fixture; ${htmlEsc(report.renders.length)} scenes; ${htmlEsc(report.modelPreview)}; <a href="report.json">Full report</a></p><main>${report.renders.map(item=>`<figure><a href="${htmlEsc(path.basename(item.captures[0].file))}"><img loading="lazy" src="${htmlEsc(path.basename(item.captures[0].file))}"></a><figcaption>${htmlEsc(`${item.scene} ${item.width}×${item.height} ${item.theme} ${item.language}`)}</figcaption></figure>`).join('')}</main>`);

 console.log((report.findings.length?'STUDIO_UI_RENDER_FINDINGS ':'STUDIO_UI_RENDER_PASS ')+JSON.stringify({checks:checks.length,renders:report.renders.length,findings:report.findings.length,report:path.join(output,'report.json')}));clearTimeout(deadline);win.destroy();app.quit();
}).catch(error=>{console.error(error.stack);app.exit(1);});
