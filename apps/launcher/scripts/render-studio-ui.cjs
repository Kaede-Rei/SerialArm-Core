/* Whole production renderer with an isolated, deny-by-default fixture API.
 * DISPLAY=:1 node_modules/.bin/electron --no-sandbox scripts/render-studio-ui.cjs \
 *   --render-only --sizes 1300x900,1000x720 --languages zh-CN,en \
 *   --scenes start,lifecycle,parameters-motion,model,workbench,calibration-result,settings \
 *   --themes light,dark --out /tmp/serialarm-studio-ui/screenshots
 * --with-model reads builtin description through the offline model executable;
 * --rescue-model inspects its real structure without claiming 3D rendering.
 * --allow-findings captures all pages while retaining issues in report.json.
 * --software explicitly selects the bundled SwiftShader preview backend.
 * --scale 2 checks high-DPI output; --collision-geometry uses actual STL visuals.
 */
const {app,BrowserWindow}=require('electron');
const fs=require('node:fs');
const path=require('node:path');
const {buildFixture}=require('./studio-ui-fixture.cjs');
const args=process.argv.slice(2),arg=(name,fallback)=>{const i=args.indexOf(name);return i<0?fallback:args[i+1];};
const output=path.resolve(arg('--out','/tmp/serialarm-studio-ui/screenshots'));
const fixture=buildFixture(path.dirname(output),{receipt:arg('--receipt',''),model:args.includes('--with-model'),commissioning:true});
if(args.includes('--collision-geometry')){
 const dataPath=path.join(fixture.directory,'data.json'),data=JSON.parse(fs.readFileSync(dataPath,'utf8'));
 if(!data.model)throw Error('--collision-geometry requires --with-model');
 for(const link of data.model.model.links)link.visuals=structuredClone(link.collisions||[]);
 fs.writeFileSync(dataPath,JSON.stringify(data));
}
const auditFile=arg('--clip-audit','');
const audit=auditFile?fs.readFileSync(auditFile,'utf8'):null;
const scenes=arg('--scenes','start,library,lifecycle,parameters-model,parameters-mapping,parameters-motion,parameters-safety,parameters-shutdown,parameters-hardware,model,workbench,geometry,calibration-teaching,calibration-gain,calibration-gravity,calibration-dynamics,calibration-result,diagnostics,run,settings').split(',');
const widths=arg('--widths','1300,1000').split(',').map(Number),themes=arg('--themes','light,dark').split(',');
const sizes=arg('--sizes','').split(',').filter(Boolean).map(value=>value.split('x').map(Number));
if(!sizes.length)for(const width of widths)sizes.push([width,Number(arg('--height','900'))]);
if(sizes.some(size=>size.length!==2||size.some(value=>!Number.isFinite(value)||value<500)))throw Error('Expected desktop dimensions such as --sizes 1300x900,1000x720');
const languages=arg('--languages','zh-CN').split(',');
const deviceScale=Number(arg('--scale','1'));
if(!Number.isFinite(deviceScale)||deviceScale<=0)throw Error('Expected a positive --scale value');
app.commandLine.appendSwitch('force-device-scale-factor',String(deviceScale));
app.setPath('userData',path.join(fixture.directory,'electron-userdata'));
if(args.includes('--software')){app.commandLine.appendSwitch('use-gl','angle');app.commandLine.appendSwitch('use-angle','swiftshader');app.commandLine.appendSwitch('enable-unsafe-swiftshader');}
function setScene(scene,theme,language='zh-CN'){
 const data=window.studioUiData;
 if(window.studioFixtureRescue)graphicsState.effectiveMode='rescue';
 Object.assign(state,{page:['lifecycle-path','lifecycle-task','lifecycle-observation'].includes(scene)?'lifecycle':scene.startsWith('parameters-')?'profile_setup':scene.startsWith('calibration-')?'calibration':scene.startsWith('workbench-')?'workbench':scene,theme,language,config:{profile:data.editor.profile,profile_file:'/offline/robot_profiles.yaml',serial_port:'',baudrate:'',bus:'',resource_paths:''},info:structuredClone(data.info),profiles:[{name:data.editor.profile}],library:structuredClone(data.library),profileEditor:structuredClone(data.editor),capabilityProfile:structuredClone(data.editor),profileGroup:scene.split('-')[1]||'model',profileDirty:false,profileParkingPath:null,profileParkingKey:'',profileRawInputs:{},profileFieldErrors:{},readiness:{state:'configuring',checks:{},evidence:{},control_available:true},commissioningDrafts:{},commissioningResults:{},commissioningContext:null,modelData:data.model?structuredClone(data.model):null,modelPositions:data.model?.native?.state?.positions?.slice()||[],modelSelection:data.model?.model?.links?.[0]?.name||'',telemetry:null,session:{state:'idle'},admittance:null,calibrationEntry:scene==='calibration-teaching'?'teaching':'gain',calibrationToolSelected:{teaching:null,gain:null,gravity:null,dynamics:null},calibrationRuntimeCandidates:{},modelCalibrationRecords:[],gainMode:'all',gainTuningMode:'quick',gainFinePurpose:'rigid',gainRequirements:{},teachTrajectories:[],geometryTrajectories:[],driverInspection:{plugin:'serial_arm_hardware_damiao',available:true,drivers:[{plugin:'serial_arm_hardware_damiao',available:true,label:'达妙 / Damiao'}]}});
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
 if(['lifecycle','lifecycle-path','lifecycle-task','lifecycle-observation'].includes(scene)){
  state.page='lifecycle';state.config={...state.config,...data.commissioning.config};state.readiness=structuredClone(data.commissioning.MIT);
  state.profileEditor=structuredClone(data.commissioning.editor);state.capabilityProfile=structuredClone(data.commissioning.editor);
  if(scene==='lifecycle-observation'){state.readiness=structuredClone(data.commissioning.CSP);state.profileEditor=structuredClone(data.commissioning.editorCSP);state.capabilityProfile=structuredClone(data.commissioning.editorCSP);}
 }
 if(['calibration-observation','calibration-response'].includes(scene)){state.calibrationEntry=scene.split('-')[1];state.profileEditor.core.control.runtime.control_path='CSP';state.capabilityProfile.core.control.runtime.control_path='CSP';}
 if(scene==='workbench'||scene.startsWith('workbench-')){state.workbenchTab=scene.split('-')[1]||'control';state.controlSettingsOpen=false;}
 if(data.receipt){state.readiness.gain_initialization=structuredClone(data.receipt);state.config.profile=data.receiptLabel;}
 renderedPage='';renderedPageViewKey='';shell();
 if(scene==='lifecycle-observation'){state.commissioningViewedPath='CSP';render();document.querySelector('[data-commissioning-group=path] > details').open=true;document.querySelector('[data-commissioning-task=observation_validation]').closest('details').open=true;}
 if(scene==='lifecycle-task'){document.querySelector('[data-commissioning-group=foundation] > details').open=true;document.querySelector('[data-commissioning-task=model_connection]').closest('details').open=true;}
 if(scene==='lifecycle-path'){state.commissioningViewedPath='CSP';render();document.querySelector('[data-commissioning-group=path] > details').open=true;}
 if(scene.startsWith('parameters-shutdown-')){const path=scene.endsWith('csp')?'CSP':'MIT';state.profileParkingPath=path;if(scene.endsWith('disabled'))state.profileEditor.core.shutdown.paths[path].enabled=false;render();}
 if(scene==='parameters-csp'){state.profileEditor.core.control.runtime.control_path='CSP';state.profileGroup='motion';render();}
 if(scene==='parameters-motion')document.querySelector('[data-gain-purpose="RIGID"]').open=true;
}
function snapshot(){
 const page=document.querySelector('#page'),texts=[...page.querySelectorAll('h2,h3,h4,summary')].map(n=>n.textContent.trim());
 const visibleText=[...page.querySelectorAll('*')].filter(n=>{const r=n.getBoundingClientRect(),style=getComputedStyle(n);return r.width&&r.height&&r.top<innerHeight&&r.bottom>0&&style.visibility!=='hidden'&&([...n.childNodes].some(child=>child.nodeType===3&&child.textContent.trim())||n.matches('input:not([type=checkbox]):not([type=radio]),textarea,select'));});
 const sizes=[...new Set(visibleText.map(n=>getComputedStyle(n).fontSize))];
 const outer=[...document.querySelectorAll('.content,.studio-page-body')].filter(n=>n.scrollWidth>n.clientWidth+1).map(n=>({class:n.className,width:n.clientWidth,scroll:n.scrollWidth}));
 const navigationLabelOverflow=[...page.querySelectorAll('.workspace-subpage-nav .calibration-entry-title')].filter(node=>node.getBoundingClientRect().left+node.scrollWidth>node.closest('button').getBoundingClientRect().right+2).map(node=>({text:node.textContent.trim(),width:node.clientWidth,scroll:node.scrollWidth,buttonWidth:node.closest('button').getBoundingClientRect().width}));
 const buttons=[...page.querySelectorAll('.button.primary')].filter(n=>n.getClientRects().length).map(n=>({id:n.id,text:n.textContent.trim(),disabled:n.disabled,color:getComputedStyle(n).color,background:getComputedStyle(n).backgroundColor}));
 const paletteProbe=document.createElement('i');paletteProbe.hidden=true;paletteProbe.style.cssText='color:var(--primary-on);background-color:var(--primary)';document.body.append(paletteProbe);
 const primaryPalette={color:getComputedStyle(paletteProbe).color,background:getComputedStyle(paletteProbe).backgroundColor};paletteProbe.remove();
 const unusualFonts=visibleText.filter(n=>parseFloat(getComputedStyle(n).fontSize)<12).map(n=>({tag:n.tagName,class:n.className,id:n.id,size:getComputedStyle(n).fontSize}));
 const parse=value=>{const parts=value.match(/[\d.]+/g)?.map(Number)||[];if(parts.length<3)return [0,0,0,0];const channels=parts.slice(0,3).map(channel=>value.startsWith('color(srgb ')?channel*255:channel);return [...channels,parts[3]??1];};
 const blend=(front,back)=>front.slice(0,3).map((channel,index)=>channel*front[3]+back[index]*(1-front[3]));
 const luminance=rgb=>rgb.map(value=>{const c=value/255;return c<=.04045?c/12.92:((c+.055)/1.055)**2.4;}).reduce((v,c,i)=>v+c*[.2126,.7152,.0722][i],0);
 const contrast=[...page.querySelectorAll('h2,h3,label,.hint,.pill,.eyebrow,.gain-section-label,.parameter-key,.breadcrumb')].filter(node=>{const r=node.getBoundingClientRect();return node.innerText?.trim()&&r.width&&r.top<innerHeight&&r.bottom>0&&!node.closest(':disabled,[aria-disabled="true"]');}).slice(0,100).map(node=>{
  let background=document.documentElement.classList.contains('light')?[255,255,255]:[0,0,0];const ancestors=[];
  for(let parent=node;parent;parent=parent.parentElement)ancestors.push(parent);
  for(const parent of ancestors.reverse())background=blend(parse(getComputedStyle(parent).backgroundColor),background);
  const style=getComputedStyle(node),foreground=blend(parse(style.color),background),lum=[luminance(foreground),luminance(background)].sort((a,b)=>b-a);
  return {tag:node.tagName,class:node.className,text:node.innerText.trim().slice(0,60),size:style.fontSize,color:style.color,ratio:Math.round((lum[0]+.05)/(lum[1]+.05)*100)/100};
 });
 return {title:texts,fonts:sizes,smallText:unusualFonts,contrast,unusualFonts,outerOverflow:outer,navigationLabelOverflow,pageWidth:page.getBoundingClientRect().width,viewport:innerWidth,devicePixelRatio,bodyWidth:document.documentElement.scrollWidth,buttons,primaryPalette,middleDots:/\s·\s/.test(page.innerText),parameterKeys:page.querySelectorAll('.parameter-key').length};
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
 document.querySelector('[data-profile-group=safety]').click();
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
async function testExecutionParameters(){
 const checks=[],assert=(ok,name)=>{if(!ok)throw Error(name);checks.push(name);},field=path=>document.querySelector('[data-profile-field='+JSON.stringify(JSON.stringify(path))+']');
 setScene('parameters-motion','light');
 const group=document.querySelector('[data-profile-group=motion]');
 assert(group.textContent==='执行路径与响应'&&!document.querySelector('[data-profile-group=execution]'),'one execution-path-and-response navigation entry');
 const editor=state.profileEditor,inspector=document.querySelector('.setup-inspector'),checksNode=document.querySelector('#profile-guide-title');
 const runtime=['core','control','runtime'],gain=['core','control','controller','rigid','kp','joint1'],mass=['core','capability','admittance','controller','mass','joint1'];
 for(const name of ['control_path','write_enabled','ctrl_frequency_hz'])assert(!!field([...runtime,name]),name+' is edited on the response page');
 const set=(path,value,type='input')=>{const node=field(path);node.value=value;node.dispatchEvent(new Event(type,{bubbles:true}));};
 set(gain,'123.456');set([...runtime,'ctrl_frequency_hz'],'220');
 const scroller=document.querySelector('.studio-page-body'),top=Math.min(140,scroller.scrollHeight-scroller.clientHeight);scroller.scrollTop=top;
 set([...runtime,'control_path'],'CSP','change');
 assert(state.profileGroup==='motion'&&field(mass)&&!document.querySelector('[data-gain-purpose]'),'CSP selection immediately replaces MIT gains with MDK on the same page');
 assert(state.profileEditor===editor&&state.profileEditor.core.control.controller.rigid.kp.joint1===123.456&&field([...runtime,'ctrl_frequency_hz']).value==='220','path switch preserves the authoritative draft and common runtime edits');
 assert(document.querySelector('.setup-inspector')===inspector&&document.querySelector('#profile-guide-title')!==checksNode&&Math.abs(scroller.scrollTop-top)<2,'path refresh retains the configuration container and page scrolling');
 set(mass,'1.875');set([...runtime,'control_path'],'MIT','change');
 assert(field(gain).value==='123.456'&&!field(mass),'returning to MIT retains its edited gains');
 set([...runtime,'control_path'],'CSP','change');assert(field(mass).value==='1.875','returning to CSP retains its edited MDK');
 const fields=[...document.querySelectorAll('[data-profile-field]')].map(node=>node.dataset.profileField);
 assert(fields.length===new Set(fields).size&&!document.querySelector('[data-setup-target*="motion"]'),'each field has one editor and the cross-page response button is removed');
 assert(state.capabilityProfile.core.control.runtime.control_path==='MIT','view editing leaves saved path and runtime baseline unchanged');
 for(const group of ['execution','runtime','gains','admittance']){
  const wrapper=document.createElement('div');wrapper.innerHTML=setupActionHtml({page:'profile_setup',group},'Open');
  assert(JSON.parse(wrapper.firstElementChild.dataset.setupTarget).group==='motion',group+' setup action targets the merged page');
 }
 switchProfileGroup('execution');assert(state.profileGroup==='motion'&&field([...runtime,'control_path']),'legacy execution navigation resolves to the merged page');
 const maxScroll=scroller.scrollHeight-scroller.clientHeight;scroller.scrollTop=Math.min(140,maxScroll);const before=scroller.scrollTop;
 switchProfileGroup('safety');switchProfileGroup('motion');await new Promise(resolve=>setTimeout(resolve,900));const head=document.querySelector('.setup-editor-head').getBoundingClientRect(),sticky=document.querySelector('.setup-save-context').getBoundingClientRect();assert(Math.abs(head.top-sticky.bottom-16)<3&&scroller.scrollTop>0,'cross-group navigation anchors the editor heading below the actual sticky save bar');
 state.page='lifecycle';render();await openSetupTarget({page:'profile_setup',group:'execution',field:'core.control.runtime.control_path'});
 assert(state.page==='profile_setup'&&state.profileGroup==='motion'&&field([...runtime,'control_path']),'preparation execution-path action opens the merged editor');
 assert(state.profileEditor.core.capability.admittance.controller.mass.joint1===1.875,'preparation navigation retains the response draft');
 const calls=await api.fixtureCalls();assert(!calls.some(name=>/workbench|profile_editor_save/.test(name)),'path editing sends no hardware command and performs no Profile write');
 return checks;
}
async function testParameterContinuity(){
 const checks=[],assert=(ok,name)=>{if(!ok)throw Error(name);checks.push(name);},wait=ms=>new Promise(resolve=>setTimeout(resolve,ms));
 const settled=async()=>{const start=performance.now();while(document.querySelector('[data-studio-content-enter],[data-studio-layout-resizing]')&&performance.now()-start<3000)await wait(40);};
 const field=path=>document.querySelector('[data-profile-field='+JSON.stringify(JSON.stringify(path))+']');
 const set=(path,value,type='input')=>{const node=field(path);node.value=value;node.dispatchEvent(new Event(type,{bubbles:true}));return node;};
 setScene('parameters-safety','light');state.page='settings';render();state.page='profile_setup';render();
 assert(document.querySelector('.setup-save-context[data-studio-content-enter]')&&document.querySelector('.setup-inspector[data-studio-content-enter]')&&document.querySelector('.setup-main[data-studio-content-enter]'),'entering parameters animates save context, configuration check and editor together');
 await settled();
 assert(!document.querySelector('[data-studio-content-enter]'),'parameter entrance clears all temporary presentation '+JSON.stringify([...document.querySelectorAll('[data-studio-content-enter]')].map(node=>[node.className,node.style.cssText])));
 const scroller=profileScrollport();switchProfileGroup('motion');await settled();
 // Error/detail navigation explicitly reveals a target; ordinary group changes preserve scroll.
 scrollProfileTarget(document.querySelector('.setup-editor-head'));
 const sticky=document.querySelector('.setup-save-context'),head=document.querySelector('.setup-editor-head');
 assert(Math.abs(head.getBoundingClientRect().top-sticky.getBoundingClientRect().bottom-16)<3&&scroller.scrollTop>0,'group switch positions the editing title below the measured sticky height');
 const selector=field(['core','control','runtime','control_path']),main=document.querySelector('.setup-main'),bar=sticky;
 const initial=scroller.scrollTop,frequency=field(['core','control','runtime','ctrl_frequency_hz']);
 set(['core','control','runtime','ctrl_frequency_hz'],'210');await wait(100);
 assert(Math.abs(scroller.scrollTop-initial)<2&&document.querySelector('.setup-main')===main&&!main.dataset.studioContentEnter,'same-group editing preserves its view without entrance replay');
 set(['core','control','runtime','control_path'],'CSP','change');
 assert(field(['core','control','runtime','control_path'])===selector&&document.querySelector('.setup-save-context')===bar,'MIT/CSP switching keeps the path selector and save area mounted');
 assert(document.querySelector('.profile-path-response').dataset.profilePathResponse==='CSP'&&!!document.querySelector('.profile-path-response [data-studio-content-enter]'),'MIT/CSP body uses its own transition key');
 const enabled=['core','capability','admittance','joint_enabled','joint1'],damping=['core','capability','admittance','controller','damping','joint1'];field(enabled).checked=false;field(enabled).dispatchEvent(new Event('change',{bubbles:true}));assert(field(damping).closest('.parameter-field').querySelector('[data-profile-axis-hint]')?.textContent.includes('≥ 0'),'a changed axis moves its differing bound beside the field');set(damping,'0');assert(state.profileEditor.core.capability.admittance.controller.damping.joint1===0&&!field(damping).validationMessage,'disabled yielding axes permit zero damping according to the current draft');field(enabled).checked=true;field(enabled).dispatchEvent(new Event('change',{bubbles:true}));set(damping,'0');assert(!!field(damping).validationMessage,'participating yielding axes require positive damping');set(damping,'6');
 await settled();assert(Math.abs(scroller.scrollTop-initial)<2&&frequency.value==='210','path-only switching retains scroll position and common draft edits');
 const cmd=['core','safety_policy','cmd_vel_scale'],stateVel=['core','safety_policy','state_vel_scale'];switchProfileGroup('safety');await settled();
 set(cmd,'.4');assert(field(stateVel).min==='0.4','related bounds follow the shared draft rather than loaded metadata');
 set(stateVel,'.3');assert(!!field(stateVel).validationMessage&&state.profileEditor.core.safety_policy.state_vel_scale!==.3,'values below a dependent bound stay invalid and do not replace the valid draft');
 set(stateVel,'.5');assert(!field(stateVel).validationMessage,'correcting the value clears field validation');
 const sourceHint=field(stateVel).closest('label').querySelector('[id$="-hint"]').textContent;assert(sourceHint.includes('≥ 0.4'),'field help shows the current dependent numeric bound');
 const margin=['core','safety_policy','position_margin','joint1'];assert(field(margin)?.type==='number'&&field(margin).min==='0','position margins are per-axis numeric inputs with legal bounds');
 assert(document.querySelector('label[title="core.safety_policy.state_vel_scale"]').textContent.includes('反馈超速阈值比例'),'feedback overspeed field explains its actual purpose');
 const guide=document.querySelector('#profile-guide-container'),beforeGuide=guide.getBoundingClientRect().height,bus=Object.keys(state.profileEditor.hardware.buses)[0],device=state.profileEditor.hardware.buses[bus].device;
 state.profileEditor.hardware.buses[bus].device='';refreshProfileGuidance();
 assert(guide.dataset.studioLayoutResizing==='true'&&Math.abs(guide.getBoundingClientRect().height-beforeGuide)<2,'configuration-check structural changes start at the current height');
 await wait(35);state.profileEditor.hardware.buses[bus].device=device;const beforeReverse=guide.getBoundingClientRect().height;refreshProfileGuidance();
 assert(Math.abs(guide.getBoundingClientRect().height-beforeReverse)<2,'reversed check updates continue from the current rendered height');await settled();
 assert(!guide.dataset.studioLayoutResizing&&!guide.style.blockSize&&!document.querySelector('[data-studio-content-enter]'),'ordinary check refresh returns to natural height without replaying page entrance');
 switchProfileGroup('mapping');const direction=field(['core','calibration','joints','joint1','direction']);
 assert(direction.tagName==='SELECT'&&[...direction.options].map(option=>option.value).join(',')==='-1,1','joint direction is a constrained signed choice');
 direction.value='-1';direction.dispatchEvent(new Event('change',{bubbles:true}));assert(state.profileEditor.core.calibration.joints.joint1.direction===-1,'direction selection writes an integer to the draft');
 const ratio=['core','calibration','joints','joint1','pos_ratio'],oldRatio=state.profileEditor.core.calibration.joints.joint1.pos_ratio,gainPath=['core','control','controller','rigid','kp','joint1'],oldGainMax=profileConstraint(gainPath).max;set(ratio,String(oldRatio*2));assert(profileConstraint(gainPath).max===oldGainMax*2,'mapped MIT gain ceilings track the edited mapping ratios');
 switchProfileGroup('hardware');const busId=field(['hardware','damiao','actuators','joint1','motor_id']);
 assert(busId.step==='1'&&busId.min==='1'&&busId.max==='2047','hardware IDs expose integer step and protocol bounds');
 busId.value='1.5';busId.dispatchEvent(new Event('input',{bubbles:true}));assert(!!busId.validationMessage,'fractional hardware IDs are rejected');busId.value='1';busId.dispatchEvent(new Event('input',{bubbles:true}));
 switchProfileGroup('motion');const hz=['core','control','runtime','ctrl_frequency_hz'];set(hz,'0');assert(!!field(hz).validationMessage,'exclusive-positive fields reject zero');set(hz,'200');
 switchProfileGroup('observation');assert(!!field(['core','capability','admittance','calibration','friction','velocity_transition','joint1']),'friction transition uses per-axis SI controls');
 switchProfileGroup('motion');set(['core','control','runtime','control_path'],'MIT','change');document.querySelector('#profile-add-hold-compensation')?.click();assert(!!field(['core','control','hold_compensation','velocity_threshold','joint1'])&&!!field(['core','control','hold_compensation','error_threshold','joint1']),'hold thresholds are per-axis controls');
 switchProfileGroup('shutdown');
 const parking=state.profileEditor.core.shutdown.paths.MIT;parking.enabled=true;render();
 assert(!!field(['core','shutdown','paths','MIT','position_tolerance','joint1'])&&!!field(['core','shutdown','paths','MIT','velocity_tolerance','joint1']),'parking tolerances use the selected path and per-axis fields');
 state.profileEditor.urdf_joint_limits.joint1.type='prismatic';render();assert(field(['core','shutdown','paths','MIT','position_tolerance','joint1']).closest('label').querySelector('[data-parameter-unit]').textContent==='m'&&field(['core','shutdown','paths','MIT','velocity_tolerance','joint2']).closest('label').querySelector('[data-parameter-unit]').textContent==='rad/s','mixed joint types show one unambiguous SI unit per field');
 const calls=await api.fixtureCalls();assert(!calls.some(name=>['workbench_command','workbench_start','workbench_stop','profile_editor_save'].includes(name)),'parameter navigation and feedback copying send no hardware command or Profile write');
 document.documentElement.dataset.reduceMotion='true';await wait(0);switchProfileGroup('safety');assert(!document.querySelector('[data-studio-content-enter]'),'reduced motion finishes parameter changes synchronously');delete document.documentElement.dataset.reduceMotion;
 return checks;
}
async function testParkingPaths(){
 const checks=[],assert=(ok,name)=>{if(!ok)throw Error(name);checks.push(name);},wait=async()=>{const start=performance.now();while(document.querySelector('[data-studio-content-enter],[data-studio-layout-resizing]')&&performance.now()-start<3000)await new Promise(resolve=>setTimeout(resolve,40));};
 const field=path=>document.querySelector('[data-profile-field='+JSON.stringify(JSON.stringify(path))+']');
 const base=path=>['core','shutdown','paths',path],set=(path,value)=>{const node=field(path);if(node.type==='checkbox')node.checked=value;else node.value=String(value);node.dispatchEvent(new Event('change',{bubbles:true}));};
 setScene('parameters-shutdown-mit','light');await wait();
 const editor=state.profileEditor,paths=editor.core.shutdown.paths,execution=editor.core.control.runtime.control_path;
 assert(paths.MIT&&paths.CSP,'actual Profile provides independent MIT and CSP parking schemas');
 paths.MIT.enabled=false;paths.CSP.enabled=false;paths.MIT.park_pos={joint1:.123};paths.CSP.park_pos={joint1:-.456};render();
 assert(!field([...base('MIT'),'strategy'])&&!field([...base('MIT'),'park_pos','joint1']),'disabled parking hides strategy and target');
 set([...base('MIT'),'enabled'],true);await wait();
 assert(paths.MIT.park_pos.joint1===.123&&editor.joint_names.slice(1).every(name=>paths.MIT.park_pos[name]===0),'enabling parking seeds only missing targets with zero and retains configured values');
 assert(field([...base('MIT'),'strategy']).options.length===2&&[...field([...base('MIT'),'strategy']).options].map(item=>item.textContent).join('|')==='全程刚性停放|先柔性再刚性停放','strategy selector describes both actual parking behaviours');
 set([...base('MIT'),'strategy'],'COMPLIANT_THEN_RIGID');set([...base('MIT'),'park_pos','joint1'],'.234');
 const tab=document.querySelector('[data-profile-parking-path=MIT]'),bar=document.querySelector('.setup-save-context'),scroll=profileScrollport();scrollProfileTarget(document.querySelector('.setup-editor-head'));const top=scroll.scrollTop,key=pageViewKey();
 document.querySelector('[data-profile-parking-path=CSP]').click();
 assert(document.querySelector('.profile-parking-path-panel').dataset.profileParkingPathPanel==='CSP'&&pageViewKey()!==key&&!!document.querySelector('.profile-parking-path-panel [data-studio-content-enter]'),'parking subpages use independent presentation keys');
 assert(editor.core.control.runtime.control_path===execution&&state.capabilityProfile.core.control.runtime.control_path===execution,'parking subpage selection leaves execution selection and saved Profile unchanged');
 assert(document.querySelector('[data-profile-parking-path=MIT]')===tab&&document.querySelector('.setup-save-context')===bar,'parking subpage changes retain navigation and the save context');
 set([...base('CSP'),'enabled'],true);set([...base('CSP'),'strategy'],'RIGID');await wait();
 assert(paths.CSP.park_pos.joint1===-.456&&paths.MIT.park_pos.joint1===.234&&paths.MIT.strategy==='COMPLIANT_THEN_RIGID','both path drafts preserve their own targets and strategies');
 assert(Math.abs(scroll.scrollTop-top)<2,'parking subpage switches preserve the editing position');
 const zeroRule=editor.field_constraints['core.shutdown.paths.CSP.park_pos.joint2'];zeroRule.min_base=.2;zeroRule.min=.2;paths.CSP.park_pos.joint2=0;refreshProfileParking();await wait();
 assert(document.querySelector('.profile-parking-path-panel .parameter-error').textContent.includes('joint2')&&paths.CSP.park_pos.joint2===0,'a zero target outside bounds displays a joint-specific error and is never clamped');
 set([...base('CSP'),'park_pos','joint2'],'.3');assert(!document.querySelector('[data-parking-target-errors] .parameter-error'),'correcting a target immediately clears its displayed range error');
 const names=editor.joint_names,positions=names.map((_,index)=>.25+index*.01);state.session={state:'running',mode:'workbench'};state.workbenchPending={kind:''};state.telemetry={feedback_age_ms:0,valid:true,write_enabled:true,robot_state:'ACTIVE',control_path:'CSP',impedance_mode:'RIGID',joint_names:names,joint:{pos:positions}};state.telemetryReceivedAt=Date.now();refreshProfileParking();
 document.querySelector('#profile-park-current').click();
 assert(JSON.stringify(paths.CSP.park_pos)===JSON.stringify(Object.fromEntries(names.map((name,index)=>[name,positions[index]])))&&paths.MIT.park_pos.joint1===.234,'explicit feedback capture affects only the viewed parking draft');
 state.readiness={control_available:true,commissioning:{selected_path:'MIT'},configuration_summary:{parking_by_path:{MIT:{enabled:false,configured:true,strategy:'RIGID'},CSP:{enabled:true,configured:true,strategy:'RIGID'}}},operation_readiness:{parking:{allowed:false,control_path:'MIT'}}};
 assert(workbenchParkingReadiness().allowed===true&&workbenchParkingReadiness().control_path==='CSP','workspace parking selects the actual runtime path instead of the saved path');
 state.readiness.configuration_summary.parking_by_path.CSP.enabled=false;assert(workbenchParkingReadiness().allowed===false,'disabled runtime-path parking blocks the explicit action');
 state.readiness.configuration_summary.parking_by_path.CSP.enabled=true;state.readiness.configuration_summary.parking_by_path.CSP.strategy='COMPLIANT_THEN_RIGID';state.admittance={validation:{observation_validated:true,response_validated:false,test_active:true}};
 assert(!workbenchParkingReadiness().allowed,'temporary finite-K tests do not authorize compliant CSP parking');
 state.admittance.validation.response_validated=true;assert(workbenchParkingReadiness().allowed,'both runtime quality permissions authorize configured CSP compliant parking');
 document.querySelector('[data-profile-parking-path=MIT]').click();await wait();assert(field([...base('MIT'),'park_pos','joint1']).value==='0.234'&&field([...base('MIT'),'strategy']).value==='COMPLIANT_THEN_RIGID','returning to MIT restores the same edited target and strategy');
 const unique=[...document.querySelectorAll('[data-profile-field]')].map(node=>node.dataset.profileField);assert(new Set(unique).size===unique.length,'parking fields have one editor per path');
 set([...base('MIT'),'enabled'],false);await wait();assert(paths.MIT.park_pos.joint1===.234&&!field([...base('MIT'),'strategy']),'turning parking off retains its configured target and strategy');
 assert(!document.querySelector('[data-profile-field*=park_on_exit]')&&!/退出时自动停放|正常断开与退出/.test(document.querySelector('.profile-parking').textContent),'parking editor contains no automatic-exit field or exit section');
 assert(document.querySelector('.setup-editor-head h2').textContent==='停放与退出策略'&&document.querySelector('.profile-parking').textContent.includes('断开工作台和关闭软件都先按此方案停放'),'one parking permission explains manual parking, workspace disconnect and app close');
 const calls=await api.fixtureCalls();assert(!calls.some(name=>/workbench|profile_editor_save/.test(name)),'editing parking sends no hardware command and performs no Profile write');
 state.modelReloadPending=false;state.telemetry.robot_state='ACTIVE';state.readiness.configuration_summary.parking_by_path.CSP.validated=false;
 assert(!workbenchDisconnectProblem(),'active disconnect uses current-path parking permission without an extra receipt gate');
 const activeStop=requestWorkbenchStop();assert(state.parkingBusy===true,'active disconnect exposes the existing interruptible parking controls');await activeStop;assert(state.parkingBusy===false,'parking controls finish with the offline stop acknowledgment');
 state.telemetry.robot_state='INACTIVE';state.readiness.configuration_summary.parking_by_path.CSP.enabled=false;const inactiveStop=requestWorkbenchStop();assert(state.parkingBusy!==true,'inactive disconnect sends stop directly without entering parking');await inactiveStop;
 const stopped=(await api.fixtureCalls()).length;state.telemetry.robot_state='ACTIVE';assert(!!workbenchDisconnectProblem(),'disabled parking requires explicit disable before normal active disconnect');state.readiness.configuration_summary.parking_by_path.CSP.enabled=true;state.modelReloadPending=true;assert(!!workbenchDisconnectProblem(),'changed saved configuration requires explicit disable before active disconnect');state.modelReloadPending=false;state.telemetry.robot_state='FAULT';assert(!!workbenchDisconnectProblem(),'faulted disconnect requires fault handling or explicit disable');
 assert((await api.fixtureCalls()).length===stopped,'blocked normal disconnect checks send no additional request');
 return checks;
}
async function testCommissioningViews(){
 const checks=[],assert=(ok,name)=>{if(!ok)throw Error(name);checks.push(name);};
 setScene('lifecycle','light');
 const key=state.readinessDisclosureKey;localStorage.removeItem(key);state.readinessDisclosureKey='';render();
 assert(document.querySelectorAll('[data-commissioning-group]').length===3,'three commissioning phase cards remain available');
 assert(!document.querySelector('.commissioning-context'),'the redundant commissioning overview card is removed');
 assert([...document.querySelectorAll('[data-commissioning-group] > details')].every(node=>!node.open),'all phase cards initially collapse');
 assert([...document.querySelectorAll('[data-readiness-section^="step:"]')].every(node=>!node.open),'all internal steps initially collapse');
 const titles=[...document.querySelectorAll('[data-commissioning-group] > details > summary .readiness-fold-title')].map(node=>node.textContent);
 assert(titles.join('|')==='一、共同基础|二、路径专项|三、可选提升','phase titles use the requested short Chinese names');
 let foundation=document.querySelector('[data-commissioning-group=foundation] > details');foundation.querySelector('summary').click();await new Promise(r=>setTimeout(r,350));
 assert(foundation.open&&[...foundation.querySelectorAll('[data-readiness-section^="step:"]')].every(node=>!node.open),'opening a parent card leaves all child steps collapsed');
 document.querySelector('[data-readiness-section="step:MIT:model_connection"] > summary').click();await new Promise(r=>setTimeout(r,350));
 assert(JSON.parse(localStorage.getItem(key))['step:MIT:model_connection']===true,'manual child preference is persisted');
 state.readinessDisclosureKey='';state.readinessSections={};render();
 assert(document.querySelector('[data-readiness-section="step:MIT:model_connection"]').open,'manual disclosure preference survives view reload');
 const profile=state.config.profile;state.config.profile='another-fixture';render();assert([...document.querySelectorAll('[data-commissioning-group] > details')].every(node=>!node.open),'another Profile starts collapsed');state.config.profile=profile;render();
 assert(state.readiness.commissioning.groups[0].steps.length===4&&state.readiness.commissioning.paths.MIT.steps.length===3&&state.readiness.commissioning.paths.CSP.steps.length===3,'production API supplies four common tasks and three tasks per control path');
 const group=document.querySelector('[data-commissioning-group=path] > details');group.querySelector('summary').click();await new Promise(r=>setTimeout(r,350));
 assert(document.querySelector('[data-commissioning-path=MIT] small')?.textContent==='当前配置','the configured path is labelled independently of viewed path');
 const config=JSON.stringify(state.profileEditor.core.control.runtime),saved=JSON.stringify(state.capabilityProfile);
 document.querySelector('[data-commissioning-path=CSP]').click();await new Promise(r=>setTimeout(r,350));
 assert(state.commissioningViewedPath==='CSP'&&state.readiness.commissioning.selected_path==='MIT'&&JSON.stringify(state.profileEditor.core.control.runtime)===config&&JSON.stringify(state.capabilityProfile)===saved,'path viewing does not mutate saved configuration or the authoritative draft');
 assert(!!document.querySelector('[data-readiness-section="step:CSP:admittance_response"]')&&!document.querySelector('[data-readiness-section="step:MIT:gravity_verification"]'),'path viewing replaces only the relevant commissioning steps');
 const currentPath=document.querySelector('[data-commissioning-group=path]');
 assert(!currentPath.querySelector('[data-commissioning-save]')&&[...currentPath.querySelectorAll('[data-setup-target]')].every(node=>JSON.parse(node.dataset.setupTarget).group==='motion'),'an unconfigured path cannot save records or issue experiment actions');
 assert([...currentPath.querySelectorAll('[data-readiness-section^="step:"]')].every(node=>!node.open),'first view of another path leaves its steps collapsed');
 assert(document.querySelector('[data-commissioning-path=MIT] small')?.textContent==='当前配置','current configuration marker stays on MIT while viewing CSP');
 document.querySelector('[data-commissioning-path=MIT]').click();await new Promise(r=>setTimeout(r,350));assert(!!document.querySelector('[data-readiness-section="step:MIT:gravity_verification"]'),'switching back restores MIT commissioning steps');
 localStorage.removeItem(key);state.readinessDisclosureKey='';render();
 return checks;
}
function testCommissioningCopy(){
 setScene('lifecycle-observation','light');const task=document.querySelector('[data-commissioning-task=observation_validation]'),text=task.textContent,checks=[];
 if(!text.includes('待完成：独立外力实测验收通过'))throw Error('a pending measured condition must be explicitly required, not stated as passed');checks.push('pending measurement copy is explicitly marked as a remaining condition');
 if((text.match(/项人工检查未确认/g) || []).length!==1)throw Error('remaining operator checks must be shown only once');checks.push('remaining operator check count is not duplicated');
 return checks;
}
async function testCommissioningCompletion(){
 const checks=await window.studioFixtureTestCommissioningViews(),assert=(ok,name)=>{if(!ok)throw Error(name);checks.push(name);};
 const wait=async predicate=>{for(let i=0;i<200;i++){if(predicate())return;await new Promise(r=>setTimeout(r,10));}throw Error('Commissioning interaction did not finish');};
 setScene('lifecycle','light');
 state.commissioningDrafts={};state.commissioningResults={};state.commissioningContext=null;
 state.readiness=await api.request('readiness',{config:state.config});render();
 const task=id=>document.querySelector('[data-commissioning-task="'+id+'"]'),step=id=>commissioningTask(id);
 assert(!document.querySelector('.stage-evidence')&&!document.querySelector('.readiness-tools-and-records')&&!document.querySelector('[data-review-stage]'),'tasks own their conditions and results without duplicate four-column or record lists');
 assert([...task('model_connection').querySelectorAll('h4')].map(node=>node.textContent).join('|')==='怎么做|完成条件|操作与结果','every expanded task has procedure, completion conditions and actions/results');
 assert(commissioningTaskStatusHtml({completed:false,evidence_status:'stale'}).includes('需复查')&&commissioningTaskStatusHtml({completed:false,evidence_status:'failed'}).includes('未通过')&&commissioningTaskStatusHtml({completed:false,evidence_status:'partial'}).includes('进行中'),'collapsed task status presents backend stale, failed and partial progress');
 let form=task('model_connection'),boxes=[...form.querySelectorAll('[data-commissioning-check]')];
 boxes[0].checked=true;boxes[0].dispatchEvent(new Event('change',{bubbles:true}));
 form.querySelector('[data-commissioning-save]').click();await wait(()=>!state.blockingBusy);
 assert(step('model_connection').source.historical.status==='partial'&&!step('model_connection').completed,'saving one real backend answer stores partial progress without completing the task');
 assert(task('model_connection').textContent.includes('尚有')&&task('model_connection').querySelector('[data-commissioning-check]').checked,'partial result lists remaining conditions and restores the saved checkbox');
 form=task('model_connection');for(const node of form.querySelectorAll('[data-commissioning-check]')){node.checked=true;node.dispatchEvent(new Event('change',{bubbles:true}));}
 form.querySelector('[data-commissioning-save]').click();await wait(()=>!state.blockingBusy);
 assert(step('model_connection').completed&&task('model_connection').textContent.includes('此任务已完成'),'all checked conditions complete a manual task through the real readiness_record API');
 state.commissioningDrafts={};state.readiness=await api.request('readiness',{config:state.config});render();
 assert(step('model_connection').completed&&[...task('model_connection').querySelectorAll('[data-commissioning-check]')].every(node=>node.checked),'persisted task results survive backend reload');
 state.session={state:'running',mode:'workbench'};state.telemetry={valid:true,feedback_age_ms:0,write_enabled:true,hardware_connected:true,sequence:1};state.telemetryReceivedAt=Date.now();refreshLifecycleBringup();
 const live=[...document.querySelectorAll('[data-readiness-live=feedback]')];assert(live.length>=2&&live.every(node=>node.textContent==='反馈已更新'),'every feedback condition updates from the current telemetry');
 state.telemetry.hardware_connected=false;refreshLifecycleBringup();assert(live.every(node=>node.textContent==='硬件未连接'),'connection loss updates every repeated feedback indicator');
 state.session={state:'idle'};state.telemetry=null;
 const action=JSON.parse(task('mapping_safety').querySelector('[data-setup-target]').dataset.setupTarget);await openSetupTarget(action);
 assert(state.page==='profile_setup'&&!!document.querySelector('[data-commissioning-return]')&&document.querySelector('.commission-context').textContent.includes('关节映射与安全边界'),'task tools retain their exact commissioning context');
 await returnToCommissioning();await new Promise(r=>requestAnimationFrame(r));
 assert(state.page==='lifecycle'&&task('mapping_safety').closest('details').open&&document.querySelector('[data-commissioning-group=foundation] > details').open,'returning from a tool refreshes and opens its exact task');
 state.readiness=await api.fixtureReadiness('select_path',{path:'CSP'});state.commissioningViewedPath='CSP';state.profileEditor=await api.request('profile_editor_load',{config:state.config});state.capabilityProfile=structuredClone(state.profileEditor);state.commissioningDrafts={};render();
 form=task('observation_validation');assert(form.querySelector('[data-commissioning-save]')&&form.querySelector('.commission-conditions').textContent.includes('实测'),'effort acceptance combines measured results and operator direction checks in one task');
 for(const node of form.querySelectorAll('[data-commissioning-check]')){node.checked=true;node.dispatchEvent(new Event('change',{bubbles:true}));}
 form.querySelector('[data-commissioning-save]').click();await wait(()=>!state.blockingBusy);
 assert(!step('observation_validation').completed&&state.readiness.configuration_summary.validation.observation_validated!==true,'manual answers cannot replace independent effort measurements');
 state.readiness=await api.fixtureReadiness('observation',{passed:false});render();
 assert(!step('observation_validation').completed&&task('observation_validation').textContent.includes('实测尚未通过'),'a failed native-fixture receipt is shown as not passed');
 const acceptedProof=await api.fixtureReadiness('observation',{passed:true});state.observationAcceptance=acceptedProof;state.readiness=acceptedProof;state.commissioningDrafts={};render();
 assert(step('observation_validation').completed&&task('observation_validation').textContent.includes('实测结果已回填并通过验收'),'passing independent measurements auto-fill the existing mixed task');
 let response=await api.fixtureReadiness('response',{axis:0});state.readiness=response.readiness;render();
 assert(!step('admittance_response').completed&&!task('admittance_response').querySelector('[data-commissioning-save]'),'response acceptance is measured automatically without a redundant operator checklist');
 assert(task('admittance_response').textContent.includes('所需关节')&&task('admittance_response').textContent.includes('尚缺关节')&&task('admittance_response').textContent.includes('joint2'),'partial response coverage names the required and missing joints');
 response=await api.fixtureReadiness('response',{axis:1});state.readiness=response.readiness;render();
 assert(step('admittance_response').completed&&task('admittance_response').textContent.includes('实测结果已回填并通过验收'),'covering the remaining enabled joint completes the measured task');
 assert(state.readiness.configuration_summary.validation.observation_validated===true&&state.readiness.configuration_summary.validation.response_validated===true,'completed response retains both measured permissions before changing confirmation');
 form=task('observation_validation');const direction=[...form.querySelectorAll('[data-commissioning-check]')].find(node=>node.dataset.commissioningCheck.endsWith('__direction'));direction.checked=false;direction.dispatchEvent(new Event('change',{bubbles:true}));form.querySelector('[data-commissioning-save]').click();await wait(()=>!state.blockingBusy);
 assert(!step('observation_validation').completed&&state.readiness.configuration_summary.validation.observation_validated===false&&state.readiness.configuration_summary.validation.response_validated===false,'withdrawing a required direction confirmation revokes effort and response permissions');
 assert(task('observation_validation').textContent.includes('独立实测已通过')&&measurementFeedbackHtml(state.observationAcceptance,'observation').includes('无需重复采样')&&!measurementFeedbackHtml(state.observationAcceptance,'observation').includes('实测验收通过'),'a passed measurement awaiting operator confirmation never claims full acceptance and does not request resampling');
 assert(!document.querySelector('.readiness-workspace').textContent.includes('打开此步骤')&&!document.querySelector('.readiness-workspace').textContent.includes('尚未验收'),'task copy uses concrete actions and relevant completion conditions');
 return checks;
}
async function testCommissioning(){
 const checks=await window.studioFixtureTestCommissioningViews(),assert=(ok,name)=>{if(!ok)throw Error(name);checks.push(name);};
 setScene('calibration-observation','light');
 assert(!document.querySelector('[data-calibration-entry=gain]')&&!!document.querySelector('[data-calibration-entry=response]'),'CSP tools expose admittance tuning and omit MIT gain calibration');
 assert(!!document.querySelector('[data-calibration-begin=static]')&&!!document.querySelector('#calibration-capture'),'effort calibration owns its sampling controls');
 assert(!!document.querySelector('[data-profile-field]')&&document.querySelectorAll('[data-profile-field]').length===new Set([...document.querySelectorAll('[data-profile-field]')].map(n=>n.dataset.profileField)).size,'effort parameter draft uses unique shared fields');
 state.calibrationEntry='gravity';render();assert(!document.querySelector('#calibration-capture'),'gravity correction no longer hides effort calibration controls');
 setScene('calibration-response','light');
 const mass=document.querySelector('[data-profile-field='+JSON.stringify(JSON.stringify(['core','capability','admittance','controller','mass','joint1']))+']');
 assert(!!mass&&!!document.querySelector('#capability-response-chart'),'MDK editing and response recording share one workspace');
 mass.value='1.75';mass.dispatchEvent(new Event('input',{bubbles:true}));
 assert(state.profileEditor.core.capability.admittance.controller.mass.joint1===1.75,'response tuning edits the authoritative Profile draft');
 assert(document.querySelector('#admittance-validation-start').disabled,'bounded test remains unavailable without backend observation validation');
 state.session={state:'running',mode:'workbench'};state.telemetry={robot_state:'ACTIVE',control_path:'CSP'};state.readiness={modes_by_path:{CSP:{COMPLIANT:'configured',DRAG:'configured'}}};state.admittance={validation:{observation_validated:false,response_validated:false}};assert(!controlGainPurposeMayChange('COMPLIANT')&&!controlGainPurposeMayChange('DRAG'),'ordinary compliant and drag mode require measured backend permissions');state.admittance.validation={observation_validated:true,response_validated:true};assert(controlGainPurposeMayChange('COMPLIANT'),'measured backend permissions permit configured compliant mode');
 state.telemetry={joint_names:state.profileEditor.joint_names,calibration:{kind:'validation',phase:'collecting',captured:2,expected:5}};refreshObservationCalibrationControls();
 setScene('calibration-observation','light');state.telemetry.calibration={kind:'validation',phase:'collecting',captured:2,expected:5};refreshObservationCalibrationControls();
 assert(document.querySelector('#calibration-capture').disabled&&document.querySelector('#calibration-finish').disabled&&!document.querySelector('#calibration-cancel').disabled,'static capture prevents duplicate samples while cancellation remains reachable');
 return checks;
}
async function testCalibrationNavigation(){
 const checks=[],moves=[],assert=(ok,label)=>{if(!ok)throw Error(label);checks.push(label);};
 const current=()=>document.querySelector('.calibration-subpage-nav');
 const visible=()=>{const nav=current(),item=nav.querySelector('[aria-current="page"]'),a=nav.getBoundingClientRect(),b=item.getBoundingClientRect();return b.left>=a.left-1&&b.right<=a.right+1;};
 assert(state.calibrationEntry==='dynamics'&&visible(),'initial full-dynamics tab is revealed inside the narrow navigation');
 assert(!current().querySelector('.workspace-subpage-status'),'calibration navigation has no verification status dot');
 for(const entry of ['gravity','gain','dynamics']){
  const nav=current(),button=nav.querySelector(`[data-calibration-entry="${entry}"]`),a=nav.getBoundingClientRect(),b=button.getBoundingClientRect(),before=nav.scrollLeft;
  const adjustment=b.left<a.left?b.left-a.left:b.right>a.right?b.right-a.right:0;
  const expected=Math.max(0,Math.min(nav.scrollWidth-nav.clientWidth,before+adjustment));
  button.click();await new Promise(resolve=>setTimeout(resolve,350));
  const after=current().scrollLeft;moves.push({entry,before,after,expected});
  assert(state.calibrationEntry===entry&&visible()&&Math.abs(after-expected)<2,`${entry}: switching tabs preserves horizontal position except the nearest reveal`);
 }
 const nav=current();nav.scrollLeft=Math.max(0,nav.scrollLeft-20);const before=nav.scrollLeft;
 refreshWorkspacePanel();await new Promise(resolve=>setTimeout(resolve,200));
 assert(Math.abs(current().scrollLeft-before)<2,'ordinary workspace refresh retains navigation scrolling');
 const body=document.querySelector('.studio-page-body');assert(body.scrollWidth<=body.clientWidth+1,'navigation scroll remains inside the task panel');
 return {checks,moves};
}
app.whenReady().then(async()=>{
 fs.mkdirSync(output,{recursive:true});
 const win=new BrowserWindow({width:1280,height:1000,show:false,webPreferences:{preload:fixture.preload,sandbox:false,backgroundThrottling:false}});
 win.webContents.on('render-process-gone',(_event,details)=>{console.error('Fixture renderer exited',JSON.stringify(details));app.exit(1);});
 const deadline=setTimeout(()=>{console.error('UI fixture exceeded its bounded render time');app.exit(1);},Number(arg('--timeout','120000')));
 const errors=[];win.webContents.on('console-message',(_e,level,message)=>{if(level>=3){errors.push(message);console.error('RENDERER',message);}});
 const settle=()=>win.webContents.executeJavaScript(`(async()=>{await document.fonts.ready;await new Promise(resolve=>setTimeout(resolve,220));const start=performance.now();while(document.querySelector('[data-studio-content-enter],[data-studio-layout-resizing]')&&performance.now()-start<2000)await new Promise(resolve=>requestAnimationFrame(resolve));await new Promise(resolve=>requestAnimationFrame(resolve));const modelStart=performance.now();while(state.modelData&&!window.studioFixtureRescue&&(state.page==='model'||workspacePreviewVisible())&&(!modelView?.modelLoaded||modelView.paused)&&performance.now()-modelStart<10000)await new Promise(resolve=>setTimeout(resolve,50));})()`);
 await win.loadFile(fixture.index);win.show();win.focus();win.webContents.focus();await settle();
 win.webContents.debugger.attach('1.3');await win.webContents.debugger.sendCommand('DOM.enable');await win.webContents.debugger.sendCommand('CSS.enable');
 // Terminal emulation is covered by the desktop smoke test. This fixture
 // replaces the shell repeatedly without starting a runtime or fitting xterm.
 await win.webContents.executeJavaScript('term?.dispose();term=null;fit=null;window.Terminal=undefined;void 0;');
 await win.webContents.executeJavaScript(`window.studioFixtureRescue=${args.includes('--rescue-model')};window.setScene=window.studioFixtureSetScene=${setScene};window.studioFixtureSnapshot=${snapshot};window.studioFixtureTestParameters=${testParameters};window.studioFixtureTestCommissioningViews=${testCommissioningViews};window.studioFixtureTestExecutionParameters=${testExecutionParameters};window.studioFixtureTestParameterContinuity=${testParameterContinuity};window.studioFixtureTestParkingPaths=${testParkingPaths};window.studioFixtureTestCommissioningCompletion=${testCommissioningCompletion};window.studioFixtureTestCommissioningCopy=${testCommissioningCopy};void 0;`);
 const checks=args.includes('--parking-only')||args.includes('--parameter-continuity-only')||args.includes('--render-only')||args.includes('--commissioning-only')||args.includes('--execution-only')||args.includes('--completion-only')||args.includes('--completion-copy-only')?[]:await win.webContents.executeJavaScript('(()=>{try{return window.studioFixtureTestParameters();}catch(error){return {error:error.stack};}})()');
 if(checks.error)throw Error(checks.error);
 if(args.includes('--parking-only'))checks.push(...await win.webContents.executeJavaScript('window.studioFixtureTestParkingPaths()'));
 else if(args.includes('--parameter-continuity-only'))for(const [width,height] of sizes){win.setContentSize(width,height);await settle();checks.push(...(await win.webContents.executeJavaScript('window.studioFixtureTestParameterContinuity()')).map(name=>`${width}x${height}: ${name}`));}
 else if(args.includes('--completion-copy-only'))checks.push(...await win.webContents.executeJavaScript('window.studioFixtureTestCommissioningCopy()'));
 else if(args.includes('--completion-only'))checks.push(...await win.webContents.executeJavaScript('window.studioFixtureTestCommissioningCompletion()'));
 else if(args.includes('--execution-only'))checks.push(...await win.webContents.executeJavaScript('window.studioFixtureTestExecutionParameters()'));
 else if(args.includes('--commissioning-only'))checks.push(...await win.webContents.executeJavaScript('window.studioFixtureTestCommissioningViews()'));
 else if(!args.includes('--render-only'))checks.push(...await win.webContents.executeJavaScript(`(${testCommissioning})()`));
 const report={runtime:process.versions.electron,deviceScale,graphicsBackend:args.includes('--software')?'bundled SwiftShader':'normal',modelPreview:args.includes('--rescue-model')?'rescue without 3D':args.includes('--collision-geometry')?'actual URDF collision STL as visuals':args.includes('--with-model')?'read-only native model':'not loaded',findings:[],source:'complete production index.html and app.js, read-only fixture API',checks,renders:[]};
 const runAudit=()=>win.webContents.executeJavaScript(`(${audit})({root:'#app',viewport:true})`);
 const finding=(name,evidence)=>{report.findings.push({name,...evidence});if(!args.includes('--allow-findings'))throw Error(name+' '+JSON.stringify(evidence));};
 for(const [width,height] of sizes)for(const theme of themes)for(const language of languages)for(const scene of scenes){
  win.setContentSize(width,height);await win.webContents.debugger.sendCommand('Emulation.setDeviceMetricsOverride',{width,height,deviceScaleFactor:deviceScale,mobile:false});await win.webContents.executeJavaScript(`window.studioFixtureSetScene(${JSON.stringify(scene)},${JSON.stringify(theme)},${JSON.stringify(language)})`);win.webContents.sendInputEvent({type:'mouseMove',x:0,y:0});await settle();
  console.log('STUDIO_SCENE '+JSON.stringify({scene,width,height,theme,language}));
  const dom=await win.webContents.executeJavaScript('window.studioFixtureSnapshot()');
  if(dom.bodyWidth>width||dom.outerOverflow.length)finding('Outer overflow',{scene,width,height,theme,language,dom});
  if(dom.navigationLabelOverflow.length)finding('Subpage label overflow',{scene,width,height,theme,language,labels:dom.navigationLabelOverflow});
  if(dom.buttons.some(b=>!b.disabled&&(b.color!==dom.primaryPalette.color||b.background!==dom.primaryPalette.background)))finding('Primary button palette',{scene,width,height,theme,language,buttons:dom.buttons,expected:dom.primaryPalette});
  const tree=await win.webContents.debugger.sendCommand('DOM.getDocument');const fontNode=await win.webContents.debugger.sendCommand('DOM.querySelector',{nodeId:tree.root.nodeId,selector:'#page .page-title,#page h1,#page h2,#page h3'});const platformFonts=fontNode.nodeId?(await win.webContents.debugger.sendCommand('CSS.getPlatformFontsForNode',{nodeId:fontNode.nodeId})).fonts:[];
  const fontState=await win.webContents.executeJavaScript(`({status:document.fonts.status,check:document.fonts.check('14px "SerialArm Maple Italic"'),faces:[...document.fonts].filter(face=>face.family.includes('SerialArm Maple Italic')).map(face=>({family:face.family,weight:face.weight,style:face.style,status:face.status}))})`);
  if(!fontState.check||!fontState.faces.some(face=>face.status==='loaded')||!platformFonts.some(font=>/MapleMono.*CN.*Italic/i.test(font.postScriptName||'')))finding('Requested Maple Italic font was not used',{scene,width,height,theme,language,platformFonts,fontState});
  const item={scene,width,height,theme,language,dom,platformFonts,fontState,captures:[]};
  const stem=`${scene}-${width}x${height}-${theme}-${language}`;
  if(dom.fonts.some(size=>![12,14,16,20,24].includes(parseFloat(size)))||dom.smallText.length)finding('Visible text scale',{scene,width,height,theme,language,fonts:dom.fonts,smallText:dom.smallText});
  if(audit){item.defaultClip=await runAudit();if(item.defaultClip.cut)finding('Default shadow clip',{scene,width,height,theme,language,clip:item.defaultClip});}
  const scroller=await win.webContents.executeJavaScript(`(()=>{const nodes=[...document.querySelectorAll('.studio-page-body,.content')];const node=nodes.find(n=>n.scrollHeight>n.clientHeight+2)||nodes[0];node.dataset.fixtureScroll='true';return {height:node.scrollHeight,viewport:node.clientHeight};})()`);
  for(const position of [0,Math.max(0,scroller.height-scroller.viewport)]){
   if(position===0&&item.captures.length)continue;
   const scrollTop=await win.webContents.executeJavaScript(`(()=>{const n=document.querySelector('[data-fixture-scroll]');n.scrollTop=${position};return n.scrollTop;})()`);await settle();
   const shot=await win.webContents.capturePage(),png=shot.toPNG();if(shot.isEmpty()||png.length<10000)throw Error('Empty screenshot');
   const file=path.join(output,`${stem}${position?'-bottom':''}.png`);fs.writeFileSync(file,png);item.captures.push({file,scrollTop,bytes:png.length});
  }
  if(['lifecycle-task','lifecycle-observation'].includes(scene)){
   const scrollTop=await win.webContents.executeJavaScript(`(()=>{const task=[...document.querySelectorAll('.commission-task')].find(node=>node.getClientRects().length&&node.closest('details').open),target=task?.querySelector('.commission-task-section:last-child'),scroller=document.querySelector('[data-fixture-scroll]');if(!target)return 0;scroller.scrollTop+=target.getBoundingClientRect().top-scroller.getBoundingClientRect().top-16;return scroller.scrollTop;})()`);await settle();
   const file=path.join(output,`${stem}-actions.png`);fs.writeFileSync(file,(await win.webContents.capturePage()).toPNG());item.captures.push({file,scrollTop});
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
  if(['parameters-motion','parameters-csp','parameters-hardware','parameters-shutdown','parameters-shutdown-empty','parameters-shutdown-mit','parameters-shutdown-csp','parameters-shutdown-disabled','parameters-safety','calibration-result','diagnostics','lifecycle','workbench'].includes(scene)){
   await win.webContents.executeJavaScript(`document.querySelectorAll('#page details').forEach(n=>n.open=true)`);await settle();
   item.expandedDom=await win.webContents.executeJavaScript('window.studioFixtureSnapshot()');
   if(item.expandedDom.outerOverflow.length)finding('Expanded overflow',{scene,width,height,theme,language,dom:item.expandedDom});
   if(audit){item.expandedClip=await runAudit();if(item.expandedClip.cut)finding('Expanded shadow clip',{scene,width,height,theme,language,clip:item.expandedClip});}
   const file=path.join(output,`${stem}-expanded.png`);fs.writeFileSync(file,(await win.webContents.capturePage()).toPNG());item.expandedFile=file;
   const target=scene==='calibration-result'?'[data-gain-model-stages]':scene==='lifecycle'?'[data-commissioning-group=foundation]':scene==='workbench'?'#control-settings':scene.startsWith('parameters-')?'.setup-editor-head':null;
   if(target){const position=await win.webContents.executeJavaScript(`(()=>{const n=document.querySelector(${JSON.stringify(target)}),s=document.querySelector('[data-fixture-scroll]');if(!n)return null;if(state.page==='profile_setup'){scrollProfileTarget(n);return s.scrollTop;}const top=s.getBoundingClientRect().top,occluded=Math.max(top,...[...document.querySelectorAll('#page > .page-head,.workspace-operation-panel')].map(node=>node.getBoundingClientRect().bottom));s.scrollTop+=n.getBoundingClientRect().top-occluded-16;return s.scrollTop;})()`);await settle();const detail=path.join(output,`${stem}-detail.png`);fs.writeFileSync(detail,(await win.webContents.capturePage()).toPNG());item.expandedDetail={file:detail,scrollTop:position};}
  }
  if(audit){await settle();item.finalClip=await runAudit();if(item.finalClip.cut)finding('Final shadow clip',{scene,width,height,theme,language,clip:item.finalClip});}
  report.renders.push(item);
 }
 if(scenes.some(scene=>scene.startsWith('calibration-'))&&sizes.some(([width])=>width===1000)&&languages.includes('en')){
  win.setContentSize(1000,720);await win.webContents.executeJavaScript(`window.studioFixtureSetScene('workbench','light','en');window.studioFixtureSetScene('calibration-dynamics','light','en')`);await settle();
  report.calibrationNavigation=await win.webContents.executeJavaScript(`(${testCalibrationNavigation})()`);
  report.checks.push(...report.calibrationNavigation.checks);
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
