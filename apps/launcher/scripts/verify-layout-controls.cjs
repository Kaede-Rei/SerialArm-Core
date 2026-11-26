/* Production split panels and control grouping, using only the offline fixture.
 * node_modules/.bin/electron --no-sandbox scripts/verify-layout-controls.cjs
 * --collision-geometry uses the same URDF's real collision meshes for constrained renderers.
 */
const assert=require('node:assert/strict'),fs=require('node:fs'),path=require('node:path'),os=require('node:os');
const {app,BrowserWindow}=require('electron');
const {buildFixture}=require('./studio-ui-fixture.cjs');
const args=process.argv.slice(2),at=args.indexOf('--out'),output=at<0?fs.mkdtempSync(path.join(os.tmpdir(),'serialarm-layout-controls-')):path.resolve(args[at+1]);
const fixture=buildFixture(path.join(output,'fixture'),{model:true,commissioning:true});
if(args.includes('--collision-geometry')){const file=path.join(fixture.directory,'data.json'),data=JSON.parse(fs.readFileSync(file,'utf8'));for(const link of data.model.model.links)link.visuals=structuredClone(link.collisions||[]);fs.writeFileSync(file,JSON.stringify(data));}
const render=fs.readFileSync(path.join(__dirname,'render-studio-ui.cjs'),'utf8');
const scene=render.slice(render.indexOf('function setScene('),render.indexOf('function snapshot('));
app.setPath('userData',path.join(fixture.directory,'userdata'));
app.commandLine.appendSwitch('force-device-scale-factor','1');
if(args.includes('--software')){app.commandLine.appendSwitch('use-gl','angle');app.commandLine.appendSwitch('use-angle','swiftshader');app.commandLine.appendSwitch('enable-unsafe-swiftshader');}
const pause=ms=>new Promise(resolve=>setTimeout(resolve,ms));
let auditWindow,auditReport;
app.whenReady().then(async()=>{
 const win=new BrowserWindow({width:1440,height:1100,frame:false,show:true,enableLargerThanScreen:true,webPreferences:{preload:fixture.preload,sandbox:false,backgroundThrottling:false}});
 auditWindow=win;
 const report=auditReport={checks:[],panes:[],states:[],source:'Production renderer with offline fixture; focused native window, pointer/keyboard events with delivery receipts'};
 const check=(ok,name,value)=>{if(!ok)report.failure={name,value};assert(ok,name+' '+JSON.stringify(value||''));report.checks.push(name)};
 const js=code=>win.webContents.executeJavaScript(code);
 const waitFor=async(predicate,label,timeout=4000)=>{const start=Date.now();while(!await predicate()){if(Date.now()-start>timeout)throw Error('Timed out waiting for '+label);await pause(20);}};
 const settle=async()=>{await pause(250);await js(`(async()=>{const start=performance.now();while(state.modelData&&(state.page==='model'||workspacePreviewVisible())&&(!modelView?.modelLoaded||modelView.paused)&&performance.now()-start<10000)await new Promise(r=>setTimeout(r,50));await new Promise(r=>requestAnimationFrame(()=>requestAnimationFrame(r)));})()`)};
 await win.loadFile(fixture.index);win.show();win.focus();win.webContents.focus();await js('document.fonts.ready');await js(`window.layoutInputLog=[];window.layoutInputSequence=0;for(const type of ['pointerdown','pointermove','pointerup','pointercancel','gotpointercapture','lostpointercapture','keydown','keyup'])document.addEventListener(type,e=>{window.layoutInputLog.push({sequence:++window.layoutInputSequence,type,at:Math.round(performance.now()),key:e.key,pointer:e.pointerId,buttons:e.buttons,x:e.clientX,y:e.clientY,target:e.target.dataset?.layoutSplit||e.target.id||e.target.className,hit:e.clientX==null?null:document.elementFromPoint(e.clientX,e.clientY)?.dataset?.layoutSplit,capture:e.target.hasPointerCapture?.(e.pointerId),focused:document.hasFocus()});if(window.layoutInputLog.length>600)window.layoutInputLog.shift();},true);window.addEventListener('blur',()=>window.layoutInputLog.push({type:'window-blur'}));void 0`);await js(`term?.dispose();term=null;fit=null;window.Terminal=undefined;window.layoutScene=${scene};void 0`);
 const set=async name=>{await js(`layoutScene(${JSON.stringify(name)},'light');void 0`);await settle();};
 const value=async name=>js(`(()=>{const n=document.querySelector('[data-layout-split="${name}"]'),r=n.getBoundingClientRect();return {value:+n.getAttribute('aria-valuenow'),min:+n.getAttribute('aria-valuemin'),max:+n.getAttribute('aria-valuemax'),x:Math.round(r.left+r.width/2),y:Math.round(r.top+Math.min(r.height/2,100)),width:r.width,height:r.height,hit:document.elementFromPoint(Math.round(r.left+r.width/2),Math.round(r.top+Math.min(r.height/2,100)))?.dataset?.layoutSplit,focus:document.hasFocus()};})()`);
 // Native events are acknowledged in the DOM before reading geometry. A fixed
 // 35ms delay races software rendering and can read the previous pointer value.
 let leftHeld=false;
 const receipt=async(sequence,type,detail)=>waitFor(()=>js(`window.layoutInputLog.some(event=>event.sequence>${sequence}&&event.type===${JSON.stringify(type)}&&${detail})`),type+' delivery');
 const mouse=async(type,point,more={})=>{
  const sequence=await js('window.layoutInputSequence');
  if(type==='mouseDown')leftHeld=true;
  const held=type==='mouseMove'&&leftHeld?{button:'left',modifiers:['leftButtonDown']}:{};
  win.webContents.sendInputEvent({type,...point,...held,...more});
  await receipt(sequence,{mouseMove:'pointermove',mouseDown:'pointerdown',mouseUp:'pointerup'}[type],`event.x===${point.x}&&event.y===${point.y}`);
  if(type==='mouseUp')leftHeld=false;
 };
 const key=async(keyCode,modifiers=[])=>{
  const code=keyCode.replace(/^Arrow/,''),domKey=['Left','Right','Up','Down'].includes(code)?'Arrow'+code:code;
  for(const type of ['keyDown','keyUp']){
   const sequence=await js('window.layoutInputSequence');win.webContents.sendInputEvent({type,keyCode:code,modifiers});
   await receipt(sequence,type.toLowerCase(),`event.key===${JSON.stringify(domKey)}`);
  }
 };
 await waitFor(()=>js('document.hasFocus()'),'native window focus');
 win.setContentSize(1920,1200);await set('parameters-hardware');
 const tall=await js(`(()=>{const layout=document.querySelector('[data-layout-ready]').getBoundingClientRect(),content=document.querySelector('.studio-page-body'),body=content.getBoundingClientRect();return {height:layout.height,gap:body.bottom-layout.bottom, padding:parseFloat(getComputedStyle(content).paddingBottom)||0};})()`);
 check(tall.height>820&&tall.gap<=tall.padding+24,'tall parameter workspace fills the available page height',tall);report.states.push({tall});
 win.setContentSize(1440,1100);
 const pane=async name=>{
  const axis=name==='referenceHeight'?'y':'x',sign=['inspector','referenceHeight'].includes(name)||(name==='reference'&&await js(`document.querySelector('[data-layout-ready]').dataset.layoutPosition==='right'`))?-1:1;
  await js(`document.querySelector('[data-layout-split="${name}"]').scrollIntoView({block:'nearest',inline:'nearest'})`);await settle();
  const before=await value(name),change=before.value+50<=before.max?50:-50,point={x:before.x,y:before.y},moved={...point,[axis]:point[axis]+change*sign};
  check(before.width>=8&&before.height>=8&&before.hit===name&&before.focus,name+' handle is reachable and receives native input',before);
  await mouse('mouseMove',point);await mouse('mouseDown',point,{button:'left',clickCount:1});await mouse('mouseMove',moved);const during=await value(name);
  check(during.value===before.value+change,name+' tracks pointer pixel for pixel',{before,during,change});
  await key('Escape');const cancelled=await value(name);await mouse('mouseUp',moved,{button:'left',clickCount:1});check(cancelled.value===before.value,name+' Escape restores prior width',cancelled);
  await mouse('mouseMove',point);await mouse('mouseDown',point,{button:'left',clickCount:1});await mouse('mouseMove',moved);await mouse('mouseUp',moved,{button:'left',clickCount:1});
  const committed=await value(name);check(committed.value===before.value+change,name+' release retains width',committed);
  const persisted=await js(`JSON.parse(localStorage.getItem('serialarm-studio-layout-v1')).widths['${name}']`);check(persisted===committed.value,name+' released width persists',persisted);
  await key('Home');const home=await value(name);check(home.value===Math.max(home.min,Math.min(home.max,{nav:224,tree:224,inspector:288,reference:360,referenceHeight:240}[name])),name+' Home resets the panel size',home);const direction=home.value+8<=home.max?1:-1;
  await key(axis==='y'?(direction*sign>0?'ArrowDown':'ArrowUp'):(direction*sign>0?'ArrowRight':'ArrowLeft'));const small=await value(name);check(small.value===home.value+direction*8,name+' arrow adjusts eight pixels',{home,small});
  await key(axis==='y'?(direction*sign>0?'ArrowDown':'ArrowUp'):(direction*sign>0?'ArrowRight':'ArrowLeft'),['shift']);const fast=await value(name);check(fast.value===Math.max(fast.min,Math.min(fast.max,small.value+direction*24)),name+' Shift arrow adjusts twenty-four pixels',{small,fast});await key('Home');
  report.panes.push({name,before,during,cancelled,committed,home,small,fast});
 };
 await set('model');await js('window.layoutInitialView=modelView;window.layoutInitialCanvas=modelView.canvas;void 0');
 for(const name of ['nav','tree','inspector'])await pane(name);
 check(await js('modelView===layoutInitialView&&modelView.canvas===layoutInitialCanvas'),'model pane resizing preserves renderer and canvas');
 // Model selection spans the row; the frame toggle keeps its own action.
 await js(`document.querySelector('.model-tree .tree-row-link .tree-item').click();void 0`);await pause(350);
 await js(`document.querySelectorAll('.model-tree .tree-row-link .tree-item')[1].click();void 0`);
 await waitFor(()=>js(`(()=>{const v=parseFloat(document.querySelectorAll('.model-tree .tree-row-link')[1].style.getPropertyValue('--studio-selection'));return v>0&&v<.99;})()`),'model selection spring');
 const modelSpring=await js(`(()=>{const row=document.querySelectorAll('.model-tree .tree-row-link')[1];return {value:+row.style.getPropertyValue('--studio-selection'),selected:row.classList.contains('active'),name:state.modelSelection};})()`);
 check(modelSpring.selected&&modelSpring.value>0&&modelSpring.value<1,'model row selection animates through intermediate values',modelSpring);
 const modelToggle=await js(`(()=>{const b=document.querySelectorAll('.model-tree .tree-row-link')[1].querySelector('.tree-frame'),before=b.getAttribute('aria-pressed'),selection=state.modelSelection;b.click();return {before,after:b.getAttribute('aria-pressed'),selection,afterSelection:state.modelSelection};})()`);
 check(modelToggle.before!==modelToggle.after&&modelToggle.selection===modelToggle.afterSelection,'frame action remains independent of row selection',modelToggle);
 // Layer chips use the whole label as the hit target, with distinct interaction states.
 const chip='.model-toolbar label:has(>input[type=checkbox])';
 const style=()=>js(`(()=>{const n=document.querySelector(${JSON.stringify(chip)}),s=getComputedStyle(n),r=n.getBoundingClientRect();return {background:s.backgroundColor,border:s.borderColor,x:Math.round(r.left+r.width/2),y:Math.round(r.top+r.height/2),checked:n.querySelector('input').checked,hover:n.matches(':hover'),active:n.matches(':active'),hit:n.contains(document.elementFromPoint(Math.round(r.left+r.width/2),Math.round(r.top+r.height/2)))};})()`);
 await mouse('mouseMove',{x:0,y:0});await pause(180);const normal=await style();await mouse('mouseMove',{x:normal.x,y:normal.y});await waitFor(async()=>{const s=await style();return s.hover&&s.hit&&s.background!==normal.background},'layer chip hover presentation');const hover=await style();await mouse('mouseDown',{x:normal.x,y:normal.y},{button:'left',clickCount:1});await waitFor(async()=>{const s=await style();return s.active&&s.background!==hover.background},'layer chip pressed presentation');const pressed=await style();
 check(!normal.hover&&hover.hover&&pressed.active&&normal.background!==hover.background&&hover.background!==pressed.background,'model layer chip shows default, hover and pressed surfaces',{normal,hover,pressed});await mouse('mouseUp',{x:0,y:0},{button:'left',clickCount:1});report.states.push({normal,hover,pressed});
 await set('workbench');await js('window.layoutInitialView=modelView;window.layoutInitialCanvas=modelView.canvas;document.querySelector("#workbench-speed").value="31";document.querySelector("#workbench-speed").dispatchEvent(new Event("input",{bubbles:true}));void 0');
 const targetPoint=await js(`(()=>{const r=document.querySelectorAll('[data-target-row]')[1].querySelector('td').getBoundingClientRect();return {x:Math.round(r.left+r.width/2),y:Math.round(r.top+r.height/2)};})()`);
 await mouse('mouseMove',targetPoint);await mouse('mouseDown',targetPoint,{button:'left',clickCount:1});await mouse('mouseUp',targetPoint,{button:'left',clickCount:1});
 check(await js('state.workbenchFineJoint===1'),'clicking a joint row outside its input selects that joint');
 const inputSelection=await js(`(()=>{const before=state.workbenchFineJoint;document.querySelector('[data-target-row="2"] input').dispatchEvent(new MouseEvent('click',{bubbles:true}));return {before,after:state.workbenchFineJoint};})()`);
 check(inputSelection.before===inputSelection.after,'interacting with a target field does not select another row',inputSelection);
 check(await js(`!document.querySelector('.control-fine-control #workbench-speed')&&!!document.querySelector('.control-execution-panel #workbench-speed')`),'global speed belongs to the execution panel, outside single-joint fine control');
 check(await js(`document.querySelector('.control-execution-panel').innerText.includes('全部关节')`),'speed scope explicitly covers all joints');
 await pane('reference');
 await js(`document.querySelector('[data-layout-action="dock"]').click();void 0`);await settle();await pane('referenceHeight');
 // Exercise the actual smallest permitted bottom dock, not only the remembered
 // Home height; the full canvas must fit inside the clipped preview body.
 const minimumBefore=await value('referenceHeight'),minimumPoint={x:minimumBefore.x,y:minimumBefore.y};
 await mouse('mouseMove',minimumPoint);await mouse('mouseDown',minimumPoint,{button:'left',clickCount:1});
 const minimumMoved={x:minimumPoint.x,y:minimumPoint.y+minimumBefore.value-minimumBefore.min};
 await mouse('mouseMove',minimumMoved);await mouse('mouseUp',minimumMoved,{button:'left',clickCount:1});
 const minimumDock=await js(`(()=>{const rect=node=>{const r=node.getBoundingClientRect();return {top:r.top,bottom:r.bottom,height:r.height};};return {preview:rect(document.querySelector('[data-workspace-preview]')),body:rect(document.querySelector('.workspace-preview-body')),viewport:rect(modelView.container),canvas:rect(modelView.canvas)};})()`);
 report.states.push({minimumDock});
 check(minimumDock.viewport.bottom<=minimumDock.body.bottom+1&&minimumDock.canvas.bottom<=minimumDock.body.bottom+1&&minimumDock.viewport.top>=minimumDock.body.top-1,'minimum bottom dock contains the complete model canvas',minimumDock);
 await key('Home');
 for(const action of ['model-focus','model-focus','dock']){await js(`document.querySelector('[data-layout-action="${action}"]').click();void 0`);await settle();}
 check(await js('modelView===layoutInitialView&&modelView.canvas===layoutInitialCanvas'),'docking and expanding preserve renderer and canvas');
 check(await js('document.querySelector("#workbench-speed").value==="31"&&state.speedScale===.31'),'panel operations retain the edited global speed');
 await js('document.querySelector("#workbench-speed").scrollIntoView({block:"center"});void 0');await settle();
 check(await js('(()=>{const a=document.querySelector(".control-fine-control").getBoundingClientRect(),b=document.querySelector(".control-execution-panel").getBoundingClientRect(),v=document.querySelector("#workbench-speed").getBoundingClientRect();return a.bottom<=b.top&&v.top>=0&&v.bottom<=innerHeight;})()'),'fine adjustment and global execution occupy separate, reachable blocks');
 fs.writeFileSync(path.join(output,'global-execution-controls.png'),(await win.webContents.capturePage()).toPNG());
 for(const [width,height] of [[1440,900],[1100,800],[900,720]]){
  win.setContentSize(width,height);await settle();await js(`document.querySelector('.studio-page-body').scrollTop=0;void 0`);await settle();
  const measured=await js(`(()=>{const v=modelView.container.getBoundingClientRect(),p=document.querySelector('.workbench-task').getBoundingClientRect(),root=document.querySelector('[data-layout-ready]'),m=document.querySelector('[data-workspace-preview]').getBoundingClientRect();return {width:innerWidth,dock:root.dataset.layoutDock,modelWidth:v.width,canvasHeight:v.height,modelHeight:m.height,taskHeight:p.height,canvasContained:v.top>=document.querySelector('.workspace-preview-body').getBoundingClientRect().top-1&&v.bottom<=document.querySelector('.workspace-preview-body').getBoundingClientRect().bottom+1,modelVisible:m.top>=0&&m.bottom<=innerHeight+1,taskVisible:p.top>=0&&p.bottom<=innerHeight+1,separate:root.dataset.layoutDock==='bottom'?p.bottom<=m.top:m.right<=p.left||p.right<=m.left,overflow:document.documentElement.scrollWidth>innerWidth};})()`);
  check(measured.modelWidth>=200&&measured.modelHeight>=170&&measured.canvasHeight>=180&&measured.canvasContained&&measured.taskHeight>=200&&measured.modelVisible&&measured.taskVisible&&measured.separate&&!measured.overflow,'usable model and task layout at '+width,measured);
  fs.writeFileSync(path.join(output,'workbench-'+width+'.png'),(await win.webContents.capturePage()).toPNG());report.states.push(measured);
 }
 report.inputEvents=await js('window.layoutInputLog');
 report.calls=await js('window.serialArm.fixtureCalls()');check(!report.calls.includes('workbench_start'),'fixture sends no device connection request');
 fs.writeFileSync(path.join(output,'report.json'),JSON.stringify(report,null,2));console.log('LAYOUT_CONTROLS_PASS '+JSON.stringify({checks:report.checks.length,report:path.join(output,'report.json')}));win.destroy();app.quit();
}).catch(async error=>{console.error(error.stack);if(auditWindow&&!auditWindow.isDestroyed()){auditReport.inputEvents=await auditWindow.webContents.executeJavaScript('window.layoutInputLog');fs.writeFileSync(path.join(output,'failure.json'),JSON.stringify(auditReport,null,2));fs.writeFileSync(path.join(output,'failure.png'),(await auditWindow.webContents.capturePage()).toPNG());console.error('Failure evidence: '+path.join(output,'failure.json'));}app.exit(1)});
