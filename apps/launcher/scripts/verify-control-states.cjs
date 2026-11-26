/* Real production renderer; offline fixture, native pointer/keyboard events and bitmap probes.
 * DISPLAY=:1 node_modules/.bin/electron --no-sandbox scripts/verify-control-states.cjs --out /tmp/serialarm-state-layout/states
 */
const assert=require('node:assert/strict'),fs=require('node:fs'),path=require('node:path'),os=require('node:os');
const {app,BrowserWindow}=require('electron');
const {buildFixture}=require('./studio-ui-fixture.cjs');
const index=process.argv.indexOf('--out'),output=path.resolve(index<0?'/tmp/serialarm-control-states':process.argv[index+1]);
const fixture=buildFixture(fs.mkdtempSync(path.join(os.tmpdir(),'serialarm-controls-')));
const render=fs.readFileSync(path.join(__dirname,'render-studio-ui.cjs'),'utf8');
const scene=render.slice(render.indexOf('function setScene('),render.indexOf('function snapshot('));
app.setPath('userData',path.join(fixture.directory,'controls-user-data'));
app.commandLine.appendSwitch('force-device-scale-factor','1');
const pause=ms=>new Promise(resolve=>setTimeout(resolve,ms));
app.whenReady().then(async()=>{
 fs.mkdirSync(output,{recursive:true});
 const win=new BrowserWindow({width:1300,height:900,frame:false,show:false,enableLargerThanScreen:true,webPreferences:{preload:fixture.preload,sandbox:false,offscreen:true,backgroundThrottling:false}});
 const deadline=setTimeout(()=>{console.error('Control verification timeout');app.exit(1);},90000);
 await win.loadFile(fixture.index);await win.webContents.executeJavaScript('document.fonts.ready');
 const js=code=>win.webContents.executeJavaScript(code);
 await js(`term?.dispose();term=null;fit=null;window.Terminal=undefined;window.controlsScene=${scene};void 0`);
 const report={source:'Production renderer; offline fixture; native input events and rendered bitmap',layouts:[],controls:[],screenshots:[],checks:[]};
 const check=(ok,name)=>{assert(ok,name);report.checks.push(name);};
 const settle=async()=>{await pause(300);await js(`(async()=>{const start=performance.now();while(document.querySelector('[data-studio-content-enter],[data-studio-layout-resizing],[data-studio-disclosure-animating]')&&performance.now()-start<3000)await new Promise(requestAnimationFrame);})()`);};
 const set=async(name,theme='light',language='zh-CN')=>{win.webContents.sendInputEvent({type:'mouseMove',x:0,y:0});await js(`window.controlsScene(${JSON.stringify(name)},${JSON.stringify(theme)},${JSON.stringify(language)})`);await settle();};
 const shot=async name=>{const file=path.join(output,name+'.png');fs.writeFileSync(file,(await win.webContents.capturePage()).toPNG());report.screenshots.push(file);return file;};
 const locate=async(selector,scroll=true)=>js(`(()=>{const n=document.querySelector(${JSON.stringify(selector)});if(!n)throw Error('Missing '+${JSON.stringify(selector)});${scroll?"n.scrollIntoView({block:'center',inline:'nearest'});":''}const r=n.getBoundingClientRect();return {x:Math.round(r.left+r.width/2),y:Math.round(r.top+r.height/2)};})()`);
 const style=selector=>js(`(()=>{const n=document.querySelector(${JSON.stringify(selector)}),s=getComputedStyle(n),r=n.getBoundingClientRect();const rgb=value=>{const c=document.createElement('canvas').getContext('2d');c.fillStyle=value;c.fillRect(0,0,1,1);return [...c.getImageData(0,0,1,1).data].slice(0,3);};return {backgroundRgb:rgb(s.backgroundColor),colorRgb:rgb(s.color),background:s.backgroundColor,color:s.color,border:s.borderColor,outline:s.outlineColor,outlineStyle:s.outlineStyle,outlineWidth:s.outlineWidth,focus:n.matches(':focus-visible'),disabled:n.disabled,checked:n.checked,hover:n.matches(':hover'),active:n.matches(':active'),rect:{x:r.x,y:r.y,width:r.width,height:r.height}};})()`);
 const move=async(selector,scroll=true)=>{const point=await locate(selector,scroll);win.focus();win.webContents.focus();win.webContents.sendInputEvent({type:'mouseMove',...point});await pause(220);return point;};
 const resetPointer=async()=>{win.webContents.sendInputEvent({type:'mouseMove',x:0,y:0});await js('getSelection()?.removeAllRanges()');await pause(180);};
 const press=async selector=>{const point=await move(selector,false);win.webContents.sendInputEvent({type:'mouseDown',button:'left',clickCount:1,...point});await pause(160);return point;};
 const release=async point=>{win.webContents.sendInputEvent({type:'mouseMove',x:0,y:0});win.webContents.sendInputEvent({type:'mouseUp',button:'left',clickCount:1,x:0,y:0});await js('getSelection()?.removeAllRanges()');await settle();};
 const geometry=()=>js(`(()=>{const box=n=>{const r=n.getBoundingClientRect();return {left:r.left,right:r.right,top:r.top,bottom:r.bottom,width:r.width};},title=document.querySelector('#profile-guide-title'),status=document.querySelector('.profile-guide-count'),settings=document.querySelector('.profile-settings');return {width:innerWidth,height:innerHeight,rootWidth:document.documentElement.scrollWidth,bodyOverflow:document.querySelector('.studio-page-body').scrollWidth>document.querySelector('.studio-page-body').clientWidth+1,settings:box(settings),head:box(document.querySelector('.profile-page-head .page-heading')),title:box(title),status:box(status),tools:[...document.querySelectorAll('.profile-disclosure-trigger')].map(n=>({id:n.id,...box(n)})),first:box(document.querySelector('.setup-main input:not(:disabled):not([type=checkbox]),.setup-main select:not(:disabled)')),grids:[...document.querySelectorAll('.setup-main .parameter-fields')].filter(n=>n.getBoundingClientRect().width).map(n=>getComputedStyle(n).gridTemplateColumns.split(' ').length),paths:[...document.querySelectorAll('.parameter-field-wide')].filter(n=>n.parentElement.matches('.parameter-fields')).map(n=>({field:box(n),grid:box(n.parentElement)}))};})()`);
 for(const [width,height,theme,language] of [[1300,900,'light','zh-CN'],[1000,720,'light','zh-CN'],[2400,1100,'light','en'],[1300,900,'dark','en'],[1000,720,'dark','en'],[2400,1100,'dark','zh-CN']]){
  win.setContentSize(width,height);await set('parameters-model',theme,language);
  const initial=await geometry();check(initial.rootWidth<=width&&!initial.bodyOverflow,'No outer overflow '+[width,theme,language]);
  check(initial.settings.width<=1281&&initial.grids.every(count=>count<=2),'Bounded reading width and two-column maximum '+[width,theme,language]);
  check(initial.status.left-initial.title.right>=7&&initial.status.left-initial.title.right<=13,'Check title and status stay adjacent '+[width,theme,language]);
  check(Math.abs(initial.head.left-initial.settings.left)<=1,'Parameter title shares content left edge '+[width,theme,language]);
  check(initial.paths.every(p=>Math.abs(p.field.width-p.grid.width)<1),'Long paths span their complete field row '+[width,theme,language]);
  if(width===1000&&language==='zh-CN'){await shot('narrow-first-editable');check(initial.first.bottom<height-160,'Narrow first editable input is fully visible above the lower action area '+JSON.stringify(initial));}
  const buttonSurface=await js(`[...document.querySelectorAll('.profile-disclosure-trigger')].map(n=>{const s=getComputedStyle(n);return {background:s.backgroundColor,border:s.borderColor,width:parseFloat(s.borderTopWidth)};})`);
  check(buttonSurface.every(s=>s.width>=1&&s.background!=='rgba(0, 0, 0, 0)'&&s.border!=='rgba(0, 0, 0, 0)'),'Summary actions have visible button surfaces '+[width,theme,language]);
  const punctuation=await js(`[...document.querySelectorAll('#page p,#page .hint')].filter(n=>n.getClientRects().length&&!n.closest('pre,code')).map(n=>n.textContent.trim()).filter(text=>/。|[。；，：！？….!?]$/.test(text))`);
  check(!punctuation.length,'Paragraph copy has no Chinese periods or terminal punctuation '+JSON.stringify(punctuation));
  await shot(`parameters-${width}x${height}-${theme}-${language}`);
  await js(`document.querySelectorAll('.profile-disclosure-trigger').forEach(n=>n.click())`);await settle();
  const expanded=await geometry();check(expanded.tools.every((tool,i)=>Math.abs(tool.left-initial.tools[i].left)<1&&Math.abs(tool.top-initial.tools[i].top)<1),'Expanding help keeps all summary entries stable '+[width,theme,language]);
  check(await js(`[...document.querySelectorAll('.profile-disclosure-trigger')].every(n=>n.getAttribute('aria-expanded')==='true')`),'Visible triggers report actual expanded state '+[width,theme,language]);
  if(width===1300)await shot(`parameters-expanded-${theme}-${language}`);
  await set('parameters-mapping',theme,language);check(await js(`document.querySelector('.profile-control-table thead [data-parameter-unit]')&&!document.querySelector('.profile-control-table tbody [data-parameter-unit]')`),'Common units move to table header '+[width,theme,language]);
  await shot(`mapping-${width}x${height}-${theme}-${language}`);
  report.layouts.push({theme,language,initial,expanded});
 }
 win.setContentSize(1300,900);await set('parameters-model');
 const summaryAction='#profile-check-details-trigger';await resetPointer();const actionDefault=await style(summaryAction);await move(summaryAction,false);const actionHover=await style(summaryAction);await shot('summary-button-hover');const actionPoint=await press(summaryAction),actionPressed=await style(summaryAction);await shot('summary-button-pressed');await release(actionPoint);
 check(actionDefault.backgroundRgb.join()!==actionHover.backgroundRgb.join()&&actionHover.backgroundRgb.join()!==actionPressed.backgroundRgb.join(),'Summary button has distinct default, hover and pressed '+JSON.stringify({actionDefault,actionHover,actionPressed,focus:await js('document.hasFocus()')}));report.controls.push({component:'summary-button',normal:actionDefault,hover:actionHover,pressed:actionPressed});

 await js(`state.profileFieldErrors['core.model.gravity']='Must contain three finite numbers';refreshProfileGuidance();void 0`);await settle();
 check(await js(`document.querySelector('.profile-guide-count').textContent.includes('待处理')&&document.querySelector('[data-setup-target]')`),'Invalid filled values are pending items with field navigation');await shot('parameters-pending-light');
 await set('parameters-model');
 const input='.setup-main input:not(:disabled):not([type=checkbox])';await locate(input);await resetPointer();const normal=await style(input);await move(input,false);const hover=await style(input);
 check(normal.border==='rgb(114, 115, 111)'&&hover.border==='rgb(78, 107, 89)','Input border darkens on hover');
 report.controls.push({component:'input',normal,hover});await shot('input-hover-light');
 await js(`document.querySelector(${JSON.stringify(input)}).setAttribute('aria-invalid','true')`);await pause(180);check((await style(input)).border!=='rgb(78, 107, 89)','Invalid border survives hover');
 await set('settings');
 const checkbox='#font-ligatures',label='.compact-checkbox';await locate(checkbox);await resetPointer();
 const bitmap=async selector=>{const {rect}=await style(selector),capture=await win.webContents.capturePage({x:Math.floor(rect.x),y:Math.floor(rect.y),width:Math.ceil(rect.width),height:Math.ceil(rect.height)}),pixels=capture.toBitmap(),counts={};for(let i=0;i<pixels.length;i+=4){const key=[pixels[i+2],pixels[i+1],pixels[i]].join(',');counts[key]=(counts[key]||0)+1;}return {rect,white:counts['255,255,255']||0,green:counts['7,193,96']||0,hover:counts['6,173,86']||0,pressed:counts['5,155,77']||0};};
 for(const checked of [false,true]){
  await js(`document.querySelector('${checkbox}').checked=${checked}`);await resetPointer();
  const normal=await style(checkbox),pixels=await bitmap(checkbox);await shot(`checkbox-${checked?'checked':'unchecked'}-default`);
  const point=await move(label,false),hover=await style(checkbox),hoverPixels=await bitmap(checkbox);await shot(`checkbox-${checked?'checked':'unchecked'}-hover`);
  win.webContents.sendInputEvent({type:'mouseDown',button:'left',clickCount:1,...point});await pause(170);
  const pressed=await style(checkbox),pressedPixels=await bitmap(checkbox);await shot(`checkbox-${checked?'checked':'unchecked'}-pressed`);await release(point);
  check(normal.background===(checked?'rgb(7, 193, 96)':'rgb(255, 254, 250)'),'Checkbox default fill '+checked);
  check(hover.background===(checked?'rgb(6, 173, 86)':'rgb(241, 239, 233)'),'Label hover reaches checkbox interior '+checked);
  check(pressed.background===(checked?'rgb(5, 155, 77)':'rgb(230, 228, 220)'),'Label pressed reaches checkbox interior '+checked);
  if(checked)check(pixels.white>=5&&pixels.green>=50&&hoverPixels.white>=5&&hoverPixels.hover>=50&&pressedPixels.white>=5&&pressedPixels.pressed>=50,'Rendered checkbox has white tick with the three exact green fills');
  report.controls.push({component:'checkbox',checked,normal,hover,pressed,pixels,hoverPixels,pressedPixels});
 }
 win.focus();win.webContents.focus();
 await js(`document.querySelector('#font-ligatures').checked=false;document.querySelector('#font-ligatures').focus()`);
 win.webContents.sendInputEvent({type:'keyDown',keyCode:'Tab',modifiers:['shift']});win.webContents.sendInputEvent({type:'keyUp',keyCode:'Tab',modifiers:['shift']});
 win.webContents.sendInputEvent({type:'keyDown',keyCode:'Tab'});win.webContents.sendInputEvent({type:'keyUp',keyCode:'Tab'});await pause(150);
 check((await style(checkbox)).focus,'Checkbox keyboard focus is visible '+JSON.stringify(await js('({active:document.activeElement?.id,focused:document.hasFocus()})')));await shot('checkbox-keyboard-focus');
 win.webContents.sendInputEvent({type:'keyDown',keyCode:'Space'});win.webContents.sendInputEvent({type:'keyUp',keyCode:'Space'});await pause(150);
 check(await js(`document.querySelector('#font-ligatures').checked&&state.fontLigatures`),'Space toggles checkbox and follows the production event chain');
 await js(`document.querySelector('#font-ligatures').disabled=true`);const disabled=await style(checkbox);const point=await move(label,false);win.webContents.sendInputEvent({type:'mouseDown',button:'left',clickCount:1,...point});win.webContents.sendInputEvent({type:'mouseUp',button:'left',clickCount:1,...point});await pause(180);
 check((await style(checkbox)).checked===disabled.checked,'Disabled checkbox cannot toggle');await shot('checkbox-disabled');
 for(const selected of [false,true]){
  await resetPointer();const card=selected?'.graphics-mode-option.active':'.graphics-mode-option:not(.active)';await locate(card);await resetPointer();const normal=await style(card);await move(card,false);const hover=await style(card),point=await press(card),pressed=await style(card);await shot(`selection-card-${selected?'selected':'normal'}-pressed`);await release(point);
  check(normal.background!==hover.background&&hover.background!==pressed.background,'Selection card has distinct hover and pressed '+selected);
  if(selected)check(normal.colorRgb.join()==='255,255,255'&&hover.backgroundRgb.join()==='6,173,86','Selected card retains white text and green hover '+JSON.stringify({normal,hover}));
  report.controls.push({component:'selection-card',selected,normal,hover,pressed});
 }
 await set('parameters-model');
 for(const [component,selector] of [['primary','#profile-save'],['secondary','#profile-reload'],['navigation','.nav-item:not(.active)'],['navigation-selected','.nav-item.active'],['disclosure','.setup-main details > summary']]){
  await locate(selector);await resetPointer();const normal=await style(selector);await move(selector,false);const hover=await style(selector),point=await press(selector),pressed=await style(selector);await shot(component+'-pressed');await release(point);report.controls.push({component,normal,hover,pressed});
  if(['secondary','navigation','disclosure'].includes(component))check(hover.backgroundRgb.join()!==pressed.backgroundRgb.join(),component+' has distinct hover and pressed');
  if(component==='primary')check(normal.color==='rgb(255, 255, 255)'&&hover.background==='rgb(6, 173, 86)'&&pressed.background==='rgb(5, 155, 77)','Primary uses approved fill and white ink in all states');
 }
 await set('library');check((await style('.robot-row-avatar')).background==='rgb(255, 255, 255)','Robot asset base stays white');await move('.robot-row-open');check((await style('.robot-row-avatar')).background==='rgb(255, 255, 255)','Row hover leaves robot asset white');await shot('robot-asset-hover');
 await set('settings','dark');await locate(checkbox);await js(`document.querySelector('#font-ligatures').checked=true`);await resetPointer();
 check((await bitmap(checkbox)).white>=5,'Dark checkbox keeps a rendered white tick');await shot('checkbox-dark');
 await set('model');await js(`document.querySelector('.model-display-options').open=true`);await settle();
 const radio='.model-display-options input[type=radio]:checked';await locate(radio);await resetPointer();const radioDefault=await style(radio);await move(radio);const radioHover=await style(radio),radioPoint=await press(radio),radioPressed=await style(radio);await shot('radio-pressed');await release(radioPoint);
 check(radioDefault.background==='rgb(7, 193, 96)'&&radioHover.background==='rgb(6, 173, 86)'&&radioPressed.background==='rgb(5, 155, 77)','Radio uses the same three checked states');
 // Render real warning markup from the actual graphics status source.
 await set('settings');await js(`graphicsState.requiresRestart=true;graphicsState.requestedMode='compatibility';render();document.querySelector('#page').insertAdjacentHTML('beforeend','<div id="fixture-graphics-diagnostics">'+graphicsDiagnosticsHtml({requiresRestart:true})+'</div>')`);await settle();
 await js(`document.querySelector('.graphics-diagnostic-status.is-warning').id='fixture-warning'`);
 check((await style('#fixture-warning')).color==='rgb(166, 83, 24)','Restart warning uses semantic warning foreground');
 report.controls.push({component:'warning',value:await style('#fixture-warning')});
 await shot('graphics-restart-warning');await js(`document.querySelector('#fixture-graphics-diagnostics').remove()`);
 clearTimeout(deadline);fs.writeFileSync(path.join(output,'report.json'),JSON.stringify(report,null,2));
 console.log('CONTROL_STATES_PASS '+JSON.stringify({checks:report.checks.length,layouts:report.layouts.length,screenshots:report.screenshots.length,report:path.join(output,'report.json')}));win.destroy();app.quit();
}).catch(error=>{console.error(error.stack);app.exit(1);});
