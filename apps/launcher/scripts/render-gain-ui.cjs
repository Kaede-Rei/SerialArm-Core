/* Execute with Electron on a desktop display, without any robot session:
 * DISPLAY=:1 node_modules/.bin/electron --no-sandbox scripts/render-gain-ui.cjs \
 *   --fixture /tmp/serialarm-gain-ui --out /tmp/serialarm-gain-ui/screenshots
 */
const {app,BrowserWindow}=require('electron');
const fs=require('node:fs');
const path=require('node:path');
const args=process.argv.slice(2);
const argument=(name,fallback)=>{const index=args.indexOf(name);return index<0?fallback:args[index+1];};
const fixture=path.resolve(argument('--fixture','/tmp/serialarm-gain-ui'));
const output=path.resolve(argument('--out',path.join(fixture,'screenshots')));
const auditPath=argument('--clip-audit','');
const scenes=argument('--scenes','idle,running,failed,empty-result,available').split(',');
const themes=argument('--themes','light').split(',');
function contrastSamples(){
 const parse=value=>{const parts=value.match(/[\d.]+/g)?.map(Number)||[];return parts.length>=3?[...parts.slice(0,3),parts[3]??1]:[0,0,0,0];};
 const blend=(front,back)=>front.slice(0,3).map((channel,index)=>channel*front[3]+back[index]*(1-front[3]));
 const luminance=rgb=>rgb.map(value=>{const channel=value/255;return channel<=.04045?channel/12.92:((channel+.055)/1.055)**2.4;}).reduce((value,channel,index)=>value+channel*[.2126,.7152,.0722][index],0);
 const result=[];
 for(const [label,selector] of [['main','.gain-page-heading h3'],['muted','.gain-time-summary dt'],['hint','.gain-acceptance-primary .hint'],['primary','.gain-task-actions .button.primary:not(:disabled)'],['start_primary','#model-calibration-replay-confirm:not(:disabled)'],['import_primary','#calibration-tool-import:not(:disabled)']]){
  const node=document.querySelector(selector);if(!node||!node.getClientRects().length)continue;
  let background=[255,255,255];const ancestors=[];for(let parent=node;parent;parent=parent.parentElement)ancestors.push(parent);
  for(const parent of ancestors.reverse())background=blend(parse(getComputedStyle(parent).backgroundColor),background);
  const color=getComputedStyle(node).color,foreground=blend(parse(color),background),lum=[luminance(foreground),luminance(background)].sort((a,b)=>b-a),ratio=(lum[0]+.05)/(lum[1]+.05);
  result.push({label,color,background,ratio:Number(ratio.toFixed(3))});
 }
 return result;
}
app.commandLine.appendSwitch('force-device-scale-factor','1');
app.whenReady().then(async()=>{
 fs.mkdirSync(output,{recursive:true});
 const win=new BrowserWindow({width:1280,height:1000,show:false,webPreferences:{backgroundThrottling:false}});
 const settle=()=>win.webContents.executeJavaScript('new Promise(resolve=>requestAnimationFrame(()=>requestAnimationFrame(resolve)))');
 const report={runtime:process.versions.electron,checks:null,renders:[]};
 const captureSegments=async file=>{
   const dimensions=await win.webContents.executeJavaScript('({width:innerWidth,height:Math.ceil(document.documentElement.scrollHeight),viewport:innerHeight,scroll:scrollY})');
   const captures=[];
   // Keep each real scrolled viewport as a separate image. Oversized GPU clips
   // may repeat pixels on this desktop, so the report records actual coverage.
   for(let requested=0;requested<dimensions.height;requested+=dimensions.viewport){
     const top=await win.webContents.executeJavaScript(`(()=>{scrollTo(0,${requested});return scrollY;})()`);await settle();
     const tile=await win.webContents.capturePage({x:0,y:0,width:dimensions.width,height:dimensions.viewport}),size=tile.getSize(),png=tile.toPNG();
     if(tile.isEmpty()||png.length<10000||size.width!==dimensions.width||size.height!==dimensions.viewport)throw Error('Viewport capture size mismatch: '+JSON.stringify(size));
     const part=captures.length?file.replace(/\.png$/,`-part${captures.length+1}.png`):file;
     fs.writeFileSync(part,png);captures.push({file:part,scrollTop:top,viewportSize:size,bytes:png.length});
     if(top+dimensions.viewport>=dimensions.height)break;
   }
   await win.webContents.executeJavaScript(`scrollTo(0,${dimensions.scroll})`);await settle();
   return {documentSize:{width:dimensions.width,height:dimensions.height},captures};
 };
 await win.loadFile(path.join(fixture,'index.html'));win.showInactive();await settle();
 report.checks=args.includes('--render-only')?{ok:true,checks:[]}:await win.webContents.executeJavaScript('window.gainFixture.runDomTests()');
 if(!report.checks?.ok)throw Error('DOM behavior checks did not pass');
 for(const width of [1280,375])for(const scene of scenes)for(const theme of themes){
   await win.loadFile(path.join(fixture,'index.html'),{query:{state:scene,theme}});win.setContentSize(width,1000);await settle();
   const dom=await win.webContents.executeJavaScript(`(()=>({width:innerWidth,scroll:document.documentElement.scrollWidth,labels:[...document.querySelectorAll('.gain-page-heading h3')].map(n=>n.textContent),unmeasured:[...document.querySelectorAll('.gain-result-state')].map(n=>n.textContent),controls:[...document.querySelectorAll('[data-gain-candidate-purpose]')].map(n=>({purpose:n.dataset.gainCandidatePurpose,checked:n.checked,disabled:n.disabled})),title:document.querySelector('#gain-calibration-phase')?.textContent}))()`);
   if(dom.scroll>width)throw Error('Page overflow at '+scene+' '+width+': '+dom.scroll);
   const file=path.join(output,`${scene}-${width}-${theme}.png`);const captured=await captureSegments(file);
   const contrast=await win.webContents.executeJavaScript(`(${contrastSamples})()`);
   if(contrast.some(sample=>!sample.label.endsWith('primary')&&sample.ratio<4.5))throw Error('Body text contrast below 4.5: '+JSON.stringify({scene,width,contrast}));
   if(contrast.some(sample=>sample.label.endsWith('primary')&&sample.color!=='rgb(255, 255, 255)'))throw Error('Primary button text must be white: '+JSON.stringify({scene,width,contrast}));
   if(scene==='empty-result'&&!contrast.some(sample=>sample.label==='import_primary'))throw Error('Import button was not measured');
   const item={scene,width,file,...captured,dom,contrast};
   if(auditPath){
     const audit=fs.readFileSync(auditPath,'utf8');const runAudit=()=>win.webContents.executeJavaScript(`(${audit})({root:'.fixture-shell',viewport:false})`);
     item.clip=await runAudit();if(item.clip.cut)throw Error('Clipped shadows at '+scene+' '+width);
     const point=await win.webContents.executeJavaScript(`(()=>{const node=[...document.querySelectorAll('.gain-calibration-page button:not(:disabled),.gain-calibration-page summary')].find(node=>{const r=node.getBoundingClientRect();return r.top>0&&r.bottom<innerHeight;});if(!node)return null;const r=node.getBoundingClientRect();return {x:Math.round(r.left+r.width/2),y:Math.round(r.top+r.height/2)};})()`);
     if(point){win.webContents.sendInputEvent({type:'mouseMove',...point});await settle();}
     item.hoverClip=await runAudit();if(item.hoverClip.cut)throw Error('Hover shadow clipped at '+scene+' '+width);
     await win.webContents.executeJavaScript(`(()=>{for(const selector of ['.gain-acceptance-details','.gain-purpose-details','.gain-diagnostics','.gain-diagnostics .calibration-locks']){const detail=document.querySelector(selector);if(detail)detail.open=true;}})()`);await settle();
     item.expandedClip=await runAudit();if(item.expandedClip.cut)throw Error('Expanded shadow clipped at '+scene+' '+width);
     const overflow=await win.webContents.executeJavaScript('document.documentElement.scrollWidth>innerWidth');if(overflow)throw Error('Expanded overflow at '+scene+' '+width);
     if(width===1280 && ['available','empty-result'].includes(scene)){
       item.expandedCapture=await captureSegments(path.join(output,`${scene}-expanded-${width}-${theme}.png`));
     }
   }
   report.renders.push(item);
 }
 if(!args.includes('--scenes')){
   await win.loadFile(path.join(fixture,'index.html'),{query:{state:'available',theme:'dark'}});win.setContentSize(1280,1200);await settle();
   report.darkCapture=await captureSegments(path.join(output,'available-1280-dark.png'));
   await win.webContents.executeJavaScript("window.gainFixture.scene('idle')");await settle();
   report.darkContrast=await win.webContents.executeJavaScript(`(${contrastSamples})()`);
   if(report.darkContrast.some(sample=>!sample.label.endsWith('primary')&&sample.ratio<4.5||sample.label.endsWith('primary')&&sample.color!=='rgb(255, 255, 255)'))throw Error('Dark button color or body text contrast failed: '+JSON.stringify(report.darkContrast));
 }
 fs.writeFileSync(path.join(output,'report.json'),JSON.stringify(report,null,2));
 console.log('GAIN_UI_DOM_RENDER_PASS '+JSON.stringify({checks:report.checks.checks.length,renders:report.renders.length,report:path.join(output,'report.json')}));
 win.destroy();app.quit();
}).catch(error=>{console.error(error.stack);app.exit(1);});
