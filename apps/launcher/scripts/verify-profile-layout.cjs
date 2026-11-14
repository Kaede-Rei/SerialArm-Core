/* Production renderer Profile layout and disclosure scrolling; fixture denies devices. */
const assert=require('node:assert/strict');
const fs=require('node:fs');
const {createHash}=require('node:crypto');
const os=require('node:os');
const path=require('node:path');
const {app,BrowserWindow}=require('electron');
const {buildFixture}=require('./studio-ui-fixture.cjs');
const directory=fs.mkdtempSync(path.join(os.tmpdir(),'serialarm-profile-layout-'));
const fixture=buildFixture(directory);
const sourcePath=path.join(__dirname,'../renderer/app.js');
const sourceHash=()=>createHash('sha256').update(fs.readFileSync(sourcePath)).digest('hex');
const sourceSha256=sourceHash();
const renderScript=fs.readFileSync(path.join(__dirname,'render-studio-ui.cjs'),'utf8');
const scene=renderScript.slice(renderScript.indexOf('function setScene('),renderScript.indexOf('function snapshot('));
const pause=ms=>new Promise(resolve=>setTimeout(resolve,ms));
const output=process.argv.includes('--out')?path.resolve(process.argv[process.argv.indexOf('--out')+1]):directory;
app.setPath('userData',path.join(directory,'electron-userdata'));
app.commandLine.appendSwitch('force-device-scale-factor','1');
function geometry(){
 const boxes=selector=>[...document.querySelectorAll(selector)].map(node=>{
  const r=node.getBoundingClientRect(),s=getComputedStyle(node);
  return {tag:node.tagName,class:node.className,id:node.id,top:r.top,bottom:r.bottom,height:r.height,
   scrollHeight:node.scrollHeight,clientHeight:node.clientHeight,scrollTop:node.scrollTop,
   overflow:s.overflow,position:s.position,display:s.display,minHeight:s.minHeight,paddingBottom:parseFloat(s.paddingBottom),flex:s.flex};
 });
 return {viewport:{width:innerWidth,height:innerHeight,rootScroll:document.scrollingElement.scrollHeight,rootWidth:document.scrollingElement.scrollWidth,rootTop:document.scrollingElement.scrollTop,bodyScroll:document.body.scrollHeight},boxes:boxes('.content-wrap,.content,#page,.studio-page-body,.profile-settings,.setup-editor,.setup-sidebar,.setup-main'),
  disclosures:boxes('#page details'),tables:[...document.querySelectorAll('.profile-control-table')].filter(n=>!n.closest('details:not([open])')&&n.getBoundingClientRect().height>0).map(table=>({
   headers:[...table.tHead.rows[0].cells].map(n=>{const r=n.getBoundingClientRect(),s=getComputedStyle(n);return {text:n.innerText,left:r.left,right:r.right,top:r.top,bottom:r.bottom,textLeft:r.left+parseFloat(s.paddingLeft),padding:s.padding,textAlign:s.textAlign};}),
   rows:[...table.tBodies[0].rows].map(row=>[...row.cells].map(n=>{const r=n.getBoundingClientRect(),s=getComputedStyle(n),input=n.querySelector('input:not([type=checkbox]):not([type=radio])'),i=input?.getBoundingClientRect(),style=input?getComputedStyle(input):null;return {left:r.left,right:r.right,top:r.top,bottom:r.bottom,border:s.borderBottomWidth,textAlign:s.textAlign,input:i?.width?{left:i.left,right:i.right,top:i.top,bottom:i.bottom,textLeft:i.left+parseFloat(style.borderLeftWidth)+parseFloat(style.paddingLeft),textAlign:style.textAlign}:null};}))
  }))};
}
function checkGeometry(value,width,height){
 assert.equal(value.viewport.width,width,'Fixture must use the requested viewport width');
 assert.equal(value.viewport.height,height,'Fixture must use the requested viewport height');
 assert.equal(value.viewport.rootScroll,height,'Hidden labels must not create a second page scroller');
 assert.equal(value.viewport.rootTop,0,'The document root must remain stationary');
 assert(value.viewport.rootWidth<=width,'Tables must use their local horizontal scroller');
 for(const table of value.tables)for(const row of table.rows){
  assert(row.every(cell=>Math.abs(cell.bottom-row[0].bottom)<.5),'Joint row rules must share a baseline');
  assert(row.every(cell=>cell.border===row[0].border),'Joint labels and cells must share row-rule visibility');
  for(const [index,cell] of row.entries()){
   if(!cell.input)continue;
   const head=table.headers[index],input=cell.input;
   assert(Math.abs((head.left+head.right-input.left-input.right)/2)<.5,'Header and input must share the same column centre');
   assert.equal(head.textAlign,input.textAlign,'Numeric headers and inputs must use the same text alignment');
   assert(Math.abs(head.textLeft-input.textLeft)<.8,'Numeric header text must align with the input text inset');
  }
 }
}
app.whenReady().then(async()=>{
 fs.mkdirSync(output,{recursive:true});
 const win=new BrowserWindow({width:1000,height:720,frame:false,enableLargerThanScreen:true,show:false,webPreferences:{preload:fixture.preload,sandbox:false,backgroundThrottling:false}});
 await win.loadFile(fixture.index);win.show();
 await win.webContents.executeJavaScript('document.fonts.ready');
 await win.webContents.executeJavaScript(`term?.dispose();term=null;fit=null;window.Terminal=undefined;window.profileLayoutScene=${scene};window.profileLayoutGeometry=${geometry};void 0`);
 const report=[];
 for(const [width,height] of [[2559,1405],[1000,720]])for(const name of ['parameters-observation','parameters-csp','parameters-motion']){
  win.setContentSize(width,height);
  await win.webContents.executeJavaScript(`window.profileLayoutScene(${JSON.stringify(name)},'light');window.profileLayoutDetail=document.querySelector('.setup-main details.calibration-locks');void 0`);await pause(500);
  const first=await win.webContents.executeJavaScript('window.profileLayoutGeometry()');
  checkGeometry(first,width,height);
  if(name==='parameters-csp'){
   await win.webContents.executeJavaScript(`(()=>{const s=document.querySelector('.studio-page-body'),table=document.querySelector('.setup-main .profile-control-table'),save=document.querySelector('.setup-save-context');s.scrollTop+=table.getBoundingClientRect().top-save.getBoundingClientRect().bottom-12;})()`);await pause(100);
   fs.writeFileSync(path.join(output,`${name}-${width}-mdk.png`),(await win.webContents.capturePage()).toPNG());
  }
  await win.webContents.executeJavaScript("document.querySelector('.studio-page-body').scrollTop=1e6;window.profileLayoutDetail.querySelector(':scope > summary').click()");await pause(700);
  const expanded=await win.webContents.executeJavaScript('window.profileLayoutGeometry()');
  checkGeometry(expanded,width,height);
  await win.webContents.executeJavaScript("document.querySelector('.studio-page-body').scrollTop=1e6");await pause(100);
  const bottom=await win.webContents.executeJavaScript('window.profileLayoutGeometry()');
  checkGeometry(bottom,width,height);
  const scroller=bottom.boxes.find(x=>x.class==='studio-page-body'),last=bottom.boxes.find(x=>x.class.includes('setup-main'));
  if(scroller.scrollHeight>scroller.clientHeight+1)assert(scroller.bottom-last.bottom<=scroller.paddingBottom+2,'Page bottom must end at the real editor content');
  fs.writeFileSync(path.join(output,`${name}-${width}-bottom.png`),(await win.webContents.capturePage()).toPNG());
  const motion=await win.webContents.executeJavaScript(`(async()=>{
   const detail=window.profileLayoutDetail,summary=detail.querySelector(':scope > summary'),scroller=document.querySelector('.studio-page-body');
   const input=[...detail.querySelectorAll('input[data-profile-field]')].find(n=>n.type!=='checkbox');
   if(input)input.focus({preventScroll:true});
   const saved=input?.value,samples=[];summary.click();
   const focusReturned=!input||document.activeElement===summary;
   const deadline=performance.now()+2000;
   do{await new Promise(requestAnimationFrame);samples.push({root:document.scrollingElement.scrollHeight,top:scroller.scrollTop,max:scroller.scrollHeight-scroller.clientHeight});}
   while(detail.hasAttribute('data-studio-disclosure-animating')&&performance.now()<deadline);
   const result={focusReturned,inputPreserved:!input||input.isConnected&&input.value===saved,open:detail.open,animating:detail.hasAttribute('data-studio-disclosure-animating'),samples};
   summary.click();await new Promise(resolve=>setTimeout(resolve,32));summary.click();await new Promise(resolve=>setTimeout(resolve,32));summary.click();
   await new Promise(resolve=>setTimeout(resolve,700));
   result.reversedOpen=detail.open;result.reversedAnimating=detail.hasAttribute('data-studio-disclosure-animating');
   return result;
  })()`);
  assert(motion.focusReturned&&motion.inputPreserved,'Disclosure must retain the editor and restore focus on close');
  assert(!motion.open&&!motion.animating,'Closing must settle to the native collapsed layout: '+JSON.stringify({name,width,...motion,samples:motion.samples.slice(-2)}));
  assert(motion.reversedOpen&&!motion.reversedAnimating,'Rapid reversal must settle to the final open state');
  assert(motion.samples.every(sample=>sample.root===height),'Animated disclosures must not extend the document root');
  for(let i=1;i<motion.samples.length;i++)assert(Math.abs(motion.samples[i].top-Math.min(motion.samples[i-1].top,motion.samples[i].max))<=1.1,'Bottom scroll must follow the shrinking content range');
  const reversed=await win.webContents.executeJavaScript('window.profileLayoutGeometry()');
  checkGeometry(reversed,width,height);
  report.push({width,height,scene:name,first,expanded,bottom,reversed,motion});
 }
 fs.writeFileSync(path.join(output,'report.json'),JSON.stringify(report,null,2));
 assert.equal(sourceHash(),sourceSha256,'Production renderer changed during layout verification');
 console.log('PROFILE_LAYOUT_PASS '+JSON.stringify({cases:report.length,report:path.join(output,'report.json'),sourceSha256,scope:'production renderer; no device API'}));
 win.destroy();app.quit();
}).catch(error=>{console.error(error.stack);app.exit(1);});
