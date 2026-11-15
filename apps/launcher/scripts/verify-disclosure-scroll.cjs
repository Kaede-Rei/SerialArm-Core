/* Production disclosure motion and scroll continuity; the fixture rejects device APIs.
 * DISPLAY=:1 node_modules/.bin/electron --no-sandbox scripts/verify-disclosure-scroll.cjs */
const assert=require('node:assert/strict');
const fs=require('node:fs'),os=require('node:os'),path=require('node:path');
const {app,BrowserWindow}=require('electron');
const {buildFixture}=require('./studio-ui-fixture.cjs');
const directory=fs.mkdtempSync(path.join(os.tmpdir(),'serialarm-disclosure-scroll-'));
const output=process.argv.includes('--out')?path.resolve(process.argv[process.argv.indexOf('--out')+1]):directory;
const fixture=buildFixture(path.join(directory,'fixture'));
const renderSource=fs.readFileSync(path.join(__dirname,'render-studio-ui.cjs'),'utf8');
const scene=renderSource.slice(renderSource.indexOf('function setScene('),renderSource.indexOf('function snapshot('));
app.setPath('userData',path.join(directory,'electron-userdata'));
app.commandLine.appendSwitch('force-device-scale-factor','1');
function disclosureChecks(){
 const pause=ms=>new Promise(resolve=>setTimeout(resolve,ms)),frame=()=>new Promise(requestAnimationFrame);
 const result={cases:[],checks:[]};
 const check=(ok,name,evidence)=>{if(!ok)throw Error(name+' '+JSON.stringify(evidence||''));result.checks.push(name);};
 const settle=async node=>{const deadline=performance.now()+2000;do{await frame();}while(node.hasAttribute('data-studio-disclosure-animating')&&performance.now()<deadline);check(!node.hasAttribute('data-studio-disclosure-animating'),'disclosure settles to natural layout');};
 return (async()=>{
  for(const scene of ['parameters-motion','parameters-csp']){
   window.disclosureScrollScene(scene,'light');await document.fonts.ready;await pause(350);
   const detail=document.querySelector('.setup-main details.calibration-locks');
   check(!!detail,'production parameter group includes its boundary disclosure '+scene);
   const summary=detail.firstElementChild,scroller=document.querySelector('.studio-page-body');
   if(!detail.open){summary.click();await settle(detail);}
   scroller.scrollTop=scroller.scrollHeight;await frame();
   const input=[...detail.querySelectorAll('input[data-profile-field]')].find(node=>node.type!=='checkbox');
   if(input)input.focus({preventScroll:true});
   const original=input?.value,before={top:scroller.scrollTop,max:scroller.scrollHeight-scroller.clientHeight,height:detail.getBoundingClientRect().height};
   check(before.top>100,'actual long parameter page reaches a scrolled bottom '+scene,before);
   summary.click();
   const immediate={top:scroller.scrollTop,max:scroller.scrollHeight-scroller.clientHeight,height:detail.getBoundingClientRect().height};
   check(!detail.open,'native open state changes synchronously '+scene);
   check(Math.abs(immediate.top-before.top)<=1,'target measurement preserves scroll before the first animation frame '+scene,{before,immediate});
   check(!input||document.activeElement===summary,'closing returns editor focus without scrolling '+scene);
   const samples=[immediate],deadline=performance.now()+2000;
   do{await frame();samples.push({top:scroller.scrollTop,max:scroller.scrollHeight-scroller.clientHeight,height:detail.getBoundingClientRect().height});}while(detail.hasAttribute('data-studio-disclosure-animating')&&performance.now()<deadline);
   check(!detail.hasAttribute('data-studio-disclosure-animating'),'closing clears temporary presentation '+scene);
   check(samples.every((sample,index)=>!index||Math.abs(sample.top-Math.min(samples[index-1].top,sample.max))<=1.1),'bottom scroll follows only the actual shrinking range '+scene,samples);
   check(!input||input.isConnected&&input.value===original,'collapsed editor retains its value and DOM '+scene);
   summary.click();await pause(30);
   const reversingHeight=detail.getBoundingClientRect().height;summary.click();
   check(!detail.open&&Math.abs(detail.getBoundingClientRect().height-reversingHeight)<=1,'rapid reversal continues from the rendered height '+scene);
   await pause(30);summary.click();await settle(detail);
   check(detail.open,'rapid reversal finishes at the final requested open state '+scene);
   // A user scroll during the spring is independent of the one-off measurement restoration.
   scroller.scrollTop=Math.min(160,scroller.scrollHeight-scroller.clientHeight);summary.click();await frame();
   const userTop=Math.min(70,scroller.scrollHeight-scroller.clientHeight);scroller.scrollTop=userTop;await frame();
   check(Math.abs(scroller.scrollTop-Math.min(userTop,scroller.scrollHeight-scroller.clientHeight))<=1.1,'active spring does not lock subsequent user scrolling '+scene);
   await settle(detail);
   summary.click();await pause(30);document.documentElement.dataset.reduceMotion='true';await frame();await frame();
   check(detail.open&&!detail.hasAttribute('data-studio-disclosure-animating')&&!detail.style.getPropertyValue('--studio-disclosure-height'),'reduced motion finishes an active disclosure at its static state '+scene);
   summary.click();
   check(!detail.open&&!detail.hasAttribute('data-studio-disclosure-animating'),'reduced motion toggle remains synchronous '+scene);
   delete document.documentElement.dataset.reduceMotion;
   result.cases.push({scene,before,immediate,samples});
  }
  window.disclosureScrollScene('parameters-model','light');await pause(350);
  const source=document.querySelector('.parameter-source'),units=document.querySelector('.parameter-si-note');
  check(source&&!source.open&&units&&!units.open,'configuration source and SI information start collapsed');
  for(const [label,node] of [['source',source],['SI units',units]]){
   check(!!node,label+' disclosure is present');
   const summary=node.firstElementChild;
   const style=getComputedStyle(summary,'::before'),closed={content:style.content,width:style.width,height:style.height,borderRightWidth:style.borderRightWidth,borderBottomWidth:style.borderBottomWidth,transform:style.transform};
   check(['\"\"',"''"].includes(closed.content)&&parseFloat(closed.width)===8&&parseFloat(closed.height)===8&&parseFloat(closed.borderRightWidth)>0&&parseFloat(closed.borderBottomWidth)>0,label+' uses one empty CSS chevron rather than a character plus borders',{content:closed.content,width:closed.width,height:closed.height});
   summary.click();await settle(node);
   const opened=getComputedStyle(summary,'::before');
   check(node.open&&closed.transform!==opened.transform,label+' chevron follows the same disclosure state');
   summary.click();await settle(node);
  }
  const inspector=document.querySelector('.setup-inspector');let review=document.querySelector('#profile-review-defaults');
  check(!document.querySelector('.profile-setup-guide > summary')&&document.querySelector('.profile-guide-heading'),'configuration checks stay visible without a collapse control');
  for(const node of [source,units,review])if(!node.open){node.firstElementChild.click();await settle(node);}
  const field=document.querySelector('[data-profile-field=\'["core","model","gravity_scale","joint1"]\']');
  field.value='1.234567';field.dispatchEvent(new Event('input',{bubbles:true}));
  review=document.querySelector('#profile-review-defaults');
  check(review.open,'updating the draft preserves the hardware-review disclosure state');
  const scroller=document.querySelector('.studio-page-body');
  scroller.scrollTop=Math.max(150,document.querySelector('.setup-main').offsetTop-120);await frame();
  for(const group of ['motion','observation','safety','model']){
   const button=document.querySelector(`[data-profile-group="${group}"]`);
   button.focus({preventScroll:true});
   const before=scroller.scrollTop;button.click();await pause(350);
   const after=scroller.scrollTop,max=scroller.scrollHeight-scroller.clientHeight;
   check(inspector===document.querySelector('.setup-inspector')&&[source,units,review].every(node=>node.isConnected&&node.open),'parameter navigation retains mounted inspector and its expanded state '+group,{inspectorConnected:inspector.isConnected,sameInspector:inspector===document.querySelector('.setup-inspector'),details:[source,units,review].map(node=>({id:node.id,class:node.className,connected:node.isConnected,open:node.open}))});
   check(Math.abs(after-Math.min(before,max))<=1.1,'parameter navigation keeps the current editor scroll without jumping to the page top '+group,{before,after,max});
   check(document.activeElement===button,'parameter navigation preserves the selected navigation focus '+group);
   check(state.profileDirty&&state.profileEditor.core.model.gravity_scale.joint1===1.234567,'parameter navigation retains the edited draft '+group);
  }
  check(document.querySelector('[data-profile-field=\'["core","model","gravity_scale","joint1"]\']').value==='1.234567','returning to a parameter group restores its edited field');
  return result;
 })();
}
app.whenReady().then(async()=>{
 fs.mkdirSync(output,{recursive:true});
 const win=new BrowserWindow({width:1280,height:900,show:false,webPreferences:{preload:fixture.preload,sandbox:false,backgroundThrottling:false}});
 const deadline=setTimeout(()=>{console.error('Disclosure checks exceeded 60 seconds');app.exit(1);},60000);
 await win.loadFile(fixture.index);win.showInactive();
 await win.webContents.executeJavaScript(`term?.dispose();term=null;fit=null;window.Terminal=undefined;window.disclosureScrollScene=${scene};void 0`);
 const report={source:'Production renderer, CSS and motion; device-rejecting offline fixture',runtime:process.versions.electron,viewports:[]};
 for(const width of [1280,680]){
  win.setContentSize(width,900);
  const result=await win.webContents.executeJavaScript(`(${disclosureChecks})()`);
  report.viewports.push({width,...result});
 }
 fs.writeFileSync(path.join(output,'report.json'),JSON.stringify(report,null,2));
 clearTimeout(deadline);
 console.log('DISCLOSURE_SCROLL_PASS '+JSON.stringify({viewports:report.viewports.length,checks:report.viewports.reduce((n,value)=>n+value.checks.length,0),report:path.join(output,'report.json')}));
 win.destroy();app.quit();
}).catch(error=>{console.error(error.stack);app.exit(1);});
