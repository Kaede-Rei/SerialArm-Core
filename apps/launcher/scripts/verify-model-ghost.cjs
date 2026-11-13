/* Actual Three.js/Electron ghost preparation and first-draw checks, no backend/device API.
 * DISPLAY=:1 node_modules/.bin/electron --no-sandbox scripts/verify-model-ghost.cjs
 * Optional --mesh-dir /path/to/STL --report /tmp/model-ghost.json
 * --baseline --view-module /tmp/old-model-view.mjs records an unmodified view;
 * --compare /tmp/before.json also checks exact outline bytes and relative latency.
 */
const fs=require('node:fs'),os=require('node:os'),path=require('node:path');
const {pathToFileURL}=require('node:url');
const {app,BrowserWindow,ipcMain}=require('electron');
const root=path.resolve(__dirname,'..'),args=process.argv.slice(2);
const argument=(flag,fallback)=>{const i=args.indexOf(flag);return i<0?fallback:args[i+1];};
const meshDirectory=argument('--mesh-dir',''),reportPath=path.resolve(argument('--report','/tmp/serialarm-model-ghost.json'));
if(!reportPath.startsWith('/tmp/'))throw Error('Performance report must be written in /tmp');
const baseline=args.includes('--baseline'),comparePath=argument('--compare','');
const viewModule=path.resolve(argument('--view-module',path.join(root,'renderer/model-view.js')));
const meshes=meshDirectory?fs.readdirSync(meshDirectory).filter(name=>/\.stl$/i.test(name)).sort().map(name=>path.resolve(meshDirectory,name)):[];
app.whenReady().then(async()=>{
 const temporary=fs.mkdtempSync(path.join(os.tmpdir(),'serialarm-model-ghost-'));
 const allowed=new Set(meshes);let reads=0;
 ipcMain.handle('ghost-fixture-resource',(_event,file)=>{
  if(!allowed.has(file))throw Error('Unexpected fixture resource');reads++;
  return {data:fs.readFileSync(file).toString('base64'),extension:'.stl'};
 });
 fs.writeFileSync(path.join(temporary,'preload.cjs'),"const{contextBridge,ipcRenderer}=require('electron');contextBridge.exposeInMainWorld('ghostFixture',{read:file=>ipcRenderer.invoke('ghost-fixture-resource',file)});");
 fs.writeFileSync(path.join(temporary,'index.html'),'<!doctype html><html><head><meta http-equiv="Content-Security-Policy" content="default-src \'self\'; style-src \'self\' \'unsafe-inline\'; script-src \'self\';"><style>body{margin:0;background:#f5f5f7}#view{width:100vw;height:100vh}</style></head><body><div id="view"></div><script type="module" src="fixture.mjs"></script></body></html>');
 fs.writeFileSync(path.join(temporary,'fixture.mjs'),`
 import {SerialArmModelView} from ${JSON.stringify(pathToFileURL(viewModule).href)};
 import * as THREE from ${JSON.stringify(pathToFileURL(path.join(root,'renderer/vendor/three/build/three.module.js')).href)};
 const files=${JSON.stringify(meshes)},checks=[];
 const check=(value,name)=>{if(!value)throw Error(name);checks.push(name);};
 const visual=file=>({name:file,geometry:{type:'mesh',path:file,scale:[1,1,1]},origin:{xyz:[0,0,0],rpy:[0,0,0]},rgba:[.6,.65,.7,1]});
 const links=files.length?files.map((file,index)=>({name:'link'+index,visuals:[visual(file)],collisions:[visual(file)]})):
  Array.from({length:4},(_,index)=>({name:'link'+index,visuals:[{geometry:{type:'sphere',radius:.08},origin:{xyz:[0,0,0],rpy:[0,0,0]},rgba:[.6,.65,.7,1]}],collisions:[]}));
 const frames=links.map((link,index)=>({name:link.name,position:[.05*index,0,.12*index],quaternion:[0,0,0,1]}));
 const native={state:{frames}},payload={model:{links,joints:[]},native};
 const tasks=[];const observer=new PerformanceObserver(list=>tasks.push(...list.getEntries().map(x=>({start:x.startTime,duration:x.duration}))));observer.observe({type:'longtask',buffered:true});
 let heartbeats=0;const heartbeat=setInterval(()=>heartbeats++,10);
 try {
 const view=new SerialArmModelView(document.getElementById('view'),file=>window.ghostFixture.read(file),'zh-CN');window.view=view;
 const started=performance.now(),loaded=await view.setModel(payload);check(loaded,'Actual model loaded');
 check(!view.ghostRoot?.visible,'Prepared resources do not display an invented target pose');
 view.setLayers({...view.layers,collision:false,jointAxes:false});view.renderOnce();await new Promise(r=>requestAnimationFrame(()=>requestAnimationFrame(r)));
 const loadedAt=performance.now(),beforePrograms=view.renderer.info.programs?.length||0,beforeDrawCalls=view.renderer.info.render.calls;
 const measuredBefore=[...view.linkGroups].map(([name,group])=>[name,group.matrix.toArray()]);
 const ghostNative={state:{frames:frames.map(frame=>({...frame,position:[frame.position[0]+.12,frame.position[1],frame.position[2]+.04]}))}};
 const firstStart=performance.now();view.setGhostPose(ghostNative);const firstSetEnd=performance.now();check(view.renderOnce(),'First ghost really drawn');const firstDrawEnd=performance.now();
 const firstDrawCalls=view.renderer.info.render.calls;
 check(firstDrawCalls>beforeDrawCalls,'First ghost submits additional real draw calls');
 const framebuffer=new Uint8Array(view.gl.drawingBufferWidth*view.gl.drawingBufferHeight*4);
 view.gl.readPixels(0,0,view.gl.drawingBufferWidth,view.gl.drawingBufferHeight,view.gl.RGBA,view.gl.UNSIGNED_BYTE,framebuffer);
 let coloredPixels=0,tealPixels=0;
 for(let i=0;i<framebuffer.length;i+=4){if(framebuffer[i+3])coloredPixels++;if(framebuffer[i+1]>framebuffer[i]+20&&framebuffer[i+2]>framebuffer[i]+20&&framebuffer[i+3])tealPixels++;}
 check(coloredPixels>100&&tealPixels>10,'Actual framebuffer contains non-background teal target pixels');
 check(view.ghostRoot?.visible,'Ghost visible only after a real target pose');
 for(const frame of ghostNative.state.frames){const group=view.ghostLinkGroups.get(frame.name);check(group?.visible&&Math.abs(group.matrix.elements[12]-frame.position[0])<1e-12,'Ghost matrix matches the supplied target frame '+frame.name);}
 check(JSON.stringify(measuredBefore)===JSON.stringify([...view.linkGroups].map(([name,group])=>[name,group.matrix.toArray()])),'Ghost never changes measured model matrices');
 let outlines=0,meshes=0;view.ghostRoot.traverse(node=>{if(node.isMesh){meshes++;check(node.material.color.getHex()===0x16b8b0&&node.material.opacity===.2&&!node.material.depthWrite,'Teal filled overlay appearance retained');}if(node.isLineSegments){outlines++;check(node.material.color.getHex()===0x079e98&&node.material.opacity===.9,'Teal outline appearance retained');}});
 check(outlines===meshes&&meshes>0,'Every full visual mesh keeps its outline');
 const secondStart=performance.now();view.setGhostPose(ghostNative);view.renderOnce();const secondEnd=performance.now();
 const cachedOutlines=[];view.ghostRoot.traverse(node=>{if(node.isLineSegments)cachedOutlines.push(node.geometry)});
 view.clearGhostPose();check(!view.ghostRoot,'Clear removes target preview before another is supplied');
 const recreateStart=performance.now();view.setGhostPose(ghostNative);view.renderOnce();const recreateEnd=performance.now();
 if(!${baseline}){let i=0;view.ghostRoot.traverse(node=>{if(node.isLineSegments)check(node.geometry===cachedOutlines[i++],'Clearing a pose does not rebuild immutable outlines')});}
 view.setGhostPose({state:{frames:[]}});check(!view.ghostRoot.visible,'Missing target frames cannot leave stale target visible');
 view.setGhostPose(ghostNative);view.renderOnce();
 const afterPrograms=view.renderer.info.programs?.length||0;
 const edgeSignatures=[];view.ghostRoot.traverse(node=>{if(node.isLineSegments){const a=node.geometry.getAttribute('position').array;let hash=2166136261;const bytes=new Uint8Array(a.buffer,a.byteOffset,a.byteLength);for(const value of bytes)hash=Math.imul(hash^value,16777619);edgeSignatures.push({count:a.length,hash:hash>>>0})}});
 const output={checks,edgeSignatures,beforeDrawCalls,firstDrawCalls,framebuffer:{coloredPixels,tealPixels},geometryFiles:files.length,visualMeshes:meshes,load_ms:loadedAt-started,firstGhostSet_ms:firstSetEnd-firstStart,firstGhostDraw_ms:firstDrawEnd-firstSetEnd,firstGhostTotal_ms:firstDrawEnd-firstStart,repeatedGhost_ms:secondEnd-secondStart,recreatedGhost_ms:recreateEnd-recreateStart,programsBefore:beforePrograms,programsAfter:afterPrograms,heartbeats,renderer:view.getRenderState()};
 await new Promise(r=>setTimeout(r,30));output.longTasks=tasks;clearInterval(heartbeat);observer.disconnect();
 window.ghostResult=output;
 window.ghostCleanup=async()=>{
  if(!${baseline}){
   const oldEdges=[...view.ghostEdges.values()];let disposedEdges=0;for(const edge of oldEdges)edge.addEventListener('dispose',()=>disposedEdges++);
   const primitive={model:{links:[{name:'replacement',visuals:[{geometry:{type:'box',size:[.1,.1,.1]},origin:{}}]}],joints:[]},native:{state:{frames:[{name:'replacement',position:[0,0,0],quaternion:[0,0,0,1]}]}}};
   check(await view.setModel(primitive),'Replacement model prepares its own resources');
   check(disposedEdges===oldEdges.length&&view.meshGeometryCache.size===0&&view.ghostEdges.size===1,'Model replacement releases all prior cached outlines and resources');
   const pending=view.setModel(primitive);view.dispose();check(await pending===false,'Disposal cancels in-progress preparation without a ready preview');
   check(view.geometryWorker===null&&view.geometryJobs.size===0&&view.ghostEdges.size===0,'Disposal terminates the geometry worker and clears every pending job');
  }else view.dispose();
  view.dispose();return checks;
 };
 }catch(error){clearInterval(heartbeat);observer.disconnect();window.ghostError=error.stack;}
 `);
 const win=new BrowserWindow({width:1100,height:800,show:false,webPreferences:{preload:path.join(temporary,'preload.cjs'),contextIsolation:true,nodeIntegration:false,sandbox:true,backgroundThrottling:false}});
 try{
  await win.loadFile(path.join(temporary,'index.html'));win.showInactive();
  let result;
  for(let attempt=0;attempt<1800;attempt++){
   result=await win.webContents.executeJavaScript('({result:window.ghostResult,error:window.ghostError})');
   if(result.error)throw Error(result.error);if(result.result)break;await new Promise(r=>setTimeout(r,100));
  }
  if(!result?.result)throw Error('Ghost fixture timed out');
  const report={baseline,resourceReads:reads,...result.result};
  if(!baseline&&meshes.length&&reads!==meshes.length)throw Error('Repeated STL geometry was reread instead of shared');
  if(comparePath){const previous=JSON.parse(fs.readFileSync(comparePath,'utf8'));if(JSON.stringify(previous.edgeSignatures)!==JSON.stringify(report.edgeSignatures))throw Error('Optimized ghost changed the original 25-degree outline geometry');if(report.firstGhostSet_ms>=previous.firstGhostSet_ms*.25)throw Error('First ghost preparation did not improve against the same-model baseline');}
  fs.writeFileSync(reportPath.replace(/\.json$/,'.png'),(await win.webContents.capturePage()).toPNG());
  report.checks=await win.webContents.executeJavaScript('window.ghostCleanup()');
  fs.writeFileSync(reportPath,JSON.stringify(report,null,2));
  console.log('MODEL_GHOST_PASS '+JSON.stringify({baseline,resourceReads:reads,firstGhostSet_ms:report.firstGhostSet_ms,firstGhostDraw_ms:report.firstGhostDraw_ms,firstGhostTotal_ms:report.firstGhostTotal_ms,load_ms:report.load_ms,checks:report.checks.length,report:reportPath}));
 }finally{if(!win.isDestroyed())win.destroy();ipcMain.removeHandler('ghost-fixture-resource');fs.rmSync(temporary,{recursive:true,force:true});}
 app.exit(0);
}).catch(error=>{console.error(error.stack);app.exit(1)});
