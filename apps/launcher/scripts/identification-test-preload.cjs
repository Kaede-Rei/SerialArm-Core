const {contextBridge,ipcRenderer}=require('electron');
contextBridge.exposeInMainWorld('serialArm',{
 request:(method,params)=>ipcRenderer.invoke('identification-test:request',method,params),
 prefs:async()=>({language:'zh-CN',theme:'light'}),savePrefs:async()=>true,
 selectResources:async()=>'/tmp/serialarm-ui-export-test',window:()=>{},onEvent:()=>()=>{},
 clipboard:{write:async()=>{},read:async()=>''},terminalMenu:async()=>{},
 modelResource:file=>ipcRenderer.invoke('identification-test:resource',file)
});
