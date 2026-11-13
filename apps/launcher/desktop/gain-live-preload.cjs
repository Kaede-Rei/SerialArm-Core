const {contextBridge,ipcRenderer}=require('electron');
contextBridge.exposeInMainWorld('gainMonitor',{
    current:()=>ipcRenderer.invoke('launcher:gain-live-current'),
    onSnapshot:callback=>{
        const listener=(_,snapshot)=>callback(snapshot);
        ipcRenderer.on('launcher:gain-live-snapshot',listener);
        return ()=>ipcRenderer.removeListener('launcher:gain-live-snapshot',listener);
    }
});
