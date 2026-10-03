const {contextBridge, ipcRenderer} = require('electron');
contextBridge.exposeInMainWorld('serialArm', {
  request: (method, params) => ipcRenderer.invoke('launcher:request', method, params),
  prefs: () => ipcRenderer.invoke('launcher:prefs'),
  savePrefs: values => ipcRenderer.invoke('launcher:save-prefs', values),
  selectProfile: () => ipcRenderer.invoke('launcher:select-profile'),
  selectResources: () => ipcRenderer.invoke('launcher:select-resources'),
  window: action => ipcRenderer.invoke('launcher:window', action),
  onEvent: callback => {
    const listener = (_, data) => callback(data);
    ipcRenderer.on('launcher:event', listener);
    return () => ipcRenderer.removeListener('launcher:event', listener);
  }
});
