const {contextBridge, ipcRenderer} = require('electron');
contextBridge.exposeInMainWorld('serialArm', {
  request: async (method, params) => {
    const result = await ipcRenderer.invoke('launcher:request', method, params);
    if (result && typeof result === 'object' && result.__serialarm_error) {
      const detail = result.__serialarm_error;
      const error = new Error(detail.message || '后台请求失败');
      error.serialarm = detail;
      throw error;
    }
    return result;
  },
  prefs: () => ipcRenderer.invoke('launcher:prefs'),
  savePrefs: values => ipcRenderer.invoke('launcher:save-prefs', values),
  selectProfile: () => ipcRenderer.invoke('launcher:select-profile'),
  selectTrajectory: () => ipcRenderer.invoke('launcher:select-trajectory'),
  selectDescription: () => ipcRenderer.invoke('launcher:select-description'),
  selectProfilePackage: () => ipcRenderer.invoke('launcher:select-profile-package'),
  selectDestination: () => ipcRenderer.invoke('launcher:select-destination'),
  selectResources: () => ipcRenderer.invoke('launcher:select-resources'),
  window: action => ipcRenderer.invoke('launcher:window', action),
  clipboard: {
    read: () => ipcRenderer.invoke('launcher:clipboard', 'read'),
    write: text => ipcRenderer.invoke('launcher:clipboard', 'write', text),
  },
  terminalMenu: options => ipcRenderer.invoke('launcher:terminal-menu', options),
  modelResource: file => ipcRenderer.invoke('launcher:model-resource', file),
  onEvent: callback => {
    const listener = (_, data) => callback(data);
    ipcRenderer.on('launcher:event', listener);
    return () => ipcRenderer.removeListener('launcher:event', listener);
  }
});
