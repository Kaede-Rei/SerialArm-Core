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
  graphics: {
    status: () => ipcRenderer.invoke('launcher:graphics-status'),
    report: report => ipcRenderer.invoke('launcher:graphics-report', report),
    exportDiagnostics: () => ipcRenderer.invoke('launcher:graphics-export'),
    setMode: (mode, options = {}) => ipcRenderer.invoke('launcher:graphics-mode', mode, options),
  },
  prefs: () => ipcRenderer.invoke('launcher:prefs'),
  savePrefs: values => ipcRenderer.invoke('launcher:save-prefs', values),
  selectProfile: () => ipcRenderer.invoke('launcher:select-profile'),
  selectGainCandidate: () => ipcRenderer.invoke('launcher:select-gain-candidate'),
  revealFile: file => ipcRenderer.invoke('launcher:reveal-file', file),
  selectTrajectory: () => ipcRenderer.invoke('launcher:select-trajectory'),
  selectDescription: () => ipcRenderer.invoke('launcher:select-description'),
  selectProfilePackage: () => ipcRenderer.invoke('launcher:select-profile-package'),
  selectDestination: () => ipcRenderer.invoke('launcher:select-destination'),
  selectResources: () => ipcRenderer.invoke('launcher:select-resources'),
  gainLive: {
    open: snapshot => ipcRenderer.invoke('launcher:gain-live-open', snapshot),
    publish: snapshot => ipcRenderer.send('launcher:gain-live-update', snapshot),
  },
  window: action => ipcRenderer.invoke('launcher:window', action),
  clipboard: {
    read: () => ipcRenderer.invoke('launcher:clipboard', 'read'),
    write: text => ipcRenderer.invoke('launcher:clipboard', 'write', text),
  },
  terminalMenu: options => ipcRenderer.invoke('launcher:terminal-menu', options),
  modelResource: file => ipcRenderer.invoke('launcher:model-resource', file),
  onCloseRequest: callback => {
    const listener = async (_, requestId) => {
      try { ipcRenderer.send('launcher:close-draft', {requestId, ...await callback()}); }
      catch (error) { ipcRenderer.send('launcher:close-draft', {requestId, error:String(error?.message || error)}); }
    };
    ipcRenderer.on('launcher:prepare-close', listener);
    ipcRenderer.send('launcher:close-ready');
    return () => ipcRenderer.removeListener('launcher:prepare-close', listener);
  },
  onEvent: callback => {
    const listener = (_, data) => callback(data);
    ipcRenderer.on('launcher:event', listener);
    return () => ipcRenderer.removeListener('launcher:event', listener);
  }
});
