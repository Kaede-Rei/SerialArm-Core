const {app, BrowserWindow, ipcMain} = require('electron');
const fs = require('node:fs');
const path = require('node:path');
app.requestSingleInstanceLock = () => true; // Avoid namespace socket restriction in this QA harness
app.setPath('appData', path.resolve(__dirname, '../../../../qa/electron-preferences'));
fs.mkdirSync(path.join(app.getPath('appData'), 'serial-arm'), {recursive: true});
fs.writeFileSync(path.join(app.getPath('appData'), 'serial-arm/launcher.json'), JSON.stringify({theme:'dark',language:'zh-CN'}));
const register = ipcMain.handle.bind(ipcMain);
ipcMain.handle = (channel, handler) => {
  if (channel === 'launcher:select-profile') return register(channel, () => path.resolve(__dirname, '../../../src/robot_supports/profiles/config/robot_profiles.yaml'));
  if (channel === 'launcher:request') {
    return register(channel, async (event, method, params) => {
      if (method === 'status') return {...await handler(event, method, params), installed: true, ros_distro: 'humble', ros2: '/opt/ros/humble/bin/ros2', terminal: '/test/bin/serial_arm_terminal', session: {state: 'idle'}};
      if (method === 'inspect') return {...await handler(event, method, params), available: {model:true,terminal:true,hardware:true,moveit:true}};
      return handler(event, method, params);
    });
  }
  return register(channel, handler);
};
require('../desktop/main.cjs');
app.whenReady().then(async () => {
  const wait = ms => new Promise(r => setTimeout(r, ms));
  let win;
  for(let i=0;i<50;i++){win=BrowserWindow.getAllWindows()[0];if(win && !win.webContents.isLoading())break;await wait(100);}
  await wait(1500);
  win.setSize(1300,900);
  await wait(300);
  const check = async (script, message) => { if(!await win.webContents.executeJavaScript(script))throw new Error(message); };
  await check("document.querySelector('#profile-select').value==='dm_arm_gray'", 'default profile missing');
  await win.webContents.executeJavaScript("document.querySelector('#source-external').click();document.querySelector('#browse-profile').click()");await wait(500);
  await check("document.querySelector('#profile_file').value.endsWith('robot_profiles.yaml') && document.querySelector('#profile-select').value==='dm_arm_gray'", 'external profile import failed');
  await win.webContents.executeJavaScript("document.querySelector('#source-builtin').click()");await wait(500);
  const screenshot=path.resolve(__dirname,'../../../docs/images');fs.mkdirSync(screenshot,{recursive:true});
  fs.writeFileSync(path.join(screenshot,'launcher-dark.png'),(await win.webContents.capturePage()).toPNG());
  await win.webContents.executeJavaScript("document.querySelector('[data-page=run]').click()");await wait(800);
  await check("document.querySelector('.xterm')!==null",'PTY terminal view missing');
  fs.writeFileSync(path.join(screenshot,'launcher-run.png'),(await win.webContents.capturePage()).toPNG());
  await win.webContents.executeJavaScript("document.querySelector('[data-mode=hardware]').click();document.querySelector('#launch-session').click()");await wait(500);
  await check("document.querySelector('#confirm-dialog').open && document.querySelector('#confirm-launch').disabled",'hardware confirmation missing');
  await win.webContents.executeJavaScript("document.querySelector('#hardware-confirmed').click()");
  await check("!document.querySelector('#confirm-launch').disabled",'confirmation does not enable launch');
  await win.webContents.executeJavaScript("document.querySelector('#cancel-launch').click();document.querySelector('[data-page=settings]').click()");await wait(300);
  await win.webContents.executeJavaScript("document.querySelector('[data-theme=light]').click();document.querySelector('[data-language=en]').click()");await wait(500);
  await check("document.documentElement.className==='light' && document.querySelector('.title-center').textContent==='Manipulator workspace'",'theme/language failed');
  fs.writeFileSync(path.join(screenshot,'launcher-light.png'),(await win.webContents.capturePage()).toPNG());
  await win.webContents.executeJavaScript("document.querySelector('[data-page=robot]').click()");await wait(300);
  await check("document.querySelector('#profile-select').value==='dm_arm_gray'",'profile not retained');
  // Confirm desktop IPC blocks unknown process commands
  await check("window.serialArm.request('shell',{}).then(()=>false,()=>true)",'unknown IPC accepted');
  console.log('ELECTRON_GUI_SMOKE_PASS');
  win.destroy();app.quit();
}).catch(error=>{console.error(error);app.exit(1)});
