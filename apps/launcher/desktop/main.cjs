const {app, BrowserWindow, dialog, ipcMain, clipboard, Menu} = require('electron');
const path = require('node:path');
const fs = require('node:fs');
const {pathToFileURL} = require('node:url');
const {spawnSync} = require('node:child_process');
const {Bridge} = require('./bridge.cjs');
const root = app.isPackaged ? path.join(process.resourcesPath, 'serial-arm') : path.resolve(__dirname, '../../..');
const entry = path.resolve(__dirname, '../renderer/index.html');
let win, bridge, closing = false;
let allowedModelResources = new Set();
app.setName('SerialArm Launcher');
app.setPath('userData', path.join(app.getPath('appData'), 'serial-arm'));
if (!app.requestSingleInstanceLock()) app.quit();
else {
  app.on('second-instance', () => { if (win) { if (win.isMinimized()) win.restore(); win.focus(); } });
  app.whenReady().then(start).catch(error => { dialog.showErrorBox('SerialArm Launcher', error.message); app.quit(); });
}
const prefsPath = () => path.join(app.getPath('userData'), 'launcher.json');
function readPrefs() { try { return JSON.parse(fs.readFileSync(prefsPath(), 'utf8')); } catch { return {}; } }
function savePrefs(values) {
  if (!values || typeof values !== 'object' || Array.isArray(values)) throw new Error('Invalid preferences');
  const next = {...readPrefs()};
  for (const k of ['profile', 'profile_file', 'serial_port', 'baudrate', 'bus', 'resource_paths', 'mode']) {
    if (typeof values[k] === 'string' && values[k].length < 8192 && !/[\0\n]/.test(values[k])) next[k] = values[k];
  }
  if (['system', 'dark', 'light'].includes(values.theme)) next.theme = values.theme;
  if (['zh-CN', 'en'].includes(values.language)) next.language = values.language;
  if (values.bounds && Number.isInteger(values.bounds.width) && Number.isInteger(values.bounds.height)) {
    next.bounds = {width: Math.max(1000, Math.min(2400, values.bounds.width)), height: Math.max(720, Math.min(1800, values.bounds.height))};
  }
  fs.mkdirSync(path.dirname(prefsPath()), {recursive: true});
  fs.writeFileSync(prefsPath() + '.tmp', JSON.stringify(next, null, 2), {mode: 0o600});
  fs.renameSync(prefsPath() + '.tmp', prefsPath()); return next;
}
function trusted(event) {
  if (!win || event.sender !== win.webContents || event.senderFrame.url !== pathToFileURL(entry).href) throw new Error('Untrusted IPC origin');
}
async function closeWindow() {
  if (closing) return;
  closing = true;
  try {
    if (['running', 'stopping'].includes(bridge.session.state)) {
      const choice = await dialog.showMessageBox(win, {type: 'question', buttons: ['停止并退出 / Stop and exit', '取消 / Cancel'], defaultId: 1, cancelId: 1,
        message: '运行会话仍在执行 / A session is still running', detail: '关闭窗口前先停止运行进程 / Stop the session before closing'});
      if (choice.response !== 0) return;
      await bridge.request('stop');
      await new Promise(resolve => {
        if (bridge.session.state === 'exited') return resolve();
        const done = message => { if (message.event === 'session' && message.state === 'exited') { clearTimeout(timer); bridge.off('event', done); resolve(); } };
        const timer = setTimeout(() => { bridge.off('event', done); resolve(); }, 8000);
        bridge.on('event', done);
      });
      if (bridge.session.state !== 'exited') {
        const force = await dialog.showMessageBox(win, {type: 'warning', buttons: ['强制结束并退出 / Force quit', '取消 / Cancel'], defaultId: 1, cancelId: 1,
          message: '进程尚未退出 / The session has not exited', detail: '强制结束会中断进程清理；请确认机器人状态 / Force quit interrupts process cleanup; check the robot state'});
        if (force.response !== 0) return;
        await bridge.request('stop', {force: true});
      }
    }
    savePrefs({bounds: win.getBounds()});
    await bridge.close(); win.destroy(); app.quit();
  } finally { closing = false; }
}
function packagedBackend() {
  const native = path.join(root, 'native');
  const distro = '/opt/ros/humble/setup.bash';
  const overlay = path.join(native, 'local_setup.bash');
  if (!fs.existsSync(distro)) throw new Error('ROS2 Humble 未安装，请先在 Ubuntu 22.04 安装 ROS2 Humble 与 MoveIt 2');
  if (!fs.existsSync(overlay)) throw new Error('发行包不包含完整 ROS2 工作区，请重新安装发行包');

  // The ROS2 installer places Profile configuration below share/ in a read-only
  // installation; use a user-owned resource overlay so Profile editing and
  // calibration work in both a mounted AppImage and a system-wide deb install
  const writable = path.join(app.getPath('userData'), 'profile-resources');
  const writableShare = path.join(writable, 'share');
  fs.mkdirSync(writableShare, {recursive: true});
  fs.cpSync(path.join(native, 'share'), writableShare,
    {recursive: true, force: false, errorOnExist: false});
  for (const folder of ['lib', 'lib64']) {
    const from = path.join(native, folder);
    const to = path.join(writable, folder);
    if (fs.existsSync(from)) {
      try {
        if (fs.lstatSync(to).isSymbolicLink() && fs.readlinkSync(to) !== from) fs.unlinkSync(to);
      } catch (error) { if (error.code !== 'ENOENT') throw error; }
      if (!fs.existsSync(to)) fs.symlinkSync(from, to, 'dir');
    }
  }

  // Source the host ROS2 environment and the bundled workspace's local setup;
  // native colcon entrypoints and Python bindings are then resolved at their
  // installed location, never at the build machine's checkout path
  const sourced = spawnSync('/bin/bash',
    ['-c', 'set -e; source "$1"; source "$2"; env -0', 'serial-arm', distro, overlay],
    {env: process.env, encoding: 'buffer', timeout: 20000, maxBuffer: 12 * 1024 * 1024});
  if (sourced.error || sourced.status !== 0) {
    throw new Error('ROS2 运行环境初始化失败: ' +
      ((sourced.stderr || '').toString('utf8').slice(-1200) || sourced.error?.message || sourced.status));
  }
  const env = {};
  for (const record of sourced.stdout.toString('utf8').split('\0')) {
    const idx = record.indexOf('=');
    if (idx > 0) env[record.slice(0, idx)] = record.slice(idx + 1);
  }
  const prepend = (key, value) => { env[key] = [value, env[key]].filter(Boolean).join(path.delimiter); };
  prepend('AMENT_PREFIX_PATH', writable);
  prepend('CMAKE_PREFIX_PATH', writable);
  prepend('SERIAL_ARM_RESOURCE_PATH', writable);
  prepend('SERIAL_ARM_RESOURCE_PATH', native);
  prepend('PYTHONPATH', path.join(root, 'python'));
  for (const folder of ['lib', 'lib64']) {
    if (fs.existsSync(path.join(native, folder))) prepend('LD_LIBRARY_PATH', path.join(native, folder));
  }
  prepend('PATH', path.join(native, 'bin'));
  env.SERIAL_ARM_BUNDLE_ROOT = root;
  env.SERIAL_ARM_BUNDLE_PREFIX = native;
  env.SERIAL_ARM_BUNDLE_PROFILE_PREFIX = writable;
  env.SERIAL_ARM_BUNDLE_ROS_DISTRO = 'humble';
  env.PYTHONNOUSERSITE = '1';
  return {python: '/usr/bin/python3', script: path.join(root, 'apps/launcher/backend/bridge.py'), env};
}

async function start() {
  const prefs = readPrefs();
  const backend = app.isPackaged ? packagedBackend() : {
    python: process.env.SERIAL_ARM_LAUNCHER_PYTHON || '/usr/bin/python3',
    script: path.resolve(__dirname, '../backend/bridge.py'), env: process.env,
  };
  bridge = new Bridge(backend.python, backend.script, {cwd: root, env: backend.env});
  await bridge.ready;
  win = new BrowserWindow({width: prefs.bounds?.width || 1300, height: prefs.bounds?.height || 900, minWidth: 1000, minHeight: 720,
    frame: false, backgroundColor: '#0a0a10', show: false,
    webPreferences: {preload: path.join(__dirname, 'preload.cjs'), contextIsolation: true, nodeIntegration: false, sandbox: true}});
  bridge.on('event', event => { if (win && !win.isDestroyed()) win.webContents.send('launcher:event', event); });
  win.webContents.setWindowOpenHandler(() => ({action: 'deny'}));
  win.webContents.on('will-navigate', event => event.preventDefault());
  win.on('close', event => { event.preventDefault(); closeWindow().catch(error => dialog.showErrorBox('SerialArm Launcher', error.message)); });
  win.once('ready-to-show', () => win.show());
  const methods = new Set(['status', 'profiles', 'profile_library', 'profile_source_inspect', 'profile_library_register', 'profile_library_remove', 'description_inspect', 'description_create_profile', 'readiness', 'readiness_mark', 'readiness_record', 'readiness_approve_mode', 'profile_editor_load', 'profile_editor_save', 'inspect', 'model', 'model_preview', 'workbench_start', 'workbench_request', 'workbench_stop', 'workbench_config_preview', 'workbench_config_save', 'workbench_export', 'model_calibration_load', 'model_calibration_model_alignment', 'model_calibration_records', 'model_calibration_preview_save', 'model_calibration_save', 'model_calibration_restore_config', 'model_calibration_export_friction', 'model_calibration_export_urdf', 'model_calibration_export_full_inertial', 'model_calibration_assess', 'model_calibration_advanced', 'model_calibration_recompute', 'calibration_tool_records', 'calibration_tool_load', 'calibration_tool_export', 'calibration_tool_export_record', 'geometry_fixed_state', 'geometry_fixed_begin', 'geometry_fixed_capture', 'geometry_fixed_solve', 'geometry_fixed_validate', 'geometry_fixed_discard', 'geometry_fixed_report', 'geometry_fixed_export_tcp', 'geometry_repeat_status', 'geometry_repeat_record_begin', 'geometry_repeat_record_stop', 'geometry_repeat_load', 'geometry_repeat_save', 'geometry_repeat_play', 'geometry_repeat_stop', 'trajectory_list', 'trajectory_save', 'trajectory_load', 'trajectory_rename', 'trajectory_remove', 'trajectory_export', 'trajectory_import', 'calibration_candidate_prepare', 'calibration_candidate_apply', 'calibration_candidate_restore', 'calibration_candidate_status', 'start', 'input', 'resize', 'stop']);
  ipcMain.handle('launcher:request', async (event, method, params = {}) => {
    trusted(event);
    if (method === 'logs') return bridge.output;
    if (typeof method !== 'string' || !methods.has(method)) return {__serialarm_error: {
      code: 'IPC_METHOD_NOT_ALLOWED', category: 'integration', severity: 'error',
      title: '前后端接口不匹配', message: '当前界面请求的功能没有在 Electron 中注册',
      advice: '请重新安装同一份完整工程，并检查 GUI 与后端是否来自同一份源码',
      technical: `Unknown launcher request: ${String(method).slice(0, 80)}`
    }};
    let result;
    try { result = await bridge.request(method, params); }
    catch (error) {
      // Electron invoke serializes thrown Error objects down to message-only
      // text. Return an explicit error envelope so the preload can restore
      // the typed operator diagnostics without losing code or context.
      return {__serialarm_error: error.serialarm || {
        code: 'BACKEND_UNAVAILABLE', category: 'communication', severity: 'error',
        title: '后台请求失败', message: error.message,
        advice: '请检查后台连接及运行日志', technical: error.stack || error.message
      }};
    }
    if (method === 'model' || method === 'description_inspect') {
      const meshFiles = method === 'description_inspect' ? (result?.preview?.mesh_files || []) : (result?.mesh_files || []);
      allowedModelResources = new Set(meshFiles.map(file => path.resolve(file)));
    }
    return result;
  });
  ipcMain.handle('launcher:prefs', event => { trusted(event); return {...readPrefs(), language: readPrefs().language || (app.getLocale().startsWith('zh') ? 'zh-CN' : 'en')}; });
  ipcMain.handle('launcher:save-prefs', (event, values) => { trusted(event); return savePrefs(values); });
  ipcMain.handle('launcher:select-profile', async event => { trusted(event); const result = await dialog.showOpenDialog(win, {properties: ['openFile'], filters: [{name: 'Robot Profiles', extensions: ['yaml', 'yml']}]}); return result.canceled ? '' : result.filePaths[0]; });
  ipcMain.handle('launcher:select-trajectory', async event => { trusted(event); const result = await dialog.showOpenDialog(win, {properties: ['openFile'], filters: [{name: 'SerialArm trajectory', extensions: ['zip']}]}); return result.canceled ? '' : result.filePaths[0]; });
  ipcMain.handle('launcher:select-description', async event => { trusted(event); const result = await dialog.showOpenDialog(win, {properties: ['openFile'], filters: [{name: 'SerialArm Description', extensions: ['zip', 'urdf', 'xacro']}]}); return result.canceled ? '' : result.filePaths[0]; });
  ipcMain.handle('launcher:select-profile-package', async event => { trusted(event); const result = await dialog.showOpenDialog(win, {properties: ['openDirectory']}); return result.canceled ? '' : result.filePaths[0]; });
  ipcMain.handle('launcher:select-destination', async event => { trusted(event); const result = await dialog.showOpenDialog(win, {properties: ['openDirectory', 'createDirectory']}); return result.canceled ? '' : result.filePaths[0]; });
  ipcMain.handle('launcher:select-resources', async event => { trusted(event); const result = await dialog.showOpenDialog(win, {properties: ['openDirectory']}); return result.canceled ? '' : result.filePaths[0]; });
  ipcMain.handle('launcher:clipboard', (event, action, text) => {
    trusted(event);
    if (action === 'read') return clipboard.readText();
    if (action !== 'write' || typeof text !== 'string' || text.length > 1024 * 1024) throw new Error('Invalid clipboard request');
    clipboard.writeText(text);
    return true;
  });
  ipcMain.handle('launcher:model-resource', (event, file) => {
    trusted(event);
    if (typeof file !== 'string' || file.length > 8192 || /[\0\n]/.test(file)) throw new Error('Invalid model resource path');
    const resolved = path.resolve(file);
    const extension = path.extname(resolved).toLowerCase();
    if (!['.stl', '.glb'].includes(extension)) throw new Error('Unsupported model resource type');
    if (!allowedModelResources.has(resolved)) throw new Error('Model resource was not resolved by the active model');
    const stat = fs.statSync(resolved);
    if (!stat.isFile() || stat.size > 64 * 1024 * 1024) throw new Error('Model resource is unavailable or too large');
    return {path: resolved, extension, data: fs.readFileSync(resolved).toString('base64')};
  });
  ipcMain.handle('launcher:terminal-menu', (event, options = {}) => {
    trusted(event);
    if (!options || typeof options !== 'object' || Array.isArray(options)) throw new Error('Invalid terminal menu request');
    const en = options.language === 'en';
    return new Promise(resolve => {
      let action = null;
      const item = (id, label, enabled = true) => ({id, label, enabled, click: () => { action = id; }});
      const menu = Menu.buildFromTemplate([
        item('copy', en ? 'Copy selection' : '复制所选', options.selection === true),
        item('copy-all', en ? 'Copy all output' : '复制全部输出'),
        item('paste', en ? 'Paste' : '粘贴', options.canPaste === true),
        {type: 'separator'},
        item('select-all', en ? 'Select all' : '全选'),
      ]);
      menu.popup({window: win, callback: () => resolve(action)});
    });
  });
  ipcMain.handle('launcher:window', (event, action) => {
    trusted(event);
    if (action === 'minimize') win.minimize();
    else if (action === 'maximize') win.isMaximized() ? win.unmaximize() : win.maximize();
    else if (action === 'close') return closeWindow();
    else throw new Error('Unknown window action');
  });
  await win.loadFile(entry);
}
app.on('window-all-closed', () => app.quit());
