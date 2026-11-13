const { app, BrowserWindow, dialog, ipcMain, clipboard, Menu, shell } = require('electron');
const path = require('node:path');
const fs = require('node:fs');
const { pathToFileURL } = require('node:url');
const { nativeEnvironment, developmentBackend } = require('./native-runtime.cjs');
const { Bridge } = require('./bridge.cjs');
const { synchronizeResources } = require('./resources.cjs');
const { GraphicsRuntime, MODES: GRAPHICS_MODES } = require('./graphics.cjs');
const { GainLiveWindow } = require('./gain-live-window.cjs');
const root = app.isPackaged ? path.join(process.resourcesPath, 'serial-arm') : path.resolve(__dirname, '../../..');
const entry = path.resolve(__dirname, '../renderer/index.html');
let gainLiveWindow;
let win, bridge, closing = false, graphicsRestarting = false, runtimeTransitions = 0;
let rendererCloseReady = false, closeRequestSerial = 0, closeDraftRequest = null;
let allowedModelResources = new Set();
app.setName('SerialArm Studio');
app.setPath('userData', path.join(app.getPath('appData'), 'serial-arm'));
const graphics = new GraphicsRuntime(app, readPrefs());
if (!app.requestSingleInstanceLock()) app.quit();
else {
    app.on('second-instance', () => { if (win) { if (win.isMinimized()) win.restore(); win.focus(); } });
    app.whenReady().then(start).catch(async error => {
        if (app.commandLine.hasSwitch('smoke-test')) {
            process.stderr.write(error.stack + '\n');
            try { await bridge?.close(); } catch (closeError) { process.stderr.write(String(closeError.message || closeError) + '\n'); }
            app.exit(1);
        }
        else { dialog.showErrorBox('SerialArm Launcher', error.message); app.quit(); }
    });
}
function prefsPath() { return path.join(app.getPath('userData'), 'launcher.json'); }
function readPrefs() { try { return JSON.parse(fs.readFileSync(prefsPath(), 'utf8')); } catch { return {}; } }
function savePrefs(values) {
    if (!values || typeof values !== 'object' || Array.isArray(values)) throw new Error('Invalid preferences');
    const next = { ...readPrefs() };
    for (const k of ['profile', 'profile_file', 'serial_port', 'baudrate', 'bus', 'resource_paths', 'mode']) {
        if (typeof values[k] === 'string' && values[k].length < 8192 && !/[\0\n]/.test(values[k])) next[k] = values[k];
    }
    if (['system', 'dark', 'light'].includes(values.theme)) next.theme = values.theme;
    if (['zh-CN', 'en'].includes(values.language)) next.language = values.language;
    if (typeof values.fontLigatures === 'boolean') next.fontLigatures = values.fontLigatures;
    else if (typeof next.fontLigatures !== 'boolean') next.fontLigatures = true;
    delete next.compact;
    if (values.bounds && Number.isInteger(values.bounds.width) && Number.isInteger(values.bounds.height)) {
        next.bounds = { width: Math.max(1000, Math.min(2400, values.bounds.width)), height: Math.max(720, Math.min(1800, values.bounds.height)) };
    }
    fs.mkdirSync(path.dirname(prefsPath()), { recursive: true });
    fs.writeFileSync(prefsPath() + '.tmp', JSON.stringify(next, null, 2), { mode: 0o600 });
    fs.renameSync(prefsPath() + '.tmp', prefsPath()); return next;
}
function trusted(event) {
    if (!win || event.sender !== win.webContents || event.senderFrame.url !== pathToFileURL(entry).href) throw new Error('Untrusted IPC origin');
}
function waitForSessionExit() {
    return new Promise(resolve => {
        if (bridge.dead || bridge.session.state === 'exited') return resolve();
        const done = message => { if (bridge.dead || (message.event === 'session' && message.state === 'exited')) { clearTimeout(timer); bridge.off('event', done); resolve(); } };
        const timer = setTimeout(() => { bridge.off('event', done); resolve(); }, 8000);
        bridge.on('event', done);
    });
}
function windowDraftState() {
    // The renderer registers before creating an editor, so an unbooted window has no draft.
    if (!rendererCloseReady) return Promise.resolve({dirty:false});
    return new Promise((resolve, reject) => {
        const requestId = ++closeRequestSerial;
        const finish = (error, result) => {
            clearTimeout(timer); closeDraftRequest = null;
            if (error) reject(error); else resolve(result);
        };
        const timer = setTimeout(() => finish(new Error('无法读取未保存的参数，请返回窗口检查后重试 / Unable to read the parameter draft; check the window and retry')), 5000);
        closeDraftRequest = {requestId, finish};
        win.webContents.send('launcher:prepare-close', requestId);
    });
}
function acceptCloseDraft(event, result) {
    trusted(event);
    if (!closeDraftRequest || result?.requestId !== closeDraftRequest.requestId) return;
    if (typeof result.dirty !== 'boolean' || result.error) closeDraftRequest.finish(new Error(String(result.error || 'Invalid parameter draft state')));
    else closeDraftRequest.finish(null, {dirty:result.dirty});
}
async function closeWindow(options = {}) {
    if (closing) {
        if (options.graphicsRestart) throw new Error('应用正在退出，请等待退出完成 / Application shutdown is already in progress');
        return false;
    }
    if (options.graphicsRestart && !canRestartForGraphics()) throw new Error('请先正常断开运行会话，再重启图形模式 / Disconnect the runtime session before restarting graphics');
    closing = true;
    if (options.graphicsRestart) graphicsRestarting = true;
    try {
        const draft = await windowDraftState();
        if (draft.dirty) {
            const choice = await dialog.showMessageBox(win, {
                type:'question', buttons:[options.graphicsRestart ? '放弃修改并重启 / Discard and restart' : '放弃修改并退出 / Discard and exit', '取消 / Cancel'], defaultId:1, cancelId:1,
                message:'参数有未保存的修改 / Unsaved parameter changes',
                detail:'放弃修改后将丢失当前参数草稿 / Discarding loses the current parameter draft'
            });
            if (choice.response !== 0) return false;
        }
        if (!options.graphicsRestart && ['running', 'stopping'].includes(bridge.session.state)) {
            const choice = await dialog.showMessageBox(win, {
                type: 'question', buttons: ['停止并退出 / Stop and exit', '取消 / Cancel'], defaultId: 1, cancelId: 1,
                message: '运行会话仍在执行 / A session is still running', detail: '关闭窗口前先停止运行进程 / Stop the session before closing'
            });
            if (choice.response !== 0) return false;
            const stopMethod = bridge.session.mode === 'workbench' ? 'workbench_stop' : 'stop';
            let stopError;
            try { if (bridge.dead) throw new Error('后台已断开，无法确认机械臂状态 / Backend disconnected; robot state is unverified'); await bridge.request(stopMethod); }
            catch (error) { stopError = error; }
            if (!stopError) await waitForSessionExit();
            if (bridge.session.state !== 'exited') {
                const force = await dialog.showMessageBox(win, {
                    type: 'warning', buttons: ['已确认硬件安全，强制退出 / Hardware secured, force quit', '取消 / Cancel'], defaultId: 1, cancelId: 1,
                    message: '正常退出未完成 / Normal shutdown did not complete',
                    detail: '无法确认电机已失能，请先可靠支撑机械臂，必要时急停或断电；强制退出仅终止进程 / Motor disable is unverified; physically secure the robot before terminating the process' +
                        (stopError ? '\n\n' + String(stopError.message || stopError).slice(0, 1200) : '')
                });
                if (force.response !== 0) return false;
                if (!bridge.dead) {
                    try { await bridge.request(stopMethod, { force: true }); }
                    catch (error) { if (!bridge.dead) throw error; }
                    await waitForSessionExit();
                }
                if (!bridge.dead && bridge.session.state !== 'exited') throw new Error('控制进程尚未退出，请检查机器人状态后重试 / Control process has not exited, check the robot before retrying');
            }
        }
        savePrefs({ bounds: win.getBounds() });
        if (bridge.dead) await bridge.forceClose();
        else await bridge.close();
        if (options.beforeDestroy) options.beforeDestroy();
        if (options.graphicsRestart) app.relaunch({ args: process.argv.slice(1).filter(arg => !arg.startsWith('--graphics-mode') && arg !== '--enable-unsafe-swiftshader' && !arg.startsWith('--use-gl=') && !arg.startsWith('--use-angle=')) });
        win.destroy(); app.quit();
        return true;
    } finally { closing = false; graphicsRestarting = false; }
}
function canRestartForGraphics() {
    return bridge && !bridge.dead && !closing && runtimeTransitions === 0 && ['idle', 'exited'].includes(bridge.session.state) && bridge.pending.size === 0;
}
async function selectGraphicsMode(mode, options = {}) {
    if (!GRAPHICS_MODES.has(mode)) throw new Error('Unknown graphics mode');
    if (!options || typeof options !== 'object' || Array.isArray(options)) throw new Error('Invalid graphics options');
    if (options.restart && !canRestartForGraphics()) throw new Error('请先正常断开运行会话，再重启图形模式；不会自动停放或停止机械臂 / Disconnect the runtime session first; graphics switching does not stop or park the robot');
    if (mode === 'compatibility') {
        if (!graphics.compatibilityAvailable) throw new Error('软件三维运行库不完整，请重新安装 GUI / Software 3D runtime is incomplete; reinstall the GUI');
        if (readPrefs().graphicsUnsafeSoftware !== true && graphics.backendMode !== 'compatibility') {
            const consent = await dialog.showMessageBox(win, {
                type: 'warning',
                buttons: ['启用兼容三维 / Enable compatible 3D', '取消 / Cancel'], defaultId: 1, cancelId: 1,
                message: '兼容三维使用 Chromium 软件渲染 / Compatible 3D uses Chromium software rendering',
                detail: '此模式启用 enable-unsafe-swiftshader，降低 Chromium 对软件 WebGL 的安全保证；仅用于受信任的本地模型，复杂模型可能较慢；Electron 沙箱继续启用；切换图形后端需要重启应用 / This enables enable-unsafe-swiftshader with lower software WebGL security guarantees. Use trusted local models; complex scenes may run slowly. Electron sandboxing remains enabled. Changing the backend requires an application restart'
            });
            if (consent.response !== 0) return { ...graphics.status(), canceled: true };
        }
    }
    const commit = () => {
        const next = { ...readPrefs(), graphicsMode: mode };
        if (mode === 'compatibility') next.graphicsUnsafeSoftware = true;
        fs.mkdirSync(path.dirname(prefsPath()), { recursive: true });
        fs.writeFileSync(prefsPath() + '.tmp', JSON.stringify(next, null, 2), { mode: 0o600 });
        fs.renameSync(prefsPath() + '.tmp', prefsPath());
        graphics.select(mode);
    };
    const requiresRestart = mode !== 'rescue' && mode !== graphics.backendMode;
    if (options.restart && requiresRestart) {
        const restarted = await closeWindow({ graphicsRestart:true, beforeDestroy:commit });
        return { ...graphics.status(), restarting:restarted, canceled:!restarted };
    }
    commit();
    return graphics.status();
}
function packagedBackend() {
    const native = path.join(root, 'native');
    if (!fs.existsSync(path.join(native, 'lib/libserial_arm_core.so'))) {
        throw new Error('发行包缺少 Core 运行库，请重新安装完整发行包');
    }

    // The installer places Profile configuration below share/ in a read-only
    // installation; use a user-owned resource overlay so Profile editing and
    // calibration work in both a mounted AppImage and a system-wide deb install
    const writable = path.join(app.getPath('userData'), 'profile-resources');
    const writableShare = path.join(writable, 'share');
    fs.mkdirSync(writableShare, { recursive: true });
    const resourceConflicts = synchronizeResources(path.join(native, 'share'), writableShare);
    if (resourceConflicts.length) dialog.showErrorBox('SerialArm resources',
        '部分本地配置与安装包默认资源不同，已保留本地修改并生成 .incoming 候选，请比较后重新验证\n' +
        resourceConflicts.slice(0, 8).map(item => item.path).join('\n'));
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

    const env = nativeEnvironment(native, process.env, path.join(root, 'python'));
    const prepend = (key, value) => { env[key] = [value, env[key]].filter(Boolean).join(path.delimiter); };
    prepend('SERIAL_ARM_RESOURCE_PATH', writable);
    env.SERIAL_ARM_BUNDLE_ROOT = root;
    env.SERIAL_ARM_BUNDLE_PREFIX = native;
    env.SERIAL_ARM_BUNDLE_PROFILE_PREFIX = writable;
    const runtimeFile = path.join(root, 'runtime.json');
    const runtime = fs.existsSync(runtimeFile) ? JSON.parse(fs.readFileSync(runtimeFile, 'utf8')) : {};
    const components = runtime.installed_components || ['core', 'terminal', 'python',
        ...(fs.existsSync(path.join(native, 'share/serial_arm_ros2_control')) ? ['ros2'] : []),
        ...(fs.existsSync(path.join(native, 'share/dm_arm_no_gripper')) ? ['moveit'] : [])];
    env.SERIAL_ARM_BUNDLE_COMPONENTS = JSON.stringify(components);
    env.SERIAL_ARM_BUNDLE_ROS_DISTRO = runtime.ros_distro || 'humble';
    env.PYTHONNOUSERSITE = '1';
    return { python: '/usr/bin/python3', script: path.join(root, 'apps/launcher/backend/bridge.py'), env };
}

async function start() {
    if (app.commandLine.hasSwitch('no-sandbox') && process.env.SERIAL_ARM_ALLOW_NO_SANDBOX !== '1') {
        throw new Error('系统未启用 Electron 沙箱，请启用用户命名空间；如需明确接受禁用沙箱运行，请设置 SERIAL_ARM_ALLOW_NO_SANDBOX=1');
    }
    const prefs = readPrefs();
    const backend = app.isPackaged ? packagedBackend() : developmentBackend(root);
    bridge = new Bridge(backend.python, backend.script, { cwd: root, env: backend.env });
    await bridge.ready;
    win = new BrowserWindow({
        width: prefs.bounds?.width || 1300, height: prefs.bounds?.height || 900, minWidth: 1000, minHeight: 720,
        frame: false, backgroundColor: '#0a0a10', show: false,
        webPreferences: { preload: path.join(__dirname, 'preload.cjs'), contextIsolation: true, nodeIntegration: false, sandbox: true }
    });
    bridge.on('event', event => { if (win && !win.isDestroyed()) win.webContents.send('launcher:event', event); });
    graphics.attach(win.webContents);
    gainLiveWindow = new GainLiveWindow({BrowserWindow, ipcMain, owner:win, trusted, directory:__dirname});
    win.webContents.setWindowOpenHandler(() => ({ action: 'deny' }));
    win.webContents.on('will-navigate', event => event.preventDefault());
    win.on('close', event => { event.preventDefault(); closeWindow().catch(error => dialog.showErrorBox('SerialArm Launcher', error.message)); });
    ipcMain.on('launcher:close-ready', event => { trusted(event); rendererCloseReady = true; });
    ipcMain.on('launcher:close-draft', acceptCloseDraft);
    win.once('ready-to-show', () => { if (!app.commandLine.hasSwitch('smoke-test')) win.show(); });
    const methods = new Set(['profile_hardware_inspect', 'profile_hardware_import', 'trajectory_preview', 'calibration_gain_validate_trajectory', 'calibration_tool_records_page', 'calibration_candidates', 'trajectory_select', 'geometry_fixed_preview_tcp', 'geometry_fixed_apply_tcp', 'status', 'profiles', 'profile_library', 'profile_source_inspect', 'profile_library_register', 'profile_library_remove', 'description_inspect', 'description_create_profile', 'readiness', 'readiness_mark', 'readiness_record', 'readiness_approve_mode', 'profile_editor_load', 'profile_editor_save', 'inspect', 'model', 'model_preview', 'workbench_start', 'workbench_request', 'workbench_stop', 'workbench_config_preview', 'workbench_config_save', 'workbench_export', 'model_calibration_load', 'model_calibration_model_alignment', 'model_calibration_records', 'model_calibration_preview_save', 'model_calibration_save', 'model_calibration_restore_config', 'model_calibration_export_friction', 'model_calibration_export_urdf', 'model_calibration_export_full_inertial', 'model_calibration_assess', 'model_calibration_advanced', 'model_calibration_recompute', 'model_calibration_idim', 'model_calibration_idim_audit', 'model_calibration_idim_export', 'calibration_tool_records', 'calibration_tool_load', 'calibration_tool_export', 'calibration_tool_export_record', 'geometry_fixed_state', 'geometry_fixed_begin', 'geometry_fixed_capture', 'geometry_fixed_solve', 'geometry_fixed_validate', 'geometry_fixed_discard', 'geometry_fixed_report', 'geometry_fixed_export_tcp', 'geometry_repeat_status', 'geometry_repeat_record_begin', 'geometry_repeat_record_stop', 'geometry_repeat_load', 'geometry_repeat_save', 'geometry_repeat_play', 'geometry_repeat_stop', 'trajectory_list', 'trajectory_save', 'trajectory_load', 'trajectory_rename', 'trajectory_remove', 'trajectory_export', 'trajectory_import', 'calibration_candidate_prepare', 'calibration_candidate_apply', 'calibration_candidate_restore', 'calibration_candidate_status', 'start', 'input', 'resize', 'stop']);
    ipcMain.handle('launcher:request', async (event, method, params = {}) => {
        trusted(event);
        if (method === 'logs') return bridge.output;
        if (typeof method !== 'string' || !methods.has(method)) return {
            __serialarm_error: {
                code: 'IPC_METHOD_NOT_ALLOWED', category: 'integration', severity: 'error',
                title: '前后端接口不匹配', message: '当前界面请求的功能没有在 Electron 中注册',
                advice: '请重新安装同一份完整工程，并检查 GUI 与后端是否来自同一份源码',
                technical: `Unknown launcher request: ${String(method).slice(0, 80)}`
            }
        };
        if (graphicsRestarting) return { __serialarm_error: { code: 'GRAPHICS_RESTART_PENDING', category: 'integration', severity: 'error', title: '应用正在退出', message: '图形模式重启期间不能提交运行请求 / Runtime requests are unavailable during the graphics restart', advice: '', technical: 'Graphics restart is in progress' } };
        const transition = ['start', 'workbench_start', 'stop', 'workbench_stop'].includes(method);
        if (transition) runtimeTransitions += 1;
        let result;
        try { result = await bridge.request(method, params); }
        catch (error) {
            // Electron invoke serializes thrown Error objects down to message-only
            // text. Return an explicit error envelope so the preload can restore
            // the typed operator diagnostics without losing code or context
            return {
                __serialarm_error: error.serialarm || {
                    code: 'BACKEND_UNAVAILABLE', category: 'communication', severity: 'error',
                    title: '后台请求失败', message: error.message,
                    advice: '请检查后台连接及运行日志', technical: error.stack || error.message
                }
            };
        }
        finally { if (transition) runtimeTransitions -= 1; }
        if (method === 'model' || method === 'description_inspect') {
            const meshFiles = method === 'description_inspect' ? (result?.preview?.mesh_files || []) : (result?.mesh_files || []);
            allowedModelResources = new Set(meshFiles.map(file => path.resolve(file)));
        }
        return result;
    });
    ipcMain.handle('launcher:graphics-status', async event => { trusted(event); await graphics.refreshGPUInfo(); return graphics.status(); });
    ipcMain.handle('launcher:graphics-report', (event, report) => { trusted(event); return graphics.report(report); });
    ipcMain.handle('launcher:graphics-mode', (event, mode, options) => { trusted(event); return selectGraphicsMode(mode, options); });
    ipcMain.handle('launcher:graphics-export', async event => {
        trusted(event); await graphics.refreshGPUInfo();
        const result = await dialog.showSaveDialog(win, { defaultPath: 'serialarm-graphics-diagnostics.json', filters: [{ name: 'Graphics diagnostics', extensions: ['json'] }] });
        if (result.canceled || !result.filePath) return { canceled: true };
        fs.writeFileSync(result.filePath, JSON.stringify(graphics.status(), null, 2) + '\n', { mode: 0o600 });
        return { canceled: false, path: result.filePath };
    });
    ipcMain.handle('launcher:prefs', event => { trusted(event); const prefs = readPrefs(); return { ...prefs, fontLigatures: prefs.fontLigatures !== false, language: prefs.language || (app.getLocale().startsWith('zh') ? 'zh-CN' : 'en') }; });
    ipcMain.handle('launcher:save-prefs', (event, values) => { trusted(event); return savePrefs(values); });
    ipcMain.handle('launcher:select-profile', async event => { trusted(event); const result = await dialog.showOpenDialog(win, { properties: ['openFile'], filters: [{ name: 'Robot Profiles', extensions: ['yaml', 'yml'] }] }); return result.canceled ? '' : result.filePaths[0]; });
    ipcMain.handle('launcher:select-gain-candidate', async event => { trusted(event); const result = await dialog.showOpenDialog(win, { properties: ['openFile'], filters: [{ name: 'Gain candidate', extensions: ['json'] }] }); return result.canceled ? '' : result.filePaths[0]; });
    ipcMain.handle('launcher:reveal-file', (event, file) => {
        trusted(event);
        if (typeof file !== 'string' || file.length > 8192 || !path.isAbsolute(file) || /[\0\n]/.test(file)) throw new Error('Invalid file path');
        const resolved = fs.realpathSync(file);
        if (!fs.statSync(resolved).isFile()) throw new Error('Source file is unavailable');
        shell.showItemInFolder(resolved);
        return true;
    });
    ipcMain.handle('launcher:select-trajectory', async event => { trusted(event); const result = await dialog.showOpenDialog(win, { properties: ['openFile'], filters: [{ name: 'SerialArm trajectory', extensions: ['zip'] }] }); return result.canceled ? '' : result.filePaths[0]; });
    ipcMain.handle('launcher:select-description', async event => { trusted(event); const result = await dialog.showOpenDialog(win, { properties: ['openFile'], filters: [{ name: 'SerialArm Description', extensions: ['zip', 'urdf', 'xacro'] }] }); return result.canceled ? '' : result.filePaths[0]; });
    ipcMain.handle('launcher:select-profile-package', async event => { trusted(event); const result = await dialog.showOpenDialog(win, { properties: ['openDirectory'] }); return result.canceled ? '' : result.filePaths[0]; });
    ipcMain.handle('launcher:select-destination', async event => { trusted(event); const result = await dialog.showOpenDialog(win, { properties: ['openDirectory', 'createDirectory'] }); return result.canceled ? '' : result.filePaths[0]; });
    ipcMain.handle('launcher:select-resources', async event => { trusted(event); const result = await dialog.showOpenDialog(win, { properties: ['openDirectory'] }); return result.canceled ? '' : result.filePaths[0]; });
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
        return { path: resolved, extension, data: fs.readFileSync(resolved).toString('base64') };
    });
    ipcMain.handle('launcher:terminal-menu', (event, options = {}) => {
        trusted(event);
        if (!options || typeof options !== 'object' || Array.isArray(options)) throw new Error('Invalid terminal menu request');
        const en = options.language === 'en';
        return new Promise(resolve => {
            let action = null;
            const item = (id, label, enabled = true) => ({ id, label, enabled, click: () => { action = id; } });
            const menu = Menu.buildFromTemplate([
                item('copy', en ? 'Copy selection' : '复制所选', options.selection === true),
                item('copy-all', en ? 'Copy all output' : '复制全部输出'),
                item('paste', en ? 'Paste' : '粘贴', options.canPaste === true),
                { type: 'separator' },
                item('select-all', en ? 'Select all' : '全选'),
            ]);
            menu.popup({ window: win, callback: () => resolve(action) });
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
    if (app.commandLine.hasSwitch('smoke-test')) {
        if (graphics.effectiveMode === 'rescue') throw new Error('Rescue mode does not provide 3D rendering; the 3D smoke check cannot pass');
        if (await win.webContents.executeJavaScript('document.hidden')) win.showInactive();
        const result = await win.webContents.executeJavaScript(`(async () => {
      const catalogue = await window.serialArm.request('profiles', {});
      if (!catalogue.profiles.length) throw new Error('No packaged profiles');
      const config = {profile: catalogue.profiles[0].name, profile_file: catalogue.profile_file};
      const model = await window.serialArm.request('model', {config});
      if (!model.native?.ok || !model.model?.links?.length) throw new Error('Packaged model failed');
      const runtime = await window.serialArm.graphics.status();
      const {SerialArmModelView} = await import('./model-view.js');
      const host = document.createElement('div');
      Object.assign(host.style, {position: 'fixed', inset: '0', width: '640px', height: '480px', zIndex: '99999'});
      document.body.append(host);
      let view;
      try {
        view = new SerialArmModelView(host, file => window.serialArm.modelResource(file), 'en', {
          mode: runtime.effectiveMode, onStatus: report => window.serialArm.graphics.report(report).catch(() => {})
        });
        if (!await view.setModel(model)) throw new Error('Model view did not finish loading');
        // A real model mesh must be drawn, not just the grid or backend data.
        for (let i = 0; i < 50; i++) {
          view.renderOnce();
          const state = view.getRenderState();
          if (state.status === 'ready' && state.modelLoaded && state.meshCount > 0 && state.drawCount > 0 && state.renderCalls > 0 && state.triangles > 0) {
            await window.serialArm.graphics.report(state);
            return {profile: config.profile, links: model.model.links.length, graphics: state};
          }
          if (['failed', 'lost'].includes(state.status)) throw new Error(state.error?.message || 'Model rendering failed');
          await new Promise(resolve => setTimeout(resolve, 100));
        }
        throw new Error('No model meshes were drawn by the WebGL2 renderer');
      } finally {
        if (view) {
          view.dispose();
          if (!view.disposed || view.animationFrame != null || view.frameTimer != null || view.canvas.isConnected) throw new Error('Model view resources were not released');
        }
        host.remove();
      }
    })()`);
        process.stdout.write('SERIAL_ARM_SMOKE_PASS ' + JSON.stringify(result) + '\n');
        await bridge.close();
        win.destroy();
        app.quit();
    }
}
app.on('before-quit', event => {
    if (!win || win.isDestroyed()) return;
    event.preventDefault();
    closeWindow().catch(error => dialog.showErrorBox('SerialArm Launcher', error.message));
});
app.on('window-all-closed', () => app.quit());
