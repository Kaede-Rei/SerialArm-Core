const { app, BrowserWindow, dialog, ipcMain, clipboard, Menu } = require('electron');
const path = require('node:path');
const fs = require('node:fs');
const { pathToFileURL } = require('node:url');
const { Bridge } = require('./bridge.cjs');
const root = path.resolve(__dirname, '../../..');
const entry = path.resolve(__dirname, '../renderer/index.html');
let win, bridge, closing = false;
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
    const next = { ...readPrefs() };
    for (const k of ['profile', 'profile_file', 'serial_port', 'baudrate', 'bus', 'resource_paths', 'mode']) {
        if (typeof values[k] === 'string' && values[k].length < 8192 && !/[\0\n]/.test(values[k])) next[k] = values[k];
    }
    if (['system', 'dark', 'light'].includes(values.theme)) next.theme = values.theme;
    if (['zh-CN', 'en'].includes(values.language)) next.language = values.language;
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
async function closeWindow() {
    if (closing) return;
    closing = true;
    try {
        if (['running', 'stopping'].includes(bridge.session.state)) {
            const choice = await dialog.showMessageBox(win, {
                type: 'question', buttons: ['停止并退出 / Stop and exit', '取消 / Cancel'], defaultId: 1, cancelId: 1,
                message: '运行会话仍在执行 / A session is still running', detail: '关闭窗口前先停止运行进程 / Stop the session before closing'
            });
            if (choice.response !== 0) return;
            await bridge.request('stop');
            await new Promise(resolve => {
                if (bridge.session.state === 'exited') return resolve();
                const done = message => { if (message.event === 'session' && message.state === 'exited') { clearTimeout(timer); bridge.off('event', done); resolve(); } };
                const timer = setTimeout(() => { bridge.off('event', done); resolve(); }, 8000);
                bridge.on('event', done);
            });
            if (bridge.session.state !== 'exited') {
                const force = await dialog.showMessageBox(win, {
                    type: 'warning', buttons: ['强制结束并退出 / Force quit', '取消 / Cancel'], defaultId: 1, cancelId: 1,
                    message: '进程尚未退出 / The session has not exited', detail: '强制结束会中断进程清理；请确认机器人状态 / Force quit interrupts process cleanup; check the robot state'
                });
                if (force.response !== 0) return;
                await bridge.request('stop', { force: true });
            }
        }
        savePrefs({ bounds: win.getBounds() });
        await bridge.close(); win.destroy(); app.quit();
    } finally { closing = false; }
}
async function start() {
    const prefs = readPrefs();
    bridge = new Bridge(process.env.SERIAL_ARM_LAUNCHER_PYTHON || '/usr/bin/python3', path.resolve(__dirname, '../backend/bridge.py'), { cwd: root });
    win = new BrowserWindow({
        width: prefs.bounds?.width || 1300, height: prefs.bounds?.height || 900, minWidth: 1000, minHeight: 720,
        frame: false, backgroundColor: '#0a0a10', show: false,
        webPreferences: { preload: path.join(__dirname, 'preload.cjs'), contextIsolation: true, nodeIntegration: false, sandbox: true }
    });
    bridge.on('event', event => { if (win && !win.isDestroyed()) win.webContents.send('launcher:event', event); });
    win.webContents.setWindowOpenHandler(() => ({ action: 'deny' }));
    win.webContents.on('will-navigate', event => event.preventDefault());
    win.on('close', event => { event.preventDefault(); closeWindow().catch(error => dialog.showErrorBox('SerialArm Launcher', error.message)); });
    win.once('ready-to-show', () => win.show());
    const methods = new Set(['status', 'profiles', 'inspect', 'start', 'input', 'resize', 'stop']);
    ipcMain.handle('launcher:request', (event, method, params = {}) => {
        trusted(event);
        if (method === 'logs') return bridge.output;
        if (!methods.has(method)) throw new Error('Unknown launcher request');
        return bridge.request(method, params);
    });
    ipcMain.handle('launcher:prefs', event => { trusted(event); return { ...readPrefs(), language: readPrefs().language || (app.getLocale().startsWith('zh') ? 'zh-CN' : 'en') }; });
    ipcMain.handle('launcher:save-prefs', (event, values) => { trusted(event); return savePrefs(values); });
    ipcMain.handle('launcher:select-profile', async event => { trusted(event); const result = await dialog.showOpenDialog(win, { properties: ['openFile'], filters: [{ name: 'Robot Profiles', extensions: ['yaml', 'yml'] }] }); return result.canceled ? '' : result.filePaths[0]; });
    ipcMain.handle('launcher:select-resources', async event => { trusted(event); const result = await dialog.showOpenDialog(win, { properties: ['openDirectory'] }); return result.canceled ? '' : result.filePaths[0]; });
    ipcMain.handle('launcher:clipboard', (event, action, text) => {
        trusted(event);
        if (action === 'read') return clipboard.readText();
        if (action !== 'write' || typeof text !== 'string' || text.length > 1024 * 1024) throw new Error('Invalid clipboard request');
        clipboard.writeText(text);
        return true;
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
}
app.on('window-all-closed', () => app.quit());
