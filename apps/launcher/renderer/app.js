'use strict';
const api = window.serialArm;
const words = {
    ros2: ['ROS2', 'ROS2'],
    workspace: ['机械臂工作台', 'Manipulator workspace'], subtitle: ['串联机械臂配置与运行', 'Serial manipulator workspace'], subline: ['控制 · 动力学 · 交互', 'Control · Dynamics · Interaction'],
    navigation: ['工作流', 'WORKFLOW'], application: ['应用', 'APPLICATION'], robot: ['机器人与连接', 'Robot & connection'], run: ['运行', 'Run'], diagnostics: ['诊断', 'Diagnostics'], settings: ['设置', 'Settings'],
    ready: ['就绪', 'Ready'], unavailable: ['不可用', 'Unavailable'], connected: ['安装已就绪', 'Installation ready'], notInstalled: ['尚未安装', 'Not installed'], idle: ['空闲', 'Idle'], running: ['运行中', 'Running'], stopping: ['正在停止', 'Stopping'], exited: ['已退出', 'Exited'],
    robotTitle: ['选择机器人，配置连接', 'Choose a robot; Configure its connection'], robotDesc: ['使用内置或外部 Profile，将设备设置作为本次运行的覆盖值', 'Use a built-in or external profile and set connection overrides for this session'],
    profile: ['Robot Profile', 'Robot Profile'], profileSub: ['配置的唯一入口', 'A single configuration entry'], source: ['配置来源', 'Profile source'], builtin: ['内置 Profile', 'Built-in profiles'], external: ['外部 Profile 文件', 'External profile file'], profileFile: ['Profile 文件路径', 'Profile file path'], browse: ['选择文件', 'Browse'], refresh: ['刷新', 'Refresh'], profileName: ['机器人', 'Robot'],
    connection: ['连接设置', 'Connection'], connectionSub: ['不修改原始 YAML 配置', 'Original YAML files stay unchanged'], serial: ['串口设备', 'Serial device'], automatic: ['使用 Profile 默认值', 'Use profile default'], baudrate: ['波特率', 'Baudrate'], bus: ['Bus 名称', 'Bus name'], resourcePaths: ['资源搜索路径', 'Resource search paths'], addRoot: ['添加目录', 'Add directory'], rootsHint: ['多个目录用冒号分隔；支持下游工作区或安装前缀', 'Separate roots with a colon; downstream workspaces and install prefixes are supported'], overrideHint: ['留空使用 Profile 默认值', 'Leave blank to use the profile default'], inspect: ['检查配置', 'Inspect configuration'], toRun: ['选择运行模式', 'Choose run mode'],
    summary: ['配置概览', 'Profile overview'], summarySub: ['实际资源与执行器写入状态', 'Resolved resources and actuator write state'], hardwarePlugin: ['硬件后端', 'Hardware backend'], writeEnabled: ['执行器写入', 'Actuator writes'], enabled: ['已开启', 'Enabled'], disabled: ['已关闭', 'Disabled'], unknown: ['未知', 'Unknown'], controllers: ['Controllers', 'Controllers'], noProfile: ['请选择一个 Profile', 'Select a profile'], status: ['环境状态', 'Environment status'], statusSub: ['只读检查，不连接执行器', 'Read-only checks, no actuator connection'], resources: ['Profile 资源', 'Profile resources'], installation: ['安装状态', 'Installation'], nativeTerminal: ['C++ Terminal', 'C++ Terminal'],
    runtimeTitle: ['选择运行方式', 'Choose how to run'], runtimeDesc: ['沿用现有 Core 与 ROS 入口；日志和 Terminal 输入集中在这里', 'Use the existing Core and ROS entry points, with logs and Terminal input in one place'],
    model: ['Model', 'Model'], modelDesc: ['查看模型、关节状态与 RViz', 'View the model, joint states and RViz'], terminal: ['Terminal', 'Terminal'], terminalDesc: ['进入 C++ 交互终端，调试机械臂', 'Open the C++ interactive terminal'], hardware: ['Hardware', 'Hardware'], hardwareDesc: ['运行 ros2_control 和 Controllers', 'Run ros2_control and controllers'], moveit: ['MoveIt', 'MoveIt'], moveitDesc: ['启动规划、硬件和 RViz', 'Start planning, hardware and RViz'], offline: ['OFFLINE', 'OFFLINE'], real: ['REAL HARDWARE', 'REAL HARDWARE'],
    launch: ['启动', 'Launch'], stop: ['停止', 'Stop'], force: ['强制结束', 'Force stop'], clear: ['清空显示', 'Clear view'], copyAll: ['复制全部', 'Copy all'], paste: ['粘贴', 'Paste'], logs: ['运行终端', 'Runtime terminal'], terminalHint: ['拖选后 Ctrl+Shift+C 复制，Ctrl+Shift+V 粘贴，右键打开菜单；Ctrl+C 用于中断', 'Select text and press Ctrl+Shift+C to copy, Ctrl+Shift+V to paste, or right-click for the menu; Ctrl+C still interrupts'], noSession: ['等待启动会话', 'Waiting for a session'], modeMissing: ['此模式缺少安装组件或 Profile 资源，请查看诊断', 'This mode needs installed components or profile resources; Check diagnostics'], selectedRobot: ['当前机器人', 'Selected robot'],
    diagTitle: ['检查环境与资源', 'Inspect environment and resources'], diagDesc: ['核对安装状态、Profile 路径和设备权限，定位启动条件', 'Check installation state, profile paths and device permissions before launching'], checkNow: ['重新检查', 'Check again'], checks: ['检查结果', 'Check results'], checkSub: ['不实例化 Hardware，也不打开设备', 'Does not instantiate hardware or open devices'], pass: ['通过', 'Pass'], fail: ['未通过', 'Failed'], installHelp: ['安装与环境', 'Installation & environment'], installHelpSub: ['从仓库根目录执行', 'Run from the repository root'], installHint: ['安装器会按用途配置组件，GUI 自动加载安装后的环境', 'The installer configures selected components; the GUI loads their environment'], validationHint: ['资源检查不代表真机验收；Core 在运行入口中执行配置与硬件能力校验', 'Resource checks are not a hardware acceptance test; Core validates configuration and hardware capabilities at runtime'],
    settingsTitle: ['让工作台适合你', 'Make the workspace yours'], settingsDesc: ['偏好会自动保存，不影响正在运行的会话', 'Preferences are saved automatically without restarting an active session'], appearance: ['外观', 'Appearance'], appearanceSub: ['选择工作台主题', 'Choose a workspace theme'], system: ['跟随系统', 'System'], dark: ['深色', 'Dark'], light: ['浅色', 'Light'], language: ['语言', 'Language'], languageSub: ['随时切换界面语言', 'Switch the interface language at any time'], prefsNote: ['上次使用的 Profile、连接覆盖值、运行模式和窗口尺寸会自动保存', 'Your last profile, connection overrides, run mode and window size are saved'],
    realTitle: ['启动真机会话', 'Start a hardware session'], realDesc: ['该入口可能连接并驱动真实执行器，请核对下面的运行配置', 'This entry point can connect to and drive real actuators; Review the configuration below'], confirm: ['我已确认机器人工作区可运行，并了解当前执行器写入状态', 'I have checked the robot workspace and understand the actuator write state'], cancel: ['取消', 'Cancel'], confirmLaunch: ['确认并启动', 'Confirm and launch'], forceTitle: ['强制结束运行进程', 'Force stop the session'], forceDesc: ['强制结束会中断运行进程的清理流程；请先确认机器人状态', 'Force stop interrupts process cleanup; Check the robot state first'], error: ['操作失败', 'Operation failed'], fileHint: ['选择 robot_profiles.yaml', 'Choose robot_profiles.yaml'], saved: ['已保存', 'Saved'], core: ['Core YAML', 'Core YAML'], description: ['模型文件', 'Model file'], ros2_control: ['ros2_control Xacro', 'ros2_control Xacro'], device: ['设备权限', 'Device access'], backendMissing: ['运行后端不可用', 'Backend unavailable'], backend: ['运行后端', 'Runtime backend'],
};
const paths = { robot: '<path d="M5 21v-4h14v4M8 17l-3-5 4-3 4 3-2 5M9 9l3-5 6 3-5 5M18 7l2-3M2 21h20"/>', run: '<path d="m8 5 11 7-11 7Z"/>', diagnostics: '<path d="M10 3h4v4h-4zM5 8h14v12H5zM9 12h6M9 16h3"/>', settings: '<path d="M4 7h16M4 17h16M8 4v6M16 14v6"/>', plug: '<path d="M8 3v4M16 3v4M6 7h12v4a6 6 0 0 1-12 0zM12 17v4"/>', folder: '<path d="M3 6h7l2 2h9v12H3z"/>', model: '<path d="m12 3 9 5v9l-9 5-9-5V8zM3 8l9 5 9-5M12 13v9"/>', terminal: '<path d="m5 7 5 5-5 5M13 17h6"/>', hardware: '<path d="M7 7h10v10H7zM4 9h3M4 15h3M17 9h3M17 15h3M9 4v3M15 4v3M9 17v3M15 17v3"/>', moveit: '<path d="M4 19V5h16M4 19h16M8 15l4-7 6 5M7 15h2M17 13h2"/>', refresh: '<path d="M20 7v5h-5M4 17v-5h5M5 8a8 8 0 0 1 13-3l2 2M4 17l2 2a8 8 0 0 0 13-3"/>', check: '<path d="m5 12 4 4L19 6"/>', close: '<path d="m6 6 12 12M18 6 6 18"/>', minus: '<path d="M5 12h14"/>', max: '<rect x="5" y="5" width="14" height="14" rx="1"/>', sun: '<circle cx="12" cy="12" r="4"/><path d="M12 2v2M12 20v2M2 12h2M20 12h2M5 5l1 1M18 18l1 1M5 19l1-1M18 6l1-1"/>', stop: '<rect x="6" y="6" width="12" height="12" rx="1"/>' };
const icon = name => `<svg viewBox="0 0 24 24" aria-hidden="true">${paths[name] || paths.robot}</svg>`;
const esc = value => String(value ?? '').replace(/[&<>"']/g, c => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;', "'": '&#39;' }[c]));
const $ = id => document.getElementById(id);
let state = { page: 'robot', language: 'zh-CN', theme: 'system', mode: 'model', source: 'builtin', config: { profile: '', profile_file: '', serial_port: '', baudrate: '', bus: '', resource_paths: '' }, info: null, status: {}, profiles: [], session: { state: 'idle' }, busy: false, backendError: '', inspecting: 0 };
let term, fit, outputBuffer = '';
const t = key => (words[key] || [key, key])[state.language === 'en' ? 1 : 0];
function toast(message, error = false) { const node = document.createElement('div'); node.className = 'toast' + (error ? ' error' : ''); node.textContent = message; $('toasts').append(node); setTimeout(() => node.remove(), 8000); }
const button = (id, key, ico, cls = '') => `<button id="${id}" class="button ${cls}">${ico ? icon(ico) : ''}${t(key)}</button>`;
const card = (title, sub, ico, body) => `<section class="card"><div class="card-head">${icon(ico)}<div><h3>${t(title)}</h3><p>${t(sub)}</p></div></div><div class="card-body">${body}</div></section>`;
function pageHead(title, desc, action = '') { return `<header class="page-head"><div><div class="eyebrow">SERIALARM · WORKSPACE</div><h2>${t(title)}</h2><p>${t(desc)}</p></div>${action}</header>`; }
function configField(key, label, placeholder = '') { return `<div class="field"><label for="${key}">${t(label)}</label><input id="${key}" value="${esc(state.config[key])}" placeholder="${esc(placeholder)}" autocomplete="off"></div>`; }
function active() { return ['running', 'stopping'].includes(state.session.state); }
async function save() { if (api) await api.savePrefs({ ...state.config, mode: state.mode, theme: state.theme, language: state.language }); }
function applyTheme() { const light = state.theme === 'light' || (state.theme === 'system' && matchMedia('(prefers-color-scheme: light)').matches); document.documentElement.className = light ? 'light' : 'dark'; }
function shell() {
    $('app').innerHTML = `<div class="shell"><header class="titlebar"><div class="brand-mini">${icon('robot')}SerialArm Launcher</div><div class="title-center">${t('workspace')}</div><div class="title-actions"><button id="theme-quick" class="title-button" aria-label="${t('appearance')}">${icon('sun')}</button><button id="win-min" class="title-button" aria-label="Minimize">${icon('minus')}</button><button id="win-max" class="title-button" aria-label="Maximize">${icon('max')}</button><button id="win-close" class="title-button close" aria-label="Close">${icon('close')}</button></div></header><div class="body"><aside class="sidebar"><div class="hero"><div class="eyebrow">AGROTECH · SCAU</div><h1>SerialArm</h1><p>${t('subtitle')}<br>${t('subline')}</p></div><div class="nav-caption">${t('navigation')}</div><nav>${['robot', 'run', 'diagnostics'].map((name, i) => `<button class="nav-item ${state.page === name ? 'active' : ''}" data-page="${name}">${icon(name)}<span>${t(name)}</span><small>0${i + 1}</small></button>`).join('')}</nav><div class="nav-caption">${t('application')}</div><nav><button class="nav-item ${state.page === 'settings' ? 'active' : ''}" data-page="settings">${icon('settings')}<span>${t('settings')}</span></button></nav><div class="sidebar-spacer"></div><div class="runtime-card" id="sidebar-runtime"></div></aside><section class="content-wrap"><div class="top-status"><span class="breadcrumb" id="breadcrumb"></span><span class="pill" id="env-pill"></span><span class="pill" id="session-pill"></span></div><main class="content"><div class="page" id="page"></div><section id="runtime-panel" class="card terminal-card is-hidden"><div class="terminal-head">${icon('terminal')}<b>${t('logs')}</b><span class="terminal-state" id="terminal-state"></span>${button('copy-log', 'copyAll', '')}${button('paste-log', 'paste', '')}${button('clear-log', 'clear', '')}${button('stop-runtime', 'stop', 'stop')}${button('force-runtime', 'force', '', 'danger')}</div><div id="terminal"></div><div class="terminal-note">${t('terminalHint')}</div></section></main></section></div></div><div id="toasts" aria-live="polite"></div><dialog id="confirm-dialog"></dialog>`;
    document.querySelectorAll('[data-page]').forEach(b => b.onclick = () => navigate(b.dataset.page));
    ['min', 'max', 'close'].forEach((n, i) => $('win-' + n).onclick = () => api.window(['minimize', 'maximize', 'close'][i]));
    $('theme-quick').onclick = async () => { state.theme = document.documentElement.classList.contains('light') ? 'dark' : 'light'; applyTheme(); await save(); };
    $('copy-log').onclick = () => operation(() => copyTerminal(true));
    $('paste-log').onclick = () => operation(pasteTerminal);
    $('clear-log').onclick = () => { outputBuffer = ''; term?.clear(); };
    $('stop-runtime').onclick = () => operation(() => api.request('stop'));
    $('force-runtime').onclick = () => forceConfirm();
    mountTerminal(); render();
}
function mountTerminal() {
    if (typeof Terminal !== 'undefined') {
        term?.dispose(); term = new Terminal({ fontFamily: '"JetBrains Mono", "Noto Sans Mono CJK SC", monospace', fontSize: 12, scrollback: 5000, convertEol: true, theme: { background: '#0c0c12', foreground: '#c9c9d0', cursor: '#f59e0b' }, allowProposedApi: false });
        fit = new FitAddon.FitAddon(); term.loadAddon(fit); term.open($('terminal')); if (outputBuffer) term.write(outputBuffer);
        term.attachCustomKeyEventHandler(terminalKey);
        $('terminal').oncontextmenu = event => { event.preventDefault(); operation(terminalMenu); };
        term.onData(data => { if (active() && state.session.mode === 'terminal') api.request('input', { data }).catch(error => toast(error.message, true)); });
        term.onResize(({ cols, rows }) => { if (active()) api.request('resize', { cols, rows }).catch(() => { }); });
    }
}
function terminalInputReady() { return active() && state.session.mode === 'terminal'; }
function terminalText() {
    if (!term) return '';
    const buffer = term.buffer.active; let text = '';
    for (let row = 0; row < buffer.length; row++) {
        const line = buffer.getLine(row); if (!line) continue;
        if (row && !line.isWrapped) text += '\n';
        text += line.translateToString(true);
    }
    return text.replace(/\n+$/, '').slice(-1024 * 1024);
}
async function copyTerminal(all = false) {
    const text = all ? terminalText() : term?.getSelection();
    if (text) await api.clipboard.write(text.slice(-1024 * 1024));
}
async function pasteTerminal() {
    if (!terminalInputReady()) return;
    const text = await api.clipboard.read();
    if (text.length > 65524) throw new Error('Clipboard text exceeds the terminal input limit');
    if (terminalInputReady() && text) { term.paste(text); term.focus(); }
}
function terminalKey(event) {
    const key = event.key.toLowerCase();
    const shortcut = (event.ctrlKey && event.shiftKey && !event.altKey && !event.metaKey) || (event.metaKey && !event.ctrlKey && !event.altKey);
    if (!shortcut || !['c', 'v', 'a'].includes(key)) return true;
    if (event.type === 'keydown') {
        event.preventDefault();
        if (key === 'c') operation(() => copyTerminal());
        else if (key === 'v') operation(pasteTerminal);
        else term.selectAll();
    }
    return false;
}
async function terminalMenu() {
    const action = await api.terminalMenu({ selection: !!term?.hasSelection(), canPaste: terminalInputReady(), language: state.language });
    if (action === 'copy') await copyTerminal();
    else if (action === 'copy-all') await copyTerminal(true);
    else if (action === 'paste') await pasteTerminal();
    else if (action === 'select-all') { term.selectAll(); term.focus(); }
}
function statusUI() {
    const ready = state.status.installed; $('sidebar-runtime').innerHTML = `<span class="dot ${ready ? 'ready' : ''}"></span>${t(ready ? 'connected' : 'notInstalled')}<small>${esc(state.info?.profile || state.config.profile || '—')}</small><small>${state.backendError ? esc(state.backendError) : 'ROS2 ' + esc(state.status.ros_distro || '—')}</small>`;
    $('env-pill').innerHTML = `<span class="dot ${ready ? 'ready' : ''}"></span>Core ${t(ready ? 'ready' : 'unavailable')}`;
    $('session-pill').innerHTML = `<span class="dot ${active() ? 'ready' : ''}"></span>${t(state.session.state || 'idle')}`;
    $('breadcrumb').textContent = t('workspace') + ' / ' + t(state.page);
    $('terminal-state').textContent = active() ? `${t(state.session.state)} · PID ${state.session.pid}` : state.session.state === 'exited' ? `${t('exited')} · ${state.session.exit_code}` : t('noSession');
    $('paste-log').disabled = !terminalInputReady();
    $('stop-runtime').disabled = !active() || state.session.state === 'stopping'; $('force-runtime').classList.toggle('is-hidden', state.session.state !== 'stopping');
}
function robotPage() {
    const info = state.info;
    const selectOptions = `<option value="">${t('noProfile')}</option>` + state.profiles.map(p => `<option value="${esc(p.name)}" ${p.name === state.config.profile ? 'selected' : ''}>${esc(p.name)}</option>`).join('');
    const profileBody = `<div class="field"><label>${t('source')}</label><div class="segmented"><button id="source-builtin" class="${state.source === 'builtin' ? 'active' : ''}">${t('builtin')}</button><button id="source-external" class="${state.source === 'external' ? 'active' : ''}">${t('external')}</button></div></div>${state.source === 'external' ? `<div class="field"><label for="profile_file">${t('profileFile')}</label><div class="input-row"><input id="profile_file" value="${esc(state.config.profile_file)}" placeholder="${t('fileHint')}">${button('browse-profile', 'browse', 'folder')}</div></div>` : ''}<div class="field"><label for="profile-select">${t('profileName')}</label><div class="input-row"><select id="profile-select">${selectOptions}</select>${button('refresh-profiles', 'refresh', 'refresh')}</div></div>`;
    const ports = (state.status.devices || []).map(p => `<option value="${esc(p)}">`).join('');
    const connBody = `${configField('serial_port', 'serial', info?.default_port || t('automatic'))}<datalist id="ports">${ports}</datalist><div class="row">${configField('baudrate', 'baudrate', String(info?.default_baudrate || '921600'))}${configField('bus', 'bus', info?.bus || t('automatic'))}</div><div class="field"><label for="resource_paths">${t('resourcePaths')}</label><div class="input-row"><input id="resource_paths" value="${esc(state.config.resource_paths)}" placeholder="/path/to/workspace">${button('add-root', 'addRoot', 'folder')}</div><div class="hint">${t('rootsHint')}</div></div><div class="actions">${button('inspect-profile', 'inspect', 'diagnostics')}${button('go-run', 'toRun', 'run', 'primary')}</div><div class="hint">${t('overrideHint')}</div>`;
    const summary = `<div class="summary-caption">ROBOT PROFILE</div><div class="summary-name">${esc(info?.profile || state.config.profile || '—')}</div><dl class="kv"><dt>${t('hardwarePlugin')}</dt><dd>${esc(info?.hardware_plugin || '—')}</dd><dt>${t('serial')}</dt><dd>${esc(state.config.serial_port || info?.default_port || '—')}</dd><dt>${t('writeEnabled')}</dt><dd style="color:var(--accent)">${t(info?.write_enabled === true ? 'enabled' : info?.write_enabled === false ? 'disabled' : 'unknown')}</dd><dt>${t('controllers')}</dt><dd>${info?.controllers?.map(name => `<div>${esc(name)}</div>`).join('') || '—'}</dd></dl><div class="callout">${t('validationHint')}</div>`;
    const checks = `<ul class="check-list">${[['installation', !!state.status.installed], ['resources', !!info && info.checks.filter(c => c.name !== 'device').every(c => c.ok)], ['nativeTerminal', !!info?.available?.terminal], ['ros2', !!state.status.ros2]].map(([key, ok]) => `<li><span class="dot ${ok ? 'ready' : ''}"></span>${t(key)}<span class="state">${t(ok ? 'ready' : 'unavailable')}</span></li>`).join('')}</ul>`;
    return pageHead('robotTitle', 'robotDesc') + `<div class="grid"><div class="card-stack">${card('profile', 'profileSub', 'robot', profileBody)}${card('connection', 'connectionSub', 'plug', connBody)}</div><div class="card-stack">${card('summary', 'summarySub', 'model', summary)}${card('status', 'statusSub', 'diagnostics', checks)}</div></div>`;
}
function runPage() {
    return pageHead('runtimeTitle', 'runtimeDesc') + `<div class="mode-grid">${['model', 'terminal', 'hardware', 'moveit'].map(mode => `<button class="mode-card ${state.mode === mode ? 'active' : ''}" data-mode="${mode}"><span class="mode-type">${t(mode === 'model' ? 'offline' : 'real')}</span>${icon(mode)}<h3>${t(mode)}</h3><p>${t(mode + 'Desc')}</p></button>`).join('')}</div><div class="card launch-panel"><div><h3>${t('selectedRobot')} · ${esc(state.config.profile || '—')}</h3><p>${state.info?.available?.[state.mode] ? t(state.mode === 'model' ? 'offline' : 'real') : t('modeMissing')}</p><div class="launch-command">${esc(state.info?.profile_file || state.config.profile_file || '')}</div></div>${button('launch-session', 'launch', 'run', 'primary')}</div>`;
}
function diagnosticsPage() {
    const checks = [{ name: 'installation', ok: !!state.status.installed, detail: state.status.prefix || t('notInstalled') }, ...(state.info?.checks || [])];
    return pageHead('diagTitle', 'diagDesc', button('check-now', 'checkNow', 'refresh')) + `<div class="grid">${card('checks', 'checkSub', 'diagnostics', checks.map(c => `<div class="diag-row"><span class="dot ${c.ok ? 'ready' : 'error'}"></span><span class="name">${t(c.name)}</span><span class="detail">${esc(c.detail)}</span><span class="state ${c.ok ? '' : 'error'}">${t(c.ok ? 'pass' : 'fail')}</span></div>`).join(''))}${card('installHelp', 'installHelpSub', 'terminal', `<p class="hint">${t('installHint')}</p><pre class="install-command"><code>./install.sh</code></pre><pre class="install-command"><code>./install.sh --preset ros2 --yes</code></pre><div class="callout">${t('validationHint')}</div>`)}</div>`;
}
function settingsPage() {
    return pageHead('settingsTitle', 'settingsDesc') + `<div class="grid">${card('appearance', 'appearanceSub', 'sun', `<div class="segmented">${['system', 'dark', 'light'].map(theme => `<button data-theme="${theme}" class="${theme === state.theme ? 'active' : ''}">${t(theme)}</button>`).join('')}</div>`)}${card('language', 'languageSub', 'settings', `<div class="segmented"><button data-language="zh-CN" class="${state.language === 'zh-CN' ? 'active' : ''}">简体中文</button><button data-language="en" class="${state.language === 'en' ? 'active' : ''}">English</button></div>`)}</div><div class="callout">${t('prefsNote')}</div>`;
}
function render() {
    $('page').innerHTML = ({ robot: robotPage, run: runPage, diagnostics: diagnosticsPage, settings: settingsPage }[state.page])();
    $('runtime-panel').classList.toggle('is-hidden', state.page !== 'run' && !active());
    document.querySelectorAll('[data-page]').forEach(b => b.classList.toggle('active', b.dataset.page === state.page));
    statusUI(); bindPage(); if (state.page === 'run') setTimeout(() => { try { fit?.fit(); } catch { } }, 50);
}
function capture() { for (const key of Object.keys(state.config)) { const element = $(key); if (element) state.config[key] = element.value; } if ($('profile-select')) state.config.profile = $('profile-select').value; }
async function operation(fn) { try { return await fn(); } catch (error) { toast(t('error') + ': ' + error.message, true); return null; } }
async function refreshProfiles() {
    capture(); if (state.source === 'external' && !state.config.profile_file) throw new Error(t('fileHint')); const data = await api.request('profiles', state.config); state.profiles = data.profiles;
    if (!state.profiles.some(p => p.name === state.config.profile)) state.config.profile = state.profiles[0]?.name || '';
    state.info = null; render(); await save(); await inspect(); render();
}
async function inspect() {
    capture(); if (state.source === 'external' && !state.config.profile_file) throw new Error(t('fileHint')); const serial = ++state.inspecting; const snapshot = { ...state.config };
    const status = await api.request('status'); const info = await api.request('inspect', snapshot);
    if (serial !== state.inspecting) return; state.status = status; state.info = info; await save(); statusUI();
}
async function navigate(page) { capture(); await operation(save); state.page = page; if (page === 'run' || page === 'diagnostics') await operation(inspect); render(); }
function bindPage() {
    for (const key of Object.keys(state.config)) {
        const element = $(key); if (element) element.onchange = () => { capture(); state.info = null; operation(save); };
    }
    if ($('serial_port')) $('serial_port').setAttribute('list', 'ports');
    if ($('source-builtin')) {
        $('source-builtin').onclick = () => operation(async () => { capture(); state.source = 'builtin'; state.config.profile_file = ''; await refreshProfiles(); });
        $('source-external').onclick = () => { capture(); state.source = 'external'; state.info = null; state.profiles = []; state.config.profile = ''; render(); };
        $('browse-profile')?.addEventListener('click', () => operation(async () => { capture(); const file = await api.selectProfile(); if (file) { state.config.profile_file = file; if ($('profile_file')) $('profile_file').value = file; await refreshProfiles(); } }));
        $('profile-select').onchange = () => operation(async () => { capture(); await inspect(); render(); });
        if ($('profile_file')) $('profile_file').onchange = () => operation(refreshProfiles);
        $('refresh-profiles').onclick = () => operation(refreshProfiles);
        $('inspect-profile').onclick = () => operation(async () => { await inspect(); render(); });
        $('go-run').onclick = () => navigate('run');
        $('add-root').onclick = () => operation(async () => { capture(); const dir = await api.selectResources(); if (dir) { state.config.resource_paths = [state.config.resource_paths, dir].filter(Boolean).join(':'); await inspect(); render(); } });
    }
    document.querySelectorAll('[data-mode]').forEach(b => { b.disabled = active(); b.onclick = () => { if (active()) return; state.mode = b.dataset.mode; operation(save); render(); }; });
    if (state.page === 'robot' && active()) document.querySelectorAll('#page input,#page select,#source-builtin,#source-external,#browse-profile,#refresh-profiles,#add-root').forEach(e => e.disabled = true);
    if ($('launch-session')) { $('launch-session').disabled = state.busy || active() || !state.info?.available?.[state.mode]; $('launch-session').onclick = () => operation(async () => { await inspect(); if (!state.info?.available?.[state.mode]) throw new Error(t('modeMissing')); if (state.mode === 'model') await launch(false); else hardwareConfirm(); }); }
    if ($('check-now')) $('check-now').onclick = () => operation(async () => { await inspect(); render(); });
    document.querySelectorAll('[data-theme]').forEach(b => b.onclick = () => { state.theme = b.dataset.theme; applyTheme(); operation(save); render(); });
    document.querySelectorAll('[data-language]').forEach(b => b.onclick = async () => { state.language = b.dataset.language; document.documentElement.lang = state.language; await operation(save); shell(); });
}
async function launch(confirmed) { state.busy = true; render(); try { state.session = await api.request('start', { mode: state.mode, config: state.config, confirmed, fingerprint: state.info?.fingerprint }); } finally { state.busy = false; render(); setTimeout(() => fit?.fit(), 50); } }
function hardwareConfirm() {
    const info = state.info; const dialog = $('confirm-dialog');
    dialog.innerHTML = `<div class="eyebrow">REAL HARDWARE</div><h3 style="margin-top:10px">${t('realTitle')}</h3><p>${t('realDesc')}</p><dl class="kv"><dt>${t('profileName')}</dt><dd>${esc(info.profile)}</dd><dt>${t('serial')}</dt><dd>${esc(info.devices.join(', ') || '—')}</dd><dt>${t('writeEnabled')}</dt><dd style="color:var(--accent)">${t(info.write_enabled === true ? 'enabled' : info.write_enabled === false ? 'disabled' : 'unknown')}</dd><dt>${t('controllers')}</dt><dd>${esc(info.controllers.join(', '))}</dd></dl><label class="confirmation"><input id="hardware-confirmed" type="checkbox"><span>${t('confirm')}</span></label><div class="actions">${button('cancel-launch', 'cancel', '')}${button('confirm-launch', 'confirmLaunch', 'run', 'primary')}</div>`;
    dialog.showModal(); $('confirm-launch').disabled = true; $('hardware-confirmed').onchange = () => $('confirm-launch').disabled = !$('hardware-confirmed').checked;
    $('cancel-launch').onclick = () => dialog.close(); $('confirm-launch').onclick = () => { dialog.close(); operation(() => launch(true)); };
}
function forceConfirm() { const dialog = $('confirm-dialog'); dialog.innerHTML = `<h3>${t('forceTitle')}</h3><p>${t('forceDesc')}</p><div class="actions">${button('cancel-force', 'cancel', '')}${button('confirm-force', 'force', '', 'danger')}</div>`; dialog.showModal(); $('cancel-force').onclick = () => dialog.close(); $('confirm-force').onclick = () => { dialog.close(); operation(() => api.request('stop', { force: true })); }; }
function onEvent(event) { if (event.event === 'output') { outputBuffer = (outputBuffer + event.data).slice(-1024 * 1024); term?.write(event.data); } if (event.event === 'session') { state.session = event; render(); } if (event.event === 'error') { state.backendError = event.error; toast(event.error, true); statusUI(); } }
async function boot() {
    if (!api) { $('app').innerHTML = '<div class="empty-state">请通过 ./launch.sh 启动桌面应用 / Start the desktop application using ./launch.sh</div>'; return; }
    try {
        const prefs = await api.prefs(); state.language = prefs.language || 'zh-CN'; state.theme = prefs.theme || 'system'; state.mode = ['model', 'terminal', 'hardware', 'moveit'].includes(prefs.mode) ? prefs.mode : 'model';
        for (const key of Object.keys(state.config)) if (typeof prefs[key] === 'string') state.config[key] = prefs[key]; state.source = state.config.profile_file ? 'external' : 'builtin';
        applyTheme(); document.documentElement.lang = state.language; shell(); api.onEvent(onEvent);
        state.status = await api.request('status'); state.session = state.status.session || { state: 'idle' }; outputBuffer = await api.request('logs'); if (outputBuffer) term?.write(outputBuffer);
        await refreshProfiles();
    } catch (error) { state.backendError = error.message; if ($('page')) { render(); toast(t('error') + ': ' + error.message, true); } }
}
window.addEventListener('resize', () => { if (state.page === 'run') { try { fit?.fit(); } catch { } } });
matchMedia('(prefers-color-scheme: light)').addEventListener('change', applyTheme);
boot();
