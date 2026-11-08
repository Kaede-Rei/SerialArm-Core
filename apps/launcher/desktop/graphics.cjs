const fs = require('node:fs');
const path = require('node:path');

const MODES = new Set(['normal', 'compatibility', 'rescue']);
const SOFTWARE_FILES = ['libEGL.so', 'libGLESv2.so', 'libvulkan.so.1', 'libvk_swiftshader.so', 'vk_swiftshader_icd.json'];

class GraphicsRuntime {
  constructor(app, preferences) {
    this.app = app;
    this.logs = [];
    this.reports = [];
    this.gpuInfo = null;
    this.gpuInfoError = '';
    this.gpuInfoWaitTimedOut = false;
    this.startedAt = new Date().toISOString();
    this.nativeLogPath = app.commandLine.getSwitchValue('log-file') || path.join(app.getPath('userData'), 'graphics-chromium.log');
    if (!app.commandLine.hasSwitch('enable-logging')) {
      fs.mkdirSync(path.dirname(this.nativeLogPath), {recursive: true});
      app.commandLine.appendSwitch('enable-logging', 'file');
      app.commandLine.appendSwitch('log-file', this.nativeLogPath);
    }
    const override = app.commandLine.getSwitchValue('graphics-mode');
    if (override && !MODES.has(override)) throw new Error('Invalid --graphics-mode; use normal, compatibility or rescue');
    this.launchOverride = override || null;
    this.requestedMode = override || (MODES.has(preferences.graphicsMode) ? preferences.graphicsMode : 'normal');
    this.startupMode = this.requestedMode;
    this.resources = SOFTWARE_FILES.map(name => {
      const file = path.join(path.dirname(process.execPath), name);
      try { return {name, path: file, present: fs.statSync(file).isFile()}; }
      catch { return {name, path: file, present: false}; }
    });
    this.compatibilityAvailable = this.resources.every(file => file.present);
    this.blockedReason = '';
    // The explicit command-line selection is also an opt-in; a saved selection
    // requires the consent recorded by the mode dialog in this application
    const consent = override === 'compatibility' || preferences.graphicsUnsafeSoftware === true;
    if (this.startupMode === 'compatibility' && (!consent || !this.compatibilityAvailable)) {
      this.blockedReason = consent ? 'SOFTWARE_RUNTIME_MISSING' : 'SOFTWARE_CONSENT_REQUIRED';
      this.startupMode = 'rescue';
    }
    this.effectiveMode = this.startupMode;
    this.backendMode = this.startupMode === 'compatibility' ? 'compatibility' : 'normal';
    if (this.backendMode === 'compatibility') {
      app.commandLine.appendSwitch('use-gl', 'angle');
      app.commandLine.appendSwitch('use-angle', 'swiftshader');
      app.commandLine.appendSwitch('enable-unsafe-swiftshader');
      this.log('mode', 'SwiftShader explicitly selected; Chromium software WebGL uses lower security guarantees for trusted local content');
    }
    app.on('gpu-info-update', () => this.refreshGPUInfo());
    app.on('child-process-gone', (_event, details) => {
      if (details.type === 'GPU') this.log('gpu-process', details);
    });
  }
  log(source, detail) {
    this.logs.push({time: new Date().toISOString(), source, detail});
    if (this.logs.length > 120) this.logs.shift();
  }
  async refreshGPUInfo(timeoutMs = 1500) {
    // Keep one outstanding Electron query. A stalled GPU process must not hold
    // mode selection, diagnostic export or the rescue interface indefinitely
    if (!this.refreshing) {
      this.refreshing = Promise.resolve().then(() => this.app.getGPUInfo('complete')).then(info => {
        this.gpuInfo = info; this.gpuInfoError = ''; this.gpuInfoWaitTimedOut = false;
      }).catch(error => {
        this.gpuInfoError = String(error.message || error); this.gpuInfoWaitTimedOut = false;
      }).finally(() => { this.refreshing = null; });
    }
    let timer;
    try {
      await Promise.race([this.refreshing, new Promise(resolve => {
        timer = setTimeout(() => { this.gpuInfoWaitTimedOut = true; resolve(); }, timeoutMs);
      })]);
    } finally { clearTimeout(timer); }
  }
  attach(webContents) {
    webContents.on('console-message', (_event, details) => {
      if (details && /webgl|three|opengl|angle|swiftshader|graphics|context lost/i.test(details.message || '')) {
        this.log('renderer-console', {level: details.level, message: String(details.message).slice(0, 4000), sourceId: details.sourceId, lineNumber: details.lineNumber});
      }
    });
    webContents.on('render-process-gone', (_event, details) => this.log('renderer-process', details));
  }
  report(value) {
    if (!value || typeof value !== 'object' || Array.isArray(value)) throw new Error('Invalid graphics report');
    const serialized = JSON.stringify(value);
    if (serialized.length > 32768) throw new Error('Graphics report is too large');
    const report = {time: new Date().toISOString(), ...JSON.parse(serialized)};
    this.reports.push(report);
    if (this.reports.length > 40) this.reports.shift();
    return {received: true};
  }
  select(mode) {
    this.requestedMode = mode;
    // Rescue does not change Chromium's process-wide backend. Returning to the
    // already running backend only needs the view to be recreated
    this.effectiveMode = mode === 'rescue' || mode === this.backendMode ? mode : this.effectiveMode;
    this.blockedReason = '';
  }
  nativeLog() {
    let file;
    try {
      file = fs.openSync(this.nativeLogPath, 'r');
      const size = fs.fstatSync(file).size;
      const buffer = Buffer.alloc(Math.min(size, 65536));
      fs.readSync(file, buffer, 0, buffer.length, Math.max(0, size - buffer.length));
      return {path: this.nativeLogPath, tail: buffer.toString('utf8')};
    } catch (error) { return {path: this.nativeLogPath, unavailable: error.code || String(error.message || error)}; }
    finally { if (file !== undefined) fs.closeSync(file); }
  }
  status() {
    const requestedBackend = this.requestedMode === 'compatibility' ? 'compatibility' : 'normal';
    return {
      effectiveMode: this.effectiveMode, requestedMode: this.requestedMode,
      backendMode: this.backendMode,
      requiresRestart: this.requestedMode !== 'rescue' && requestedBackend !== this.backendMode,
      unsafeSoftware: this.app.commandLine.hasSwitch('enable-unsafe-swiftshader'),
      compatibilityAvailable: this.compatibilityAvailable, blockedReason: this.blockedReason,
      launchOverride: this.launchOverride,
      diagnostics: {
        capturedAt: new Date().toISOString(), startedAt: this.startedAt,
        versions: {...process.versions}, platform: process.platform, arch: process.arch,
        commandLine: process.argv.slice(), chromiumSwitches: Object.fromEntries(['use-gl', 'use-angle', 'enable-unsafe-swiftshader', 'disable-gpu', 'disable-software-rasterizer', 'ignore-gpu-blocklist', 'ozone-platform', 'no-sandbox', 'enable-logging', 'log-file'].filter(name => this.app.commandLine.hasSwitch(name)).map(name => [name, this.app.commandLine.getSwitchValue(name)])),
        environment: Object.fromEntries(['DISPLAY', 'WAYLAND_DISPLAY', 'XDG_SESSION_TYPE', 'ELECTRON_OZONE_PLATFORM_HINT', 'LIBGL_ALWAYS_SOFTWARE', 'MESA_LOADER_DRIVER_OVERRIDE', 'VK_ICD_FILENAMES'].filter(key => process.env[key]).map(key => [key, process.env[key]])),
        gpuFeatureStatus: this.app.getGPUFeatureStatus(), gpuInfo: this.gpuInfo, gpuInfoError: this.gpuInfoError,
        gpuInfoPending: !!this.refreshing, gpuInfoWaitTimedOut: this.gpuInfoWaitTimedOut,
        softwareRuntime: this.resources, nativeLog: this.nativeLog(), logs: this.logs.slice(), rendererReports: this.reports.slice(),
      },
    };
  }
}

module.exports = {GraphicsRuntime, MODES, SOFTWARE_FILES};
