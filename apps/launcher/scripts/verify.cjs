const fs = require('node:fs');
const path = require('node:path');
const root = path.resolve(__dirname, '..');
for (const f of ['desktop/main.cjs', 'desktop/gain-live-window.cjs', 'desktop/gain-live-preload.cjs', 'renderer/gain-live.html', 'renderer/gain-live.js', 'renderer/gain-live.css', 'desktop/preload.cjs', 'desktop/bridge.cjs', 'desktop/resources.cjs', 'desktop/graphics.cjs', 'backend/bridge.py', 'backend/gain_context.py', 'backend/profiles.py', 'backend/runtime.py', 'backend/pty_child.py', 'backend/machine.py', 'renderer/index.html', 'renderer/app.js', 'renderer/capability-response.js', 'renderer/motion-goals.js', 'renderer/model-view.js', 'renderer/model-geometry-worker.mjs', 'renderer/model-camera-zoom.mjs', 'renderer/model-camera-transition.mjs', 'renderer/styles.css', 'renderer/studio.css', 'renderer/studio-motion.js', 'renderer/brand/arm.png', 'renderer/brand/association.png', 'scripts/vendor-three.cjs', 'node_modules/@xterm/xterm/lib/xterm.js', 'node_modules/@xterm/addon-fit/lib/addon-fit.js', 'node_modules/three/build/three.module.js', 'renderer/vendor/three/build/three.module.js', 'renderer/vendor/three/jsm/controls/OrbitControls.js', 'renderer/vendor/three/jsm/loaders/STLLoader.js', 'renderer/vendor/three/jsm/loaders/GLTFLoader.js']) {
  if (!fs.statSync(path.join(root, f)).isFile()) throw new Error('Missing launcher file: ' + f);
}
for (const name of ['maple-mono-cn-italic.woff2', 'maple-mono-cn-medium-italic.woff2', 'maple-mono-cn-semibold-italic.woff2', 'OFL.txt']) {
  if (!fs.statSync(path.join(root, 'renderer/fonts', name)).isFile()) throw new Error('Missing bundled font resource: ' + name);
}
// The file:// model view owns this local module worker; no CDN or alternate
// geometry implementation is required to prepare a truthful target preview.
const modelViewSource=fs.readFileSync(path.join(root,'renderer/model-view.js'),'utf8');
if(!modelViewSource.includes("new Worker(new URL('./model-geometry-worker.mjs',import.meta.url),{type:'module'})"))
  throw Error('Model preview geometry worker is not referenced by the source view');
// Compatibility rendering uses the software backend shipped with this Electron
// runtime; do not replace a missing library with host graphics dependencies
const {SOFTWARE_FILES} = require('../desktop/graphics.cjs');
const electronDist = path.join(root, 'node_modules/electron/dist');
for (const name of SOFTWARE_FILES) {
  const file = path.join(electronDist, name);
  if (!fs.existsSync(file) || !fs.statSync(file).isFile()) {
    throw new Error('Missing Electron graphics runtime: ' + name + '; reinstall the GUI');
  }
}
const softwareICD = JSON.parse(fs.readFileSync(path.join(electronDist, 'vk_swiftshader_icd.json'), 'utf8'));
if (softwareICD.ICD?.library_path !== './libvk_swiftshader.so') {
  throw new Error('Electron SwiftShader ICD does not reference its bundled library');
}
// Every renderer request must be present in Electron's exact IPC whitelist.
// Renderer groups such as geometry_repeat_ + op are prefixes, not actual RPC methods.
// Expand and validate their concrete RPC names instead of falsely rejecting the prefix
// or unconditionally skipping validation
const main = fs.readFileSync(path.join(root, 'desktop/main.cjs'), 'utf8');
const renderer = fs.readFileSync(path.join(root, 'renderer/app.js'), 'utf8');
const bridge = fs.readFileSync(path.join(root, 'backend/bridge.py'), 'utf8');
const section = main.match(/const methods = new Set\(\[([\s\S]*?)\]\);/);
if (!section) throw new Error('Cannot inspect Electron IPC method whitelist');
const allowed = new Set([...section[1].matchAll(/'([a-z_]+)'/g)].map(m => m[1]));
const literalMethods = new Set([...renderer.matchAll(/\bapi\.request\(\s*'([a-z_]+)'/g)].map(m => m[1]));
const dynamicGroups = new Map([
  ['geometry_fixed_', ['state', 'begin', 'capture', 'solve', 'validate', 'discard', 'report', 'preview_tcp', 'export_tcp', 'apply_tcp']],
  ['geometry_repeat_', ['status', 'record_begin', 'record_stop', 'load', 'save', 'play', 'stop']],
]);
for (const [prefix, operations] of dynamicGroups) {
  if (!literalMethods.has(prefix)) throw new Error('Missing renderer request family: ' + prefix);
  if (!bridge.includes(`method.startswith('${prefix}')`)) throw new Error('Missing Python Bridge route: ' + prefix);
  for (const operation of operations) {
    if (!allowed.has(prefix + operation)) {
      throw new Error('Geometry request not allowed by Electron: ' + prefix + operation);
    }
  }
  const expected = new Set(operations);
  for (const method of allowed) {
    if (method.startsWith(prefix) && !expected.has(method.slice(prefix.length))) {
      throw new Error('Unrecognised geometry request in Electron whitelist: ' + method);
    }
  }
}
for (const method of literalMethods) {
  if (method === 'logs' || method === 'calibration_candidate_' || dynamicGroups.has(method)) continue;
  if (!allowed.has(method)) throw new Error('Renderer request not allowed by Electron: ' + method);
}
require('./verify-native-runtime.cjs');
require('./verify-gain-calibration.cjs');
console.log('LAUNCHER_STATIC_PASS');
