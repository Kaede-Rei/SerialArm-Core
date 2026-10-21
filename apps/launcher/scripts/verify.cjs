const fs = require('node:fs');
const path = require('node:path');
const root = path.resolve(__dirname, '..');
for (const f of ['desktop/main.cjs', 'desktop/preload.cjs', 'desktop/bridge.cjs', 'backend/bridge.py', 'backend/profiles.py', 'backend/runtime.py', 'backend/pty_child.py', 'backend/machine.py', 'renderer/index.html', 'renderer/app.js', 'renderer/model-view.js', 'renderer/styles.css', 'scripts/vendor-three.cjs', 'node_modules/@xterm/xterm/lib/xterm.js', 'node_modules/@xterm/addon-fit/lib/addon-fit.js', 'node_modules/three/build/three.module.js', 'renderer/vendor/three/build/three.module.js', 'renderer/vendor/three/jsm/controls/OrbitControls.js', 'renderer/vendor/three/jsm/loaders/STLLoader.js', 'renderer/vendor/three/jsm/loaders/GLTFLoader.js']) {
  if (!fs.statSync(path.join(root, f)).isFile()) throw new Error('Missing launcher file: ' + f);
}
// Every renderer request must be present in Electron's exact IPC whitelist.
// Renderer groups such as geometry_repeat_ + op are prefixes, not actual RPC methods.
// Expand and validate their concrete RPC names instead of falsely rejecting the prefix
// or unconditionally skipping validation.
const main = fs.readFileSync(path.join(root, 'desktop/main.cjs'), 'utf8');
const renderer = fs.readFileSync(path.join(root, 'renderer/app.js'), 'utf8');
const bridge = fs.readFileSync(path.join(root, 'backend/bridge.py'), 'utf8');
const section = main.match(/const methods = new Set\(\[([\s\S]*?)\]\);/);
if (!section) throw new Error('Cannot inspect Electron IPC method whitelist');
const allowed = new Set([...section[1].matchAll(/'([a-z_]+)'/g)].map(m => m[1]));
const literalMethods = new Set([...renderer.matchAll(/\bapi\.request\(\s*'([a-z_]+)'/g)].map(m => m[1]));
const dynamicGroups = new Map([
  ['geometry_fixed_', ['state', 'begin', 'capture', 'solve', 'validate', 'discard', 'report', 'export_tcp']],
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
console.log('LAUNCHER_STATIC_PASS');
