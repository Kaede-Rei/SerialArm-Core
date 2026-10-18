const fs = require('node:fs');
const path = require('node:path');
const root = path.resolve(__dirname, '..');
for (const f of ['desktop/main.cjs', 'desktop/preload.cjs', 'desktop/bridge.cjs', 'backend/bridge.py', 'backend/profiles.py', 'backend/runtime.py', 'backend/pty_child.py', 'backend/machine.py', 'renderer/index.html', 'renderer/app.js', 'renderer/model-view.js', 'renderer/styles.css', 'scripts/vendor-three.cjs', 'node_modules/@xterm/xterm/lib/xterm.js', 'node_modules/@xterm/addon-fit/lib/addon-fit.js', 'node_modules/three/build/three.module.js', 'renderer/vendor/three/build/three.module.js', 'renderer/vendor/three/jsm/controls/OrbitControls.js', 'renderer/vendor/three/jsm/loaders/STLLoader.js', 'renderer/vendor/three/jsm/loaders/GLTFLoader.js']) {
  if (!fs.statSync(path.join(root, f)).isFile()) throw new Error('Missing launcher file: ' + f);
}
// Public renderer requests must be accepted by Electron's explicit IPC whitelist.
// Dynamic calibration_candidate_* calls are covered by their concrete backend methods.
const main = fs.readFileSync(path.join(root, 'desktop/main.cjs'), 'utf8');
const renderer = fs.readFileSync(path.join(root, 'renderer/app.js'), 'utf8');
const section = main.match(/const methods = new Set\(\[([\s\S]*?)\]\);/);
if (!section) throw new Error('Cannot inspect Electron IPC method whitelist');
const allowed = new Set([...section[1].matchAll(/'([a-z_]+)'/g)].map(m => m[1]));
const literalMethods = new Set([...renderer.matchAll(/\bapi\.request\(\s*'([a-z_]+)'/g)].map(m => m[1]));
for (const method of literalMethods) {
  if (method === 'logs' || method === 'calibration_candidate_') continue;
  if (!allowed.has(method)) throw new Error('Renderer request not allowed by Electron: ' + method);
}
console.log('LAUNCHER_STATIC_PASS');
