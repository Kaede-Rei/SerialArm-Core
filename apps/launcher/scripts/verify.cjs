const fs = require('node:fs');
const path = require('node:path');
const root = path.resolve(__dirname, '..');
for (const f of ['desktop/main.cjs', 'desktop/preload.cjs', 'desktop/bridge.cjs', 'backend/bridge.py', 'backend/profiles.py', 'backend/runtime.py', 'backend/pty_child.py', 'renderer/index.html', 'renderer/app.js', 'renderer/styles.css', 'node_modules/@xterm/xterm/lib/xterm.js', 'node_modules/@xterm/addon-fit/lib/addon-fit.js']) {
  if (!fs.statSync(path.join(root, f)).isFile()) throw new Error('Missing launcher file: ' + f);
}
console.log('LAUNCHER_STATIC_PASS');
