const test = require('node:test');
const assert = require('node:assert/strict');
const fs = require('node:fs');
const path = require('node:path');

const root = path.resolve(__dirname, '../../..');
const read = rel => fs.readFileSync(path.join(root, rel), 'utf8');

test('embedded model uses bounded IPC and keeps renderer sandboxed', () => {
  const main = read('apps/launcher/desktop/main.cjs');
  const preload = read('apps/launcher/desktop/preload.cjs');
  const model = read('apps/launcher/renderer/model-view.js');
  assert.match(main, /allowedModelResources = new Set/);
  assert.match(main, /Model resource was not resolved by the active model/);
  assert.match(main, /\['\.stl', '\.glb'\]/);
  assert.match(main, /nodeIntegration: false/);
  assert.match(main, /contextIsolation: true/);
  assert.match(preload, /modelResource:/);
  assert.match(model, /STLLoader/);
  assert.match(model, /GLTFLoader/);
  assert.match(model, /AxesHelper/);
  assert.match(model, /inertiaShape/);
});

test('workbench uses TerminalApp machine mode instead of terminal menu automation', () => {
  const bridge = read('apps/launcher/backend/bridge.py');
  const machine = read('apps/launcher/backend/machine.py');
  const terminal = read('src/serial_arm/core/app/serial_arm_terminal.cpp');
  assert.match(bridge, /workbench_request/);
  assert.match(bridge, /'--machine'/);
  assert.match(machine, /json\.dumps\(\{'id': number, 'method': method, 'params': params\}/);
  assert.match(terminal, /int run_machine\(\)/);
  assert.match(terminal, /method == "hold"/);
  assert.match(terminal, /method == "shutdown"/);
  assert.match(terminal, /park_and_deactivate\(\)/);
  assert.match(terminal, /machine_validate_target/);
  assert.doesNotMatch(bridge, /menu.*number|stdin.*menu/i);
});

test('standalone CMake installs native model probe and shared terminal machine interface', () => {
  const cmake = read('src/serial_arm/core/CMakeLists.txt');
  const terminal = read('src/serial_arm/core/app/serial_arm_terminal.cpp');
  assert.match(cmake, /add_executable\(serial_arm_model_probe/);
  assert.match(cmake, /install\(TARGETS serial_arm_terminal serial_arm_model_probe/);
  assert.match(terminal, /--machine/);
  assert.doesNotMatch(cmake, /serial_arm_session_server/);
});

test('model is a top-level workspace and runtime page only contains executable modes', () => {
  const app = read('apps/launcher/renderer/app.js');
  assert.match(app, /\['terminal','hardware','moveit'\]\.map/);
  assert.doesNotMatch(app, /\['model','terminal','hardware','moveit'\]\.map/);
  assert.match(app, /modelTitle:\['模型工作台'/);
  assert.match(app, /modelPageDesc:\['查看机器人结构、坐标系、关节运动与动力学参数'/);
});

test('model preview separates overlays, supports per-object visibility, and streams drag updates', () => {
  const app = read('apps/launcher/renderer/app.js');
  const model = read('apps/launcher/renderer/model-view.js');
  const probe = read('src/serial_arm/core/app/serial_arm_model_probe.cpp');
  assert.match(app, /linkFrames:true,jointAxes:true,com:true,inertia:true/);
  assert.match(app, /data-model-visible/);
  assert.match(app, /model_preview/);
  assert.match(app, /requestAnimationFrame/);
  assert.doesNotMatch(app, /setTimeout\(\(\)=>operation\(previewModel\),80\)/);
  assert.match(model, /setObjectVisible\(kind,name,visible\)/);
  assert.match(model, /userData\.layer='linkFrames'/);
  assert.match(model, /userData\.layer='jointAxes'/);
  assert.match(model, /userData\.layer='com'/);
  assert.match(model, /userData\.layer='inertia'/);
  assert.match(probe, /--server/);
  assert.match(probe, /while\(std::getline\(std::cin, line\)\)/);
});


test('model visibility is strict and inspection follows URDF field names', () => {
  const app = read('apps/launcher/renderer/app.js');
  const model = read('apps/launcher/renderer/model-view.js');
  assert.match(app, /model-layers-all/);
  assert.match(app, /model-layers-none/);
  assert.match(app, /model-show-all/);
  assert.match(app, /model-hide-all/);
  assert.match(app, /state\.modelSelection===next\?'':next/);
  assert.match(app, /&lt;origin xyz&gt;/);
  assert.match(app, /&lt;mass value&gt;/);
  assert.match(app, /`&lt;inertia \${k}&gt;`/);
  assert.match(app, /&lt;axis xyz&gt;/);
  assert.match(model, /mouseButtons\.MIDDLE = THREE\.MOUSE\.PAN/);
  assert.match(model, /requiredLayers\.every/);
  assert.match(model, /\['com','labels'\]/);
  assert.doesNotMatch(model, /frameLabels/);
});

test('link frames use independent visibility and high-contrast XYZ arrows', () => {
  const app = read('apps/launcher/renderer/app.js');
  const model = read('apps/launcher/renderer/model-view.js');
  assert.match(app, /modelVisibility:\{links:\{\},linkFrames:\{\},joints:\{\}\}/);
  assert.match(app, /data-model-frame=/);
  assert.match(app, /setLinkFrameVisible\(link\.name,visible\)/);
  assert.match(model, /ownerKind='linkFrames'/);
  assert.match(model, /objectVisibility=\{links:new Map\(\),linkFrames:new Map\(\),joints:new Map\(\)\}/);
  assert.match(app, /modelView\?\.setLinkFrameVisible\(name,next\)/);
  assert.match(model, /setLinkFrameVisible\(name,visible\)/);
  assert.match(model, /helper\.visible=!!this\.layers\.linkFrames&&next/);
  assert.match(model, /for\(const \[name,helper\] of this\.frameHelpers\)helper\.visible=!!this\.layers\.linkFrames/);
  assert.match(model, /for \(const link of payload\.model\?\.links \|\| \[]\)/);
  assert.match(model, /const frame = frames\.get\(link\.name\)/);
  assert.doesNotMatch(model, /for \(const frame of payload\.native\?\.state\?\.frames \|\| \[]\) this\.addFrame/);
  assert.match(model, /if \(this\.frameHelpers\.has\(frame\.name\)\) return/);
  assert.match(model, /new THREE\.ArrowHelper\(direction,new THREE\.Vector3\(\),length,color/);
  assert.match(model, /0xef4444/);
  assert.match(model, /0x22c55e/);
  assert.match(model, /0x3b82f6/);
  assert.match(model, /depthTest=false/);
});
