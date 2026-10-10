const test = require('node:test');
const assert = require('node:assert/strict');
const fs = require('node:fs');
const path = require('node:path');
const root = path.resolve(__dirname, '../../..');
const read = rel => fs.readFileSync(path.join(root, rel), 'utf8');

test('C++ local gravity solver locks parameters before matrix assembly', () => {
  const cpp=read('src/serial_arm/core/src/dynamics/gravity_calibration.cpp');
  assert.match(cpp,/std::vector<Eigen::Index> active_columns/);
  assert.match(cpp,/active_columns\[col\]/);
  assert.match(cpp,/locked link does not belong/);
  assert.match(cpp,/all identifiable links are locked/);
  assert.match(cpp,/std::find\(fitted_links\.begin\(\),fitted_links\.end\(\),info\.link_name\)/);
  assert.match(cpp,/result\.parameter_observable\[static_cast<std::size_t>\(active_columns\[col\]\)\]/);
});

test('Local mode is carried through GUI, offline calibration and link ownership', () => {
  const ui=read('apps/launcher/renderer/app.js');
  const native=read('src/serial_arm/core/app/serial_arm_terminal.cpp');
  const offline=read('src/serial_arm/core/app/serial_arm_model_calibrator.cpp');
  assert.match(ui,/model-cal-mode-local/);
  assert.match(ui,/data-gravity-locked/);
  assert.match(ui,/calibrationSelectableLinks/);
  assert.match(ui,/model_calibration_recompute.*?captureModelCalibrationOptions\(\)/);
  assert.match(native,/model_calibration_links_cache_/);
  assert.match(native,/model_calibration_options_\.locked_links/);
  assert.match(native,/model_calibration_options_\.identification_mode/);
  assert.match(offline,/--lock-link/);
  assert.match(offline,/--mode/);
});

test('Friction candidate export has Python route and Electron allowlist', () => {
  const main=read('apps/launcher/desktop/main.cjs');
  const bridge=read('apps/launcher/backend/bridge.py');
  const persistence=read('apps/launcher/backend/persistence.py');
  assert.match(main,/'model_calibration_export_friction'/);
  assert.match(bridge,/elif method == 'model_calibration_export_friction'/);
  assert.match(persistence,/schema': 'serial_arm_friction_candidate_review'/);
  assert.match(persistence,/friction_matches_gravity/);
  assert.match(persistence,/locked Link first moments were modified/);
});
