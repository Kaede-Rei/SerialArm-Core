const test = require('node:test');
const assert = require('node:assert/strict');
const fs = require('node:fs');
const path = require('node:path');
const root = path.resolve(__dirname, '../../..');
const read = rel => fs.readFileSync(path.join(root, rel), 'utf8');

test('calibration uses one reverse static traversal, one forward dynamics traversal and quality gates', () => {
  const source = read('src/serial_arm/core/app/serial_arm_terminal.cpp');
  const start = source.indexOf('void model_calibration_worker()');
  const end = source.indexOf('void machine_model_calibration_start(', start);
  const worker = source.slice(start, end);
  assert.match(worker, /model_calibration_static_pass\(true\)/);
  assert.match(worker, /model_calibration_dynamic_pass\(false,model_calibration_replay_rate_,2/);
  assert.doesNotMatch(worker, /model_calibration_static_pass\(false\)/);
  assert.doesNotMatch(worker, /model_calibration_dynamic_pass\(true/);
  assert.match(worker, /r\.observable\.size\(\)==cfg_\.joint_names\.size\(\)/);
  assert.match(worker, /r\.validation_pass\.size\(\)==cfg_\.joint_names\.size\(\)/);
  assert.match(source, /2\.0 \* planned_pass_s \+ static_collection_s/);
  assert.match(source, /model_calibration_direction_\.store\(reverse \? -1 : 1\)/);
});

test('full inertia candidate stays opt-in and cannot be applied by a launcher command', () => {
  const main=read('apps/launcher/desktop/main.cjs');
  const bridge=read('apps/launcher/backend/bridge.py');
  const app=read('apps/launcher/renderer/app.js');
  const tool=read('apps/launcher/backend/full_inertial.py');
  assert.match(main, /'model_calibration_export_full_inertial'/);
  assert.match(bridge, /export_full_inertial_candidate/);
  assert.match(app, /model-calibration-export-inertial/);
  assert.match(tool, /all_parameters_identifiable/);
  assert.match(tool, /physical_inertia/);
  assert.doesNotMatch(tool, /robot\.set_cmd/);
});
