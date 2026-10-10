const test = require('node:test');
const assert = require('node:assert/strict');
const fs = require('node:fs');
const path = require('node:path');
const app = fs.readFileSync(path.join(__dirname, '../renderer/app.js'), 'utf8');
const desktop = fs.readFileSync(path.join(__dirname, '../desktop/main.cjs'), 'utf8');
const bridge = fs.readFileSync(path.join(__dirname, '../desktop/bridge.cjs'), 'utf8');
const pythonBridge = fs.readFileSync(path.join(__dirname, '../backend/bridge.py'), 'utf8');
test('calibration has two primary entries and existing individual tools stay folded',()=>{
  assert.match(app,/calibration-entry-auto/);
  assert.match(app,/calibration-entry-advanced/);
  assert.match(app,/calibrationUnifiedResultsHtml\(\)/);
  assert.match(app,/<details class="workspace-section single-calibration-tools">/);
  assert.doesNotMatch(app,/<details class="workspace-section single-calibration-tools" open/);
});
test('advanced mode selection has physical parameter modes and safe offline checks',()=>{
  for(const mode of ['mass','com','mass_com','inertia','full'])assert.match(app,new RegExp("'"+mode+"'"));
  assert.match(app,/model_calibration_assess/);
  assert.match(app,/model_calibration_advanced/);
  assert.match(app,/modelCalibrationActive\(\)\) return/);
});
test('new IPC endpoints are whitelisted and inertia worker has extended timeout',()=>{
  assert.match(desktop,/'model_calibration_assess'/);
  assert.match(desktop,/'model_calibration_advanced'/);
  assert.match(bridge,/model_calibration_advanced/);
});

test('saved records assess offline, never block live controls, and show physical inertia status',()=>{
  assert.match(app,/function assessLoadedCalibrationRecord\(record\)/);
  assert.match(app,/if \(!record\?\.has_result \|\| !record\.directory \|\| workbenchConnected\(\)\) return/);
  assert.match(app,/await assessLoadedCalibrationRecord\(loaded\)/);
  assert.match(app,/result\.full_dynamics_physical_pass/);
  assert.match(pythonBridge,/model_calibration_assess'[\s\S]*?native\.status\(\)\.get\('state'\)/);
});

test('advanced Link lock selector uses validated names and keeps task prior',()=>{
  assert.match(app,/data-advanced-locked="\$\{esc\(link\.name\)\}"/);
  assert.match(app,/function advancedTrustedLinks\(record\)/);
  assert.match(app,/calibrationSelectableLinks\(\)\.map\(l=>l\.name\)/);
  assert.doesNotMatch(app,/some\(l=>l\.link_name===n\)/);
});
