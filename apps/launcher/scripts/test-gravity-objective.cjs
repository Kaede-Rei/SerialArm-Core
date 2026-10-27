const test = require('node:test');
const assert = require('node:assert/strict');
const fs = require('node:fs');
const vm = require('node:vm');
const source = fs.readFileSync(require('node:path').join(__dirname, '../renderer/app.js'), 'utf8');
function context(objective) {
  const sandbox = {state:{modelCalibrationObjective:objective}};
  vm.createContext(sandbox);
  const match = source.match(/function adoptGravityRecord\([^]*?\n\}/);
  assert.ok(match);
  vm.runInContext(match[0], sandbox);
  return sandbox;
}
test('loading or refreshing raw report preserves next bias-separated choice', () => {
  const ui = context('bias_separated');
  const report = {fit_objective:'urdf',static_pass:false};
  const record = {result:{},recomputed_candidate:report};
  ui.adoptGravityRecord(record);
  ui.adoptGravityRecord(record);
  assert.equal(ui.state.modelCalibrationObjective,'bias_separated');
  assert.equal(record.result.gravity_result.fit_objective,'urdf');
  assert.equal(record.result.static_pass,false);
});
test('loading bias report preserves explicit raw experiment choice', () => {
  const ui = context('urdf');
  const record = {result:{},recomputed_candidate:{fit_objective:'bias_separated',static_pass:false}};
  ui.adoptGravityRecord(record);
  assert.equal(ui.state.modelCalibrationObjective,'urdf');
  assert.equal(record.result.gravity_result.fit_objective,'bias_separated');
});
test('loading online or legacy report does not silently choose a fitting target', () => {
  const ui = context('bias_separated');
  ui.adoptGravityRecord({result:{}});
  assert.equal(ui.state.modelCalibrationObjective,'bias_separated');
  assert.equal(ui.state.modelCalibrationCandidateSource,'online');
  ui.adoptGravityRecord({result:{},recomputed_candidate:{static_pass:false}});
  assert.equal(ui.state.modelCalibrationObjective,'bias_separated');
});
