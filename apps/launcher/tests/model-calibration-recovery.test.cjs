const test=require('node:test');
const assert=require('node:assert/strict');
const fs=require('node:fs');
const path=require('node:path');
const base=path.resolve(__dirname,'../../..');
const read=p=>fs.readFileSync(path.join(base,p),'utf8');
test('record inspector recognizes no-result task without starting hardware',()=>{
  const persist=read('apps/launcher/backend/persistence.py');
  assert.match(persist,/record_kind = 'teaching_only'/);
  assert.match(persist,/can_resume = trajectory\['valid'\]/);
  const bridge=read('apps/launcher/backend/bridge.py');
  const branch=bridge.slice(bridge.indexOf("elif method == 'model_calibration_load':"),bridge.indexOf("elif method == 'model_calibration_preview_save':"));
  assert.doesNotMatch(branch,/native\.request/);
});
test('recover imports without actuation and operator separately confirms replay',()=>{
  const native=read('src/serial_arm/core/app/serial_arm_terminal.cpp');
  const ui=read('apps/launcher/renderer/app.js');
  assert.match(native,/machine_model_calibration_import_trajectory/);
  assert.match(native,/source_task_id_.*empty/);
  assert.match(native,/model_calibration_file_fingerprint\(config_path_\)/);
  assert.match(native,/model_calibration_file_fingerprint\(cfg_\.dynamics\.urdf_path\)/);
  assert.match(native,/imported replay start pose mismatch/);
  const importer=native.slice(native.indexOf('void machine_model_calibration_import_trajectory('),native.indexOf('void machine_model_calibration_teach_stop('));
  assert.doesNotMatch(importer,/model_calibration_worker_=std::thread/);
  assert.match(ui,/model-calibration-import-trajectory/);
  assert.match(ui,/model-calibration-back-live/);
  assert.match(ui,/model_calibration_start.*modelCalibrationTaskParams\(\{ released: true \}\)/);
});
