const test=require('node:test');
const assert=require('node:assert/strict');
const fs=require('node:fs');
const path=require('node:path');
const root=path.resolve(__dirname,'../../..');
const read=x=>fs.readFileSync(path.join(root,x),'utf8');
test('imported trajectory selects closest original endpoint without an extra joint-space reposition',()=>{
  const cpp=read('src/serial_arm/core/app/serial_arm_terminal.cpp');
  const block=cpp.slice(cpp.indexOf('void machine_model_calibration_import_trajectory('),cpp.indexOf('void machine_model_calibration_teach_stop('));
  assert.match(block,/calibration_recovery::select_endpoint/);
  assert.match(block,/std::reverse\(candidate\.positions\.begin\(\),candidate\.positions\.end\(\)\)/);
  assert.match(block,/if\(!endpoint_choice\.valid\)/);
  assert.match(block,/calibration_retime::plan/);
  assert.doesNotMatch(block,/(?:begin_stream|machine_move_to)\(/);
});
test('manual alignment requires operator support and cannot start replay while dragging',()=>{
  const cpp=read('src/serial_arm/core/app/serial_arm_terminal.cpp');
  const begin=cpp.slice(cpp.indexOf('void machine_model_calibration_alignment_begin('),cpp.indexOf('void machine_model_calibration_alignment_finish('));
  const stop=cpp.slice(cpp.indexOf('void machine_model_calibration_alignment_finish('),cpp.indexOf('void machine_model_calibration_start('));
  assert.match(begin,/params\["supported"\]/);
  assert.match(begin,/COMPLIANT_DRAG/);
  assert.match(begin,/feedback_time_ns_/);
  assert.match(stop,/RIGID_HOLD/);
  assert.match(cpp,/if\(model_calibration_alignment_active_\.load\(\)\)[\s\S]{0,180}finish manual endpoint alignment/);
});
test('GUI displays per-joint deltas and requires separate replay confirmation',()=>{
  const js=read('apps/launcher/renderer/app.js');
  assert.match(js,/calibrationAlignment\(status/);
  assert.match(js,/model-calibration-align-rows/);
  assert.match(js,/model-calibration-alignment-begin/);
  assert.match(js,/model-calibration-alignment-finish/);
  assert.match(js,/model_calibration_start.*released: true/);
  assert.match(js,/no environment collision planner/i);
});
