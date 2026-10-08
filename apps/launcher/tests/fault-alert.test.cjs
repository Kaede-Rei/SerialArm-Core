const test = require('node:test');
const assert = require('node:assert/strict');
const fs = require('node:fs');
const vm = require('node:vm');
const path = require('node:path');
const root = path.resolve(__dirname, '../../..');
const app = fs.readFileSync(path.join(root, 'apps/launcher/renderer/app.js'), 'utf8');
const cpp = fs.readFileSync(path.join(root, 'src/serial_arm/core/app/serial_arm_terminal.cpp'), 'utf8');
const resolver = fs.readFileSync(path.join(root, 'src/serial_arm/core/src/config/limit_resolver.cpp'), 'utf8');

function errorHarness() {
  const ids = ['priority-error-overlay','priority-error-title','priority-error-detail','priority-error-guidance','priority-error-ack','confirm-dialog'];
  const nodes = Object.fromEntries(ids.map(id => [id, { hidden: true, textContent: '', attributes: {}, setAttribute(k,v){ this.attributes[k] = v; }, focus() { this.focused = true; } }]));
  nodes['priority-error-overlay'].open = false;
  nodes['priority-error-overlay'].showModal = () => { nodes['priority-error-overlay'].open = true; };
  nodes['priority-error-overlay'].close = () => { nodes['priority-error-overlay'].open = false; };
  nodes['confirm-dialog'].open = true;
  nodes['confirm-dialog'].close = () => { nodes['confirm-dialog'].open = false; };
  let loading = 'running';
  const begin = app.indexOf('const priorityErrorQueue = []');
  const end = app.indexOf('const button = ', begin);
  assert.ok(begin > 0 && end > begin, 'priority error functions exist');
  const js = app.slice(begin,end) + '; globalThis.api = {showPriorityError, acknowledgePriorityError, checkRuntimeAlerts, describeRuntimeFault};';
  const sandbox = {
    $: id => nodes[id],
    busyText: zh => zh,
    t: key => key,
    setOperationOverlay: value => { loading = value; },
    updateModelCalibrationTaskOverlay: () => {},
    state: { language: 'zh-CN' },
    console,
    Number, JSON, Set,
  };
  vm.runInNewContext(js, sandbox);
  return { api: sandbox.api, nodes, get loading() {return loading;} };
}

test('a safety fault supersedes a running operation and confirmation', () => {
  const h = errorHarness();
  h.api.checkRuntimeAlerts({robot_state:'FAULT',joint_names:['joint1'], fault: {
    code: 'SAFETY_FAILED',safety_code:'JOINT_VEL_LIMIT', joint_index:0,joint_name:'joint1',value:2.6,limit:2.5
  },safety_limits:{max_cmd_vel:[1.35],max_state_vel:[5.4]}});
  assert.equal(h.nodes['priority-error-overlay'].open,true);
  assert.equal(h.nodes['confirm-dialog'].open,false);
  assert.equal(h.loading,'');
  assert.match(h.nodes['priority-error-detail'].textContent,/joint1/);
  assert.match(h.nodes['priority-error-detail'].textContent,/2\.6000/);
  assert.match(h.nodes['priority-error-detail'].textContent,/2\.5000/);
  assert.match(h.nodes['priority-error-detail'].textContent,/5\.4000/);
  h.api.acknowledgePriorityError();
  assert.equal(h.nodes['priority-error-overlay'].open,false);
  // The same telemetry fault should not reopen after acknowledgment.
  h.api.checkRuntimeAlerts({robot_state:'FAULT',fault: {code:'SAFETY_FAILED', safety_code:'JOINT_VEL_LIMIT',joint_index:0,joint_name:'joint1',value:2.6,limit:2.5}});
  assert.equal(h.nodes['priority-error-overlay'].open,false);
});

test('hardware FAULT preempts even an active ordinary error dialog', () => {
  const h = errorHarness();
  h.api.showPriorityError('Earlier noncritical failure');
  assert.match(h.nodes['priority-error-detail'].textContent,/Earlier/);
  h.api.showPriorityError('JOINT_VEL_LIMIT', 'fault');
  assert.match(h.nodes['priority-error-detail'].textContent,/JOINT_VEL_LIMIT/);
  assert.match(h.nodes['priority-error-title'].textContent,/安全故障/);
  h.api.acknowledgePriorityError();
  assert.match(h.nodes['priority-error-detail'].textContent,/Earlier/);
});

test('model calibration failure surfaces independently of the calibration overlay', () => {
  const h = errorHarness();
  h.api.checkRuntimeAlerts({robot_state:'ACTIVE',model_calibration:{task_id:'abc',phase:'failed',error:'estimated automatic calibration duration exceeds configured maximum'}});
  assert.equal(h.nodes['priority-error-overlay'].open,true);
  assert.match(h.nodes['priority-error-detail'].textContent,/exceeds configured maximum/);
  h.api.acknowledgePriorityError();
  h.api.checkRuntimeAlerts({robot_state:'ACTIVE',model_calibration:{task_id:'abc',phase:'failed',error:'estimated automatic calibration duration exceeds configured maximum'}});
  assert.equal(h.nodes['priority-error-overlay'].open,false);
});

test('native snapshot exposes effective velocity and safety error; limit resolver keeps state scale', () => {
  assert.match(cpp, /\\"safety_limits\\"/);
  assert.match(cpp, /\\"safety_code\\"/);
  assert.match(cpp, /model_calibration_fault_interrupted_\.store\(true\)/);
  assert.match(resolver, /cfg\.state_vel_fault_ratio = resolved\.joints\.front\(\)\.max_state_vel\s*\/\s*resolved\.joints\.front\(\)\.max_cmd_vel/);
});


test('native modal alert has priority over teaching overlay and hidden controls stay hidden', () => {
  const styles = fs.readFileSync(path.join(root,'apps/launcher/renderer/styles.css'),'utf8');
  assert.match(app, /<dialog id="priority-error-overlay"/);
  assert.doesNotMatch(app, /<dialog id="priority-error-overlay"[^>]*\shidden(?:\s|>)/);
  assert.match(app, /layer\.showModal\(\)/);
  assert.match(app, /calibrationOverlay\.hidden = true/);
  assert.match(app, /!priorityErrorOpen && modelCalibrationLongRunning\(status\)/);
  assert.match(styles, /\[hidden\]\{display:none!important\}/);
  assert.match(styles, /\.priority-error-overlay:not\(\[open\]\)\{display:none!important\}/);
  const h = errorHarness();
  h.api.showPriorityError('A failed calibration');
  assert.equal(h.nodes['priority-error-overlay'].open,true);
  h.api.acknowledgePriorityError();
  assert.equal(h.nodes['priority-error-overlay'].open,false);
});

test('calibration preview explicitly shows actual and recorded duration and correct replay count', () => {
  assert.match(cpp, /teaching_wall_duration_s/);
  assert.match(cpp, /trajectory_duration_s/);
  assert.match(cpp, /2\.0 \* planned_pass_s \+ static_collection_s/);
  assert.match(cpp, /calibration_retime::plan/);
  assert.match(app, /single_pass_duration_s/);
  assert.match(app, /estimated_duration_s/);
  assert.doesNotMatch(app, /model-cal-max-duration/);
  assert.match(app, /model_calibration_status', \{\}/);
});

test('failed teaching validation restores rigid hold when robot remains active', () => {
  const start = cpp.indexOf('void machine_model_calibration_teach_stop(');
  const end = cpp.indexOf('bool model_calibration_within_time_limit()', start);
  const body = cpp.slice(start, end);
  assert.ok(start > 0 && end > start);
  assert.ok((body.match(/machine_robot_state\(\) == RobotState::ACTIVE/g) || []).length >= 5);
  assert.ok((body.match(/set_tuning_impedance_mode\(JointImpedanceMode::RIGID_HOLD\)/g) || []).length >= 5);
  assert.match(body, /model_calibration_original_duration_s_/);
  assert.match(body, /model_calibration_replay_rate_ = 1\.0/);
});


test('park timeout yields actionable diagnostic but retains strict disabling interlock', () => {
  assert.match(cpp, /park timeout: position error=/);
  assert.match(cpp, /park_last_error_\.empty\(\)/);
  assert.match(cpp, /no automatic disable/);
  assert.match(cpp, /max_position_error <= cfg_\.shutdown\.position_tolerance/);
  assert.match(cpp, /if\(elapsed_s > cfg_\.shutdown\.timeout_s\)/);
});

test('retimed calibration shows whole workflow ETA and has no arbitrary GUI duration cap', () => {
  const timerHeader = fs.readFileSync(path.join(root,'src/serial_arm/core/include/serial_arm/dynamics/calibration_retime.hpp'),'utf8');
  assert.match(timerHeader, /Ramer-Douglas-Peucker/);
  assert.match(timerHeader, /Cubic Hermite interpolation/);
  assert.match(timerHeader, /max_acc/);
  assert.match(cpp, /calibration_retime::plan/);
  assert.match(cpp, /calibration_retime::interpolate/);
  assert.match(cpp, /estimated_total_duration_s/);
  assert.match(app, /estimated_total_duration_s/);
  assert.match(app, /T总 ≈/);
  assert.match(app, /totalEstimate <= 200/);
  assert.doesNotMatch(app, /model-cal-max-duration/);
  assert.doesNotMatch(cpp, /estimated automatic calibration duration exceeds configured maximum/);
});

test('INVALID_DT is described to operators in milliseconds instead of raw seconds', () => {
  const h = errorHarness();
  const text = h.api.describeRuntimeFault({fault:{code:'SAFETY_FAILED',safety_code:'INVALID_DT',value:0.0221243,limit:0.02}});
  assert.match(text,/22\.12 ms/);
  assert.match(text,/20\.00 ms/);
});
