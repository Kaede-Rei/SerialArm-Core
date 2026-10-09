const { test } = require('node:test');
const assert = require('node:assert/strict');
const fs = require('node:fs');
const path = require('node:path');
const root = path.resolve(__dirname, '../../..');
const src = file => fs.readFileSync(path.join(root, file), 'utf8');

test('DT watchdog faults after three consecutive overruns and is reset on recovery', () => {
  const guard = src('src/serial_arm/core/include/serial_arm/core/dt_overrun_watchdog.hpp');
  const robot = src('src/serial_arm/core/src/robot.cpp');
  assert.match(guard, /kConsecutiveLimit = 3/);
  assert.match(guard, /consecutive_ = 0/);
  assert.match(robot, /dt_overrun_watchdog_\.fault_required\(dt, cfg_\.safety\.max_dt_s\)/);
  assert.match(robot, /dt_overrun_watchdog_\.reset\(\)/);
  assert.match(robot, /safety_\.check_joint_cmd\(joint_state\.value\(\), joint_cmd, controller_dt\)/);
});

test('Teaching checkpoints are atomic, fault retains valid samples and never triggers motion', () => {
  const cpp = src('src/serial_arm/core/src/dynamics/gravity_calibration.cpp');
  const terminal = src('src/serial_arm/core/app/serial_arm_terminal.cpp');
  assert.match(cpp, /trajectory\.checkpoint\.csv/);
  assert.match(cpp, /std::filesystem::rename\(temp, destination/);
  assert.match(terminal, /model_calibration_recorder_\.write_trajectory_checkpoint/);
  assert.match(terminal, /if\(robot_\.get_state\(\) != RobotState::ACTIVE\) break/);
});

test('Tomato park uses configurable faster acceleration-bounded tracking and earlier final hold', () => {
  const config = src('src/robot_supports/robots/tomato_picker/tomato_picker_description/config/serialarm/core.yaml');
  const terminal = src('src/serial_arm/core/app/serial_arm_terminal.cpp');
  assert.match(config, /speed_scale: 0\.10\s/);
  assert.match(terminal, /rigid_finish_switch_s = 0\.40 \* cfg_\.shutdown\.timeout_s/);
  assert.match(terminal, /cfg_\.safety\.limits\.max_vel\[i\] \* stream_\.speed_scale/);
  assert.match(terminal, /const double max_acc = cfg_\.safety\.limits\.max_acc\[i\]/);
  assert.match(terminal, /park timeout: fresh joint feedback unavailable/);
});

test('Record load and restore reread disk instead of relying on stale UI state', () => {
  const app = src('apps/launcher/renderer/app.js');
  const bridge = src('apps/launcher/backend/bridge.py');
  const main = src('apps/launcher/desktop/main.cjs');
  assert.match(app, /api\.request\('model_calibration_load', \{directory: selected\.directory\}\)/);
  assert.match(app, /model-calibration-refresh-record/);
  assert.match(bridge, /method == 'model_calibration_records'/);
  assert.match(main, /'model_calibration_records'/);
});

test('Fault telemetry never discards a user-selected historical calibration record', () => {
  const app = src('apps/launcher/renderer/app.js');
  const start = app.indexOf("if (event.event === 'telemetry')");
  const segment = app.slice(start, app.indexOf("if (event.event === 'session')", start));
  assert.match(segment, /if \(calibration\?\.directory && !state\.modelCalibrationOffline\) state\.modelCalibrationDirectory/);
  assert.doesNotMatch(segment, /state\.modelCalibrationOffline = null/);
});
