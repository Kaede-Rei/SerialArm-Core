const test = require('node:test');
const assert = require('node:assert/strict');
const fs = require('node:fs');
const path = require('node:path');
const root = path.resolve(__dirname, '../../..');
const src = fs.readFileSync(path.join(root, 'src/serial_arm/core/app/serial_arm_terminal.cpp'), 'utf8');
const profiles = fs.readFileSync(path.join(root, 'apps/launcher/backend/profiles.py'), 'utf8');
const summary = fs.readFileSync(path.join(root, 'apps/launcher/backend/persistence.py'), 'utf8');

test('fault during demonstration stops sampling without discarding already captured valid cycles', () => {
  const collect = src.slice(src.indexOf('std::optional<FrictionCalibrationTrajectory> collect_friction_drag_trajectory_until_stop('), src.indexOf('FrictionCalibrationTrajectory smooth_friction_trajectory('));
  assert.match(collect, /if\(robot_\.get_state\(\) != RobotState::ACTIVE\) break;/);
  assert.doesNotMatch(collect, /if\(robot_\.get_state\(\) != RobotState::ACTIVE\) return std::nullopt;/);
});
test('fault saves partial trajectory as file only, with no motion/fault clear', () => {
  const begin = src.slice(src.indexOf('void machine_model_calibration_teach_begin('), src.indexOf('std::vector<ModelCalibrationPoseTarget> model_calibration_select_pose_targets('));
  assert.match(begin, /if\(machine_robot_state\(\) == RobotState::FAULT\)/);
  assert.match(begin, /model_calibration_recorder_\.write_trajectory/);
  assert.match(begin, /interrupted\.txt/);
  assert.doesNotMatch(begin, /robot_\.clear_fault\(/);
});
test('fault in waiting confirmation is terminal and releases recorder', () => {
  const report = src.slice(src.indexOf('void report_background_fault('), src.indexOf('void print_banner('));
  assert.match(report, /calibration_phase == ModelCalibrationPhase::WAITING_REPLAY_CONFIRMATION/);
  assert.match(report, /model_calibration_recorder_\.stop\(\)/);
});
test('interrupted record is distinguished from completed and local terminal takes precedence over PATH', () => {
  assert.match(summary, /\(folder \/ 'interrupted\.txt'\)\.is_file\(\)/);
  const resolution = profiles.slice(profiles.indexOf('def machine_terminal('), profiles.indexOf('def model_calibrator('));
  assert.ok(resolution.indexOf('.install/standalone/bin') < resolution.indexOf("shutil.which('serial_arm_terminal')"));
});
