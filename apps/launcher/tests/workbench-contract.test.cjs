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
  assert.match(app, /\[\s*'terminal'\s*,\s*'hardware'\s*,\s*'moveit'\s*\]\.map/);
  assert.doesNotMatch(app, /\[\s*'model'\s*,\s*'terminal'\s*,\s*'hardware'\s*,\s*'moveit'\s*\]\.map/);
  assert.match(app, /modelTitle\s*:\s*\[\s*'模型工作台'/);
  assert.match(app, /modelPageDesc\s*:\s*\[\s*'查看机器人结构、坐标系、关节运动与动力学参数'/);
});

test('model preview separates overlays, supports per-object visibility, and streams drag updates', () => {
  const app = read('apps/launcher/renderer/app.js');
  const model = read('apps/launcher/renderer/model-view.js');
  const probe = read('src/serial_arm/core/app/serial_arm_model_probe.cpp');
  assert.match(app, /linkFrames\s*:\s*true\s*,\s*jointAxes\s*:\s*true\s*,\s*com\s*:\s*true\s*,\s*inertia\s*:\s*true/);
  assert.match(app, /data-model-visible/);
  assert.match(app, /model_preview/);
  assert.match(app, /requestAnimationFrame/);
  assert.doesNotMatch(app, /setTimeout\(\(\)=>operation\(previewModel\),80\)/);
  assert.match(model, /setObjectVisible\(\s*kind\s*,\s*name\s*,\s*visible\s*\)/);
  assert.match(model, /userData\.layer\s*=\s*'linkFrames'/);
  assert.match(model, /userData\.layer\s*=\s*'jointAxes'/);
  assert.match(model, /userData\.layer\s*=\s*'com'/);
  assert.match(model, /userData\.layer\s*=\s*'inertia'/);
  assert.match(probe, /--server/);
  assert.match(probe, /while\s*\(\s*std::getline\(\s*std::cin\s*,\s*line\s*\)\s*\)/);
});


test('model visibility is strict and inspection follows URDF field names', () => {
  const app = read('apps/launcher/renderer/app.js');
  const model = read('apps/launcher/renderer/model-view.js');
  assert.match(app, /model-layers-all/);
  assert.match(app, /model-layers-none/);
  assert.match(app, /model-show-all/);
  assert.match(app, /model-hide-all/);
  assert.match(app, /state\.modelSelection\s*===\s*next\s*\?\s*''\s*:\s*next/);
  assert.match(app, /&lt;origin xyz&gt;/);
  assert.match(app, /&lt;mass value&gt;/);
  assert.match(app, /`&lt;inertia \${k}&gt;`/);
  assert.match(app, /&lt;axis xyz&gt;/);
  assert.match(model, /mouseButtons\.MIDDLE\s*=\s*THREE\.MOUSE\.PAN/);
  assert.match(model, /requiredLayers\s*\.\s*every/);
  assert.match(model, /\[\s*'com'\s*,\s*'labels'\s*\]/);
  assert.doesNotMatch(model, /frameLabels/);
});

test('link frames use independent visibility and high-contrast XYZ arrows', () => {
  const app = read('apps/launcher/renderer/app.js');
  const model = read('apps/launcher/renderer/model-view.js');
  assert.match(app, /modelVisibility\s*:\s*\{\s*links\s*:\s*\{\s*\}\s*,\s*linkFrames\s*:\s*\{\s*\}\s*,\s*joints\s*:\s*\{\s*\}\s*\}/);
  assert.match(app, /data-model-frame=/);
  assert.match(app, /setLinkFrameVisible\(\s*link\.name\s*,\s*visible\s*\)/);
  assert.match(model, /ownerKind\s*=\s*'linkFrames'/);
  assert.match(model, /objectVisibility\s*=\s*\{\s*links\s*:\s*new Map\(\)\s*,\s*linkFrames\s*:\s*new Map\(\)\s*,\s*joints\s*:\s*new Map\(\)\s*\}/);
  assert.match(app, /modelView\?\.setLinkFrameVisible\(\s*name\s*,\s*next\s*\)/);
  assert.match(model, /setLinkFrameVisible\(\s*name\s*,\s*visible\s*\)/);
  assert.match(model, /helper\.visible\s*=\s*!!this\.layers\.linkFrames\s*&&\s*next/);
  assert.match(model, /for\s*\(\s*const \[\s*name\s*,\s*helper\s*\] of this\.frameHelpers\s*\)\s*helper\.visible\s*=\s*!!this\.layers\.linkFrames/);
  assert.match(model, /for \(const link of payload\.model\?\.links \|\| \[]\)/);
  assert.match(model, /const frame = frames\.get\(link\.name\)/);
  assert.doesNotMatch(model, /for \(const frame of payload\.native\?\.state\?\.frames \|\| \[]\) this\.addFrame/);
  assert.match(model, /if \(this\.frameHelpers\.has\(frame\.name\)\) return/);
  assert.match(model, /new THREE\.ArrowHelper\(\s*direction\s*,\s*new THREE\.Vector3\(\)\s*,\s*length\s*,\s*color/);
  assert.match(model, /0xef4444/);
  assert.match(model, /0x22c55e/);
  assert.match(model, /0x3b82f6/);
  assert.match(model, /depthTest\s*=\s*false/);
});

test('control workspace shares native telemetry across control tuning calibration and diagnostics', () => {
  const app = read('apps/launcher/renderer/app.js');
  const bridge = read('apps/launcher/backend/bridge.py');
  const terminal = read('src/serial_arm/core/app/serial_arm_terminal.cpp');
  assert.match(app, /workbenchTabs/);
  assert.match(app, /controlTab/);
  assert.match(app, /tuningTab/);
  assert.match(app, /calibrationTab/);
  assert.match(app, /runtimeDiagTab/);
  assert.match(app, /event\.event\s*===\s*'telemetry'/);
  assert.match(app, /modelView\?*\.updatePose/);
  assert.match(bridge, /workbench_start/);
  assert.match(terminal, /method == "activate"/);
  assert.match(terminal, /method == "set_impedance_mode"/);
  assert.match(terminal, /method == "move_relative"/);
  assert.match(terminal, /method == "get_admittance"/);
  assert.match(terminal, /method == "set_admittance"/);
});

test('calibration machine interface reuses existing fit functions and exposes cancellable phases', () => {
  const terminal = read('src/serial_arm/core/app/serial_arm_terminal.cpp');
  assert.match(terminal, /method == "calibration_begin"/);
  assert.match(terminal, /method == "calibration_capture"/);
  assert.match(terminal, /method == "calibration_finish"/);
  assert.match(terminal, /method == "calibration_cancel"/);
  assert.match(terminal, /calibrate_admittance_static\(machine_calibration_poses_/);
  assert.match(terminal, /evaluate_admittance_static_validation\(machine_calibration_poses_/);
  assert.match(terminal, /calibrate_admittance_friction_cross_validated/);
  assert.match(terminal, /machine_friction_stop_\.load\(\)/);
  assert.match(terminal, /friction_replaying/);
});

test('runtime application and file persistence stay separate and saving is conflict checked', () => {
  const app = read('apps/launcher/renderer/app.js');
  const bridge = read('apps/launcher/backend/bridge.py');
  const persistence = read('apps/launcher/backend/persistence.py');
  assert.match(app, /applyTuning/);
  assert.match(app, /workbench_config_preview/);
  assert.match(app, /workbench_config_save/);
  assert.match(bridge, /preview_config/);
  assert.match(bridge, /save_config/);
  assert.match(persistence, /configuration changed outside the workspace/);
  assert.match(persistence, /\.tmp/);
  assert.match(persistence, /\.bak-/);
});


test('workbench model canvas is clipped to its column and cannot cover controls', () => {
  const css = read('apps/launcher/renderer/styles.css');
  const model = read('apps/launcher/renderer/model-view.js');
  assert.match(css, /\.workbench-layout\{[^}]*isolation:isolate/);
  assert.match(css, /\.workbench-model\{[^}]*overflow:hidden/);
  assert.match(css, /#workbench-viewport\{[^}]*overflow:hidden[^}]*contain:paint/);
  assert.match(css, /#workbench-viewport canvas\{[^}]*width:100%!important[^}]*height:100%!important/);
  assert.match(css, /\.workbench-task\{[^}]*z-index:3[^}]*background:var\(--card\)/);
  assert.match(model, /renderer\.domElement\.style/);
  assert.match(model, /width:'100%'/);
  assert.match(model, /height:'100%'/);
});


test('workbench backend operations use one blocking glass overlay with live confirmed state', () => {
  const app = read('apps/launcher/renderer/app.js');
  const css = read('apps/launcher/renderer/styles.css');
  const terminal = read('src/serial_arm/core/app/serial_arm_terminal.cpp');
  assert.match(app, /workbenchPending\s*:\s*\{\s*kind\s*:\s*''\s*,\s*value\s*:\s*''\s*\}\s*,\s*blockingBusy\s*:\s*false/);
  assert.match(app, /id=\"operation-overlay\" class=\"operation-overlay\"/);
  assert.match(app, /function setOperationOverlay\(\s*message\s*=\s*''\s*\)/);
  assert.match(app, /async function blockingOperation\(\s*message\s*,\s*fn\s*,\s*minimumMs\s*=\s*320\s*\)/);
  assert.match(app, /正在切换阻抗模式至/);
  assert.match(app, /正在使能/);
  assert.match(app, /正在应用导纳参数/);
  assert.match(app, /正在采集当前姿态/);
  assert.match(app, /正在保存配置/);
  assert.match(app, /classList\.toggle\(\s*'active'\s*,\s*x\.impedance_mode\s*===\s*mode\s*\)/);
  assert.match(app, /button\.disabled\s*=\s*!active\s*\|\|\s*pending\.kind\s*===\s*'impedance'/);
  assert.match(app, /button\.disabled\s*=\s*!inactive\s*\|\|\s*pending\.kind\s*===\s*'feedforward'/);
  assert.match(app, /模型前馈只能在机器人处于 INACTIVE 时修改/);
  assert.match(app, /Model feedforward can only be changed while the robot is INACTIVE/);
  assert.match(css, /\.operation-overlay\{[^}]*backdrop-filter:blur\(9px\)/);
  assert.match(css, /\.operation-progress\{/);
  assert.match(css, /\.operation-spinner\{/);
  assert.match(css, /@keyframes operation-spin/);
  assert.match(terminal, /machine_set_impedance_mode[\s\S]*emit_reply\(id, machine_snapshot_json\(\)\)/);
  assert.match(terminal, /machine_set_model_feedforward_mode[\s\S]*emit_reply\(id, machine_snapshot_json\(\)\)/);
});

test('model calibration wizard follows backend task state and keeps safety actions reachable', () => {
  const app = read('apps/launcher/renderer/app.js');
  const css = read('apps/launcher/renderer/styles.css');
  const terminal = read('src/serial_arm/core/app/serial_arm_terminal.cpp');
  assert.match(app, /model_calibration_teach_begin/);
  assert.match(app, /model_calibration_teach_stop/);
  assert.match(app, /model_calibration_start/);
  assert.match(app, /model_calibration_pause/);
  assert.match(app, /model_calibration_resume/);
  assert.match(app, /model_calibration_cancel/);
  assert.match(app, /id="model-calibration-task-stop-teach"/);
  assert.match(app, /id="model-calibration-task-hold"/);
  assert.match(app, /id="model-calibration-task-disable"/);
  assert.match(app, /phaseBoundary/);
  assert.match(css, /\.model-calibration-overlay\{[^}]*backdrop-filter:blur\(10px\)/);
  assert.match(terminal, /model_calibration_worker_=std::thread/);
  assert.match(terminal, /task_id/);
  assert.match(terminal, /model_calibration_require_task/);
  assert.match(terminal, /if\(model_calibration_phase_\.load\(\) == ModelCalibrationPhase::WAITING_REPLAY_CONFIRMATION\)\s*machine_model_calibration_cancel\(\)/);

  assert.match(terminal, /stale or missing model calibration task_id/);
  assert.match(app, /modelCalibrationTaskParams/);
  assert.match(app, /minimum_information_score/);
  assert.doesNotMatch(app, /model-cal-max-duration/);
  assert.match(terminal, /calibration_retime::plan/);
  assert.match(terminal, /model_calibration_estimated_duration_s_ = 2\.0/);
  assert.match(terminal, /resume_confirmation_required/);
});

test('model calibration records control-cycle data and supports offline recompute and candidate comparison', () => {
  const app = read('apps/launcher/renderer/app.js');
  const bridge = read('apps/launcher/backend/bridge.py');
  const terminal = read('src/serial_arm/core/app/serial_arm_terminal.cpp');
  const calibration = read('src/serial_arm/core/src/dynamics/gravity_calibration.cpp');
  const model = read('apps/launcher/renderer/model-view.js');
  assert.match(terminal, /record_model_calibration_cycle/);
  assert.match(calibration, /frames\.csv/);
  assert.match(calibration, /calibration_options/);
  assert.match(bridge, /model_calibration_recompute/);
  assert.match(bridge, /candidate = json\.loads\(Path\(output\)\.read_text\(\)\)/);
  assert.match(app, /setGravityComparison/);
  assert.match(model, /setGravityComparison\(result(?:, options = \{\})?\)/);
  assert.match(model, /candidatePoint/);
  assert.match(model, /originalPoint/);
  assert.match(app, /modelCalibrationModelBinding/);
  assert.match(app, /模型指纹变化，正在核对历史快照与当前模型的兼容性/);
  assert.match(app, /model_calibration_model_alignment/);
  const probe = read('src/serial_arm/core/app/serial_arm_model_probe.cpp');
  assert.match(probe, /urdf_fingerprint/);
  assert.match(probe, /model_calibration_file_fingerprint/);
});

test('candidate persistence is fingerprint checked reversible and keeps full dynamics out of scope', () => {
  const bridge = read('apps/launcher/backend/bridge.py');
  const persistence = read('apps/launcher/backend/persistence.py');
  const terminal = read('src/serial_arm/core/app/serial_arm_terminal.cpp');
  assert.match(bridge, /model_calibration_preview_save/);
  assert.match(bridge, /model_calibration_restore_config/);
  assert.match(bridge, /model_calibration_export_urdf/);
  assert.match(persistence, /verified_source_urdf/);
  assert.match(persistence, /matches_recorded_fingerprint/);
  assert.match(persistence, /application\.json/);
  assert.match(persistence, /restore\.json/);
  assert.match(persistence, /candidate-urdf-verification\.json/);
  assert.match(persistence, /mass_and_inertia_values_preserved/);
  assert.match(terminal, /candidate gravity is not compatible with FULL_INVERSE_DYNAMICS/);
  assert.match(terminal, /candidate gravity is not compatible with FULL_ID observer mode/);
});


test('workbench copy follows selected language and non-GUI calibration keeps feature parity', () => {
  const app = read('apps/launcher/renderer/app.js');
  const terminal = read('src/serial_arm/core/app/serial_arm_terminal.cpp');
  const cli = read('tools/model_calibration_cli.py');
  const docs = read('docs/launcher.md');

  assert.match(app, /poseBudget:\s*\['姿态数量上限',\s*'Pose budget'\]/);
  assert.match(app, /validationFraction:\s*\['留出验证比例',\s*'Validation fraction'\]/);
  assert.match(app, /singleCalibrationTools:\s*\['单项标定工具',\s*'Individual calibration tools'\]/);
  assert.match(app, /function commandStateText/);
  assert.match(app, /function calibrationTaskKindText/);
  assert.match(app, /function calibrationTaskPhaseText/);
  assert.doesNotMatch(app, /原有单项标定工具|OFFLINE RECORD|LIVE SESSION|legacy-calibration/);

  assert.match(terminal, /void run_model_calibration_menu\(\)/);
  assert.match(terminal, /machine_model_calibration_teach_begin/);
  assert.match(terminal, /machine_model_calibration_teach_stop/);
  assert.match(terminal, /machine_model_calibration_pause/);
  assert.match(terminal, /machine_model_calibration_resume/);
  assert.match(terminal, /machine_model_calibration_cancel_request/);
  assert.match(terminal, /machine_model_calibration_apply/);
  assert.match(terminal, /machine_model_calibration_restore/);
  assert.match(cli, /preview-save/);
  assert.match(cli, /recompute/);
  assert.match(cli, /export-urdf/);
  assert.match(cli, /restore-config/);
  assert.match(docs, /非 GUI 使用模型校正/);

  for (const content of [app, terminal, docs]) {
    assert.doesNotMatch(content, /\bv?0\.5\.\d+\b/);
  }
});

test('launcher starts from lifecycle entrances and Profile Library only manages references', () => {
  const app = read('apps/launcher/renderer/app.js');
  const backend = read('apps/launcher/backend/profile_library.py');
  const main = read('apps/launcher/desktop/main.cjs');
  const architecture = read('docs/architecture.md');
  const cli = read('tools/profile_workspace_cli.py');

  assert.match(app, /page:\s*'start'/);
  assert.match(app, /option\('description','newDescription'/);
  assert.match(app, /option\('continue','continueProfile'/);
  assert.match(app, /option\('use','useProfile'/);
  assert.match(app, /profile_library_register/);
  assert.match(app, /description_create_profile/);
  assert.match(app, /readiness_mark/);
  assert.match(app, /description-browse-dir/);
  assert.match(main, /launcher:select-description/);
  assert.match(main, /launcher:select-profile-package/);
  assert.match(backend, /profile-library\.yaml/);
  assert.match(backend, /def create_profile/);
  assert.match(backend, /'write_enabled':False/);
  assert.match(backend, /def remove\(self, entry_id\)/);
  assert.doesNotMatch(backend, /def remove\(self, entry_id\)[\s\S]{0,600}rmtree/);
  assert.match(architecture, /Profile Library/);
  assert.match(architecture, /Description[\s\S]*Profile Setup[\s\S]*Model Check[\s\S]*Hardware Bring-up/);
  assert.match(cli, /inspect-description/);
  assert.match(cli, /create-profile/);
  assert.match(cli, /import-profile/);
  assert.match(cli, /remove-profile/);
});

test('launcher navigation and primary interactions use visible spring motion without blocking Profile selection', () => {
  const app = read('apps/launcher/renderer/app.js');
  const css = read('apps/launcher/renderer/styles.css');
  const backend = read('apps/launcher/backend/profile_library.py');

  assert.match(app, /state\.page === 'settings'[\s\S]{0,220}state\.previousPage !== 'settings'/);
  assert.match(app, /const found = await blockingOperation\([\s\S]{0,180}profile_source_inspect/);
  assert.doesNotMatch(app, /正在检查并导入 Profile[\s\S]{0,160}importProfileSource/);
  assert.match(backend, /def _builtin_readiness\(/);
  assert.match(backend, /'state':'ready'/);
  assert.match(backend, /ignored=\{'.git','build','install','log','logs','node_modules'/);

  assert.match(css, /@keyframes page-spring-in/);
  assert.match(css, /@keyframes start-choice-spring/);
  assert.match(css, /@keyframes operation-window-spring/);
  assert.match(css, /@keyframes dialog-spring-in/);
  assert.match(css, /\.button:not\(:disabled\):active\{[^}]*scale\(\.935\)/);
  assert.match(css, /\.start-choice:hover\{[^}]*translateY\(-7px\)[^}]*scale\(1\.025\)/);
  assert.match(css, /@media\(prefers-reduced-motion:reduce\)[\s\S]*animation:none!important/);
});
