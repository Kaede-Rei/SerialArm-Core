const test = require('node:test');
const assert = require('node:assert/strict');
const fs = require('node:fs'), path = require('node:path'), vm = require('node:vm');
const response = require('../renderer/capability-response.js');
const source = fs.readFileSync(path.join(__dirname, '../renderer/app.js'), 'utf8');
const near = (a, b) => assert.ok(Math.abs(a - b) < 1e-10, `${a} != ${b}`);
const point = (time, position, nominal = 0) => ({time_s: time, pos: [position], nominal: [nominal], virtual: [position], accepted: [position - 0.01], external: [0.2], observer_valid: true, reference_limited: false, torque_limited: false});
function functions(names, state) {
    const sandbox = { state, workbenchConnected: () => true, workbenchRecoveryRequired: () => false, modelCalibrationActive: () => false,
        currentAdmittanceProfile: () => ({ enabled: true }), SerialArmResponse: response, busyText: value => value };
    vm.createContext(sandbox);
    for (const name of names) {
        const match = source.match(new RegExp('(?:async )?function ' + name + '\\([^]*?\\n\\}'));
        assert.ok(match, name); vm.runInContext(match[0], sandbox);
    }
    return sandbox;
}
test('native timestamp and observer validity are required; missing values stay unknown', () => {
    assert.equal(response.sample({valid: true, joint_names: ['a']}), null);
    const sample = response.sample({valid: true, monotonic_ns: 2500000000, joint_names: ['a'], observer_valid: false,
        reference_limited: [0], torque_limited: [1], joint: {pos: [0], tau_ext_hat: [100]}});
    near(sample.time_s, 2.5); assert.deepEqual(sample.external, [null]); assert.deepEqual(sample.accepted, [null]);
    assert.equal(sample.reference_limited, false); assert.equal(sample.torque_limited, true);
});
test('manual release measures overshoot, fixed-reference settling and tracking independently', () => {
    const samples = [0, 0.1, 0.2, 0.1, -0.05, 0, 0, 0, 0].map((position, i) => point(i * 0.25, position));
    samples[0].reference_limited = true; samples[1].torque_limited = true;
    const summary = response.summarize(samples, 0.5, 0.01), joint = summary.joints[0];
    near(joint.tracking_rms, 0.01); near(joint.tracking_max, 0.01); near(joint.rebound, 0.05);
    near(joint.peak_deflection, 0.2); near(joint.settling_s, 0.75); near(joint.final_offset, 0);
    near(summary.reference_limited_ratio, 1 / 9); near(summary.torque_limited_ratio, 1 / 9);
});
test('drift stays visible, and a moving nominal reference cannot earn return-time credit', () => {
    const drift = response.summarize([point(0, 0.1), point(0.5, 0.2), point(1, 0.3)], 0, 0.01).joints[0];
    assert.equal(drift.settling_s, null); near(drift.post_release_change, 0.2);
    const moving = response.summarize([point(0, 0), point(0.5, 1, 1), point(1, 2, 2)], 0, 0.01).joints[0];
    assert.equal(moving.nominal_moving, true); assert.equal(moving.settling_s, null);
    assert.equal(response.summarize([point(0, 0)], null).joints[0].final_offset, null);
});
test('unknown accepted references do not become zero-error scores', () => {
    const sample = point(0, 0); sample.accepted = [null]; sample.nominal = [null]; sample.external = [null];
    const result = response.summarize([sample], null).joints[0];
    assert.equal(result.tracking_rms, null); assert.equal(result.peak_deflection, null); assert.equal(result.peak_external_nm, null);
});
test('explicit strategy gates tuning and strategy changes require INACTIVE', () => {
    const state = { telemetry: {robot_state: 'ACTIVE', valid: true, feedback_age_ms: 2, reference_behavior: 'NATIVE_IMPEDANCE'}, admittance: {enabled: true} };
    const ui = functions(['referenceBehavior', 'referenceBehaviorMayChange', 'admittanceMayTune'], state);
    assert.equal(ui.admittanceMayTune(), false); assert.equal(ui.referenceBehaviorMayChange(), false);
    state.telemetry.reference_behavior = 'MODEL_ADMITTANCE'; assert.equal(ui.admittanceMayTune(), true);
    state.telemetry.robot_state = 'INACTIVE'; state.telemetry.valid = false;
    assert.equal(ui.admittanceMayTune(), true); assert.equal(ui.referenceBehaviorMayChange(), true);
    state.telemetry.teach_replay = {phase: 'recording'}; assert.equal(ui.referenceBehaviorMayChange(), false);
});
test('a recording rejects repeated cycles and stops when experimental conditions change', () => {
    const state = {session: {session_id: 'one'}, admittance: {reference_behavior: 'MODEL_ADMITTANCE'}, telemetry: {robot_state: 'ACTIVE', reference_behavior: 'MODEL_ADMITTANCE', valid: true, monotonic_ns: 1e9, feedback_age_ms: 2},
        responseCapture: {recording: true, session_id: 'one', behavior: 'MODEL_ADMITTANCE', impedance_mode: 'RIGID_HOLD', model_feedforward_mode: 'GRAVITY', joint_names: ['a'], samples: []}};
    const ui = functions(['referenceBehavior', 'responseMayRecord', 'recordResponseSnapshot'], state);
    const snapshot = {...state.telemetry, sequence: 1, reference_behavior: 'MODEL_ADMITTANCE', impedance_mode: 'RIGID_HOLD', model_feedforward_mode: 'GRAVITY', joint_names: ['a'], joint: {pos: [0]}};
    ui.recordResponseSnapshot(snapshot); ui.recordResponseSnapshot(snapshot);
    assert.equal(state.responseCapture.samples.length, 1);
    ui.recordResponseSnapshot({...snapshot, impedance_mode: 'COMPLIANT_HOLD'});
    assert.equal(state.responseCapture.recording, false); assert.equal(state.responseCapture.samples.length, 1);
});
test('selected and effective behavior are distinct while admittance is suspended', () => {
    const state = { admittance: {reference_behavior: 'MODEL_ADMITTANCE'},
        telemetry: {robot_state: 'ACTIVE', valid: true, monotonic_ns: 1e9, feedback_age_ms: 2, reference_behavior: 'NATIVE_IMPEDANCE'} };
    const ui = functions(['referenceBehavior', 'responseMayRecord'], state);
    assert.equal(ui.referenceBehavior(), 'MODEL_ADMITTANCE');
    assert.equal(ui.responseMayRecord(), false);
});
test('reference-budget faults retain a distinct actionable cause', () => {
    const ui = functions(['describeRuntimeFault'], {language: 'zh-CN'});
    const infeasible = ui.describeRuntimeFault({fault: {code: 'REFERENCE_GOVERNOR_FAILED', reference_governor_error: 'INFEASIBLE'}});
    assert.match(infeasible.message, /总力矩预算.*找不到可执行/);
    assert.equal(infeasible.category, 'safety'); assert.match(infeasible.technical, /INFEASIBLE/);
    const invalid = ui.describeRuntimeFault({fault: {code: 'REFERENCE_GOVERNOR_FAILED', reference_governor_error: 'INVALID_INPUT'}});
    assert.match(invalid.message, /输入/); assert.doesNotMatch(invalid.message, /模型计算失败/);
});
test('a tuning confirmation cannot submit into a replacement native session', async () => {
    const state = {language: 'en', workbenchEpoch: 1, admittance: {enabled: true, reference_behavior: 'MODEL_ADMITTANCE', mass: [1], damping: [1], stiffness: [1]},
        tuneDraft: {mass: [1], damping: [0], stiffness: [0]}, telemetry: {robot_state: 'ACTIVE', valid: true, feedback_age_ms: 1, joint_names: ['a']}};
    const ui = functions(['referenceBehavior', 'admittanceMayTune', 'applyTuning'], state);
    let writes = 0;
    ui.safetyConfirmation = async () => { state.workbenchEpoch++; return true; };
    ui.workbenchRequest = async () => { writes++; };
    await ui.applyTuning(); assert.equal(writes, 0);
});
