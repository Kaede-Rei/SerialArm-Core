const test = require('node:test'), assert = require('node:assert/strict');
const fs = require('node:fs'), path = require('node:path'), vm = require('node:vm');
const source = fs.readFileSync(path.join(__dirname, '../renderer/app.js'), 'utf8');
function ui(names, state = {}, overrides = {}) {
    const context = {state, workbenchConnected: () => true, workbenchRecoveryRequired: () => false,
        modelCalibrationActive: () => false, referenceBehavior: () => 'NATIVE_IMPEDANCE',
        currentAdmittanceProfile: () => ({enabled: true}), esc: value => String(value ?? ''),
        t: value => value, busyText: value => value, gainPurposeLabel: value => value,
        document: {querySelector: () => null, querySelectorAll: () => []}, $: () => null,
        ...overrides};
    vm.createContext(context);
    for (const name of names) {
        const body = source.match(new RegExp('(?:async )?function ' + name + '\\([^]*?\\n\\}'));
        assert.ok(body, name); vm.runInContext(body[0], context);
    }
    return context;
}
const plain = value => JSON.parse(JSON.stringify(value));
test('playback carries its own explicit strategy and gain use rather than inheriting an old mode', () => {
    const state = {teachReplayBehavior: 'NATIVE_IMPEDANCE', teachGainPurpose: 'SERVO_TRACKING',
        geometryReplayBehavior: 'NATIVE_IMPEDANCE', geometryGainPurpose: 'COMPLIANT'};
    const context = ui(['replaySelection'], state);
    assert.deepEqual(plain(context.replaySelection('teach')), {mode: 'RIGID_TRACKING', reference_behavior: 'NATIVE_IMPEDANCE', gain_profile: 'SERVO_TRACKING'});
    assert.deepEqual(plain(context.replaySelection('geometry')), {mode: 'COMPLIANT_TRACKING', reference_behavior: 'NATIVE_IMPEDANCE', gain_profile: 'COMPLIANT'});
    state.teachReplayBehavior = 'MODEL_ADMITTANCE'; state.teachGainPurpose = 'RIGID';
    assert.equal(context.replaySelection('teach').gain_profile, 'SERVO_TRACKING');
    assert.equal(context.replaySelection('geometry').gain_profile, 'COMPLIANT');
    delete state.teachReplayBehavior; context.referenceBehavior = () => 'MODEL_ADMITTANCE';
    assert.equal(context.replaySelection('teach').reference_behavior, 'MODEL_ADMITTANCE');
});
test('four gain purposes map onto separate hold, movement and drag tasks', () => {
    const state = {telemetry: {impedance_mode: 'RIGID_HOLD'}};
    const context = ui(['gainPurposeCommand'], state);
    assert.deepEqual(plain(context.gainPurposeCommand('SERVO_TRACKING')), {mode: 'RIGID_HOLD', gain_profile: 'SERVO_TRACKING'});
    assert.equal(context.gainPurposeCommand('COMPLIANT').mode, 'COMPLIANT_HOLD');
    state.telemetry.impedance_mode = 'RIGID_TRACKING';
    assert.equal(context.gainPurposeCommand('COMPLIANT').mode, 'COMPLIANT_TRACKING');
    assert.equal(context.gainPurposeCommand('DRAG').mode, 'COMPLIANT_DRAG');
    assert.throws(() => context.gainPurposeCommand('identification'));
});
test('model interaction exposes servo use and blocks contradictory native gain selections', () => {
    const state = {telemetry: {robot_state: 'ACTIVE', impedance_mode: 'RIGID_HOLD'},
        readiness: {modes: {RIGID_HOLD: 'configured', COMPLIANT_HOLD: 'configured', COMPLIANT_DRAG: 'configured', SERVO_TRACKING: 'configured'}},
        admittance: {execution_gains: {servo_tracking: {kp: [30], kd: [1]}}}};
    const context = ui(['gainPurposeConfigured', 'controlGainPurposeMayChange'], state, {referenceBehavior: () => 'MODEL_ADMITTANCE'});
    assert.equal(context.controlGainPurposeMayChange('RIGID'), false);
    assert.equal(context.controlGainPurposeMayChange('COMPLIANT'), false);
    assert.equal(context.controlGainPurposeMayChange('SERVO_TRACKING'), true);
    assert.equal(context.controlGainPurposeMayChange('DRAG'), true);
    state.telemetry.teach_replay = {phase: 'playing'};
    assert.equal(context.controlGainPurposeMayChange('SERVO_TRACKING'), false);
    state.telemetry.teach_replay.phase = 'ready'; state.telemetry.robot_state = 'FAULT';
    assert.equal(context.controlGainPurposeMayChange('SERVO_TRACKING'), false);
});
test('reverse and return previews use the endpoint required by native playback admission', () => {
    const state = {teachSelectedId: 'path', teachTrajectories: [{id: 'path', start: [0, 1], end: [2, 3]}], teachDirection: 'forward'};
    const context = ui(['replayStartPose'], state);
    for (const direction of ['forward', 'cycle']) assert.deepEqual(plain(context.replayStartPose(direction)), [0, 1]);
    for (const direction of ['reverse', 'return']) assert.deepEqual(plain(context.replayStartPose(direction)), [2, 3]);
    state.teachTrajectories = []; state.telemetry = {teach_replay: {start: [4], end: [5]}};
    assert.deepEqual(plain(context.replayStartPose('return')), [5]);
});
test('changing playback direction requests a fresh ghost without a hardware command', () => {
    const nodes = {'teach-direction': {value: 'reverse'}, 'teach-rate': {value: '.3'}, 'teach-repeats': {value: '2'}};
    const state = {page: 'workbench', workbenchTab: 'replay', teachPreviewDirection: 'return'};
    let previews = 0, commands = 0;
    const context = ui(['wireTeachReplay'], state, {$: id => nodes[id], wireReplaySelection: () => {},
        updateStartGhost: force => {assert.equal(force, true); previews++;}, workbenchRequest: () => commands++});
    context.wireTeachReplay(); nodes['teach-direction'].onchange();
    assert.equal(state.teachDirection, 'reverse'); assert.equal(state.teachPreviewDirection, null);
    assert.equal(previews, 1); assert.equal(commands, 0);
});
test('faulted playback requires retained trajectory evidence to offer an explicit retry', () => {
    const context = ui(['replayTrajectoryAvailable']);
    assert.equal(context.replayTrajectoryAvailable({phase: 'failed', has_trajectory: true}), true);
    assert.equal(context.replayTrajectoryAvailable({phase: 'failed', has_trajectory: false}), false);
    assert.equal(context.replayTrajectoryAvailable({phase: 'recording', has_trajectory: true}), false);
    assert.equal(context.replayTrajectoryAvailable({phase: 'ready'}), true);
});
test('playback settings unlock after completion while model servo selection stays fixed', () => {
    const nodes = Object.fromEntries(['teach-reference-behavior', 'teach-gain-purpose', 'teach-direction', 'teach-rate', 'teach-repeats'].map(id => [id, {}]));
    const state = {teachReplayBehavior: 'NATIVE_IMPEDANCE', teachGainPurpose: 'RIGID'};
    const context = ui(['replaySelection', 'refreshReplaySelection'], state, {$: id => nodes[id]});
    context.refreshReplaySelection('teach', {phase: 'playing'});
    assert.ok(Object.values(nodes).every(node => node.disabled));
    context.refreshReplaySelection('teach', {phase: 'ready'});
    assert.ok(Object.values(nodes).every(node => !node.disabled));
    state.teachReplayBehavior = 'MODEL_ADMITTANCE'; context.refreshReplaySelection('teach', {phase: 'failed'});
    assert.equal(nodes['teach-reference-behavior'].disabled, false); assert.equal(nodes['teach-gain-purpose'].disabled, true);
});
test('telemetry updates enable and park without forcing users to change tabs', () => {
    const state = {page: 'workbench', workbenchTab: 'replay', teachTrajectories: [], telemetry: {robot_state: 'INACTIVE', teach_replay: {phase: 'idle'}}};
    const nodes = Object.fromEntries(['teach-enable', 'teach-park', 'teach-record', 'teach-record-stop', 'teach-save', 'teach-play', 'teach-return', 'teach-stop'].map(id => [id, {}]));
    const empty = () => {};
    const context = ui(['replayTrajectoryAvailable', 'taskStateLabel', 'updateWorkbenchLive'], state,
        {$: id => nodes[id], refreshReplaySelection: empty, refreshCapabilityDiagnostics: empty,
         refreshWorkbenchTargetFields: empty, updateWorkbenchControlState: empty,
         refreshCalibrationQuickControls: empty, refreshCalibrationActionControls: empty,
         refreshGainApplyControls: empty, updateWorkbenchModel: empty, updateStartGhost: empty,
         updateModelCalibrationLive: empty, updateModelCalibrationTaskOverlay: empty, refreshWorkspaceTelemetryRows: empty});
    context.updateWorkbenchLive(); assert.equal(nodes['teach-enable'].disabled, false); assert.equal(nodes['teach-park'].disabled, true);
    state.telemetry.robot_state = 'ACTIVE'; context.updateWorkbenchLive();
    assert.equal(nodes['teach-enable'].disabled, true); assert.equal(nodes['teach-park'].disabled, false);
    state.telemetry.teach_replay.phase = 'playing'; context.updateWorkbenchLive();
    assert.equal(nodes['teach-park'].disabled, true); assert.equal(nodes['teach-stop'].disabled, false);
});
test('geometry refresh preserves expanded withdrawal options, input focus and outer scroll', () => {
    const oldDetail = {id: 'geometry-withdraw-options', open: true}, newDetail = {id: 'geometry-withdraw-options', open: false};
    const scroll = {scrollTop: 475}; let generation = 0, focused = false;
    const input = {id: 'geometry-rate', value: '.27', checked: false, selectionStart: null, selectionEnd: null};
    const nextInput = {id: input.id, type: 'number', tagName: 'INPUT', value: '.2', focus: options => {assert.equal(options.preventScroll, true); focused = true;}};
    const panel = {closest: selector => {assert.equal(selector, '.content'); return scroll;},
        querySelectorAll: () => [generation ? newDetail : oldDetail], contains: node => node === input,
        set innerHTML(value) {assert.equal(value, 'new geometry'); generation++; scroll.scrollTop = 0;}};
    const context = ui(['refreshGeometryPanel'], {page: 'geometry'}, {document: {querySelector: () => panel, activeElement: input},
        $: id => id === input.id ? nextInput : null, geometryPanelHtml: () => 'new geometry',
        wireGeometry: () => {}, wireCalibrationQuickControls: () => {}, refreshCalibrationQuickControls: () => {}, window: {}});
    context.refreshGeometryPanel();
    assert.equal(newDetail.open, true); assert.equal(scroll.scrollTop, 475); assert.equal(nextInput.value, '.27'); assert.equal(focused, true);
});
test('interaction tuning has no strategy selector, persistence controls or monitoring table', () => {
    const state = {language: 'zh-CN'};
    const context = ui(['tuningPanelHtml'], state, {currentAdmittanceProfile: () => null, workbenchConnected: () => false,
        admittanceMayTune: () => false, admittanceCapabilityState: () => ({key: 'ready', text: 'ready'}),
        admittanceUpdateText: () => '', responsePanelHtml: () => '<section>PASSIVE_RESPONSE</section>',
        referenceBehaviorHtml: () => {throw Error('duplicated strategy selector');},
        runtimeDefaultsHtml: () => {throw Error('misplaced saved defaults');}, capabilityDiagnosticsHtml: () => {throw Error('misplaced monitoring');}});
    const html = context.tuningPanelHtml(); assert.match(html, /PASSIVE_RESPONSE/);
    assert.doesNotMatch(html, /data-reference-behavior|preview-config|capability-reference-body/);
});
test('runtime default import refuses to overwrite unsaved parameter form edits', () => {
    const state = {language: 'zh-CN', profileDirty: true, savePreview: {changed: true}};
    const context = ui(['runtimeDefaultsHtml'], state, {runtimeSavePreviewHtml: () => '', runtimeExecutionSettingsHtml: () => ''});
    const html = context.runtimeDefaultsHtml();
    assert.match(html, /id="preview-config"[^>]*disabled/); assert.match(html, /id="save-config"[^>]*disabled/);
});
test('a completed gain load cannot change a new workspace through success, failure or cleanup', async () => {
    for (const failed of [false, true]) {
        let resolve, reject;
        const pending = new Promise((yes, no) => {resolve = yes; reject = no;});
        const state = {workbenchEpoch: 1, calibrationToolSelected: {}, page: 'calibration', calibrationEntry: 'gain',
            telemetry: {model_calibration: {workflow_kind: 'gain', phase: 'complete', directory: 'old-task'}}};
        const context = ui(['syncCompletedGainCandidate'], state, {api: {request: () => pending},
            refreshCalibrationOrDefer: () => {throw Error('stale refresh');}});
        context.syncCompletedGainCandidate();
        state.workbenchEpoch = 2; state.calibrationToolSelected.gain = {directory: 'new-task'};
        state.gainCandidateLoadPending = 'new-task'; state.gainCandidateLoadError = 'new error';
        state.gainCandidateLoadAttempted = 'new-task';
        if (failed) reject(Error('old error')); else resolve({directory: 'old-task'});
        await new Promise(setImmediate);
        assert.equal(state.calibrationToolSelected.gain.directory, 'new-task');
        assert.equal(state.gainCandidateLoadPending, 'new-task');
        assert.equal(state.gainCandidateLoadAttempted, 'new-task');
        assert.equal(state.gainCandidateLoadError, 'new error');
    }
});
test('late model mounts preserve the newer view and failed mounts allow a clean retry', async () => {
    const views = []; let host = {}, fail = false, notices = 0;
    class View {
        constructor(container) {this.container = container; this.disposals = 0; views.push(this);}
        setModel(payload) {this.payload = payload; return new Promise((resolve, reject) => {this.finish = () => fail ? reject(Error('mesh unavailable')) : resolve();});}
        dispose() {this.disposals++;}
        setLayers() {}
    }
    const state = {page: 'calibration', modelData: {id: 1}};
    const context = ui(['mountWorkbenchView'], state, {workspaceMount: null, modelView: null,
        modelModule: {SerialArmModelView: View}, $: () => host, api: {modelResource: () => ''},
        updateWorkbenchModel: () => {}, updateModelCalibrationComparison: () => {}, updateStartGhost: () => {},
        toast: () => notices++});
    const old = context.mountWorkbenchView();
    host = {}; state.modelData = {id: 2};
    const newer = context.mountWorkbenchView();
    views[1].finish(); await newer; views[0].finish(); await old;
    assert.equal(context.modelView, views[1]); assert.equal(views[1].disposals, 0);
    assert.ok(views[0].disposals >= 1); assert.equal(context.workspaceMount, null);
    host = {}; state.modelData = {id: 3}; fail = true;
    const rejected = context.mountWorkbenchView(); views[2].finish(); await rejected;
    assert.equal(context.modelView, null); assert.equal(context.workspaceMount, null); assert.equal(notices, 1);
    fail = false; const retried = context.mountWorkbenchView(); views[3].finish(); await retried;
    assert.equal(context.modelView, views[3]); assert.equal(views[3].disposals, 0);
});
