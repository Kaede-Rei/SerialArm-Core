/* Production renderer and native parameter metadata; no hardware writes.
 * DISPLAY=:1 node_modules/.bin/electron --no-sandbox scripts/verify-workbench-target-layout.cjs
 */
const assert = require('node:assert/strict');
const fs = require('node:fs');
const os = require('node:os');
const path = require('node:path');
const {app, BrowserWindow} = require('electron');
const {buildFixture} = require('./studio-ui-fixture.cjs');

const outputIndex = process.argv.indexOf('--out');
const output = path.resolve(outputIndex < 0 ? '/tmp/serialarm-workbench-target-layout' : process.argv[outputIndex + 1]);
const fixture = buildFixture(fs.mkdtempSync(path.join(os.tmpdir(), 'serialarm-target-layout-')), {model:true});
const rendererHarness = fs.readFileSync(path.join(__dirname, 'render-studio-ui.cjs'), 'utf8');
const setSceneSource = rendererHarness.slice(rendererHarness.indexOf('function setScene('), rendererHarness.indexOf('function snapshot('));
app.setPath('userData', path.join(fixture.directory, 'electron-userdata'));
app.commandLine.appendSwitch('force-device-scale-factor', '1');

function verifyScenario({kind, theme, language}) {
    const checks = [];
    const check = (condition, name) => { if (!condition) throw Error(`${kind}/${theme}/${language}: ${name}`); checks.push(name); };
    const close = (actual, expected, tolerance = 1e-8) => Math.abs(actual - expected) <= tolerance;
    const $ = selector => document.querySelector(selector);
    const input = (node, value) => {
        state.telemetryReceivedAt = Date.now();
        node.value = String(value);
        node.dispatchEvent(new Event('input', {bubbles:true}));
    };
    window.targetLayoutScene('workbench', theme, language);
    const names = state.profileEditor.joint_names;
    const selected = 1;
    if (kind === 'mixed') {
        state.profileEditor.urdf_joint_limits[names[selected]].type = 'prismatic';
        state.capabilityProfile.urdf_joint_limits[names[selected]].type = 'prismatic';
        state.modelData.model.joints.find(joint => joint.name === names[selected]).type = 'prismatic';
    }
    state.session = {state:'running', mode:'workbench'};
    state.telemetry = {
        valid:true, write_enabled:false, feedback_age_ms:0, sequence:1, robot_state:'INACTIVE',
        joint_names:names, joint:{pos:names.map((_, index) => index === selected ? .125 : index * .1)},
        safety_limits:{min_pos:names.map(() => -2), max_pos:names.map(() => 2), pos_margin:names.map(() => .01),
            has_position_limit:names.map(() => true), max_cmd_vel:names.map((_, index) => 2 + index), max_acc:names.map(() => 4)},
        impedance_mode:'RIGID', control_path:'MIT'
    };
    state.telemetryReceivedAt = Date.now();
    state.workbenchEpoch += 1;
    state.workbenchTargetEpoch = -1;
    state.workbenchTargets = [];
    state.workbenchFineJoint = 0;
    renderedPage = ''; renderedPageViewKey = ''; render();
    state.telemetryReceivedAt = Date.now(); refreshWorkbenchTargetFields();

    const rows = [...document.querySelectorAll('.control-joint-table tbody tr')];
    check(rows.length === names.length, 'There is exactly one target row per joint');
    check(rows.every((row, index) => row.querySelector(`#target-actual-${index}`)
        && row.querySelector(`[data-target-joint="${index}"]`)
        && row.querySelector(`#target-difference-${index}`)
        && row.cells.length === 5), 'Actual, target and difference remain in the same joint row');
    check(rows.every(row => !row.querySelector('input[type="range"]')), 'Joint rows do not expand into a second editor');
    check(document.querySelectorAll('[data-target-range]').length === 1, 'Exactly one fine-adjustment slider exists');
    check([...document.querySelectorAll('[data-target-joint]')].every(node => !node.disabled), 'Fresh read-only feedback enables draft editing');
    const nativeBefore = state.workbenchTargets.slice();
    const speedBefore = state.speedScale;
    $('[data-target-select="1"]').click();
    check(JSON.stringify(state.workbenchTargets) === JSON.stringify(nativeBefore), 'Selecting a fine-adjustment joint does not edit any target');
    check(state.workbenchFineJoint === selected && $('[data-target-range]').dataset.targetRange === String(selected)
        && rows[selected].classList.contains('is-selected'), 'Joint selection updates the single editor and selected row');
    check(state.speedScale === speedBefore, 'Selecting a joint leaves whole-motion speed unchanged');

    const factor = kind === 'mixed' ? 1000 : 180 / Math.PI;
    const unit = kind === 'mixed' ? 'mm' : '°';
    check(rows[selected].lastElementChild.textContent === unit, 'The selected joint shows its actual display unit');
    check(close(Number($(`#target-actual-${selected}`).textContent), .125 * factor, .00051), 'Actual position converts from native units for display');
    const targetInput = $(`[data-target-joint="${selected}"]`);
    const typedValue = kind === 'mixed' ? 135 : 90;
    targetInput.focus(); input(targetInput, typedValue);
    check(close(state.workbenchTargets[selected], typedValue / factor), 'Numeric target input converts degrees/rad or mm/m correctly');
    check(close(Number($('[data-target-range]').value), typedValue, .011), 'Numeric target editing updates the same fine-adjustment slider');
    check(close(Number($(`#target-difference-${selected}`).textContent), typedValue - .125 * factor, .00051), 'Difference compares target and actual in the same displayed unit');
    check(state.workbenchTargets.every((value, index) => index === selected || value === nativeBefore[index]), 'Numeric editing leaves every other joint unchanged');

    const slider = $('[data-target-range]');
    const draggedValue = kind === 'mixed' ? -50 : -30;
    slider.focus(); input(slider, draggedValue);
    check(close(state.workbenchTargets[selected], Number(slider.value) / factor), 'Fine adjustment stores native joint units');
    check(close(Number(targetInput.value), Number(slider.value), .00051), 'Fine adjustment updates the corresponding numeric target');
    check(state.workbenchTargets.every((value, index) => index === selected || value === nativeBefore[index]), 'Fine adjustment leaves every other joint unchanged');
    check(state.speedScale === speedBefore, 'Joint fine adjustment does not change whole-motion speed');

    const edited = state.workbenchTargets.slice();
    const select = $('#workbench-fine-joint');
    select.value = '2'; select.dispatchEvent(new Event('change', {bubbles:true}));
    check(state.workbenchFineJoint === 2 && $('[data-target-range]').dataset.targetRange === '2'
        && JSON.stringify(state.workbenchTargets) === JSON.stringify(edited), 'The joint dropdown changes selection without changing the draft');
    const speed = $('#workbench-speed');
    const execution = speed.closest('.control-execution-panel');
    check(execution && !speed.closest('#workbench-target-editor,.control-fine-control,.workbench-target-card'), 'Whole-motion speed belongs to a separate execution section');
    check(speed.getAttribute('aria-describedby').split(' ').includes('workbench-speed-scope')
        && $('#workbench-speed-scope').textContent.includes(language === 'en' ? 'all joints' : '全部关节'), 'The speed scope is explicit and programmatically associated');
    input(speed, 35);
    check(state.speedScale === .35 && JSON.stringify(state.workbenchTargets) === JSON.stringify(edited), 'Changing speed modifies only whole-motion speed, preserving the joint draft');
    const limitRows = [...document.querySelectorAll('#workbench-motion-limits tbody tr')];
    check(limitRows.length === names.length && limitRows.every((row, index) => close(parseFloat(row.cells[1].textContent), (2 + index) * .35, .00001)), 'The speed ratio updates the reference-speed limit for every joint');
    check(limitRows.every(row => parseFloat(row.cells[2].textContent) === 4), 'Changing speed leaves the acceleration limits unchanged');
    input(speed, -1);
    check(state.speedScale === .35 && speed.getAttribute('aria-invalid') === 'true'
        && JSON.stringify(state.workbenchTargets) === JSON.stringify(edited), 'Invalid speed is reported without changing speed or target values');
    input(speed, 35);

    const limits = $('#workbench-motion-limits').closest('details.control-motion-limits');
    check(limits && execution.contains(limits) && !limits.open, 'Motion limits start collapsed inside the whole-motion section');
    const summary = limits.querySelector(':scope > summary');
    const summaryStyle = getComputedStyle(summary);
    check(parseFloat(summaryStyle.borderTopWidth) >= 1 && summaryStyle.backgroundColor !== 'rgba(0, 0, 0, 0)', 'Motion limits have a visible disclosure-button surface');
    summary.click();
    check(limits.open, 'Motion-limit disclosure opens through its real click handler');
    const bounds = $('.control-fine-bounds');
    const [lower, description, upper] = [...bounds.children].map(node => node.getBoundingClientRect());
    check(Math.abs(lower.top - upper.top) < 1 && description.top >= lower.bottom - 1
        && bounds.scrollWidth <= bounds.clientWidth + 1, 'Fine-adjustment bounds align while their explanation wraps below without overflow');
    const actionStyle = getComputedStyle(execution.querySelector('.target-execute-actions'));
    check(parseFloat(actionStyle.borderTopWidth) >= 1 && parseFloat(actionStyle.paddingTop) >= 12, 'Execution status and action are visibly separated from settings');

    state.session = {state:'idle'};
    state.telemetryReceivedAt = Date.now();
    refreshWorkbenchTargetFields();
    check([...document.querySelectorAll('[data-target-joint],[data-target-range]')].every(node => node.disabled), 'Disconnect disables every joint target and the fine-adjustment slider');
    check($('#target-move').disabled && $('#target-sync').disabled, 'Disconnected motion and synchronization actions stay disabled');
    return {kind, theme, language, checks};
}

app.whenReady().then(async () => {
    fs.mkdirSync(output, {recursive:true});
    const win = new BrowserWindow({width:1000, height:720, show:false, frame:false,
        webPreferences:{preload:fixture.preload, sandbox:false, backgroundThrottling:false}});
    const timeout = setTimeout(() => { console.error('Target-layout verification timed out'); app.exit(1); }, 60000);
    await win.loadFile(fixture.index);
    await win.webContents.executeJavaScript('document.fonts.ready.then(()=>true)');
    // No canvas is needed for target editing; leave GPU/pointer ownership to the
    // separate native-pointer and real-model verification processes.
    await win.webContents.executeJavaScript(`term?.dispose();term=null;fit=null;window.Terminal=undefined;window.studioFixtureRescue=true;window.targetLayoutScene=${setSceneSource};void 0`);
    const report = {source:'Production renderer, read-only fixture, hidden window; real input/change/click event bindings', variants:[]};
    for (const scenario of [
        {kind:'rotary', theme:'light', language:'zh-CN'},
        {kind:'mixed', theme:'light', language:'en'},
        {kind:'rotary', theme:'dark', language:'en'},
        {kind:'mixed', theme:'dark', language:'zh-CN'}
    ]) {
        const result = await win.webContents.executeJavaScript(`(()=>{try{return (${verifyScenario})(${JSON.stringify(scenario)})}catch(error){return {error:error.stack}}})()`);
        assert(!result.error, result.error);
        report.variants.push(result);
    }
    report.apiCalls = await win.webContents.executeJavaScript('serialArm.fixtureCalls()');
    assert(report.apiCalls.every(name => ['status', 'logs', 'profile_library', 'hardware_inspect'].includes(name)), 'No device/write request is allowed: ' + report.apiCalls.join(', '));
    fs.writeFileSync(path.join(output, 'report.json'), JSON.stringify(report, null, 2));
    console.log('WORKBENCH_TARGET_LAYOUT_PASS ' + JSON.stringify({variants:report.variants.length,
        checks:report.variants.reduce((total, variant) => total + variant.checks.length, 0), report:path.join(output, 'report.json')}));
    clearTimeout(timeout); win.destroy(); app.quit();
}).catch(error => {console.error(error.stack); app.exit(1);});
