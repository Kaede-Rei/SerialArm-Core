const test = require('node:test');
const assert = require('node:assert/strict');
const fs = require('node:fs');
const path = require('node:path');
const vm = require('node:vm');

const source = fs.readFileSync(path.join(__dirname, '../renderer/app.js'), 'utf8');
const functions = source.slice(source.indexOf('function calibrationSelectableLinks()'), source.indexOf('function modelCalibrationReplayConfirm()'));
const bindings = source.slice(source.indexOf("        for (const id of ['model-cal-pose-budget'"), source.indexOf("        if ($('model-calibration-teach-start'))"));
assert.ok(functions.startsWith('function calibrationSelectableLinks()'));
assert.ok(bindings.includes('model-cal-mode-local'));

function createFixture({ preview = null, connected = false, historical = null, loadPreview = null } = {}) {
    const controls = new Map();
    for (const id of ['model-cal-mode-global', 'model-cal-mode-local', 'model-cal-lock-select',
        'cal-link-lock-all', 'cal-link-unlock-all', 'cal-link-cancel', 'cal-link-confirm',
        'model-cal-pose-budget', 'model-cal-validation', 'model-cal-regularization', 'model-cal-svd', 'model-cal-information']) {
        controls.set(id, {value: id === 'model-cal-pose-budget' ? '8' : '0.25', dataset:{}, disabled:false});
    }
    let selected = [];
    const dialog = {
        innerHTML:'', open:false,
        showModal(){this.open=true;selected = [...this.innerHTML.matchAll(/<input type="checkbox" data-gravity-locked="([^"]+)"( checked)?/g)].map(m=>({dataset:{gravityLocked:m[1]},checked:!!m[2]}));},
        close(){this.open=false;this.onclose?.();},
        querySelectorAll(selector){assert.equal(selector,'[data-gravity-locked]');return selected;}
    };
    const toasts = [];
    const state = {
        modelCalibrationOptions:{mode:'global',locked_links:[],pose_budget:8,validation_fraction:0.25,
            regularization:0.01,svd_relative_threshold:0.0001,minimum_information_score:0.0001},
        modelCalibrationOffline:historical, modelData:preview, telemetry:null,
        config:{profile:'tomato_picker'}, info:{available:{model:true}},language:'zh-CN',modelCalibrationSavePreview:null
    };
    const context = vm.createContext({
        state, $:(id)=>id==='confirm-dialog'?dialog:controls.get(id),
        t:()=> '取消', esc:(text)=>String(text), toast:(...args)=>toasts.push(args),
        workbenchConnected:()=>connected,
        loadModelData:async()=>{state.modelData=loadPreview;},
        render:()=>{},operation:async(fn)=>{try{return await fn();}catch(e){toasts.push([e.message,true]);return null;}}
    });
    vm.runInContext(functions,context);
    vm.runInContext(bindings,context);
    return {controls, state, selected:()=>selected,dialog,toasts,ctx:context};
}
const preview = {
    profile:'tomato_picker', native:{config:{joint_names:['joint1','joint2','joint3']}},
    model:{
        links:[{name:'Link1',inertial:{mass:1}}, {name:'Link2',inertial:{mass:0.5,valid:false}},
            {name:'tool0',inertial:null}],
        joints:[{name:'joint1',child:'Link1'}, {name:'joint2',child:'Link2'}, {name:'joint3',child:'tool0'}]
    }
};

test('local mode opens trusted Link picker from the same offline URDF preview as model workbench', async()=>{
    const f=createFixture({preview});
    await f.controls.get('model-cal-mode-local').onclick();
    assert.equal(f.state.modelCalibrationOptions.mode,'local');
    assert.equal(f.dialog.open,true);
    assert.deepEqual([...f.state.modelCalibrationOptions.locked_links],['Link1','Link2']);
    // The last controlled joint has no inertial Link; only valid selectable links appear
    assert.equal(f.selected().length,2);
    assert.match(f.dialog.innerHTML,/Link2/);
    f.selected()[1].checked=false;
    f.controls.get('cal-link-confirm').onclick();
    assert.equal(f.dialog.open,false);
    assert.deepEqual([...f.state.modelCalibrationOptions.locked_links],['Link1']);
    assert.equal(f.state.modelCalibrationOptions.mode,'local');
    assert.equal(f.toasts.length,0);
});

test('link picker has only generic lock controls and manual selection, without terminal-link shortcut', async()=>{
    const f=createFixture({preview});
    await f.controls.get('model-cal-mode-local').onclick();
    assert.doesNotMatch(f.dialog.innerHTML,/cal-link-only-last|Only correct final controlled joint link|仅校正最后受控关节的 Link/);
    assert.match(f.dialog.innerHTML,/cal-link-lock-all/);
    assert.match(f.dialog.innerHTML,/cal-link-unlock-all/);
    f.controls.get('cal-link-lock-all').onclick();
    assert.equal(f.selected().every(item=>item.checked),true);
    f.selected()[1].checked=false;
    f.controls.get('cal-link-confirm').onclick();
    assert.deepEqual([...f.state.modelCalibrationOptions.locked_links],['Link1']);
});

test('switching global is never blocked by an unfinished local selection or a missing list', async()=>{
    const f=createFixture();
    f.state.modelCalibrationOptions.mode='local';
    f.state.modelCalibrationOptions.locked_links=[];
    await f.controls.get('model-cal-mode-global').onclick();
    assert.equal(f.state.modelCalibrationOptions.mode,'global');
    assert.deepEqual([...f.state.modelCalibrationOptions.locked_links],[]);
    assert.equal(f.toasts.length,0);
});

test('missing link data never traps GUI in local mode or silently does nothing',async()=>{
    const f=createFixture();
    await f.controls.get('model-cal-mode-local').onclick();
    assert.equal(f.state.modelCalibrationOptions.mode,'global');
    assert.equal(f.dialog.open,false);
    assert.match(f.toasts[0][0],/加载模型|标定记录/);
    await f.controls.get('model-cal-mode-global').onclick();
    assert.equal(f.state.modelCalibrationOptions.mode,'global');
});

test('cancel on first entering local reverts to global, while editing stays local', async()=>{
    const f=createFixture({preview});
    await f.controls.get('model-cal-mode-local').onclick();
    f.controls.get('cal-link-cancel').onclick();
    assert.equal(f.state.modelCalibrationOptions.mode,'global');
    assert.equal(f.dialog.open,false);
    await f.controls.get('model-cal-mode-local').onclick();
    f.selected()[0].checked=false;
    f.controls.get('cal-link-confirm').onclick();
    await f.controls.get('model-cal-lock-select').onclick();
    f.controls.get('cal-link-cancel').onclick();
    assert.equal(f.state.modelCalibrationOptions.mode,'local');
});

test('outdated connected Core cannot substitute approximate preview Link mapping',async()=>{
    const f=createFixture({preview,connected:true});
    await f.controls.get('model-cal-mode-local').onclick();
    assert.equal(f.state.modelCalibrationOptions.mode,'global');
    assert.match(f.toasts[0][0],/重新编译/);
});

test('model discovery is retried offline and never picks tool0 as an inertial Link',async()=>{
    const f=createFixture({loadPreview:preview});
    await f.controls.get('model-cal-mode-local').onclick();
    assert.equal(f.state.modelCalibrationOptions.mode,'local');
    assert.equal(f.selected().length,2);
});

test('invalid local selections block task submission, not navigation or value editing', async()=>{
    const f=createFixture({preview});
    await f.controls.get('model-cal-mode-local').onclick();
    assert.throws(()=>vm.runInContext('captureModelCalibrationOptions()',f.ctx),/Link 选择无效/);
    await f.controls.get('model-cal-mode-global').onclick();
    assert.doesNotThrow(()=>vm.runInContext('captureModelCalibrationOptions()',f.ctx));
});

test('re-clicking local mode preserves the existing confirmed Link selection', async()=>{
    const f=createFixture({preview});
    await f.controls.get('model-cal-mode-local').onclick();
    f.selected()[1].checked=false;
    f.controls.get('cal-link-confirm').onclick();
    await f.controls.get('model-cal-mode-local').onclick();
    assert.deepEqual([...f.state.modelCalibrationOptions.locked_links],['Link1']);
    assert.equal(f.selected()[0].checked,true);
    assert.equal(f.selected()[1].checked,false);
});

test('native link mapping takes precedence over the offline preview', async()=>{
    const f=createFixture({preview,connected:true});
    f.state.telemetry={model_calibration:{identifiable_links:[{name:'PhysicalLink',joint:'joint1',mass:0.1}]},joint_names:['joint1']};
    await f.controls.get('model-cal-mode-local').onclick();
    assert.deepEqual([...f.state.modelCalibrationOptions.locked_links],['PhysicalLink']);
    assert.equal(f.selected().length,1);
});

test('historical link metadata takes precedence over a changed current model', async()=>{
    const history={metadata:{joint_names:['jointA']},identifiable_links:[{name:'OldLink',joint:'jointA',mass:0.7}]};
    const f=createFixture({preview,historical:history});
    await f.controls.get('model-cal-mode-local').onclick();
    assert.deepEqual([...f.state.modelCalibrationOptions.locked_links],['OldLink']);
});

test('offline recompute re-reads verified history instead of making a metadata-less placeholder',()=>{
    assert.match(source,/if \(!state\.modelCalibrationOffline\) state\.modelCalibrationOffline = await api\.request\('model_calibration_load', \{directory: state\.modelCalibrationDirectory\}\)/);
    assert.doesNotMatch(source,/modelCalibrationOffline = \{ directory: state\.modelCalibrationDirectory, metadata: \{\}, result: \{\} \}/);
});
