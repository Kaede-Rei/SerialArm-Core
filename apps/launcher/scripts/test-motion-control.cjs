const test = require('node:test'), assert = require('node:assert/strict');
const fs = require('node:fs'), path = require('node:path'), vm = require('node:vm');
const goals = require('../renderer/motion-goals.js');
const source = fs.readFileSync(path.join(__dirname, '../renderer/app.js'), 'utf8');
function ui(names, state, stubs = {}) {
    const context = {state, workbenchConnected:() => true, workbenchRecoveryRequired:()=>false, modelCalibrationActive:()=>false,
        busyText:value=>value, refreshWorkbenchTargetFields:()=>{}, queueWorkbenchTargetGhost:()=>{}, ...stubs};
    vm.createContext(context);
    for (const name of names) { const body = source.match(new RegExp('(?:async )?function '+name+'\\([^]*?\\n\\}')); assert.ok(body,name); vm.runInContext(body[0],context); }
    return context;
}
const near = (a,b) => assert.ok(Math.abs(a-b)<1e-9*Math.max(1,Math.abs(a)),`${a} != ${b}`);
function stepError(zeta,t) {
    if(zeta===1) return Math.exp(-t)*(1+t);
    if(zeta<1) {const w=Math.sqrt(1-zeta*zeta);return Math.exp(-zeta*t)*(Math.cos(w*t)+zeta/w*Math.sin(w*t));}
    const d=Math.sqrt(zeta*zeta-1),a=-zeta+d,b=-zeta-d;return(-b*Math.exp(a*t)+a*Math.exp(b*t))/(a-b);
}
test('return goals respect specified compliance, damping and continuous 5% settling',()=>{
    for(const zeta of [.7,.9,1,1.2]) {
        const result=goals.convert({kind:'return',time:1.7,compliance:.04,zeta});
        near(1/result.stiffness,.04);near(result.damping/(2*Math.sqrt(result.mass*result.stiffness)),zeta);
        const omega=Math.sqrt(result.stiffness/result.mass);near(stepError(zeta,omega*1.7),.05);
        for(let time=1.7;time<20;time+=.01) assert.ok(Math.abs(stepError(zeta,omega*time))<=.05000001);
    }
});
test('free response has no spring and decays speed to 5% at the requested time',()=>{
    const result=goals.convert({kind:'free',time:2.5,acceleration:.4});
    assert.equal(result.stiffness,0);near(1/result.mass,.4);near(Math.exp(-result.damping/result.mass*2.5),.05);
    assert.throws(()=>goals.convert({kind:'free',time:1,acceleration:Number.MIN_VALUE}));
    assert.throws(()=>goals.convert({kind:'return',time:1,compliance:.1,zeta:.3}));
});
test('representable raw values round trip without changing authored M/D/K',()=>{
    for(const parameters of [[2,8,8],[2,3,0]]) { const [mass,damping,stiffness]=parameters,result=goals.convert(goals.fromMdk(...parameters)); near(result.mass,mass);near(result.damping,damping);near(result.stiffness,stiffness); }
});
function targetState(){return {language:'zh-CN',workbenchEpoch:2,workbenchTargetEpoch:null,workbenchTargets:[],telemetry:{valid:true,robot_state:'ACTIVE',feedback_age_ms:2,joint_names:['turn','slide'],joint:{pos:[.1,.2]},sequence:10,impedance_mode:'RIGID_HOLD'},readiness:{modes:{RIGID_TRACKING:'verified'}},modelData:{model:{joints:[{name:'turn',type:'revolute',limit:{lower:-2,upper:2}},{name:'slide',type:'prismatic',limit:{lower:0,upper:.5}}]}}};}
test('target draft anchors once and telemetry cannot silently overwrite user edits',()=>{
    const state=targetState();let moves=0;const context=ui(['workbenchTargetFeedbackReady','ensureWorkbenchTargetDraft','workbenchTargetProblem','workbenchTargetMayMove','editWorkbenchTarget'],state,{workbenchRequest:()=>moves++});
    context.ensureWorkbenchTargetDraft();assert.deepEqual(state.workbenchTargets,[.1,.2]);
    context.editWorkbenchTarget(0,'.7');state.telemetry.joint.pos=[.3,.25];context.ensureWorkbenchTargetDraft();
    assert.equal(state.workbenchTargets[0],.7);assert.equal(moves,0);assert.equal(context.workbenchTargetMayMove(),true);
    context.editWorkbenchTarget(1,'-.01');assert.match(context.workbenchTargetProblem(),/限位/);assert.equal(context.workbenchTargetMayMove(),false);
    context.editWorkbenchTarget(1,'');assert.equal(state.workbenchTargets[1],null);assert.equal(context.workbenchTargetMayMove(),false);
    state.workbenchEpoch++;assert.equal(context.workbenchTargetMayMove(),false);context.ensureWorkbenchTargetDraft();assert.deepEqual(state.workbenchTargets,[.3,.25]);
    state.telemetry.impedance_mode='COMPLIANT_DRAG';assert.equal(context.workbenchTargetMayMove(),false);
});
test('external measurement is absent until explicitly confirmed and all values entered',()=>{
    const state={externalTcpDraft:{}};const context=ui(['readExternalTcpMeasurement'],state);
    assert.equal(context.readExternalTcpMeasurement(),null);state.externalTcpDraft={confirmed:true,x:'',y:'',z:'',tolerance:''};assert.throws(()=>context.readExternalTcpMeasurement());
    state.externalTcpDraft={confirmed:true,x:'0',y:'-0.2',z:'.4',tolerance:'1'};
    const measured=JSON.parse(JSON.stringify(context.readExternalTcpMeasurement()));assert.deepEqual(measured,{source:'manual_external_measurement',displacement_m:[0,-.0002,.0004],tolerance_m:.001});
    state.externalTcpDraft.confirmed=false;assert.equal(context.readExternalTcpMeasurement(),null);
});
test('gain deployment requires evidence for each use rather than a shared pass flag',()=>{
    const context=ui(['gainCandidateDeployable'],{});
    const rigid={category:'rigid_tracking',validated:true,hold_gains:{kp:[80],kd:[2]},hold_verification:{validated:true}};
    const compliant={category:'compliant_tracking',validated:true,hold_gains:{kp:[10],kd:[.3]},compliant_separation_validated:true,hold_verification:{validated:true},rigid_reference_kp:[80],rigid_reference_kd:[2],tested_speed_fraction:.65};
    const drag={category:'compliant_drag',validated:true,drag_validation:{validated:true}};
    const servo={category:'servo_tracking',validated:true,native_validation:{validated:true},admittance_validation:{validated:true}};
    const all={category:'all',validated:true,targets:{rigid_tracking:rigid,compliant_tracking:compliant,compliant_drag:drag,servo_tracking:servo}};
    assert.equal(context.gainCandidateDeployable(all),true);
    assert.equal(context.gainCandidateDeployable({...servo,admittance_validation:{validated:false}}),false);
    assert.equal(context.gainCandidateDeployable({...rigid,hold_verification:undefined}),false);
    assert.equal(context.gainCandidateDeployable({...rigid,hold_gains:undefined}),false);
    assert.equal(context.gainCandidateDeployable({...servo,category:'identification_tracking'}),false);
    all.targets.extra={validated:true};assert.equal(context.gainCandidateDeployable(all),false);delete all.targets.extra;
    assert.equal(context.gainCandidateDeployable({...rigid,hold_verification:undefined,hold_compensation:{enabled:true},hold_compensation_verification:{validated:true}}),true);
    assert.equal(context.gainCandidateDeployable({...drag,drag_validation:undefined}),false);
    delete all.targets.compliant_drag;assert.equal(context.gainCandidateDeployable(all),false);
});
test('only explicit calibrated acceptance changes are sent and cannot bypass supported bounds',()=>{
    const state={};const context=ui(['readGainRequirements'],state);
    assert.deepEqual(JSON.parse(JSON.stringify(context.readGainRequirements())),{});
    state.gainRequirements={peak_error_rad:'.01',settling_time_s:'1.1'};
    assert.deepEqual(JSON.parse(JSON.stringify(context.readGainRequirements())),{peak_error_rad:.01,settling_time_s:1.1});
    state.gainRequirements.peak_error_rad='';assert.throws(()=>context.readGainRequirements());
    state.gainRequirements.peak_error_rad='.041';assert.throws(()=>context.readGainRequirements());
});
test('native candidate arrays expose measured response, observer noise and advisory contact scope',()=>{
    const state={language:'zh-CN'};
    const context=ui(['capabilityNumber','gainCheckStatus','gainQualificationScopeHtml','gainResponseModelsHtml','gainObserverAndBoundsHtml'],state,{esc:value=>String(value??''),t:key=>key});
    const actual={response_model:[{status:'frequency_response_only',natural_frequency_rad_s:null,damping_ratio:null,effective_delay_s:null,heldout_fit_error:.17,observed_cutoff_hz:null,tested_follow_frequency_hz:1.2}],observer_model:[{bias_nm:.12,noise_rms_nm:.003,samples:64,source:'no_contact_residual'}],feasible_admittance:[{status:'local_no_contact_bound',outer_frequency_max_hz:.24,mass_min:1.23,deadband_nm:.009,contact_validation:'not_tested'}]};
    const response=context.gainResponseModelsHtml(actual.response_model,['axis']);assert.match(response,/axis/);assert.match(response,/0\.170/);assert.match(response,/—/);assert.doesNotMatch(response,/NaN|undefined/);
    const details=context.gainObserverAndBoundsHtml(actual,['axis']);assert.match(details,/0\.12000/);assert.match(details,/0\.00300/);assert.match(details,/64/);assert.match(details,/0\.240/);assert.match(details,/1\.23000/);assert.match(details,/0\.00900/);assert.match(details,/未测试/);assert.match(details,/不会自动修改/);
    const scope=context.gainQualificationScopeHtml({trajectory:'local_multifrequency',external_tip:'pending',disturbance_release:'pending',contact:'not_tested'});assert.match(scope,/轨迹测量/);assert.match(scope,/局部多频测量/);assert.match(scope,/针尖外测/);assert.match(scope,/接触测试/);assert.doesNotMatch(scope,/external_tip|local_multifrequency/);
});
test('runtime saved settings display actual execution and compensation snapshots, not editable drafts',()=>{
    const state={language:'zh-CN',telemetry:{joint_names:['axis']},admittance:{execution_gains:{rigid_hold:{kp:[12],kd:[.2]},rigid_tracking:{kp:[30],kd:[.6]},servo_tracking:{kp:[60],kd:[1.2]}},hold_compensation:{enabled:false,integral_gain:[.15],max_torque:[.17],max_rate:[.05],contact_threshold:[.2],velocity_threshold:.02,error_threshold:.03}},tuneDraft:{execution_gains:{rigid_hold:{kp:[999],kd:[999]}},hold_compensation:{enabled:true}}};
    const context=ui(['capabilityNumber','runtimeExecutionSettingsHtml'],state,{esc:value=>String(value??''),t:key=>key,paramLabel:key=>key});
    const html=context.runtimeExecutionSettingsHtml();assert.match(html,/rigid_hold/);assert.match(html,/12\.0000/);assert.match(html,/30\.0000/);assert.match(html,/60\.0000/);assert.match(html,/OFF/);assert.match(html,/0\.1700/);assert.doesNotMatch(html,/999|· ON/);
});
test('temporary gain apply refreshes the native execution snapshot before rendering',async()=>{
    const state={telemetry:{robot_state:'INACTIVE'},calibrationToolSelected:{gain:{directory:'/saved/measured-task'}}};
    const calls=[];let fresh=false;
    const context=ui(['temporaryCandidateAction'],state,{blockingOperation:async(_label,run)=>run(),api:{request:async(method,params)=>{calls.push({method,params});return {gain_applied:true};}},loadAdmittance:async()=>{fresh=true;},render:()=>assert.equal(fresh,true)});
    await context.temporaryCandidateAction('apply','gain');assert.equal(calls.length,1);assert.equal(calls[0].method,'calibration_candidate_apply');assert.equal(calls[0].params.directory,'/saved/measured-task');assert.equal(calls[0].params.kind,'gain');assert.equal(state.calibrationRuntimeCandidates.gain_applied,true);
});
