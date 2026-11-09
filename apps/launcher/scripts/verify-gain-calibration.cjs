const fs = require('node:fs');
const path = require('node:path');
const vm = require('node:vm');
const assert = require('node:assert/strict');
const {spawnSync} = require('node:child_process');
const root = path.resolve(__dirname, '..');
const source = fs.readFileSync(path.join(root, 'renderer/app.js'), 'utf8');
function extract(name) {
  const start = source.indexOf(`function ${name}(`);
  assert(start >= 0, `Missing renderer function ${name}`);
  const next = source.indexOf('\nfunction ', start + 1);
  return source.slice(start, next < 0 ? undefined : next);
}
const context = vm.createContext({state:{language:'zh-CN',gainMode:'all',gainTuningMode:'quick',gainFinePurpose:'rigid'},
  esc:String, busyText:(zh)=>zh, capabilityNumber:String, gainCalibrationHasLinearAxes:()=>false,
  gainRequirementFieldsHtml:()=>'',gainProbeStatusHtml:()=>'',calibrationTaskLoaded:()=>true,
  gainCandidateReviewHtml:()=>'',gainTaskWarningsHtml:()=>'',gainModelInitializationHtml:()=>''});
for (const name of ['gainTimeText','gainBudgetValues','gainBudgetHtml','gainCalibrationOptionsHtml','readGainRequirements','gainCandidateGroups','gainCandidatePurposeAvailable','gainCandidateDeployable']) vm.runInContext(extract(name),context);
const html = vm.runInContext('gainCalibrationOptionsHtml({gain_budget_source_duration_s:25},null,false)',context);
assert(html.includes('四用途快速标定') && html.includes('目标用时') && html.includes('5 分 0 秒') && html.includes('6 分 15 秒'));
context.state.gainTuningMode='fine';context.state.gainFinePurpose='servo_tracking';
const fine = vm.runInContext('gainCalibrationOptionsHtml({gain_budget_source_duration_s:25},null,false)',context);
assert(fine.includes('10 分 0 秒') && fine.includes('id="gain-target"'));
assert.equal(context.state.gainMode,'servo_tracking');
const peak = vm.runInContext('readGainRequirements().peak_error_rad',context);
assert(Math.abs(peak*180/Math.PI-.1)<1e-12);
context.state.gainRequirements={peak_error_rad:0};
assert.throws(()=>vm.runInContext('readGainRequirements()',context));
const tracking = {independent:true,duration_s:1,measured_duration_s:1,metrics:[{samples:200,measurement_samples:200,rms_error_rad:.01,peak_error_rad:.02,velocity_rms_rad_s:.01,tail_error_rad:.01,settling_s:.2}]};
const warning = {category:'servo_tracking',joint_names:['j1'],validation_scope:'independent_demonstration_trajectory',kp:[3],kd:[.4],validated:false,available:true,native_validation:tracking,admittance_required:false,admittance_validation:{required:false,status:'not_requested'}};
context.candidate=warning;
assert.equal(vm.runInContext('gainCandidateDeployable(candidate)',context),true,'Measured warnings remain usable without extra approval');
context.candidate={...warning,native_validation:{...tracking,completed:false}};
assert.equal(vm.runInContext('gainCandidateDeployable(candidate)',context),false);
context.candidate={...warning,validation_scope:'legacy_local_probe'};
assert.equal(vm.runInContext('gainCandidateDeployable(candidate)',context),false,'Legacy records never acquire missing validation');
// Match the terminal's heterogeneous experiment records: parameter rows have
// no measurement conclusion; disturbance axes alternate FRF and recovery rows.
Object.assign(context,{t:key=>key,localizedError:String,gainProbeMessageLabel:String,gainPurposeLabel:String,
  jointDisplayValue:value=>typeof value === 'number' && Number.isFinite(value) ? String(value) : '—',
  gainPathValidationHtml:()=>'',gainTaughtPathMetricsHtml:()=>'',gainObserverAndBoundsHtml:()=>'',
  gainQualificationScopeHtml:()=>'',selectedGainPurposes:()=>['servo_tracking'],setupActionHtml:()=>''});
context.state.calibrationRuntimeCandidates={};
for (const name of ['capabilityNumber','gainProbeMessageLabel','gainCheckStatus','gainTaskConclusion','gainWarningLabel','gainCandidateAvailabilityLabel','gainCandidateResultCounts','gainCandidateParametersHtml','gainModelInitializationHtml','gainResponseModelsHtml','gainMotionEstablishmentHtml','gainExperimentHtml','gainCandidateReviewHtml'])vm.runInContext(extract(name),context);
context.terminalStatus={task_id:'cancelled-task',phase:'cancelled',error:'short response experiment interrupted',fault_interrupted:false};
context.terminalCandidate={task_id:'cancelled-task',outcome:'execution_fault',available:false};
const historicalStop=vm.runInContext('gainTaskConclusion(terminalStatus,terminalCandidate)',context);
assert(historicalStop.outcome==='cancelled'&&historicalStop.review&&historicalStop.title==='标定已停止'&&historicalStop.reason.includes('短测响应试验已中断'),'Historical result cancellation takes precedence over the old candidate fault classification');
context.terminalStatus={task_id:'x',phase:'complete',gain_available:false,termination:{kind:'insufficient_measurement',reason:'axis_excitation_input_budget_plateau_at_low_frequency_boundary',blocked_joint_names:['joint3']}};
const terminalLimit=vm.runInContext('gainTaskConclusion(terminalStatus)',context);
assert(terminalLimit.review&&!terminalLimit.success&&terminalLimit.reason.includes('joint3')&&terminalLimit.reason.includes('低频激励与输入预算边界'));
context.terminalStatus={task_id:'x',phase:'complete',gain_available:true,termination:{kind:'qualified',reason:'',blocked_joint_names:[]}};
assert.equal(vm.runInContext('gainTaskConclusion(terminalStatus).review',context),false);
context.motionProbe={kind:'motion_establishment_probe',kp:[20],kd:[.4],axes:[]};
const motionProbe=vm.runInContext("gainExperimentHtml(motionProbe,['J1'])",context);
assert(motionProbe.includes('双向运动建立短测')&&motionProbe.includes('仍需多频辨识与用途验证')&&!motionProbe.includes('通过'));
for(const reason of ['motion_establishment_only_no_spectral_fit','spectral_excitation_boundary_requires_symmetric_motion_probe_at_same_complete_gains','confirmed_response_plateau_requires_symmetric_motion_probe_at_same_complete_gains','symmetric_motion_probe_insufficient_at_tested_input_not_hardware_capability']){
  context.reason=reason;
  for(const language of ['zh-CN','en']){context.state.language=language;assert.notEqual(vm.runInContext('gainWarningLabel(reason)',context),reason);}
}
context.state.language='zh-CN';
const measuredReasons=[
  ['response_below_encoder_noise_floor','响应低于编码器与噪声门槛','Response is below the encoder and noise threshold'],
  ['closed_loop_model_not_identifiable','闭环模型尚不可辨识','The closed-loop model is not yet identifiable'],
  ['independent_response_below_motion_floor_static_behavior_only','独立扰动位移不足，仅有静态证据','Independent disturbance displacement is insufficient; only static evidence is available'],
  ['sustained_oscillation_after_release','释放后持续位置往复','Sustained position oscillation after release'],
];
for(const [code,zh,en] of measuredReasons){
  context.reason=code;context.state.language='zh-CN';
  assert.equal(vm.runInContext('gainWarningLabel(reason)',context),zh);
  context.state.language='en';assert.equal(vm.runInContext('gainWarningLabel(reason)',context),en);
}
context.state.language='zh-CN';
for(const code of ['axis_response_to_increased_effort_plateau_confirmed_by_slower_complete_set_measurement','response_effort_plateau_requires_slower_complete_gain_set_confirmation','gain_increase_rejected_without_distinguishable_excitation_input_growth','axis_response_effort_plateau_at_excitation_boundary','axis_excitation_input_budget_plateau_at_low_frequency_boundary','slow_confirmation_deferred_for_oscillation_feedback_reduction','gain_change_deferred_during_same_complete_gain_set_confirmation']){
  context.reason=code;
  for(const language of ['zh-CN','en']){context.state.language=language;assert.notEqual(vm.runInContext('gainWarningLabel(reason)',context),code,'Motion establishment reasons have both language labels');}
}
context.state.language='zh-CN';
context.motion={kind:'motion_establishment',record_path:'window.csv',established:false,retry:false,reasons:['measured_insufficient_motion_bounded_stiffness_step_no_damping_inference'],next_kp:[500],next_kd:[2.8],axes:[{qualified:false,positive_travel_rad:.0019,negative_travel_rad:.0011,required_travel_rad:.0023,applied_pd_peak_nm:2.78}]};
const missingMotion=vm.runInContext("gainExperimentHtml(motion,['J3'])",context);
assert(missingMotion.includes('本阶段已停止，尚未建立有效运动')&&missingMotion.includes('<td>Kp —<br>Kd —</td><td>Kp 500'),'Missing current gains are never replaced with the proposed gains');
context.measuredMotion=[{record_path:'other.csv',kp:[9],kd:[1]},{record_path:'window.csv',kp:[300],kd:[2]}];
const historicalMotion=vm.runInContext("gainMotionEstablishmentHtml(motion,['J3'],measuredMotion)",context);
assert(historicalMotion.includes('<td>Kp 300<br>Kd 2</td>'),'Historical current gains come only from the matching measured record');
context.motion.current_kp=[320];context.motion.current_kd=[2.1];context.motion.blocked_axes=[0];
context.motion.axes[0].input_response_progress={planned_input_nm:1.2,proposed_input_nm:1.21,observed_input_nm:2.78,response_plateau_pending:true,input_budget_plateau:true,blocked:true,reason:'response_below_encoder_noise_floor'};
const progressMotion=vm.runInContext("gainExperimentHtml(motion,['J3'])",context);
assert(progressMotion.includes('受阻关节 J3')&&progressMotion.includes('等待确认测量')&&progressMotion.includes('本轴停止'));
assert(progressMotion.includes('<td>2.78</td><td>本窗 1.2<br>候选 1.21</td>'),'Actual PD and planned reference drive remain separate measurements');
assert(progressMotion.includes('<td>Kp 320<br>Kd 2.1</td>')&&progressMotion.includes('title="response_below_encoder_noise_floor"'));
context.motionWarning='identification_insufficient:native: motion_establishment_incomplete;joint3:response_below_encoder_noise_floor';
const motionWarning=vm.runInContext('gainWarningLabel(motionWarning)',context);
assert(motionWarning.includes('起动测量尚未建立有效运动；joint3')&&motionWarning.includes('响应低于编码器与噪声门槛')&&!motionWarning.includes('identification_insufficient:'));
context.reasonExperiment={kind:'independent_torque_disturbance',axes:[
  {identifiable:false,tracking_valid:false,disturbance_valid:false,reason:measuredReasons[0][0]},
  {identifiable:false,tracking_valid:false,disturbance_valid:true,reason:measuredReasons[1][0]},
  {kind:'disturbance_recovery',measurement_complete:false,reason:measuredReasons[2][0]},
  {kind:'disturbance_recovery',measurement_complete:true,validated:false,reason:measuredReasons[3][0]},
]};
const reasonEvidence=vm.runInContext("gainExperimentHtml(reasonExperiment,['J1','J2'])",context);
for(const [code,zh] of measuredReasons)assert(reasonEvidence.includes(`title="${code}"`)&&reasonEvidence.includes(zh),'Measured reason has a readable label and retains its raw code in the tooltip');
assert.equal(vm.runInContext('capabilityNumber(84.56,6)',context),'84.56');
assert.equal(vm.runInContext('capabilityNumber(0.00123456,6)',context),'0.001235');
assert.equal(vm.runInContext('capabilityNumber(100,0)',context),'100');
assert.equal(vm.runInContext('capabilityNumber(-0.00001,4)',context),'0');
context.initialization={source:'urdf_mass_matrix_path_envelope',measured:false,kp:[2,4],kd:[.2,.4],inertia_min:[.1,.2],inertia_max:[.2,.3]};
assert(vm.runInContext("gainModelInitializationHtml(initialization,['J2','J1'])",context).includes('<th>J2</th>'));
context.bootstrap={source:'urdf_inertia_and_unmeasured_recovery_bootstrap',response_demand_source:'explicit_recovery_targets_and_urdf_inertia',kp:[1],kd:[.1],unconstrained_kp:[2],unconstrained_kd:[.2]};
const bootstrapHtml=vm.runInContext("gainModelInitializationHtml(bootstrap,['J1'])",context);
assert(bootstrapHtml.includes('URDF 惯量与未实测恢复目标的起动候选')&&bootstrapHtml.includes('整轨需求只作离线对照，不作为起动初值'));
const response={identifiable:false,tracking_valid:true,disturbance_valid:false,effective_delay_s:null,damping_ratio:null,held_out_error:null,effective_stiffness_nm_rad:999,reason:'closed_loop_fit_insufficient'};
context.experiment={kind:'independent_torque_disturbance',kp:[2,4],kd:[.2,.4],axes:[
  response,{kind:'disturbance_recovery',measurement_complete:true,validated:false,reason:'final_static_error_above_goal'},
  response,{kind:'disturbance_recovery',measurement_complete:false,validated:true,reason:'insufficient_final_window'}]};
const evidence=vm.runInContext("gainExperimentHtml(experiment,['J2','J1'])",context);
assert(evidence.includes('测量未完成') && evidence.includes('证据不足'));
assert.equal((evidence.match(/<th>J1<\/th>/g)||[]).length,2,'Alternating recovery rows keep their joint identity');
assert(vm.runInContext("gainProbeMessageLabel('hold_final_window')",context).includes('最终保持窗口'));
assert(!evidence.includes('NaN') && !evidence.includes('999'),'Tracking data cannot manufacture measured stiffness');
context.fit={identifiable:false,tracking_valid:true,disturbance_valid:true,effective_delay_s:.01125,damping_ratio:.65,effective_stiffness_nm_rad:42,held_out_error:.321};
const unknownFit=vm.runInContext("gainResponseModelsHtml([fit],['J1'])",context);
assert(!unknownFit.includes('11.25')&&!unknownFit.includes('0.65')&&!unknownFit.includes('>42<')&&!unknownFit.includes('0.321'),'Invalid fits never display default or unqualified fitted numbers');
context.fit.identifiable=true;context.fit.effective_delay_s=0;
const knownFit=vm.runInContext("gainResponseModelsHtml([fit],['J1'])",context);
assert(knownFit.includes('<td>0</td>')&&knownFit.includes('<td>0.65</td>')&&knownFit.includes('<td>42</td>')&&knownFit.includes('<td>0.321</td>'),'Explicitly valid fitted evidence retains measured values including zero delay');
context.parameters={kind:'hold_parameters',integral_enabled:true,kp:[3],kd:[.3],integral_gain:[.7],max_torque:[.2],record_path:'gain-hold-failed-integral.csv'};
const parameters=vm.runInContext("gainExperimentHtml(parameters,['J1'])",context);
assert(parameters.includes('保持实际参数') && parameters.includes('J1: 0.7') && !parameters.includes('通过'));
context.candidate={...warning,warnings:['servo_tracking:disturbance_stiffness_or_recovery_not_met'],experiments:[context.parameters,context.experiment]};
const reviewed=vm.runInContext('gainCandidateReviewHtml(candidate)',context);
assert(reviewed.includes('独立扰动刚度或释放恢复未达指标') && !reviewed.includes('NaN'));
context.candidate.candidate_comparisons=[context.motion];context.candidate.experiments.push(...context.measuredMotion);
const compared=vm.runInContext('gainCandidateReviewHtml(candidate)',context);
assert(compared.includes('起动测量决策 1')&&compared.includes('参考驱动计划峰值 Nm')&&compared.includes('原始比较记录'),'Historical motion decisions are readable alongside the original comparison data');
const backend=spawnSync('python3',[path.join(__dirname,'verify_gain_initialization.py')],{encoding:'utf8'});
if(backend.status!==0)throw new Error(backend.stdout+'\n'+backend.stderr);
if(process.env.SERIAL_ARM_GAIN_UI_RENDER==='1'){
  const {buildFixture}=require('./gain-ui-fixture.cjs');
  const fixture=buildFixture({out:process.env.SERIAL_ARM_GAIN_UI_FIXTURE || '/tmp/serialarm-gain-ui-verify'});
  const render=spawnSync(path.join(root,'node_modules/electron/dist/electron'),['--no-sandbox',path.join(__dirname,'render-gain-ui.cjs'),'--fixture',fixture.directory],{encoding:'utf8',timeout:60000});
  if(render.status!==0 || !render.stdout.includes('GAIN_UI_DOM_RENDER_PASS'))throw Error(render.stdout+'\n'+render.stderr);
  console.log(render.stdout.trim());
}
console.log('LAUNCHER_ADAPTIVE_GAIN_PASS');
