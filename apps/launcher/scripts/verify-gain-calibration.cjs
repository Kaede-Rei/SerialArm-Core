const fs = require('node:fs');
const path = require('node:path');
const vm = require('node:vm');
const assert = require('node:assert/strict');
const {spawnSync} = require('node:child_process');
const root = path.resolve(__dirname, '..');
const source = fs.readFileSync(path.join(root, 'renderer/app.js'), 'utf8');
const errorContext=vm.createContext({});
vm.runInContext(fs.readFileSync(path.join(root,'renderer/error-i18n.js'),'utf8'),errorContext);
const translateError=errorContext.serialArmErrorI18n.translate;
for(const [language,label] of [['zh-CN','结果已保存'],['en','Results saved']])assert.equal(translateError('reference_analysis_available',language),label,'A recorded normal-completion code is not translated as a failure');
assert(/失败|操作未完成/.test(translateError('model_calibration_gain_begin failed: reference_analysis_available')),'Only the exact completion status is localized as a normal result; a failed operation keeps its failure context');
const rangeError='trajectory_range_violation:joint=joint2;sample=42;value=-0.003;lower=0;upper=2;margin=0;unit=rad;scope=command_reference';
for(const prefix of ['', 'cannot safely plan demonstration: ', 'Error invoking remote method \'launcher:request\': Error: imported trajectory: ']){
  const translated=translateError(prefix+rangeError,'zh-CN');
  assert(translated.includes('joint2')&&translated.includes('采样索引 42')&&translated.includes('-0.003 rad')&&translated.includes('-0.1719°'));
  assert(translated.includes('[0, 2] rad')&&translated.includes('用户 Safety 留量 0 rad')&&translated.includes('映射后驱动位置能力的交集'));
  assert(!translated.includes('0.05')&&!translated.includes('扩大')&&!translated.includes('超过 URDF'),'Range errors use the exact reported margin and effective interval without inventing a hidden margin or a URDF-only cause');
}
const englishRange=translateError(rangeError,'en');
assert(englishRange.includes('joint2, sample index 42')&&englishRange.includes('effective interval [0, 2] rad')&&englishRange.includes('mapped driver position capability'));
const recordingRange=translateError(rangeError.replace('scope=command_reference','scope=recorded_position'));
assert(recordingRange.includes('录制位置')&&recordingRange.includes('本次录制校验留量 0 rad')&&recordingRange.includes('命令留量另用于回放参考检查')&&!recordingRange.includes('用户 Safety 留量 0 rad'),'Recording hard-range validity is distinct from the user command margin');
assert(translateError(rangeError.replace('scope=command_reference','scope=recorded_position'),'en').includes('checked separately for playback'));
assert(translateError(rangeError.replace(';scope=command_reference','')).includes('本次检查留量'),'Historical scope omission does not imply an unused user Safety margin was configured to zero');
assert(translateError('robot is not aligned with the selected trajectory endpoint at joint2').includes('按现有引导安全起步'),'Starting-condition errors use the existing guidance without demanding an exact return to an old endpoint');
assert(translateError('gain_endpoint_transition_insufficient_response:joint3').includes('关节=joint3')&&translateError('gain_endpoint_transition_insufficient_response:joint3').includes('保留已测结果'));
assert(translateError('gain endpoint transition interrupted').includes('端点自动过渡已中断'));
assert(translateError('CALIBRATION_RESPONSE_UNSTABLE').includes('标定响应保护')&&!translateError('CALIBRATION_RESPONSE_UNSTABLE').includes('机械臂安全故障'),'Response protection is distinct from a measured Safety-limit failure');
assert(translateError('model calibration stopped by safety fault').includes('具体故障'),'The legacy umbrella interruption points to the actual fault instead of inventing a Safety-limit cause');
for(const reason of ['measurement_center_mapping_invalid','measurement_center_fresh_position_invalid','measurement_center_demonstrated_path_invalid','measurement_center_bounds_invalid','measurement_center_fresh_position_outside_safety','measurement_center_outside_existing_endpoint_alignment','measurement_center_no_demonstrated_safety_intersection','measurement_center_no_inward_entry_within_existing_alignment','measurement_center_no_inward_route_within_existing_alignment','path_progress_input_invalid','path_progress_mapping_invalid','path_progress_feedback_or_limits_invalid','path_progress_state_outside_segment','path_reference_outside_feedback_lead','path_feedback_lead_below_required_braking_distance','path_feedback_lead_cannot_be_retained_with_acceleration_limit']){
  const text=translateError('Error invoking remote method \'launcher:request\': Error: '+reason);
  assert(text.includes('测量区')&&!text.includes(reason)&&!text.includes('扩大')&&!text.includes('精确回到'),'Measurement-area errors remain readable in the existing priority-error wrapper without tightening guidance');
}
for(const reason of ['measurement_center_route_unavailable','measurement_center_entry_insufficient_reserved_budget','measurement_center_entry_response_or_budget_insufficient','measurement_center_entry_control_mode_rejected','measurement_center_entry_feedback_unavailable','measurement_center_entry_feedback_stale','measurement_center_entry_no_further_calibrated_input','measurement_center_entry_execution_limited','measurement_center_entry_gain_step_consumes_feedback_lead','measurement_center_entry_gain_step_rejected','measurement_center_entry_interrupted','measurement_center_entry_interrupted_or_insufficient_response','measurement_center_evidence_write_failed']){
  const text=translateError('model_calibration_gain_begin failed: '+reason);
  assert(!text.includes(reason)&&!text.includes('扩大')&&!text.includes('精确回到'),'New native measurement-area failures use the existing priority-error localization');
}
const measurementReturnReasons=['measurement_center_original_endpoint_route_outside_safety','measurement_return_original_route_unavailable','measurement_return_endpoint_does_not_match_original_demonstration','measurement_return_original_route_invalid','measurement_return_outside_existing_alignment','measurement_return_no_nearby_original_path_projection','fresh_projection_then_reverse_original_demonstrated_route','path_position_uncertainty_consumes_feedback_lead'];
for(const reason of measurementReturnReasons){
  const text=translateError('Error invoking remote method \'launcher:request\': Error: '+reason);
  assert(!text.includes(reason)&&!text.includes('扩大')&&!text.includes('精确回到'),'New native return-path reasons are readable without changing guidance requirements');
}
assert(translateError('identification_insufficient:measurement_center_entry_insufficient_reserved_budget').includes('剩余预算不足'),'The final native identification prefix preserves the actual reserved-budget reason');
const measurementReserveReasons=['measurement_path_feedback_reserve_input_invalid','measurement_path_position_resolution_or_noise_consumes_feedback_lead','observed_position_step_and_stationary_noise_reserve','unidentified_position_step_internal_guard_buffer'];
for(const reason of measurementReserveReasons)assert(!translateError(reason).includes(reason),'Native position-reserve reasons have readable priority-error explanations');
const linearRange=translateError('trajectory_range_violation:joint=slide;sample=3;value=-1e-4;lower=0;upper=.2;margin=.001;unit=m');
assert(linearRange.includes('-0.0001 m')&&linearRange.includes('[0, 0.2] m')&&linearRange.includes('留量 0.001 m')&&!linearRange.includes('°'));
assert(!translateError(rangeError.replace('value=-0.003','value=NaN')).includes('有效区间'),'An incomplete or nonfinite range diagnostic does not invent valid numerical evidence');
assert(!translateError(rangeError.replace(';margin=0','')).includes('有效区间'),'Missing margin is not silently replaced by zero');
for(const legacy of ['demonstration trajectory exceeds safe replay range','trajectory is too short or exceeds URDF joint safety margins','re-timed trajectory violates joint margins','interpolated playback would exceed URDF joint safety margins']){
  const translated=translateError(legacy);
  assert(!translated.includes('超过 URDF')&&!translated.includes('超出 URDF')&&!translated.includes('扩大')&&translated.includes('范围'),'Historical range diagnostics no longer imply the URDF alone is wrong');
}
function extract(name) {
  const start = source.indexOf(`function ${name}(`);
  assert(start >= 0, `Missing renderer function ${name}`);
  const next = source.indexOf('\nfunction ', start + 1);
  return source.slice(start, next < 0 ? undefined : next);
}
const alertContext=vm.createContext({state:{language:'zh-CN'},busyText:zh=>zh,t:()=> '错误',localizedError:raw=>translateError(raw),
  reportedCalibrationErrors:new Set(),lastRuntimeFaultKey:'',lastTeachReplayErrorKey:'',alerts:[],gainCalibrationQualityWarning:()=>false,
  gainCalibrationFailure:status=>({code:'GAIN_CALIBRATION_FAILED',message:status.error}),jointDisplaySpec:()=>({factor:180/Math.PI,unit:'°',velocityUnit:'°/s',effortUnit:'Nm'})});
for(const name of ['normalizeDialogError','describeRuntimeFault','checkRuntimeAlerts'])vm.runInContext(extract(name),alertContext);
alertContext.showPriorityError=(raw,kind)=>alertContext.alerts.push(alertContext.normalizeDialogError(raw,kind));
const responseFault={robot_state:'FAULT',fault:{code:'CALIBRATION_RESPONSE_UNSTABLE'},model_calibration:{task_id:'response-stop',workflow_kind:'gain',phase:'failed',fault_interrupted:true,error:'model calibration stopped by safety fault'},teach_replay:{phase:'failed',error:'model calibration stopped by safety fault'}};
alertContext.snapshot=responseFault;
vm.runInContext('checkRuntimeAlerts(snapshot);checkRuntimeAlerts(snapshot)',alertContext);
assert.equal(alertContext.alerts.length,1,'The same response stop is shown once across runtime, gain and replay telemetry');
assert.equal(alertContext.alerts[0].title,'标定响应保护已触发');
assert(alertContext.alerts[0].kind==='fault'&&alertContext.alerts[0].responseProtection&&alertContext.alerts[0].raw.includes('CALIBRATION_RESPONSE_UNSTABLE')&&alertContext.alerts[0].advice.includes('不会清除'),'The named response stop retains the actual latched-fault recovery semantics and raw diagnostics');
alertContext.snapshot={...responseFault,teach_replay:{phase:'failed',error:'trajectory file not found'}};
vm.runInContext('checkRuntimeAlerts(snapshot)',alertContext);
assert.equal(alertContext.alerts.length,2,'An unrelated replay error remains visible while a response stop is latched');
alertContext.snapshot={robot_state:'ACTIVE'};vm.runInContext('checkRuntimeAlerts(snapshot)',alertContext);
alertContext.snapshot={...responseFault,fault:{code:'SAFETY_FAILED',safety_code:'CMD_TIMEOUT'}};vm.runInContext('checkRuntimeAlerts(snapshot)',alertContext);
assert.equal(alertContext.alerts.at(-1).title,'机械臂安全故障','Actual Safety errors keep the existing fault presentation');
const context = vm.createContext({state:{language:'zh-CN',gainMode:'all',gainTuningMode:'quick',gainFinePurpose:'rigid'},
  esc:String, busyText:(zh)=>zh, capabilityNumber:String, gainCalibrationHasLinearAxes:()=>false,
  gainProbeStatusHtml:()=>'',calibrationTaskLoaded:()=>true,
  gainCandidateReviewHtml:()=>'',gainTaskWarningsHtml:()=>'',gainModelInitializationHtml:()=>''});
for (const name of ['gainTimeText','gainBudgetValues','gainBudgetHtml','gainCalibrationOptionsHtml','gainCandidateGroups','gainCandidatePurposeAvailable','gainCandidateReviewReady','gainCandidateDeployable']) vm.runInContext(extract(name),context);
const html = vm.runInContext('gainCalibrationOptionsHtml({gain_budget_source_duration_s:25},null,false)',context);
assert(html.includes('四用途快速标定') && html.includes('目标用时') && html.includes('7 分 30 秒') && html.includes('10 分 0 秒'));
context.state.gainTuningMode='fine';context.state.gainFinePurpose='servo_tracking';
const fine = vm.runInContext('gainCalibrationOptionsHtml({gain_budget_source_duration_s:25},null,false)',context);
assert(fine.includes('5 分 0 秒') && fine.includes('6 分 15 秒') && fine.includes('id="gain-target"'));
assert.equal(context.state.gainMode,'servo_tracking');
assert(!html.includes('data-gain-requirement')&&!fine.includes('data-gain-requirement')&&html.includes('选择需要采用的用途'),'Both modes omit obsolete accuracy inputs and state the next user action');
for(const [mode,target,maximum] of [['quick',450,600],['fine',300,375]]){
 context.budgetInput={gain_tuning_mode:mode,gain_budget_source_duration_s:25};
 const budget=vm.runInContext('gainBudgetValues(budgetInput,true)',context);
 assert.equal(budget.target,target);assert.equal(budget.maximum,maximum);
}
context.budgetInput={schema:'serial_arm_gain_candidate',gain_tuning_mode:'quick',budget_source_duration_s:28.14,target_budget_s:337.68,time_budget_s:422.1};
const historicalBudget=vm.runInContext('gainBudgetValues(budgetInput,true)',context);
assert.equal(historicalBudget.target,337.68);assert.equal(historicalBudget.maximum,422.1,'Old task budgets are displayed exactly as recorded');
delete context.budgetInput.target_budget_s;delete context.budgetInput.time_budget_s;
assert.equal(vm.runInContext('gainBudgetValues(budgetInput,true).maximum',context),0,'Missing historical budget fields remain unknown rather than adopting a new policy');
assert(!source.includes('gain_requirements:')&&!source.includes('function readGainRequirements'),'Frontend does not submit obsolete accuracy thresholds');
const tracking = {independent:true,duration_s:1,measured_duration_s:1,metrics:[{samples:200,measurement_samples:200,rms_error_rad:.01,peak_error_rad:.02,velocity_rms_rad_s:.01,tail_error_rad:.01,settling_s:.2}]};
const warning = {category:'servo_tracking',joint_names:['j1'],validation_scope:'independent_demonstration_trajectory',kp:[3],kd:[.4],validated:false,available:true,qualification_complete:true,disturbance_verification:{available:true,completed:true,record_path:'disturbance.csv'},native_validation:tracking,admittance_required:false,admittance_validation:{required:false,status:'not_requested'}};
context.candidate=warning;
assert.equal(vm.runInContext('gainCandidateDeployable(candidate)',context),true,'Measured warnings remain usable without extra approval');
context.candidate={...warning,available:false};
assert.equal(vm.runInContext('gainCandidateDeployable(candidate)',context),true,'Modern aggregate unavailability cannot hide independently complete parameter measurements');
context.candidate={...warning,available:false,error:'measurement_in_progress'};
assert.equal(vm.runInContext('gainCandidateDeployable(candidate)',context),false,'Parameters still being measured remain unavailable');
context.candidate={...warning,available:false,warnings:['not_started']};
assert.equal(vm.runInContext('gainCandidateDeployable(candidate)',context),false,'An explicit unmeasured use remains unavailable even with leftover nested fields');
context.candidate={...warning,available:false,native_validation:{...tracking,available:false}};
assert.equal(vm.runInContext('gainCandidateDeployable(candidate)',context),false,'Nested incomplete measurement remains unavailable');
context.candidate={...warning,native_validation:{...tracking,completed:false}};
assert.equal(vm.runInContext('gainCandidateDeployable(candidate)',context),false);
context.candidate={...warning,validation_scope:'legacy_local_probe'};
assert.equal(vm.runInContext('gainCandidateDeployable(candidate)',context),false,'Legacy records never acquire missing validation');
context.candidate={category:'compliant_drag',joint_names:['j1'],validation_scope:'independent_demonstration_trajectory',kp:[0],kd:[.4],validated:false,available:false,drag_validation:{available:true,completed:true,samples:100,initial_velocity_rad_s:[.1],decay_s:[.8]}};
assert.equal(vm.runInContext('gainCandidateDeployable(candidate)',context),true,'Historical completed drag vectors retain their measured parameter contract');
context.candidate.drag_validation.completed=false;
assert.equal(vm.runInContext('gainCandidateDeployable(candidate)',context),false,'Incomplete historical drag is not available');
context.candidate={category:'compliant_drag',joint_names:['j1'],validation_scope:'independent_demonstration_trajectory',kp:[0],kd:[.4],available:true,validated:true,drag_validation:{available:true,completed:true,samples:100,axes:[{joint:'j1',samples:100,quiet_stability_measured:true,quiet_stability_passed:true,quiet_position_span_rad:.0001,quiet_speed_rad_s:.0002,release_decay_measured:false,release_decay_passed:false,initial_velocity_rad_s:null,decay_s:null}]}};
assert.equal(vm.runInContext('gainCandidateDeployable(candidate)',context),true,'Measured quiet-scope drag remains temporarily applicable without inventing release input or decay');
context.candidate.validated=false;context.candidate.drag_validation.axes[0].quiet_stability_passed=false;
assert.equal(vm.runInContext('gainCandidateDeployable(candidate)',context),true,'Complete but unmet quiet measurements retain warning-application semantics');
context.candidate.drag_validation.axes[0].quiet_stability_measured=false;
assert.equal(vm.runInContext('gainCandidateDeployable(candidate)',context),false,'An unmeasured quiet scope is unavailable even when aggregate flags or legacy fields remain');
// Match the terminal's heterogeneous experiment records: parameter rows have
// no measurement conclusion; disturbance axes alternate FRF and recovery rows.
Object.assign(context,{t:key=>key,localizedError:String,gainProbeMessageLabel:String,gainPurposeLabel:String,
  jointDisplayValue:value=>typeof value === 'number' && Number.isFinite(value) ? String(value) : '—',
  gainPathValidationHtml:()=>'',gainTaughtPathMetricsHtml:()=>'',gainObserverAndBoundsHtml:()=>'',
  gainQualificationScopeHtml:()=>'',selectedGainPurposes:()=>['servo_tracking'],setupActionHtml:()=>'',gainCalibrationPhaseText:()=>'任务已结束'});
context.state.calibrationRuntimeCandidates={};
for (const name of ['busyText','canonicalGainPurpose','gainPurposeLabel','capabilityNumber','gainProbePhaseLabel','gainProbeStageLabel','gainProbeMessageLabel','gainProbeDiagnostic','gainProbeStatusValues','gainProbeGhostSource','gainCheckStatus','gainCompliantScopeSummaryHtml','gainPurposeDirectionEvidenceHtml','gainEvidenceRecordHtml','bindGainEvidencePaths','gainCompliantScopeEvidenceHtml','gainDirectDisturbanceScope','gainMeasurementSourceKey','gainPurposeMeasurementEvidence','gainPurposeMeasurementWarnings','gainDirectDisturbanceEvidenceHtml','gainDragScopes','gainDragScopeSummaryHtml','gainDragScopeEvidenceHtml','gainCalibrationQualityWarning','gainTaskConclusion','gainMeasurementPhaseLabel','gainMeasurementReasonLabel','gainWarningLabel','gainCandidateAvailabilityLabel','gainCandidateResultCounts','gainCandidateParameterSourceHtml','gainCandidateParametersHtml','gainModelInitializationHtml','gainResponseModelsHtml','gainMotionEstablishmentHtml','gainFixedReferenceEvidenceHtml','gainExperimentHtml','gainTaughtPathMetricsHtml','gainCandidateReviewHtml'])vm.runInContext(extract(name),context);
assert(vm.runInContext("gainProbeMessageLabel('bounded_endpoint_transition')",context).includes('最新实测位置平滑过渡'));
assert(vm.runInContext("gainProbeStageLabel('trajectory_discovery')",context).includes('沿示教路径建立响应'));
for(const message of ['taught_path_original_speed_discovery','taught_path_feedback_bounded_discovery','taught_path_candidate_window','taught_path_discovery_complete']) {
  context.message=message;
  for(const language of ['zh-CN','en']){context.state.language=language;assert(!vm.runInContext('gainProbeMessageLabel(message)',context).includes(message),'The taught-path discovery stage has an explanatory label in both languages');}
}
context.state.language='zh-CN';
assert(vm.runInContext("gainProbeMessageLabel('taught_path_original_speed_discovery')",context).includes('1×')&&vm.runInContext("gainProbeMessageLabel('taught_path_original_speed_discovery')",context).includes('不额外减速'),'The current discovery message reports original-speed execution while preserving historical governor labels');
assert(vm.runInContext("gainProbeMessageLabel('taught_path_candidate_window')",context).includes('仍需固定参数整轨分析'),'Segment candidate comparison cannot be displayed as final frozen-gain acceptance');
context.dragScope={available:true,completed:true,validated:true,record_path:'drag-quiet.csv',axes:[{joint:'J1',quiet_stability_measured:true,quiet_stability_passed:true,quiet_position_span_rad:.0001,quiet_speed_rad_s:.0002,release_decay_measured:false,release_decay_passed:false,decay_s:0,validation_scope:'zero_stiffness_quiet_stability_only_release_unmeasured'}]};
const quietScopeHtml=vm.runInContext("gainDragScopeEvidenceHtml(dragScope,['J1'])",context);
assert(quietScopeHtml.includes('data-gain-drag-scopes')&&quietScopeHtml.includes('无外力稳定')&&quietScopeHtml.includes('释放衰减')&&quietScopeHtml.includes('<td>未测量</td><td>—</td>')&&quietScopeHtml.includes('drag-quiet.csv'),'Quiet validation never promotes an unexcited release or its default decay value to a measured result');
const quietScopeSummary=vm.runInContext('gainDragScopeSummaryHtml(dragScope)',context);
assert(quietScopeSummary.includes('<strong>已测量</strong>')&&quietScopeSummary.includes('释放衰减：未测量'),'A completed quiet scope remains visible separately from missing release evidence');
context.dragScope.axes.push({joint:'J2',quiet_stability_measured:true,quiet_stability_passed:false,release_decay_measured:true,release_decay_passed:true,decay_s:.7});
assert.equal(vm.runInContext('gainDragScopes(dragScope).quiet.passed',context),false,'One failed joint keeps the all-axis quiet scope from passing');
assert.equal(vm.runInContext('gainDragScopes(dragScope).release.measured',context),false,'All participating joints need actual release evidence for an all-axis measured release scope');
Object.assign(context.dragScope.axes[0],{release_decay_measured:true,release_decay_passed:true,decay_s:.6});
context.dragScope.axes[1].quiet_stability_passed=true;
assert(vm.runInContext('gainDragScopes(dragScope).quiet.passed&&gainDragScopes(dragScope).release.passed',context),'Both measured all-axis scopes may pass independently');
Object.assign(context.dragScope.axes[0],{preparation_motion_observed:true,release_motion_observed:false,fast_stop_observed:true,decay_time_is_upper_bound:true,decay_time_upper_bound_s:.025,decay_s:0,reason:'drag_release_fast_stop_upper_bound_recorded'});
const fastStopRecord=JSON.stringify(context.dragScope);
for(const language of ['zh-CN','en']){
 context.state.language=language;
 const html=vm.runInContext('gainDragScopeEvidenceHtml(dragScope)',context);
 assert(html.includes('data-gain-stop-upper-bound>≤ 0.025')&&html.includes(language==='en'?'Stopping-time upper bound':'停止时间上界'),'Fast release records a stopping-time upper bound rather than an exact zero decay');
 assert(html.includes(language==='en'?'Motion before release':'释放前运动')&&html.includes(language==='en'?'Motion after release':'释放后运动'),'Preparation and post-release motion remain distinct evidence');
 assert(html.includes('<td>0.7</td>'),'A historical measured decay without a bound flag retains its recorded value');
}
assert.equal(JSON.stringify(context.dragScope),fastStopRecord,'Rendering does not rewrite fast-stop evidence');
context.dragScope.axes[0].decay_time_upper_bound_s=null;
assert(!vm.runInContext('gainDragScopeEvidenceHtml(dragScope)',context).includes('data-gain-stop-upper-bound'),'A missing upper bound cannot fall back to an exact default decay');
context.dragScope.axes[0].decay_time_upper_bound_s=.025;context.dragScope.axes[0].release_decay_measured=false;
assert(!vm.runInContext('gainDragScopeEvidenceHtml(dragScope)',context).includes('data-gain-stop-upper-bound'),'An unmeasured release cannot display a stale bound as measured');
context.state.language='zh-CN';
context.dragScope={available:true,completed:true,validated:true,axes:[{joint:'J1',passed:true,decay_s:0}]};
assert.equal(vm.runInContext('gainDragScopes(dragScope)',context),null);
assert.equal(vm.runInContext('gainDragScopeEvidenceHtml(dragScope)',context),'');
assert(vm.runInContext('gainDragScopeSummaryHtml(dragScope)',context).includes('历史拖拽测量'),'Historical aggregate success does not invent either new quiet or release scope');
context.compliantScope={record_path:'compliant-disturbance.csv',axes:[{joint:'J1',measurement_complete:true,input_measurement_complete:true,passed:true,yield_observed:true,stable_recovery:true,peak_displacement_rad:.01,displacement_limit_rad:.02,initial_reference_error_rad:.03,final_equilibrium_error_rad:.001,release_recovery_s:.5,final_drift_rad:.0001,reason:'bounded_compliant_yield_and_stable_recovery_qualified',validation_scope:'known_joint_torque_yield_and_unloaded_equilibrium_recovery_not_contact_force'}]};
const compliantScopeHtml=vm.runInContext("gainCompliantScopeEvidenceHtml(compliantScope,['J1'])",context);
assert(compliantScopeHtml.includes('data-gain-compliant-scope')&&compliantScopeHtml.includes('最终平衡误差 °')&&compliantScopeHtml.includes('初始参考误差 °')&&compliantScopeHtml.includes('独立输入已测')&&compliantScopeHtml.includes('已测得柔顺位移及稳定恢复')&&compliantScopeHtml.includes('compliant-disturbance.csv'),'Compliant use reports actual independent yielding and equilibrium recovery separately from reference tracking');
assert(vm.runInContext('gainCompliantScopeSummaryHtml(compliantScope)',context).includes('<strong>已测量</strong>'));
context.compliantScope.axes[0].measurement_complete=false;
assert(vm.runInContext('gainCompliantScopeSummaryHtml(compliantScope)',context).includes('测量未完成')&&!vm.runInContext('gainCompliantScopeSummaryHtml(compliantScope)',context).includes('<strong>已测量</strong>'),'An old or inconsistent passed flag cannot complete an unmeasured compliant scope');
assert.equal(vm.runInContext('gainCompliantScopeEvidenceHtml({validated:true})',context),'','Legacy tracking success does not supply missing compliant-use evidence');
context.compliantExperiment={kind:'independent_torque_disturbance',kp:[8],kd:[.4],axes:[],compliant_use_axes:context.compliantScope.axes};
assert(vm.runInContext("gainExperimentHtml(compliantExperiment,['J1'])",context).includes('data-gain-compliant-scope'),'Live typed compliant experiment records use the same purpose evidence renderer');
for(const reason of ['zero_stiffness_quiet_stability_qualified_release_separate','drag_release_decay_unmeasured:joint5','drag_release_motion_unresolved_quiet_result_remains_available','drag_release_fast_stop_upper_bound_recorded','drag_release_actual_input_or_timing_unknown','drag_release_reference_velocity_not_zero','drag_input_at_impulse_and_protocol_domain_ceiling','drag_input_retry_requires_complete_known_unlimited_input','drag_known_unresolved_input_bounded_increase','drag_pulse_stopped_at_measured_domain_margin','zero_stiffness_bias_drift_requires_feedforward_review','bounded_compliant_yield_and_stable_recovery_qualified','compliant_yield_unresolved_do_not_substitute_rigid_tracking','compliant_unloaded_equilibrium_recovery_incomplete']){
 context.reason=reason;for(const language of ['zh-CN','en']){context.state.language=language;assert(!vm.runInContext('gainWarningLabel(reason)',context).includes(reason));}
}
context.state.language='zh-CN';
context.pathSegment={kind:'taught_path_segment_candidate',window:2,source_time_s:1.2,admitted:true,measured:false,requires_frozen_path_validation:true,kp:[8,12],kd:[.4,.5],reasons:['measured_taught_segment_tracking_deficit_small_stiffness_only_comparison','segment_feedback_gains_still_changing'],axes:[{joint:'J1',valid:true,constant_gains:true,samples:200,rms_error_rad:.002,peak_error_rad:.004},{joint:'J2',valid:true,constant_gains:false,samples:200,rms_error_rad:.001,peak_error_rad:.002}]};
const segmentHtml=vm.runInContext("gainExperimentHtml(pathSegment,['J1','J2'])",context);
assert(segmentHtml.includes('data-gain-taught-path="segment"')&&segmentHtml.includes('本窗含增益过渡')&&segmentHtml.includes('小步比较较高 Kp')&&segmentHtml.includes('仍待分析')&&!segmentHtml.includes('验收通过'),'Segment diagnostics show measured error and changing gains without promoting the candidate to validation');
context.pathDiscovery={kind:'taught_path_discovery',completed:false,validated:false,source_time_s:1.2,source_duration_s:4,elapsed_s:2.5,windows:2,gain_revisions:1,kp:[8,12],kd:[.4,.5],record_path:'gain-trajectory-native-discovery.csv'};
const discoveryHtml=vm.runInContext("gainExperimentHtml(pathDiscovery,['J1','J2'])",context);
assert(discoveryHtml.includes('data-gain-taught-path="discovery"')&&discoveryHtml.includes('未完成，部分记录已保留')&&discoveryHtml.includes('gain-trajectory-native-discovery.csv')&&discoveryHtml.includes('固定整组参数完成整轨测量'),'Partial path discovery retains its scope and recording');
context.pathDiscovery.discovery_peak_error_rad=.02;context.pathDiscovery.final_peak_error_rad=Math.PI/1800;
context.pathDiscovery.playback_rate=1;context.pathDiscovery.reference_schedule='control_cycle_monotonic';
const actualDiscoveryTargets=vm.runInContext("gainExperimentHtml(pathDiscovery,['J1','J2'])",context);
assert(!actualDiscoveryTargets.includes('建立响应门槛 °')&&!actualDiscoveryTargets.includes('最终整轨峰值目标 °'),'Historical accuracy thresholds are no longer presented as active requirements');
assert(actualDiscoveryTargets.includes('本次回放速度')&&actualDiscoveryTargets.includes('1×')&&actualDiscoveryTargets.includes('控制周期单调时钟'),'Original-speed execution and reference timing are displayed only when recorded');
assert(!discoveryHtml.includes('建立响应门槛 °'),'Historical records without an actual discovery threshold do not acquire a guessed default');
for(const message of ['taught_path_waiting_for_soft_feedback_recovery','taught_path_braking_for_soft_feedback_recovery','taught_path_soft_feedback_recovered']){
  context.message=message;for(const language of ['zh-CN','en']){context.state.language=language;assert(!vm.runInContext('gainProbeMessageLabel(message)',context).includes(message));}
}
context.state.language='zh-CN';
for(const reason of ['complete_taught_segment_measurement','insufficient_complete_segment_measurement','segment_feedback_gains_still_changing','limited_segment','discontinuous_segment_feedback','velocity_feedback_noise_dominates_pd','uncommanded_segment_reversals','measured_segment_velocity_feedback_noise_reduce_damping_only','measured_segment_uncommanded_reversals_reduce_stiffness_only','segment_timing_insufficient_for_feedback_increase','measured_taught_segment_unresolved_motion_bounded_stiffness_only_not_hardware_limit','measured_taught_segment_tracking_deficit_small_stiffness_only_comparison','segment_requires_no_supported_gain_increase','rejected_segment_candidate_early_oscillation_rollback','taught_path_discovery_control_mode_rejected','taught_path_discovery_feedback_unavailable','taught_path_discovery_reserved_budget_reached','taught_path_discovery_interrupted','taught_path_discovery_interrupted_or_partial']) {
  context.reason=reason;for(const language of ['zh-CN','en']){context.state.language=language;assert(!vm.runInContext('gainWarningLabel(reason)',context).includes(reason));}
}
context.state.language='zh-CN';
assert(vm.runInContext("gainProbeStageLabel('bounded_endpoint_transition')",context).includes('自动过渡'));
assert(vm.runInContext("gainProbePhaseLabel('feedback_bounded_endpoint_transition')",context).includes('反馈约束'));
for(const reason of ['gain_endpoint_transition_insufficient_response','gain_endpoint_transition_insufficient_response:joint3','gain endpoint transition interrupted','gain endpoint transition needs fresh measured origin','gain endpoint transition lost continuous feedback','gain endpoint transition feedback mapping invalid','gain endpoint transition needs fresh per-joint feedback']){
  context.reason=reason;
  for(const language of ['zh-CN','en']){context.state.language=language;const text=vm.runInContext('gainWarningLabel(reason)',context);assert(!text.includes(reason.split(':')[0]));if(reason.includes(':joint3'))assert(text.includes('joint3'));}
}
context.state.language='zh-CN';
context.transition={kind:'feedback_bounded_endpoint_transition',completed:false,elapsed_s:.8,kp:[3],kd:[.2],initial_reference_source:'fresh_measured_endpoint_observation',initial_reference_rad:[.001],target_rad:[0],reference_error_cap_rad:[.01],max_velocity_rad_s:[.05],max_acceleration_rad_s2:[.2],record_path:'gain-transition-native.csv'};
const transition=vm.runInContext("gainExperimentHtml(transition,['J1'])",context);
assert(transition.includes('端点自动过渡证据')&&transition.includes('最新实测位置')&&transition.includes('未完成，证据保留')&&transition.includes('参考误差约束 °')&&transition.includes('gain-transition-native.csv')&&!transition.includes('通过'),'Endpoint transition evidence remains a separate incomplete transition, not a qualification result');
context.transition.initial_reference_source='fresh_measured_feedback';
assert(vm.runInContext("gainExperimentHtml(transition,['J1'])",context).includes('最新实测位置'),'The final native fresh_measured_feedback source is displayed alongside the historical endpoint-observation value');
assert(vm.runInContext("gainProbeStageLabel('trajectory_endpoint_transition')",context).includes('全轴端点自动过渡'));
const endpointReasons=['endpoint_reference_ready','endpoint_reference_dwell','endpoint_feedback_waiting','gain_endpoint_transition_reference_timeout','gain_endpoint_transition_feedback_wait_timeout','gain_endpoint_transition_insufficient_reserved_budget','gain_endpoint_transition_execution_limited','gain_endpoint_transition_feedback_unavailable','gain_endpoint_transition_feedback_stale','gain_endpoint_transition_interrupted'];
for(const reason of endpointReasons){context.reason=reason;for(const language of ['zh-CN','en']){context.state.language=language;assert(!vm.runInContext('gainWarningLabel(reason)',context).includes(reason));}if(reason.startsWith('gain_'))assert(!translateError(reason).includes(reason));}
context.state.language='zh-CN';
assert(vm.runInContext("gainWarningLabel('gain_endpoint_transition:path_position_uncertainty_consumes_feedback_lead')",context).includes('内部制动预留'),'Native endpoint prefix preserves the actual path-helper reason');
context.liveTransition={...context.transition,stage:'trajectory_endpoint_transition',joint:'all',available:false,completed:false,reason:'bounded_endpoint_transition',elapsed_s:.8,timeout_s:2,initial_reference_rad:[.001,.002],target_rad:[0,.002],reference_rad:[.0005,.002],actual_rad:[.0007,.0021],reference_arrived:true,measured_samples:[72,70],kp:[3,4],kd:[.2,.3],record_path:'gain-transition-native-frozen.csv',validation_scope:'positioning_only_not_identification_or_purpose_qualification'};
const liveTransition=vm.runInContext("gainExperimentHtml(liveTransition,['J1','J2'])",context);
assert(liveTransition.includes('全部关节')&&liveTransition.includes('本次当前参考 °')&&liveTransition.includes('本次实际反馈 °')&&liveTransition.includes('本次过渡各轴实际采样数')&&liveTransition.includes('J1: 72')&&liveTransition.includes('J2: 70')&&liveTransition.includes('过渡最长用时 s'));
assert(liveTransition.includes('参考生成已到终点')&&liveTransition.includes('未完成，证据保留')&&!liveTransition.includes('已完成定位过渡')&&liveTransition.includes('本次样本与整轨响应测量、用途分析分别记录'),'Reference arrival and transition samples never become full-path or purpose qualification');
context.state.calibrationToolSelected={gain:null};context.state.telemetry={joint_names:['J1','J2']};
context.transitionStatus={workflow_kind:'gain',task_id:'live-endpoint',phase:'gain_adaptation',gain_probe_stage:'trajectory_frozen',gain_probe_joint:'J1',gain_probe_amplitude_rad:.75,gain_probe_frequency_hz:[17],gain_probe_elapsed_s:888,gain_probe_duration_s:999,gain_candidate_index:9,gain_probe_kp:444,gain_probe_kd:4.5,gain_probe_reference_joint_positions:[.75,.8],gain_probe_diagnostic:context.liveTransition};
const endpointValues=vm.runInContext('gainProbeStatusValues(transitionStatus)',context);
assert(endpointValues.phase.includes('全轴端点自动过渡')&&endpointValues.joint==='全部关节'&&endpointValues.amplitude==='0.057°'&&endpointValues.amplitudeLabel==='端点间最大位移','Actual transition evidence overrides stale full-path stage, single-joint and amplitude metadata');
assert(endpointValues.frequency==='—'&&endpointValues.candidate==='—'&&endpointValues.gains==='—'&&endpointValues.window.includes('0.8 s')&&endpointValues.window.includes('2.0 s')&&!endpointValues.window.includes('888'),'Transition progress uses its own clock and suppresses cached frequency, candidate and single-joint gains');
assert.deepEqual(Array.from(vm.runInContext('gainProbeGhostSource(transitionStatus).q',context)),[.0005,.002],'The preview uses the actual transition reference instead of the old full-path pose');
context.transitionStatus.gain_probe_diagnostic={kind:'feedback_bounded_endpoint_transition',elapsed_s:.8};
const missingTransition=vm.runInContext('gainProbeStatusValues(transitionStatus)',context);
assert(missingTransition.amplitude==='—'&&missingTransition.fraction===null&&missingTransition.window.includes('最长 — s')&&vm.runInContext('gainProbeGhostSource(transitionStatus)',context)===null,'Missing transition fields do not fall back to unrelated full-path values');
context.transitionStatus.gain_probe_diagnostic=context.liveTransition;
context.oldTransitionStatus={task_id:'endpoint-history',directory:'/offline/endpoint-history',phase:'complete',termination:{kind:'insufficient_measurement',reason:'identification_insufficient:gain_endpoint_transition_insufficient_response'},gain_probe_diagnostic:{stage:'trajectory_frozen',kind:'native-frozen',record_path:'gain-trajectory-native-frozen.csv',samples:0}};
context.state.calibrationToolSelected.gain={directory:'/offline/endpoint-history',gain_candidate:{task_id:'endpoint-history',experiments:[context.transition]}};
assert(vm.runInContext('gainProbeDiagnostic(oldTransitionStatus).record_path',context)==='gain-transition-native.csv','Matching historical transition records replace the known stale full-path diagnostic only for an endpoint-transition termination');
assert(vm.runInContext('gainProbeStatusValues(oldTransitionStatus).pathsLabel',context).includes('历史整轨调度计数'),'The known historical counter incremented before transition is not presented as proof of completed full-path execution');
context.state.calibrationToolSelected.gain.directory='/offline/other-history';
assert(vm.runInContext('gainProbeDiagnostic(oldTransitionStatus).record_path',context)==='gain-trajectory-native-frozen.csv','A record from another directory never supplies transition evidence');
context.state.calibrationToolSelected.gain=null;
Object.assign(context,{priorityErrors:[],reportedCalibrationErrors:new Set(),lastRuntimeFaultKey:'',lastTeachReplayErrorKey:'',showPriorityError:value=>context.priorityErrors.push(value),describeRuntimeFault:()=>({code:'SAFETY_FAILED'}),gainCalibrationFailure:()=>({code:'EXECUTION_FAULT'})});
vm.runInContext(extract('checkRuntimeAlerts'),context);
context.qualityStatus={workflow_kind:'gain',task_id:'quality-warning',phase:'failed',error:'identification_insufficient:gain_endpoint_transition_reference_timeout',termination:{kind:'insufficient_measurement',reason:'identification_insufficient:gain_endpoint_transition_reference_timeout',fault_interrupted:false}};
assert(vm.runInContext('gainCalibrationQualityWarning(qualityStatus)',context));
assert(vm.runInContext('gainTaskConclusion(qualityStatus).title',context).includes('待补测'),'A structured non-Safety shortfall retains a neutral terminal review even in historical failed phases');
vm.runInContext("checkRuntimeAlerts({robot_state:'ACTIVE',model_calibration:qualityStatus})",context);assert.equal(context.priorityErrors.length,0,'Non-Safety measurement shortfalls do not produce red execution-fault popups');
context.qualityStatus.termination.kind='execution_fault';vm.runInContext("checkRuntimeAlerts({robot_state:'ACTIVE',model_calibration:qualityStatus})",context);assert.equal(context.priorityErrors.length,1,'A declared actual execution fault still produces the existing fault prompt');
context.qualityStatus.fault_interrupted=true;context.qualityStatus.termination.kind='insufficient_measurement';vm.runInContext("checkRuntimeAlerts({robot_state:'FAULT',fault:{code:'SAFETY_FAILED'},model_calibration:qualityStatus})",context);assert.equal(context.priorityErrors.length,2,'Robot Safety faults remain visible regardless of an unfinished measurement result');
assert(vm.runInContext("gainProbeStageLabel('measurement_center_entry')",context).includes('沿示教路径进入测量区'));
assert(vm.runInContext("gainProbeStageLabel('measurement_center_return')",context).includes('沿示教路径返回端点'));
assert(vm.runInContext("gainProbeMessageLabel('demonstrated_path_measurement_entry')",context).includes('最新实测位置沿示教路径'));
assert(vm.runInContext("gainProbeMessageLabel('measurement_center_reached_remeasurement_required')",context).includes('需重新测量响应'));
assert(vm.runInContext("gainProbePhaseLabel('demonstrated_measurement_center_entry')",context).includes('沿示教路径进入测量区'));
for(const reason of ['measurement_center_mapping_invalid','measurement_center_fresh_position_invalid','measurement_center_demonstrated_path_invalid','measurement_center_bounds_invalid','measurement_center_fresh_position_outside_safety','measurement_center_outside_existing_endpoint_alignment','measurement_center_no_demonstrated_safety_intersection','measurement_center_no_inward_entry_within_existing_alignment','measurement_center_no_inward_route_within_existing_alignment','demonstrated_path_has_no_requested_bidirectional_room','earliest_priority_measurement_room_on_demonstrated_path','partial_measurement_room_on_demonstrated_path','path_progress_input_invalid','path_progress_mapping_invalid','path_progress_feedback_or_limits_invalid','path_progress_state_outside_segment','path_reference_outside_feedback_lead','path_feedback_lead_below_required_braking_distance','path_feedback_lead_cannot_be_retained_with_acceleration_limit','demonstrated_path_target_reached','demonstrated_path_stationary_node','feedback_bounded_demonstrated_path_progress','demonstrated_path_progress']){
  context.reason=reason;
  for(const language of ['zh-CN','en']){context.state.language=language;const text=vm.runInContext('gainWarningLabel(reason)',context);assert(!text.includes(reason)&&!text.includes('扩大 URDF')&&!text.includes('精确回到'));
  }
}
context.state.language='zh-CN';
for(const reason of measurementReturnReasons){
  context.reason=reason;
  for(const language of ['zh-CN','en']){context.state.language=language;assert(!vm.runInContext('gainWarningLabel(reason)',context).includes(reason));}
}
for(const reason of measurementReserveReasons){
  context.reason=reason;
  for(const language of ['zh-CN','en']){context.state.language=language;assert(!vm.runInContext('gainWarningLabel(reason)',context).includes(reason));}
}
context.state.language='zh-CN';
assert.equal(vm.runInContext("gainWarningLabel('identification_insufficient:measurement_center_entry_insufficient_reserved_budget')",context),vm.runInContext("gainWarningLabel('measurement_center_entry_insufficient_reserved_budget')",context),'Structured identification prefixes do not hide the concrete reserved-budget failure');
assert(vm.runInContext("gainWarningLabel('fresh_projection_then_reverse_original_demonstrated_route')",context).includes('已规划'),'A valid return plan is not represented as a completed physical return');
assert(vm.runInContext("gainWarningLabel('demonstrated_path_target_reached')",context).includes('辨识与用途分析尚待完成'),'Reaching the measurement area is separate from identification and use-specific qualification');
assert(vm.runInContext("gainWarningLabel('native: path_reference_outside_feedback_lead:joint3')",context).includes('joint3'),'Measurement-area path reasons remain readable inside existing prefixed diagnostics');
for(const reason of ['measurement_center_route_unavailable','measurement_center_entry_insufficient_reserved_budget','measurement_center_entry_response_or_budget_insufficient','measurement_center_entry_control_mode_rejected','measurement_center_entry_feedback_unavailable','measurement_center_entry_feedback_stale','measurement_center_entry_no_further_calibrated_input','measurement_center_entry_execution_limited','measurement_center_entry_gain_step_consumes_feedback_lead','measurement_center_entry_gain_step_rejected','measurement_center_entry_interrupted','measurement_center_entry_interrupted_or_insufficient_response','measurement_center_reached_remeasurement_required','measurement_center_evidence_write_failed','measured_one_way_path_response_bounded_kp_step']){
  context.reason=reason;
  for(const language of ['zh-CN','en']){context.state.language=language;assert(!vm.runInContext('gainWarningLabel(reason)',context).includes(reason));}
}
context.state.language='zh-CN';
context.measurementEntry={kind:'demonstrated_measurement_center_entry',completed:false,reason:'feedback_bounded_demonstrated_path_progress',direction:'toward_measurement_region',validation_scope:'positioning_only_not_identification_or_purpose_qualification',counted_in_budget:true,elapsed_s:1.2,geometric_cost_s:2.4,entry_source_index:12,target_source_index:17,reverse:false,current_segment:1,segment_count:3,fresh_origin_rad:[.01,-.02],selected_center_rad:[.04,-.03],desired_room_rad:[.015,.01],available_room_rad:[.02,.012],reference_rad:[.013,-.022],initial_kp:[2,4],initial_kd:[.2,.4],kp:[3,4],kd:[.2,.4],source_indices:[null,12,17],gain_steps:[{time_s:.8,reason:'measured_one_way_path_response_bounded_kp_step',actual_rad:[.011,-.021],reference_rad:[.013,-.022],kp:[3,4],kd:[.2,.4]}],record_path:'gain-transition-measurement-entry.csv'};
const measurementEntry=vm.runInContext("gainExperimentHtml(measurementEntry,['Shoulder','Elbow'])",context);
assert(measurementEntry.includes('沿示教路径进入测量区的证据')&&measurementEntry.includes('定位尚未完成')&&measurementEntry.includes('不是已识别机械硬限')&&measurementEntry.includes('计入本次标定既有总预算'));
assert(measurementEntry.includes('Shoulder')&&measurementEntry.includes('Elbow')&&measurementEntry.includes('0.57296')&&measurementEntry.includes('2.29183')&&measurementEntry.includes('0.63025'),'Mapped joints use actual native radian fields and convert only their displayed values');
assert(measurementEntry.includes('起步整组增益')&&measurementEntry.includes('本次整组增益')&&measurementEntry.includes('调整后整组增益')&&measurementEntry.includes('Kp 2')&&measurementEntry.includes('Kp 3')&&measurementEntry.includes('实际位置 °')&&measurementEntry.includes('gain-transition-measurement-entry.csv'));
assert.equal((measurementEntry.match(/单向路径跟随响应不足，已按实测条件小步调整 Kp/g)||[]).length,1,'One complete-set adjustment reason is not repeated as a separate failed response on unchanged joints');
Object.assign(context.measurementEntry,{max_velocity_rad_s:[.05,.04],max_acceleration_rad_s2:[.2,.15],feedback_lead_rad:[.01,.005],position_uncertainty_rad:[.001,.002]});
const entryLimits=vm.runInContext("gainExperimentHtml(measurementEntry,['Shoulder','Elbow'])",context);
assert(entryLimits.includes('参考速度上限 °/s')&&entryLimits.includes('参考加速度上限 °/s²')&&entryLimits.includes('允许参考跟随距离 °')&&entryLimits.includes('参考跟随距离内制动预留 °')&&entryLimits.includes('不是已测位置精度')&&entryLimits.includes('Shoulder: 2.86479')&&entryLimits.includes('Elbow: 0.28648')&&entryLimits.includes('gain-transition-measurement-entry.csv'),'Optional native execution constraints retain mapped joints, units and CSV evidence without representing braking reserve as measured accuracy');
Object.assign(context.measurementEntry,{actual_rad:[.0117,-.0212],reference_error_rad:[.0013,-.0008],observed_position_step_rad:[0,.001],position_reserve_source:['unidentified_position_step_internal_guard_buffer','observed_position_step_and_stationary_noise_reserve'],unknown_resolution_guard_fraction:.25});
const reserveEntry=vm.runInContext("gainExperimentHtml(measurementEntry,['Shoulder','Elbow'])",context);
assert(reserveEntry.includes('本次实际反馈位置 °')&&reserveEntry.includes('本次参考减实测误差 °')&&reserveEntry.includes('Shoulder: 0.67036')&&reserveEntry.includes('Elbow: -0.04584'),'Live measured positions and signed reference errors use their actual reported fields');
assert(reserveEntry.includes('已观测最小位置步进 °')&&reserveEntry.includes('Shoulder: 未测得（未知）；Elbow: 0.0573')&&reserveEntry.includes('最小位置步进尚未知，按跟随距离比例设置内部制动预留')&&reserveEntry.includes('25%')&&reserveEntry.includes('不是协议声明的编码分辨率'),'An unobserved position step is explicitly unknown, with the declared internal guard fraction separate from observed feedback');
assert(reserveEntry.includes('title="unidentified_position_step_internal_guard_buffer;observed_position_step_and_stationary_noise_reserve"'),'Original per-joint guard sources remain available');
delete context.measurementEntry.unknown_resolution_guard_fraction;
assert(!vm.runInContext("gainExperimentHtml(measurementEntry,['Shoulder','Elbow'])",context).includes('位置步进未知时的内部预留比例'),'Historical omission does not invent a default guard fraction');
assert(vm.runInContext("gainProbePhaseLabel('native-measurement-center-noise')",context).includes('测量区静止噪声'));
vm.runInContext(extract('gainProbeEvidenceHtml'),context);
context.centerNoise={task_id:'center-task',gain_probe_diagnostic:{stage:'trajectory_noise',kind:'native-measurement-center-noise',completed:true,reason:'noise_measured',record_path:'gain-native-measurement-center-noise.csv'}};
assert(vm.runInContext('gainProbeEvidenceHtml(centerNoise)',context).includes('gain-native-measurement-center-noise.csv'),'Noise rerecording after path entry retains its actual native CSV path');
context.centerNoise.gain_probe_diagnostic.axes=[{joint:'Shoulder',encoder_step_rad:0,effective_encoder_step_rad:.001,encoder_step_observed:true,stationary:true,samples:250,measured_rms_rad:0,noise_rms_rad:.0001},{joint:'Elbow',encoder_step_rad:0,effective_encoder_step_rad:.002,encoder_step_observed:false,stationary:true,samples:250,measured_rms_rad:0,noise_rms_rad:0}];
const observedNoise=vm.runInContext('gainProbeEvidenceHtml(centerNoise)',context);
assert(observedNoise.includes('data-gain-noise-evidence')&&observedNoise.includes('本窗最小实测位置步进 °')&&observedNoise.includes('Shoulder: 未测得（未知）；Elbow: 未测得（未知）')&&observedNoise.includes('本轮有效实测位置步进 °')&&observedNoise.includes('Shoulder: 0.057296；Elbow: 未测得（未知）'),'A raw zero in stationary data stays unknown while an explicitly observed effective step can come from earlier fresh motion');
assert(!observedNoise.includes('Elbow: 0.114592')&&observedNoise.includes('0 表示未知')&&observedNoise.includes('gain-native-measurement-center-noise.csv')&&!observedNoise.includes('已达标'),'A false observation flag never turns a stale numeric step into measured precision or qualification');
delete context.centerNoise.gain_probe_diagnostic.axes[0].encoder_step_observed;
assert(vm.runInContext('gainProbeEvidenceHtml(centerNoise)',context).includes('Shoulder: 本记录未确认'),'Missing observation flags remain unconfirmed in old records');
assert(measurementEntry.includes('最新实测起步、12、17')&&measurementEntry.includes('已完成路径段')&&!measurementEntry.includes('NaN')&&!measurementEntry.includes('undefined'));
context.measurementEntry.completed=true;context.measurementEntry.reason='measurement_center_reached_remeasurement_required';
const centerReached=vm.runInContext("gainExperimentHtml(measurementEntry,['Shoulder','Elbow'])",context);
assert(centerReached.includes('已进入测量区')&&centerReached.includes('需重新测量响应')&&!centerReached.includes('已达标'),'Successful positioning never becomes identification or gain-use qualification');
context.measurementEntry.direction='return_to_demonstrated_endpoint';context.measurementEntry.reverse=true;
assert(vm.runInContext("gainExperimentHtml(measurementEntry,['Shoulder','Elbow'])",context).includes('已返回示教端点'),'The same native entry record supports the actual return direction without claiming inward movement');
context.measurementEntry.completed=false;context.measurementEntry.reason='measurement_center_entry_interrupted_or_insufficient_response';
assert(vm.runInContext("gainExperimentHtml(measurementEntry,['Shoulder','Elbow'])",context).includes('本次移动已结束，定位未完成，证据保留'));
context.emptyEntry={kind:'demonstrated_measurement_center_entry',completed:false,reason:'measurement_center_entry_feedback_unavailable'};
const emptyEntry=vm.runInContext("gainExperimentHtml(emptyEntry,['Shoulder','Elbow'])",context);
assert(!emptyEntry.includes('Kp 0')&&!emptyEntry.includes('NaN')&&!emptyEntry.includes('undefined'),'Absent feedback or gain fields are not filled with zero or a planning value');
assert(vm.runInContext("gainProbeStageLabel('frequency_response_identification')",context).includes('内部位置频响辨识'));
assert(vm.runInContext("gainProbeMessageLabel('isolated_frequency_response')",context).includes('其余轴固定参考'));
for(const reason of ['excitation_amplitude_calibration_increase_reference_drive_at_fixed_gains','excitation_amplitude_calibration_bounded_kp_step_for_unresolved_motion','excitation_amplitude_calibration_slower_single_tone_measurement','excitation_amplitude_calibration_insufficient_reserved_budget','excitation_amplitude_calibration_no_legal_input_growth_in_observed_domain','excitation_calibration_deferred_for_oscillation_feedback_reduction']){
  context.reason=reason;
  for(const language of ['zh-CN','en']){context.state.language=language;const text=vm.runInContext('gainWarningLabel(reason)',context);assert(!text.includes(reason)&&!text.includes('硬件能力用尽'));
  }
}
context.state.language='zh-CN';
const frequencyPoints=[.2,.5,1.1].map((hz,index)=>({hz,coherence:.981+index*.001,half_relative_error:.042+index*.001,velocity_half_relative_error:.061+index*.001,loop_input_amplitude_nm:.12,output_amplitude_rad:.003}));
context.frf={kind:'frequency_response_pd_design',design_target_crossover_hz:.7,design_target_phase_margin_deg:60,minimum_phase_margin_deg:50,validation_required:true,candidate_requires_complete_group_comparison:true,kp:[4,5,6],kd:[.3,.4,.5],axes:[
  {joint:'J1',axis_index:0,measurement_complete:true,record_path:'frf-j1.csv',candidate_valid:true,candidate_kp:4,candidate_kd:.3,reason:'requested_crossover_outside_measured_frequency_band',missing_evidence:['higher_frequency_plant_evidence'],response:{valid:true,isolated_axis_excitation:true,fresh_samples:1800,duplicate_feedback:3,points:frequencyPoints,reason:'empirical_mit_plant_measured_in_sampled_host_frequency_band'},measured_loop:{valid:true,crossover_covered:true,crossover_hz:.4123,phase_margin_deg:57.4},predicted_candidate_loop:{valid:true,crossover_covered:false,crossover_hz:123.456,phase_margin_deg:999.87,reason:'unity_gain_crossover_not_bracketed',missing_evidence:['higher_or_denser_frequency_loop_evidence']}},
  {joint:'J2',axis_index:1,measurement_complete:false,record_path:'frf-j2-partial.csv',candidate_valid:false,response:{valid:false,reason:'frequency_response_window_incomplete',fresh_samples:120,duplicate_feedback:0,points:[]},predicted_candidate_loop:{valid:true,crossover_covered:true,crossover_hz:888.76,phase_margin_deg:777.65}},
  {joint:'J3',axis_index:2,measurement_complete:true,record_path:'frf-j3.csv',candidate_valid:true,candidate_kp:6,candidate_kd:.5,response:{valid:true,isolated_axis_excitation:true,points:frequencyPoints},predicted_candidate_loop:{valid:true,crossover_covered:true,crossover_hz:.7345,phase_margin_deg:60}}
]};
const frequencyDesign=vm.runInContext("gainExperimentHtml(frf,['J1','J2','J3'])",context);
assert(frequencyDesign.includes('逐轴频响与 PD 设计')&&frequencyDesign.includes('设计目标交越频率 Hz')&&frequencyDesign.includes('设计目标相位裕度 °'));
assert(frequencyDesign.includes('交越 0.4123 Hz')&&frequencyDesign.includes('裕度 57.4°')&&frequencyDesign.includes('交越 0.7345 Hz')&&frequencyDesign.includes('裕度 60°'));
for(const sentinel of ['123.456','999.87','888.76','777.65'])assert(!frequencyDesign.includes(sentinel),'Uncovered or incomplete loop predictions are unmeasured even if a default or stale number exists');
assert(frequencyDesign.includes('未测')&&frequencyDesign.includes('测量未完成')&&frequencyDesign.includes('缺少证据')&&frequencyDesign.includes('0.042')&&frequencyDesign.includes('0.981')&&frequencyDesign.includes('重复反馈'));
assert(frequencyDesign.includes('data-gain-frf-candidate')&&frequencyDesign.includes('完整设计候选（需整组实测比较）')&&!frequencyDesign.includes('已达标'));
context.partialFrf={kind:'frequency_response_axis_measurement',axis:context.frf.axes[0]};
const partialFrequency=vm.runInContext("gainExperimentHtml(partialFrf,['J1','J2','J3'])",context);
assert(partialFrequency.includes('J1')&&partialFrequency.includes('frf-j1.csv')&&!partialFrequency.includes('data-gain-frf-candidate')&&partialFrequency.includes('尚无完整整组候选'),'One completed axis is retained without synthesizing a complete gain set');
context.isoParameters={kind:'short_experiment_parameters',kp:[4,5,6],kd:[.3,.4,.5],isolated_axis_index:1,other_axes_execution:'fixed_reference_same_complete_gains'};
const isolatedParameters=vm.runInContext("gainExperimentHtml(isoParameters,['J1','J2','J3'])",context);
assert(isolatedParameters.includes('隔离激励关节')&&isolatedParameters.includes('J2')&&isolatedParameters.includes('固定参考，整组增益不变'));
context.isoParameters.isolated_axis_index=null;context.isoParameters.other_axes_execution='not_isolated';
assert(vm.runInContext("gainExperimentHtml(isoParameters,['J1','J2','J3'])",context).includes('非逐轴隔离'));
context.terminalStatus={task_id:'cancelled-task',phase:'cancelled',error:'short response experiment interrupted',fault_interrupted:false};
context.terminalCandidate={task_id:'cancelled-task',outcome:'execution_fault',available:false};
const historicalStop=vm.runInContext('gainTaskConclusion(terminalStatus,terminalCandidate)',context);
assert(historicalStop.outcome==='cancelled'&&!historicalStop.review&&historicalStop.diagnostic&&historicalStop.title==='标定已停止'&&historicalStop.reason.includes('短测响应试验已中断'),'Historical cancellation retains diagnostics without creating a result for review');
context.terminalStatus={phase:'failed',error:'short response input rejected'};
const inputRejected=vm.runInContext('gainTaskConclusion(terminalStatus)',context);
assert(inputRejected.reason.includes('短测输入被拒绝')&&!inputRejected.reason.includes('short response')&&inputRejected.rawReason==='short response input rejected','Known task reasons are translated before a generic error wrapper and retain their raw tooltip');
context.responseStopStatus={task_id:'response-stop',phase:'failed',fault_interrupted:true,error:'model calibration stopped by safety fault',termination:{kind:'execution_fault'}};
context.responseStopCandidate={task_id:'response-stop',response_guard_events:[{kind:'calibration_response_guard',joint:'joint4',position_reversals:4,window_s:.248738299,residual_peak_to_peak_rad:.0045818680546686206}]};
const responseStopConclusion=vm.runInContext('gainTaskConclusion(responseStopStatus,responseStopCandidate)',context);
assert(responseStopConclusion.title==='标定响应保护已触发'&&responseStopConclusion.reason.includes('joint4')&&responseStopConclusion.reason.includes('4 次')&&responseStopConclusion.reason.includes('0.2625°')&&!responseStopConclusion.success,'A matching saved response event explains the actual protected axis and measurement instead of the old umbrella Safety text');
context.responseStopCandidate.task_id='unrelated-task';
assert(!vm.runInContext('gainTaskConclusion(responseStopStatus,responseStopCandidate).reason',context).includes('0.2625°'),'Another task’s guard event cannot supply this interruption’s evidence');
context.terminalStatus={task_id:'x',phase:'complete',gain_available:false,termination:{kind:'insufficient_measurement',reason:'axis_excitation_input_budget_plateau_at_low_frequency_boundary',blocked_joint_names:['joint3']}};
const terminalLimit=vm.runInContext('gainTaskConclusion(terminalStatus)',context);
assert(!terminalLimit.review&&terminalLimit.diagnostic&&!terminalLimit.success&&terminalLimit.reason.includes('joint3')&&terminalLimit.reason.includes('低频激励与输入预算边界'));
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
for(const reason of ['disturbance_measurement_incomplete','hold_disturbance_measurement_incomplete','short response input rejected','native servo tracking reference did not qualify']){
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
for(const code of ['independent_disturbance_not_requested_without_bidirectional_demonstrated_room','shared_symmetric_motion_confirmation_at_same_complete_gains_not_axis_input_plateau','demonstration_probe_range_insufficient:joint3','demonstration_probe_reference_outside_observed_range:joint3','demonstration_probe_feedback_outside_observed_range:joint3','same_complete_gain_set_confirmation_axis_excitation_unavailable:demonstration_probe_range_insufficient:joint3']){
  context.reason=code;
  for(const language of ['zh-CN','en']){context.state.language=language;const label=vm.runInContext('gainWarningLabel(reason)',context);assert(!label.includes(code.split(':')[0]));if(code.includes('joint3'))assert(label.includes('joint3'),'Structured local-range reasons retain the affected joint');}
}
context.state.language='zh-CN';context.reason='identification_insufficient:native: demonstration_probe_reference_outside_observed_range:joint2';
assert(vm.runInContext('gainWarningLabel(reason)',context).includes('joint2　参考位置越出本轮局部试验范围'),'Context prefixes do not swallow local-range stopping evidence');
context.probeParameters={kind:'short_experiment_parameters',kp:[3,4],kd:[.3,.4],position_domain_source:'checked_demonstration_and_fresh_feedback_intersect_explicit_safety',independent_disturbance:true,disturbance_axis_eligible:[false,true],demonstrated_path_lower_rad:[0,-.2],demonstrated_path_upper_rad:[1,.2],plans:[{amplitude_rad:.02,frequencies_hz:[.3,.6,.9],reference_bias_rad:.01},{amplitude_rad:.03,frequencies_hz:[.2,.4,.6],reference_bias_rad:0}],working_domain:[{reference_lower_rad:0,reference_upper_rad:.08},{reference_lower_rad:-.1,reference_upper_rad:.1}]};
const plannedDisturbance=vm.runInContext("gainExperimentHtml(probeParameters,['J2','J3'])",context);
assert(plannedDisturbance.includes('参考偏置 °')&&plannedDisturbance.includes('示教路径区间 °')&&plannedDisturbance.includes('[0, 57.2958]')&&plannedDisturbance.includes('[0, 4.5837]')&&plannedDisturbance.includes('0.573'));
assert(plannedDisturbance.includes('未安排，抗扰未测完整')&&plannedDisturbance.includes('可安排，尚需实测')&&!plannedDisturbance.includes('通过'),'An unexcited axis is not reported as an input failure or a passed disturbance test');
assert(plannedDisturbance.includes('不作为已识别的机械硬限')&&plannedDisturbance.includes('已示教路径与本次新鲜实测位置共同限定')&&plannedDisturbance.includes('title="checked_demonstration_and_fresh_feedback_intersect_explicit_safety"'));
context.probeParameters.independent_disturbance=false;context.probeParameters.disturbance_axis_eligible=[false,false];
const plannedReference=vm.runInContext("gainExperimentHtml(probeParameters,['J2','J3'])",context);
assert(plannedReference.includes('参考激励，不施加独立扰动')&&!plannedReference.includes('未安排，抗扰未测完整'),'Reference experiments do not appear to have skipped a required disturbance');
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
assert.equal(vm.runInContext("gainProbePhaseLabel('taught_path_original_speed_with_interior_identification')",context),'沿示教轨迹测量');
assert.equal(vm.runInContext("gainProbePhaseLabel('mit_indirect_frequency_autotune')",context),'实测闭环整定');
context.mitTrial={kind:'mit_closed_loop_controller_experiment',joint:'J2',controller_index:3,valid:true,fresh_samples:100,
  kp:[3,4],kd:[.3,.4],reason:'independent_command_to_returned_position_three_tone_response_measured',tones:[
    {hz:2,H_real:.012,H_imag:-.004,input_amplitude_nm:.2,output_amplitude_rad:.002,input_tone_fit_r_squared:.94,output_tone_fit_r_squared:.91,half_relative_error:.07}]};
const mitTrialHtml=vm.runInContext("gainExperimentHtml(mitTrial,['J1','J2'])",context);
assert(mitTrialHtml.includes('内部位置频响辨识')&&mitTrialHtml.includes('输入拟合 R²')&&mitTrialHtml.includes('输出拟合 R²')&&mitTrialHtml.includes('不是频谱相干度'));
assert(mitTrialHtml.includes('0.012 − 0.004i')&&mitTrialHtml.includes('0.94')&&mitTrialHtml.includes('本次试验实际整组增益'));
assert(!mitTrialHtml.includes('用途通过')&&!mitTrialHtml.includes('真实带宽')&&!mitTrialHtml.includes('undefined'));
context.mitTrial.valid=false;context.mitTrial.tones[0].output_tone_fit_r_squared=null;
const invalidMitTrial=vm.runInContext("gainExperimentHtml(mitTrial,['J1','J2'])",context);
assert(!invalidMitTrial.includes('0.012 − 0.004i')&&invalidMitTrial.includes('有效证据不足')&&invalidMitTrial.includes('<td>—</td>'),
  'Invalid closed-loop estimates do not display default or stale transfer functions; measured input and fit quality remain separate');
context.mitDesign={kind:'mit_indirect_frequency_autotune',joint:'J2',identified:true,independent_gain_holdout_verified:true,candidate_generated:true,
  kp:[3,5],kd:[.3,.6],model_definition:'inverse_affine',plant_definition:'effective_port',velocity_definition:'internal_channel',
  frequency_response:{valid:true,points:[{hz:2,plant:[.025,-.04],feedback_velocity_per_position:[0,12],input_amplitude_nm:.2,output_amplitude_rad:.002,half_relative_error:.04,coherence:null}]},
  identification_points:[{hz:2,design_condition_number:3.4,gain_holdout_relative_error:.08,half_plant_relative_error:.04,half_velocity_relative_error:.03,a:[1,2],b:[3,4],c:[5,6]}],
  measured_loop:{valid:true,crossover_covered:true,crossover_hz:2.2,phase_margin_deg:56,points:[]},
  candidate_loop:{valid:true,crossover_covered:true,crossover_hz:2.5,phase_margin_deg:60,points:[]}};
const mitDesignHtml=vm.runInContext("gainExperimentHtml(mitDesign,['J1','J2'])",context);
assert(mitDesignHtml.includes('实测闭环整定')&&mitDesignHtml.includes('控制组留出误差')&&mitDesignHtml.includes('0.08')&&mitDesignHtml.includes('设计条件数'));
assert(mitDesignHtml.includes('0.025 − 0.04i')&&mitDesignHtml.includes('0 + 12i')&&mitDesignHtml.includes('<td>2.2</td>')&&mitDesignHtml.includes('<td>60</td>'));
assert(mitDesignHtml.includes('不是裸机械对象')&&mitDesignHtml.includes('仍待整组实测')&&!mitDesignHtml.includes('用途通过')&&!mitDesignHtml.includes('coherence'));
context.mitDesign.independent_gain_holdout_verified=false;
const noMitHoldout=vm.runInContext("gainExperimentHtml(mitDesign,['J1','J2'])",context);
assert(noMitHoldout.includes('尚未验证一致')&&!noMitHoldout.includes('<td>2.2</td>')&&!noMitHoldout.includes('<td>60</td>')&&!noMitHoldout.includes('<td>0.08</td>'),
  'A controller holdout must be independently verified before loop margins or holdout errors appear');
context.mitDesign.identified=false;context.mitDesign.frequency_response.valid=false;
const noMitModel=vm.runInContext("gainExperimentHtml(mitDesign,['J1','J2'])",context);
assert(!noMitModel.includes('0.025 − 0.04i')&&!noMitModel.includes('0 + 12i')&&noMitModel.includes('证据不足'));
assert(vm.runInContext("gainWarningLabel('frequency_identification_warning:J2:mit_independent_position_tone_below_noise')",context).includes('J2　频响辨识证据不足'));
for(const language of ['zh-CN','en']) {
  context.state.language=language;
  const refused=vm.runInContext("gainWarningLabel('candidate_not_admitted_retained_complete_measured_controller:native-before-frozen-install')",context);
  assert(refused.includes(language==='en'?'previous complete gain set':'上一整组参数')&&!refused.includes('candidate_not_admitted'),'An inadmissible proposal explains retained measurement and continuation in both languages');
}
context.state.language='zh-CN';
assert(translateError('frequency_identification_warning:J2:mit_affine_inverse_response_gain_holdout_failed').includes('留出响应与模型预测不一致'));
context.initialization={source:'urdf_mass_matrix_path_envelope',measured:false,kp:[2,4],kd:[.2,.4],inertia_min:[.1,.2],inertia_max:[.2,.3]};
assert(vm.runInContext("gainModelInitializationHtml(initialization,['J2','J1'])",context).includes('<th>J2</th>'));
context.bootstrap={source:'urdf_inertia_and_unmeasured_recovery_bootstrap',response_demand_source:'explicit_recovery_targets_and_urdf_inertia',kp:[1],kd:[.1],unconstrained_kp:[2],unconstrained_kd:[.2]};
const bootstrapHtml=vm.runInContext("gainModelInitializationHtml(bootstrap,['J1'])",context);
assert(bootstrapHtml.includes('URDF 惯量与未实测恢复目标的起动候选')&&bootstrapHtml.includes('整轨需求只作离线对照，不作为起动初值'));
context.nominal={design_target_source:'explicit_generic_response_design_not_urdf_measurement',configuration_initialized:true,
  design_frequency_hz:2,damping_ratio:.7,inertia_basis:[4.3],unconstrained_kp:[679.028],unconstrained_kd:[75.649],
  axes:[{joint:'J3',kp:500,kd:5,unconstrained_kp:679.028,unconstrained_kd:75.649}],hardware_constraints:'declared_protocol_encoding'};
const nominalHtml=vm.runInContext("gainModelInitializationHtml(nominal,['J3'])",context);
assert(nominalHtml.includes('模型初始增益（仅供参考）')&&nominalHtml.includes('原始 Kp')&&nominalHtml.includes('配置 Kp'));
assert(nominalHtml.includes('679.028')&&nominalHtml.includes('<td>500</td>')&&nominalHtml.includes('不是由 URDF 测出的执行器带宽或阻尼'));
assert(!nominalHtml.includes('恢复目标起动候选')&&!nominalHtml.includes('当前运行与恢复参数保持原值'),
  'New generated gains display actual initialization and separate raw model design from encoding');
context.nominal.configuration_initialized=false;context.nominal.hardware_constraints='pending_hardware_selection';
const nominalPreview=vm.runInContext("gainModelInitializationHtml(nominal,['J3'])",context);
assert(nominalPreview.includes('既有配置参数保持原值')&&nominalPreview.includes('执行前选择硬件'),'Authored gains and pending encoding remain explicit');
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
context.oneSidedParameters={kind:'short_experiment_parameters',kp:[3,4],kd:[.3,.4],independent_disturbance:true,independent_input_baseline_s:.25,disturbance_axis_eligible:[true,true],independent_input_axes:[{direction:1,bidirectional:false,required_room_rad:.001,validation_scope:'positive_only_known_joint_torque',reason:'demonstrated_room_supports_one_sided_smooth_disturbance'},{direction:0,bidirectional:true,required_room_rad:.002,validation_scope:'bidirectional_known_joint_torque',reason:'demonstrated_room_supports_symmetric_disturbance'}]};
const oneSidedParameters=vm.runInContext("gainExperimentHtml(oneSidedParameters,['J1','J2'])",context);
assert(oneSidedParameters.includes('独立输入前的无载基线 s')&&oneSidedParameters.includes('0.25')&&oneSidedParameters.includes('仅正向')&&oneSidedParameters.includes('双向')&&oneSidedParameters.includes('验证范围不补成双向实测')&&!oneSidedParameters.includes('验收通过'),'Planned one-sided torque and its actual unloaded baseline are distinct from measured bidirectional qualification');
context.candidate={...warning,warnings:['servo_tracking:disturbance_stiffness_or_recovery_not_met'],experiments:[context.parameters,context.experiment]};
const reviewed=vm.runInContext('gainCandidateReviewHtml(candidate)',context);
assert(reviewed.includes('独立扰动刚度或释放恢复未达指标') && !reviewed.includes('NaN'));
for(const phase of ['native-noise','native-training','native-identification','native-measurement-return','native-frozen','native-fallback-trajectory','native-local-correction','native-disturbance','rigid_tracking-hold','rigid_tracking-hold-disturbance','compliant_tracking-training','compliant_tracking-fallback-trajectory','compliant_tracking-disturbance','compliant-identification','compliant-frozen','compliant-local-correction','compliant_drag','servo_tracking']){
  context.measurementWarning=`measurement_warning:${phase}:gain_endpoint_transition_feedback_wait_timeout`;
  for(const language of ['zh-CN','en']){
    context.state.language=language;const label=vm.runInContext('gainWarningLabel(measurementWarning)',context);
    assert(!label.includes('measurement_warning:')&&!label.includes('gain_endpoint_transition_feedback_wait_timeout')&&!label.includes(phase)&&label.includes(language==='en'?'workflow continued':'流程已继续'),'Actual run_measurement phases retain their readable scope and continued-execution meaning');
  }
}
context.state.language='zh-CN';
context.measurementWarning='measurement_warning:native-training:gain_endpoint_transition:measurement_path_position_resolution_or_noise_consumes_feedback_lead';
assert(vm.runInContext('gainWarningLabel(measurementWarning)',context).includes('位置步进或噪声所需预留占满'),'Nested endpoint and path-helper causes survive the nonfatal warning prefix');
context.unknownWarning='measurement_warning:native-training:UNMAPPED_PROTOCOL_TECHNICAL_DETAIL';
assert(!vm.runInContext('gainWarningLabel(unknownWarning)',context).includes('UNMAPPED_PROTOCOL_TECHNICAL_DETAIL'),'An unmapped technical code is retained in original data without filling the ordinary warning with machine text');
context.measurementWarningRecord={kind:'measurement_warning',phase:'native-training',reason:'gain_endpoint_transition_feedback_wait_timeout',continued:true,completed:false,validated:false,exit_reference:'fresh_measured_position_hold',kp:[3],kd:[.4],blocked_joint_names:['j1']};
const measurementWarningRecord=vm.runInContext("gainExperimentHtml(measurementWarningRecord,['j1'])",context);
assert(measurementWarningRecord.includes('data-gain-measurement-warning')&&measurementWarningRecord.includes('共享原生跟踪增益整定')&&measurementWarningRecord.includes('按最新实测位置保持')&&measurementWarningRecord.includes('流程已继续安排可执行测量')&&measurementWarningRecord.includes('j1: 3')&&measurementWarningRecord.includes('title="gain_endpoint_transition_feedback_wait_timeout"'));
assert(!measurementWarningRecord.includes('workspace-error')&&!measurementWarningRecord.includes('已达标'),'A continued incomplete stage has neutral evidence and no manufactured qualification');
context.warningReview={...warning,task_id:'warning-review',review_required:true,nonfatal_stage_warnings:1,execution_policy:'continue_available_measurements_until_stop_fault_or_original_budget',termination:{kind:'measured_not_qualified',reason:'measurement_warning:native-training:gain_endpoint_transition_feedback_wait_timeout',fault_interrupted:false},warnings:['measurement_warning:native-training:gain_endpoint_transition_feedback_wait_timeout'],experiments:[context.measurementWarningRecord]};
const warningReview=vm.runInContext('gainCandidateReviewHtml(warningReview)',context);
assert(warningReview.includes('data-gain-result-review')&&warningReview.includes('data-gain-execution-summary')&&warningReview.includes('已继续执行的阶段警告')&&warningReview.includes('在原预算内继续可执行测量')&&warningReview.includes('阶段测量记录')&&!warningReview.includes('continue_available_measurements_until_stop_fault_or_original_budget'));
assert(vm.runInContext('gainCandidateDeployable(warningReview)',context),'review_required is a result-view hint, not a new approval or application gate');
context.exitFaultReview={...warning,validated:true,task_id:'late-exit-fault',termination:{kind:'execution_fault',reason:'recording exit failed',fault_interrupted:true}};
const exitFaultReview=vm.runInContext('gainCandidateReviewHtml(exitFaultReview)',context);
assert(exitFaultReview.includes('标定执行已中断')&&!exitFaultReview.includes('gain-result-outcome is-pass')&&exitFaultReview.includes('data-gain-candidate-purpose="servo_tracking"'),'A late execution fault stays visible while retaining an earlier complete parameter set');
context.exitFaultReview.termination={kind:'budget_exhausted',reason:'time_budget_reached'};
assert(!vm.runInContext('gainTaskConclusion({},exitFaultReview).success',context)&&vm.runInContext('gainCandidateReviewHtml(exitFaultReview)',context).includes('标定已到时限'),'A task that reached its hard budget cannot inherit a passed task headline from retained measured parameters');
context.warningReview.native_validation={...tracking,completed:false};assert(!vm.runInContext('gainCandidateDeployable(warningReview)',context),'The review hint never makes incomplete measurements applicable');
context.summaryStatus={workflow_kind:'gain',phase:'complete',task_id:'warning-review',termination:{kind:'insufficient_measurement',reason:'measurement_warning:native-training:gain_endpoint_transition_feedback_wait_timeout'}};
const warningConclusion=vm.runInContext('gainTaskConclusion(summaryStatus)',context);
assert(warningConclusion.title.includes('待补测')&&warningConclusion.reason.includes('流程已继续')&&!warningConclusion.reason.includes('measurement_warning:'),'The first warning remains a readable final cause rather than an execution-fault headline');
context.summaryStatus.phase='gain_adaptation';context.summaryStatus.error=context.summaryStatus.termination.reason;context.summaryStatus.termination.kind='running';
assert(vm.runInContext('gainProbeStatusValues(summaryStatus).error',context)==='','A continued stage warning does not populate the red live execution-error field');
context.partialPassed={...context.warningReview,category:'all',targets:{servo_tracking:{...warning,validated:true},rigid_tracking:{error:'not_started'},compliant_tracking:{error:'not_started'},compliant_drag:{error:'not_started'}}};
const partialPassed=vm.runInContext('gainCandidateReviewHtml(partialPassed)',context);
assert(partialPassed.includes('gain-result-outcome is-reference')&&!partialPassed.includes('gain-result-outcome is-pass'),'One passed available use does not color the whole task as passed while other uses remain unmeasured');
assert(partialPassed.includes('部分用途待补测') && partialPassed.includes('data-gain-candidate-purpose="servo_tracking"') && partialPassed.includes('data-tool-export="gain"'));
assert(vm.runInContext("gainCandidateDeployable(partialPassed,['servo_tracking'])",context),'One completely measured use remains applicable while other four-use measurements are incomplete');
assert(!vm.runInContext("gainCandidateDeployable(partialPassed,['rigid_tracking'])",context),'Unmeasured uses remain unavailable for explicit selection');
context.incompleteReview={...warning,measurement_complete:false,review_ready:false,qualification_complete:false,review_required:false,disturbance_verification:{available:false,completed:false}};
assert(vm.runInContext('gainCandidateDeployable(incompleteReview)',context),'Modern aggregate qualification flags cannot hide a complete measured parameter set');
assert(vm.runInContext('gainTaskConclusion({phase:"complete"},incompleteReview).review',context));
assert(!context.incompleteReview.validated,'Making complete parameters available does not promote missing disturbance qualification to validated');
const incompleteScopeHtml=vm.runInContext('gainCandidateReviewHtml(incompleteReview)',context);
assert(incompleteScopeHtml.includes('抗扰响应待补测')&&incompleteScopeHtml.includes('data-gain-candidate-purpose="servo_tracking"'),'Missing independent disturbance remains a visible warning beside applicable measured gains');
context.incompleteReview.task_id='retained-parameters';
assert(vm.runInContext('gainTaskConclusion({phase:"complete",task_id:"retained-parameters",termination:{kind:"insufficient_measurement"}},incompleteReview).title',context).includes('参数已测量'),'A final missing-purpose-evidence outcome does not imply that complete parameter measurements vanished');
vm.runInContext(extract('gainCalibrationPhaseText'),context);
assert(vm.runInContext('gainCalibrationPhaseText({phase:"complete",task_id:"retained-parameters",termination:{kind:"insufficient_measurement"}},incompleteReview)',context).includes('参数已测量'),'Top state distinguishes available measured parameters from incomplete qualification');
context.incompleteReview.native_validation={...tracking,completed:false};
assert(!vm.runInContext('gainCandidateDeployable(incompleteReview)',context),'Actual incomplete trajectory measurements still prevent application');
context.incompleteReview={...warning,validated:true,validation_scope:'legacy_local_probes',measurement_complete:false};
assert(!vm.runInContext('gainCandidateDeployable(incompleteReview)',context),'Historical local-probe aggregate completion flags keep their original contract');
context.candidate.candidate_comparisons=[context.motion];context.candidate.experiments.push(...context.measuredMotion);
const compared=vm.runInContext('gainCandidateReviewHtml(candidate)',context);
assert(compared.includes('起动测量决策 1')&&compared.includes('参考驱动计划峰值 Nm')&&compared.includes('原始比较记录'),'Historical motion decisions are readable alongside the original comparison data');
context.frequencyCandidate={...warning,joint_names:['J1','J2','J3'],kp:[3,4,5],kd:[.2,.3,.4],native_validation:{...tracking,metrics:Array.from({length:3},()=>({...tracking.metrics[0]}))},experiments:[context.partialFrf],candidate_comparisons:[context.frf]};
const aggregated=vm.runInContext('gainCandidateReviewHtml(frequencyCandidate)',context);
assert.equal((aggregated.match(/data-gain-frf-design/g)||[]).length,1,'Aggregate comparison records display FRF design once and merge matching axis evidence by record_path');
context.frequencyCandidate.candidate_comparisons=[];
const interruptedFrequency=vm.runInContext('gainCandidateReviewHtml(frequencyCandidate)',context);
assert(interruptedFrequency.includes('逐轴频响测量')&&interruptedFrequency.includes('frf-j1.csv'),'Stopping before the aggregate is written retains the completed axis measurement');
context.frequencyCandidate.candidate_comparisons=[context.frf];context.frequencyCandidate.experiments.push({kind:'frequency_response_axis_measurement',axis:{...context.frf.axes[0],record_path:'frf-j1-next-round.csv'}});
assert.equal((vm.runInContext('gainCandidateReviewHtml(frequencyCandidate)',context).match(/data-gain-frf-design/g)||[]).length,2,'An unmatched interrupted measurement from a later round is not hidden by a previous aggregate');
// Final positioning scope is explicit; dynamic lag remains visible without being relabeled a positioning failure.
context.endpointCandidate={...warning,native_validation:{...tracking,acceptance_scope:'final_stationary_target_window_v1',metrics:[{...tracking.metrics[0],peak_error_rad:.3,rms_error_rad:.1,endpoint_window_available:true,endpoint_passed:true,endpoint_peak_error_rad:.0008,endpoint_measurement_samples:80}]}};
const endpointTable=vm.runInContext('gainTaughtPathMetricsHtml(endpointCandidate,["j1"])',context);
assert(endpointTable.includes('最终到位误差')&&endpointTable.includes('过程峰值（辅助）')&&endpointTable.includes('data-gain-acceptance-scope="endpoint"')&&endpointTable.includes('已测量')&&!/达标|验收通过/.test(endpointTable),'Endpoint result and dynamic metrics are displayed separately');
const endpointSummary=vm.runInContext('gainCandidateReviewHtml(endpointCandidate)',context);
assert(endpointSummary.includes('最终到位误差')&&endpointSummary.includes('0.0458°')&&endpointSummary.includes('过程峰值'),'The purpose summary uses actual endpoint error, keeping large dynamic error visible');
context.endpointCandidate.native_validation.metrics[0].endpoint_window_available=false;
assert(vm.runInContext('gainTaughtPathMetricsHtml(endpointCandidate,["j1"])',context).includes('未测量'),'Missing stationary-window evidence is not backfilled from tail means');
const oldTable=vm.runInContext('gainTaughtPathMetricsHtml({native_validation:tracking},["j1"])',Object.assign(context,{tracking}));
assert(oldTable.includes('历史记录包含全程误差')&&!oldTable.includes('最终到位误差')&&!oldTable.includes('过程峰值（辅助）'),'Legacy full-path acceptance is not rewritten to endpoint acceptance');
assert.equal(vm.runInContext("gainProbeStageLabel('trajectory_adapting')",context),'运动中整定');

// New reference-only records retain performance numbers and never become pass/fail UI.
context.referenceOnly={...warning,task_id:'reference-only',reference_only:true,accuracy_thresholds_enabled:false,acceptance_scope:'measured_response_reference_only_v1',termination:{kind:'measured_reference_complete',reason:'reference_analysis_available'},native_validation:{...tracking,completed:true,available:true,acceptance_scope:'measured_response_reference_only_v1',metrics:[{...tracking.metrics[0],endpoint_window_available:true,endpoint_passed:false,endpoint_peak_error_rad:.08}]} };
const originalReferenceRecord=JSON.stringify(context.referenceOnly);
for(const language of ['zh-CN','en']){
 context.state.language=language;
 const referenceHtml=vm.runInContext('gainCandidateReviewHtml(referenceOnly)',context);
 assert(!/达标|验收通过|Target met|Target not met|Outside reference limits|>Passed</.test(referenceHtml));
 assert(referenceHtml.includes(language==='en'?'Review the measured motion':'请根据实测表现')&&referenceHtml.includes('4.5837°')&&referenceHtml.includes('data-gain-candidate-purpose="servo_tracking"'),'Reference-only results keep measured values and temporary-application availability');
}
assert.equal(JSON.stringify(context.referenceOnly),originalReferenceRecord,'Rendering never rewrites new or historical task records');
context.state.language='zh-CN';
assert(vm.runInContext('gainTaskConclusion({phase:"complete"},referenceOnly).success',context),'Reference-only completed measurements are normal completion');
// Completion summaries share a single state interpretation in the live panel and saved review.
context.completedReferenceStatus={phase:'complete',workflow_kind:'gain',task_id:'reference-only',gain_available:true,reference_only:true,termination:{kind:'measured_reference_complete',reason:'reference_analysis_available'}};
const statusBefore=JSON.stringify(context.completedReferenceStatus);
context.state.calibrationToolSelected={gain:{gain_candidate:context.referenceOnly}};
for(const language of ['zh-CN','en']){
 context.state.language=language;
 const expected=language==='en'?'Results saved':'结果已保存';
 const conclusion=vm.runInContext('gainTaskConclusion(completedReferenceStatus,referenceOnly)',context);
 assert(conclusion.success&&conclusion.reason===expected&&conclusion.rawReason==='reference_analysis_available','Completed reference analyses have a readable normal summary and retain their original technical code');
 assert.equal(vm.runInContext('gainProbeStatusValues(completedReferenceStatus).message',context),expected,'The live measurement summary uses the same completion text as the saved review');
 const review=vm.runInContext('gainCandidateReviewHtml(referenceOnly)',context);
 assert(review.includes(expected)&&!review.includes('操作未完成')&&!/Target met|验收通过/.test(review),'A completed reference-only review does not invent a failure or a pass verdict');
 for(const failure of [
  {phase:'failed',error:'feedback unavailable'},
  {fault_interrupted:true},
  {termination:{kind:'execution_fault',reason:'reference_analysis_available',fault_interrupted:true}},
  {termination:{kind:'budget_exhausted',reason:'reference_analysis_available'}},
  {termination:{kind:'insufficient_measurement',reason:'reference_analysis_available'}},
  {phase:'cancelled'},
  {user_cancelled:true}
 ]){
  context.stoppedReferenceStatus={...context.completedReferenceStatus,...failure};
  const stopped=vm.runInContext('gainTaskConclusion(stoppedReferenceStatus,referenceOnly)',context);
  assert(!stopped.success&&stopped.reason!==expected,'A stale completion reason never overrides an actual fault, stop, missing evidence or time expiry');
  if(failure.error)assert(stopped.rawReason===failure.error&&stopped.reason.includes('feedback unavailable'),'An execution failure uses its actual error before a stale completion reason');
 }
}
assert.equal(JSON.stringify(context.completedReferenceStatus),statusBefore,'Status rendering never mutates completion metadata');
assert.equal(JSON.stringify(context.referenceOnly),originalReferenceRecord,'Completion localization never rewrites the saved candidate');
if(process.env.SERIAL_ARM_GAIN_UI_RECORD_DIR){
 const recordDirectory=process.env.SERIAL_ARM_GAIN_UI_RECORD_DIR;
 context.actualReferenceCandidate=JSON.parse(fs.readFileSync(path.join(recordDirectory,'gain-candidate.json'),'utf8'));
 context.actualReferenceStatus=JSON.parse(fs.readFileSync(path.join(recordDirectory,'result.json'),'utf8'));
 context.state.calibrationToolSelected={gain:{gain_candidate:context.actualReferenceCandidate}};
 const candidateBefore=JSON.stringify(context.actualReferenceCandidate),actualStatusBefore=JSON.stringify(context.actualReferenceStatus);
 for(const language of ['zh-CN','en']){
  context.state.language=language;
  const actual=vm.runInContext('gainTaskConclusion(actualReferenceStatus,actualReferenceCandidate)',context);
  assert(actual.success&&actual.rawReason==='reference_analysis_available'&&!actual.reason.includes('操作未完成'),'The read-only recorded completion reproduces the corrected real summary');
  assert.equal(vm.runInContext('gainProbeStatusValues(actualReferenceStatus).message',context),actual.reason);
 }
 assert.equal(JSON.stringify(context.actualReferenceCandidate),candidateBefore);assert.equal(JSON.stringify(context.actualReferenceStatus),actualStatusBefore);
}
context.state.calibrationToolSelected={};
context.state.language='zh-CN';
assert.equal(vm.runInContext("gainProbePhaseLabel('native-online-0')",context),'沿示教运动整定');
assert.equal(vm.runInContext("gainProbePhaseLabel('compliant-online-0')",context),'沿示教运动整定');

// Interior visits and local rigid comparisons are ordinary reference records.
const rigidHeader=fs.readFileSync(path.resolve(root,'../../src/serial_arm/core/include/serial_arm/dynamics/gain_rigid_tuning.hpp'),'utf8');
const rigidReasonCodes=[...new Set([...rigidHeader.matchAll(/"(rigid_[a-z_]+)"/g)].map(match=>match[1]))];
const visitReasons=['visiting_demonstrated_interior_measurement_anchor','demonstrated_measurement_anchor_reached','no_measurable_demonstrated_interior','demonstrated_anchor_visit_invalid','budget_reserves_frozen_group_and_other_purposes'];
context.purposeRecords=[
 {kind:'taught_measurement_anchor_visit',completed:true,source_index_from:0,source_index_to:45,planned_cost_s:3.2,elapsed_s:3.3,record_path:'purpose-visit.csv',source:'checked_demonstration_Hermite_with_rest_to_rest_source_clock',arrival_error_gate:false},
 {kind:'rigid_purpose_local_comparison',kept:false,reference_only:true,previous_kp:[10],previous_kd:[1],proposed_kp:[11.2],proposed_kd:[1],record_path:'rigid-next.csv',baseline_record_path:'rigid-old.csv',reasons:['rigid_resolved_displacement_or_static_error_bounded_stiffness_comparison']},
 {kind:'interior_purpose_measurement_skipped',completed:false,endpoint_fallback:false,reason:'budget_reserves_frozen_group_and_other_purposes'}
];
const purposeBefore=JSON.stringify(context.purposeRecords);
for(const language of ['zh-CN','en']){
 context.state.language=language;
 assert(vm.runInContext("gainProbeStageLabel('trajectory_measurement_visit')",context)!=='trajectory_measurement_visit','Interior measurement travel has a readable stage');
 for(const code of [...rigidReasonCodes,...visitReasons]){context.reason=code;assert(!vm.runInContext('gainWarningLabel(reason)',context).includes(code),'All recorded rigid reasons have human-readable bilingual descriptions');}
 for(const code of visitReasons.slice(0,2)){context.reason=code;assert(!vm.runInContext('gainProbeMessageLabel(reason)',context).includes(code),'Live travel messages are readable');}
 const recordHtml=vm.runInContext('purposeRecords.map(record=>gainExperimentHtml(record,["J1"])).join("")',context);
 assert((recordHtml.match(/data-gain-purpose-record=/g)||[]).length===3&&!/操作未完成|Calibration failed|验收通过/.test(recordHtml),'Normal travel, rigid comparison and unscheduled supplement records never become a failure or a pass verdict');
 assert(recordHtml.includes(language==='en'?'Retain the previous measured whole set':'保留原实测整组参数')&&recordHtml.includes('rigid-next.csv')&&recordHtml.includes('rigid-old.csv'),'The actually retained group and both same-input records stay visible');
 context.recordCandidate={...context.referenceOnly,experiments:[context.purposeRecords[0],context.purposeRecords[2]],candidate_comparisons:[context.purposeRecords[1]]};
 const completeHtml=vm.runInContext('gainCandidateReviewHtml(recordCandidate)',context);
 assert((completeHtml.match(/data-gain-purpose-record=/g)||[]).length===3,'Saved rigid comparisons are included in typed result diagnostics instead of silently hidden');
}
assert.equal(JSON.stringify(context.purposeRecords),purposeBefore,'Purpose diagnostics are read-only');
context.state.language='zh-CN';

context.passingPurpose={kind:'taught_path_purpose_dwell',completed:true,source_index:45,reverse:true,elapsed_s:28.4,measurement_dwell_s:8.2,resumed:true,arrival_error_gate:false,record_path:'gain-trajectory-native-passing-purpose.csv',reason:'taught_path_purpose_dwell_complete'};
const passingBefore=JSON.stringify(context.passingPurpose);
for(const language of ['zh-CN','en']){
 context.state.language=language;
 assert(!vm.runInContext("gainProbeStageLabel('trajectory_purpose_pass')",context).includes('trajectory_purpose_pass'));
 assert(!vm.runInContext("gainProbePhaseLabel('native-passing')",context).includes('native-passing'));
 assert(!vm.runInContext("gainProbeMessageLabel('normal_taught_path_with_purpose_dwell')",context).includes('normal_taught_path_with_purpose_dwell'));
 for(const purpose of ['servo','compliant']){
  context.resumedStatus={phase:'running',gain_protocol:'taught_trajectory',gain_active_target:purpose,gain_probe_stage:'trajectory_purpose_pass',gain_probe_kind:purpose==='compliant'?'compliant-passing':'native-passing',gain_probe_message:'normal_taught_path_with_purpose_dwell'};
  assert(vm.runInContext('gainProbeStatusValues(resumedStatus).use===gainPurposeLabel(resumedStatus.gain_active_target)',context),'After a purpose dwell the live use follows the resumed path, not the last hold or drag measurement');
  assert(vm.runInContext("gainProbeStatusValues(resumedStatus).phase===gainProbeStageLabel('trajectory_purpose_pass')",context),'After a purpose dwell the live stage shows resumed path traversal');
 }
 const html=vm.runInContext('gainExperimentHtml(passingPurpose,["J1"])',context);
 assert(html.includes('data-gain-passing-purpose')&&html.includes('28.4')&&html.includes('8.2')&&html.includes('gain-trajectory-native-passing-purpose.csv'));
 assert(html.includes(language==='en'?'Resumed in the same direction':'已继续原方向')&&!/验收通过|Calibration failed|Qualified/.test(html));
 context.partialPassing={...context.passingPurpose,completed:false,resumed:false,elapsed_s:48.926704956,pause_begin_s:7.888727158,pause_end_s:0,measurement_dwell_s:0,reason:'taught_path_purpose_dwell_interrupted_or_partial'};
 const partialHtml=vm.runInContext('gainExperimentHtml(partialPassing,["J1"])',context);
 assert(partialHtml.includes(language==='en'?'Not yet resumed':'尚未继续')&&!partialHtml.includes(language==='en'?'Traversal complete':'本趟已走完'));
 assert(partialHtml.includes(language==='en'?'Measurement interrupted; path not yet resumed':'测量中断，尚未恢复轨迹')&&partialHtml.includes('41.04')&&partialHtml.includes(language==='en'?'Dwell elapsed before interruption s':'中断前已停留用时 s'),'An interrupted callback shows its elapsed partial dwell, not the uncommitted zero source-clock insertion');
 context.unknownPassing={kind:'taught_path_purpose_dwell'};
 const unknownHtml=vm.runInContext('gainExperimentHtml(unknownPassing,["J1"])',context);
 assert(!unknownHtml.includes(language==='en'?'Resumed in the same direction':'已继续原方向')&&!unknownHtml.includes('0.00'),'Missing legacy dwell timing is not shown as a measured zero');
}
assert.equal(JSON.stringify(context.passingPurpose),passingBefore);
context.state.language='zh-CN';

context.onlineUpdate={kind:'online_trajectory_PD_update',stage:'trajectory_adapting',method:'indirect_local_IV_ARX_projected_PD',reference_only:true,accuracy_thresholds_enabled:false,completed:false,time_s:4,record_path:'gain-trajectory-native-online-0.csv',candidate_generated:true,admission_allowed:true,installed:true,previous_kp:[10,20],previous_kd:[1,2],proposed_kp:[11,20],proposed_kd:[.9,2],models:[{joint:'Alpha',valid:true,reason:'held_out_local_iv_model_supported',dt_s:.02,delay_s:.04,iv_rank:5,training_samples:100,validation_samples:40,validation_rms_rad:.0002,arx:[1.7,-.7,.004,.002,0],predicted_cost_before:1,predicted_cost_after:.9},{joint:'Beta',valid:false,reason:'independent_input_not_informative_for_local_model',iv_rank:3,arx:[999,998,997,996,995],predicted_cost_before:777,predicted_cost_after:666}],reasons:['online_iv_predictive_small_pd_step_reference_only_external_gain_ramp_required','independent_input_not_informative_for_local_model']};
Object.assign(context.onlineUpdate.models[0],{fresh_samples:788,command_samples:800,resampled_samples:199,feedback_interval_min_s:.0014,feedback_interval_max_s:.0131,observed_encoder_step_rad:25/65536,fit_attempts:7,rank_rejections:0,plausibility_rejections:4,prediction_rejections:1,structure:'arx2_tied_zoh_input_coefficient',parameter_count:4,training_span_rad:.035,validation_error_limit_rad:.007});
const onlineBefore=JSON.stringify(context.onlineUpdate);
const onlineHtml=vm.runInContext('gainExperimentHtml(onlineUpdate,["Alpha","Beta"])',context);
assert(onlineHtml.includes('data-gain-online-update')&&onlineHtml.includes('已提交平滑执行')&&onlineHtml.includes('上一组参数')&&onlineHtml.includes('拟更新参数')&&onlineHtml.includes('实际生效 Kp / Kd 请查看独立实时增益窗口'));
assert(onlineHtml.includes('本窗输入与响应的信息不足')&&onlineHtml.includes('留出预测 RMS °')&&!onlineHtml.includes('999')&&!onlineHtml.includes('777'),'Invalid local models retain missing-evidence reasons without showing stale prediction coefficients or costs');
assert(onlineHtml.includes('局部模型结构')&&onlineHtml.includes('共用 ZOH 输入系数')&&onlineHtml.includes('模型参数数目')&&onlineHtml.includes('训练窗实测行程 °')&&onlineHtml.includes('2.00535')&&onlineHtml.includes('局部模型留出误差尺度 °')&&onlineHtml.includes('0.40107'),'Supported local-model structure, measurement span and prediction evidence scale are inspectable separately from trajectory accuracy');
assert(onlineHtml.includes('等间隔建模点数')&&onlineHtml.includes('199')&&onlineHtml.includes('最大反馈间隔 s')&&onlineHtml.includes('模型系数不合理次数'),'Timing, source resolution and the actual model rejection stage stay visible without changing the original record');
assert(!/达标|验收通过|已中断/.test(onlineHtml)&&!onlineHtml.includes('实际整组增益'),'An in-progress proposal is neither a qualification verdict nor executed gains');
assert.equal(JSON.stringify(context.onlineUpdate),onlineBefore,'Online event rendering is read-only');
context.onlineUpdate.installed=false;context.onlineUpdate.admission_allowed=false;
assert(vm.runInContext('gainExperimentHtml(onlineUpdate)',context).includes('执行约束不允许本次更新'));
context.onlineUpdate.candidate_generated=false;
assert(vm.runInContext('gainExperimentHtml(onlineUpdate)',context).includes('本次未生成增益更新，继续测量'));
context.onlineCandidate={...warning,candidate_comparisons:[context.onlineUpdate]};
assert(vm.runInContext('gainCandidateReviewHtml(onlineCandidate)',context).includes('data-gain-online-update'),'Persisted online update comparisons receive typed diagnostics');
for(const reason of ['insufficient_online_measurements','held_out_local_iv_model_supported','independent_input_not_informative_for_local_model','local_iv_model_not_supported_by_next_window_prediction','local_iv_model_not_physically_plausible','no_known_independent_excitation_model_remains_prior','nonuniform_model_time_grid','insufficient_future_taught_reference_model_updates_continue','local_predictor_outside_supported_neighborhood','unresolved_local_gain_sensitivity','online_iv_predictive_small_pd_step_reference_only_external_gain_ramp_required','local_model_has_no_resolved_admissible_gain_step_keep_measured_group_and_continue']){
 context.reason=reason;assert(!vm.runInContext('gainWarningLabel(reason)',context).includes(reason),'Online update reasons have readable explanations');
}

const finalWindowReasons=['final_target_window_not_measured','final_target_window_incomplete','final_target_nonfinite','final_target_feedback_discontinuous','final_target_reference_not_stationary','final_target_limited','final_target_feedback_stale','final_target_feedback_insufficient','final_target_window_qualified','final_target_error_not_qualified','compliant restoring response is below executable protocol resolution'];
for(const reason of finalWindowReasons){
  context.reason=reason;
  for(const language of ['zh-CN','en']){context.state.language=language;const label=vm.runInContext('gainWarningLabel(reason)',context);assert(!label.includes(reason)&&!/已停止|已中断|stopped|interrupted|达标|验收通过|met its target|missed its acceptance/.test(label),'Endpoint accuracy evidence cannot imply a dynamic-execution stop');}
  assert(!translateError(reason).includes(reason),'Priority errors localize the same actual measurement reason');
}
context.state.language='zh-CN';
// Direct pulse evidence is a measured input/output scope, separate from a model fit.
context.directPulse={kind:'independent_torque_disturbance',method:'isolated_smooth_torque_pulse',model_identified:false,
 direct_measurement_complete:true,available:false,completed:true,record_path:'gain-purpose-rigid.csv',kp:[10],kd:[1],
 axes:[{joint:'J1',input_measurement_complete:true,direct_response_complete:true,measurement_complete:false,hold_settled:true,
  hold_signed_displacement_rad:-.004,effective_compliance_rad_nm:.02,reason:'disturbance_response_qualified'}]};
context.pulseParameters={kind:'short_experiment_parameters',waveform:'isolated_smooth_pulse',isolation:'sequential_single_axis',
 other_axes_execution:'fixed_reference_same_complete_gains',kp:[10],kd:[1],pulse_axes:[{joint:'J1',scheduled:true,start_s:.2,release_s:1,end_s:2,amplitude_nm:.1,direction:-1,reason:'demonstrated_room_supports_one_sided_smooth_disturbance'}]};
context.actualDrag={kind:'drag_actual_zero_stiffness_measurement',actual_gains_ready_s:.16,quiet_start_s:.2,quiet_end_s:1,
 record_path:'drag-decay-actual.csv',axes:[{joint:'J1',quiet_stability_measured:true,quiet_stability_passed:false,quiet_position_span_rad:.001,
 quiet_speed_rad_s:.002,release_decay_measured:false,release_decay_passed:false}]};
context.fixedReference={kind:'hold_parameters',target_source:'completed_taught_anchor_nominal_reference_not_actual_rezero',target_pos:[.2,-.1],entry_actual_pos:[.17,-.15]};
context.inputSupplement={kind:'purpose_input_supplement_decision',joint:'J1',attempt:1,previous_input_nm:.02,next_input_nm:.03,next_window_planned:true,reason:'measured_unresolved_response_bounded_input_increase',record_path:'gain-purpose-rigid.csv'};
context.dragInput={kind:'drag_input_plan',joint:'J1',scheduled:true,input_nm:.08,next_input_nm:.12,input_ceiling_nm:.2,torque_quantum_nm:.005,prior_valid:false,prior_input_nm:999,duration_s:1.6,record_path:'drag-decay-actual.csv',reason:'drag_known_unresolved_input_bounded_increase'};
for(const language of ['zh-CN','en']){
 context.state.language=language;
 for(const kind of ['drag_input_plan','drag_input_window','drag_input_supplement_decision']){
  context.dragInput.kind=kind;
  const before=JSON.stringify(context.dragInput),html=vm.runInContext('gainExperimentHtml(dragInput, ["J1"])',context);
  assert(html.includes(`data-gain-drag-input="${kind}"`)&&html.includes('0.08')&&html.includes('0.005')&&html.includes('drag-decay-actual.csv'));
  assert(!html.includes('999')&&!html.includes('验收通过')&&!html.includes('>Passed<'),'Invalid prior numbers and unmeasured pass claims stay out of drag plans');
  assert(html.includes(kind==='drag_input_window'?(language==='en'?'Executed window':'已执行窗口'):kind==='drag_input_plan'?(language==='en'?'Input window scheduled':'已安排输入窗口'):(language==='en'?'Follow-up scheduled':'已安排补测')),'Plans, executed windows and follow-up decisions have separate statuses');
  assert.equal(JSON.stringify(context.dragInput),before);
 }
 context.dragInput.scheduled=false;
 const skipped=vm.runInContext('gainExperimentHtml(dragInput, ["J1"])',context);
 assert(!skipped.includes('0.12')&&skipped.includes(language==='en'?'No further window scheduled':'不再安排补测'),'A rejected follow-up cannot show its stale proposed input as scheduled');
 context.dragInput.scheduled=true;
}
for(const language of ['zh-CN','en']){
 context.state.language=language;
 const html=vm.runInContext('gainExperimentHtml(inputSupplement,["J1"])',context);
 assert(html.includes('data-gain-input-supplement')&&html.includes('0.02')&&html.includes('0.03')&&html.includes(language==='en'?'remaining budget':'剩余预算'));
 assert(!/验收通过|>Passed</.test(html),'An unexecuted supplement plan never becomes a measured pass');
}
context.localComparison={kind:'taught_local_complete_group_comparison',kept:false,model_identified:false,source_index_from:80,source_index_to:30,reverse:true,playback_rate:1,tail_s:1.5,feedforward_scope:'gravity',load_scope:'declared_unchanged_task_load',changed_joint:'J1',coordinate:'Kp',previous_kp:[10],previous_kd:[1],proposed_kp:[11],proposed_kd:[1],reason:'empirical_bounded_coordinate_poll_for_measured_static_bias_or_same_direction_lag',baseline_record_path:'gain-trajectory-local-native-baseline.csv',record_path:'gain-trajectory-local-native-candidate-0.csv',baseline_axes:[{complete:true,static_measured:true,rms_error_rad:.02,peak_error_rad:.05,static_error_rad:.01,same_direction_lag_s:.1,comparison_uncertainty_rad:.001}],candidate_axes:[{joint:'J1',complete:true,static_measured:true,rms_error_rad:.03,peak_error_rad:.06,static_error_rad:.02,same_direction_lag_s:.11,comparison_uncertainty_rad:.001}]};
const localOriginal=JSON.stringify(context.localComparison);
for(const language of ['zh-CN','en']){
 context.state.language=language;
 const html=vm.runInContext('gainExperimentHtml(localComparison,["J1"])',context);
 assert(html.includes('data-gain-taught-local')&&html.includes('data-gain-local-metrics')&&html.includes('baseline.csv')&&html.includes('candidate-0.csv'));
 assert(html.includes(language==='en'?'Retain the prior complete set':'保留原完整组')&&html.includes(language==='en'?'Not identified; direct comparison':'未辨识，使用直接实测比较'),'A rejected direct comparison neither becomes a fitted model nor a retained gain set');
 assert(!/验收通过|>Passed</.test(html));
 const withSaved=vm.runInContext('gainCandidateReviewHtml({...referenceOnly,candidate_comparisons:[localComparison]})',context);
 assert(withSaved.includes('data-gain-taught-local'),'Persisted local comparison diagnostics remain accessible');
}
assert.equal(JSON.stringify(context.localComparison),localOriginal,'Local comparison display never rewrites the measured record');
context.comparisonContexts=[
 {kind:'taught_local_comparison_repeat',bounded_repeats:1,raw_metrics_preserved:true,reason:'taught_local_initial_state_not_comparable_retry_same_segment',original_baseline_record_path:'original-baseline.csv',original_candidate_record_path:'original-candidate.csv'},
 {kind:'drag_damping_comparison_conditions',comparable:false,relative_M_change:.075,max_gravity_change_nm:.03,arrival_error_gate:false,reason:'purpose_comparison_inertia_changed_beyond_experiment_tolerance'},
 {kind:'drag_damping_comparison_skipped',reason:'complete_reset_comparison_and_exit_do_not_fit_remaining_budget'}
];
const contextsBefore=JSON.stringify(context.comparisonContexts);
for(const language of ['zh-CN','en']){
 context.state.language=language;
 context.withComparisonReason={...context.localComparison,comparison_reason:'taught_local_unchanged_axis_entry_bias_requires_matched_repeat'};
 const local=vm.runInContext('gainExperimentHtml(withComparisonReason,["J1"])',context);
 assert(local.includes(language==='en'?'Comparison conclusion':'比较结论')&&local.includes(language==='en'?'different offset':'入口偏差不同'),'Actual comparison reasons are separate from proposal directions');
 const html=vm.runInContext('gainCandidateReviewHtml({...referenceOnly,candidate_comparisons:comparisonContexts})',context);
 assert((html.match(/data-gain-comparison-context/g)||[]).length===3&&html.includes('original-baseline.csv')&&html.includes('original-candidate.csv'),'Matched-repeat source evidence and actual drag conditions stay accessible');
 assert(html.includes(language==='en'?'Not comparable':'不可直接比较')&&html.includes('0.075')&&!html.includes('purpose_comparison_inertia_changed_beyond_experiment_tolerance</'));
}
assert.equal(JSON.stringify(context.comparisonContexts),contextsBefore);
context.inputPriors={kind:'purpose_input_prior',record_path:'gain-purpose-native-disturbance.csv',input_priors:[{joint:'J1',valid:true,independent_samples:80,low_speed_pd_samples:12,independent_peak_nm:.12,low_speed_pd_level_nm:.15,seed_nm:.15,source:'gain-trajectory-native-online-0.csv',reason:'measured_known_input_and_low_speed_PD_scale_prior'},{joint:'J2',valid:false,independent_samples:0,low_speed_pd_samples:0,independent_peak_nm:999,low_speed_pd_level_nm:998,seed_nm:997,source:'gain-trajectory-native-online-0.csv',reason:'purpose_input_prior_insufficient_known_samples'}]};
for(const language of ['zh-CN','en']){
 context.state.language=language;
 const html=vm.runInContext('gainExperimentHtml(inputPriors,["J1","J2"])',context);
 assert(html.includes('data-gain-input-prior')&&html.includes('0.12')&&html.includes('0.15')&&html.includes('gain-trajectory-native-online-0.csv'));
 assert(!html.includes('999')&&!html.includes('998')&&!html.includes('997'),'Invalid measured scale does not render default or stale numeric evidence');
 assert(html.includes(language==='en'?'not an identified friction coefficient':'不是已辨识的摩擦系数'));
}
const fixedOriginal=JSON.stringify(context.fixedReference);
for(const language of ['zh-CN','en']){
 context.state.language=language;
 const html=vm.runInContext('gainFixedReferenceEvidenceHtml(fixedReference,["J1","J2"])',context);
 assert(html.includes('data-gain-fixed-reference')&&html.includes('1.7189')&&html.includes('2.8648'),'Entry target error is computed from separate recorded positions, not replaced by zero');
 assert(html.includes(language==='en'?'not an arrival gate':'不作为到位门槛'),'Showing true entry error cannot introduce an alignment gate');
 assert.equal(vm.runInContext('gainFixedReferenceEvidenceHtml({available:true,completed:true},["J1"])',context),'','Old aggregate results do not acquire an invented nominal target or actual entry');
 const legacy=vm.runInContext('gainFixedReferenceEvidenceHtml({target_pos:[.2],entry_actual_pos:[.1]},["J1"])',context);
 assert(!legacy.includes(language==='en'?'retains the emitted':'保留已发送'),'Legacy target source is not upgraded to a nominal taught anchor');
}
assert.equal(JSON.stringify(context.fixedReference),fixedOriginal,'Reference diagnostics never rewrite recorded position evidence');
const directOriginal=JSON.stringify(context.directPulse);
for(const language of ['zh-CN','en']){
 context.state.language=language;
 const direct=vm.runInContext('gainDirectDisturbanceScope(directPulse)',context);
 assert(direct.measured&&direct.modelIdentified===false,'Explicit direct response completion does not require a fitted model or compliant-yield pass');
 const html=vm.runInContext('gainExperimentHtml(directPulse,["J1"])',context);
 assert(html.includes('data-gain-direct-disturbance')&&html.includes(language==='en'?'Not fitted in this direct measurement':'本次直接测量不拟合模型')&&html.includes(language==='en'?'Measured':'已测量'));
 assert(!/闭环模型拟合质量不足|有效证据不足|验收通过|>Passed</.test(html),'Direct pulse evidence never fabricates a failed model fit or a pass');
 assert.equal(vm.runInContext('gainDirectDisturbanceScope({available:true,completed:true,record_path:"legacy.csv",axes:[{measurement_complete:true}]})',context),null,'Historical aggregate fields never acquire a new direct-input scope');
 const parameters=vm.runInContext('gainExperimentHtml(pulseParameters,["J1"])',context);
 assert(parameters.includes('data-gain-pulse-plan')&&parameters.includes(language==='en'?'Sequential single-axis measurements':'逐轴顺序测量')&&parameters.includes(language==='en'?'Negative only':'仅反向'));
 const drag=vm.runInContext('gainExperimentHtml(actualDrag,["J1"])',context);
 assert(drag.includes('data-gain-actual-drag')&&drag.includes('data-gain-drag-scopes')&&drag.includes(language==='en'?'Not measured':'未测量'),'Actual quiet drag and unmeasured release remain distinct');
 assert.equal(vm.runInContext("gainProbeStageLabel('trajectory_frozen')",context),language==='en'?'Final-gain path analysis':'最终参数回放分析');
 assert.equal(vm.runInContext("gainProbePhaseLabel('native-frozen')",context),language==='en'?'Final-gain path analysis':'最终参数回放分析');
 const dragPlan=vm.runInContext('gainExperimentHtml({kind:"drag_excitation_parameters",source:"actual_zero_stiffness_then_quiet_then_isolated_known_torque_release",actual_gains_ready_s:.16,quiet_start_s:.16,quiet_end_s:.86,kp:[0],kd:[.2]},["J1"])',context);
 assert(dragPlan.includes('data-gain-actual-drag')&&dragPlan.includes('0.16')&&!dragPlan.includes('data-gain-drag-scopes'),'Actual transition timings remain visible in the emitted parameter record without inventing unrecorded scope evidence');
}
assert.equal(JSON.stringify(context.directPulse),directOriginal,'New direct evidence rendering leaves historical and current records unchanged');
context.state.language='zh-CN';
context.sharedSourceCandidate=require('./gain-ui-fixture.cjs').makeCandidate();
const sharedMissing={method:'isolated_smooth_torque_pulse',model_identified:false,direct_measurement_complete:false,available:false,completed:false,record_path:'shared-purpose.csv',axes:[]};
Object.assign(context.sharedSourceCandidate.targets.rigid_tracking,{disturbance_verification:{...sharedMissing},hold_disturbance_verification:{...sharedMissing},warnings:['disturbance_measurement_incomplete','hold_disturbance_measurement_incomplete']});
Object.assign(context.sharedSourceCandidate.targets.servo_tracking,{disturbance_verification:{...sharedMissing},warnings:['disturbance_measurement_incomplete']});
const sharedOriginal=JSON.stringify(context.sharedSourceCandidate);
const sharedHtml=vm.runInContext('gainCandidateReviewHtml(sharedSourceCandidate)',context),sharedText=sharedHtml.replace(/<[^>]+>/g,' ');
assert.equal((sharedText.match(/抗扰响应待补测/g)||[]).length,1,'One missing shared acquisition is described once across disturbance, hold and reused servo scopes');
assert(!sharedText.includes('保持扰动测量未完成'),'A hold alias of the same missing record is not shown as a second missing experiment');
assert.equal(JSON.stringify(context.sharedSourceCandidate),sharedOriginal,'Warning deduplication never rewrites the shared historical source objects');
context.sharedSourceCandidate.targets.rigid_tracking.hold_disturbance_verification={shared_with:'disturbance_verification',record_path:'alias-view.csv'};
assert(!vm.runInContext('gainCandidateReviewHtml(sharedSourceCandidate)',context).replace(/<[^>]+>/g,' ').includes('保持扰动测量未完成'),'An explicit same-purpose shared_with reference deduplicates its source even when the alias path differs');
context.sharedSourceCandidate.targets.rigid_tracking.hold_disturbance_verification={...sharedMissing};
context.sharedSourceCandidate.targets.rigid_tracking.hold_disturbance_verification.record_path='distinct-hold.csv';
assert(vm.runInContext('gainCandidateReviewHtml(sharedSourceCandidate)',context).replace(/<[^>]+>/g,' ').includes('保持扰动测量未完成'),'Different source records retain separate missing-scope explanations');
context.sharedSourceCandidate.targets.rigid_tracking.hold_disturbance_verification={...sharedMissing,record_path:'alias.csv',shared_source:'shared-purpose.csv'};
context.sharedSourceCandidate.targets.rigid_tracking.disturbance_verification.shared_source='shared-purpose.csv';
assert(!vm.runInContext('gainCandidateReviewHtml(sharedSourceCandidate)',context).replace(/<[^>]+>/g,' ').includes('保持扰动测量未完成'),'An explicit shared source is honored even when aliases use different record paths');
context.completedDirectCandidate={...context.referenceOnly,disturbance_verification:context.directPulse,hold_disturbance_verification:context.directPulse};
assert(!vm.runInContext('gainCandidateReviewHtml(completedDirectCandidate)',context).replace(/<[^>]+>/g,' ').includes('独立扰动测量不足'),'New complete direct measurements are not downgraded by legacy model availability=false');
context.completedDirectCandidate.hold_disturbance_verification={shared_with:'disturbance_verification',method:'isolated_smooth_torque_pulse',available:false,record_path:context.directPulse.record_path};
assert(!vm.runInContext('gainCandidateReviewHtml(completedDirectCandidate)',context).replace(/<[^>]+>/g,' ').includes('保持扰动测量未完成'),'The explicit hold alias reuses complete direct evidence without copying or fabricating it');
assert.equal((vm.runInContext('gainCandidateReviewHtml(completedDirectCandidate)',context).match(/data-gain-direct-disturbance/g)||[]).length,1,'A same-record hold alias renders its actual acquisition once');
const purposePulseCodes=['purpose_input_prior_not_measured','purpose_input_prior_insufficient_known_samples','measured_known_input_and_low_speed_PD_scale_prior','purpose_pulse_actual_command_or_feedback_timing_invalid','purpose_pulse_resolved_response_opposes_known_input','purpose_pulse_response_below_measured_resolution','purpose_pulse_retry_requires_complete_unlimited_known_input','purpose_pulse_resolved_response_no_input_increase','purpose_pulse_retry_requires_measured_positive_input','purpose_pulse_current_domain_ceiling_no_resolved_response','measured_unresolved_response_bounded_input_increase','purpose_pulse_not_planned','purpose_pulse_model_or_capability_missing','purpose_pulse_invalid_measured_input','purpose_pulse_stiffness_mode_mismatch','purpose_pulse_measured_domain_or_budget_missing','purpose_pulse_demonstrated_room_below_resolution','purpose_pulse_no_representable_input','purpose_pulse_window_does_not_fit_remaining_budget','single_axis_model_noise_and_measured_room_bounded_pulse','purpose_pulse_not_measured','purpose_pulse_plan_or_samples_missing','purpose_pulse_fresh_feedback_missing','purpose_pulse_baseline_plateau_or_release_missing','purpose_pulse_actual_input_unknown','purpose_pulse_actual_input_does_not_match_command_plan','purpose_pulse_input_or_response_limited','purpose_pulse_requires_known_constant_restoring_gains','purpose_pulse_reference_not_fixed','purpose_pulse_input_measurement_incomplete','purpose_pulse_plateau_not_settled','purpose_pulse_yield_below_resolution_or_opposite_direction','purpose_pulse_yield_above_declared_local_envelope','purpose_pulse_unloaded_recovery_incomplete','purpose_pulse_resolved_plateau_yield_and_stable_recovery'];
for(const language of ['zh-CN','en']){
 context.state.language=language;
 for(const reason of purposePulseCodes){context.reason=reason;assert.notEqual(vm.runInContext('gainWarningLabel(reason)',context),reason,'Direct pulse evidence has a readable label in each language');}
 for(const reason of ['isolated_purpose_pulse','purpose_direct_input_measured','purpose_direct_measurement_partial','drag_actual_zero_stiffness_handover','quiet_drag_goal_measured','drag_measurement_available_for_review','drag_quiet','drag_input','drag_release']){context.reason=reason;assert.notEqual(vm.runInContext('gainProbeMessageLabel(reason)',context),reason,'Live purpose-measurement progress is readable');}
}
context.state.language='zh-CN';
const dragFields=context.sharedSourceCandidate.targets.compliant_drag;
dragFields.warnings=['drag_release_decay_unmeasured'];
context.dragCandidate={...context.sharedSourceCandidate,category:'compliant_drag',...dragFields,joint_names:context.sharedSourceCandidate.joint_names};
const dragNotes=vm.runInContext('gainPurposeMeasurementWarnings(dragCandidate,"compliant_drag",dragCandidate)',context);
assert(!dragNotes.includes('drag_release_decay_unmeasured'),'A complete quiet drag result keeps release absence in its separate scope line, not as a second whole-group warning');
const {makeCandidate}=require('./gain-ui-fixture.cjs');
context.partialUseResult=makeCandidate();
context.partialUseResult.termination={kind:'insufficient_measurement',reason:'drag_release_decay_unmeasured:joint1'};
Object.assign(context.partialUseResult.targets.compliant_tracking,{available:false,native_validation:{completed:false,metrics:[]},warnings:['compliant_measurement_incomplete']});
const partialBefore=JSON.stringify(context.partialUseResult);
for(const language of ['zh-CN','en']){
 context.state.language=language;
 const summary=vm.runInContext('gainTaskConclusion({phase:"complete"},partialUseResult)',context);
 assert(summary.title.includes(language==='en'?'some uses need more measurement':'部分用途待补测'));
 assert(summary.reason.includes(language==='en'?'Compliant need more measurement':'柔性待补测')&&!summary.reason.includes('joint1'),'Usable groups and the missing use lead the result instead of an unrelated optional-release warning');
 assert.equal(summary.rawReason,'drag_release_decay_unmeasured:joint1','The original end reason remains available in diagnostics');
}
assert.equal(JSON.stringify(context.partialUseResult),partialBefore);
context.state.language='zh-CN';
context.originCandidate=makeCandidate();
context.originCandidate.model_initialization.source='urdf_geometry_time_weighted_path_mass_matrix_nominal_response_design_double_diagonalization_diagonal_projection';
const soft=context.originCandidate.targets.compliant_tracking;
soft.native_validation.record_path='gain-trajectory-compliant-frozen.csv';
context.originCandidate.candidate_comparisons=[{kind:'taught_local_complete_group_comparison',purpose:'compliant',kept:false,previous_kp:soft.kp,previous_kd:soft.kd,proposed_kp:soft.kp.map(value=>value*.9),proposed_kd:soft.kd}];
assert(vm.runInContext('gainCandidateParameterSourceHtml(originCandidate,"compliant_tracking")',context).includes('已比较 1 组候选，保留原整组参数'),'A measured rejected candidate is distinct from never attempting adjustment');
context.originCandidate.candidate_comparisons=[{kind:'online_trajectory_PD_update',source_record_path:'gain-trajectory-compliant-online-0.csv',installed:true,proposed_kp:soft.kp,proposed_kd:soft.kd}];
assert(vm.runInContext('gainCandidateParameterSourceHtml(originCandidate,"compliant_tracking")',context).includes('已在运动中调整，并完成整轨复测'));
context.originCandidate.candidate_comparisons[0].installed=false;
assert(!vm.runInContext('gainCandidateParameterSourceHtml(originCandidate,"compliant_tracking")',context).includes('已在运动中调整'),'An uninstalled proposal does not become a measured parameter change');
if(process.env.SERIAL_ARM_GAIN_UI_PARTIAL_RECORD_DIR){
 const folder=process.env.SERIAL_ARM_GAIN_UI_PARTIAL_RECORD_DIR;
 context.actualPartialCandidate=JSON.parse(fs.readFileSync(path.join(folder,'gain-candidate.json'),'utf8'));
 context.actualPartialStatus=JSON.parse(fs.readFileSync(path.join(folder,'result.json'),'utf8'));
 const before=JSON.stringify([context.actualPartialCandidate,context.actualPartialStatus]);
 const summary=vm.runInContext('gainTaskConclusion(actualPartialStatus,actualPartialCandidate)',context);
 assert(summary.review&&!summary.success&&summary.reason.includes('柔性待补测')&&!summary.reason.includes('joint1'));
 assert.equal(JSON.stringify([context.actualPartialCandidate,context.actualPartialStatus]),before,'Actual user records remain unchanged');
}
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

// New live states preserve actual experiment type and its raw evidence.
context.state.calibrationToolSelected={gain:{directory:'/tmp/gain-current'}};
context.state.telemetry={model_calibration:{directory:'/tmp/gain-other'}};
context.dragRelease={available:true,record_path:'/tmp/gain-current/drag-pulse.csv',axes:[{joint:'J1',quiet_stability_measured:true,quiet_position_span_rad:0,quiet_speed_rad_s:0,release_decay_measured:true,decay_s:.3,demonstrated_motion_preparation:true,release_record_path:'drag-taught-release.csv',preparation_motion_observed:true,release_motion_observed:true,reason:'zero_stiffness_release_decay_qualified_no_position_return_required'}]};
context.directionEvidence={axes:[{joint:'J1',input_measurement_complete:true,direct_response_complete:true,unresolved_input_direction_mask:3,unresolved_both_input_directions_at_ceiling:true,yield_observed:false,measurement_complete:false}]};
context.responseTrial={kind:'online_trajectory_PD_update',installed:true,previous_kp:[1],proposed_kp:[1.2],previous_kd:[.1],proposed_kd:[.1],models:[{joint:'J1',valid:false,response_establishment_trial:true,measured_error_bias_rad:.03,unresolved_trial_origin_kp:1,unresolved_trial_kp_ceiling:1.3,arx:[98765],predicted_cost_after:87654}],reasons:['online_measured_static_bias_or_same_direction_lag_bounded_P_response_trial_not_identification']};
context.taughtReleaseEvent={kind:'drag_demonstrated_motion_release',record_path:'/tmp/gain-current/drag-taught-release.csv',preparation_duration_s:2,actual_zero_stiffness_s:2.3,axes:[{joint:'J1',measurement_complete:false,reason:'drag_release_motion_unresolved_quiet_result_remains_available'}]};
const untouchedEvidence=JSON.stringify([context.dragRelease,context.directionEvidence,context.responseTrial]);
for(const language of ['zh-CN','en']) {
 context.state.language=language;
 for(const reason of ['drag_demonstrated_motion_release','drag_demonstrated_preparation']) {
  context.reason=reason;
  assert.notEqual(vm.runInContext('gainProbeMessageLabel(reason)',context),reason);
  assert.notEqual(translateError(reason,language),reason);
 }
 const drag=vm.runInContext('gainDragScopeEvidenceHtml(dragRelease)',context);
 assert(drag.includes(language==='en'?'Taught-motion release':'示教带动后释放')&&drag.includes('data-gain-evidence-path="/tmp/gain-current/drag-taught-release.csv"')&&drag.includes('/tmp/gain-current/drag-pulse.csv'),'Actual release CSV resolves against its own pulse record rather than a different live task');
 const event=vm.runInContext('gainExperimentHtml(taughtReleaseEvent)',context);
 assert(event.includes('data-gain-taught-release')&&event.includes(language==='en'?'Release not resolved':'释放尚未分辨')&&event.includes('/tmp/gain-current/drag-taught-release.csv'));
 const directions=vm.runInContext('gainCompliantScopeEvidenceHtml(directionEvidence)',context);
 assert(directions.includes('data-gain-direction-evidence')&&directions.includes(language==='en'?'Both inputs measured at their bounds; displacement unresolved':'两方向输入均测到边界，位移仍未分辨')&&!/达标|>Passed</.test(directions));
 const trial=vm.runInContext('gainExperimentHtml(responseTrial)',context);
 assert(trial.includes(language==='en'?'Bounded Kp response trial; Kd retained, model not identified':'有界 Kp 响应试探，Kd 保持；尚未辨识模型')&&trial.includes(language==='en'?'response trial record':'响应试探记录')&&!trial.includes('98765')&&!trial.includes('87654'),'A bounded response trial never claims a successful model or exposes stale prediction results');
 assert(trial.includes(language==='en'?'Recorded Kd before → proposed':'记录 Kd 原值 → 拟值'));
 assert(!vm.runInContext('gainEvidenceRecordHtml("ambiguous.csv")',context).includes('data-gain-evidence-path'),'An ambiguous old/live task directory does not produce a wrong raw-record link');
}
assert.equal(JSON.stringify([context.dragRelease,context.directionEvidence,context.responseTrial]),untouchedEvidence,'New evidence renderers preserve original records');
const recordButton={dataset:{gainEvidencePath:'/tmp/gain-current/drag-taught-release.csv'}};
context.recordRoot={querySelectorAll:selector=>selector==='[data-gain-evidence-path]'?[recordButton]:[]};
context.operation=fn=>fn();context.api={revealFile:path=>{context.revealedRecord=path;}};
vm.runInContext('bindGainEvidencePaths(recordRoot)',context);recordButton.onclick();
assert.equal(context.revealedRecord,'/tmp/gain-current/drag-taught-release.csv','Raw-record action uses the existing desktop reveal command without writing files');
console.log('New taught-release, directional-input and bounded-response UI checks passed');
