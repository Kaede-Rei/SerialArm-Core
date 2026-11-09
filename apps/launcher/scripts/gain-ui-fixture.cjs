/* Offline DOM/render fixture for the actual calibration renderer functions.
 * node scripts/gain-ui-fixture.cjs --out /tmp/serialarm-gain-ui [--record task/gain-candidate.json]
 * Open index.html in Electron or serve that directory; no backend or device API is loaded.
 */
const fs=require('node:fs');
const path=require('node:path');
const root=path.resolve(__dirname,'..');
const functions=[
  'canonicalGainPurpose','gainPurposeLabel','jointDisplaySpec','jointDisplayValue','jointMdkUnits','capabilityNumber',
  'modelCalibrationPhaseText','modelCalibrationActive','gainCandidateGroups','gainCandidatePurposeAvailable','gainCandidateDeployable',
  'gainCandidateAvailabilityLabel','gainCalibrationQualityWarning','gainTaskConclusion','gainWarningLabel','gainSelectionKey','selectedGainPurposes',
  'gainRequirementFieldsHtml','readGainRequirements','gainCalibrationHasLinearAxes','gainProbePhaseLabel','gainProbeStageLabel',
  'gainProbeMessageLabel','gainProbeStatusValues','gainMotionEstablishmentHtml','gainExperimentHtml','gainProbeEvidenceHtml','gainProbeStatusHtml','refreshGainProbeStatus',
  'bindGainRequirements','bindGainCandidateSelection','bindGainCalibrationOptions','gainTimeText','gainBudgetValues','gainBudgetHtml',
  'gainCalibrationPhaseText','gainCandidateResultCounts','gainCalibrationOptionsHtml','gainModelInitializationHtml','gainCheckStatus',
  'gainQualificationScopeHtml','gainResponseModelsHtml','gainTaughtPathMetricsHtml','gainCandidateParametersHtml','gainObserverAndBoundsHtml',
  'gainCandidateReviewHtml','gainPathValidationHtml','calibrationStandalonePanelHtml','refreshGainApplyControls','calibrationTaskGuidance','gainCalibrationConfirmHtml'
];
function extract(source,name){const start=source.indexOf(`function ${name}(`);if(start<0)throw Error('Missing function '+name);const next=source.indexOf('\nfunction ',start+1);return source.slice(start,next<0?undefined:next);}
function makeCandidate(kind='available'){
  const names=Array.from({length:6},(_,index)=>'joint'+(index+1));
  const pathValidation={independent:true,available:true,completed:true,validated:true,duration_s:25,measured_duration_s:25,
    metrics:names.map((name,index)=>({samples:5000,measurement_samples:5000,peak_error_rad:.001-index*.00008,rms_error_rad:.0003,velocity_rms_rad_s:.001,tail_error_rad:.0001,settling_s:.4}))};
  const hold={available:true,completed:true,validated:true,samples:260,fresh_samples:names.map(()=>260),worst_error_rad:.0001,worst_velocity_rad_s:.001,worst_drift_rad:.00001,reason:'qualified'};
  const gains={kp:[21.14,16.32,9.26,4.61,2.75,1.86],kd:[1.9,1.8,1.2,.5,.3,.2]};
  const candidate={schema:'serial_arm_gain_candidate',task_id:'ui-calibration-result',category:'all',joint_names:names,validation_scope:'independent_demonstration_trajectory',gain_tuning_mode:'quick',target_budget_s:300,time_budget_s:375,budget_source_duration_s:25,actual_elapsed_s:239,
    model_initialization:{source:'urdf_task_response_and_protocol_grid',measured:false,kp:gains.kp,kd:gains.kd,inertia_min:[.8,.6,.3,.1,.05,.02],inertia_max:[1,.8,.4,.2,.1,.04],working_kp_max:[70,65,60,50,40,30],working_kd_max:[5,5,4,3,2,1],task_frequency_hz:names.map(()=>.45),protocol_gain:Object.fromEntries(names.map(joint=>[joint,{kp_max:500,kd_max:5,kp_quantum:500/4095,kd_quantum:5/4095,source:'damiao_mit_declared_motor'}]))},warnings:[],experiments:[],candidate_comparisons:[],targets:{
    rigid_tracking:{...structuredClone(gains),available:true,validated:true,native_validation:structuredClone(pathValidation),hold_gains:structuredClone(gains),hold_verification:structuredClone(hold),qualification_scope:{trajectory:'demonstration_trajectory',hold:true}},
    compliant_tracking:{...structuredClone(gains),available:true,validated:true,native_validation:structuredClone(pathValidation),hold_gains:structuredClone(gains),hold_verification:structuredClone(hold),tested_speed_fraction:.65,rigid_reference_kp:gains.kp,rigid_reference_kd:gains.kd,qualification_scope:{trajectory:'demonstration_trajectory',hold:true}},
    compliant_drag:{kp:names.map(()=>0),kd:[.8,.7,.4,.2,.15,.1],available:true,validated:true,drag_validation:{available:true,completed:true,validated:true,samples:500,axes:names.map(joint=>({joint,samples:500,initial_velocity_rad_s:.05,peak_velocity_rad_s:.05,final_velocity_rad_s:.001,decay_s:.8}))},qualification_scope:{drag:true}},
    servo_tracking:{...structuredClone(gains),available:true,validated:false,native_validation:{...structuredClone(pathValidation),validated:false},admittance_required:false,admittance_validation:{required:false,status:'not_requested'},warnings:['tracking_reference_not_met'],qualification_scope:{trajectory:'demonstration_trajectory'}}}};
  candidate.targets.servo_tracking.native_validation.metrics[2].peak_error_rad=.0022;
  if(kind==='empty-result')for(const target of Object.values(candidate.targets)){target.available=false;target.validated=false;target.error='not_started';if(target.native_validation)target.native_validation={independent:true,available:false,completed:false,duration_s:25,metrics:[]};if(target.drag_validation)target.drag_validation={available:false,completed:false,samples:0,axes:[]};}
  return candidate;
}
function buildFixture(options={}){
 const destination=path.resolve(options.out||'/tmp/serialarm-gain-ui');fs.mkdirSync(destination,{recursive:true});
 const source=fs.readFileSync(path.join(root,'renderer/app.js'),'utf8');
 const data={available:makeCandidate(),empty:options.record?JSON.parse(fs.readFileSync(options.record,'utf8')):makeCandidate('empty-result')};
 for(const name of ['styles.css','studio.css'])fs.copyFileSync(path.join(root,'renderer',name),path.join(destination,name));
 const runtime=`'use strict';\nconst fixtureData=${JSON.stringify(data)};\n`+fs.readFileSync(path.join(__dirname,'gain-ui-fixture-runtime.js'),'utf8');
 fs.writeFileSync(path.join(destination,'fixture.js'),runtime.replace('/* RENDERER_FUNCTIONS */',functions.map(name=>extract(source,name)).join('\n')));
 fs.writeFileSync(path.join(destination,'index.html'),`<!doctype html><html lang="zh-CN" class="light"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>SerialArm calibration UI fixture</title><link rel="stylesheet" href="styles.css"><link rel="stylesheet" href="studio.css"><style>body{height:auto;min-height:100vh}.fixture-shell{max-width:1120px;padding:24px;margin:auto}.fixture-controls{display:flex;align-items:center;gap:12px;flex-wrap:wrap;margin-bottom:24px;font-size:12px}.fixture-controls>span{color:var(--muted)}.fixture-controls select{height:36px;padding:0 12px;color:var(--text);background:var(--subtle);border:.5px solid var(--line);border-radius:6px}.fixture-output{white-space:pre-wrap;font-size:12px}.fixture-history{margin-top:24px;padding-top:16px;border-top:.5px solid var(--line);font-size:12px;color:var(--muted)}@media(max-width:600px){.fixture-shell{padding:12px}}</style></head><body><main class="fixture-shell"><div class="fixture-controls"><label>离线界面状态 <select id="fixture-state"><option value="idle">空闲</option><option value="running">运行中</option><option value="failed">执行失败</option><option value="empty-result">未获得结果</option><option value="available">已有候选</option></select></label><label>主题 <select id="fixture-theme"><option value="light">明亮</option><option value="dark">暗色</option></select></label><button class="button" id="fixture-tick">更新一周期</button><button class="button" id="fixture-test">运行交互验收</button><span>合成或只读记录数据，无后端及设备接口</span></div><div id="fixture-page"></div><pre id="fixture-output" class="fixture-output"></pre></main><script src="fixture.js"></script></body></html>`);
 return {directory:destination,index:path.join(destination,'index.html')};
}
if(require.main===module){const args=process.argv.slice(2);const value=name=>{const i=args.indexOf(name);return i<0?undefined:args[i+1];};console.log(JSON.stringify(buildFixture({out:value('--out'),record:value('--record')})));}
module.exports={buildFixture,makeCandidate,extract,functions};
