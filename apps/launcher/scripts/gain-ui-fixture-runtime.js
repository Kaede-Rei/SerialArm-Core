// This test harness runs the production render functions and event handlers.
// Device transport, model enablement and command APIs are deliberately absent.
const $=id=>document.getElementById(id);
const esc=value=>String(value??'').replace(/[&<>"']/g,c=>({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;',"'":'&#39;'}[c]));
const busyText=(zh,en)=>state.language==='en'?en:zh;
const t=key=>({joint:'关节',cancel:'取消',processing:'处理中'}[key]||key);
const localizedError=String;
const graphicsAvailabilityProblem=()=>'';
const workspaceModelNoteHtml=()=>'';
const calibrationAlignment=()=>({aligned:true});
const setupActionHtml=()=>'';
const calibrationTaskLoaded=()=>!!state.calibrationToolSelected.gain;
const calibrationToolHistoryHtml=()=>'<div class="fixture-history">任务记录（正式应用沿用原历史入口）</div>';
const workbenchConnected=()=>true;
let fixtureScene='idle';
let state={};
/* RENDERER_FUNCTIONS */
function scene(name){
 fixtureScene=name;
 const candidate=name==='available'?structuredClone(fixtureData.available):name==='empty-result'?structuredClone(fixtureData.empty):null;
 const names=(candidate||fixtureData.available).joint_names;
 state={language:'zh-CN',page:'calibration',calibrationEntry:'gain',config:{profile:'fixture'},gainMode:'all',gainTuningMode:'quick',gainFinePurpose:'rigid',gainRequirements:{},calibrationRuntimeCandidates:{},calibrationToolSelected:{gain:candidate?{directory:'fixture-record',gain_candidate:candidate}:name==='idle'?{directory:'fixture-trajectory'}:null},telemetry:{robot_state:name==='available'?'INACTIVE':'ACTIVE',joint_names:names,model_calibration:{workflow_kind:'gain',phase:name==='running'?'gain_adaptation':name==='failed'?'failed':name==='idle'?'waiting_replay_confirmation':'complete',task_id:candidate?.task_id||'ui-calibration-task',gain_protocol:'taught_trajectory',gain_tuning_mode:'quick',gain_budget_source_duration_s:candidate?.budget_source_duration_s||25,gain_target_budget_s:candidate?.target_budget_s||300,gain_time_budget_s:candidate?.time_budget_s||375,gain_elapsed_s:name==='running'?71:candidate?.actual_elapsed_s||239,gain_remaining_s:name==='running'?304:Math.max(0,(candidate?.time_budget_s||375)-(candidate?.actual_elapsed_s||239)),gain_probe_stage:'short_identification',gain_probe_kind:'native-short-1',gain_probe_message:'bounded_reference_identification',gain_active_target:'rigid_tracking',gain_probe_elapsed_s:1,gain_probe_duration_s:5,error:name==='failed'?'反馈中断，执行已结束':''}},modelData:{model:{joints:names.map(name=>({name,type:'revolute'}))}}};
 $('fixture-state').value=name;render();
 if(name==='running') {
  state.telemetry.model_calibration.gain_probe_diagnostic={stage:'trajectory_search',kind:'native',joint:'joint1',candidate_index:2,kp:35,kd:1.5,safety_kp_max:500,safety_kd_max:5,completed:true,reason:'dynamic_tracking_failed',measurement_samples:2000,samples:2010,continuous:true,limited_samples:0,reference_positive_span_rad:.04,reference_negative_span_rad:.04,actual_positive_span_rad:.039,actual_negative_span_rad:.038,response_rms_ratio:.98,metrics_passed:false,metrics:{rms_error_rad:.0008,peak_error_rad:.0022,overshoot_rad:.0004,settling_time_s:.4},requirements:{rms_error_rad:.001,peak_error_rad:.1*Math.PI/180,overshoot_rad:.001,settling_time_s:1}};
  refreshGainProbeStatus(state.telemetry.model_calibration);
 }
}
function render(){
 $('fixture-page').innerHTML=calibrationStandalonePanelHtml('gain');
 bindGainRequirements();bindGainCalibrationOptions();bindGainCandidateSelection();refreshGainApplyControls();
 const stop=$('model-calibration-cancel');if(stop)stop.onclick=()=>{state.telemetry.model_calibration.phase='cancelled';render();};
}
function tick(){const status=state.telemetry.model_calibration;status.gain_elapsed_s+=.005;status.gain_remaining_s=Math.max(0,status.gain_remaining_s-.005);status.gain_probe_elapsed_s+=.005;refreshGainProbeStatus(status);refreshGainApplyControls();}
function runDomTests(){
 const checks=[];const assert=(condition,name)=>{if(!condition)throw Error(name);checks.push(name);};
 scene('idle');
 assert(document.querySelectorAll('.gain-page-heading h3').length===1,'页面只保留一个标定标题');
 const field=document.querySelector('[data-gain-requirement="peak_error_rad"]');field.value='0.08';field.dispatchEvent(new Event('input',{bubbles:true}));
 $('gain-tuning-mode').value='fine';$('gain-tuning-mode').dispatchEvent(new Event('change',{bubbles:true}));
 assert(!!$('gain-target')&&state.gainMode==='rigid','精调模式显示用途选择');
 $('gain-target').value='servo_tracking';$('gain-target').dispatchEvent(new Event('change',{bubbles:true}));
 assert(state.gainMode==='servo_tracking'&&Math.abs(readGainRequirements().peak_error_rad-.08*Math.PI/180)<1e-10,'用途切换保留已输入验收目标');
 const times=[...document.querySelectorAll('.gain-time-summary dd')].map(node=>node.textContent);assert(times[0]==='5 分 0 秒'&&times[1]==='10 分 0 秒','目标时间与最长时间分别显示');
 scene('empty-result');
 assert(document.querySelectorAll('[data-gain-candidate-purpose]').length===0,'未完成测量用途不提供采用勾选');
 assert(document.querySelector('.gain-result-outcome strong').textContent==='标定已结束，有效测量不足','无可用结果显示真实空结果');
 assert(!document.querySelector('[data-tool-export="gain"]')&&$('calibration-gain-temp-apply').disabled,'无可用结果不能导出或应用');
 assert(!document.querySelector('.gain-result-outcome').textContent.includes('可应用'),'空结果不出现已测可应用提示');
 scene('available');
 const warning=document.querySelector('[data-gain-candidate-purpose="servo_tracking"]');
 assert(!!warning&&warning.checked&&!warning.disabled,'未达标但测量完整的用途仍可采用');
 assert(!$('calibration-gain-temp-apply').disabled,'警告实测结果沿用原应用语义');
 const detail=document.querySelector('[data-gain-result="rigid_tracking"] details');detail.open=true;
 tick();tick();assert(detail.open&&detail.isConnected,'周期刷新保留参数展开状态');
 document.querySelectorAll('[data-gain-candidate-purpose]').forEach(input=>{input.checked=false;input.dispatchEvent(new Event('change',{bubbles:true}));});
 assert($('calibration-gain-temp-apply').disabled&&document.querySelector('[data-tool-export="gain"]').disabled,'取消全部选择后禁用应用和导出');
 warning.checked=true;warning.dispatchEvent(new Event('change',{bubbles:true}));
 assert(!$('calibration-gain-temp-apply').disabled,'仅勾选警告用途也可应用');
 scene('running');const live=document.querySelector('#gain-probe details');live.open=true;tick();assert(live.open,'实时测量详情在刷新后保持展开');
 assert(document.querySelector('#gain-probe-diagnostic').textContent.includes('峰值跟踪误差（实测）')&&document.querySelector('#gain-probe-diagnostic').textContent.includes('峰值跟踪误差（上限）')&&!document.querySelector('#gain-probe-diagnostic').textContent.includes(' · '),'测量诊断分开标记实测值与上限');
 state.telemetry.model_calibration.gain_probe_diagnostic={kind:'short_experiment_parameters',kp:[20],kd:[.2],working_domain:[{protocol_kp_max:500,protocol_kd_max:5,protocol_kp_quantum:500/4095,protocol_kd_quantum:5/4095,working_kp_max:35,working_kd_max:.8,error_bound_rad:.01,velocity_error_bound_rad:.1,feedback_effort_budget_nm:2}]};tick();
 assert(document.querySelector('#gain-probe-diagnostic').textContent.includes('本次试验工作域')&&document.querySelector('#gain-probe-diagnostic').textContent.includes('协议 Kp 最大值')&&!document.querySelector('#gain-probe-diagnostic').textContent.includes('Safety'),'本次工作域与协议编码保留各自来源');
 assert($('gain-tuning-mode').disabled&&document.querySelector('[data-gain-requirement]').disabled,'运行中设置只读');
 $('model-calibration-cancel').click();assert(state.telemetry.model_calibration.phase==='cancelled'&&!$('model-calibration-cancel'),'停止后退出运行状态');
 scene('failed');assert(document.querySelector('#gain-probe-message').textContent.includes('反馈中断'),'执行失败保留实际原因');
 assert(!document.querySelector('[data-tool-export="gain"]'),'失败且无结果不显示导出');
 assert(gainCalibrationPhaseText({phase:'complete',task_id:'not-loaded'})==='任务已结束','候选未载入时只报告任务结束');
 scene('available');
 assert(document.querySelector('.gain-diagnostics').textContent.includes('补充路径验收（可选）'),'保留可选补充路径验收');
 assert(document.querySelector('.gain-diagnostics').textContent.includes('映射后的协议编码')&&document.querySelector('.gain-diagnostics').textContent.includes('本次试验 Kp 可执行范围'),'协议编码与本次试验约束分别展示');
 const preview=structuredClone(fixtureData.available);preview.model_initialization.configuration_applied=false;preview.model_initialization.protocol_gain={};state.calibrationToolSelected.gain.gain_candidate=preview;render();
 assert(document.querySelector('.gain-diagnostics').textContent.includes('模型建议增益（未应用、未实测）')&&!document.querySelector('.gain-diagnostics').textContent.includes('映射后的协议编码'),'未应用模型建议不伪造协议能力');
 scene('available');
 const legacy=structuredClone(fixtureData.available);legacy.targets.servo_tracking.validated=false;legacy.targets.servo_tracking.available=false;legacy.targets.servo_tracking.response_model=[{tracking_valid:false,disturbance_valid:false,reason:'insufficient_fresh_feedback'}];
 state.calibrationToolSelected.gain.gain_candidate=legacy;render();
 assert(document.querySelector('[data-gain-result="servo_tracking"]').textContent.includes('历史响应诊断'),'旧响应记录保留高级诊断');
 scene('available');
 const report={ok:true,checks};$('fixture-output').textContent=JSON.stringify(report,null,2);return report;
}
$('fixture-state').onchange=()=>scene($('fixture-state').value);
$('fixture-theme').onchange=()=>document.documentElement.classList.toggle('light',$('fixture-theme').value==='light');
$('fixture-tick').onclick=tick;
$('fixture-test').onclick=()=>{try{runDomTests();}catch(error){$('fixture-output').textContent=error.stack;throw error;}};
window.gainFixture={scene,tick,runDomTests,getState:()=>state};
const query=new URLSearchParams(location.search);const theme=query.get('theme')||'light';$('fixture-theme').value=theme;document.documentElement.classList.toggle('light',theme==='light');scene(query.get('state')||'idle');
