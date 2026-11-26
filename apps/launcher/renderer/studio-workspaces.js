'use strict';
// Navigation describes the user's task; existing business routes keep owning
// Core commands, live state, parameter validation and model lifetimes.
function studioSections() {
    return [
        ['onboarding','接入与优化','Connect & improve','readiness','lifecycle'],
        ['operation','运行与轨迹','Motion & trajectories','workbench','workbench:control'],
        ['parameters','参数配置','Parameters','parameters','profile_setup'],
        ['models','模型与标定','Models & calibration','model','model'],
        ['tuning','控制与整定','Control & tuning','calibration','workbench:tuning'],
        ['hardware','执行与硬件','Execution & hardware','hardware','hardware'],
        ['observation','状态与观测','State & observation','diagnostics','workbench:monitor'],
    ];
}
function studioSection() {
    if (state.page === 'lifecycle') return 'onboarding';
    if (state.page === 'profile_setup') return 'parameters';
    if (['hardware','run','diagnostics'].includes(state.page)) return 'hardware';
    if (state.page === 'events') return 'observation';
    if (state.page === 'workbench') return state.workbenchTab === 'monitor' ? 'observation' : state.workbenchTab === 'tuning' ? 'tuning' : 'operation';
    if (state.page === 'calibration') return state.calibrationEntry === 'observation' ? 'observation' : ['gain','response'].includes(state.calibrationEntry) ? 'tuning' : 'models';
    if (['model','geometry'].includes(state.page)) return 'models';
    return '';
}
function studioSectionLabel() {
    const section = studioSections().find(item=>item[0]===studioSection());
    return section ? busyText(section[1],section[2]) : pageLabel();
}
function studioNavigationHtml() {
    const entries=studioSections();
    const item=([key,zh,en,ico,route])=>`<button type="button" class="nav-item" data-studio-section="${key}" data-studio-route="${route}" title="${busyText(zh,en)}" aria-controls="page" ${!state.config.profile?'disabled':''}>${icon(ico)}<span>${busyText(zh,en)}</span></button>`;
    return `<section class="sidebar-nav-group"><div class="nav-caption">${busyText('机械臂工作区','WORKSPACE')}</div><nav aria-label="${busyText('工作区','Workspaces')}">${entries.slice(0,3).map(item).join('')}<div class="nav-caption studio-special-caption">${busyText('专项工具','SPECIALIST TOOLS')}</div>${entries.slice(3).map(item).join('')}</nav></section>`;
}
function studioRouteKey() {
    return state.page === 'workbench' ? `workbench:${state.workbenchTab}` : state.page === 'calibration' ? `calibration:${state.calibrationEntry}` : state.page;
}
function studioSubpages() {
    switch (studioSection()) {
        case 'operation': return [['workbench:control','关节目标','Joint targets'],['workbench:replay','示教与回放','Teach & replay']];
        case 'models': return [['model','模型查看','Model viewer'],['geometry','几何与定位','Geometry & positioning'],...calibrationAvailableTools().filter(item=>['gravity_verify','teaching','gravity','dynamics'].includes(item[0])).map(([key,zh,en])=>[`calibration:${key}`,zh,en])];
        case 'tuning': return [['workbench:tuning','响应与交互','Response & interaction'],...calibrationAvailableTools().filter(item=>['gain','response'].includes(item[0])).map(([key,zh,en])=>[`calibration:${key}`,zh,en])];
        case 'hardware': return [['hardware','设备与映射','Devices & mapping'],['run','运行适配层','Runtime adapters'],['diagnostics','连接诊断','Connection diagnostics']];
        case 'observation': return [['workbench:monitor','实时观测','Live observation'],['calibration:observation','外力估计校准','External-force calibration'],['events','事件与故障','Events & faults']];
        default:return [];
    }
}
function studioSubnavigation() {
    const pages=studioSubpages(); if(!pages.length)return '';
    const locked=state.page==='calibration'&&(modelCalibrationActive()||state.telemetry?.gravity_verification?.active);
    const nav=(items,key,active)=>`<nav class="workspace-subpage-nav studio-family-nav" data-studio-subpage-nav="family-${esc(key)}" aria-label="${esc(studioSectionLabel())}">${items.map(([route,zh,en])=>{const selected=active(route);return `<button type="button" class="workspace-subpage-button ${selected?'selected':''}" data-studio-route="${route}" ${locked&&studioRouteKey()!==route?'disabled':''} ${selected?'aria-current="page"':''}>${busyText(zh,en)}</button>`;}).join('')}</nav>`;
    if(studioSection()==='models'){
        const overview=[['model','模型查看','Model viewer'],['geometry','几何与模型标定','Geometry & model calibration']];
        const primary=nav(overview,'models',route=>route==='model'?state.page==='model':state.page!=='model');
        const tools=pages.filter(([route])=>route!=='model');
        return primary+(state.page==='model'?'':nav(tools,'model-tools',route=>studioRouteKey()===route));
    }
    return nav(pages,studioSection(),route=>studioRouteKey()===route);
}
async function studioNavigate(route) {
    const [page,subpage]=route.split(':');
    if(document.querySelector('[data-profile-field]')){try{collectProfileEditor();}catch{}}
    const previous={page:state.page,tab:state.workbenchTab,entry:state.calibrationEntry};
    if(page==='workbench')state.workbenchTab=subpage||'control';
    if(page==='calibration'){if(state.page==='calibration'&&(modelCalibrationActive()||state.telemetry?.gravity_verification?.active)&&subpage!==state.calibrationEntry)return;state.calibrationEntry=subpage||'gravity_verify';}
    if(page===state.page)render();else await navigate(page);
    if(state.page!==page){state.workbenchTab=previous.tab;state.calibrationEntry=previous.entry;render();return;}
    if(page==='workbench'&&subpage==='replay'){await refreshTeachTrajectories();render();}
}
function studioBreadcrumbHtml() {
    const section=studioSection()?studioSectionLabel():'';
    return `<span>${esc(state.config.profile||t('workspace'))}</span>${icon('arrow')}${section&&section!==pageLabel()?`<span>${esc(section)}</span>${icon('arrow')}`:''}<strong>${esc(pageLabel())}</strong>`;
}
function studioSyncNavigation() {
    document.querySelectorAll('[data-studio-section]').forEach(node=>{
        const selected=node.dataset.studioSection===studioSection();node.classList.toggle('active',selected);
        if(selected)node.setAttribute('aria-current','page');else node.removeAttribute('aria-current');
    });
    const breadcrumb=$('breadcrumb');if(breadcrumb)breadcrumb.innerHTML=studioBreadcrumbHtml();
    const head=$('page')?.querySelector(':scope > .page-head');
    if(head&&['model','geometry','hardware','run','diagnostics','events'].includes(state.page)&&!$('studio-family-navigation'))head.insertAdjacentHTML('afterend',`<div id="studio-family-navigation">${studioSubnavigation()}</div>`);
}
function studioContextWorkspaceHtml(content,kind) {
    return `<div class="studio-workspace studio-context-workspace" data-context-workspace="${esc(kind)}">${kind==='hardware'?workspaceControlsHtml():''}<div class="workbench-layout studio-workspace-layout"><details class="workspace-preview-disclosure" data-workspace-preview><summary><span>${busyText('模型对照','Model reference')}</span></summary><div class="workspace-preview-body"><section class="workbench-model studio-workspace-preview"><div id="workbench-viewport">${state.modelData?'':`<div class="empty-state">${t('modelNotLoaded')}</div>`}</div><div class="workbench-model-note">${workspaceModelNoteHtml()}</div></section><div class="workspace-preview-tools"></div></div></details><section class="workbench-task studio-workspace-task">${content}</section></div></div>`;
}
function studioHardwareObjects(editor) {
    const objects=[{key:'driver',label:busyText('驱动与执行','Driver & execution'),kind:'driver'}];
    for(const name of Object.keys(editor.hardware?.buses || {}))objects.push({key:'bus:'+name,label:name,kind:'bus',name});
    for(const name of Object.keys(editor.hardware?.damiao?.actuators || {}))objects.push({key:'joint:'+name,label:name,kind:'joint',name});
    return objects;
}
function studioHardwarePage() {
    const editor=state.profileEditor;
    if(!editor)return pageHead(busyText('执行与硬件','Execution & hardware'),'')+`<div class="empty-state">${busyText('参数尚未加载','Parameters are not loaded')}</div>`;
    const objects=studioHardwareObjects(editor),selected=objects.find(item=>item.key===state.studioHardwareObject)||objects[0];
    state.studioHardwareObject=selected.key;
    let fields='',scope=['hardware','profile_entry.hardware','core.control.runtime'];
    if(selected.kind==='bus'){
        fields=profileTree(editor.hardware.buses[selected.name],['hardware','buses',selected.name]);scope=['hardware.buses.'+selected.name];
    } else if(selected.kind==='joint'){
        fields=`<section class="studio-device-fields"><h3>${busyText('执行器地址与类型','Actuator address & type')}</h3>${profileTree(editor.hardware.damiao.actuators[selected.name],['hardware','damiao','actuators',selected.name])}</section><section class="studio-device-fields"><h3>${busyText('方向、比例与零位','Direction, ratios & offsets')}</h3>${editor.core?.calibration?.joints?.[selected.name]?profileTree(editor.core.calibration.joints[selected.name],['core','calibration','joints',selected.name]):`<p class="hint">${busyText('当前关节尚未配置映射','No mapping is configured for this joint')}</p>`}</section>`;
        scope=['hardware.damiao.actuators.'+selected.name,'core.calibration.joints.'+selected.name];
    } else {
        const driver=Object.fromEntries(Object.entries(editor.hardware?.damiao||{}).filter(([key])=>key!=='actuators'));
        fields=`<section class="studio-device-fields"><h3>${busyText('硬件驱动','Hardware driver')}</h3><div class="parameter-fields">${profileField(['profile_entry','hardware','plugin'],editor.profile_entry?.hardware?.plugin||'')}</div>${profileHardwareToolsHtml(editor)}${profileTree(driver,['hardware','damiao'])}</section>${profileExecutionHtml(editor)}`;
    }
    const fieldTemplate=document.createElement('template');fieldTemplate.innerHTML=fields;
    scope=[...fieldTemplate.content.querySelectorAll('[data-profile-field]')].map(field=>JSON.parse(field.dataset.profileField).join('.'));
    const content=`<header class="studio-task-heading"><div><h2>${busyText('设备与关节映射','Devices & joint mapping')}</h2><p class="hint">${busyText('与参数中心共享同一份草稿','Shared with the parameter center')}</p></div><button class="button" data-studio-review-scope="${esc(JSON.stringify(scope))}">${busyText('审阅此对象改动','Review this object')}</button></header><div class="studio-device-layout"><nav class="studio-device-directory" aria-label="${busyText('硬件对象','Hardware objects')}">${objects.map(item=>`<button type="button" class="button ${item.key===selected.key?'selected':''}" data-studio-hardware-object="${esc(item.key)}" aria-pressed="${item.key===selected.key}">${icon(item.kind==='joint'?'geometry':'hardware')}<span>${esc(item.label)}</span></button>`).join('')}</nav><section class="studio-device-editor"><h3 class="studio-device-name">${esc(selected.label)}</h3>${fields}</section></div>`;
    return pageHead(busyText('执行与硬件','Execution & hardware'),'',workspaceSessionActionHtml())+studioContextWorkspaceHtml(content,'hardware');
}

function studioRecordEvent(event) {
    const list=state.studioEvents ||= [];
    let kind=event.event,title='',detail=event;
    if(kind==='telemetry') {
        if(!workbenchConnected()||(event.session_id&&event.session_id!==state.session?.session_id))return;
        const current=event.data?.robot_state,prior=state.telemetry?.robot_state;
        if(!current||current===prior)return;
        kind=current==='FAULT'?'fault':'state';title=busyText('机械臂状态','Robot state')+` · ${current}`;detail={robot_state:current,previous:prior||null,session_id:event.session_id,...(current==='FAULT'?{fault:event.data?.fault||{code:event.data?.last_fault||'FAULT'},description:describeRuntimeFault(event.data)}:{})};
    } else if(kind==='session')title=busyText('会话','Session')+` · ${event.state}`;
    else if(kind==='error'){kind='error';title=String(event.error||busyText('后端错误','Backend error'));}
    else return;
    list.unshift({id:`${Date.now()}-${list.length}`,time:new Date().toISOString(),kind,title,detail});
    if(list.length>300)list.length=300;
    if(state.page==='events'&&event.event!=='session')refreshStudioEvents();
}
function studioEventExplanation(item) {
    const detail=item.detail||{},description=detail.description;
    const message=(typeof description==='string'?description:description?.message)||detail.error||detail.reason||detail.message||item.title;
    const impact=description?.advice||(item.kind==='session'?busyText('连接状态已更新；实时反馈以当前会话为准','Session state has changed; live feedback follows the current session'):item.kind==='state'?busyText('机械臂状态由原生控制会话上报','Robot state reported by the native control session'):busyText('查看连接诊断与当前反馈，按具体原因处理','Review connection diagnostics and current feedback for the reported cause'));
    return `<dl class="studio-event-facts"><dt>${busyText('发生了什么','What happened')}</dt><dd>${esc(localizedError(message))}</dd><dt>${busyText('影响与处理','Effect and action')}</dt><dd>${esc(impact)}</dd>${detail.fault?.joint_name?`<dt>${t('joint')}</dt><dd>${esc(detail.fault.joint_name)}</dd>`:''}</dl><div class="workspace-actions"><button class="button" data-studio-route="workbench:monitor">${busyText('查看当前观测','View current observation')}</button><button class="button" data-studio-route="diagnostics">${busyText('连接诊断','Connection diagnostics')}</button>${description?.action?setupActionHtml(description.action,busyText('查看相关配置','View related configuration')):''}</div>`;
}
function studioEventsContent() {
    const all=state.studioEvents||[],filter=state.studioEventFilter||'all',items=all.filter(item=>filter==='all'||(filter==='fault'?['error','fault'].includes(item.kind):!['error','fault'].includes(item.kind)));
    const selected=items.find(item=>item.id===state.studioEventId)||items[0];
    return `<header class="studio-task-heading"><div><h2>${busyText('当前工作区事件','Workspace events')}</h2><p class="hint">${busyText('记录本次打开工作区后的会话、状态变化与错误','Sessions, state changes and errors since this workspace was opened')}</p></div><label class="studio-field"><span>${busyText('筛选','Filter')}</span><select id="studio-event-filter"><option value="all" ${filter==='all'?'selected':''}>${busyText('全部事件','All events')}</option><option value="fault" ${filter==='fault'?'selected':''}>${busyText('故障与错误','Faults & errors')}</option><option value="state" ${filter==='state'?'selected':''}>${busyText('会话与状态','Sessions & state')}</option></select></label></header><div class="studio-record-layout"><div class="studio-record-list" aria-label="${busyText('事件列表','Event list')}">${items.length?items.map(item=>`<button type="button" class="studio-record-row ${selected?.id===item.id?'selected':''}" data-studio-event="${esc(item.id)}" aria-pressed="${selected?.id===item.id}"><span class="pill ${['fault','error'].includes(item.kind)?'error':''}">${esc(item.kind)}</span><strong>${esc(item.title)}</strong><time>${esc(new Date(item.time).toLocaleTimeString())}</time></button>`).join(''):`<div class="empty-state">${busyText('暂无匹配事件','No matching events')}</div>`}</div><section class="studio-record-detail">${selected?`<h3>${esc(selected.title)}</h3><p class="hint">${esc(new Date(selected.time).toLocaleString())}</p>${studioEventExplanation(selected)}<details><summary>${busyText('原始诊断','Raw diagnostics')}</summary><pre>${esc(JSON.stringify(selected.detail,null,2))}</pre></details>`:`<p class="hint">${busyText('事件发生后在这里查看完整详情','Event details will appear here')}</p>`}</section></div>`;
}
function studioEventsPage(){return pageHead(busyText('状态与观测','State & observation'),'',workspaceSessionActionHtml())+`<div id="studio-events">${studioEventsContent()}</div>`;}
function refreshStudioEvents(){const host=$('studio-events');if(host){host.innerHTML=studioEventsContent();bindStudioWorkspaces();window.serialArmStudio?.enhanceSelects();}}
function studioObservationChartHtml(){
    const names=state.telemetry?.joint_names?.length?state.telemetry.joint_names:state.modelData?.native?.config?.joint_names||state.profileEditor?.joint_names||state.info?.joint_names||[];
    const selected=names.includes(state.studioObservedJoint)?state.studioObservedJoint:names[0]||'';
    state.studioObservedJoint=selected;
    return `<section class="workspace-section studio-observation-chart"><div class="studio-task-heading"><h3>${busyText('关节时序','Joint history')}</h3><div class="studio-observation-fields"><label><span>${busyText('对象','Joint')}</span><select id="studio-observed-joint">${names.map(name=>`<option ${name===selected?'selected':''}>${esc(name)}</option>`).join('')}</select></label><label><span>${busyText('信号','Signal')}</span><select id="studio-observed-signal">${[['pos','位置 / 参考','Position / reference'],['vel','速度','Velocity'],['tor','力矩 / 推力','Effort'],['residual_raw','模型残差','Model residual']].map(([key,zh,en])=>`<option value="${key}" ${(state.studioObservedSignal||'pos')===key?'selected':''}>${busyText(zh,en)}</option>`).join('')}</select></label></div></div><canvas id="workbench-chart" height="240" tabindex="0" aria-label="${busyText('关节历史曲线；左右方向键移动时间游标','Joint history; left and right arrows move the time cursor')}"></canvas><output id="studio-observation-value" class="studio-observation-value" aria-live="off"></output><p class="hint">${busyText('来自当前会话的有效反馈；灰线为位置参考，过期或缺失区间断开显示','Current-session feedback; grey line shows position reference, stale or missing intervals are gaps')}</p></section>`;
}
function studioDrawObservationChart(){
    const canvas=$('workbench-chart');if(!canvas||canvas.getBoundingClientRect().width<=0||document.hidden)return;
    const w=canvas.getBoundingClientRect().width,h=240,ratio=devicePixelRatio||1,ctx=canvas.getContext('2d');canvas.width=w*ratio;canvas.height=h*ratio;ctx.scale(ratio,ratio);
    const style=getComputedStyle(canvas),token=name=>style.getPropertyValue(name).trim();ctx.fillStyle=token('--viz-canvas');ctx.fillRect(0,0,w,h);ctx.font=`12px ${token('--studio-mono-font')||'sans-serif'}`;ctx.fillStyle=token('--viz-axis');
    const samples=state.telemetryHistory.slice(-300),names=state.telemetry?.joint_names||[],name=state.studioObservedJoint||names[0],signal=state.studioObservedSignal||'pos',unit=jointNativeUnits(name||'')[signal==='pos'?'position':signal==='vel'?'velocity':'effort'];
    const series=samples.map(sample=>{const i=sample.joint_names?.indexOf(name);return sample.valid===false||sample.feedback_age_ms>500||i<0?null:sample.joint?.[signal]?.[i];});
    const reference=signal==='pos'?samples.map(sample=>{const i=sample.joint_names?.indexOf(name);return sample.valid===false||sample.feedback_age_ms>500||i<0?null:sample.joint?.ref_pos?.[i];}):[];
    const values=[...series,...reference].filter(Number.isFinite);if(!values.length){ctx.fillText(busyText('等待有效反馈','Waiting for valid feedback'),12,24);return;}
    const min=Math.min(...values),max=Math.max(...values),padding=Math.max((max-min)*.12,.01),lo=min-padding,hi=max+padding,times=samples.map(sample=>Number(sample.monotonic_ns)/1e9),duration=times.at(-1)-times[0],timed=times.every(Number.isFinite)&&duration>0;
    const left=70,right=w-16,top=32,bottom=h-30,px=i=>left+(timed?(times[i]-times[0])/duration:i/Math.max(1,samples.length-1))*(right-left),py=v=>bottom-(v-lo)/(hi-lo)*(bottom-top);
    for(let i=0;i<5;i++){const value=lo+(hi-lo)*i/4,y=py(value);ctx.strokeStyle=token('--viz-grid');ctx.beginPath();ctx.moveTo(left,y);ctx.lineTo(right,y);ctx.stroke();ctx.fillStyle=token('--viz-axis');ctx.fillText(value.toFixed(3),4,y+4);}
    const line=(data,color)=>{ctx.beginPath();let open=false;data.forEach((v,i)=>{if(!Number.isFinite(v)){open=false;return;}if(open)ctx.lineTo(px(i),py(v));else ctx.moveTo(px(i),py(v));open=true;});ctx.strokeStyle=color;ctx.lineWidth=1.8;ctx.stroke();};
    line(reference,token('--viz-axis'));line(series,token('--brand-solid'));ctx.fillStyle=token('--viz-axis');ctx.fillText(`${name} · ${signal} / ${unit}`,12,18);ctx.fillText(timed?`0 — ${duration.toFixed(2)} s`:`1 — ${samples.length} ${busyText('样本','samples')}`,left,h-8);
    const chosen=Number.isFinite(state.studioObservedTime)&&timed?times.reduce((best,time,i)=>Math.abs(time-state.studioObservedTime)<Math.abs(times[best]-state.studioObservedTime)?i:best,0):samples.length-1;
    const x=px(chosen);ctx.strokeStyle=token('--viz-axis');ctx.setLineDash([3,3]);ctx.beginPath();ctx.moveTo(x,top);ctx.lineTo(x,bottom);ctx.stroke();ctx.setLineDash([]);
    canvas.studioHistory={times,timed,left,right,duration};
    const output=$('studio-observation-value');if(output)output.textContent=`${name} · ${timed?(times[chosen]-times[0]).toFixed(3)+' s':'#'+(chosen+1)} · ${Number.isFinite(series[chosen])?series[chosen].toFixed(5):'—'} ${unit}${signal==='pos'?' · '+busyText('参考','Reference')+' '+(Number.isFinite(reference[chosen])?reference[chosen].toFixed(5):'—')+' '+unit:''}`;
}
async function studioReviewScope(scope) {
    await openSetupTarget({page:'profile_setup',field:scope[0]});
    if(state.page==='profile_setup')openProfileReview(scope);
}
function bindStudioWorkspaces(){
    document.querySelectorAll('[data-profile-field]').forEach(field=>{field.addEventListener('focus',()=>{let path;try{path=JSON.parse(field.dataset.profileField);}catch{return;}const names=state.profileEditor?.joint_names||[],joint=path.find(part=>names.includes(part));if(joint){state.studioContextJoint=joint;modelView?.select?.(joint);}});});
    const chart=$('workbench-chart');
    if(chart&&state.page==='workbench'&&state.workbenchTab==='monitor'){
        chart.onpointermove=event=>{const data=chart.studioHistory;if(!data?.timed)return;const fraction=Math.max(0,Math.min(1,(event.clientX-chart.getBoundingClientRect().left-data.left)/(data.right-data.left)));state.studioObservedTime=data.times[0]+fraction*data.duration;studioDrawObservationChart();};
        chart.onkeydown=event=>{if(!['ArrowLeft','ArrowRight','End'].includes(event.key))return;const data=chart.studioHistory;if(!data?.timed)return;event.preventDefault();const current=Number.isFinite(state.studioObservedTime)?data.times.reduce((best,time,i)=>Math.abs(time-state.studioObservedTime)<Math.abs(data.times[best]-state.studioObservedTime)?i:best,0):data.times.length-1;state.studioObservedTime=event.key==='End'?null:data.times[Math.max(0,Math.min(data.times.length-1,current+(event.key==='ArrowRight'?1:-1)))];studioDrawObservationChart();};
    }
    document.querySelectorAll('[data-studio-hardware-object]').forEach(button=>button.onclick=()=>{for(const field of document.querySelectorAll('[data-profile-field]'))try{readProfileInput(field);}catch{}state.studioHardwareObject=button.dataset.studioHardwareObject;render();});
    document.querySelectorAll('[data-studio-review-scope]').forEach(button=>button.onclick=()=>operation(async()=>{const scope=JSON.parse(button.dataset.studioReviewScope);await studioReviewScope(scope);}));
    studioSyncNavigation();
    document.querySelectorAll('[data-studio-route]').forEach(node=>node.onclick=()=>operation(()=>studioNavigate(node.dataset.studioRoute)));
    document.querySelectorAll('[data-studio-event]').forEach(node=>node.onclick=()=>{state.studioEventId=node.dataset.studioEvent;refreshStudioEvents();});
    if($('studio-event-filter'))$('studio-event-filter').onchange=e=>{state.studioEventFilter=e.target.value;refreshStudioEvents();};
    if($('studio-observed-joint'))$('studio-observed-joint').onchange=e=>{state.studioObservedJoint=e.target.value;studioDrawObservationChart();};
    if($('studio-observed-signal'))$('studio-observed-signal').onchange=e=>{state.studioObservedSignal=e.target.value;studioDrawObservationChart();};
    if(state.page==='workbench'&&state.workbenchTab==='monitor')requestAnimationFrame(studioDrawObservationChart);
}
