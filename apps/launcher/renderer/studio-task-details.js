/* Task stages are views over the existing controls: moving a panel preserves
   listeners, drafts and live IDs; choosing a stage never issues a command. */
const studioTaskViewState = new Map();
function studioTaskViewKey(key) {
    return JSON.stringify([state.config.profile_file || '',state.config.profile || '',key]);
}
function studioTaskStageLayout(host, key, stages, initial = 0) {
    if (!host || host.dataset.taskStagesReady || !stages.length) return;
    host.dataset.taskStagesReady = 'true';
    host.classList.add('task-stage-host');
    const identity = studioTaskViewKey(key), selected = studioTaskViewState.get(identity) ?? initial;
    const layout = document.createElement('div'); layout.className = 'task-stage-layout';
    const nav = document.createElement('nav'); nav.className = 'task-stage-navigation'; nav.setAttribute('aria-label',busyText('任务阶段','Task stages'));
    const content = document.createElement('div'); content.className = 'task-stage-content';
    const panels = [], buttons = [];
    let visibleStage = null;
    const choose = (index, focus = false) => {
        const changed=visibleStage!==null && visibleStage!==index;visibleStage=index;
        studioTaskViewState.set(identity,index);
        panels.forEach((panel,i) => { panel.hidden=i!==index; buttons[i].classList.toggle('selected',i===index); buttons[i].setAttribute('aria-current',i===index?'step':'false'); });
        if(changed)window.serialArmStudio?.enterSurface(panels[index]);
        if(focus)panels[index].querySelector('h3,h4,h5')?.focus({preventScroll:true});
    };
    stages.forEach((stage,index) => {
        const button = document.createElement('button'); button.type='button'; button.className='task-stage-button';
        const id=`task-stage-${key}-${index}`; button.setAttribute('aria-controls',id);
        button.innerHTML=`<span class="task-stage-index">${String(index+1).padStart(2,'0')}</span><span>${esc(stage.label)}${stage.caption?`<small>${esc(stage.caption)}</small>`:''}</span>`;
        button.onclick=()=>choose(index); nav.append(button); buttons.push(button);
        const panel=document.createElement('section'); panel.className='task-stage-panel'; panel.id=id;
        stage.nodes.filter(Boolean).forEach(node=>panel.append(node));
        if(!panel.querySelector('h3,h4')){const heading=document.createElement('h4');heading.textContent=stage.label;panel.prepend(heading);}
        panel.querySelector('h3,h4')?.setAttribute('tabindex','-1');
        const footer=document.createElement('div');footer.className='task-stage-footer';
        if(index>0){const back=document.createElement('button');back.type='button';back.className='button';back.textContent=busyText('上一阶段','Previous stage');back.onclick=()=>choose(index-1,true);footer.append(back);}
        if(index<stages.length-1){const next=document.createElement('button');next.type='button';next.className='button';next.textContent=busyText('查看下一阶段','View next stage');next.onclick=()=>choose(index+1,true);footer.append(next);}
        panel.append(footer);content.append(panel);panels.push(panel);
    });
    layout.append(nav,content);host.append(layout);choose(Math.max(0,Math.min(selected,stages.length-1)));
}

function bindStudioGeometryStages(root) {
    const flow=root.querySelector('#geometry-task-panel > .geometry-task-flow');if(!flow||flow.dataset.taskStagesReady)return;
    const cards=[...flow.children].filter(node=>node.matches('.geometry-card'));if(cards.length<2)return;
    const first=cards[0],second=cards[1];
    if(state.geometryTab==='fixed') {
        const steps=[...first.querySelectorAll(':scope > .geometry-step')];if(steps.length!==4)return;
        const setup=[...first.children].filter(node=>!steps.includes(node));
        studioTaskStageLayout(flow,'geometry-fixed',[
            {label:busyText('实验设置','Experiment setup'),nodes:setup},
            {label:busyText('训练采样','Training poses'),caption:`${state.geometryState?.train||0} / 10`,nodes:[steps[0]]},
            {label:busyText('求解候选','Fit candidate'),nodes:[steps[1]]},
            {label:busyText('独立验证','Independent validation'),caption:`${state.geometryState?.validation||0} / 5`,nodes:[steps[2],steps[3]]},
            {label:busyText('结果与采用','Results and adoption'),nodes:[second]},
        ],state.geometryState?.report?4:state.geometryState?.candidate?3:state.geometryState?.active?1:0);
        first.remove();
    }else{
        const saved=first.querySelector(':scope > .geometry-step');
        studioTaskStageLayout(flow,'geometry-repeat',[
            {label:busyText('触点与示教','Contact and teaching'),nodes:[first]},
            {label:busyText('轨迹记录','Saved trajectory'),nodes:[saved]},
            {label:busyText('往返验证','Round-trip validation'),nodes:[second]},
        ],state.geometryContactReady?2:0);
    }
}
function bindStudioCalibrationStages(root) {
    const idim=root.querySelector('#idim-workflow');
    if(idim&&!idim.dataset.taskStagesReady){
        const steps=[...idim.querySelectorAll(':scope > .calibration-step')];
        studioTaskStageLayout(idim,'dynamics',steps.map((node,index)=>({label:[busyText('采集数据','Acquisition data'),busyText('辨识设置','Identification'),busyText('结果对比','Results'),busyText('导出成果','Export')][index],nodes:[node]})),state.idimReport?2:0);
    }
    const supplement=root.querySelector('.idim-supplement > .calibration-advanced-card');
    if(supplement&&!supplement.dataset.taskStagesReady){
        const steps=[...supplement.querySelectorAll(':scope > .calibration-step')];
        studioTaskStageLayout(supplement,'dynamics-static',steps.map((node,index)=>({label:[busyText('已有数据','Saved data'),busyText('辨识设置','Fit settings'),busyText('审核与导出','Review and export')][index],nodes:[node]})),state.advancedReport?2:0);
    }
    const gain=root.querySelector('.gain-calibration-page');
    if(gain&&!gain.dataset.taskStagesReady){
        const settings=gain.querySelector(':scope > .gain-settings'),progress=gain.querySelector(':scope > .gain-probe-status');
        const actions=gain.querySelector(':scope > .gain-task-actions');if(actions)actions.classList.add('task-persistent-actions');
        const resultNodes=[...gain.children].filter(node=>node.matches('.gain-results,.gain-results-empty,.gain-result-actions,#calibration-gain-action-hint'));
        const setupNodes=[settings,...gain.querySelectorAll(':scope > .calibration-empty-guidance')];
        const progressNodes=[progress,...gain.querySelectorAll(':scope > .gain-details')];
        const empty=document.createElement('p');empty.className='hint task-progress-empty';empty.textContent=busyText('导入轨迹并开始后，在此查看采集进度','Import a trajectory and start calibration to view acquisition progress');progressNodes.push(empty);
        studioTaskStageLayout(gain,'gain',[
            {label:busyText('任务设置','Task setup'),nodes:setupNodes},
            {label:busyText('采集与测量','Acquisition'),nodes:progressNodes},
            {label:busyText('结果与采用','Review and apply'),nodes:resultNodes},
        ],state.calibrationToolSelected.gain?.gain_candidate?2:modelCalibrationActive()?1:0);
    }
    const gravity=root.querySelector('.model-calibration-card');
    if(gravity&&!gravity.dataset.taskStagesReady){
        const children=[...gravity.children], heading=gravity.querySelector(':scope > .workspace-section-head'), actions=gravity.querySelector(':scope > .workspace-actions');
        const result=gravity.querySelector(':scope > .task-gravity-results'), actionIndex=children.indexOf(actions);
        const setup=children.filter((node,index)=>node!==heading&&node!==actions&&node!==result&&index<actionIndex);
        const capture=children.filter((node,index)=>node!==heading&&node!==actions&&node!==result&&index>actionIndex);
        if(actions)actions.classList.add('task-persistent-actions');
        studioTaskStageLayout(gravity,'gravity',[
            {label:busyText('任务设置','Task setup'),nodes:setup},
            {label:busyText('采集与进度','Acquisition'),nodes:capture},
            {label:busyText('候选与验证','Candidate and validation'),nodes:[result]},
        ],currentModelCalibration()?.gravity_result?2:modelCalibrationActive()?1:0);
    }
}
function bindStudioTeachDetails(root) {
    const panel=root.querySelector('.teach-replay-panel');if(!panel||panel.dataset.taskDetailsReady)return;
    panel.dataset.taskDetailsReady='true';
    const sections=[...panel.querySelectorAll(':scope > .workspace-section')];if(sections.length!==3)return;
    const [recording,library,playback]=sections, selected=state.teachTrajectories.find(item=>item.id===state.teachSelectedId);
    const heading=document.createElement('div');heading.className='task-record-heading';
    const back=document.createElement('button');back.type='button';back.className='button';back.textContent=busyText('返回轨迹列表','Back to trajectories');
    const title=document.createElement('h3');title.textContent=selected?.name||busyText('记录详情','Record details');heading.append(back,title);
    const detail=document.createElement('section');detail.className='teach-record-detail';
    const list=library.querySelector('.teach-trajectory-list'), timeline=library.querySelector('.teach-timeline');
    const selectedPath=timeline?.previousElementSibling?.matches('.teach-storage-location')?timeline.previousElementSibling:null;
    const detailActions=document.createElement('div');detailActions.className='workspace-actions';
    for(const id of ['teach-export','teach-rename','teach-delete']){const action=library.querySelector(`#${id}`);if(action)detailActions.append(action);}
    detail.append(heading);if(selectedPath)detail.append(selectedPath);detail.append(detailActions);detail.append(playback);panel.append(detail);
    const preview=document.createElement('section');preview.className='teach-record-preview';
    const previewTitle=document.createElement('h3');previewTitle.textContent=selected?.name||busyText('轨迹预览','Trajectory preview');preview.append(previewTitle);
    const previewInfo=document.createElement('p');previewInfo.className='hint';previewInfo.textContent=selected?`${Number(selected.duration_s).toFixed(1)} s · ${selected.samples} ${busyText('点','samples')}`:busyText('选择记录后查看轨迹与回放设置','Select a record to preview its trajectory and playback settings');preview.append(previewInfo);
    const split=document.createElement('div');split.className='teach-record-browser';list.before(split);split.append(list,preview);
    const identity=studioTaskViewKey('teach-detail'), selectionKey=identity+':selected';
    if(selected&&studioTaskViewState.get(selectionKey)!==selected.id)studioTaskViewState.set(identity,false);
    studioTaskViewState.set(selectionKey,selected?.id||'');
    const showDetail=show=>{library.hidden=show;detail.hidden=!show;recording.hidden=show;if(timeline)(show?detail:preview).insertBefore(timeline,show?playback:preview.querySelector('button'));studioTaskViewState.set(identity,show);};
    back.onclick=()=>{showDetail(false);window.serialArmStudio?.enterSurface(library);list?.querySelector('.selected')?.focus({preventScroll:true});};
    if(selected){
        const open=document.createElement('button');open.type='button';open.className='button';open.textContent=busyText('打开所选记录详情','Open selected record');open.onclick=()=>{showDetail(true);window.serialArmStudio?.enterSurface(detail);};preview.append(open);
    }
    root.querySelectorAll('[data-teach-select]').forEach(button=>button.addEventListener('click',()=>studioTaskViewState.set(identity,false)));
    showDetail(!!selected&&(studioTaskViewState.get(identity)??false));
    if(selected&&timeline)refreshTeachTimeline();
}
function bindStudioCalibrationRecords(root) {
    const panel=root.querySelector('#calibration-task-panel'),history=panel?.querySelector(':scope > .calibration-tool-history');
    if(!panel||!history||history.dataset.taskRecordReady)return;
    history.dataset.taskRecordReady='true';
    const kind=state.calibrationEntry,identity=studioTaskViewKey(`records-${kind}`),toolbar=document.createElement('div');toolbar.className='task-record-toolbar';
    const open=document.createElement('button');open.type='button';open.className='button';
    open.textContent=busyText('浏览任务记录','Browse task records');
    const back=document.createElement('button');back.type='button';back.className='button';back.textContent=busyText('返回当前任务','Back to current task');
    const context=document.createElement('div');context.className='task-record-active-context';
    const selected=state.calibrationToolSelected[kind];
    if(selected){const label=document.createElement('span');label.className='hint';label.textContent=busyText('当前记录　','Current record　')+(selected.task_id||selected.directory);context.append(label);}
    toolbar.append(open,back,context);panel.prepend(toolbar);
    const candidates=history.querySelector(':scope > .candidate-rounds');if(candidates)panel.append(candidates);
    const taskNodes=[...panel.children].filter(node=>node!==toolbar&&node!==history);
    const priorHidden=new Map(taskNodes.map(node=>[node,node.hidden]));
    const setView=records=>{
        studioTaskViewState.set(identity,records);history.hidden=!records;open.hidden=records;back.hidden=!records;
        for(const node of taskNodes)node.hidden=records||priorHidden.get(node);
    };
    open.onclick=()=>setView(true);back.onclick=()=>setView(false);
    history.querySelectorAll('[data-tool-select],[data-tool-load]').forEach(button=>button.addEventListener('click',()=>studioTaskViewState.set(identity,false)));
    setView(studioTaskViewState.get(identity)??false);
}
function studioGainDraftChanges(candidate = state.calibrationToolSelected.gain?.gain_candidate) {
    const editor=state.profileEditor,purposes=selectedGainPurposes(candidate),names=editor?.core?.model?.joint_names || editor?.joint_names || [];
    if(!editor)throw Error(busyText('请先加载当前机械臂的参数','Load the current robot configuration first'));
    if(!gainCandidateDeployable(candidate,purposes))throw Error(busyText('请选择已完成测量的增益用途','Select measured gain uses'));
    if(purposes.some(key=>!['rigid','compliant','drag'].includes(key)))throw Error(busyText('当前参数中心支持刚性、柔性与拖拽用途，请取消旧版独立伺服用途','The parameter center supports rigid, compliant and drag gains; deselect legacy servo-only gains'));
    if(JSON.stringify(candidate.joint_names)!==JSON.stringify(names))throw Error(busyText('候选关节顺序与当前机械臂不一致','Candidate joint order differs from the current robot'));
    const currentFingerprint=state.modelData?.native?.config?.urdf_fingerprint;
    if(currentFingerprint&&candidate.urdf_fingerprint&&currentFingerprint!==candidate.urdf_fingerprint)throw Error(busyText('候选来自不同模型，请使用对应机械臂的标定记录','The candidate belongs to a different model; choose its matching robot'));
    const changes=[],groups=gainCandidateGroups(candidate);
    const add=(path,value)=>{if(typeof value==='number')validateProfileConstraint(path,value);changes.push({path,value});};
    for(const purpose of purposes){
        const value=groups[purpose];
        for(const term of ['kp','kd'])names.forEach((name,index)=>add(['core','control','controller',purpose,term,name],value.gains[term][index]));
        if(purpose==='rigid'){
            const hold=value.hold_compensation || {enabled:false};
            add(['core','control','hold_compensation','enabled'],hold.enabled);
            for(const key of ['integral_gain','max_torque','max_rate','contact_threshold','velocity_threshold','error_threshold']){
                const values=hold[key];if(values==null&&!hold.enabled)continue;
                if(!Array.isArray(values)||values.length!==names.length||values.some(number=>typeof number!=='number'||!Number.isFinite(number)))throw Error(busyText('候选缺少完整保持补偿参数：','Incomplete hold compensation: ')+key);
                names.forEach((name,index)=>add(['core','control','hold_compensation',key,name],values[index]));
            }
        }
    }
    return changes;
}
async function studioAdoptGainDraft({review = true} = {}) {
    const changes=studioGainDraftChanges();profileSavedValues();
    for(const {path,value} of changes){
        profileAssignPath(state.profileEditor,path,value);
        const id=path.join('.');delete state.profileRawInputs?.[id];delete state.profileFieldErrors?.[id];
    }
    state.profileDirty=profileChanges().length>0;state.profileSaveNotice=false;state.savePreview=null;
    const paths=changes.map(item=>item.path.join('.'));
    state.profileReviewSelection=profileChanges().filter(item=>paths.includes(item.id)).map(item=>item.id);
    if(review){await openSetupTarget({page:'profile_setup',group:'motion',field:changes[0]?.path.join('.')});if(state.page==='profile_setup')openProfileReview(paths);}
    return changes;
}
function bindStudioGainDraft(root) {
    const actions=root.querySelector('.gain-export-actions');if(!actions||actions.querySelector('[data-gain-adopt-draft]'))return;
    const button=document.createElement('button');button.type='button';button.className='button primary';button.dataset.gainAdoptDraft='true';button.textContent=busyText('采用所选用途到草稿','Adopt selected uses into draft');actions.prepend(button);
    const hint=document.createElement('p');hint.className='hint gain-draft-scope';hint.textContent=busyText('采用后进入参数改动审阅；保存默认值与临时应用到会话分别操作','Adoption opens parameter review; saving defaults and applying to this session are separate actions');actions.after(hint);
    const refresh=()=>{try{studioGainDraftChanges();button.disabled=false;button.title='';}catch(error){button.disabled=true;button.title=error.message;}};
    button.onclick=()=>operation(()=>studioAdoptGainDraft());
    root.querySelectorAll('[data-gain-candidate-purpose]').forEach(input=>input.addEventListener('change',refresh));refresh();
}
function bindStudioTaskDetails(root = document) {
    bindStudioGeometryStages(root);
    bindStudioCalibrationStages(root);
    bindStudioGainDraft(root);
    bindStudioTeachDetails(root);
    bindStudioCalibrationRecords(root);
}
