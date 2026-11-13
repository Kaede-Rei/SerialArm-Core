'use strict';
let gainSnapshot=null;
const element=id=>document.getElementById(id);
const numeric=value=>Number.isFinite(value)?value.toFixed(3):'—';
function paintGainMonitor(snapshot=gainSnapshot,now=Date.now()) {
    gainSnapshot=snapshot;
    const en=snapshot?.language==='en',s=snapshot||{};
    const text=(zh,enText)=>en?enText:zh;
    const light=s.theme==='light'||(s.theme!=='dark'&&matchMedia('(prefers-color-scheme: light)').matches);
    document.documentElement.classList.toggle('light',light);
    document.documentElement.dataset.fontLigatures=s.font_ligatures===false?'off':'on';
    document.documentElement.lang=en?'en':'zh-CN';
    document.title=element('gain-live-title').textContent=text('实时执行增益','Live executed gains');
    const age=Number.isFinite(s.received_at)&&Number.isFinite(s.feedback_age_ms)?s.feedback_age_ms+Math.max(0,now-s.received_at):Infinity;
    const fresh=s.connected===true&&s.valid===true&&age<=250;
    const status=element('gain-live-status');status.dataset.state=fresh?'live':'stale';
    status.textContent=!s.connected?text('已断开，显示最后快照','Disconnected — last snapshot'):!fresh?text('反馈已过期，显示最后快照','Stale feedback — last snapshot'):s.active?text('标定中，实时更新','Calibrating — live'):text('当前运行参数','Current runtime gains');
    element('gain-live-source').textContent=text('控制周期实际输出的 Kp / Kd','Actual Kp / Kd from the control-cycle output');
    element('gain-live-purpose-label').textContent=text('当前用途','Current use');
    element('gain-live-phase-label').textContent=text('当前阶段','Current stage');
    element('gain-live-age-label').textContent=text('反馈年龄','Feedback age');
    element('gain-live-purpose').textContent=s.purpose||'—';element('gain-live-phase').textContent=s.phase||'—';
    element('gain-live-age').textContent=Number.isFinite(age)?Math.round(age)+' ms':'—';
    element('gain-live-task').textContent=s.task_id||text('尚无标定任务','No calibration task');
    element('gain-live-note').textContent=text('关闭此窗口不影响标定','Closing this window leaves calibration running');
    const headings=[text('关节','Joint'),'Kp','Kd',text('实际','Actual'),text('参考','Reference'),text('参考 − 实际','Reference − actual'),text('PD 力矩 Nm','PD torque Nm')];
    const head=element('gain-live-columns');if(head.dataset.language!==String(en)){head.replaceChildren(...headings.map(label=>{const th=document.createElement('th');th.textContent=label;return th;}));head.dataset.language=String(en);}
    const body=element('gain-live-rows'),rows=Array.isArray(s.rows)?s.rows:[],signature=JSON.stringify(rows.map(r=>r.name));
    if(body.dataset.names!==signature){body.replaceChildren();for(const row of rows){const tr=document.createElement('tr');for(let i=0;i<7;i++)tr.append(document.createElement(i===0?'th':'td'));body.append(tr);}body.dataset.names=signature;}
    rows.forEach((row,index)=>{const angle=value=>Number.isFinite(value)&&Number.isFinite(row.factor)?numeric(value*row.factor)+' '+row.unit:'—';
        const values=[row.name,numeric(row.kp),numeric(row.kd),angle(row.actual),angle(row.reference),angle(Number.isFinite(row.reference)&&Number.isFinite(row.actual)?row.reference-row.actual:null),numeric(row.pd_torque)];
        Array.from(body.children[index].children).forEach((cell,i)=>{if(cell.textContent!==values[i])cell.textContent=values[i];});});
    if(!rows.length){const tr=document.createElement('tr'),cell=document.createElement('td');cell.colSpan=7;cell.className='gain-live-empty';cell.textContent=text('等待实际控制周期参数','Waiting for actual control-cycle gains');tr.append(cell);body.replaceChildren(tr);}
}
window.gainMonitor.onSnapshot(paintGainMonitor);
window.gainMonitor.current().then(paintGainMonitor);
setInterval(()=>paintGainMonitor(),250);
