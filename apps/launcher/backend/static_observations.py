"""One static-data contract for gravity and full IDIM; never operate hardware."""
import csv
from collections import defaultdict
from pathlib import Path
import numpy as np


def load_static_groups(folder, joints):
    folder=Path(folder);accepted=folder/'accepted_static.csv'
    if not accepted.is_file():
        # Historic tasks may use approved-looking reverse groups in frames.csv.
        import json
        meta=json.loads((folder/'metadata.json').read_text())
        if (meta.get('measurement_policy')or{}).get('static_source')=='accepted_static.csv':
            raise ValueError('缺少已验收静态文件 accepted_static.csv')
        accepted=folder/'frames.csv'
    samples=defaultdict(list);flags=defaultdict(set);rejected=0
    with accepted.open(newline='',encoding='utf-8')as stream:
        for line,row in enumerate(csv.DictReader(stream),2):
            phase=row.get('phase');gid=row.get('pose_group');flag=row.get('validation')
            if phase not in ('static_reverse','static_forward')or row.get('valid')!='1' or not gid or gid=='0' or flag not in ('0','1'):continue
            flags[gid].add(flag)
            direction='reverse'if phase=='static_reverse'else'forward'
            if row.get('direction')not in (None,'',direction):raise ValueError('静态阶段与运动方向冲突')
            try:
                q=np.array([float(row['q:'+j])for j in joints]);v=np.array([float(row['dq:'+j])for j in joints]);tau=np.array([float(row['tau:'+j])for j in joints]);age=float(row.get('feedback_age_ms')or'nan')
                if not np.isfinite(np.r_[q,v,tau,age]).all()or not 0<=age<=30 or max(abs(v))>=.04:rejected+=1;continue
            except (KeyError,ValueError):rejected+=1;continue
            samples[(gid,direction)].append((q,tau,line))
    if any(len(x)!=1 for x in flags.values()):raise ValueError('同一姿态组训练/验证标签冲突')
    groups={}
    for (gid,direction),rows in samples.items():
        if len(rows)<30:continue
        rows=rows[-60:];q=np.array([x[0]for x in rows]);t=np.array([x[1]for x in rows])
        groups.setdefault(gid,dict(group=gid,validation=next(iter(flags[gid]))=='1',members=[]))['members'].append(dict(
            direction=direction,q=np.median(q,axis=0),tau=np.median(t,axis=0),
            torque_mad=1.4826*np.median(abs(t-np.median(t,axis=0)),axis=0),frames=len(samples[(gid,direction)]),csv_lines=[x[2]for x in rows]))
    result=sorted(groups.values(),key=lambda x:int(x['group'])if x['group'].isdigit()else x['group'])
    if not result:raise ValueError('没有满足质量判据的静态姿态')
    return result


def static_equations(engine, folder):
    groups=load_static_groups(folder,engine.meta['joint_names']);nj=len(engine.qi);rows=[]
    for g in groups:
        ys=np.array([engine.Y(m['q'],np.zeros(nj),np.zeros(nj))for m in g['members']]);tau=np.array([m['tau']for m in g['members']]);base=ys@engine.theta
        # Evaluate every approach at its ACTUAL q; averaging q is nonlinear and wrong.
        Y=ys.mean(0);y=tau.mean(0);paired=len(ys)==2
        half=.5*((base[0]-tau[0])-(base[1]-tau[1]))if paired else np.zeros(nj)
        spread=max(np.max(abs(g['members'][0]['q']-g['members'][1]['q'])),0.)if paired else None
        within=np.mean([m['torque_mad']for m in g['members']],axis=0)
        uncertainty=np.maximum(within,.02 if paired else .05)
        # Hysteresis is uncertainty, not a measured sensor offset or exact odd friction.
        if paired:uncertainty=np.maximum(uncertainty,abs(half))
        rows.append(dict(group=g['group'],held=g['validation'],Y=Y,baseline=Y@engine.theta,tau=y,
                         uncertainty=uncertainty,paired=paired,q_delta_rad=spread,half_difference=half,members=g['members']))
    return rows
