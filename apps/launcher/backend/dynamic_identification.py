"""Global/local inverse-dynamics identification from archived observations.

No device access. Derivatives come exclusively from zero-phase filtered actual
positions. A local solve fixes trusted blocks BEFORE fitting; output is always
the full selected inertial model, never a post-fit mask or mass/COM-only splice.
"""
from __future__ import annotations
import csv
import hashlib
import json
import math
import platform
from pathlib import Path
import shutil
import uuid
import xml.etree.ElementTree as ET
import numpy as np

PHASES = ('static_reverse', 'static_forward', 'friction_reverse_slow',
          'friction_forward_slow', 'friction_reverse_fast', 'friction_forward_fast')
SOLVER = 'joint_physical_idim_v2'


def _plain(value):
    if isinstance(value, np.ndarray): return value.tolist()
    if isinstance(value, np.generic): return value.item()
    if isinstance(value, dict): return {str(k): _plain(v) for k, v in value.items()}
    if isinstance(value, (tuple, list)): return [_plain(v) for v in value]
    if isinstance(value, float) and not math.isfinite(value): return None
    return value


def _sha(path): return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def _json(path, value):
    Path(path).write_text(json.dumps(_plain(value), ensure_ascii=False, indent=2,
                                    allow_nan=False) + '\n', encoding='utf-8')


def _receipt(folder):
    folder = Path(folder)
    return {name: _sha(folder/name) if (folder/name).is_file() else None
            for name in ('frames.csv', 'metadata.json', 'result.json',
                         'accepted_static.csv', 'source/model.urdf', 'source/core.yaml')}


def load_observations(directory):
    """Preserve invalid rows as segment boundaries, never bridge over a gap."""
    folder = Path(directory).expanduser().resolve()
    meta = json.loads((folder/'metadata.json').read_text())
    names = meta.get('joint_names')
    if not isinstance(names, list) or not names or len(set(names)) != len(names):
        raise ValueError('采样关节顺序无效')
    if any((meta.get('units') or {}).get(k, unit) != unit for k, unit in
           [('position', 'rad'), ('velocity', 'rad/s'), ('torque', 'Nm')]):
        raise ValueError('完整辨识需要明确的 rad、rad/s、Nm 单位')
    with (folder/'frames.csv').open(newline='', encoding='utf-8') as stream:
        reader = csv.DictReader(stream)
        rows = list(reader)
    if len(rows) < 30: raise ValueError('原始记录不足，请先执行短采集验收')
    ns = np.array([int(row['monotonic_ns']) for row in rows], dtype=np.int64)
    if np.any(np.diff(ns) <= 0): raise ValueError('原始时间戳不单调，不能构造动力学导数')
    t = (ns-ns[0])/1e9
    def matrix(prefix):
        try: return np.array([[float(row[prefix+':'+j]) for j in names] for row in rows])
        except (KeyError, ValueError, TypeError) as exc:
            raise ValueError('记录缺少有效 '+prefix+' 反馈列') from exc
    q, v, tau = matrix('q'), matrix('dq'), matrix('tau')
    phases = np.array([row.get('phase', '') for row in rows])
    age = np.array([float(row.get('feedback_age_ms') or 'nan') for row in rows])
    good = np.array([row.get('valid') == '1' for row in rows])
    good &= (np.isfinite(q).all(1) & np.isfinite(v).all(1) & np.isfinite(tau).all(1)
             & np.isfinite(age) & (age >= 0) & (age <= 30) & np.isin(phases, PHASES))
    # Slice complete ~10s blocks exactly once, independent of fit/filter scope.
    segments, lo = [], None
    for i in range(len(t)+1):
        boundary = (i == len(t) or not good[i] or
                    (i > 0 and (t[i]-t[i-1] > .025 or phases[i] != phases[i-1])))
        if boundary and lo is not None:
            start = lo
            while start < i:
                end = min(i, int(np.searchsorted(t, t[start]+10., side='left')))
                if end-start >= 12:
                    segments.append(dict(id=len(segments), lo=start, hi=end,
                                         phase=str(phases[start])))
                start = max(end, start+1)
            lo = None
        if i < len(t) and good[i] and lo is None: lo = i
    if not segments: raise ValueError('没有连续有效自动运动段')
    return dict(folder=folder, metadata=meta, names=names, t=t, q=q, v=v,
                tau=tau, phase=phases, valid=good, segments=segments)


def audit_record(directory):
    from kinematic_consistency_audit import audit_angular_kinematics
    d = load_observations(directory)
    before = _receipt(d['folder'])
    motion = audit_angular_kinematics(d['t'], d['q'], d['v'], phase=d['phase'],
                                    valid=d['valid'], joint_names=d['names'])
    phases = {p: int(np.sum(d['valid'] & (d['phase'] == p))) for p in PHASES}
    observed = {p for p, n in phases.items() if n}
    sufficiently_long={s['phase']for s in d['segments']if d['t'][s['hi']-1]-d['t'][s['lo']]>=3.5}
    required = set(PHASES)
    warnings = []
    if required-observed: warnings.append('缺少双向静态/双向双速度阶段: '+', '.join(sorted(required-observed)))
    cap = d['metadata'].get('actuator_capabilities') or {}
    cap_ok = (cap.get('source') == 'hardware_backend_after_successful_activate'
              and len(cap.get('axes', [])) == len(d['names'])
              and all(all(math.isfinite(float(a.get(k,0))) and float(a.get(k,0))>0
                          for k in ('max_pos','max_vel','max_effort')) for a in cap.get('axes',[])))
    if not cap_ok: warnings.append('未归档可信的连接后逐轴量程，不能确认采集编解码范围')
    from model_calibration_source import verified_source_urdf
    root=ET.parse(verified_source_urdf(d['folder'],d['metadata'])).getroot()
    unmeasured=[x.get('name')for x in root.findall('joint')
                if x.get('type')!='fixed' and x.get('name')not in d['names']]
    if unmeasured:warnings.append('末端未采样关节须保持已知固定位置；辨识时声明其位置: '+', '.join(unmeasured))
    if required-sufficiently_long:warnings.append('部分阶段不足连续 3.5 秒，不能完成独立滤波: '+', '.join(sorted(required-sufficiently_long)))
    if before != _receipt(d['folder']): raise ValueError('审计期间原数据变化')
    return dict(schema='serial_arm_idim_audit_v1', task_id=d['metadata'].get('task_id'),
                phases=phases, missing_phases=sorted(required-observed),
                kinematics=motion, actuator_capabilities_recorded=bool(cap_ok),
                unmeasured_joints=unmeasured,
                measurement_ready=bool(cap_ok and motion['status']=='unit_scale_consistent'),
                can_attempt_position_derived_fit=True,
                acquisition_complete=required.issubset(sufficiently_long),
                load_description_recorded=bool(str(d['metadata'].get('load_description')or'').strip()not in ('','unspecified')),
                torque_physically_calibrated=False, derivative_source='offline_filtered_actual_position',
                warnings=warnings, input_sha256=before, source_unchanged=True)


class Model:
    def __init__(self, folder, fixed_joint_positions=None, load_description=None):
        import pinocchio as pin
        from full_inertial import (_prepare_inputs, _apply_recorded_gravity,
                                   _articulated_mapping, _sampling_map)
        self.folder, self.meta, _, self.source, _ = _prepare_inputs(folder)
        self.recorded_load_description=self.meta.get('load_description')or'unspecified'
        self.operator_load_description=str(load_description or '').strip()
        if self.operator_load_description:
            if self.recorded_load_description not in ('','unspecified',self.operator_load_description):raise ValueError('补充装机说明与采集记录冲突，请核对实际会话')
            self.meta=dict(self.meta,load_description=self.operator_load_description)
        self.pin = pin
        self.model = _apply_recorded_gravity(pin.buildModelFromUrdf(str(self.source)), self.meta)
        self.root = ET.parse(self.source).getroot()
        self.mapped, _ = _articulated_mapping(self.root, self.model, self.meta['joint_names'])
        self.qi, self.vi, self.cols = _sampling_map(self.model, self.mapped)
        self.theta = np.concatenate([self.model.inertias[j].toDynamicParameters()
                                     for j in range(1, self.model.njoints)])
        self.prior = self.theta[self.cols].copy()
        self.neutral = pin.neutral(self.model)
        self.unmeasured=[str(self.model.names[j])for j in range(1,self.model.njoints)
                         if j not in {x[0]for x in self.mapped}]
        self.fixed_joint_positions=dict(fixed_joint_positions or {})
        if not set(self.fixed_joint_positions).issubset(self.unmeasured):
            raise ValueError('固定位置只能指定模型中未采样的关节，不能覆盖实际反馈')
        from full_inertial import _assign_configuration
        for name,value in self.fixed_joint_positions.items():
            if not isinstance(value,(int,float))or not math.isfinite(value):raise ValueError('未采样关节位置无效')
            joint=self.model.joints[self.model.getJointId(name)]
            if joint.nq not in (1,2)or joint.nv!=1:raise ValueError('未采样关节不是标量自由度')
            _assign_configuration(self.model,self.neutral,[joint.idx_q],[float(value)])
        self.unmeasured_known=set(self.fixed_joint_positions)==set(self.unmeasured)
        self.zero = np.zeros(self.model.nv)
        self.data = self.model.createData()

    def Y(self, q, v, a):
        from full_inertial import _assign_configuration
        qq = self.neutral.copy(); vv = self.zero.copy(); aa = self.zero.copy()
        _assign_configuration(self.model, qq, self.qi, q)
        vv[self.vi] = v; aa[self.vi] = a
        regressor=np.asarray(self.pin.computeJointTorqueRegressor(
            self.model, self.data, qq, vv, aa)).reshape(self.model.nv,-1)
        return regressor[self.vi].copy()

    def scope(self, mode, locked_links):
        from gravity_legacy import _scope
        if mode == 'global' and locked_links: raise ValueError('全局模式不能指定可信 Link')
        locked = _scope(self.mapped, mode, locked_links)
        if mode == 'local' and not locked: raise ValueError('局部模式必须明确指定可信 Link，全部自由请用全局模式')
        free = np.concatenate([np.arange(10*i, 10*i+10) for i, (_, name, _) in enumerate(self.mapped)
                               if name not in locked])
        return locked, free

    def basis(self, free):
        """Use model-only random states, never validation observations."""
        rng = np.random.default_rng(201997); rows = []
        j = len(self.qi)
        for _ in range(240):
            rows.append(self.Y(rng.uniform(-1.5,1.5,j), rng.uniform(-1,1,j),
                               rng.uniform(-2,2,j))[:, np.asarray(self.cols)[free]])
        _, s, vt = np.linalg.svd(np.concatenate(rows), full_matrices=False)
        rank = int(np.sum(s > max(1e-12, s[0]*1e-10)))
        if not rank: raise ValueError('所选自由参数没有动力学可观察方向')
        self.null_basis = vt[rank:].T
        return vt[:rank].T, s


def preprocess(observations, engine, free, basis, cutoff=5.):
    from scipy.signal import butter, sosfiltfilt
    if cutoff not in (2., 5., 10.): raise ValueError('位置滤波带宽仅支持已核验的 2/5/10 Hz')
    d = observations; packs = []; j = len(d['names'])
    for seg in d['segments']:
        lo, hi = seg['lo'], seg['hi']; t = d['t'][lo:hi]
        if t[-1]-t[0] < 3.5: continue
        grid = np.linspace(t[0], t[-1], len(t)); dt = grid[1]-grid[0]; fs = 1/dt
        if fs < 4*max(cutoff, 5.): raise ValueError('采样率不足以使用指定滤波配置')
        q = np.column_stack([np.interp(grid,t,d['q'][lo:hi,k]) for k in range(j)])
        tau = np.column_stack([np.interp(grid,t,d['tau'][lo:hi,k]) for k in range(j)])
        qf = sosfiltfilt(butter(4,cutoff,fs=fs,output='sos'),q,axis=0)
        v = np.gradient(qf,dt,axis=0,edge_order=2)
        a = np.gradient(v,dt,axis=0,edge_order=2)
        xs, baselines = [], []
        for qi, vi, ai in zip(qf,v,a):
            y = engine.Y(qi,vi,ai); f = np.zeros((j,3*j))
            for axis in range(j): f[axis,3*axis:3*axis+3] = [1.,np.sign(vi[axis]),vi[axis]]
            xs.append(np.concatenate([y[:,np.asarray(engine.cols)[free]]@basis,f],axis=1))
            baselines.append(y@engine.theta)
        # Same anti-alias filter for every regressor, known baseline and target.
        sos = butter(4,5.,fs=fs,output='sos')
        x = sosfiltfilt(sos,np.asarray(xs),axis=0)
        baseline = sosfiltfilt(sos,np.asarray(baselines),axis=0)
        target = sosfiltfilt(sos,tau,axis=0)
        ids = np.arange(0,len(grid),max(1,int(round(fs/20.))))
        ids = ids[(grid[ids]>grid[0]+1.5)&(grid[ids]<grid[-1]-1.5)]
        if not len(ids): continue
        packs.append(dict(X=x[ids], baseline=baseline[ids], tau=target[ids],
                          moving=abs(v[ids])>.03, segment=np.full(len(ids),seg['id']),
                          phase=np.full(len(ids),seg['phase']), time=grid[ids],
                          q=qf[ids], v=v[ids], a=a[ids]))
    if not packs: raise ValueError('有效连续段太短，需至少 3.5 秒以独立滤波并去除边缘')
    z = {key:np.concatenate([p[key] for p in packs]) for key in packs[0]}
    if np.any(z['moving'].sum(0)<30): raise ValueError('至少一轴动态激励不足 30 个有效方程，请检查轨迹覆盖')
    return z


def solve(z, rank, method='WLS', prior_only=False):
    if method not in ('OLS','WLS','IRLS'): raise ValueError('未知动力学估计器')
    nj = z['tau'].shape[1]
    x = z['X'].reshape(-1,rank+3*nj); y = (z['tau']-z['baseline']).ravel()
    axes = np.tile(np.arange(nj),len(z['tau'])); mask = z['moving'].ravel()
    if prior_only: x = x[:,rank:]
    A, b, ax = x[mask], y[mask], axes[mask]
    scale = np.maximum(np.linalg.norm(A,axis=0),1e-10); S = A/scale
    def ls(weight):
        return np.linalg.lstsq(S*np.sqrt(weight)[:,None],b*np.sqrt(weight),rcond=1e-8)[0]/scale
    beta = ls(np.ones(len(b))); sigma = np.ones(nj); iterations = 1
    if method in ('WLS','IRLS'):
        residual = A@beta-b
        sigma = np.array([max(float(np.sqrt(np.mean(residual[ax==j]**2))),1e-5) for j in range(nj)])
        weight = 1/sigma[ax]**2; beta = ls(weight); iterations += 1
        if method == 'IRLS':
            for _ in range(50):
                h = np.minimum(1.,1.345/np.maximum(abs((A@beta-b)/sigma[ax]),1e-15))
                nxt = ls(weight*h); iterations += 1
                diff = np.linalg.norm(A@(nxt-beta))/max(np.linalg.norm(A@beta),1e-12)
                beta = nxt
                if diff < 1e-7: break
    if prior_only: beta = np.r_[np.zeros(rank),beta]
    sv = np.linalg.svd(S,compute_uv=False)
    return dict(beta=beta, sigma=sigma, iterations=iterations,
                equation_rank=int(np.sum(sv>max(1e-12,sv[0]*1e-8))),
                equation_columns=A.shape[1], singular_values=sv)


def score(z, fit):
    err = z['baseline']+np.einsum('nji,i->nj',z['X'],fit['beta'])-z['tau']
    return dict(rms_nm=float(np.sqrt(np.mean(err[z['moving']]**2))),
                joint_rms_nm=[float(np.sqrt(np.mean(err[z['moving'][:,j],j]**2))) for j in range(err.shape[1])],
                equations=int(z['moving'].sum()))


def physical_review(parameters, mapped):
    from full_inertial import _dynamic_parameters_to_urdf_fields, physical_inertia
    links=[]
    for i, (_, name, _) in enumerate(mapped):
        p = parameters[i*10:i*10+10]
        try:
            mass,com,I = _dynamic_parameters_to_urdf_fields(p)
            ok = physical_inertia(mass,com,I)
            links.append(dict(link=name, physical=bool(ok), mass_kg=mass, COM_m=com,
                              inertia_com_eigenvalues=np.linalg.eigvalsh(I)))
        except ValueError as exc:
            links.append(dict(link=name,physical=False,reason=str(exc)))
    return links


def pseudo_inertia(p):
    """Linear mass-distribution moment matrix, using inertia at link origin."""
    I=np.array([[p[4],p[5],p[7]],[p[5],p[6],p[8]],[p[7],p[8],p[9]]])
    J=np.zeros((4,4));J[:3,:3]=.5*np.trace(I)*np.eye(3)-I
    J[:3,3]=p[1:4];J[3,:3]=p[1:4];J[3,3]=p[0]
    return J


def complete_physical(prior, candidate, free, null_basis):
    """Find a physical representative WITHOUT changing observable parameters.

    Convex PSD feasibility in the model-only structural nullspace; empirical
    mass/COM bounds are absent. Failure leaves the raw solution diagnostic.
    """
    from scipy.optimize import minimize
    N=np.asarray(null_basis);k=N.shape[1]
    if not k:return candidate.copy(),dict(status='no_nullspace',applied=False)
    blocks=[];linear=[];centers=[]
    try:
        for start in range(0,len(free),10):
            indices=free[start:start+10]
            L=np.linalg.cholesky(pseudo_inertia(prior[indices]));inv=np.linalg.inv(L)
            center=inv@pseudo_inertia(candidate[indices])@inv.T
            d=np.array([inv@pseudo_inertia(N[start:start+10,j])@inv.T for j in range(k)])
            blocks.append((center,d));linear.append(d.reshape(k,16).T)
            centers.append((center-np.eye(4)).ravel())
    except np.linalg.LinAlgError:
        return candidate.copy(),dict(status='source_not_strictly_physical',applied=False)
    A=np.concatenate(linear);b=np.concatenate(centers)
    H=A.T@A/max(1,len(b));g=A.T@b/max(1,len(b))
    def fun(x):return .5*float(x@H@x)+float(g@x)
    def jac(x):return H@x+g
    def constraints(x):
        return np.concatenate([np.linalg.eigvalsh(c+np.einsum('k,kij->ij',x,d))-1e-9 for c,d in blocks])
    def cjac(x):
        out=[]
        for c,d in blocks:
            _,u=np.linalg.eigh(c+np.einsum('k,kij->ij',x,d))
            out.extend(np.einsum('ik,lij,jk->kl',u,d,u))
        return np.asarray(out)
    result=minimize(fun,np.zeros(k),jac=jac,method='SLSQP',
                    constraints=[dict(type='ineq',fun=constraints,jac=cjac)],
                    options=dict(maxiter=300,ftol=1e-10))
    minimum=float(np.min(constraints(result.x)))
    if not result.success or minimum < -1e-8:
        return candidate.copy(),dict(status='no_verified_physical_completion',applied=False,
                                      optimizer_message=str(result.message),minimum_cone_margin=minimum)
    completed=candidate.copy();completed[free]+=N@result.x
    return completed,dict(status='physical_representative',applied=True,
                          minimum_cone_margin=minimum,iterations=int(result.nit))


def _gravity_review(engine, parameters, directory, *, held_only=False):
    from static_observations import static_equations
    try:rows=static_equations(engine,directory)
    except (OSError,ValueError)as exc:return dict(passed=False,status='unavailable',reason=str(exc))
    if held_only:rows=[s for s in rows if s['held']]
    if len(rows)<2:return dict(passed=False,status='insufficient_heldout_poses',poses=len(rows))
    old=np.array([r['baseline']-r['tau']for r in rows])
    new=np.array([r['baseline']+r['Y'][:,engine.cols]@(parameters-engine.prior)-r['tau']for r in rows])
    before=float(np.sqrt(np.mean(old*old)));after=float(np.sqrt(np.mean(new*new)))
    tolerance=max(.001,.02*before);improved=after<before-tolerance
    return dict(passed=improved,non_regression=after<=before+tolerance,
                status='improved'if improved else 'equivalent'if after<=before+tolerance else 'degraded',
                baseline_rms_nm=before,candidate_rms_nm=after,poses=len(rows),
                joint_baseline_rms_nm=np.sqrt(np.mean(old*old,axis=0)),joint_candidate_rms_nm=np.sqrt(np.mean(new*new,axis=0)),
                paired_poses=sum(r['paired']for r in rows),single_direction_poses=sum(not r['paired']for r in rows),
                pose_quality=[dict(group=r['group'],paired=r['paired'],q_delta_rad=r['q_delta_rad'],uncertainty_nm=r['uncertainty'])for r in rows],
                source='actual_q_per_direction_pose_group_mean_with_hysteresis_uncertainty_not_independent_gravity_truth',
                motion_bias_added_to_gravity=False,validation_scope='heldout_pose_groups'if held_only else 'independent_session')


def _disjoint_split(z):
    groups=np.unique(z['segment'])
    if len(groups)<5: raise ValueError('独立连续块不足 5 个，无法留出完整时间块')
    held=set(groups[4::5]);mask=np.isin(z['segment'],list(held))
    return ({k:v[~mask] for k,v in z.items()}, {k:v[mask] for k,v in z.items()})


def _compatible(a,b):
    from artifact_exports import _same_dynamics_model
    # Identical complete models, mappings, gravity, and joint order for honest
    # session transfer. No silent rewriting of the validation model snapshot.
    if _sha(a.source)!=_sha(b.source):
        if not _same_dynamics_model(a.source,b.source):
            raise ValueError('验证会话源模型不同，请使用同装机和同输入模型的独立会话')
    for key in ('joint_names','gravity','mapping','load_description','actuator_capabilities'):
        if a.meta.get(key)!=b.meta.get(key): raise ValueError('跨会话 '+key+' 不一致')


def identify(directory, *, mode='global', locked_links=None, validation_directory=None,
             estimator='WLS', cutoff_hz=5., destination=None, parameter_target='full',fixed_joint_positions=None,load_description=None):
    engine=Model(directory,fixed_joint_positions,load_description);folder=engine.folder;before=_receipt(folder)
    observations=load_observations(folder);audit=audit_record(folder)
    locked,free=engine.scope(mode,locked_links or [])
    if parameter_target not in ('full','inertia'):raise ValueError('完整动力学参数目标无效')
    if parameter_target=='inertia':free=free[free%10>=4]
    basis,sv=engine.basis(free);rank=basis.shape[1]
    z=preprocess(observations,engine,free,basis,float(cutoff_hz))
    validation_engine=None;validation_receipt=None
    if validation_directory:
        validation_engine=Model(validation_directory,fixed_joint_positions,load_description);_compatible(engine,validation_engine)
        validation_receipt=_receipt(validation_engine.folder)
        if before['frames.csv']==validation_receipt['frames.csv']:
            raise ValueError('训练与验证是同一份 frames.csv，复制目录不能充当独立会话')
        hold=preprocess(load_observations(validation_engine.folder),validation_engine,free,basis,float(cutoff_hz))
        train=z;scope='independent_session'
    else: train,hold=_disjoint_split(z);scope='same_session_disjoint_blocks'
    from static_observations import static_equations
    from joint_identification import fit_joint,fit_prior
    static_rows=static_equations(engine,folder)
    static_train=[r for r in static_rows if not r['held']]
    full,fit,completion=fit_joint(train,static_train,engine,free,basis,estimator,parameter_target)
    # Unconstrained IDIM is a benchmark diagnostic, never silently substituted.
    raw_fit=solve(train,rank,estimator);raw_parameters=engine.prior.copy()
    raw_parameters[free]+=basis@raw_fit['beta'][:rank]
    base=fit_prior(train,static_train,engine,free,basis,estimator)
    for i,(_,name,_)in enumerate(engine.mapped):
        if name in locked and not np.array_equal(full[10*i:10*i+10],engine.prior[10*i:10*i+10]):raise ValueError('可信 Link 被修改')
    physical=physical_review(full,engine.mapped)
    motion_before,motion_after=score(hold,base),score(hold,fit)
    motion_pass=motion_after['rms_nm']<motion_before['rms_nm']*.98
    gravity=_gravity_review(validation_engine or engine,full,validation_engine.folder if validation_engine else folder,held_only=not bool(validation_engine))
    physical_pass=all(x['physical']for x in physical)
    identifiable=fit['equation_rank']==fit['equation_columns']
    validation_audit=audit_record(validation_engine.folder)if validation_engine else None
    checks=dict(training_measurement=audit['measurement_ready'],training_phases=audit['acquisition_complete'],
                load_description=bool(str(engine.meta.get('load_description')or'').strip()not in ('','unspecified')),unmeasured_joint_positions=engine.unmeasured_known,
                independent_session=bool(validation_engine),
                validation_measurement=bool(validation_audit and validation_audit['measurement_ready']),
                validation_phases=bool(validation_audit and validation_audit['acquisition_complete']),
                validation_load_description=bool(validation_engine and str(validation_engine.meta.get('load_description')or'').strip()not in ('','unspecified')))
    quality_pass=all(checks.values());changed=bool(np.max(abs(full-engine.prior))>1e-10)
    exportable=bool(changed and not completion.get('prior_preferred') and scope=='independent_session' and motion_pass and gravity['passed'] and physical_pass and quality_pass and identifiable)
    reason=[]
    labels=dict(training_measurement='训练记录量程或 q/dq 一致性未通过',training_phases='训练记录六阶段未齐全',
                load_description='未记录装机与载荷说明',unmeasured_joint_positions='未声明未采样关节的实际固定位置',
                independent_session='缺少另一个独立验证会话',validation_measurement='独立验证记录的量程/反馈一致性尚未通过',
                validation_phases='独立验证记录六阶段尚未齐全',validation_load_description='独立验证记录未声明装机与载荷')
    reason.extend(labels[k]for k,passed in checks.items()if not passed and (validation_engine or not k.startswith('validation_')))
    if not changed:reason.append('没有新的参数更新')
    if completion.get('prior_preferred'):reason.append('训练交叉验证推荐保留原模型；当前拟合结果仅作为待审核对照')
    if not motion_pass:reason.append('运动预测未达到 2% 改善')
    if not gravity['passed']:reason.append('纯重力项未证明改善: '+str(gravity['status']))
    if not physical_pass:reason.append('完整候选或锁定源惯量非物理，禁止 URDF 导出')
    if not identifiable:reason.append('运动回归缺秩，需审核激励与干扰项可区分性')
    run_id=uuid.uuid4().hex
    import scipy
    report=dict(schema='serial_arm_idim_candidate_v1',solver=SOLVER,run_id=run_id,
        environment=dict(python=platform.python_version(),numpy=np.__version__,scipy=scipy.__version__,pinocchio=engine.pin.__version__,cvxpy=__import__('cvxpy').__version__),
        task_id=engine.meta.get('task_id'),directory=str(folder),experiment_conditions=dict(recorded_load_description=engine.recorded_load_description,operator_load_description=engine.operator_load_description,source='recorded_metadata_plus_explicit_operator_declaration_not_measurement'),identification_mode=mode,locked_links=locked,
        parameter_target=parameter_target,recommendation=completion.get('recommendation'),recommended_dynamic_parameters=engine.prior if completion.get('prior_preferred')else full,
        fitted_links=[name for _,name,_ in engine.mapped if name not in locked],
        local_scope_method='fixed_before_solve' if mode=='local' else None,
        estimator=estimator,cutoff_hz=float(cutoff_hz),derivative_source='offline_filtered_actual_position',
        unmeasured_joints=engine.unmeasured,fixed_joint_positions=engine.fixed_joint_positions,
        fixed_positions_provenance='operator_declared_constant_positions_not_measured',
        preprocessing=dict(position_filter='zero_phase_butterworth4',parallel_filter_hz=5.,
                           decimation_hz=20.,block_s=10.,edge_guard_s=1.5,min_joint_speed=.03),
        parameter_basis=basis,free_parameter_indices=free,structural_rank=rank,
        structural_singular_values=sv,numerical_fit={k:v for k,v in fit.items() if k!='beta'},
        original_dynamic_parameters=engine.prior,raw_dynamic_parameters=raw_parameters,
        selected_dynamic_parameters=full,physical_completion=completion,
        training_motion=dict(baseline=score(train,base),candidate=score(train,fit)),
        validation_motion=dict(baseline=motion_before,candidate=motion_after,passed=bool(motion_pass)),
        validation_scope=scope,gravity_validation=gravity,physical_review=physical,
        physical_pass=physical_pass,exportable=exportable,review_exportable=physical_pass and changed,report_exportable=True,candidate_changed=changed,automatic_application_allowed=False,
        validated_on_hardware=False,status='prior_retained'if not changed else 'offline_candidate_ready_for_supervised_test'if exportable else 'review_only_candidate'if physical_pass else 'diagnostic_only',
        blocking_reasons=reason,input_checks=checks,audit=audit,validation_audit=validation_audit,quality_pass=quality_pass,
        static_training_groups=[r['group']for r in static_train],static_aggregation='actual_q_per_direction_equal_pose_group_with_hysteresis_uncertainty',
        unconstrained_benchmark=dict(validation_motion=score(hold,raw_fit),physical_review=physical_review(raw_parameters,engine.mapped)),
        training_nuisance=fit['beta'][rank:].reshape(-1,3),
        nuisance_convention='measured_minus_model; never add to gravity feedforward',comparison_nuisance_policy='baseline_and_candidate_use_same_joint_static_motion_training_system_then_freeze',
        input_sha256=before,validation_directory=str(validation_engine.folder) if validation_engine else None,
        validation_input_sha256=validation_receipt,source_urdf=str(engine.source),
        source_code_sha256={name:_sha(Path(__file__).with_name(name)) for name in
                            ('dynamic_identification.py','joint_identification.py','static_observations.py','full_inertial.py','kinematic_consistency_audit.py')},
        limitations=['电机反馈尚非独立物理力矩标定','未测末端关节的声明固定位置不是传感器观测；未声明时仅按中立位诊断',
                     '运动预测、重力观测与实机补偿分别验收；不继承输入模型的硬件验证'])
    if before!=_receipt(folder) or (validation_engine and validation_receipt!=_receipt(validation_engine.folder)):
        raise ValueError('计算期间源数据变化，拒绝发布候选')
    dest=Path(destination).expanduser().resolve() if destination else folder/'idim_candidates'/run_id
    dest.mkdir(parents=True,exist_ok=False)
    report['report_path']=str(dest/'idim-report.json')
    _json(dest/'idim-report.json',report)
    _json(dest/'receipt.json',dict(run_id=run_id,report_sha256=_sha(dest/'idim-report.json')))
    # Latest pointer only: the immutable per-run report is the source of export.
    if destination is None:
        pointer=folder/'idim-candidate.json';tmp=folder/('idim-candidate-'+run_id+'.tmp')
        _json(tmp,report);tmp.replace(pointer)
    return _plain(report)


def export_candidate(report_path, destination=None, *, review_only=False, report_only=False):
    """Export immutable selected parameters after independent offline checks.

    Never refits, replaces inertia, adds bias, writes Core YAML or drives hardware.
    """
    from full_inertial import _dynamic_parameters_to_urdf_fields
    from model_calibration_source import resolve_relative_mesh
    path=Path(report_path).expanduser().resolve();raw=path.read_bytes();report=json.loads(raw)
    receipt=json.loads((path.parent/'receipt.json').read_text())
    if receipt.get('report_sha256')!=hashlib.sha256(raw).hexdigest() or receipt.get('run_id')!=report.get('run_id'):
        raise ValueError('候选报告被修改，请重新计算')
    if (report.get('solver')!=SOLVER and not (report_only and report.get('solver')=='filtered_idim_v1')) or (not report_only and not report.get('exportable') and not (review_only and report.get('review_exportable'))):
        raise ValueError('候选未通过独立运动、重力及物理检查；仅可查看诊断报告，拒绝导出上机模型')
    folder=Path(report['directory'])
    engine=Model(folder,report.get('fixed_joint_positions'),(report.get('experiment_conditions')or{}).get('operator_load_description'))
    if _receipt(folder)!=report['input_sha256']: raise ValueError('候选源数据变化，请重算')
    validation_folder=report.get('validation_directory')
    if not validation_folder and not (review_only or report_only):raise ValueError('缺独立验证会话')
    if validation_folder and _receipt(validation_folder)!=report['validation_input_sha256']:raise ValueError('验证数据变化，请重算')
    parameters=np.asarray(report['selected_dynamic_parameters'],float)
    if parameters.shape!=engine.prior.shape or not np.isfinite(parameters).all():raise ValueError('候选参数无效')
    if report_only:
        dest=Path(destination).expanduser().resolve()if destination else path.parent/'diagnostics'
        dest.mkdir(parents=True,exist_ok=False)
        (dest/'idim-report.json').write_bytes(raw)
        shutil.copy2(path.parent/'receipt.json',dest/'receipt.json')
        _json(dest/'parameters.json',dict(original=report['original_dynamic_parameters'],selected=report['selected_dynamic_parameters'],raw_benchmark=report['raw_dynamic_parameters']))
        return dict(directory=str(dest),report=str(dest/'idim-report.json'),report_only=True,validated_on_hardware=False)
    reviews=physical_review(parameters,engine.mapped)
    if not all(x['physical']for x in reviews):raise ValueError('候选非物理')
    locked=set(report['locked_links'])
    for i,(_,name,link)in enumerate(engine.mapped):
        p=parameters[10*i:10*i+10]
        if name in locked:
            if not np.array_equal(p,engine.prior[10*i:10*i+10]):raise ValueError('锁定 Link 参数被篡改')
            continue
        m,c,I=_dynamic_parameters_to_urdf_fields(p);inertial=link.find('inertial')
        origin=inertial.find('origin')
        if origin is None:origin=ET.SubElement(inertial,'origin')
        origin.set('xyz',' '.join(f'{x:.17g}'for x in c));origin.set('rpy','0 0 0')
        inertial.find('mass').set('value',f'{m:.17g}')
        t=inertial.find('inertia')
        for name,x in zip(('ixx','ixy','ixz','iyy','iyz','izz'),(I[0,0],I[0,1],I[0,2],I[1,1],I[1,2],I[2,2])):t.set(name,f'{x:.17g}')
    dest=Path(destination).expanduser().resolve() if destination else path.parent/'export'
    if dest.exists():raise ValueError('导出目录已存在，请使用新目录')
    # Work in a sibling stage; publish only after model roundtrip and hashes.
    dest.parent.mkdir(parents=True,exist_ok=True);stage=dest.with_name(dest.name+'.tmp-'+uuid.uuid4().hex)
    stage.mkdir()
    try:
        for mesh in engine.root.findall('.//mesh'):
            ref=mesh.get('filename','')
            if not ref or ref.startswith('package://'):continue
            if Path(ref).is_absolute():
                src=Path(ref)
                if not src.is_file():raise ValueError('缺少模型资源: '+ref)
            else:src=resolve_relative_mesh(ref,engine.source,engine.meta)
            if src is None:raise ValueError('无法解析模型资源: '+ref)
            rel=Path('resources')/(_sha(src)[:16]+'-'+src.name)
            (stage/rel).parent.mkdir(exist_ok=True);shutil.copy2(src,stage/rel);mesh.set('filename',rel.as_posix())
        model_path=stage/'robot.urdf';ET.ElementTree(engine.root).write(model_path,encoding='utf-8',xml_declaration=True)
        parsed=engine.pin.buildModelFromUrdf(str(model_path));expected=engine.theta.copy();expected[engine.cols]=parameters
        readback=np.concatenate([parsed.inertias[j].toDynamicParameters()for j in range(1,parsed.njoints)])
        if not np.allclose(readback,expected,rtol=1e-10,atol=1e-12):raise ValueError('URDF 惯性参数回读不一致')
        # Inertial changes must preserve actual FK, joint topology and dimensions.
        rng=np.random.default_rng(17);max_fk=0.
        for _ in range(16):
            q=engine.neutral.copy();from full_inertial import _assign_configuration
            _assign_configuration(engine.model,q,engine.qi,rng.uniform(-1.,1.,len(engine.qi)))
            da=engine.model.createData();db=parsed.createData()
            engine.pin.forwardKinematics(engine.model,da,q);engine.pin.forwardKinematics(parsed,db,q)
            for j in range(1,parsed.njoints):max_fk=max(max_fk,float(np.max(abs(da.oMi[j].homogeneous-db.oMi[j].homogeneous))))
        if max_fk>1e-12:raise ValueError('导出改变了运动学')
        if path.read_bytes()!=raw or _receipt(folder)!=report['input_sha256'] or (validation_folder and _receipt(validation_folder)!=report['validation_input_sha256']):
            raise ValueError('导出期间报告或数据变化')
        (stage/'idim-report.json').write_bytes(raw)
        shutil.copy2(path.parent/'receipt.json',stage/'receipt.json')
        _json(stage/'verification.json',dict(report_sha256=hashlib.sha256(raw).hexdigest(),
              urdf_sha256=_sha(model_path),inertial_roundtrip=True,fk_max_abs=max_fk,
              validated_on_hardware=False,review_only=bool(review_only or not report['exportable']),blocking_reasons=report['blocking_reasons'],observer_bias_exported=False,gravity_scale_required=1.))
        (stage/'README.txt').write_text(('待审核候选；未完成独立验证，不可据此声称补偿改善\n'if review_only else '离线检查已通过；尚未真机验收\n')+'不包含控制增益、摩擦或 observer 偏置；完整模型禁止拆分拼接\n'+'\n'.join(report['blocking_reasons']),encoding='utf-8')
        stage.rename(dest)
    except Exception:
        shutil.rmtree(stage,ignore_errors=True);raise
    return dict(directory=str(dest),candidate_urdf=str(dest/'robot.urdf'),validated_on_hardware=False)
