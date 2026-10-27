"""Read-only angular position/velocity consistency audit, requiring only NumPy.

This diagnostic estimates relative scale/bias between recorded q and dq. It is
NOT an external sensor calibration and NEVER applies corrections. Unwrapped
angular positions (rad), angular velocities (rad/s), and monotonic seconds are
required. Touching windows share endpoint observations; no confidence interval
or claim of independent repetitions is produced.
"""
from __future__ import annotations
import numpy as np


def audit_angular_kinematics(time_s, q_rad, dq_rad_s, *, phase=None, valid=None,
        joint_names=None, window_s=1.0, max_gap_s=0.025, min_samples=12,
        min_windows=6, min_velocity_excitation_rms_rad_s=0.02,
        scale_review_tolerance=0.05):
    """Fit Δq = scale * integral(dq dt) + bias * Δt per excited joint.

    All thresholds are explicit engineering review choices, not noise estimates.
    Phase changes, invalid/nonfinite samples and large time gaps split windows.
    Near-constant velocity cannot separate scale from bias and is not assigned
    an interpretable scale. This function reads arrays without modifying them.
    """
    t=np.asarray(time_s,dtype=float);q=np.asarray(q_rad,dtype=float);v=np.asarray(dq_rad_s,dtype=float)
    if t.ndim!=1 or q.ndim!=2 or q.shape!=v.shape or len(t)!=len(q) or q.shape[1]<1:
        raise ValueError('Expected t[N], q[N,J], dq[N,J] with matching dimensions')
    if len(t)<2 or not np.isfinite(t).all() or np.any(np.diff(t)<=0):
        raise ValueError('Time must contain finite, strictly increasing seconds')
    numeric=[window_s,max_gap_s,min_velocity_excitation_rms_rad_s,scale_review_tolerance]
    if not np.isfinite(numeric).all() or any(x<=0 for x in numeric) or min_samples<2 or min_windows<3:
        raise ValueError('Invalid positive diagnostic thresholds/counts')
    n,j=q.shape
    phases=np.zeros(n,dtype=int)if phase is None else np.asarray(phase)
    flags=np.ones(n,dtype=bool)if valid is None else np.asarray(valid,dtype=bool)
    if phases.shape!=(n,)or flags.shape!=(n,):raise ValueError('phase/valid must have shape [N]')
    names=[str(x)for x in (joint_names if joint_names is not None else range(1,j+1))]
    if len(names)!=j or len(set(names))!=j:raise ValueError('joint_names must be unique and match J')
    good=flags & np.isfinite(q).all(1)&np.isfinite(v).all(1)
    segments=[];start=None
    for i in range(n+1):
        split=i==n or not good[i]or(i>0 and(t[i]-t[i-1]>max_gap_s or phases[i]!=phases[i-1]))
        if split and start is not None:
            segments.append((start,i));start=None
        if i<n and good[i]and start is None:start=i
    records=[];increments=[];integrals=[];durations=[]
    for seg,(lo,end)in enumerate(segments):
        cursor=lo
        while cursor+min_samples<=end:
            last=int(np.searchsorted(t,t[cursor]+window_s,side='left'))
            if last>=end:break
            if last-cursor+1<min_samples:
                cursor=last;continue
            sl=slice(cursor,last+1);duration=float(t[last]-t[cursor])
            integrator=np.trapezoid if hasattr(np,'trapezoid')else np.trapz
            integral=integrator(v[sl],t[sl],axis=0)
            increment=q[last]-q[cursor]
            if not np.isfinite(integral).all()or not np.isfinite(increment).all():
                raise ValueError('Nonfinite derived position/velocity integral')
            increments.append(increment);integrals.append(integral);durations.append(duration)
            records.append(dict(start_index=cursor,end_index_inclusive=last,
                start_s=float(t[cursor]),end_s=float(t[last]),duration_s=duration,
                phase=str(phases[cursor]),segment=seg))
            cursor=last
    D=np.asarray(increments,dtype=float).reshape(-1,j);I=np.asarray(integrals,dtype=float).reshape(-1,j);L=np.asarray(durations)
    out=[]
    for axis,name in enumerate(names):
        if not len(L):
            out.append(dict(joint=name,status='insufficient_windows',scale_is_interpretable=False,
                scale=None,bias_rad_s=None,window_count=0,velocity_excitation_rms_rad_s=None,
                raw_window_error_rms_rad=None,affine_window_error_rms_rad=None));continue
        u=I[:,axis]/L;y=D[:,axis]/L
        excitation=float(np.sqrt(np.mean((u-u.mean())**2)))
        raw=D[:,axis]-I[:,axis]
        record=dict(joint=name,window_count=len(L),velocity_excitation_rms_rad_s=excitation,
            velocity_mean_rad_s=float(u.mean()),velocity_p10_p90_rad_s=np.percentile(u,[10,90]).tolist(),
            position_increment_rms_rad=float(np.sqrt(np.mean(D[:,axis]**2))),
            raw_window_error_rms_rad=float(np.sqrt(np.mean(raw**2))),
            raw_window_error_mean_rad=float(raw.mean()),raw_window_error_max_abs_rad=float(abs(raw).max()),
            scale_is_interpretable=False,scale=None,bias_rad_s=None,affine_window_error_rms_rad=None)
        if len(L)<min_windows:record['status']='insufficient_windows'
        elif excitation<min_velocity_excitation_rms_rad_s:record['status']='insufficient_velocity_excitation'
        else:
            X=np.column_stack([I[:,axis],L]);coef,_,rank,_=np.linalg.lstsq(X,D[:,axis],rcond=1e-12)
            if rank<2:record['status']='scale_bias_confounded'
            else:
                residual=D[:,axis]-X@coef
                record.update(scale_is_interpretable=True,scale=float(coef[0]),bias_rad_s=float(coef[1]),
                    affine_window_error_rms_rad=float(np.sqrt(np.mean(residual**2))),
                    status='scale_requires_review'if abs(coef[0]-1)>scale_review_tolerance else 'unit_scale_consistent')
        out.append(record)
    interpretable=[x for x in out if x['scale_is_interpretable']]
    overall='requires_review'if any(x['status']=='scale_requires_review'for x in out)else('limited_excitation'if len(interpretable)<j else 'unit_scale_consistent')
    return dict(schema='angular_kinematic_consistency_audit_v1',status=overall,joints=out,windows=records,
        input_rows=n,used_windows=len(records),continuous_segments=len(segments),invalid_rows=int((~good).sum()),
        thresholds=dict(window_s=window_s,max_gap_s=max_gap_s,min_samples=min_samples,min_windows=min_windows,
            min_velocity_excitation_rms_rad_s=min_velocity_excitation_rms_rad_s,scale_review_tolerance=scale_review_tolerance,
            interpretation='Engineering review thresholds; not measurement variance or statistical significance'),
        units=dict(time='s',position='rad',velocity='rad/s',bias='rad/s'),
        behavior=dict(read_only=True,corrections_applied=False,independent_calibration=False,hardware_validated=False),
        limitations=['Relative q/dq consistency cannot identify which sensor scale is correct',
            'Host clock/feedback timing and filtering can contribute to mismatch',
            'Near-constant velocity confounds scale and bias',
            'Input positions must be unwrapped; wrap jumps are not automatically corrected',
            'Touching windows share endpoints; no independent-repetition statistics reported'])
