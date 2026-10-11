"""Bounded mass/first-moment gravity fit using a trusted URDF prior

This solver does NOT infer individual inertia tensors from a rank-deficient
full-inertial regression. The six URDF COM inertia entries remain immutable.
An explicit physical feasibility constraint is imposed during optimization.
"""
from __future__ import annotations

import numpy as np


def _indices(n_links):
    return np.asarray([10 * link + j for link in range(n_links) for j in range(4)], dtype=int)


def _dynamic_prior_with_unchanged_com_tensor(prior, proposal):
    """Pinocchio inertia about joint frame while retaining URDF COM tensor."""
    p = np.asarray(prior, dtype=float)
    out = np.asarray(proposal, dtype=float).copy()
    for i in range(0, len(p), 10):
        original, updated = p[i:i+10], out[i:i+10]
        mass0, mass1 = original[0], updated[0]
        if not (np.isfinite(mass1) and mass1 > 0 and mass0 > 0):
            raise ValueError('质量必须为正且有限')
        c0, c1 = original[1:4]/mass0, updated[1:4]/mass1
        def shift(m, c):
            return m * (np.dot(c,c)*np.eye(3) - np.outer(c,c))
        Io = np.array([[original[4],original[5],original[7]],
                       [original[5],original[6],original[8]],
                       [original[7],original[8],original[9]]])
        I = Io - shift(mass0,c0) + shift(mass1,c1)
        out[i+4:i+10] = [I[0,0],I[0,1],I[1,1],I[0,2],I[1,2],I[2,2]]
    return out


def fit_global_mass_com(old_fit, training, heldout, max_shift=0.05,
                        mass_fraction=0.25, static_weight=0.75):
    """Directly identify bounded mass/first moments without fitting inertia

    Parameters
    ----------
    old_fit: historical fit dictionary supplying an identical sampled dynamic
        regressor, prior and held-out rows; its numerical *candidate* is unused
    training, heldout: independent stationary pose tuples (Y, fixed, tau)
    Returns physically feasible parameters and diagnostic scores
    """
    from scipy.optimize import minimize
    prior = np.asarray(old_fit['prior'], dtype=float)
    if prior.ndim != 1 or len(prior)%10:
        raise ValueError('原始惯性参数维度无效')
    if not (np.isfinite(prior).all() and np.all(prior[::10]>1e-7)):
        raise ValueError('源模型包含无效质量或惯性参数')
    if not (np.isfinite(max_shift) and 0 < max_shift <= .2
            and 0 < mass_fraction <= .5):
        raise ValueError('质量与质心约束范围无效')
    count = len(prior)//10
    inds = _indices(count)
    fixed_inds = np.asarray([10*i+j for i in range(count) for j in range(4,10)], int)
    A,b = old_fit['train']; V, vb = old_fit['validation']
    A,b,V,vb = (np.asarray(v,float) for v in (A,b,V,vb))
    nfric=A.shape[1]-len(prior)
    if nfric < 0 or A.shape[1]!=V.shape[1] or b.ndim!=1:
        raise ValueError('动态回归数据维度不匹配')
    nj=nfric//5
    if nj <= 0 or nfric!=5*nj or len(b)%nj or len(vb)%nj:
        raise ValueError('摩擦辅助矩阵与关节数量不一致')
    units=np.asarray([v for k in range(count)
                      for v in [prior[10*k]*mass_fraction]+[prior[10*k]*max_shift]*3])
    origin_com = np.asarray([prior[10*k+1:10*k+4]/prior[10*k] for k in range(count)])
    Ydyn = A[:,:len(prior)]; Fdyn=A[:,len(prior):]
    Yval=V[:,:len(prior)]; Fval=V[:,len(prior):]
    # Only first four parameters of each link can change. All dynamic inertia
    # terms are recomputed with the parallel-axis theorem, not independently fit
    Gdyn=Ydyn[:,inds]*units
    def pose_matrix(rows):
        if not rows:
            return np.zeros((0,len(prior))), np.zeros(0), np.zeros((0,nfric))
        yy, tt, ff = [], [], []
        for Y, fixed, torque in rows:
            if np.shape(Y)!=(nj,len(prior)) or np.shape(fixed)!=(nj,):
                raise ValueError('静态验证关节矩阵维度不匹配')
            yy.append(np.asarray(Y,float))
            tt.append(np.asarray(torque,float)-np.asarray(fixed,float))
            f=np.zeros((nj,nfric))
            for j in range(nj): f[j,5*j]=1.0
            ff.append(f)
        return np.concatenate(yy),np.concatenate(tt),np.concatenate(ff)
    Ys,bs,Fs=pose_matrix(training)
    Yhold, bhold, Fhold=pose_matrix(heldout)
    if not len(Ys) or not len(Yhold):
        raise ValueError('缺少独立的静态训练与留出姿态')
    Gs=Ys[:,inds]*units
    z0=np.zeros(len(inds))
    nuisance=np.zeros(nfric)
    # Explicit mass and COM *joint* cone constraint, not independent Cartesian
    # clips which can admit a COM outside the radius
    def feasible(z):
        for k in range(count):
            m=prior[10*k]+units[4*k]*z[4*k]
            h=prior[10*k+1:10*k+4]+units[4*k+1:4*k+4]*z[4*k+1:4*k+4]
            if m < prior[10*k]*(1-mass_fraction)-1e-8 or m > prior[10*k]*(1+mass_fraction)+1e-8:
                return False
            if np.linalg.norm(h-m*origin_com[k])>max_shift*m+1e-8:
                return False
        return True
    def cone(z, k):
        m=prior[10*k]+units[4*k]*z[4*k]
        h=prior[10*k+1:10*k+4]+units[4*k+1:4*k+4]*z[4*k+1:4*k+4]
        d=h-m*origin_com[k]
        return (max_shift*m)**2-np.dot(d,d)
    def cone_grad(z,k):
        m=prior[10*k]+units[4*k]*z[4*k]
        h=prior[10*k+1:10*k+4]+units[4*k+1:4*k+4]*z[4*k+1:4*k+4]
        d=h-m*origin_com[k]
        grad=np.zeros(len(inds))
        grad[4*k]=2*units[4*k]*(max_shift**2*m+np.dot(d,origin_com[k]))
        grad[4*k+1:4*k+4]=-2*d*units[4*k+1:4*k+4]
        return grad
    constraints=[{'type':'ineq','fun':lambda z,k=k:cone(z,k),
                  'jac':lambda z,k=k:cone_grad(z,k)} for k in range(count)]
    bounds=[(-1.,1.) if k%4==0 else (-1.5,1.5) for k in range(len(inds))]
    # Per-joint scale is measured only from *training* residuals
    base=(b-A[:,:len(prior)]@prior).reshape(-1,nj)
    sigma=np.maximum(np.median(np.abs(base-np.median(base,axis=0)),axis=0)*1.4826,.035)
    scale_dyn=np.tile(1./sigma,len(b)//nj)
    scale_static=np.tile(1./sigma,len(bs)//nj)
    # Static and moving segments each contribute a well-defined fraction
    w_dyn=scale_dyn*np.sqrt((1-static_weight)/max(len(b),1))
    w_static=scale_static*np.sqrt(static_weight/max(len(bs),1))
    Gw=np.vstack([Gdyn*w_dyn[:,None],Gs*w_static[:,None]])
    Fw=np.vstack([Fdyn*w_dyn[:,None],Fs*w_static[:,None]])
    bprior=np.concatenate([b-Ydyn@prior,bs-Ys@prior])
    # Damping on friction nuisance prevents the bias terms from arbitrarily
    # absorbing every physical mass and moment signal
    nuisance_penalty=0.005*np.eye(nfric)
    candidates=[]
    # Validation targets pure gravity; static holdout is used to choose the
    # regularization and is not an independent final generalization test
    for ridge in (0.002,0.01,0.04,0.12):
        z=np.zeros(len(inds))
        u=np.zeros(nfric)
        ok=True
        for iteration in range(3):
            parameters=prior.copy(); parameters[inds]+=z*units
            converted=_dynamic_prior_with_unchanged_com_tensor(prior,parameters)
            extra=converted-prior
            extra[inds]=0.
            offset=np.concatenate([Ydyn@extra,Ys@extra])
            r=(bprior-offset)*np.concatenate([w_dyn,w_static])
            # Independent per-observation Huber weighting
            if iteration:
                residual=r-Gw@z-Fw@u
                threshold=max(.02,float(np.median(np.abs(residual)))*2.5)
                robust=np.sqrt(np.minimum(1.,threshold/np.maximum(np.abs(residual),1e-12)))
            else:
                robust=np.ones_like(r)
            G=Gw*robust[:,None]; F=Fw*robust[:,None]; target=r*robust
            # Eliminate nuisance analytically; minimize the resulting small
            # constrained convex quadratic in 4 parameters per Link
            K=F.T@F+nuisance_penalty
            Kfg=np.linalg.solve(K,F.T@G)
            Kft=np.linalg.solve(K,F.T@target)
            H=G.T@(G-F@Kfg)+ridge*np.eye(len(inds))
            H=(H+H.T)*.5
            f=G.T@(target-F@Kft)
            sol=minimize(lambda x:float(.5*x@H@x-f@x),z,
                         jac=lambda x:H@x-f,method='SLSQP',bounds=bounds,
                         constraints=constraints,
                         options={'maxiter':250,'ftol':1e-10})
            if not sol.success or not feasible(sol.x):
                ok=False
                break
            z=np.asarray(sol.x)
            u=np.linalg.solve(K,F.T@(target-G@z))
        if not ok: continue
        candidate=prior.copy();candidate[inds]+=z*units
        physical=_dynamic_prior_with_unchanged_com_tensor(prior,candidate)
        def scored(Y,tt,F,model_params,fric):
            return float(np.sqrt(np.mean((tt-Y@model_params-F@fric)**2)))
        dynamic_error=scored(Yval,vb,Fval,physical,u)
        # Evaluate the *deployed URDF* using only gravity. A fitted torque
        # bias is not included in the exported URDF and must not make the
        # candidate appear better than it would be on the robot
        def static_score(p):
            held=(bhold-Yhold@p).reshape(-1,nj)/sigma
            rms=float(np.sqrt(np.mean(held**2)))
            worst_joint=float(np.max(np.sqrt(np.mean(held**2,axis=0))))
            return rms,worst_joint
        static_error,static_worst=static_score(physical)
        static_baseline,_=static_score(prior)
        # Prefer generalization across all observed joints, not only average
        # moving-trajectory accuracy, and penalize large per-joint residuals
        metric=(2.0*static_error + 0.4*static_worst
                + 0.25*dynamic_error/max(float(np.median(sigma)),.035))
        candidates.append((metric,ridge,candidate,physical,z,u,dynamic_error,
                           static_error,static_baseline))
    if not candidates:
        raise ValueError('约束质量与质心求解未收敛，请检查关节反馈、摩擦或动态激励数据')
    # Remove directions which can be explained entirely by velocity-dependent
    # friction before reporting the effective mass/moment rank
    projected = Gdyn - Fdyn @ np.linalg.lstsq(Fdyn, Gdyn, rcond=1e-9)[0]
    # Baseline nuisance belongs to the prior URDF, never borrow the fit of
    # the candidate when reporting the baseline dynamic score
    prior_weighted_f = Fdyn*w_dyn[:,None]
    prior_target = (b-Ydyn@prior)*w_dyn
    prior_nuisance = np.linalg.solve(prior_weighted_f.T@prior_weighted_f + nuisance_penalty,
                                     prior_weighted_f.T@prior_target)
    prior_dynamic_rms = float(np.sqrt(np.mean((vb-Yval@prior-Fval@prior_nuisance)**2)))
    projected_scale = np.maximum(np.linalg.norm(projected, axis=0), 1e-10)
    effective_rank = int(np.linalg.matrix_rank(projected/projected_scale, tol=1e-3))
    metric,ridge,candidate,physical,z,u,drms,srms,sbase=min(candidates,key=lambda x:x[0])
    if not feasible(z) or not np.isfinite(physical).all():
        raise ValueError('候选参数未通过质量与质心物理可行性检查')
    return {
        'prior':prior,'candidate':candidate,'exported':physical,
        'nuisance':u,'regularization':ridge,
        'dynamic_holdout_rms':drms,'dynamic_prior_holdout_rms':prior_dynamic_rms,
        'static_holdout_scaled_rms':srms,
        'static_prior_scaled_rms':sbase,
        'mass_moment_columns':len(inds),
        'active_regressor_rank':effective_rank,
        'joint_noise_scale_nm':sigma.tolist(),
        'solver':'bounded_direct_mass_first_moment',
        'constraints':{'mass_fraction':mass_fraction,'max_com_offset_m':max_shift},
    }
