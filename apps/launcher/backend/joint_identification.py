"""Physical joint static/dynamic convex regression, with training-only CV.

No empirical mass/COM boxes. Normalized pseudo-inertia cones constrain the
fit itself. Prior/no-update is a genuine model candidate, never a fake success.
"""
import numpy as np


def from_pseudo(J):
    S=J[:3,:3];I=np.trace(S)*np.eye(3)-S
    return np.r_[J[3,3],J[:3,3],I[0,0],I[0,1],I[1,1],I[0,2],I[1,2],I[2,2]]


def physical_coordinates(prior, free):
    from dynamic_identification import pseudo_inertia
    E=[]
    for i in range(4):
        for j in range(i,4):
            M=np.zeros((4,4));M[i,j]=M[j,i]=1 if i==j else 1/np.sqrt(2);E.append(M)
    E=np.array(E);blocks=sorted(set(int(i//10)for i in free));T=np.zeros((len(prior),10*len(blocks)))
    for k,block in enumerate(blocks):
        try:L=np.linalg.cholesky(pseudo_inertia(prior[10*block:10*block+10]))
        except np.linalg.LinAlgError as exc:raise ValueError('源惯性不能提供严格物理的归一化参考，请先审核源模型')from exc
        T[10*block:10*block+10,10*k:10*k+10]=np.column_stack([from_pseudo(L@x@L.T)for x in E])
    return T,E,blocks


def _subset(z,mask):return {k:v[mask]for k,v in z.items()}


def _system(z,statics,basis,free,T,method):
    from dynamic_identification import solve
    rank=basis.shape[1];j=z['tau'].shape[1];mask=z['moving'].ravel();n=T.shape[1]
    raw=solve(z,rank,method)
    reference=solve(z,rank,'OLS',True)
    error=z['baseline']+np.einsum('nji,i->nj',z['X'],reference['beta'])-z['tau']
    reference_scale=np.array([np.sqrt(np.mean(error[z['moving'][:,axis],axis]**2))for axis in range(j)])
    sigma=np.maximum(reference_scale,.02)if method!='OLS'else np.full(j,.05)
    ns=np.ravel(np.column_stack([np.full(j,.02),np.full(j,.2),.2/np.maximum(np.std(z['v'],axis=0),.05)]))
    x=z['X'].reshape(-1,rank+3*j);target=(z['tau']-z['baseline']).ravel()
    motion=np.column_stack([x[:,:rank]@(basis.T@T[free]),x[:,rank:]*ns])
    w=np.tile(1/sigma,len(z['tau']))[mask]/np.sqrt(max(1,mask.sum()))
    A=motion[mask]*w[:,None];b=target[mask]*w
    if method=='IRLS':
        residual=(z['baseline']+np.einsum('nji,i->nj',z['X'],raw['beta'])-z['tau']).ravel()[mask]
        robust=np.sqrt(np.minimum(1.,1.345/np.maximum(abs(residual)/np.tile(sigma,len(z['tau']))[mask],1e-12)))
        A*=robust[:,None];b*=robust
    if statics:
        S=[];t=[]
        for s in statics:
            nuisance=np.zeros((j,3*j))
            for axis in range(j):nuisance[axis,3*axis]=1
            weight=1/s['uncertainty']/np.sqrt(len(statics)*j)
            S.append(np.column_stack([s['mapped_Y']@T,nuisance*ns])*weight[:,None])
            t.append((s['tau']-s['baseline'])*weight)
        A=np.r_[A,np.concatenate(S)];b=np.r_[b,np.concatenate(t)]
    # Tiny nuisance regularization; b is a nuisance estimate, not known sensor calibration.
    R=np.zeros((3*j,n+3*j));R[:,n:]=np.diag(np.tile([.1,.001,.001],j))/np.sqrt(j)
    return np.r_[A,R],np.r_[b,np.zeros(3*j)],ns,raw


def constrained_fit(z,statics,engine,free,basis,method,penalty,parameter_target='full'):
    try:import cvxpy as cp
    except ImportError as exc:raise ValueError('联合物理辨识需要 CVXPY/Clarabel，请运行安装器更新工作台专用环境')from exc
    T,E,blocks=physical_coordinates(engine.prior,free);n=T.shape[1];j=z['tau'].shape[1]
    A,b,ns,raw=_system(z,statics,basis,free,T,method)
    # Small square-root system from the exact quadratic sufficient statistics.
    H=A.T@A;g=A.T@b;eig,U=np.linalg.eigh(.5*(H+H.T));eig=np.maximum(eig,0)
    R=np.sqrt(eig)[:,None]*U.T;rhs=np.divide(U.T@g,np.sqrt(eig),out=np.zeros_like(g),where=eig>1e-14)
    x=cp.Variable(n+3*j);constraints=[]
    for k in range(len(blocks)):
        cone=np.eye(4)+sum(x[10*k+c]*E[c]for c in range(10))
        constraints.append(cone >> 1e-6*np.eye(4))
    if parameter_target=='inertia':
        indices=[i for i in range(len(engine.prior))if i%10<4]
        constraints.append(T[indices]@x[:n]==0)
    objective=cp.sum_squares(R@x-rhs)+float(penalty)*cp.sum_squares(x[:n])/max(1,n)
    problem=cp.Problem(cp.Minimize(objective),constraints)
    problem.solve(solver='CLARABEL',tol_gap_abs=1e-8,tol_feas=1e-8,tol_gap_rel=1e-8,max_iter=300)
    if problem.status!='optimal' or x.value is None:raise ValueError('物理联合求解未收敛: '+str(problem.status))
    vector=np.asarray(x.value).ravel();delta=T@vector[:n];candidate=engine.prior+delta
    minimum=min(float(np.linalg.eigvalsh(np.eye(4)+np.einsum('k,kij->ij',vector[10*k:10*k+10],E))[0])for k in range(len(blocks)))
    if minimum<5e-7:raise ValueError('物理锥数值校验未通过，未发布候选')
    beta=np.r_[basis.T@delta[free],vector[n:]*ns]
    return candidate,dict(beta=beta,sigma=raw['sigma'],iterations=int(problem.solver_stats.num_iters or 0),
                         equation_rank=raw['equation_rank'],equation_columns=raw['equation_columns'],singular_values=raw['singular_values']),dict(
                         status='physical_constraints_in_joint_fit',solver='CLARABEL',solver_status=problem.status,
                         minimum_normalized_cone_eigenvalue=minimum,regularization=float(penalty),physical_prior_metric='source_pseudo_inertia_congruence_frobenius',empirical_mass_com_bounds=False,weighting='common_training_source_residual_scale_not_measured_sensor_noise',robust_preweighting='unconstrained_IRLS_Huber_then_constrained_refit'if method=='IRLS'else None)


def fit_prior(z,statics,engine,free,basis,method='WLS'):
    """Freeze source inertias but fit nuisance with the SAME joint objective."""
    T,_,_=physical_coordinates(engine.prior,free)
    for s in statics:s['mapped_Y']=s['Y'][:,engine.cols]
    A,b,ns,raw=_system(z,statics,basis,free,T,method);n=T.shape[1]
    nuisance=np.linalg.lstsq(A[:,n:],b,rcond=1e-10)[0]*ns
    beta=np.r_[np.zeros(basis.shape[1]),nuisance]
    error=z['baseline']+np.einsum('nji,i->nj',z['X'],beta)-z['tau']
    sigma=np.array([np.sqrt(np.mean(error[z['moving'][:,j],j]**2))for j in range(z['tau'].shape[1])])
    return dict(beta=beta,sigma=sigma,iterations=1,equation_rank=raw['equation_rank'],equation_columns=raw['equation_columns'],singular_values=raw['singular_values'])


def _selection_score(z,statics,fit,parameters,prior,motion_scale):
    from dynamic_identification import score
    # Calibration/validation scale is derived exclusively from this fold's training system.
    motion=(score(z,fit)['rms_nm']/max(.02,float(motion_scale)))**2
    gravity=0.
    if statics:
        errors=np.array([(s['baseline']+s['mapped_Y']@(parameters-prior)-s['tau'])/s['uncertainty']for s in statics])
        gravity=float(np.mean(errors**2))
    return float(motion+gravity)


def fit_joint(train,statics,engine,free,basis,method='WLS',parameter_target='full'):
    from dynamic_identification import solve
    import importlib.util
    if importlib.util.find_spec('cvxpy')is None:raise ValueError('缺少 CVXPY/Clarabel，请使用安装器生成的 GUI Python 或重新安装工作台依赖')
    for s in statics:s['mapped_Y']=s['Y'][:,engine.cols]
    groups=np.unique(train['segment']);count=min(3,len(groups),len(statics))
    if count<3:raise ValueError('至少需要三个训练姿态组和三个连续时间块进行联合模型选择')
    grid=[None,.01,.1,1.,10.,100.,1000.];scores=[];failures=[]
    for penalty in grid:
        losses=[];errors=[]
        for fold in range(count):
            held=np.isin(train['segment'],groups[fold::count]);tr=_subset(train,~held);va=_subset(train,held)
            ts=[s for i,s in enumerate(statics)if i%count!=fold];vs=[s for i,s in enumerate(statics)if i%count==fold]
            if min(tr['moving'].sum(0))<10 or min(va['moving'].sum(0))<5:raise ValueError('分块验证时至少一轴激励不足，不能可靠选择完整模型')
            try:
                if penalty is None:params=engine.prior;fit=fit_prior(tr,ts,engine,free,basis,method)
                else:params,fit,_=constrained_fit(tr,ts,engine,free,basis,method,penalty,parameter_target)
                baseline_fit=fit_prior(tr,ts,engine,free,basis,method)
                common_scale=float(np.sqrt(np.mean(baseline_fit['sigma']**2)))
                losses.append(_selection_score(va,vs,fit,params,engine.prior,common_scale))
            except ValueError as exc:errors.append(str(exc))
        scores.append(float(np.mean(losses))if len(losses)==count else float('inf'));failures.append(errors)
    recommendation=int(np.argmin(scores))
    # Always retain the best converged physical FIT as an explicit review
    # alternative, even when training CV recommends the unchanged source.
    choice=1+int(np.argmin(scores[1:]));penalty=grid[choice]
    if not np.isfinite(scores[choice]):raise ValueError('全部物理候选求解失败；没有把原模型伪装成新候选')
    params,fit,diagnostic=constrained_fit(train,statics,engine,free,basis,method,penalty,parameter_target)
    diagnostic.update(cv_penalties=grid,cv_scores=scores,cv_failures=failures,prior_preferred=recommendation==0,recommendation='keep_source_prior'if recommendation==0 else'physical_fit',
                      cv_scope='training_only_disjoint_time_blocks_and_pose_groups',folds=count,
                      selection_metric='normalized_motion_mse_plus_raw_gravity_mse')
    return params,fit,diagnostic
