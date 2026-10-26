"""Static gravity regression with fixed trusted parameters and observable updates.

Only mass and first moments enter gravity; COM inertia tensors stay unchanged.
No empirical mass or COM bounds are imposed on the numerical solution.
"""
from __future__ import annotations

import numpy as np


def observable_gravity_basis(regressor, joints, noise_scale, relative_threshold=1e-3):
    """Configuration-dependent gravity combinations observable from static poses.

    Per-joint centering projects out unknown constant torque offsets before the
    SVD. Unobservable combinations are pinned to the trusted prior, not filled
    by the dynamic trajectory or arbitrary individual-link parameter changes.
    """
    G = np.asarray(regressor, dtype=float)
    sigma = np.asarray(noise_scale, dtype=float)
    if G.ndim != 2 or joints < 1 or G.shape[0] % joints or G.shape[0] < 2*joints:
        raise ValueError('静态重力回归矩阵需要多个完整姿态')
    if sigma.shape != (joints,) or np.any(sigma <= 0) or not np.isfinite(G).all():
        raise ValueError('静态重力回归或噪声尺度无效')
    # Unknown motor offsets are indistinguishable from columns that do not
    # vary with configuration. The centering must be done per joint.
    poses = G.reshape(-1, joints, G.shape[1])
    centered = poses - np.mean(poses, axis=0, keepdims=True)
    weighted = (centered / sigma[None, :, None]).reshape(-1, G.shape[1])
    _, singular, right = np.linalg.svd(weighted, full_matrices=False)
    if not len(singular) or singular[0] <= 1e-12:
        raise ValueError('示教轨迹静态姿态不足以辨识重力参数')
    rank = int(np.sum(singular > max(1e-10, relative_threshold*singular[0])))
    if rank == 0:
        raise ValueError('静态重力回归矩阵没有可辨识参数')
    return right[:rank, :].T, singular, rank


def solve_static_updates(designs, residuals, *, relative_threshold=1e-3):
    """Fit tau-model updates in observable coordinates using training poses only.

    Each pose and joint has equal weight; joint-wise centering removes offsets.
    Scaling and SVD are recomputed within each leave-one-pose-out fold.
    Returned bias always means model-minus-measured, as in the runtime observer.
    """
    designs = np.asarray(designs, dtype=float)
    residuals = np.asarray(residuals, dtype=float)
    if (designs.ndim != 3 or residuals.shape != designs.shape[:2]
            or designs.shape[0] < 3 or designs.shape[2] == 0
            or not np.isfinite(designs).all() or not np.isfinite(residuals).all()):
        raise ValueError('静态求解需要至少三个有效姿态及有限的自由参数回归矩阵')
    def solve(indices, penalty):
        a, b = designs[indices], residuals[indices]
        centered = (a-a.mean(axis=0)).reshape(-1, a.shape[-1])
        target = (b-b.mean(axis=0)).reshape(-1)
        energy = np.linalg.norm(a.reshape(-1, a.shape[-1]), axis=0)
        scale = np.linalg.norm(centered, axis=0)
        informative = scale > np.maximum(1e-12, energy*1e-10)
        scale = np.where(informative, scale, 1.0)
        standardized = centered/scale
        standardized[:, ~informative] = 0
        u, singular, vt = np.linalg.svd(standardized, full_matrices=False)
        cutoff = max(1e-10, relative_threshold*(singular[0] if len(singular) else 0))
        rank = int(np.count_nonzero(singular > cutoff))
        delta = np.zeros(a.shape[-1])
        if penalty is not None and rank:
            # Solve in the supported subspace from the beginning, rather than
            # projecting an unconstrained optimum after fitting its nuisance terms
            latent = singular[:rank]*(u[:, :rank].T@target)/(singular[:rank]**2+penalty)
            delta = (vt[:rank].T@latent)/scale
        bias = -np.mean(b-np.einsum('pji,i->pj', a, delta), axis=0)
        return delta, bias, rank, singular
    indices = np.arange(len(designs))
    penalties = (None, 0.01, 0.1, 1.0, 10.0, 100.0, 1000.0)
    scores = []
    for penalty in penalties:
        errors = []
        for held in indices:
            delta, bias, _, _ = solve(indices[indices != held], penalty)
            error = residuals[held]-designs[held]@delta+bias
            errors.append(float(np.mean(error**2)))
        scores.append(float(np.mean(errors)))
    choice = int(np.argmin(scores))
    delta, bias, rank, singular = solve(indices, penalties[choice])
    return {'delta': delta, 'torque_bias': bias, 'rank': rank,
            'columns': designs.shape[-1], 'singular_values': singular.tolist(),
            'regularization': penalties[choice], 'prior_preferred': choice == 0,
            'cross_validation_scores': scores,
            'cross_validation_penalties': list(penalties)}


def fit_static_mass_com(prior, training, mapped, locked_links):
    """Keep locked blocks in the full model baseline and fit free m/h only."""
    prior = np.asarray(prior, dtype=float)
    active = [10*k+i for k, (_, name, _) in enumerate(mapped)
              if name not in locked_links for i in range(4)]
    if not active:
        raise ValueError('所有可辨识质量与一阶矩参数均已锁定')
    designs = [x[:, active] for x, fixed, tau in training]
    residuals = [tau-(x@prior+fixed) for x, fixed, tau in training]
    fit = solve_static_updates(designs, residuals)
    candidate = prior.copy()
    candidate[active] += fit['delta']
    return dict(fit, prior=prior, candidate=candidate, active_indices=active,
                solver='static_centered_svd_ridge_pose_cv')
