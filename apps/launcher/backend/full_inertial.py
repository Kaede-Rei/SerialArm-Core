"""Offline, opt-in full-inertial URDF candidate from completed model calibration.

Never modifies the source URDF or running robot. Rank-deficient fits are marked
prior-constrained, not fully identified. Raw motor torques need verified signs,
no external contacts, and accurate joint accelerations to be trusted.
"""

from __future__ import annotations

import csv
import json
import math
import shutil
from pathlib import Path
import xml.etree.ElementTree as ET


def physical_inertia(mass, com, inertia, *, min_mass=1e-7):
    """Require SPD center inertia and principal-moment triangle inequalities."""
    import numpy as np

    if not np.isfinite(mass) or mass <= min_mass or not np.isfinite(com).all():
        return False
    tensor = np.asarray(inertia, dtype=float)
    if tensor.shape != (3, 3) or not np.isfinite(tensor).all():
        return False
    if np.max(np.abs(tensor - tensor.T)) > 1e-9:
        return False
    eig = np.linalg.eigvalsh(tensor)
    return bool(eig[0] > 1e-10 and eig[2] <= eig[0] + eig[1] + 1e-9)


def _prepare_inputs(directory):
    directory = Path(directory).expanduser().resolve()
    metadata = json.loads((directory / "metadata.json").read_text())
    result = json.loads((directory / "result.json").read_text())
    if (
        result.get("task_id") != metadata.get("task_id")
        or result.get("phase") != "complete"
    ):
        raise ValueError("完整惯性辨识需要已完成且任务标识一致的自动标定记录")
    if not (directory / "frames.csv").is_file():
        raise ValueError("缺少 frames.csv")
    # Use snapshot to ensure offline identification cannot silently switch CAD source.
    snapshot = directory / "source" / "model.urdf"
    source = Path(metadata["urdf_path"]).expanduser().resolve()
    if not snapshot.is_file():
        raise ValueError(
            "缺少任务启动时保存的 source/model.urdf；禁止使用可能已修改的在线 URDF"
        )
    return directory, metadata, result, snapshot, source


def _articulated_mapping(root, model):
    """Only 1:1 revolute joints may be written; aggregated fixed bodies are unsafe."""
    joint_entries = {j.get("name"): j for j in root.findall("joint")}
    links = {link.get("name"): link for link in root.findall("link")}
    mapped = []
    nonwritten = []
    for jid in range(1, model.njoints):
        jname = str(model.names[jid])
        item = joint_entries.get(jname)
        if item is None or item.get("type") not in (
            "revolute",
            "continuous",
            "prismatic",
        ):
            raise ValueError(f"无法将 Pinocchio 关节 {jname} 唯一映射到 URDF 可动 Link")
        child = item.find("child")
        if child is None or child.get("link") not in links:
            raise ValueError(f"{jname} 缺少子 Link")
        linkname = child.get("link")
        child_link = links[linkname]
        if child_link.find("inertial") is None:
            raise ValueError(f"可动 Link {linkname} 缺少 inertial")
        mapped.append((jid, linkname, child_link))
    names = {entry[1] for entry in mapped}
    # Pinocchio aggregates fixed-body inertias: cannot unmix individual original
    # URDF inertia values unless they have their own independent measurements.
    for name, link in links.items():
        if name not in names and link.find("inertial") is not None:
            nonwritten.append(name)
    # Reject an articulated link with nonzero-mass fixed descendants; its
    # Pinocchio inertia is an aggregate and must not be copied into the child.
    for j in root.findall("joint"):
        if j.get("type") == "fixed":
            p = j.find("parent")
            c = j.find("child")
            if p is not None and c is not None and p.get("link") in names:
                childlink = links.get(c.get("link"))
                if childlink is not None and childlink.find("inertial") is not None:
                    raise ValueError(
                        "有带惯量的固定附属 Link；Pinocchio 合并了刚体，不能安全地逐 Link 反写全部惯量"
                    )
    return mapped, nonwritten


def _extract_rows(csvpath, joints, limit=500):
    import numpy as np

    valid = []
    with open(csvpath, newline="") as inp:
        reader = csv.DictReader(inp)
        required = [f"{k}:{j}" for k in ("q", "dq", "acc", "tau") for j in joints]
        missing = [k for k in required if k not in (reader.fieldnames or [])]
        if missing:
            raise ValueError("标定帧缺少列：" + ", ".join(missing))
        for row in reader:
            # Drag demonstration is not a closed-loop motor model identification dataset.
            phase = row.get("phase", "")
            if (
                phase not in ("static_reverse", "friction_forward_fast")
                or row.get("valid") != "1"
            ):
                continue
            if float(row.get("feedback_age_ms") or 999) > 30:
                continue
            try:
                q = np.array([float(row[f"q:{j}"]) for j in joints])
                v = np.array([float(row[f"dq:{j}"]) for j in joints])
                a = np.array([float(row[f"acc:{j}"]) for j in joints])
                tau = np.array([float(row[f"tau:{j}"]) for j in joints])
            except (TypeError, ValueError):
                continue
            if (
                not all(np.isfinite(x).all() for x in (q, v, a, tau))
                or np.max(np.abs(v)) < 0.06
            ):
                continue
            valid.append((phase, q, v, a, tau))
    if len(valid) < 120:
        raise ValueError(
            f"有效自动运动帧不足（{len(valid)}）；需要更多双向激励，不能从手动拖拽帧估计惯量"
        )
    if set(item[0] for item in valid) != {"static_reverse", "friction_forward_fast"}:
        raise ValueError("缺少反向或正向回放数据，无法做双向辨识")
    if len(valid) > limit:
        stride = len(valid) / limit
        valid = [valid[min(len(valid) - 1, int(i * stride))] for i in range(limit)]
    return valid


def _fit_regressor(rows, model, pin, moving_joints, *, regularization=0.15):
    import numpy as np

    data = model.createData()
    nj = len(moving_joints)
    nparam = 10 * (model.njoints - 1)
    prior = np.concatenate(
        [
            np.asarray(model.inertias[j].toDynamicParameters(), dtype=float)
            for j in range(1, model.njoints)
        ]
    )
    X = []
    y = []
    labels = []
    for phase, q, v, a, tau in rows:
        Y = np.asarray(
            pin.computeJointTorqueRegressor(model, data, q, v, a), dtype=float
        )
        if Y.shape != (model.nv, nparam):
            raise ValueError("Pinocchio 力矩回归矩阵维度不匹配")
        # Per-axis asymmetric Coulomb + viscous friction + constant torque bias
        # as nuisance variables, estimated jointly to prevent inertia absorbing friction.
        F = np.zeros((nj, 5 * nj))
        for k in range(nj):
            vel = v[k]
            F[k, 5 * k] = 1
            F[k, 5 * k + 1] = float(vel > 0.05)
            F[k, 5 * k + 2] = max(vel, 0.0)
            F[k, 5 * k + 3] = float(vel < -0.05)
            F[k, 5 * k + 4] = min(vel, 0.0)
        X.append(np.concatenate((Y, F), axis=1))
        y.append(tau)
        labels.append(phase)
    X = np.concatenate(X)
    y = np.concatenate(y)
    nper = nj
    # contiguous time-block holdout within each direction
    # keeps calibration/train runs distinct in time, rather than random nearby points.
    phase_counts = {p: [i for i, l in enumerate(labels) if l == p] for p in set(labels)}
    hold = set()
    for phase, indices in phase_counts.items():
        hold.update(indices[int(0.8 * len(indices)) :])
    train_indices = np.concatenate(
        [np.arange(i * nj, (i + 1) * nj) for i in range(len(labels)) if i not in hold]
    )
    val_indices = np.concatenate(
        [np.arange(i * nj, (i + 1) * nj) for i in range(len(labels)) if i in hold]
    )
    A = X[train_indices]
    b = y[train_indices]
    V = X[val_indices]
    t = y[val_indices]
    # Normalize columns before ridge (prevents SI unit scales from changing regularization).
    colnorm = np.linalg.norm(A, axis=0)
    scale = np.maximum(colnorm, 1e-10)
    # Report inertia rank AFTER removing the nuisance friction/bias subspace.
    # Otherwise friction/excitation correlations overstate identifiability.
    dynamic = A[:, :nparam]
    friction = A[:, nparam:]
    cleaned = dynamic - friction @ np.linalg.lstsq(friction, dynamic, rcond=1e-9)[0]
    clean_scale = np.maximum(np.linalg.norm(cleaned, axis=0), 1e-10)
    rank = int(np.linalg.matrix_rank(cleaned / clean_scale, tol=1e-3))
    nuisance = np.zeros(5 * nj)
    # Rigid dynamics source prior; deviations are regularized, nuisance unregularized
    xprior = np.concatenate((prior, nuisance))
    A2 = A / scale
    reg = np.ones(A.shape[1]) * regularization
    reg[nparam:] = regularization * 0.1
    rhs = b - A @ xprior
    delta = (
        np.linalg.lstsq(
            np.vstack((A2, np.diag(reg))),
            np.concatenate((rhs, np.zeros(len(reg)))),
            rcond=1e-9,
        )[0]
        / scale
    )
    return {
        "prior": prior,
        "candidate": prior + delta[:nparam],
        "nuisance": delta[nparam:],
        "rank": rank,
        "degrees": nparam,
        "train": (A, b),
        "validation": (V, t),
        "X": X,
        "full_prior": xprior,
    }


def _validated_candidate(fit, model, pin):
    import numpy as np

    prior = fit["prior"]
    target = fit["candidate"]
    n = fit["degrees"]
    V, t = fit["validation"]
    # The physical prior is checked first; if invalid, do not issue an untrusted estimate.
    for j in range(1, model.njoints):
        old = model.inertias[j]
        if not physical_inertia(
            old.mass, np.asarray(old.lever), np.asarray(old.inertia)
        ):
            raise ValueError(
                f"原始 URDF 关节 {model.names[j]} 的惯量不满足物理一致性，需先修复 CAD"
            )
    baseline = np.sqrt(np.mean((t - V @ np.concatenate((prior, fit["nuisance"]))) ** 2))
    if not np.isfinite(baseline) or baseline < 1e-6:
        raise ValueError("原始 URDF 的留出集 RMS 异常，无法验证动力学改进")
    improved = None
    for alpha in (1.0, 0.5, 0.25, 0.125, 0.0625, 0.03125, 0.015625):
        proposed = prior + alpha * (target - prior)
        accepted = True
        for j in range(1, model.njoints):
            new = pin.Inertia.FromDynamicParameters(proposed[10 * (j - 1) : 10 * j])
            old = model.inertias[j]
            if not physical_inertia(
                float(new.mass), np.asarray(new.lever), np.asarray(new.inertia)
            ):
                accepted = False
                break
            if abs(float(new.mass) - float(old.mass)) > 0.30 * float(old.mass):
                accepted = False
                break
            if np.linalg.norm(np.asarray(new.lever) - np.asarray(old.lever)) > 0.05:
                accepted = False
                break
        if not accepted:
            continue
        trial = np.concatenate((proposed, fit["nuisance"]))
        rms = float(np.sqrt(np.mean((t - V @ trial) ** 2)))
        if np.isfinite(rms) and rms < 0.95 * baseline:
            improved = (proposed, rms, alpha)
            break
    if improved is None:
        raise ValueError(
            f"惯性参数候选未通过物理约束与时间块留出验证；原模型 RMS={baseline:.4f} Nm；未导出未经验证的全部惯量"
        )
    return improved, float(baseline)


def export_full_inertial_candidate(directory, destination=None):
    import numpy as np

    try:
        import pinocchio as pin
    except ImportError as exc:
        raise ValueError(
            "完整惯性辨识需要 Python Pinocchio；此功能可选，不会自动安装；请在 Launcher 使用的 Python 环境安装 pinocchio 后重试"
        ) from exc
    directory, metadata, result, snapshot, source = _prepare_inputs(directory)
    tree = ET.parse(snapshot)
    root = tree.getroot()
    model = pin.buildModelFromUrdf(str(snapshot))
    joints = metadata.get("joint_names", [])
    if (
        list(map(str, model.names[1:])) != joints
        or model.nq != len(joints)
        or model.nv != len(joints)
    ):
        raise ValueError(
            "URDF 约简、闭环、固定附属关节或顺序与采样配置不一致；拒绝反写惯量"
        )
    mapped, unchanged = _articulated_mapping(root, model)
    frames = _extract_rows(directory / "frames.csv", joints)
    fit = _fit_regressor(frames, model, pin, mapped)
    (theta, rms, alpha), baseline = _validated_candidate(fit, model, pin)
    for jid, linkname, link in mapped:
        inertia = pin.Inertia.FromDynamicParameters(theta[(jid - 1) * 10 : jid * 10])
        mat = np.asarray(inertia.inertia)
        inertial = link.find("inertial")
        origin = inertial.find("origin")
        if origin is None:
            origin = ET.SubElement(inertial, "origin")
        origin.attrib.update(
            xyz=" ".join(f"{v:.12g}" for v in inertia.lever), rpy="0 0 0"
        )
        mass = inertial.find("mass")
        mass.attrib["value"] = f"{inertia.mass:.12g}"
        tensor = inertial.find("inertia")
        if tensor is None:
            tensor = ET.SubElement(inertial, "inertia")
        for name, value in (
            ("ixx", mat[0, 0]),
            ("ixy", mat[0, 1]),
            ("ixz", mat[0, 2]),
            ("iyy", mat[1, 1]),
            ("iyz", mat[1, 2]),
            ("izz", mat[2, 2]),
        ):
            tensor.attrib[name] = f"{value:.12g}"
    output = (
        Path(destination).expanduser().resolve()
        if destination
        else directory / "candidate_full_inertial"
    )
    output.mkdir(parents=True, exist_ok=True)
    # Copy relative visual/collision meshes into the candidate bundle. Read only
    # matching original files; never follow arbitrary unverified task metadata.
    from persistence import sha256_file

    if source.is_file() and sha256_file(source) == sha256_file(snapshot):
        for index, mesh in enumerate(root.findall(".//mesh")):
            filename = mesh.get("filename", "")
            if (
                not filename
                or filename.startswith("package://")
                or Path(filename).is_absolute()
            ):
                continue
            src = (source.parent / filename).resolve()
            if not src.is_file():
                raise ValueError(f"候选资源文件缺失: {filename}")
            resources = output / "resources"
            resources.mkdir(parents=True, exist_ok=True)
            dest = resources / f"{index:03d}-{src.name}"
            shutil.copy2(src, dest)
            mesh.set("filename", dest.relative_to(output).as_posix())
    else:
        relative = [
            mesh.get("filename", "")
            for mesh in root.findall(".//mesh")
            if mesh.get("filename", "")
            and not mesh.get("filename", "").startswith("package://")
            and not Path(mesh.get("filename", "")).is_absolute()
        ]
        if relative:
            raise ValueError("原模型位置或指纹变化，无法安全复制相对网格资源")
    dst = output / "candidate-inertial.urdf"
    tmp = output / "candidate-inertial.urdf.tmp"
    tree.write(tmp, encoding="utf-8", xml_declaration=True)
    # Validate full generated inertial URDF is parsable by Pinocchio.
    parsed = pin.buildModelFromUrdf(str(tmp))
    if parsed.nv != model.nv:
        tmp.unlink(missing_ok=True)
        raise ValueError("候选 URDF 回读自由度不匹配")
    for jid, _, _ in mapped:
        expected = theta[(jid - 1) * 10 : jid * 10]
        actual = np.asarray(parsed.inertias[jid].toDynamicParameters())
        if np.linalg.norm(expected - actual) > 1e-7 * max(
            1.0, np.linalg.norm(expected)
        ):
            tmp.unlink(missing_ok=True)
            raise ValueError(
                f"候选惯量写入与 Pinocchio 重新载入不一致：{model.names[jid]}"
            )
    tmp.replace(dst)
    report = {
        "candidate_urdf": str(dst),
        "status": "prior_regularized_candidate_not_certified",
        "verified_joint_inertials": [name for _, name, _ in mapped],
        "unchanged_inertials": unchanged,
        "joint_regressor_rank": fit["rank"],
        "joint_regressor_columns": fit["degrees"],
        "all_parameters_identifiable": fit["rank"] == fit["degrees"] and not unchanged,
        "validation_original_rms_nm": baseline,
        "validation_candidate_rms_nm": rms,
        "prior_to_fit_step": alpha,
        "frames_used": len(frames),
        "constraints": [
            "positive mass",
            "positive definite COM tensor",
            "principal moment triangle inequalities",
            "mass change <=30%",
            "COM delta <=0.05m",
            "holdout RMS >=5% improvement",
        ],
        "limitations": "Dynamic regressor with motor torque data; friction and motor drive effects are not fully isolated. Do not deploy without independent experiments and physical review.",
        "source_snapshot": str(snapshot),
    }
    (output / "inertial-identification-report.json").write_text(
        json.dumps(report, indent=2, ensure_ascii=False)
    )
    return report
