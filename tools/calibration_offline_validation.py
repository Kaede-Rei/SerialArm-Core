#!/usr/bin/env python3
"""Deterministic offline gravity replay for saved Tomato-Picker calibration tasks

This is an independent URDF-kinematics sanity check, not Pinocchio and not a
substitute for C++ model or hardware validation. It never commands a robot
and does not modify the original records. Requires NumPy.
"""
from __future__ import annotations
import argparse
import collections
import csv
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np


def rotate(axis, angle):
    axis = np.asarray(axis, dtype=float)
    axis /= np.linalg.norm(axis)
    x, y, z = axis
    c, s = math.cos(angle), math.sin(angle)
    return np.eye(3) * c + (1 - c) * np.outer(axis, axis) + s * np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])


def xyz(text):
    return np.array([float(x) for x in (text or '0 0 0').split()], dtype=float)


def origin(element):
    if element is None:
        return np.eye(3), np.zeros(3)
    roll, pitch, yaw = xyz(element.get('rpy'))
    return rotate([0, 0, 1], yaw) @ rotate([0, 1, 0], pitch) @ rotate([1, 0, 0], roll), xyz(element.get('xyz'))


class GravityModel:
    def __init__(self, path, joints, gravity):
        root = ET.parse(path).getroot()
        self.joints = list(joints)
        self.gravity = np.asarray(gravity, dtype=float)
        self.links = {}
        for node in root.findall('link'):
            inertia = node.find('inertial')
            if inertia is not None and inertia.find('mass') is not None:
                mass = float(inertia.find('mass').get('value'))
                if mass > 0:
                    com = xyz(inertia.find('origin').get('xyz') if inertia.find('origin') is not None else None)
                    self.links[node.get('name')] = (mass, com)
        self.children = collections.defaultdict(list)
        self.fit_links = []
        for node in root.findall('joint'):
            name = node.get('name')
            parent = node.find('parent').get('link')
            child = node.find('child').get('link')
            typ = node.get('type')
            axis = xyz(node.find('axis').get('xyz') if node.find('axis') is not None else '1 0 0')
            r, p = origin(node.find('origin'))
            self.children[parent].append((name, child, typ, axis, r, p))
            if name in self.joints and child in self.links:
                self.fit_links.append(child)
        assert len(self.fit_links) == len(self.joints), f'sampled link mapping mismatch {self.fit_links}'

    def regressor(self, q):
        joint_q = dict(zip(self.joints, q))
        joint_axes = {}
        link_poses = {}
        def travel(name, R, p, ancestors):
            link_poses[name] = (R, p, tuple(ancestors))
            for jname, child, typ, axis, r, shift in self.children[name]:
                jr, jp = R @ r, p + R @ shift
                aligned = jr @ axis
                next_anc = ancestors
                if jname in joint_q:
                    joint_axes[jname] = (aligned, jp)
                    next_anc = ancestors + [jname]
                if typ in ('revolute', 'continuous'):
                    child_r, child_p = jr @ rotate(axis, float(joint_q.get(jname, 0.0))), jp
                elif typ == 'prismatic':
                    child_r, child_p = jr, jp + aligned * float(joint_q.get(jname, 0.0))
                else:
                    child_r, child_p = jr, jp
                travel(child, child_r, child_p, next_anc)
        travel('base_link', np.eye(3), np.zeros(3), [])
        g = np.zeros(len(self.joints))
        y = np.zeros((len(self.joints), 3 * len(self.fit_links)))
        for link, (mass, com) in self.links.items():
            if link not in link_poses:
                continue
            R, p, ancestors = link_poses[link]
            center = p + R @ com
            for joint in ancestors:
                i = self.joints.index(joint)
                axis, pivot = joint_axes[joint]
                # URDF mass/gravity gravitational generalized torque
                g[i] -= mass * self.gravity.dot(np.cross(axis, center - pivot))
                if link in self.fit_links:
                    col = self.fit_links.index(link) * 3
                    for k in range(3):
                        y[i, col + k] -= self.gravity.dot(np.cross(axis, R[:, k]))
        return g, y


def frames_grouped(path):
    groups = collections.defaultdict(list)
    phase_counts = collections.Counter()
    with open(path, newline='') as f:
        reader = csv.DictReader(f)
        if not reader.fieldnames or 'pose_group' not in reader.fieldnames:
            raise ValueError('not a calibration frame file')
        for row in reader:
            phase_counts[row['phase']] += 1
            if row['pose_group'] != '0' and 'static' in row['phase']:
                groups[int(row['pose_group'])].append(row)
    selected = []
    raw_count = sum(len(group) for group in groups.values())
    for ident, group in sorted(groups.items()):
        last = int(group[-1]['monotonic_ns'])
        window = [r for r in group if last - int(r['monotonic_ns']) <= 700_000_000]
        window = [r for r in window if r['valid'] == '1' and max(abs(float(r[f'dq:joint{i}'])) for i in range(1, 7)) <= .05]
        if not window:
            continue
        pos = np.array([[float(r[f'q:joint{i}']) for i in range(1, 7)] for r in window]); torque = np.array([[float(r[f'tau:joint{i}']) for i in range(1, 7)] for r in window])
        selected.append({'id': ident, 'validation': group[0]['validation'] == '1', 'q': pos.mean(axis=0), 'tau': torque.mean(axis=0), 'sigma': torque.std(axis=0), 'count': len(window)})
    return selected, raw_count, phase_counts


def fitting(model, poses, regularization=.01, relative_rank=1e-4):
    train = [p for p in poses if not p['validation']]
    validation = [p for p in poses if p['validation']]
    if len(train) < 4 or len(validation) < 1:
        return {'status':'not_fit','reason':'insufficient_static_pose_groups','training_groups':len(train),'validation_groups':len(validation)}
    # Independent whole-pose training-only leave-one-out for ridge model choice
    training_y = np.stack([model.regressor(p['q'])[1] for p in train])
    training_b = np.stack([p['tau'] - model.regressor(p['q'])[0] for p in train])
    def solve_subset(keep, ridge):
        ym, bm = training_y[keep], training_b[keep]
        a = (ym - ym.mean(axis=0)).reshape(-1, training_y.shape[2])
        b = (bm - bm.mean(axis=0)).reshape(-1)
        scale = np.sqrt(np.mean(a*a,axis=0))
        rms_ref = np.sqrt(np.mean(ym*ym,axis=(0,1)))
        active = scale > np.maximum(1e-12,1e-9*rms_ref)
        normalized = a.copy()
        normalized[:,active] /= scale[active]
        normalized[:,~active] = 0
        u,s,vt = np.linalg.svd(normalized,full_matrices=False)
        coeff = u.T@b
        coeff *= np.where((s > s[0]*relative_rank) if len(s) and s[0] else np.zeros(len(s),dtype=bool),s/(s*s+ridge),0)
        step = vt.T@coeff
        return np.where(active,step/np.where(active,scale,1),0)
    candidate_lambdas = sorted(set([regularization, max(.1,regularization), max(1,regularization),max(10,regularization),max(100,regularization),max(1000,regularization)]))
    cv_errors = {}
    all_idx = np.arange(len(train))
    for ridge in [None]+candidate_lambdas:
        residual_sum = 0.0
        for held in range(len(train)):
            keep = all_idx[all_idx!=held]
            dh = np.zeros(training_y.shape[2]) if ridge is None else solve_subset(keep,ridge)
            y_center = training_y[keep].mean(axis=0)
            b_center = training_b[keep].mean(axis=0)
            predicted = (training_y[held]-y_center)@dh-(training_b[held]-b_center)
            residual_sum += float(predicted@predicted)
        cv_errors['prior' if ridge is None else str(ridge)] = residual_sum
    best = min(cv_errors,key=cv_errors.get)
    prior_preferred = best=='prior'
    regularization = regularization if prior_preferred else float(best)
    train_pred, train_reg = zip(*(model.regressor(p['q']) for p in train))
    orig = np.stack(train_pred)
    Y = np.stack(train_reg)
    measured = np.stack([p['tau'] for p in train])
    # identical per-joint centering to native C++
    A = (Y - Y.mean(axis=0)).reshape(-1, Y.shape[2])
    target = ((measured - orig) - (measured - orig).mean(axis=0)).reshape(-1)
    scale = np.sqrt(np.mean(A * A, axis=0))
    raw = np.sqrt(np.mean(Y * Y, axis=(0, 1)))
    active = scale > np.maximum(1e-12, 1e-9*raw)
    norm = A.copy()
    norm[:, active] /= scale[active]
    norm[:, ~active] = 0
    u, s, vt = np.linalg.svd(norm, full_matrices=False)
    rank = int(np.sum(s > s[0]*relative_rank)) if len(s) and s[0] else 0
    w = np.ones(len(target))
    delta = np.zeros(Y.shape[2])
    for _ in range(5 if not prior_preferred else 0):
        weight = np.sqrt(w)
        uw, sw, vtw = np.linalg.svd(norm*weight[:, None], full_matrices=False)
        coeff = uw.T @ (target*weight)
        coeff *= np.where(sw > max(sw[0]*relative_rank, 1e-12), sw/(sw*sw+regularization), 0)
        delta = vtw.T @ coeff
        residual = (norm @ delta-target).reshape(len(train), 6)
        for j in range(6):
            med = np.median(abs(residual[:, j])); huber = max(.05, 1.345*1.4826*med)
            w[j::6] = np.minimum(1, huber/np.maximum(huber, abs(residual[:, j])))
    dh = np.where(active, delta/np.where(active, scale, 1), 0)
    bias = np.stack([model.regressor(p['q'])[0] + model.regressor(p['q'])[1] @ dh-p['tau'] for p in train]).mean(axis=0)
    original_bias = (orig-measured).mean(axis=0)
    def rms(items, candidate):
        err=[]
        for p in items:
            g, m = model.regressor(p['q'])
            err.append(g + (m @ dh if candidate else 0) - p['tau'] - (bias if candidate else original_bias))
        return np.sqrt(np.mean(np.array(err)**2,axis=0))
    original = rms(validation, False)
    candidate = rms(validation, True)
    allowed = np.maximum(.005, .05*original)
    passes=bool(rank > 0 and not prior_preferred and np.all(candidate <= original+allowed) and np.sum(candidate**2) <= np.sum(original**2)+1e-12)
    return {'status':'fit','rank':rank,'columns':int(Y.shape[2]),'training_groups':len(train),'validation_groups':len(validation),'train_samples':sum(p['count'] for p in train),'validation_samples':sum(p['count'] for p in validation),'validation_original_rms':original.round(6).tolist(),'validation_candidate_rms':candidate.round(6).tolist(),'approval':passes,'fit_links':model.fit_links,'max_com_shift_m':float(max(np.linalg.norm(dh[3*k:3*k+3]/model.links[link][0]) for k,link in enumerate(model.fit_links))), 'first_moment_delta':dh.tolist(),'selected_regularization':regularization,'prior_preferred':prior_preferred,'training_cv_rms':{k:round(float(math.sqrt(v/(len(train)*6))),6) for k,v in cv_errors.items()}}


def run(directory):
    directory = Path(directory).resolve()
    metadata = json.loads((directory/'metadata.json').read_text())
    poses, raw, phases=frames_grouped(directory/'frames.csv')
    result={'task':directory.name,'static_groups_raw':len(set(p['id'] for p in poses)),'raw_grouped_frames':raw,'selected_groups':len(poses),'selected_static_frames':sum(p['count'] for p in poses),'phases':dict(phases)}
    model = GravityModel(directory/'source/model.urdf', metadata['joint_names'], metadata['gravity'])
    result['static_fit']=fitting(model, poses, regularization=metadata['calibration_options']['regularization'],relative_rank=metadata['calibration_options']['svd_relative_threshold'])
    return result


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('directories',nargs='+');parser.add_argument('--output');args=parser.parse_args()
    reports=[run(d) for d in args.directories]
    print(json.dumps(reports,ensure_ascii=False,indent=2))
    if args.output:Path(args.output).write_text(json.dumps(reports,ensure_ascii=False,indent=2)+'\n')
