"""Mass and center of mass export source and locked-link safety"""
import csv
import json
import math
from types import SimpleNamespace
from unittest import mock
from pathlib import Path
import sys
import tempfile
import unittest
from xml.etree import ElementTree as ET

import numpy as np
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'backend'))
from gravity_legacy import (_digest, export_recomputed, recompute,
                            _raw_gravity_candidate, _static_validation, _preserve_com_inertia,
                            _com_reason)
from legacy_full_inertial import _extract_rows, _fit_regressor
from model_calibration_source import recorded_file_fingerprint


class OldNumericalCompatibility(unittest.TestCase):
    def test_exact_global_stride_and_filtering(self):
        with tempfile.TemporaryDirectory() as tmp:
            file = Path(tmp) / 'frames.csv'
            with file.open('w', newline='') as stream:
                writer = csv.writer(stream)
                writer.writerow(['phase','valid','feedback_age_ms','q:joint1','dq:joint1','acc:joint1','tau:joint1'])
                for i in range(1000):
                    writer.writerow(['static_reverse' if i < 501 else 'friction_forward_fast', '1', '1',
                                     str(i / 1000), '0.2', '0.3', '0.5'])
            rows = _extract_rows(file, ['joint1'])
            self.assertEqual(len(rows), 500)
            self.assertEqual(rows[250][0], 'static_reverse')
            self.assertEqual(rows[251][0], 'friction_forward_fast')
            self.assertAlmostEqual(rows[1][1][0], 0.002)



class LegacyRawCandidateTests(unittest.TestCase):
    def test_exact_raw_link6_is_never_overwritten_by_online_baseline(self):
        prior = np.array([1., 0., .01, .02, .03, 0, .04, 0, 0, .05,
                          .8, 0, .008, .024, .02, 0, .03, 0, 0, .04])
        raw = prior.copy()
        raw[2] = .19  # Physically impossible shoulder center: kept in report
        raw[12] = -.0064  # Effective end-effector correction
        fit = {'prior': prior, 'candidate': raw}
        result = _raw_gravity_candidate(fit, [(1, 'Link4', None), (2, 'Link6', None)],
                                        {}, 'global', set())
        original, proposed, masses, moments, _offsets, shifts, invalid = result
        np.testing.assert_array_equal(proposed, raw)
        self.assertEqual(moments[1]['value'][1], raw[12])
        self.assertEqual(masses['Link6'], raw[10])
        self.assertIn('Link4', invalid)
        self.assertNotIn('Link6', invalid)
        self.assertGreater(shifts['Link4'], .05)

    def test_locked_link_is_preserved_and_the_raw_other_link_survives(self):
        prior = np.array([1., 0., 0., 0., .1, 0, .2, 0, 0, .3,
                          .8, 0, .008, .024, .1, 0, .2, 0, 0, .3])
        raw = prior.copy(); raw[12] = -.005
        raw_fit = {'prior': prior, 'candidate': raw}
        result = _raw_gravity_candidate(raw_fit,
                    [(1, 'Link4', None), (2, 'Link6', None)], {}, 'local', {'Link4'})
        np.testing.assert_array_equal(result[1][:10], prior[:10])
        self.assertAlmostEqual(result[3][1]['value'][1], -.005)
        raw[2] = .02
        with self.assertRaisesRegex(ValueError, '可信 Link 参数被修改'):
            _raw_gravity_candidate(raw_fit,
                [(1, 'Link4', None), (2, 'Link6', None)], {}, 'local', {'Link4'})

    def test_static_validation_uses_candidate_bias_without_mutating_candidate(self):
        prior = np.r_[1., np.zeros(9)]
        candidate = prior.copy(); candidate[1] = .2
        X = np.zeros((1, 10)); X[0, 1] = 2
        def sample(value): return (X, np.zeros(1), np.asarray([value]))
        train = [sample(.4), sample(.4), sample(.4)]
        held = [sample(.4), sample(.4)]
        before, after, scale = _static_validation(train, held, prior, candidate)
        self.assertAlmostEqual(before['score'], 0.0)
        self.assertAlmostEqual(after['score'], 0.0)
        np.testing.assert_array_equal(candidate, np.r_[1., .2, np.zeros(8)])

    def test_parallel_axis_keeps_the_urdf_com_tensor(self):
        prior = np.array([.8, 0., .008, .024, .11, .002, .12, .003, .004, .13])
        selected = prior.copy(); selected[0] = .79; selected[2] = -.005
        after = _preserve_com_inertia(prior, selected)
        def about_com(p):
            m=p[0]; c=p[1:4]/m
            o=np.array([[p[4], p[5], p[7]], [p[5], p[6], p[8]], [p[7], p[8], p[9]]])
            return o-m*((c@c)*np.eye(3)-np.outer(c,c))
        np.testing.assert_allclose(about_com(after), about_com(prior), rtol=0, atol=1e-13)
        self.assertNotEqual(after[6], prior[6])


class GlobalCandidateStabilityTests(unittest.TestCase):
    def test_direct_mass_moment_solver_on_rank_deficient_synthetic_data(self):
        from gravity_mass_com import fit_global_mass_com, _dynamic_prior_with_unchanged_com_tensor
        rng = np.random.default_rng(10)
        nj, count, nf = 3, 3, 15
        prior = np.zeros(10 * count)
        for i in range(count):
            mass = 1 - .1*i
            prior[10*i:10*i+4] = [mass, .015*mass, .01*mass, .02*mass]
            prior[10*i+4:10*i+10] = [.02, 0, .03, 0, 0, .04]
        true = prior.copy()
        true[20] = .78
        true[21:24] = prior[21:24]*.78/.8 + [0.0001,-.01,.002]
        physical = _dynamic_prior_with_unchanged_com_tensor(prior, true)
        def dynamic(frames):
            Y = rng.normal(0,3,(frames,nj,len(prior))) * .04
            for i in range(count):
                Y[:,:,10*i+4:10*i+10] *= .03
            F = np.zeros((frames,nj,nf))
            for j in range(nj):
                F[:,j,5*j] = 1
            Y,F = Y.reshape(frames*nj,-1),F.reshape(frames*nj,-1)
            return np.c_[Y,F],Y@physical+rng.normal(0,.004,frames*nj)
        def stationary(frames):
            results=[]
            for _ in range(frames):
                Y=rng.normal(0,3,(nj,len(prior)))*.06
                for i in range(count):
                    Y[:,10*i+4:10*i+10]=0
                results.append((Y,np.zeros(nj),Y@physical+rng.normal(0,.002,nj)))
            return results
        fit = {'prior':prior,'train':dynamic(320),'validation':dynamic(100)}
        proposed = fit_global_mass_com(fit,stationary(24),stationary(8))
        mass = proposed['candidate'][20]
        center = proposed['candidate'][21:24]/mass
        origin = prior[21:24]/prior[20]
        self.assertAlmostEqual(mass,.78,delta=.045)
        self.assertLess(np.linalg.norm(center-np.array([.015,.01,.02])-[.0001/.78,-.01/.78,.002/.78]), .02)
        self.assertLessEqual(np.linalg.norm(center-origin), .05+1e-8)
        self.assertLess(proposed['static_holdout_scaled_rms'],proposed['static_prior_scaled_rms'])
        np.testing.assert_array_equal(proposed['candidate'][4:10],prior[4:10])
        self.assertEqual(proposed['mass_moment_columns'],12)

    def test_global_candidate_rejects_invalid_mass_or_com(self):
        prior=np.array([.8,0,.008,.024]);trial=prior.copy();trial[0]=-.2
        self.assertIn('质量',_com_reason(prior,trial,.05))
        trial=prior.copy();trial[1]=1
        self.assertIn('质心',_com_reason(prior,trial,.05))

class ExportSourceTests(unittest.TestCase):
    def setUp(self):
        t = tempfile.TemporaryDirectory(); self.addCleanup(t.cleanup)
        self.task = Path(t.name) / 'record'; (self.task / 'source').mkdir(parents=True)
        self.source = self.task / 'source/model.urdf'
        self.source.write_text('''<robot name="fixture"><link name="base"/>
        <link name="Link1"><inertial><origin xyz="0 0 0"/><mass value="1"/>
        <inertia ixx="0.2" ixy="0" ixz="0" iyy="0.3" iyz="0" izz="0.4"/></inertial></link>
        <link name="Link6"><inertial><origin xyz="0 0.01 0.03"/><mass value="0.8"/>
        <inertia ixx="0.02" ixy="0" ixz="0" iyy="0.03" iyz="0" izz="0.04"/></inertial></link>
        <joint name="joint1" type="revolute"><parent link="base"/><child link="Link1"/></joint>
        <joint name="joint6" type="revolute"><parent link="Link1"/><child link="Link6"/></joint>
        </robot>''')
        self.metadata = {'task_id':'x', 'joint_names':['joint1','joint6'],
                         'core_fingerprint':'cfg', 'urdf_fingerprint':recorded_file_fingerprint(self.source),
                         'urdf_path':str(self.source)}
        (self.task/'metadata.json').write_text(json.dumps(self.metadata))
        (self.task/'result.json').write_text(json.dumps({'task_id':'x','phase':'complete'}))
        (self.task/'frames.csv').write_text('some stable frame input\n')
        self.rec = {'solver':'legacy_full_inertial_exact', 'task_id':'x', 'core_fingerprint':'cfg',
             'urdf_fingerprint':self.metadata['urdf_fingerprint'],
             'frames_sha256':_digest(self.task/'frames.csv'), 'source_sha256':_digest(self.source),
             'identification_mode':'local', 'locked_links':['Link1'], 'fitted_links':['Link6'],
             'candidate_masses':{'Link1':1.,'Link6':.78}, 'constraints_ok':True,
             'first_moments':[{'link_name':'Link1','value':[0,0,0]},
                              {'link_name':'Link6','value':[.002, -.006, .02184]}]}

    def write(self): (self.task/'recomputed-candidate.json').write_text(json.dumps(self.rec))

    def test_local_recovers_original_full_fit_link6_without_refitting(self):
        prior = np.array([1., 0., .01, .02, .03, 0, .04, 0, 0, .05,
                          .8, 0, .008, .024, .02, 0, .03, 0, 0, .04])
        full = prior.copy()
        full[2] = .19
        full[10] = .774251897736
        full[11:14] = full[10] * np.array([.000949845843049, -.00724726249031, .0280370075459])
        online = {'first_moments': [
            {'link_name':'Link1','value':prior[1:4].tolist()},
            {'link_name':'Link6','value':prior[11:14].tolist()}]}
        (self.task/'result.json').write_text(json.dumps({'task_id':'x','phase':'complete','gravity_result':online}))
        fitting = {'prior': prior, 'candidate': full, 'rank': 10, 'degrees':20,
                   'nuisance':np.zeros(10),
                   'validation':(np.zeros((2,30)),np.zeros(2))}
        mapped = [(1,'Link1',None),(2,'Link6',None)]
        mod=SimpleNamespace(buildModelFromUrdf=lambda _: object())
        fake_static = [(np.zeros((2,20)),np.zeros(2),np.zeros(2)) for _ in range(3)]
        with mock.patch.dict(sys.modules, {'pinocchio':mod}), \
             mock.patch('gravity_legacy._articulated_mapping',return_value=(mapped,[])), \
             mock.patch('gravity_legacy._extract_rows',return_value=[1]), \
             mock.patch('gravity_legacy._fit_regressor',return_value=fitting), \
             mock.patch('gravity_legacy._static_holdout',return_value=(fake_static,fake_static[:2])):
            report = recompute(self.task,'local',['Link1'])['candidate']
        self.assertEqual(report['raw_dynamic_parameters'],full.tolist())
        np.testing.assert_array_equal(report['scoped_dynamic_parameters'][:10],prior[:10])
        np.testing.assert_allclose(report['scoped_dynamic_parameters'][10:14],full[10:14])
        exported = export_recomputed(self.task)
        root = ET.parse(exported['candidate_urdf']).getroot()
        assert root.find("./link[@name='Link1']/inertial/origin").get('xyz') == '0 0 0'
        l6 = root.find("./link[@name='Link6']/inertial")
        self.assertAlmostEqual(float(l6.find('mass').get('value')),full[10])
        self.assertAlmostEqual(float(l6.find('origin').get('xyz').split()[1]),-.00724726249031)

    def test_selected_mass_com_only_inertia_byte_equal(self):
        self.write()
        report = export_recomputed(self.task)
        original = ET.parse(self.source).getroot()
        new = ET.parse(report['candidate_urdf']).getroot()
        for name in ('Link1','Link6'):
            src = original.find(f"./link[@name='{name}']/inertial")
            dst = new.find(f"./link[@name='{name}']/inertial")
            self.assertEqual(ET.tostring(src.find('inertia')), ET.tostring(dst.find('inertia')))
            if name == 'Link1': self.assertEqual(ET.tostring(src), ET.tostring(dst))
        self.assertEqual(float(new.find("./link[@name='Link6']/inertial/mass").get('value')), .78)
        self.assertNotEqual(original.find("./link[@name='Link6']/inertial/origin").get('xyz'),
                            new.find("./link[@name='Link6']/inertial/origin").get('xyz'))
        self.assertEqual(report['candidate_source'], 'recomputed-candidate.json')

    def test_global_recompute_uses_direct_solver_not_full_inertia_raw_candidate(self):
        prior = np.array([1., 0., 0., 0., .03, 0, .04, 0, 0, .05,
                          .8, 0, .008, .024, .02, 0, .03, 0, 0, .04])
        raw = prior.copy(); raw[2] = .19; raw[12] = -.0064
        direct = prior.copy();direct[12] = -.0064
        result = {'task_id':'x', 'phase':'complete', 'gravity_result':{'first_moments':[]}}
        (self.task/'result.json').write_text(json.dumps(result))
        fitting = {'prior': prior, 'candidate': raw, 'rank': 10, 'degrees':20,
                   'nuisance':np.zeros(10),
                   'validation':(np.zeros((2,30)),np.zeros(2))}
        def direct_result(*_args,**_kwargs):
            return {'solver':'bounded_direct_mass_first_moment',
                    'candidate':direct.copy(),'exported':direct.copy(),
                    'nuisance':np.zeros(10), 'active_regressor_rank':8,
                    'mass_moment_columns':8, 'constraints':{'mass_fraction':.25,'max_com_offset_m':.05},
                    'regularization':.01,'dynamic_holdout_rms':.1,
                    'static_holdout_scaled_rms':.1,'static_prior_scaled_rms':.2,
                    'joint_noise_scale_nm':[.05,.05]}
        mapped = [(1,'Link1',None),(2,'Link6',None)]
        mod=SimpleNamespace(buildModelFromUrdf=lambda _: object())
        fake_static = [(np.zeros((2,20)),np.zeros(2),np.zeros(2)) for _ in range(3)]
        with mock.patch.dict(sys.modules, {'pinocchio':mod}), \
             mock.patch('gravity_legacy._articulated_mapping',return_value=(mapped,[])), \
             mock.patch('gravity_legacy._extract_rows',return_value=[1]), \
             mock.patch('gravity_legacy._fit_regressor',return_value=fitting), \
             mock.patch('gravity_legacy.fit_global_mass_com',side_effect=direct_result), \
             mock.patch('gravity_legacy._static_holdout',return_value=(fake_static,fake_static[:2])):
            out = recompute(self.task)
        written = json.loads((self.task/'recomputed-candidate.json').read_text())
        self.assertEqual(written['solver'],'bounded_direct_mass_first_moment')
        self.assertEqual(written['first_moments'][1]['value'][1],direct[12])
        self.assertEqual(written['raw_dynamic_parameters'],raw.tolist())
        self.assertIn('Link1', written['raw_link_review'])
        self.assertNotIn('Link1', written['link_review'])
        self.assertTrue(written['constraints_ok'])
        self.assertEqual(written['global_selection']['Link1']['source'],'bounded_direct_mass_first_moment')
        self.assertEqual(json.loads((self.task/'result.json').read_text()),result)
        output=export_recomputed(self.task)
        root=ET.parse(output['candidate_urdf']).getroot()
        self.assertAlmostEqual(float(root.find("./link[@name='Link6']/inertial/origin").get('xyz').split()[1]),-.008)

    def test_stale_unguarded_global_candidate_refused(self):
        self.rec['identification_mode'] = 'global'
        self.rec['locked_links'] = []
        self.rec['fitted_links'] = ['Link1','Link6']
        self.write()
        with self.assertRaisesRegex(ValueError, '请重新执行离线重算'):
            export_recomputed(self.task)

    def test_unphysical_com_rejected_and_no_file(self):
        self.rec['first_moments'][1]['value'] = [0,0,.5]
        self.write()
        with self.assertRaisesRegex(ValueError, '质心偏移超过允许范围'):
            export_recomputed(self.task)
        self.assertFalse((self.task/'candidate/offline/candidate-offline.urdf').exists())

    def test_stale_candidate_rejected(self):
        self.write()
        (self.task/'frames.csv').write_text('changed data')
        with self.assertRaisesRegex(ValueError, '不匹配'):
            export_recomputed(self.task)


if __name__ == '__main__': unittest.main()
