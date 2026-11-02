"""Numerical and data-contract tests; never access physical hardware."""
import csv
import hashlib
import json
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'backend'))
import dynamic_identification as di


class SolverTests(unittest.TestCase):
    def dataset(self):
        rng=np.random.default_rng(481);n=160;j=2;r=4
        X=rng.normal(size=(n,j,r+3*j));beta=rng.normal(size=r+3*j)
        baseline=rng.normal(size=(n,j))
        z=dict(X=X,tau=baseline+np.einsum('nji,i->nj',X,beta),baseline=baseline,
               moving=np.ones((n,j),bool),segment=np.repeat(np.arange(8),20))
        return z,beta

    def test_all_estimators_recover_known_linear_model(self):
        z,beta=self.dataset()
        for method in ('OLS','WLS','IRLS'):
            fit=di.solve(z,4,method)
            np.testing.assert_allclose(fit['beta'],beta,atol=1e-8)
            self.assertLess(di.score(z,fit)['rms_nm'],1e-8)

    def test_local_excludes_locked_contribution_before_solving(self):
        z,beta=self.dataset()
        # First two columns are a known locked-block contribution. The local
        # equation moves these to the fixed baseline before fitting anything.
        fixed=np.einsum('nji,i->nj',z['X'][:,:,:2],beta[:2])
        local={**z,'X':z['X'][:,:,2:],'baseline':z['baseline']+fixed}
        fit=di.solve(local,2,'OLS')
        np.testing.assert_allclose(fit['beta'],beta[2:],atol=1e-9)
        self.assertLess(di.score(local,fit)['rms_nm'],1e-9)

    def test_complete_block_split_never_shares_a_segment(self):
        z,_=self.dataset();a,b=di._disjoint_split(z)
        self.assertFalse(set(a['segment'])&set(b['segment']))
        self.assertEqual(len(a['segment'])+len(b['segment']),160)

    def test_invalid_estimator_rejected(self):
        z,_=self.dataset()
        with self.assertRaises(ValueError):di.solve(z,4,'unknown')

    def test_physical_completion_uses_only_allowed_nullspace(self):
        # Uniform ellipsoid-like origin inertia and positive mass.
        prior=np.array([1.,0,0,0,.04,0,.05,0,0,.06])
        # A deliberately unphysical mass direction, declared unobservable in
        # this synthetic test. Feasible completion must preserve all else.
        raw=prior.copy();raw[0]=-1
        N=np.zeros((10,1));N[0]=1
        candidate,report=di.complete_physical(prior,raw,np.arange(10),N)
        self.assertTrue(report['applied'],report)
        self.assertGreater(candidate[0],0)
        np.testing.assert_allclose(candidate[1:],raw[1:],atol=1e-12)
        self.assertGreater(np.linalg.eigvalsh(di.pseudo_inertia(candidate))[0],0)

    def test_infeasible_completion_does_not_change_the_raw_solution(self):
        prior=np.array([1.,0,0,0,.04,0,.05,0,0,.06])
        raw=prior.copy();raw[0]=-1
        candidate,report=di.complete_physical(prior,raw,np.arange(10),np.zeros((10,0)))
        self.assertFalse(report['applied'])
        np.testing.assert_array_equal(candidate,raw)


def pendulum_task(folder,session):
    """Independent analytic one-link torque truth, not a Y@theta generator."""
    folder.mkdir();(folder/'source').mkdir()
    model=folder/'source/model.urdf'
    model.write_text('''<robot name="analytic_pendulum"><link name="base"/>
    <link name="Link1"><inertial><origin xyz="0.12 0 0.18" rpy="0 0 0"/>
    <mass value="1"/><inertia ixx="0.03" ixy="0" ixz="0" iyy="0.025" iyz="0" izz="0.02"/>
    </inertial></link><joint name="joint1" type="revolute"><parent link="base"/>
    <child link="Link1"/><axis xyz="0 1 0"/><limit lower="-2" upper="2" effort="20" velocity="3"/>
    </joint></robot>''')
    meta=dict(task_id='synthetic-'+str(session),joint_names=['joint1'],urdf_path=str(model),
              urdf_fingerprint=hashlib.sha256(model.read_bytes()).hexdigest(),gravity=[0,0,-9.81],
              units=dict(position='rad',velocity='rad/s',torque='Nm'),mapping=dict(pos_ratio=[1],tor_ratio=[1],direction=[1]),
              load_description='analytic fixture, not a hardware experiment',
              actuator_capabilities=dict(source='hardware_backend_after_successful_activate',axes=[dict(max_pos=2,max_vel=3,max_effort=20)]),
              measurement_policy=dict(static_source='accepted_static.csv',gain_independent_of_runtime_tuning=True))
    (folder/'metadata.json').write_text(json.dumps(meta));(folder/'result.json').write_text(json.dumps(dict(task_id=meta['task_id'])))
    fields=['monotonic_ns','phase','valid','feedback_age_ms','pose_group','validation','direction','q:joint1','dq:joint1','tau:joint1']
    truth_m,truth_x,truth_z,truth_I=1.08,.14,.15,.028
    def gravity(q):return -truth_m*9.81*(truth_x*np.cos(q)+truth_z*np.sin(q))
    def tau(q,v,a):
        return (truth_I+truth_m*(truth_x**2+truth_z**2))*a+gravity(q)+.035*np.sign(v)+.015*v+.01
    with (folder/'frames.csv').open('w',newline='')as stream:
        writer=csv.DictWriter(stream,fieldnames=fields);writer.writeheader()
        for k,phase in enumerate(di.PHASES):
            for i in range(2400):
                u=i*.005;f=.6+.03*session;shift=.17*session+.1*k
                q=.25+.65*np.sin(f*u+shift)+.18*np.sin(1.4*u+.2)
                v=.65*f*np.cos(f*u+shift)+.18*1.4*np.cos(1.4*u+.2)
                a=-.65*f*f*np.sin(f*u+shift)-.18*1.4**2*np.sin(1.4*u+.2)
                writer.writerow(dict(zip(fields,[1000000000+int((k*12+u)*1e9),phase,1,1,0,0,'reverse'if'reverse'in phase else'forward',q,v,tau(q,v,a)])))
    with (folder/'accepted_static.csv').open('w',newline='')as stream:
        writer=csv.DictWriter(stream,fieldnames=fields);writer.writeheader()
        n=0
        for pose,q in enumerate(np.linspace(-1.1,1.2,16),1):
            for direction in ['reverse','forward']:
                for i in range(35):
                    n+=1;writer.writerow(dict(zip(fields,[200000000000+n*5000000,'static_'+direction,1,1,pose,int(pose>12),direction,q,0,gravity(q)+(.02 if direction=='forward'else-.02)])))
    return gravity


class RecordedPipelineTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        try:import pinocchio;import cvxpy
        except ImportError:raise unittest.SkipTest('Pinocchio/CVXPY unavailable in this interpreter; full suite requires installed GUI Python')
        cls.temp=tempfile.TemporaryDirectory();cls.root=Path(cls.temp.name)
        cls.a=cls.root/'a';cls.b=cls.root/'b'
        cls.truth=pendulum_task(cls.a,1);pendulum_task(cls.b,2)
        cls.before=di._receipt(cls.a)
        cls.report=di.identify(cls.a,validation_directory=cls.b,estimator='WLS',destination=cls.root/'fit')

    @classmethod
    def tearDownClass(cls):
        if hasattr(cls,'temp'):cls.temp.cleanup()

    def test_analytic_pendulum_motion_and_gravity_generalize(self):
        r=self.report
        self.assertTrue(r['validation_motion']['passed'],r['validation_motion'])
        self.assertTrue(r['gravity_validation']['passed'],r['gravity_validation'])
        self.assertTrue(r['quality_pass'],r['audit'])
        self.assertEqual(di._receipt(self.a),self.before)
        self.assertFalse((self.a/'idim-candidate.json').exists())

    def test_duplicate_session_not_independent_validation(self):
        import shutil
        dup=self.root/'duplicate';shutil.copytree(self.a,dup)
        with self.assertRaisesRegex(ValueError,'同一份'):
            di.identify(self.a,validation_directory=dup,destination=self.root/'duplicate-fit')

    def test_local_scope_requires_explicit_trusted_links(self):
        e=di.Model(self.a)
        with self.assertRaises(ValueError):e.scope('global',['Link1'])
        with self.assertRaises(ValueError):e.scope('local',[])
        with self.assertRaises(ValueError):e.scope('local',['Link1'])
        with self.assertRaises(ValueError):e.scope('local',['Missing'])

    def test_unmeasured_joint_requires_known_fixed_pose_and_cannot_override_feedback(self):
        import shutil
        folder=self.root/'with-finger';shutil.copytree(self.a,folder)
        source=folder/'source/model.urdf';text=source.read_text().replace('</robot>','''
        <link name="finger"><inertial><origin xyz="0.1 0 0"/><mass value="0.1"/>
        <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.001"/></inertial></link>
        <joint name="finger_joint" type="revolute"><parent link="Link1"/><child link="finger"/>
        <axis xyz="0 1 0"/><limit lower="-2" upper="2" effort="1" velocity="1"/></joint></robot>''')
        source.write_text(text);meta=json.loads((folder/'metadata.json').read_text())
        meta.update(urdf_path=str(source),urdf_fingerprint=hashlib.sha256(source.read_bytes()).hexdigest())
        (folder/'metadata.json').write_text(json.dumps(meta))
        assumed=di.Model(folder);declared=di.Model(folder,{'finger_joint':.5})
        self.assertFalse(assumed.unmeasured_known);self.assertTrue(declared.unmeasured_known)
        self.assertGreater(np.max(abs(assumed.Y([.3],[0],[0])@assumed.theta-declared.Y([.3],[0],[0])@declared.theta)),1e-4)
        with self.assertRaises(ValueError):di.Model(folder,{'joint1':0})

    def test_source_change_invalidates_export(self):
        # Use a copied report with its valid hash receipt; source change alone
        # must be detected before any output folder is created.
        r=self.report
        if not r['exportable']:self.skipTest('fixture representative did not meet all export checks')
        original=(self.a/'frames.csv').read_bytes()
        try:
            (self.a/'frames.csv').write_bytes(original+b'\n')
            with self.assertRaises(ValueError):di.export_candidate(r['report_path'],self.root/'bad-export')
            self.assertFalse((self.root/'bad-export').exists())
        finally:(self.a/'frames.csv').write_bytes(original)

    def test_exported_full_model_roundtrip_and_fk(self):
        r=self.report
        self.assertTrue(r['exportable'],r['blocking_reasons'])
        result=di.export_candidate(r['report_path'],self.root/'export')
        verification=json.loads((self.root/'export/verification.json').read_text())
        self.assertTrue(verification['inertial_roundtrip']);self.assertLess(verification['fk_max_abs'],1e-12)
        self.assertFalse(verification['validated_on_hardware']);self.assertFalse(verification['observer_bias_exported'])
        self.assertTrue(Path(result['candidate_urdf']).is_file())

    def test_report_tamper_rejected(self):
        p=Path(self.report['report_path']);original=p.read_bytes()
        try:
            r=json.loads(original);r['exportable']=True;r['selected_dynamic_parameters'][0]=99
            p.write_text(json.dumps(r))
            with self.assertRaisesRegex(ValueError,'报告被修改'):di.export_candidate(p,self.root/'tampered')
        finally:p.write_bytes(original)

    def test_invalid_rows_create_boundaries_and_do_not_silently_bridge(self):
        d=di.load_observations(self.a)
        self.assertEqual(len(d['segments']),12) # each 12s phase -> 10s + 2s
        self.assertEqual(set(s['phase']for s in d['segments']),set(di.PHASES))
        self.assertEqual(d['names'],['joint1'])
        import shutil
        copy=self.root/'invalid-rows';shutil.copytree(self.a,copy)
        with (copy/'frames.csv').open(newline='')as stream:
            reader=csv.DictReader(stream);fields=reader.fieldnames;rows=list(reader)
        rows[1234]['valid']='0';rows[4567]['q:joint1']='nan'
        with (copy/'frames.csv').open('w',newline='')as stream:
            writer=csv.DictWriter(stream,fieldnames=fields);writer.writeheader();writer.writerows(rows)
        checked=di.load_observations(copy)
        for bad in [1234,4567]:
            self.assertFalse(any(s['lo']<=bad<s['hi']for s in checked['segments']))

    def test_heldout_static_torques_never_enter_model_selection(self):
        import shutil
        folder=self.root/'heldout-change';shutil.copytree(self.a,folder)
        with (folder/'accepted_static.csv').open(newline='')as f:
            reader=csv.DictReader(f);fields=reader.fieldnames;rows=list(reader)
        for row in rows:
            if row['validation']=='1':row['tau:joint1']=str(float(row['tau:joint1'])+2.)
        with (folder/'accepted_static.csv').open('w',newline='')as f:
            writer=csv.DictWriter(f,fieldnames=fields);writer.writeheader();writer.writerows(rows)
        r=di.identify(folder,validation_directory=self.b,destination=self.root/'heldout-fit')
        np.testing.assert_allclose(r['selected_dynamic_parameters'],self.report['selected_dynamic_parameters'],atol=1e-8)
        self.assertEqual(r['physical_completion']['regularization'],self.report['physical_completion']['regularization'])

    def test_review_export_separates_same_session_and_validated_status(self):
        r=di.identify(self.a,destination=self.root/'same-session')
        self.assertFalse(r['exportable']);self.assertTrue(r['review_exportable'])
        with self.assertRaises(ValueError):di.export_candidate(r['report_path'],self.root/'incorrect-ready')
        result=di.export_candidate(r['report_path'],self.root/'review-export',review_only=True)
        verification=json.loads((Path(result['directory'])/'verification.json').read_text())
        self.assertTrue(verification['review_only']);self.assertFalse(verification['validated_on_hardware'])
        diagnostics=di.export_candidate(r['report_path'],self.root/'diagnostics-export',report_only=True)
        self.assertTrue(diagnostics['report_only']);self.assertFalse((Path(diagnostics['directory'])/'robot.urdf').exists())

    def test_static_approaches_share_one_pose_group_but_keep_actual_q(self):
        from static_observations import load_static_groups,static_equations
        groups=load_static_groups(self.a,['joint1']);self.assertEqual(len(groups),16)
        self.assertTrue(all(len(g['members'])==2 for g in groups))
        rows=static_equations(di.Model(self.a),self.a)
        self.assertEqual(len(rows),16);self.assertEqual(sum(r['held']for r in rows),4)
        self.assertTrue(all(r['paired']for r in rows))

    def test_physical_constraints_are_part_of_fit_not_nullspace_repair(self):
        r=self.report;self.assertEqual(r['physical_completion']['solver'],'CLARABEL')
        self.assertGreater(r['physical_completion']['minimum_normalized_cone_eigenvalue'],0)
        self.assertTrue(r['physical_pass']);self.assertFalse(r['physical_completion']['empirical_mass_com_bounds'])

    def test_history_loads_v2_report_and_marks_changed_input_stale(self):
        from persistence import load_model_calibration_summary
        pointer=self.a/'idim-candidate.json'
        pointer.write_text(json.dumps(self.report))
        original=(self.a/'frames.csv').read_bytes()
        try:
            loaded=load_model_calibration_summary(self.a)['idim_candidate']
            self.assertEqual(loaded['solver'],'joint_physical_idim_v2')
            self.assertEqual(loaded['selected_dynamic_parameters'],self.report['selected_dynamic_parameters'])
            self.assertFalse(loaded.get('source_stale',False))
            (self.a/'frames.csv').write_bytes(original+b'\n')
            stale=load_model_calibration_summary(self.a)['idim_candidate']
            self.assertTrue(stale['source_stale']);self.assertFalse(stale['exportable']);self.assertFalse(stale['review_exportable'])
        finally:
            (self.a/'frames.csv').write_bytes(original);pointer.unlink()

    def test_explicit_mounting_declaration_is_not_written_back(self):
        import shutil
        folder=self.root/'missing-mounting';shutil.copytree(self.a,folder)
        path=folder/'metadata.json';meta=json.loads(path.read_text());meta['load_description']='unspecified';path.write_text(json.dumps(meta));before=di._receipt(folder)
        engine=di.Model(folder,load_description='known rigid fixture')
        self.assertEqual(engine.recorded_load_description,'unspecified')
        self.assertEqual(engine.meta['load_description'],'known rigid fixture')
        self.assertEqual(di._receipt(folder),before)
        with self.assertRaises(ValueError):di.Model(self.a,load_description='contradicts original fixture')


if __name__=='__main__':unittest.main()
