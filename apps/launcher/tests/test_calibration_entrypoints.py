import importlib.util
import json
import os
from pathlib import Path
import sys
import tempfile
import unittest
from unittest import mock

ROOT=Path(__file__).resolve().parents[3]
sys.path.insert(0,str(ROOT/'apps/launcher/backend'))
import calibration_jobs
from profiles import Inspector
spec=importlib.util.spec_from_file_location('calibration_cli',ROOT/'tools/model_calibration_cli.py')
cli=importlib.util.module_from_spec(spec);spec.loader.exec_module(cli)

class CalibrationEntrypoints(unittest.TestCase):
    def test_process_success_does_not_imply_static_pass(self):
        result={'path':'candidate.json','candidate':{'static_pass':False,'prior_preferred':True},'static_pass':False}
        with mock.patch.object(cli,'recompute_gravity_isolated',return_value=result) as job:
            value=cli.recompute(Path('core.yaml'),Path('task'),'local',['link2'])
        job.assert_called_once_with(Path('task'),'local',['link2'])
        self.assertFalse(value['static_validation_passed'])
        self.assertTrue(value['candidate']['prior_preferred'])
    def test_shared_worker_preserves_local_scope(self):
        with tempfile.TemporaryDirectory() as temporary:
            with mock.patch.object(calibration_jobs,'run_full_inertial_isolated',return_value={'static_pass':False}) as job:
                calibration_jobs.recompute_gravity_isolated(temporary,'local',['link2'])
            self.assertEqual(job.call_args.kwargs['mode'],'legacy_gravity_recompute')
            self.assertEqual(job.call_args.kwargs['locked_links'],{'mode':'local','locked_links':['link2']})
            with self.assertRaises(ValueError):calibration_jobs.recompute_gravity_isolated(temporary,'global',['link2'])
            with self.assertRaises(ValueError):calibration_jobs.recompute_gravity_isolated(temporary,'local',[{}])
    def test_native_tools_prefer_selected_installation(self):
        with tempfile.TemporaryDirectory() as temporary:
            root=Path(temporary);prefix=root/'active';old=root/'.install/standalone/bin'
            (prefix/'bin').mkdir(parents=True);old.mkdir(parents=True)
            for folder in (prefix/'bin',old):
                for name in ('serial_arm_terminal','serial_arm_model_probe','serial_arm_model_calibrator'):
                    target=folder/name;target.write_text('#!/bin/sh\nexit 0\n');target.chmod(0o755)
            inspector=Inspector(root)
            with mock.patch.object(inspector,'manifest',return_value={'install_prefix':str(prefix)}),mock.patch.dict(os.environ,{},clear=True):
                self.assertEqual(inspector.machine_terminal(),str(prefix/'bin/serial_arm_terminal'))
                self.assertEqual(inspector.model_probe(),str(prefix/'bin/serial_arm_model_probe'))
                self.assertEqual(inspector.model_calibrator(),str(prefix/'bin/serial_arm_model_calibrator'))
    def test_offline_export_verifies_selected_candidate_without_refitting(self):
        output={'candidate_urdf':'/tmp/candidate/offline/model.urdf'}
        verified={'ok':True,'quality_validation_performed':False}
        completed=mock.Mock(returncode=0,stdout=json.dumps(verified),stderr='')
        with mock.patch.object(cli,'run_full_inertial_isolated',return_value=output) as job, \
             mock.patch.object(cli,'find_calibrator',return_value='/test/calibrator'), \
             mock.patch.object(cli.subprocess,'run',return_value=completed) as run, \
             mock.patch.object(cli,'update_candidate_urdf_verification',return_value=verified):
            result=cli.export_verified(Path('core.yaml'),Path('task'),None,'offline')
        job.assert_called_once_with(Path('task'),None,mode='legacy_gravity_export')
        self.assertEqual(run.call_args.args[0][-2:],['--candidate-source','offline'])
        self.assertFalse(result['verification']['quality_validation_performed'])

if __name__=='__main__':unittest.main()
