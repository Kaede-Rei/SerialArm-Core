"""Offline identification invariants without hardware or Pinocchio native bindings"""
import pathlib
import sys
import unittest
from unittest.mock import patch
import numpy as np
import xml.etree.ElementTree as ET

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1] / 'backend'))
from advanced_identification import assess_record, _fit_static, _write_static_candidate, _physical_urdf_links, PARAMETERS
from full_inertial import _fit_regressor

class Joint:
    idx_q = idx_v = 0
    nq = nv = 1

class Inertia:
    def toDynamicParameters(self):
        # Two kilogram link at com x=0.1 m
        return np.array([2., 0.2, 0, 0, 1, 0, 1, 0, 0, 1])

class Model:
    names=['universe','joint1']
    njoints=2
    nq=nv=1
    joints=[None,Joint()]
    inertias=[None,Inertia()]
    def createData(self):return object()

class Pin:
    @staticmethod
    def neutral(model):return np.zeros(model.nq)
    @staticmethod
    def computeJointTorqueRegressor(model,data,q,v,a):
        x=float(q[0]);return np.array([[np.sin(x),np.cos(x),np.sin(2*x),np.cos(2*x),
                                         x*0, x*0, x*0, x*0, x*0, x*0]])

class AdvancedIdentificationTests(unittest.TestCase):
    def test_historical_record_data_precheck(self):
        root=pathlib.Path(__file__).resolve().parents[3]
        complete=root/'.install/model-calibration/model-calibration-ok'
        trajectory=root/'.install/model-calibration/model-calibration-only-trajectory'
        if not complete.exists() or not trajectory.exists(): self.skipTest('offline task fixtures unavailable')
        full=assess_record(complete)
        self.assertEqual((full['training_groups'],full['holdout_groups']),(6,2))
        self.assertGreater(full['valid_dynamic_samples'],1000)
        self.assertTrue(full['gravity_ready'])
        self.assertTrue(full['dynamic_precheck'])
        demo=assess_record(trajectory)
        self.assertFalse(demo['gravity_ready'])
        self.assertFalse(demo['dynamic_precheck'])

    def samples(self, mass_delta=0.4, bias=0.03):
        result=[]
        for index,angle in enumerate(np.linspace(-1.2,1.2,9)):
            phi=Pin.computeJointTorqueRegressor(None,None,np.array([angle]),None,None)[0]
            torque=phi @ Inertia().toDynamicParameters() + mass_delta*(phi[0]+0.1*phi[1])+bias
            result.append((index+1,index>=7,np.array([angle]),np.array([torque]),120))
        return result

    def test_physical_inertia_check_catches_negative_tensor_without_modifying_urdf(self):
        xml = ('<robot><link name="tool0"/><link name="Link6"><inertial>'
               '<origin xyz="0 0 0"/><mass value="1"/>'
               '<inertia ixx="0.1" ixy="0" ixz="0" iyy="0.2" iyz="0" izz="0.2"/>'
               '</inertial></link></robot>')
        root = ET.fromstring(xml)
        self.assertEqual(_physical_urdf_links(root), [])
        root.find('.//inertia').set('ixx', '-0.1')
        self.assertEqual(_physical_urdf_links(root), ['Link6'])
        self.assertIsNone(root.find(".//link[@name='tool0']/inertial"))

    def test_physical_inertia_check_rejects_invalid_mass_and_principal_moments(self):
        root = ET.fromstring('<robot><link name="bad"><inertial><mass value="1"/>'
                             '<inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="1"/>'
                             '</inertial></link></robot>')
        self.assertEqual(_physical_urdf_links(root), ['bad'])
        root.find('.//inertia').set('izz', '0.015')
        self.assertEqual(_physical_urdf_links(root), [])
        root.find('.//mass').set('value', '-1')
        self.assertEqual(_physical_urdf_links(root), ['bad'])

    def test_mass_only_preserves_com_and_center_inertia(self):
        with patch('advanced_identification._pose_groups',return_value=self.samples()):
            fit=_fit_static(None,Model(),Pin(),[(1,'Link1',None)],'mass',set())
        self.assertEqual(fit['columns'],1)
        original=fit['prior'];after=original+fit['full_delta']
        np.testing.assert_allclose(after[1:4]/after[0],original[1:4]/original[0],atol=1e-10)
        np.testing.assert_array_equal(after[4:],original[4:])
        root=ET.fromstring('<robot><link name="Link1"><inertial><origin xyz="0.1 0 0"/><mass value="2"/><inertia ixx="1" ixy="0" ixz="0" iyy="1" iyz="0" izz="1"/></inertial></link></robot>')
        _write_static_candidate(root,[(1,'Link1',root.find('link'))],fit)
        self.assertAlmostEqual(float(root.find('link/inertial/mass').get('value'))*0.1, float(root.find('link/inertial/mass').get('value'))*float(root.find('link/inertial/origin').get('xyz').split()[0]))
        self.assertEqual(root.find('link/inertial/inertia').get('ixx'),'1')

    def test_com_only_preserves_mass(self):
        with patch('advanced_identification._pose_groups',return_value=self.samples()):
            fit=_fit_static(None,Model(),Pin(),[(1,'Link1',None)],'com',set())
        self.assertEqual(fit['columns'],3)
        self.assertEqual(fit['full_delta'][0],0.0)

    def test_locked_link_and_invalid_mode(self):
        with patch('advanced_identification._pose_groups',return_value=self.samples()):
            with self.assertRaisesRegex(ValueError,'没有可辨识'):
                _fit_static(None,Model(),Pin(),[(1,'Link1',None)],'mass',{'Link1'})
        self.assertEqual(PARAMETERS['inertia'],(4,5,6,7,8,9))
        self.assertEqual(PARAMETERS['full'],tuple(range(10)))


    def test_dynamic_physical_projection_preserves_nonselected_parameters(self):
        from advanced_identification import _physical_dynamic_candidate
        from full_inertial import _dynamic_parameters_to_urdf_fields, physical_inertia
        source = np.array([2., 0.1, 0., 0., .2, 0., .2, 0., 0., .2])
        original = np.r_[source, source]
        target = original.copy()
        target[4] = -50.
        target[6] = -50.
        target[9] = -50.
        target[14] = -50.
        selected = (4, 5, 6, 7, 8, 9)
        candidate, notes = _physical_dynamic_candidate(original, target,
                    [(1, 'Link1', None), (2, 'Link2', None)], {'Link2'}, selected)
        np.testing.assert_array_equal(candidate[10:], original[10:])
        np.testing.assert_array_equal(candidate[:4], original[:4])
        mass, com, tensor = _dynamic_parameters_to_urdf_fields(candidate[:10])
        self.assertTrue(physical_inertia(mass, com, tensor))
        self.assertTrue(notes)

    def test_dynamic_validation_fits_independent_train_friction(self):
        from advanced_identification import _validate_dynamic_candidate
        # One dynamic column and one nuisance column, evaluated on separate rows
        prior = np.array([1.])
        train_A = np.array([[1., 1.], [2., 1.], [3., 1.], [4., 1.]])
        hold_A = np.array([[5., 1.], [6., 1.]])
        fit = {'prior':prior, 'train':(train_A, np.array([2., 3., 4., 5.])),
               'validation':(hold_A, np.array([6., 7.]))}
        base, candidate = _validate_dynamic_candidate(fit, np.array([2.]))
        self.assertLess(base, 1e-3)
        self.assertGreater(candidate, base)

    def test_dynamic_parameter_mask_preserves_locked_prior(self):
        # Every zero-column must have exactly zero delta even if torque differs
        rows=[]
        for phase in ('static_reverse','friction_forward_fast'):
            for index in range(120):
                q=np.array([np.sin(index/14)])
                v=np.array([0.2*np.cos(index/12)])
                a=np.array([0.5*np.sin(index/11)])
                tau=np.array([Pin.computeJointTorqueRegressor(None,None,q,v,a)[0]@Inertia().toDynamicParameters() + 0.02])
                rows.append((phase,q,v,a,tau))
        result=_fit_regressor(rows,Model(),Pin(),[(1,'Link1',None)],active_parameters=PARAMETERS['inertia'])
        np.testing.assert_array_equal(result['candidate'][:4],result['prior'][:4])
        self.assertEqual(result['degrees'],6)

if __name__=='__main__':unittest.main()

class HistoricalExportContract(unittest.TestCase):
    """Exercise the complete offline static export with a Pinocchio-shaped mock

    Validates the historical file contracts, mapping, output and lock scope
    without presenting the mock dynamics as a physical performance measurement
    """
    def test_real_task_link6_only_candidate_export(self):
        import tempfile
        import json
        import sys
        from unittest.mock import patch
        from advanced_identification import run_advanced
        rootdir=pathlib.Path(__file__).resolve().parents[3]
        task=rootdir/'.install/model-calibration/model-calibration-ok'
        if not task.exists(): self.skipTest('historical task fixture missing')
        class FakeJoint:
            nq=nv=1
            def __init__(self,index):self.idx_q=self.idx_v=index
        class FakeInertia:
            def __init__(self,link):
                elt=link.find('inertial')
                if elt is None:self.p=np.zeros(10);return
                mass=float(elt.find('mass').get('value'))
                origin=elt.find('origin');com=np.array([float(x) for x in origin.get('xyz').split()]) if origin is not None else np.zeros(3)
                inertia=elt.find('inertia')
                ixx,iyy,izz=[float(inertia.get(name)) for name in ('ixx','iyy','izz')]
                tensor=np.diag([ixx,iyy,izz])+mass*(np.dot(com,com)*np.eye(3)-np.outer(com,com))
                self.p=np.array([mass,*(mass*com),tensor[0,0],tensor[0,1],tensor[1,1],tensor[0,2],tensor[1,2],tensor[2,2]])
            def toDynamicParameters(self):return self.p
        class FakeModel:
            def __init__(self,source):
                xml=ET.parse(source).getroot()
                links={link.get('name'):link for link in xml.findall('link')}
                names=[];bodies=[]
                for joint in xml.findall('joint'):
                    if joint.get('type')=='fixed':continue
                    names.append(joint.get('name'))
                    bodies.append(links[joint.find('child').get('link')])
                self.names=['universe']+names
                self.njoints=len(self.names)
                self.joints=[None]+[FakeJoint(i) for i in range(len(names))]
                self.inertias=[None]+[FakeInertia(link) for link in bodies]
                self.nv=self.nq=len(names)
            def createData(self):return object()
        class FakePin:
            @staticmethod
            def buildModelFromUrdf(source):return FakeModel(source)
            @staticmethod
            def neutral(model):return np.zeros(model.nq)
            @staticmethod
            def computeJointTorqueRegressor(model,data,q,v,a):
                matrix=np.zeros((model.nv,10*(model.njoints-1)))
                for row in range(model.nv):
                    for body in range(row,model.njoints-1):
                        offset=10*body
                        angle=q[row]
                        matrix[row,offset:offset+4]=[
                            0.08*np.sin(angle),
                            0.5*np.cos(angle+0.07*body),
                            np.sin(angle*1.7+0.11*body),
                            np.cos(angle*2+0.23*body)]
                return matrix
        with tempfile.TemporaryDirectory() as tmp:
            with patch.dict(sys.modules,{'pinocchio':FakePin}):
                report=run_advanced(task,'com',tmp,['Link1','Link2','Link3','Link4','Link5'])
            source=ET.parse(task/'source/model.urdf').getroot()
            candidate=ET.parse(report['candidate_urdf']).getroot()
            def inertia_xml(root,name):return ET.tostring(next(link for link in root.findall('link') if link.get('name')==name).find('inertial'))
            for name in ['Link1','Link2','Link3','Link4','Link5']:
                self.assertEqual(inertia_xml(source,name),inertia_xml(candidate,name))
            self.assertEqual(report['locked_links'],['Link1','Link2','Link3','Link4','Link5'])
            self.assertEqual(report['identification_mode'],'com')
            self.assertEqual(report['regressor_columns'],3)
            self.assertFalse(report['automatic_application_allowed'])
            self.assertFalse(report['friction_validation_reusable'])
            self.assertEqual(json.loads((pathlib.Path(report['candidate_urdf']).parent/'identification-report.json').read_text())['source_urdf_fingerprint'],json.loads((task/'metadata.json').read_text())['urdf_fingerprint'])
