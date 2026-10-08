"""No Pinocchio dependency required for conservative candidate guards."""
import pathlib
import sys
import unittest
import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1] / 'backend'))
from full_inertial import physical_inertia, _fit_regressor


class ConservativeEstimatorTest(unittest.TestCase):
    def test_physical_consistency(self):
        self.assertTrue(physical_inertia(1.2,np.zeros(3),np.diag([1.,1.5,2.])))
        self.assertFalse(physical_inertia(0.0,np.zeros(3),np.eye(3)))
        self.assertFalse(physical_inertia(1.0,np.zeros(3),np.diag([0.1,0.1,0.5])))
        self.assertFalse(physical_inertia(1.0,np.zeros(3),np.diag([-1.,2.,2.])))

    def test_joint_torque_regression_includes_nuisance_terms(self):
        class I:
            def toDynamicParameters(self):
                return np.array([1.,0.,0.,0.,1.,0.,1.,0.,0.,1.])
        class M:
            njoints=2
            nv=1
            names=['universe','joint1']
            inertias=[None,I()]
            def createData(self):return object()
        class P:
            @staticmethod
            def computeJointTorqueRegressor(model,data,q,v,a):
                return np.array([[q[0], v[0], a[0], q[0]*v[0], 1., q[0]**2, v[0]**2, q[0]*a[0], v[0]*a[0], a[0]**2]])
        rows=[]
        for direction in ('static_reverse','friction_forward_fast'):
            for i in range(110):
                q=np.array([np.sin(i/13.)]);v=np.array([0.15+0.1*np.cos(i/9.)]);a=np.array([0.4*np.sin(i/11.)]);
                Y=P.computeJointTorqueRegressor(None,None,q,v,a)
                torque=Y @ I().toDynamicParameters()+np.array([0.03+0.1*v[0]])
                rows.append((direction,q,v,a,torque))
        result=_fit_regressor(rows,M(),P(),[(1,'link1',None)])
        self.assertEqual(result['candidate'].shape,(10,))
        self.assertEqual(result['nuisance'].shape,(5,))
        self.assertTrue(np.isfinite(result['candidate']).all())
        self.assertGreater(result['rank'],0)
        self.assertLessEqual(result['rank'],10)


if __name__ == '__main__':
    unittest.main()
