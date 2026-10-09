"""No Pinocchio dependency required for conservative candidate guards."""
import pathlib
import sys
import unittest
import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1] / 'backend'))
from full_inertial import physical_inertia, _fit_regressor, _dynamic_parameters_to_urdf_fields, _review_candidate


class ConservativeEstimatorTest(unittest.TestCase):
    def test_physical_consistency(self):
        self.assertTrue(physical_inertia(1.2,np.zeros(3),np.diag([1.,1.5,2.])))
        self.assertFalse(physical_inertia(0.0,np.zeros(3),np.eye(3)))
        self.assertFalse(physical_inertia(1.0,np.zeros(3),np.diag([0.1,0.1,0.5])))
        self.assertFalse(physical_inertia(1.0,np.zeros(3),np.diag([-1.,2.,2.])))

    def test_unbounded_dynamic_parameters_do_not_clip_mass_or_com(self):
        # m=4 kg, c=(0.4, -0.2, 0.1) m: much larger than old ±5 cm bound.
        m=4.0;com=np.array([0.4,-0.2,0.1]);I_com=np.diag([0.3,0.4,0.5]);I0=I_com+m*((com@com)*np.eye(3)-np.outer(com,com))
        p=np.array([m,*(m*com),I0[0,0],I0[0,1],I0[1,1],I0[0,2],I0[1,2],I0[2,2]])
        m2,c2,i2=_dynamic_parameters_to_urdf_fields(p)
        self.assertEqual(m2,m)
        np.testing.assert_allclose(c2,com)
        np.testing.assert_allclose(i2,I_com)

    def test_holdout_failure_is_warning_not_a_candidate_gate(self):
        class I:
            mass=1.0
            lever=np.zeros(3)
            inertia=np.eye(3)
        class M:
            njoints=2
            names=['universe','joint1']
            inertias=[None,I()]
        class P:
            class Inertia:
                @staticmethod
                def FromDynamicParameters(parameters):
                    class New:
                        mass=float(parameters[0])
                        lever=np.array([float(parameters[1])/mass,0,0])
                        inertia=np.eye(3)
                    return New()
        prior=np.array([1.,0,0,0,1,0,1,0,0,1])
        fit={'prior':prior, 'candidate':np.array([1.,0.8,0,0,1,0,1,0,0,1]),
            'nuisance':np.zeros(0),'validation':(np.ones((2,10)),np.zeros(2)), 'rank':3,'degrees':10}
        (candidate, _, alpha), _, warnings=_review_candidate(fit,M(),P())
        self.assertEqual(alpha,1.0)
        np.testing.assert_allclose(candidate,fit['candidate'])
        self.assertTrue(any('质心偏移' in w for w in warnings))

    def test_joint_torque_regression_includes_nuisance_terms(self):
        class I:
            def toDynamicParameters(self):
                return np.array([1.,0.,0.,0.,1.,0.,1.,0.,0.,1.])
        class J:
            nq=nv=1
            idx_q=idx_v=0
        class M:
            njoints=2
            nq=nv=1
            names=['universe','joint1']
            joints=[None,J()]
            inertias=[None,I()]
            def createData(self):return object()
        class P:
            @staticmethod
            def neutral(model):return np.zeros(model.nq)
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
