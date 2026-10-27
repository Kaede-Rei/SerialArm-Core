from pathlib import Path
import sys
sys.path.insert(0, str(Path(__file__).resolve().parents[1]/"backend"))
import unittest
import numpy as np
from kinematic_consistency_audit import audit_angular_kinematics

class AuditTests(unittest.TestCase):
    def signals(self):
        t=np.linspace(0,30,6001)
        q=np.column_stack([.4*np.sin(.8*t),.3*np.sin(1.1*t),np.zeros(len(t))])
        truev=np.column_stack([.32*np.cos(.8*t),.33*np.cos(1.1*t),np.zeros(len(t))])
        return t,q,truev
    def test_known_relative_scale_and_bias(self):
        t,q,truev=self.signals();scale=np.array([1.25,1,1]);half=np.array([.00195360195,.007326007326,0])
        v=truev/scale-half;result=audit_angular_kinematics(t,q,v)
        self.assertEqual(result['status'],'requires_review')
        self.assertAlmostEqual(result['joints'][0]['scale'],1.25,places=4)
        self.assertAlmostEqual(result['joints'][0]['bias_rad_s'],1.25*half[0],places=5)
        self.assertAlmostEqual(result['joints'][1]['scale'],1,places=4)
        self.assertAlmostEqual(result['joints'][1]['bias_rad_s'],half[1],places=5)
        self.assertFalse(result['joints'][2]['scale_is_interpretable'])
        self.assertIsNone(result['joints'][2]['scale'])
    def test_constant_velocity_does_not_identify_scale(self):
        t=np.linspace(0,20,4001);v=np.full((len(t),1),.2);q=(.5*t)[:,None]
        r=audit_angular_kinematics(t,q,v)
        self.assertEqual(r['joints'][0]['status'],'insufficient_velocity_excitation')
        self.assertIsNone(r['joints'][0]['scale'])
    def test_no_mutation_and_phase_boundaries(self):
        t,q,v=self.signals();phase=np.where(t<15,'reverse','forward');q[t>=15,0]+=4
        t0,q0,v0=t.copy(),q.copy(),v.copy()
        r=audit_angular_kinematics(t,q,v,phase=phase)
        np.testing.assert_array_equal(t,t0);np.testing.assert_array_equal(q,q0);np.testing.assert_array_equal(v,v0)
        self.assertFalse(any(w['start_s']<15<=w['end_s']for w in r['windows']))
        self.assertAlmostEqual(r['joints'][0]['scale'],1,places=4)
    def test_invalid_samples_split_continuity(self):
        t,q,v=self.signals();v[2500]=np.nan
        r=audit_angular_kinematics(t,q,v)
        self.assertEqual(r['invalid_rows'],1)
        self.assertFalse(any(w['start_index']<=2500<=w['end_index_inclusive']for w in r['windows']))
    def test_insufficient_windows_not_acceptance(self):
        t,q,v=self.signals();r=audit_angular_kinematics(t[:500],q[:500],v[:500])
        self.assertFalse(r['joints'][0]['scale_is_interpretable'])
        self.assertEqual(r['status'],'limited_excitation')
    def test_bad_time_rejected(self):
        t,q,v=self.signals();t[5]=t[4]
        with self.assertRaises(ValueError):audit_angular_kinematics(t,q,v)
    def test_invalid_threshold_rejected(self):
        t,q,v=self.signals()
        with self.assertRaises(ValueError):audit_angular_kinematics(t,q,v,window_s=0)

if __name__=='__main__':unittest.main()
