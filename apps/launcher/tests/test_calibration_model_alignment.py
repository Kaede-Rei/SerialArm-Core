import json
import sys
import tempfile
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'backend'))
from model_calibration_source import recorded_file_fingerprint
from model_calibration_alignment import compare_calibration_urdfs


URDF = '''<robot name="Demo">
<link name="base_link"/>
<link name="Link6"><inertial><origin xyz="0 0 0"/><mass value="1"/>
<inertia ixx="0.2" ixy="0" ixz="0" iyy="0.2" iyz="0" izz="0.2"/></inertial>
<visual><geometry><box size="0.1 0.1 0.1"/></geometry></visual></link>
<link name="tool0"/>
<joint name="joint6" type="revolute"><parent link="base_link"/><child link="Link6"/><axis xyz="0 0 1"/></joint>
<joint name="tool0_joint" type="fixed"><parent link="Link6"/><child link="tool0"/></joint>
</robot>'''


class ModelAlignmentTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.folder = Path(self.tmp.name)
        (self.folder / 'source').mkdir()
        self.snapshot = self.folder / 'source/model.urdf'
        self.snapshot.write_text(URDF)
        self.current = self.folder / 'current.urdf'
        self.current.write_text(URDF)
        self.metadata = {'urdf_fingerprint': recorded_file_fingerprint(self.snapshot)}

    def check(self):
        return compare_calibration_urdfs(self.folder, self.metadata, self.current)

    def test_only_inertial_changes_allow_readonly_preview_with_mass_source(self):
        root = ET.parse(self.current)
        link = root.find(".//link[@name='Link6']")
        link.find('inertial/mass').set('value', '0.8')
        link.find('inertial/origin').set('xyz', '0 0.01 0')
        link.find('inertial/inertia').set('ixx', '-0.01')
        root.write(self.current)
        result = self.check()
        self.assertTrue(result['comparable'])
        self.assertTrue(result['preview_only'])
        self.assertEqual(result['changed_links'], ['Link6'])
        self.assertEqual(result['source_inertials']['Link6']['mass'], 1)
        self.assertEqual(result['current_inertials']['Link6']['mass'], 0.8)
        self.assertTrue(any('正定性' in w for w in result['warnings']))
        self.assertNotIn('tool0', result['source_inertials'])

    def test_joint_axis_change_rejected(self):
        self.current.write_text(URDF.replace('axis xyz="0 0 1"', 'axis xyz="0 1 0"'))
        self.assertFalse(self.check()['comparable'])

    def test_geometry_change_rejected(self):
        self.current.write_text(URDF.replace('0.1 0.1 0.1', '0.5 0.1 0.1'))
        self.assertFalse(self.check()['comparable'])

    def test_corrupt_snapshot_rejected(self):
        self.snapshot.write_text(URDF.replace('Demo', 'Changed'))
        with self.assertRaisesRegex(ValueError, '指纹'):
            self.check()

    def test_no_change_uses_normal_preview(self):
        result = self.check()
        self.assertTrue(result['comparable'])
        self.assertTrue(result['same_model'])
        self.assertEqual(result['changed_links'], [])

    def test_real_tomato_picker_inertial_only_mismatch(self):
        repo = Path(__file__).resolve().parents[3]
        record = repo / '.install/model-calibration/model-calibration-ok'
        current = repo / 'src/robot_supports/robots/tomato_picker/tomato_picker_description/urdf/tomato_picker.urdf'
        if not (record / 'metadata.json').is_file():
            self.skipTest('historical user fixture not installed')
        metadata = json.loads((record / 'metadata.json').read_text())
        result = compare_calibration_urdfs(record, metadata, current)
        self.assertTrue(result['comparable'])
        self.assertEqual(result['changed_links'], ['Link6'])
        # The user's current URDF may have a corrected positive definite
        # Link6; only require a warning when this actual tensor is nonphysical
        from xml.etree import ElementTree as ET
        import numpy as np
        link = next(x for x in ET.parse(current).getroot().findall('link') if x.get('name') == 'Link6')
        inertia = link.find('inertial/inertia')
        mat = np.array([[float(inertia.get('ixx')), float(inertia.get('ixy')), float(inertia.get('ixz'))],
                        [float(inertia.get('ixy')), float(inertia.get('iyy')), float(inertia.get('iyz'))],
                        [float(inertia.get('ixz')), float(inertia.get('iyz')), float(inertia.get('izz'))]])
        eig = np.linalg.eigvalsh(mat)
        nonphysical = eig[0] <= 0 or eig[2] > eig[0] + eig[1] + 1e-9
        self.assertEqual(any('正定性' in w for w in result['warnings']), bool(nonphysical))


if __name__ == '__main__':
    unittest.main()
