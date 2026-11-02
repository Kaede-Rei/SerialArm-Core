import json
from pathlib import Path
import sys
import tempfile
import threading
import time
import unittest
import yaml

BUILD, PLUGIN, ROOT = sys.argv[1:4]
sys.argv = sys.argv[:1]
sys.path.insert(0, BUILD)
import _serial_arm as arm

class PythonRuntime(unittest.TestCase):
    def test_offline_start_preserves_capabilities_without_using_hardware_state(self):
        with tempfile.TemporaryDirectory(prefix='serialarm-offline-session-') as temporary:
            folder=Path(temporary)
            original=Path(ROOT)/'src/robot_supports/robots/dm_arm/description/config/core/gray.yaml'
            data=yaml.safe_load(original.read_text())
            data['model']['urdf_path']=str((original.parent/data['model']['urdf_path']).resolve())
            data['control']['runtime']['write_enabled']=False
            data['control']['runtime']['model_feedforward_mode']='NONE'
            data['capability']['admittance']['enabled']=False
            names=data['model']['joint_names']
            # The configured backend deliberately reports an impossible pose.
            # Offline operation must only use its static capabilities and must
            # start the internal simulator, which begins at zero, instead.
            hardware=folder/'hardware.yaml'
            hardware.write_text(yaml.safe_dump({'joints':names,'positions':[1000.0]*len(names)}))
            config=folder/'core.yaml';config.write_text(yaml.safe_dump(data,sort_keys=False))
            session=arm._RobotSession();session.configure(str(config),PLUGIN,str(hardware))
            limits=list(session.config.safety.limits.max_effort)
            for _ in range(2):
                try:
                    session.start()
                    deadline=time.monotonic()+2
                    while not session.snapshot.valid and session.running and time.monotonic()<deadline:
                        time.sleep(.01)
                    snapshot=session.snapshot
                    self.assertTrue(snapshot.valid,snapshot.last_error)
                    self.assertEqual(snapshot.robot_state,arm.RobotState.ACTIVE)
                    self.assertTrue(all(abs(value)<1e-9 for value in snapshot.cycle.joint_state.pos))
                    self.assertFalse(session.config.runtime.write_enabled)
                    self.assertEqual(list(session.config.safety.limits.max_effort),limits)
                    self.assertEqual([info.joint_name for info in session.actuator_info],names)
                    self.assertTrue(all(info.max_vel>0 and info.max_effort>0 for info in session.actuator_info))
                finally:
                    session.stop()
                self.assertFalse(session.running)
                self.assertEqual(session.state,arm.RobotState.INACTIVE)

    def test_start_and_stop_are_serialized(self):
        with tempfile.TemporaryDirectory(prefix='serialarm-session-') as temporary:
            folder=Path(temporary)
            original=Path(ROOT)/'src/robot_supports/robots/dm_arm/description/config/core/gray.yaml'
            data=yaml.safe_load(original.read_text())
            data['model']['urdf_path']=str((original.parent/data['model']['urdf_path']).resolve())
            data['control']['runtime']['write_enabled']=True
            data['control']['runtime']['model_feedforward_mode']='NONE'
            data['capability']['admittance']['enabled']=False
            names=data['model']['joint_names']
            positions=[]
            for name in names:
                mapping=data['calibration']['joints'][name]
                positions.append(mapping['actuator_zero_offset']+mapping['direction']*mapping['pos_ratio']*(data['shutdown']['park_pos'][name]-mapping['joint_zero_offset']))
            hardware=folder/'hardware.yaml';hardware.write_text(yaml.safe_dump({'joints':names,'positions':positions,'activation_delay_ms':150}))
            config=folder/'core.yaml';config.write_text(yaml.safe_dump(data,sort_keys=False))
            for _ in range(3):
                session=arm._RobotSession();session.configure(str(config),PLUGIN,str(hardware))
                errors=[]
                def start():
                    try: session.start()
                    except Exception as error: errors.append(error)
                worker=threading.Thread(target=start);worker.start();time.sleep(.03)
                session.stop();worker.join(3)
                self.assertFalse(worker.is_alive());self.assertFalse(errors)
                self.assertFalse(session.running)
                self.assertEqual(session.state,arm.RobotState.INACTIVE)

if __name__=='__main__':unittest.main()
