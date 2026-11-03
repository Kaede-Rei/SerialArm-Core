import json
from pathlib import Path
import sys
import tempfile
import threading
import time
import unittest
import importlib.util
import contextlib
import io
from unittest.mock import patch
import yaml

BUILD, PLUGIN, ROOT = sys.argv[1:4]
sys.argv = sys.argv[:1]
sys.path.insert(0, BUILD)
import _serial_arm as arm
public_path=Path(ROOT)/'src/serial_arm/core/python/serial_arm/__init__.py'
public_spec=importlib.util.spec_from_file_location('serialarm_public_review',public_path,
    submodule_search_locations=[str(public_path.parent)])
public=importlib.util.module_from_spec(public_spec)
sys.modules['serialarm_public_review._serial_arm']=arm
public_spec.loader.exec_module(public)

class PythonRuntime(unittest.TestCase):
    def test_motion_units_follow_the_loaded_urdf(self):
        with tempfile.TemporaryDirectory(prefix='serialarm-linear-unit-') as temporary:
            model=Path(temporary)/'linear.urdf'
            model.write_text('<robot name="linear"><link name="base"/><link name="tool">'
                '<inertial><mass value="1"/><inertia ixx="0.1" ixy="0" ixz="0" iyy="0.1" iyz="0" izz="0.1"/></inertial>'
                '</link><joint name="slide" type="prismatic"><parent link="base"/><child link="tool"/>'
                '<axis xyz="1 0 0"/><limit lower="0" upper="1" effort="100" velocity="1"/></joint></robot>')
            cfg=arm.DynamicsCfg();cfg.urdf_path=str(model);cfg.joint_names=['slide']
            cfg.base_frame='base';cfg.tool_frame='tool';cfg.gravity_scale=[1.0]
            dynamics=arm.Dynamics();dynamics.configure(cfg)
            self.assertEqual(list(dynamics.info.joint_types),['prismatic'])

    def test_python_terminal_selects_four_gain_uses(self):
        spec=importlib.util.spec_from_file_location('serialarm_terminal_review',Path(ROOT)/'src/serial_arm/core/app/serial_arm_terminal.py')
        module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
        with tempfile.TemporaryDirectory(prefix='serialarm-python-control-') as temporary:
            folder=Path(temporary);source=Path(ROOT)/'src/robot_supports/robots/dm_arm/description/config/core/gray.yaml'
            data=yaml.safe_load(source.read_text());names=data['model']['joint_names']
            data['model']['urdf_path']=str((source.parent/data['model']['urdf_path']).resolve())
            data['control']['runtime'].update(write_enabled=False,model_feedforward_mode='GRAVITY',tracking_gain_profile='SERVO_TRACKING')
            data['capability']['admittance']['reference_behavior']='NATIVE_IMPEDANCE'
            config=folder/'core.yaml';config.write_text(yaml.safe_dump(data,sort_keys=False))
            hardware=folder/'hardware.yaml';hardware.write_text(yaml.safe_dump({'joints':names,'positions':[0.]*len(names)}))
            session=arm._RobotSession();session.configure(str(config),PLUGIN,str(hardware))
            self.assertEqual(session.config.runtime.tracking_gain_profile,arm.JointGainProfile.SERVO_TRACKING)
            terminal=module.Terminal.__new__(module.Terminal)
            terminal.session=session;terminal.serial_arm_module=arm;terminal.cfg=session.config
            try:
                session.start()
                deadline=time.monotonic()+2
                while not session.snapshot.valid and time.monotonic()<deadline:time.sleep(.01)
                for choice,expected in [(1,arm.JointGainProfile.RIGID),(2,arm.JointGainProfile.COMPLIANT),
                                        (3,arm.JointGainProfile.DRAG),(4,arm.JointGainProfile.SERVO_TRACKING)]:
                    with patch('builtins.input',return_value=str(choice)),contextlib.redirect_stdout(io.StringIO()):
                        terminal.set_impedance()
                    deadline=time.monotonic()+2
                    while time.monotonic()<deadline:
                        snapshot=session.snapshot
                        if snapshot.valid and snapshot.cycle.gain_profile==expected:break
                        time.sleep(.01)
                    self.assertTrue(snapshot.valid)
                    self.assertEqual(snapshot.cycle.gain_profile,expected)
                    self.assertEqual(snapshot.cycle.reference_behavior,arm.ReferenceBehavior.NATIVE_IMPEDANCE)
            finally:session.stop()

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
            session.set_reference_behavior(arm.ReferenceBehavior.NATIVE_IMPEDANCE)
            self.assertEqual(session.config.capability.admittance.reference_behavior,arm.ReferenceBehavior.NATIVE_IMPEDANCE)
            limits=list(session.config.safety.limits.max_effort)
            for _ in range(2):
                try:
                    session.start()
                    deadline=time.monotonic()+2
                    while not session.snapshot.valid and session.running and time.monotonic()<deadline:
                        time.sleep(.01)
                    snapshot=session.snapshot
                    self.assertTrue(snapshot.valid,snapshot.last_error)
                    self.assertGreater(snapshot.sequence,0)
                    self.assertGreater(snapshot.monotonic_ns,0)
                    self.assertEqual(snapshot.cycle.reference_behavior,arm.ReferenceBehavior.NATIVE_IMPEDANCE)
                    self.assertEqual(len(snapshot.cycle.accepted_ref_pos),len(names))
                    with self.assertRaises(Exception):
                        session.set_reference_behavior(arm.ReferenceBehavior.MODEL_ADMITTANCE)
                    update=session.config.capability.admittance
                    update.controller.mass=[2.0]*len(names)
                    session.set_admittance_cfg(update)
                    self.assertEqual(list(session.config.capability.admittance.controller.mass),[2.0]*len(names))
                    update.enabled=True
                    with self.assertRaises(Exception):
                        session.set_admittance_cfg(update)
                    self.assertEqual(snapshot.robot_state,arm.RobotState.ACTIVE)
                    self.assertTrue(all(abs(value)<1e-9 for value in snapshot.cycle.joint_state.pos))
                    self.assertFalse(session.config.runtime.write_enabled)
                    goal=list(snapshot.cycle.joint_state.pos);goal[0]+=.01
                    facade=public.RobotSession.__new__(public.RobotSession);facade._session=session
                    motion_id=facade.move_to(goal,.1)
                    self.assertGreater(motion_id,0)
                    initial=session.snapshot.motion
                    self.assertEqual(initial.id,motion_id)
                    self.assertEqual(initial.state,'running')
                    self.assertEqual(list(initial.units),['rad']*len(names))
                    deadline=time.monotonic()+3
                    while time.monotonic()<deadline:
                        motion=session.snapshot.motion
                        if motion.state=='completed':break
                        time.sleep(.01)
                    self.assertEqual(motion.state,'completed')
                    self.assertTrue(motion.reference_complete)
                    goal[0]+=.1
                    next_id=session.move_to(goal,.1)
                    self.assertGreater(next_id,motion_id)
                    session.hold_current()
                    self.assertEqual(session.snapshot.motion.state,'interrupted')
                    session.move_to([0.0]*len(names),.1)
                    deadline=time.monotonic()+3
                    while time.monotonic()<deadline and session.snapshot.motion.state!='completed':time.sleep(.01)
                    self.assertEqual(session.snapshot.motion.state,'completed')
                    session.hold_current()
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
