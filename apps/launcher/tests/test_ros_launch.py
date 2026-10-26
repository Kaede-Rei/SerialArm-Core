import importlib.util
from pathlib import Path
import tempfile
import unittest
from unittest import mock
from types import SimpleNamespace

ROOT=Path(__file__).resolve().parents[3]
try:
    import launch
    import xacro
    ROS=True
except ImportError:
    ROS=False

@unittest.skipUnless(ROS,'ROS launch dependencies are unavailable')
class LaunchContracts(unittest.TestCase):
    def test_spaces_and_namespaced_manager(self):
        path=ROOT/'src/serial_arm/bringup/ros2_control/launch/hardware.launch.py'
        spec=importlib.util.spec_from_file_location('serialarm_hardware_launch_contract',path)
        module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
        with tempfile.TemporaryDirectory(prefix='serialarm model space ') as temporary:
            folder=Path(temporary)
            model=folder/'robot model.xacro'
            model.write_text('<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="test"><xacro:arg name="config_file" default=""/><link name="base"/></robot>')
            controllers=folder/'controllers config.yaml'
            controllers.write_text('controller_manager:\n  ros__parameters:\n    update_rate: 200\n')
            profile={'core_config_path':str(folder/'core config.yaml'),'hardware_plugin':'mock',
                'hardware_config_path':str(folder/'hardware config.yaml'),'controllers_path':str(controllers),
                'ros2_control_xacro_path':str(model),'controller_names':['joint_state_broadcaster']}
            context=SimpleNamespace(launch_configurations={})
            context.launch_configurations.update({'robot_profile':'test','controller_manager_name':'/arm/custom_manager'})
            with mock.patch.object(module,'load_profile',return_value=profile),mock.patch.object(module,'Node',side_effect=lambda **values:values):
                nodes=module.resolve_profile(context)
            manager=next(node for node in nodes if node['executable']=='ros2_control_node')
            self.assertEqual(manager['name'],'custom_manager');self.assertEqual(manager['namespace'],'/arm')
            self.assertIn({'update_rate':200},manager['parameters'])
            spawner=next(node for node in nodes if node['executable']=='spawner')
            self.assertEqual(spawner['namespace'],'/arm')
            self.assertIn('/arm/custom_manager',spawner['arguments'])
