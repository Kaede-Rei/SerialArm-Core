"""Read-only configuration inspection; never instantiate hardware or open buses."""
import glob
import json
import hashlib
import os
from pathlib import Path
import shutil
import xml.etree.ElementTree as ET
import yaml

MODES = ('model', 'terminal', 'hardware', 'moveit')


def text(value):
    if value is None: return ''
    if not isinstance(value, str) or '\x00' in value or '\n' in value or len(value) > 8192:
        raise ValueError('invalid text parameter')
    return value.strip()


def config_values(config):
    if not isinstance(config, dict): raise ValueError('configuration must be an object')
    result = {key: text(config.get(key, '')) for key in
              ('profile', 'profile_file', 'serial_port', 'baudrate', 'bus', 'resource_paths')}
    if result['baudrate'] and (not result['baudrate'].isdigit() or int(result['baudrate']) < 1):
        raise ValueError('baudrate must be a positive integer')
    return result


def read_yaml(path):
    path = Path(path).expanduser().resolve()
    if path.stat().st_size > 2 * 1024 * 1024: raise ValueError('YAML exceeds 2 MiB')
    data = yaml.safe_load(path.read_text())
    if not isinstance(data, dict): raise ValueError(f'YAML must contain a mapping: {path}')
    return data


def command_for(mode, config):
    if mode not in MODES: raise ValueError('unknown run mode')
    c = config_values(config)
    if not c['profile']: raise ValueError('select a robot profile')
    env = {}
    if c['resource_paths']:
        env['SERIAL_ARM_RESOURCE_PATH'] = c['resource_paths'] + (os.pathsep + os.environ['SERIAL_ARM_RESOURCE_PATH'] if os.environ.get('SERIAL_ARM_RESOURCE_PATH') else '')
    if mode == 'terminal':
        args = ['serial_arm_terminal', '--robot-profile', c['profile']]
        for key, flag in [('profile_file', '--profile-file'), ('serial_port', '--serial-port'),
                          ('baudrate', '--baudrate'), ('bus', '--bus')]:
            if c[key]: args += [flag, c[key]]
    else:
        args = ['ros2', 'launch', 'serial_arm_ros2_control', mode + '.launch.py', 'robot_profile:=' + c['profile']]
        keys = ['profile_file', 'resource_paths']
        if mode != 'model': keys += ['serial_port', 'baudrate', 'bus']
        args += [key + ':=' + c[key] for key in keys if c[key]]
    return args, env


class Inspector:
    def __init__(self, root):
        self.root = Path(root).resolve()

    def manifest(self):
        path = self.root / '.install/manifest.json'
        if not path.exists(): return {}
        value = json.loads(path.read_text())
        if not isinstance(value, dict): raise ValueError('invalid installation manifest')
        return value

    def status(self):
        m = self.manifest()
        attempt = self.root / '.install/last_attempt.json'
        return {'installed': m.get('status') == 'verified',
                'components': m.get('installed_components', []), 'gui': bool(m.get('gui')),
                'prefix': m.get('install_prefix', ''), 'ros_distro': m.get('ros_distro') if m.get('ros2') else '',
                'terminal': shutil.which('serial_arm_terminal') or '', 'ros2': shutil.which('ros2') or '',
                'last_attempt': json.loads(attempt.read_text()) if attempt.exists() else {},
                'devices': sorted(set(glob.glob('/dev/ttyACM*') + glob.glob('/dev/ttyUSB*') + glob.glob('/dev/serial/by-id/*')))}

    def profiles(self, config):
        c = config_values(config)
        prefix = self.manifest().get('install_prefix', '')
        installed = Path(prefix) / 'share/serial_arm_robot_profiles/config/robot_profiles.yaml' if prefix else None
        path = Path(c['profile_file']).expanduser() if c['profile_file'] else (
            installed if installed and installed.is_file() else self.root / 'src/robot_supports/profiles/config/robot_profiles.yaml')
        data = read_yaml(path).get('profiles')
        if not isinstance(data, dict): raise ValueError('Profile file must contain a profiles mapping')
        if any(not isinstance(k, str) or not isinstance(v, dict) for k, v in data.items()):
            raise ValueError('Profile names must be strings and profiles must be mappings')
        return {'profile_file': str(path.resolve()),
                'profiles': [{'name': name, 'data': value} for name, value in sorted(data.items())]}

    def roots(self, config, file):
        values = config['resource_paths'].split(os.pathsep) if config['resource_paths'] else []
        m = self.manifest()
        values += [m.get('install_prefix', '')]
        values += os.environ.get('SERIAL_ARM_RESOURCE_PATH', '').split(os.pathsep)
        values += os.environ.get('AMENT_PREFIX_PATH', '').split(os.pathsep)
        values += [str(p) for p in Path(file).parents][:3]
        values += [str(self.root / 'src')]
        return list(dict.fromkeys(str(Path(v).expanduser().resolve()) for v in values if v))

    def share(self, package, roots):
        package = text(package)
        if not package or '/' in package or package in ('.', '..'): raise ValueError('invalid resource package')
        for value in roots:
            root = Path(value)
            for candidate in (root / 'share' / package, root / package, root / 'install' / package / 'share' / package, root / 'src' / package):
                if candidate.is_dir(): return candidate
            # Match the Core resolver's source-workspace layout, with bounded
            # traversal and excluded dependency / output trees
            visited = 0
            for folder, dirs, files in os.walk(root):
                dirs[:] = [d for d in dirs if d not in ('node_modules', '.git', 'build', 'log', '.install', '__pycache__')]
                visited += 1
                if visited > 12000: break
                if 'package.xml' in files:
                    try:
                        if ET.parse(Path(folder) / 'package.xml').findtext('name') == package: return Path(folder)
                    except (OSError, ET.ParseError): pass
        raise ValueError(f'resource package not found: {package}')

    def inspect(self, config):
        c = config_values(config)
        listing = self.profiles(c)
        profile = next((p['data'] for p in listing['profiles'] if p['name'] == c['profile']), None)
        if profile is None: raise ValueError('robot profile not found: ' + c['profile'])
        roots = self.roots(c, listing['profile_file'])
        resources, checks = {}, []
        specs = [('core', 'core', 'package', 'config'), ('hardware', 'hardware', 'config_package', 'config'),
                 ('description', 'description', 'package', 'xacro' if (profile.get('description') or {}).get('xacro') else 'urdf'),
                 ('ros2_control', 'description', 'package', 'ros2_control_xacro'),
                 ('controllers', 'controllers', 'package', 'config')]
        for key, section, pkgkey, filekey in specs:
            try:
                data = profile.get(section, {})
                if not isinstance(data, dict): raise ValueError(f'invalid {section} mapping')
                relative = text(data.get(filekey))
                if not relative: raise ValueError(f'missing {section}.{filekey}')
                path = (self.share(data.get(pkgkey), roots) / relative).resolve()
                resources[key] = str(path)
                if not path.is_file(): raise ValueError('file not found: ' + str(path))
                checks.append({'name': key, 'ok': True, 'detail': str(path)})
            except (ValueError, OSError) as error:
                checks.append({'name': key, 'ok': False, 'detail': str(error)})
        core = read_yaml(resources['core']) if any(x['name'] == 'core' and x['ok'] for x in checks) else {}
        hw = read_yaml(resources['hardware']) if any(x['name'] == 'hardware' and x['ok'] for x in checks) else {}
        write_enabled = (core.get('control', {}).get('runtime', {}) or {}).get('write_enabled')
        if not isinstance(write_enabled, bool): write_enabled = None
        buses = hw.get('buses', {})
        if not isinstance(buses, dict): raise ValueError('hardware buses must be a mapping')
        bus_name = c['bus'] or (hw.get('damiao') or {}).get('bus', '')
        if c['bus'] and c['bus'] not in buses: raise ValueError('selected bus is not in hardware configuration')
        if not bus_name and len(buses) == 1: bus_name = next(iter(buses))
        selected = buses.get(bus_name, {})
        default_port = selected.get('device', '')
        devices = [text(v.get('device')) for v in buses.values() if isinstance(v, dict) and v.get('device')]
        if c['serial_port']:
            if not bus_name and len(buses) > 1: raise ValueError('choose a bus before overriding a multi-bus serial port')
            devices = [d for d in devices if d != default_port] + [c['serial_port']]
        devices = list(dict.fromkeys(d for d in devices if d.startswith('/')))
        for device in devices:
            checks.append({'name': 'device', 'ok': os.path.exists(device) and os.access(device, os.R_OK | os.W_OK), 'detail': device})
        m = self.manifest()
        installed = m.get('status') == 'verified' and Path(m.get('backend_setup', '/missing')).is_file()
        okay = {x['name']: x['ok'] for x in checks if x['name'] != 'device'}
        components = m.get('installed_components', [])
        def ament_ready(package):
            try:
                from ament_index_python.packages import get_package_share_directory
                return Path(get_package_share_directory(package)).is_dir()
            except Exception:
                prefixes = [m.get('install_prefix', ''), *os.environ.get('AMENT_PREFIX_PATH', '').split(os.pathsep)]
                return any((Path(p) / 'share/ament_index/resource_index/packages' / package).is_file() for p in prefixes if p)
        generic_ros = ament_ready('serial_arm_ros2_control') and installed and 'ros2' in components and 'python' in components and bool(shutil.which('ros2'))
        available = {'terminal': installed and 'terminal' in components and bool(shutil.which('serial_arm_terminal')) and okay.get('core', False) and okay.get('hardware', False),
                     'model': generic_ros and okay.get('description', False) and ament_ready((profile.get('description') or {}).get('package', '')),
                     'hardware': generic_ros and all(okay.get(k, False) for k in ('core', 'hardware', 'ros2_control', 'controllers')) and all(ament_ready((profile.get(k) or {}).get('package', '')) for k in ('description', 'controllers'))}
        moveit_package = (profile.get('moveit') or {}).get('package', '')
        try: moveit_ready = bool(moveit_package and (self.share(moveit_package, roots) / 'package.xml').is_file())
        except ValueError: moveit_ready = False
        available['moveit'] = available['hardware'] and moveit_ready and ament_ready(moveit_package)
        fingerprint = hashlib.sha256((json.dumps(profile, sort_keys=True) + json.dumps(c, sort_keys=True) + ''.join(Path(v).read_text(errors='replace') for k, v in resources.items() if k in ('core', 'hardware') and Path(v).is_file())).encode()).hexdigest()
        return {'fingerprint': fingerprint, 'profile': c['profile'], 'profile_file': listing['profile_file'], 'resources': resources,
                'checks': checks, 'available': available, 'write_enabled': write_enabled,
                'devices': devices, 'default_port': default_port, 'default_baudrate': selected.get('baudrate', ''),
                'bus': bus_name, 'hardware_plugin': (profile.get('hardware') or {}).get('plugin', ''),
                'controllers': (profile.get('controllers') or {}).get('spawn', ['joint_state_broadcaster', 'joint_trajectory_controller']),
                'moveit_package': moveit_package, 'resource_paths': c['resource_paths']}
