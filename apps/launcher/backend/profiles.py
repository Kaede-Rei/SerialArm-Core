"""Read-only configuration inspection; never instantiate hardware or open buses."""
import glob
import json
import hashlib
import os
from pathlib import Path
import shutil
import math
import subprocess
import xml.etree.ElementTree as ET
import yaml

MODES = ('terminal', 'hardware', 'moveit')
ROS_LAUNCH = {'hardware': 'hardware.launch.py', 'moveit': 'moveit.launch.py'}


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
        args = ['ros2', 'launch', 'serial_arm_ros2_control', ROS_LAUNCH[mode], 'robot_profile:=' + c['profile']]
        keys = ['profile_file', 'resource_paths', 'serial_port', 'baudrate', 'bus']
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

    def machine_terminal(self):
        candidates = [
            os.environ.get('SERIAL_ARM_TERMINAL', ''),
            # Use the local checked-out project's binary before an older
            # global PATH installation. The explicit env override still wins.
            str(self.root / '.install/standalone/bin/serial_arm_terminal'),
            str(self.root / 'install/standalone/bin/serial_arm_terminal'),
            str(self.root / 'build/serial_arm_core/serial_arm_terminal'),
            shutil.which('serial_arm_terminal') or '',
        ]
        for value in candidates:
            if value and Path(value).is_file() and os.access(value, os.X_OK): return str(Path(value).resolve())
        return ''

    def model_calibrator(self):
        candidates = [
            os.environ.get('SERIAL_ARM_MODEL_CALIBRATOR', ''),
            shutil.which('serial_arm_model_calibrator') or '',
            str(self.root / '.install/standalone/bin/serial_arm_model_calibrator'),
            str(self.root / 'install/standalone/bin/serial_arm_model_calibrator'),
            str(self.root / 'build/serial_arm_core/serial_arm_model_calibrator'),
        ]
        for value in candidates:
            if value and Path(value).is_file() and os.access(value, os.X_OK): return str(Path(value).resolve())
        return ''

    def model_probe(self):
        candidates = [
            os.environ.get('SERIAL_ARM_MODEL_PROBE', ''),
            shutil.which('serial_arm_model_probe') or '',
            str(self.root / '.install/standalone/bin/serial_arm_model_probe'),
            str(self.root / 'install/standalone/bin/serial_arm_model_probe'),
            str(self.root / 'build/serial_arm_core/serial_arm_model_probe'),
        ]
        for value in candidates:
            if value and Path(value).is_file() and os.access(value, os.X_OK): return str(Path(value).resolve())
        return ''

    @staticmethod
    def _numbers(value, count, default):
        if not value: return list(default)
        parts = value.split()
        if len(parts) != count: raise ValueError(f'expected {count} numeric values, got {value!r}')
        result = [float(item) for item in parts]
        if any(not math.isfinite(item) for item in result): raise ValueError('non-finite numeric value')
        return result

    @staticmethod
    def _origin(node):
        origin = node.find('origin') if node is not None else None
        return {'xyz': Inspector._numbers(origin.get('xyz') if origin is not None else '', 3, (0, 0, 0)),
                'rpy': Inspector._numbers(origin.get('rpy') if origin is not None else '', 3, (0, 0, 0))}

    def _resolve_mesh(self, value, urdf, roots):
        value = text(value)
        if value.startswith('package://'):
            tail = value[len('package://'):]
            package, sep, relative = tail.partition('/')
            if not sep: raise ValueError('invalid package mesh URI: ' + value)
            return str((self.share(package, roots) / relative).resolve())
        path = Path(value).expanduser()
        if not path.is_absolute(): path = Path(urdf).parent / path
        return str(path.resolve())

    def _geometry(self, geometry, urdf, roots, mesh_files):
        if geometry is None: raise ValueError('missing geometry')
        box = geometry.find('box')
        if box is not None: return {'type': 'box', 'size': self._numbers(box.get('size'), 3, (1, 1, 1))}
        cylinder = geometry.find('cylinder')
        if cylinder is not None:
            return {'type': 'cylinder', 'radius': float(cylinder.get('radius')), 'length': float(cylinder.get('length'))}
        sphere = geometry.find('sphere')
        if sphere is not None: return {'type': 'sphere', 'radius': float(sphere.get('radius'))}
        mesh = geometry.find('mesh')
        if mesh is not None:
            path = self._resolve_mesh(mesh.get('filename'), urdf, roots)
            if not Path(path).is_file(): raise ValueError('mesh file not found: ' + path)
            mesh_files.add(path)
            return {'type': 'mesh', 'path': path, 'source': mesh.get('filename'),
                    'scale': self._numbers(mesh.get('scale') or '', 3, (1, 1, 1))}
        raise ValueError('unsupported URDF geometry')

    @staticmethod
    def _inertia_valid(matrix, mass):
        if not math.isfinite(mass) or mass <= 0: return False, 'mass must be finite and greater than zero'
        a, b, c = matrix[0]
        _, d, e = matrix[1]
        _, _, f = matrix[2]
        det2 = a * d - b * b
        det3 = a * (d * f - e * e) - b * (b * f - e * c) + c * (b * e - d * c)
        if not all(math.isfinite(x) for row in matrix for x in row): return False, 'inertia contains non-finite values'
        if a <= 0 or det2 <= 0 or det3 <= 0: return False, 'inertia matrix is not positive definite'
        return True, ''

    def _parse_urdf(self, urdf, roots):
        root = ET.parse(urdf).getroot()
        if root.tag != 'robot': raise ValueError('URDF root must be robot')
        links, joints, mesh_files = [], [], set()
        for link in root.findall('link'):
            item = {'name': text(link.get('name')), 'visuals': [], 'collisions': [], 'inertial': None}
            inertial = link.find('inertial')
            if inertial is not None:
                mass_node, inertia_node = inertial.find('mass'), inertial.find('inertia')
                if mass_node is None or inertia_node is None:
                    item['inertial'] = {'valid': False, 'reason': 'inertial requires mass and inertia'}
                else:
                    mass = float(mass_node.get('value'))
                    ixx, ixy, ixz = (float(inertia_node.get(k)) for k in ('ixx', 'ixy', 'ixz'))
                    iyy, iyz, izz = (float(inertia_node.get(k)) for k in ('iyy', 'iyz', 'izz'))
                    matrix = [[ixx, ixy, ixz], [ixy, iyy, iyz], [ixz, iyz, izz]]
                    valid, reason = self._inertia_valid(matrix, mass)
                    item['inertial'] = {'origin': self._origin(inertial), 'mass': mass, 'inertia': matrix,
                                        'valid': valid, 'reason': reason}
            for kind in ('visual', 'collision'):
                target = item[kind + 's']
                for index, node in enumerate(link.findall(kind)):
                    try:
                        entry = {'name': node.get('name') or f'{kind}-{index}', 'origin': self._origin(node),
                                 'geometry': self._geometry(node.find('geometry'), urdf, roots, mesh_files)}
                        material = node.find('material')
                        color = material.find('color') if material is not None else None
                        if color is not None and color.get('rgba'):
                            entry['rgba'] = self._numbers(color.get('rgba'), 4, (0.7, 0.7, 0.7, 1))
                        target.append(entry)
                    except ValueError as error:
                        target.append({'name': node.get('name') or f'{kind}-{index}', 'origin': self._origin(node),
                                       'error': str(error)})
            links.append(item)
        for joint in root.findall('joint'):
            parent, child, axis, limit, mimic = joint.find('parent'), joint.find('child'), joint.find('axis'), joint.find('limit'), joint.find('mimic')
            entry = {'name': text(joint.get('name')), 'type': text(joint.get('type')), 'origin': self._origin(joint),
                     'parent': text(parent.get('link')) if parent is not None else '',
                     'child': text(child.get('link')) if child is not None else '',
                     'axis': self._numbers(axis.get('xyz') if axis is not None else '', 3, (1, 0, 0))}
            if limit is not None:
                entry['limit'] = {key: float(limit.get(key)) for key in ('lower', 'upper', 'effort', 'velocity') if limit.get(key) is not None}
            if mimic is not None:
                entry['mimic'] = {'joint': text(mimic.get('joint')), 'multiplier': float(mimic.get('multiplier', '1')),
                                  'offset': float(mimic.get('offset', '0'))}
            joints.append(entry)
        return {'robot_name': root.get('name') or '', 'links': links, 'joints': joints, 'mesh_files': sorted(mesh_files)}

    @staticmethod
    def validate_positions(positions):
        if positions is None: return
        if not isinstance(positions, list) or any(not isinstance(v, (int, float)) or not math.isfinite(v) for v in positions):
            raise ValueError('positions must be a finite numeric array')

    def model_payload(self, config, native, info=None):
        c = config_values(config)
        info = info or self.inspect(c)
        core = info['resources'].get('core')
        if not core or not Path(core).is_file(): raise ValueError('Core model configuration is unavailable')
        if not isinstance(native, dict) or not native.get('ok'): raise ValueError((native or {}).get('error') or 'model probe failed')
        roots = self.roots(c, info['profile_file'])
        urdf = native['config']['urdf_path']
        raw = self._parse_urdf(urdf, roots)
        controlled = set(native['config']['joint_names'])
        for joint in raw['joints']:
            joint['controlled'] = joint['name'] in controlled
            if joint['controlled']:
                joint['preview_status'] = 'controlled'
            elif joint['type'] == 'fixed':
                joint['preview_status'] = 'fixed'
            elif joint.get('mimic'):
                joint['preview_status'] = 'mimic-readonly'
            else:
                joint['preview_status'] = 'noncontrolled-locked'
        description_path = info['resources'].get('description', '')
        description_mismatch = bool(description_path and Path(description_path).resolve() != Path(urdf).resolve())
        return {'profile': info['profile'], 'core_config': core, 'urdf': urdf, 'description_urdf': description_path,
                'description_mismatch': description_mismatch, 'native': native, 'model': raw,
                'mesh_files': raw['mesh_files'], 'offline': True}

    def model(self, config, positions=None):
        c = config_values(config)
        info = self.inspect(c)
        core = info['resources'].get('core')
        if not core or not Path(core).is_file(): raise ValueError('Core model configuration is unavailable')
        probe = self.model_probe()
        if not probe: raise ValueError('serial_arm_model_probe is not installed')
        self.validate_positions(positions)
        argv = [probe, '--config', core]
        if positions is not None:
            argv += ['--positions', ','.join(format(float(v), '.17g') for v in positions)]
        result = subprocess.run(argv, capture_output=True, text=True, timeout=8, check=False)
        try: native = json.loads(result.stdout.strip())
        except json.JSONDecodeError: raise ValueError('model probe returned invalid JSON')
        if result.returncode != 0 or not native.get('ok'): raise ValueError(native.get('error') or result.stderr.strip() or 'model probe failed')
        return self.model_payload(c, native, info)

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
                     'model': okay.get('core', False) and bool(self.model_probe()),
                     'workbench': okay.get('core', False) and okay.get('hardware', False) and bool(self.machine_terminal()),
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
