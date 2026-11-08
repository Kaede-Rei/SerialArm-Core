"""Model-specific operator trust for compliant fault recovery

Compare the values used by dynamics, rather than file paths or YAML formatting.
No controller gains participate in this comparison
"""
from __future__ import annotations

import copy
from pathlib import Path
import xml.etree.ElementTree as ET

import yaml


def _model_values(path):
    try:
        root = ET.parse(path).getroot()
        def element(node):
            if node is None:
                return None
            attributes = {}
            for key, value in node.attrib.items():
                try:
                    attributes[key] = tuple(float(item) for item in value.split()) if key not in ('name', 'type', 'link', 'joint') else value
                except ValueError:
                    attributes[key] = value
            return (node.tag, attributes, tuple(sorted((element(child) for child in node), key=repr)))
        return {'links': {node.get('name'): element(node.find('inertial')) for node in root.findall('link')},
                'joints': {node.get('name'): element(node) for node in root.findall('joint')}}
    except (OSError, ET.ParseError):
        return {'unavailable': str(Path(path).resolve())}


def _correction_values(value, core_path, correction=None):
    if correction is None and value:
        try:
            correction = yaml.safe_load((Path(core_path).parent / value).read_text(encoding='utf-8'))
        except (OSError, ValueError, yaml.YAMLError):
            return {'unavailable': str((Path(core_path).parent / value).resolve())}
    if not value and correction is None:
        return None
    if not isinstance(correction, dict):
        return {'invalid': True}
    return {key: copy.deepcopy(correction.get(key)) for key in ('joint_names', 'first_moments', 'candidate_masses')}


def gravity_validation_context(core, core_path, hardware=None, plugin=None, *, correction=None):
    model = core.get('model') or {}
    result = {
        'model': {'parameters': _model_values(Path(core_path).parent / str(model.get('urdf_path') or '')),
                  **{key: copy.deepcopy(model.get(key)) for key in ('joint_names', 'base_frame', 'tool_frame')}},
        'mapping': copy.deepcopy(core.get('calibration') or {}),
        'gravity': copy.deepcopy(model.get('gravity')),
        'gravity_scale': copy.deepcopy(model.get('gravity_scale')),
        'gravity_correction': _correction_values(model.get('gravity_correction_path'), core_path, correction),
    }
    if hardware is not None:
        result['hardware_mapping'] = {'plugin': plugin,
            'actuators': copy.deepcopy((hardware.get('damiao') or {}).get('actuators'))}
    return result


def gravity_validation_changes(before, after):
    return [key for key in ('model', 'mapping', 'hardware_mapping', 'gravity', 'gravity_scale', 'gravity_correction')
            if key in before and before.get(key) != after.get(key)]


def gravity_trusted(core):
    return ((core.get('safety_policy') or {}).get('fault_recovery') or {}).get('gravity_model_validated') is True


def revoke_gravity_trust(text):
    """Preserve the current YAML layout through the existing narrow patcher"""
    from persistence import patch_text
    if not gravity_trusted(yaml.safe_load(text) or {}):
        return text
    return patch_text(text, {('safety_policy', 'fault_recovery', 'gravity_model_validated'): 'false'})
