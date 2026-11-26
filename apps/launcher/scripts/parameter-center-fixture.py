#!/usr/bin/env python3
"""Temporary-file persistence fixture for the production parameter-center UI."""
from pathlib import Path
import json
import sys
import tempfile
import yaml
ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / 'apps/launcher/backend'))
from profile_library import ProfileLibrary
home = Path(sys.argv[1]).resolve()
if not home.is_relative_to(Path(tempfile.gettempdir()).resolve()):
    raise ValueError('Parameter fixture writes only inside the temporary directory')
home.mkdir(parents=True, exist_ok=True)
params = json.load(sys.stdin)
core_path, hardware_path, profile_path = [home / name for name in ('core.yaml', 'hardware.yaml', 'profiles.yaml')]
if sys.argv[2] == 'initialize':
    editor = params['editor']
    editor['core']['model']['urdf_path'] = editor['urdf_path']
    editor['core']['model']['gravity_model_validated'] = False
    core_path.write_text(yaml.safe_dump(editor['core'], sort_keys=False))
    hardware_path.write_text(yaml.safe_dump(editor['hardware'], sort_keys=False))
    profile_path.write_text(yaml.safe_dump({'profiles': {'review': editor['profile_entry']}}, sort_keys=False))
core = yaml.safe_load(core_path.read_text())
entry = yaml.safe_load(profile_path.read_text())['profiles']['review']
class Inspector:
    def inspect(self, config):
        return {'profile': 'review', 'profile_file': str(profile_path), 'hardware_plugin': entry['hardware']['plugin'],
                'resources': {'core': str(core_path), 'hardware': str(hardware_path), 'description': core['model']['urdf_path']}}
    def validate_model_config(self, *args): return {'valid': True}
    def profiles(self, config): return {'profile_file': str(profile_path)}
library = ProfileLibrary(home, Inspector())
result = library.profile_editor_save({}, params['payload']) if sys.argv[2] == 'profile_editor_save' else library.profile_editor_load({})
print(json.dumps(result))
