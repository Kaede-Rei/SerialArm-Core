import os
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[3]

class LaunchTests(unittest.TestCase):
    def test_launch_sources_environment_and_passes_app_as_literal_path(self):
        with tempfile.TemporaryDirectory(prefix='launcher path ') as temp:
            root = Path(temp)
            shutil.copy(ROOT / 'launch.sh', root / 'launch.sh')
            state = root / '.install'
            (state / 'gui-venv/bin').mkdir(parents=True)
            (state / 'setup.bash').write_text('export LAUNCH_TEST_SETUP=ready\n')
            python = state / 'gui-venv/bin/python'
            python.write_text('#!/bin/sh\nexit 0\n')
            python.chmod(0o755)
            electron = root / 'apps/launcher/node_modules/electron/dist/electron'
            electron.parent.mkdir(parents=True)
            electron.write_text('#!/bin/sh\ntest "$LAUNCH_TEST_SETUP" = ready || exit 8\ntest -x "$SERIAL_ARM_LAUNCHER_PYTHON" || exit 9\nprintf "%s" "$1"\n')
            electron.chmod(0o755)
            three = root / 'apps/launcher/renderer/vendor/three/build/three.module.js'
            three.parent.mkdir(parents=True)
            three.write_text('export {};\n')
            proc = subprocess.run(['bash', str(root / 'launch.sh')], text=True, capture_output=True)
            self.assertEqual(proc.returncode, 0, proc.stderr)
            self.assertEqual(proc.stdout, str(root / 'apps/launcher'))

    def test_missing_gui_has_install_guidance(self):
        with tempfile.TemporaryDirectory() as temp:
            script = Path(temp) / 'launch.sh'
            shutil.copy(ROOT / 'launch.sh', script)
            proc = subprocess.run(['bash', str(script)], text=True, capture_output=True)
            self.assertEqual(proc.returncode, 1)
            self.assertIn('--gui-only', proc.stderr)

if __name__ == '__main__': unittest.main()
