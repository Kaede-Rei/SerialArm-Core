#!/usr/bin/env python3
"""Run the Robot MIT/CSP integration fixture against the standalone build."""
from pathlib import Path
import shlex
import subprocess
import tempfile

root = Path(__file__).resolve().parents[1]
build = root / 'build/unified/standalone-dm_arm-t1-p1-m0/core'
link = [value for value in shlex.split((build / 'CMakeFiles/serial_arm_terminal.dir/link.txt').read_text()) if value != '-DNDEBUG']
link[1:1] = ['-std=c++17', '-I' + str(root / 'src/serial_arm/core/include'),
             '-I' + str(root / 'src/serial_arm/core/third_lib/tl/include')]
with tempfile.TemporaryDirectory(prefix='serial-arm-robot-paths-') as temp:
    for fixture in ('robot_paths_test', 'interaction_dynamics_test', 'interaction_modes_test', 'position_safety_test'):
        command = link.copy()
        executable = Path(temp) / fixture
        command[next(i for i, value in enumerate(command) if value.endswith('serial_arm_terminal.cpp.o'))] = str(root / 'tools' / (fixture + '.cpp'))
        command[command.index('-o') + 1] = str(executable)
        subprocess.run(command, cwd=build, check=True)
        subprocess.run([str(executable)], check=True)
