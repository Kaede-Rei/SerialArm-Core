#!/usr/bin/env python3
"""Run the Robot MIT/CSP integration fixture against the standalone build."""
from pathlib import Path
import shlex
import subprocess
import tempfile

root = Path(__file__).resolve().parents[1]
build = root / 'build/unified/standalone-dm_arm-t1-p1-m0/core'
link = shlex.split((build / 'CMakeFiles/serial_arm_terminal.dir/link.txt').read_text())
object_index = next(i for i, value in enumerate(link) if value.endswith('serial_arm_terminal.cpp.o'))
with tempfile.TemporaryDirectory(prefix='serial-arm-robot-paths-') as temp:
    executable = Path(temp) / 'robot_paths_test'
    link[object_index] = str(root / 'tools/robot_paths_test.cpp')
    link[link.index('-o') + 1] = str(executable)
    link[1:1] = ['-std=c++17', '-I' + str(root / 'src/serial_arm/core/include'),
                 '-I' + str(root / 'src/serial_arm/core/third_lib/tl/include')]
    subprocess.run(link, cwd=build, check=True)
    subprocess.run([str(executable)], check=True)
