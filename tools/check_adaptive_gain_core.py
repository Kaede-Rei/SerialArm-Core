#!/usr/bin/env python3
"""Build/run adaptive gain behavior checks, using only existing project dependencies.

The algorithm check uses a deterministic MIT plant. The Robot check substitutes
an in-memory MotorBus and never loads a hardware plugin. All new build outputs
live in a temporary directory; --core-build supplies already-built libraries.
"""
from __future__ import annotations

import argparse
from pathlib import Path
import shlex
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]


def run(core_build: Path) -> None:
    core_build = core_build.resolve()
    link_file = core_build / 'CMakeFiles/serial_arm_terminal.dir/link.txt'
    if not link_file.is_file():
        raise ValueError('--core-build must be the existing standalone Core build directory with serial_arm_terminal link.txt')
    includes = ['-I', str(ROOT / 'src/serial_arm/core/include'),
                '-I', str(ROOT / 'src/serial_arm/core/third_lib/tl/include'),
                '-I', '/usr/include/eigen3']
    flags = ['-std=c++17', '-O1', '-Wall', '-Wextra', '-Wpedantic', *includes]
    link = shlex.split(link_file.read_text())
    compiler = link[0]
    with tempfile.TemporaryDirectory(prefix='serial-arm-gain-tests-') as folder:
        temporary = Path(folder)
        algorithm = temporary / 'adaptive_gain_offline_test'
        subprocess.run([compiler, *flags, str(ROOT / 'tools/adaptive_gain_offline_test.cpp'), '-o', str(algorithm)], check=True)
        subprocess.run([str(algorithm)], check=True)
        robot_object = temporary / 'current_robot.o'
        test_object = temporary / 'robot_test.o'
        executable = temporary / 'adaptive_gain_robot_test'
        for source, output in [(ROOT / 'src/serial_arm/core/src/robot.cpp', robot_object),
                               (ROOT / 'tools/adaptive_gain_robot_test.cpp', test_object)]:
            subprocess.run([compiler, *flags, '-c', str(source), '-o', str(output)], check=True)
        main_index = next(i for i, argument in enumerate(link) if argument.endswith('serial_arm_terminal.cpp.o'))
        link[main_index] = str(test_object)
        link[link.index('-o') + 1] = str(executable)
        # Supply the current Robot implementation before the existing static
        # library; the latter provides only its remaining referenced members.
        link.insert(main_index + 1, str(robot_object))
        subprocess.run(link, cwd=core_build, check=True)
        subprocess.run([str(executable)], check=True)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--core-build', type=Path, default=ROOT / 'build/unified/standalone-dm_arm-t1-p1-m0/core')
    args = parser.parse_args()
    try:
        run(args.core_build)
    except (ValueError, OSError, StopIteration, subprocess.CalledProcessError) as error:
        parser.exit(1, f'Adaptive gain behavior check failed: {error}\n')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
