#!/usr/bin/env python3
"""Build/run hardware-free PD configuration, capability and protocol checks."""
import argparse
from pathlib import Path
import shlex
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--core-build', type=Path, default=ROOT / 'build/unified/standalone-dm_arm-t1-p1-m0/core')
    args = parser.parse_args()
    build = args.core_build.resolve()
    link = shlex.split((build / 'CMakeFiles/serial_arm_terminal.dir/link.txt').read_text())
    with tempfile.TemporaryDirectory(prefix='serial-arm-pd-config-') as directory:
        temporary = Path(directory)
        obj = temporary / 'config_pd_limits_test.o'
        executable = temporary / 'config_pd_limits_test'
        includes = [ROOT / 'src/serial_arm/core/include', ROOT / 'src/serial_arm/core/third_lib/tl/include',
                    ROOT / 'src/robot_supports/hardware/damiao/third_lib/damiao/include', Path('/usr/include/eigen3')]
        subprocess.run([link[0], '-std=c++17', '-O1', '-Wall', '-Wextra',
                        *[f'-I{path}' for path in includes], '-c',
                        str(ROOT / 'tools/config_pd_limits_test.cpp'), '-o', str(obj)], check=True)
        index = next(i for i, value in enumerate(link) if value.endswith('serial_arm_terminal.cpp.o'))
        link[index] = str(obj)
        link[link.index('-o') + 1] = str(executable)
        subprocess.run(link, cwd=build, check=True)
        subprocess.run([str(executable), str(ROOT), str(temporary)], check=True)


if __name__ == '__main__':
    main()
