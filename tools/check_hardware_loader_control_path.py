#!/usr/bin/env python3
"""Check the actual dynamic MotorBus wrapper with a transport-free plugin."""
import argparse
from pathlib import Path
import shlex
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--core-build', type=Path,
                        default=ROOT / 'build/unified/standalone-dm_arm-t1-p1-m0/core')
    args = parser.parse_args()
    build = args.core_build.resolve()
    link = shlex.split((build / 'CMakeFiles/serial_arm_terminal.dir/link.txt').read_text())
    source = ROOT / 'tools/hardware_loader_control_path_test.cpp'
    includes = ['-I' + str(ROOT / 'src/serial_arm/core/include'),
                '-I' + str(ROOT / 'src/serial_arm/core/third_lib/tl/include')]
    with tempfile.TemporaryDirectory(prefix='serialarm-loader-path-') as directory:
        directory = Path(directory)
        plugin = directory / 'control_path_plugin.so'
        executable = directory / 'control_path_test'
        config = directory / 'hardware.yaml'
        config.write_text('{}\n')
        subprocess.run([link[0], '-std=c++17', '-O1', '-shared', '-fPIC', *includes,
                        '-DSERIAL_ARM_CONTROL_PATH_TEST_PLUGIN', str(source), '-o', str(plugin)], check=True)
        index = next(i for i, value in enumerate(link) if value.endswith('serial_arm_terminal.cpp.o'))
        link[index:index + 1] = [str(source), str(ROOT / 'src/serial_arm/core/src/hardware/hardware_loader.cpp')]
        link[link.index('-o') + 1] = str(executable)
        link[1:1] = ['-std=c++17', '-O1', *includes]
        subprocess.run(link, cwd=build, check=True)
        subprocess.run([str(executable), str(plugin), str(config)], check=True)


if __name__ == '__main__':
    main()
