#!/usr/bin/env python3
"""Build the actual Damiao read path against a hardware-free CAN receiver."""
import argparse
from pathlib import Path
import shlex
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--build', type=Path, default=ROOT / 'build/unified/standalone-dm_arm-t1-p1-m0')
    args = parser.parse_args()
    build = args.build.resolve()
    link = shlex.split((build / 'serial_arm_hardware_damiao/CMakeFiles/serial_arm_hardware_damiao.dir/link.txt').read_text())
    includes = [ROOT / 'src/serial_arm/core/include', ROOT / 'src/serial_arm/core/third_lib/tl/include',
                ROOT / 'src/robot_supports/hardware/damiao/include',
                ROOT / 'src/robot_supports/hardware/damiao/third_lib/damiao/include',
                ROOT / 'src/robot_supports/protocol/damiao_usb2can/include', Path('/usr/include/eigen3')]
    libraries = [item for item in link if Path(item).is_absolute() and '.so' in Path(item).name]
    with tempfile.TemporaryDirectory(prefix='serial-arm-damiao-runtime-') as directory:
        executable = Path(directory) / 'damiao_runtime_feedback_test'
        subprocess.run([link[0], '-std=c++17', '-O1', '-Wall', '-Wextra',
                        *[f'-I{path}' for path in includes],
                        str(ROOT / 'tools/damiao_runtime_feedback_test.cpp'),
                        str(ROOT / 'src/robot_supports/hardware/damiao/src/damiao_motor_bus.cpp'),
                        str(ROOT / 'src/serial_arm/core/src/core/joint_actuator_mapper.cpp'),
                        str(ROOT / 'src/serial_arm/core/src/core/ordered_motor_bus.cpp'),
                        *libraries, *[item for item in link if item.startswith('-Wl,-rpath,')],
                        '-pthread', '-o', str(executable)], check=True)
        subprocess.run([str(executable)], check=True, timeout=15)


if __name__ == '__main__':
    main()
