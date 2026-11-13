#!/usr/bin/env python3
"""Build/run adaptive gain behavior checks, using only existing project dependencies.

The algorithm checks use deterministic MIT plants and measured frequency-loop
evidence. The Robot check substitutes
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


def run(core_build: Path, demonstration: Path | None = None) -> None:
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
        for name in ('adaptive_gain_offline_test', 'frequency_response_gain_test', 'gain_measurement_center_test', 'gain_motion_guard_test',
                     'gain_trajectory_residual_test', 'gain_damping_progression_test', 'gain_taught_path_following_test',
                     'gain_taught_measurement_schedule_test', 'gain_taught_anchor_visit_test', 'gain_rigid_tuning_test', 'urdf_nominal_gain_design_test', 'gain_frequency_plan_test',
                     'gain_use_validation_test', 'gain_purpose_measurement_test', 'gain_drag_measurement_test', 'gain_taught_local_correction_test', 'joint_motion_tracker_test', 'calibration_stop_remap_test', 'gain_endpoint_continuation_test',
                     'gain_hardware_context_test', 'gain_candidate_admission_test', 'gain_complete_group_poll_test', 'gain_online_trajectory_test', 'gain_offline_optimization_test'):
            algorithm = temporary / name
            libraries = ['-lyaml-cpp'] if name == 'gain_hardware_context_test' else []
            sources = [str(ROOT / 'tools' / f'{name}.cpp')]
            if name == 'gain_offline_optimization_test':
                sources.append(str(ROOT / 'src/serial_arm/core/src/dynamics/gain_offline_optimization.cpp'))
            subprocess.run([compiler, *flags, *sources, *libraries, '-o', str(algorithm)], check=True)
            arguments = [str(algorithm)]
            if name == 'gain_taught_path_following_test' and demonstration is not None:
                arguments.append(str(demonstration.resolve()))
            subprocess.run(arguments, check=True)
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
    parser.add_argument('--demonstration', type=Path, help='Also replay a recorded trajectory.csv against the offline taught-path follower')
    args = parser.parse_args()
    try:
        run(args.core_build, args.demonstration)
    except (ValueError, OSError, StopIteration, subprocess.CalledProcessError) as error:
        parser.exit(1, f'Adaptive gain behavior check failed: {error}\n')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
