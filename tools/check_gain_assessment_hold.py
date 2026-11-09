#!/usr/bin/env python3
"""Exercise slow post-trajectory recording through the real offline native IPC.

The fixture never supplies a hardware plugin and forces write_enabled=false.
A temporary FIFO stalls the actual CSV writer beyond CMD_TIMEOUT, while the
control thread must already own a stationary HOLD reference.
"""
import argparse
import json
import os
from pathlib import Path
import select
import tempfile
import threading
import time

from check_adaptive_gain_flow import Native, ROOT, prepare_config, synthetic_record


def run(terminal):
    with tempfile.TemporaryDirectory(prefix='serialarm-assessment-hold-') as folder:
        directory = Path(folder)
        config, names = prepare_config(directory)
        storage = directory / 'storage'
        storage.mkdir()
        native = Native(terminal, config, storage)
        reader_fd = None
        drain_thread = None
        draining = threading.Event()
        consumed = [0]

        def drain():
            while draining.is_set():
                if select.select([reader_fd], [], [], .05)[0]:
                    block = os.read(reader_fd, 65536)
                    consumed[0] += len(block)
                    if not block:
                        time.sleep(.005)

        try:
            native.request('activate')
            native.ready()
            synthetic_record(native, directory / 'source', names)
            native.ready()
            task = native.request('model_calibration_import_trajectory', {
                'directory': str(directory / 'source'), 'pose_budget': 5})
            native.ready()
            native.request('model_calibration_start', {
                'task_id': task['task_id'], 'workflow': 'gain', 'gain_mode': 'servo_tracking',
                'gain_tuning_mode': 'fine', 'released': True,
                'gain_trajectory_confirmed': True, 'gain_motion_range_confirmed': True})
            target = Path(task['directory']) / 'gain-trajectory-native-frozen.csv'
            # Allow three complete fine comparisons, including their matching
            # baseline/disturbance measurements; the native T budget is unchanged.
            deadline = time.monotonic() + 180
            while not target.exists():
                status = native.request('status')
                assert status['robot_state'] == 'ACTIVE', status
                calibration = native.request('model_calibration_status')
                assert calibration['phase'] not in ('failed', 'cancelled', 'complete'), calibration
                if time.monotonic() > deadline:
                    raise AssertionError('Worker did not enter the real full-path verification')
                time.sleep(.05)
            # The live stream still owns the old inode. Its final full-cycle
            # replacement opens this FIFO, then blocks when its buffer fills.
            target.rename(target.with_suffix('.live.csv'))
            os.mkfifo(target)
            reader_fd = os.open(target, os.O_RDONLY | os.O_NONBLOCK)
            while True:
                status = native.request('status')
                assert status['robot_state'] == 'ACTIVE', status
                if status['impedance_mode'] == 'RIGID_HOLD':
                    break
                if time.monotonic() > deadline:
                    raise AssertionError('Trajectory did not enter HOLD before assessment')
                time.sleep(.01)
            held_since = time.monotonic()
            samples = 0
            while time.monotonic() - held_since < .35:
                status = native.request('status')
                assert status['robot_state'] == 'ACTIVE', status
                assert status['impedance_mode'] == 'RIGID_HOLD', status
                samples += 1
                time.sleep(.015)
            # Resume the slow sink, then stop through the real worker exit.
            draining.set()
            drain_thread = threading.Thread(target=drain)
            drain_thread.start()
            native.request('model_calibration_cancel', {'task_id': task['task_id']})
            deadline = time.monotonic() + 10
            while True:
                calibration = native.request('model_calibration_status')
                if calibration['phase'] in ('failed', 'cancelled', 'complete'):
                    break
                assert time.monotonic() < deadline, calibration
                time.sleep(.03)
            assert calibration['phase'] == 'cancelled', calibration
            assert native.request('status')['robot_state'] == 'ACTIVE'
            assert consumed[0] > 65536, consumed
            candidate = json.loads((Path(task['directory']) / 'gain-candidate.json').read_text())
            comparisons = [item for item in candidate['candidate_comparisons']
                           if item.get('kind') == 'comparable_candidate']
            assert len(comparisons) <= 3, comparisons
            assert all('excitation_retests' in item for item in candidate['candidate_comparisons']
                       if item.get('kind') == 'motion_establishment')
            return {'status': 'passed', 'hold_observations': samples,
                    'blocked_recording_hold_s': time.monotonic() - held_since,
                    'recorded_bytes': consumed[0], 'cmd_timeout_s': .1,
                    'worker_exit': calibration['phase'], 'local_comparisons': len(comparisons),
                    'scope': 'actual_native_worker_and_FIFO_no_hardware'}
        finally:
            if reader_fd is not None and drain_thread is None:
                draining.set()
                drain_thread = threading.Thread(target=drain)
                drain_thread.start()
            native.close()
            draining.clear()
            if drain_thread:
                drain_thread.join(timeout=2)
            if reader_fd is not None:
                os.close(reader_fd)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--terminal', type=Path,
                        default=ROOT / 'build/unified/standalone-dm_arm-t1-p1-m0/core/serial_arm_terminal')
    print(json.dumps(run(parser.parse_args().terminal.resolve()), indent=2))
