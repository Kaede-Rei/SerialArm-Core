#!/usr/bin/env python3
"""Exercise CSP and all four interactions through the offline Terminal IPC."""
from pathlib import Path
import argparse
import json
import tempfile
import time

from check_adaptive_gain_flow import Native, prepare_config


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--terminal', required=True, type=Path)
    terminal = parser.parse_args().terminal.resolve()
    with tempfile.TemporaryDirectory(prefix='serialarm-csp-ipc-') as temp:
        directory = Path(temp)
        config, names = prepare_config(directory)
        document = json.loads(config.read_text())
        document['safety_policy']['max_tracking_error'] = {name: 0.5 for name in names}
        config.write_text(json.dumps(document))
        native = Native(terminal, config, directory / 'storage')
        try:
            native.request('set_control_path', {'path': 'CSP'})
            native.request('activate')
            native.ready()
            results = {}
            for mode in ('RIGID', 'COMPLIANT', 'DRAG', 'SERVO_TRACKING'):
                previous = native.request('status')['sequence']
                native.request('set_impedance_mode', {'mode': mode})
                deadline = time.monotonic() + 3
                while time.monotonic() < deadline:
                    status = native.request('status')
                    if status.get('robot_state') == 'FAULT':
                        raise AssertionError(status.get('fault'))
                    if status.get('valid') and status.get('impedance_mode') == mode and status.get('sequence', 0) > previous:
                        break
                    time.sleep(.02)
                else:
                    raise AssertionError(f'{mode}: no fresh status: {status}')
                assert status['write_enabled'] is False and status['control_path'] == 'CSP', status
                joint = status['joint']
                assert all(value == 0 for value in joint['kp'] + joint['kd']), status
                assert len(joint['ref_pos']) == len(names) and len(joint['virtual_ref_pos']) == len(names), status
                assert status['admittance_active'] == (mode in ('COMPLIANT', 'DRAG')), status
                # The mock rejects CSP writes containing any nonzero kp/kd/torque.
                results[mode] = {'admittance_active': status['admittance_active']}
            print(json.dumps({'status': 'passed', 'modes': results,
                              'scope': 'write-disabled mock IPC; no physical hardware'}))
        finally:
            native.close()


if __name__ == '__main__':
    main()
