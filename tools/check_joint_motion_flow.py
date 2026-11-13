#!/usr/bin/env python3
"""Check real native point-motion completion IPC with write-disabled Mock only."""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import tempfile
import time

from check_adaptive_gain_flow import Native, ROOT, prepare_config


def run(terminal: Path) -> dict:
    with tempfile.TemporaryDirectory(prefix='serialarm-joint-motion-') as temporary:
        directory=Path(temporary)
        config,names=prepare_config(directory)
        before=config.read_text()
        storage=directory/'storage';storage.mkdir()
        native=Native(terminal,config,storage)
        try:
            native.request('activate');initial=native.ready()
            target=[float(q)+.025 for q in initial['joint']['pos']]
            motion=native.request('move_to',{'positions':target,'speed_scale':.2})
            motion_id=motion['id'];seen=[];started=time.monotonic()
            while time.monotonic()-started<8:
                snapshot=native.request('status');motion=snapshot['motion']
                assert snapshot['robot_state']=='ACTIVE' and snapshot['write_enabled'] is False,snapshot
                assert motion['id']==motion_id,motion
                if motion['state']=='completed':
                    assert motion['reference_complete'] and motion['target_reached'],motion
                    assert motion['reason']=='target_reached' and motion['settling_velocity_source']=='fresh_position_window',motion
                    assert len(motion['position_error'])==len(names),motion
                    assert all(abs(e)<=tol for e,tol in zip(motion['position_error'],motion['position_tolerance'])),motion
                    break
                assert motion['state']=='running',motion
                seen.append(motion['reference_complete'])
                time.sleep(.01)
            else:
                raise AssertionError('The real native motion did not complete from fresh stationary feedback')
            assert any(seen),'No actual settling phase was observed after reference completion'
            second=native.request('move_relative',{'delta':[.2]*len(names),'speed_scale':.2})
            assert second['id']>motion_id,second
            native.request('hold')
            stopped=native.request('status')['motion']
            assert stopped['id']==second['id'] and stopped['state']=='interrupted',stopped
            assert config.read_text()==before,'Point motion changed its input Profile'
            return {'scope':'write_disabled_native_Mock_only','joint_count':len(names),
                    'completed':motion,'elapsed_s':time.monotonic()-started,
                    'next_motion_and_explicit_hold':stopped,'input_profile_unchanged':True}
        finally:
            native.close()


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--terminal',type=Path,default=ROOT/'build/unified/standalone-dm_arm-t1-p1-m0/core/serial_arm_terminal')
    args=parser.parse_args()
    print(json.dumps(run(args.terminal.resolve()),indent=2))


if __name__=='__main__':
    main()
