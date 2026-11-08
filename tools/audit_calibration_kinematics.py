#!/usr/bin/env python3
"""Run read-only angular q/dq audit on one or more frames.csv files"""
import argparse,csv,hashlib,json,sys
from pathlib import Path
import numpy as np
sys.path.insert(0, str(Path(__file__).resolve().parents[1]/"apps/launcher/backend"))
from kinematic_consistency_audit import audit_angular_kinematics


def audit_csv(path):
    path=Path(path).expanduser().resolve();before=hashlib.sha256(path.read_bytes()).hexdigest()
    with path.open(newline='')as f:
        reader=csv.DictReader(f);fields=reader.fieldnames or []
        joints=[k[2:]for k in fields if k.startswith('q:')];rows=list(reader)
    if not joints or not rows:raise ValueError('CSV needs q:<joint> fields and data rows')
    ns=np.array([int(r['monotonic_ns'])for r in rows],dtype=np.int64)
    q=np.array([[float(r['q:'+j])for j in joints]for r in rows]);v=np.array([[float(r['dq:'+j])for j in joints]for r in rows])
    phase=np.array([r['phase']for r in rows]);age=np.array([float(r.get('feedback_age_ms')or 'nan')for r in rows])
    valid=np.array([r.get('valid')=='1'for r in rows])&np.isin(phase,['static_reverse','friction_forward_fast'])&np.isfinite(age)&(age>=0)&(age<=30)
    r=audit_angular_kinematics((ns-ns[0])/1e9,q,v,phase=phase,valid=valid,joint_names=joints)
    after=hashlib.sha256(path.read_bytes()).hexdigest()
    if after!=before:raise ValueError('Input changed during audit')
    return dict(input=str(path),sha256=before,source_unchanged=True,audit=r)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('frames',nargs='+')
    parser.add_argument('--output',help='Write a NEW report; omitted prints JSON to stdout')
    args=parser.parse_args()
    try:
        output=Path(args.output).expanduser().resolve() if args.output else None
        if output is not None and (output.exists() or output in [Path(p).expanduser().resolve() for p in args.frames]):
            raise ValueError('Refusing to overwrite an existing file or source CSV')
        result=[audit_csv(p) for p in args.frames]
        text=json.dumps(result,ensure_ascii=False,indent=2,allow_nan=False)+'\n'
        if output is None:
            print(text,end='')
        else:
            with output.open('x',encoding='utf-8') as stream:
                stream.write(text)
            print(str(output))
        return 0
    except (ValueError,OSError,KeyError,IndexError) as exc:
        print('Audit failed: '+str(exc),file=sys.stderr)
        return 1

if __name__=='__main__':raise SystemExit(main())
