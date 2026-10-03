import argparse
import json
from pathlib import Path
from profiles import Inspector

parser = argparse.ArgumentParser(description='SerialArm read-only environment and resource inspection')
parser.add_argument('--profile', default='dm_arm_gray')
parser.add_argument('--profile-file', default='')
parser.add_argument('--resource-paths', default='')
args = parser.parse_args()
inspector = Inspector(Path(__file__).resolve().parents[3])
try:
    result = inspector.inspect({'profile': args.profile, 'profile_file': args.profile_file, 'resource_paths': args.resource_paths})
    print(json.dumps({'environment': inspector.status(), 'profile': result}, ensure_ascii=False, indent=2))
    raise SystemExit(0 if all(c['ok'] for c in result['checks']) and inspector.status()['installed'] else 1)
except (ValueError, OSError) as error:
    print(json.dumps({'error': str(error)}, ensure_ascii=False))
    raise SystemExit(1)
