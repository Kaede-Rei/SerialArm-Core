#!/usr/bin/env python3
from pathlib import Path
import re
import sys

ROOT = Path(__file__).resolve().parents[3]
TERMINAL = ROOT / "serial_arm/core/app/serial_arm_terminal.cpp"

def main():
    text = TERMINAL.read_text()
    match = re.search(r"bool\s+run_admittance_friction_calibration\s*\([^)]*\)\s*\{(.*?)\n\s*bool\s+run_admittance_static_calibration", text, re.S)
    assert match, "run_admittance_friction_calibration body not found"
    body = match.group(1)
    after_forward = body.split("回放 2/2：正放", 1)[1]
    before_analysis = after_forward.split("AdmittanceFrictionCalibrationCfg calibration_cfg", 1)[0]
    assert "set_tuning_impedance_mode(JointImpedanceMode::RIGID_HOLD)" in before_analysis, (
        "friction replay must leave RIGID_TRACKING and clear external command timeout before cross validation"
    )
    robot = (ROOT / "serial_arm/core/src/robot.cpp").read_text()
    mode = re.search(r"Robot::set_impedance_mode\(.*?\n\}", robot, re.S)
    assert mode and "has_external_cmd_ = false;" in mode.group(0), "RIGID_HOLD transition must clear external command state"
    print("TERMINAL_FRICTION_REPLAY_CONTRACT_PASS")

if __name__ == "__main__":
    try:
        main()
    except Exception as exc:
        print(f"TERMINAL_FRICTION_REPLAY_CONTRACT_FAIL: {exc}", file=sys.stderr)
        raise
