#!/usr/bin/env python3
from pathlib import Path
import re
import sys
import yaml

ROOT = Path(__file__).resolve().parents[3]
GRAY = ROOT / 'robot_supports/robots/dm_arm/description/config/core/gray.yaml'
WHITE = ROOT / 'robot_supports/robots/dm_arm/description/config/core/white.yaml'
CONFIG_HPP = ROOT / 'serial_arm/core/include/serial_arm/config/config.hpp'
TERMINAL_CPP = ROOT / 'serial_arm/core/app/serial_arm_terminal.cpp'
ROBOT_CPP = ROOT / 'serial_arm/core/src/robot.cpp'

LEGACY_KEYS = {
    'observer_mode', 'momentum_gain', 'filter_alpha',
    'mass', 'damping', 'stiffness', 'torque_bias', 'torque_threshold',
    'friction_compensation', 'variable_admittance', 'max_delta_q', 'max_delta_q_dot',
}
EXPECTED_TOP = {'enabled', 'joint_enabled', 'observer', 'calibration', 'feel'}
EXPECTED_OBSERVER = {'mode', 'momentum_gain', 'filter_alpha'}
EXPECTED_CAL = {'torque_bias', 'torque_threshold', 'friction'}
EXPECTED_FEEL = {
    'comfortable_torque', 'follow_speed', 'start_response_s',
    'q_elastic_start_speed', 'return_time_s', 'max_retreat',
    'max_correction_speed', 'q_elastic_max_resistance_ratio',
}


def load_adm(path: Path):
    data = yaml.safe_load(path.read_text())
    return data['capability']['admittance']


def check_yaml(path: Path):
    adm = load_adm(path)
    assert set(adm) == EXPECTED_TOP, (path.name, set(adm), EXPECTED_TOP)
    assert not (LEGACY_KEYS & set(adm)), (path.name, LEGACY_KEYS & set(adm))
    assert set(adm['observer']) == EXPECTED_OBSERVER
    assert set(adm['calibration']) == EXPECTED_CAL
    assert set(adm['feel']) == EXPECTED_FEEL


def main():
    check_yaml(GRAY)
    check_yaml(WHITE)

    hpp = CONFIG_HPP.read_text()
    for token in ('AdmittanceObserverCfg', 'AdmittanceCalibrationCfg', 'AdmittanceFeelCfg', 'derive_admittance_controller_cfg'):
        assert token in hpp, token
    public_struct = re.search(r'struct AdmittanceCapabilityCfg\s*\{(.*?)\n\};', hpp, re.S)
    assert public_struct, 'AdmittanceCapabilityCfg missing'
    body = public_struct.group(1)
    for raw in ('JointVector mass;', 'JointVector damping;', 'JointVector stiffness;', 'JointVector max_delta_q;', 'JointVector max_delta_q_dot;', 'VariableAdmittanceCfg variable;'):
        assert raw not in body, raw

    terminal = TERMINAL_CPP.read_text()
    assert '直接 M / D / K' not in terminal
    assert '&AdmittanceCapabilityCfg::mass' not in terminal
    assert '&AdmittanceCapabilityCfg::damping' not in terminal
    assert '&AdmittanceCapabilityCfg::stiffness' not in terminal
    assert 'derived_M' in terminal and 'derived_D_follow' in terminal and 'derived_K_return' in terminal

    robot = ROBOT_CPP.read_text()
    assert 'derive_admittance_controller_cfg' in robot
    assert 'admittance.mass' not in robot
    assert 'admittance.damping' not in robot
    assert 'admittance.stiffness' not in robot

    print('ADMITTANCE_SCHEMA_CONTRACT_PASS')


if __name__ == '__main__':
    try:
        main()
    except Exception as exc:
        print(f'ADMITTANCE_SCHEMA_CONTRACT_FAIL: {exc}', file=sys.stderr)
        raise
