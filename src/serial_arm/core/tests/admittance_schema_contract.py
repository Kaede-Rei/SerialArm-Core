#!/usr/bin/env python3
from pathlib import Path
import re
import sys
import yaml

ROOT = Path(__file__).resolve().parents[3]
GRAY = ROOT / 'robot_supports/robots/dm_arm/description/config/core/gray.yaml'
WHITE = ROOT / 'robot_supports/robots/dm_arm/description/config/core/white.yaml'
CONFIG_HPP = ROOT / 'serial_arm/core/include/serial_arm/config/config.hpp'
JOINT_ADM_HPP = ROOT / 'serial_arm/core/include/serial_arm/interaction/joint_admittance_controller.hpp'
EXTERNAL_HPP = ROOT / 'serial_arm/core/include/serial_arm/interaction/external_torque_observer.hpp'
INTERACTION_HPP = ROOT / 'serial_arm/core/include/serial_arm/interaction/interaction_controller.hpp'
TERMINAL_CPP = ROOT / 'serial_arm/core/app/serial_arm_terminal.cpp'
ROBOT_CPP = ROOT / 'serial_arm/core/src/robot.cpp'

EXPECTED_TOP = {'enabled', 'joint_enabled', 'observer', 'calibration', 'controller'}
EXPECTED_OBSERVER = {'mode', 'momentum_gain', 'filter_alpha'}
EXPECTED_CAL = {'torque_bias', 'torque_threshold', 'friction'}
EXPECTED_FRICTION = {
    'enabled', 'velocity_transition',
    'positive_coulomb', 'positive_viscous',
    'negative_coulomb', 'negative_viscous',
}
EXPECTED_CONTROLLER = {'mass', 'damping', 'stiffness', 'max_delta_q', 'max_delta_q_dot'}
REMOVED_TOKENS = {
    'feel', 'comfortable_torque', 'follow_speed', 'start_response_s',
    'q_elastic_start_speed', 'return_time_s', 'max_retreat',
    'max_correction_speed', 'q_elastic_max_resistance_ratio',
    'zero_velocity_adaptation_s', 'kinetic_feedforward_scale',
}


def load_adm(path: Path):
    data = yaml.safe_load(path.read_text())
    return data['capability']['admittance']


def check_inline_joint_map(text: str, key: str):
    pattern = rf'^\s*{re.escape(key)}:\s*\{{joint1:.*joint2:.*joint3:.*joint4:.*joint5:.*joint6:.*\}}\s*(?:#.*)?$'
    assert re.search(pattern, text, re.M), f'{key} must stay a single-line joint map'


def check_yaml(path: Path):
    text = path.read_text()
    adm = load_adm(path)
    assert set(adm) == EXPECTED_TOP, (path.name, set(adm), EXPECTED_TOP)
    assert set(adm['observer']) == EXPECTED_OBSERVER
    assert set(adm['calibration']) == EXPECTED_CAL
    assert set(adm['calibration']['friction']) == EXPECTED_FRICTION
    assert set(adm['controller']) == EXPECTED_CONTROLLER
    for token in REMOVED_TOKENS:
        assert token not in adm, (path.name, token)

    for key in (
        'joint_enabled', 'momentum_gain', 'torque_bias', 'torque_threshold',
        'positive_coulomb', 'positive_viscous', 'negative_coulomb', 'negative_viscous',
        'mass', 'damping', 'stiffness', 'max_delta_q', 'max_delta_q_dot',
    ):
        check_inline_joint_map(text, key)

    # Every public admittance scalar/vector parameter line must have a concise comment line immediately above it
    lines = text.splitlines()
    parameter_keys = {
        'enabled', 'joint_enabled', 'mode', 'momentum_gain', 'filter_alpha',
        'torque_bias', 'torque_threshold', 'velocity_transition',
        'positive_coulomb', 'positive_viscous', 'negative_coulomb', 'negative_viscous',
        'mass', 'damping', 'stiffness', 'max_delta_q', 'max_delta_q_dot',
    }
    for i, line in enumerate(lines):
        stripped = line.strip()
        if ':' not in stripped or stripped.startswith('#'):
            continue
        key = stripped.split(':', 1)[0]
        if key not in parameter_keys:
            continue
        assert i > 0 and lines[i - 1].lstrip().startswith('#'), f'{path.name}:{i+1} {key} needs one comment line above'


def main():
    check_yaml(GRAY)
    check_yaml(WHITE)

    hpp = CONFIG_HPP.read_text()
    for token in ('AdmittanceObserverCfg', 'AdmittanceCalibrationCfg', 'AdmittanceControllerCfg'):
        assert token in hpp, token
    for token in ('AdmittanceFeelCfg', 'derive_admittance_controller_cfg'):
        assert token not in hpp, token
    public_struct = re.search(r'struct AdmittanceCapabilityCfg\s*\{(.*?)\n\};', hpp, re.S)
    assert public_struct, 'AdmittanceCapabilityCfg missing'
    body = public_struct.group(1)
    assert 'AdmittanceControllerCfg controller;' in body

    joint_hpp = JOINT_ADM_HPP.read_text()
    assert 'VariableAdmittanceCfg' not in joint_hpp
    assert 'contact_confidence' not in joint_hpp
    assert 'contact_blend' not in joint_hpp

    external_hpp = EXTERNAL_HPP.read_text()
    assert 'contact_confidence' not in external_hpp
    assert 'static_friction_baseline_' not in external_hpp

    interaction_hpp = INTERACTION_HPP.read_text()
    for token in ('contact_confidence', 'contact_blend', 'effective_damping', 'effective_stiffness'):
        assert token not in interaction_hpp, token

    terminal = TERMINAL_CPP.read_text()
    assert ' 1. 一次性标定' in terminal
    assert ' 2. 静态残差标定' in terminal
    assert ' 3. 静态残差验证' in terminal
    assert ' 4. 摩擦参数标定' in terminal
    assert 'M / D / K 设置' in terminal or 'M/D/K 设置' in terminal
    assert 'derived_M' not in terminal
    assert 'q_elastic' not in terminal
    assert '摩擦主动助力' not in terminal

    robot = ROBOT_CPP.read_text()
    assert 'derive_admittance_controller_cfg' not in robot
    assert 'kinetic_feedforward_scale' not in robot
    assert 'contact_confidence' not in robot

    print('ADMITTANCE_SCHEMA_CONTRACT_PASS')


if __name__ == '__main__':
    try:
        main()
    except Exception as exc:
        print(f'ADMITTANCE_SCHEMA_CONTRACT_FAIL: {exc}', file=sys.stderr)
        raise
