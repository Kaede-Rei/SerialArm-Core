"""Saved commissioning progress, separate from live connection and runtime admission."""
from copy import deepcopy


# Operator reports remain operator reports; only the dedicated measurement
# endpoints can supply measured observation and response evidence.
REVIEWS = {
    'model_check': ('模型与硬件能力', 'Model and hardware capabilities', [('chain', '模型受控链与实际机构一致'), ('limits', '执行器模式与位置、速度、力矩反馈能力已核对'), ('inertial', '硬件参数、模型来源与已知缺项已记录')]),
    'mapping_calibration': ('关节映射检查', 'Joint mapping', [('joint_ids', '每个关节与电机 ID 已实测对应'), ('directions', '每个关节反馈方向已实测正确'), ('offsets', '零位、比例与偏移量已确认')]),
    'bringup': ('失能连接与反馈', 'Disabled connection and feedback', [('connection', '已在 INACTIVE 状态检查全部关节通信与新反馈')]),
    'safety_setup': ('运动边界与初始驱动', 'Motion limits and initial drive settings', [('limits', '位置、速度、加速度、力矩与跟踪误差上限已核对'), ('initial_drive', '当前执行路径的初始保持参数已核对'), ('emergency_stop', '硬件急停与失能方式已确认')]),
    'startup_hold': ('保持与逐轴双向运动', 'Hold and bidirectional joint motion', [('hold_gains', '全部关节初始保持参数已核对'), ('supported', '机械支撑与运动空间已确认'), ('emergency_stop', '失能与急停可用'), ('bidirectional', '已逐轴完成受限双向运动与当前位置保持')]),
    'stop_fault_check': ('停止与故障基础检查', 'Basic stopping and fault checks', [('stop', '已验证正常停止'), ('disable', '已验证立即失能及相应机械支撑'), ('fault', '已核对故障原因显示和清除条件')]),
    'gravity_verification': ('重力补偿验证', 'Gravity compensation verification', [('distinct_poses', '已记录至少三个不同稳定姿态'), ('support', '机械支撑与急停措施已检查'), ('restored', '已失能并确认临时参数恢复')]),
    'mit_drag_validation': ('MIT 拖拽与释放', 'MIT drag and release', [('quiet', '无接触时没有持续漂移'), ('release', '释放后运动能衰减停止'), ('bounds', '拖拽全过程保持在运动边界内')]),
    'mit_tracking_validation': ('MIT 增益与轨迹跟踪', 'MIT gains and trajectory tracking', [('gains', '当前用途的增益来源与实测响应已核对'), ('tracking', '已完成受限轨迹跟踪'), ('hold', '到位保持稳定')]),
    'mit_mode_validation': ('MIT 四模式交接', 'MIT interaction mode transitions', [('rigid', '刚性与柔性切换连续'), ('drag', '拖拽换位后切回定位无参考跳变'), ('tracking', '伺服跟踪进入与退出连续')]),
    'csp_execution_validation': ('CSP 执行动态', 'CSP execution dynamics', [('positioning', '逐轴小幅定位正常'), ('delay', '实际速度、延迟与到位响应已记录'), ('tracking', '跟踪误差上限与驱动侧限制已核对')]),
    'effort_model_validation': ('反馈力矩与模型一致性', 'Reported effort and model consistency', [('units', '力矩方向、比例与关节侧单位已核对'), ('model', '不同无接触姿态的模型残差已检查'), ('hysteresis', '双向到位与回程迟滞已记录')]),
    'observation_calibration': ('外力估计校准', 'External effort calibration', [('static', '零偏与连续死区来源已核对'), ('friction', '低速双向摩擦补偿已校准或明确关闭'), ('direction', '受控正反加载方向已人工确认')]),
    'observation_validation': ('外力估计独立验收', 'Independent external effort validation', [('contact_free', '独立姿态采集时无外部接触'), ('direction', '已检查正反加载方向'), ('coverage', '已检查验证覆盖与允许残差')]),
    'admittance_response': ('有限刚度导纳响应', 'Finite-stiffness admittance response', [('space', '受限退让试验空间已清空'), ('release', '已明确记录释放时刻'), ('response', '已检查实际位移与稳定响应')]),
    'admittance_release': ('释放与原目标回位', 'Release and return to the original target', [('release', '释放时刻已记录'), ('return', '已核对原目标与稳定带'), ('no_reset', '试验中未用当前位置重锚代替回位')]),
    'csp_drag_validation': ('零刚度拖拽', 'Zero-stiffness drag', [('quiet', '无接触无持续单向运动'), ('release', '推拉释放后能停止'), ('bounds', '位移与速度边界已验证')]),
    'upper_layer_validation': ('模式交接与上层接口', 'Mode transitions and upper-layer interfaces', [('modes', '当前路径各交互模式切换连续'), ('commands', '上层目标、取消与停止命令正常'), ('recovery', '故障恢复使用当前合法参考且不会自行续跑')]),
    'parking_validation': ('停放与退出验收', 'Parking and exit validation', [('path', '停放目标与整段路径已核对'), ('tracking', '所选停放方式到位稳定'), ('disable', '到位后失能与机械支撑已确认')]),
    'gravity_correction': ('重力模型校正', 'Gravity model correction', [('holdout', '独立留出姿态已比较'), ('hardware', '候选模型已实机对照')]),
    'dynamics_calibration': ('完整动力学辨识', 'Full dynamics identification', [('coverage', '激励与参数可辨识性已检查'), ('holdout', '独立轨迹预测已比较')]),
    'geometry_calibration': ('TCP 与几何标定', 'TCP and geometric calibration', [('frames', 'Base 与 Tool 坐标系已实测核对'), ('zero_pose', '零位与机构位置一致'), ('tool_pose', 'TCP 与工具姿态已核对')]),
    'position_repeatability': ('重复到位与重复定位', 'Point return and positioning repeatability', [('cycles', '往返方向、循环和停留条件已记录'), ('measurement', '重复误差来源与统计方法已记录')]),
    'absolute_accuracy': ('绝对定位精度', 'Absolute positioning accuracy', [('reference', '独立外部参考与测量方法已记录'), ('workspace', '工作空间与负载条件已记录')]),
    'validation': ('综合验收', 'Overall acceptance', [('tracking', '已检查真实跟踪效果'), ('stopping', '已检查停放和失能'), ('faults', '已核对故障处理')]),
}

FOUNDATION = ('model_connection', 'mapping_safety', 'startup_hold', 'stop_fault_check')
PATH_STEPS = {
    'MIT': ('gravity_verification', 'mit_tracking_validation', 'mit_interaction'),
    'CSP': ('observation_validation', 'admittance_response', 'csp_interaction'),
}
OPTIONAL = ('parking_validation', 'gravity_correction', 'dynamics_calibration', 'geometry_calibration', 'position_repeatability', 'absolute_accuracy')
SHARED = frozenset(('model_check', 'mapping_calibration', 'bringup', 'gravity_verification', 'validation',
                    *(stage for stage in OPTIONAL if stage != 'parking_validation')))
CHECK_EN = {
    'chain': 'The modeled control chain matches the physical mechanism',
    'limits': 'Applicable actuator capabilities and motion limits have been checked',
    'inertial': 'Hardware settings, model provenance and known omissions are recorded',
    'joint_ids': 'Every joint has been matched to its physical motor ID',
    'directions': 'Every joint feedback direction has been physically checked',
    'offsets': 'Zero positions, ratios and offsets have been confirmed',
    'connection': 'Communication and fresh feedback were checked for every joint while INACTIVE',
    'initial_drive': 'Initial hold settings for the selected execution path have been checked',
    'emergency_stop': 'Emergency stop and disabling are available and understood',
    'hold_gains': 'Initial hold settings have been checked for every joint',
    'supported': 'Mechanical support and the motion area have been checked',
    'bidirectional': 'Bounded motion in both directions and holding were checked for every joint',
    'stop': 'Normal stopping has been tested', 'disable': 'Immediate disabling and mechanical support were tested',
    'fault': 'Fault reporting and clearing conditions were checked',
    'distinct_poses': 'At least three distinct stable poses were recorded',
    'support': 'Mechanical support and emergency stopping have been checked',
    'restored': 'The arm was disabled and temporary settings were restored',
    'quiet': 'No sustained drift occurs without contact',
    'release': 'Release was explicitly marked and the resulting response checked',
    'bounds': 'The test remained inside motion and effort limits',
    'gains': 'The source of the selected gains and measured response were reviewed',
    'tracking': 'The relevant bounded tracking test was completed', 'hold': 'Position holding is stable',
    'rigid': 'Rigid and compliant transitions are continuous',
    'drag': 'Leaving drag after repositioning does not jump the reference',
    'positioning': 'Small positioning moves were checked for every joint',
    'delay': 'Actual speed, delay and settling response were recorded',
    'units': 'Effort direction, scaling and joint-side units were checked',
    'model': 'Model consistency across contact-free poses was checked',
    'hysteresis': 'Bidirectional approaches and return hysteresis were recorded',
    'static': 'The sources of bias and continuous deadband were reviewed',
    'friction': 'Bidirectional low-speed friction was calibrated or explicitly disabled',
    'direction': 'The response direction under controlled positive and negative loading was checked',
    'contact_free': 'Independent validation poses were collected without external contact',
    'coverage': 'Validation coverage and allowed residuals were reviewed',
    'space': 'The bounded yielding test area was cleared',
    'response': 'Actual displacement and a stable response were checked',
    'return': 'The original target and stable return band were checked',
    'no_reset': 'Re-anchoring was not used as a substitute for returning',
    'modes': 'Interaction modes on the selected execution path switch continuously',
    'commands': 'Upper-layer target, cancellation and stop commands were tested',
    'recovery': 'Fault recovery uses a valid reference and never resumes motion by itself',
    'path': 'The parking target and complete path were checked',
    'holdout': 'Independent held-out measurements were compared',
    'hardware': 'The candidate was compared on physical hardware',
    'frames': 'Base and tool frames were physically checked', 'zero_pose': 'The zero pose matches the mechanism',
    'tool_pose': 'The TCP and tool orientation were checked',
    'cycles': 'Approach directions, cycles and dwell conditions were recorded',
    'measurement': 'Measurement sources and statistical methods were recorded',
    'reference': 'An independent external reference and measurement method were recorded',
    'workspace': 'Workspace and payload conditions were recorded',
    'stopping': 'Parking and disabling were checked', 'faults': 'Fault handling was reviewed',
}


def stage_key(stage, path):
    return stage if stage in SHARED else path + ':' + stage



def _check(stage, key, label=None, *, also=(), label_en=None):
    text = dict(REVIEWS[stage][2])[key]
    return {'id': stage + '__' + key, 'label': label or text, 'label_en': label_en or CHECK_EN[key],
            'sources': [(stage, key), *also]}


# Tasks group existing scoped records. Sources remain readable by old tools and
# retain their own invalidation rules; there is no second progress store.
TASKS = {
    'model_connection': {
        'label': '模型与硬件连接', 'label_en': 'Model and hardware connection',
        'stages': ('model_check', 'bringup'), 'config': ('model', 'hardware'),
        'instructions': [('打开模型，核对受控关节、机构和执行器支持的控制模式', 'Inspect the modeled joints, mechanism and supported actuator modes'),
                         ('在诊断页连接硬件并保持失能，检查每个关节的位置、速度和力矩反馈', 'Connect while disabled and inspect position, velocity and effort feedback for every joint')],
        'actions': [('查看模型', 'Inspect model', {'page': 'model'}), ('连接并检查反馈', 'Connect and inspect feedback', {'page': 'diagnostics', 'anchor': 'connection-settings'})],
        'checks': [_check('model_check', 'chain'), _check('model_check', 'limits'), _check('model_check', 'inertial'), _check('bringup', 'connection')],
    },
    'mapping_safety': {
        'label': '关节映射与安全边界', 'label_en': 'Joint mapping and safety limits',
        'stages': ('mapping_calibration', 'safety_setup'), 'config': ('hardware', 'safety', 'hold'),
        'instructions': [('失能并可靠支撑机械臂，按接线与模型填写电机 ID、方向、零位与比例', 'Support and disable the arm; configure motor IDs, directions, zeros and ratios from wiring and the model'),
                         ('设置运动边界、初始保持和急停方式，下一步再进行受限运动', 'Configure motion limits, initial holding and emergency stopping before the next bounded-motion task')],
        'actions': [('核对关节映射', 'Check joint mapping', {'page': 'profile_setup', 'group': 'mapping'}), ('设置安全边界', 'Set safety limits', {'page': 'profile_setup', 'group': 'safety'}), ('设置初始保持', 'Set initial holding', {'page': 'profile_setup', 'group': 'motion'})],
        'checks': [_check('mapping_calibration', 'joint_ids', '关节与电机 ID 配置已按接线核对', label_en='Joint and motor ID settings match the wiring'), _check('mapping_calibration', 'directions', '方向、比例与零位配置已按模型和接线核对', label_en='Direction, ratio and zero settings match the model and wiring'), _check('safety_setup', 'limits'), _check('safety_setup', 'initial_drive'), _check('safety_setup', 'emergency_stop')],
    },
    'startup_hold': {
        'label': '刚性保持与逐轴运动', 'label_en': 'Rigid holding and joint motion',
        'stages': ('startup_hold',), 'config': ('hold',),
        'instructions': [('清空运动空间并准备机械支撑，使能后先检查当前位置刚性保持', 'Clear the movement area, prepare support and check rigid holding at the current position'),
                         ('逐轴执行小幅、低速、双向运动，核对实际关节、方向和到位稳定性', 'Move each joint slowly in both directions over a small range and check correspondence and settling')],
        'actions': [('检查保持与逐轴运动', 'Check holding and joint motion', {'page': 'workbench', 'tab': 'control'})],
        'checks': [_check('startup_hold', 'supported')],
    },
    'stop_fault_check': {
        'label': '停止与故障处理', 'label_en': 'Stopping and fault handling',
        'stages': ('stop_fault_check',), 'config': ('hold',),
        'instructions': [('在受限运动中验证正常停止，再在可靠支撑下验证立即失能', 'Check normal stopping during bounded motion and immediate disabling with reliable support'),
                         ('核对故障原因、清除条件和恢复入口，确认清除后不会自行继续原运动', 'Review fault causes, clearing conditions and recovery; clearing must not resume previous motion')],
        'actions': [('检查停止与故障处理', 'Check stopping and faults', {'page': 'workbench', 'tab': 'control'})],
    },
    'gravity_verification': {
        'label': '重力补偿验证', 'label_en': 'Gravity compensation verification',
        'stages': ('gravity_verification',), 'config': ('gravity',),
        'instructions': [('核对当前重力方向、负载和补偿比例；已有有效模型确认可沿用', 'Review gravity direction, payload and compensation scales; retain a valid existing model confirmation'),
                         ('未确认时打开验证工具，在可靠支撑下完成稳定姿态检查并保存确认', 'If unconfirmed, use the verification tool with reliable support and save the stable-pose checks')],
        'actions': [('验证并确认重力补偿', 'Verify gravity compensation', {'page': 'calibration', 'entry': 'gravity_verify'})],
    },
    'mit_tracking_validation': {
        'label': '增益与轨迹跟踪', 'label_en': 'Gains and trajectory tracking',
        'stages': ('mit_tracking_validation',), 'config': ('hold',),
        'instructions': [('设置或标定当前用途的 MIT 增益，保存后进行受限轨迹跟踪', 'Set or calibrate MIT gains for the intended use, save them and test a bounded trajectory'),
                         ('检查跟踪误差与到位保持，记录所用增益和测试结果', 'Inspect tracking error and settled holding; record the gains and test results')],
        'actions': [('设置执行增益', 'Set execution gains', {'page': 'profile_setup', 'group': 'motion'}), ('标定并验证增益', 'Calibrate and validate gains', {'page': 'calibration', 'entry': 'gain'})],
    },
    'mit_interaction': {
        'label': '拖拽、模式切换与上层接口', 'label_en': 'Dragging, mode transitions and upper-layer interfaces',
        'stages': ('mit_drag_validation', 'mit_mode_validation', 'upper_layer_validation'), 'config': ('drag', 'mit_modes'),
        'instructions': [('重力模型确认后，在运动边界内检查无接触静止、推拉和释放停止', 'After confirming gravity, check contact-free rest, dragging and release within the limits'),
                         ('拖拽换位后切回刚性或柔性，再检查伺服跟踪、上层目标、取消和故障恢复', 'Reposition while dragging, switch back to rigid or compliant mode and check tracking, goals, cancellation and recovery')],
        'actions': [('检查交互与上层控制', 'Check interaction and upper-layer control', {'page': 'workbench', 'tab': 'control'})],
        'checks': [_check('mit_drag_validation', 'quiet', '无接触时无持续漂移，推拉释放后能停稳', also=(('mit_drag_validation', 'release'),), label_en='No contact-free drift; motion settles after releasing a push or pull'), _check('mit_drag_validation', 'bounds'), _check('mit_mode_validation', 'drag', '拖拽换位后切回刚性或柔性，参考与运动连续', also=(('mit_mode_validation', 'rigid'),), label_en='References and motion remain continuous after dragging and switching to rigid or compliant mode'), _check('mit_mode_validation', 'tracking', also=(('upper_layer_validation', 'modes'),)), _check('upper_layer_validation', 'commands'), _check('upper_layer_validation', 'recovery')],
    },
    'observation_validation': {
        'label': '外力估计校准与验收', 'label_en': 'External effort calibration and acceptance',
        'stages': ('effort_model_validation', 'observation_calibration', 'observation_validation'), 'config': ('hold', 'observation'),
        'instructions': [('在刚性保持下核对力矩方向、比例和不同无接触姿态的残差；按需要校准零偏、死区与摩擦', 'While holding rigidly, check effort direction, scale and contact-free residuals; adjust bias, deadband and friction as needed'),
                         ('使用 5 个独立无接触姿态验证，每个姿态静止采样 15 s，并保存外力验收', 'Validate at five independent contact-free poses, sample each for 15 s and save effort acceptance'),
                         ('进行受控正反加载，确认估计方向，再保存本页检查结果', 'Apply controlled loads in both directions, confirm the estimated direction and save the inspection')],
        'actions': [('校准并验证外力估计', 'Calibrate and validate effort estimation', {'page': 'calibration', 'entry': 'observation'}), ('修改外力估计参数', 'Edit effort estimation parameters', {'page': 'profile_setup', 'group': 'observation'})],
        'checks': [_check('effort_model_validation', 'units'), _check('effort_model_validation', 'model', '不同姿态及双向到位的模型残差与回程差异已检查', also=(('effort_model_validation', 'hysteresis'),), label_en='Model residuals across poses and bidirectional approaches were checked'), _check('observation_calibration', 'static', '零偏、死区与摩擦补偿来源已确认，未使用的补偿已关闭', also=(('observation_calibration', 'friction'),), label_en='Bias, deadband and friction sources were checked; unused compensation is disabled'), _check('observation_validation', 'contact_free'), _check('observation_validation', 'direction', also=(('observation_calibration', 'direction'),))],
    },
    'admittance_response': {
        'label': '导纳退让与释放回位', 'label_en': 'Admittance yielding and release return',
        'stages': ('admittance_response', 'admittance_release'), 'config': ('response',),
        'instructions': [('外力估计验收通过后，设置 M、D、K 和退让边界，先使用有限刚度试验', 'After effort acceptance, set M, D, K and yielding limits and begin with finite stiffness'),
                         ('开始记录，施加外力后标记释放，等待回到原目标并稳定，再结束记录并分析保存', 'Record loading, mark release, wait for return and settling, then stop and save the analyzed recording'),
                         ('逐轴补齐启用关节的试验覆盖；通过后自动完成本任务', 'Cover every enabled joint; a passing saved measurement completes this task automatically')],
        'actions': [('整定并验证导纳响应', 'Tune and validate admittance response', {'page': 'calibration', 'entry': 'response'})],
        'checks': [],
    },
    'csp_interaction': {
        'label': '拖拽、模式切换与上层接口', 'label_en': 'Dragging, mode transitions and upper-layer interfaces',
        'stages': ('csp_drag_validation', 'upper_layer_validation'), 'config': ('drag', 'response'),
        'instructions': [('外力与有限刚度响应验收通过后，检查零刚度拖拽和释放停止', 'After effort and finite-stiffness acceptance, check zero-stiffness dragging and release'),
                         ('拖拽换位后切回刚性或柔性，核对上层目标、取消、停止与故障恢复', 'Reposition while dragging, switch back to rigid or compliant mode and check goals, cancellation, stopping and recovery')],
        'actions': [('检查拖拽与上层控制', 'Check dragging and upper-layer control', {'page': 'workbench', 'tab': 'control'})],
        'checks': [_check('csp_drag_validation', 'quiet', '无接触时无持续单向运动，推拉释放后能停稳', also=(('csp_drag_validation', 'release'),), label_en='No contact-free drift; motion settles after releasing a push or pull'), _check('csp_drag_validation', 'bounds'), _check('upper_layer_validation', 'modes'), _check('upper_layer_validation', 'commands'), _check('upper_layer_validation', 'recovery')],
    },
}

CONFIG_LABELS = {
    'model': ('模型与受控链有效', 'Valid model and controlled chain'), 'hardware': ('硬件与通信配置完整', 'Complete hardware and connection settings'),
    'safety': ('运动边界与安全参数完整', 'Complete motion and safety limits'), 'hold': ('当前路径的初始保持参数完整', 'Complete initial holding settings for this path'),
    'gravity': ('重力方向与补偿比例有效', 'Valid gravity direction and compensation scales'), 'drag': ('当前路径的拖拽参数完整', 'Complete dragging settings for this path'),
    'mit_modes': ('MIT 各交互模式参数完整', 'Complete MIT interaction settings'), 'observation': ('外力观测与补偿参数完整', 'Complete effort observer and compensation settings'),
    'response': ('导纳参数与退让边界完整', 'Complete admittance settings and yielding limits'), 'parking': ('停放许可、目标与执行参数完整', 'Complete parking permission, target and execution settings'),
}
OPTIONAL_COPY = {
    'parking_validation': ('设置停放目标，检查整段受限路径；在可靠支撑下验证到位后失能', 'Set the parking target, check the bounded path and verify disabling after arrival with reliable support', '设置并检查停放', 'Set and check parking', {'page': 'profile_setup', 'group': 'shutdown'}),
    'gravity_correction': ('采集并拟合重力模型，用独立留出姿态比较候选模型后保存检查结果', 'Fit a gravity model and compare independent held-out poses before saving the inspection', '校正重力模型', 'Correct gravity model', {'page': 'calibration', 'entry': 'gravity'}),
    'dynamics_calibration': ('采集辨识轨迹，比较独立轨迹预测并保存检查结果', 'Collect identification trajectories and compare held-out predictions before saving the inspection', '辨识完整动力学', 'Identify full dynamics', {'page': 'calibration', 'entry': 'dynamics'}),
    'geometry_calibration': ('核对 Base、Tool 与 TCP，完成外部测量并保存检查结果', 'Check Base, Tool and TCP, take external measurements and save the inspection', '标定 TCP 与几何', 'Calibrate TCP and geometry', {'page': 'geometry', 'tool': 'fixed'}),
    'position_repeatability': ('记录往返方向、循环与停留条件，完成重复到位测量并保存结果', 'Record approach directions, cycles and dwell, then measure repeated positioning and save the result', '测量重复到位', 'Measure repeated positioning', {'page': 'geometry', 'tool': 'repeat'}),
    'absolute_accuracy': ('使用独立外部参考测量工作空间内的绝对误差，记录方法和结果', 'Measure absolute workspace error against an independent external reference and record the method and results', None, None, None),
}


def task_spec(task, path, joints=()):
    if task in TASKS:
        result = deepcopy(TASKS[task])
    else:
        label, en, _ = REVIEWS[task]
        zh, english, action, action_en, target = OPTIONAL_COPY[task]
        result = {'label': label, 'label_en': en, 'stages': (task,), 'config': ('parking',) if task == 'parking_validation' else ('model',),
                  'instructions': [(zh, english)], 'actions': [(action, action_en, target)] if target else []}
    result.setdefault('checks', [_check(task, key) for key, _ in REVIEWS.get(task, ('', '', []))[2]])
    if task == 'startup_hold':
        result['checks'].extend({'id': 'startup_hold__joint:' + joint, 'label': joint + '：关节对应、双向运动与当前位置保持已实测正常',
            'label_en': joint + ': correspondence, bidirectional motion and holding checked', 'axis': joint,
            'sources': [('startup_hold', 'joint:' + joint)], 'legacy_sources': [('startup_hold', 'bidirectional')]}
            for joint in joints)
    if task == 'startup_hold' and path == 'CSP':
        result['stages'] += ('csp_execution_validation',)
        result['checks'].append(_check('csp_execution_validation', 'delay'))
        result['instructions'].append(('同时记录实际速度、延迟与到位时间，核对驱动器的速度限制', 'Record actual speed, delay and settling time and check the drive velocity limits'))
    # Keep single-source IDs unchanged for existing task tools.
    if len(result['stages']) == 1:
        for check in result['checks']: check['id'] = check['sources'][0][1]
    return result


def review_requirements(path='MIT', joints=()):
    result = {key: {'title': value[0], 'title_en': value[1], 'keys': deepcopy(value[2]),
                    'keys_en': [[item, CHECK_EN[item]] for item, _ in value[2]]} for key, value in REVIEWS.items()}
    for task in (*FOUNDATION, *PATH_STEPS['MIT'], *PATH_STEPS['CSP'], *OPTIONAL):
        spec = task_spec(task, path, joints)
        result[task] = {'title': spec['label'], 'title_en': spec['label_en'],
                        'keys': [[check['id'], check['label']] for check in spec['checks']],
                        'keys_en': [[check['id'], check['label_en']] for check in spec['checks']]}
    return result


def task_answers(task, answers, path, joints=()):
    """Expand one task form into the original stage records, without migration."""
    if task not in TASKS or (task in REVIEWS and not any('__' in key for key in answers)):
        return None
    spec = task_spec(task, path, joints)
    records = {}
    for check in spec['checks']:
        value = answers.get(check['id'], False)
        if not isinstance(value, bool): raise ValueError('检查项必须为布尔值')
        for source, key in check['sources']:
            records.setdefault(source, {})[key] = value
    return records


def observation_confirmed(stages, scopes):
    entry = stages.get(stage_key('observation_validation', 'CSP')) or {}
    checked = entry.get('observations') or {}
    return (entry.get('result') in ('recorded', 'passed') and entry.get('scope') == scopes.get(stage_key('observation_validation', 'CSP')) and
            checked.get('direction') is True and checked.get('contact_free') is True)


def snapshot(core, configuration, meta, scopes, status, *, gravity_validated=False, modes_by_path=None):
    path = configuration['control_path']
    stages, measurements = meta.get('stages') or {}, meta.get('measurements') or {}
    path_configuration = {}
    for choice in PATH_STEPS:
        modes = (modes_by_path or {}).get(choice) or {}
        configured = lambda mode: modes.get(mode) in ('configured', 'verified', 'warning', 'stale')
        hold = configured('RIGID')
        path_configuration[choice] = {**configuration, 'hold': hold, 'drag': configured('DRAG'),
            'parking': ((configuration.get('parking_by_path') or {}).get(choice) or {}).get('enabled') is True and
                       ((configuration.get('parking_by_path') or {}).get(choice) or {}).get('configured') is True,
            'observation': hold and configuration.get('observation') is True if choice == 'CSP' else configuration.get('observation') is True,
            'response': choice == 'CSP' and hold and configuration.get('observation') is True and configuration.get('response') is True}

    def entry_for(stage, viewed_path):
        key = stage_key(stage, viewed_path)
        entry = stages.get(key) or stages.get(stage) or {}
        result = status(entry, scopes.get(key))
        if stage == 'startup_hold' and not entry:
            entry = (meta.get('modes') or {}).get('RIGID') or {}
            result = status(entry, scopes.get('startup_hold'))
        return entry, result

    def step(task, required, viewed_path):
        spec = task_spec(task, viewed_path, configuration.get('joint_names') or [])
        requirements, observations, historical_sources = [], {}, []
        for group in spec['config']:
            configured = path_configuration[viewed_path].get(group) is True
            zh, en = CONFIG_LABELS[group]
            requirements.append({'id': 'configuration:' + group, 'kind': 'configuration', 'required': True,
                                 'label': zh, 'label_en': en, 'status': 'configured' if configured else 'pending'})
        configuration_ready = all(item['status'] == 'configured' for item in requirements)
        for source in spec['stages']:
            entry, evidence = entry_for(source, viewed_path)
            historical_sources.append({'stage': source, 'status': evidence, **{key: entry.get(key) for key in ('kind', 'note', 'recorded_at', 'observations')}})
        for check in spec['checks']:
            results = [(entry_for(source, viewed_path), key) for source, key in check['sources']]
            confirmed = all((entry.get('observations') or {}).get(key) is True and evidence not in ('stale', 'failed', 'pending')
                            for ((entry, evidence), key) in results)
            if check.get('legacy_sources') and all(key not in (entry.get('observations') or {}) for ((entry, _), key) in results):
                legacy = [(entry_for(source, viewed_path), key) for source, key in check['legacy_sources']]
                confirmed = all((entry.get('observations') or {}).get(key) is True and evidence not in ('stale', 'failed', 'pending')
                                for ((entry, evidence), key) in legacy)
            stale = any(evidence == 'stale' for ((entry, evidence), key) in results)
            observations[check['id']] = confirmed
            requirements.append({'id': check['id'], 'kind': 'manual', 'required': True,
                                 'label': check['label'], 'label_en': check['label_en'],
                                 **({'axis': check['axis']} if check.get('axis') else {}),
                                 'status': 'recorded' if confirmed else 'stale' if stale else 'pending'})
        manual = [item for item in requirements if item['kind'] == 'manual']
        historical_status = ('recorded' if manual and all(item['status'] == 'recorded' for item in manual) else
                             'stale' if any(item['status'] == 'stale' for item in manual) else
                             'partial' if any(item['status'] == 'recorded' for item in manual) else 'pending')
        measured, measurement_state = {}, 'pending'
        completion = 'manual'
        if task in ('observation_validation', 'admittance_response'):
            measured = measurements.get(stage_key(task, viewed_path)) or {}
            measurement_state = status(measured, scopes.get(stage_key(task, viewed_path)))
            if measurement_state == 'pending' and task == 'admittance_response':
                legacy = measurements.get(stage_key('admittance_release', viewed_path)) or {}
                if status(legacy, scopes.get(stage_key('admittance_release', viewed_path))) == 'verified':
                    measured, measurement_state = legacy, 'verified'
            completion = 'mixed' if task == 'observation_validation' else 'measured'
            summary = dict(measured.get('summary') or {})
            required_axes = summary.get('required_axes') or (configuration.get('joint_names') if task == 'observation_validation' else []) or []
            missing = [axis for axis in required_axes if axis not in (summary.get('passed_axes') or [])]
            failed_axes = [joint for joint, passed in zip(required_axes, summary.get('pass') or []) if passed not in (True, 1)] if task == 'observation_validation' else summary.get('failed_axes') or []
            if task == 'observation_validation':
                missing = required_axes[len(summary.get('pass') or []):]
            reason = summary.get('binding_error') or summary.get('reason') or ''
            if failed_axes: reason = '独立残差未通过：' + '、'.join(failed_axes) if task == 'observation_validation' else reason
            elif missing and measured: reason = '尚缺关节实测：' + '、'.join(missing)
            elif measurement_state == 'failed' and not reason: reason = '实测记录未满足当前验收条件，请查看逐轴结果'
            summary.update(required_axes=required_axes, missing_axes=missing, failed_axes=failed_axes, reason=reason)
            measured = {**measured, 'summary': summary}
            requirements.append({'id': 'measurement:' + task, 'kind': 'measured', 'required': True,
                'label': '独立外力实测验收通过' if task == 'observation_validation' else '退让、释放回位与稳定段实测通过',
                'label_en': 'Independent effort measurements passed' if task == 'observation_validation' else 'Measured yielding, release return and settling passed',
                'status': measurement_state, 'reason': reason,
                'required_axes': required_axes, 'missing_axes': missing, 'failed_axes': failed_axes})
        if task == 'gravity_verification':
            entry, evidence = entry_for(task, viewed_path)
            proof = (evidence == 'verified' and entry.get('kind') in ('native_gravity_completed', 'operator_and_live'))
            measured, measurement_state = (entry, 'verified') if proof else ({}, 'pending')
            completion = 'confirmation'
            # A still-valid explicit model confirmation survives this UI change.
            requirements = [item for item in requirements if item['kind'] != 'manual']
            requirements.append({'id': 'model_confirmation', 'kind': 'confirmation', 'required': True,
                'label': '重力补偿实测记录有效' if proof else '当前重力模型已确认',
                'label_en': 'Valid measured gravity verification' if proof else 'Current gravity model confirmed',
                'status': 'verified' if proof and gravity_validated else 'recorded' if gravity_validated else 'pending'})
        if task == 'mit_interaction':
            requirements.append({'id': 'gravity_confirmation', 'kind': 'confirmation', 'required': True,
                'label': '当前重力模型已确认', 'label_en': 'Current gravity model confirmed',
                'status': 'recorded' if gravity_validated else 'pending'})
        if task in ('admittance_response', 'csp_interaction'):
            validation = configuration.get('validation') or {}
            for key, zh, en in [('observation_validated', '独立外力验收与加载方向已确认', 'Independent effort acceptance and load direction confirmed'),
                                *([('response_validated', '有限刚度响应已验收', 'Finite-stiffness response accepted')] if task == 'csp_interaction' else [])]:
                requirements.append({'id': key, 'kind': 'permission', 'required': True, 'label': zh, 'label_en': en,
                                     'status': 'verified' if validation.get(key) is True else 'pending'})
        passed = {'configured', 'recorded', 'verified'}
        incomplete = [item for item in requirements if item['required'] and item['status'] not in passed]
        completed = not incomplete
        blockers = [item['label'] + ('：' + item['reason'] if item.get('reason') else '') for item in incomplete if item['kind'] != 'manual']
        blockers_en = [item['label_en'] + (': ' + item['reason'] if item.get('reason') else '') for item in incomplete if item['kind'] != 'manual']
        missing_manual = [item for item in incomplete if item['kind'] == 'manual']
        if missing_manual:
            axes = [item['axis'] for item in missing_manual if item.get('axis')]
            blockers.append('待核对关节：' + '、'.join(axes) if axes else f'尚有 {len(missing_manual)} 项人工检查未确认')
            blockers_en.append('Joints awaiting checks: ' + ', '.join(axes) if axes else f'{len(missing_manual)} operator checks remain')
        actions = [{**target, 'label': zh, 'label_en': en} for zh, en, target in spec['actions']]
        live = 'feedback' if task in ('model_connection', 'startup_hold') else 'observer' if task == 'observation_validation' else 'none'
        if live != 'none':
            requirements.append({'id': 'live:' + live, 'kind': 'live', 'live_kind': live, 'required': False,
                                 'label': '本次连接反馈' if live == 'feedback' else '本次外力观测',
                                 'label_en': 'Current feedback' if live == 'feedback' else 'Current effort observation', 'status': 'session_required'})
        evidence_status = ('verified' if completed and measurement_state == 'verified' else 'recorded' if completed else
                           'stale' if any(item['status'] == 'stale' for item in requirements) else
                           'failed' if measurement_state == 'failed' else 'partial' if historical_status in ('recorded', 'partial') or measurement_state == 'partial' else 'pending')
        notes = list(dict.fromkeys(item['note'] for item in historical_sources if item.get('note')))
        return {'id': task, 'label': spec['label'], 'label_en': spec['label_en'],
            'description': spec['instructions'][0][0], 'description_en': spec['instructions'][0][1],
            'instructions': [item[0] for item in spec['instructions']], 'instructions_en': [item[1] for item in spec['instructions']],
            'control_path': viewed_path, 'view_only': viewed_path != path, 'required': required,
            'configuration_ready': configuration_ready, 'completed': completed, 'evidence_status': evidence_status,
            'completion_kind': completion, 'requirements': requirements, 'blockers': blockers, 'blockers_en': blockers_en,
            'actions': actions, 'action': actions[0] if actions else None,
            'record_stage': task if spec['checks'] and task != 'gravity_verification' else None,
            'review_stages': list(spec['stages']),
            'source': {'historical': {'status': historical_status, 'observations': observations, 'note': '\n'.join(notes), 'records': historical_sources},
                       'measurement': {'status': measurement_state, **{key: measured.get(key) for key in ('kind', 'recorded_at', 'summary', 'artifact_path')}},
                       'live': {'kind': live, 'status': 'session_required'}}}

    paths = {choice: {'label': '阻抗 + MIT' if choice == 'MIT' else '导纳 + CSP',
                     'label_en': 'Impedance + MIT' if choice == 'MIT' else 'Admittance + CSP',
                     'steps': [step(task, True, choice) for task in ids]} for choice, ids in PATH_STEPS.items()}
    for view in paths.values(): view['completed'] = all(item['completed'] for item in view['steps'])
    groups = []
    for key, label, en, required, ids in (
            ('foundation', '共同基础', 'Common foundation', True, FOUNDATION),
            ('path', '路径专项', 'Path-specific commissioning', True, PATH_STEPS[path]),
            ('optional', '可选提升', 'Optional improvements', False, OPTIONAL)):
        steps = paths[path]['steps'] if key == 'path' else [step(task, required, path) for task in ids]
        groups.append({'id': key, 'label': label, 'label_en': en, 'required': required,
                       'completed': all(item['completed'] for item in steps), 'steps': steps})
    return {'selected_path': path, 'completed': all(group['completed'] for group in groups if group['required']),
            'groups': groups, 'paths': paths}
