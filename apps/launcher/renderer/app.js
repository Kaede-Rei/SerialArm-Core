'use strict';
const api = window.serialArm;
const words = {
    ros2: ['ROS2', 'ROS2'],
    workspace: ['SerialArm Studio', 'SerialArm Studio'], subtitle: ['串联机械臂配置与运行', 'Serial manipulator workspace'], subline: ['控制 · 动力学 · 交互', 'Control · Dynamics · Interaction'],
    navigation: ['工作流', 'WORKFLOW'], application: ['应用', 'APPLICATION'], robot: ['机械臂与连接', 'Manipulator & connection'], run: ['运行', 'Run'], diagnostics: ['诊断', 'Diagnostics'], settings: ['设置', 'Settings'],
    ready: ['就绪', 'Ready'], unavailable: ['不可用', 'Unavailable'], connected: ['安装已就绪', 'Installation ready'], notInstalled: ['尚未安装', 'Not installed'], idle: ['空闲', 'Idle'], running: ['运行中', 'Running'], stopping: ['正在停止', 'Stopping'], exited: ['已退出', 'Exited'],
    robotTitle: ['选择机械臂并配置连接', 'Choose a manipulator and configure its connection'], robotDesc: ['使用内置或外部 Profile，将设备设置作为本次运行的覆盖值', 'Use a built-in or external profile and set connection overrides for this session'],
    profile: ['机械臂 Profile', 'Manipulator Profile'], profileSub: ['引用机械臂的模型、控制与硬件配置', 'References the model, control and hardware configuration'], source: ['配置来源', 'Profile source'], builtin: ['内置 Profile', 'Built-in profiles'], external: ['外部 Profile 文件', 'External profile file'], profileFile: ['Profile 文件路径', 'Profile file path'], browse: ['选择文件', 'Browse'], refresh: ['刷新', 'Refresh'], profileName: ['机械臂', 'Manipulator'],
    connection: ['连接设置', 'Connection'], connectionSub: ['本次连接使用的覆盖值', 'Overrides for this connection'], serial: ['串口设备', 'Serial device'], automatic: ['使用 Profile 默认值', 'Use profile default'], baudrate: ['波特率', 'Baudrate'], bus: ['Bus 名称', 'Bus name'], resourcePaths: ['资源搜索路径', 'Resource search paths'], addRoot: ['添加目录', 'Add directory'], rootsHint: ['多个目录用冒号分隔；支持下游工作区或安装前缀', 'Separate roots with a colon; downstream workspaces and install prefixes are supported'], overrideHint: ['留空使用 Profile 默认值', 'Leave blank to use the profile default'], inspect: ['检查配置', 'Inspect configuration'], toRun: ['选择运行模式', 'Choose run mode'],
    summary: ['配置概览', 'Profile overview'], summarySub: ['实际资源与执行器写入状态', 'Resolved resources and actuator write state'], hardwarePlugin: ['硬件驱动', 'Hardware driver'], writeEnabled: ['执行器写入', 'Actuator writes'], enabled: ['已开启', 'Enabled'], disabled: ['已关闭', 'Disabled'], unknown: ['未知', 'Unknown'], controllers: ['控制器', 'Controllers'], noProfile: ['请选择一个 Profile', 'Select a profile'], status: ['环境状态', 'Environment status'], statusSub: ['只读检查，不连接执行器', 'Read-only checks, no actuator connection'], resources: ['Profile 资源', 'Profile resources'], installation: ['安装状态', 'Installation'], nativeTerminal: ['C++ Terminal', 'C++ Terminal'],
    runtimeTitle: ['启动适配层', 'Start adapters'], runtimeDesc: ['按实际安装能力启动 Standalone Terminal、ROS2 Hardware 或 MoveIt', 'Launch installed Standalone Terminal, ROS2 Hardware or MoveIt adapters'],
    model: ['模型查看', 'Model viewer'], modelDesc: ['查看模型、坐标系与动力学参数', 'Inspect model, frames and dynamics'], terminal: ['Terminal', 'Terminal'], terminalDesc: ['进入 C++ 交互终端，调试机械臂', 'Open the C++ interactive terminal'], hardware: ['Hardware', 'Hardware'], hardwareDesc: ['运行 ros2_control 和 Controllers', 'Run ros2_control and controllers'], moveit: ['MoveIt', 'MoveIt'], moveitDesc: ['启动规划、硬件和 RViz', 'Start planning, hardware and RViz'], offline: ['离线', 'Offline'], real: ['真实硬件', 'Real hardware'],
    launch: ['启动', 'Launch'], stop: ['停止', 'Stop'], force: ['强制结束', 'Force stop'], clear: ['清空显示', 'Clear view'], copyAll: ['复制全部', 'Copy all'], paste: ['粘贴', 'Paste'], logs: ['运行终端', 'Runtime terminal'], terminalHint: ['拖选后 Ctrl+Shift+C 复制，Ctrl+Shift+V 粘贴，右键打开菜单；Ctrl+C 仍用于中断', 'Select text and press Ctrl+Shift+C to copy, Ctrl+Shift+V to paste, or right-click for the menu; Ctrl+C still interrupts'], noSession: ['等待启动会话', 'Waiting for a session'], modeMissing: ['此模式缺少安装组件或 Profile 资源，请查看诊断', 'This mode needs installed components or profile resources; Check diagnostics'], selectedRobot: ['当前机械臂', 'Selected manipulator'],
    diagTitle: ['连接与诊断', 'Connection & diagnostics'], diagDesc: ['连接覆盖参数、环境检查、实时执行器状态和运行日志', 'Connection overrides, environment checks, live actuator state and runtime logs'], checkNow: ['重新检查', 'Check again'], checks: ['检查结果', 'Check results'], checkSub: ['检查配置文件与安装组件', 'Check configuration files and installed components'], pass: ['通过', 'Pass'], fail: ['未通过', 'Failed'], installHelp: ['安装与环境', 'Installation & environment'], installHelpSub: ['从仓库根目录执行', 'Run from the repository root'], installHint: ['安装器会按用途配置组件，GUI 自动加载安装后的环境', 'The installer configures selected components; the GUI loads their environment'], validationHint: ['检查配置与驱动资源，连接后核对真实电机反馈', 'Check configuration and driver resources, then connect to inspect motor feedback'],
    settingsTitle: ['工作台设置', 'Workspace settings'], settingsDesc: ['偏好会自动保存，不影响正在运行的会话', 'Preferences are saved automatically without restarting an active session'], appearance: ['外观', 'Appearance'], appearanceSub: ['选择工作台主题', 'Choose a workspace theme'], system: ['跟随系统', 'System'], dark: ['深色', 'Dark'], light: ['浅色', 'Light'], language: ['语言', 'Language'], languageSub: ['随时切换界面语言', 'Switch the interface language at any time'], prefsNote: ['语言、主题和窗口偏好会自动保存；机械臂每次从启动入口明确选择', 'Language, theme, and window preferences are saved; choose the robot explicitly from the start workflow each time'],
    realTitle: ['启动真机会话', 'Start a hardware session'], realDesc: ['该入口可能连接并驱动真实执行器，请核对下面的运行配置', 'This entry point can connect to and drive real actuators; Review the configuration below'], confirm: ['已检查运动空间，并核对执行器写入状态', 'I have checked the motion area and actuator write state'], cancel: ['取消', 'Cancel'], confirmLaunch: ['确认并启动', 'Confirm and launch'], forceTitle: ['强制结束运行进程', 'Force stop the session'], forceDesc: ['强制结束会中断清理流程，请先支撑机械臂并确认执行器状态', 'Force stop interrupts cleanup; support the arm and check the actuators first'], error: ['操作失败', 'Operation failed'], fileHint: ['选择 robot_profiles.yaml', 'Choose robot_profiles.yaml'], saved: ['已保存', 'Saved'], core: ['Core YAML', 'Core YAML'], description: ['模型文件', 'Model file'], ros2_control: ['ros2_control Xacro', 'ros2_control Xacro'], device: ['设备权限', 'Device access'], backendMissing: ['运行后端不可用', 'Backend unavailable'], backend: ['运行后端', 'Runtime backend'],
    modelTitle: ['模型查看', 'Model viewer'], modelPageDesc: ['查看机械臂结构、坐标系、关节运动与动力学参数', 'Inspect the manipulator structure, frames, joint motion and dynamics'],
    workbenchNav: ['控制工作台', 'Control workspace'], workbenchTitle: ['控制工作台', 'Control workspace'], workbenchDesc: ['运动控制、示教回放、交互调试与运行监视', 'Motion control, teaching playback, interaction tuning and live monitoring'],
    geometryNav: ['几何标定', 'Geometry calibration'], geometryTitle: ['几何标定', 'Geometry calibration'], geometryDesc: ['固定点几何标定与定点重复到位测试', 'Fixed-point geometry calibration and visual repeatability test'],
    calibrationNav: ['动力学标定', 'Dynamics calibration'], calibrationTitle: ['动力学标定', 'Dynamics calibration'], calibrationDesc: ['重力补偿验证、拖拽示教、执行器增益自动标定与模型辨识', 'Gravity verification, teaching, automatic actuator gain calibration and model identification'],
    parametersNav: ['参数配置', 'Parameter configuration'], adapterNav: ['启动适配层', 'Start adapters'], connectionDiagNav: ['连接与诊断', 'Connection & diagnostics'], monitorTab: ['运行监视', 'Live monitor'],
    controlTab: ['控制', 'Control'], replayTab: ['示教回放', 'Teach & replay'], tuningTab: ['交互调试', 'Interaction tuning'], calibrationTab: ['标定', 'Calibration'], runtimeDiagTab: ['诊断', 'Diagnostics'], startWorkbench: ['连接工作台', 'Connect workspace'], stopWorkbench: ['停放并断开', 'Park & disconnect'], activateRobot: ['使能', 'Activate'], deactivateRobot: ['立即失能', 'Immediate disable'], holdRobot: ['当前位置保持', 'Hold current'], parkRobot: ['停放并失能', 'Park & disable'], clearFault: ['清除故障', 'Clear fault'], compliantRecovery: ['柔性恢复', 'Compliant recovery'], rigidRecovery: ['返回刚性保持', 'Return rigid hold'], apply: ['应用', 'Apply'], reset: ['重置', 'Reset'], previewSave: ['预览保存差异', 'Preview save diff'], saveConfig: ['确认保存', 'Save confirmed changes'], exportData: ['导出会话数据', 'Export session data'], actual: ['实际', 'Actual'], target: ['目标', 'Target'], delta: ['相对增量', 'Relative delta'], speedScale: ['速度比例', 'Speed scale'], executeAbsolute: ['执行绝对目标', 'Execute absolute target'], executeRelative: ['执行相对目标', 'Execute relative target'], observerMode: ['Observer 模式', 'Observer mode'], staticCalibration: ['静态残差标定', 'Static residual calibration'], staticValidation: ['独立静态验证', 'Independent static validation'], frictionCalibration: ['双向摩擦标定', 'Bidirectional friction calibration'], capturePose: ['采集当前姿态', 'Capture current pose'], finishFit: ['完成拟合', 'Finish fit'], cancelTask: ['取消任务', 'Cancel task'], startRecording: ['开始示教记录', 'Start demonstration'], stopRecording: ['结束示教记录', 'Stop demonstration'], startReplay: ['确认安全并开始回放', 'Confirm safety & replay'], feedbackAge: ['反馈年龄', 'Feedback age'], commandState: ['命令状态', 'Command state'],
    modelCalibration: ['重力模型校正', 'Gravity model calibration'], modelCalibrationDesc: ['示教后采集双向静态与双向双速度数据，保存后统一离线分析与审核', 'Capture bidirectional static and two-speed motion data, then identify and review offline'], modelCalibrationStart: ['开始拖动示教', 'Start demonstration'], modelCalibrationStopTeach: ['结束示教', 'Stop demonstration'], modelCalibrationConfirmReplay: ['确认松手并开始自动回放', 'Confirm release and start replay'], pauseTask: ['暂停', 'Pause'], resumeTask: ['继续', 'Resume'], applyCandidate: ['应用候选重力', 'Apply candidate gravity'], restoreCandidate: ['恢复校正前运行配置', 'Restore pre-calibration runtime configuration'], saveCandidate: ['保存候选', 'Save candidate'], exportCandidate: ['导出候选 URDF', 'Export candidate URDF'], loadRecord: ['加载任务记录', 'Load task record'], recomputeRecord: ['离线重算', 'Recompute offline'],
    impedanceMode: ['阻抗模式', 'Impedance mode'], modelFeedforward: ['模型前馈', 'Model feedforward'], jointCommand: ['关节命令', 'Joint command'], joint: ['关节', 'Joint'], admittanceParameters: ['导纳 M / D / K', 'Admittance M / D / K'], gravityScale: ['重力补偿比例', 'Gravity scale'], noChanges: ['没有变化', 'No changes'],
    sourceLive: ['在线会话', 'Live session'], sourceOffline: ['离线记录', 'Offline record'], sourceWaiting: ['离线', 'Offline'], poseBudget: ['姿态数量上限', 'Pose budget'], validationFraction: ['留出验证比例', 'Validation fraction'], regularizationStrength: ['正则化起点（按姿态自动交叉验证）', 'Regularization baseline (pose-group CV)'], svdRelativeThreshold: ['SVD 相对阈值', 'SVD relative threshold'], minimumInformationScore: ['最小信息量', 'Minimum information score'], taskIdentifier: ['任务标识', 'Task identifier'], progressLabel: ['任务进度', 'Progress'], trajectorySamples: ['轨迹采样点', 'Trajectory samples'], trainingValidationGroups: ['训练 / 留出姿态组', 'Training / validation groups'], validStaticSamples: ['有效静态样本', 'Valid static samples'], acceptedDroppedFrames: ['已接收 / 丢弃原始帧', 'Accepted / dropped raw frames'],
    currentUrdfRms: ['原模型 RMS', 'Source model RMS'], currentScaleRms: ['当前 gravity_scale RMS', 'Current gravity_scale RMS'], candidateRms: ['候选 RMS', 'Candidate RMS'], candidateP99: ['候选 P99', 'Candidate P99'], candidateMaximum: ['候选最大误差', 'Candidate maximum'], noiseRms: ['噪声 RMS', 'Noise RMS'], staticPassed: ['静态验证通过', 'Static validation passed'], staticFailed: ['静态验证未通过', 'Static validation failed'], frictionPassed: ['摩擦验证通过', 'Friction validation passed'], frictionFailed: ['摩擦验证未通过', 'Friction validation failed'], dynamicsNotIdentified: ['完整惯性参数尚未验证', 'Full inertial parameters not yet validated'], numericalRank: ['数值秩', 'Numerical rank'], candidateComparison: ['候选模型比较', 'Candidate model comparison'], candidateScope: ['静态重力校正 · 质心处惯量沿用原模型', 'Static gravity correction · source inertia-at-COM retained'], fixedMass: ['固定质量', 'Fixed mass'], currentCom: ['当前 COM', 'Current COM'], candidateCom: ['候选 COM', 'Candidate COM'], comOffset: ['偏移', 'Offset'], observableAxes: ['可观测轴', 'Observable axes'], restoreSavedConfig: ['恢复校正前保存配置', 'Restore configuration from before calibration save'], taskDirectory: ['任务目录', 'Task directory'], singleCalibrationTools: ['单项标定工具', 'Individual calibration tools'], staticCalibrationDesc: ['逐姿态采集并拟合静态残差', 'Capture poses and fit static residuals'], start: ['开始', 'Start'], startValidation: ['开始验证', 'Start validation'], taskType: ['任务类型', 'Task type'], taskPhase: ['任务阶段', 'Task phase'], errorLabel: ['错误', 'Error'], replayReadyHint: ['示教路径已冻结', 'Demonstration path is frozen'], samplePoints: ['采样姿态', 'Sampling poses'], replayRate: ['回放速度比例', 'Replay rate'], estimatedDuration: ['预计自动阶段', 'Estimated automatic stage'], durationLimit: ['任务上限', 'Task limit'], releaseSafetyHint: ['请检查机械臂路径、负载、线缆和周围空间，并完全松手后确认回放', 'Check the robot path, load, cables, and surrounding space, then fully release the robot before confirming replay'],
    telemetryTitle: ['实时遥测', 'Live telemetry'], frameTitle: ['坐标系', 'Frame'], dynamicsTitle: ['动力学', 'Dynamics'], jointActuatorTitle: ['关节 / 执行器', 'Joint / actuator'], onlineLabel: ['在线', 'Online'], enabledLabel: ['使能', 'Enabled'], positionLabel: ['位置', 'Position'], quaternionLabel: ['四元数 xyzw', 'Quaternion xyzw'], gravityLabel: ['重力项', 'Gravity'], gravityCompLabel: ['重力补偿', 'Gravity compensation'], coriolisLabel: ['科氏项', 'Coriolis'], inverseDynamicsLabel: ['逆动力学', 'Inverse dynamics'], actualTelemetry: ['实际状态 · 实时反馈', 'Actual state · live telemetry'], offlineWaiting: ['离线状态 · 等待工作台连接', 'Offline · waiting for workspace connection'], modelNotLoaded: ['模型尚未加载', 'Model is not loaded'], processing: ['正在处理...', 'Processing...'], stopTeaching: ['结束示教', 'Stop demonstration'],
    modelLoad: ['加载模型', 'Load model'], modelReload: ['重新加载', 'Reload'], modelTree: ['模型树', 'Model tree'], modelViewport: ['模型视图', 'Model view'], modelInspector: ['参数检查器', 'Inspector'],
    visualLayer: ['Visual', 'Visual'], collisionLayer: ['Collision', 'Collision'], linkFrameLayer: ['Link Frame', 'Link frames'], jointAxisLayer: ['Joint Axis', 'Joint axes'], comLayer: ['质心', 'COM'], inertiaLayer: ['惯性', 'Inertia'], labelsLayer: ['标签', 'Labels'], allLayers: ['全选图层', 'All layers'], noLayers: ['全不选', 'Clear layers'], showAllObjects: ['全部显示', 'Show all'], hideAllObjects: ['全部隐藏', 'Hide all'], resetView: ['复位视角', 'Reset view'],
    offlinePreview: ['模型预览', 'Model preview'], jointPreview: ['关节预览', 'Joint preview'], urdfInertial: ['URDF &lt;inertial&gt;', 'URDF &lt;inertial&gt;'], coreReducedInertia: ['Core 约简后惯性', 'Core reduced inertia'], effectiveInertia: ['Core 有效惯性', 'Core effective inertia'],
    modelUnavailable: ['模型组件不可用，请检查 Standalone 安装', 'Model component unavailable; check the Standalone installation'], noModel: ['尚未加载模型', 'Model is not loaded'],
    startQuestion: ['准备从哪里开始', 'Where do you want to start?'], startDesc: ['选择最符合当前机械臂状态的入口，工作台会继续引导下一步', 'Choose the entry that matches the robot state; the workspace will guide the next step'],
    newDescription: ['新建机械臂 Profile', 'New manipulator Profile'], newDescriptionDesc: ['从 SerialArm Description 包、URDF 或 Xacro 创建新的 Profile', 'Create a Profile from a SerialArm Description package, URDF, or Xacro'],
    continueProfile: ['继续配置机械臂', 'Continue configuring a manipulator'], continueProfileDesc: ['打开仍需检查、校准、标定或验证的 Profile', 'Open a Profile that still needs inspection, calibration, or validation'],
    useProfile: ['使用机械臂', 'Use a manipulator'], useProfileDesc: ['查看配置并连接工作台', 'Review configuration and connect the workspace'],
    continueLibrary: ['待配置的 Profile', 'Profiles to configure'], readyLibrary: ['配置有效', 'Configuration valid'], newModelCheck: ['模型检查', 'Model check'], newModelHint: ['检查 URDF、关节限位、控制链与模型资源', 'Check the URDF, joint limits, control chain and model resources'], newHardwareHint: ['硬件驱动、总线、通信协议及执行器映射请在下一步 Profile 配置中填写', 'Configure hardware, buses, communication and actuators in the next Profile setup step'], wizardBack: ['上一步', 'Back'], wizardNext: ['下一步', 'Continue'], profileLibrary: ['Profile Library', 'Profile Library'], manageProfiles: ['机械臂列表', 'Manipulators'], importExternalProfile: ['导入外部 Profile', 'Import external Profile'], removeFromLibrary: ['从 Library 移除', 'Remove from Library'],
    configuring: ['配置中', 'Configuring'], pendingValidation: ['待验证', 'Pending validation'], needsRevalidation: ['需要重新验证', 'Needs revalidation'], usable: ['配置有效', 'Configuration valid'],
    descriptionImport: ['Description 导入', 'Description import'], selectDescription: ['选择 Description 文件', 'Choose Description'], descriptionSource: ['Description 来源', 'Description source'], analyzeDescription: ['分析 Description', 'Analyze Description'],
    controlChain: ['控制链', 'Controlled chain'], hardwareMapping: ['硬件映射', 'Hardware mapping'], createProfile: ['生成 Profile', 'Create Profile'], addToLibrary: ['加入 Profile Library', 'Add to Profile Library'], notNow: ['暂不加入', 'Not now'],
    baseFrame: ['Base Frame', 'Base Frame'], toolFrame: ['Tool Frame', 'Tool Frame'], controlledJoints: ['受控关节', 'Controlled joints'], profileId: ['Profile 标识', 'Profile id'], destination: ['保存位置', 'Destination'], chooseDirectory: ['选择目录', 'Choose directory'],
    actuatorName: ['执行器名称', 'Actuator name'], motorId: ['Motor ID', 'Motor ID'], masterId: ['Master ID', 'Master ID'], motorType: ['电机型号', 'Motor type'], hardwareDriver: ['硬件驱动', 'Hardware driver'],
    gravityStage: ['重力补偿验证', 'Gravity verification'],
    readiness: ['机械臂准备状态', 'Manipulator readiness'], readinessDesc: ['检查配置与连接，先完成重力补偿验证和增益标定', 'Check configuration and connection, then verify gravity compensation and calibrate gains'], nextStep: ['下一步', 'Next step'], confirmStage: ['确认当前阶段完成', 'Confirm current stage'],
    descriptionStage: ['描述模型', 'Description'], profileSetupStage: ['Profile 配置', 'Profile Setup'], modelCheckStage: ['模型检查', 'Model check'], bringupStage: ['真机检查', 'Hardware bring-up'], mappingStage: ['映射校准', 'Mapping calibration'], geometryStage: ['几何标定', 'Geometry calibration'], dynamicsStage: ['动力学校准', 'Dynamics calibration'], validationStage: ['验证', 'Validation'], readyStage: ['可使用', 'Ready'],
    expertTools: ['专业工具', 'Expert tools'], backHome: ['返回开始页', 'Back to start'], profileSetup: ['参数配置', 'Parameter configuration'], editProfile: ['编辑 Profile', 'Edit Profile'], continueAction: ['继续配置', 'Continue configuration'], saveProfile: ['保存 Profile', 'Save Profile'], profileSetupSubtitle: ['编辑机械臂默认参数，保存后重新连接生效', 'Edit saved defaults; reconnect to apply them'], profileManagedHint: ['从机械臂列表移除后，磁盘文件仍保留', 'Removing a manipulator from the list keeps its files on disk'],
};
const paths = { workbench: '<path d="M4 5h16v14H4zM8 9h8M8 13h5M6 3v4M18 3v4"/>', robot: '<path d="M5 21v-4h14v4M8 17l-3-5 4-3 4 3-2 5M9 9l3-5 6 3-5 5M18 7l2-3M2 21h20"/>', run: '<path d="m8 5 11 7-11 7Z"/>', diagnostics: '<path d="M10 3h4v4h-4zM5 8h14v12H5zM9 12h6M9 16h3"/>', settings: '<path d="M4 7h16M4 17h16M8 4v6M16 14v6"/>', plug: '<path d="M8 3v4M16 3v4M6 7h12v4a6 6 0 0 1-12 0zM12 17v4"/>', folder: '<path d="M3 6h7l2 2h9v12H3z"/>', model: '<path d="m12 3 9 5v9l-9 5-9-5V8zM3 8l9 5 9-5M12 13v9"/>', terminal: '<path d="m5 7 5 5-5 5M13 17h6"/>', hardware: '<path d="M7 7h10v10H7zM4 9h3M4 15h3M17 9h3M17 15h3M9 4v3M15 4v3M9 17v3M15 17v3"/>', moveit: '<path d="M4 19V5h16M4 19h16M8 15l4-7 6 5M7 15h2M17 13h2"/>', refresh: '<path d="M20 7v5h-5M4 17v-5h5M5 8a8 8 0 0 1 13-3l2 2M4 17l2 2a8 8 0 0 0 13-3"/>', check: '<path d="m5 12 4 4L19 6"/>', close: '<path d="m6 6 12 12M18 6 6 18"/>', minus: '<path d="M5 12h14"/>', max: '<rect x="5" y="5" width="14" height="14" rx="1"/>', sun: '<circle cx="12" cy="12" r="4"/><path d="M12 2v2M12 20v2M2 12h2M20 12h2M5 5l1 1M18 18l1 1M5 19l1-1M18 6l1-1"/>', stop: '<rect x="6" y="6" width="12" height="12" rx="1"/>' };
Object.assign(paths, { plus: '<path d="M12 5v14M5 12h14"/>', search: '<circle cx="10.5" cy="10.5" r="7.5"/><path d="m16 16 5 5"/>', more: '<path d="M5 12h.01M12 12h.01M19 12h.01"/>', chevron: '<path d="m5 9 7 7 7-7"/>', arrow: '<path d="m9 5 7 7-7 7"/>', trash: '<path d="M3 6h18M9 6V3h6v3M6 6l1 15h10l1-15M10 10v7M14 10v7"/>' });
const robotArtwork = () => '<img class="association-icon" src="brand/arm.png" alt="" aria-hidden="true">';
const associationArtwork = (className = '') => `<img class="association-mark ${className}" src="brand/association.png" alt="${state.language === 'en' ? 'Association logo' : '协会标志'}">`;
const icon = name => `<svg viewBox="0 0 24 24" aria-hidden="true">${paths[name] || paths.robot}</svg>`;
const esc = value => String(value ?? '').replace(/[&<>"']/g, c => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;', "'": '&#39;' }[c]));
const $ = id => document.getElementById(id);
let state = { workbenchEpoch: 0, workbenchIdentity: null, page: 'start', previousPage: 'start', language: 'zh-CN', theme: 'system', mode: 'terminal', source: 'builtin', entryIntent: '', libraryQuery: '', libraryFilter: 'all', library: [], libraryPath: '', activeLibraryId: '', readiness: null, descriptionAnalysis: null, descriptionStep: 1, createdProfileWizard: false, descriptionDraft: { source: '', profile: '', destination: '', base_frame: '', tool_frame: '', joint_names: [], hardware_plugin: 'serial_arm_hardware_damiao', bus: 'main_can', device: '/dev/ttyACM0', baudrate: '921600', actuators: [] }, profileEditor: null, config: { profile: '', profile_file: '', serial_port: '', baudrate: '', bus: '', resource_paths: '' }, info: null, status: {}, profiles: [], session: { state: 'idle' }, busy: false, backendError: '', inspecting: 0, modelData: null, modelLoading: false, modelPositions: [], modelSelection: '', modelLayers: { visual: true, collision: false, linkFrames: false, jointAxes: true, com: false, inertia: false, labels: false }, modelInertiaSource: 'urdf', modelVisibility: { links: {}, linkFrames: {}, joints: {} }, workbenchTab: 'control', capabilityProfile: null, capabilityProfileError: '', telemetry: null, telemetryHistory: [], admittance: null, tuneDraft: null, workbenchTargets: [], speedScale: 0.2, commandState: 'idle', pendingTarget: null, savePreview: null, lastAction: '', calibrationFlow: '', selectedFrame: '', telemetryReceivedAt: 0, workbenchPending: { kind: '', value: '' }, blockingBusy: false, modelCalibrationOffline: null, modelCalibrationRecords: [], modelCalibrationDirectory: '', modelCalibrationSavePreview: null, modelCalibrationExport: null, modelCalibrationObjective: 'bias_separated', modelCalibrationCandidateSource: 'online', modelCalibrationSaved: null, modelCalibrationFrictionExport: null, modelCalibrationDisplayScale: 1, modelCalibrationShowUnchanged: false, modelCalibrationPreviewAlignment: null, calibrationEntry: 'teaching', calibrationToolRecords: { teaching: [], gain: [], gravity: [], dynamics: [] }, calibrationToolSelected: { teaching: null, gain: null, gravity: null, dynamics: null }, calibrationRuntimeCandidates: null, gainCandidateLoadPending: '', gainCandidateLoadAttempted: '', gainCandidateLoadError: '', gainMode: 'all', idimReport: null, idimAudit: null, idimScope: 'global', idimLocked: [], idimPreview: true, idimLoadDescription: '', idimEstimator: 'WLS', idimCutoff: 5, idimValidationDirectory: '', idimFixedJointText: '', advancedTarget: 'mass_com', advancedAssessment: null, advancedReport: null, advancedLocked: [], advancedProgress: '', autoAssessedTaskId: '', teachTrajectories: [], teachSelectedId: '', teachLoadedId: '', teachRate: 0.3, teachRepeats: 30, teachDirection: 'forward', teachReplayBehavior: null, teachGainPurpose: 'RIGID', teachPreviewDirection: null, teachStorageDirectory: '', teachLastFilePath: '', teachRuntime: null, geometryTab: 'fixed', geometryState: null, geometryPurpose: 'diagnostic', geometryFrame: '', geometryRms: 2, geometryMax: 4, geometryCycles: 20, geometryDwell: 3, geometryRate: 0.2, geometryReplayBehavior: null, geometryGainPurpose: 'RIGID', geometryTrajectories: [], geometrySelectedId: '', geometryLoadedId: '', geometryContactReady: false, geometryRepeatSupported: null, geometryRepeatError: '', ghostRequestKey: '', modelCalibrationOptions: { load_description: '', mode: 'global', locked_links: [], pose_budget: 12, validation_fraction: 0.25, regularization: 0.01, svd_relative_threshold: 0.0001, minimum_information_score: 0.0001 } };
Object.assign(state, {speedInputRaw:null, speedInputError:'', pendingMotionId:null, modelIdentity:null, calibrationHistoryPages:{}, calibrationCandidateRounds:{}, calibrationCandidateSelection:{}, geometryTcpPreview:null, responseTolerances:{}});
Object.assign(state, {driverInspection:null,controlSettingsOpen:true,teachPreviewFraction:0,teachPosePreview:null,readinessSections:{}});
const profileWorkspaceDefaults = structuredClone(state);
let term, fit, outputBuffer = '', modelView = null, modelModule = null;
const t = key => (words[key] || [key, key])[state.language === 'en' ? 1 : 0];
function commandStateText(value) {
    const labels = {
        idle: ['空闲', 'Idle'], edited: ['已编辑，等待提交', 'Edited, awaiting submission'], submitting: ['正在提交', 'Submitting'], accepted: ['命令已接受', 'Command accepted'], running: ['运动中', 'Moving'], complete: ['目标已到达', 'Target reached'], failed: ['运动失败', 'Motion failed'], interrupted: ['运动已中断', 'Motion interrupted'],
    };
    const text = labels[value] || [value || '—', value || '—'];
    return state.language === 'en' ? text[1] : text[0];
}
function calibrationTaskKindText(value) {
    const labels = {
        none: ['无任务', 'No task'], static: ['静态残差标定', 'Static residual calibration'], validation: ['独立静态验证', 'Independent static validation'], friction_recording: ['摩擦示教记录', 'Friction demonstration recording'], friction_ready: ['等待摩擦回放', 'Waiting for friction replay'], friction_replaying: ['摩擦回放与拟合', 'Friction replay and fitting'], friction: ['摩擦参数标定', 'Friction calibration'],
    };
    const text = labels[value] || [value || '—', value || '—'];
    return state.language === 'en' ? text[1] : text[0];
}
function calibrationTaskPhaseText(value) {
    const labels = {
        idle: ['未启动', 'Idle'], collecting: ['采集中', 'Collecting'], recording: ['示教记录中', 'Recording demonstration'], ready: ['等待下一步', 'Ready for next step'], replaying: ['回放采集中', 'Collecting replay data'], fitting: ['拟合中', 'Fitting'], validating: ['验证中', 'Validating'], complete: ['已完成', 'Complete'], cancelled: ['已取消', 'Cancelled'], failed: ['失败', 'Failed'],
    };
    const text = labels[value] || [value || '—', value || '—'];
    return state.language === 'en' ? text[1] : text[0];
}
function localizedError(raw) {
    return window.serialArmErrorI18n?.translate(raw, state.language) ?? String(raw ?? '');
}
const priorityErrorQueue = [];
let priorityErrorOpen = false;
let activePriorityError = null;
let lastRuntimeFaultKey = '';
let lastTeachReplayErrorKey = '';
const reportedCalibrationErrors = new Set();
// All user-facing failures use the existing centred native modal; the
// priority queue prevents a telemetry fault from being hidden by an operation.
function normalizeDialogError(source, kind = 'operation') {
    const detail = source?.serialarm || (source && typeof source === 'object' && source.code ? source : null);
    if (detail?.severity === 'critical' || (detail?.category === 'safety' && detail?.code !== 'REFERENCE_LIMIT_REJECTED')) kind = 'fault';
    if (detail?.code === 'REFERENCE_LIMIT_REJECTED' && state.telemetry?.robot_state === 'FAULT') kind = 'fault';
    const sourceText = typeof source === 'string' ? source : (source?.message || String(source ?? ''));
    const technical = detail?.technical || (typeof source?.stack === 'string' ? source.stack : sourceText);
    const rawSummary = detail?.message || sourceText || technical || 'Unknown error';
    const defaultTitle = kind === 'fault' ? busyText('机械臂安全故障', 'Robot safety fault')
        : kind === 'replay' ? busyText('示教回放失败', 'Trajectory playback failed') : t('error');
    const title = kind === 'fault' ? defaultTitle : (detail?.title || defaultTitle);
    const summary = detail?.message && /[\u3400-\u9fff]/.test(detail.message) ? detail.message : localizedError(detail?.message || rawSummary);
    const advice = detail?.advice || (kind === 'fault'
        ? busyText('请先确认机械臂、供电与执行器状态，关闭弹窗不会恢复运动或自动清除故障', 'Inspect the robot and hardware. Dismissing does not recover motion or clear faults')
        : busyText('查看具体原因，修正后重试', 'Review the cause, correct it and retry'));
    const metadata = detail ? [
        detail.code && `code: ${detail.code}`,
        detail.operation && `operation: ${detail.operation}`,
        detail.diagnostic_id && `diagnostic_id: ${detail.diagnostic_id}`,
        ...Object.entries(detail.details || {}).map(([name, value]) => `${name}: ${typeof value === 'object' ? JSON.stringify(value) : String(value)}`)
    ].filter(Boolean).join('\n') : '';
    const raw = [metadata, technical, (!technical && sourceText) ? sourceText : ''].filter(Boolean).join('\n\n') || '未收到后端技术诊断，请检查运行日志';
    return {
        kind, title, summary, advice, raw, action:detail?.action || null, gainEvidence:detail?.gain_evidence || null,
        id: detail?.diagnostic_id || '',
        key: detail?.diagnostic_id || `${kind}:${rawSummary}:${technical}`
    };
}
function showNextPriorityError() {
    const layer = $('priority-error-overlay');
    if (!layer || priorityErrorOpen || !priorityErrorQueue.length) return;
    activePriorityError = priorityErrorQueue.shift();
    priorityErrorOpen = true;
    paintPriorityError();
}
function paintPriorityError() {
    const entry = activePriorityError;
    const layer = $('priority-error-overlay');
    if (!entry || !layer) return;
    const confirm = $('confirm-dialog');
    if (confirm?.open) confirm.close();
    setOperationOverlay('');
    setCalibrationTaskModalState(false);
    const cal = $('model-calibration-overlay');
    if (cal) { cal.hidden = true; cal.setAttribute('aria-hidden', 'true'); }
    $('priority-error-title').textContent = entry.title;
    $('priority-error-detail').textContent = entry.summary;
    paintGainFailureEvidence(entry);
    $('priority-error-guidance').textContent = entry.advice;
    const kicker = $('priority-error-kicker');
    if (kicker) kicker.textContent = entry.kind === 'fault' ? 'SERIALARM · SAFETY FAULT' : 'SERIALARM · ERROR';
    const technical = $('priority-error-raw');
    if (technical) technical.textContent = entry.raw;
    const expansion = $('priority-error-technical');
    if (expansion) expansion.open = false;
    const copy = $('priority-error-copy');
    if (copy) copy.onclick = () => api.clipboard.write([entry.title, entry.summary, entry.advice, entry.raw, entry.gainEvidence?.gain_failure_candidate ? JSON.stringify(entry.gainEvidence.gain_failure_candidate) : ''].filter(Boolean).join('\n\n'));
    const action = $('priority-error-action');
    if (action) {action.hidden=!entry.action?.page;action.textContent=busyText('前往处理','Open relevant settings');action.onclick=()=>{const target=entry.action;acknowledgePriorityError();operation(()=>openSetupTarget(target));};}
    const ack = $('priority-error-ack');
    if (ack) ack.textContent = entry.kind === 'fault' ? busyText('我已知晓（不恢复运行）', 'Acknowledge (no recovery)') : busyText('关闭', 'Close');
    if (!layer.open) layer.showModal();
    layer.setAttribute('aria-hidden', 'false');
    if (ack) ack.focus();
}
function showPriorityError(message, kind = 'operation') {
    const entry = normalizeDialogError(message, kind);
    if (activePriorityError?.key === entry.key || priorityErrorQueue.some(item => item.key === entry.key)) return;
    // A real machine fault preempts an ordinary error. It never silently
    // clears the latched fault or retries a control command.
    if (entry.kind === 'fault' && activePriorityError?.kind !== 'fault') {
        if (activePriorityError) priorityErrorQueue.unshift(activePriorityError);
        activePriorityError = entry;
        priorityErrorOpen = true;
        paintPriorityError();
        return;
    }
    if (entry.kind === 'fault') priorityErrorQueue.unshift(entry);
    else priorityErrorQueue.push(entry);
    showNextPriorityError();
}
function acknowledgePriorityError() {
    const layer = $('priority-error-overlay');
    if (!layer) return;
    if (layer.open) layer.close();
    layer.setAttribute('aria-hidden', 'true');
    priorityErrorOpen = false;
    activePriorityError = null;
    showNextPriorityError();
    if (!priorityErrorOpen) updateModelCalibrationTaskOverlay();
}
function toast(message, error = false) {
    if (error) { showPriorityError(message); return; }
    const node = document.createElement('div'); node.className = 'toast'; node.textContent = message;
    $('toasts').append(node); setTimeout(() => node.remove(), 8000);
}
function describeRuntimeFault(snapshot) {
    const f = snapshot?.fault || {};
    const code = f.code || snapshot?.last_fault || 'FAULT';
    const reason = [code, f.motor_bus_error, f.safety_code].filter(Boolean).join(' / ');
    const name = f.joint_name || (Number.isInteger(f.joint_index) ? `joint${f.joint_index + 1}` : '');
    if (code === 'REFERENCE_GOVERNOR_FAILED') {
        const en = state.language === 'en', error = f.reference_governor_error;
        const message = error === 'INFEASIBLE'
            ? (en ? 'No executable reference satisfies both motion limits and the total PD/feedforward torque budget' : '运动限制与 PD / 前馈总力矩预算内找不到可执行的参考')
            : error === 'INVALID_INPUT'
                ? (en ? 'The reference governor received invalid reference or torque-budget input' : '参考调节器收到无效的参考或力矩预算输入')
                : (en ? 'The reference governor could not accept the command' : '参考调节器无法接受当前命令');
        return { code, category: 'safety', message,
            advice: en ? 'Inspect the nominal trajectory, measured position, PD gains and total torque budget before fault recovery'
                : '请检查名义轨迹、实测位置、PD 增益和总力矩预算，再按故障恢复流程处理',
            technical: JSON.stringify(f, null, 2) };
    }
    if(f.safety_code==='STATE_TIMEOUT') {
        const en=state.language==='en';
        const value=Number.isFinite(f.value)?(f.value*1000).toFixed(2):'?';
        const limit=Number.isFinite(f.limit)?(f.limit*1000).toFixed(2):'?';
        const received=f.feedback_age_source==='host_transport_receive';
        const subject=received?(en?`${name || 'Joint'} host-received feedback age`:`${name || '关节'} 的主机接收反馈年龄`):
            (en?'State read interval (receive timestamp unavailable)':'状态读取间隔（缺少接收时间戳）');
        return {code:'STATE_TIMEOUT',category:'safety',
            message:en?`${subject} ${value} ms exceeds ${limit} ms`:`${subject} ${value} ms，超过 ${limit} ms 安全限制`,
            advice:en?'Inspect feedback continuity and control scheduling; the session remains faulted and will not resume automatically':
                '请检查对应关节的反馈连续性与控制调度，故障仍保持锁存，关闭提示不会自动恢复示教或运动',
            technical:JSON.stringify(f,null,2)};
    }
    const isDt = f.safety_code === 'INVALID_DT';
    const spec = jointDisplaySpec(name);
    const quantity = ['JOINT_VEL_LIMIT','CMD_VEL_LIMIT','CMD_VEL_STEP_LIMIT'].includes(f.safety_code) ? 'velocity' : ['JOINT_POS_LIMIT','CMD_POS_LIMIT','CMD_POS_STEP_LIMIT'].includes(f.safety_code) ? 'position' : null;
    const factor = isDt ? 1000 : quantity ? spec.factor : 1;
    const value = Number.isFinite(f.value) ? (f.value * factor).toFixed(isDt?2:4) : null;
    const limit = Number.isFinite(f.limit) ? (f.limit * factor).toFixed(isDt?2:4) : null;
    const unit = isDt ? ' ms' : quantity === 'velocity' ? ' '+spec.velocityUnit : quantity === 'position' ? ' '+spec.unit : ['JOINT_EFFORT_LIMIT','CMD_EFFORT_LIMIT'].includes(f.safety_code) ? ' '+spec.effortUnit : '';
    const effective = snapshot?.safety_limits || {};
    const jointIndex = Number.isInteger(f.joint_index) ? f.joint_index : (snapshot?.joint_names || []).indexOf(name);
    const cmdLimit = Number(effective.max_cmd_vel?.[jointIndex]);
    const stateLimit = Number(effective.max_state_vel?.[jointIndex]);
    const effectiveBounds = jointIndex >= 0 && Number.isFinite(cmdLimit) && Number.isFinite(stateLimit)
        ? `\n${state.language === 'en' ? 'Effective command/state velocity limits' : '实际生效命令/状态速度上限'}：${jointDisplayValue(cmdLimit,name,'velocity')} / ${jointDisplayValue(stateLimit,name,'velocity')}` : '';
    return [reason, name, value === null ? '' : `value=${value}${unit}`, limit === null ? '' : `limit=${limit}${unit}`, effectiveBounds].filter(Boolean).join(' / ');
}
function checkRuntimeAlerts(snapshot) {
    if (snapshot?.robot_state === 'FAULT') {
        const key = JSON.stringify(snapshot.fault || snapshot.last_fault || 'FAULT');
        if (key !== lastRuntimeFaultKey) { lastRuntimeFaultKey = key; showPriorityError(describeRuntimeFault(snapshot), 'fault'); }
    } else lastRuntimeFaultKey = '';
    const cal = snapshot?.model_calibration;
    if (cal?.phase === 'failed' && cal.error && !cal.fault_interrupted && !gainCalibrationQualityWarning(cal)) {
        const key = `${cal.task_id || 'unknown'}:${cal.error}`;
        if (!reportedCalibrationErrors.has(key)) { reportedCalibrationErrors.add(key); showPriorityError(cal.workflow_kind === 'gain' ? gainCalibrationFailure(cal) : cal.error); }
    }
    const replay = snapshot?.teach_replay;
    if (replay?.phase === 'failed' && replay.error) {
        if (lastTeachReplayErrorKey !== replay.error) {
            lastTeachReplayErrorKey = replay.error;
            showPriorityError(replay.error, 'replay');
        }
    } else if (replay && (replay.phase === 'playing' || (replay.phase === 'ready' && !replay.error))) {
        lastTeachReplayErrorKey = '';
    }
    const singleCal = snapshot?.calibration;
    if (singleCal?.phase === 'failed' && singleCal.error) {
        const key = `single:${singleCal.kind}:${singleCal.error}`;
        if (!reportedCalibrationErrors.has(key)) { reportedCalibrationErrors.add(key); showPriorityError(singleCal.error); }
    }
}
const button = (id, key, ico, cls = '') => `<button id="${id}" class="button ${cls}">${ico ? icon(ico) : ''}${t(key)}</button>`;
const card = (title, sub, ico, body) => `<section class="card"><div class="card-head">${icon(ico)}<div><h3>${t(title)}</h3><p>${t(sub)}</p></div></div><div class="card-body">${body}</div></section>`;
function pageHead(title, desc, action = '') { return `<header class="page-head"><div class="page-heading"><div class="eyebrow">SERIALARM · WORKSPACE</div><h2>${t(title)}</h2><p>${t(desc)}</p></div>${action ? `<div class="page-head-actions">${action}</div>` : ''}</header>`; }
function configField(key, label, placeholder = '') { return `<div class="field"><label for="${key}">${t(label)}</label><input id="${key}" value="${esc(state.config[key])}" placeholder="${esc(placeholder)}" autocomplete="off"></div>`; }
function active() { return ['running', 'stopping'].includes(state.session.state); }
async function save() { if (api) await api.savePrefs({ ...state.config, mode: state.mode, theme: state.theme, language: state.language }); }
function themeIsLight(theme = state.theme) { return theme === 'light' || (theme === 'system' && matchMedia('(prefers-color-scheme: light)').matches); }
function applyTheme() {
    const theme = themeIsLight() ? 'light' : 'dark';
    document.documentElement.className = theme;
    document.documentElement.dataset.theme = theme;
    if (term) term.options.theme = theme === 'light'
        ? { background: '#fbfbfc', foreground: '#1a1a2e', cursor: '#d97706' }
        : { background: '#0c0c12', foreground: '#c9c9d0', cursor: '#f59e0b' };
}
async function switchTheme(nextTheme, _event = null, rerender = false) {
    if (!['system', 'dark', 'light'].includes(nextTheme) || nextTheme === state.theme) return;
    state.theme = nextTheme; applyTheme();
    if (rerender) render();
    await save();
}
function pageLabel(page = state.page) {
    const keys = { start: 'manageProfiles', library: state.entryIntent === 'use' ? 'readyLibrary' : 'continueLibrary', description: 'newDescription', lifecycle: 'readiness', profile_setup: 'profileSetup', workbench: 'workbenchNav', geometry: 'geometryNav', calibration: 'calibrationNav', run: 'adapterNav', diagnostics: 'connectionDiagNav', model: 'model', settings: 'settings' };
    return t(keys[page] || page);
}
async function openSettings() {
    if (state.page === 'settings') { await navigate(state.previousPage || 'start'); return; }
    const previous = state.page;
    await navigate('settings');
    if (state.page === 'settings') state.previousPage = previous;
}
async function startIntent(intent) {
    if (active()) { toast(busyText('请先停止当前运行会话', 'Stop the active session first'), true); return; }
    if (state.config.profile) { await resetToStart(); if (state.page !== 'start') return; }
    if (intent === 'description') {
        await pageExit(); state.entryIntent = intent; state.descriptionStep = 1; state.createdProfileWizard = false;
        state.descriptionAnalysis = null;
        state.descriptionDraft = { source: '', profile: '', destination: '', base_frame: '', tool_frame: '', joint_names: [] };
        state.page = 'description'; render(); return;
    }
    state.entryIntent = intent; state.libraryQuery = ''; await refreshLibrary(); await pageExit(); state.page = 'library'; render();
}
async function showProfilePicker() {
    if ($('profile-picker')?.open) return;
    await refreshLibrary();
    const dialog = $('profile-picker');
    if (dialog.open) return;
    dialog.innerHTML = profileManager(true);
    bindProfileManager(dialog);
    dialog.showModal();
    $('picker-search')?.focus();
}
function shell() {
    setCalibrationTaskModalState(false);
    // A new Shell intentionally preserves the pending fault alert, if any.
    const onboarding = ['start', 'library', 'description'].includes(state.page) || (state.page === 'settings' && !state.config.profile);
    const quickTheme = `<button id="theme-quick" class="title-button" aria-label="${t('appearance')}">${icon('sun')}</button>`;
    const titlebar = `<header class="titlebar"><div class="brand-mini">${associationArtwork()}<strong>SerialArm Studio</strong></div><div class="title-center">${esc(state.config.profile || t('workspace'))}<span class="title-connection" id="title-connection"></span></div><div class="title-actions">${quickTheme}<button id="settings-quick" class="title-button" aria-label="${t('settings')}">${icon('settings')}</button><div class="window-controls"><button id="win-min" class="title-button" aria-label="${busyText('最小化', 'Minimize')}">${icon('minus')}</button><button id="win-max" class="title-button" aria-label="${busyText('最大化或还原', 'Maximize or restore')}">${icon('max')}</button><button id="win-close" class="title-button close" aria-label="${busyText('关闭窗口', 'Close window')}">${icon('close')}</button></div></div></header>`;
    const runtimePanel = `<section id="runtime-panel" class="card terminal-card is-hidden"><div class="terminal-head">${icon('terminal')}<b>${t('logs')}</b><span class="terminal-state" id="terminal-state"></span>${button('copy-log', 'copyAll', '')}${button('paste-log', 'paste', '')}${button('clear-log', 'clear', '')}${button('stop-runtime', 'stop', 'stop')}${button('force-runtime', 'force', '', 'danger')}</div><div id="terminal"></div><div class="terminal-note">${t('terminalHint')}</div></section>`;
    const overlays = `<div id="toasts" aria-live="polite"></div><dialog id="confirm-dialog" class="studio-dialog"></dialog><dialog id="profile-picker" class="project-sheet" aria-label="${t('manageProfiles')}"></dialog><dialog id="priority-error-overlay" class="priority-error-overlay" role="alertdialog" aria-modal="true" aria-labelledby="priority-error-title" aria-hidden="true"><div class="priority-error-card"><div id="priority-error-kicker" class="priority-error-kicker">SERIALARM · ERROR</div><h3 id="priority-error-title"></h3><pre id="priority-error-detail"></pre><div id="priority-error-evidence" hidden></div><details id="priority-error-technical" class="error-technical"><summary>${state.language === 'en' ? 'Raw diagnostic details' : '展开原始技术诊断'}</summary><pre id="priority-error-raw"></pre></details><p id="priority-error-guidance"></p><div class="actions"><button type="button" id="priority-error-action" class="button" hidden></button><button type="button" id="priority-error-copy" class="button">${state.language === 'en' ? 'Copy diagnostics' : '复制诊断信息'}</button><button id="priority-error-ack" class="button danger">${state.language === 'en' ? 'Understood (no recovery)' : '我已知晓（不恢复运行）'}</button></div></div></dialog><div id="operation-overlay" class="operation-overlay" hidden aria-hidden="true"><div class="operation-progress" role="status" aria-live="assertive" aria-busy="true"><span class="operation-spinner" aria-hidden="true"></span><div><div class="operation-kicker">SERIALARM</div><div id="operation-message" class="operation-message">${t('processing')}</div><div id="operation-safety-actions" class="workspace-actions" hidden><button id="operation-hold" class="button">${t('holdRobot')}</button><button id="operation-disable" class="button danger">${t('deactivateRobot')}</button><button id="operation-force-disconnect" class="button danger">${state.language === 'en' ? 'Force disconnect' : '强制断开'}</button></div></div></div></div><div id="model-calibration-overlay" class="model-calibration-overlay" hidden aria-hidden="true"><div class="model-calibration-task-window" role="dialog" aria-modal="true" aria-labelledby="model-calibration-task-title"><div class="task-window-head"><span class="operation-spinner" aria-hidden="true"></span><div><div id="model-calibration-task-kicker" class="operation-kicker">${state.language === 'en' ? 'Calibration task' : '标定任务'}</div><div id="model-calibration-task-title" class="operation-message">${t('modelCalibration')}</div></div></div><div id="model-calibration-task-preview" class="task-preview-slot" hidden></div><div class="task-window-content"><div class="task-progress-track"><span id="model-calibration-task-progress-bar"></span></div><div class="task-live-grid"><span id="model-calibration-task-progress">0%</span><span id="model-calibration-task-samples">${t('validStaticSamples')} —</span><span id="model-calibration-task-feedback">${t('feedbackAge')} —</span><span id="model-calibration-task-id">${t('taskIdentifier')} —</span></div><details class="calibration-locks task-joint-values" open><summary>${state.language === 'en' ? 'Joint targets and differences' : '关节目标与偏差'}</summary><div class="table-scroll"><table class="workspace-table"><thead><tr><th>${t('joint')}</th><th>${t('actual')}</th><th>${t('target')}</th><th>${state.language === 'en' ? 'Target − actual' : '目标 − 当前'}</th></tr></thead><tbody id="model-calibration-task-joints"></tbody></table></div></details>${gainProbeStatusHtml({}, 'gain-task-probe')}</div><div class="task-window-actions"><button id="model-calibration-task-stop-teach" class="button primary">${t('stopTeaching')}</button><button id="model-calibration-task-pause" class="button">${t('pauseTask')}</button><button id="model-calibration-task-resume" class="button">${t('resumeTask')}</button><button id="model-calibration-task-hold" class="button">${t('holdRobot')}</button><button id="model-calibration-task-cancel" class="button danger">${t('cancelTask')}</button><button id="model-calibration-task-disable" class="button danger">${t('deactivateRobot')}</button></div><div id="model-calibration-task-error" class="workspace-error"></div></div></div>`;
    const lifecycle = [['lifecycle', 'readiness', 'check'], ['profile_setup', 'profileSetup', 'robot']];
    const expert = [['model', 'model', 'model'], ['geometry', 'geometryNav', 'check'], ['calibration', 'calibrationNav', 'hardware'], ['workbench', 'workbenchNav', 'workbench'], ['run', 'adapterNav', 'run'], ['diagnostics', 'connectionDiagNav', 'diagnostics']];
    const nav = ([name, key, ico]) => `<button class="nav-item" data-page="${name}" title="${t(key)}" aria-label="${t(key)}" ${!state.config.profile ? 'disabled' : ''}>${icon(ico)}<span>${state.language === 'en' ? ({ lifecycle: 'Readiness', profile_setup: 'Parameters', model: 'Model viewer', workbench: 'Control', geometry: 'Geometry', calibration: 'Dynamics', run: 'Adapters', diagnostics: 'Diagnostics' }[name] || t(key)) : t(key)}</span></button>`;
    const lifecycleLabel = busyText('机械臂生命周期', 'ROBOT LIFECYCLE');
    const navigation = `<section class="sidebar-nav-group" aria-labelledby="lifecycle-caption"><div class="nav-caption" id="lifecycle-caption">${lifecycleLabel}</div><nav aria-label="${lifecycleLabel}">${lifecycle.map(nav).join('')}</nav></section><section class="sidebar-nav-group" aria-labelledby="expert-caption"><div class="nav-caption" id="expert-caption">${t('expertTools')}</div><nav aria-label="${t('expertTools')}">${expert.map(nav).join('')}</nav></section>`;
    const body = onboarding
        ? `<div class="onboarding-body"><main class="content onboarding-content"><div class="page" id="page"></div>${runtimePanel}</main></div>`
        : `<div class="body"><aside class="sidebar"><button id="profile-picker-open" class="sidebar-project project-picker-trigger" aria-label="${t('manageProfiles')}" aria-haspopup="dialog">${robotArtwork()}<span>${esc(state.config.profile || busyText('选择机械臂', 'Choose a manipulator'))}</span>${icon('chevron')}</button>${navigation}<div class="sidebar-spacer"></div><button id="back-home" class="nav-item" title="${t('backHome')}" aria-label="${t('backHome')}">${icon('folder')}<span>${t('backHome')}</span></button><button id="settings-nav" class="nav-item" data-page="settings" title="${t('settings')}" aria-label="${t('settings')}">${icon('settings')}<span>${t('settings')}</span></button><div class="runtime-card" id="sidebar-runtime"></div></aside><section class="content-wrap"><div class="studio-location top-status"><span class="studio-route breadcrumb" id="breadcrumb"></span><div class="studio-location-end"><span class="pill" id="env-pill"></span><span class="pill" id="session-pill"></span></div></div><main class="content"><div class="page" id="page"></div>${runtimePanel}</main></section></div>`;
    $('app').innerHTML = `<div class="shell app atil-shell ${onboarding ? 'onboarding-shell' : ''}">${titlebar}${body}</div>${overlays}`;
    document.querySelectorAll('[data-page]').forEach(b => b.onclick = () => b.dataset.page === 'settings' ? openSettings() : navigate(b.dataset.page));
    ['min', 'max', 'close'].forEach((n, i) => { const node = $('win-' + n); if (node) node.onclick = () => api.window(['minimize', 'maximize', 'close'][i]); });
    $('settings-quick').onclick = openSettings;
    if ($('back-home')) $('back-home').onclick = () => operation(resetToStart);
    if ($('profile-picker-open')) $('profile-picker-open').onclick = () => operation(showProfilePicker);
    $('theme-quick').onclick = event => operation(() => switchTheme(document.documentElement.classList.contains('light') ? 'dark' : 'light', event));
    if ($('copy-log')) $('copy-log').onclick = () => operation(() => copyTerminal(true));
    if ($('paste-log')) $('paste-log').onclick = () => operation(pasteTerminal);
    if ($('clear-log')) $('clear-log').onclick = () => { outputBuffer = ''; term?.clear(); };
    if ($('stop-runtime')) $('stop-runtime').onclick = () => workbenchConnected() ? confirmWorkbenchDisconnect() : blockingOperation(busyText('正在停止运行...', 'Stopping runtime...'), () => api.request('stop'));
    if ($('force-runtime')) $('force-runtime').onclick = () => forceConfirm();
    if ($('model-calibration-task-stop-teach')) $('model-calibration-task-stop-teach').onclick = () => calibrationTaskAction('stop-recording');
    if ($('model-calibration-task-pause')) $('model-calibration-task-pause').onclick = () => modelCalibrationQuick('model_calibration_pause');
    if ($('model-calibration-task-resume')) $('model-calibration-task-resume').onclick = () => resumeModelCalibrationWithConfirmation();
    if ($('model-calibration-task-hold')) $('model-calibration-task-hold').onclick = () => calibrationTaskAction('hold');
    if ($('model-calibration-task-cancel')) $('model-calibration-task-cancel').onclick = () => calibrationTaskAction('cancel');
    if ($('model-calibration-task-disable')) $('model-calibration-task-disable').onclick = () => calibrationTaskAction('deactivate');
    if ($('operation-hold')) $('operation-hold').onclick = () => interruptParking('hold');
    if ($('operation-disable')) $('operation-disable').onclick = () => interruptParking('deactivate');
    if ($('operation-force-disconnect')) $('operation-force-disconnect').onclick = confirmUnresponsiveDisconnect;
    mountTerminal(); applyTheme(); render();
    if ($('priority-error-ack')) $('priority-error-ack').onclick = acknowledgePriorityError;
    if ($('confirm-dialog')) $('confirm-dialog').addEventListener('close', refreshCalibrationAfterDialog);
    if ($('priority-error-overlay')) $('priority-error-overlay').addEventListener('cancel', event => event.preventDefault());
    if (activePriorityError) paintPriorityError();
}
function mountTerminal() {
    if (typeof Terminal !== 'undefined') {
        term?.dispose(); term = new Terminal({ fontFamily: '"JetBrains Mono", "Noto Sans Mono CJK SC", monospace', fontSize: 12, scrollback: 5000, convertEol: true, theme: { background: '#0c0c12', foreground: '#c9c9d0', cursor: '#f59e0b' }, allowProposedApi: false });
        fit = new FitAddon.FitAddon(); term.loadAddon(fit); term.open($('terminal')); if (outputBuffer) term.write(outputBuffer);
        term.attachCustomKeyEventHandler(terminalKey);
        $('terminal').oncontextmenu = event => { event.preventDefault(); operation(terminalMenu); };
        term.onData(data => { if (active() && state.session.mode === 'terminal') api.request('input', { data }).catch(error => toast(error.message, true)); });
        term.onResize(({ cols, rows }) => { if (active()) api.request('resize', { cols, rows }).catch(() => { }); });
    }
}
function terminalInputReady() { return active() && state.session.mode === 'terminal'; }
function terminalText() {
    if (!term) return '';
    const buffer = term.buffer.active; let text = '';
    for (let row = 0; row < buffer.length; row++) {
        const line = buffer.getLine(row); if (!line) continue;
        if (row && !line.isWrapped) text += '\n';
        text += line.translateToString(true);
    }
    return text.replace(/\n+$/, '').slice(-1024 * 1024);
}
async function copyTerminal(all = false) {
    const text = all ? terminalText() : term?.getSelection();
    if (text) await api.clipboard.write(text.slice(-1024 * 1024));
}
async function pasteTerminal() {
    if (!terminalInputReady()) return;
    const text = await api.clipboard.read();
    if (text.length > 65524) throw new Error('Clipboard text exceeds the terminal input limit');
    if (terminalInputReady() && text) { term.paste(text); term.focus(); }
}
function terminalKey(event) {
    const key = event.key.toLowerCase();
    const shortcut = (event.ctrlKey && event.shiftKey && !event.altKey && !event.metaKey) || (event.metaKey && !event.ctrlKey && !event.altKey);
    if (!shortcut || !['c', 'v', 'a'].includes(key)) return true;
    if (event.type === 'keydown') {
        event.preventDefault();
        if (key === 'c') operation(() => copyTerminal());
        else if (key === 'v') operation(pasteTerminal);
        else term.selectAll();
    }
    return false;
}
async function terminalMenu() {
    const action = await api.terminalMenu({ selection: !!term?.hasSelection(), canPaste: terminalInputReady(), language: state.language });
    if (action === 'copy') await copyTerminal();
    else if (action === 'copy-all') await copyTerminal(true);
    else if (action === 'paste') await pasteTerminal();
    else if (action === 'select-all') { term.selectAll(); term.focus(); }
}
function readinessStateText(value) {
    const map = { configuring: 'configuring', pending_validation: 'pendingValidation', needs_revalidation: 'needsRevalidation', ready: 'usable' };
    if (value === 'drag_ready') return state.language === 'en' ? 'Drag verified' : '拖拽已验收';
    if (value === 'control_partial') return state.language === 'en' ? 'Some controls verified' : '部分控制已验收';
    return t(map[value] || 'unknown');
}
function statusUI() {
    const ready = state.status.installed;
    const sidebar = $('sidebar-runtime'); if (sidebar) sidebar.innerHTML = `<span class="dot ${ready ? 'ready' : ''}"></span>${t(ready ? 'connected' : 'notInstalled')}<small>${esc(state.info?.profile || state.config.profile || '—')}</small><small>${state.backendError ? esc(localizedError(state.backendError)) : busyText('原生运行时', 'Native runtime')}</small>`;
    const env = $('env-pill'); if (env) env.innerHTML = `<span class="dot ${ready ? 'ready' : ''}"></span>Core ${t(ready ? 'ready' : 'unavailable')}`;
    const session = $('session-pill'); if (session) session.innerHTML = `<span class="dot ${active() ? 'ready' : ''}"></span>${t(state.session.state || 'idle')}`;
    const breadcrumb = $('breadcrumb'); if (breadcrumb) breadcrumb.innerHTML = `<span>${esc(state.config.profile || t('workspace'))}</span>${icon('arrow')}<strong>${pageLabel()}</strong>`;
    const connection = $('title-connection'); if (connection) { connection.innerHTML = `<span class="dot ${active() ? 'ready' : ''}"></span>`; connection.title = t(active() ? 'running' : 'idle'); }
    const terminalState = $('terminal-state'); if (terminalState) terminalState.textContent = active() ? `${t(state.session.state)} · PID ${state.session.pid}` : state.session.state === 'exited' ? `${t('exited')} · ${state.session.exit_code}` : t('noSession');
    const stop = $('stop-runtime'); if (stop) stop.disabled = !active() || state.session.state === 'stopping'; const force = $('force-runtime'); if (force) force.classList.toggle('is-hidden', state.session.state !== 'stopping');
}

function resetProfileWorkspaceState() {
    graphicsState.records.clear(); graphicsState.current = null; graphicsState.ghostKey = '';
    const preserve = new Set(['page','previousPage','language','theme','mode','source','entryIntent','libraryQuery','libraryFilter','library','libraryPath','activeLibraryId','config','status','profiles','session','busy','blockingBusy','backendError','inspecting','workbenchEpoch']);
    for (const [key,value] of Object.entries(profileWorkspaceDefaults)) if (!preserve.has(key)) state[key]=structuredClone(value);
    state.profileMissingTarget=null;
    state.workbenchEpoch=(state.workbenchEpoch || 0)+1;
    state.setupReturn=null;state.setupFocus=null;state.profileSavedForSession=false;state.profileSaveNotice=false;
    state.profileDirty=false;state.profileGroup='model';clearProfileFormState();
    state.profileInteractionGoal=null;state.runtimeInteractionGoal=null;state.responseAnalysis=null;state.responseRecording=null;state.gravityVerifyDraft=null;
    state.gainRequirements=null;state.savePreview=null;state.pendingTarget=null;state.pendingMotionId=null;
    reportedCalibrationErrors.clear();lastTeachReplayErrorKey='';lastRuntimeFaultKey='';
}
async function resetToStart() {
    if (state.page === 'profile_setup' && state.profileDirty && !await safetyConfirmation(state.language === 'en' ? 'Discard changes' : '放弃未保存的参数', state.language === 'en' ? 'Unsaved changes will be discarded on return' : '返回首页后未保存的修改将丢失', '', [], 'general')) return;
    if (active()) { toast(state.language === 'en' ? 'Stop the active session before returning to the start page' : '请先停止当前运行会话', true); return; }
    await pageExit(); resetProfileWorkspaceState();
    state.page = 'start'; state.entryIntent = ''; state.descriptionStep = 1; state.createdProfileWizard = false; state.activeLibraryId = ''; state.readiness = null; state.info = null; state.modelData = null; state.profileEditor = null; state.capabilityProfile = null; state.capabilityProfileError = ''; state.admittance = null; state.tuneDraft = null; state.config = { profile: '', profile_file: '', serial_port: '', baudrate: '', bus: '', resource_paths: '' }; renderedPage = ''; shell();
}
function profileManager(picker = false) {
    const prefix = picker ? 'picker' : 'library';
    const filter = picker ? 'all' : state.libraryFilter || 'all';
    const title = picker || state.page === 'start' ? t('manageProfiles') : pageLabel();
    const rows = state.library.map(item => {
        const ready = item.readiness?.control_available === true;
        const current = state.activeLibraryId === item.id;
        const source = item.source === 'builtin' ? t('builtin') : item.profile_file;
        return `<li class="robot-manager-row ${current ? 'is-current' : ''}" data-library-row data-ready="${ready}" data-search="${esc((item.profile + ' ' + (source || '')).toLowerCase())}"><button class="robot-row-open" data-library-open="${esc(item.id)}" ${active() && !current ? 'disabled' : ''}><span class="robot-row-avatar">${robotArtwork()}</span><span class="robot-row-description"><strong>${esc(item.profile)}</strong><small title="${esc(source)}">${esc(source)}</small></span><span class="readiness-chip ${libraryStateClass(item.readiness?.state)}">${readinessStateText(item.readiness?.state)}</span><span class="robot-switch-label">${current ? busyText('使用中', 'Current') : ready ? t('useProfile') : t('continueAction')}${icon('arrow')}</span></button><button class="robot-row-more" data-library-menu="${esc(item.id)}" title="${busyText('更多操作', 'More actions')}" aria-label="${esc(item.profile)} · ${busyText('更多操作', 'More actions')}" aria-haspopup="menu" aria-expanded="false">${icon('more')}</button></li>`;
    }).join('');
    return `<section class="robot-manager" data-manager-filter="${filter}"><header class="robot-manager-heading"><div><h2>${title}</h2><span class="robot-manager-count">${state.library.length}</span></div>${picker ? `<button class="robot-manager-close" data-picker-close aria-label="${busyText('关闭 Profile 管理', 'Close Profile manager')}">${icon('close')}</button>` : `${state.page === 'start' ? '' : `<button id="library-home" class="button">${icon('folder')}${t('backHome')}</button>`}`}</header><div class="robot-manager-toolbar"><label class="robot-manager-search">${icon('search')}<input id="${prefix}-search" type="search" aria-label="${busyText('搜索机械臂', 'Search manipulators')}" placeholder="${busyText('搜索机械臂', 'Search manipulators')}" value="${esc(picker ? '' : state.libraryQuery)}"></label>${!picker ? `<button id="library-import-dir" class="button">${icon('folder')}${busyText('打开机械臂 Profile', 'Open manipulator Profile')}</button><button class="button primary" data-start-intent="description">${icon('plus')}${t('newDescription')}</button>` : ''}</div>${true ? `<div class="manager-filters" role="group" aria-label="${busyText('Profile 状态', 'Profile status')}">${[['all', busyText('全部', 'All')], ['configuring', t('continueLibrary')], ['ready', t('readyLibrary')]].map(([key, label]) => `<button data-library-filter="${key}" aria-pressed="${filter === key}">${label}</button>`).join('')}</div>` : ''}<ul class="robot-manager-list" aria-label="${t('profileLibrary')}">${rows}<li class="robot-manager-empty" data-library-empty hidden>${robotArtwork()}<strong>${busyText('没有找到机械臂', 'No manipulators found')}</strong><p>${busyText('试试其他名称，或打开已有 Profile', 'Try another name, or open an existing Profile')}</p></li></ul>${!picker ? `<footer class="robot-manager-footer"><button id="library-import-file" class="button">${busyText('选择机械臂 Profile 文件', 'Choose a manipulator Profile file')} · YAML</button></footer>` : `<p class="robot-switch-note">${active() ? busyText('当前会话运行中，停止后可以切换机械臂', 'Stop the active session before switching manipulators') : t('profileManagedHint')}</p>`}<div id="${prefix}-menu" class="studio-popover robot-row-menu" popover="auto" role="menu"><button data-library-remove="" role="menuitem" class="robot-menu-delete">${icon('trash')}${t('removeFromLibrary')}</button></div></section>`;
}
function startPage() {
    return `<section class="start-hero"><div class="home-brand-lockup">${associationArtwork('home-association-mark')}<div class="home-brand-title"><h1>SerialArm Studio</h1></div></div><p>${busyText('选择机械臂，或新建、打开机械臂 Profile', 'Choose a manipulator, or create or open a manipulator Profile')}</p></section>${profileManager()}`;
}
function libraryStateClass(value) { return value === 'ready' ? 'ready' : value === 'needs_revalidation' ? 'warning' : value === 'configuring' ? 'muted' : 'pending'; }
function libraryPage() { return profileManager(); }
function bindProfileManager(root) {
    const manager = root.querySelector('.robot-manager');
    if (!manager) return;
    const search = manager.querySelector('input[type="search"]');
    const picker = root.id === 'profile-picker';
    const update = () => {
        const query = search.value.trim().toLowerCase();
        if (!picker) state.libraryQuery = search.value;
        let count = 0;
        manager.querySelectorAll('[data-library-row]').forEach(row => {
            const match = row.dataset.search.includes(query) && (manager.dataset.managerFilter === 'all' || (row.dataset.ready === 'true') === (manager.dataset.managerFilter === 'ready'));
            row.hidden = !match; if (match) count++;
        });
        manager.querySelector('[data-library-empty]').hidden = count > 0;
    };
    search.oninput = update;
    manager.querySelectorAll('[data-library-filter]').forEach(button => button.onclick = () => {
        manager.dataset.managerFilter = button.dataset.libraryFilter;
        if (!picker) state.libraryFilter = button.dataset.libraryFilter;
        manager.querySelectorAll('[data-library-filter]').forEach(item => item.setAttribute('aria-pressed', String(item === button)));
        update();
    });
    manager.querySelector('[data-picker-close]')?.addEventListener('click', () => root.close());
    manager.querySelectorAll('[data-library-open]').forEach(button => button.onclick = () => operation(async () => {
        if (picker && button.dataset.libraryOpen === state.activeLibraryId) { root.close(); return; }
        if (active()) { toast(busyText('请先停止当前运行会话', 'Stop the active session first'), true); return; }
        if (state.profileDirty && !await safetyConfirmation(busyText('放弃未保存的参数', 'Discard unsaved changes'), busyText('切换机械臂将放弃未保存的参数修改', 'Switching manipulators discards unsaved parameter changes'), '', [], 'general')) return;
        if (picker) root.close();
        await blockingOperation(busyText('正在打开 Profile...', 'Opening Profile...'), () => openLibraryEntry(button.dataset.libraryOpen));
    }));
    const menu = manager.querySelector('[popover]');
    manager.querySelectorAll('[data-library-menu]').forEach(button => button.onclick = () => {
        const rect = button.getBoundingClientRect();
        menu.querySelector('[data-library-remove]').dataset.libraryRemove = button.dataset.libraryMenu;
        menu.querySelector('[data-library-remove]').disabled = active();
        menu.style.left = `${Math.max(8, Math.min(innerWidth - 200, rect.right - 184))}px`;
        menu.style.top = `${Math.min(innerHeight - 64, rect.bottom + 5)}px`;
        menu.showPopover(); button.setAttribute('aria-expanded', 'true');
        const closed = event => { if (event.newState === 'closed') { button.setAttribute('aria-expanded', 'false'); menu.removeEventListener('toggle', closed); } };
        menu.addEventListener('toggle', closed);
    });
    menu.querySelector('[data-library-remove]').onclick = () => {
        const id = menu.querySelector('[data-library-remove]').dataset.libraryRemove;
        menu.hidePopover(); if (picker) root.close();
        confirmLibraryRemoval(id);
    };
    update();
}
function confirmLibraryRemoval(id) {
    const entry = state.library.find(item => item.id === id);
    const dialog = $('confirm-dialog');
    dialog.innerHTML = `<h3>${t('removeFromLibrary')}</h3><p>${t('profileManagedHint')}</p><dl class="kv"><dt>${t('profileName')}</dt><dd>${esc(entry?.profile || id)}</dd></dl><div class="actions"><button id="remove-cancel" class="button">${t('cancel')}</button><button id="remove-confirm" class="button danger">${t('removeFromLibrary')}</button></div>`;
    dialog.showModal(); $('remove-cancel').onclick = () => dialog.close();
    $('remove-confirm').onclick = () => { dialog.close(); blockingOperation(busyText('正在从 Profile Library 移除...', 'Removing from Profile Library...'), async () => { await api.request('profile_library_remove', { id }); await refreshLibrary(); render(); }); };
}
function readinessStageText(value) { const keys = { description: 'descriptionStage', profile_setup: 'profileSetupStage', model_check: 'modelCheckStage', bringup: 'bringupStage', mapping_calibration: 'mappingStage', geometry_calibration: 'geometryStage', gravity_verification: 'gravityStage', dynamics_calibration: 'dynamicsStage', validation: 'validationStage', ready: 'readyStage' }; return t(keys[value] || 'unknown'); }
function computeChain(base, tool, joints) { const byChild = new Map(joints.map(j => [j.child, j])); const reverse = []; let node = tool; const seen = new Set(); while (node && node !== base && !seen.has(node)) { seen.add(node); const j = byChild.get(node); if (!j) return []; reverse.push(j); node = j.parent; } if (node !== base) return []; return reverse.reverse(); }
function syncDescriptionChain() {
    const a = state.descriptionAnalysis, d = state.descriptionDraft;
    if (!a || !a.joints?.length) return;
    const chain = computeChain(d.base_frame, d.tool_frame, a.joints);
    d.joint_names = chain.filter(j => j.type !== 'fixed' && !j.mimic).map(j => j.name);
}
function descriptionStepper(step = state.descriptionStep) {
    const stages = ['newModelCheck', 'controlChain', 'createProfile', 'profileSetup'];
    return `<div class="setup-stepper setup-stepper-four" aria-label="${state.language === 'en' ? 'Creation steps' : '新建机械臂步骤'}">${stages.map((key, i) => `<span class="${i + 1 === step ? 'active' : i + 1 < step ? 'complete' : ''}" ${i + 1 === step ? 'aria-current="step"' : ''}>${i + 1} ${t(key)}</span>`).join('')}</div>`;
}
function descriptionPage() {
    const d = state.descriptionDraft, a = state.descriptionAnalysis;
    const step = a ? state.descriptionStep : 1;
    const pageTitle = pageHead('newDescription', 'newDescriptionDesc', `<div class="actions description-head-actions"><button id="description-home" class="button">${t('backHome')}</button>${a ? `<button id="description-reset" class="button">${state.language === 'en' ? 'Choose another Description' : '重新选择模型'}</button>` : ''}</div>`);
    const upload = `<div class="description-empty card"><div class="card-body"><div class="field"><label>${t('descriptionSource')}</label><div class="input-row"><input id="description-source" value="${esc(d.source)}" placeholder=".zip / .urdf / .xacro"><button id="description-browse" class="button">${t('selectDescription')}</button><button id="description-browse-dir" class="button">${state.language === 'en' ? 'Choose directory' : '选择目录'}</button></div></div><div class="actions"><button id="description-analyze" class="button primary">${t('analyzeDescription')}</button></div><p class="hint">${t('newModelHint')}</p></div></div>`;
    if (!a) return pageTitle + descriptionStepper(1) + upload;
    if (a.model_choices) return pageTitle + descriptionStepper(1) + upload +
        `<div class="card"><div class="card-body"><label>${state.language === 'en' ? 'Choose the robot model' : '选择机器人模型'}</label><select id="description-model"><option value="">${state.language === 'en' ? 'Select a model' : '请选择模型'}</option>${a.model_choices.map(name => `<option value="${esc(name)}">${esc(name)}</option>`).join('')}</select></div></div>`;
    syncDescriptionChain();
    const movable = a.joints.filter(j => j.type !== 'fixed' && !j.mimic);
    const basicOk = !!(a.links.length && a.roots?.length && movable.length && a.model_valid !== false);
    const linkOptions = a.links.map(x => `<option value="${esc(x)}" ${x === d.base_frame ? 'selected' : ''}>${esc(x)}</option>`).join('');
    const toolOptions = a.links.map(x => `<option value="${esc(x)}" ${x === d.tool_frame ? 'selected' : ''}>${esc(x)}</option>`).join('');
    const chain = computeChain(d.base_frame, d.tool_frame, a.joints);
    const jointSummary = chain.map(j => `<span class="chain-joint ${j.type === 'fixed' ? 'fixed' : ''}">${esc(j.name)}<small>${esc(j.type)}</small></span>`).join('') || `<span class="warning-text">${state.language === 'en' ? 'Base and Tool are not on one connected chain' : 'Base 与 Tool 不构成连续控制链'}</span>`;
    const stats = `<div class="description-facts"><span>${a.links.length} Links</span><span>${a.joints.length} Joints</span><span>${movable.length} ${state.language === 'en' ? 'Movable' : '可动关节'}</span></div>`;
    const preview = `<section class="card description-preview"><div class="card-head">${icon('model')}<div><h3>${esc(a.package_name)}</h3><p>${esc(a.urdf)}</p></div></div><div id="description-viewport"></div>${stats}</section>`;
    let form = '';
    if (step === 1) {
        form = `<h3>${t('newModelCheck')}</h3><p class="parameter-guidance">${t('newModelHint')}</p><div class="wizard-check ${basicOk ? 'passed' : 'warning'}"><span>${basicOk ? '✓' : '!'}</span><div><b>${basicOk ? (state.language === 'en' ? 'Basic model check passed' : '模型基础检查通过') : (state.language === 'en' ? 'Resolve the model issues below' : '请修正下方模型问题')}</b><small>${state.language === 'en' ? 'URDF parsed and model topology loaded' : 'URDF 已解析，模型结构已读取'}</small></div></div><dl class="kv"><dt>Base</dt><dd>${esc(a.roots?.[0] || '—')}</dd><dt>Tool</dt><dd>${esc(d.tool_frame || '—')}</dd><dt>${t('controlledJoints')}</dt><dd>${movable.length}</dd></dl><div class="wizard-actions"><button id="description-next-chain" class="button primary" ${basicOk ? '' : 'disabled'}>${t('wizardNext')} →</button></div>`;
    } else if (step === 2) {
        form = `<h3>${t('controlChain')}</h3><p class="parameter-guidance">${state.language === 'en' ? 'Select Base and Tool to derive the controlled joint sequence' : '选择 Base 与 Tool，自动识别沿途受控关节及其顺序'}</p><div class="row"><div class="field"><label>${t('baseFrame')}</label><select id="draft-base">${linkOptions}</select></div><div class="field"><label>${t('toolFrame')}</label><select id="draft-tool">${toolOptions}</select></div></div><div class="chain-preview">${jointSummary}</div><div class="field"><label>${t('controlledJoints')}</label><div class="chip-row">${d.joint_names.map(x => `<span class="chip">${esc(x)}</span>`).join('') || '—'}</div></div><div class="wizard-actions"><button id="description-back" class="button">← ${t('wizardBack')}</button><button id="description-next-profile" class="button primary" ${d.joint_names.length ? '' : 'disabled'}>${t('wizardNext')} →</button></div>`;
    } else {
        form = `<h3>${t('createProfile')}</h3><p class="parameter-guidance">${t('newHardwareHint')}</p><div class="wizard-summary"><div><small>Base → Tool</small><b>${esc(d.base_frame)} → ${esc(d.tool_frame)}</b></div><div><small>${t('controlledJoints')}</small><b>${d.joint_names.map(esc).join(' · ')}</b></div></div><div class="field"><label>${t('profileId')}</label><input id="draft-profile" value="${esc(d.profile)}"></div><div class="field"><label>${t('destination')}</label><div class="input-row"><input id="draft-destination" value="${esc(d.destination)}"><button id="draft-destination-browse" class="button">${t('chooseDirectory')}</button></div></div><div class="wizard-actions"><button id="description-back" class="button">← ${t('wizardBack')}</button><button id="draft-create" class="button primary">${t('createProfile')} →</button></div>`;
    }
    const modelIssues = (a.model_issues || []).map(issue=>`<p class="parameter-error">${esc(issue.message)}${issue.path ? `<code>${esc(issue.path)}</code>` : ''}</p>`).join('');
    return pageTitle + descriptionStepper(step) + `<div class="description-layout">${preview}<section class="card setup-form"><div class="card-body">${modelIssues}${form}</div></section></div>`;
}
const readinessReviews = {
    model_check: { title: '模型检查', keys: [['chain', '受控链、关节与坐标系一致'], ['limits', '运动范围和安全限位已核对'], ['inertial', '惯性参数与模型结构已检查']] },
    bringup: { title: '真机连接检查', keys: [['connection', '已在 INACTIVE 状态检查电机通信与所有关节反馈']] },
    startup_hold: { title: '启动保持安全验收', keys: [['hold_gains', '全部关节保持增益与上限已核对'], ['supported', '已采取机械支撑与清空运动空间等措施'], ['emergency_stop', '硬件急停和失能途径已确认']] },
    mapping_calibration: { title: '映射校准', keys: [['joint_ids', '每个关节与电机 ID 已实测对应'], ['directions', '每个关节运动方向已实测正确'], ['offsets', '零位与偏移量已实测确认']] },
    geometry_calibration: { title: '几何标定', keys: [['frames', 'Base 与 Tool 坐标系已实测核对'], ['zero_pose', '零位与机构位置一致'], ['tool_pose', 'TCP 与工具姿态已核对']] },
    gravity_verification: { title: '重力补偿真机验收', keys: [['distinct_poses', '已记录至少三个实际不同的稳定拖拽姿态'], ['support', '已检查机械支撑和急停措施'], ['restored', '已在机械支撑下失能并确认所有临时比例恢复']] },
    validation: { title: '综合验收', keys: [['tracking', '已检查获准控制模式的真实跟踪效果'], ['stopping', '已检查正常停放与失能过程'], ['faults', '已核对故障与紧急处理结果']] },
};
function readinessResultLabel(value) {
    const labels = { verified: ['已验证', 'Verified'], warning: ['未达参考指标', 'Reference metrics not met'], recorded: ['已自查', 'Self-checked'], partial: ['部分自查', 'Partially checked'], pending: ['未验证', 'Not verified'], historical: ['历史记录', 'Historical record'], configured: ['参数已配置', 'Configured'], blocked: ['前置条件未满足', 'Blocked'], stale: ['相关参数已变化', 'Stale'], failed: ['配置错误', 'Failed'], unconfigured: ['参数尚未满足要求', 'Unconfigured'] };
    return (labels[value] || labels.pending)[state.language === 'en' ? 1 : 0];
}
function lifecycleStageLabel(stage, value) {
    const en = state.language === 'en';
    if (stage === 'description' && value === 'verified') return en ? 'Model parsed' : '模型已解析';
    if (stage === 'profile_setup' && value === 'verified') return en ? 'Configuration valid' : '配置有效';
    if (['dynamics_calibration','gravity_model','advanced_dynamics','geometry_calibration'].includes(stage) && value === 'pending') return en ? 'Not calibrated' : '待标定';
    if (stage === 'gravity_verification' && value === 'pending') return en ? 'Not verified' : '待验证';
    return readinessResultLabel(value);
}
const setupDestinations = {
    description: {page:'model'}, profile_setup: {page:'profile_setup',group:'hardware'},
    model_check: {page:'model'}, bringup: {page:'diagnostics',anchor:'connection-settings'},
    mapping_calibration: {page:'profile_setup',group:'mapping'},
    geometry_calibration: {page:'geometry',tool:'fixed'},
    gravity_verification: {page:'calibration',tool:'gravity_verify'},
    dynamics_calibration: {page:'calibration',tool:'gain'}, gravity_model: {page:'calibration',tool:'gravity'}, advanced_dynamics: {page:'calibration',tool:'dynamics'}, validation: {page:'workbench',tab:'control',anchor:'robot-operation'},
};
function setupActionHtml(action, label, primary = false) {
    return `<button type="button" class="button ${primary ? 'primary' : ''}" data-setup-target="${esc(JSON.stringify(action))}">${esc(label)}</button>`;
}
function readinessConnectionState(editor = null) {
    const saved = state.readiness?.connection_readiness;
    if (!editor) return saved || {
        structurally_valid: state.readiness?.control_available === true,
        read_only_connectable: state.readiness?.control_available === true,
        write_enabled: null, basic_calibration_ready: null, basic_calibration_prerequisites: []
    };
    const core = editor.core || {}, names = editor.joint_names || core.model?.joint_names || [];
    const model = core.model || {}, controller = core.control?.controller || {}, safety = core.safety_policy || {};
    const positive = value => typeof value === 'number' && Number.isFinite(value) && value > 0;
    const condition = (id, ready, field, group) => ({id, ready,
        action: {page:'profile_setup', group, field}});
    const dragJoint = names.find(joint => !positive(controller.compliant_drag?.kd?.[joint]));
    const badGravity = !Array.isArray(model.gravity) || model.gravity.length !== 3 ||
        model.gravity.some(value => typeof value !== 'number' || !Number.isFinite(value)) ||
        model.gravity.reduce((sum, value) => sum + value * value, 0) <= 1e-12;
    const scaleJoint = names.find(joint => !positive(model.gravity_scale?.[joint]));
    let holdField = '';
    for (const axis of ['kp','kd']) for (const joint of names) {
        const value = controller.rigid_hold?.[axis]?.[joint], limit = safety[`max_${axis}_override`]?.[joint];
        if (!positive(value) || (typeof limit === 'number' && value > limit)) {
            holdField = `core.control.controller.rigid_hold.${axis}.${joint}`; break;
        }
    }
    const prerequisites = [
        condition('write_enabled', core.control?.runtime?.write_enabled === true, 'core.control.runtime.write_enabled', 'motion'),
        condition('drag_damping', names.length > 0 && !dragJoint, `core.control.controller.compliant_drag.kd${dragJoint ? '.' + dragJoint : ''}`, 'motion'),
        condition('gravity', names.length > 0 && !badGravity && !scaleJoint, badGravity ? 'core.model.gravity' : `core.model.gravity_scale${scaleJoint ? '.' + scaleJoint : ''}`, 'model'),
        condition('hold_gains', names.length > 0 && !holdField, holdField || 'core.control.controller.rigid_hold.kp', 'motion')
    ];
    const structure = state.profileDirty ? null : saved?.structurally_valid ?? null;
    return {structurally_valid: structure, read_only_connectable: structure,
        write_enabled: prerequisites[0].ready,
        basic_calibration_ready: prerequisites.every(item => item.ready) ? structure : false,
        basic_calibration_prerequisites: prerequisites};
}
function readinessPrerequisiteText(item) {
    const labels = {
        write_enabled: ['控制写入关闭，核对硬件后手动开启', 'Control writing is off; review hardware before enabling it'],
        drag_damping: ['各关节拖拽 Kd 必须为正值', 'Each drag Kd must be positive'],
        gravity: ['填写非零重力向量及各关节正值补偿比例', 'Set a nonzero gravity vector and positive compensation scales'],
        hold_gains: ['各关节刚性保持 Kp/Kd 须为正值且不超过安全上限', 'Rigid hold Kp/Kd must be positive and within safety limits']
    };
    return labels[item.id]?.[state.language === 'en' ? 1 : 0] || item.label || item.id;
}
function readinessPrerequisitesHtml(connection, {onlyMissing = true, draft = false} = {}) {
    const items = (connection.basic_calibration_prerequisites || []).filter(item => !onlyMissing || !item.ready);
    if (!items.length) return '';
    return `<div class="readiness-todos readiness-prerequisites" data-calibration-prerequisites>${items.map(item =>
        `<div data-prerequisite="${esc(item.id)}"><span>${esc(readinessPrerequisiteText(item))}${draft ? ` · ${busyText('编辑值，保存后生效','Draft · save to apply')}` : ''}</span>${item.action ? setupActionHtml(item.action,busyText('检查此项','Review setting')) : ''}</div>`).join('')}</div>`;
}
function readinessConnectionHtml(connection) {
    const unknown = busyText('保存后检查','Check after saving');
    const items = [
        ['structure',busyText('配置结构','Configuration'),connection.structurally_valid,busyText('有效','Valid'),busyText('待完善','Incomplete')],
        ['read_only',busyText('离线会话','Offline session'),connection.offline_connectable ?? connection.read_only_connectable,busyText('配置允许','Configured'),busyText('先补齐配置','Complete configuration first')],
        ['write',busyText('控制写入','Control writing'),connection.write_enabled,busyText('已开放','Enabled'),busyText('已关闭','Off')],
        ['calibration',busyText('基础标定条件','Calibration prerequisites'),connection.basic_calibration_ready,busyText('已具备','Configured'),busyText('待配置','Incomplete')]
    ];
    return `<section class="readiness-overview" aria-label="${busyText('连接与标定条件','Connection and calibration prerequisites')}"><div class="readiness-connection">${items.map(([id,label,ready,yes,no]) =>
        `<div data-readiness-condition="${id}"><b>${label}</b><span class="readiness-chip ${ready === true ? 'ready' : ready === false ? 'pending' : 'muted'}">${ready == null ? unknown : ready ? yes : no}</span></div>`).join('')}</div>${connection.write_enabled===false ? `<p class="hint">${busyText('写入关闭时使用离线后端，不连接电机；完成硬件核对并开放写入后，重新连接工作台以读取真实反馈','With writing off, the offline backend does not connect to motors. Review hardware, enable writing and reconnect for real feedback')}</p>` : ''}</section>`;
}
function readinessSectionHtml(key,title,description,body,defaultOpen=true) {
    const open=state.readinessSections?.[key] ?? defaultOpen;
    return `<details class="card readiness-fold" data-readiness-section="${esc(key)}" ${open?'open':''}><summary><span class="readiness-fold-title">${esc(title)}</span>${description?`<span class="readiness-fold-description">${esc(description)}</span>`:''}</summary><div class="readiness-fold-body">${body}</div></details>`;
}
function bindReadinessSections() {
    document.querySelectorAll('[data-readiness-section]').forEach(section=>section.ontoggle=()=>{
        if(section.isConnected===false)return;
        state.readinessSections={...(state.readinessSections || {}),[section.dataset.readinessSection]:section.open};
    });
}
function profileGroupForField(field = '') {
    if (/^(hardware|profile_entry|profile\.)/.test(field)) return 'hardware';
    if (field.startsWith('core.capability.') || field.startsWith('core.control.')) return 'motion';
    return profileGroups.find(([, , key]) => field.startsWith('core.' + key))?.[0] || 'model';
}
async function openSetupTarget(action) {
    if (!action?.page) return;
    const before = {page:state.page,group:state.profileGroup,tool:state.calibrationEntry,tab:state.workbenchTab,
        scroll:document.querySelector('.content')?.scrollTop || 0};
        await navigate(action.page);
    if (state.page !== action.page) return; // An unsaved form may cancel navigation
    if (action.page === 'profile_setup') {
        if (before.page !== 'profile_setup') state.setupReturn = before;
        state.profileGroup = action.group || profileGroupForField(action.field);
    }
    if (action.page === 'calibration' && (action.tool || action.entry)) state.calibrationEntry = action.tool || action.entry;
    if (action.page === 'geometry' && action.tool) state.geometryTab = action.tool;
    if (action.page === 'workbench' && action.tab) state.workbenchTab = action.tab;
    if (action.page === 'model' && action.joint) state.modelSelection = action.joint;
    state.setupFocus = action;
    render();
}
function focusSetupTarget() {
    const action = state.setupFocus;
    if (!action || action.page !== state.page) return;
    document.querySelectorAll('.setup-highlight').forEach(node=>node.classList.remove('setup-highlight'));
    let target = null;
    if (action.field) target = [...document.querySelectorAll('[data-profile-field]')].find(node => {
        const path = JSON.parse(node.dataset.profileField).join('.');
        return path === action.field || path.startsWith(action.field + '.');
    });
    if (action.field && !target && state.page === 'profile_setup') state.profileMissingTarget=action.field;
    else if (target && action.field) state.profileMissingTarget=null;
    if (!target && action.purpose) target = document.querySelector(`[data-gain-purpose="${action.purpose}"]`);
    if (!target && action.anchor) target = $(action.anchor);
    if (!target && action.page === 'profile_setup') target = document.querySelector('.parameter-card');
    if (!target && action.page === 'model') target = document.querySelector('.model-inspector');
    if (!target && ['calibration','geometry'].includes(action.page)) target = document.querySelector('.workspace-panel');
    if (target) {
        for (let parent = target; parent; parent = parent.parentElement) if (parent.tagName === 'DETAILS') parent.open = true;
        target = target.closest('.parameter-field') || target;
        if (action.field && target.classList.contains('parameter-field')) {target.classList.add('setup-highlight');setTimeout(()=>target.classList.remove('setup-highlight'),1800);}
        target.scrollIntoView({block:action.field ? 'center' : 'start',behavior:'instant'});
        const input = target.matches('input,select,textarea,button') ? target : target.querySelector('input:not(:disabled),select:not(:disabled),button');
        const focus = input?.classList.contains('studio-native-select') ? input.closest('.studio-select')?.querySelector('.ros-select-trigger') : input;
        if (action.field) focus?.focus({preventScroll:true});
    }
    state.setupFocus = null;
}
async function returnToPreparation() {
    const previous = state.setupReturn;
    await navigate(previous?.page || 'lifecycle');
    if (state.page === 'profile_setup') return;
    state.setupReturn = null;
    if (previous && state.page === previous.page) {state.profileGroup=previous.group;state.calibrationEntry=previous.tool;state.workbenchTab=previous.tab;render();}
    if (previous && state.page === previous.page) document.querySelector('.content')?.scrollTo({top:previous.scroll,behavior:'instant'});
}
function lifecyclePage() {
    const r = state.readiness || {}, checks = r.checks || {}, en = state.language === 'en';
    const connection = readinessConnectionState();
    const offlineConnectable = (connection.offline_connectable ?? connection.read_only_connectable) === true;
    const workspaceAvailable = r.control_available === true || offlineConnectable;
    const plugin = state.info?.hardware_plugin || r.info?.hardware_plugin || state.capabilityProfile?.profile_entry?.hardware?.plugin;
    const damiao = plugin === 'serial_arm_hardware_damiao';
    const row = (id, label, actionLabel) => {
        const recorded = checks[id] || 'pending';
        let value = recorded;
        let labelText = lifecycleStageLabel(id,value);
        if (id === 'dynamics_calibration') {
            const purposes = Object.values(r.gain_purposes || {}), verified = purposes.filter(item=>item.evidence_status === 'verified').length;
            const warning = purposes.some(item=>item.evidence_status === 'warning');
            value = purposes.length === 4 && verified === 4 ? 'verified' : purposes.some(item=>item.evidence_status === 'stale') ? 'stale' : warning ? 'warning' : verified ? 'partial' : 'pending';
            labelText = warning ? busyText('已采用实测增益，部分结果未达参考指标','Measured gains adopted; some results are outside reference limits') : verified === 4 ? busyText('四种用途均已验证','All four uses verified') : verified ? busyText('部分用途已验证','Some uses verified') : purposes.length === 4 && purposes.every(item=>item.configured) ? busyText('增益已配置，待实测验证','Gains configured · awaiting validation') : busyText('先配置初始增益，再进行标定','Configure initial gains before calibration');
        }
        if (id === 'bringup' && workbenchConnected()) {
            const x=state.telemetry || {}, fresh=telemetryFresh(x) && x.write_enabled === true;
            value=fresh ? 'verified' : recorded;labelText=x.write_enabled === false ? busyText('离线会话，不读取电机反馈','Offline session · no motor feedback') : fresh ? busyText('已连接，反馈正常','Connected · feedback current') : busyText('已连接，等待新鲜反馈','Connected · waiting for fresh feedback');
            if (recorded !== 'pending') labelText += ' · ' + readinessResultLabel(recorded);
        }
        const reviewStage = id === 'description' ? 'model_check' : id;
        const reviewStatus = checks[reviewStage] || 'pending';
        const reviewAction = readinessReviews[reviewStage] ? `<button type="button" class="button" data-review-stage="${reviewStage}">${['recorded','partial','verified','stale'].includes(reviewStatus) ? busyText('查看或更新自查','Review self-check') : busyText('记录自查结果','Record self-check')}</button>` : '';
        if (id === 'description' && checks.model_check && checks.model_check !== 'pending') labelText += ' · ' + busyText('人工检查：','Inspection: ') + readinessResultLabel(checks.model_check);
        return `<article class="lifecycle-step ${esc(value)}" data-lifecycle-stage="${esc(id)}"><span class="stage-dot"></span><div><b>${esc(label)}</b><small>${esc(labelText)}</small></div><div class="lifecycle-actions">${setupActionHtml(setupDestinations[id],actionLabel)}${reviewAction}</div></article>`;
    };
    const issues = r.setup_issues || [];
    const next = issues[0];
    const nextAction = next?.action || (next ? {page:'profile_setup',group:next.group,field:next.path} : {page:'diagnostics',anchor:'connection-settings'});
    const gainRows = ['RIGID','COMPLIANT','DRAG','SERVO_TRACKING'].map(purpose => {
        const item = r.gain_purposes?.[purpose] || {};
        return `<div class="readiness-gain-row"><div class="readiness-gain-heading"><b>${esc(gainPurposeLabel(purpose))}</b>${setupActionHtml({page:'profile_setup',group:'motion',purpose},busyText('查看参数','View gains'))}</div><div class="readiness-gain-status"><span>${item.configured ? busyText('增益已配置','Gains configured') : busyText('增益未配置','Gains missing')}</span><span class="${item.evidence_status === 'warning' ? 'readiness-chip warning' : ''}">${busyText('验证记录','Validation record')} · ${readinessResultLabel(item.evidence_status || 'pending')}</span></div>${(item.checks || []).length ? `<div class="readiness-gain-checks">${item.checks.map(check=>`<p class="${check.status === 'warning' ? 'warning-text' : ''}">${esc(gainCheckLabel(check.id,check.label))} · ${readinessResultLabel(check.status)}</p>`).join('')}</div>` : ''}</div>`;
    }).join('');
    const records = Object.keys(readinessReviews).filter(id=>id !== 'startup_hold').map(id=>`<div class="readiness-record"><span>${readinessStageText(id)} · ${readinessResultLabel(checks[id] || 'pending')}</span><button class="button" data-review-stage="${id}">${busyText('记录检查结果','Record check result')}</button></div>`).join('');
    const firstConnection = readinessSectionHtml('first-connection',busyText('机械臂首次接入','First robot connection'),busyText('硬件配置 → 关节映射 → 反馈核对 → 基础标定','Hardware → joint mapping → feedback checks → foundation calibration'),`<ol class="readiness-first-steps">
      <li><b>${busyText('配置硬件驱动与连接','Configure the hardware driver and connection')}</b><p>${damiao ? busyText('选择实际设备路径和总线，按接线填写每个关节的 motor_id、master_id 与 motor_type','Select the connected device and bus, and enter motor_id, master_id and motor_type for each joint') : busyText('选择所用驱动，按该驱动的配置文件填写通信连接与执行器参数','Select the driver and fill in its connection and actuator settings')}</p>${plugin?`<p class="hint">${busyText('当前驱动','Selected driver')} · <code>${esc(plugin)}</code></p>`:''}${setupActionHtml({page:'profile_setup',group:'hardware',field:damiao?'hardware.damiao.bus':'profile_entry.hardware.plugin'},busyText('硬件配置','Hardware configuration'))}</li>
      <li><b>${busyText('核对方向、比例与零位','Review direction, ratios and zero')}</b><p>${busyText('将关节名与实物、电机 ID 对应，核对正方向、位置比例及两侧零位','Match joint names to the mechanism and motor IDs; review positive direction, position ratio and both zero offsets')}</p>${setupActionHtml({page:'profile_setup',group:'mapping'},busyText('配置关节映射','Configure joint mapping'))}</li>
      <li><b>${busyText('核对使能条件','Review activation settings')}</b><p>${busyText('先核对刚性保持增益、位置与力矩限制；确认硬件后手动开放控制写入','Review rigid hold gains and position and effort limits; enable control writing after checking the hardware')}</p>${setupActionHtml({page:'profile_setup',group:'motion',field:'core.control.controller.rigid_hold.kp'},busyText('核对初始增益','Review initial gains'))}${setupActionHtml({page:'profile_setup',group:'safety',field:'core.safety_policy.max_acc'},busyText('核对安全限制','Review safety limits'))}</li>
      <li><b>${busyText('在工作台逐关节对照','Check each joint in the workspace')}</b><p>${busyText('选择关节，对照关节映射与实际反馈，完成实机检查后填写映射自查记录','Select a joint and compare its mapping with live feedback; save inspection notes after checking the hardware')}</p>${setupActionHtml({page:'workbench',tab:'control',anchor:'damiao-joint-check'},busyText('逐关节检查','Joint check'))}</li>
      <li><b>${busyText('验证重力补偿，再标定增益','Verify gravity compensation, then calibrate gains')}</b><p>${busyText('在当前机械臂与负载下完成基础标定，再使用拖拽、回放与导纳','Complete foundation calibration for the current robot and payload before dragging, playback and admittance')}</p>${setupActionHtml({page:'calibration',entry:'gravity_verify'},busyText('验证重力补偿','Verify gravity compensation'))}</li>
      </ol>`,false);
    return pageHead('readiness','readinessDesc', `<span class="readiness-chip ${r.control_available ? 'ready' : 'pending'}">${r.control_available ? busyText('配置检查通过','Configuration valid') : offlineConnectable ? busyText('可离线使用','Offline workspace available') : busyText('配置待完善','Configuration incomplete')}</span>`) +
      `<div class="readiness-summary card"><div><div class="eyebrow">${esc(state.config.profile || '—')}</div><h3>${r.control_available ? busyText('配置检查通过，下一步检查连接','Configuration valid · check the connection next') : offlineConnectable ? busyText('模型可离线使用，接入电机前完成硬件配置','The model is ready offline; complete hardware settings before connecting motors') : busyText('下一步：补齐配置','Next: complete configuration')}</h3><p>${esc((en && next?.path ? next.path + ': configure this field' : next?.message) || (r.control_available ? busyText('检查设备与关节反馈后，再使能机械臂','Inspect device and joint feedback before activation') : r.reason || busyText('打开参数配置查看缺失项','Open configuration to view missing fields')))}</p></div><div class="workspace-actions">${setupActionHtml(nextAction,next ? busyText('配置此项','Configure this item') : busyText('检查连接','Check connection'),true)}${workspaceAvailable ? `<button id="ready-control" class="button">${r.control_available ? busyText('进入控制工作台','Open workspace') : busyText('打开离线工作台','Open offline workspace')}</button>` : ''}</div></div>
      ${readinessConnectionHtml(connection)}${firstConnection}
      <div class="readiness-sections">
      ${readinessSectionHtml('configuration',busyText('连接前配置','Before connecting'),busyText('模型、硬件与关节映射','Model, hardware and joint mapping'),`<div class="lifecycle-list">${row('description',busyText('模型与控制链','Model and control chain'),busyText('查看模型','View model'))}${row('profile_setup',busyText('硬件与控制参数','Hardware and control parameters'),busyText('检查配置','Review configuration'))}${row('mapping_calibration',t('mappingStage'),busyText('配置关节映射','Configure joint mapping'))}</div>${issues.length ? `<div class="readiness-todos">${issues.map(issue=>`<div><span>${esc(issue.message)}</span>${setupActionHtml(issue.action || {page:'profile_setup',group:issue.group,field:issue.path},busyText('去配置','Configure'))}</div>`).join('')}</div>` : ''}`)}
      ${readinessSectionHtml('runtime',busyText('运行检查','Runtime checks'),busyText('连接反馈、运动与停放','Feedback, motion and parking'),`<div class="lifecycle-list">${row('bringup',busyText('设备连接与反馈','Connection and feedback'),busyText('检查连接','Check connection'))}${row('validation',busyText('运动与停放','Motion and parking'),busyText('检查运动与停放','Check motion and parking'))}</div>`)}
      ${readinessSectionHtml('foundation',busyText('基础标定','Foundation calibration'),busyText('先验证重力补偿，再标定执行器增益','Verify gravity compensation, then calibrate actuator gains'),`<p class="hint">${connection.basic_calibration_ready === false ? busyText('先补齐下列条件，再连接检查反馈；控制写入由你核对硬件后手动开启','Complete the settings below, then connect and inspect feedback; enable control writing after reviewing the hardware') : busyText('在当前机械臂与负载下完成验证，为拖拽、导纳和辨识建立基础','Validate the current robot and payload for dragging, admittance and identification')}</p>${readinessPrerequisitesHtml(connection)}<div class="lifecycle-list">${row('gravity_verification',t('gravityStage'),busyText('验证重力补偿','Verify gravity compensation'))}${row('dynamics_calibration',busyText('执行器增益自动标定','Automatic actuator gain calibration'),busyText('标定执行器增益','Calibrate actuator gains'))}</div>`)}
      ${readinessSectionHtml('specialist',busyText('专项标定','Specialist calibration'),busyText('按任务选择模型校正、动力学辨识或几何标定','Choose model calibration, dynamics identification or geometry calibration for the task'),`<div class="lifecycle-list">${row('gravity_model',busyText('重力模型校正','Gravity model calibration'),busyText('校正重力模型','Calibrate gravity model'))}${row('advanced_dynamics',busyText('高级动力学辨识','Advanced dynamics identification'),busyText('辨识动力学模型','Identify dynamics model'))}${row('geometry_calibration',t('geometryStage'),busyText('进入几何标定','Open geometry calibration'))}</div>`,false)}
      ${readinessSectionHtml('gains',busyText('控制配置检查','Control configuration'),busyText('四种增益用途的配置与验证记录','Configuration and validation for the four gain presets'),`${gainRows}<div class="workspace-actions"><button class="button" id="approve-gain-mode">${busyText('导入增益验证记录','Import gain validation')}</button></div>`,false)}
      ${readinessSectionHtml('records',busyText('检查记录','Inspection records'),busyText('观察结果与日志位置','Observations and log locations'),`<div class="readiness-inspection-records">${records}</div>`,false)}</div>`;
}
function refreshLifecycleBringup() {
    if(state.page!=='lifecycle' || !workbenchConnected())return;
    const row=document.querySelector('[data-lifecycle-stage="bringup"]');if(!row)return;
    const snapshot=state.telemetry || {},fresh=telemetryFresh(snapshot) && snapshot.write_enabled===true;
    const recorded=state.readiness?.checks?.bringup || 'pending';
    row.className='lifecycle-step '+(fresh?'verified':recorded);
    const label=row.querySelector('small');if(label)label.textContent=snapshot.write_enabled===false ?
        busyText('离线会话，不读取电机反馈','Offline session · no motor feedback') : fresh ?
        busyText('已连接，反馈正常','Connected · feedback current') : busyText('已连接，等待新鲜反馈','Connected · waiting for fresh feedback');
    if(label && recorded!=='pending')label.textContent+=' · '+readinessResultLabel(recorded);
}
function readinessReviewText(stage, key, fallback) {
    const labels = {
      model_check:{title:'Model inspection',chain:'Controlled chain, joints and frames match',limits:'Motion ranges and safety limits checked',inertial:'Inertial values and model structure checked'},
      bringup:{title:'Connection inspection',connection:'Motor communication and joint feedback checked while inactive'},
      startup_hold:{title:'Startup hold',hold_gains:'Hold gains and limits checked for every joint',supported:'Robot supported and workspace cleared',emergency_stop:'Emergency stop and disable available'},
      mapping_calibration:{title:'Joint mapping',joint_ids:'Joint-to-motor IDs checked physically',directions:'Motion directions checked physically',offsets:'Zeros and offsets checked physically'},
      geometry_calibration:{title:'Geometry calibration',frames:'Base and tool frames checked physically',zero_pose:'Zero pose matches the mechanism',tool_pose:'TCP and tool orientation checked'},
      gravity_verification:{title:'Gravity compensation verification',distinct_poses:'At least three distinct stable dragging poses recorded',support:'Support and emergency stop checked',restored:'Disabled with support and all temporary scales restored'},
      validation:{title:'Motion and parking inspection',tracking:'Trajectory following checked',stopping:'Parking and disable checked',faults:'Fault and emergency handling checked'},
    };
    return state.language === 'en' ? labels[stage]?.[key] || fallback : fallback;
}
function gainCheckLabel(key, fallback) {
    const en = {tracking:'Trajectory following',hold:'Position hold',hold_compensation:'Hold compensation state',drag:'Stable dragging',native_tracking:'Native impedance tracking',admittance_tracking:'Model admittance tracking'};
    return state.language === 'en' ? en[key] || key : fallback || key;
}
function showReadinessReview(stage) {
    const review = readinessReviews[stage]; if (!review) return;
    const dialog = $('confirm-dialog');
    const previous = state.readiness?.evidence?.[stage] || {};
    const recorded = previous.observations || {};
    dialog.innerHTML = `<h3>${esc(readinessReviewText(stage,'title',review.title))} · ${busyText('检查记录','Inspection record')}</h3><p class="hint">${busyText('记录本次观察结果与日志位置','Record observations and log locations')}${stage === 'gravity_verification' ? busyText('；已完成的原生验证将另存实测结果','; completed native verification is recorded separately') : ''}</p><div class="readiness-review-items">${review.keys.map(([name, label]) => `<label><input type="checkbox" data-review-key="${name}" ${recorded[name] === true ? 'checked' : ''}>${esc(readinessReviewText(stage,name,label))}</label>`).join('')}</div><label class="field"><span>${busyText('检查记录','Inspection notes')}</span><textarea id="readiness-review-note" rows="3" maxlength="600" placeholder="${busyText('选填，记录观察结果或日志位置','Optional observations or log location')}">${esc(previous.note || '')}</textarea></label><div class="actions"><button class="button" id="readiness-review-cancel">${t('cancel')}</button><button class="button primary" id="readiness-review-submit">${busyText('保存记录','Save record')}</button></div>`;
    dialog.showModal();
    $('readiness-review-cancel').onclick = () => dialog.close();
    $('readiness-review-submit').onclick = () => {
        const answers = { note: $('readiness-review-note').value };
        dialog.querySelectorAll('[data-review-key]').forEach(node => { answers[node.dataset.reviewKey] = node.checked; });
        dialog.close();
        blockingOperation(busyText('正在保存检查记录','Saving inspection record'), async () => {
            state.readiness = await api.request('readiness_record', { config: state.config, stage, answers });
            await refreshLibrary(); render();
        });
    };
}
async function showGainAdmission() {
    const candidate_path = await api.selectGainCandidate();
    if (!candidate_path) return;
    await blockingOperation(busyText('正在核验增益记录','Checking gain evidence'),async()=>{
        state.readiness = await api.request('readiness_approve_mode',{config:state.config,mode:'all',candidate_path});
        await refreshLibrary(); render();
    });
}
// One authoritative draft is shared by creation and continued configuration
const profileGroups = [
    ['model', '模型与重力', 'model'],
    ['mapping', '关节映射', 'calibration'],
    ['motion', '运动与交互控制', 'control'],
    ['safety', '安全策略', 'safety_policy'],
    ['shutdown', '安全停放', 'shutdown'],
    ['hardware', '硬件与总线', 'hardware'],
];
const profileEnums = {
    'core.control.runtime.model_feedforward_mode': ['NONE', 'GRAVITY', 'FULL_INVERSE_DYNAMICS'],
    'core.control.runtime.tracking_gain_profile': ['RIGID', 'COMPLIANT', 'SERVO_TRACKING'],
    'core.safety_policy.fault_recovery.default_mode': ['rigid_hold'],
    'core.capability.admittance.observer.mode': ['MOMENTUM', 'FULL_ID'],
    'core.capability.admittance.reference_behavior': ['NATIVE_IMPEDANCE', 'MODEL_ADMITTANCE'],
};
// GUI suggestions mirror the existing Damiao Backend parser
// Other plugins and transport backends remain editable by their own exact identifiers
const knownMotorTypes = ['DM4310', 'DM4310_48V', 'DM4340', 'DM4340_48V', 'DM6006', 'DM6248P', 'DM8006', 'DM8009', 'DM10010L', 'DM10010', 'DMH3510', 'DMH6215', 'DMG6220', 'DMJH11'];
const knownHardwarePlugins = ['serial_arm_hardware_damiao'];
const knownBusTypes = ['can'];
const knownBusBackends = ['damiao_usb2can'];
function hardwareFieldOptions(path) {
    const joined = path.join('.');
    if (joined === 'profile_entry.hardware.plugin') return ['', ...knownHardwarePlugins];
    if (/^hardware\.buses\.[^.]+\.type$/.test(joined)) return knownBusTypes;
    if (/^hardware\.buses\.[^.]+\.backend$/.test(joined)) return knownBusBackends;
    if (/^hardware\.damiao\.actuators\.[^.]+\.motor_type$/.test(joined)) return ['', ...knownMotorTypes];
    return null;
}
function hardwareChoiceField(path, value, disabled = false) {
    const options = hardwareFieldOptions(path);
    if (!options) return null;
    const custom = !options.includes(value);
    const data = esc(JSON.stringify(path));
    const label = state.language === 'en' ? 'Custom identifier…' : '自定义标识…';
    const note = state.language === 'en' ? 'Custom values require support from the actual installed Backend' : '自定义项必须由实际安装的硬件后端支持';
    const labelFor = x => x || (state.language === 'en' ? 'Not configured' : '未配置');
    return `<div class="hardware-choice" data-hardware-choice="${data}"><select data-choice-selector="${data}" ${disabled ? 'disabled' : ''} aria-label="${esc(path.at(-1))}">${options.map(x => `<option value="${esc(x)}" ${!custom && x === value ? 'selected' : ''}>${esc(labelFor(x))}</option>`).join('')}<option value="__custom__" ${custom ? 'selected' : ''}>${label}</option></select><input type="text" data-profile-field="${data}" value="${esc(value)}" class="hardware-custom-value" ${custom ? '' : 'hidden'} ${disabled ? 'disabled' : ''} placeholder="${label}"><small class="hardware-custom-hint" ${custom ? '' : 'hidden'}>${note}</small></div>`;
}
const profileLabels = {
    urdf_path: 'URDF 路径', joint_names: '受控关节顺序', base_frame: '基坐标系', tool_frame: '工具坐标系',
    gravity: '重力向量', gravity_scale: '重力补偿比例', gravity_correction_path: '重力校正文件',
    joints: '关节标定参数', direction: '方向', pos_ratio: '位置比例', tor_ratio: '力矩比例',
    joint_zero_offset: '关节零位偏置', actuator_zero_offset: '电机零位偏置',
    ctrl_frequency_hz: '控制频率 Hz', joint_acc_filter_alpha: '加速度滤波系数',
    write_enabled: '允许硬件命令写入', model_feedforward_mode: '模型前馈', tracking_gain_profile: '默认跟踪增益用途', tracking_impedance_mode: '兼容跟踪任务',
    allow_full_cmd: '允许完整关节命令', rigid_hold: '刚性保持', rigid_tracking: '刚性跟踪',
    compliant_hold: '柔性保持', compliant_drag: '拖拽', compliant_tracking: '柔性跟踪',
    servo_tracking: '伺服跟踪', identification_tracking: '伺服跟踪', kp: '位置刚度 Kp', kd: '速度阻尼 Kd',
    position_margin: '关节位置余量 rad', cmd_vel_scale: '命令速度比例', state_vel_scale: '状态速度比例',
    max_acc: '关节最大加速度 rad/s²', max_effort_override: 'PD 与前馈总力矩预算 Nm',
    max_kp_override: 'Kp 安全上限', max_kd_override: 'Kd 安全上限',
    max_dt_s: '最大控制周期 s', state_timeout_s: '反馈超时 s', cmd_timeout_s: '命令超时 s',
    require_all_actuators_online: '要求全部电机在线', require_all_actuators_enabled: '要求全部电机使能',
    reject_motor_error: '拒绝电机故障', require_continuous_cmd: '强制命令连续',
    fault_recovery: '故障恢复', default_mode: '故障保持模式', allow_compliant_recovery: '允许受限柔性恢复',
    require_operator_request: '恢复需人工请求', gravity_model_validated: '重力模型已验证',
    recovery_timeout_s: '恢复超时 s', compliant_recovery: '柔性恢复参数',
    max_vel: '最大恢复速度 rad/s', effort_scale: '力矩比例',
    park_before_disable: '停放后失能', park_pos: '停放目标 rad', speed_scale: '停放速度比例',
    compliant_phase_ratio: '柔性回位时间比例', position_tolerance: '位置容差 rad',
    velocity_tolerance: '速度容差 rad/s', settle_time_s: '到位稳定时间 s',
    relaxed_tolerance_ratio: '超时前宽松容差倍率', timeout_s: '停放超时 s',
    enabled: '启用', reference_behavior: '控制方式', joint_enabled: '参与导纳的关节', observer: '力矩观测器',
    mode: '模式', momentum_gain: '动量观测器增益', calibration: '标定参数',
    torque_bias: '残差偏置 Nm', torque_threshold: '残差死区 Nm', friction: '摩擦补偿',
    velocity_transition: '摩擦速度过渡 rad/s', positive_coulomb: '正向库仑项',
    positive_viscous: '正向黏性项', negative_coulomb: '反向库仑项', negative_viscous: '反向黏性项',
    hold_compensation: '刚性定位保持补偿', integral_gain: '保持积分增益', max_torque: '补偿力矩上限 Nm', max_rate: '补偿变化率 Nm/s', contact_threshold: '接触冻结阈值 Nm', velocity_threshold: '积分速度门槛（关节单位/s）', error_threshold: '积分误差门槛（关节单位）', controller: '控制参数', mass: '虚拟质量 M', damping: '虚拟阻尼 D', stiffness: '虚拟刚度 K',
    max_delta_q: '最大位置退让 rad', max_delta_q_dot: '最大速度退让 rad/s',
    buses: '总线配置', damiao: '达妙执行器', bus: '使用总线', type: '总线类型', backend: '通信后端',
    device: '设备路径', baudrate: '波特率', refresh_state_in_read: '读取时刷新状态',
    feedback_timeout_s: '电机反馈超时 s', activation_retries: '使能重试次数',
    startup_read_cycles: '启动读取周期数', stop_kp: '停止 Kp', stop_kd: '停止 Kd',
    stop_cycles: '停止周期数', actuators: '执行器映射', name: '名称',
    motor_id: '电机 ID', master_id: '主机 ID', motor_type: '电机型号', plugin: '硬件驱动',
};
const profileLabelsEn = {
 urdf_path:'URDF file',joint_names:'Controlled joint order',base_frame:'Base frame',tool_frame:'Tool frame',gravity:'Gravity vector',gravity_scale:'Gravity scale',gravity_correction_path:'Gravity correction file',
 joints:'Joint mapping',direction:'Direction',pos_ratio:'Position ratio',tor_ratio:'Torque ratio',joint_zero_offset:'Joint zero offset',actuator_zero_offset:'Actuator zero offset',ctrl_frequency_hz:'Control frequency Hz',joint_acc_filter_alpha:'Acceleration filter',write_enabled:'Enable actuator writes',model_feedforward_mode:'Model feedforward',tracking_impedance_mode:'Legacy tracking task',tracking_gain_profile:'Default tracking gain use',allow_full_cmd:'Allow complete joint commands',
 rigid_hold:'Rigid hold',rigid_tracking:'Rigid tracking',compliant_hold:'Compliant hold',compliant_drag:'Drag',compliant_tracking:'Compliant tracking',servo_tracking:'Servo tracking',identification_tracking:'Servo tracking',kp:'Position stiffness Kp',kd:'Velocity damping Kd',
 position_margin:'Position margin rad',cmd_vel_scale:'Command velocity scale',state_vel_scale:'Measured velocity scale',max_acc:'Maximum acceleration rad/s²',max_effort_override:'Total command effort limit Nm',max_kp_override:'Kp limit',max_kd_override:'Kd limit',max_dt_s:'Maximum cycle interval s',state_timeout_s:'Feedback timeout s',cmd_timeout_s:'Command timeout s',require_all_actuators_online:'Require all actuators online',require_all_actuators_enabled:'Require all actuators enabled',reject_motor_error:'Reject motor faults',require_continuous_cmd:'Require continuous commands',fault_recovery:'Fault recovery',default_mode:'Recovery hold mode',allow_compliant_recovery:'Allow compliant recovery',require_operator_request:'Require operator request',gravity_model_validated:'Gravity model verified',recovery_timeout_s:'Recovery timeout s',compliant_recovery:'Compliant recovery',max_vel:'Maximum recovery velocity rad/s',effort_scale:'Effort scale',
 park_before_disable:'Park before disable',park_pos:'Park target rad',speed_scale:'Parking speed scale',compliant_phase_ratio:'Compliant phase ratio',position_tolerance:'Position tolerance rad',velocity_tolerance:'Velocity tolerance rad/s',settle_time_s:'Settling time s',relaxed_tolerance_ratio:'Timeout tolerance ratio',timeout_s:'Parking timeout s',enabled:'Enabled',reference_behavior:'Control method',joint_enabled:'Participating joints',observer:'External torque observer',mode:'Mode',momentum_gain:'Momentum observer gain',calibration:'Calibration',torque_bias:'Torque bias Nm',torque_threshold:'Torque deadband Nm',friction:'Friction compensation',velocity_transition:'Friction transition rad/s',positive_coulomb:'Positive Coulomb term',positive_viscous:'Positive viscous term',negative_coulomb:'Negative Coulomb term',negative_viscous:'Negative viscous term',hold_compensation:'Rigid hold compensation',integral_gain:'Integral gain',max_torque:'Compensation limit Nm',max_rate:'Compensation slew Nm/s',contact_threshold:'Contact threshold Nm',velocity_threshold:'Velocity gate (joint units/s)',error_threshold:'Error gate (joint units)',controller:'Control parameters',mass:'Virtual mass M',damping:'Virtual damping D',stiffness:'Virtual stiffness K',max_delta_q:'Maximum displacement rad',max_delta_q_dot:'Maximum offset velocity rad/s',
 buses:'Buses',damiao:'Damiao actuators',bus:'Assigned bus',type:'Bus type',backend:'Transport backend',device:'Device path',baudrate:'Baudrate',refresh_state_in_read:'Refresh during read',feedback_timeout_s:'Motor feedback timeout s',activation_retries:'Activation retries',startup_read_cycles:'Startup read cycles',stop_kp:'Stop Kp',stop_kd:'Stop Kd',stop_cycles:'Stop cycles',actuators:'Actuator assignments',name:'Name',motor_id:'Motor ID',master_id:'Master ID',motor_type:'Motor type',plugin:'Hardware driver',
};
const paramLabel = key => (state.language === 'en' ? profileLabelsEn[key] : profileLabels[key]) || key;
function profileOptionLabel(value) {
    if (canonicalGainPurpose(value)) return gainPurposeLabel(value);
    const labels = {
        NATIVE_IMPEDANCE:['原生阻抗','Native impedance'],MODEL_ADMITTANCE:['模型导纳','Model admittance'],
        NONE:['关闭','Off'],GRAVITY:['重力补偿','Gravity compensation'],FULL_INVERSE_DYNAMICS:['完整动力学','Full dynamics'],
        MOMENTUM:['动量观测器','Momentum observer'],FULL_ID:['逆动力学观测器','Inverse dynamics observer'],
    };
    return labels[value]?.[state.language === 'en' ? 1 : 0] || value || busyText('未配置','Not configured');
}
function robotStateLabel(value) {
    const labels = {ACTIVE:['已使能','Active'],INACTIVE:['未使能','Inactive'],FAULT:['故障','Fault'],OFFLINE:['未连接','Offline']};
    return labels[value]?.[state.language === 'en' ? 1 : 0] || value || '—';
}
function gainSourceLabel(value) {
    const labels = {CONFIGURED:['配置默认','Profile default'],CALIBRATION_OVERRIDE:['标定试用','Calibration trial'],PARK_OVERRIDE:['停放临时参数','Parking'],FULL_COMMAND:['完整命令','Full command'],TRANSITION:['过渡中','Transitioning']};
    return labels[value]?.[state.language === 'en' ? 1 : 0] || value || '—';
}
function profileGet(draft, path) { return path.reduce((item, key) => item?.[key], draft); }
function profileField(path, value) {
    const id = path.join('.');
    let label = id === 'core.capability.admittance.enabled' ? (state.language === 'en' ? 'External observation and interaction calculation' : '外力观测与交互计算') : id === 'core.capability.admittance.reference_behavior' ? (state.language === 'en' ? 'Default control method' : '默认控制方式') : paramLabel(path.at(-1));
    const joint = path.find(part => state.profileEditor?.joint_names?.includes(part));
    const linear = joint && state.profileEditor?.urdf_joint_limits?.[joint]?.type === 'prismatic';
    if (linear) label = label.replace(/rad/g, 'm').replace(/Nm/g, 'N');
    const readonly = id === 'core.model.joint_names';
    const opts = profileEnums[id] ? [...new Set([...profileEnums[id], String(value)])] : null;
    const hardwareChoice = typeof value === 'string' ? hardwareChoiceField(path, value, readonly) : null;
    const data = `data-profile-field="${esc(JSON.stringify(path))}"`;
    const units = id.includes('gravity_model_validated') ? (state.language === 'en' ? 'Only enable after real gravity compensation testing on this exact robot and payload; takes effect after reconnect' : '仅在当前机械臂与负载已完成真机重力补偿验证后开启 · 重新连接工作台后生效') :
        id.includes('joint_names') ? (state.language === 'en' ? 'Determined when creating the control chain' : '关节顺序由新建时选择的控制链决定') :
            id === 'core.capability.admittance.enabled' ? (state.language === 'en' ? 'Native impedance: observe only; model admittance: observe and generate the virtual response' : '原生阻抗下仅观测，模型导纳下同时观测并生成虚拟响应') : '';
    let input;
    if (hardwareChoice) input = hardwareChoice;
    else if (opts) input = `<select ${data} ${readonly ? 'disabled' : ''}>${opts.map(o => `<option value="${esc(o)}" ${value === o ? 'selected' : ''}>${esc(profileOptionLabel(o))}</option>`).join('')}</select>`;
    else if (typeof value === 'boolean') input = `<div class="parameter-switch" data-switch-state="${value ? 'on' : 'off'}"><span class="switch-value switch-on">ON</span><input type="checkbox" role="switch" aria-label="${esc(label)}" aria-checked="${value}" ${data} ${value ? 'checked' : ''} ${readonly ? 'disabled' : ''}><span class="switch-value switch-off">OFF</span></div>`;
    else if (typeof value === 'number') input = `<input type="number" step="any" ${data} value="${esc(value)}" ${readonly ? 'disabled' : ''}>`;
    else if (typeof value === 'string') input = `<input type="text" ${data} value="${esc(value)}" ${readonly ? 'disabled' : ''}>`;
    else if (value === null) input = `<input type="text" ${data} value="" placeholder="${busyText('填写配置值','Enter a value')}" ${readonly ? 'disabled' : ''}>`;
    else if (Array.isArray(value)) input = `<input type="text" ${data} value="${esc(value.join(', '))}" ${readonly ? 'disabled' : ''} title="${state.language === 'en' ? 'Comma-separated values' : '用英文逗号分隔多个值'}">`;
    else return '';
    const required = id === 'profile_entry.hardware.plugin' || /^hardware\.damiao\.actuators\.[^.]+\.motor_type$/.test(id) || /^hardware\.buses\.[^.]+\.(type|backend|device)$/.test(id);
    const missingValue = required && typeof value === 'string' && !value.trim();
    const fieldHint = missingValue ? (state.language === 'en' ? 'Required for a real hardware connection' : '连接真实硬件前填写') : '';
    const err = state.profileFieldErrors?.[id] || (state.profileError?.startsWith(id) ? state.profileError : '');
    return `<label class="parameter-field ${err || missingValue ? 'invalid' : ''}"><span>${esc(label)}${required ? `<strong class="parameter-required">${state.language === 'en' ? 'Hardware required' : '实机必填'}</strong>` : ''}</span>${input}${units ? `<small>${esc(units)}</small>` : ''}<details class="parameter-key"><summary>${busyText('配置键','Configuration key')}</summary><code>${esc(id)}</code></details>${fieldHint ? `<em>${fieldHint}</em>` : ''}${err ? `<em data-field-error>${esc(err)}</em>` : ''}</label>`;
}
function profileTree(value, path, depth = 0) {
    if (value && !Array.isArray(value) && typeof value === 'object') {
        const children = Object.entries(value).map(([key, child]) => profileTree(child, [...path, key], depth + 1)).join('');
        if (depth === 0) return `<div class="parameter-fields">${children}</div>`;
        const leaves = Object.keys(value).length;
        const isHardware = path[0] === 'hardware';
        if (!isHardware && depth > 1 && leaves <= 12 && Object.values(value).every(item => item === null || typeof item !== 'object' || Array.isArray(item)))
            return `<fieldset class="parameter-subgroup"><legend>${esc(paramLabel(path.at(-1)))}</legend><div class="parameter-fields">${children}</div></fieldset>`;
        const expanded = isHardware ? (depth <= 1 || !!state.profileError?.includes(path.join('.'))) : true;
        return `<details class="parameter-subgroup ${isHardware ? 'hardware-subgroup' : ''}" ${expanded ? 'open' : ''}><summary>${esc(paramLabel(path.at(-1)))}${isHardware ? `<small>${leaves} ${state.language === 'en' ? 'items' : '项'}</small>` : ''}</summary><div class="parameter-fields">${children}</div></details>`;
    }
    return profileField(path, value);
}
function profileHardwareDefaults(names) {
    const a = Object.fromEntries(names.map((j, i) => [j, { name: `actuator${i + 1}`, motor_id: i + 1, master_id: 0, motor_type: '' }]));
    return { buses: { main_can: { type: 'can', backend: 'damiao_usb2can', device: '/dev/ttyACM0', baudrate: 921600 } }, damiao: { bus: 'main_can', refresh_state_in_read: false, feedback_timeout_s: 0.05, activation_retries: 3, startup_read_cycles: 5, stop_kp: 10, stop_kd: 0.15, stop_cycles: 5, actuators: a } };
}
// Configuration requirements are separate from operator verification records
function profileGuidanceData(editor) {
    const en = state.language === 'en';
    const core = editor?.core || {}, hardware = editor?.hardware || {};
    const names = editor?.joint_names || core.model?.joint_names || [];
    const plugin = String(editor?.profile_entry?.hardware?.plugin || '').trim();
    const missing = [];
    const add = (path, message) => missing.push({ path, message });
    if (!plugin) add('profile_entry.hardware.plugin', en ? 'Select a hardware driver' : '请选择硬件驱动');
    if (plugin === 'serial_arm_hardware_damiao') {
        const buses = hardware.buses || {}, damiao = hardware.damiao || {};
        const bus = damiao.bus;
        if (!bus || !buses[bus]) add('hardware.damiao.bus', en ? 'Select a configured bus' : '请选择此驱动使用的已配置总线');
        else {
            for (const key of ['type', 'backend', 'device']) if (!String(buses[bus]?.[key] || '').trim())
                add(`hardware.buses.${bus}.${key}`, (en ? 'Missing bus ' : '总线缺少 ') + paramLabel(key));
            const baud = Number(buses[bus]?.baudrate);
            if (!Number.isInteger(baud) || baud <= 0) add(`hardware.buses.${bus}.baudrate`, en ? 'Set a positive baudrate' : '波特率必须是正整数');
        }
        const actuators = damiao.actuators || {};
        const ids = new Set();
        for (const joint of names) {
            const item = actuators[joint];
            if (!item) { add(`hardware.damiao.actuators.${joint}`, `${joint} ${en ? 'actuator mapping is missing' : '缺少执行器映射'}`); continue; }
            const motor = String(item.motor_type || '').trim();
            if (!motor) add(`hardware.damiao.actuators.${joint}.motor_type`, `${joint} ${en ? 'motor type not selected' : '未选择电机型号'}`);
            else if (!knownMotorTypes.includes(motor)) add(`hardware.damiao.actuators.${joint}.motor_type`, `${joint} ${en ? 'unsupported motor type: ' : '驱动不支持该型号 '}${motor}`);
            const id = item.motor_id;
            if (!Number.isInteger(id) || id <= 0) add(`hardware.damiao.actuators.${joint}.motor_id`, `${joint} ${en ? 'motor ID must be positive' : '电机 ID 必须是正整数'}`);
            else if (ids.has(id)) add(`hardware.damiao.actuators.${joint}.motor_id`, `${joint} ${en ? 'motor ID duplicated' : '电机 ID 重复'}`);
            else ids.add(id);
        }
    }
    const safety = core.safety_policy || {}, accel = safety.max_acc || {};
    const safetyMissing = names.filter(j => !(typeof accel[j] === 'number' && Number.isFinite(accel[j]) && accel[j] > 0));
    const urdf = editor?.urdf_joint_limits || {};
    const urdfWarnings = [];
    for (const joint of names) {
        const limit = urdf[joint];
        if (!limit) { urdfWarnings.push(`${joint} ${en ? 'limit data unavailable' : '没有可读取的限位数据'}`); continue; }
        if (['revolute', 'prismatic'].includes(limit.type) &&
            (!Number.isFinite(limit.lower) || !Number.isFinite(limit.upper) || limit.lower >= limit.upper))
            urdfWarnings.push(`${joint} ${en ? 'position limits missing or invalid' : '位置上下限缺失或无效'}`);
        if (!Number.isFinite(limit.velocity) || limit.velocity <= 0)
            urdfWarnings.push(`${joint} ${en ? 'velocity limit missing or invalid' : '速度上限缺失或无效'}`);
    }
    return { missing, safetyMissing, urdfWarnings, plugin, names };
}
function profileGuidanceHtml(editor) {
    const en = state.language === 'en', data = profileGuidanceData(editor), safety = editor.core?.safety_policy || {};
    const connection = readinessConnectionState(editor);
    const pending = [...data.missing.map(item=>({...item,group:'hardware'})), ...data.safetyMissing.map(joint=>({group:'safety',path:`core.safety_policy.max_acc.${joint}`,message:`${joint} ${en ? 'maximum acceleration is missing' : '缺少最大加速度'}`}))];
    for (const [path,message] of Object.entries(state.profileFieldErrors || {})) if (!pending.some(item=>item.path === path)) pending.unshift({path,message,group:profileGroupForField(path)});
    const first = pending[0];
    const text = value => typeof value === 'object' ? Object.entries(value || {}).map(([key,v])=>`${key}: ${v}`).join(' / ') : String(value ?? '—');
    const action = (group,field,label) => setupActionHtml({page:'profile_setup',group,field},label);
    return `<section class="profile-setup-guide"><div class="profile-guide-head"><div><h3>${first ? busyText('下一步：','Next: ') + esc(first.message) : busyText('必填项已填写，继续核对映射与运行限制','Required fields are present · review mapping and limits')}</h3><p>${busyText('保存后重新检查准备状态，连接工作台读取实际反馈','Save and recheck preparation, then connect to read live feedback')}</p></div><span class="profile-guide-count">${pending.length ? `${pending.length} ${busyText('项待填写','fields to complete')}` : busyText('必填项已填写','Required fields present')}</span></div>
      ${pending.length ? `<div class="readiness-todos">${pending.map(item=>`<div><span>${esc(item.message)}</span>${action(item.group,item.path,busyText('配置此项','Configure'))}</div>`).join('')}</div>` : ''}
      ${readinessConnectionHtml(connection)}
      ${connection.basic_calibration_prerequisites.some(item => !item.ready) ? `<div class="profile-calibration-guide"><b>${busyText('开始基础标定前','Before foundation calibration')}</b><p class="hint">${busyText('以下按当前编辑值检查；核对硬件后自行开启写入，保存并重新连接后生效','Checked against the current draft; review hardware before enabling writing, then save and reconnect')}</p>${readinessPrerequisitesHtml(connection,{draft:true})}</div>` : ''}
      <details class="profile-review-defaults"><summary>${busyText('核对当前值与实机','Review values against hardware')}</summary><div class="profile-guide-grid"><section><b>${busyText('关节映射','Joint mapping')}</b><p>${busyText('逐关节核对电机 ID、方向、比例和零位','Check motor ID, direction, ratios and zero for each joint')}</p>${action('mapping',`core.calibration.joints.${data.names[0] || ''}`,busyText('检查映射','Review mapping'))}</section><section><b>${busyText('运行限制','Runtime limits')}</b><p>${busyText('最大加速度','Maximum acceleration')} · ${esc(text(safety.max_acc))}</p><p>${busyText('反馈超时','Feedback timeout')} · ${esc(text(safety.state_timeout_s))} s</p>${action('safety','core.safety_policy.max_acc',busyText('检查安全策略','Review safety policy'))}</section><section><b>${busyText('模型限位','Model limits')}</b><p>${data.urdfWarnings.length ? esc(data.urdfWarnings.join('；')) : busyText('位置、速度和力矩限值来自 URDF','Position, velocity and effort bounds come from the URDF')}</p>${setupActionHtml({page:'model',joint:data.names[0]},busyText('查看关节与限位','Inspect joint limits'))}</section></div></details></section>`;
}
function refreshProfileGuidance() {
    const container = document.getElementById('profile-guide-container');
    if (container && state.profileEditor) { container.innerHTML = profileGuidanceHtml(state.profileEditor); container.querySelectorAll('[data-setup-target]').forEach(button=>button.onclick=()=>operation(()=>openSetupTarget(JSON.parse(button.dataset.setupTarget)))); }
}
function profileUrdfLimitsHtml(editor) {
    const en = state.language === 'en';
    const limits = editor?.urdf_joint_limits || {};
    const joints = editor?.joint_names || [];
    const fmt = value => Number.isFinite(value) ? Number(value).toFixed(4) : '—';
    const rows = joints.map(joint => {
        const item = limits[joint] || {};
        const hasPosition = !['revolute', 'prismatic'].includes(item.type) || (Number.isFinite(item.lower) && Number.isFinite(item.upper) && item.lower < item.upper);
        const hasSpeed = Number.isFinite(item.velocity) && item.velocity > 0;
        return `<tr class="${hasPosition && hasSpeed ? '' : 'profile-limit-warning'}"><td><b>${esc(joint)}</b></td><td>${esc(item.type || '—')}</td><td>${item.type === 'continuous' ? (en ? 'Unbounded' : '连续旋转') : fmt(item.lower) + (item.type === 'prismatic' ? ' m' : ' rad')}</td><td>${item.type === 'continuous' ? (en ? 'Unbounded' : '连续旋转') : fmt(item.upper) + (item.type === 'prismatic' ? ' m' : ' rad')}</td><td>${fmt(item.velocity)} ${item.type === 'prismatic' ? 'm/s' : 'rad/s'}</td></tr>`;
    }).join('');
    return `<details class="profile-limit-review" ${state.profileGroup === 'safety' ? 'open' : ''}><summary>${en ? 'Review original URDF limits' : '查看 URDF 原始关节限位'}</summary><p>${en ? 'Read-only values from the current URDF file. Check physical zero, direction, position and speed limits. Edit the source URDF to correct incorrect values' : '以下为当前 URDF 中的只读限位值，必须结合实机核对零位、方向、位置上下限和最大速度，发现错误时应修改原始 URDF 文件'}</p><div class="table-scroll"><table class="profile-limit-table"><thead><tr><th>Joint</th><th>Type</th><th>${en ? 'Lower' : '下限'}</th><th>${en ? 'Upper' : '上限'}</th><th>${en ? 'Velocity' : '速度'}</th></tr></thead><tbody>${rows || `<tr><td colspan="5">${en ? 'No controlled joint limit data' : '未读取到受控关节限位'}</td></tr>`}</tbody></table></div><div class="workspace-actions">${setupActionHtml({page:'model',joint:joints[0]},busyText('查看关节与限位','Inspect joint limits'))}<button type="button" class="button" data-urdf-source>${busyText('定位 URDF 源文件','Show URDF source file')}</button></div></details>`;
}
function profileGainPurposeHtml(editor) {
    const en = state.language === 'en';
    return `<section class="calibration-step"><div class="workspace-section-head"><div><h3>${en ? 'Actuator gains by use' : '执行器增益 · 四种用途'}</h3><p class="hint">${en ? 'Choose Rigid, Compliant, Drag or Servo tracking. Holding and trajectory tracking are task parameters within a use' : '统一使用刚性、柔性、拖拽、伺服跟踪；定位保持与轨迹跟踪是用途内的任务参数'}</p></div></div><div class="profile-gain-purpose-grid">${gainPurposeSettings(editor.core.control.controller).map(group => `<details class="profile-gain-purpose" data-gain-purpose="${group.purpose}"><summary>${esc(gainPurposeLabel(group.purpose))}</summary><p class="hint">${esc(group.description)}</p>${group.tasks.map(task => `<div class="profile-gain-task"><b>${esc(task.label)}</b>${profileTree(task.gains, ['core', 'control', 'controller', task.key])}</div>`).join('') || `<p class="hint">${en ? 'No gain preset configured' : '尚未配置此用途增益'}</p>`}</details>`).join('')}</div><div class="workspace-actions"><button type="button" class="button" id="profile-open-auto-calibration">${en ? 'Open automatic actuator gain calibration' : '前往执行器增益自动标定'}</button></div></section>`;
}
function gainPurposeSettings(gains = {}) {
    const en = state.language === 'en';
    const task = (key, zh, english) => ({key, label: en ? english : zh, gains: gains[key]});
    return [
        {purpose:'RIGID', description:en ? 'Position holding and trajectory tracking' : '用于定位保持与轨迹跟踪', tasks:[task('rigid_hold','定位保持','Position hold'),task('rigid_tracking','轨迹跟踪','Trajectory tracking')]},
        {purpose:'COMPLIANT', description:en ? 'Lower-stiffness holding and trajectory tracking' : '以较低刚度保持位置或跟踪轨迹', tasks:[task('compliant_hold','定位保持','Position hold'),task('compliant_tracking','轨迹跟踪','Trajectory tracking')]},
        {purpose:'DRAG', description:en ? 'Gravity-compensated manual dragging and damping' : '用于重力补偿下的手动拖拽与速度阻尼', tasks:[task('compliant_drag','拖拽参数','Drag parameters')]},
        {purpose:'SERVO_TRACKING', description:en ? 'Track virtual references and identification trajectories' : '跟踪虚拟参考与辨识轨迹', tasks:[task(gains.servo_tracking ? 'servo_tracking' : 'identification_tracking','跟踪参数','Tracking parameters')]},
    ].map(group => ({...group,tasks:group.tasks.filter(task => task.gains && typeof task.gains === 'object')}));
}
function profileMotionControlHtml(editor) {
    const en = state.language === 'en', admittance = editor.core.capability.admittance;
    return `<div class="profile-motion-control"><section class="calibration-step"><h3>${en ? 'Control method and default behavior' : '控制方式与默认行为'}</h3><p class="hint">${en ? 'These are saved defaults for the next connection. The workspace reports session selection and effective cycle behavior separately' : '保存为下次连接使用的默认值；当前会话在控制工作台调整'}</p><div class="parameter-fields">${profileField(['core', 'capability', 'admittance', 'enabled'], admittance.enabled)}${profileField(['core', 'capability', 'admittance', 'reference_behavior'], admittance.reference_behavior)}</div><div class="motion-control-chain"><span>${en ? 'Nominal task target' : '任务名义目标'}</span><span>→</span><span>${en ? 'Native reference / virtual M/D/K response' : '原生参考 / 虚拟 M/D/K 响应'}</span><span>→</span><span>${en ? 'Actuator Kp/Kd tracking' : '执行器 Kp/Kd 跟踪'}</span></div></section>${profileGainPurposeHtml(editor)}<section class="calibration-step"><h3>${en ? 'Model interaction response' : '模型交互响应'}</h3><p class="hint">${en ? 'Only MODEL_ADMITTANCE applies the M/D/K response; native impedance may still use external-torque observation' : '模型导纳使用 M/D/K 生成交互响应'}</p>${interactionGoalEditorHtml('profile')}<details class="calibration-locks"><summary>${en ? 'Expert M/D/K and reference limits' : '专家 M/D/K 与参考限幅'}</summary>${profileTree(admittance.controller, ['core', 'capability', 'admittance', 'controller'])}</details></section><details class="calibration-locks"><summary>${en ? 'External-torque observation, participating joints and calibration' : '外力观测、参与关节与标定参数'}</summary>${profileTree(admittance.joint_enabled, ['core', 'capability', 'admittance', 'joint_enabled'])}${profileTree(admittance.observer, ['core', 'capability', 'admittance', 'observer'])}${profileTree(admittance.calibration, ['core', 'capability', 'admittance', 'calibration'])}</details><details class="calibration-locks"><summary>${en ? 'Rigid position hold compensation' : '刚性定位保持补偿'}</summary><p class="hint">${en ? 'Limited integral torque applies only to native Rigid gain use while holding; compliant, drag, servo and model-admittance control do not apply it. Disabled by default and qualified separately' : '受限积分力矩只用于原生阻抗的刚性定位保持，柔性、拖拽、伺服与模型导纳均不应用；默认关闭，独立验收'}</p>${editor.core.control.hold_compensation ? profileTree(editor.core.control.hold_compensation, ['core', 'control', 'hold_compensation']) : `<p class="hint">${en ? 'Not configured · disabled' : '未配置 · 默认关闭'}</p><button type="button" id="profile-add-hold-compensation" class="button">${en ? 'Add disabled compensation settings' : '添加关闭状态的补偿参数'}</button>`}</details><details class="calibration-locks"><summary>${en ? 'Advanced runtime settings' : '高级运行参数'}</summary>${profileTree(Object.fromEntries(Object.entries(editor.core.control.runtime).filter(([key])=>key !== 'tracking_impedance_mode')), ['core', 'control', 'runtime'])}${profileField(['core', 'control', 'controller', 'allow_full_cmd'], editor.core.control.controller.allow_full_cmd)}</details></div>`;
}
function gravityValidationNoticeHtml(result, preview = false) {
    if (!result?.gravity_validation_reset) return '';
    const names = {model:['模型与控制链','Model and control chain'], mapping:['关节映射','Joint mapping'], hardware_mapping:['执行器映射','Actuator mapping'], gravity:['重力向量','Gravity vector'], gravity_scale:['重力补偿比例','Gravity scales'], gravity_correction:['重力校正','Gravity correction']};
    const changes = (result.gravity_validation_changes || []).map(key => names[key]?.[state.language === 'en' ? 1 : 0] || key).join(' / ');
    return `<div class="callout gravity-validation-notice" role="status"><b>${preview ? busyText('保存后需重新确认重力模型','Gravity model confirmation will be reset') : busyText('重力模型确认已撤销','Gravity model confirmation reset')}</b><p>${esc(changes)}${changes ? ' · ' : ''}${preview ? busyText('将关闭故障柔性恢复的重力模型确认；保存后核对当前机械臂和负载，再到安全策略重新确认','Saving will turn off gravity confirmation for compliant fault recovery; verify the robot and payload, then confirm it again in Safety policy') : busyText('已关闭故障柔性恢复的重力模型确认；核对当前机械臂和负载后，在安全策略中重新确认','Gravity confirmation for compliant fault recovery is off; verify the current robot and payload, then confirm it again in Safety policy')}</p>${setupActionHtml({page:'profile_setup',group:'safety',field:'core.safety_policy.fault_recovery.gravity_model_validated'},busyText('查看重力模型确认','Review gravity model confirmation'))}</div>`;
}
function runtimeSavePreviewHtml() {
    const en = state.language === 'en', preview = state.savePreview;
    if (!preview) return `<p class="hint">${en ? 'Preview the exact strategy, participating joints, response, execution gains and hold compensation that would be saved' : '先预览将写入的策略、参与关节、响应参数、执行增益与刚性保持补偿的实际差异'}</p>`;
    return `${gravityValidationNoticeHtml(preview,true)}<div class="task-directory"><code>${esc(preview.path || '')}</code></div><div class="diff-box">${(preview.changes || []).map(change => `<div>${change.before ? `<del>${esc(change.before)}</del>` : ''}${change.after ? `<ins>${esc(change.after)}</ins>` : ''}</div>`).join('') || `<p>${en ? 'No saved defaults would change' : '持久化默认没有变化'}</p>`}</div>`;
}
function runtimeExecutionSettingsHtml() {
    const en = state.language === 'en', runtime = workbenchConnected() ? state.admittance : null;
    if (!runtime) return `<p class="hint">${en ? 'Read the connected session first' : '连接工作台后读取当前会话参数'}</p>`;
    const names = state.telemetry?.joint_names || [], gains = runtime.execution_gains || {}, hold = runtime.hold_compensation;
    const vector = values => Array.isArray(values) ? values.map(value => capabilityNumber(value,4)).join(' / ') : '—';
    const groups = gainPurposeSettings(gains);
    const rows = groups.map(group => `<tbody data-session-gain-purpose="${group.purpose}">${group.tasks.length ? group.tasks.map((task,index) => `<tr>${index === 0 ? `<th rowspan="${group.tasks.length}">${esc(gainPurposeLabel(group.purpose))}</th>` : ''}<td>${esc(task.label)}</td><td>${esc(vector(task.gains.kp))}</td><td>${esc(vector(task.gains.kd))}</td></tr>`).join('') : `<tr><th>${esc(gainPurposeLabel(group.purpose))}</th><td colspan="3">${en ? 'Not provided by this session' : '当前会话未提供此用途增益'}</td></tr>`}</tbody>`).join('');
    return `<p class="hint">${en ? 'Joint order' : '关节顺序'} · ${esc(names.join(' / '))}</p><div class="table-scroll"><table class="workspace-table"><thead><tr><th>${en ? 'Gain use' : '增益用途'}</th><th>${en ? 'Task parameters' : '任务参数'}</th><th>Kp</th><th>Kd</th></tr></thead>${rows}</table></div>${hold ? `<h4>${en ? 'Session rigid hold compensation' : '会话刚性定位保持补偿'} · ${hold.enabled === true ? 'ON' : 'OFF'}</h4><p class="hint">${en ? 'Velocity / error gates (joint units)' : '速度 / 误差门槛（关节单位）'} · ${capabilityNumber(hold.velocity_threshold)} / ${capabilityNumber(hold.error_threshold)}</p><div class="table-scroll"><table class="workspace-table"><thead><tr><th>${t('joint')}</th><th>${en ? 'Integral gain' : '积分增益'}</th><th>${en ? 'Torque ceiling' : '力矩上限'}</th><th>${en ? 'Rate ceiling' : '变化率上限'}</th><th>${en ? 'Contact gate' : '接触阈值'}</th></tr></thead><tbody>${names.map((name,index) => `<tr><th>${esc(name)}</th>${['integral_gain','max_torque','max_rate','contact_threshold'].map(key => `<td>${capabilityNumber(hold[key]?.[index],4)}</td>`).join('')}</tr>`).join('')}</tbody></table></div>` : ''}<p class="hint">${en ? 'Read from get_admittance for this session; these values are separate from the M/D/K draft and saved Profile defaults' : '以上来自本会话 get_admittance 快照，与 M/D/K 草稿及磁盘 Profile 默认值分别记录'}</p>`;
}
function runtimeDefaultsHtml() {
    const en = state.language === 'en';
    return `<section class="workspace-section"><div class="workspace-section-head"><div><h3>${en ? 'Session settings and saved defaults' : '当前会话与持久化默认'}</h3><p class="hint">${state.profileDirty ? (en ? 'Save or discard the parameter form edits before importing session defaults' : '请先保存或重新加载上方参数修改，再导入当前会话默认值') : (en ? 'Import actual connected-session settings into saved defaults; only the previewed fields are written' : '将当前连接会话的实际参数导入持久化默认，仅写入预览列出的字段')}</p></div></div><div class="workspace-actions"><button id="preview-config" class="button" ${workbenchConnected() && !state.profileDirty ? '' : 'disabled'}>${en ? 'Preview saved defaults' : '预览保存为默认'}</button><button id="save-config" class="button primary" ${workbenchConnected() && !state.profileDirty && state.savePreview?.changed ? '' : 'disabled'}>${en ? 'Confirm writing defaults' : '确认写入默认'}</button></div><div id="runtime-config-preview">${runtimeSavePreviewHtml()}</div><details class="calibration-locks"><summary>${en ? 'Session execution gains and rigid hold compensation' : '本会话执行增益与刚性保持补偿'}</summary><div id="runtime-execution-settings">${runtimeExecutionSettingsHtml()}</div></details></section>`;
}
function refreshRuntimeSavePreview() {
    const preview = $('runtime-config-preview');
    if (preview) {
        preview.innerHTML = runtimeSavePreviewHtml();
        preview.querySelectorAll('[data-setup-target]').forEach(button=>button.onclick=()=>operation(()=>openSetupTarget(JSON.parse(button.dataset.setupTarget))));
    }
    if ($('runtime-execution-settings')) $('runtime-execution-settings').innerHTML = runtimeExecutionSettingsHtml();
    if ($('preview-config')) $('preview-config').disabled = !workbenchConnected() || !!state.profileDirty;
    if ($('save-config')) $('save-config').disabled = !workbenchConnected() || !!state.profileDirty || state.savePreview?.changed !== true;
}
function mergeMissingProfileFields(current, defaults) {
    if (!current || typeof current !== 'object' || Array.isArray(current)) current={};
    for (const [key,value] of Object.entries(defaults)) {
        if (current[key] === undefined || current[key] === null) current[key]=structuredClone(value);
        else if (value && typeof value === 'object' && !Array.isArray(value)) current[key]=mergeMissingProfileFields(current[key],value);
    }
    return current;
}
function profileJointTemplateMissing(editor,kind) {
    const hardware=kind === 'hardware', root=hardware ? editor.hardware?.damiao?.actuators : editor.core?.calibration?.joints;
    const keys=hardware ? ['name','motor_id','master_id','motor_type'] : ['direction','pos_ratio','tor_ratio','joint_zero_offset','actuator_zero_offset'];
    return (editor.joint_names || []).some(name=>keys.some(key=>root?.[name]?.[key] === undefined || root[name][key] === null));
}
function completeProfileFields(kind) {
    const editor=state.profileEditor,names=editor.joint_names || [];
    let target=state.profileMissingTarget;
    if (target && !(kind === 'hardware' ? target.startsWith('hardware.') : target.startsWith('core.calibration.'))) target=null;
    if (kind === 'hardware') {
        const defaults=profileHardwareDefaults(names);
        for (const item of Object.values(defaults.damiao.actuators)) item.motor_id=0;
        defaults.buses.main_can.device='';
        editor.hardware=mergeMissingProfileFields(editor.hardware,defaults);
        if (!editor.profile_entry.hardware?.plugin) editor.profile_entry.hardware={...editor.profile_entry.hardware,plugin:'serial_arm_hardware_damiao'};
        target ||= `hardware.damiao.actuators.${names[0]}.motor_id`;
    } else {
        const joints=Object.fromEntries(names.map(name=>[name,{direction:1,pos_ratio:1,tor_ratio:1,joint_zero_offset:0,actuator_zero_offset:0}]));
        editor.core.calibration=mergeMissingProfileFields(editor.core.calibration,{joints});
        target ||= `core.calibration.joints.${names[0]}.direction`;
    }
    state.profileDirty=true;state.setupFocus={page:'profile_setup',field:target};render();
}
function profileJointTableHtml(editor, kind) {
    const names = editor.joint_names || [], hardware = kind === 'hardware';
    const root = hardware ? editor.hardware?.damiao?.actuators : editor.core?.calibration?.joints;
    if (!root) return '';
    const columns = hardware ? ['name','motor_id','master_id','motor_type'] : ['direction','pos_ratio','tor_ratio','joint_zero_offset','actuator_zero_offset'];
    const extra = names.map(name=>{const remaining=Object.fromEntries(Object.entries(root[name] || {}).filter(([key])=>!columns.includes(key)));return Object.keys(remaining).length ? `<details class="parameter-subgroup"><summary>${esc(name)} · ${busyText('高级参数','Advanced settings')}</summary>${profileTree(remaining,hardware ? ['hardware','damiao','actuators',name] : ['core','calibration','joints',name])}</details>` : '';}).join('');
    return `<div class="table-scroll"><table class="profile-joint-table"><thead><tr><th>${t('joint')}</th>${columns.map(key=>`<th>${esc(paramLabel(key))}</th>`).join('')}</tr></thead><tbody>${names.map(name=>`<tr><th>${esc(name)}</th>${columns.map(key=>`<td>${profileField(hardware ? ['hardware','damiao','actuators',name,key] : ['core','calibration','joints',name,key],root[name]?.[key])}</td>`).join('')}</tr>`).join('')}</tbody></table></div>${extra ? `<div class="profile-joint-advanced">${extra}</div>` : ''}`;
}
function profileHardwareHtml(editor) {
    if (!editor.hardware?.damiao?.actuators) return profileTree(editor.hardware,['hardware']);
    const other = structuredClone(editor.hardware); delete other.damiao.actuators;
    return profileTree(other,['hardware']) + `<h4>${busyText('关节与执行器对应','Joint actuator assignments')}</h4>` + profileJointTableHtml(editor,'hardware');
}
function recordingContextText(phase) {
    const x = state.telemetry || {};
    if (!workbenchConnected()) return busyText('连接工作台后拖拽定位','Connect the workspace to position by dragging');
    const label = phase === 'recording' ? busyText('录制期间','Recording') : phase === 'ready' ? busyText('记录结束 · 当前','Recording finished · current') : busyText('当前','Current');
    return `${label} · ${profileOptionLabel(x.reference_behavior || referenceBehavior())} · ${gainPurposeLabel(x.gain_profile)} · ${profileOptionLabel(x.model_feedforward_mode)}`;
}
function profileHardwareToolsHtml(editor) {
    const en=state.language==='en', inspection=state.driverInspection;
    const result=inspection && inspection.plugin===editor.profile_entry?.hardware?.plugin ? `<div class="calibration-muted-note" role="status">${inspection.installed ? busyText('已找到驱动文件','Driver file found') : busyText('未找到驱动文件，请安装所选驱动','Driver file missing; install the selected driver')}<code class="teach-file-path">${esc(inspection.path || inspection.message || '')}</code></div>` : '';
    return `<section class="hardware-tools"><p class="hint">${en?'Configure the selected driver or import its Hardware YAML, then check the parameters against that driver’s requirements':'填写所选驱动的硬件参数，或导入对应的硬件配置 YAML，再按该驱动的说明核对参数'}</p><div class="workspace-actions"><button type="button" id="profile-driver-check" class="button">${en?'Check driver file':'检查驱动文件'}</button><button type="button" id="profile-import-hardware" class="button">${en?'Import Hardware YAML':'导入硬件配置 YAML'}</button></div>${result}${editor.hardware_imported ? `<p class="hint">${en?'Imported into the draft; save and reconnect to apply':'已导入草稿，保存并重新连接后生效'} · ${esc(editor.hardware_import_source || '')}</p>` : ''}</section>`;
}
function bindHardwareTools() {
    if($('profile-driver-check'))$('profile-driver-check').onclick=()=>operation(async()=>{
        for(const field of document.querySelectorAll('[data-profile-field]'))if(!field.disabled)readProfileInput(field);
        const editor=state.profileEditor,epoch=state.workbenchEpoch,config=structuredClone(state.config),key=JSON.stringify(config);
        const inspection=await api.request('profile_hardware_inspect',{config,plugin:editor.profile_entry?.hardware?.plugin});
        if(editor!==state.profileEditor || epoch!==state.workbenchEpoch || JSON.stringify(state.config)!==key)return;
        state.driverInspection=inspection;render();
    });
    if($('profile-import-hardware'))$('profile-import-hardware').onclick=()=>operation(async()=>{
        const editor=state.profileEditor,epoch=state.workbenchEpoch,config=structuredClone(state.config),key=JSON.stringify(config);
        const source=await api.selectProfile();if(!source)return;
        if(editor!==state.profileEditor || epoch!==state.workbenchEpoch || JSON.stringify(state.config)!==key)return;
        // Preserve edits in the other groups and their conflict hashes
        for(const field of document.querySelectorAll('[data-profile-field]'))if(!field.disabled){try{readProfileInput(field);}catch{}}
        const imported=await api.request('profile_hardware_import',{config,path:source,plugin:editor.profile_entry?.hardware?.plugin});
        if(editor!==state.profileEditor || epoch!==state.workbenchEpoch || JSON.stringify(state.config)!==key)return;
        editor.hardware=imported.hardware;editor.hardware_imported=true;editor.hardware_import_source=imported.hardware_import_source;
        editor.profile_entry.hardware=structuredClone(imported.profile_entry.hardware);
        for(const map of [state.profileFieldErrors,state.profileRawInputs])for(const key of Object.keys(map || {}))if(key==='hardware'||key.startsWith('hardware.')||key.startsWith('profile_entry.hardware.'))delete map[key];
        state.driverInspection=null;state.profileDirty=true;state.profileError='';render();
    });
}
function profileSetupPage() {
    const e = state.profileEditor;
    if (!e) return pageHead('profileSetup', 'profileSetupSubtitle') + `<div class="empty-state">${state.language === 'en' ? 'Loading configuration' : '正在加载机械臂参数'}</div>`;
    const key = ['runtime', 'gains', 'admittance'].includes(state.profileGroup) ? 'motion' : state.profileGroup || 'model';
    const current = profileGroups.find(g => g[0] === key) || profileGroups[0];
    const value = key === 'hardware' ? e.hardware : profileGet(e.core, current[2].split('.'));
    const info = key === 'motion' ? (state.language === 'en' ? 'Choose reference generation first, then configure actuator gains for the task; virtual M/D/K and actuator Kp/Kd operate at different layers' : '先选择参考生成方式，再按任务配置执行器增益；M/D/K 与执行器 Kp/Kd 分属不同层级') :
        key === 'model' ? (state.language === 'en' ? 'URDF model and joint ordering are shared by all groups' : '模型和受控关节顺序由全部参数分组共享') :
            key === 'hardware' ? (state.language === 'en' ? 'Select an installed driver that supports the actuators, then configure its bus and actuator assignments' : '选择支持当前执行器的已安装驱动，再填写对应的总线与执行器映射') : '';
    const profilePlugin = profileField(['profile_entry', 'hardware', 'plugin'], e.profile_entry?.hardware?.plugin || '');
    const optional = [
        key === 'motion' && !e.core.control.controller.servo_tracking && !e.core.control.controller.identification_tracking ? `<button class="button" id="profile-add-identification">${state.language === 'en' ? 'Add servo tracking gains' : '添加伺服跟踪增益'}</button>` : '',
        key === 'model' && !('gravity_correction_path' in e.core.model) ? `<button class="button" id="profile-add-correction">${state.language === 'en' ? 'Add optional correction path' : '添加可选重力校正文件路径'}</button>` : '',
        key === 'safety' && !('max_effort_override' in e.core.safety_policy) ? `<button class="button" id="profile-add-effort">${state.language === 'en' ? 'Add optional torque limits' : '添加可选最大力矩覆盖'}</button>` : '',
        key === 'motion' && !('joint_acc_filter_alpha' in e.core.control.runtime) ? `<button class="button" id="profile-add-filter">${state.language === 'en' ? 'Add acceleration filter' : '添加加速度滤波系数'}</button>` : '',
        key === 'shutdown' && !('compliant_phase_ratio' in e.core.shutdown) ? `<button class="button" id="profile-add-park-ratio">${state.language === 'en' ? 'Add compliant park ratio' : '添加柔性停放时间比例'}</button>` : '',
        key === 'safety' && !('require_continuous_cmd' in e.core.safety_policy) ? `<button class="button" id="profile-add-continuity">${state.language === 'en' ? 'Add command continuity check' : '添加命令连续性配置'}</button>` : '',
    ].join('');
    const noHardware = key === 'hardware' && !Object.keys(e.hardware || {}).length;
    const completeHardware = (e.profile_entry?.hardware?.plugin === 'serial_arm_hardware_damiao' && (!e.hardware?.damiao || !Object.keys(e.hardware?.buses || {}).length || profileJointTemplateMissing(e,'hardware'))) ? `<button type="button" class="button" data-profile-complete="hardware">${busyText('补齐缺少的硬件字段','Add missing hardware fields')}</button>` : '';
    const completeMapping = key === 'mapping' && profileJointTemplateMissing(e,'mapping') ? `<button type="button" class="button" data-profile-complete="mapping">${busyText('补齐缺少的映射字段','Add missing mapping fields')}</button>` : '';
    const hardwarePluginId = e.profile_entry?.hardware?.plugin || '';
    const hardwareInitialization = !hardwarePluginId || hardwarePluginId === 'serial_arm_hardware_damiao' ? `<button id="profile-init-damiao" class="button">${hardwarePluginId ? busyText('初始化达妙驱动配置','Initialize Damiao driver configuration') : busyText('使用内置达妙驱动并初始化','Use the built-in Damiao driver and initialize')}</button>` : '';
    const hardwarePanel = noHardware ? `<div class="parameter-empty">${busyText('尚无硬件参数，请导入所选驱动对应的配置 YAML','Hardware parameters are empty; import the configuration YAML for the selected driver')}${hardwareInitialization}</div>` : profileHardwareHtml(e);
    const body = (key === 'hardware' ? `${profilePlugin}${profileHardwareToolsHtml(e)}${completeHardware}${hardwarePanel}` : key === 'motion' ? profileMotionControlHtml(e) + runtimeDefaultsHtml() : key === 'mapping' && e.core.calibration?.joints ? profileJointTableHtml(e,'mapping') : profileTree(value, ['core', ...current[2].split('.')])) + (key === 'safety' || key === 'model' ? profileUrdfLimitsHtml(e) : '');
    const changed = state.profileDirty;
    const savedNote = state.profileSavedForSession && workbenchConnected() ? busyText('已保存，重新连接后生效','Saved · reconnect to apply') : state.profileSaveNotice ? t('saved') : '';
    const backAction = `<button type="button" id="profile-return-ready" class="button">${busyText('重新检查并返回','Recheck and return')}</button>`;
    const err = state.profileError ? `<div class="parameter-error" role="alert">${esc(state.profileError)}</div>` : '';
    return `<div id="profile-guide-container">${profileGuidanceHtml(e)}</div>` + (state.createdProfileWizard ? descriptionStepper(4) + `<div class="wizard-profile-note">${t('newHardwareHint')}</div>` : '') + pageHead('profileSetup', 'profileSetupSubtitle', `<div class="parameter-top-actions"><span id="profile-dirty-state" class="hint">${changed ? busyText('有未保存修改','Unsaved changes') : savedNote || busyText('已加载默认配置','Saved defaults loaded')}</span>${backAction}<button id="profile-reload" class="button">${state.language === 'en' ? 'Reload' : '重新加载'}</button><button id="profile-save" class="button primary">${t('saveProfile')}</button></div>`) + gravityValidationNoticeHtml(e) + `<section class="parameter-layout"><nav class="parameter-groups">${profileGroups.map(([id, title]) => `<button class="parameter-group ${id === key ? 'active' : ''}" data-profile-group="${id}">${esc(state.language === 'en' ? ({model:'Model and gravity',mapping:'Joint mapping',motion:'Motion and interaction',safety:'Safety policy',shutdown:'Parking',hardware:'Hardware and buses'}[id]) : title)}${id === 'motion' ? '<small>Kp/Kd · M/D/K</small>' : ''}</button>`).join('')}</nav><section class="card parameter-card"><div class="card-head"><div><h3>${esc(state.language === 'en' ? ({model:'Model and gravity',mapping:'Joint mapping',motion:'Motion and interaction',safety:'Safety policy',shutdown:'Parking',hardware:'Hardware and buses'}[key]) : current[1])}</h3><details class="parameter-source"><summary>${busyText('配置文件','Configuration file')}</summary><code>${esc(key === 'hardware' ? e.hardware_path : e.core_path)}</code></details></div></div><div class="card-body">${info ? `<p class="parameter-guidance">${info}</p>` : ''}${err}${completeMapping}${body}${optional}</div></section></section>`;
}
function connectionSettingsHtml() {
    const info = state.info;
    const selectOptions = `<option value="">${t('noProfile')}</option>` + state.profiles.map(p => `<option value="${esc(p.name)}" ${p.name === state.config.profile ? 'selected' : ''}>${esc(p.name)}</option>`).join('');
    const profileBody = `<div class="field"><label>${t('source')}</label><div class="segmented"><button id="source-builtin" class="${state.source === 'builtin' ? 'active' : ''}">${t('builtin')}</button><button id="source-external" class="${state.source === 'external' ? 'active' : ''}">${t('external')}</button></div></div>${state.source === 'external' ? `<div class="field"><label for="profile_file">${t('profileFile')}</label><div class="input-row"><input id="profile_file" value="${esc(state.config.profile_file)}" placeholder="${t('fileHint')}">${button('browse-profile', 'browse', 'folder')}</div></div>` : ''}<div class="field"><label for="profile-select">${t('profileName')}</label><div class="input-row"><select id="profile-select">${selectOptions}</select>${button('refresh-profiles', 'refresh', 'refresh')}</div></div>`;
    const ports = (state.status.devices || []).map(p => `<option value="${esc(p)}">`).join('');
    const connBody = `${configField('serial_port', 'serial', info?.default_port || t('automatic'))}<datalist id="ports">${ports}</datalist><div class="row">${configField('baudrate', 'baudrate', String(info?.default_baudrate || '921600'))}${configField('bus', 'bus', info?.bus || t('automatic'))}</div><div class="field"><label for="resource_paths">${t('resourcePaths')}</label><div class="input-row"><input id="resource_paths" value="${esc(state.config.resource_paths)}" placeholder="/path/to/workspace">${button('add-root', 'addRoot', 'folder')}</div><div class="hint">${t('rootsHint')}</div></div><div class="actions">${button('inspect-profile', 'inspect', 'diagnostics')}${button('go-run', 'toRun', 'run', 'primary')}</div><div class="hint">${t('overrideHint')}</div>`;
    const summary = `<div class="summary-caption">ROBOT PROFILE</div><div class="summary-name">${esc(info?.profile || state.config.profile || '—')}</div><dl class="kv"><dt>${t('hardwarePlugin')}</dt><dd>${esc(info?.hardware_plugin || '—')}</dd><dt>${t('serial')}</dt><dd>${esc(state.config.serial_port || info?.default_port || '—')}</dd><dt>${t('writeEnabled')}</dt><dd style="color:var(--accent)">${t(info?.write_enabled === true ? 'enabled' : info?.write_enabled === false ? 'disabled' : 'unknown')}</dd><dt>${t('controllers')}</dt><dd>${info?.controllers?.map(name => `<div>${esc(name)}</div>`).join('') || '—'}</dd></dl><div class="callout">${t('validationHint')}</div>`;
    const checks = `<ul class="check-list">${[['installation', !!state.status.installed], ['resources', !!info && info.checks.filter(c => c.name !== 'device').every(c => c.ok)], ['nativeTerminal', !!info?.available?.terminal], ['ros2', !!state.status.ros2]].map(([key, ok]) => `<li><span class="dot ${ok ? 'ready' : ''}"></span>${t(key)}<span class="state">${t(ok ? 'ready' : 'unavailable')}</span></li>`).join('')}</ul>`;
    return `<div class="grid"><div class="card-stack">${card('profile', 'profileSub', 'robot', profileBody)}<section id="connection-settings">${card('connection', 'connectionSub', 'plug', connBody)}</section></div><div class="card-stack">${card('summary', 'summarySub', 'model', summary)}${card('status', 'statusSub', 'diagnostics', checks)}</div></div>`;
}
function robotPage() { return pageHead('robotTitle', 'robotDesc') + connectionSettingsHtml(); }
function vectorText(values, digits = 5) { return Array.isArray(values) ? values.map(v => Number(v).toFixed(digits)).join(', ') : '—'; }
function inertiaFields(matrix) { if (!Array.isArray(matrix) || matrix.length !== 3) return []; return [['ixx', matrix[0][0]], ['ixy', matrix[0][1]], ['ixz', matrix[0][2]], ['iyy', matrix[1][1]], ['iyz', matrix[1][2]], ['izz', matrix[2][2]]]; }
function inspectorSection(title, rows) { return `<section class="inspector-section"><h4>${title}</h4><dl class="kv">${rows.map(([k, v, cls = '']) => `<dt>${k}</dt><dd class="${cls}">${v}</dd>`).join('')}</dl></section>`; }
function modelInspectorHtml() {
    const data = state.modelData; if (!data) return `<div class="empty-state">${t('noModel')}</div>`;
    const link = data.model.links.find(x => x.name === state.modelSelection); const joint = data.model.joints.find(x => x.name === state.modelSelection);
    if (link) {
        const inertial = link.inertial; const childJoint = data.model.joints.find(j => j.child === link.name && j.controlled); const effective = data.native.info.effective_inertias.find(x => x.joint_name === childJoint?.name);
        const urdfRows = inertial ? [['&lt;origin xyz&gt;', vectorText(inertial.origin?.xyz)], ['&lt;origin rpy&gt;', vectorText(inertial.origin?.rpy)], ['&lt;mass value&gt;', `${Number(inertial.mass).toFixed(6)} kg`], ...inertiaFields(inertial.inertia).map(([k, v]) => [`&lt;inertia ${k}&gt;`, Number(v).toPrecision(6)]), ['校验', inertial.valid ? t('pass') : esc(inertial.reason || '—')]] : [['状态', 'URDF 未定义 &lt;inertial&gt;']];
        const effectiveRows = effective ? [['关联关节', esc(effective.joint_name)], ['参考坐标系', esc(link.name) + ' / ' + esc(effective.joint_name)], ['mass', `${Number(effective.mass).toFixed(6)} kg`], ['center_of_mass', vectorText(effective.center_of_mass)], ['inertia matrix', effective.inertia.map(r => r.map(v => Number(v).toPrecision(6)).join('  ')).join('\n'), 'mono-block']] : [['状态', '当前 Link 无对应 Core 约简惯性']];
        return `<div class="summary-caption">LINK</div><div class="summary-name">${esc(link.name)}</div><button class="button" data-urdf-source>${busyText('定位 URDF 源文件','Show URDF source file')}</button>${inspectorSection(t('urdfInertial'), urdfRows)}<p class="inertia-description">${state.language === 'en' ? 'Inertia belongs to the link. Core values are reduced-body equivalents associated with a controlled joint, not joint mass' : '质量与惯量属于连杆；Core 显示受控关节对应约简刚体的等效惯性'}</p>${inspectorSection(t('coreReducedInertia'), effectiveRows)}`;
    }
    if (joint) {
        const lim = joint.limit || {}; const origin = joint.origin || {}; const urdfRows = [['&lt;joint type&gt;', esc(joint.type)], ['&lt;parent link&gt;', esc(joint.parent)], ['&lt;child link&gt;', esc(joint.child)], ['&lt;origin xyz&gt;', vectorText(origin.xyz)], ['&lt;origin rpy&gt;', vectorText(origin.rpy)], ['&lt;axis xyz&gt;', vectorText(joint.axis, 4)]];
        for (const key of ['lower', 'upper', 'effort', 'velocity']) if (Number.isFinite(lim[key])) urdfRows.push([`&lt;limit ${key}&gt;`, `${Number(lim[key]).toPrecision(7)}${['lower','upper'].includes(key) ? (joint.type === 'prismatic' ? ' m' : ' rad') : key === 'velocity' ? (joint.type === 'prismatic' ? ' m/s' : ' rad/s') : joint.type === 'prismatic' ? ' N' : ' Nm'}`]);
        if (joint.mimic) { urdfRows.push(['&lt;mimic joint&gt;', esc(joint.mimic.joint)]); urdfRows.push(['&lt;mimic multiplier&gt;', Number(joint.mimic.multiplier).toPrecision(7)]); urdfRows.push(['&lt;mimic offset&gt;', Number(joint.mimic.offset).toPrecision(7)]); }
        const coreRows = [['controlled', joint.controlled ? t('enabled') : t('disabled')], ['preview_status', esc(joint.preview_status || '—')]];
        return `<div class="summary-caption">JOINT</div><div class="summary-name">${esc(joint.name)}</div><button class="button" data-urdf-source>${busyText('定位 URDF 源文件','Show URDF source file')}</button>${inspectorSection('URDF &lt;joint&gt;', urdfRows)}${inspectorSection('Core mapping', coreRows)}`;
    }
    const native = data.native; return `<div class="summary-caption">CORE MODEL</div><div class="summary-name">${esc(data.model.robot_name || data.profile)}</div><dl class="kv"><dt>URDF</dt><dd class="path-chip">${esc(data.urdf)}</dd><dt>base_frame</dt><dd>${esc(native.config.base_frame)}</dd><dt>tool_frame</dt><dd>${esc(native.config.tool_frame)}</dd><dt>joints_count</dt><dd>${native.info.joints_count}</dd><dt>URDF total_mass</dt><dd>${Number(native.info.total_mass).toFixed(4)} kg</dd><dt>Core reduced_total_mass</dt><dd>${Number(native.info.reduced_total_mass).toFixed(4)} kg</dd><dt>${t('effectiveInertia')}</dt><dd>${native.info.effective_inertias.length}</dd></dl>${data.description_mismatch ? `<div class="callout">Core URDF 与 description 资源不同，当前以 Core URDF 为准</div>` : ''}`;
}
function modelObjectVisible(kind, name) { return state.modelVisibility[kind]?.[name] !== false; }
function modelTreeHtml() {
    const data = state.modelData; if (!data) return `<div class="empty-state">${t('noModel')}</div>`;
    const eye = (kind, name, label) => `<button class="tree-eye ${modelObjectVisible(kind, name) ? 'on' : ''}" data-model-visible="${kind}" data-model-name="${esc(name)}" title="${esc(label)}">${modelObjectVisible(kind, name) ? '●' : '○'}</button>`;
    const linkRow = x => `<div class="tree-row tree-row-link ${state.modelSelection === x.name ? 'active' : ''}"><button class="tree-item" data-model-select="${esc(x.name)}">${icon('model')}<span>${esc(x.name)}</span></button>${eye('links', x.name, state.language === 'en' ? 'Show or hide link geometry' : '显示或隐藏 Link 几何')}<button class="tree-frame ${modelObjectVisible('linkFrames', x.name) ? 'on' : ''}" data-model-frame="${esc(x.name)}" aria-pressed="${modelObjectVisible('linkFrames', x.name) ? 'true' : 'false'}" title="${state.language === 'en' ? 'Show or hide this link frame' : '单独显示或隐藏此 Link Frame'}"><span>X</span><span>Y</span><span>Z</span></button></div>`;
    const jointRow = x => `<div class="tree-row ${state.modelSelection === x.name ? 'active' : ''}"><button class="tree-item" data-model-select="${esc(x.name)}"><span class="axis-dot ${x.controlled ? 'controlled' : ''}"></span><span>${esc(x.name)}</span><small>${esc(x.type)}</small></button>${eye('joints', x.name, state.language === 'en' ? 'Show or hide joint axis' : '显示或隐藏 Joint Axis')}</div>`;
    const actions = `<div class="tree-actions"><button id="model-show-all" class="tree-action">${t('showAllObjects')}</button><button id="model-hide-all" class="tree-action">${t('hideAllObjects')}</button></div>`;
    return actions + `<div class="tree-section"><b>Links</b><div class="tree-control-legend"><span></span><span>${state.language === 'en' ? 'Link' : '几何'}</span><span>Frame</span></div>${data.model.links.map(linkRow).join('')}</div><div class="tree-section"><b>Joints</b>${data.model.joints.map(jointRow).join('')}</div>`;
}
function jointDisplaySpec(name, nativeUnit = null) {
    const joint = state.modelData?.model?.joints?.find(item => item.name === name);
    const kind = joint?.type || state.profileEditor?.urdf_joint_limits?.[name]?.type;
    const linear = nativeUnit ? nativeUnit === 'm' : kind === 'prismatic';
    return {linear, factor:linear ? 1000 : 180 / Math.PI, unit:linear ? 'mm' : '°',
        velocityUnit:linear ? 'mm/s' : '°/s', accelerationUnit:linear ? 'mm/s²' : '°/s²',
        effortUnit:linear ? 'N' : 'N·m', nativeUnit:linear ? 'm' : 'rad'};
}
function jointDisplayValue(value, name, quantity = 'position', digits = 3, nativeUnit = null) {
    if (!Number.isFinite(value)) return '—';
    const spec = jointDisplaySpec(name, nativeUnit), factor = quantity === 'effort' ? 1 : spec.factor;
    const unit = quantity === 'effort' ? spec.effortUnit : quantity === 'velocity' ? spec.velocityUnit : quantity === 'acceleration' ? spec.accelerationUnit : spec.unit;
    return `${(value * factor).toFixed(digits)} ${unit}`;
}
function jointInputToNative(name, raw) {
    return String(raw).trim() === '' ? NaN : Number(raw) / jointDisplaySpec(name).factor;
}
function jointMdkUnits(name) {
    return jointDisplaySpec(name).linear ? {mass:'kg',damping:'N·s/m',stiffness:'N/m'} :
        {mass:'N·m·s²/rad',damping:'N·m·s/rad',stiffness:'N·m/rad'};
}
function jointMdkText(values,name) {
    const units=jointMdkUnits(name);
    return ['mass','damping','stiffness'].map((key,index)=>`${['M','D','K'][index]} ${capabilityNumber(values[key])} ${units[key]}`).join(' · ');
}
function effectiveJointBounds(name, index, joint = {}) {
    const runtime = workbenchConnected() ? state.telemetry?.safety_limits : null;
    const safety = runtime?.min_pos ? runtime : state.readiness?.safety_summary;
    const lookup = key => Array.isArray(safety?.[key]) ? safety[key][index] : safety?.[key]?.[name];
    const lower = lookup('min_pos'), upper = lookup('max_pos'), margin = lookup('pos_margin');
    const limited = lookup('has_position_limit');
    if (limited === false || limited === 0 || joint.type === 'continuous') return {limited:false, source:'runtime'};
    if ([lower, upper, margin].every(Number.isFinite) && margin >= 0)
        return {limited:true, lower:lower + margin, upper:upper - margin, source:'safety'};
    const limits = joint.limit || {};
    const configuredMargin = state.readiness?.safety_summary?.position_margin;
    const fallbackMargin = typeof configuredMargin === 'number' ? configuredMargin : configuredMargin?.[name];
    if (Number.isFinite(limits.lower) && Number.isFinite(limits.upper) && Number.isFinite(fallbackMargin) && fallbackMargin >= 0)
        return {limited:true, lower:limits.lower + fallbackMargin, upper:limits.upper - fallbackMargin, source:'profile'};
    return {limited:true, lower:null, upper:null, source:'unavailable'};
}
function jointSliderRowHtml(name, index, joint, value, kind = 'model', actual = null, enabled = true) {
    const spec = jointDisplaySpec(name, joint.type === 'prismatic' ? 'm' : 'rad'), unit = spec.unit;
    const limits = kind === 'target' ? effectiveJointBounds(name,index,joint) : joint.limit || {};
    const continuous = joint.type === 'continuous';
    const bounded = Number.isFinite(limits.lower) && Number.isFinite(limits.upper) && limits.lower < limits.upper;
    const known = kind === 'model' || continuous || bounded;
    const center = Number.isFinite(value) ? value : Number.isFinite(actual) ? actual : 0;
    const lower = (bounded ? limits.lower : Math.min(0, center - Math.PI)) * spec.factor, upper = (bounded ? limits.upper : Math.max(0, center + Math.PI)) * spec.factor;
    const slider = kind === 'model' ? `data-model-joint="${index}"` : `data-target-range="${index}"`;
    const number = kind === 'model' ? `data-model-number="${index}"` : `data-target-joint="${index}"`;
    const displayId = kind === 'model' ? `joint-value-${index}` : `target-value-${index}`;
    const disabled = enabled && known ? '' : 'disabled';
    return `<div class="joint-preview-row ${kind === 'target' ? 'target-joint-row' : ''}"><label>${esc(name)} <span id="${displayId}">${Number.isFinite(value) ? (value * spec.factor).toFixed(3) : '—'} ${unit}</span>${kind === 'target' ? `<small>${state.language === 'en' ? 'Actual' : '实测'} <b id="target-actual-${index}">${Number.isFinite(actual) ? (actual * spec.factor).toFixed(3) : '—'}</b> ${unit}</small>` : ''}</label><input type="range" ${slider} min="${lower}" max="${upper}" step="0.01" value="${center * spec.factor}" ${disabled} aria-label="${esc(name)} ${unit}"><input class="joint-number" ${number} type="number" ${bounded ? `min="${lower}" max="${upper}"` : ''} step="0.01" value="${Number.isFinite(value) ? (value * spec.factor).toFixed(3) : ''}" ${disabled} aria-label="${esc(name)} ${unit}"></div>`;
}
function modelJointHtml() {
    const data = state.modelData; if (!data) return '';
    const controlled = new Map(data.model.joints.filter(joint => joint.controlled).map(joint => [joint.name, joint]));
    return data.native.config.joint_names.map((name, index) => jointSliderRowHtml(name, index, controlled.get(name) || {}, state.modelPositions[index] ?? 0)).join('');
}
function modelPage() {
    const available = state.info?.available?.model; const action = `${button('model-reload', state.modelData ? 'modelReload' : 'modelLoad', 'refresh', 'primary')}`;
    const empty = !state.modelData ? `<div class="model-empty"><div>${icon('model')}</div><h3>${available ? t('noModel') : t('modelUnavailable')}</h3><p>${t('offlinePreview')}</p></div>` : '';
    return pageHead('modelTitle', 'modelPageDesc', action) + `<div class="model-toolbar"><label><input type="checkbox" data-model-layer="visual" ${state.modelLayers.visual ? 'checked' : ''}>${t('visualLayer')}</label><label><input type="checkbox" data-model-layer="collision" ${state.modelLayers.collision ? 'checked' : ''}>${t('collisionLayer')}</label><label><input type="checkbox" data-model-layer="linkFrames" ${state.modelLayers.linkFrames ? 'checked' : ''}>${t('linkFrameLayer')}</label><label><input type="checkbox" data-model-layer="jointAxes" ${state.modelLayers.jointAxes ? 'checked' : ''}>${t('jointAxisLayer')}</label><label><input type="checkbox" data-model-layer="com" ${state.modelLayers.com ? 'checked' : ''}>${t('comLayer')}</label><label><input type="checkbox" data-model-layer="inertia" ${state.modelLayers.inertia ? 'checked' : ''}>${t('inertiaLayer')}</label><label><input type="checkbox" data-model-layer="labels" ${state.modelLayers.labels ? 'checked' : ''}>${t('labelsLayer')}</label><span class="inertia-source-options" role="group" aria-label="${state.language === 'en' ? 'Inertia data source' : '惯性数据来源'}"><span>${state.language === 'en' ? 'Inertia source' : '惯性来源'}</span><label><input type="radio" name="model-inertia-source" value="urdf" data-model-inertia-source ${state.modelInertiaSource === 'urdf' ? 'checked' : ''}>${state.language === 'en' ? 'Link / URDF' : 'Link 原始'}</label><label><input type="radio" name="model-inertia-source" value="core" data-model-inertia-source ${state.modelInertiaSource === 'core' ? 'checked' : ''}>${state.language === 'en' ? 'Core reduced' : 'Core 约简'}</label></span><div class="layer-actions"><button id="model-layers-all" class="tree-action">${t('allLayers')}</button><button id="model-layers-none" class="tree-action">${t('noLayers')}</button></div><span class="model-legend"><i class="legend-frame"></i>XYZ <i class="legend-axis"></i>Axis <span id="model-com-legend"><i class="${state.modelInertiaSource === 'core' ? 'legend-effective' : 'legend-com'}"></i>${state.modelInertiaSource === 'core' ? 'Core COM' : 'Link COM'}</span><i class="legend-total-com"></i>Total COM</span></div><div class="model-workspace"><aside class="model-tree">${modelTreeHtml()}</aside><section class="model-center"><div id="model-viewport">${empty}</div><div class="joint-preview"><div class="joint-preview-head"><b>${t('jointPreview')}</b><button id="model-zero" class="button">Zero</button></div>${modelJointHtml()}</div></section><aside class="model-inspector">${modelInspectorHtml()}</aside></div>`;
}
async function loadModelData(positions = null) {
    if (!state.info) await inspect(); if (!state.info?.available?.model) throw new Error(t('modelUnavailable')); state.modelLoading = true;
    try { const data = await api.request('model', { config: state.config, positions: positions || undefined }); state.modelData = data; state.modelPositions = data.native.state.positions.slice(); if (!state.modelSelection) state.modelSelection = data.model.links[0]?.name || ''; }
    finally { state.modelLoading = false; }
}
// Graphics readiness belongs to the preview, independently of native robot state.
const graphicsState = {effectiveMode: 'normal', requestedMode: 'normal', requiresRestart: false, records: new Map(), current: null, ghostKey: '', status: null, actionError: ''};
function graphicsModeLabel(mode) {
    return ({normal: busyText('正常 GPU 三维', 'Normal GPU 3D'), compatibility: busyText('兼容三维', 'Compatibility 3D'), rescue: busyText('无三维救援', 'Rescue without 3D')})[mode] || mode;
}
function previewSlot() {
    return state.page === 'description' ? 'description' : state.page === 'model' ? 'model' : 'workspace';
}
function previewRecord(host, payload) {
    const key = previewSlot();
    let record = graphicsState.records.get(key);
    if (!record || record.payload !== payload) {
        record = {key, payload, host, viewState: null, fault: null, report: null};
        graphicsState.records.set(key, record);
    }
    record.host = host; graphicsState.current = record;
    return record;
}
function disposeModelPreview() {
    restoreCalibrationTaskPreview();
    if (!modelView) return;
    const record = graphicsState.current;
    if (record?.payload === modelView.payload) record.viewState = modelView.captureViewState?.() || record.viewState;
    const old = modelView; modelView = null; graphicsState.ghostKey = '';
    if (ghostPending?.view === old) { ghostPending.resolve(); ghostPending = null; }
    if (ghostDrawnView === old) { ghostDrawnView = null; ghostDrawnAt = 0; ghostDrawnKey = ''; }
    old.dispose();
}
function graphicsAvailabilityProblem(requireGhost = false) {
    const report = modelView?.getRenderState?.();
    if (graphicsState.effectiveMode === 'rescue' || report?.status !== 'ready' || !report.modelLoaded || modelView?.gl?.isContextLost())
        return busyText('三维预览不可用，请先恢复预览，再确认目标或路径', 'Restore the 3D preview before confirming a target or path');
    const targetChanged = state.page === 'workbench' && state.workbenchTab === 'control' && !state.ghostRequestKey?.endsWith(':' + JSON.stringify(state.workbenchTargets));
    if (requireGhost && (targetChanged || !graphicsState.ghostKey || graphicsState.ghostKey !== state.ghostRequestKey))
        return busyText('正在绘制所选目标，请等待预览完成', 'Wait for the selected target preview to finish drawing');
    return '';
}
function frictionReplayProblem() {
    const graphics = graphicsAvailabilityProblem();
    if (graphics) return graphics;
    if (!workbenchConnected() || workbenchRecoveryRequired()) return busyText('请先连接工作台并恢复反馈','Connect the workspace and restore feedback first');
    const snapshot = state.telemetry || {}, calibration = snapshot.calibration || {};
    if (snapshot.robot_state !== 'ACTIVE') return busyText('请先使能机械臂','Activate the robot first');
    if (calibration.kind !== 'friction_ready' || calibration.phase !== 'waiting_replay_confirmation' || !(calibration.trajectory_samples >= 20)) return busyText('请先录制并结束摩擦标定轨迹','Finish recording a friction calibration trajectory first');
    return '';
}
function refreshFrictionReplayControls() {
    const reason = frictionReplayProblem();
    for (const id of ['friction-replay','friction-confirm-replay']) {
        const node = $(id); if (node) { node.disabled = !!reason; node.title = reason; }
    }
}
function refreshGraphicsDependentActions() {
    refreshFrictionReplayControls();
    if ($('target-move')) $('target-move').disabled = !workbenchTargetMayMove();
    if ($('target-draft-hint')) refreshWorkbenchTargetFields();
    if ($('teach-play')) refreshTeachEndpointState();
    if (state.page === 'calibration') { refreshCalibrationActionControls(); updateModelCalibrationTaskOverlay(); const resume = $('model-calibration-resume'); if (resume) resume.disabled = !!graphicsAvailabilityProblem(); }
    if (state.page === 'geometry') updateWorkbenchLive();
    const replay = $('model-calibration-replay-confirm');
    if (replay && graphicsAvailabilityProblem()) replay.disabled = true;
    for (const id of ['geometry-repeat-play', 'geometry-repeat-return']) { const node = $(id); if (node && graphicsAvailabilityProblem()) node.disabled = true; }
}
function graphicsActionsHtml() {
    const descriptions = {
        normal: busyText('使用显卡绘制，日常使用首选', 'Use the GPU for everyday model previews'),
        compatibility: busyText('使用 CPU 软件绘制，适合显卡预览异常时使用；画质与刷新率较低', 'Use CPU software rendering if GPU previews fail, at lower resolution and frame rate'),
        rescue: busyText('关闭三维预览，继续配置、查看数值和导出记录', 'Turn off 3D previews and keep configuration, numeric status and record export')
    };
    return '<div class="graphics-mode-options" role="group" aria-label="' + busyText('选择三维渲染方式', 'Choose a 3D rendering mode') + '">' +
        ['normal', 'compatibility', 'rescue'].map(mode => '<button type="button" class="graphics-mode-option' + (graphicsState.requestedMode === mode ? ' active' : '') + '" data-graphics-mode="' + mode + '" aria-pressed="' + (graphicsState.requestedMode === mode) + '"><span class="graphics-mode-title"><strong>' + graphicsModeLabel(mode) + '</strong><span class="graphics-mode-check">' + icon('check') + '</span></span><span class="graphics-mode-description">' + descriptions[mode] + '</span></button>').join('') + '</div>' +
        '<p class="hint graphics-security-note">' + busyText('兼容三维需确认软件 WebGL 的安全降级，渲染器沙箱仍保留；更换图形后端需要重启应用', 'Compatibility 3D requires confirmation of reduced software WebGL security; the renderer sandbox stays enabled. Changing the graphics backend requires an app restart') + '</p>' +
        (graphicsState.requiresRestart ? '<div class="graphics-pending" role="status"><div><strong>' + busyText('待重启生效', 'Restart required') + ' · ' + graphicsModeLabel(graphicsState.requestedMode) + '</strong><p class="hint">' + busyText('当前仍使用', 'Still using') + ' ' + graphicsModeLabel(graphicsState.effectiveMode) + busyText('；请先结束运行会话，再手动重启应用', '; end the runtime session before restarting the app') + '</p></div><button type="button" class="button primary" data-graphics-restart ' + (active() || state.blockingBusy ? 'disabled' : '') + '>' + busyText('重启并使用', 'Restart with') + ' ' + graphicsModeLabel(graphicsState.requestedMode) + '</button></div>' : '') +
        (graphicsState.actionError ? '<p class="parameter-error" role="alert">' + esc(graphicsState.actionError) + '</p>' : '');
}
function graphicsFailureHtml(record) {
    const rescue = graphicsState.effectiveMode === 'rescue';
    const title = rescue ? busyText('无三维救援模式', 'Rescue mode without 3D') : busyText('三维预览不可用', '3D preview unavailable');
    const detail = record?.fault?.message || busyText('当前不创建三维渲染器', 'The 3D renderer is disabled');
    return '<section class="graphics-failure" data-graphics-panel role="region" aria-label="' + title + '"><h3>' + title + '</h3><p>' +
        busyText('配置、数值状态、记录和导出仍可使用；需要目标或路径预览的执行确认，请先恢复三维', 'Configuration, numeric status, records and exports remain available. Restore 3D before confirming motions that require a target or path preview') +
        '</p><details><summary>' + busyText('错误详情', 'Error details') + '</summary><pre>' + esc(detail) + '</pre></details>' +
        '<div class="workspace-actions"><button type="button" class="button" data-graphics-retry>' + busyText('重试三维预览', 'Retry 3D preview') + '</button><button type="button" class="button" data-graphics-diagnostics>' + busyText('查看图形诊断', 'View graphics diagnostics') +
        '</button><button type="button" class="button" data-graphics-export>' + busyText('导出诊断', 'Export diagnostics') + '</button></div>' + graphicsActionsHtml() + '</section>';
}
function showGraphicsFailure(record) {
    if (!record?.host?.isConnected) return;
    record.host.innerHTML = graphicsFailureHtml(record);
    bindGraphicsActions(record.host);
    refreshGraphicsDependentActions();
}
function reportPreviewState(record, view, report) {
    record.report = report;
    if (report.status === 'ready' && report.modelLoaded) {
        record.fault = null;
        api.graphics?.report({...report, page: state.page}).catch(() => {});
        refreshGraphicsDependentActions();
    } else if (['failed', 'lost'].includes(report.status)) {
        record.viewState = view?.captureViewState?.() || record.viewState;
        record.fault = {message: report.error?.message || report.error || busyText('WebGL 图形上下文已失效', 'The WebGL context is unavailable'), category: 'graphics'};
        api.graphics?.report({...report, page: state.page}).catch(() => {});
        queueMicrotask(() => {
            if (modelView !== view || graphicsState.current !== record) return;
            disposeModelPreview(); showGraphicsFailure(record);
        });
    }
}
function failModelPreview(record, error, view, phase) {
    record.viewState = view?.captureViewState?.() || record.viewState;
    if (modelView === view) modelView = null;
    view?.dispose(); graphicsState.ghostKey = '';
    record.fault = {message: error.message || String(error), category: error.category || (phase === 'renderer' ? 'graphics' : 'model')};
    api.graphics?.report({status: 'failed', page: state.page, phase, error: record.fault, diagnostics: error.diagnostics}).catch(() => {});
    showGraphicsFailure(record);
}
async function refreshGraphicsStatus() {
    if (!api.graphics?.status) return;
    const status = await api.graphics.status();
    graphicsState.status = status;
    for (const key of ['effectiveMode', 'requestedMode', 'requiresRestart']) if (status[key] !== undefined) graphicsState[key] = status[key];
    return status;
}
function mountCurrentPreview() {
    if (state.page === 'model') return mountModelView();
    if (state.page === 'description') return mountDescriptionView();
    if (['workbench', 'geometry', 'calibration'].includes(state.page)) return mountWorkbenchView();
}
async function retryModelPreview() {
    if (graphicsState.effectiveMode === 'rescue') return changeGraphicsMode(graphicsState.status?.backendMode || 'normal');
    const record = graphicsState.current;
    if (record) record.fault = null;
    pageModelMount = null; workspaceMount = null; disposeModelPreview();
    await mountCurrentPreview();
}
async function changeGraphicsMode(mode, restart = false) {
    graphicsState.actionError = '';
    try {
        if (!api.graphics?.setMode) throw new Error(busyText('图形设置接口不可用，请使用完整应用启动', 'Graphics settings require the complete desktop application'));
        if (restart && (active() || state.blockingBusy)) throw new Error(busyText('请先手动结束运行会话，再重启应用', 'End the running session manually before restarting'));
        const result = await api.graphics.setMode(mode, {restart});
        if (result?.canceled) return;
        const previous = graphicsState.effectiveMode;
        await refreshGraphicsStatus();
        if (previous !== graphicsState.effectiveMode) {
            pageModelMount = null; workspaceMount = null; disposeModelPreview();
            if (graphicsState.current) graphicsState.current.fault = null;
            await mountCurrentPreview();
        }
    } catch (error) { graphicsState.actionError = error.message; }
    refreshGraphicsPanels();
}
function refreshGraphicsPanels() {
    const body = $('graphics-settings-body');
    if (body) { body.innerHTML = graphicsSettingsBodyHtml(); bindGraphicsActions(body); }
    if (graphicsState.current?.fault || graphicsState.effectiveMode === 'rescue') showGraphicsFailure(graphicsState.current);
}
function graphicsDiagnosticsHtml(status = {}) {
    const d = status.diagnostics || {}, gpu = d.gpuInfo || {}, reports = d.rendererReports || [], latest = reports.at(-1);
    const gl = latest?.diagnostics || {}, unknown = busyText('未获取', 'Not available');
    const value = item => item === null || item === undefined || item === '' ? unknown : String(item);
    const cell = item => esc(value(item));
    const yesNo = item => item === true ? busyText('是', 'Yes') : item === false ? busyText('否', 'No') : unknown;
    const stamp = item => item && Number.isFinite(Date.parse(item)) ? new Date(item).toLocaleString(state.language === 'en' ? 'en-GB' : 'zh-CN') : value(item);
    const badge = (label, tone = 'neutral') => '<span class="graphics-diagnostic-status is-' + tone + '">' + esc(label) + '</span>';
    const kv = rows => '<dl class="graphics-diagnostic-kv">' + rows.map(([label, item]) => '<dt>' + esc(label) + '</dt><dd>' + cell(item) + '</dd>').join('') + '</dl>';
    const empty = label => '<p class="graphics-diagnostic-empty">' + esc(label) + '</p>';
    const table = (headings, rows) => '<div class="graphics-diagnostic-table-wrap"><table class="graphics-diagnostic-table"><thead><tr>' + headings.map(label => '<th scope="col">' + esc(label) + '</th>').join('') + '</tr></thead><tbody>' + rows.map(row => '<tr>' + row.map(item => '<td>' + item + '</td>').join('') + '</tr>').join('') + '</tbody></table></div>';
    const section = (title, body, folded = false) => folded
        ? '<details class="graphics-diagnostic-section"><summary>' + esc(title) + '</summary>' + body + '</details>'
        : '<section class="graphics-diagnostic-section"><h4>' + esc(title) + '</h4>' + body + '</section>';
    const result = report => {
        if (!report) return badge(busyText('尚无预览记录', 'No preview recorded'));
        const labels = {ready: report.modelLoaded && report.drawCount > 0 ? busyText('模型已绘制', 'Model rendered') : busyText('渲染器已就绪', 'Renderer ready'), initializing: busyText('正在初始化', 'Initializing'), failed: busyText('预览失败', 'Preview failed'), lost: busyText('图形上下文丢失', 'Graphics context lost'), disposed: busyText('预览已关闭', 'Preview closed')};
        return badge(labels[report.status] || busyText('未完成绘制', 'Rendering incomplete'), ['failed', 'lost'].includes(report.status) ? 'error' : report.status === 'ready' ? 'ok' : 'neutral');
    };
    const pageLabel = page => ({model: t('model'), description: t('descriptionImport'), workbench: t('workbenchNav'), geometry: t('geometryNav'), calibration: t('calibrationNav')})[page] || value(page);
    const errorText = report => typeof report?.error === 'string' ? report.error : report?.error?.message || '';
    const summary = [
        [busyText('当前模式', 'Current mode'), esc(graphicsModeLabel(status.effectiveMode || 'normal'))],
        [busyText('最近一次预览', 'Latest preview'), result(latest)],
        [busyText('兼容模式运行库', 'Software 3D runtime'), status.compatibilityAvailable === true ? badge(busyText('文件齐全', 'Files present'), 'ok') : status.compatibilityAvailable === false ? badge(busyText('文件缺失', 'Files missing'), 'error') : badge(unknown)],
        [busyText('设置生效状态', 'Settings status'), status.requiresRestart ? badge(busyText('等待重启', 'Restart required'), 'warning') : badge(busyText('无需重启', 'No restart required'))]
    ];
    let html = '<div class="graphics-diagnostic-summary">' + summary.map(([label, item]) => '<div><span>' + esc(label) + '</span><strong>' + item + '</strong></div>').join('') + '</div>';
    const blocked = {SOFTWARE_RUNTIME_MISSING: busyText('兼容模式运行库不完整，已进入无三维救援模式', 'Software 3D files are missing; rescue mode is active'), SOFTWARE_CONSENT_REQUIRED: busyText('尚未确认软件渲染的安全影响，已进入无三维救援模式', 'Software rendering consent is missing; rescue mode is active')};
    const modeRows = [[busyText('已选模式', 'Selected mode'), graphicsModeLabel(status.requestedMode || 'normal')],
        [busyText('启动时指定模式', 'Startup override'), status.launchOverride ? graphicsModeLabel(status.launchOverride) : busyText('未指定，使用已保存设置', 'None; using saved settings')],
        [busyText('软件 WebGL 安全降级', 'Reduced software WebGL security'), status.unsafeSoftware === true ? busyText('已启用', 'Enabled') : status.unsafeSoftware === false ? busyText('未启用', 'Not enabled') : unknown]];
    html += section(busyText('渲染设置', 'Rendering settings'), kv(modeRows) + (status.blockedReason ? '<p class="graphics-diagnostic-note">' + cell(blocked[status.blockedReason] || status.blockedReason) + '</p>' : ''));
    html += section(busyText('最近一次三维预览', 'Latest 3D preview'), latest ? kv([
        [busyText('记录时间', 'Recorded at'), stamp(latest.time)], [busyText('所在页面', 'Page'), pageLabel(latest.page)],
        [busyText('预览模式', 'Preview mode'), latest.mode ? graphicsModeLabel(latest.mode) : unknown],
        [busyText('实际渲染后端', 'Actual renderer'), gl.unmaskedRenderer || gl.renderer],
        [busyText('渲染厂商', 'Rendering vendor'), gl.unmaskedVendor || gl.vendor],
        ['WebGL2', gl.webgl2 === true ? busyText('已创建', 'Created') : gl.webgl2 === false ? busyText('未创建', 'Not created') : unknown],
        [busyText('WebGL 版本', 'WebGL version'), gl.version],
        [busyText('抗锯齿', 'Antialiasing'), yesNo(gl.attributes?.antialias)],
        [busyText('像素比例', 'Pixel ratio'), gl.pixelRatio],
        [busyText('绘图缓冲区', 'Drawing buffer'), Array.isArray(gl.drawingBuffer) ? gl.drawingBuffer.join(' × ') : null],
        [busyText('模型网格数', 'Model meshes'), latest.meshCount], [busyText('绘制次数', 'Draw count'), latest.drawCount],
        [busyText('单帧绘制调用', 'Draw calls per frame'), latest.renderCalls], [busyText('单帧三角形数', 'Triangles per frame'), latest.triangles],
        ...(errorText(latest) ? [[busyText('错误原因', 'Error'), errorText(latest)]] : []),
        ...(gl.creationErrors?.length ? [[busyText('上下文创建错误', 'Context creation errors'), gl.creationErrors.join('\n')]] : [])
    ]) : empty(busyText('尚未创建三维预览；打开模型或标定页面后，可在这里查看实际绘制结果', 'No 3D preview has been created yet. Open a model or calibration page to record an actual rendering result')));
    const devices = gpu.gpuDevice || [], aux = gpu.auxAttributes || {};
    const hex = item => Number.isInteger(item) ? '0x' + item.toString(16) : value(item);
    html += section(busyText('显卡与驱动', 'GPU and driver'), (d.gpuInfoWaitTimedOut || d.gpuInfoPending || d.gpuInfoError ? '<p class="graphics-diagnostic-note">' + cell(d.gpuInfoError || (d.gpuInfoWaitTimedOut ? busyText('显卡信息读取超时；已获取的信息显示如下', 'GPU information timed out; available information is shown below') : busyText('显卡信息仍在读取', 'GPU information is still loading'))) + '</p>' : '') +
        (devices.length ? table([busyText('设备', 'Device'), busyText('PCI 标识', 'PCI ID'), busyText('驱动', 'Driver'), busyText('Chromium 活动设备', 'Active in Chromium')], devices.map((device, index) => [cell(device.deviceString || device.driverVendor || 'GPU ' + (index + 1)), cell(hex(device.vendorId) + ' / ' + hex(device.deviceId)), cell([device.driverVendor, device.driverVersion, device.driverDate].filter(Boolean).join(' · ')), cell(yesNo(device.active))])) : empty(busyText('未获取到显卡设备信息', 'GPU device information is unavailable'))) +
        kv([[busyText('系统图形实现', 'System graphics implementation'), aux.glImplementationParts], [busyText('系统渲染器', 'System renderer'), aux.glRenderer], [busyText('系统图形版本', 'System graphics version'), aux.glVersion]]));
    const featureLabels = {webgl:'WebGL', webgl2:'WebGL2', opengl:'OpenGL', vulkan:'Vulkan', webgpu:'WebGPU', gpu_compositing:busyText('GPU 合成', 'GPU compositing'), rasterization:busyText('光栅化', 'Rasterization'), '2d_canvas':'2D Canvas', video_decode:busyText('视频解码', 'Video decoding'), video_encode:busyText('视频编码', 'Video encoding')};
    const featureRows = Object.entries(d.gpuFeatureStatus || {}).map(([key, feature]) => {
        const enabled = /^enabled(?:_|$)/.test(feature), software = /software/.test(feature), disabled = /^disabled(?:_|$)/.test(feature);
        const label = software ? busyText('软件处理', 'Software') : enabled ? busyText('已启用', 'Enabled') : disabled ? busyText('未启用', 'Disabled') : feature;
        return [cell(featureLabels[key] || key.replaceAll('_', ' ')), badge(label, enabled ? 'ok' : 'neutral'), cell(feature)];
    });
    html += section(busyText('Chromium 图形功能', 'Chromium graphics features'), featureRows.length ? table([busyText('功能', 'Feature'), busyText('状态', 'Status'), busyText('状态代码', 'Status code')], featureRows) : empty(unknown), true);
    const resources = d.softwareRuntime || [];
    html += section(busyText('兼容模式运行库', 'Software 3D runtime'), resources.length ? table([busyText('文件', 'File'), busyText('状态', 'Status'), busyText('位置', 'Location')], resources.map(file => [cell(file.name), file.present === true ? badge(busyText('存在', 'Present'), 'ok') : file.present === false ? badge(busyText('缺失', 'Missing'), 'error') : badge(unknown), cell(file.path)])) : empty(unknown), true);
    const versions = d.versions || {}, switches = Object.entries(d.chromiumSwitches || {}), environment = Object.entries(d.environment || {});
    html += section(busyText('应用与启动环境', 'Application and startup'), kv([
        [busyText('操作系统', 'Operating system'), [d.platform, d.arch].filter(Boolean).join(' · ')],
        ['Electron', versions.electron], ['Chromium', versions.chrome], ['Three.js', gl.threeRevision ? 'r' + gl.threeRevision : null], ['Node.js', versions.node],
        [busyText('应用启动时间', 'Application started'), stamp(d.startedAt)]
    ]) + '<h4>' + busyText('图形启动选项', 'Graphics launch switches') + '</h4>' + (switches.length ? kv(switches.map(([key, item]) => ['--' + key, item === '' ? busyText('已启用', 'Enabled') : item])) : empty(busyText('未设置额外图形选项', 'No additional graphics switches'))) +
        '<h4>' + busyText('图形环境变量', 'Graphics environment') + '</h4>' + (environment.length ? kv(environment) : empty(busyText('未设置相关环境变量', 'No relevant environment variables'))) +
        '<h4>' + busyText('启动命令参数', 'Launch arguments') + '</h4><ul>' + (d.commandLine || []).map(arg => '<li><code>' + cell(arg) + '</code></li>').join('') + '</ul>', true);
    html += section(busyText('预览记录', 'Preview history') + ' · ' + reports.length, reports.length ? table([busyText('时间', 'Time'), busyText('页面', 'Page'), busyText('模式', 'Mode'), busyText('结果', 'Result'), busyText('错误', 'Error')], [...reports].reverse().map(report => [cell(stamp(report.time)), cell(pageLabel(report.page)), cell(report.mode ? graphicsModeLabel(report.mode) : unknown), result(report), esc(errorText(report))])) : empty(busyText('尚无预览记录', 'No preview recorded')), true);
    const logs = d.logs || [], native = d.nativeLog || {};
    const events = logs.map(log => '<article>' + kv([[busyText('时间', 'Time'), stamp(log.time)], [busyText('来源', 'Source'), log.source], ...(log.detail && typeof log.detail === 'object' ? Object.entries(log.detail).filter(([, item]) => item === null || typeof item !== 'object') : [[busyText('内容', 'Message'), log.detail]])]) + '</article>').join('');
    html += section(busyText('图形日志', 'Graphics logs'), (events || empty(busyText('没有应用图形事件记录', 'No application graphics events recorded'))) +
        '<h4>Chromium</h4>' + kv([[busyText('日志文件', 'Log file'), native.path]]) + (native.tail ? '<pre class="graphics-diagnostic-log">' + esc(native.tail) + '</pre>' : empty(native.unavailable ? busyText('日志暂不可读', 'Log unavailable') + ' · ' + native.unavailable : busyText('日志为空', 'Log is empty'))), true);
    return html;
}
async function showGraphicsDiagnostics() {
    try {
        await refreshGraphicsStatus();
        let dialog = $('graphics-diagnostics');
        if (!dialog) { dialog = document.createElement('dialog'); dialog.id = 'graphics-diagnostics'; dialog.className = 'studio-dialog graphics-dialog'; document.body.append(dialog); }
        dialog.setAttribute('aria-labelledby', 'graphics-diagnostics-title');
        dialog.innerHTML = '<header class="graphics-dialog-head">' + icon('diagnostics') + '<div><h3 id="graphics-diagnostics-title">' + busyText('图形诊断', 'Graphics diagnostics') + '</h3><p>' + busyText('查看渲染结果和图形环境；导出文件保留完整诊断数据', 'Inspect rendering results and the graphics environment; the export includes complete diagnostic data') + '</p></div></header><div class="graphics-dialog-body">' + graphicsDiagnosticsHtml(graphicsState.status) +
            '</div><footer class="graphics-dialog-footer"><button type="button" class="button" data-graphics-export>' + busyText('导出诊断文件', 'Export diagnostic file') +
            '</button><button type="button" class="button primary" data-graphics-close>' + busyText('关闭', 'Close') + '</button></footer>';
        bindGraphicsActions(dialog); if (!dialog.open) dialog.showModal();
    } catch (error) { graphicsState.actionError = error.message; refreshGraphicsPanels(); }
}
function bindGraphicsActions(root = document) {
    root.querySelectorAll('[data-graphics-mode]').forEach(button => button.onclick = () => changeGraphicsMode(button.dataset.graphicsMode));
    root.querySelectorAll('[data-graphics-retry]').forEach(button => button.onclick = () => retryModelPreview());
    root.querySelectorAll('[data-graphics-restart]').forEach(button => button.onclick = () => changeGraphicsMode(graphicsState.requestedMode, true));
    root.querySelectorAll('[data-graphics-diagnostics]').forEach(button => button.onclick = showGraphicsDiagnostics);
    root.querySelectorAll('[data-graphics-close]').forEach(button => button.onclick = () => button.closest('dialog').close());
    root.querySelectorAll('[data-graphics-export]').forEach(button => button.onclick = async () => {
        try { await api.graphics.exportDiagnostics(); }
        catch (error) { graphicsState.actionError = error.message; refreshGraphicsPanels(); }
    });
}
function graphicsSettingsBodyHtml() {
    return '<p class="graphics-current-mode">' + busyText('当前使用', 'Currently using') + '<strong>' + graphicsModeLabel(graphicsState.effectiveMode) + '</strong></p>' + graphicsActionsHtml() +
        '<div class="graphics-diagnostics-row"><div><h4>' + busyText('排查显示问题', 'Troubleshoot display issues') + '</h4><p class="hint">' + busyText('查看显卡、渲染后端和错误日志，或导出 JSON 诊断文件', 'Inspect the GPU, rendering backend and error logs, or export a JSON diagnostic file') + '</p></div><div class="workspace-actions"><button type="button" class="button" data-graphics-diagnostics>' + icon('diagnostics') + busyText('查看图形诊断', 'View graphics diagnostics') + '</button><button type="button" class="button" data-graphics-export>' + busyText('导出诊断文件', 'Export diagnostic file') + '</button></div></div>';
}
let pageModelMount = null;
async function mountPageModelView(page, hostId, getPayload, configure) {
    const host = $(hostId), payload = getPayload();
    if (state.page !== page || !payload || !host) return;
    if (pageModelMount?.host === host && pageModelMount.payload === payload) return pageModelMount.promise;
    if (modelView?.container === host && modelView.payload === payload) return;
    const record = previewRecord(host, payload);
    if (graphicsState.effectiveMode === 'rescue' || record.fault) { showGraphicsFailure(record); return; }
    const request = {host, payload}; pageModelMount = request;
    request.promise = (async () => {
        let view, phase = 'renderer';
        const current = () => pageModelMount === request && state.page === page && $(hostId) === host && getPayload() === payload;
        try {
            if (!modelModule) modelModule = await import('./model-view.js');
            if (!current()) return;
            disposeModelPreview();
            view = new modelModule.SerialArmModelView(host, file => api.modelResource(file), state.language, {mode: graphicsState.effectiveMode, onStatus: report => reportPreviewState(record, view, report)});
            modelView = view;
            phase = 'model';
            await view.setModel(payload);
            if (!current() || modelView !== view) { view.dispose(); return; }
            configure(view);
            if (record.viewState) view.restoreViewState(record.viewState);
            view.renderOnce();
        } catch (error) {
            if (current()) failModelPreview(record, error, view, phase);
            else view?.dispose();
        } finally { if (pageModelMount === request) pageModelMount = null; }
    })();
    return request.promise;
}
async function mountModelView() {
    return mountPageModelView('model', 'model-viewport', () => state.modelData, view => {
        view.setLayers(state.modelLayers);
        view.setInertiaSource(state.modelInertiaSource);
        for (const [name, value] of Object.entries(state.modelVisibility.links)) view.setObjectVisible('links', name, value);
        for (const [name, value] of Object.entries(state.modelVisibility.linkFrames)) view.setLinkFrameVisible(name, value);
        for (const [name, value] of Object.entries(state.modelVisibility.joints)) view.setObjectVisible('joints', name, value);
        view.select(state.modelSelection);
    });
}
async function previewModel() {
    const payload = state.modelData, epoch = state.workbenchEpoch;
    const native = await api.request('model_preview', { config: state.config, positions: state.modelPositions.slice() });
    if (state.page !== 'model' || state.modelData !== payload || state.workbenchEpoch !== epoch) return;
    payload.native = native;
    if (modelView?.container === $('model-viewport') && modelView.payload === payload) modelView.updatePose(native);
}
function workbenchConnected() { return state.session?.state === 'running' && state.session?.mode === 'workbench'; }
function telemetryAgeMs(snapshot = state.telemetry, now = Date.now()) {
    if (!snapshot || !Number.isFinite(snapshot.feedback_age_ms) || snapshot.feedback_age_ms < 0) return Infinity;
    if (!Number.isFinite(state.telemetryReceivedAt) || state.telemetryReceivedAt <= 0) return Infinity;
    return snapshot.feedback_age_ms + Math.max(0, now - state.telemetryReceivedAt);
}
function telemetryFresh(snapshot = state.telemetry, limitMs = 250) {
    return workbenchConnected() && snapshot?.valid === true && telemetryAgeMs(snapshot) <= limitMs;
}
function acceptTelemetrySnapshot(snapshot) {
    const previous = state.telemetry;
    const duplicate = previous && snapshot.sequence === previous.sequence && snapshot.monotonic_ns === previous.monotonic_ns;
    if (!duplicate || !state.telemetryReceivedAt) state.telemetryReceivedAt = Date.now();
    state.telemetry = snapshot;
}
function refreshTelemetryFreshness() {
    if (!workbenchConnected() || !state.telemetry) return;
    refreshLifecycleBringup();
    const age = telemetryAgeMs(), band = age <= 150 ? 0 : age <= 250 ? 1 : age <= 500 ? 2 : 3;
    refreshWorkbenchStatus();
    const ageText = Number.isFinite(age) ? age.toFixed(0) + ' ms' : '—';
    if ($('monitor-age')) $('monitor-age').textContent = ageText;
    if (state.telemetryFreshnessBand !== band) {
        state.telemetryFreshnessBand = band;
        if (band >= 2 && state.responseCapture?.recording) {
            state.responseCapture.recording = false;
            state.responseCapture.stop_reason = busyText('记录停止：反馈已过期，请检查连接后开始新记录', 'Recording stopped: feedback expired; check the connection and start a new recording');
        }
        updateWorkbenchLive();
    }
}
function workbenchTabs() {
    const en = state.language === 'en';
    const tabs = [
        ['control', 'controlTab', en ? 'Gain use and joint targets' : '增益用途与关节目标'],
        ['replay', 'replayTab', en ? 'Record and replay trajectories' : '录制与回放示教轨迹'],
        ['tuning', 'tuningTab', en ? 'Adjust interaction and record response' : '调整交互响应并记录'],
        ['monitor', 'monitorTab', en ? 'Feedback, references and torque' : '反馈、参考与力矩'],
    ];
    return `<nav class="calibration-entry-grid workbench-tab-grid" aria-label="${en ? 'Workspace tools' : '工作台工具'}">${tabs.map(([key, label, description]) => `<button type="button" data-workbench-tab="${key}" class="calibration-entry ${state.workbenchTab === key ? 'selected' : ''}" aria-pressed="${state.workbenchTab === key}"><span class="calibration-entry-title">${t(label)}</span><span>${description}</span></button>`).join('')}</nav>`;
}
function taskStateLabel(mode) {
    if (mode === 'COMPLIANT_DRAG') return state.language === 'en' ? 'Dragging' : '拖拽';
    if (mode?.endsWith('_HOLD')) return state.language === 'en' ? 'Holding' : '保持';
    if (mode?.endsWith('_TRACKING')) return state.language === 'en' ? 'Moving' : '跟踪移动';
    return '—';
}
function workbenchStatusValues() {
    const x = state.telemetry || {};
    const age = telemetryAgeMs(x), stale = workbenchConnected() && x.valid === true && !telemetryFresh(x);
    return {robot:robotStateLabel(x.robot_state) + (x.write_enabled===false ? ' · '+busyText('离线后端','Offline backend') : stale ? ' · '+busyText('反馈已过期','Feedback expired') : ''), behavior:profileOptionLabel(x.reference_behavior || referenceBehavior()),
        task:taskStateLabel(x.impedance_mode) + (x.gain_profile ? ' · ' + gainPurposeLabel(x.gain_profile) : ''),
        age:t('feedbackAge') + ' ' + (Number.isFinite(age) ? age.toFixed(0) + ' ms' : '—'), fault:x.fault?.code || 'OK'};
}
function workbenchStatusBar() {
    const values=workbenchStatusValues(), device=(state.session?.devices || []).join(', ') || t('sourceWaiting');
    return `<div class="workspace-status"><span><b>${esc(state.config.profile || '—')}</b></span><span>${esc(device)}</span>${Object.entries(values).map(([field,value])=>`<span data-status-field="${field}" class="${field==='fault' && value!=='OK' ? 'workspace-error' : ''}">${esc(value)}</span>`).join('')}</div>`;
}
function refreshWorkbenchStatus() {
    const values=workbenchStatusValues();
    for(const [field,value] of Object.entries(values)) document.querySelectorAll(`[data-status-field="${field}"]`).forEach(node=>{
        node.textContent=value;node.classList.toggle('workspace-error',field==='fault' && value!=='OK');
    });
}
function updateWorkbenchSpeed(raw) {
    state.speedInputRaw=String(raw);
    const value=Number(raw);
    state.speedInputError=String(raw).trim() && Number.isFinite(value) && value>0 && value<=1 ? '' : busyText('速度比例必须大于 0 且不超过 1','Speed fraction must be greater than 0 and at most 1');
    if(!state.speedInputError) state.speedScale=value;
    const input=$('workbench-speed');if(input){input.setCustomValidity(state.speedInputError);input.setAttribute('aria-invalid',String(!!state.speedInputError));}
    const error=$('workbench-speed-error');if(error) error.textContent=state.speedInputError;
    refreshWorkbenchTargetFields();
}
function updateMotionCompletion(snapshot) {
    const motion=snapshot?.motion;
    if(state.pendingMotionId==null || !motion || motion.id!==state.pendingMotionId) return;
    state.commandState=({completed:'complete',running:'running',failed:'failed',interrupted:'interrupted'})[motion.state] || state.commandState;
    if(['completed','failed','interrupted'].includes(motion.state)){state.pendingTarget=null;state.pendingMotionId=null;}
    if($('target-command-state')) $('target-command-state').textContent=commandStateText(state.commandState);
}
function workbenchTargetFeedbackReady() {
    const x = state.telemetry;
    return telemetryFresh(x)
        && Array.isArray(x.joint_names) && x.joint_names.length > 0 && x.joint?.pos?.length === x.joint_names.length
        && x.joint.pos.every(value => typeof value === 'number' && Number.isFinite(value));
}
function ensureWorkbenchTargetDraft() {
    if (!workbenchTargetFeedbackReady()) return;
    const names = state.telemetry.joint_names;
    if (state.workbenchTargetEpoch !== state.workbenchEpoch || state.workbenchTargets.length !== names.length) {
        state.workbenchTargets = state.telemetry.joint.pos.slice();
        state.workbenchTargetEpoch = state.workbenchEpoch;
        state.workbenchTargetBaseline = state.telemetry.sequence;
        state.workbenchTargetDirty = false;
    }
}
function workbenchTargetProblem() {
    const en = state.language === 'en', names = state.telemetry?.joint_names || [];
    if(state.modelReloadPending) return en ? 'Model changed on disk; reconnect before moving' : '模型已修改，请重新连接后再移动';
    if(state.speedInputError) return state.speedInputError;
    if (workbenchConnected() && !telemetryFresh()) return en ? 'Waiting for fresh feedback before moving' : '等待新鲜反馈后才能移动，请检查连接与电机反馈';
    if (state.workbenchTargetEpoch !== state.workbenchEpoch || !names.length || state.workbenchTargets.length !== names.length)
        return en ? 'Synchronize fresh feedback to create a target draft' : '请等待新鲜反馈或同步实际位置，以建立目标草稿';
    const joints = new Map((state.modelData?.model?.joints || []).map(joint => [joint.name, joint]));
    for (const [index, name] of names.entries()) {
        const value = state.workbenchTargets[index], joint = joints.get(name), limits = effectiveJointBounds(name,index,joint);
        if (!Number.isFinite(value)) return `${name} · ${en ? 'enter a finite target' : '请输入有效目标值'}`;
        if (limits.limited && (!Number.isFinite(limits.lower) || !Number.isFinite(limits.upper)))
            return `${name} · ${en ? 'Effective safety bounds are unavailable; refresh the Profile' : '尚未读取有效安全限位，请重新加载 Profile'}`;
        if (limits.limited && (value < limits.lower || value > limits.upper))
            return `${name} · ${en ? 'Target exceeds the allowed range' : '目标超出可执行范围'} ${jointDisplayValue(limits.lower,name)} ~ ${jointDisplayValue(limits.upper,name)}`;
    }
    return '';
}
function workbenchTargetMayMove() {
    const x = state.telemetry || {};
    return workbenchTargetFeedbackReady() && !workbenchTargetProblem() && !graphicsAvailabilityProblem(true) && x.robot_state === 'ACTIVE'
        && !workbenchRecoveryRequired() && !modelCalibrationActive() && x.gravity_verification?.active !== true
        && !['recording', 'playing'].includes(x.teach_replay?.phase)
        && ['RIGID_HOLD', 'RIGID_TRACKING', 'COMPLIANT_HOLD', 'COMPLIANT_TRACKING'].includes(x.impedance_mode)
        && ['configured', 'verified', 'stale'].includes(state.readiness?.modes?.[x.gain_profile === 'SERVO_TRACKING' ? 'SERVO_TRACKING' : x.impedance_mode.includes('COMPLIANT') ? 'COMPLIANT_TRACKING' : 'RIGID_TRACKING']);
}
function workbenchJointRows() {
    ensureWorkbenchTargetDraft();
    const names = state.telemetry?.joint_names || state.modelData?.native?.config?.joint_names || [];
    const joints = new Map((state.modelData?.model?.joints || []).map(joint => [joint.name, joint]));
    return names.map((name, index) => jointSliderRowHtml(name, index, joints.get(name) || {}, state.workbenchTargets[index], 'target', state.telemetry?.joint?.pos?.[index], workbenchTargetFeedbackReady())).join('');
}
function refreshWorkbenchTargetFields(writeInputs = false) {
    if (!$('workbench-target-editor')) return;
    const before = state.workbenchTargetEpoch; ensureWorkbenchTargetDraft();
    const initialized = before !== state.workbenchTargetEpoch;
    for (const [index, value] of state.workbenchTargets.entries()) {
        const name = state.telemetry?.joint_names?.[index], spec = jointDisplaySpec(name);
        if ($('target-value-' + index)) $('target-value-' + index).textContent = jointDisplayValue(value,name);
        const actual = state.telemetry?.joint?.pos?.[index];
        if ($('target-actual-' + index)) $('target-actual-' + index).textContent = Number.isFinite(actual) ? (actual * spec.factor).toFixed(3) : '—';
        for (const selector of [`[data-target-joint="${index}"]`, `[data-target-range="${index}"]`]) {
            const input = document.querySelector(selector); if (!input) continue;
            input.disabled = !workbenchTargetFeedbackReady();
            if ((writeInputs || initialized) && document.activeElement !== input && Number.isFinite(value)) input.value = (value * spec.factor).toFixed(3);
        }
    }
    const problem = workbenchTargetProblem() || graphicsAvailabilityProblem(true);
    if ($('target-draft-hint')) $('target-draft-hint').textContent = problem || (state.workbenchTargetDirty
        ? busyText('目标草稿已编辑 · 青绿色虚影仅预览，点击移动才执行', 'Target draft edited · the teal ghost is preview only; Move sends the command')
        : busyText('拖动滑条预览，点击“移动到目标”执行', 'Target anchored to live feedback · telemetry leaves the draft unchanged; disconnect clears it'));
    if ($('target-move')) $('target-move').disabled = !workbenchTargetMayMove();
    if ($('target-sync')) $('target-sync').disabled = !workbenchTargetFeedbackReady();
    if ($('target-command-state')) $('target-command-state').textContent = commandStateText(state.commandState);
    if (initialized) queueWorkbenchTargetGhost();
}
let workbenchTargetGhostFrame = 0;
function queueWorkbenchTargetGhost() {
    if (workbenchTargetGhostFrame) return;
    workbenchTargetGhostFrame = requestAnimationFrame(() => {
        workbenchTargetGhostFrame = 0;
        if (state.page === 'workbench' && state.workbenchTab === 'control') updateStartGhost();
    });
}
function editWorkbenchTarget(index, raw) {
    const value = jointInputToNative(state.telemetry?.joint_names?.[index], raw);
    state.workbenchTargets[index] = Number.isFinite(value) ? value : null;
    state.workbenchTargetDirty = true; state.commandState = 'edited'; state.pendingTarget = null; state.pendingMotionId = null;
    refreshWorkbenchTargetFields(true); queueWorkbenchTargetGhost();
}
function robotOperationHtml() {
    const en = state.language === 'en';
    const robotState = workbenchConnected() ? state.telemetry?.robot_state || 'UNKNOWN' : 'OFFLINE';
    const actions = items => items.map(([action, title]) => {
        const reason = controlOperationProblem(action);
        return `<button type="button" data-workbench-action="${action}" class="button ${action === 'deactivate' ? 'danger' : action === 'activate' ? 'primary' : ''}" ${reason ? 'disabled' : ''} title="${esc(reason)}">${title}</button>`;
    }).join('');
    return `<section class="workspace-section robot-operation-bar" id="robot-operation" aria-label="${en ? 'Robot controls' : '机械臂控制'}">
      <div class="robot-operation-state"><span class="robot-operation-dot ${robotState === 'ACTIVE' ? 'on' : 'off'}" aria-hidden="true"></span><strong>${en ? 'Robot state' : '机械臂状态'}</strong><span id="control-task-state">${esc(robotState)}</span><button type="button" class="button robot-operation-connect" data-robot-connect ${workbenchConnected() ? 'hidden' : ''}>${t('startWorkbench')}</button></div>
      <div class="robot-operation-actions"><div class="robot-operation-group">${actions([['activate', t('activateRobot')], ['park', t('parkRobot')], ['deactivate', t('deactivateRobot')]])}</div><div class="robot-operation-group robot-operation-recovery">${actions([['rigid_hold', en ? 'Rigid hold here' : '当前位置刚性保持'], ['fault_compliant', t('compliantRecovery')], ['clear_fault', t('clearFault')]])}</div></div>
      <p id="control-operation-hint" class="hint"></p><div id="control-operation-next"></div>
    </section>`;
}
let robotOperationHeaderObserver = null;
function updateRobotOperationOffset() {
    const page = $('page'), head = page?.querySelector(':scope > .page-head');
    const body = page?.querySelector(':scope > .studio-page-body');
    const sharedScroller = body && getComputedStyle(body).overflowY === 'visible';
    if (page) {
        const offset = head && sharedScroller ? `${head.getBoundingClientRect().height}px` : '0px';
        page.style.setProperty('--page-heading-offset', offset);
        page.style.setProperty('--robot-controls-top', offset);
    }
}
function observeRobotOperationHeader() {
    robotOperationHeaderObserver?.disconnect();
    updateRobotOperationOffset();
    const head = $('page')?.querySelector(':scope > .page-head');
    if (head) {
        robotOperationHeaderObserver ||= new ResizeObserver(updateRobotOperationOffset);
        robotOperationHeaderObserver.observe(head);
    }
}
function damiaoJointCheckData() {
    const editor = state.capabilityProfile || state.profileEditor;
    const plugin = state.info?.hardware_plugin || state.readiness?.info?.hardware_plugin || editor?.profile_entry?.hardware?.plugin;
    const damiao = plugin === 'serial_arm_hardware_damiao';
    const names = editor?.joint_names || editor?.core?.model?.joint_names || state.telemetry?.joint_names || [];
    const name = names.includes(state.damiaoCheckJoint) ? state.damiaoCheckJoint : names[0];
    const actuator = damiao ? editor?.hardware?.damiao?.actuators?.[name] || {} : null;
    const mapping = editor?.core?.calibration?.joints?.[name] || {};
    const snapshot = state.telemetry || {}, jointIndex = (snapshot.joint_names || []).indexOf(name);
    const actuatorIndex = actuator?.name ? (snapshot.actuator_info || []).findIndex(item => item.name === actuator.name) : -1;
    const index = actuatorIndex >= 0 ? actuatorIndex : damiao && snapshot.actuator?.pos?.length === snapshot.joint_names?.length ? jointIndex : -1;
    return {names,name,plugin,damiao,actuator,mapping,snapshot,jointIndex,actuatorIndex:index,
        fresh:telemetryFresh(snapshot) && snapshot.write_enabled === true};
}
function damiaoJointCheckBodyHtml(data) {
    const {names,name,plugin,damiao,actuator,mapping}=data,en=state.language==='en';
    const field=(label,value)=>`<dt>${label}</dt><dd>${esc(value ?? '—')}</dd>`;
    const direction=mapping.direction===1 ? busyText('正向 +1','Forward +1') : mapping.direction===-1 ? busyText('反向 -1','Reverse -1') : '—';
    const hardwareFields=damiao ? `${field(en?'Configured actuator':'配置执行器',actuator.name)}${field('motor_id / master_id',`${actuator.motor_id ?? '—'} / ${actuator.master_id ?? '—'}`)}${field('motor_type',actuator.motor_type)}` : '';
    const hardwareAction={page:'profile_setup',group:'hardware',field:damiao && name?`hardware.damiao.actuators.${name}.motor_id`:'profile_entry.hardware.plugin'};
    return `<label>${busyText('检查关节','Joint to inspect')}<select id="damiao-check-joint">${names.map(joint=>`<option value="${esc(joint)}" ${joint===name?'selected':''}>${esc(joint)}</option>`).join('')}</select></label>
      <p class="hint">${busyText('对照实物与接线逐轴检查执行器对应、正方向和零位；选择关节只查看配置与反馈','Check actuator assignments, positive direction and zeros against the mechanism and wiring; selecting a joint displays its settings and feedback')}</p>
      <dl class="kv readiness-joint-settings">${field(en?'Hardware driver':'硬件驱动',plugin || busyText('尚未选择','Not selected'))}${hardwareFields}${field(en?'Direction':'方向',direction)}${field(en?'Position / effort ratio':'位置 / 力矩比例',`${mapping.pos_ratio ?? '—'} / ${mapping.tor_ratio ?? '—'}`)}${field(en?'Joint zero offset':'关节零位偏移',jointDisplayValue(mapping.joint_zero_offset,name))}${damiao?field(en?'Actuator zero offset':'执行器零位偏移',jointDisplayValue(mapping.actuator_zero_offset,name,'position',3,'rad')):''}</dl>
      <p class="hint" id="damiao-check-feedback-state"></p><dl class="kv">
        <dt>${busyText('关节位置','Joint position')}</dt><dd id="damiao-check-position">—</dd><dt>${busyText('关节速度','Joint velocity')}</dt><dd id="damiao-check-velocity">—</dd><dt>${busyText('关节力 / 力矩','Joint force / torque')}</dt><dd id="damiao-check-effort">—</dd>
        ${damiao?`<dt>${busyText('执行器原始位置','Raw actuator position')}</dt><dd id="damiao-check-actuator-position">—</dd><dt>${busyText('电机在线 / 使能 / 错误码','Motor online / enabled / error')}</dt><dd id="damiao-check-actuator-state">—</dd>`:''}</dl>
      <div class="workspace-actions">${setupActionHtml(hardwareAction,busyText('硬件配置','Hardware configuration'))}${setupActionHtml({page:'profile_setup',group:'mapping',field:name?`core.calibration.joints.${name}.direction`:'core.calibration.joints'},busyText('配置方向与零位','Configure direction and zero'))}<button type="button" class="button" data-review-stage="mapping_calibration">${busyText('填写映射自查记录','Record mapping inspection')}</button></div>`;
}
function damiaoJointCheckHtml() {
    const data=damiaoJointCheckData();
    return `<details class="workspace-section readiness-joint-check" id="damiao-joint-check" data-readiness-section="joint-check" ${state.readinessSections?.['joint-check']?'open':''}><summary>${busyText('逐关节检查','Joint check')}</summary><div id="damiao-joint-check-body">${damiaoJointCheckBodyHtml(data)}</div></details>`;
}
function refreshDamiaoJointCheck() {
    if (!$('damiao-joint-check-body')) return;
    const data=damiaoJointCheckData();if(!data)return;
    const {name,snapshot,jointIndex,actuatorIndex,fresh}=data;
    const text=(id,value)=>{if($(id))$(id).textContent=value;};
    text('damiao-check-feedback-state',!workbenchConnected()?busyText('连接工作台后查看关节反馈','Connect the workspace to view joint feedback'):
        snapshot.write_enabled===false?busyText('当前为离线后端，不读取真实硬件','The current offline backend does not read hardware'):
        fresh?busyText('新鲜实机反馈，仍需与实物逐项对照','Current hardware feedback; compare each item with the mechanism'):
        busyText('等待新鲜关节反馈，请检查连接与机械臂状态','Waiting for fresh joint feedback; check the connection and robot state'));
    for(const [id,key,quantity] of [['damiao-check-position','pos','position'],['damiao-check-velocity','vel','velocity'],['damiao-check-effort','tor','effort']])
        text(id,jointDisplayValue(fresh?snapshot.joint?.[key]?.[jointIndex]:null,name,quantity));
    text('damiao-check-actuator-position',jointDisplayValue(fresh?snapshot.actuator?.pos?.[actuatorIndex]:null,name,'position',3,'rad'));
    const flag=value=>value===true || value===1?busyText('是','Yes'):value===false || value===0?busyText('否','No'):'—';
    text('damiao-check-actuator-state',fresh && actuatorIndex>=0 ? `${flag(snapshot.actuator?.online?.[actuatorIndex])} / ${flag(snapshot.actuator?.enabled?.[actuatorIndex])} / ${snapshot.actuator?.err_code?.[actuatorIndex] ?? '—'}`:'—');
}
function bindDamiaoJointCheck() {
    const body=$('damiao-joint-check-body');if(!body)return;
    if($('damiao-check-joint'))$('damiao-check-joint').onchange=event=>{
        state.damiaoCheckJoint=event.target.value;
        const data=damiaoJointCheckData();if(!data)return;
        body.innerHTML=damiaoJointCheckBodyHtml(data);bindDamiaoJointCheck();window.serialArmStudio?.enhanceSelects();
    };
    body.querySelectorAll('[data-setup-target]').forEach(button=>button.onclick=()=>operation(()=>openSetupTarget(JSON.parse(button.dataset.setupTarget))));
    body.querySelectorAll('[data-review-stage]').forEach(button=>button.onclick=()=>showReadinessReview(button.dataset.reviewStage));
    refreshDamiaoJointCheck();
}
function controlPanelHtml() {
    const en = state.language === 'en', x = state.telemetry || {}, inactive = x.robot_state === 'INACTIVE';
    const purposes = ['RIGID', 'COMPLIANT', 'DRAG', 'SERVO_TRACKING'];
    return `<div class="workspace-panel control-workspace"><section class="workspace-section workbench-target-card"><div class="workspace-section-head"><div><h3>${en ? 'Joint target' : '关节目标'}</h3><p class="hint">${en ? 'Preview with the sliders, then select Move to target' : '拖动滑条预览，点击“移动到目标”执行'}</p></div><label>${t('speedScale')}<input id="workbench-speed" class="workspace-number" type="number" min="0.01" max="1" step="0.01" value="${esc(state.speedInputRaw ?? state.speedScale)}" aria-invalid="${!!state.speedInputError}"><small id="workbench-speed-error" role="alert">${esc(state.speedInputError)}</small></label></div><div id="workbench-target-editor">${workbenchJointRows()}</div><p id="target-draft-hint" class="hint"></p><div class="workspace-actions"><button id="target-sync" class="button">${en ? 'Sync actual position' : '同步实际'}</button><button id="target-move" class="button primary" ${workbenchTargetMayMove() ? '' : 'disabled'}>${en ? 'Move to target' : '移动到目标'}</button><span id="target-command-state" class="command-state">${esc(commandStateText(state.commandState))}</span></div><div class="workspace-actions secondary-target-actions"><button id="target-zero" class="button">${en ? 'Zero target draft' : '目标归零（仅草稿）'}</button></div></section><details class="workspace-section control-settings" id="control-settings" ${state.controlSettingsOpen ? 'open' : ''}><summary>${en?'Control parameters':'控制参数'}</summary><div class="control-settings-body">${damiaoJointCheckHtml()}${referenceBehaviorHtml()}<section class="workspace-section"><div class="workspace-section-head"><div><h3>${en ? 'Actuator gain use' : '执行器增益用途'}</h3><p class="hint">${en ? 'Choose one of four gain presets; Hold and Move determine the task. Model admittance uses servo gains; dragging temporarily uses native impedance' : '四种用途对应四组增益，保持与移动决定任务；模型导纳使用伺服跟踪增益，拖拽暂用原生阻抗'}</p></div></div><div class="mode-buttons">${purposes.map(purpose => `<button data-gain-purpose="${purpose}" class="button ${x.gain_profile === purpose ? 'active' : ''}" ${controlGainPurposeMayChange(purpose) ? '' : 'disabled'}>${esc(gainPurposeLabel(purpose))}</button>`).join('')}</div><div id="control-gain-context">${controlGainContextHtml()}</div></section><section class="workspace-section"><h3>${t('modelFeedforward')}</h3><div class="mode-buttons">${['NONE', 'GRAVITY', 'FULL_INVERSE_DYNAMICS'].map(mode => `<button data-feedforward-mode="${mode}" class="button ${x.model_feedforward_mode === mode ? 'active' : ''}" ${inactive ? '' : 'disabled'}>${esc(profileOptionLabel(mode))}</button>`).join('')}</div></section></div></details></div>`;
}
function canonicalGainPurpose(purpose) {
    const key = String(purpose || '').toUpperCase();
    return ({RIGID:'RIGID',RIGID_HOLD:'RIGID',RIGID_TRACKING:'RIGID',COMPLIANT:'COMPLIANT',COMPLIANT_HOLD:'COMPLIANT',COMPLIANT_TRACKING:'COMPLIANT',DRAG:'DRAG',COMPLIANT_DRAG:'DRAG',SERVO:'SERVO_TRACKING',SERVO_TRACKING:'SERVO_TRACKING',IDENTIFICATION:'SERVO_TRACKING',IDENTIFICATION_TRACKING:'SERVO_TRACKING'})[key] || '';
}
function gainPurposeLabel(purpose) {
    const labels = {RIGID:['刚性','Rigid'],COMPLIANT:['柔性','Compliant'],DRAG:['拖拽','Drag'],SERVO_TRACKING:['伺服跟踪','Servo tracking']};
    return labels[canonicalGainPurpose(purpose)]?.[state.language === 'en' ? 1 : 0] || purpose || '—';
}
function controlGainContextHtml() {
    const en = state.language === 'en', x = state.telemetry || {};
    const vector = key => Array.isArray(x[key]) ? x[key].map((value,index) => key === 'hold_target_pos' ? jointDisplayValue(value,x.joint_names?.[index]) : key === 'hold_integral_torque' ? jointDisplayValue(value,x.joint_names?.[index],'effort') : capabilityNumber(value,3)).join(' / ') : '—';
    return `<dl class="kv control-gain-context"><dt>${en ? 'Task state' : '任务状态'}</dt><dd>${esc(taskStateLabel(x.impedance_mode))}</dd><dt>${en ? 'Gain use' : '增益用途'}</dt><dd>${esc(gainPurposeLabel(x.gain_profile))}</dd><dt>${en ? 'Gain source' : '增益来源'}</dt><dd>${esc(gainSourceLabel(x.gain_source))}</dd><dt>${en ? 'Scope' : '适用范围'}</dt><dd>${x.reference_behavior === 'MODEL_ADMITTANCE' ? (en ? 'Servo gains track the virtual reference; M/D/K defines interaction response' : '伺服增益跟踪虚拟参考，M/D/K 决定交互响应') : (en ? 'Native actuator PD follows the nominal reference' : '执行器原生 PD 跟踪名义参考')}</dd><dt>${en ? 'Effective Kp / Kd' : '实际 Kp / Kd'}</dt><dd class="mono-block">${esc(vector('effective_kp'))}<br>${esc(vector('effective_kd'))}</dd>${x.impedance_mode?.endsWith('_HOLD') ? `<dt>${en ? 'Fixed hold target' : '固定保持目标'}</dt><dd class="mono-block">${esc(vector('hold_target_pos'))}</dd><dt>${en ? 'Hold integral torque' : '保持积分力矩'}</dt><dd class="mono-block">${esc(vector('hold_integral_torque'))}</dd>` : ''}</dl>`;
}
// Offline capability configuration is read from the selected Profile
// Runtime values are taken only from the active Machine session
function interactionGoalData(context) {
    if (context === 'profile') {
        const editor = state.profileEditor, names = editor?.joint_names || [];
        const controller = editor?.core?.capability?.admittance?.controller || {};
        return { names, mass: names.map(name => controller.mass?.[name]), damping: names.map(name => controller.damping?.[name]), stiffness: names.map(name => controller.stiffness?.[name]) };
    }
    return { names: state.telemetry?.joint_names || [], ...(state.tuneDraft || state.admittance || {}) };
}
function interactionGoalDraft(context) {
    const data = interactionGoalData(context), index = Math.max(0, Math.min(state[context + 'GoalJoint'] || 0, data.names.length - 1));
    if (!state[context + 'InteractionGoal']) state[context + 'InteractionGoal'] = SerialArmMotionGoals.fromMdk(data.mass?.[index], data.damping?.[index], data.stiffness?.[index]);
    return { data, index, goal: state[context + 'InteractionGoal'] };
}
function interactionGoalFieldsHtml(context) {
    const en = state.language === 'en', { data, index, goal } = interactionGoalDraft(context);
    const name = data.names[index], linear = state.modelData?.model?.joints?.find(j => j.name === name)?.type === 'prismatic'
        || state.profileEditor?.urdf_joint_limits?.[name]?.type === 'prismatic';
    const spec = jointDisplaySpec(name,linear?'m':'rad');
    const unit = `${spec.unit} / ${spec.effortUnit}`, accelerationUnit = `${spec.accelerationUnit} / ${spec.effortUnit}`;
    const disabled = context === 'profile' ? '' : admittanceMayTune() ? '' : 'disabled';
    const number = (key, label, minimum, maximum = '') => {
        const factor = ['compliance','acceleration'].includes(key) ? spec.factor : 1;
        return `<label>${label}<input type="number" step="0.001" min="${minimum}" ${maximum ? `max="${maximum}"` : ''} data-goal-context="${context}" data-goal-key="${key}" value="${Number.isFinite(goal[key]) ? Number((goal[key] * factor).toPrecision(6)) : ''}" ${disabled}></label>`;
    };
    return `<div class="model-calibration-options interaction-goal-fields"><label>${en ? 'Joint' : '关节'}<select data-goal-joint="${context}" ${disabled}>${data.names.map((joint, i) => `<option value="${i}" ${i === index ? 'selected' : ''}>${esc(joint)}</option>`).join('')}</select></label><label>${en ? 'Interaction target' : '交互目标'}<select data-goal-context="${context}" data-goal-key="kind" ${disabled}><option value="return" ${goal.kind === 'return' ? 'selected' : ''}>${en ? 'Return after release' : '受扰后回位'}</option><option value="free" ${goal.kind === 'free' ? 'selected' : ''}>${en ? 'Move freely, slow after release' : '自由移动，松手减速'}</option></select></label>${number('time', en ? '95% response time (s)' : '95% 响应时间 s', .001)}${goal.kind === 'return' ? number('compliance', (en ? 'Steady yielding · ' : '恒力稳态退让 · ') + unit, .000001) + number('zeta', en ? 'Damping ratio' : '阻尼比 ζ', .7, 1.2) : number('acceleration', (en ? 'Acceleration per unit effort · ' : '单位外力初始加速度 · ') + accelerationUnit, .000001)}</div><p class="hint">${goal.kind === 'return' ? (en ? 'The time is the ideal virtual step response entering a 5% position band, with ζ between 0.7 and 1.2' : '时间指理想虚拟阶跃响应进入 5% 位置稳定带，阻尼比限定在 0.7～1.2') : (en ? 'The time is the ideal velocity decay to 5% after release; K=0 leaves the released position free' : '时间指松手后理想速度衰减至 5%，K=0 不拉回原位置')}${en ? ' · Verify the actual response using interaction recording' : ' · 实际响应使用交互记录验证'}</p><div id="${context}-goal-preview" class="calibration-muted-note"></div><div class="workspace-actions"><button class="button" data-goal-generate="${context}" ${data.names.length && !disabled ? '' : 'disabled'}>${en ? 'Generate parameter draft for this joint' : '生成所选关节参数草稿'}</button></div>`;
}
function interactionGoalEditorHtml(context) {
    return `<section class="calibration-step interaction-goal-editor"><h4>${state.language === 'en' ? 'Interaction response target' : '交互响应目标'}</h4><div id="${context}-interaction-goal">${interactionGoalFieldsHtml(context)}</div></section>`;
}
function updateInteractionGoalPreview(context) {
    const node = $(context + '-goal-preview'); if (!node) return;
    try { const draft=interactionGoalDraft(context),p = SerialArmMotionGoals.convert(draft.goal); node.textContent = `${jointMdkText(p,draft.data.names[draft.index])} · ${busyText('参数预览', 'Parameter preview')}`; }
    catch (error) { node.textContent = busyText('请填写合法的正时间、退让或加速度，回位阻尼比需为 0.7～1.2', 'Enter positive time and compliance or acceleration; return damping ratio must be 0.7–1.2'); }
}
function bindInteractionGoals() {
    document.querySelectorAll('[data-goal-context]').forEach(input => input.oninput = () => {
        const context = input.dataset.goalContext, draft = interactionGoalDraft(context), goal = draft.goal, key = input.dataset.goalKey;
        const factor = ['compliance','acceleration'].includes(key) ? jointDisplaySpec(draft.data.names[draft.index]).factor : 1;
        goal[key] = key === 'kind' ? input.value : input.value.trim() ? Number(input.value) / factor : NaN;
        if (key === 'kind') { const container = $(context + '-interaction-goal'); if (container) container.innerHTML = interactionGoalFieldsHtml(context); bindInteractionGoals(); }
        updateInteractionGoalPreview(context);
    });
    document.querySelectorAll('[data-goal-joint]').forEach(select => select.onchange = () => {
        const context = select.dataset.goalJoint; state[context + 'GoalJoint'] = Number(select.value); state[context + 'InteractionGoal'] = null;
        $(context + '-interaction-goal').innerHTML = interactionGoalFieldsHtml(context); bindInteractionGoals();
    });
    document.querySelectorAll('[data-goal-generate]').forEach(button => button.onclick = () => operation(() => {
        const context = button.dataset.goalGenerate, { data, index, goal } = interactionGoalDraft(context), values = SerialArmMotionGoals.convert(goal);
        if (context === 'profile') {
            const controller = state.profileEditor.core.capability.admittance.controller, name = data.names[index];
            for (const key of ['mass', 'damping', 'stiffness']) controller[key][name] = values[key];
            state.profileDirty = true;
            document.querySelectorAll('[data-profile-field]').forEach(input => { const path = JSON.parse(input.dataset.profileField); if (path.slice(0, 4).join('.') === 'core.capability.admittance.controller' && path.at(-1) === name) input.value = profileGet(state.profileEditor, path); });
            if ($('profile-dirty-state')) $('profile-dirty-state').textContent = busyText('存在未保存修改', 'Unsaved changes');
        } else {
            if (!admittanceMayTune()) return;
            state.tuneDraft = structuredClone(state.tuneDraft || state.admittance);
            for (const key of ['mass', 'damping', 'stiffness']) state.tuneDraft[key][index] = values[key];
            state.tuneDraftOrigin = 'goal'; state.savePreview = null; refreshRuntimeSavePreview(); refreshTuningInputs();
        }
        $(context + '-goal-preview').textContent = `${jointMdkText(values,data.names[index])} · ${busyText('草稿已生成，请应用或保存', 'Draft generated; apply or save it')}`;
    }));
    for (const context of ['profile', 'runtime']) updateInteractionGoalPreview(context);
}
function currentAdmittanceProfile() { return state.capabilityProfile?.core?.capability?.admittance || null; }
function admittanceCapabilityState() {
    const en = state.language === 'en';
    const profile = currentAdmittanceProfile();
    const runtime = workbenchConnected() ? state.admittance : null;
    const feedback = telemetryFresh();
    if (!profile) return { key: 'unconfigured', text: en ? 'Not configured' : '未配置', reason: state.capabilityProfileError || (en ? 'Cannot read capability configuration from Profile' : '尚未读取到 Profile 能力配置') };
    if (profile.enabled !== true) return { key: 'disabled', text: en ? 'Disabled in Profile' : 'Profile 未启用', reason: en ? 'Open configuration and enable external observation and interaction calculation' : '前往参数配置，开启外力观测与交互计算' };
    if (!workbenchConnected()) return { key: 'ready', text: en ? 'Configured · offline' : '配置已具备 · 离线', reason: en ? 'Connect the workspace to view its live state' : '连接工作台后查看实时状态' };
    if (state.telemetry?.robot_state === 'FAULT' || state.telemetry?.gravity_verification?.active || modelCalibrationActive())
        return { key: 'suspended', text: en ? 'Suspended or faulted' : '已挂起或故障', reason: en ? 'Current control or calibration state blocks normal operation' : '当前故障或标定状态不允许正常使用' };
    if (runtime?.enabled === false) return { key: 'disabled', text: en ? 'Disabled in session' : '当前会话未启用', reason: en ? 'Runtime configuration overrides saved values' : '运行会话配置与 Profile 中的配置不同' };
    if (referenceBehavior() === 'NATIVE_IMPEDANCE') return { key: 'ready', text: state.telemetry?.observer_valid === true ? (en ? 'Native impedance · torque observation' : '原生阻抗 · 外力观测') : (en ? 'Native impedance selected' : '当前选择原生阻抗'), reason: en ? 'The nominal reference is used without model-admittance displacement; an enabled observer may still estimate external torque' : '当前按名义参考执行，不叠加模型导纳退让；启用的观测器仍可估计外力矩' };
    if (feedback && state.telemetry.admittance_active === true)
        return { key: 'running', text: en ? 'Running' : '运行中', reason: en ? 'Model admittance is running' : '模型导纳运行中' };
    if (feedback && state.telemetry.admittance_suspended === true)
        return { key: 'suspended', text: en ? 'Suspended' : '已挂起', reason: en ? 'Model admittance is suspended' : '导纳已暂停' };
    if (state.telemetry?.robot_state === 'INACTIVE') return { key: 'ready', text: en ? 'Configured · robot disabled' : '已配置 · 机械臂失能', reason: en ? 'The selected model-admittance behavior takes effect after activation' : '已选择模型导纳，使能后才参与控制' };
    return { key: 'ready', text: en ? 'Waiting for cycle state' : '等待控制周期状态', reason: en ? 'Waiting for native admittance activation telemetry' : '等待原生运行时报告导纳参与状态' };
}
function admittanceMayTune() {
    const x = state.telemetry || {};
    return workbenchConnected() && !workbenchRecoveryRequired() && state.admittance?.enabled === true
        && currentAdmittanceProfile()?.enabled === true
        && referenceBehavior() === 'MODEL_ADMITTANCE'
        && (x.robot_state === 'INACTIVE' || (x.robot_state === 'ACTIVE' && telemetryFresh(x)
            && x.admittance_suspended !== true && x.impedance_mode !== 'COMPLIANT_DRAG'))
        && x.gravity_verification?.active !== true && !modelCalibrationActive();
}
function tuningRows() {
    const en = state.language === 'en';
    const runtime = workbenchConnected() && state.admittance;
    const a = runtime ? state.tuneDraft || runtime : null;
    if (!a) return '';
    const names = state.telemetry?.joint_names || [];
    const disabled = admittanceMayTune() ? '' : 'disabled';
    return names.map((name, i) => {
        const M = Number(a.mass?.[i] ?? 0), D = Number(a.damping?.[i] ?? 0), K = Number(a.stiffness?.[i] ?? 0);
        const wn = M > 0 && K > 0 ? Math.sqrt(K / M) : NaN;
        const dcrit = M > 0 && K > 0 ? 2 * Math.sqrt(M * K) : NaN;
        const z = dcrit > 0 ? D / dcrit : NaN;
        const units = jointMdkUnits(name), spec=jointDisplaySpec(name);
        return `<tr><td>${esc(name)}</td>${[['mass', M], ['damping', D], ['stiffness', K]].map(([key, value]) => `<td><input class="workspace-number" data-tune-key="${key}" data-tune-index="${i}" type="number" min="${key === 'mass' ? '0.000001' : '0'}" step="0.001" value="${Number(value).toFixed(4)}" ${disabled}><small>${units[key]}</small></td>`).join('')}<td>${Number.isFinite(wn) ? wn.toFixed(3) : '—'}</td><td>${Number.isFinite(z) ? z.toFixed(3) : '—'}</td><td>${K > 0 ? (spec.factor / K).toFixed(4) : '—'} ${spec.unit}/${spec.effortUnit}</td></tr>`;
    }).join('');
}
function tuningDraftSourceText() { return state.tuneDraftOrigin === 'goal' ? busyText('草稿来源：交互目标 · 尚未应用', 'Draft source: interaction target · not applied') : state.tuneDraftOrigin === 'expert' ? busyText('草稿来源：专家原值 · 尚未应用', 'Draft source: expert values · not applied') : busyText('当前显示本会话参数，保存默认需单独预览', 'Showing session parameters; saving defaults requires a separate preview'); }
function bindTuningInputs() {
    document.querySelectorAll('[data-tune-key]').forEach(input => input.oninput = () => {
        if (!state.tuneDraft) state.tuneDraft = structuredClone(state.admittance || {});
        const key = input.dataset.tuneKey, index = Number(input.dataset.tuneIndex);
        state.tuneDraft[key] = Array.isArray(state.tuneDraft[key]) ? state.tuneDraft[key].slice() : [];
        state.tuneDraft[key][index] = input.value.trim() ? Number(input.value) : NaN;
        state.tuneDraftOrigin = 'expert'; state.savePreview = null; refreshRuntimeSavePreview();
        if ($('tune-draft-source')) $('tune-draft-source').textContent = tuningDraftSourceText();
        const row = input.closest('tr'), mass = state.tuneDraft.mass?.[index], damping = state.tuneDraft.damping?.[index], stiffness = state.tuneDraft.stiffness?.[index];
        const frequency = mass > 0 && stiffness > 0 ? Math.sqrt(stiffness / mass) : NaN;
        if (row) { const spec=jointDisplaySpec(state.telemetry?.joint_names?.[index]);row.cells[4].textContent = capabilityNumber(frequency, 3); row.cells[5].textContent = capabilityNumber(frequency > 0 ? damping / (2 * Math.sqrt(mass * stiffness)) : null, 3); row.cells[6].textContent = `${capabilityNumber(stiffness > 0 ? spec.factor / stiffness : null)} ${spec.unit}/${spec.effortUnit}`; }
    });
}
function refreshTuningInputs() {
    if ($('capability-tune-body')) { $('capability-tune-body').innerHTML = tuningRows(); bindTuningInputs(); }
    if ($('tune-draft-source')) $('tune-draft-source').textContent = tuningDraftSourceText();
    refreshCapabilityDiagnostics();
}
function gravityVerificationPhaseText(phase,verify=state.telemetry?.gravity_verification || {}) {
    const labels={idle:['待开始','Not started'],prepared:state.telemetry?.robot_state==='ACTIVE' ?
        (verify.ramp_settled ? ['补偿已到位，可进入拖拽','Compensation settled; ready to drag'] : ['正在建立补偿','Building compensation']) : ['待使能','Ready to activate'],
        dragging:['正在拖拽采样','Drag sampling'],stopped:['已失能，待恢复设置','Disabled; ready to restore settings'],
        completed:['已完成，临时设置已恢复','Complete; temporary settings restored'],cancelled:['已取消','Cancelled'],failed:['验证失败','Verification failed']};
    return labels[phase]?.[state.language==='en'?1:0] || phase;
}
function gravityVerificationPanelHtml() {
    const en=state.language==='en',tr=(zh,english)=>en?english:zh;
    const x = state.telemetry || {};
    const verify = x.gravity_verification || {};
    const phase = verify.phase || 'idle';
    const connected = workbenchConnected();
    const active = verify.active === true;
    const inactive = x.robot_state === 'INACTIVE';
    const dragging = active && phase === 'dragging' && x.impedance_mode === 'COMPLIANT_DRAG';
    const connection = readinessConnectionState();
    const prerequisites = connection.basic_calibration_ready ?? ['configured', 'verified', 'stale'].includes(state.readiness?.modes?.RIGID_HOLD);
    const preflight = [];
    if (!active) {
        if (!connection.structurally_valid) {
            const issue = state.readiness?.setup_issues?.[0];
            preflight.push({id:'configuration',label:busyText('先补齐模型、硬件映射与 Safety 配置','Complete model, hardware mapping and safety settings'),
                action:issue?.action || {page:'profile_setup',group:issue?.group || 'hardware',field:issue?.path}});
        }
        if (!connected) preflight.push({id:'connection',label:busyText('连接工作台后，在失能状态准备验证','Connect the workspace and prepare verification while inactive'),action:{page:'diagnostics',anchor:'connection-settings'}});
        else if (!inactive) preflight.push({id:'inactive',label:x.robot_state === 'FAULT' ? busyText('先处理当前故障，再准备验证','Resolve the current fault before verification') : busyText('先停放并失能，再开始验证','Park and deactivate before verification'),action:{page:'workbench',tab:'control',anchor:'robot-operation'}});
    }
    const preflightHtml = !active ? `<div class="gravity-preflight" data-gravity-preflight>${preflight.length ? `<div class="readiness-todos">${preflight.map(item=>`<div data-preflight="${item.id}"><span>${esc(item.label)}</span>${setupActionHtml(item.action,busyText('处理此项','Resolve'))}</div>`).join('')}</div>` : ''}${readinessPrerequisitesHtml(connection)}</div>` : '';
    const sampleCount = Number(verify.sample_count || 0);
    const phaseText = gravityVerificationPhaseText(phase,verify);
    const requested = Array.isArray(state.gravityVerifyDraft) && state.gravityVerifyDraft.length === (x.joint_names || []).length ? state.gravityVerifyDraft : (verify.target?.length ? verify.target : x.gravity_scale || []);
    const rows = (x.joint_names || []).map((joint, i) => `<div class="gravity-verify-joint"><label>${esc(joint)}</label><input type="range" data-verify-scale-slider="${i}" min="0" max="2" step="0.01" value="${Number(requested[i] ?? 1).toFixed(2)}" ${dragging ? '' : 'disabled'}><input type="number" class="workspace-number" data-verify-scale-index="${i}" min="0" max="2" step="0.01" value="${Number(requested[i] ?? 1).toFixed(3)}" ${dragging ? '' : 'disabled'}><small>${Number(x.gravity_scale?.[i] ?? 0).toFixed(3)}</small></div>`).join('');
    return `<section class="workspace-section gravity-verification"><div class="workspace-section-head"><div><h3>${tr('重力补偿验证','Gravity compensation verification')}</h3><p class="hint">${tr('调整补偿比例，记录至少 3 个稳定姿态','Adjust compensation and record at least 3 stable poses')}</p></div><span class="readiness-chip ${phase === 'completed' ? 'ready' : 'pending'}" id="gravity-verify-phase">${esc(phaseText)}</span></div>
      <p class="parameter-guidance">${tr('准备机械支撑与急停；使能后先等补偿平滑到位，再进入受控拖拽','Prepare mechanical support and an emergency stop; after activation, wait for compensation to settle before entering controlled drag')}</p>${preflightHtml}
      <div class="workspace-actions"><button id="gravity-verify-begin" class="button primary" ${connected && prerequisites && inactive && !active ? '' : 'disabled'}>${tr('开始验证','Start verification')}</button><button id="gravity-verify-activate" class="button" ${active && inactive && phase === 'prepared' ? '' : 'disabled'}>${tr('使能并建立补偿','Activate and build compensation')}</button><button id="gravity-verify-enter-drag" class="button" ${active && phase === 'prepared' && x.robot_state === 'ACTIVE' && verify.ramp_settled ? '' : 'disabled'}>${tr('进入受控拖拽','Enter controlled drag')}</button></div>
      <p class="hint">${tr('滑条编辑目标比例，点击应用后平滑生效；当前比例显示实际生效值','Sliders edit target scales; Apply transitions them smoothly. Current scale shows the applied value')}</p>
      <div class="gravity-verify-grid"><div class="gravity-verify-joint gravity-verify-head"><b>${tr('关节','Joint')}</b><b>${tr('目标比例','Target scale')}</b><b></b><b>${tr('当前比例','Current scale')}</b></div>${rows}</div>
      <div class="workspace-actions"><button id="gravity-verify-apply" class="button" ${dragging ? '' : 'disabled'}>${tr('平滑应用目标','Apply target scales')}</button><button id="gravity-verify-sample" class="button" ${dragging && verify.ramp_settled ? '' : 'disabled'}>${tr('记录稳定姿态','Record stable pose')}</button><span class="hint" id="gravity-verify-samples">${tr('有效姿态','Valid poses')} ${sampleCount} / 3</span></div>
      <div class="workspace-actions"><button id="gravity-verify-park" class="button" ${active && x.robot_state === 'ACTIVE' ? '' : 'disabled'}>${tr('支撑后失能','Support, then disable')}</button><button id="gravity-verify-finish" class="button primary" ${active && inactive && phase === 'stopped' && sampleCount >= 3 ? '' : 'disabled'}>${tr('恢复设置并完成','Restore settings and finish')}</button><button id="gravity-verify-cancel" class="button" ${active && inactive ? '' : 'disabled'}>${tr('取消并恢复','Cancel and restore')}</button><button id="gravity-verify-evidence" class="button" ${phase === 'completed' ? '' : 'disabled'}>${tr('记录真机验收','Save hardware verification')}</button></div>
      <p class="hint">${tr('结束时先支撑再失能，不执行自动停放；故障或提前失能后需重新验证','Support the robot before disabling; this flow does not park automatically. Repeat verification after a fault or premature disable')}</p></section>`;
}
function referenceBehavior() {
    const profile = currentAdmittanceProfile();
    return (workbenchConnected() ? state.admittance?.reference_behavior || state.telemetry?.reference_behavior : null)
        || profile?.reference_behavior || (profile ? profile.enabled === true ? 'MODEL_ADMITTANCE' : 'NATIVE_IMPEDANCE' : '');
}
function admittanceUpdateText() {
    const update = state.telemetry?.parameter_update;
    if (!update || !Number.isFinite(update.progress)) return '';
    const progress = Math.max(0, Math.min(1, update.progress));
    return progress < 1 ? busyText('参数正在保留状态过渡', 'State-preserving parameter transition') + ` · ${Math.round(progress * 100)}%`
        : busyText('参数过渡已完成', 'Parameter transition complete');
}
function referenceBehaviorMayChange() {
    return workbenchConnected() && !workbenchRecoveryRequired() && state.telemetry?.robot_state === 'INACTIVE'
        && !modelCalibrationActive() && state.telemetry?.gravity_verification?.active !== true
        && !['recording', 'playing'].includes(state.telemetry?.teach_replay?.phase);
}
function referenceBehaviorHtml() {
    const en = state.language === 'en', behavior = referenceBehavior();
    const can = referenceBehaviorMayChange();
    const available = state.admittance?.enabled === true && currentAdmittanceProfile()?.enabled === true;
    const choices = [
        ['NATIVE_IMPEDANCE', en ? 'Native motor impedance' : '原生阻抗', en ? 'Kp / Kd follow the nominal reference; no model admittance offset' : '直接跟踪目标，使用所选用途的 Kp / Kd'],
        ['MODEL_ADMITTANCE', en ? 'Model reference admittance' : '模型导纳', en ? 'The observer drives virtual q / v / a; safety accepts the final reference and total torque' : '根据外力估计生成退让运动，由伺服跟踪执行'],
    ];
    return `<section class="workspace-section capability-behavior"><div class="workspace-section-head"><div><h3>${en ? 'Control method' : '控制方式'}</h3><p class="hint">${en ? 'Choose how a mode produces its reference; impedance mode and model feedforward are separate settings' : '选择当前会话的控制方式'}</p></div><span class="result-badge neutral" id="reference-behavior-current">${esc(profileOptionLabel(behavior))}</span></div><p class="hint">${en ? 'Effective control method' : '当前生效'} · <b id="reference-behavior-effective">${esc(workbenchConnected() ? profileOptionLabel(state.telemetry?.reference_behavior) : '—')}</b></p><div class="calibration-entry-grid">${choices.map(([key, title, description]) => `<button type="button" class="calibration-entry ${behavior === key ? 'selected' : ''}" data-reference-behavior="${key}" aria-pressed="${behavior === key}" ${can && (key === 'NATIVE_IMPEDANCE' || available) ? '' : 'disabled'}><span class="calibration-entry-title">${title}</span><span>${description}</span></button>`).join('')}</div><p class="hint">${en ? 'Park and disable before changing behavior. Model admittance also requires the capability enabled in Profile' : '停放并失能后可切换策略，模型导纳还需在 Profile 启用能力'}</p></section>`;
}
function capabilityNumber(value, digits = 4) {
    return typeof value === 'number' && Number.isFinite(value) ? value.toFixed(digits) : '—';
}
function capabilityReferenceRows() {
    const x = workbenchConnected() ? state.telemetry || {} : {}, joint = x.joint || {};
    const triple = (prefix, i) => ['pos', 'vel', 'acc'].map((key,index) => jointDisplayValue(telemetryFresh(x) ? joint[prefix + key]?.[i] : null,x.joint_names[i],['position','velocity','acceleration'][index])).join(' / ');
    return (x.joint_names || []).map((name, i) => `<tr><th>${esc(name)}</th><td>${triple('nominal_ref_', i)}</td><td>${triple('virtual_ref_', i)}</td><td>${triple('accepted_ref_', i)}</td><td>${triple('',i)}</td></tr>`).join('');
}
function capabilityTorqueRows() {
    const en = state.language === 'en', x = workbenchConnected() ? state.telemetry || {} : {}, joint = x.joint || {};
    return (x.joint_names || []).map((name, i) => `<tr><th>${esc(name)} · ${jointDisplaySpec(name).effortUnit}</th>${['pd_torque','model_feedforward','predicted_total_torque','effort_limit','tor'].map(key=>`<td>${capabilityNumber(telemetryFresh(x)?joint[key]?.[i]:null)}</td>`).join('')}<td>${telemetryFresh(x) && x.observer_valid === true ? capabilityNumber(joint.tau_ext_hat?.[i]) : en ? 'Invalid' : '无效'}</td></tr>`).join('');
}
function capabilityObservationHtml() {
    const en = state.language === 'en', x = workbenchConnected() ? state.telemetry || {} : {};
    const validity = !telemetryFresh(x) && x.valid === true ? (en ? 'Feedback expired' : '反馈已过期') : x.observer_valid === true ? (en ? 'Valid estimate' : '估计有效') : x.observer_valid === false ? (en ? 'Invalid / suspended' : '无效 / 已挂起') : '—';
    const affected = flags => Array.isArray(flags) ? flags.flatMap((flag, i) => flag ? [x.joint_names?.[i] || String(i + 1)] : []).join(', ') : '';
    const referenceLimits = affected(x.reference_limited), torqueLimits = affected(x.torque_limited);
    const reasons = [referenceLimits ? `${en ? 'Reference limited' : '参考受限'}: ${referenceLimits}` : '', torqueLimits ? `${en ? 'Total torque budget limited' : '总力矩预算受限'}: ${torqueLimits}` : ''].filter(Boolean);
    return `<div class="model-calibration-summary capability-observer-summary"><div><span>${en ? 'Disturbance estimate' : '扰动估计质量'}</span><b>${validity}</b></div><div><span>${en ? 'Measurement age' : '反馈年龄'}</span><b>${capabilityNumber(telemetryAgeMs(x), 2)} ms</b></div><div><span>${en ? 'Measurement interval' : '测量时间间隔'}</span><b>${capabilityNumber(typeof x.measurement_dt === 'number' ? x.measurement_dt * 1000 : null, 2)} ms</b></div><div><span>${en ? 'Joint timing skew' : '关节反馈时差'}</span><b>${capabilityNumber(x.feedback_skew_ms, 2)} ms</b></div><div><span>${en ? 'Current cycle measurement' : '本周期测量'}</span><b>${x.new_measurement === true ? (en ? 'New feedback' : '新反馈') : x.new_measurement === false ? (en ? 'No new feedback' : '无新反馈') : '—'}</b></div><div><span>${en ? 'Limits' : '限幅状态与原因'}</span><b>${reasons.length ? esc(reasons.join(' · ')) : Array.isArray(x.reference_limited) && Array.isArray(x.torque_limited) ? (en ? 'Within budget' : '预算内') : '—'}</b></div></div>`;
}
function capabilityDiagnosticsHtml() {
    const en = state.language === 'en';
    return `<section class="workspace-section capability-diagnostics"><div class="workspace-section-head"><div><h3>${en ? 'References, observer and torque budget' : '参考、观测器与力矩预算'}</h3><p class="hint">${en ? 'Rotary joints use degrees; linear joints use mm. Effort uses N·m or N per joint' : '旋转关节显示角度，直线关节显示 mm；力或力矩按关节显示 N 或 N·m'}</p></div></div><div id="capability-observation">${capabilityObservationHtml()}</div><h4>${en ? 'Nominal → virtual → accepted → measured' : '名义 → 虚拟 → 最终接受 → 实测'}</h4><div class="table-scroll"><table class="workspace-table capability-reference-table"><thead><tr><th>${t('joint')}</th><th>${en ? 'Nominal q / v / a' : '名义 q / v / a'}</th><th>${en ? 'Virtual q / v / a' : '虚拟 q / v / a'}</th><th>${en ? 'Accepted q / v / a' : '最终 q / v / a'}</th><th>${en ? 'Measured q / v / a' : '实测 q / v / a'}</th></tr></thead><tbody id="capability-reference-body">${capabilityReferenceRows()}</tbody></table></div><h4>${en ? 'Accepted command torque' : '最终命令力矩'}</h4><div class="table-scroll"><table class="workspace-table capability-torque-table"><thead><tr><th>${t('joint')}</th><th>PD</th><th>${en ? 'Feedforward' : '前馈'}</th><th>${en ? 'Predicted total' : '预测总力矩'}</th><th>${en ? 'Total limit' : '总预算'}</th><th>${en ? 'Measured torque' : '实测力矩'}</th><th>${en ? 'External estimate' : '扰动估计'}</th></tr></thead><tbody id="capability-torque-body">${capabilityTorqueRows()}</tbody></table></div></section>`;
}
function returnVerificationHtml(value) {
    const en = state.language === 'en';
    if (!value) return '';
    const verdict = result => result === true ? (en ? 'Passed' : '通过') : result === false ? (en ? 'Not passed' : '未通过') : (en ? 'Not verified' : '未验证');
    return `<div class="calibration-step"><h4>${en ? 'Return verification after release' : '受扰释放后回位验收'}</h4><div class="model-calibration-summary"><div><span>${en ? 'Original target unchanged' : '原目标保持不变'}</span><b>${verdict(value.target_unchanged)}</b></div><div><span>${en ? 'Joint return' : '关节回位'}</span><b>${verdict(value.joint_pass)}</b></div><div><span>${en ? 'Independent tool-tip measurement' : '针尖独立外部测量'}</span><b>${verdict(value.external_tcp_pass)}</b></div><div><span>${en ? 'External tool-tip error' : '针尖外部误差'}</span><b>${capabilityNumber(typeof value.tcp_error_m === 'number' ? value.tcp_error_m * 1000 : null, 3)} mm</b></div></div>${Array.isArray(value.joint_errors) ? `<div class="table-scroll"><table class="workspace-table"><thead><tr><th>${t('joint')}</th><th>${en ? 'Return error' : '回位误差'}</th><th>${en ? 'Allowed band' : '允许稳定带'}</th><th>${en ? 'Result' : '结果'}</th></tr></thead><tbody>${value.joint_errors.map(item => `<tr><th>${esc(item.joint)}</th><td>${jointDisplayValue(item.maximum_error,item.joint,'position',3,item.unit)}</td><td>${jointDisplayValue(item.tolerance,item.joint,'position',3,item.unit)}</td><td>${verdict(item.passed)}</td></tr>`).join('')}</tbody></table></div>` : ''}${value.reason ? `<p class="hint">${esc(value.reason)}</p>` : ''}</div>`;
}
function responseAnalysisHtml() {
    const en = state.language === 'en', analysis = state.responseAnalysis;
    if (!analysis) return '';
    return `${returnVerificationHtml(analysis.return_verification)}<div class="calibration-step"><h4>${en ? 'Execution dynamics estimate' : '执行动态估计'}</h4><div class="table-scroll"><table class="workspace-table capability-torque-table"><thead><tr><th>${t('joint')}</th><th>${en ? 'Delay ms' : '延迟 ms'}</th><th>${en ? 'Bandwidth Hz' : '带宽 Hz'}</th><th>ωn / 2π Hz</th><th>ζ</th><th>${en ? 'Holdout NRMSE' : '留出 NRMSE'}</th></tr></thead><tbody>${(analysis.joints || []).map(item => `<tr><th>${esc(item.joint)}</th><td>${capabilityNumber(typeof item.delay_s === 'number' ? item.delay_s * 1000 : null, 2)}</td><td>${capabilityNumber(item.bandwidth_hz, 3)}</td><td>${capabilityNumber(item.natural_frequency_hz, 3)}</td><td>${capabilityNumber(item.damping_ratio, 3)}</td><td>${capabilityNumber(item.validation_nrmse, 4)}</td></tr>${!item.identified || item.reason ? `<tr><td colspan="6" class="hint">${esc(item.reason || (en ? 'Insufficient data for identification' : '辨识数据不足'))}</td></tr>` : ''}`).join('')}</tbody></table></div><p class="hint">${en ? 'Execution response estimated from recorded references and motion, with held-out validation' : '根据最终参考与实测运动估计执行响应，并用留出数据验证'}</p></div>`;
}
function responseMayRecord() {
    const x = state.telemetry;
    return telemetryFresh(x) && !workbenchRecoveryRequired() && x?.robot_state === 'ACTIVE'
        && referenceBehavior() === x.reference_behavior
        && Number.isFinite(x.monotonic_ns)
        && !(Number.isFinite(x.parameter_update?.progress) && x.parameter_update.progress < 1)
        && !modelCalibrationActive() && x.gravity_verification?.active !== true;
}
function responseToleranceValues() {
    const capture = state.responseCapture, names = capture?.joint_names || state.telemetry?.joint_names || [];
    return names.map((name,index) => {
        const authored = state.responseTolerances?.[name];
        if (Number.isFinite(authored) && authored > 0) return authored;
        return jointDisplaySpec(name,capture?.joint_units?.[index]).linear ? 0.0001 : 0.01;
    });
}
function responseToleranceFieldsHtml() {
    const capture = state.responseCapture, names = capture?.joint_names || state.telemetry?.joint_names || [], values = responseToleranceValues();
    return `<details class="calibration-locks"><summary>${busyText('逐轴回位稳定带','Joint return bands')}</summary><div class="model-calibration-options">${names.map((name,index) => {
        const spec = jointDisplaySpec(name,capture?.joint_units?.[index]);
        return `<label>${esc(name)} · ${spec.unit}<input type="number" data-response-tolerance="${esc(name)}" min="0.000001" step="0.01" value="${Number((values[index] * spec.factor).toPrecision(6))}"></label>`;
    }).join('')}</div></details>`;
}
function responseMetricHtml() {
    const en = state.language === 'en', capture = state.responseCapture;
    if (!capture?.samples.length) return `<div class="calibration-empty-guidance">${en ? 'No response recording. Start a passive capture, then mark the moment external force is released' : '尚无响应记录，开始被动记录后，可手动标记外力释放时刻'}</div>`;
    const summary = SerialArmResponse.summarize(capture.samples, capture.release_time_s, responseToleranceValues());
    const index = Math.min(state.responseJoint || 0, summary.joints.length - 1), metric = summary.joints[index] || {};
    const spec = jointDisplaySpec(capture.joint_names[index],capture.joint_units?.[index]);
    const position = value => Number.isFinite(value) ? value * spec.factor : value;
    const fields = [
        [en ? 'Accepted tracking RMS' : '最终参考跟踪 RMS', position(metric.tracking_rms), ' '+spec.unit],
        [en ? 'Peak tracking error' : '最大跟踪偏差', position(metric.tracking_max), ' '+spec.unit],
        [en ? 'Peak measured deflection' : '最大实测退让', position(metric.peak_deflection), ' '+spec.unit],
        [en ? 'Peak estimated disturbance' : '最大估计外力', metric.peak_external_nm, ' '+spec.effortUnit],
        [en ? 'Final offset after release' : '释放后末态偏移', position(metric.final_offset), ' '+spec.unit],
        [en ? 'Opposite-side rebound' : '越过名义点的回弹峰值', position(metric.rebound), ' '+spec.unit],
        [en ? 'Post-release offset change' : '释放后偏移变化', position(metric.post_release_change), ' '+spec.unit],
        [en ? 'Return / settle time' : '回位稳定时间', metric.settling_s, ' s'],
    ];
    return `<div class="model-calibration-summary capability-response-metrics">${fields.map(([title, value, unit]) => `<div><span>${title}</span><b>${capabilityNumber(value)}${unit}</b></div>`).join('')}</div><p class="hint">${summary.samples} ${en ? 'snapshots' : '帧'} · ${summary.duration_s.toFixed(2)} s · ${en ? 'Valid observer' : '观测有效'} ${(summary.observer_valid_ratio * 100).toFixed(1)}% · ${en ? 'Reference limited' : '参考受限'} ${(summary.reference_limited_ratio * 100).toFixed(1)}% · ${en ? 'Torque limited' : '力矩受限'} ${(summary.torque_limited_ratio * 100).toFixed(1)}%</p><p class="hint">${metric.nominal_moving ? (en ? 'Nominal reference moved after release; no return-time score is assigned' : '释放后名义参考仍在运动，不计算回位稳定时间') : (en ? 'Return time requires a fixed nominal reference, the selected position band and 0.5 s continuous settling' : '回位时间按固定名义参考、所选位置稳定带与连续 0.5 s 稳定计算')}${capture.stop_reason ? ` · ${esc(capture.stop_reason)}` : ''}</p>`;
}
function externalTcpMeasurementHtml() {
    const en = state.language === 'en', draft = state.externalTcpDraft || {}, capture = state.responseCapture;
    return `<details class="calibration-locks"><summary>${en ? 'Optional external tool-tip return measurement' : '可选：针尖外部回位测量'}</summary><p class="hint">${en ? 'Manual independent measurement relative to the original target after release; these values are not computed from FK' : '松手后，使用独立外部测量记录针尖相对原目标的位移，以下数值不由 FK 计算'}</p><label class="confirmation"><input type="checkbox" id="response-tcp-measured" ${draft.confirmed ? 'checked' : ''} ${capture && !capture.recording ? '' : 'disabled'}>${en ? 'I measured the displacement with an external instrument' : '已使用外部测量设备记录位移'}</label><div class="model-calibration-options">${[['x', 'Δx mm'], ['y', 'Δy mm'], ['z', 'Δz mm'], ['tolerance', en ? 'Allowed error mm' : '允许误差 mm']].map(([key, label]) => `<label>${label}<input type="number" step="0.001" ${key === 'tolerance' ? 'min="0.000001"' : ''} data-response-tcp="${key}" value="${esc(draft[key] ?? '')}" placeholder="${en ? 'Not measured' : '未填写'}" ${capture && !capture.recording ? '' : 'disabled'}></label>`).join('')}</div><p class="hint" id="response-tcp-state">${draft.confirmed ? (en ? 'External measurement will be checked at export' : '导出时检查并保存外部测量') : (en ? 'External tool-tip return is not verified' : '针尖外部回位未验证')}</p></details>`;
}
function readExternalTcpMeasurement() {
    const draft = state.externalTcpDraft || {};
    if (!draft.confirmed) return null;
    const numbers = ['x', 'y', 'z', 'tolerance'].map(key => typeof draft[key] === 'string' && draft[key].trim() !== '' ? Number(draft[key]) : NaN);
    if (!numbers.every(Number.isFinite) || numbers[3] <= 0) throw new Error(busyText('外部测量需明确填写三轴位移与正数允许误差，未测量时请取消勾选', 'Enter all three measured displacements and a positive tolerance, or uncheck the external measurement'));
    return { source: 'manual_external_measurement', displacement_m: numbers.slice(0, 3).map(value => value / 1000), tolerance_m: numbers[3] / 1000 };
}
function responsePanelHtml() {
    const en = state.language === 'en', capture = state.responseCapture, names = capture?.joint_names || state.telemetry?.joint_names || [];
    return `<section class="workspace-section capability-response"><div class="workspace-section-head"><div><h3>${en ? 'Interaction response recording' : '交互响应记录'}</h3><p class="hint">${en ? 'Record the existing motion and force response; these controls issue no motion command' : '记录当前运动与外力响应，以下操作不发送运动命令'}</p></div><span class="result-badge neutral" id="response-phase">${capture?.recording ? (en ? 'Recording' : '记录中') : capture?.samples.length ? (en ? 'Stopped' : '已停止') : (en ? 'Not started' : '未开始')}</span></div>
      <div class="workspace-actions"><button id="response-start" class="button primary" ${responseMayRecord() && !capture?.recording ? '' : 'disabled'}>${en ? 'Start new recording' : '开始新记录'}</button><button id="response-release" class="button" ${capture?.recording && capture.samples.length && capture.release_time_s == null ? '' : 'disabled'}>${en ? 'Mark force released' : '标记外力已释放'}</button><button id="response-stop" class="button" ${capture?.recording ? '' : 'disabled'}>${en ? 'Stop recording' : '结束记录'}</button><button id="response-export" class="button" ${capture?.samples.length && !capture.recording ? '' : 'disabled'}>${en ? 'Analyze and export recording' : '分析并导出记录'}</button></div>
      <label>${en ? 'Observed joint' : '观测关节'}<select id="response-joint">${names.map((name, i) => `<option value="${i}" ${i === (state.responseJoint || 0) ? 'selected' : ''}>${esc(name)}</option>`).join('')}</select></label>${responseToleranceFieldsHtml()}
      <canvas id="capability-response-chart" height="190" aria-label="${en ? 'Nominal, virtual, accepted and measured position history' : '名义、虚拟、最终与实测位置曲线'}"></canvas><div class="capability-chart-legend"><span data-line="nominal">${en ? 'Nominal' : '名义'}</span><span data-line="virtual">${en ? 'Virtual' : '虚拟'}</span><span data-line="accepted">${en ? 'Accepted' : '最终'}</span><span data-line="pos">${en ? 'Measured' : '实测'}</span></div><div id="response-metrics">${responseMetricHtml()}</div><div id="response-analysis">${responseAnalysisHtml()}</div>${externalTcpMeasurementHtml()}<p class="hint">${en ? 'Use a separate recording for each strategy, parameter set, model or load; mark the release time and keep the comparison conditions unchanged' : '每组策略、参数、模型或载荷使用独立记录；松手时标记释放，比较时保持其余条件一致'}</p></section>`;
}
function drawResponseChart() {
    const canvas = $('capability-response-chart'); if (!canvas) return;
    const rect = canvas.getBoundingClientRect(), width = Math.max(1, rect.width), height = 190, ratio = devicePixelRatio || 1;
    canvas.width = width * ratio; canvas.height = height * ratio;
    const ctx = canvas.getContext('2d'); ctx.scale(ratio, ratio); ctx.clearRect(0, 0, width, height);
    const capture = state.responseCapture, samples = capture?.samples || [];
    if (samples.length < 2) return;
    const index = state.responseJoint || 0, spec = jointDisplaySpec(capture.joint_names[index],capture.joint_units?.[index]);
    const values = samples.flatMap(s => ['nominal', 'virtual', 'accepted', 'pos'].map(key => s[key][index])).filter(x => typeof x === 'number' && Number.isFinite(x));
    if (!values.length) return;
    const low = Math.min(...values), high = Math.max(...values), range = Math.max(0.01 / spec.factor, high - low), duration = samples.at(-1).time_s - samples[0].time_s;
    if (duration <= 0) return;
    const px = sample => 8 + (sample.time_s - samples[0].time_s) / duration * (width - 16);
    const py = value => height - 25 - (value - low + range * 0.05) / (range * 1.1) * (height - 40);
    for (const [key, color] of [['nominal', '#8a8a92'], ['virtual', '#df7b00'], ['accepted', '#22a4ba'], ['pos', '#9b72d4']]) {
        ctx.beginPath(); let open = false;
        for (const sample of samples) { const value = sample[key][index]; if (typeof value !== 'number' || !Number.isFinite(value)) { open = false; continue; } if (open) ctx.lineTo(px(sample), py(value)); else ctx.moveTo(px(sample), py(value)); open = true; }
        ctx.strokeStyle = color; ctx.lineWidth = key === 'pos' ? 2 : 1.25; ctx.stroke();
    }
    ctx.fillStyle = '#85858d'; ctx.font = '10px sans-serif'; ctx.fillText(`${(low * spec.factor).toFixed(3)} … ${(high * spec.factor).toFixed(3)} ${spec.unit}`, 8, 12); ctx.fillText(`${duration.toFixed(2)} s`, Math.max(8, width - 68), height - 5);
    if (capture.release_time_s != null) { const x = px({ time_s: capture.release_time_s }); ctx.setLineDash([4, 4]); ctx.beginPath(); ctx.moveTo(x, 15); ctx.lineTo(x, height - 20); ctx.strokeStyle = '#bd6262'; ctx.stroke(); ctx.setLineDash([]); }
}
function responseConditions(snapshot=state.telemetry) {
    const x=snapshot || {};
    return JSON.stringify({behavior:x.reference_behavior, task:x.impedance_mode, feedforward:x.model_feedforward_mode,
        gain_profile:x.gain_profile, kp:x.effective_kp || x.joint?.kp, kd:x.effective_kd || x.joint?.kd,
        gain_source:x.gain_source, gravity_scale:x.gravity_scale,
        hold_compensation:state.admittance?.hold_compensation, interaction:state.admittance && {enabled:state.admittance.enabled,
            joint_enabled:state.admittance.joint_enabled,mass:state.admittance.mass,damping:state.admittance.damping,
            stiffness:state.admittance.stiffness,torque_threshold:state.admittance.torque_threshold,observer_mode:state.admittance.observer_mode}});
}
function recordResponseSnapshot(snapshot) {
    const capture = state.responseCapture; if (!capture?.recording) return;
    if ((capture.conditions && capture.conditions!==responseConditions(snapshot)) || !responseMayRecord() || capture.session_id !== state.session?.session_id || capture.behavior !== snapshot.reference_behavior
        || capture.impedance_mode !== snapshot.impedance_mode || capture.model_feedforward_mode !== snapshot.model_feedforward_mode
        || JSON.stringify(capture.joint_names) !== JSON.stringify(snapshot.joint_names)) {
        capture.recording = false; capture.stop_reason = state.language === 'en' ? 'Stopped: session, control state, parameters or behavior changed' : '记录停止：会话、控制状态、参数或策略发生变化'; return;
    }
    const sample = SerialArmResponse.sample(snapshot), previous = capture.samples.at(-1);
    if (!sample || (previous && (sample.sequence === previous.sequence || sample.time_s <= previous.time_s ||
        (Number.isFinite(sample.feedback_receive_ns) && Number.isFinite(previous.feedback_receive_ns) && sample.feedback_receive_ns <= previous.feedback_receive_ns)))) return;
    sample.feedback_age_ms = telemetryAgeMs(snapshot);
    capture.samples.push(sample);
    if (capture.samples.length >= 6000) { capture.recording = false; capture.stop_reason = state.language === 'en' ? 'Recording capacity reached (6000 snapshots)' : '记录达到 6000 帧容量'; }
}
function refreshCapabilityDiagnostics() {
    if ($('capability-reference-body')) $('capability-reference-body').innerHTML = capabilityReferenceRows();
    if ($('capability-torque-body')) $('capability-torque-body').innerHTML = capabilityTorqueRows();
    if ($('capability-observation')) $('capability-observation').innerHTML = capabilityObservationHtml();
    const capture = state.responseCapture;
    if ($('response-phase')) $('response-phase').textContent = capture?.recording ? busyText('记录中', 'Recording') : capture?.samples.length ? busyText('已停止', 'Stopped') : busyText('未开始', 'Not started');
    const states = { 'response-start': responseMayRecord() && !capture?.recording, 'response-release': capture?.recording && capture.samples.length && capture.release_time_s == null, 'response-stop': capture?.recording, 'response-export': capture?.samples.length && !capture.recording };
    Object.entries(states).forEach(([id, allowed]) => { if ($(id)) $(id).disabled = !allowed; });
    if ($('response-metrics')) $('response-metrics').innerHTML = responseMetricHtml();
    if ($('response-analysis')) $('response-analysis').innerHTML = responseAnalysisHtml();
    for (const input of document.querySelectorAll('[data-response-tcp],#response-tcp-measured')) input.disabled = !capture || capture.recording;
    if ($('reference-behavior-current')) $('reference-behavior-current').textContent = profileOptionLabel(referenceBehavior());
    if ($('reference-behavior-effective')) $('reference-behavior-effective').textContent = workbenchConnected() ? profileOptionLabel(state.telemetry?.reference_behavior) : '—';
    if ($('capability-robot-state')) $('capability-robot-state').textContent = workbenchConnected() ? state.telemetry?.robot_state || '—' : busyText('离线', 'Offline');
    if ($('capability-tuning-state')) {
        const editable = admittanceMayTune();
        $('capability-tuning-state').textContent = editable ? busyText('允许受控调节', 'Ready to adjust') : busyText('只读', 'Read only');
        $('capability-tuning-state').className = 'readiness-chip ' + (editable ? 'pending' : 'muted');
    }
    if ($('admittance-update-state')) $('admittance-update-state').textContent = admittanceUpdateText();
    document.querySelectorAll('[data-reference-behavior]').forEach(button => {
        const selected = referenceBehavior() === button.dataset.referenceBehavior;
        button.classList.toggle('selected', selected); button.setAttribute('aria-pressed', String(selected));
        button.disabled = !referenceBehaviorMayChange() || (button.dataset.referenceBehavior === 'MODEL_ADMITTANCE' && (state.admittance?.enabled !== true || currentAdmittanceProfile()?.enabled !== true));
    });
    drawResponseChart();
}
function tuningPanelHtml() {
    const en = state.language === 'en', p = currentAdmittanceProfile(), status = admittanceCapabilityState();
    const connected = workbenchConnected(), a = connected ? state.admittance : null, editable = admittanceMayTune();
    const mode = connected && a ? a.observer_mode : p?.observer?.mode;
    return `<div class="workspace-panel capability-workspace"><section class="workspace-section capability-overview"><div class="workspace-section-head"><div><h3>${en ? 'Interaction tuning' : '交互调试'}</h3><p class="hint">${en ? 'Adjust interaction response and verify a recorded response here; choose strategy, actuator gain use and motion targets in Control' : '在这里调整交互响应并验收实测记录；控制策略、执行器增益用途与运动目标统一在控制页操作'}</p></div><span id="capability-runtime-status" class="readiness-chip ${status.key === 'running' ? 'ready' : status.key === 'suspended' ? 'warning' : 'pending'}">${esc(status.text)}</span></div><p id="capability-runtime-reason" class="hint">${esc(status.reason)}</p><dl class="kv"><dt>${en ? 'Saved source' : '配置来源'}</dt><dd>${p ? 'Profile · core.yaml' : '—'}</dd><dt>Observer</dt><dd>${esc(mode || '—')}</dd><dt>${en ? 'Control state' : '机械臂状态'}</dt><dd id="capability-robot-state">${esc(connected ? state.telemetry?.robot_state || '—' : en ? 'Offline' : '离线')}</dd></dl><div class="workspace-actions"><button id="open-persistent-config" type="button" class="button">${en ? 'Configure capability in Profile' : '前往 Profile 配置'}</button><button id="open-capability-calibration" type="button" class="button">${en ? 'Open dynamics calibration' : '前往动力学标定'}</button></div></section><section class="workspace-section capability-runtime"><div class="workspace-section-head"><div><h3>${en ? 'Online M / D / K' : '在线 M / D / K'}</h3><p class="hint">ωn=√(K/M) · ζ=D/(2√(MK)) · Δqss=τ/K</p></div><span id="capability-tuning-state" class="readiness-chip ${editable ? 'pending' : 'muted'}">${editable ? (en ? 'Ready to adjust' : '允许受控调节') : (en ? 'Read only' : '只读')}</span></div>${a ? interactionGoalEditorHtml('runtime') : ''}${!connected ? `<div class="empty-state">${en ? 'Connect the workspace to read runtime parameters' : '连接工作台后读取实际运行时参数'}</div>` : !a ? `<div class="empty-state">${en ? 'Loading runtime parameters' : '正在读取运行时参数'}</div>` : `<details class="calibration-locks"><summary>${en ? 'Expert M/D/K values' : '专家 M/D/K 原值'}</summary><div class="table-scroll"><table class="workspace-table capability-tune-table"><thead><tr><th>${t('joint')}</th><th>M</th><th>D</th><th>K</th><th>ωn</th><th>ζ</th><th>${en ? 'Yield per unit effort' : '单位外力退让'}</th></tr></thead><tbody id="capability-tune-body">${tuningRows()}</tbody></table></div></details><p id="tune-draft-source" class="hint">${tuningDraftSourceText()}</p><div class="workspace-actions"><button id="apply-tuning" class="button primary" ${editable ? '' : 'disabled'}>${en ? 'Apply bumpless update' : '应用到当前会话'}</button><button id="reset-tuning" class="button" ${editable ? '' : 'disabled'}>${t('reset')}</button></div>`}<p class="hint">${en ? 'M must be positive; D and K may be zero. Active updates transition on control cycles while preserving the accepted virtual motion state. Profile settings stay unchanged. K=0 provides no spring return' : 'M 必须大于零，D / K 允许为零；在线更新按控制周期保留已接受状态过渡，不写入 Profile；K=0 时不提供弹簧回位'}</p><p id="admittance-update-state" class="hint">${admittanceUpdateText()}</p>${!editable && connected ? `<p class="parameter-guidance">${en ? 'Select model admittance in Control first; disabled parameters may be preset, ACTIVE updates require fresh feedback outside drag or calibration' : '先在控制页选择模型导纳；失能时可预设参数，使能后需有效反馈、非拖拽且无标定任务才能在线调整'}</p>` : ''}</section>${responsePanelHtml()}</div>`;
}
// A persisted offline fit is a separate, explicitly selected gravity source
// No live control command is sent when reading or displaying it
function adoptGravityRecord(record) {
    state.modelCalibrationOffline = record;
    if (record?.selected_candidate) {
        const candidate=record.selected_candidate,kind=candidate.kind || 'gravity';
        state.calibrationCandidateSelection[kind]={...candidate,directory:record.directory};
    } else {
        for(const kind of ['gravity','dynamics']) if(state.calibrationCandidateSelection[kind]?.directory!==record?.directory) {
            state.calibrationCandidateSelection[kind]=null;state.calibrationCandidateRounds[kind]=[];
        }
    }
    state.idimReport = record?.idim_candidate || null;
    state.idimAudit = null;
    const offline = record?.recomputed_candidate;
    if (offline) {
        record.result = {
            ...(record.result || {}), gravity_result: offline,
            static_pass: offline.static_pass === true,
            friction_status: 'requires_refit_or_review', friction_pass: null,
        };
        state.modelCalibrationCandidateSource = 'offline';
        // Loading a result must not overwrite the user's next fitting choice.
        // Its original objective remains visible in the candidate summary.
    } else {
        state.modelCalibrationCandidateSource = 'online';
    }
}
function gravityCandidatePreviewState() {
    const result = currentModelCalibration()?.gravity_result;
    const offline = state.modelCalibrationCandidateSource === 'offline';
    const live = state.telemetry?.model_calibration;
    const directory = state.modelCalibrationOffline?.directory
        || (live?.workflow_kind === 'gravity' ? live.directory : null)
        || state.calibrationToolSelected.gravity?.directory || state.modelCalibrationDirectory;
    const binding = modelCalibrationModelBinding();
    const validated = result?.static_pass === true && result?.constraints_ok !== false
        && Array.isArray(result?.first_moments) && result.first_moments.length > 0;
    const inactive = workbenchConnected() && state.telemetry?.robot_state === 'INACTIVE';
    const applied = !!state.calibrationRuntimeCandidates?.gravity_applied;
    return {
        offline, directory, validated, comparable: binding.comparable,
        historical: binding.historical === true,
        applied, canApply: !!directory && validated && binding.comparable && !binding.historical && inactive && !applied
    };
}
function currentModelCalibration() {
    if (state.modelCalibrationOffline?.result) return state.modelCalibrationOffline.result;
    return workbenchConnected() ? state.telemetry?.model_calibration || null : null;
}
function modelCalibrationModelBinding() {
    const expected = state.modelCalibrationOffline?.metadata?.urdf_fingerprint || currentModelCalibration()?.urdf_fingerprint || '';
    const actual = state.modelData?.native?.config?.urdf_fingerprint || '';
    if (!expected || !actual) return { comparable: !state.modelCalibrationOffline, expected, actual, reason: state.modelCalibrationOffline ? (state.language === 'en' ? 'Model fingerprint is unavailable; candidate preview is disabled' : '模型指纹不可用，候选预览已禁用') : '' };
    if (expected !== actual) {
        const aligned = state.modelCalibrationPreviewAlignment;
        if (state.modelCalibrationOffline && aligned?.comparable && aligned.expected === expected && aligned.actual === actual && aligned.directory === state.modelCalibrationOffline.directory) {
            return { comparable: true, expected, actual, historical: true, alignment: aligned, reason: '' };
        }
        if (aligned?.directory === state.modelCalibrationOffline?.directory && aligned?.expected === expected && aligned?.actual === actual) {
            return { comparable: false, expected, actual, reason: aligned.error || aligned.warnings?.join(' · ') || '模型不兼容，无法预览历史候选' };
        }
        return { comparable: false, expected, actual, reason: state.language === 'en' ? 'Model fingerprint differs; verifying read-only historical comparison' : '模型指纹变化，正在核对历史快照与当前模型的兼容性' };
    }
    return { comparable: true, expected, actual, historical: false, reason: '' };
}
async function alignHistoricalCalibrationPreview(record = state.modelCalibrationOffline) {
    state.modelCalibrationPreviewAlignment = null;
    if (!record?.directory || !state.modelData) return;
    const expected = record.metadata?.urdf_fingerprint;
    const actual = state.modelData?.native?.config?.urdf_fingerprint;
    if (!expected || !actual || expected === actual) return;
    let alignment;
    try { alignment = await api.request('model_calibration_model_alignment', { config: state.config, directory: record.directory }); }
    catch (error) { alignment = { comparable: false, expected, actual, warnings: [], error: `历史候选对齐失败: ${error.message}` }; }
    if (state.modelCalibrationOffline?.directory !== record.directory) return;
    state.modelCalibrationPreviewAlignment = { ...alignment, directory: record.directory };
    render();
}

function modelCalibrationPhaseText(phase) {
    const labels = {
        idle: ['未启动', 'Idle'], teaching: ['拖动示教中', 'Teaching'], waiting_replay_confirmation: ['等待松手与回放确认', 'Waiting for replay confirmation'], gain_adaptation: ['执行器测量与增益验收', 'Actuator measurement and gain validation'],
        recorded_only: ['已录制轨迹 · 未标定', 'Recorded · Not calibrated'],
        static_reverse: ['反向回放 · 重力姿态及动力学数据', 'Reverse replay · gravity and dynamic data'], static_forward: ['正向静态停留采样', 'Forward static sampling'], gravity_fitting: ['拟合候选重力模型', 'Fitting gravity candidate'],
        friction_reverse_slow: ['低速反向摩擦采集', 'Slow reverse friction sampling'], friction_forward_slow: ['低速正向摩擦采集', 'Slow forward friction sampling'], friction_reverse_fast: ['高速反向摩擦采集', 'Fast reverse friction sampling'], friction_forward_fast: ['正向回放 · 动力学采集', 'Forward dynamic sampling'],
        friction_fitting: ['拟合并验证摩擦模型', 'Fitting and validating friction'], paused: ['已暂停并保持当前位置', 'Paused and holding'], complete: ['任务完成', 'Complete'], cancelled: ['任务已取消', 'Cancelled'], failed: ['任务失败', 'Failed'],
    };
    const value = labels[phase] || [phase || '—', phase || '—']; return state.language === 'en' ? value[1] : value[0];
}
function modelCalibrationActive(status = state.telemetry?.model_calibration) { return !!status && !['idle', 'complete', 'cancelled', 'failed', 'waiting_replay_confirmation'].includes(status.phase); }
function gainCalibrationSeconds(oneWay, mode, plan = currentModelCalibration() || {}) {
    const native = Number(plan.gain_estimated_duration_s ?? (plan.workflow_kind === 'gain' ? plan.estimated_duration_s : 0));
    return Number.isFinite(native) && native > 0 ? native : 0;
}
function gainCandidateGroups(candidate) {
    return candidate?.category === 'all' ? candidate.targets || {} : candidate ? {[candidate.category]:candidate} : {};
}
function gainCandidatePurposeAvailable(candidate, key) {
    const item = gainCandidateGroups(candidate)[key], count = Array.isArray(candidate?.joint_names) ? candidate.joint_names.length : 0;
    const finite = value => typeof value === 'number' && Number.isFinite(value);
    const vector = value => Array.isArray(value) && value.length === count && value.every(value => finite(value) && value >= 0);
    if (!item || item.available === false || !count || !vector(item.kp) || !vector(item.kd) || ['not_started','measurement_in_progress'].includes(item.error)) return false;
    if (Object.hasOwn(item,'hold_compensation') && (!item.hold_compensation || typeof item.hold_compensation !== 'object' || Array.isArray(item.hold_compensation) || typeof item.hold_compensation.enabled !== 'boolean')) return false;
    if (candidate.validation_scope === 'independent_demonstration_trajectory') {
        const path = value => value?.available !== false && value?.completed !== false && value?.independent === true && finite(value.duration_s) && value.duration_s > 0 &&
            (!Object.hasOwn(value,'measured_duration_s') || finite(value.measured_duration_s) && value.measured_duration_s >= value.duration_s - .06) &&
            Array.isArray(value.metrics) && value.metrics.length === count && value.metrics.every(row =>
                row && typeof row === 'object' && finite(row.samples) && row.samples > 0 && (!Object.hasOwn(row,'measurement_samples') || finite(row.measurement_samples) && row.measurement_samples > 0) && ['rms_error_rad','peak_error_rad','velocity_rms_rad_s','tail_error_rad','settling_s'].every(field => finite(row[field]) && row[field] >= 0));
        const hold = value => value?.available !== false && value?.completed !== false && finite(value?.samples) && value.samples > 0 && !['not_started','measurement_in_progress'].includes(value.reason) &&
            (!Object.hasOwn(value,'fresh_samples') || Array.isArray(value.fresh_samples) && value.fresh_samples.length === count && value.fresh_samples.every(sample=>finite(sample) && sample>0)) &&
            ['worst_error_rad','worst_velocity_rad_s'].every(field => finite(value[field]) && value[field] >= 0);
        if (key === 'rigid_tracking') return path(item.native_validation) && vector(item.hold_gains?.kp) && vector(item.hold_gains?.kd) &&
            hold(item.hold_compensation?.enabled === true ? item.hold_compensation_verification : item.hold_verification);
        if (key === 'compliant_tracking') return path(item.native_validation) && vector(item.hold_gains?.kp) && vector(item.hold_gains?.kd) &&
            hold(item.hold_verification) && vector(item.rigid_reference_kp) && vector(item.rigid_reference_kd) && finite(item.tested_speed_fraction) && item.tested_speed_fraction > 0;
        if (key === 'servo_tracking') {
            const nativeOnly = item.admittance_required === false && item.admittance_validation?.required === false && item.admittance_validation?.status === 'not_requested';
            return path(item.native_validation) && (nativeOnly || path(item.admittance_validation));
        }
        if (key === 'compliant_drag') {
            const value = item.drag_validation;
            return item.kp.every(value=>value === 0) && value?.available !== false && value?.completed === true && finite(value.samples) && value.samples > 0 && Array.isArray(value.axes) && value.axes.length === count && value.axes.every(axis =>
                axis && typeof axis === 'object' && finite(axis.samples) && axis.samples > 0 && ['initial_velocity_rad_s','peak_velocity_rad_s','final_velocity_rad_s','decay_s'].every(field => finite(axis[field]) && axis[field] >= 0));
        }
        return false;
    }
    // Older local-probe records retain their recorded qualification contract.
    if (item.validated !== true) return false;
    if (key === 'compliant_tracking') return !!item.hold_gains &&
        item.hold_verification?.validated === true && vector(item.rigid_reference_kp) && vector(item.rigid_reference_kd) && Number(item.tested_speed_fraction) > 0;
    if (key === 'rigid_tracking') return !!item.hold_gains &&
        (item.hold_compensation?.enabled === true ? item.hold_compensation_verification : item.hold_verification)?.validated === true;
    if (key === 'compliant_drag') return item.drag_validation?.validated === true;
    return key === 'servo_tracking' && item.native_validation?.validated === true && item.admittance_validation?.validated === true;
}
function gainCandidateDeployable(candidate, purposes = null) {
    if (!candidate) return false;
    const groups = gainCandidateGroups(candidate);
    if (purposes === null && candidate.category === 'all' && Object.keys(groups).length !== 4) return false;
    const keys = purposes === null ? candidate.category === 'all' ? ['rigid_tracking','compliant_tracking','compliant_drag','servo_tracking'] : [candidate.category] : purposes;
    return Array.isArray(keys) && keys.length > 0 && keys.every(key => gainCandidatePurposeAvailable(candidate,key));
}
function gainCandidateAvailabilityLabel(candidate, key) {
    const item = gainCandidateGroups(candidate)[key];
    if (!gainCandidatePurposeAvailable(candidate,key)) return busyText(item?.error === 'not_started' ? '未测量 · 不可选' : '测量不完整或参数不可用', item?.error === 'not_started' ? 'Not measured · unavailable' : 'Incomplete measurement or unavailable parameters');
    return item.validated === true ? busyText('达到参考指标 · 可应用/导出','Within reference limits · apply/export available') : busyText('未达参考指标 · 可应用/导出','Outside reference limits · apply/export available');
}
function gainCalibrationQualityWarning(status) {
    return status?.workflow_kind === 'gain' && /^gain candidate failed continuous measured qualification:/i.test(status.error || '');
}
function gainWarningLabel(warning) {
    const parts = String(warning).split(':'), purpose = ['rigid_tracking','compliant_tracking','compliant_drag','servo_tracking'].includes(parts[0]) ? gainPurposeLabel(parts.shift()) : '';
    if (parts[0] === 'noise_not_stationary') return `${parts[1] || ''} · ${busyText('静止窗口仍有运动，噪声估计已记录','Movement remained during the stationary window; noise estimates were recorded')}`;
    const compensation = parts[0] === 'hold_compensation'; if (compensation) parts.shift();
    if (parts[0] === 'endpoint_alignment_required') return [purpose,busyText('端点尚未回稳，后续回放未开始；已完成的测量仍可采用，请查看关节目标与偏差','The endpoint has not settled; later replay did not start. Completed measurements remain usable; review joint targets and differences')].filter(Boolean).join(' · ');
    if (['observer_unavailable','hold_compensation_inactive'].includes(parts[0])) return [purpose,compensation ? busyText('保持补偿','Hold compensation') : '',parts[0] === 'observer_unavailable' ? busyText('观测器有效采样未达参考要求','Observer samples are below the reference requirement') : busyText('保持补偿未充分建立','Hold compensation did not fully establish')].filter(Boolean).join(' · ');
    const labels = {tracking_reference_not_met:['跟踪未达参考指标','Tracking is outside reference limits'],drag_reference_not_met:['拖拽释放未达参考指标','Drag release is outside reference limits'],admittance_reference_not_met:['导纳未达参考指标','Admittance is outside reference limits'],hold_position_error:['保持位置误差超出参考指标','Hold position error exceeds the reference limit'],hold_velocity_exceeded:['保持速度超出参考指标','Hold speed exceeds the reference limit'],hold_not_stationary:['保持阶段仍有运动','Movement remained during the hold check'],insufficient_measurements:['采样量未达参考要求','The sample count is below the reference requirement'],limited:['测量期间出现限幅','Limiting occurred during measurement'],measurement_incomplete:['测量未完成','Measurement is incomplete'],not_started:['尚未测量','Not measured'],measurement_in_progress:['测量未完成','Measurement is incomplete'],time_budget_reached:['已到本次标定时限，未完成用途保留为未测量','The time budget was reached; unfinished uses remain unmeasured'],time_budget_may_limit_measurements:['按当前安全重定时，部分测量可能超出本次时限','With the current safety retiming, some measurements may exceed this time budget']};
    const text = labels[parts[0]]?.[state.language === 'en' ? 1 : 0] || String(warning);
    return [purpose,compensation ? busyText('保持补偿','Hold compensation') : '',text].filter(Boolean).join(' · ');
}
function gainTaskWarningsHtml(status = {}) {
    const warnings = Array.isArray(status.gain_warnings) ? status.gain_warnings : [];
    if (!warnings.length && !gainCalibrationQualityWarning(status)) return '';
    const complete = status.phase === 'complete' || gainCalibrationQualityWarning(status);
    return `<div class="callout gain-quality-warning" role="status"><b>${complete ? busyText('标定已结束，测量提示已保留','Calibration finished; measurement notes retained') : busyText('标定进行中，已记录测量提示','Calibration in progress; measurement notes recorded')}</b><p>${complete ? busyText('完整测量且参数有效的用途可以直接应用或导出，实测结果保留供查看','Fully measured uses with valid parameters can be applied or exported; measured results remain available') : busyText('自动测量继续进行，提示随任务结果保存','Automatic measurement continues; notes are saved with the task')}</p>${warnings.length ? `<ul>${warnings.map(value=>`<li>${esc(gainWarningLabel(value))}</li>`).join('')}</ul>` : ''}</div>`;
}
function gainSelectionKey(candidate) {
    return JSON.stringify([state.config?.profile_file,state.config?.profile,state.calibrationToolSelected?.gain?.directory,candidate?.task_id,candidate?.category]);
}
function selectedGainPurposes(candidate = state.calibrationToolSelected?.gain?.gain_candidate) {
    if (!candidate) return [];
    const available = Object.keys(gainCandidateGroups(candidate)).filter(key => gainCandidateDeployable(candidate,[key]));
    const saved = state.gainPurposeSelections?.[gainSelectionKey(candidate)];
    return Array.isArray(saved) ? saved.filter(key => available.includes(key)) : available;
}
function gainVerificationDescription(candidate, en) {
    if (!candidate) return '';
    const stages = candidate.category === 'all' ? (candidate.targets || {}) : { [candidate.category]: candidate };
    return Object.entries(stages).map(([key, data]) => {
        const name = gainPurposeLabel(key);
        const speed = Math.round(Number(data.tested_speed_fraction || 1) * 100);
        const hold = data.hold_compensation?.enabled === true ? data.hold_compensation_verification : data.hold_verification;
        return `${name} ${gainCandidateAvailabilityLabel(candidate,key)} · ${en ? 'Test speed' : '测试速度比例'} ${speed}%${hold ? ` · ${en ? 'Hold reference' : '保持参考指标'} ${hold.validated ? 'PASS' : 'WARNING'}` : ''}`;
    }).join(' · ');
}
function modelCalibrationLongRunning(status = state.telemetry?.model_calibration) { return !!status && ['teaching', 'gain_adaptation', 'static_reverse', 'static_forward', 'gravity_fitting', 'friction_reverse_slow', 'friction_forward_slow', 'friction_reverse_fast', 'friction_forward_fast', 'friction_fitting', 'paused'].includes(status.phase); }
function gravityValidationSummary(result) {
    const report = result?.gravity_validation;
    if (!report) return '';
    const en = state.language === 'en';
    const labels = {
        improved: en ? 'Offline raw prediction improved' : '离线原始预测改善',
        equivalent: en ? 'No meaningful improvement demonstrated' : '未证明有效改善',
        degraded: en ? 'Raw prediction degraded' : '原始预测退化',
        prior_retained: en ? 'Input model retained' : '保留输入模型',
        insufficient_evidence: en ? 'Insufficient evidence' : '证据不足',
    };
    const number = value => Number.isFinite(value) ? value.toFixed(5) : '—';
    const row = (label, scores) => `<tr><td>${label}</td><td>${number(scores?.validation?.baseline?.aggregate_rms_nm)}</td><td>${number(scores?.validation?.candidate?.aggregate_rms_nm)}</td></tr>`;
    return `<div class="callout"><b>${esc(labels[report.status] || report.status)}</b><div>${result.fit_objective === 'urdf' ? (en ? 'Fitted direct URDF prediction' : '拟合目标：URDF 直接预测') : (en ? 'Fitted with constant feedback offsets separated' : '拟合目标：分离反馈常值偏置')}${result.constraints_ok === false ? (en ? ' · Parameter constraints failed' : ' · 参数物理检查未通过') : ''}</div><table><thead><tr><th>${en ? 'Holdout aggregate RMS (N m)' : '留出总 RMS（N m）'}</th><th>${en ? 'Input model' : '输入模型'}</th><th>${en ? 'Candidate' : '候选模型'}</th></tr></thead><tbody>${row(en ? 'Raw URDF prediction' : '原始 URDF 预测', report.raw_urdf)}${row(en ? 'Training offsets removed' : '扣除训练集偏置后', report.bias_corrected)}</tbody></table><div>${en ? 'Offline improvement is scored from raw prediction on held-out poses from this recording; verify hardware compensation in a separate session' : '离线改善按本次记录留出姿态的原始预测评价；另开会话验证实机补偿'}</div></div>`;
}
function modelCalibrationMetricTable(status) {
    const result = status?.gravity_result; if (!result) return '';
    const offline = state.modelCalibrationCandidateSource === 'offline';
    const en = state.language === 'en';
    const names = status.joint_names || state.telemetry?.joint_names || [];
    const original = result.validation_original || {}, scaled = result.validation_scaled || {}, candidate = result.validation_candidate || {};
    const metric = (source, key, i) => {
        const value = source?.[key]?.[i];
        return value !== undefined && value !== null && Number.isFinite(Number(value)) ? Number(value).toFixed(4) : '—';
    };
    const rows = names.map((name, i) => `<tr><td>${esc(name)}</td><td>${metric(original, 'rms', i)}</td><td>${metric(scaled, 'rms', i)}</td><td>${metric(candidate, 'rms', i)}</td><td>${metric(candidate, 'p99', i)}</td><td>${metric(candidate, 'max', i)}</td><td>${metric({ noise: result.noise_rms }, 'noise', i)}</td></tr>`).join('');
    const testName = offline ? (en ? 'Offline static holdout' : '离线静态留出评价') : (en ? 'Online static holdout' : '在线静态留出评价');
    const testResult = result.static_validation_status === 'unavailable' ? (en ? 'Unavailable' : '不可用')
        : result.static_validation_status === 'limited' ? (en ? 'Limited validation · review only' : '验证姿态不足 · 仅供审核')
            : (result.static_pass ? (en ? 'Within validation limits' : '满足验证容差') : (en ? 'Exceeds validation limits' : '超出验证容差'));
    const noiseTitle = offline ? (en ? 'Scoring scale (not sensor noise)' : '评分归一化尺度（非传感器噪声）') : t('noiseRms');
    return `<div class="model-calibration-results"><div class="result-badges"><span class="result-badge neutral">${testName}</span><span class="result-badge ${result.static_validation_status === 'unavailable' ? 'neutral' : result.static_pass ? 'pass' : 'warn'}">${testResult}</span><span class="result-badge ${status.friction_status === 'not_evaluated' || status.friction_status === 'requires_refit_or_review' ? 'neutral' : status.friction_pass ? 'pass' : 'warn'}">${status.friction_status === 'not_evaluated' || status.friction_status === 'requires_refit_or_review' ? (en ? 'Friction not retested' : '摩擦未复验') : t(status.friction_pass ? 'frictionPassed' : 'frictionFailed')}</span><span class="result-badge neutral">${t('dynamicsNotIdentified')}</span><span class="result-badge neutral">${t('numericalRank')} ${Number(result.numerical_rank ?? 0)}</span>${Number.isFinite(Number(result.selected_regularization)) ? `<span class="result-badge neutral">CV λ ${Number(result.selected_regularization).toPrecision(3)}</span>` : ''}${result.constraints_ok === false ? `<span class="result-badge fail">${en ? 'Mass/COM out of bounds' : '质量或质心超界'}</span>` : ''}${result.prior_preferred ? `<span class="result-badge warn">${en ? 'Prior preferred' : '交叉验证倾向原模型'}</span>` : ''}</div><p class="hint">${offline ? (en ? 'Offline scoring evaluates a different full-data fit and does not share the online acceptance threshold. It cannot certify unvisited poses or hardware holding.' : '离线以完整记录重算质量与质心，评价门槛与在线不同，不代表未示教姿态或真机保持已验证') : (en ? 'Online tests only the fixed-mass first-moment fit with its own validation and torque-offset model, not the offline URDF.' : '在线仅评价固定质量的一阶质量矩候选，验证时使用自身的力矩偏置口径，不能与离线 URDF 评分直接比较')}</p><div class="table-scroll"><table class="workspace-table calibration-metrics"><thead><tr><th>${t('joint')}</th><th>${t('currentUrdfRms')}</th><th>${t('currentScaleRms')}</th><th>${t('candidateRms')}</th><th>${t('candidateP99')}</th><th>${t('candidateMaximum')}</th><th>${noiseTitle}</th></tr></thead><tbody>${rows}</tbody></table></div></div>`;
}
function modelCalibrationCandidateTable(status) {
    const result = status?.gravity_result; const moments = result?.first_moments || []; if (!moments.length) return '';
    const links = new Map((state.modelData?.model?.links || []).map(link => [link.name, link]));
    const observable = result.parameter_observable || []; const offsets = new Map((result.com_offsets_m || []).map(x => [x.link_name, x.value]));
    const rows = moments.map((item, index) => {
        const link = links.get(item.link_name); const mass = Number(link?.inertial?.mass || 0); const original = link?.inertial?.origin?.xyz || [0, 0, 0];
        const binding = modelCalibrationModelBinding();
        const savedInertial = binding.historical ? binding.alignment.source_inertials?.[item.link_name] : null;
        const sourceMass = Number(savedInertial?.mass ?? mass);
        const sourceCom = Array.isArray(savedInertial?.com) ? savedInertial.com : original;
        const candidateMass = Number(result.candidate_masses?.[item.link_name] ?? sourceMass);
        const candidate = candidateMass > 0 ? item.value.map(v => Number(v) / candidateMass) : [NaN, NaN, NaN];
        const delta = candidate.map((v, i) => v - Number(sourceCom[i] || 0));
        const offset = Math.hypot(...delta);
        const axes = observable.length >= (index + 1) * 3 ? String(observable.slice(index * 3, index * 3 + 3).reduce((sum, v) => sum + (Number(v) ? 1 : 0), 0)) + '/3' : '—';
        const massText = Number.isFinite(candidateMass) && candidateMass > 0
            ? `${candidateMass.toFixed(4)} kg${Math.abs(sourceMass - candidateMass) > 1e-8 ? `<br><small>${state.language === 'en' ? 'Source' : '原始'} ${sourceMass.toFixed(4)} kg</small>` : ''}`
            : '—';
        return `<tr><td>${esc(item.link_name)}</td><td>${massText}</td><td>${esc(vectorText(sourceCom, 5))}</td><td>${candidate.every(Number.isFinite) ? esc(vectorText(candidate, 5)) : '—'}</td><td>${Number.isFinite(offset) ? (offset * 1000).toFixed(2) + ' mm' : '—'}</td><td>${axes}</td></tr>`;
    }).join('');
    return `<div class="table-scroll"><table class="workspace-table"><thead><tr><th>Link</th><th>${state.language === 'en' ? 'Candidate mass' : '候选质量'}</th><th>${t('currentCom')}</th><th>${modelCalibrationModelBinding().historical ? (state.language === 'en' ? 'Historical candidate COM' : '历史候选 COM') : t('candidateCom')}</th><th>${t('comOffset')}</th><th>${t('observableAxes')}</th></tr></thead><tbody>${rows}</tbody></table></div>`;
}
// The view mirrors Core's hard replay preflight, but never substitutes for it.
function calibrationAlignment(status = currentModelCalibration(), telemetry = state.telemetry) {
    const target = status?.replay_start_joint_positions;
    const pos = telemetry?.joint?.pos;
    const vel = telemetry?.joint?.vel;
    const targetValid = Array.isArray(target) && target.length > 0 && target.every(Number.isFinite);
    if (!targetValid) return {valid:false,aligned:false,maxError:Infinity,maxSpeed:Infinity,maxErrorRatio:Infinity,maxSpeedRatio:Infinity,rows:[]};
    const fresh = telemetryFresh(telemetry,500);
    const valid = !!status?.source_task_id &&
        Array.isArray(pos) && pos.length === target.length &&
        Array.isArray(vel) && vel.length === target.length &&
        telemetry?.robot_state === 'ACTIVE' && fresh &&
        [...target, ...pos, ...vel].every(Number.isFinite);
    const rows = target.map((q, i) => {
        const name = (status.joint_names || telemetry?.joint_names || [])[i] || `joint${i + 1}`;
        const unit = status.joint_units?.[i] || jointDisplaySpec(name).nativeUnit;
        const positionTolerance = status.alignment_position_tolerance?.[i] ?? (unit === 'm' ? .001 : .08);
        const velocityTolerance = status.alignment_velocity_tolerance?.[i] ?? (unit === 'm' ? .001 : .05);
        const now = fresh && Number.isFinite(pos?.[i]) ? pos[i] : NaN;
        const speed = fresh && Number.isFinite(vel?.[i]) ? vel[i] : NaN;
        return {name,unit,now,goal:q,delta:q-now,speed,positionTolerance,velocityTolerance};
    });
    const maxError = Math.max(...rows.map(x => Math.abs(x.delta)));
    const maxSpeed = Math.max(...rows.map(x => Math.abs(x.speed)));
    const maxErrorRatio = Math.max(...rows.map(x => x.positionTolerance > 0 ? Math.abs(x.delta)/x.positionTolerance : Infinity));
    const maxSpeedRatio = Math.max(...rows.map(x => x.velocityTolerance > 0 ? Math.abs(x.speed)/x.velocityTolerance : Infinity));
    return {
        valid,aligned:valid && maxErrorRatio <= 1 && maxSpeedRatio <= 1 && !status.alignment_active,
        maxError,maxSpeed,maxErrorRatio,maxSpeedRatio,rows
    };
}
function calibrationAlignmentRowsHtml(alignment) {
    return alignment.rows.map(row => `<tr><td>${esc(row.name)}</td><td>${jointDisplayValue(row.now,row.name,'position',3,row.unit)}</td><td>${jointDisplayValue(row.goal,row.name,'position',3,row.unit)}</td><td>${jointDisplayValue(row.delta,row.name,'position',3,row.unit)}</td><td>${jointDisplayValue(row.positionTolerance,row.name,'position',3,row.unit)}</td><td>${jointDisplayValue(row.speed,row.name,'velocity',3,row.unit)} / ${jointDisplayValue(row.velocityTolerance,row.name,'velocity',3,row.unit)}</td></tr>`).join('');
}
function calibrationAlignmentSummary(alignment, manual) {
    return calibrationAlignmentStatus(alignment,manual) + (alignment.valid ? ` · ${busyText('最大位置容差占用','Largest position tolerance use')} ${(alignment.maxErrorRatio*100).toFixed(0)}% · ${busyText('最大速度容差占用','Largest velocity tolerance use')} ${(alignment.maxSpeedRatio*100).toFixed(0)}%` : '');
}
function calibrationAlignmentStatus(align, manual) {
    if (!align.valid) return state.language === 'en' ? 'Waiting for fresh robot feedback (max 500 ms)' : '等待新鲜关节反馈（最大 500 ms）';
    if (manual) return state.language === 'en' ? 'Guided drag active; support arm and move slowly' : '手动拖拽引导中：请支撑机械臂缓慢调整';
    if (align.aligned) return state.language === 'en' ? 'Ready for separate replay confirmation' : '姿态已对齐，可以再次确认自动回放';
    return state.language === 'en' ? 'Pose not aligned; replay blocked' : '姿态尚未对齐，暂不允许自动回放';
}
function calibrationAlignmentPanelHtml(status = state.telemetry?.model_calibration) {
    if (!workbenchConnected() || state.modelCalibrationOffline || !status?.source_task_id || status.phase !== 'waiting_replay_confirmation') return '';
    const en = state.language === 'en', alignment = calibrationAlignment(status);
    const action = status.alignment_active
        ? `<button id="model-calibration-alignment-finish" class="button">${en ? 'Finish alignment and hold' : '结束引导并保持当前位置'}</button>`
        : `<button id="model-calibration-alignment-begin" class="button" ${state.telemetry?.robot_state === 'ACTIVE' ? '' : 'disabled'}>${en ? 'Guided manual alignment' : '引导对齐回放起点'}</button>`;
    return `<section class="workspace-section calibration-alignment-panel"><div class="workspace-section-head"><h3>${en ? 'Align to the replay start' : '回放起点对齐'}</h3>${action}</div><p id="model-calibration-align-status" class="hint">${esc(calibrationAlignmentSummary(alignment,status.alignment_active))}</p><div class="table-scroll"><table class="workspace-table"><thead><tr><th>${en ? 'Joint' : '关节'}</th><th>${en ? 'Current' : '当前'}</th><th>${en ? 'Target' : '目标'}</th><th>${en ? 'Difference' : '差值'}</th><th>${en ? 'Position tolerance' : '位置容差'}</th><th>${en ? 'Speed / limit' : '速度 / 门槛'}</th></tr></thead><tbody id="model-calibration-align-rows">${calibrationAlignmentRowsHtml(alignment)}</tbody></table></div><p class="hint">${en ? 'The teal overlay marks the required start pose. Every joint must meet its own position and speed limits with feedback no older than 500 ms; finish guided alignment before confirming playback' : '青绿色虚影为目标起点；每个关节均需满足位置和速度门槛，反馈不超过 500 ms，结束引导后再确认回放'}</p></section>`;
}
function modelCalibrationPanelHtml() {
    const status = currentModelCalibration() || { phase: 'idle', progress: 0, recorder: {} }; const live = !state.modelCalibrationOffline;
    const phase = status.phase || 'idle'; const result = status.gravity_result || null; const taskActive = modelCalibrationActive(status); const robotActive = state.telemetry?.robot_state === 'ACTIVE'; const robotInactive = state.telemetry?.robot_state === 'INACTIVE';
    const training = (status.pose_targets || []).filter(x => !x.validation).length; const validation = (status.pose_targets || []).filter(x => x.validation).length;
    const directory = status.directory || state.modelCalibrationDirectory || state.modelCalibrationOffline?.directory || '';
    const optionsDisabled = live && !['idle', 'complete', 'cancelled', 'failed'].includes(phase);
    const options = state.modelCalibrationOptions;
    const alignment = calibrationAlignment(status);
    const sourceTag = state.modelCalibrationOffline ? `<span class="result-badge neutral">${t('sourceOffline')}</span>` : `<span class="result-badge ${workbenchConnected() ? 'pass' : 'neutral'}">${t(workbenchConnected() ? 'sourceLive' : 'sourceWaiting')}</span>`;
    const actions = [];
    if (!live) actions.push(`<button id="model-calibration-back-live" class="button">${state.language === 'en' ? 'Back to live task' : '返回实时任务'}</button>`);
    const activeTitle = robotActive ? '' : (state.language === 'en' ? 'Activate the robot before starting the demonstration' : '请先使能机械臂再开始示教');
    if (live && ['idle', 'complete', 'cancelled', 'failed'].includes(phase)) actions.push(`<button id="model-calibration-teach-start" class="button primary" ${robotActive ? '' : 'disabled'} title="${activeTitle}">${t('modelCalibrationStart')}</button>`);
    if (live && phase === 'teaching') actions.push(`<button id="model-calibration-teach-stop" class="button primary">${t('modelCalibrationStopTeach')}</button>`);
    if (live && phase === 'waiting_replay_confirmation') {
        actions.push(`<button id="model-calibration-replay-confirm" class="button danger" ${status.source_task_id && !alignment.aligned ? 'disabled' : ''}>${t('modelCalibrationConfirmReplay')}</button>`);
    }
    if (live && status.gain_protocol !== 'taught_trajectory' && ['gain_adaptation', 'static_reverse', 'static_forward', 'gravity_fitting', 'friction_reverse_slow', 'friction_forward_slow', 'friction_reverse_fast', 'friction_forward_fast', 'friction_fitting'].includes(phase)) actions.push(`<button id="model-calibration-pause" class="button">${t('pauseTask')}</button>`);
    if (live && phase === 'paused') actions.push(`<button id="model-calibration-resume" class="button primary">${t('resumeTask')}</button>`);
    if (live && (taskActive || phase === 'waiting_replay_confirmation')) actions.push(`<button id="model-calibration-cancel" class="button danger">${t('cancelTask')}</button>`);
    if (!live && state.modelCalibrationOffline?.can_resume) {
        const readyToImport = workbenchConnected() && robotActive && ['idle', 'complete', 'cancelled', 'failed'].includes(state.telemetry?.model_calibration?.phase || 'idle');
        actions.push(`<button id="model-calibration-import-trajectory" class="button primary" ${readyToImport ? '' : 'disabled'} title="${state.language === 'en' ? 'Requires an active robot, idle calibration, and matching configuration' : '需要机械臂 ACTIVE、当前无标定任务、且配置匹配'}">${state.language === 'en' ? 'Reuse trajectory (no movement)' : '恢复轨迹到当前会话（不运动）'}</button>`);
    }
    if (live && result?.static_pass) actions.push(`<button id="model-calibration-apply" class="button primary" ${robotInactive && !taskActive ? '' : 'disabled'}>${t('applyCandidate')}</button>`);
    if (live && status.candidate_applied) actions.push(`<button id="model-calibration-restore" class="button" ${robotInactive && !taskActive ? '' : 'disabled'}>${t('restoreCandidate')}</button>`);
    const singlePass = Number(status.single_pass_duration_s || 0);
    const gainPreview = state.calibrationEntry === 'gain';
    const estimate = gainPreview && singlePass > 0
        ? gainCalibrationSeconds(singlePass, state.gainMode, status)
        : Number(status.estimated_duration_s || 0);
    const groups = Array.isArray(status.pose_targets) ? status.pose_targets.length : 0;
    const teaching = Number(status.teaching_wall_duration_s || 0);
    const totalEstimate = gainPreview ? teaching + estimate : Number(status.estimated_total_duration_s || teaching + estimate);
    const recorded = Number(status.recorded_duration_s || 0);
    const planningReady = live && phase === 'waiting_replay_confirmation' && status.planner_id === 'local_hermite' && status.calibration_strategy === 'bidirectional_two_speed_v1' && (gainPreview || Number.isFinite(estimate) && estimate > 0) && singlePass > 0;
    const replayHint = planningReady ? `<div class="callout"><b>${state.language === 'en' ? 'Online capture time estimate' : '已选示教轨迹 · 在线采集耗时预估'}</b>
      <div>${state.language === 'en' ? 'Demonstration' : '自由示教'} ${teaching.toFixed(1)} s · ${state.language === 'en' ? 'Recorded track' : '原始轨迹'} ${recorded.toFixed(1)} s · ${state.language === 'en' ? 'Retimed one-way pass' : '优化后单程'} ${singlePass.toFixed(1)} s</div>
      <div>${state.language === 'en' ? 'Parking now cancels the pending automatic replay; saved task records remain on disk.' : '此时选择停放会结束待确认的自动回放，已记录的任务数据仍保存在磁盘中'}</div>
      <div>${state.language === 'en' ? 'Retimed waypoints / original samples' : '重定时关键点 / 原始采样'} ${Number(status.geometric_waypoints || 0)} / ${Number(status.original_samples || 0)}</div>
      <div>${gainPreview ? (state.language === 'en' ? 'Actuator gain calibration measures all joints along the demonstrated path, including candidate adjustment and independent validation within a budget of 12 times the original recorded trajectory duration. The confirmation shows the planned passes and duration' : '执行器增益标定沿示教路径联合测量所有关节，候选调整与独立验收共用原始轨迹时长 12 倍的预算；开始确认时显示计划趟数与预计用时') : `${state.language === 'en' ? 'Gravity correction: reverse + forward measurement passes and static holds' : '重力校正：正反向正式采集及静态停留'} ≈ <b>${totalEstimate.toFixed(1)} s</b> (${state.language === 'en' ? 'automatic stage' : '自动阶段'} ${estimate.toFixed(1)} s)`}</div>
      <div>${gainPreview ? (state.language === 'en' ? 'At the deadline, further motion stops in hold; recordings and qualified uses remain available' : '到达时限后结束后续运动并进入保持，保留实测记录和已验收用途') : (state.language === 'en' ? 'Actual settling and fitting may take longer; offline full inertial fitting is not included.' : '实际停稳与拟合可能更久，且不包含后续离线惯量计算')} ${t('releaseSafetyHint')}</div></div>` : (phase === 'teaching' ? `<div class="callout">${state.language === 'en' ? 'Teaching is not time-limited.' : '示教时间不设上限，结束示教后根据轨迹运动量和关节安全限制计算自动回放时间'}</div>` : (phase === 'waiting_replay_confirmation' ? `<div class="callout error-callout">${state.language === 'en' ? 'Native Core is outdated; reinstall before automatic playback.' : '正在运行的 C++ Core 未加载新的轨迹规划器，请重新安装编译后再自动回放'}</div>` : ''));
    const resumeSafetyHint = live && status.source_task_id ? `<div class="callout"><b>${state.language === 'en' ? 'Imported demonstration' : '已恢复历史轨迹'}：${esc(status.source_task_id)}</b>
      <div>${status.original_start_selected ? (state.language === 'en' ? 'Current pose was closer to original FIRST pose: imported path was reversed. The same recorded path is replayed, without an extra move.' : '当前位置更接近原示教起点：已自动选择从这一端开始，沿原示教路径采集，无须额外归位') : (state.language === 'en' ? 'Current pose was closer to original LAST pose: standard reverse/forward collection.' : '当前位置更接近原示教终点：采用原反向／正向采集流程')}</div>
      <div>${state.language === 'en' ? 'Return to the starting pose by manual dragging. Support the arm and check the workspace; standalone Core has no environment collision planner' : '此处通过人工拖拽返回起点，请支撑机械臂并检查周围空间；Standalone Core 未提供环境碰撞规划'}</div></div>` : '';
    const recordInventory = state.modelCalibrationRecords.length ? `<div class="callout"><b>${state.language === 'en' ? 'Saved task inventory' : '已保存任务记录'}</b>${state.modelCalibrationRecords.map((r, i) => {
        const fields = r.files || {};
        const items = ['frames.csv', 'trajectory.csv', 'trajectory.checkpoint.csv', 'result.json'];
        return `<div class="task-directory"><button id="model-calibration-record-${i}" class="button" title="${esc(r.directory || '')}">${esc(r.task_id || '任务')}</button> ${items.map(name => `<span class="result-badge ${fields[name] ? 'pass' : 'neutral'}">${esc(name)} ${fields[name] ? '✓' : '—'}</span>`).join(' ')}<div>${esc(r.note || '')}</div></div>`;
    }).join('')}</div>` : '';
    const offlineRecord = state.modelCalibrationOffline;
    const offlineHint = offlineRecord ? `<div class="callout"><b>${esc(state.language === 'en' ? ({ teaching_only: 'Recorded demonstration', interrupted: 'Interrupted task', completed: 'Completed calibration' }[offlineRecord.record_kind] || 'Task record') : ({ teaching_only: '仅有示教轨迹', interrupted: '标定中断', completed: '标定已完成' }[offlineRecord.record_kind] || '任务记录'))}</b><div>${esc(offlineRecord.note || '')}</div><div>${state.language === 'en' ? 'Saved trajectory' : '保存的轨迹'}：${Number(offlineRecord.trajectory?.samples || 0)} ${state.language === 'en' ? 'samples' : '帧'} · ${Number(offlineRecord.trajectory?.duration_s || 0).toFixed(1)} s</div><div>${esc(Object.entries(offlineRecord.files || {}).map(([name, exists]) => `${name}: ${exists ? '✓' : '—'}`).join(' · '))}</div><div>${offlineRecord.trajectory?.source === 'trajectory.checkpoint.csv' ? (state.language === 'en' ? 'Partial teaching checkpoint · must verify before replay' : '中断示教检查点 · 回放前必须重新校验') : ''}</div><div>${state.language === 'en' ? 'Importing the saved path creates a new task. Confirm replay separately; acquisition restarts from the beginning' : '导入保存路径后创建新任务，自动采集从头执行，并需再次确认回放'}</div></div>` : '';
    const resultBasis = state.modelCalibrationCandidateSource === 'offline'
        ? (['static_centered_svd_ridge_pose_cv', 'static_raw_svd_ridge_pose_cv'].includes(result?.solver)
            ? (state.language === 'en' ? 'Offline static mass and first moments · trusted links fixed throughout fitting · not applied automatically' : '离线静态质量与一阶矩求解 · 可信连杆在整个求解过程中固定 · 不自动应用')
            : (state.language === 'en' ? 'Historical offline candidate · inspect the recorded solver and scope before reuse' : '历史离线候选 · 重用前核对记录的求解方法与辨识范围'))
        : (state.language === 'en' ? 'Online quick preview; fixed mass; final analysis uses the saved bidirectional observations' : '在线快速预览 · 固定质量；最终模型以保存后的双向数据分析为准');
    const resultBlock = result?.first_moments?.length && ['complete', 'failed'].includes(phase) ? `<div class="callout"><b>${state.language === 'en' ? 'Candidate meaning' : '当前候选含义'}</b><div>${resultBasis}</div>${result.prior_preferred ? `<div class="idim-recommendation">${state.language==='en'?'Source model retained; no new mass or COM update':'保留原模型：没有新的质量或质心更新；偏置变化不等于重力补偿改善'}</div>` : ''}<div>${state.language === 'en' ? 'Validation covers the saved holdout poses; test the candidate on hardware before adopting it' : '验证结果覆盖本次留出姿态，采用候选前还需进行实机对比'}</div></div><div class="candidate-compare-head"><h4>${t('candidateComparison')}</h4><span>${t('candidateScope')}</span></div>${(!result.static_pass || result.review_warnings?.length) ? `<div class="callout"><b>${state.language === 'en' ? 'Needs human review' : '需要人工审核'}</b> · ${esc(result.failure_reason || 'holdout result not approved')} · ${state.language === 'en' ? 'Candidate may be exported for manual review if values are finite and physically valid, but automatic application requires passing validation' : '未通过自动验收的候选仍可导出供人工审核，但不会被允许临时应用到真机'}</div>` : ''}${result.identification_mode === 'local' ? `<div class="callout">${state.language === 'en' ? 'Local fit' : '局部辨识'} · ${state.language === 'en' ? 'Locked' : '锁定'} ${esc((result.locked_links || []).join(', ') || '—')} · ${state.language === 'en' ? 'Fitted' : '允许校正'} ${esc((result.fitted_links || []).join(', ') || '—')}</div>` : ''}${status.friction_status === 'requires_refit_or_review' ? `<div class="callout">${state.language === 'en' ? 'Friction result belongs to a different gravity candidate; do not reuse it' : '摩擦参数属于原来的重力候选，当前离线重算结果尚未重新匹配摩擦模型，不可直接复用'}</div>` : ''}${gravityValidationSummary(result)}${modelCalibrationMetricTable(status)}${modelCalibrationCandidateTable(status)}<div class="workspace-actions">${result.static_pass ? `<button id="model-calibration-preview-save" class="button">${t('previewSave')}</button>${state.modelCalibrationSavePreview ? `<button id="model-calibration-save" class="button primary">${t('saveCandidate')}</button>` : ''}` : ''}<span class="gravity-export-hint">${state.modelCalibrationCandidateSource === 'offline' ? (state.language === 'en' ? 'Offline candidate mass and COM, computed from saved data; source inertia-at-COM remains unchanged; review holdout separately' : '离线重算的质量与质心候选，质量和质心可同时变化；惯性张量保留原值，须独立评估留出表现') : (state.language === 'en' ? 'Original masses and inertia tensors are retained; only fitted COM values are updated' : '保留原始质量和惯性张量，仅更新辨识质心')}</span>${state.modelCalibrationSaved ? `<button id="model-calibration-restore-config" class="button">${t('restoreSavedConfig')}</button>` : ''}</div>${gravityValidationNoticeHtml(state.modelCalibrationSaved)}${state.modelCalibrationSavePreview ? `${gravityValidationNoticeHtml(state.modelCalibrationSavePreview,true)}<div class="diff-box"><b>${esc(state.modelCalibrationSavePreview.path)}</b>${(state.modelCalibrationSavePreview.changes || []).map(x => `<div><del>${esc(x.before)}</del><ins>${esc(x.after)}</ins></div>`).join('') || `<p>${t('noChanges')}</p>`}</div>` : ''}${state.modelCalibrationFrictionExport ? `<div class="callout"><b>${state.language === 'en' ? 'Friction candidate · review only' : '摩擦候选 · 仅供审核'}</b><div class="path-chip">${esc(state.modelCalibrationFrictionExport.path || '')}</div><div>${state.modelCalibrationFrictionExport.matches_current_candidate ? (state.language === 'en' ? 'Gravity source matches' : '重力来源一致') : (state.language === 'en' ? 'Gravity model has changed; friction fit needs revalidation' : '重力候选已变化，原摩擦结果需要重新验证')}</div></div>` : ''}${state.modelCalibrationExport ? `<div class="callout"><b>${t('exportCandidate')}</b><div class="path-chip">${esc(state.modelCalibrationExport.path || state.modelCalibrationExport.candidate_urdf || '')}</div>${state.modelCalibrationExport.verification_warning ? `<div class="callout">${esc(state.modelCalibrationExport.verification_warning)}</div>` : ''}${state.modelCalibrationExport.verification ? `<div>gravity RMS ${Number(state.modelCalibrationExport.verification.gravity_rms_nm || 0).toExponential(3)} Nm · FK Δp ${Number(state.modelCalibrationExport.verification.fk_max_position_error_m || 0).toExponential(3)} m</div>` : ''}</div>` : ''}` : '';
    return `<section class="workspace-section model-calibration-card"><div class="workspace-section-head"><div><h3>${t('modelCalibration')}</h3><p class="hint">${t('modelCalibrationDesc')}</p></div><div class="result-badges">${sourceTag}<span id="model-calibration-phase" class="result-badge neutral">${esc(modelCalibrationPhaseText(phase))}</span></div></div>${calibrationTaskGuidance('gravity')}<label>${state.language === 'en' ? 'Tool, payload and gripper state' : '末端配置、载荷与夹爪状态'}<input id="model-cal-load" type="text" value="${esc(options.load_description || '')}" placeholder="${state.language === 'en' ? 'Record the actual mounting and gripper state before acquisition' : '采集前填写，例如：剪夹+相机，夹爪闭合，无额外载荷'}" ${optionsDisabled ? 'disabled' : ''}></label><div class="model-calibration-identification"><b>${state.language === 'en' ? 'Gravity identification' : '重力模型辨识范围'}</b><div class="mode-buttons"><button id="model-cal-mode-global" type="button" class="button ${options.mode !== 'local' ? 'active' : ''}" ${optionsDisabled ? 'disabled' : ''}>${state.language === 'en' ? 'Global' : '全局辨识'}</button><button id="model-cal-mode-local" type="button" class="button ${options.mode === 'local' ? 'active' : ''}" ${optionsDisabled ? 'disabled' : ''}>${state.language === 'en' ? 'Local' : '局部辨识'}</button>${options.mode === 'local' ? `<button id="model-cal-lock-select" type="button" class="button" ${optionsDisabled ? 'disabled' : ''}>${state.language === 'en' ? 'Trusted links' : '选择可信 Link'}</button><span>${state.language === 'en' ? 'Locked' : '已锁定'} ${(options.locked_links || []).length}</span>` : ''}</div><p class="hint">${options.mode === 'local' ? (state.language === 'en' ? 'Trusted links stay fixed; online estimates first moments, offline fits mass and first moments from static poses' : '可信 Link 始终固定；在线仅修正一阶质量矩，离线使用静态姿态估计质量与一阶质量矩') : (state.language === 'en' ? 'Global selects all controlled links; online fixes mass, offline fits mass + first moments' : '全局选择全部受控 Link；在线固定质量，离线联合估计质量与一阶质量矩')}${!calibrationSelectableLinks().length ? (state.language === 'en' ? ' · No link list yet: load the model or a calibration record' : ' · 尚未取得 Link 清单，请加载模型或历史任务') : ''}</p></div><div class="model-calibration-options"><label>${t('poseBudget')}<input id="model-cal-pose-budget" type="number" min="5" max="16" step="1" value="${Number(options.pose_budget)}" ${optionsDisabled ? 'disabled' : ''}></label><label>${t('validationFraction')}<input id="model-cal-validation" type="number" min="0.15" max="0.45" step="0.05" value="${Number(options.validation_fraction)}" ${optionsDisabled ? 'disabled' : ''}></label><label>${t('regularizationStrength')}<input id="model-cal-regularization" type="number" min="0.00000001" max="10" step="0.001" value="${Number(options.regularization)}" ${optionsDisabled ? 'disabled' : ''}></label><label>${t('svdRelativeThreshold')}<input id="model-cal-svd" type="number" min="0.00000001" max="0.2" step="0.0001" value="${Number(options.svd_relative_threshold)}" ${optionsDisabled ? 'disabled' : ''}></label><label>${t('minimumInformationScore')}<input id="model-cal-information" type="number" min="0.00000001" max="1000" step="0.0001" value="${Number(options.minimum_information_score)}" ${optionsDisabled ? 'disabled' : ''}></label></div><div class="model-calibration-options"><label>${state.language === 'en' ? 'Next offline fitting objective' : '下一次离线拟合目标'}<select id="model-calibration-objective" ${taskActive ? 'disabled' : ''}><option value="urdf" ${state.modelCalibrationObjective === 'urdf' ? 'selected' : ''}>${state.language === 'en' ? 'Raw URDF prediction (experimental)' : 'URDF 直接预测（实验对照）'}</option><option value="bias_separated" ${state.modelCalibrationObjective === 'bias_separated' ? 'selected' : ''}>${state.language === 'en' ? 'Separate constant feedback offsets (compatibility default)' : '分离反馈常值偏置（兼容默认）'}</option></select></label></div><p class="hint">${state.language === 'en' ? 'Used for the next recompute. Raw prediction may absorb feedback errors into mass/COM; both objectives use raw URDF holdout evaluation before hardware comparison' : '仅用于下一次重算，加载报告不改变此选择；直接预测是实验对照，可能把反馈误差吸收入质量/质心；两种目标均不保证实机改善，仍按原始 URDF 留出预测验收'}</p><div class="workspace-actions">${actions.join('')}<button id="model-calibration-load-record" class="button">${t('loadRecord')}</button><button id="model-calibration-scan-records" class="button">${state.language === 'en' ? 'Scan saved tasks' : '扫描已有任务'}</button>${state.modelCalibrationOffline ? `<button id="model-calibration-refresh-record" class="button">${state.language === 'en' ? 'Refresh record' : '重新检查记录'}</button>` : ''}${directory && (!state.modelCalibrationOffline || state.modelCalibrationOffline.has_result) ? `<button id="model-calibration-recompute" class="button">${t('recomputeRecord')}</button>` : ''}</div><div class="model-calibration-summary"><div><span>${t('taskIdentifier')}</span><b id="model-calibration-task-id-inline">${esc(status.task_id || '—')}</b></div><div><span>${t('progressLabel')}</span><b id="model-calibration-progress-inline">${Math.round(Number(status.progress || 0) * 100)}%</b></div><div><span>${t('trajectorySamples')}</span><b id="model-calibration-trajectory-inline">${esc(status.trajectory_samples ?? '—')}</b></div><div><span>${t('trainingValidationGroups')}</span><b id="model-calibration-groups-inline">${training} / ${validation}</b></div><div><span>${t('validStaticSamples')}</span><b id="model-calibration-static-inline">${esc(status.valid_static_samples ?? '—')}</b></div><div><span>${t('acceptedDroppedFrames')}</span><b id="model-calibration-recorder-inline">${esc(status.recorder?.accepted ?? '—')} / ${esc(status.recorder?.dropped ?? '—')}</b></div></div>${status.static_pair_quality?.position_warnings ? `<div class="callout">双向静态姿态重复性：${Number(status.static_pair_quality.position_warnings)} 组位置差超过 0.01 rad；观测均已保留，离线按实际姿态与迟滞不确定性分析，不视为精确摩擦抵消</div>` : ''}${status.preview_error ? `<div class="callout">${state.language === 'en' ? 'Static quick preview unavailable; complete observations are retained for offline IDIM' : '静态快速预览未得到候选；完整采集继续保留，后续使用离线 IDIM 验证'}: ${esc(status.preview_error)}</div>` : ''}${status.error ? `<div class="callout error-callout">${esc(localizedError(status.error))}</div>` : ''}${!modelCalibrationModelBinding().comparable ? `<div class="callout error-callout">${esc(modelCalibrationModelBinding().reason)}</div>` : ''}${modelCalibrationModelBinding().historical ? `<div class="callout"><b>${state.language === 'en' ? 'Historical model preview · read only' : '历史模型对照 · 仅预览'}</b><div>${state.language === 'en' ? 'Only inertial parameters differ, geometry and joints match, this does not validate the updated dynamics or authorize replay or application' : '已核对只有惯量参数改变，几何与关节保持一致，历史 RMS 不代表当前模型的预测精度，此预览不授权回放或参数应用'}</div><div>${state.language === 'en' ? 'Changed links' : '变化的 Link'} ${esc(modelCalibrationModelBinding().alignment.changed_links.join(', '))}</div>${(modelCalibrationModelBinding().alignment.warnings || []).map(w => `<div class="workspace-error">${esc(w)}</div>`).join('')}</div>` : ''}${replayHint}${resumeSafetyHint}${offlineHint}${recordInventory}${resultBlock}${directory ? `<div class="task-directory"><span>${t('taskDirectory')}</span><code>${esc(directory)}</code></div>` : ''}</section>`;
}
function singleCalibrationToolsHtml() {
    const c = state.telemetry?.calibration || { kind: 'none', phase: 'idle', captured: 0, expected: 0 }; const progress = c.expected ? `${c.captured}/${c.expected}` : '';
    return `<details class="workspace-section single-calibration-tools"><summary>${t('singleCalibrationTools')}</summary><div class="single-calibration-tools-body"><section><h4>${t('staticCalibration')}</h4><p>${t('staticCalibrationDesc')}</p><div class="workspace-actions"><button data-calibration-begin="static" class="button">${t('start')}</button><button id="calibration-capture" class="button">${t('capturePose')}</button><button id="calibration-finish" class="button">${t('finishFit')}</button></div></section><section><h4>${t('staticValidation')}</h4><button data-calibration-begin="validation" class="button">${t('startValidation')}</button></section><section><h4>${t('frictionCalibration')}</h4><div class="workspace-actions"><button id="friction-record-start" class="button">${t('startRecording')}</button><button id="friction-record-stop" class="button">${t('stopRecording')}</button><button id="friction-replay" class="button danger" ${frictionReplayProblem() ? 'disabled' : ''} title="${esc(frictionReplayProblem())}">${t('startReplay')}</button></div></section><dl class="kv compact"><dt>${t('taskType')}</dt><dd id="cal-task-kind">${esc(calibrationTaskKindText(c.kind))}</dd><dt>${t('taskPhase')}</dt><dd id="cal-task-phase">${esc(calibrationTaskPhaseText(c.phase))}</dd><dt>${t('progressLabel')}</dt><dd id="cal-task-progress">${esc(progress || '—')}</dd><dt>${t('trajectorySamples')}</dt><dd id="cal-task-trajectory">${esc(c.trajectory_samples ?? '—')}</dd><dt>${t('errorLabel')}</dt><dd id="cal-task-error" class="workspace-error">${esc(c.error ? localizedError(c.error) : '—')}</dd></dl><div class="workspace-actions"><button id="calibration-cancel" class="button danger">${t('cancelTask')}</button><button id="export-session" class="button">${t('exportData')}</button></div></div></details>`;
}
function idimResultHtml(report) {
    const en = state.language === 'en';
    if (!report) return `<div class="calibration-empty-guidance" role="status">${en ? 'No identification result yet. Choose a saved task, review the settings and run identification to compare the source and candidate models' : '尚无辨识结果，请先选择采集任务并确认设置，计算后在此查看原模型、候选与推荐结论'}</div>`;
    const metric = x => x != null && Number.isFinite(Number(x)) ? Number(x).toFixed(5) : '—';
    const m = report.validation_motion, g = report.gravity_validation || {};
    const names = report.physical_review || [];
    const original = report.original_dynamic_parameters || [], selected = report.selected_dynamic_parameters || [];
    const com = p => p[0] ? p.slice(1, 4).map(x => x / p[0]) : [NaN, NaN, NaN];
    const tensor = p => { const c = com(p), norm = c.reduce((a,x) => a+x*x,0); return [p[4]-p[0]*(norm-c[0]*c[0]),p[5]+p[0]*c[0]*c[1],p[6]-p[0]*(norm-c[1]*c[1]),p[7]+p[0]*c[0]*c[2],p[8]+p[0]*c[1]*c[2],p[9]-p[0]*(norm-c[2]*c[2])]; };
    const rows = names.map((link,i) => {
        const a = original.slice(i*10,i*10+10), b = selected.slice(i*10,i*10+10); if (a.length !== 10 || b.length !== 10) return '';
        const ca = com(a), cb = com(b), ia = tensor(a), ib = tensor(b);
        const dc = Math.sqrt(ca.reduce((sum,x,k) => sum+(cb[k]-x)**2,0))*1000;
        const di = Math.sqrt(ia.reduce((sum,x,k) => sum+(ib[k]-x)**2*([1,3,4].includes(k)?2:1),0));
        return `<tr><th>${esc(link.link)}${(report.locked_links || []).includes(link.link)?' · '+(en?'fixed':'锁定'):''}</th><td>${metric(a[0])} → ${metric(b[0])}</td><td>${ca.map(x=>metric(x*1000)).join(', ')}<br>→ ${cb.map(x=>metric(x*1000)).join(', ')}</td><td>${dc.toFixed(3)}</td><td>${di.toExponential(3)}</td><td>${link.physical?'✓':'✗'}</td></tr>`;
    }).join('');
    const recommendation = report.recommendation === 'keep_source_prior' || report.status === 'prior_retained'
        ? (en ? 'Keep the source model; fitted alternative is for review' : '推荐保留原模型；拟合候选仅供审核对照')
        : report.exportable ? (en ? 'Offline checks passed; hardware validation pending' : '离线检查通过；尚未完成真机验收') : (en ? 'Review candidate; checks remain incomplete' : '待审核候选；尚有检查未完成');
    const inputLabels = {
        training_measurement: en ? 'Training feedback' : '训练反馈', training_phases: en ? 'Training phases' : '训练六阶段',
        load_description: en ? 'Mounting record' : '装机说明', unmeasured_joint_positions: en ? 'Unmeasured joint positions' : '未采样关节位置',
        independent_session: en ? 'Independent session' : '独立会话', validation_measurement: en ? 'Validation feedback' : '验证反馈',
        validation_phases: en ? 'Validation phases' : '验证六阶段', validation_load_description: en ? 'Validation mounting record' : '验证装机说明',
    };
    return `<div class="callout"><b>${esc(recommendation)}</b><div class="result-badges"><span class="result-badge neutral">${en?'Model changed':'参数更新'} ${report.candidate_changed?'✓':'—'}</span><span class="result-badge ${report.physical_pass?'pass':'warn'}">${en?'Physical consistency':'物理一致性'} ${report.physical_pass?'✓':'✗'}</span></div></div>
      <div class="calibration-capability-grid"><div><span>${en?'Motion prediction RMS':'运动预测 RMS'}</span><b>${metric(m?.baseline?.rms_nm)} → ${metric(m?.candidate?.rms_nm)} Nm</b><small>${esc(({same_session_disjoint_blocks:en?'Same-session block holdout; independent session still needed':'同会话按时间块留出；仍需独立会话',independent_session:en?'Independent recording session':'独立采集会话'})[report.validation_scope] || report.validation_scope || '')}</small></div><div><span>${en?'Gravity-only RMS':'纯重力项 RMS'}</span><b>${metric(g.baseline_rms_nm)} → ${metric(g.candidate_rms_nm)} Nm</b><small>${Number(g.poses || 0)} ${en?'pose groups; no fitted offset added':'组姿态；未叠加拟合偏置'}</small></div></div>
      <div class="result-badges">${Object.entries(report.input_checks || {}).map(([name,passed])=>`<span class="result-badge ${passed?'pass':'neutral'}">${esc(inputLabels[name]||name)} ${passed?'✓':en?'Pending':'待补'}</span>`).join('')}</div>
      ${(report.blocking_reasons || []).length?`<div class="callout"><b>${en?'Outstanding checks':'尚待完成的检查'}</b><ul class="idim-reasons">${report.blocking_reasons.map(x=>`<li>${esc(x)}</li>`).join('')}</ul></div>`:''}
      ${rows?`<details class="calibration-locks" open><summary>${en?'Full parameter comparison':'完整参数对比'}</summary><div class="table-scroll"><table class="workspace-table"><thead><tr><th>Link</th><th>${en?'Mass (kg)':'质量 kg'}</th><th>COM xyz · mm</th><th>|ΔCOM| mm</th><th>||ΔI_COM|| kg·m²</th><th>${en?'Physical':'物理'}</th></tr></thead><tbody>${rows}</tbody></table></div></details>`:''}
      <details class="calibration-locks"><summary>${en?'Solver and model selection':'求解与模型选择依据'}</summary><p>${esc(report.solver || '')} · ${esc(report.physical_completion?.solver_status || report.physical_completion?.status || '')}</p><p>${en?'Regularization':'正则系数'}: ${esc(report.physical_completion?.regularization ?? '—')} · ${en?'Training-only CV':'仅训练集交叉验证'}</p><div class="task-directory"><code>${esc(report.report_path || '')}</code></div></details>`;
}
function idimPanelHtml() {
    const en = state.language === 'en';
    const directory = state.modelCalibrationDirectory || currentModelCalibration()?.directory || '';
    const report = state.idimReport?.task_id === currentModelCalibration()?.task_id ? state.idimReport : null;
    const locked = new Set(state.idimLocked || []);
    const links = calibrationSelectableLinks();
    const retained = report?.recommendation === 'keep_source_prior' || report?.status === 'prior_retained';
    const resultLabel = !report ? (en ? 'Not computed' : '未计算') : retained ? (en ? 'Source retained' : '保留原模型')
        : report.exportable ? (en ? 'Offline checks passed' : '离线检查通过') : (en ? 'Needs review' : '待审核');
    const runHint = workbenchConnected()
        ? (en ? 'Safely park and disconnect the workspace before offline computation' : '计算前请安全停放并断开工作台，再执行离线辨识')
        : !directory ? (en ? 'Choose a saved acquisition task first' : '请先选择已保存的采集任务')
            : (en ? 'Uses saved data; computation does not move the robot or apply parameters' : '使用已保存的数据计算，不驱动机械臂或自动应用参数');
    return `<section class="workspace-section calibration-advanced-card" id="idim-workflow"><div class="workspace-section-head"><div><h3>${en?'Advanced dynamics identification':'高级动力学辨识'}</h3><p class="hint">${en?'Compare mass, COM and inertia candidates using saved observations' : '复用采集任务，离线对比质量、质心与惯量候选'}</p></div><span id="idim-result-status" class="result-badge ${report?.exportable && !retained?'pass':report&&!retained?'warn':'neutral'}">${resultLabel}</span></div>
      <section class="calibration-step"><h4>${en?'01 · Acquisition data':'01 · 采集数据'}</h4><div class="workspace-actions"><button class="button" id="idim-load">${en?'Choose task':'选择采集任务'}</button><button class="button" id="idim-audit" ${directory?'':'disabled'}>${en?'Audit observations':'检查数据'}</button></div>${directory ? `<div class="task-directory"><code>${esc(directory)}</code></div>` : `<div class="calibration-empty-guidance" role="status">${en?'No task selected. Load a saved acquisition task to inspect its data' : '尚未选择任务，请加载已保存的采集任务并检查数据'}</div>`}<div id="idim-audit-result">${idimAuditHtml()}</div></section>
      <section class="calibration-step"><h4>${en?'02 · Identification settings':'02 · 辨识设置'}</h4><div class="model-calibration-options"><label>${en?'Scope':'辨识范围'}<select id="idim-scope"><option value="global" ${state.idimScope==='global'?'selected':''}>${en?'Global':'全局辨识'}</option><option value="local" ${state.idimScope==='local'?'selected':''}>${en?'Local':'局部辨识'}</option></select></label><label>${en?'Independent validation task':'独立验证任务'}<input id="idim-validation" value="${esc(state.idimValidationDirectory || '')}" placeholder="${en?'Optional task folder':'可选，填写独立任务目录'}"></label></div><p class="hint">${en?'Without an independent task, validation holds out complete time blocks and pose groups from the same session and remains review only' : '未指定独立任务时，按同会话完整时间块与姿态组留出验证，结果仅供审核'}</p>
      <fieldset class="idim-locks"><legend>${en?'Trusted links · checked = fixed':'可信连杆 · 勾选代表锁定'}</legend>${links.map(l=>`<label class="calibration-advanced-link"><input type="checkbox" data-idim-locked="${esc(l.name)}" ${locked.has(l.name)?'checked':''} ${state.idimScope==='local'?'':'disabled'}><span>${esc(l.name)}</span></label>`).join('') || `<span class="hint">${en?'Load a model or task to select links':'加载模型或任务后可选择连杆'}</span>`}<small id="idim-lock-hint">${en?'Global fits all links; local preserves selected links throughout fitting':'全局同时拟合全部连杆；局部在整个求解过程中保持所选连杆不变'}</small></fieldset>
      <details class="calibration-locks idim-advanced"><summary>${en?'Advanced settings and experimental conditions':'高级设置与实验条件'}</summary><div class="model-calibration-options"><label>${en?'Estimator':'估计器'}<select id="idim-estimator">${['OLS','WLS','IRLS'].map(x=>`<option ${state.idimEstimator===x?'selected':''}>${x}</option>`).join('')}</select></label><label>${en?'Position bandwidth (Hz)':'位置滤波带宽 Hz'}<select id="idim-cutoff">${[2,5,10].map(x=>`<option ${Number(state.idimCutoff)===x?'selected':''}>${x}</option>`).join('')}</select></label></div><label>${en?'Known mounting / payload (operator declaration)':'补充已知装机与载荷（操作者声明，不改原记录）'}<input id="idim-load-description" value="${esc(state.idimLoadDescription || '')}" placeholder="${en?'Only declare known conditions held constant throughout':'已知且全程不变时填写；不能凭猜测补录'}"></label><label>${en?'Known fixed unmeasured joints (rad / m)':'未采样关节的已知固定位置 rad / m'}<input id="idim-fixed-joints" value="${esc(state.idimFixedJointText || '')}" placeholder="${en?'joint7=0, joint7-1=0; use actual positions':'joint7=0, joint7-1=0；按实际状态填写'}"></label><p class="hint">${en?'Fitting enforces physical consistency without empirical mass/COM limits. Undeclared positions remain reported assumptions':'拟合直接施加物理一致性约束，不设置质量、质心经验硬边界；未声明的位置以假设记录在报告中'}</p></details><div class="workspace-actions"><button class="button primary" id="idim-run" ${directory&&!workbenchConnected()?'':'disabled'} title="${esc(runHint)}">${en?'Identify and compare':'计算并对比'}</button><span id="idim-draft-state" class="hint"></span></div><p class="hint">${runHint}</p></section>
      <section class="calibration-step"><h4>${en?'03 · Results and recommendation':'03 · 结果与推荐结论'}</h4><label class="idim-preview-toggle"><input type="checkbox" id="idim-preview" ${state.idimPreview!==false?'checked':''}>${en?'Show candidate COM comparison · preview only':'显示候选质心对照 · 仅预览，不影响控制'}</label><div id="idim-results">${idimResultHtml(report)}</div></section>
      <section class="calibration-step"><h4>${en?'04 · Export results':'04 · 导出成果'}</h4><div id="idim-export-actions">${idimExportHtml(report)}</div><p class="hint">${en?'After computation, the diagnostic report can be exported for any result. Review URDF requires physical validity; checked model export additionally requires all checks. Export does not apply parameters':'计算后均可导出诊断报告；待审核 URDF 要求物理有效，通过离线检查的模型还需完成全部检查；导出不自动应用参数'}</p></section></section>`;
}
function idimAuditHtml() {
    const audit = state.idimAudit;
    if (!audit) return '';
    const en = state.language === 'en';
    return `<div class="result-badges"><span class="result-badge ${audit.measurement_ready?'pass':'neutral'}">${en?'Range and feedback':'量程与反馈'} ${audit.measurement_ready?'✓':en?'Pending':'待检查'}</span><span class="result-badge ${audit.acquisition_complete?'pass':'neutral'}">${en?'Acquisition phases':'六阶段'} ${audit.acquisition_complete?'✓':en?'Incomplete':'不齐全'}</span></div>${(audit.warnings || []).map(x=>`<p class="hint">${esc(x)}</p>`).join('')}`;
}
function idimExportHtml(report) {
    const en = state.language === 'en';
    return `<div class="workspace-actions"><button id="idim-report-export" class="button" ${report?'':'disabled'}>${en?'Export diagnostic report':'导出诊断报告'}</button><button id="idim-review-export" class="button" ${report?.review_exportable?'':'disabled'}>${en?'Export review URDF':'导出待审核 URDF'}</button><button id="idim-export" class="button primary" ${report?.exportable?'':'disabled'}>${en?'Export checked model':'导出通过离线检查的模型'}</button></div><div id="idim-export-result" class="hint"></div>`;
}
function idimDraftChanged() {
    const node = $('idim-draft-state'); if (node) node.textContent = state.language==='en'?'Settings changed; recompute to update results':'设置已修改；下面仍为上次计算结果，请重新计算';
}
function refreshIdimResults() {
    if ($('idim-results')) $('idim-results').innerHTML = idimResultHtml(state.idimReport);
    if ($('idim-export-actions')) { $('idim-export-actions').innerHTML = idimExportHtml(state.idimReport); bindIdimExports(); }
    if ($('idim-result-status')) {
        const en = state.language === 'en', report = state.idimReport;
        const retained = report?.recommendation === 'keep_source_prior' || report?.status === 'prior_retained';
        $('idim-result-status').textContent = !report ? (en ? 'Not computed' : '未计算') : retained ? (en ? 'Source retained' : '保留原模型') : report.exportable ? (en ? 'Offline checks passed' : '离线检查通过') : (en ? 'Needs review' : '待审核');
        $('idim-result-status').className = `result-badge ${report?.exportable && !retained ? 'pass' : report && !retained ? 'warn' : 'neutral'}`;
    }
    if ($('idim-draft-state')) $('idim-draft-state').textContent = '';
    updateModelCalibrationComparison();
}
function bindIdimExports() {
    for (const [id,flags] of [['idim-report-export',{report_only:true}],['idim-review-export',{review_only:true}],['idim-export',{}]]) {
        if ($(id)) $(id).onclick = () => artifactExport($(id),async()=>{
            const destination = await api.selectResources(); if (!destination) return;
            const result = await api.request('model_calibration_idim_export',{report_path:state.idimReport.report_path,destination:destination+'/idim-'+state.idimReport.run_id+'-'+id,...flags});
            if ($('idim-export-result')) $('idim-export-result').textContent = result.candidate_urdf || result.report || result.directory;
        });
    }
}
function advancedIdentificationPanelHtml() {
    const en = state.language === 'en';
    const dir = state.modelCalibrationDirectory || currentModelCalibration()?.directory || '';
    const status = currentModelCalibration() || {};
    const options = [
        ['mass', en ? 'Mass (COM prior)' : '质量辨识 · 固定 COM'],
        ['com', en ? 'COM (mass prior)' : '质心辨识 · 固定质量'],
        ['mass_com', en ? 'Joint mass and COM' : '质量与质心联合辨识'],
    ];
    const assessment = state.advancedAssessment?.directory === dir ? state.advancedAssessment : null;
    const result = state.advancedReport?.task_id === status.task_id ? state.advancedReport : null;
    const dynamic = ['inertia', 'full'].includes(state.advancedTarget);
    const missing = !dir || !state.modelCalibrationOffline?.metadata;
    const disable = missing || workbenchConnected() || !assessment || (dynamic && !assessment.dynamic_precheck) || (!dynamic && !assessment.gravity_ready) || (calibrationSelectableLinks().length > 0 && state.advancedLocked.length >= calibrationSelectableLinks().length);
    const limits = en ? 'Offline-only candidate, no motor action or automatic application. Rank, physical consistency and validation are checked separately' : '仅离线分析，不发送电机指令或自动应用参数，分别核验回归秩、物理一致性和留出误差';
    const available = calibrationSelectableLinks();
    const locks = new Set(state.advancedLocked);
    const lockList = available.map(link => `<label class="calibration-advanced-link"><input type="checkbox" data-advanced-locked="${esc(link.name)}" aria-label="${en ? 'Keep original parameters for ' : '保留原始参数 '}${esc(link.name)}" ${locks.has(link.name) ? 'checked' : ''}><span>${esc(link.name)}</span></label>`).join('');
    return `${idimPanelHtml()}<details class="workspace-section idim-supplement"><summary>${en ? 'Supplementary static analysis · separate result' : '专项静态参数分析 · 独立结果'}</summary><div class="calibration-advanced-card">
      <div class="workspace-section-head"><div><h3>${en ? 'Supplementary static parameter analysis' : '静态参数补充分析'}</h3><p class="hint">${en ? 'Continue from saved data, identify only missing parameters' : '复用现有数据 · 仅补充未完成的参数辨识'}</p></div><span class="result-badge neutral">${en ? 'Offline only' : '离线专用'}</span></div>
      ${calibrationTaskGuidance('dynamics')}
      <section class="calibration-step"><h4>${en ? '01 · Saved data' : '01 · 已有数据'}</h4>
      <div class="workspace-actions"><button id="advanced-temp-restore-unconditional" class="button" ${!state.calibrationRuntimeCandidates?.dynamics_applied ? 'hidden' : ''} ${state.calibrationRuntimeCandidates?.dynamics_applied && workbenchConnected() && state.telemetry?.robot_state === 'INACTIVE' ? '' : 'disabled'}>${en ? 'Undo temporary dynamics' : '撤销临时动力学模型'}</button><button id="advanced-load-record" class="button">${t('loadRecord')}</button><button id="advanced-assess-record" class="button" ${!missing ? '' : 'disabled'}>${en ? 'Assess saved data' : '分析已有数据'}</button><span class="hint">${esc(status.task_id || (en ? 'No record selected' : '尚未选择任务'))}</span></div>
      ${assessment ? `<div class="calibration-capability-grid"><div>${en ? 'Static pose groups' : '静态姿态组'}<b>${assessment.training_groups} / ${assessment.holdout_groups}</b><small>${en ? 'Train / holdout' : '训练 / 留出'}</small></div><div>${en ? 'Dynamic samples' : '有效动态帧'}<b>${assessment.valid_dynamic_samples}</b><small>${assessment.dynamic_precheck ? (en ? 'Precheck ready' : '初步具备动态数据') : (en ? 'Insufficient paired replay' : '双向激励可能不足')}</small></div></div>` : `<div class="calibration-empty-guidance" role="status">${en ? 'Load a completed task, then assess its static poses and dynamic observations' : '请先加载已完成任务，再分析静态姿态组和动态采样数据'}</div>`}</section>
      <section class="calibration-step"><h4>${en ? '02 · Target and settings' : '02 · 辨识目标与设置'}</h4><div class="model-calibration-options"><label>${en ? 'Identification target' : '辨识目标'}<select id="advanced-target">${options.map(([id, name]) => `<option value="${id}" ${state.advancedTarget === id ? 'selected' : ''}>${name}</option>`).join('')}</select></label></div>
      <div class="calibration-muted-note">${dynamic ? (en ? 'Requires bidirectional dynamic excitation, sufficient velocity and acceleration samples, and an independent holdout check' : '需要双向动态激励及足够的速度、加速度样本，辨识后还必须检查物理一致性与留出误差') : (en ? 'Uses gravity information from static pose groups; dynamic excitation is not required for this target' : '使用静态姿态组中的重力信息，不要求动态激励数据')}</div>
      <details class="calibration-locks"><summary>${en ? 'Lock trusted links' : '锁定可信 Link'} · ${state.advancedLocked.length}</summary><div class="calibration-advanced-links">${lockList || (en ? 'Load URDF / record first' : '请先加载 URDF 或标定任务')}</div></details>
      <p class="hint">${en ? (state.advancedLocked.length ? 'Locked links will keep their original inertial data' : 'No locked links: all sampled links are eligible for fitting') : (state.advancedLocked.length ? '已锁定 Link 保留原始惯性参数' : '未锁定 Link，当前将尝试全局辨识')}</p><div class="callout">${limits}</div>
      <div class="calibration-advanced-next">${result ? (en ? 'Candidate generated: review validation results, then export or reconnect to apply' : '候选已生成，请先审核验证指标并导出成果，如需临时应用再连接工作台') : missing ? (en ? 'Next: load a completed task' : '下一步：加载已完成任务') : !assessment ? (en ? 'Next: assess data' : '下一步：分析已有数据') : workbenchConnected() ? (en ? 'Next: safely disconnect to run offline' : '下一步：安全停放并断开工作台，再离线辨识') : disable ? (dynamic && !assessment.dynamic_precheck ? (en ? 'Dynamic data precheck did not pass. Select a static target or collect more data' : '动态数据预检查未通过，请改选静态目标或重新采集双向激励数据') : !dynamic && !assessment.gravity_ready ? (en ? 'Static data is insufficient' : '静态数据不足，需重新采集') : en ? 'Check trusted links and available targets' : '请检查可信 Link 选择和可辨识范围') : (en ? 'Ready: review target and start offline identification' : '数据具备初步条件，可确认目标并开始离线辨识')}</div><div class="workspace-actions"><button class="button primary" id="advanced-run" ${disable ? 'disabled' : ''}>${en ? 'Fit offline candidate for review' : '离线辨识并生成审核候选'}</button>${workbenchConnected() ? `<span class="hint">${en ? 'Park and disconnect the workspace first' : '请先安全停放并断开工作台，再执行耗时离线计算'}</span>` : missing ? `<span class="hint">${en ? 'Requires a saved completed task' : '需要加载具有合法来源信息的历史任务记录'}</span>` : ''}</div></section>
      <section class="calibration-step"><h4>${en ? '03 · Review and export' : '03 · 审核与导出'}</h4>
      ${result ? `<div class="callout"><b>${en ? 'Identification result' : '高级辨识结果'} · ${esc(result.identification_mode)}</b><div><span class="result-badge ${result.status === 'review_passed_offline' ? 'pass' : 'warn'}">${result.status === 'review_passed_offline' ? (en ? 'Passed offline review' : '离线审核通过') : (en ? 'Needs review' : '需要审核')}</span> ${en ? 'Rank' : '数值秩'} ${Number(result.regressor_rank)}/${Number(result.regressor_columns)}</div><div>${en ? 'Holdout RMS' : '留出 RMS'} ${Number(result.original_holdout_rms_nm).toFixed(4)} → ${Number(result.candidate_holdout_rms_nm).toFixed(4)} Nm</div><div class="path-chip">${esc(result.candidate_urdf || '')}</div>${(result.warnings || []).map(w => `<div>${esc(w)}</div>`).join('')}<div>${en ? 'Validation scope' : '评价范围'} ${esc(result.validation_scope === 'static_gravity' ? (en ? 'Static gravity only' : '仅静态重力') : (en ? 'Dynamic inertial model' : '动态惯量'))}</div><div>${en ? 'Physical inertia check' : '惯量物理一致性'} ${result.full_dynamics_physical_pass ? (en ? 'Passed' : '通过') : (en ? 'Failed; do not apply' : '未通过，不可应用')}</div><div>${en ? 'No automatic application or hardware validation' : '未自动应用，也不代表已完成真机验证'}</div><div class="workspace-actions"><button id="advanced-temp-apply" class="button" ${result.status === 'review_passed_offline' && result.validation_scope === 'dynamic_inertia' && result.full_dynamics_physical_pass && workbenchConnected() && state.telemetry?.robot_state === 'INACTIVE' ? '' : 'disabled'}>${en ? 'Temporarily apply candidate (INACTIVE)' : '临时应用惯量候选（需 INACTIVE）'}</button><button id="advanced-temp-restore" class="button" ${state.calibrationRuntimeCandidates?.dynamics_applied && workbenchConnected() && state.telemetry?.robot_state === 'INACTIVE' ? '' : 'disabled'}>${en ? 'Restore original model' : '撤销并恢复原模型'}</button></div></div>` : ''}
    ${result?.candidate_urdf ? `<div class="workspace-actions"><button class="button" data-tool-export="dynamics">${en?'Export supplementary static result':'导出专项静态结果（非完整 IDIM）'}</button></div>` : ''}${!result ? `<div class="calibration-empty-guidance" role="status">${en ? 'No supplementary result yet. Review appears after the offline fit' : '尚无专项静态结果，完成离线辨识后在此审核并导出'}</div>` : ''}</section></div></details>`;
}
async function assessLoadedCalibrationRecord(record) {
    // The CSV scan never competes with a running robot's control IPC
    if (!record?.directory || workbenchConnected()) return;
    try {
        const report = await api.request('model_calibration_assess', { directory: record.directory });
        if (state.modelCalibrationOffline?.directory !== record.directory) return;
        state.advancedAssessment = report;
        state.advancedProgress = '';
        if (state.page === 'calibration') render();
    } catch (error) {
        if (state.modelCalibrationOffline?.directory !== record.directory) return;
        state.advancedProgress = localizedError(String(error.message || error));
        if (state.page === 'calibration') render();
    }
}
function calibrationUnifiedResultsHtml() {
    const en = state.language === 'en';
    const record = currentModelCalibration() || {};
    const gravity = record.gravity_result || {};
    const assessment = state.advancedAssessment?.task_id === record.task_id ? state.advancedAssessment : null;
    const report = state.advancedReport?.task_id === record.task_id ? state.advancedReport : null;
    const stages = [
        [en ? 'Gravity model' : '重力模型', !gravity.first_moments?.length ? 'neutral' : gravity.static_pass ? 'pass' : 'warn', !gravity.first_moments?.length ? (en ? 'Not run' : '未执行') : gravity.static_pass ? (en ? 'Static validation passed' : '静态验证通过') : (en ? 'Needs review' : '需要审核')],
        [en ? 'Friction model' : '摩擦模型', !record.friction && !record.friction_pass ? 'neutral' : record.friction_status === 'requires_refit_or_review' ? 'warn' : record.friction_pass ? 'pass' : 'warn', record.friction_status === 'requires_refit_or_review' ? (en ? 'Gravity source changed' : '重力来源不匹配') : record.friction_pass ? (en ? 'Verified' : '已通过') : (en ? 'Not verified' : '尚未验证')],
        [en ? 'Dynamic inertias' : '动态惯量', report?.status === 'review_passed_offline' ? 'warn' : 'neutral', report ? (en ? 'Offline candidate' : '离线候选') : (assessment?.dynamic_precheck ? (en ? 'Data may be sufficient' : '待离线辨识') : (en ? 'Unverified' : '尚未验证'))],
    ];
    return `<section class="workspace-section calibration-results-card"><h3>${en ? 'Calibration records and candidates' : '标定记录与候选结果'}</h3><div class="calibration-result-stages">${stages.map(([name, tone, label]) => `<div><b>${name}</b><span class="result-badge ${tone}">${label}</span></div>`).join('')}</div><p class="hint">${en ? 'Review each result, then explicitly apply or export the candidate' : '按各项结果审核候选，确认后再临时应用或导出'}</p>${assessment ? `<p class="hint">${en ? 'Data precheck' : '数据质量预评估'} · ${assessment.training_groups} / ${assessment.holdout_groups} ${en ? 'static poses' : '静态姿态'} · ${assessment.valid_dynamic_samples} ${en ? 'potential dynamic frames' : '动态候选帧'} · ${en ? 'excitation observability not yet validated' : '惯量可辨识性仍需正式计算'}</p>` : ''}${state.advancedProgress ? `<p class="workspace-error">${esc(state.advancedProgress)}</p>` : ''}${record.directory ? `<div class="task-directory">${esc(record.directory)}</div>` : ''}</section>`;
}
const calibrationToolExportLabels = {
    teaching: ['导出示教轨迹数据包', 'Export teaching trajectory'],
    gain: ['导出增益标定结果', 'Export calibrated gains'],
    gravity: ['导出重力模型校正结果', 'Export gravity correction'],
    dynamics: ['导出高级动力学辨识结果', 'Export dynamics identification'],
};
const calibrationToolDefinitions = [
    ['teaching', '拖拽示教', 'Teaching', '轨迹采集与复用', 'Record and reuse trajectories'],
    ['gain', '执行器增益自动标定', 'Actuator gain calibration', '刚性、柔性、拖拽与伺服跟踪', 'Rigid, compliant, drag and servo gains'],
    ['gravity', '重力模型校正', 'Gravity correction', '全局 / 局部 · 静态采样验证', 'Global / local gravity'],
    ['dynamics', '高级动力学辨识', 'Advanced dynamics', '惯性参数与离线验证', 'Inertial identification'],
];
function selectedCandidateParams(kind) {
    const stored=state.calibrationCandidateSelection[kind];
    const directory=state.modelCalibrationOffline?.directory || state.modelCalibrationDirectory || state.telemetry?.model_calibration?.directory;
    const candidate=stored && (!stored.directory || stored.directory===directory) ? stored : null;
    return kind==='gravity' ? {candidate_source:candidate?.source || state.modelCalibrationCandidateSource,
        ...(candidate?.path ? {candidate_path:candidate.path} : {})} : candidate?.path ? {report:candidate.path} : {};
}
function candidateSummaryRows(item) {
    const r=item?.summary || {}, raw=r.gravity_validation?.raw_urdf?.validation || r.evaluation?.raw_urdf?.validation || r.raw_urdf?.validation || r.metrics?.raw_urdf?.validation || {};
    const rows=[['轮次 / Run',item?.id],['来源 / Source',item?.source],['范围 / Scope',r.identification_mode || r.mode || r.scope],
        ['可信连杆 / Locked links',(r.locked_links || []).join(', ') || '—'],['目标 / Objective',r.objective || r.fit_objective],
        ['基线 RMS / Baseline RMS',raw.baseline?.aggregate_rms_nm ?? r.baseline_rms_nm ?? r.metrics?.validation?.baseline_rms_nm],
        ['候选 RMS / Candidate RMS',raw.candidate?.aggregate_rms_nm ?? r.candidate_rms_nm ?? r.metrics?.validation?.candidate_rms_nm],
        ['结论 / Verdict',r.recommendation || r.status || (r.static_pass===true ? 'static_pass' : r.static_pass===false ? 'static_failed' : '—')]];
    return rows.filter(([,v])=>v!==undefined && v!==null).map(([label,value])=>`<tr><th>${esc(label)}</th><td>${esc(typeof value==='object' ? JSON.stringify(value) : value)}</td></tr>`).join('');
}
function calibrationCandidatesHtml(kind) {
    const items=state.calibrationCandidateRounds[kind] || [];if(!['gravity','dynamics'].includes(kind) || !items.length)return '';
    const selected=state.calibrationCandidateSelection[kind], compare=items.find(item=>item.id===state.calibrationCandidateCompare?.[kind]);
    const options=items.map((item,index)=>`<option value="${esc(item.id)}" ${item.id===selected?.id ? 'selected' : ''}>${index+1} · ${esc(item.source)} · ${esc(item.id)} · ${new Date(item.created_at*1000).toLocaleString()}</option>`).join('');
    return `<section class="candidate-rounds"><h4>${busyText('候选轮次与比较','Candidate runs and comparison')}</h4><label>${busyText('当前选中轮次，应用与导出均使用此结果','Selected run used by Apply and Export')}<select data-candidate-run="${kind}">${options}</select></label><label>${busyText('对照轮次','Compare with')}<select data-candidate-compare="${kind}"><option value="">${busyText('不比较','No comparison')}</option>${items.map(item=>`<option value="${esc(item.id)}" ${item.id===compare?.id ? 'selected' : ''}>${esc(item.id)}</option>`).join('')}</select></label><div class="candidate-comparison-grid"><table class="workspace-table"><tbody>${candidateSummaryRows(selected)}</tbody></table>${compare ? `<table class="workspace-table"><tbody>${candidateSummaryRows(compare)}</tbody></table>` : ''}</div><details><summary>${busyText('所选结果文件','Selected result file')}</summary><code>${esc(selected?.path || '')}</code></details></section>`;
}
function calibrationToolHistoryHtml(kind) {
    const selected=state.calibrationToolSelected[kind],records=state.calibrationToolRecords[kind] || [],page=state.calibrationHistoryPages[kind] || {},recent=state.telemetry?.model_calibration || {};
    const canExport=!!selected || (kind==='dynamics' && !!state.advancedReport?.candidate_urdf) || (kind==='gravity' && !!state.modelCalibrationOffline?.result?.gravity_result) || (recent.workflow_kind===kind && !!recent.directory);
    return `<section class="workspace-section calibration-tool-history"><div class="workspace-section-head"><div><h3>${busyText('任务记录','Task records')}</h3><p class="hint">${busyText('记录浏览、选择和导出无需连接；复制历史成果可在工作台连接期间完成','Browse and export saved records without connecting; historical exports also work while connected')}</p></div></div><div class="workspace-actions"><label>${busyText('查找任务','Find tasks')}<input type="search" data-tool-query="${kind}" value="${esc(page.query || '')}" placeholder="${busyText('任务、日期或载荷','Task, date or payload')}"></label><button class="button" data-tool-scan="${kind}">${busyText('查找已有任务','Find saved tasks')}</button><button class="button" data-tool-load="${kind}">${busyText('打开任务文件夹','Open task folder')}</button>${kind==='dynamics' ? '' : `<button class="button primary" data-tool-export="${kind}" ${canExport ? '' : 'disabled'}>${calibrationToolExportLabels[kind][state.language==='en' ? 1 : 0]}</button>`}</div>${selected ? `<div class="callout"><b>${esc(selected.task_id || '')}</b><div>${esc(selected.directory)}</div></div>` : ''}${records.length ? `<div class="calibration-tool-list">${records.map((r,i)=>`<button class="button calibration-tool-record" data-tool-select="${kind}" data-tool-index="${i}"><b>${esc(r.task_id || '')}</b><small>${esc(r.record_kind || '')} · ${esc(r.trajectory?.samples ?? 0)} ${busyText('帧','frames')}</small></button>`).join('')}</div>` : `<p class="hint">${page.total===0 ? busyText('没有匹配的记录，请调整筛选或打开任务文件夹','No matches; change the filter or open a task folder') : busyText('查找已保存的任务，或直接打开任务文件夹','Find saved tasks or open a task folder')}</p>`}<div class="workspace-actions"><button class="button" data-tool-page="${kind}" data-offset="${Math.max(0,(page.offset || 0)-30)}" ${page.offset>0 ? '' : 'disabled'}>${busyText('上一页','Previous')}</button><span>${page.total==null ? '' : `${Math.min((page.offset || 0)+1,page.total)}–${Math.min((page.offset || 0)+records.length,page.total)} / ${page.total}`}</span><button class="button" data-tool-page="${kind}" data-offset="${(page.offset || 0)+30}" ${page.has_more ? '' : 'disabled'}>${busyText('下一页','Next')}</button></div>${calibrationCandidatesHtml(kind)}</section>`;
}
async function scanCalibrationHistory(kind,offset=0) {
    const query=document.querySelector(`[data-tool-query="${kind}"]`)?.value ?? state.calibrationHistoryPages[kind]?.query ?? '';
    const page=await api.request('calibration_tool_records_page',{kind,config:state.config,offset,limit:30,query});
    state.calibrationToolRecords[kind]=page.records;state.calibrationHistoryPages[kind]={...page,query};render();
}
async function refreshCalibrationCandidates(kind,directory,selectedPath) {
    if(!['gravity','dynamics'].includes(kind))return;
    const result=await api.request('calibration_candidates',{kind,directory});
    result.candidates=result.candidates.map(item=>({...item,directory:result.directory || directory}));
    state.calibrationCandidateRounds[kind]=result.candidates;
    const old=state.calibrationCandidateSelection[kind];
    state.calibrationCandidateSelection[kind]=result.candidates.find(item=>item.path===(selectedPath || old?.path) || item.id===selectedPath) || result.candidates[0] || null;
}
async function selectCalibrationToolRecord(kind,selection,candidate=null) {
    if(!selection?.directory)throw new Error(busyText('请选择有效任务','Choose a valid task'));
    state.calibrationToolSelected[kind]=selection;
    if(['gravity','dynamics'].includes(kind)) {
        await refreshCalibrationCandidates(kind,selection.directory,candidate?.path);
        const selected=candidate || state.calibrationCandidateSelection[kind];
        const record=await api.request('calibration_tool_load',{kind,directory:selection.directory,...(selected?.path ? {candidate_path:selected.path} : {})});
        adoptGravityRecord(record);state.modelCalibrationDirectory=record.directory;state.advancedReport=null;state.advancedAssessment=null;state.advancedLocked=advancedTrustedLinks(record);
        if(kind==='dynamics' && record.dynamic_candidates?.length)state.advancedReport={...record.dynamic_candidates[0],task_id:record.task_id};
        state.modelCalibrationSavePreview=null;state.modelCalibrationExport=null;
    }
    render();
}

// Read terminal task records without changing motor state. Failure dialogs keep
// their own task-bound copy even if a later task starts before this read ends.
function syncCompletedGainCandidate(calibration = state.telemetry?.model_calibration) {
    if (!workbenchConnected() || calibration?.workflow_kind !== 'gain' ||
        !['complete','failed'].includes(calibration?.phase) || !calibration.directory) return;
    const directory = calibration.directory, task_id = calibration.task_id, epoch = state.workbenchEpoch;
    const task = {directory,task_id}, current = () => epoch === state.workbenchEpoch && state.telemetry?.model_calibration?.task_id === task_id && state.telemetry.model_calibration.directory === directory;
    const selected = state.calibrationToolSelected.gain;
    if (selected?.directory === directory && selected?.gain_candidate?.task_id === task_id) {
        updateGainFailureRecordEvidence(task,selected); return;
    }
    if (state.gainCandidateLoadPending === directory) return;
    if (state.gainCandidateLoadAttempted === directory) {
        updateGainFailureRecordEvidence(task,null,state.gainCandidateLoadError || busyText('本任务未提供可用的失败用途记录','No usable failed-use record is available for this task')); return;
    }
    state.gainCandidateLoadPending = directory;
    api.request('calibration_tool_load', { kind: 'gain', directory }).then(record => {
        if (record?.directory !== directory || record?.task_id !== task_id || record?.gain_candidate?.task_id !== task_id)
            throw new Error(busyText('读取的增益记录与失败任务不一致','The gain record does not match the failed task'));
        updateGainFailureRecordEvidence(task,record);
        if (!current()) return;
        state.calibrationToolSelected.gain = record;
        state.gainCandidateLoadError = '';
        if (state.page === 'calibration' && state.calibrationEntry === 'gain') refreshCalibrationOrDefer();
    }).catch(error => {
        updateGainFailureRecordEvidence(task,null,error.message || String(error));
        if (!current()) return;
        state.gainCandidateLoadError = error.message || String(error);
        if (state.page === 'calibration' && state.calibrationEntry === 'gain') refreshCalibrationOrDefer();
    }).finally(() => {
        if (state.gainCandidateLoadPending !== directory) return;
        state.gainCandidateLoadPending = '';
        if (current()) state.gainCandidateLoadAttempted = directory;
    });
}
async function temporaryCandidateAction(action, kind) {
    return blockingOperation(busyText('正在切换临时参数', 'Applying temporary parameters'), async () => {
        if (!workbenchConnected()) throw new Error('请先连接工作台');
        if (state.telemetry?.robot_state !== 'INACTIVE') throw new Error('临时应用与撤销需要先安全停放机械臂（INACTIVE）');
        const preview = kind === 'gravity' ? gravityCandidatePreviewState() : null;
        const directory = preview?.directory || state.calibrationToolSelected[kind]?.directory || state.modelCalibrationDirectory;
        if (action === 'apply' && !directory) throw new Error('请选择候选任务');
        if (action === 'apply' && kind === 'gravity' && !preview.canApply)
            throw new Error('重力候选未通过静态验收，或当前模型不匹配，请检查任务和机械臂状态');
        const report = kind === 'dynamics' ? (state.calibrationCandidateSelection.dynamics?.path || state.advancedReport?.report || (state.advancedReport?.candidate_urdf ? `${state.advancedReport.candidate_urdf.replace(/\/[^/]+$/, '')}/identification-report.json` : '')) : '';
        if (action === 'apply' && kind === 'dynamics' && !report) throw new Error('请先运行高级惯量辨识并获得离线审核通过的候选');
        const gainPurposes = kind === 'gain' && action === 'apply' ? selectedGainPurposes() : null;
        if (gainPurposes && !gainPurposes.length) throw new Error(busyText('请选择至少一种已完成测量的增益用途','Select at least one measured gain use'));
        state.calibrationRuntimeCandidates = await api.request('calibration_candidate_' + action, {
            kind, directory, report, ...(['gravity','dynamics'].includes(kind) ? selectedCandidateParams(kind) : {}), ...(gainPurposes ? {gain_purposes:gainPurposes} : {}), prefer_recomputed: kind === 'gravity' && preview?.offline === true
        });
        if (kind === 'gain') await loadAdmittance();
        render();
    });
}
function gainRequirementFieldsHtml(active) {
    const en = state.language === 'en', draft = state.gainRequirements || {};
    const fields = [['peak_error_rad',.012,.04,en ? 'Peak tracking error °' : '峰值跟踪误差 °'],['rms_error_rad',.004,.02,en ? 'RMS tracking error °' : '均方根跟踪误差 °'],['hold_error_rad',.004,.03,en ? 'Stationary hold error °' : '静态保持误差 °'],['settling_time_s',.7,2,en ? 'Settling time s' : '稳定时间 s']];
    return `<div class="model-calibration-options interaction-goal-fields">${fields.map(([key,value,maximum,label]) => {
        const factor = key === 'settling_time_s' ? 1 : 180/Math.PI;
        const raw = draft[key] ?? value, display = raw === '' ? '' : Number(raw)*factor;
        return `<label>${label}<input type="number" min="0.000001" max="${maximum*factor}" step="0.001" data-gain-requirement="${key}" data-gain-factor="${factor}" value="${esc(display)}" ${active ? 'disabled' : ''}></label>`;
    }).join('')}</div><p class="hint">${en ? 'The current automatic probes support rotary joints. Angle inputs are displayed in degrees and submitted in radians' : '当前自动探测支持转动关节，角度以度显示，提交时转换为弧度'}</p>`;
}
function gainCalibrationHasLinearAxes() {
    const names = state.telemetry?.joint_names || state.modelData?.native?.config?.joint_names || [];
    return names.some(name => jointDisplaySpec(name).linear);
}
function readGainRequirements() {
    const limits = {peak_error_rad:.04,rms_error_rad:.02,hold_error_rad:.03,settling_time_s:2,local_range_rad:.04};
    const result = {};
    for (const [key,raw] of Object.entries(state.gainRequirements || {})) {
        const value = typeof raw === 'string' && !raw.trim() ? NaN : Number(raw);
        if (!Object.hasOwn(limits,key) || !Number.isFinite(value) || value <= 0 || value > limits[key]) throw new Error(busyText('标定验收条件需为正有限值，且不超过各项输入上限', 'Calibration requirements must be positive finite values within the displayed limits'));
        result[key] = value;
    }
    return result;
}
function gainProbePhaseLabel(kind) {
    const en = state.language === 'en';
    if (/^(?:trajectory-(?:base|soft)-search-|(?:native|compliant)-train-)\d+$/.test(kind || '')) return en ? 'Joint path gain search' : '全关节路径增益搜索';
    if (/^trajectory-.*frozen/.test(kind || '')) return en ? 'Independent path validation' : '独立路径验收';
    if (['trajectory-noise','noise-all'].includes(kind)) return en ? 'Joint stationary measurement' : '全关节静止测量';
    if (/^noise-\d+$/.test(kind || '')) return en ? 'Stationary noise measurement' : '静止噪声测量';
    if (/^(?:base|soft)-motion-spectrum-\d+$/.test(kind || '')) return en ? 'Multifrequency motion check' : '多频运动确认';
    if (/^(?:(?:base|soft)-)?motion-\d+$/.test(kind || '')) return en ? 'Bidirectional motion probe' : '双向运动探测';
    if (/^base-\d+$/.test(kind || '')) return en ? 'Baseline search' : '基线增益搜索';
    if (/^soft-\d+$/.test(kind || '')) return en ? 'Compliant gain search' : '柔性增益搜索';
    if (/^drag-search-\d+$/.test(kind || '')) return en ? 'Drag damping search' : '拖拽阻尼搜索';
    const labels = {'native-frozen':['原生跟踪独立验收','Independent native tracking check'], 'soft-frozen':['柔性跟踪独立验收','Independent compliant tracking check'], 'compliant-frozen':['柔性跟踪独立验收','Independent compliant tracking check'], 'admittance-frozen':['模型导纳独立验收','Independent admittance check'], hold:['定位保持验收','Hold check'], 'drag-decay':['拖拽阻尼衰减验收','Drag damping decay check'], 'drag-train':['全关节拖拽阻尼测量','Joint drag damping measurement'], 'drag-frozen':['拖拽阻尼独立验收','Independent drag damping check']};
    return labels[kind]?.[en ? 1 : 0] || kind || '—';
}
function gainProbeStageLabel(stage) {
    if (stage === 'trajectory_endpoint') return busyText('等待轨迹端点回稳','Wait for the trajectory endpoint to settle');
    const labels = {noise:['测量静止噪声','Measure stationary noise'], motion_discovery:['建立双向运动','Establish bidirectional motion'], dynamic_search:['动态增益搜索','Search dynamic gains'], trajectory_search:['全关节轨迹增益搜索','Joint trajectory gain search'], trajectory_frozen:['独立轨迹验收','Independent trajectory validation'], frozen:['独立跟踪验收','Independent tracking check'], hold:['保持验收','Hold check'], drag:['拖拽阻尼验收','Drag damping check']};
    return labels[stage]?.[state.language === 'en' ? 1 : 0] || stage || '—';
}
function gainProbeMessageLabel(message) {
    const endpoint = {endpoint_settling:['保持示教端点，等待增益切换后的关节回稳','Holding the demonstrated endpoint while joints settle after the gain change'],endpoint_ready:['端点姿态与速度已回稳，准备下一趟','Endpoint position and speed have settled; preparing the next pass']};
    if (endpoint[message]) return endpoint[message][state.language === 'en' ? 1 : 0];
    if (message === 'endpoint_alignment_required') return gainWarningLabel(message);
    if (message === 'noise_reference_warning') return busyText('静止窗口提示已记录，继续测量','Stationary-window notes recorded; measurement continues');
    if (['time_budget_reached','time_budget_may_limit_measurements'].includes(message)) return gainWarningLabel(message);
    const release = {drag_not_stationary:['释放后仍有持续运动','Motion persisted after release'],drag_decay_too_slow:['释放速度未在限定时间内衰减','Release velocity did not decay within the time limit'],drag_speed_increased:['释放后的持续速度超过阈值','Sustained speed after release exceeded the limit']};
    if (release[message]) return release[message][state.language === 'en' ? 1 : 0];
    const pathMessages = {trajectory_endpoint_check:['正在检查轨迹端点与关节反馈','Checking the trajectory endpoint and joint feedback'],trajectory_replay:['正在沿示教路径联合测量所有关节','Measuring all joints along the demonstrated path'],trajectory_settling:['正在测量轨迹端点稳定性','Measuring trajectory endpoint settling'],noise_measurement:['正在联合测量各关节静止噪声','Measuring stationary noise on all joints']};
    if (pathMessages[message]) return pathMessages[message][state.language === 'en' ? 1 : 0];
    const trajectory = {trajectory_coverage_insufficient:['示教路径未覆盖本轴足够的正反向运动','The demonstrated path lacks sufficient motion in both directions on this joint'],weak_response:['参考已运动，实测响应幅度仍不足','Measured movement remains too small relative to the reference'],budget_exhausted:['已到自动标定时限，未验收用途保留测量结果','The calibration time limit was reached; unqualified uses retain their measurements'],trajectory_tracking:['正在沿示教路径联合测量所有关节','Measuring all joints along the demonstrated path']};
    if (trajectory[message]) return trajectory[message][state.language === 'en' ? 1 : 0];
    if (message === 'drag_decay_not_qualified') return state.language === 'en' ? 'Release velocity did not decay within the limit, or the measured speed was excessive' : '释放后的速度未在限定时间内衰减，或实测速度超过阈值';
    if (message === 'noise_measured') return state.language === 'en' ? 'Stationary noise measurement complete' : '已完成静止噪声测量';
    if (message === 'measurement_pending') return state.language === 'en' ? 'Waiting for measurements from this window' : '等待本窗测量数据';
    if (message === 'measurement_interrupted') return state.language === 'en' ? 'This window was interrupted; only captured measurements are shown' : '本窗测量已中断，仅显示已采集数据';
    const measurements = {low_bandwidth:['有效跟随频率不足','The measured following bandwidth is too low'], excessive_lag:['实测响应相位滞后过大','The measured response has excessive phase lag'], resonance:['检测到共振响应','A resonant response was detected'], hold_drift:['静态保持漂移超过阈值','Stationary hold drift exceeds the acceptance limit'], unstable_noise:['静止反馈仍不稳定，无法建立噪声基线','Stationary feedback is unstable; a noise baseline is unavailable']};
    if (measurements[message]) return measurements[message][state.language === 'en' ? 1 : 0];
    const labels = {noise_sampling:['正在测量编码器噪声与分辨率','Measuring encoder noise and resolution'], noise_sampled:['已完成静止噪声测量','Stationary noise measurement complete'], establishing_motion:['正在确认参考与实测均有双向运动','Checking bidirectional reference and measured motion'], motion_plan_ready:['双向运动已建立，准备动态增益搜索','Bidirectional motion established; preparing dynamic gain search'], motion_established:['双向有效运动已建立','Effective bidirectional motion established'], no_motion:['未测到有效运动','No effective measured motion'], one_sided_response:['正、反向有效行程未同时达到下限','Effective travel did not meet the minimum in both directions'], low_coherence:['参考与实测响应的一致性不足','Reference and measured response have low coherence'], dynamic_tracking_failed:['动态跟踪未通过本窗验收','Dynamic tracking did not pass this window'], ringing:['检测到持续振荡','Sustained oscillation detected'], oscillating:['检测到持续振荡','Sustained oscillation detected'], overdamped:['本窗响应过慢，阻尼偏大','The measured response is slow with excessive damping'], delayed_response:['实测响应延迟过大','Measured response delay is too large'], insufficient_measurements:['有效连续测量不足','Insufficient fresh continuous measurements'], limited:['测量中出现参考或力矩限幅','Reference or torque was limited during measurement'], reference_too_small:['参考运动不足以判断响应','Reference motion is too small to assess the response'], configured_baseline:['使用已配置增益作为起点','Starting from configured gains'], model_seed:['使用模型估计增益作为起点','Starting from model-estimated gains'], settling:['正在建立增益并等待稳定','Applying gains and settling'], measuring:['正在采集关节响应','Measuring joint response'], assessing:['正在评估本窗响应','Assessing this measurement window'], no_response:['本轴未检测到有效跟随响应','No effective following response on this joint'], tracking_not_qualified:['本次跟踪未通过验收','This tracking trial did not qualify'], qualified:['本次验收通过','This trial qualified'], candidate_updated:['已调整候选增益，继续测量','Candidate gains updated; continuing measurement'], safety_bound:['探测受到安全限制，无法继续','A configured safety limit prevents further probing'], hold_verification:['正在检查定位保持','Checking stationary hold'], drag_verification:['正在检查释放后的速度衰减','Checking velocity decay after release'], drag_excitation:['使用已验收跟踪增益建立释放前速度','Building release velocity with qualified tracking gains'], failed:['标定失败，请查看错误详情','Calibration failed; see error details'], cancelled:['任务已取消','Task cancelled'], complete:['标定已完成','Calibration complete']};
    return labels[message]?.[state.language === 'en' ? 1 : 0] || (message ? localizedError(message) : '—');
}
function gainProbeStatusValues(status = {}) {
    const en = state.language === 'en', number = value => typeof value === 'number' && Number.isFinite(value);
    const elapsed = status.gain_probe_elapsed_s, duration = status.gain_probe_duration_s;
    const measurable = number(elapsed) && elapsed >= 0 && number(duration) && duration > 0;
    const fraction = measurable ? Math.max(0, Math.min(1, elapsed / duration)) : null;
    const index = status.gain_candidate_index, limit = status.gain_candidate_limit;
    const windows = Math.max(0, Number(status.gain_adaptation_passes) || 0), maximum = Math.max(0, Number(status.gain_adaptation_max_passes) || 0);
    const jointPath = status.gain_protocol === 'taught_trajectory';
    return {use:gainPurposeLabel(status.gain_active_target), joint:jointPath ? (en ? 'All joints' : '全部关节') : status.gain_probe_joint || (['hold','drag-decay'].includes(status.gain_probe_kind) ? (en ? 'All joints' : '全部关节') : '—'),
        phase:status.gain_probe_stage ? gainProbeStageLabel(status.gain_probe_stage) : gainProbePhaseLabel(status.gain_probe_kind), frequency:(!jointPath || status.gain_probe_stage === 'drag') && Array.isArray(status.gain_probe_frequency_hz) && status.gain_probe_frequency_hz.some(value=>number(value) && value>0) ? status.gain_probe_frequency_hz.filter(number).map(value=>value.toFixed(2)).join(' / ') + ' Hz' : '—', candidate:number(index) && index > 0 ? `${index}${number(limit) && limit > 0 ? ' / ' + limit : ''}` : '—',
        amplitude:(!jointPath && status.gain_probe_kind !== 'hold' || jointPath && status.gain_probe_stage === 'drag') && number(status.gain_probe_amplitude_rad) ? `${(status.gain_probe_amplitude_rad * 180 / Math.PI).toFixed(3)}°` : '—',
        gains:status.gain_probe_joint && (!jointPath || ['trajectory_search','trajectory_frozen','drag'].includes(status.gain_probe_stage)) && number(status.gain_probe_kp) && number(status.gain_probe_kd) ? `${jointPath ? (state.telemetry?.joint_names?.[0] || (en ? 'First joint' : '首轴')) + ' · ' : ''}${status.gain_probe_kp.toFixed(3)} / ${status.gain_probe_kd.toFixed(3)}` : '—',
        window:measurable ? `${Math.min(elapsed,duration).toFixed(1)} / ${duration.toFixed(1)} s · ${Math.round(fraction * 100)}%` : (status.gain_probe_message === 'settling' ? (en ? 'Settling before measurement' : '测量前稳定中') : '—'),
        windows:`${windows}${maximum ? ' / ' + maximum : ''}`, paths:jointPath && number(status.gain_path_pass_limit) ? `${Number(status.gain_path_pass_current || 0)} / ${status.gain_path_pass_limit}` : '—',
        budget:jointPath && number(status.gain_time_budget_s) ? `${number(status.gain_elapsed_s) ? status.gain_elapsed_s.toFixed(1) : '—'} / ${status.gain_time_budget_s.toFixed(1)} s · ${en ? 'remaining' : '剩余'} ${number(status.gain_remaining_s) ? status.gain_remaining_s.toFixed(1) : '—'} s` : '—', fraction, message:gainProbeMessageLabel(status.gain_probe_message),
        error:status.error ? localizedError(status.error) : ''};
}
function gainProbeEvidenceHtml(status = {}) {
    const en = state.language === 'en', diagnostic = status.gain_probe_diagnostic;
    const finite = value => typeof value === 'number' && Number.isFinite(value);
    const number = (value, digits = 3) => finite(value) ? value.toFixed(digits) : '—';
    const angle = value => finite(value) ? `${(value * 180 / Math.PI).toFixed(5)}°` : '—';
    const yes = value => value === true ? (en ? 'Yes' : '是') : value === false ? (en ? 'No' : '否') : '—';
    const fields = [[en ? 'Task' : '任务', status.task_id || '—']];
    if (!diagnostic || typeof diagnostic !== 'object' || Array.isArray(diagnostic)) return `<dl class="gain-probe-evidence-grid">${fields.map(([label,value])=>`<div><dt>${esc(label)}</dt><dd>${esc(value)}</dd></div>`).join('')}</dl><p class="hint">${en ? 'No usable measurement-window summary was provided for this task' : '本任务尚未提供可用的测量窗摘要'}</p>`;
    if (diagnostic.stage === 'trajectory_endpoint') {
        const axes = Array.isArray(diagnostic.axes) ? diagnostic.axes : [];
        const degrees = value => finite(value) ? (value*180/Math.PI).toFixed(3) : '—';
        const pair = (actual,limit) => `${degrees(actual)} / ${degrees(limit)}`;
        return `<p class="gain-evidence-reason">${esc(gainProbeMessageLabel(diagnostic.reason))}</p><p class="hint">${en ? 'Settling wait / maximum' : '回稳等待 / 最长等待'} · ${number(diagnostic.elapsed_s,1)} / ${number(diagnostic.timeout_s,1)} s</p><div class="table-scroll"><table class="workspace-table"><thead><tr><th>${t('joint')}</th><th>${en ? 'Current °' : '当前 °'}</th><th>${en ? 'Target °' : '目标 °'}</th><th>${en ? 'Error / limit °' : '偏差 / 允许值 °'}</th><th>${en ? 'Speed / limit °/s' : '速度 / 允许值 °/s'}</th><th>${en ? 'Feedback ms' : '反馈 ms'}</th><th>${en ? 'State' : '状态'}</th></tr></thead><tbody>${axes.map(axis=>`<tr><th>${esc(axis.joint || '—')}</th><td>${degrees(axis.actual)}</td><td>${degrees(axis.target)}</td><td>${pair(axis.error,axis.position_tolerance)}</td><td>${pair(axis.velocity,axis.velocity_tolerance)}</td><td>${number(axis.feedback_age_ms,1)}</td><td>${esc(axis.ready === true ? (en ? 'Ready' : '已回稳') : (en ? 'Waiting' : '等待回稳'))}</td></tr>`).join('')}</tbody></table></div>`;
    }
    const pair = (a,b,format = angle) => `${format(a)} / ${format(b)}`;
    const pending = diagnostic.reason === 'measurement_pending';
    const measured = value => pending ? null : value;
    const assessed = diagnostic.completed !== false && !pending;
    const hold = diagnostic.stage === 'hold', drag = diagnostic.stage === 'drag';
    const pathMeasurement = ['trajectory_search','trajectory_frozen'].includes(diagnostic.stage);
    const frequencyResponse = !hold && !drag && diagnostic.stage !== 'noise' && !pathMeasurement;
    const alignedMotion = diagnostic.motion_observation_basis === 'reference_aligned_travel_v1';
    fields.push([en ? 'Measurement stage' : '测量阶段', gainProbeStageLabel(diagnostic.stage)], [en ? 'Joint / probe' : '关节 / 探测', `${diagnostic.joint || (hold ? en ? 'All joints' : '全部关节' : '—')} · ${gainProbePhaseLabel(diagnostic.kind || (hold ? 'hold' : drag ? 'drag-decay' : ''))}`],
        [en ? 'Candidate trial' : '候选轮次', finite(diagnostic.candidate_index) && diagnostic.candidate_index > 0 ? String(diagnostic.candidate_index) : '—'],
        [en ? 'This window Kp / Kd' : '本窗 Kp / Kd', pair(diagnostic.kp,diagnostic.kd,number)], [en ? 'Amplitude' : '幅度', angle(diagnostic.amplitude_rad)]);
    if (finite(diagnostic.safety_kp_max) && finite(diagnostic.safety_kd_max)) fields.push([en ? 'Safety Kp / Kd limits' : 'Safety Kp / Kd 上限', pair(diagnostic.safety_kp_max,diagnostic.safety_kd_max,number)]);
    if (pathMeasurement && diagnostic.tuning?.valid === true) {
        const tuning = diagnostic.tuning;
        fields.push([en ? 'Tuning estimate · path offset' : '调参估计 · 路径偏置', angle(tuning.bias_rad)],
            [en ? 'Tuning estimate · centered RMS error' : '调参估计 · 去偏置 RMS 误差', angle(tuning.centered_rms_rad)],
            [en ? 'Tuning estimate · in-phase amplitude ratio' : '调参估计 · 同相幅值比', number(tuning.in_phase_gain,4)],
            [en ? 'Tuning estimate · velocity-correlated lag' : '调参估计 · 速度相关滞后', `${finite(tuning.lag_s) ? number(tuning.lag_s*1000,2) : '—'} ms`]);
    }
    if (assessed && Array.isArray(diagnostic.unmet_requirements) && diagnostic.unmet_requirements.length) {
        const labels = {measurement_samples:['有效采样数量','Fresh sample count'],continuous_feedback:['反馈连续性','Feedback continuity'],not_limited:['参考与力矩限幅','Reference and torque limits'],bidirectional_motion:['正、反向有效行程','Effective travel in both directions'],motion_coherence:['运动相关性','Motion correlation'],response_rms_ratio:['响应幅度比','Response amplitude ratio'],observable:['响应可测性','Response observability'],rms_error_rad:['RMS 跟踪误差','RMS tracking error'],peak_error_rad:['峰值跟踪误差','Peak tracking error'],velocity_rms_rad_s:['速度 RMS 误差','Velocity RMS error'],overshoot_rad:['超调','Overshoot'],settled:['尾部稳定性','Tail stability'],settling_time_s:['稳定时间','Settling time'],oscillation:['持续振荡','Sustained oscillation'],follow_frequency_hz:['跟随频率','Following frequency']};
        fields.push([en ? 'Requirements not met' : '未通过的项目', diagnostic.unmet_requirements.map(key=>labels[key]?.[en ? 1 : 0] || key).join(en ? ', ' : '、')]);
    }
    if (frequencyResponse) fields.push([en ? 'Frequencies' : '频率', Array.isArray(diagnostic.frequencies_hz) && diagnostic.frequencies_hz.length ? diagnostic.frequencies_hz.filter(finite).map(value=>number(value,2)).join(' / ') + ' Hz' : '—'],
        [en ? 'Reference motion + / −' : '参考运动 正向 / 反向', pair(measured(diagnostic.reference_positive_span_rad),measured(diagnostic.reference_negative_span_rad))],
        [en ? alignedMotion ? 'Measured range about reference center + / −' : 'Measured range about window start + / −' : alignedMotion ? '相对参考中心的实测范围 正向 / 反向' : '相对窗初位置的实测范围 正向 / 反向', pair(measured(diagnostic.actual_positive_span_rad),measured(diagnostic.actual_negative_span_rad))],
        [en ? 'Response RMS ratio / coherence' : '响应 RMS 比 / 一致性', pair(measured(diagnostic.response_rms_ratio),measured(diagnostic.coherence),number)],
        [en ? 'Fresh / total samples' : '新鲜反馈 / 总样本', `${finite(diagnostic.measurement_samples) ? diagnostic.measurement_samples : '—'} / ${finite(diagnostic.samples) ? diagnostic.samples : '—'}`],
        [en ? 'Continuous / limited samples' : '测量连续 / 限幅样本', `${yes(measured(diagnostic.continuous))} / ${!pending && finite(diagnostic.limited_samples) ? diagnostic.limited_samples : '—'}`],
        [en ? alignedMotion ? 'Bidirectional motion / tracking metrics passed' : 'Observable response / tracking metrics passed' : alignedMotion ? '双向运动 / 跟踪指标通过' : '响应可测 / 跟踪指标通过', `${yes(assessed ? alignedMotion ? diagnostic.motion_qualified : diagnostic.observable : null)} / ${yes(assessed ? diagnostic.metrics_passed : null)}`],
        [en ? 'Qualified following frequency' : '已通过跟随频率', assessed && finite(diagnostic.follow_frequency_hz) ? `${number(diagnostic.follow_frequency_hz)} Hz` : '—']);
    else {
        fields.push([en ? 'Samples' : '样本数', finite(diagnostic.samples) ? diagnostic.samples : '—']);
        if (hold || drag || pathMeasurement) fields.push([en ? 'Acceptance passed' : '验收通过', yes(assessed ? diagnostic.metrics_passed : null)]);
    }
    if (pathMeasurement) fields.push(
        [en ? 'Reference range about path start + / −' : '相对回放起点的参考范围 正向 / 反向', pair(measured(diagnostic.reference_positive_span_rad),measured(diagnostic.reference_negative_span_rad))],
        [en ? 'Measured range about path start + / −' : '相对回放起点的实测范围 正向 / 反向', pair(measured(diagnostic.actual_positive_span_rad),measured(diagnostic.actual_negative_span_rad))],
        [en ? 'Response RMS ratio' : '响应 RMS 比', number(measured(diagnostic.response_rms_ratio))],
        [en ? 'Bidirectional response' : '双向运动响应', yes(assessed ? diagnostic.motion_qualified : null)],
        [en ? 'Fresh / total samples' : '新鲜反馈 / 总样本', `${finite(diagnostic.measurement_samples) ? diagnostic.measurement_samples : '—'} / ${finite(diagnostic.samples) ? diagnostic.samples : '—'}`],
        [en ? 'Continuous / limited samples' : '测量连续 / 限幅样本', `${yes(measured(diagnostic.continuous))} / ${!pending && finite(diagnostic.limited_samples) ? diagnostic.limited_samples : '—'}`]);
    if (!hold && !drag) fields.push([en ? 'Observed position step / noise RMS' : '观测位置步长 / 噪声 RMS', `${finite(diagnostic.encoder_step_rad) && diagnostic.encoder_step_rad > 0 ? angle(diagnostic.encoder_step_rad) : en ? 'Not observed' : '未测得'} / ${angle(diagnostic.noise_rms_rad)}`]);
    if ((frequencyResponse || pathMeasurement) && alignedMotion) fields.push(
        [en ? 'Position error at window start' : '窗初位置偏差', angle(measured(diagnostic.initial_error_rad))],
        [en ? 'Reference-aligned measured travel + / −' : '与参考同向的实测行程 正向 / 反向', pair(measured(diagnostic.actual_positive_travel_rad),measured(diagnostic.actual_negative_travel_rad))],
        [en ? 'Motion correlation / alignment delay' : '运动相关性 / 对齐延迟', `${number(measured(diagnostic.motion_coherence))} / ${finite(measured(diagnostic.motion_alignment_delay_s)) ? number(diagnostic.motion_alignment_delay_s) + ' s' : '—'}`]);
    if (finite(diagnostic.minimum_motion_span_rad)) fields.push([en ? alignedMotion ? 'Minimum effective travel in each direction' : 'Minimum effective motion in each direction' : alignedMotion ? '正、反向有效行程下限' : '单侧有效运动下限', angle(diagnostic.minimum_motion_span_rad)]);
    if (drag) {
        const speed = value => finite(measured(value)) ? `${(value * 180 / Math.PI).toFixed(4)}°/s` : '—';
        const metrics = diagnostic.metrics || diagnostic;
        fields.push([en ? 'Initial release velocity' : '释放初始速度', speed(metrics.initial_velocity_rad_s)], [en ? 'Peak release velocity from position' : '位置变化测得的释放峰值速度', speed(metrics.peak_velocity_rad_s)],
            [en ? 'Raw velocity-code peak' : '原始速度码峰值', speed(metrics.raw_peak_velocity_rad_s)], [en ? 'Final release velocity' : '释放末段速度', speed(metrics.final_velocity_rad_s)],
            [en ? 'Decay time / limit' : '衰减时间 / 上限', `${number(metrics.decay_s)} / ${number(diagnostic.requirements?.decay_s)} s`]);
    }
    if (diagnostic.metrics && typeof diagnostic.metrics === 'object') {
        const metrics = [['rms_error_rad','RMS 跟踪误差','RMS tracking error','angle'],['peak_error_rad','峰值跟踪误差','Peak tracking error','angle'],['velocity_rms_rad_s','速度 RMS 误差','Velocity RMS error','velocity'],['overshoot_rad','超调','Overshoot','angle'],['tail_error_rad','尾部位置误差','Tail position error','angle','settling_error_rad'],['settling_time_s','稳定时间','Settling time','time']];
        for (const [key,zh,english,unit,limitKey = key] of metrics) {
            if (!frequencyResponse && (!Object.hasOwn(diagnostic.metrics,key) || hold && key === 'tail_error_rad')) continue;
            const format = value => unit === 'time' ? `${number(value)} s` : finite(value) ? `${(value * 180 / Math.PI).toFixed(4)}${unit === 'velocity' ? '°/s' : '°'}` : '—';
            const title = hold && key === 'peak_error_rad' ? (en ? 'Maximum hold position error' : '最大保持位置误差') : hold && key === 'velocity_rms_rad_s' ? (en ? 'Maximum observed hold speed' : '最大观测保持速度') : drag && key === 'settling_time_s' ? (en ? 'Release velocity decay time' : '释放速度衰减时间') : en ? english : zh;
            const comparison = key === 'tail_error_rad' ? (en ? 'tail peak / settled-error threshold' : '尾部峰值 / 稳定误差阈值') : en ? 'actual / limit' : '实测 / 上限';
            fields.push([`${title} · ${comparison}`, `${format(measured(diagnostic.metrics[key]))} / ${format(diagnostic.requirements?.[limitKey])}`]);
        }
    }
    const axes = drag && Array.isArray(diagnostic.axes) ? `<p class="hint">${en ? 'Acceptance uses continuous position changes; raw velocity-code peaks remain diagnostic data' : '按连续位置变化判断运动与衰减，原始速度码峰值单独保留'}</p><div class="table-scroll"><table class="workspace-table"><thead><tr><th>${t('joint')}</th><th>${en ? 'Release °/s' : '初速 °/s'}</th><th>${en ? 'Peak °/s' : '位置峰速 °/s'}</th><th>${en ? 'Raw peak °/s' : '原始峰速 °/s'}</th><th>${en ? 'Final °/s' : '末速 °/s'}</th><th>${en ? 'Decay s' : '衰减 s'}</th><th>${en ? 'Result' : '结果'}</th></tr></thead><tbody>${diagnostic.axes.map(axis=>`<tr><th>${esc(axis.joint || '—')}</th>${['initial_velocity_rad_s','peak_velocity_rad_s','raw_peak_velocity_rad_s','final_velocity_rad_s'].map(key=>`<td>${finite(axis[key]) ? number(axis[key]*180/Math.PI,4) : '—'}</td>`).join('')}<td>${number(axis.decay_s)}</td><td>${esc(axis.passed ? (en ? 'Passed' : '通过') : gainProbeMessageLabel(axis.reason))}</td></tr>`).join('')}</tbody></table></div>` : '';
    return `<p class="gain-evidence-reason">${esc(gainProbeMessageLabel(diagnostic.reason))}</p><dl class="gain-probe-evidence-grid">${fields.map(([label,value])=>`<div><dt>${esc(label)}</dt><dd>${esc(value)}</dd></div>`).join('')}</dl>${axes}${diagnostic.error ? `<p class="workspace-error">${esc(localizedError(diagnostic.error))}</p>` : ''}${diagnostic.record_path ? `<p class="hint gain-evidence-path">${en ? 'Window recording' : '本窗记录'}<br><code>${esc(diagnostic.record_path)}</code></p>` : ''}`;
}
function gainFailureRecordMatches(task, record) {
    return !!task.task_id && !!task.directory && record?.task_id === task.task_id && record.directory === task.directory && record.gain_candidate?.task_id === task.task_id;
}
function updateGainFailureRecordEvidence(task, record, error = '') {
    for (const entry of [activePriorityError,...priorityErrorQueue]) {
        const evidence = entry?.gainEvidence;
        if (!evidence || evidence.task_id !== task.task_id || evidence.directory !== task.directory || evidence.gain_failure_candidate) continue;
        if (gainFailureRecordMatches(task,record)) {
            evidence.gain_failure_candidate = structuredClone(record.gain_candidate);
            evidence.gain_failure_files = structuredClone(record.available_files || record.files || {});
            evidence.gain_failure_record_state = 'ready';
        } else {
            evidence.gain_failure_record_state = 'unavailable';
            evidence.gain_failure_record_error = error || busyText('未取得与本任务对应的增益候选','No matching gain candidate was returned');
        }
        if (entry === activePriorityError) paintGainFailureEvidence(entry);
    }
}
function gainFailureCsvPath(evidence, validation, purpose, test) {
    if (typeof validation?.record_path === 'string' && validation.record_path.trim()) {
        const path = validation.record_path.trim();
        return path.startsWith('/') ? path : `${evidence.directory}/${path}`;
    }
    if (evidence.gain_failure_candidate?.validation_scope !== 'independent_demonstration_trajectory') return '';
    const name = test === 'admittance' ? 'gain-trajectory-admittance-frozen.csv'
        : purpose === 'compliant_drag' ? 'drag-decay-drag-frozen.csv'
        : purpose === 'compliant_tracking' ? 'gain-trajectory-compliant-frozen.csv' : 'gain-trajectory-native-frozen.csv';
    return evidence.gain_failure_files?.[name] === true ? `${evidence.directory}/${name}` : '';
}
function gainFailureTrackingHtml(evidence, purpose, item, validation, test) {
    const en = state.language === 'en';
    if (validation?.required === false && validation.status === 'not_requested') return '';
    const names = evidence.gain_failure_candidate?.joint_names || [], limits = {...evidence.gain_failure_candidate?.gain_requirements,...validation?.requirements};
    const metrics = Array.isArray(validation?.metrics) ? validation.metrics : [];
    const format = (value,time = false) => typeof value === 'number' && Number.isFinite(value) ? time ? value.toFixed(3)+' s' : (value*180/Math.PI).toFixed(4)+'°' : '—';
    const compare = (value,limit,time = false) => `<span ${Number.isFinite(value) && Number.isFinite(limit) && value>limit ? 'class="warning-text"' : ''}>${format(value,time)} / ${format(limit,time)}</span>`;
    const gains = value => typeof value === 'number' && Number.isFinite(value) ? value.toFixed(3) : '—';
    const path = gainFailureCsvPath(evidence,validation,purpose,test);
    return `<section class="calibration-step"><h4>${test === 'admittance' ? (en ? 'Admittance path validation' : '导纳路径独立验收') : (en ? 'Native path validation' : '原生路径独立验收')} · ${gainCheckStatus(validation)}</h4>${metrics.length ? `<p class="hint">${en ? 'Measured value / acceptance limit' : '实测值 / 验收上限'}</p><div class="table-scroll"><table class="workspace-table"><thead><tr><th>${t('joint')}</th><th>Kp / Kd</th><th>RMS</th><th>${en ? 'Peak' : '峰值'}</th><th>${en ? 'Tail error' : '尾差'}</th><th>${en ? 'Settling' : '稳定时间'}</th></tr></thead><tbody>${metrics.map((row,index)=>`<tr><th>${esc(names[index] || row.joint || String(index+1))}</th><td>${gains(item.kp?.[index])} / ${gains(item.kd?.[index])}</td><td>${compare(row.rms_error_rad,limits.rms_error_rad)}</td><td>${compare(row.peak_error_rad,limits.peak_error_rad)}</td><td>${compare(row.tail_error_rad,limits.settling_error_rad)}</td><td>${compare(row.settling_s ?? row.settling_time_s,limits.settling_time_s,true)}</td></tr>`).join('')}</tbody></table></div>` : `<p class="hint">${en ? 'This validation did not provide per-joint measurements' : '该项验收未提供逐关节实测摘要'}</p>`}<p class="hint gain-evidence-path">${en ? 'Validation recording' : '验收记录'}<br>${path ? `<code>${esc(path)}</code><br><button type="button" class="button" data-gain-failure-path="${esc(path)}">${en ? 'Show CSV in folder' : '定位 CSV 文件'}</button>` : (en ? 'No confirmed CSV path was provided' : '未提供可确认的 CSV 路径')}</p></section>`;
}
function gainFailureHoldHtml(candidate, item) {
    const hold = item.hold_compensation?.enabled ? item.hold_compensation_verification : item.hold_verification;
    const en = state.language === 'en', limits = candidate.gain_requirements || {};
    if (!hold) return item.hold_compensation?.enabled ? `<section class="calibration-step"><h4>${en ? 'Stationary hold validation' : '定位保持验收'} · ${gainCheckStatus(hold)}</h4><p class="hint">${en ? 'No validation summary is available for the enabled hold compensation' : '已启用保持补偿，但缺少对应的验收摘要'}</p></section>` : '';
    const angle = value => typeof value === 'number' && Number.isFinite(value) ? (value*180/Math.PI).toFixed(4)+'°' : '—';
    return `<section class="calibration-step"><h4>${en ? 'Stationary hold validation' : '定位保持验收'} · ${gainCheckStatus(hold)}</h4><dl class="kv"><dt>${en ? 'Maximum error / limit' : '最大误差 / 上限'}</dt><dd>${angle(hold.worst_error_rad)} / ${angle(limits.hold_error_rad)}</dd><dt>${en ? 'Position span / limit' : '位置变化范围 / 上限'}</dt><dd>${angle(hold.position_span_rad)} / ${angle(limits.hold_error_rad)}</dd><dt>${en ? 'Observed speed / limit' : '观测速度 / 上限'}</dt><dd>${angle(hold.worst_velocity_rad_s)}/s / ${angle(limits.hold_velocity_rad_s)}/s</dd><dt>${en ? 'Raw velocity-code peak' : '原始速度码峰值'}</dt><dd>${angle(hold.raw_peak_velocity_rad_s)}/s · ${en ? 'diagnostic' : '诊断值'}</dd></dl></section>`;
}
function gainFailureEvidenceHtml(evidence, selectedPurpose = '') {
    const en = state.language === 'en', candidate = evidence.gain_failure_candidate;
    const groups = gainCandidateGroups(candidate), failed = Object.entries(groups).filter(([,item])=>item?.validated !== true);
    if (!candidate || !failed.length) {
        const diagnostic = evidence.gain_probe_diagnostic;
        const failedWindow = diagnostic?.metrics_passed === false && !['qualified','complete'].includes(diagnostic.reason);
        const message = evidence.gain_failure_record_state === 'loading'
            ? (en ? 'Loading validation records for the failed gain uses' : '正在读取未通过用途的验收记录')
            : (en ? 'No failed-use validation summary is available' : '尚无失败用途的验收摘要');
        return `<p class="hint">${message}${evidence.gain_failure_record_error ? ' · '+esc(evidence.gain_failure_record_error) : ''}</p>${failedWindow ? gainProbeEvidenceHtml(evidence) : ''}`;
    }
    const selected = failed.some(([key])=>key===selectedPurpose) ? selectedPurpose : failed[0][0];
    const item = groups[selected], passed = Object.entries(groups).filter(([,value])=>value?.validated === true).map(([key])=>gainPurposeLabel(key));
    const measurements = selected === 'compliant_drag' ? gainProbeEvidenceHtml({task_id:evidence.task_id,gain_probe_diagnostic:{...item.drag_validation,stage:'drag',metrics_passed:item.drag_validation?.validated,reason:item.drag_validation?.axes?.find(axis=>axis.passed === false)?.reason || 'drag_decay_not_qualified',record_path:gainFailureCsvPath(evidence,item.drag_validation,selected,'drag')}})
        : gainFailureTrackingHtml(evidence,selected,item,item.native_validation,'native') + (item.admittance_validation ? gainFailureTrackingHtml(evidence,selected,item,item.admittance_validation,'admittance') : '') + gainFailureHoldHtml(candidate,item);
    return `<p class="gain-evidence-reason">${en ? 'Not qualified' : '未通过'}：${failed.map(([key])=>esc(gainPurposeLabel(key))).join('、')}${passed.length ? ` · ${en ? 'Passed' : '通过'}：${passed.map(esc).join('、')}` : ''}</p><div class="workspace-actions" role="group" aria-label="${en ? 'Failed gain use' : '未通过的增益用途'}">${failed.map(([key])=>`<button type="button" class="button ${key===selected?'primary':''}" data-gain-failure-purpose="${esc(key)}" aria-pressed="${key===selected}">${esc(gainPurposeLabel(key))}</button>`).join('')}</div>${item.error ? `<p class="workspace-error">${esc(localizedError(item.error))}</p>` : ''}${measurements}`;
}
function paintGainFailureEvidence(entry) {
    const node = $('priority-error-evidence'); if (!node) return;
    node.hidden = !entry.gainEvidence;
    node.innerHTML = entry.gainEvidence ? gainFailureEvidenceHtml(entry.gainEvidence,entry.gainFailurePurpose) : '';
    node.querySelectorAll('[data-gain-failure-purpose]').forEach(button=>button.onclick=()=>{entry.gainFailurePurpose=button.dataset.gainFailurePurpose;paintGainFailureEvidence(entry);});
    node.querySelectorAll('[data-gain-failure-path]').forEach(button=>button.onclick=()=>operation(()=>api.revealFile(button.dataset.gainFailurePath)));
}
function gainCalibrationFailure(status) {
    const evidence = structuredClone(status), record = state.calibrationToolSelected.gain;
    evidence.gain_failure_record_state = status.directory ? 'loading' : 'unavailable';
    if (gainFailureRecordMatches(status,record)) {
        evidence.gain_failure_candidate = structuredClone(record.gain_candidate);
        evidence.gain_failure_files = structuredClone(record.available_files || record.files || {});
        evidence.gain_failure_record_state = 'ready';
    }
    return {code:'GAIN_CALIBRATION_FAILED', title:busyText('执行器增益标定未通过','Actuator gain calibration did not pass'), message:localizedError(status.error), technical:status.error,
        diagnostic_id:`gain:${status.task_id || 'unknown'}:${status.error}`, gain_evidence:evidence,
        details:{task_id:status.task_id, gain_probe_diagnostic:status.gain_probe_diagnostic || null}, advice:busyText('查看未通过用途的实测结果和原始记录，再决定是否调整配置或重新标定','Review the failed gain uses and their recordings before adjusting the configuration or recalibrating')};
}
function gainProbeStatusHtml(status = {}, prefix = 'gain-probe') {
    const en = state.language === 'en', value = gainProbeStatusValues(status);
    const fields = [['use','当前用途','Current use'],['joint','测量关节','Measured joints'],['phase','测量阶段','Measurement stage'],['candidate','候选轮次','Candidate trial'],['paths','整轨回放 / 趟数上限','Path pass / pass limit'],['budget','自动阶段用时 / 时限','Automatic-stage elapsed / limit'],['amplitude','局部释放幅度','Local release amplitude'],['frequency','局部探测频率','Local probe frequencies'],['gains','当前关节 Kp / Kd','Current joint Kp / Kd'],['windows','已完成窗 / 计划上限','Completed windows / plan limit']];
    return `<div id="${prefix}" class="gain-probe-status" ${status.workflow_kind === 'gain' ? '' : 'hidden'}><dl class="gain-probe-metrics">${fields.map(([key,zh,english])=>`<div><dt>${en ? english : zh}</dt><dd id="${prefix}-${key}">${esc(value[key])}</dd></div>`).join('')}</dl><div class="gain-probe-window"><span>${en ? 'Current window' : '当前测量窗'}</span><b id="${prefix}-window">${esc(value.window)}</b></div><progress id="${prefix}-progress" max="1" ${value.fraction === null ? '' : `value="${value.fraction}"`} aria-label="${en ? 'Current measurement window' : '当前测量窗进度'}"></progress><p id="${prefix}-message" class="hint">${esc(value.message)}</p><p id="${prefix}-error" class="workspace-error" ${value.error ? '' : 'hidden'}>${esc(value.error)}</p><details class="gain-probe-evidence"><summary>${en ? 'Current measurement window' : '当前测量窗'}</summary><div id="${prefix}-diagnostic">${gainProbeEvidenceHtml(status)}</div></details></div>`;
}
function refreshGainProbeStatus(status = state.telemetry?.model_calibration || {}) {
    for (const prefix of ['gain-probe','gain-task-probe']) {
        const block = $(prefix); if (!block) continue;
        // Clearing a non-model task's modal must not erase the completed gain
        // task still shown on the calibration page underneath it.
        const current = prefix === 'gain-probe' && !status.workflow_kind ? state.telemetry?.model_calibration || status : status;
        const value = gainProbeStatusValues(current);
        block.hidden = current.workflow_kind !== 'gain';
        for (const key of ['use','joint','phase','candidate','paths','budget','amplitude','frequency','gains','windows','window','message','error']) {
            const node = $(`${prefix}-${key}`); if (node) { if (node.textContent !== value[key]) node.textContent = value[key]; if (key === 'error') node.hidden = !value.error; }
        }
        const evidence = $(`${prefix}-diagnostic`), evidenceHtml = gainProbeEvidenceHtml(current);
        if (evidence && evidence.innerHTML !== evidenceHtml) evidence.innerHTML = evidenceHtml;
        const progress = $(`${prefix}-progress`);
        if (progress) { if (value.fraction === null) progress.removeAttribute('value'); else progress.value = value.fraction; }
    }
    const current = state.telemetry?.model_calibration || status;
    const phase = $('gain-calibration-phase'); if (phase) phase.textContent = modelCalibrationPhaseText(gainCalibrationQualityWarning(current) ? 'complete' : current.phase || 'idle');
    if (state.page === 'calibration' && state.calibrationEntry === 'gain') {
        const note = document.querySelector('.workbench-model-note'), html = workspaceModelNoteHtml();
        if (note && note.innerHTML !== html) note.innerHTML = html;
    }
}
function gainCalibrationOptionsHtml(status, candidate, active) {
    const en = state.language === 'en';
    const purpose = canonicalGainPurpose(state.gainMode);
    if (purpose) state.gainMode = ({RIGID:'rigid',COMPLIANT:'compliant',DRAG:'drag',SERVO_TRACKING:'servo_tracking'})[purpose];
    const options = [['all', '全部四种用途', 'All four uses'], ['rigid', '刚性', 'Rigid'], ['compliant', '柔性', 'Compliant'], ['drag', '拖拽', 'Drag'], ['servo_tracking', '伺服跟踪', 'Servo tracking']];
    return `<section class="calibration-step"><h4>${en ? 'Automatic actuator gain calibration' : '执行器增益自动标定'}</h4><p class="hint">${en ? 'Replay the demonstrated path to measure all joints together; the automatic-stage budget is 12 times the original recorded trajectory duration' : '沿示教路径联合测量所有关节，自动阶段预算按原始轨迹时长的 12 倍计算'}</p><p class="hint">${en ? 'The plan includes candidate adjustment, independent frozen validation, stationary hold and drag release. Fully measured uses can be applied or exported' : '计划包含候选调整、固定参数后的独立测量、定位保持与拖拽释放测试；已完成测量的用途可单独应用或导出'}</p>${gainCalibrationHasLinearAxes() ? `<div class="callout">${en ? 'The loaded chain contains a prismatic joint; automatic gain probing is unavailable for this chain' : '当前控制链含移动关节，暂不支持执行器增益自动探测'}</div>` : ''}<details class="calibration-locks"><summary>${en ? 'Gain uses and reference limits' : '增益用途与参考指标'}</summary><label>${en ? 'Gain use' : '增益用途'}<select id="gain-target" ${active ? 'disabled' : ''}>${options.map(([key, zh, english]) => `<option value="${key}" ${state.gainMode === key ? 'selected' : ''}>${en ? english : zh}</option>`).join('')}</select></label>${gainRequirementFieldsHtml(active)}</details>${gainProbeStatusHtml(status)}${!calibrationTaskLoaded('gain') ? calibrationTaskGuidance('gain') : ''}${candidate ? gainCandidateReviewHtml(candidate) : gainTaskWarningsHtml(status)}</section>`;
}
function gainCheckStatus(value) {
    const en = state.language === 'en';
    if (value?.required === false && value?.status === 'not_requested') return en ? 'Not enabled for this task' : '本次未启用';
    if (value?.completed === false || value?.reason === 'not_started' || value?.status === 'not_started' || (Array.isArray(value?.metrics) && !value.metrics.length)) return en ? 'Not measured' : '未测量';
    if (value?.available === false) return en ? 'Unavailable' : '不可用';
    if (value?.passed === true || value?.validated === true || value?.status === 'passed' || value?.status === 'pass') return en ? 'Passed' : '通过';
    if (value?.passed === false || value?.validated === false || value?.status === 'failed') return en ? 'Outside reference limits' : '未达参考指标';
    return en ? 'Not tested' : '未测试';
}
function gainQualificationScopeHtml(scope) {
    const en = state.language === 'en';
    if (typeof scope === 'string') return esc(scope);
    if (!scope || typeof scope !== 'object') return en ? 'Legacy recorded scope' : '以旧记录范围为准';
    const labels = {trajectory:['轨迹测量','Trajectory measurement'],external_tip:['针尖外测','External tool tip'],contact:['接触测试','Contact test'],native_tracking:['原生跟踪','Native tracking'],admittance_servo:['导纳伺服','Admittance servo'],external_tcp:['针尖外测','External tool tip'],external_tcp_return:['针尖外部回位','External tool-tip return'],disturbance_release:['受扰释放','Disturbance release'],contact_validation:['接触测试','Contact test'],hold:['保持','Hold'],drag:['拖拽','Drag']};
    const status = value => value === true || value === 'validated' || value === 'passed' ? (en ? 'Passed' : '通过') : value === false || value === 'failed' ? (en ? 'Not passed' : '未通过') : ['pending','not_tested','unverified'].includes(value) ? (en ? 'Not tested' : '未测试') : value === 'local_multifrequency' ? (en ? 'Local multifrequency' : '局部多频测量') : value === 'demonstration_trajectory' ? (en ? 'Demonstrated path' : '示教路径') : value === 'not_requested' ? (en ? 'Not enabled for this task' : '本次未启用') : String(value ?? '—');
    return Object.entries(scope).map(([key,value]) => `${esc(labels[key]?.[en ? 1 : 0] || key)} · ${esc(typeof value === 'object' && value !== null ? gainCheckStatus(value) : status(value))}`).join('<br>');
}
function gainResponseModelsHtml(models, names = []) {
    const en = state.language === 'en', joints = Array.isArray(models) ? models : models?.joints || [];
    if (!joints.length) return `<p class="hint">${esc(models?.reason || models?.status || (en ? 'No measured response model' : '无实测响应模型'))}</p>`;
    return `<div class="table-scroll"><table class="workspace-table"><thead><tr><th>${t('joint')}</th><th>${en ? 'Measured delay ms' : '实测延迟 ms'}</th><th>${en ? 'Observed cutoff Hz' : '观测截止 Hz'}</th><th>ωn / 2π Hz</th><th>ζ</th><th>${en ? 'Held-out error' : '留出误差'}</th><th>${en ? 'Status' : '状态'}</th></tr></thead><tbody>${joints.map((joint,index) => `<tr><th>${esc(joint.joint || names[index] || String(index + 1))}</th><td>${capabilityNumber(typeof joint.effective_delay_s === 'number' ? joint.effective_delay_s * 1000 : null, 2)}</td><td>${capabilityNumber(joint.observed_cutoff_hz, 3)}</td><td>${capabilityNumber(typeof joint.natural_frequency_rad_s === 'number' ? joint.natural_frequency_rad_s / (2 * Math.PI) : null, 3)}</td><td>${capabilityNumber(joint.damping_ratio, 3)}</td><td>${capabilityNumber(joint.heldout_fit_error, 3)}</td><td>${esc(joint.status || '—')}</td></tr>`).join('')}</tbody></table></div><p class="hint">${en ? 'Observed cutoff comes from the tested frequency response; an unobserved cutoff stays unknown' : '截止频率来自实际测试频响，未观测到截止点时保留未知'}</p>`;
}
function gainTaughtPathMetricsHtml(value, names = []) {
    const en = state.language === 'en', validation = value.native_validation;
    const rows = Array.isArray(validation?.metrics) ? validation.metrics : [];
    if (!rows.length) return `<p class="hint">${en ? 'No complete path measurement is available for this use' : '本用途尚无完整路径测量结果'}</p>`;
    return `${gainCandidateParametersHtml(value,names)}<p class="hint">${en ? 'Measured on the demonstrated path with fixed gains; no frequency sweep or bandwidth identification was performed' : '以下结果来自固定增益后的示教路径测量，本次未进行扫频或带宽辨识'}</p><div class="table-scroll"><table class="workspace-table"><thead><tr><th>${t('joint')}</th><th>RMS</th><th>${en ? 'Peak error' : '峰值误差'}</th><th>${en ? 'Tail peak error' : '尾部峰值误差'}</th><th>${en ? 'Settling time' : '稳定时间'}</th><th>${en ? 'Fresh samples' : '新鲜采样'}</th></tr></thead><tbody>${rows.map((row,index)=>{const name=names[index] || String(index+1);return `<tr><th>${esc(name)}</th><td>${jointDisplayValue(row.rms_error_rad,name,'position',4)}</td><td>${jointDisplayValue(row.peak_error_rad,name,'position',4)}</td><td>${jointDisplayValue(row.tail_error_rad,name,'position',4)}</td><td>${capabilityNumber(row.settling_s,3)} s</td><td>${esc(row.measurement_samples ?? row.samples ?? '—')}</td></tr>`;}).join('')}</tbody></table></div>`;
}
function gainCandidateParametersHtml(value, names = []) {
    const en = state.language === 'en', hold = value.hold_gains;
    return `<h4>${en ? 'Measured gain parameters' : '本次测量的增益参数'}</h4><div class="table-scroll"><table class="workspace-table"><thead><tr><th>${t('joint')}</th><th>${en ? 'Tracking Kp' : '跟踪 Kp'}</th><th>${en ? 'Tracking Kd' : '跟踪 Kd'}</th>${hold ? `<th>${en ? 'Hold Kp' : '保持 Kp'}</th><th>${en ? 'Hold Kd' : '保持 Kd'}</th>` : ''}</tr></thead><tbody>${names.map((name,index)=>`<tr><th>${esc(name)}</th><td>${capabilityNumber(value.kp?.[index],3)}</td><td>${capabilityNumber(value.kd?.[index],3)}</td>${hold ? `<td>${capabilityNumber(hold.kp?.[index],3)}</td><td>${capabilityNumber(hold.kd?.[index],3)}</td>` : ''}</tr>`).join('')}</tbody></table></div>`;
}
function gainObserverAndBoundsHtml(value, names = []) {
    const en = state.language === 'en', observer = value.observer_model, bounds = value.feasible_admittance;
    const label = (item,index) => esc(item.joint || names[index] || String(index + 1));
    const observations = Array.isArray(observer) && observer.length ? `<h4>${en ? 'No-contact residual observation' : '无接触残差观测'}</h4><div class="table-scroll"><table class="workspace-table"><thead><tr><th>${t('joint')}</th><th>${en ? 'Bias' : '偏置'}</th><th>${en ? 'Noise RMS' : '噪声 RMS'}</th><th>${en ? 'Samples' : '样本数'}</th></tr></thead><tbody>${observer.map((item,index) => `<tr><th>${label(item,index)}</th><td>${jointDisplayValue(item.bias_nm,item.joint || names[index],'effort',5)}</td><td>${jointDisplayValue(item.noise_rms_nm,item.joint || names[index],'effort',5)}</td><td>${esc(item.samples ?? '—')}</td></tr>`).join('')}</tbody></table></div>` : '';
    const available = Array.isArray(bounds) && bounds.length ? `<h4>${en ? 'Local admittance feasibility estimate' : '局部导纳可行范围估计'}</h4><div class="table-scroll"><table class="workspace-table"><thead><tr><th>${t('joint')}</th><th>${en ? 'Outer frequency ceiling Hz' : '外环频率上限 Hz'}</th><th>${en ? 'Minimum virtual M' : '虚拟 M 下限'}</th><th>${en ? 'Deadband' : '死区'}</th><th>${en ? 'Contact validation' : '接触验收'}</th></tr></thead><tbody>${bounds.map((item,index) => `<tr><th>${label(item,index)}</th><td>${capabilityNumber(item.outer_frequency_max_hz,3)}</td><td>${capabilityNumber(item.mass_min,5)} ${jointMdkUnits(item.joint || names[index]).mass}</td><td>${jointDisplayValue(item.deadband_nm,item.joint || names[index],'effort',5)}</td><td>${gainCheckStatus({status:item.contact_validation})}</td></tr>`).join('')}</tbody></table></div><p class="hint">${en ? 'Estimated from no-contact frequency response and residual noise; contact response requires a separate test' : '根据无接触频响与残差噪声估计，接触响应需另行测试'}</p>` : '';
    return observations + available;
}
function gainCandidateReviewHtml(candidate) {
    const en = state.language === 'en';
    const targets = Object.entries(gainCandidateGroups(candidate)), selected = selectedGainPurposes(candidate);
    const rows = targets.map(([key,value]) => {
        const available = gainCandidateDeployable(candidate,[key]), warning = available && value.validated !== true;
        const hold = value.hold_compensation?.enabled === true ? value.hold_compensation_verification : value.hold_verification;
        return `<tr><td><input type="checkbox" data-gain-candidate-purpose="${esc(key)}" aria-label="${esc((en ? 'Use ' : '采用')+gainPurposeLabel(key))}" ${selected.includes(key) ? 'checked' : ''} ${available ? '' : 'disabled'}></td><th>${esc(gainPurposeLabel(key))}<small class="${warning ? 'warning-text' : ''}">${esc(gainCandidateAvailabilityLabel(candidate,key))}</small>${value.error && value.error !== 'not_started' ? `<div class="${available ? 'warning-text' : 'workspace-error'} gain-qualification-error">${esc(localizedError(value.error))}</div>` : ''}</th><td>${gainQualificationScopeHtml(value.qualification_scope)}</td><td>${gainCheckStatus(value.native_validation)}</td><td>${gainCheckStatus(value.admittance_validation)}</td><td>${gainCheckStatus(value.drag_validation || value.drag_verification || hold)}</td></tr>`;
    }).join('');
    const warningUses = targets.filter(([key,value]) => gainCandidateDeployable(candidate,[key]) && value.validated !== true).map(([key]) => gainPurposeLabel(key));
    const warnings = Array.isArray(candidate.warnings) ? candidate.warnings : [];
    const summary = warningUses.length || warnings.length ? `<div class="callout gain-quality-warning" role="status"><b>${warningUses.length ? (en ? 'Results outside reference limits can be applied or exported' : '未达参考指标的已测结果可直接应用或导出') : (en ? 'Measured results can be applied or exported' : '已测结果可直接应用或导出')}</b>${warningUses.length ? `<p>${esc(warningUses.join('、'))} · ${en ? 'The measured values and reference limits remain recorded' : '实测值与参考指标保持原样'}</p>` : ''}${warnings.length ? `<ul>${warnings.map(value=>`<li>${esc(gainWarningLabel(value))}</li>`).join('')}</ul>` : ''}</div>` : '';
    return `<div class="calibration-step"><h4>${en ? 'Select measured gain uses' : '选择已测量的增益用途'}</h4><p class="hint">${en ? 'Task' : '任务'} · ${esc(candidate.task_id || candidate.candidate_id || '—')} · ${candidate.validation_scope === 'independent_demonstration_trajectory' ? (en ? 'Joint path measurement and independent validation' : '全关节路径测量与独立验收') : (en ? 'Local joint probes and stationary tests' : '局部关节探测与静态测试')}</p>${summary}<div class="table-scroll"><table class="workspace-table gain-qualification-table"><thead><tr><th>${en ? 'Use' : '采用'}</th><th>${en ? 'Gain purpose' : '增益用途'}</th><th>${en ? 'Test scope' : '测试范围'}</th><th>${en ? 'Native tracking reference' : '原生跟踪指标'}</th><th>${en ? 'Admittance servo reference' : '导纳伺服指标'}</th><th>${en ? 'Hold / drag reference' : '保持 / 拖拽指标'}</th></tr></thead><tbody>${rows}</tbody></table></div><p class="hint">${en ? 'Apply and export affect the checked uses only. Save defaults uses the gains currently applied to the session. The rigid reference used during compliant measurement is retained; a mismatch with the adopted reference is reported without blocking apply or export' : '应用与导出只修改勾选用途；保存默认为当前会话实际已应用的增益；保留柔性测量时的刚性参考；与本次采用的参考不同会提示，不影响应用与导出'}</p><div class="workspace-actions"><button class="button" data-tool-export="gain" ${selected.length ? '' : 'disabled'}>${en ? 'Export selected gains' : '导出所选增益'}</button>${state.calibrationRuntimeCandidates?.gain_applied ? setupActionHtml({page:'profile_setup',group:'motion',anchor:'preview-config'},en ? 'Save applied gains as defaults' : '将已应用增益保存为默认') : ''}</div>${targets.some(([, value]) => value.native_validation || value.response_model || value.observer_model || value.feasible_admittance) ? `<details class="calibration-locks"><summary>${en ? 'Measured execution response' : '实测执行响应'}</summary>${targets.filter(([, value]) => value.native_validation || value.response_model || value.observer_model || value.feasible_admittance).map(([key, value]) => `<div><b>${esc(gainPurposeLabel(key))}</b>${candidate.validation_scope === 'independent_demonstration_trajectory' ? gainTaughtPathMetricsHtml(value,candidate.joint_names) : gainResponseModelsHtml(value.response_model, value.joint_names || candidate.joint_names)}${gainObserverAndBoundsHtml(value, value.joint_names || candidate.joint_names)}</div>`).join('')}</details>` : ''}</div>${gainPathValidationHtml(candidate)}`;
}
function gainPathValidationHtml(candidate) {
    const en = state.language === 'en', capture = state.responseCapture;
    const taughtPath = candidate?.validation_scope === 'independent_demonstration_trajectory';
    const purposes = Object.keys(gainCandidateGroups(candidate)).filter(key => key !== 'compliant_drag' && gainCandidateDeployable(candidate,[key]));
    const selected = purposes.includes(state.gainTrajectoryPurpose) ? state.gainTrajectoryPurpose : purposes[0] || '';
    const report = state.gainTrajectoryValidations?.[gainSelectionKey(candidate)+':'+selected];
    const ready = selected && capture && !capture.recording && capture.samples?.length >= 100;
    return `<section class="calibration-step" id="gain-path-validation"><h4>${taughtPath ? (en ? 'Supplementary path check' : '补充路径验收') : (en ? 'Independent path tracking check' : '实际路径独立跟踪验收')}</h4><p class="hint">${en ? 'Apply the selected gains, record an ordinary demonstration replay in Interaction tuning, then return here after stopping the recording. The check uses the actual gains, model and fresh feedback saved in that recording' : '应用所选增益，在交互调试中记录普通示教回放，结束记录后回到这里；按记录中的实际增益、模型与新鲜反馈验收'}</p><div class="workspace-actions">${setupActionHtml({page:'workbench',tab:'replay'},en ? 'Choose a replay trajectory' : '选择验证轨迹')}${setupActionHtml({page:'workbench',tab:'tuning',anchor:'response-start'},en ? 'Record replay response' : '记录回放响应')}</div><label>${en ? 'Gain use to check' : '验收增益用途'}<select id="gain-path-purpose" ${purposes.length ? '' : 'disabled'}>${purposes.map(key=>`<option value="${key}" ${key===selected?'selected':''}>${esc(gainPurposeLabel(key))}</option>`).join('')}</select></label><p class="hint">${capture?.recording ? (en ? 'Recording is still running; stop it before checking' : '当前仍在记录，请结束记录后验收') : capture?.samples?.length ? `${en ? 'Recorded feedback frames' : '已记录反馈帧'} ${capture.samples.length}` : (en ? 'No response recording yet' : '尚无响应记录')} · ${en ? 'At least 100 frames and 2 seconds are required' : '至少需要 100 帧和 2 秒连续记录'}</p><button id="gain-path-validate" class="button primary" ${ready ? '' : 'disabled'}>${en ? 'Check this recording' : '验收本次记录'}</button>${report ? `<div class="geometry-result ${report.passed?'is-pass':'is-warning'}">${report.passed ? (en ? 'Recorded path passed' : '记录路径通过验收') : (en ? 'Recorded path needs improvement' : '记录路径未通过验收')}</div><div class="table-scroll"><table class="workspace-table"><thead><tr><th>${t('joint')}</th><th>RMS</th><th>${en?'Peak':'峰值'}</th><th>${en?'Reference range':'参考范围'}</th></tr></thead><tbody>${report.joints.map(row=>`<tr><th>${esc(row.joint)}</th><td>${jointDisplayValue(row.rms_error,row.joint,'position',3,row.unit)} / ${jointDisplayValue(row.rms_limit,row.joint,'position',3,row.unit)}</td><td>${jointDisplayValue(row.peak_error,row.joint,'position',3,row.unit)} / ${jointDisplayValue(row.peak_limit,row.joint,'position',3,row.unit)}</td><td>${jointDisplayValue(row.reference_min,row.joint,'position',3,row.unit)} ~ ${jointDisplayValue(row.reference_max,row.joint,'position',3,row.unit)}</td></tr>`).join('')}</tbody></table></div><code>${esc(report.report_path)}</code>` : ''}<p class="hint">${taughtPath ? (en ? 'The automatic stage already includes an independent demonstrated-path check. Use this supplementary check for another path or load; contact tests remain separate' : '自动阶段已包含示教路径独立验收；更换路径或负载后可在这里补充记录，接触测试另行进行') : (en ? 'Results cover the recorded reference path and conditions; local probe qualification and contact tests remain separate' : '结果覆盖本次记录的参考路径与条件，局部探测资格与接触测试分别保留')}</p></section>`;
}
async function validateGainPathRecording() {
    const record = state.calibrationToolSelected.gain, capture = state.responseCapture;
    if (!record?.directory || !capture || capture.recording) throw new Error(busyText('请选择增益候选，并先结束响应记录','Select a gain candidate and stop the response recording first'));
    const purpose = $('gain-path-purpose')?.value;
    if (!purpose) throw new Error(busyText('请选择已完成测量的跟踪用途','Choose a measured tracking use'));
    const key = gainSelectionKey(record.gain_candidate)+':'+purpose;
    const report = await api.request('calibration_gain_validate_trajectory',{directory:record.directory,response_window:structuredClone(capture),purpose});
    state.gainTrajectoryValidations = {...(state.gainTrajectoryValidations || {}),[key]:report};
    refreshCalibrationOrDefer();
    return report;
}
function calibrationStandalonePanelHtml(kind) {
    const en = state.language === 'en';
    const isTeach = kind === 'teaching';
    const status = (workbenchConnected() ? state.telemetry?.model_calibration : null) || {};
    const phase = status.phase || 'idle';
    const live = workbenchConnected();
    const robotActive = state.telemetry?.robot_state === 'ACTIVE';
    const selected = state.calibrationToolSelected[kind];
    const waiting = live && phase === 'waiting_replay_confirmation';
    const active = live && modelCalibrationActive(status);
    const buttons = [];
    if (live && ['idle', 'complete', 'cancelled', 'failed'].includes(phase) && isTeach)
        buttons.push(`<button id="model-calibration-teach-start" class="button primary" ${robotActive ? '' : 'disabled'}>${en ? 'Start demonstration' : '开始拖拽示教'}</button>`);
    if (live && phase === 'teaching' && isTeach)
        buttons.push(`<button id="model-calibration-teach-stop" class="button primary">${robotActive ? (en ? 'Stop demonstration' : '结束示教') : (en ? 'Close interrupted demonstration' : '结束中断的示教并清理')}</button>`);
    if (live && isTeach && status.teaching_cleanup_pending && ['failed', 'cancelled'].includes(phase))
        buttons.push(`<button id="model-calibration-teach-cleanup" class="button">${en ? 'Finish interrupted teaching cleanup' : '清理中断示教'}</button>`);
    if (isTeach && waiting)
        buttons.push(`<button id="calibration-finish-teaching" class="button primary">${en ? 'Save trajectory and finish' : '保存轨迹并结束任务'}</button>`);
    if (kind === 'gain' && waiting)
        buttons.push(`<button id="model-calibration-replay-confirm" class="button danger" ${status.source_task_id && !calibrationAlignment(status).aligned ? 'disabled' : ''}>${en ? 'Confirm automatic calibration' : '确认开始自动标定'}</button>`);
    if (kind === 'gain' && selected && live && ['idle', 'complete', 'failed', 'cancelled'].includes(phase))
        buttons.push(`<button id="calibration-tool-import" class="button primary" ${robotActive ? '' : 'disabled'} title="${robotActive ? '' : (en ? 'Activate the robot before importing the trajectory' : '请先使能机械臂再导入轨迹')}">${en ? 'Import selected trajectory (no motion)' : '导入所选示教轨迹（不运动）'}</button>`);
    if (kind === 'gain' && live && status.workflow_kind === 'gain' && status.directory && ['complete', 'failed'].includes(phase))
        buttons.push(`<button id="calibration-gain-load-live" class="button">${en ? 'Review this gain task' : '查看本次增益候选'}</button>`);
    if (active && kind === 'gain')
        buttons.push(`<button id="model-calibration-cancel" class="button danger">${en ? 'Cancel current task' : '取消当前任务'}</button>`);
    const gainCandidate = (selected?.gain_candidate || null);
    const isInactive = state.telemetry?.robot_state === 'INACTIVE';
    const gainValidated = gainCandidateDeployable(selected?.gain_candidate,selectedGainPurposes(selected?.gain_candidate));
    const gainApplied = !!state.calibrationRuntimeCandidates?.gain_applied;
    const gainCanApply = gainValidated && isInactive && workbenchConnected() && !gainApplied;
    let gainActionHint = '';
    if (kind === 'gain') {
        buttons.push(`<button id="calibration-gain-temp-apply" class="button" ${gainCanApply ? '' : 'disabled'} title="${esc(!workbenchConnected() ? (en ? 'Connect the workspace first' : '请先连接工作台') : !gainValidated ? (en ? 'Select measured gain uses first' : '请先勾选已完成测量的用途') : gainApplied ? (en ? 'Restore the existing temporary candidate first' : '请先撤销当前临时增益') : !isInactive ? (en ? 'Park and disable the robot first' : '请先停放并失能机械臂') : '')}">${en ? 'Temporarily apply selected gains' : '临时应用所选增益'}</button>`);
        buttons.push(`<button id="calibration-gain-temp-restore" class="button" ${gainApplied && isInactive ? '' : 'disabled'}>${en ? 'Undo temporary gains' : '撤销增益临时应用'}</button>`);
        if (gainValidated && workbenchConnected() && !isInactive) {
            gainActionHint = en ? 'Applying gains requires an INACTIVE robot. Confirm the workspace is clear, then park and disable it' : '临时应用要求机械臂处于 INACTIVE 状态，请确认机械臂已被安全支撑及周围环境无障碍后先停放并失能';
        } else if (!gainValidated && status.workflow_kind === 'gain' && status.phase === 'complete') {
            gainActionHint = state.gainCandidateLoadPending ? (en ? 'Loading measured candidate' : '正在读取已测候选') :
                (state.gainCandidateLoadError || (en ? 'Please review this completed task to load its candidate' : '正在读取候选记录，必要时点击查看本次增益候选'));
        } else if (gainValidated && gainApplied) {
            gainActionHint = en ? 'A temporary gain candidate is already active. Park the robot before restoring it' : '临时增益已应用，恢复原参数前请先停放机械臂';
        }
    }
    return `<section class="workspace-section calibration-standalone-card"><div class="workspace-section-head"><div><h3>${isTeach ? (en ? 'Demonstration' : '拖拽示教') : (en ? 'Automatic actuator gain calibration' : '执行器增益自动标定')}</h3><p class="hint">${isTeach ? (en ? 'Capture once, reuse across calibration tools' : '只记录安全示教轨迹，不强制运行辨识') : (en ? 'One trajectory, four gain uses, measurements and reference metrics reported per use' : '一条轨迹，四种增益用途，分别记录实测结果与参考指标')}</p></div><span ${kind === 'gain' ? 'id="gain-calibration-phase"' : ''} class="result-badge neutral">${esc(modelCalibrationPhaseText(kind === 'gain' && gainCalibrationQualityWarning(status) ? 'complete' : phase))}</span></div>
       <div class="workspace-actions">${buttons.join('')}</div>
       ${kind === 'gain' ? gainCalibrationOptionsHtml(status, gainCandidate, active) : ''}
       ${kind === 'gain' ? `<div id="calibration-gain-action-hint" class="callout" ${gainActionHint ? '' : 'hidden'}>${esc(gainActionHint)}</div>` : ''}
       ${status.preview_error ? `<div class="callout">${state.language === 'en' ? 'Static quick preview unavailable; complete observations are retained for offline IDIM' : '静态快速预览未得到候选；完整采集继续保留，后续使用离线 IDIM 验证'}: ${esc(status.preview_error)}</div>` : ''}${status.error && kind !== 'gain' ? `<div class="callout error-callout">${esc(localizedError(status.error))}</div>` : ''}
       ${status.directory ? `<div class="task-directory">${esc(status.directory)}</div>` : ''}
       </section>${calibrationToolHistoryHtml(kind)}`;
}
function refreshGainApplyControls() {
    if (state.page !== 'calibration' || state.calibrationEntry !== 'gain') return;
    const apply = $('calibration-gain-temp-apply');
    const restore = $('calibration-gain-temp-restore');
    const hint = $('calibration-gain-action-hint');
    if (!apply || !restore || !hint) return;
    const en = state.language === 'en';
    const connected = workbenchConnected();
    const inactive = state.telemetry?.robot_state === 'INACTIVE';
    const candidate = state.calibrationToolSelected.gain?.gain_candidate;
    const selected = selectedGainPurposes(candidate);
    const validated = gainCandidateDeployable(candidate,selected);
    const applied = !!state.calibrationRuntimeCandidates?.gain_applied;
    const status = state.telemetry?.model_calibration || {};
    apply.disabled = !(connected && inactive && validated && !applied);
    restore.disabled = !(connected && inactive && applied);
    const reason = !connected ? (en ? 'Connect the workspace first' : '请先连接工作台')
        : !validated ? (en ? 'Select at least one measured gain use' : '请至少勾选一种已完成测量的增益用途')
            : applied ? (en ? 'Restore the existing temporary candidate first' : '请先撤销当前临时增益')
                : !inactive ? (en ? 'Park and disable the robot first' : '请先停放并失能机械臂') : '';
    apply.title = reason;
    const message = connected && validated && !inactive
        ? (en ? 'Applying gains requires an INACTIVE robot. Confirm the workspace is clear, then park and disable it' : '临时应用要求机械臂处于 INACTIVE 状态，请确认机械臂已被安全支撑及周围环境无障碍后先停放并失能')
        : !validated && status.workflow_kind === 'gain' && status.phase === 'complete'
            ? (state.gainCandidateLoadPending ? (en ? 'Loading measured candidate' : '正在读取已测候选')
                : state.gainCandidateLoadError || (en ? 'Please review this completed task to load its candidate' : '正在读取候选记录，必要时点击查看本次增益候选'))
            : validated && applied ? (en ? 'A temporary gain candidate is already active. Park the robot before restoring it' : '临时增益已应用，恢复原参数前请先停放机械臂') : '';
    hint.textContent = message;
    hint.hidden = !message;
    document.querySelectorAll('[data-tool-export="gain"]').forEach(button => { button.disabled = !validated; button.title = validated ? '' : busyText('请先勾选已完成测量的用途','Select measured gain uses first'); });
    const confirm = $('model-calibration-replay-confirm');
    if (confirm && gainCalibrationHasLinearAxes()) { confirm.disabled = true; confirm.title = busyText('当前增益自动探测仅支持转动关节','Automatic gain probes currently support rotary joints only'); }
}
function calibrationTaskLoaded(kind) {
    if (kind === 'dynamics') return !!(state.modelCalibrationOffline?.metadata && state.modelCalibrationDirectory);
    if (kind === 'gravity' && state.modelCalibrationOffline?.metadata && state.modelCalibrationDirectory) return true;
    if (state.calibrationToolSelected[kind]?.directory) return true;
    const live = state.telemetry?.model_calibration || {};
    return workbenchConnected() && live.workflow_kind === kind && !!live.directory;
}
function calibrationTaskGuidance(kind) {
    const en = state.language === 'en';
    if (calibrationTaskLoaded(kind)) return '';
    const labels = {
        gain: en ? 'No trajectory selected yet. Load a teaching task below or select a saved gain task, then import the trajectory before calibration' : '尚未加载任务，请先在下方任务记录中选择或加载拖拽示教轨迹，再导入轨迹进行增益标定',
        gravity: en ? 'No gravity task loaded. Connect and activate the robot to demonstrate a trajectory, or load an existing gravity task to inspect and export results' : '尚未加载重力标定任务，可连接并使能机械臂开始示教，或加载已有标定任务查看和导出结果',
        dynamics: en ? 'No saved task loaded. Load completed gravity data first, then assess the data before choosing an identification target' : '尚未加载任务，请先加载已完成的标定记录，检查数据质量后再选择辨识目标',
    };
    return `<div class="calibration-empty-guidance" role="status">${labels[kind] || ''}</div>`;
}
function wireCalibrationQuickControls() {
    document.querySelectorAll('[data-robot-connect]').forEach(button => button.onclick = () => startWorkbench());
    document.querySelectorAll('[data-workbench-action]').forEach(button => button.onclick = () => {
        const action = button.dataset.workbenchAction;
        if (controlOperationProblem(action)) return;
        if (confirmCalibrationPark(action)) return;
        return blockingOperation(actionBusyMessage(action), async () => {
            const result = await workbenchCommand(action);
            if (result === null) return;
            const snapshot = result?.robot_state ? result : await workbenchRequest('status');
            applyWorkbenchSnapshot(snapshot);
            updateWorkbenchControlState(); refreshCalibrationActionControls(); refreshGainApplyControls();
        });
    });
    updateWorkbenchControlState();
}
function calibrationQuickControlsHtml() {
    return `<div class="workspace-operation-panel">${robotOperationHtml()}</div>`;
}
// Refresh controls that depend on live robot state without replacing the panel or dialogs
function refreshCalibrationActionControls() {
    if (state.page !== 'calibration') return;
    const connected = workbenchConnected() && !workbenchRecoveryRequired();
    const robotState = connected ? state.telemetry?.robot_state : 'OFFLINE';
    const active = robotState === 'ACTIVE';
    const inactive = robotState === 'INACTIVE';
    const live = ['teaching', 'gain'].includes(state.calibrationEntry) || !state.modelCalibrationOffline;
    const phase = state.telemetry?.model_calibration?.phase || 'idle';
    const ready = ['idle', 'complete', 'cancelled', 'failed'].includes(phase);
    const en = state.language === 'en';
    const setButton = (id, enabled, reason) => {
        const button = $(id);
        if (!button) return;
        button.disabled = !enabled;
        button.title = enabled ? '' : reason;
    };
    setButton('model-calibration-teach-start', connected && active && live && ready,
        !connected ? (en ? 'Connect the workspace first' : '请先连接工作台')
            : !active ? (en ? 'Activate the robot before demonstrating' : '请先使能机械臂再开始示教')
                : (en ? 'Complete or cancel the current calibration task first' : '请先完成或取消当前标定任务'));
    setButton('calibration-tool-import', connected && active && live && ready && !!state.calibrationToolSelected.gain?.directory,
        en ? 'Connect and activate the robot before importing a saved trajectory' : '请先连接并使能机械臂再导入所选轨迹');
    setButton('model-calibration-import-trajectory', connected && active && !live && ready && !!state.modelCalibrationOffline?.can_resume,
        en ? 'Requires an active robot and idle calibration' : '需要机械臂已使能且当前无运行中的标定任务');
    setButton('model-calibration-alignment-begin', connected && active && live && phase === 'waiting_replay_confirmation',
        en ? 'Requires an active robot and pending replay' : '需要机械臂已使能并等待轨迹回放确认');
    setButton('model-calibration-replay-confirm', connected && active && live && !graphicsAvailabilityProblem() && phase === 'waiting_replay_confirmation'
        && (!state.telemetry?.model_calibration?.source_task_id || calibrationAlignment(state.telemetry.model_calibration).aligned),
        en ? 'Check robot activation and replay alignment first' : '请先使能机械臂并检查回放起点对齐状态');
    setButton('model-calibration-apply', connected && inactive && live && !modelCalibrationActive(),
        en ? 'Park the robot before applying a candidate' : '应用候选前请先停放并失能机械臂');
    setButton('model-calibration-restore', connected && inactive && live && !modelCalibrationActive(),
        en ? 'Park the robot before restoring a candidate' : '撤销候选前请先停放并失能机械臂');
    if (state.calibrationEntry === 'gravity') {
        const gravity = gravityCandidatePreviewState();
        const message = !gravity.directory ? (en ? 'Load a gravity task first' : '请先加载重力校正任务')
            : !gravity.validated ? (en ? 'Independent static validation has not passed' : '尚未通过独立静态验收')
                : !gravity.comparable ? (en ? 'Source URDF differs from the running model' : '源 URDF 与当前模型不一致，不能临时应用')
                    : gravity.historical ? (en ? 'Historical COM comparison is read-only, the live URDF fingerprint differs' : '历史候选仅供只读对照，当前 URDF 指纹不同，不能临时应用')
                        : !connected ? (en ? 'Connect the workspace first' : '请先连接工作台')
                            : !inactive ? (en ? 'Park and disable the robot first' : '请先停放并失能机械臂')
                                : gravity.applied ? (en ? 'A temporary gravity override is already applied' : '当前已有临时重力覆盖') : '';
        setButton('calibration-gravity-temp-apply', gravity.canApply, message);
        setButton('calibration-gravity-temp-restore', gravity.applied && connected && inactive,
            en ? 'Park the robot to restore the original gravity' : '请先停放并失能机械臂，再撤销临时重力');
        const hint = $('calibration-gravity-preview-hint');
        if (hint) { hint.textContent = message; hint.hidden = !message; }
    }
}
function refreshCalibrationQuickControls() {
    if ($('robot-operation')) updateWorkbenchControlState();
}
function calibrationNavigationHtml(entry, locked) {
    const en=state.language==='en';
    const gravityStatus=state.readiness?.checks?.gravity_verification;
    const gravityLabel=gravityStatus==='verified'?(en?'Verified':'已验收'):gravityStatus==='stale'?(en?'Needs review':'需复验'):(en?'Pending verification':'待验证');
    return `<div class="calibration-entry-stack"><button data-calibration-entry="gravity_verify" type="button" class="calibration-entry calibration-entry-featured ${entry === 'gravity_verify' ? 'selected' : ''}" ${locked && entry !== 'gravity_verify' ? 'disabled' : ''}><span class="calibration-entry-featured-main"><span class="calibration-entry-title">${en ? 'Gravity compensation verification' : '重力补偿验证'}</span><span>${en ? 'Controlled drag, gravity scaling and on-robot acceptance' : '受控拖拽、补偿比例验证与真机验收'}</span></span><span class="readiness-chip ${gravityStatus === 'verified' ? 'ready' : gravityStatus === 'stale' ? 'warning' : 'pending'}">${gravityLabel}</span></button><div class="calibration-entry-grid">${calibrationToolDefinitions.map(([key, zh, enName, zhDesc, enDesc]) => `<button data-calibration-entry="${key}" type="button" class="calibration-entry ${entry === key ? 'selected' : ''}" ${locked && entry !== key ? 'disabled' : ''}><span class="calibration-entry-title">${en ? enName : zh}</span><span>${en ? enDesc : zhDesc}</span></button>`).join('')}</div></div>`;
}
function calibrationPanelHtml() {
    const en = state.language === 'en';
    const verifying = state.telemetry?.gravity_verification?.active === true;
    const locked = modelCalibrationActive() || verifying;
    const entry = state.calibrationEntry;
    const gravityStatus = state.readiness?.checks?.gravity_verification;
    const gravityLabel = gravityStatus === 'verified' ? (en ? 'Verified' : '已验收') : gravityStatus === 'stale' ? (en ? 'Needs review' : '需复验') : (en ? 'Pending verification' : '待验证');
    return `<div class="workspace-panel calibration-panel">${calibrationQuickControlsHtml()}${calibrationNavigationHtml(entry,locked)}${calibrationAlignmentPanelHtml()}
      ${(state.calibrationRuntimeCandidates?.gain_applied || state.calibrationRuntimeCandidates?.gravity_applied || state.calibrationRuntimeCandidates?.dynamics_applied) ? `<section class="workspace-section"><h3>${en ? 'Active temporary candidates' : '当前临时覆盖'}</h3><p class="hint">${en ? 'Support and disable the arm, then undo temporary settings and disconnect' : '先支撑并失能，再撤销临时参数并断开'}</p><div class="workspace-actions">${[['gain', 'gain_applied', en ? 'Gains' : '增益'], ['gravity', 'gravity_applied', en ? 'Gravity' : '重力'], ['dynamics', 'dynamics_applied', en ? 'Dynamics' : '动力学']].filter(([key, flag]) => state.calibrationRuntimeCandidates?.[flag]).map(([key, flag, name]) => `<button class="button" data-candidate-undo="${key}" ${workbenchConnected() && state.telemetry?.robot_state === 'INACTIVE' ? '' : 'disabled'}>${en ? 'Undo ' : '撤销'}${name}</button>`).join('')}</div></section>` : ''}
      ${entry === 'gravity_verify' ? gravityVerificationPanelHtml() :
            entry === 'teaching' ? calibrationStandalonePanelHtml('teaching') :
                entry === 'gain' ? calibrationStandalonePanelHtml('gain') :
                    entry === 'gravity' ? modelCalibrationPanelHtml() + calibrationToolHistoryHtml('gravity') + gravityCandidatePreviewPanelHtml() :
                        advancedIdentificationPanelHtml() + calibrationToolHistoryHtml('dynamics')}
      ${entry === 'gravity' ? calibrationUnifiedResultsHtml() : ''}
      ${entry === 'gravity' ? singleCalibrationToolsHtml() : ''}
    </div>`;
}

function matrixHtml(matrix) { if (!Array.isArray(matrix)) return '—'; return `<table class="matrix-table">${matrix.map(row => `<tr>${row.map(v => `<td>${Number(v).toFixed(4)}</td>`).join('')}</tr>`).join('')}</table>`; }
function diagnosticsPanelHtml() { const x = state.telemetry || {}; const names = x.joint_names || []; const frames = x.frames || []; if (!state.selectedFrame && frames.length) state.selectedFrame = frames[0].name; const frame = frames.find(f => f.name === state.selectedFrame) || frames[0]; const yes = state.language === 'en' ? 'yes' : '是'; const no = state.language === 'en' ? 'no' : '否'; return `<div class="workspace-panel"><section class="workspace-section"><h3>${t('jointActuatorTitle')}</h3><div class="table-scroll"><table class="workspace-table"><thead><tr><th>${t('joint')}</th><th>q</th><th>dq</th><th>τ</th><th>ref</th><th>τmodel</th><th>residual</th><th>τext</th></tr></thead><tbody id="diagnostic-joint-body">${diagnosticJointRows(x)}</tbody></table></div><div class="table-scroll"><table class="workspace-table"><thead><tr><th>Actuator</th><th>${t('onlineLabel')}</th><th>${t('enabledLabel')}</th><th>${t('errorLabel')}</th><th>q</th><th>dq</th><th>τ</th></tr></thead><tbody id="diagnostic-actuator-body">${(x.actuator_info || []).map((a, i) => `<tr><td>${esc(a.name)}</td><td>${x.actuator?.online?.[i] ? yes : no}</td><td>${x.actuator?.enabled?.[i] ? yes : no}</td><td>${esc(x.actuator?.err_code?.[i] ?? '—')}</td><td>${Number(x.actuator?.pos?.[i] ?? 0).toFixed(4)}</td><td>${Number(x.actuator?.vel?.[i] ?? 0).toFixed(4)}</td><td>${Number(x.actuator?.tor?.[i] ?? 0).toFixed(4)}</td></tr>`).join('')}</tbody></table></div></section><section class="workspace-section"><h3>${t('telemetryTitle')}</h3><canvas id="workbench-chart" height="180"></canvas></section><section class="workspace-section"><div class="workspace-section-head"><h3>${t('frameTitle')}</h3><select id="diagnostic-frame">${frames.map(f => `<option ${f.name === state.selectedFrame ? 'selected' : ''}>${esc(f.name)}</option>`).join('')}</select></div><dl class="kv"><dt>${t('positionLabel')}</dt><dd>${esc(vectorText(frame?.position))}</dd><dt>${t('quaternionLabel')}</dt><dd>${esc(vectorText(frame?.quaternion))}</dd></dl></section><section class="workspace-section"><h3>${t('dynamicsTitle')}</h3><dl class="kv"><dt>${t('gravityLabel')}</dt><dd>${esc(vectorText(x.dynamics?.gravity))}</dd><dt>${t('gravityCompLabel')}</dt><dd>${esc(vectorText(x.dynamics?.gravity_compensation))}</dd><dt>${t('coriolisLabel')}</dt><dd>${esc(vectorText(x.dynamics?.coriolis))}</dd><dt>${t('inverseDynamicsLabel')}</dt><dd>${esc(vectorText(x.dynamics?.inverse_dynamics))}</dd><dt>COM</dt><dd>${esc(vectorText(x.dynamics?.center_of_mass))}</dd></dl>${matrixHtml(x.dynamics?.mass_matrix)}</section>${capabilityDiagnosticsHtml()}</div>`; }
function monitorPanelHtml() {
    const x = state.telemetry || {};
    const names = x.joint_names || [];
    const live = telemetryFresh(x);
    const stateLabel = live ? busyText('实时反馈','Live feedback') : x.valid ? busyText('反馈已过期，请检查连接','Feedback expired; check the connection') : busyText('等待实时会话','Waiting for live session');
    return `<div class="workspace-panel"><section class="workspace-section"><h3>${state.language === 'en' ? 'Current robot state' : '当前机械臂状态'}</h3><p class="hint" id="monitor-live-state">${stateLabel}</p><dl class="kv"><dt>Robot</dt><dd id="monitor-robot-state">${esc(live ? x.robot_state : '—')}</dd><dt>${state.language === 'en' ? 'Task state' : '任务状态'}</dt><dd id="monitor-impedance">${esc(live ? taskStateLabel(x.impedance_mode) : '—')}</dd><dt>${t('modelFeedforward')}</dt><dd id="monitor-feedforward">${esc(live ? x.model_feedforward_mode : '—')}</dd><dt>${state.language === 'en' ? 'Effective behavior' : '有效策略'}</dt><dd id="monitor-reference-behavior">${esc(live ? x.reference_behavior || '—' : '—')}</dd><dt>${state.language === 'en' ? 'Observer / admittance' : '观测 / 导纳'}</dt><dd id="monitor-capability-state">${live ? `${x.observer_valid === true ? '✓' : '—'} / ${x.admittance_active === true ? '✓' : '—'}` : '—'}</dd><dt>${t('feedbackAge')}</dt><dd id="monitor-age">${Number.isFinite(telemetryAgeMs(x)) ? telemetryAgeMs(x).toFixed(0) + ' ms' : '—'}</dd></dl></section><section class="workspace-section"><h3>${t('jointActuatorTitle')}</h3><div class="table-scroll"><table class="workspace-table"><thead><tr><th>${t('joint')}</th><th>q</th><th>dq</th><th>τ</th></tr></thead><tbody id="monitor-joint-body">${monitorJointRows(x,live)}</tbody></table></div></section>${capabilityDiagnosticsHtml()}</div>`;
}
function refreshWorkspaceTelemetryRows() {
    const x = state.telemetry || {};
    if (!workbenchConnected() || !x.valid) return;
    const fresh = telemetryFresh(x);
    const value = (key, raw) => { const node = $(key); if (node) node.textContent = String(raw); };
    if (state.page === 'workbench' && state.workbenchTab === 'monitor') {
        value('monitor-live-state', fresh ? busyText('实时反馈','Live feedback') : busyText('反馈已过期，请检查连接','Feedback expired; check the connection'));
        value('monitor-robot-state', fresh ? x.robot_state || '—' : '—');
        value('monitor-impedance', fresh ? taskStateLabel(x.impedance_mode) : '—');
        value('monitor-feedforward', profileOptionLabel(x.model_feedforward_mode));
        value('monitor-reference-behavior', x.reference_behavior || '—');
        value('monitor-capability-state', fresh ? `${x.observer_valid === true ? '✓' : '—'} / ${x.admittance_active === true ? '✓' : '—'}` : '—');
        value('monitor-age', Number.isFinite(telemetryAgeMs(x)) ? telemetryAgeMs(x).toFixed(0) + ' ms' : '—');
        const body = $('monitor-joint-body');
        if (body) body.innerHTML = monitorJointRows(x,fresh);
        const capabilityBody = $('monitor-capability-body');
        if (capabilityBody) capabilityBody.innerHTML = (x.joint_names || []).map((name, i) => `<tr><td>${esc(name)}</td><td>${fresh && x.observer_valid === true ? jointDisplayValue(x.joint?.tau_ext_hat?.[i],name,'effort') : '—'}</td><td>${jointDisplayValue(fresh ? x.joint?.delta_q?.[i] : null,name)}</td></tr>`).join('');
    }
    if (state.page === 'diagnostics') {
        const body = $('diagnostic-joint-body');
        if (body) body.innerHTML = diagnosticJointRows(x,fresh);
        const actuators = $('diagnostic-actuator-body');
        if (actuators) actuators.innerHTML = (x.actuator_info || []).map((a, i) => `<tr><td>${esc(a.name)}</td><td>${x.actuator?.online?.[i] ? (state.language === 'en' ? 'yes' : '是') : (state.language === 'en' ? 'no' : '否')}</td><td>${x.actuator?.enabled?.[i] ? (state.language === 'en' ? 'yes' : '是') : (state.language === 'en' ? 'no' : '否')}</td><td>${esc(x.actuator?.err_code?.[i] ?? '—')}</td>${['pos', 'vel', 'tor'].map(key => `<td>${Number(x.actuator?.[key]?.[i] ?? 0).toFixed(4)}</td>`).join('')}</tr>`).join('');
    }
}
function monitorJointRows(snapshot,fresh=telemetryFresh(snapshot)) {
    return (snapshot.joint_names || []).map((name,index)=>`<tr><td>${esc(name)}</td>${['pos','vel','tor'].map((key,kind)=>`<td>${jointDisplayValue(fresh?snapshot.joint?.[key]?.[index]:null,name,['position','velocity','effort'][kind])}</td>`).join('')}</tr>`).join('');
}
function diagnosticJointRows(snapshot,fresh=telemetryFresh(snapshot)) {
    return (snapshot.joint_names || []).map((name,index)=>`<tr><td>${esc(name)}</td>${['pos','vel','tor','ref_pos','model_feedforward','residual_raw','tau_ext_hat'].map(key=>`<td>${jointDisplayValue(fresh && (!['residual_raw','tau_ext_hat'].includes(key) || snapshot.observer_valid===true) ? snapshot.joint?.[key]?.[index] : null,name,key==='vel'?'velocity':['pos','ref_pos'].includes(key)?'position':'effort')}</td>`).join('')}</tr>`).join('');
}
function teachReplayShortError(raw) {
    if (!raw) return '';
    const en = state.language === 'en';
    const joint = String(raw).match(/\bjoint=([\w-]+)/);
    const error = String(raw).match(/\berror=([-+\d.eE]+)/);
    if (/teaching.*interrupt|recording.*interrupt|feedback.*fault|STATE_TIMEOUT|SAFETY_FAILED/i.test(raw)) return en ? 'Teaching interrupted by a control fault · retained data requires a new review before playback' : '示教因控制故障中断 · 保留数据需重新检查后才能回放';
    if (/trajectory replay soft following recovery timed out/i.test(raw)) {
        const detail = error ? jointDisplayValue(Number(error[1]),joint?.[1]) : '';
        return en ? `Playback paused: ${joint?.[1] || 'joint'} could not catch up${detail ? ` (error ${detail})` : ''}. Lower playback speed or review compliant gains` :
            `回放已停止 · ${joint?.[1] || '关节'} 跟踪滞后${detail ? ` ${detail}` : ''} · 请降低回放速度或检查柔性跟踪增益`;
    }
    if (/trajectory replay hard following limit reached/i.test(raw))
        return en ? 'Playback stopped at the hard following-error limit' : '回放已因触及硬跟踪误差上限而停止';
    return en ? 'Playback stopped · Review the error dialog for details' : '回放未完成 · 请查看错误弹窗中的详细原因';
}
function gainPurposeConfigured(purpose, tracking = true) {
    if (purpose === 'SERVO_TRACKING') return ['configured', 'verified', 'stale'].includes(state.readiness?.modes?.SERVO_TRACKING);
    const mode = purpose === 'DRAG' ? 'COMPLIANT_DRAG' : (purpose === 'COMPLIANT' ? 'COMPLIANT' : 'RIGID') + (tracking ? '_TRACKING' : '_HOLD');
    return ['configured', 'verified', 'stale'].includes(state.readiness?.modes?.[mode]);
}
function controlGainPurposeMayChange(purpose) {
    const x = state.telemetry || {};
    if (!workbenchConnected() || workbenchRecoveryRequired() || x.robot_state !== 'ACTIVE' || state.workbenchPending?.kind || state.blockingBusy || modelCalibrationActive() || ['recording', 'playing'].includes(x.teach_replay?.phase)) return false;
    if (referenceBehavior() === 'MODEL_ADMITTANCE' && !['DRAG', 'SERVO_TRACKING'].includes(purpose)) return false;
    return gainPurposeConfigured(purpose, x.impedance_mode?.endsWith('_TRACKING'));
}
function gainPurposeCommand(purpose) {
    if (!['RIGID', 'COMPLIANT', 'DRAG', 'SERVO_TRACKING'].includes(purpose)) throw new Error('invalid gain purpose');
    const tracking = state.telemetry?.impedance_mode?.endsWith('_TRACKING');
    return { mode: purpose === 'DRAG' ? 'COMPLIANT_DRAG' : (purpose === 'COMPLIANT' ? 'COMPLIANT' : 'RIGID') + (tracking ? '_TRACKING' : '_HOLD'), gain_profile: purpose };
}
async function switchWorkbenchGainPurpose(purpose) {
    if (!controlGainPurposeMayChange(purpose)) return;
    state.workbenchPending = { kind: 'gain', value: purpose }; updateWorkbenchControlState();
    try {
        return await blockingOperation(busyText('正在切换执行器增益用途', 'Switching actuator gain use'), async () => {
            const snapshot = await workbenchRequest('set_impedance_mode', gainPurposeCommand(purpose));
            applyWorkbenchSnapshot(snapshot); return snapshot;
        });
    } finally { state.workbenchPending = {kind: '', value: ''}; updateWorkbenchControlState(); }
}
function replayTrajectoryAvailable(status) {
    return status?.phase === 'ready' || (status?.phase === 'failed' && status.has_trajectory === true);
}
function refreshReplaySelection(context, status) {
    const running = ['playing', 'recording'].includes(status?.phase);
    const selection = replaySelection(context);
    const behavior = $(context + '-reference-behavior'), purpose = $(context + '-gain-purpose');
    if (behavior) behavior.disabled = running || workbenchRecoveryRequired();
    if (purpose) purpose.disabled = running || workbenchRecoveryRequired() || selection.reference_behavior === 'MODEL_ADMITTANCE';
    for (const id of context === 'teach' ? ['teach-direction', 'teach-rate', 'teach-repeats'] : ['geometry-cycles', 'geometry-dwell', 'geometry-rate']) {
        if ($(id)) $(id).disabled = running || workbenchRecoveryRequired();
    }
    if(context === 'teach') refreshTeachEndpointState();
}
function replaySelection(context) {
    const behaviorKey = context === 'geometry' ? 'geometryReplayBehavior' : 'teachReplayBehavior';
    const purposeKey = context === 'geometry' ? 'geometryGainPurpose' : 'teachGainPurpose';
    const behavior = state[behaviorKey] || referenceBehavior() || 'NATIVE_IMPEDANCE';
    const purpose = behavior === 'MODEL_ADMITTANCE' ? 'SERVO_TRACKING' : (['RIGID', 'COMPLIANT', 'SERVO_TRACKING'].includes(state[purposeKey]) ? state[purposeKey] : 'RIGID');
    return { reference_behavior: behavior, gain_profile: purpose, mode: purpose === 'COMPLIANT' ? 'COMPLIANT_TRACKING' : 'RIGID_TRACKING' };
}
function replaySelectionHtml(context, running = false) {
    const en = state.language === 'en', selected = replaySelection(context);
    const modelAvailable = (workbenchConnected() ? state.admittance?.enabled : currentAdmittanceProfile()?.enabled) === true;
    return `<label>${en ? 'Playback strategy' : '本次回放策略'}<select id="${context}-reference-behavior" ${running ? 'disabled' : ''}><option value="NATIVE_IMPEDANCE" ${selected.reference_behavior === 'NATIVE_IMPEDANCE' ? 'selected' : ''}>${en ? 'Native impedance' : '原生阻抗'}</option><option value="MODEL_ADMITTANCE" ${selected.reference_behavior === 'MODEL_ADMITTANCE' ? 'selected' : ''} ${modelAvailable ? '' : 'disabled'}>${en ? 'Model admittance' : '模型导纳'}</option></select></label><label>${en ? 'Actuator gain use' : '执行器增益用途'}<select id="${context}-gain-purpose" ${running || selected.reference_behavior === 'MODEL_ADMITTANCE' ? 'disabled' : ''}>${(selected.reference_behavior === 'MODEL_ADMITTANCE' ? ['SERVO_TRACKING'] : ['RIGID', 'COMPLIANT', 'SERVO_TRACKING']).map(purpose => `<option value="${purpose}" ${selected.gain_profile === purpose ? 'selected' : ''}>${esc(gainPurposeLabel(purpose))}</option>`).join('')}</select></label>`;
}
function wireReplaySelection(context, refresh) {
    const behavior = $(context + '-reference-behavior'), purpose = $(context + '-gain-purpose');
    if (behavior) behavior.onchange = () => {
        state[context === 'geometry' ? 'geometryReplayBehavior' : 'teachReplayBehavior'] = behavior.value;
        state[context === 'geometry' ? 'geometryGainPurpose' : 'teachGainPurpose'] = behavior.value === 'MODEL_ADMITTANCE' ? 'SERVO_TRACKING' : 'RIGID';
        refresh();
    };
    if (purpose) purpose.onchange = () => { state[context === 'geometry' ? 'geometryGainPurpose' : 'teachGainPurpose'] = purpose.value; };
}
function replayTrajectoryMetadata() {
    return state.teachTrajectories.find(item => item.id === state.teachSelectedId) || state.telemetry?.teach_replay || state.teachRuntime || {};
}
function replayStartPose(direction = state.teachDirection) {
    const trajectory = replayTrajectoryMetadata();
    return ['reverse', 'return', 'contact_cycle'].includes(direction) ? trajectory.end || null : trajectory.start || null;
}
function replayPlan(direction = state.teachDirection) {
    const trajectory=replayTrajectoryMetadata(), cyclic=['cycle','contact_cycle'].includes(direction);
    const cycles=cyclic?Number(state.teachRepeats):1, reverse=['reverse','return','contact_cycle'].includes(direction);
    const start=reverse?trajectory.end:trajectory.start;
    const end=cyclic?start:reverse?trajectory.start:trajectory.end;
    return {direction,cycles,passes:cyclic?2*cycles:1,start,end,rate:Number(state.teachRate)};
}
function replayInputProblem(direction = state.teachDirection) {
    const plan=replayPlan(direction);
    if(String(state.teachRate).trim()===''||!Number.isFinite(plan.rate)||plan.rate<.05||plan.rate>1)
        return busyText('回放速度比例必须在 0.05 到 1 之间','Playback speed must be between 0.05 and 1');
    if(['cycle','contact_cycle'].includes(direction)&&(!Number.isInteger(plan.cycles)||plan.cycles<1||plan.cycles>100))
        return busyText('往返次数必须是 1 到 100 的整数','Round-trip count must be an integer from 1 to 100');
    return '';
}
function replayEndpointState(direction = state.teachDirection) {
    const plan=replayPlan(direction), x=state.telemetry || {};
    const names=state.modelData?.native?.config?.joint_names || x.joint_names || [];
    const types=new Map((state.modelData?.model?.joints || []).map(joint=>[joint.name,joint.type]));
    const available=names.length>0&&plan.start?.length===names.length&&plan.end?.length===names.length&&[...plan.start,...plan.end].every(Number.isFinite);
    const fresh=telemetryFresh(x,150);
    const rows=names.map((name,i)=>{
        const linear=types.get(name)==='prismatic', positionLimit=linear ? .001 : .08, speedLimit=linear ? .001 : .05;
        const actual=x.joint?.pos?.[i], velocity=x.joint?.vel?.[i], error=Number.isFinite(actual)&&available?actual-plan.start[i]:null;
        return {name,unit:linear?'m':'rad',start:plan.start?.[i],end:plan.end?.[i],actual,error,velocity,positionLimit,speedLimit,
            aligned:fresh&&error!==null&&Math.abs(error)<=positionLimit&&Number.isFinite(velocity)&&Math.abs(velocity)<=speedLimit};
    });
    return {plan,available,fresh,rows,aligned:available&&fresh&&rows.every(row=>row.aligned)};
}
function replayEndpointHtml(direction = state.teachDirection) {
    const endpoint=replayEndpointState(direction), en=state.language==='en';
    if(!endpoint.available)return '<p class="hint">'+busyText('选择轨迹后显示起终点和逐轴对齐情况','Select a trajectory to inspect its endpoints and joint alignment')+'</p>';
    const summary=endpoint.plan.passes===1?busyText('本次执行 1 个单程','This playback executes one pass'):
        busyText('本次执行 '+endpoint.plan.cycles+' 个完整往返，共 '+endpoint.plan.passes+' 个单程','This playback executes '+endpoint.plan.cycles+' complete round trips, '+endpoint.plan.passes+' passes');
    return '<p class="hint teach-plan-summary">'+summary+'</p><div class="table-scroll"><table class="workspace-table"><thead><tr>'+
        ['关节','计划起点','完成后终点','实际位置','起点偏差','位置门槛','当前速度 / 门槛'].map((label,index)=>'<th>'+esc(en?['Joint','Start','Final endpoint','Measured','Start error','Position band','Speed / limit'][index]:label)+'</th>').join('')+
        '</tr></thead><tbody>'+endpoint.rows.map(row=>'<tr><td>'+esc(row.name)+'</td><td>'+jointDisplayValue(row.start,row.name)+'</td><td>'+jointDisplayValue(row.end,row.name)+'</td><td>'+jointDisplayValue(endpoint.fresh?row.actual:null,row.name)+'</td><td>'+jointDisplayValue(endpoint.fresh?row.error:null,row.name)+'</td><td>±'+jointDisplayValue(row.positionLimit,row.name)+'</td><td>'+jointDisplayValue(endpoint.fresh?row.velocity:null,row.name,'velocity')+' / '+jointDisplayValue(row.speedLimit,row.name,'velocity')+'</td></tr>').join('')+
        '</tbody></table></div><p class="hint">'+(endpoint.aligned?busyText('已对齐所选起点','Aligned with the selected starting endpoint'):endpoint.fresh?busyText('尚未对齐起点，请先填入端点草稿并在控制页预览移动','Starting endpoint is not aligned; fill the target draft, then preview and move in Control'):busyText('连接工作台并取得新鲜反馈后检查对齐','Connect the workspace and obtain fresh feedback to check alignment'))+'</p>';
}
function replayStartProblem(direction=state.teachDirection) {
    const selected=state.teachTrajectories.find(item=>item.id===state.teachSelectedId);
    const status=state.telemetry?.teach_replay || state.teachRuntime;
    if (!selected) return busyText('先选择一条轨迹', 'Select a trajectory first');
    if (selected.compatible === false) return selected.incompatibility_reason || busyText('轨迹与当前模型不匹配，仅供离线查看', 'This trajectory differs from the current model and is available for offline review only');
    const inputProblem = replayInputProblem(direction); if (inputProblem) return inputProblem;
    if (!workbenchConnected()) return busyText('连接工作台后可载入并回放', 'Connect the workspace to load and play the trajectory');
    if (workbenchRecoveryRequired()) return busyText('控制会话无响应，请先恢复连接', 'Recover the unresponsive control session first');
    if (state.modelReloadPending) return busyText('模型已修改，请重新连接后回放', 'The model changed; reconnect before playback');
    if (state.telemetry?.robot_state !== 'ACTIVE') return busyText('先使能机械臂', 'Activate the robot first');
    if (state.telemetry?.impedance_mode === 'COMPLIANT_DRAG') return busyText('先保持当前位置，退出拖拽后再回放', 'Hold the current position to leave drag mode before playback');
    if (modelCalibrationActive() || state.telemetry?.gravity_verification?.active) return busyText('先结束当前标定或重力验证任务', 'Finish the current calibration or gravity verification task first');
    if (selected.id !== state.teachLoadedId || !replayTrajectoryAvailable(status)) return busyText('将所选轨迹载入当前会话', 'Load the selected trajectory into this session');
    const selection = replaySelection('teach');
    if (selection.reference_behavior === 'MODEL_ADMITTANCE' && state.admittance?.enabled !== true)
        return busyText('当前会话未启用模型导纳，请选择原生阻抗或先配置导纳', 'Model admittance is not enabled in this session; select native impedance or configure admittance first');
    if (!gainPurposeConfigured(selection.gain_profile,true)) return busyText('先配置本次回放使用的执行器增益', 'Configure the selected playback gains first');
    const endpoint = replayEndpointState(direction);
    if (!endpoint.available) return busyText('轨迹端点数据不完整，请重新读取轨迹', 'The trajectory endpoints are incomplete; read the trajectory again');
    if (!endpoint.fresh) return busyText('等待新鲜反馈后检查起点对齐', 'Wait for fresh feedback to check starting alignment');
    if (!endpoint.aligned) return busyText('尚未对齐起点，请填入目标草稿后到控制页移动', 'The starting endpoint is not aligned; fill the target draft and move in Control');
    if (graphicsAvailabilityProblem(true)) return graphicsAvailabilityProblem(true);
    return '';
}
function replayMayStart(direction=state.teachDirection) {
    return !replayStartProblem(direction);
}
function refreshTeachEndpointState() {
    const endpoint=$('teach-endpoint-check');if(endpoint)endpoint.innerHTML=replayEndpointHtml();
    const problem=replayInputProblem(), error=$('teach-input-error');if(error)error.textContent=problem;
    for(const id of ['teach-rate','teach-repeats']) {
        const node=$(id);if(!node)continue;
        const invalid=id==='teach-rate'?(String(state.teachRate).trim()===''||!Number.isFinite(Number(state.teachRate))||Number(state.teachRate)<.05||Number(state.teachRate)>1):
            state.teachDirection==='cycle'&&(!Number.isInteger(Number(state.teachRepeats))||Number(state.teachRepeats)<1||Number(state.teachRepeats)>100);
        node.setCustomValidity(invalid?problem:'');node.setAttribute('aria-invalid',String(invalid));
    }
    const startProblem = replayStartProblem(), returnProblem = replayStartProblem('return');
    if($('teach-play')){$('teach-play').disabled=!!startProblem;$('teach-play').title=startProblem;}
    if($('teach-return')){$('teach-return').disabled=!!returnProblem;$('teach-return').title=returnProblem;}
    if($('teach-start-reason'))$('teach-start-reason').textContent=startProblem;
    if($('teach-load'))$('teach-load').disabled=!workbenchConnected()||workbenchRecoveryRequired()||!state.teachSelectedId||state.teachTrajectories.find(item=>item.id===state.teachSelectedId)?.compatible===false||modelCalibrationActive()||['playing','recording'].includes(state.telemetry?.teach_replay?.phase);
    if($('teach-fill-start'))$('teach-fill-start').disabled=state.teachTrajectories.find(item=>item.id===state.teachSelectedId)?.compatible===false||!replayEndpointState().available||!workbenchTargetFeedbackReady();
    if($('teach-confirm-start'))$('teach-confirm-start').disabled=!$('teach-released')?.checked||!replayMayStart(state.teachPreviewDirection || state.teachDirection);
}
function fillReplayEndpointDraft() {
    if(state.teachTrajectories.find(item=>item.id===state.teachSelectedId)?.compatible===false)
        throw new Error(busyText('历史轨迹与当前模型不兼容，请重新采集','The historical trajectory differs from this model; record it again'));
    const target=replayStartPose();
    if(!workbenchTargetFeedbackReady()||!Array.isArray(target)||target.length!==state.telemetry.joint_names.length||!target.every(Number.isFinite))
        throw new Error(busyText('连接工作台并取得新鲜反馈后再填入端点','Connect the workspace and obtain fresh feedback before filling the endpoint'));
    state.workbenchTargets=target.slice();state.workbenchTargetEpoch=state.workbenchEpoch;state.workbenchTargetBaseline=state.telemetry.sequence;
    state.workbenchTargetDirty=true;state.pendingTarget=null;state.pendingMotionId=null;state.commandState='edited';
    state.workbenchTab='control';render();queueWorkbenchTargetGhost();
}

function teachTimelineHtml() {
    const selected=state.teachTrajectories.find(item=>item.id===state.teachSelectedId);if(!selected)return '';
    return `<section class="teach-timeline"><h4>${busyText('整段轨迹预览','Recorded trajectory preview')}</h4><p class="hint">${selected.compatible===false ? esc(selected.incompatibility_reason || busyText('历史模型轨迹，仅显示记录值','Historical model; recorded values only')) : busyText('拖动时间轴查看记录姿态，青绿色虚影显示预览位置','Scrub through recorded poses; the teal ghost shows the preview')}</p><input id="teach-preview-time" type="range" min="0" max="1" step="any" value="${state.teachPreviewFraction}" aria-label="${busyText('轨迹预览时间','Trajectory preview time')}"><output id="teach-preview-clock"></output><div id="teach-preview-values"></div><p class="hint" id="teach-preview-error"></p></section>`;
}
function refreshTeachTimeline() {
    const frame=state.teachPosePreview;
    if(!frame || frame.id!==state.teachSelectedId || frame.epoch!==state.workbenchEpoch)return;
    if($('teach-preview-clock'))$('teach-preview-clock').textContent=`${Number(frame.time_s).toFixed(3)} / ${Number(frame.duration_s).toFixed(3)} s · ${frame.sample_index+1} / ${frame.samples}`;
    if($('teach-preview-values'))$('teach-preview-values').innerHTML=`<div class="chip-row">${frame.joint_names.map((name,i)=>`<span class="chip">${esc(name)} · ${esc(frame.compatible===false ? Number(frame.positions[i]).toPrecision(6)+' SI' : jointDisplayValue(frame.positions[i],name))}</span>`).join('')}</div>`;
}
let teachPreviewPending=null,teachPreviewWorker=false;
async function loadTeachPosePreview(fraction=0) {
    state.teachPreviewFraction=Math.max(0,Math.min(1,Number(fraction)||0));
    teachPreviewPending={id:state.teachSelectedId,epoch:state.workbenchEpoch,fraction:state.teachPreviewFraction,config:structuredClone(state.config)};
    if(teachPreviewWorker)return;
    teachPreviewWorker=true;
    try{
        while(teachPreviewPending){
            const request=teachPreviewPending;teachPreviewPending=null;if(!request.id)continue;
            try{
                const frame=await api.request('trajectory_preview',{config:request.config,id:request.id,fraction:request.fraction});
                if(request.id!==state.teachSelectedId || request.epoch!==state.workbenchEpoch)continue;
                if(teachPreviewPending)continue;
                state.teachPosePreview={...frame,id:request.id,epoch:request.epoch};refreshTeachTimeline();
                if($('teach-preview-error'))$('teach-preview-error').textContent='';
                await updateStartGhost(true);
            }catch(error){if(request.id===state.teachSelectedId && request.epoch===state.workbenchEpoch && $('teach-preview-error'))$('teach-preview-error').textContent=error.message || String(error);}
        }
    }finally{teachPreviewWorker=false;}
}
function teachReplayPanelHtml() {
    const en = state.language === 'en', tx = (zh, english) => en ? english : zh;
    const status = state.telemetry?.teach_replay || state.teachRuntime || {};
    const selected = state.teachTrajectories.find(x => x.id === state.teachSelectedId);
    const online = workbenchConnected(), robotState = state.telemetry?.robot_state;
    const active = online && robotState === 'ACTIVE', inactive = online && robotState === 'INACTIVE';
    const recording = status.phase === 'recording', playing = status.phase === 'playing', busy = recording || playing;
    const phases = { idle: tx('未开始', 'Idle'), recording: tx('正在记录', 'Recording'), ready: tx('轨迹已就绪', 'Ready'), playing: tx('正在回放', 'Playing'), failed: tx('任务失败', 'Failed') };
    const actions = items => items.map(([id, label, disabled, variant]) => `<button type="button" class="button ${variant || ''}" id="${id}" ${disabled ? 'disabled' : ''}>${label}</button>`).join('');
    const itemRows = state.teachTrajectories.map(x => `<button type="button" class="teach-trajectory-item ${state.teachSelectedId === x.id ? 'selected' : ''}" data-teach-select="${esc(x.id)}" ${busy ? 'disabled' : ''}><b>${esc(x.name)}</b>${x.compatible===false ? `<span class="warning-text">${tx('历史模型 · 仅供查看','Historical model · view only')}</span>` : ''}<small>${x.samples} ${tx('点', 'samples')} · ${Number(x.duration_s).toFixed(1)} s</small><span class="teach-path-label">${tx('文件夹位置', 'Folder')}</span><code class="teach-file-path">${esc(x.directory || '')}</code></button>`).join('') || `<p class="hint">${tx('当前机械臂尚无已保存的示教轨迹', 'No saved trajectories for this robot')}</p>`;
    return `<div class="workspace-panel teach-replay-panel">
      <section class="workspace-section"><div class="workspace-section-head"><div><h3>${tx('轨迹示教', 'Trajectory teaching')}</h3><p class="hint">${tx('使能后开始示教，使用拖拽增益记录关节轨迹', 'Activate and start teaching with drag gains to record joint motion')}</p></div><span class="result-badge neutral" id="teach-replay-phase">${esc(phases[status.phase] || phases.idle)}</span></div>
        <div class="workspace-actions">${actions([['teach-record', tx('开始示教', 'Start teaching'), !active || busy, ''], ['teach-record-stop', tx('结束记录', 'Stop recording'), !recording, ''], ['teach-save', tx('命名并保存', 'Name and save'), !online || busy || status.phase !== 'ready', '']])}</div>
        <p class="hint" id="teach-replay-error">${esc(status.error ? teachReplayShortError(status.error) : status.samples ? status.samples + ' ' + tx('点', 'samples') : '')}</p>
      </section>
      <section class="workspace-section"><div class="workspace-section-head"><div><h3>${tx('轨迹管理', 'Trajectory library')}</h3><p class="hint">${tx('示教数据按机械臂独立保存在 .serialarm/teach-replay 下', 'Trajectories are saved per robot under .serialarm/teach-replay')}</p></div></div>
        <div class="teach-storage-location"><span>${tx('轨迹存储目录', 'Storage directory')}</span><code class="teach-file-path">${esc(state.teachStorageDirectory || '—')}</code></div>
        <div class="teach-trajectory-list">${itemRows}</div>
        <div class="workspace-actions">${actions([['teach-refresh', tx('刷新', 'Refresh'), false, ''], ['teach-import', tx('导入 ZIP', 'Import ZIP'), busy, ''], ['teach-export', tx('导出 ZIP', 'Export ZIP'), !selected, ''], ['teach-rename', tx('重命名', 'Rename'), !selected, ''], ['teach-delete', tx('删除', 'Delete'), !selected || busy, '']])}</div>
        ${selected ? `<div class="teach-storage-location"><span>${tx('选中轨迹位置', 'Selected trajectory folder')}</span><code class="teach-file-path">${esc(selected.directory || '')}</code></div>` : `<p class="hint">${tx('选择轨迹可离线预览起点，连接后再载入本次会话', 'Select a trajectory for offline preview; load it into the session after connecting')}</p>`}
        ${teachTimelineHtml()}${state.teachLastFilePath ? `<div class="teach-storage-location"><span>${tx('最近导入或导出的文件位置', 'Last imported or exported file')}</span><code class="teach-file-path">${esc(state.teachLastFilePath)}</code></div>` : ''}
      </section>
      <section class="workspace-section"><h3>${tx('轨迹回放', 'Trajectory playback')}</h3><div class="workspace-actions">${actions([['teach-load',tx('载入所选轨迹到当前会话','Load selected trajectory into session'),!online||busy||!selected||selected.compatible===false||modelCalibrationActive(),''],['teach-fill-start',tx('将所选起点填入目标草稿','Fill starting endpoint into target draft'),!selected||selected.compatible===false||!workbenchTargetFeedbackReady(),'']])}</div>
        <div class="row">${replaySelectionHtml('teach', busy)}
        <label>${tx('方向', 'Direction')}<select id="teach-direction"><option value="forward" ${state.teachDirection === 'forward' ? 'selected' : ''}>${tx('正放', 'Forward')}</option><option value="reverse" ${state.teachDirection === 'reverse' ? 'selected' : ''}>${tx('倒放', 'Reverse')}</option><option value="cycle" ${state.teachDirection === 'cycle' ? 'selected' : ''}>${tx('往返循环', 'Round trip')}</option></select></label>
        <label>${tx('速度比例', 'Speed fraction')}<input type="number" id="teach-rate" min="0.05" max="1" step="0.05" value="${esc(state.teachRate)}"></label>
        <label ${state.teachDirection === 'cycle' ? '' : 'hidden'}>${tx('完整往返次数', 'Complete round trips')}<input type="number" id="teach-repeats" min="1" max="100" step="1" value="${esc(state.teachRepeats)}"></label></div>
        <div class="workspace-actions">${actions([['teach-play', tx('按方向执行', 'Play trajectory'), !replayMayStart(), 'primary'], ['teach-return', tx('沿记录轨迹回到起点', 'Return along recorded path'), !replayMayStart('return'), ''], ['teach-stop', tx('停止并保持', 'Stop and hold'), !playing, 'danger']])}</div>
        <p class="hint">${tx('本次回放显式使用所选策略与增益用途，结束后恢复原会话策略并保持', 'Playback uses its selected strategy and gain preset; the original session strategy is restored on completion and the robot holds')}</p>
        <p id="teach-start-reason" class="hint">${esc(replayStartProblem())}</p><p id="teach-input-error" class="workspace-error">${esc(replayInputProblem())}</p><div id="teach-endpoint-check">${replayEndpointHtml()}</div><p class="hint">${tx('填入端点后，在控制页预览并点击移动；返回起点沿记录的反向路径执行','After filling the endpoint, preview and move in Control; return follows the recorded reverse path')}</p>
        <p class="hint" id="teach-completed">${tx('已完成单程', 'Completed passes')} ${Number(status.completed || 0)}</p>
      </section></div>`;
}

async function refreshTeachTrajectories() {
    if (!state.config.profile) return;
    const epoch=state.workbenchEpoch,config=structuredClone(state.config),key=JSON.stringify(config);
    const listing = await api.request('trajectory_list', { config });
    if(epoch!==state.workbenchEpoch || JSON.stringify(state.config)!==key)return;
    const previous=new Map(state.teachTrajectories.map(item=>[item.id,item]));
    state.teachTrajectories = (Array.isArray(listing) ? listing : (listing.items || [])).map(item=>({...previous.get(item.id),...item}));
    state.teachStorageDirectory = Array.isArray(listing) ? '' : (listing.directory || '');
    if (!state.teachTrajectories.some(x => x.id === state.teachSelectedId)) { state.teachSelectedId = ''; state.teachLoadedId = ''; }
}
async function teachNative(method, params = {}) {
    const result = await workbenchRequest('teach_replay_' + method, params);
    if (result && typeof result === 'object') { state.teachRuntime = result; if (state.telemetry) state.telemetry.teach_replay = result; }
    return result;
}
function teachNameDialog(title, initial, handler) {
    const dialog = $('confirm-dialog'); if (!dialog || dialog.open) return;
    const tx = (zh, en) => busyText(zh, en);
    dialog.innerHTML = `<h3>${esc(title)}</h3><label>${tx('轨迹名称', 'Trajectory name')}<input id="teach-name-input" maxlength="80" value="${esc(initial || '')}"></label><p class="hint">${tx('轨迹保存在当前 Profile 的 .serialarm/teach-replay/轨迹名-时间戳 目录', 'Saved under this Profile: .serialarm/teach-replay/name-timestamp')}</p><div class="actions"><button id="teach-name-cancel" class="button">${tx('取消', 'Cancel')}</button><button id="teach-name-okay" class="button primary">${tx('确认', 'Confirm')}</button></div>`;
    dialog.showModal(); $('teach-name-cancel').onclick = () => dialog.close();
    $('teach-name-okay').onclick = () => { const name = $('teach-name-input').value.trim(); if (!name) return; dialog.close(); blockingOperation(busyText('正在保存轨迹...', 'Saving trajectory...'), () => handler(name)); };
}
function teachSafetyConfirm(direction) {
    if (graphicsAvailabilityProblem(true)) return;
    const dialog=$('confirm-dialog');if(!dialog||dialog.open)return;
    const problem=replayInputProblem(direction);
    if(problem)return toast(problem,true);
    const startProblem = replayStartProblem(direction);
    if(startProblem)return toast(startProblem,true);
    state.teachPreviewDirection=direction;updateStartGhost(true);
    const labels={return:busyText('沿记录路径返回起点','Return along recorded path'),reverse:busyText('倒放轨迹','Reverse playback'),
        cycle:busyText('完整往返循环','Complete round-trip playback'),forward:busyText('正放轨迹','Forward playback')};
    const interrupted=(state.telemetry?.teach_replay || state.teachRuntime)?.fault_interrupted===true;
    dialog.innerHTML='<h3>'+esc(labels[direction] || labels.forward)+'</h3>'+
        (interrupted?'<p class="parameter-guidance">'+busyText('这是故障中断后保留的轨迹，请核对本次路径与起点','This trajectory was retained after a fault; review its path and starting endpoint')+'</p>':'')+
        replayEndpointHtml(direction)+'<p>'+busyText('检查整条路径、负载、线缆和周围空间后确认执行','Check the complete path, payload, cables and workspace before starting')+'</p>'+
        '<p>'+busyText('回放策略 / 增益用途','Playback strategy / gain use')+' · '+esc(profileOptionLabel(replaySelection('teach').reference_behavior))+' / '+esc(gainPurposeLabel(replaySelection('teach').gain_profile))+'</p>'+
        '<label class="confirmation"><input type="checkbox" id="teach-released"> '+busyText('已确认空间安全，人员离开运动范围且手已松开','The workspace is clear, people are outside the motion area and the robot is released')+'</label>'+
        '<div class="actions"><button class="button" id="teach-confirm-cancel">'+t('cancel')+'</button><button class="button danger" id="teach-confirm-start" disabled>'+busyText('开始执行','Start playback')+'</button></div>';
    dialog.showModal();
    dialog.addEventListener('close',()=>{state.teachPreviewDirection=null;updateStartGhost(true);},{once:true});
    $('teach-confirm-cancel').onclick=()=>{dialog.close();state.teachPreviewDirection=null;updateStartGhost(true);};
    $('teach-released').onchange=()=>{ $('teach-confirm-start').disabled=!$('teach-released').checked||!replayMayStart(direction); };
    $('teach-confirm-start').onclick=()=>{
        if(!$('teach-released').checked||!replayMayStart(direction))return;
        const plan=replayPlan(direction);dialog.close();
        blockingOperation(busyText('正在启动示教轨迹','Starting trajectory'),async()=>{
            await teachNative('play',{direction,rate:plan.rate,cycles:plan.cycles,...replaySelection('teach'),confirmed:true});
            refreshTeachReplayPanel();
        });
    };
}

function refreshTeachReplayPanel() {
    if (state.page !== 'workbench' || state.workbenchTab !== 'replay') return;
    const task = document.querySelector('.workbench-task-content'); if (!task) return;
    const scrollContainer = task.closest('.content'), scroll = scrollContainer?.scrollTop || 0;
    const active = document.activeElement, selectId = active?.closest?.('.studio-select')?.querySelector('select')?.id;
    const focusedId = selectId || (task.contains(active) ? active?.id : null);
    task.innerHTML = teachReplayPanelHtml(); wireTeachReplay();
    window.serialArmStudio?.enhanceSelects();
    if (state.page === 'profile_setup') restoreProfileFields();
    if (state.setupFocus?.page === state.page) focusSetupTarget(); updateWorkbenchControlState();
    const focused = focusedId ? $(focusedId) : null;
    if (focused && !focused.disabled) (selectId ? focused.nextElementSibling : focused)?.focus({preventScroll:true});
    if (scrollContainer) scrollContainer.scrollTop = scroll;
}
function wireTeachReplay() {
    if (state.page !== 'workbench' || state.workbenchTab !== 'replay') return;
    const call = (id, fn) => { const b = $(id); if (b) b.onclick = () => fn(); };
    call('teach-refresh', () => operation(async () => { await refreshTeachTrajectories(); refreshTeachReplayPanel(); }));
    call('teach-record', () => blockingOperation(busyText('正在开始示教记录...', 'Starting trajectory recording...'), async () => { state.teachSelectedId = ''; state.teachLoadedId = ''; await teachNative('record_begin'); refreshTeachReplayPanel(); }));
    call('teach-record-stop', () => blockingOperation(busyText('正在结束记录...', 'Stopping trajectory recording...'), async () => { await teachNative('record_stop'); refreshTeachReplayPanel(); }));
    call('teach-stop', () => blockingOperation(busyText('正在停止回放...', 'Stopping trajectory playback...'), async () => { await teachNative('stop'); refreshTeachReplayPanel(); }));
    call('teach-save', () => teachNameDialog(busyText('保存示教轨迹', 'Save trajectory'), '', async name => { const r = await api.request('trajectory_save', { config: state.config, name }); await refreshTeachTrajectories(); state.teachSelectedId = r.id; state.teachLoadedId = r.id; state.teachLastFilePath = r.directory || ''; refreshTeachReplayPanel(); await updateStartGhost(true); }));
    call('teach-rename', () => { const x = state.teachTrajectories.find(x => x.id === state.teachSelectedId); if (x) teachNameDialog(busyText('重命名轨迹', 'Rename trajectory'), x.name, async name => { await api.request('trajectory_rename', { config: state.config, id: x.id, name }); await refreshTeachTrajectories(); refreshTeachReplayPanel(); }); });
    call('teach-import', () => operation(async () => { const source = await api.selectTrajectory(); if (!source) return; await blockingOperation(busyText('正在导入示教轨迹...', 'Importing trajectory...'), async () => { const r = await api.request('trajectory_import', { config: state.config, source }); await refreshTeachTrajectories(); state.teachSelectedId = r.id; state.teachLoadedId = ''; state.teachPosePreview=null;state.teachPreviewFraction=0;state.teachLastFilePath = r.directory || ''; refreshTeachReplayPanel();await loadTeachPosePreview(0); }); }));
    call('teach-export', () => artifactExport($('teach-export'), async () => { const r = await api.request('trajectory_export', { config: state.config, id: state.teachSelectedId }); state.teachLastFilePath = r.path || ''; toast(busyText('轨迹已导出至 ', 'Trajectory exported to ') + r.path); refreshTeachReplayPanel(); }));
    call('teach-delete', () => { const dialog = $('confirm-dialog'); if (!dialog || dialog.open) return; dialog.innerHTML = `<h3>${busyText('删除已保存的轨迹', 'Delete saved trajectory')}</h3><p>${busyText('将删除以下目录内的轨迹数据，该操作无法撤销', 'The trajectory data in this folder will be removed permanently')}</p><code class="teach-file-path">${esc(state.teachTrajectories.find(x => x.id === state.teachSelectedId)?.directory || '')}</code><div class="actions"><button class="button" id="teach-del-cancel">${busyText('取消', 'Cancel')}</button><button class="button danger" id="teach-del-ok">${busyText('确认删除', 'Delete')}</button></div>`; dialog.showModal(); $('teach-del-cancel').onclick = () => dialog.close(); $('teach-del-ok').onclick = () => { dialog.close(); blockingOperation(busyText('正在删除轨迹...', 'Removing trajectory...'), async () => { await api.request('trajectory_remove', { config: state.config, id: state.teachSelectedId }); await refreshTeachTrajectories(); state.teachLoadedId = ''; refreshTeachReplayPanel(); }); }; });
    document.querySelectorAll('[data-teach-select]').forEach(button=>button.onclick=()=>blockingOperation(busyText('正在读取轨迹','Reading trajectory'),async()=>{
        const id=button.dataset.teachSelect, epoch=state.workbenchEpoch;
        const metadata=await api.request('trajectory_select',{config:state.config,id});
        if(epoch!==state.workbenchEpoch)return;
        state.teachTrajectories=state.teachTrajectories.map(item=>item.id===id?{...item,...metadata,id}:item);
        state.teachLoadedId='';
        state.teachSelectedId=id;state.teachPosePreview=null;state.teachPreviewFraction=0;refreshTeachReplayPanel();await loadTeachPosePreview(0);
    }));
    call('teach-load',()=>blockingOperation(busyText('正在载入轨迹到当前会话','Loading trajectory into session'),async()=>{
        if(!workbenchConnected())throw new Error(busyText('请先连接工作台','Connect the workspace first'));
        const id=state.teachSelectedId,epoch=state.workbenchEpoch;
        if(!id)throw new Error(busyText('请选择已保存轨迹','Select a saved trajectory'));
        if(state.teachTrajectories.find(item=>item.id===id)?.compatible===false)throw new Error(busyText('历史轨迹与当前模型不兼容，请重新采集','The historical trajectory differs from this model; record it again'));
        const result=await api.request('trajectory_load',{config:state.config,id});
        if(epoch!==state.workbenchEpoch||id!==state.teachSelectedId)return;
        state.teachLoadedId=id;
        state.teachRuntime=result?.phase?result:await teachNative('status');
        if(state.telemetry)state.telemetry.teach_replay=state.teachRuntime;
        refreshTeachReplayPanel();await updateStartGhost(true);
    }));
    call('teach-fill-start',()=>operation(async()=>fillReplayEndpointDraft()));
    const direction = $('teach-direction'), rate = $('teach-rate'), repeats = $('teach-repeats');
    wireReplaySelection('teach', () => refreshTeachReplayPanel());
    if (direction) direction.onchange = () => { state.teachDirection = direction.value; state.teachPreviewDirection = null; state.teachPosePreview=null; refreshTeachReplayPanel(); loadTeachPosePreview(direction.value==='reverse' ? 1 : 0); };
    if (rate) rate.oninput = rate.onchange = () => { state.teachRate = rate.value; refreshTeachEndpointState(); };
    if (repeats) repeats.oninput = repeats.onchange = () => { state.teachRepeats = repeats.value; refreshTeachEndpointState(); };
    call('teach-play', () => teachSafetyConfirm(state.teachDirection));
    call('teach-return', () => teachSafetyConfirm('return'));
    if($('teach-preview-time'))$('teach-preview-time').oninput=event=>loadTeachPosePreview(event.target.value);
    refreshTeachTimeline();
    refreshTeachEndpointState();
}
let ghostGeneration = 0, ghostPending = null, ghostActive = null, ghostPump = null;
let ghostDrawnView = null, ghostDrawnAt = 0, ghostDrawnKey = '';
function gainProbeGhostSource(status = state.telemetry?.model_calibration) {
    if (status?.workflow_kind !== 'gain' || status.phase !== 'gain_adaptation') return null;
    const q = status.gain_probe_reference_joint_positions, names = state.telemetry?.joint_names || [];
    if (!Array.isArray(q) || !q.length || q.length !== names.length || !q.every(value => typeof value === 'number' && Number.isFinite(value))) return null;
    return {q:q.slice(), context:JSON.stringify([status.task_id, status.gain_probe_kind, status.gain_probe_joint, status.gain_candidate_index, status.gain_adaptation_passes])};
}
function calibrationStartGhostSource(status = state.telemetry?.model_calibration) {
    if (state.modelCalibrationOffline || !status) return null;
    const q = status.replay_start_joint_positions, names = state.telemetry?.joint_names || state.modelData?.native?.config?.joint_names || [];
    const waiting = status.phase === 'waiting_replay_confirmation';
    if (status.workflow_kind === 'gain' && !waiting) return null;
    const currentWorkflow = !status.workflow_kind || status.workflow_kind === state.calibrationEntry;
    if ((!waiting && !currentWorkflow) || ['idle','teaching','complete','cancelled','failed'].includes(status.phase) || !Array.isArray(q) || !q.length || q.length !== names.length || !q.every(Number.isFinite)) return null;
    return q.slice();
}
function currentLiveGhostSource(snapshot = state.telemetry) {
    if (!['calibration','geometry'].includes(state.page)) return null;
    const status = snapshot?.model_calibration;
    if (state.page === 'calibration' && state.calibrationEntry === 'gain' && status?.workflow_kind === 'gain' && status.phase === 'gain_adaptation') {
        const probe = gainProbeGhostSource(status);
        return probe ? {...probe, probe:true} : {q:null,context:''};
    }
    let context = null;
    if (state.page === 'calibration' && (status?.workflow_kind !== 'gain' || status.phase === 'paused') && ['static_reverse','static_forward','gravity_fitting','friction_reverse_slow','friction_forward_slow','friction_reverse_fast','friction_forward_fast','friction_fitting','paused'].includes(status?.phase))
        context = ['model-reference',status.task_id,status.phase];
    const legacy = snapshot?.calibration;
    if (!context && state.page === 'calibration' && ['friction_recording','friction_replaying'].includes(legacy?.kind) && ['replaying','fitting'].includes(legacy.phase))
        context = ['friction-reference',legacy.task_id,legacy.kind,legacy.phase];
    const replay = snapshot?.teach_replay;
    if (!context && state.page === 'geometry' && state.geometryTab === 'repeat' && replay?.phase === 'playing')
        context = ['geometry-reference',replay.task_id,replay.plan?.kind,replay.phase];
    if (!context) return null;
    const names = snapshot?.joint_names || [], valid = q => Array.isArray(q) && q.length === names.length && q.length > 0 && q.every(Number.isFinite);
    const nominal = snapshot?.joint?.nominal_ref_pos, reference = snapshot?.joint?.ref_pos;
    const q = telemetryFresh(snapshot,500) ? valid(nominal) ? nominal : valid(reference) ? reference : null : null;
    return {q:q?.slice() || null,context:JSON.stringify(context),probe:false};
}
function ghostContextCurrent(job) {
    return job.generation === ghostGeneration && modelView === job.view && job.view.payload === job.payload && state.modelData === job.payload
        && state.workbenchEpoch === job.epoch && state.page === job.page
        && state.workbenchTab === job.tab && state.calibrationEntry === job.entry
        && (!job.probeContext || currentLiveGhostSource()?.context === job.probeContext);
}
function pumpStartGhost() {
    if (ghostPump) return;
    ghostPump = (async () => {
        while (ghostPending) {
            const job = ghostPending; ghostPending = null; ghostActive = job;
            try {
                if (!ghostContextCurrent(job)) continue;
                if (job.view.mode === 'compatibility' && ghostDrawnView === job.view) {
                    const wait = 1000 / 30 - (performance.now() - ghostDrawnAt);
                    if (wait > 0) await new Promise(resolve => setTimeout(resolve, wait));
                    if (!ghostContextCurrent(job)) continue;
                }
                const native = await api.request('model_preview', { config: job.config, positions: job.q });
                if (!ghostContextCurrent(job) || !state.ghostRequestKey) continue;
                const targetCurrent = job.page !== 'workbench' || job.tab !== 'control' ||
                    state.workbenchTargetEpoch === job.epoch && JSON.stringify(job.q) === JSON.stringify(state.workbenchTargets);
                const probeCurrent = !job.probeContext || JSON.stringify(job.q) === JSON.stringify(currentLiveGhostSource()?.q);
                const latest = job.key === state.ghostRequestKey && targetCurrent && probeCurrent;
                // Show completed samples during a continuous drag; only the exact
                // current draft can authorize Move. Requests never overtake one another.
                if (latest || job.probeContext || job.page === 'workbench' && job.tab === 'control') {
                    job.view.setGhostPose?.(native);
                    const drawn = job.view.renderOnce?.();
                    if (drawn) { ghostDrawnView = job.view; ghostDrawnAt = performance.now(); ghostDrawnKey = latest ? job.key : ''; }
                    graphicsState.ghostKey = !job.probeContext && latest && drawn ? job.key : '';
                    refreshGraphicsDependentActions();
                }
            } catch (error) {
                if (ghostContextCurrent(job) && job.key === state.ghostRequestKey) {
                    graphicsState.ghostKey = ''; ghostDrawnKey = ''; job.view.clearGhostPose?.(); refreshGraphicsDependentActions();
                }
            } finally {
                ghostActive = null; job.resolve();
            }
        }
    })().finally(() => { ghostPump = null; if (ghostPending) pumpStartGhost(); });
}
async function updateStartGhost(force = false) {
    if (!modelView || !['workbench', 'calibration', 'geometry'].includes(state.page)) return;
    let q = null, probeContext = '';
    const liveReference = currentLiveGhostSource();
    if (liveReference) {
        q = liveReference.q; probeContext = liveReference.context;
    } else if (state.page === 'workbench' && state.workbenchTab === 'control') {
        q = state.workbenchTargetEpoch === state.workbenchEpoch && !workbenchTargetProblem() ? state.workbenchTargets.slice() : null;
    } else if (state.page === 'workbench' && state.workbenchTab === 'replay') {
        const selected=state.teachTrajectories.find(item=>item.id===state.teachSelectedId), frame=state.teachPosePreview;
        const currentFrame=frame?.id===state.teachSelectedId && frame?.epoch===state.workbenchEpoch;
        q = selected?.compatible===false || currentFrame && frame.model_preview_available===false ? null : !state.teachPreviewDirection && currentFrame && frame?.model_preview_available ? frame.positions : replayStartPose(state.teachPreviewDirection || state.teachDirection);
    } else if (state.page === 'calibration') {
        q = calibrationStartGhostSource();
    }
    const note = document.querySelector('.workbench-model-note');
    if (note) { const html = workspaceModelNoteHtml(); if (note.innerHTML !== html) note.innerHTML = html; }
    const view = modelView, epoch = state.workbenchEpoch;
    const key = q ? `${state.config.profile}:${state.page}:${state.workbenchTab}:${state.calibrationEntry}:${epoch}:${probeContext ? probeContext + ':' : ''}${JSON.stringify(q)}` : '';
    if (!q) {
        ++ghostGeneration; ghostPending?.resolve(); ghostPending = null;
        state.ghostRequestKey = ''; graphicsState.ghostKey = ''; ghostDrawnKey = ''; view.clearGhostPose?.();
        return;
    }
    const same = job => job?.key === key && ghostContextCurrent(job);
    if (same(ghostPending)) return ghostPending.promise;
    if (same(ghostActive)) {
        // A drag can return to the in-flight pose before it finishes.
        ghostPending?.resolve(); ghostPending = null; state.ghostRequestKey = key;
        return ghostActive.promise;
    }
    if (!force && key === state.ghostRequestKey && (probeContext ? ghostDrawnView === view && ghostDrawnKey === key : graphicsState.ghostKey === key)) return;
    state.ghostRequestKey = key; graphicsState.ghostKey = '';
    ghostPending?.resolve();
    const job = {view, payload:view.payload, epoch, page:state.page, tab:state.workbenchTab,
        entry:state.calibrationEntry, generation:ghostGeneration, config:{...state.config}, q, key, probeContext};
    job.promise = new Promise(resolve => { job.resolve = resolve; });
    ghostPending = job; pumpStartGhost();
    return job.promise;
}

function workbenchPanelHtml() { if (state.workbenchTab === 'replay') return teachReplayPanelHtml(); if (state.workbenchTab === 'tuning') return tuningPanelHtml(); if (state.workbenchTab === 'monitor') return monitorPanelHtml(); return controlPanelHtml(); }
function gravityCandidatePreviewPanelHtml() {
    const en = state.language === 'en';
    const candidate = gravityCandidatePreviewState();
    const viewSource = candidate.offline ? (en ? 'Offline mass and COM candidate' : '离线质量与质心候选') : (en ? 'Online COM candidate' : '在线质心候选');
    const reason = !candidate.directory ? (en ? 'Load a gravity task first' : '请先加载重力校正任务')
        : !candidate.validated ? (en ? 'Independent static validation has not passed' : '尚未通过独立静态验收')
            : !candidate.comparable ? (en ? 'Source URDF differs from the running model' : '源 URDF 与当前模型不一致，不能临时应用')
                : candidate.historical ? (en ? 'Historical COM comparison is read-only, the live URDF fingerprint differs' : '历史候选仅供只读对照，当前 URDF 指纹不同，不能临时应用')
                    : !workbenchConnected() ? (en ? 'Connect the workspace first' : '请先连接工作台')
                        : state.telemetry?.robot_state !== 'INACTIVE' ? (en ? 'Park and disable the robot first' : '请先停放并失能机械臂')
                            : candidate.applied ? (en ? 'A temporary gravity override is already applied' : '当前已有临时重力覆盖') : '';
    const description = candidate.offline
        ? (en ? 'COM markers on the left show the offline result. A temporary gravity-only preview changes gravity masses and first moments, leaving dynamic inertia unchanged. It never writes the original URDF or YAML'
            : '左侧质心标记显示离线候选 · 临时应用替换重力质量与一阶矩，不修改动力学惯性张量，也不覆盖原始 URDF 和 YAML')
        : (en ? 'The left model shows the candidate COM. Temporary application changes gravity only and leaves the original files untouched'
            : '左侧模型显示候选质心 · 临时应用只调整重力项，不覆盖原始文件');
    return `<section class="workspace-section"><div class="workspace-section-head"><div><h3>${en ? 'Gravity candidate preview' : '重力候选预览'}</h3><p class="hint">${viewSource}</p></div><span class="result-badge ${candidate.validated ? 'pass' : 'warn'}">${candidate.validated ? (en ? 'Static check passed' : '静态评价通过') : (en ? 'Needs review' : '尚待审核')}</span></div><div class="calibration-muted-note">${description}</div><div class="workspace-actions"><button id="calibration-gravity-temp-apply" class="button" ${candidate.canApply ? '' : 'disabled'} title="${esc(reason)}">${en ? 'Apply gravity-only preview (INACTIVE)' : '临时应用候选重力项'}</button><button id="calibration-gravity-temp-restore" class="button" ${candidate.applied && workbenchConnected() && state.telemetry?.robot_state === 'INACTIVE' ? '' : 'disabled'}>${en ? 'Undo temporary gravity' : '撤销重力临时应用'}</button></div>${reason ? `<p id="calibration-gravity-preview-hint" class="hint">${esc(reason)}</p>` : `<p id="calibration-gravity-preview-hint" class="hint" hidden></p>`}</section>`;
}
function calibrationComOverlayHtml() {
    if (state.page !== 'calibration' || !['gravity', 'dynamics'].includes(state.calibrationEntry)) return '';
    const binding = modelCalibrationModelBinding();
    const data = currentModelCalibration()?.gravity_result;
    const hasComparison = binding.comparable && (state.calibrationEntry==='dynamics' ? state.idimReport?.physical_pass===true : Array.isArray(data?.first_moments) && data.first_moments.length > 0);
    const scale = state.modelCalibrationDisplayScale || 1;
    const title = state.calibrationEntry==='dynamics' ? (state.language==='en'?'Full-model candidate COM':'完整模型候选质心对照') : (state.language === 'en' ? 'Gravity COM comparison' : '重力质心对比');
    const pending = state.language === 'en' ? 'Available once a model candidate is fitted' : '完成重力候选拟合后自动显示';
    const original = binding.historical ? (state.language === 'en' ? 'Current URDF COM' : '当前 URDF 质心') : (state.language === 'en' ? 'Original COM' : '原始质心');
    const candidate = binding.historical ? (state.language === 'en' ? 'Historical candidate COM' : '历史候选质心') : (state.language === 'en' ? 'Candidate COM' : '候选质心');
    const zoom = state.language === 'en' ? 'Visual displacement only' : '仅放大显示位移，不更改实际坐标';
    return `<div class="calibration-com-overlay" aria-label="${title}"><div class="calibration-com-heading">${title}</div><div class="calibration-com-legend"><span><i class="com-swatch original"></i>${original}</span><span><i class="com-swatch candidate"></i>${candidate}</span><span class="com-connector">─</span>${state.language === 'en' ? 'Offset' : '质心偏移'}</div>${hasComparison ? `<div class="calibration-com-scale"><span>${state.language === 'en' ? 'Display' : '显示比例'}</span>${[1, 12, 25].map(value => `<button type="button" class="${scale === value ? 'active' : ''}" data-calibration-com-scale="${value}" aria-pressed="${scale === value}">×${value}</button>`).join('')}</div><label class="calibration-com-unchanged"><input type="checkbox" id="calibration-com-unchanged" ${state.modelCalibrationShowUnchanged ? 'checked' : ''}>${state.language === 'en' ? 'Show unchanged links' : '显示未变化的 Link'}</label><div class="calibration-com-hint">${scale === 1 ? (state.language === 'en' ? 'True 3D locations' : '真实三维位置') : zoom}</div>` : `<div class="calibration-com-hint">${binding.comparable ? pending : esc(binding.reason)}</div>`}</div>`;
}
function synchronizeWorkbenchSession(session) {
    const identity=session.session_id || session.pid;
    const running=['running','stopping'].includes(session.state);
    if(!running && state.workbenchIdentity && identity && identity!==state.workbenchIdentity)return false;
    if((running && identity!==state.workbenchIdentity) || (!running && state.workbenchIdentity)) {
        state.profileMissingTarget=null;
    state.workbenchEpoch=(state.workbenchEpoch || 0)+1;
        if (state.responseCapture?.recording) { state.responseCapture.recording = false; state.responseCapture.stop_reason = busyText('记录停止：工作台会话结束或变更', 'Stopped: workspace session ended or changed'); }
        state.telemetry=null;
        state.telemetryHistory=[];
        state.telemetryReceivedAt=0;
        state.admittance=null;
        state.tuneDraft=null;
        state.runtimeInteractionGoal=null;
        state.savePreview=null;
        state.calibrationRuntimeCandidates=null;
        state.workbenchTargets=[];
        state.workbenchTargetEpoch=null;
        state.workbenchTargetBaseline=null;
        state.workbenchTargetDirty=false;
        state.pendingTarget=null;state.pendingMotionId=null;state.speedInputRaw=null;state.speedInputError='';
        if(!running) state.modelReloadPending=false;
        state.teachReplayBehavior=null; state.geometryReplayBehavior=null; state.teachPreviewDirection=null;
        state.ghostRequestKey='';
        ghostGeneration++;
        modelView?.clearGhostPose?.();
        if(!state.modelCalibrationOffline)state.modelCalibrationDirectory='';
        state.confirmedActivationPid=null;
        state.commandState='idle';
        lastRuntimeFaultKey='';
        lastTeachReplayErrorKey='';
        reportedCalibrationErrors.clear();
        state.workbenchIdentity=running?identity:null;
    }
    state.session=session;
    return true;
}
function workbenchRecoveryRequired() { return workbenchConnected() && state.session?.recovery_required === true; }
function workbenchForceDisconnectAvailable() {
    return workbenchConnected() && (workbenchRecoveryRequired() ||
        state.session?.disconnect_blocked === true || state.telemetry?.robot_state === 'FAULT' || state.parkingBusy === true);
}
function workbenchRecoveryNotice() {
    if (!workbenchRecoveryRequired()) return '';
    const en = state.language === 'en';
    return `<section class="workspace-section" role="alert"><h3>${en ? 'Control session not responding' : '控制会话无响应'}</h3><p class="hint">${en ? 'The previous hardware command timed out. It may still be executing, and the displayed joint state may be stale. Do not repeat enable or disable. Secure the robot using physical safety measures, then use Disconnect workspace in the top toolbar to confirm disconnection and reconnect' : '上一条硬件命令已超时，原生命令可能仍在执行，界面关节反馈可能过期，请勿重复使能或失能；先采取物理安全措施，再使用顶部“断开工作台”确认断连并重新连接'}</p></section>`;
}
function workspaceSessionActionHtml() {
    const connected = workbenchConnected();
    if (!connected) return `<button id="start-workbench" class="button primary">${t('startWorkbench')}</button>`;
    return `<button id="stop-workbench" class="button danger">${state.language === 'en' ? 'Disconnect workspace' : '断开工作台'}</button>`;
}
function workspaceModelNoteHtml() {
    const base = t(workbenchConnected() ? 'actualTelemetry' : 'offlineWaiting');
    const liveReference = currentLiveGhostSource();
    if (liveReference) {
        return base + (liveReference.q ? `<span class="teach-ghost-legend">${liveReference.probe ? busyText('当前探测目标 · 青绿色半透明', 'Current probe target · translucent teal') : busyText('当前参考位姿 · 青绿色半透明', 'Current reference pose · translucent teal')}</span>` : '');
    }
    const target = state.page === 'workbench' && state.workbenchTab === 'control';
    const start = (state.page === 'workbench' && state.workbenchTab === 'replay') || (state.page === 'calibration' && !!calibrationStartGhostSource());
    return base + (target ? `<span class="teach-ghost-legend">${busyText('目标草稿 · 青绿色半透明', 'Target draft · translucent teal')}</span>` : start ? `<span class="teach-ghost-legend">${busyText('起始位姿 · 青绿色半透明', 'Start pose · translucent teal')}</span>` : '');
}
function workbenchPage() {
    return pageHead('workbenchTitle', 'workbenchDesc', workspaceSessionActionHtml()) + workbenchStatusBar() + `<div id="workspace-recovery-notice">${workbenchRecoveryNotice()}</div><div class="workbench-layout"><section class="workbench-model"><div id="workbench-viewport">${state.modelData ? '' : `<div class="empty-state">${t('modelNotLoaded')}</div>`}</div><div class="workbench-model-note">${workspaceModelNoteHtml()}</div></section><section class="workbench-task"><div class="workspace-panel workspace-operation-panel">${robotOperationHtml()}</div><div class="workspace-panel">${workbenchTabs()}</div><div class="workbench-task-content">${workbenchPanelHtml()}</div></section></div>`;
}

async function geometryCommand(method, params = {}) {
    const response = await api.request('geometry_fixed_' + method, { config: state.config, ...params });
    if (response && typeof response === 'object' && 'active' in response) { state.geometryState=response;if(response.active){state.geometryFrame=response.parent_frame;state.geometryPurpose=response.purpose;if(response.report){state.geometryRms=response.report.limits_mm?.rms ?? state.geometryRms;state.geometryMax=response.report.limits_mm?.max ?? state.geometryMax;}} }
    return response;
}
async function geometryRepeatRequest(op, params = {}) {
    const response = await api.request('geometry_repeat_' + op, { config: state.config, ...params });
    if (response && typeof response === 'object' && typeof response.phase === 'string') {
        state.teachRuntime = response;
        if (state.telemetry) state.telemetry.teach_replay = response;
    }
    if (op === 'status') {
        state.geometryRepeatSupported = response.supported === true;
        state.geometryRepeatError = response.reason || '';
        state.geometryContactReady = response.contact_ready === true;
    }
    return response;
}
async function refreshGeometryRepeatStatus() {
    if (!workbenchConnected()) {
        state.geometryRepeatSupported = null; state.geometryRepeatError = ''; state.geometryContactReady = false;
        return;
    }
    try { await geometryRepeatRequest('status'); }
    catch (error) { state.geometryRepeatSupported = false; state.geometryRepeatError = String(error?.message || error); state.geometryContactReady = false; }
}
let geometryTcpCache = {key:'', preview:null, output:null};
function geometryTcpSessionActive() { return ['running', 'stopping'].includes(state.session?.state); }
function geometryTcpViewState() {
    const fixed = state.geometryState || {};
    const key = JSON.stringify([state.config.profile_file, state.config.profile, state.info?.fingerprint,
        fixed.directory, fixed.parent_frame, fixed.purpose, fixed.candidate, fixed.report]);
    if (geometryTcpCache.key !== key) geometryTcpCache = {key, preview:null, output:null};
    return geometryTcpCache;
}
function geometryTcpSummaryHtml() {
    const {preview, output} = geometryTcpViewState();
    if (!preview) return '';
    const tr = (zh, en) => busyText(zh, en);
    const change = (preview.change_summary || []).find(item => item.field === 'core.model.tool_frame');
    const operation = preview.operation === 'update' ? tr('更新已有工具固定关节','Update the existing fixed tool joint') : tr('新增工具固定关节','Create a fixed tool joint');
    const orientation = (preview.tcp_rpy_rad || [0,0,0]).map((value,index)=>`${['R','P','Y'][index]} ${(Number(value)*180/Math.PI).toFixed(3)}°`).join(' · ');
    return `<div class="geometry-step" id="geometry-tcp-diff"><b>${tr('TCP 配置变更', 'TCP configuration changes')}</b><p>${operation}</p><dl class="kv"><dt>${tr('采样参考坐标系','Sampling frame')}</dt><dd>${esc(preview.sample_parent_frame || preview.parent_frame)}</dd><dt>${tr('写入的父坐标系', 'Exported parent frame')}</dt><dd>${esc(preview.parent_frame)}</dd><dt>${tr('工具坐标系', 'Tool frame')}</dt><dd>${esc(change?.before || '—')} → ${esc(preview.tool_frame)}</dd><dt>${tr('针尖位置', 'Tip position')}</dt><dd>${(preview.tcp_xyz_m || []).map((value,i)=>`${'XYZ'[i]} ${(Number(value)*1000).toFixed(3)} mm`).join(' · ')}</dd><dt>${tr('工具朝向 RPY', 'Tool orientation RPY')}</dt><dd>${orientation}</dd></dl><p class="hint">${tr('位置已换算到写入的父坐标系，朝向沿用采样参考坐标系；固定点实验只标定平移', 'Position is expressed in the exported parent frame; orientation follows the sampling frame. Fixed-point calibration identifies translation')}</p><p class="hint">${tr('导出独立 URDF、Core、硬件配置、Profile 和网格；采用后新增一份机械臂配置，保留原文件', 'Export an independent URDF, Core, hardware configuration, Profile and meshes; adoption adds a manipulator configuration and preserves the source files')}</p>${output?.bundle_directory ? `<div class="task-directory"><span>${tr('已导出候选目录', 'Exported candidate directory')}</span><code>${esc(output.bundle_directory)}</code></div>` : ''}</div>`;
}
async function geometryTcpPreview() {
    const cache = geometryTcpViewState();
    const preview = await geometryCommand('preview_tcp');
    if (geometryTcpViewState() !== cache) return null;
    cache.preview = preview;
    refreshGeometryPanel();
    return preview;
}
function wireGeometryTcp() {
    const tr = (zh, en) => busyText(zh, en);
    if ($('geometry-preview-tcp')) $('geometry-preview-tcp').onclick = () => blockingOperation(tr('正在预览 TCP 配置', 'Previewing TCP configuration'), geometryTcpPreview);
    if ($('geometry-export-tcp')) $('geometry-export-tcp').onclick = async () => {
        const destination = await api.selectResources();
        if (!destination) return;
        await artifactExport($('geometry-export-tcp'), async () => {
            const cache = geometryTcpViewState();
            const output = await geometryCommand('export_tcp', {destination});
            if (geometryTcpViewState() !== cache) return;
            cache.preview = output; cache.output = output;
            refreshGeometryPanel();
            toast(tr('已导出到 ', 'Exported to ') + (output.bundle_directory || output.directory || ''));
        });
    };
    if ($('geometry-apply-tcp')) $('geometry-apply-tcp').onclick = async () => {
        if (active()) return toast(tr('请先停放并断开会话，再采用 TCP 配置', 'Park and disconnect the session before adopting the TCP configuration'), true);
        const cache = geometryTcpViewState(), preview = cache.preview;
        if (!preview) return toast(tr('请先预览 TCP 配置差异', 'Preview the TCP configuration changes first'), true);
        if (!await safetyConfirmation(tr('采用独立 TCP 配置', 'Adopt independent TCP configuration'),
            tr('将新增并打开一份机械臂 Profile，保留原配置；新配置不会自动连接机械臂', 'A separate manipulator Profile will be added and opened, preserving the source configuration; the robot will remain disconnected'),
            `<div class="geometry-vector"><span>${esc(preview.parent_frame)} → ${esc(preview.tool_frame)}</span>${(preview.tcp_xyz_m || []).map((value,i)=>`<span>${'XYZ'[i]} ${(Number(value)*1000).toFixed(3)} mm</span>`).join('')}</div>`, [], 'general')) return;
        await blockingOperation(tr('正在采用 TCP 配置', 'Adopting TCP configuration'), async () => {
            if (active() || geometryTcpViewState() !== cache) throw new Error(tr('会话或候选已变化，请重新预览', 'The session or candidate changed; preview again'));
            const result = await geometryCommand('apply_tcp', {confirmed:true, expected_hashes:preview.source_hashes});
            if (!result?.config || !result?.library_entry?.id) throw new Error(tr('TCP 配置已生成，请在机械臂列表中重新打开', 'The TCP configuration was created; reopen it from the manipulator list'));
            await refreshLibrary();
            await pageExit();
            await openLibraryEntry(result.library_entry.id);
            toast(tr('已打开独立 TCP 配置，机械臂保持断开', 'Independent TCP configuration opened; the robot remains disconnected'));
        });
    };
}
function wireGeometry() {
    const tr = (zh, en) => busyText(zh, en);
    const btn = (id, fn) => { if ($(id)) $(id).onclick = () => blockingOperation(tr('正在处理几何实验...', 'Processing geometry experiment...'), fn); };
    document.querySelectorAll('[data-geometry-tab]').forEach(b => b.onclick = () => { if (['recording', 'playing'].includes(state.telemetry?.teach_replay?.phase)) return toast(tr('先停止示教或回放', 'Stop recording or playback first'), true); state.geometryTab = b.dataset.geometryTab; refreshGeometryPanel(true); if (state.geometryTab === 'repeat') operation(async () => { await refreshGeometryRepeatStatus(); refreshGeometryPanel(); }); });
    if ($('geometry-purpose')) $('geometry-purpose').onchange = () => { state.geometryPurpose = $('geometry-purpose').value; };
    if ($('geometry-frame')) $('geometry-frame').onchange = () => { state.geometryFrame = $('geometry-frame').value; };
    btn('geometry-begin', async () => {
        state.geometryState = await geometryCommand('begin', { frame: $('geometry-frame').value, purpose: $('geometry-purpose').value });
        refreshGeometryPanel();
    });
    if ($('geometry-discard')) $('geometry-discard').onclick = async () => {
        if (!await safetyConfirmation(tr('清除当前实验', 'Discard experiment'), tr('当前实验的采样、候选及验证结果将清除，正式机器人配置不会变化', 'Collected poses, fit and validation will be discarded without modifying robot configuration'))) return;
        await blockingOperation(tr('正在清除实验', 'Discarding experiment'), async () => { await geometryCommand('discard'); state.geometryState = await geometryCommand('state'); refreshGeometryPanel(); });
    };
    for (const [id, phase] of [['geometry-capture-train', 'train'], ['geometry-capture-validation', 'validation']]) btn(id, async () => {
        await geometryCommand('capture', { phase });
        refreshGeometryPanel();
    });
    btn('geometry-solve', async () => { await geometryCommand('solve'); refreshGeometryPanel(); });
    for (const [id, key, initial] of [['geometry-rms', 'geometryRms', 2], ['geometry-max', 'geometryMax', 4]]) if ($(id)) $(id).onchange = () => { const value = Number($(id).value); state[key] = Number.isFinite(value) && value >= 0.05 && value <= 100 ? value : initial; $(id).value = String(state[key]); };
    btn('geometry-validate', async () => { await geometryCommand('validate', { rms_mm: state.geometryRms, max_mm: state.geometryMax }); refreshGeometryPanel(); });
    btn('geometry-report', async () => {
        const output = await geometryCommand('report');
        if ($('geometry-output-path')) $('geometry-output-path').textContent = output.path || '';
        toast(tr('已导出到 ', 'Exported to ') + (output.path || ''));
    });
    wireGeometryTcp();
    btn('geometry-fixed-drag', async () => { await workbenchRequest('set_impedance_mode', { mode: 'COMPLIANT_DRAG' }); refreshGeometryPanel(); });
    btn('geometry-repeat-drag', async () => { await workbenchRequest('set_impedance_mode', { mode: 'COMPLIANT_DRAG' }); refreshGeometryPanel(); });
    btn('geometry-repeat-record', async () => {
        const response = await geometryRepeatRequest('record_begin');
        state.teachRuntime = response; if (state.telemetry) state.telemetry.teach_replay = response; state.geometryLoadedId = ''; state.geometrySelectedId = ''; state.geometryContactReady = false; refreshGeometryPanel();
    });
    btn('geometry-repeat-mark', async () => {
        const response = await geometryRepeatRequest('record_stop');
        state.teachRuntime = response; if (state.telemetry) state.telemetry.teach_replay = response; state.geometryLoadedId = ''; state.geometryContactReady = response.phase === 'ready'; refreshGeometryPanel();
    });
    if ($('geometry-traj')) $('geometry-traj').onchange = () => { state.geometrySelectedId = $('geometry-traj').value; };
    btn('geometry-load-traj', async () => {
        if (!state.geometrySelectedId) throw new Error(tr('请选择已保存轨迹', 'Select a saved trajectory'));
        await geometryRepeatRequest('load', { id: state.geometrySelectedId });
        state.geometryLoadedId = state.geometrySelectedId; state.geometryContactReady = true; state.teachRuntime = await geometryRepeatRequest('status'); if (state.telemetry) state.telemetry.teach_replay = state.teachRuntime; refreshGeometryPanel();
    });
    btn('geometry-save-traj', async () => {
        teachNameDialog(tr('保存重复到位轨迹', 'Save repeatability trajectory'), tr('定点重复到位轨迹', 'Point repeatability path'), async name => {
            if (!state.geometryContactReady) throw new Error(tr('请先完成触点到安全点的示教', 'Record a contact-to-safe trajectory first'));
            const saved = await geometryRepeatRequest('save', { name });
            const listing = await api.request('trajectory_list', { config: state.config });
            state.geometryTrajectories = listing.items || []; state.geometrySelectedId = saved.id; state.geometryLoadedId = saved.id; refreshGeometryPanel();
        });
    });
    const limits = [['geometry-cycles', 'geometryCycles', 1, 100], ['geometry-dwell', 'geometryDwell', 0, 30], ['geometry-rate', 'geometryRate', 0.05, 1]];
    for (const [id, key, min, max] of limits) if ($(id)) $(id).onchange = () => {
        const v = Number($(id).value); if (!Number.isFinite(v) || v < min || v > max || (key === 'geometryCycles' && !Number.isInteger(v))) { $(id).value = String(state[key]); return; }
        state[key] = v;
    };
    wireReplaySelection('geometry', () => refreshGeometryPanel());
    if ($('geometry-repeat-return')) $('geometry-repeat-return').onclick = async () => {
        if (graphicsAvailabilityProblem()) return;
        const status = state.telemetry?.teach_replay || state.teachRuntime || {};
        if (!replayTrajectoryAvailable(status)) return toast(tr('请先完成或加载轨迹', 'Record or load a trajectory first'), true);
        if (!state.geometryContactReady) return toast(tr('请先完成触点到安全点的示教', 'Record a contact-to-safe trajectory first'), true);
        if (!await safetyConfirmation(tr('沿示教轨迹撤离到安全点 A', 'Withdraw to safe point A'),
            tr('此操作仅用于机械臂当前停在触点 B 时，将从 B 正放到安全点 A，请确认当前位置在 B 附近、路径无碰撞且人员远离', 'Use only if the arm is currently at contact B: the taught path will play forward to safe point A. Confirm clear motion and an empty workspace'))) return;
        await blockingOperation(tr('正在撤离到安全点', 'Withdrawing to safe point'), async () => {
            if (graphicsAvailabilityProblem()) return;
            const response = await geometryRepeatRequest('play', { kind: 'withdraw', rate: state.geometryRate, ...replaySelection('geometry'), confirmed: true });
            state.teachRuntime = response; if (state.telemetry) state.telemetry.teach_replay = response; refreshGeometryPanel();
        });
    };
    if ($('geometry-repeat-play')) $('geometry-repeat-play').onclick = async () => {
        if (graphicsAvailabilityProblem()) return;
        for (const [id, key, min, max] of limits) { const el = $(id); const value = Number(el.value); if (!Number.isFinite(value) || value < min || value > max || (key === 'geometryCycles' && !Number.isInteger(value))) { toast(tr('测试参数无效', 'Invalid test parameter'), true); el.focus(); return; } state[key] = value; }
        const status = state.telemetry?.teach_replay || state.teachRuntime || {};
        if (!replayTrajectoryAvailable(status) || !state.geometryContactReady) return toast(tr('请先录制或加载触点到安全点的轨迹', 'Record or load a contact-to-safe trajectory first'), true);
        if (!await safetyConfirmation(tr('确认自动重复到位测试', 'Confirm automatic repeatability test'),
            tr('机械臂将自动往返运动，请确保整条轨迹已在低速下验证，并确认 B 点不会对刚性针尖产生危险压力，周围没有人员，系统不会自动识别碰撞', 'The robot will travel forward and back automatically; verify the full path and safe clearance at B, keep all people outside the workspace, and note that collision detection is unavailable'))) return;
        await blockingOperation(tr('正在开始重复测试', 'Starting repeatability test'), async () => { if (graphicsAvailabilityProblem()) return; const response = await geometryRepeatRequest('play', { kind: 'cycle', cycles: state.geometryCycles, rate: state.geometryRate, ...replaySelection('geometry'), endpoint_dwell_s: state.geometryDwell, confirmed: true }); state.teachRuntime = response; if (state.telemetry) state.telemetry.teach_replay = response; refreshGeometryPanel(); });
    };
    btn('geometry-repeat-stop', async () => {
        const response = await geometryRepeatRequest('stop'); state.teachRuntime = response; if (state.telemetry) state.telemetry.teach_replay = response; refreshGeometryPanel();
    });
}
function geometryPanelHtml() {
    const en = state.language === 'en';
    const active = workbenchConnected() && state.telemetry?.robot_state === 'ACTIVE';
    const replay = state.telemetry?.teach_replay || state.teachRuntime || {};
    const running = ['playing', 'recording'].includes(replay.phase);
    const fixed = state.geometryState || {};
    const fixedActive = fixed.active;
    const training = fixed.train || 0, validation = fixed.validation || 0;
    const candidate = fixed.candidate, report = fixed.report;
    const selectedPurpose = fixedActive ? fixed.purpose : state.geometryPurpose;
    const options = (fixed.frame_options || []).map(frame => `<option value="${esc(frame)}" ${frame === (fixedActive ? fixed.parent_frame : state.geometryFrame || fixed.suggested_frame) ? 'selected' : ''}>${esc(frame)}</option>`).join('');
    const tr = (zh, english) => en ? english : zh;
    const fmt = x => Number.isFinite(Number(x)) ? Number(x).toFixed(3) : '—';
    const tab = `<div class="calibration-entry-grid">${[
        ['fixed', tr('固定点几何标定', 'Fixed-point geometry'), tr('多姿态采样、针尖位置求解与独立验证', 'Multi-pose sampling, tip fitting and independent validation')],
        ['repeat', tr('定点重复到位测试', 'Point repeatability test'), tr('触点撤离示教与自动往返验证', 'Teach contact withdrawal and verify repeated round trips')],
    ].map(([key, title, description]) => `<button type="button" class="calibration-entry ${state.geometryTab === key ? 'selected' : ''}" data-geometry-tab="${key}" aria-pressed="${state.geometryTab === key}" ${running && state.geometryTab !== key ? 'disabled' : ''}><span class="calibration-entry-title">${title}</span><span>${description}</span></button>`).join('')}</div>`;
    const fixedContent = `<div class="geometry-task-flow"><section class="workspace-section geometry-card"><div class="workspace-section-head"><div><h3>${tr('固定点实验', 'Fixed-point experiment')}</h3><p class="hint">${tr('使用同一组多姿态接触数据，求解针尖相对于末端 Frame 的临时偏移，并验证 FK 跨姿态一致性', 'Fit the temporary tip offset and verify FK consistency from one multi-pose fixed-point experiment')}</p></div><span class="result-badge ${report?.passed ? 'pass' : report ? 'warn' : 'neutral'}">${report ? tr(report.passed ? '验证通过' : '待审核', report.passed ? 'Validated' : 'Needs review') : candidate ? tr('待验证', 'Pending validation') : fixedActive ? tr('采样中', 'Sampling') : tr('未启动', 'Not started')}</span></div>
      <label>${tr('实验用途', 'Purpose')}<select id="geometry-purpose" ${fixedActive ? 'disabled' : ''}><option value="diagnostic" ${selectedPurpose === 'diagnostic' ? 'selected' : ''}>${tr('临时针尖 · FK 诊断', 'Temporary tip · FK diagnosis')}</option><option value="tcp" ${selectedPurpose === 'tcp' ? 'selected' : ''}>${tr('真实工具尖端 · TCP 位置标定', 'Actual tool tip · TCP position calibration')}</option></select></label>
      <label>${tr('末端参考 Frame', 'Tip parent frame')}<select id="geometry-frame" ${fixedActive ? 'disabled' : ''}>${options}</select></label>
      <div class="workspace-actions"><button id="geometry-fixed-drag" class="button" ${!active || running ? 'disabled' : ''}>${tr('拖拽定位', 'Position by dragging')}</button></div>
      <div class="workspace-actions"><button id="geometry-begin" class="button primary" ${!state.config.profile || fixedActive || running ? 'disabled' : ''}>${tr('开始实验', 'Start experiment')}</button><button id="geometry-discard" class="button danger" ${!fixedActive || running ? 'disabled' : ''}>${tr('清除当前实验', 'Discard experiment')}</button></div>
      <div class="geometry-step"><b>01 · ${tr('训练姿态', 'Training poses')}　${training} / 10</b><p class="hint">${tr('柔性拖拽使针尖从不同末端朝向接触同一个固定点，保持静止后手动采样', 'Touch the same fixed point from diverse wrist orientations, hold still and capture')}</p><button id="geometry-capture-train" class="button" ${!fixedActive || !active || !!candidate || running ? 'disabled' : ''}>${tr('记录训练姿态', 'Capture training pose')}</button></div>
      <div class="geometry-step"><b>02 · ${tr('求解临时针尖位置', 'Solve tip position')}</b><p class="hint">${tr('最少 8 组，推荐 10 组，求解后冻结候选以供独立验证', 'Minimum 8, recommended 10 poses, candidate frozen for validation')}</p><button id="geometry-solve" class="button" ${!fixedActive || training < 8 || !!candidate || running ? 'disabled' : ''}>${tr('求解固定点模型', 'Solve fixed-point model')}</button></div>
      <div class="geometry-step"><b>03 · ${tr('独立验证', 'Independent validation')}　${validation} / 5</b><p class="hint">${tr('另外使用至少 5 个新的末端姿态，不参与求解', 'Capture at least 5 new wrist orientations excluded from the fit')}</p><button id="geometry-capture-validation" class="button" ${!active || !candidate || running ? 'disabled' : ''}>${tr('记录验证姿态', 'Capture validation pose')}</button></div>
      <div class="geometry-step"><b>04 · ${tr('验收阈值', 'Acceptance limits')}</b><div class="geometry-fields"><label>RMS (mm)<input id="geometry-rms" type="number" min="0.05" max="100" step="0.1" value="${esc(state.geometryRms)}"></label><label>${tr('最大误差', 'Max error')} (mm)<input id="geometry-max" type="number" min="0.05" max="100" step="0.1" value="${esc(state.geometryMax)}"></label></div><button id="geometry-validate" class="button primary" ${validation < 5 || !candidate || running ? 'disabled' : ''}>${tr('计算独立验证结果', 'Validate held-out poses')}</button></div>
    </section><section class="workspace-section geometry-card"><h3>${tr('几何实验结果', 'Geometry experiment results')}</h3>
      <div class="geometry-stats"><div><small>${tr('训练 RMS', 'Training RMS')}</small><strong>${candidate ? fmt(candidate.training_rms_mm) + ' mm' : '—'}</strong></div><div><small>${tr('独立 RMS', 'Validation RMS')}</small><strong>${report ? fmt(report.rms_mm) + ' mm' : '—'}</strong></div><div><small>${tr('最大偏差', 'Maximum error')}</small><strong>${report ? fmt(report.max_mm) + ' mm' : '—'}</strong></div><div><small>${tr('条件数', 'Condition number')}</small><strong>${candidate ? fmt(candidate.condition) : '—'}</strong></div></div>
      ${candidate ? `<h4>${tr('针尖偏移（相对于末端 Frame）', 'Tip offset (in parent frame)')}</h4><div class="geometry-vector">${candidate.tcp_xyz_m.map((v, i) => `<span>${'XYZ'[i]} ${fmt(v * 1000)} mm</span>`).join('')}</div>` : `<p class="hint">${tr('完成训练采样并求解后，在此查看数值结果', 'Capture and fit training poses to view results')}</p>`}
      ${report ? `<div class="geometry-result ${report.passed ? 'is-pass' : 'is-warning'}">${report.passed ? tr('独立一致性验证通过', 'Independent consistency passed') : tr('独立一致性验证未通过', 'Independent consistency failed')} · ${validation} ${tr('组验证姿态', 'validation poses')}</div><h4>${tr('每组验证的三维残差范数', 'Per-pose residual norms')}</h4><div class="geometry-error-list">${report.errors_mm.map((v, i) => `<span>#${i + 1}　${fmt(v)} mm</span>`).join('')}</div>` : ''}
      <p class="hint">${tr('固定点残差用于评价跨姿态一致性；本方法标定 TCP 平移，朝向沿用采样参考 Frame；绝对定位精度需另行测量', 'Fixed-point residuals measure cross-pose consistency. TCP translation is fitted and sampling-frame orientation is retained; absolute accuracy requires a separate measurement')}</p>
      ${selectedPurpose === 'diagnostic' ? `<p class="geometry-note">${tr('诊断模式导出 FK 实验报告；如需生成工具配置，请选择 TCP 标定', 'Diagnostic mode exports an FK report; choose TCP calibration to generate tool configuration')}</p><button id="geometry-report" class="button" ${!report ? 'disabled' : ''}>${tr('导出 FK 验证报告', 'Export FK report')}</button>` : `<p class="geometry-note">${tr('独立验证通过后，预览工具坐标系变化，再导出或采用独立机械臂 Profile', 'After independent validation, preview the tool-frame changes, then export or adopt a separate manipulator Profile')}</p><div class="workspace-actions"><button id="geometry-preview-tcp" class="button" ${!report?.passed || running ? 'disabled' : ''}>${tr('预览 TCP 配置差异', 'Preview TCP changes')}</button><button id="geometry-export-tcp" class="button" ${!report?.passed || running ? 'disabled' : ''}>${tr('导出完整 TCP 配置', 'Export complete TCP configuration')}</button><button id="geometry-apply-tcp" class="button primary" ${!report?.passed || !geometryTcpViewState().preview || geometryTcpSessionActive() || running ? 'disabled' : ''}>${tr('采用为独立机械臂 Profile', 'Adopt as separate manipulator Profile')}</button></div>${geometryTcpSessionActive() && report?.passed ? `<p class="hint">${tr('导出可在连接期间进行；采用配置前请先停放并断开会话', 'Export is available while connected; park and disconnect before adopting')}</p>` : ''}${geometryTcpSummaryHtml()}`}
      <p id="geometry-output-path" class="hint"></p>
    </section></div>`;
    const optionsTraj = state.geometryTrajectories.filter(item => item.kind === 'contact_first').map(item => `<option value="${esc(item.id)}" ${item.id === state.geometrySelectedId ? 'selected' : ''}>${esc(item.name)}</option>`).join('');
    const replayStatus = { idle: tr('尚无轨迹', 'No trajectory'), recording: tr('正在示教', 'Recording'), ready: tr('轨迹已就绪', 'Trajectory ready'), playing: tr('正在循环', 'Playing'), failed: tr('任务失败', 'Failed') }[replay.phase] || tr('等待连接', 'Waiting for connection');
    const repeatContent = `${workbenchConnected() && state.geometryRepeatSupported === false ? `<div class="geometry-note" role="alert">${esc(state.geometryRepeatError || tr('当前 C++ Terminal 不支持本实验，请重新编译安装后重新连接', 'Running C++ terminal does not support this experiment; rebuild and reconnect'))}</div>` : ''}<div class="geometry-task-flow"><section class="workspace-section geometry-card"><div class="workspace-section-head"><div><h3>${tr('从触点示教撤离轨迹', 'Teach withdrawal from contact')}</h3><p class="hint">${tr('拖拽定位触点 B，稳定后开始记录；拖至安全点 A 后结束记录', 'First align and stabilize at contact B, then start recording and drag away to safe point A before finishing')}</p></div></div>
      <div class="geometry-result" id="geometry-repeat-status">${replayStatus}</div><p id="geometry-record-context" class="hint">${esc(recordingContextText(replay.phase))}</p>
      <div class="workspace-actions"><button id="geometry-repeat-drag" class="button" ${!active || running ? 'disabled' : ''}>${tr('拖拽定位', 'Position by dragging')}</button><button id="geometry-repeat-record" class="button primary" ${!active || running || state.geometryRepeatSupported !== true ? 'disabled' : ''}>${tr('从触点开始记录', 'Confirm contact and record')}</button><button id="geometry-repeat-mark" class="button" ${replay.phase !== 'recording' || state.geometryRepeatSupported !== true ? 'disabled' : ''}>${tr('到达安全点并结束记录', 'Finish at safe point')}</button></div>
      <p class="hint">${tr('在触点 B 开始记录，拖至安全点 A 后结束', 'Contact B is the recorded start and safe A is the end; teaching temporarily uses native impedance and gravity compensation, then restores the session strategy and holds')}</p>
      <div class="geometry-step"><b>${tr('轨迹管理（可选）', 'Saved trajectory (optional)')}</b><label>${tr('载入现有轨迹', 'Load saved trajectory')}<select id="geometry-traj"><option value="">${tr('本次新录制轨迹', 'Current unsaved trajectory')}</option>${optionsTraj}</select></label><div class="workspace-actions"><button id="geometry-load-traj" class="button" ${!workbenchConnected() || running || !state.geometrySelectedId || state.geometryRepeatSupported !== true ? 'disabled' : ''}>${tr('载入所选轨迹', 'Load selected')}</button><button id="geometry-save-traj" class="button" ${!workbenchConnected() || replay.phase !== 'ready' || !state.geometryContactReady || state.geometryRepeatSupported !== true ? 'disabled' : ''}>${tr('保存轨迹', 'Save trajectory')}</button></div></div>
    </section><section class="workspace-section geometry-card"><h3>${tr('自动往返测试', 'Automatic round-trip test')}</h3><p class="hint">${tr('轨迹录制方向为 B → A，自动测试从安全点 A 倒放至触点 B，停留 M 秒后正放撤离到 A，重复 N 次', 'Recorded path: B → A. Each test cycle reverses A → B, holds M seconds, then withdraws forward B → A, repeated N times')}</p>
      <div class="geometry-fields"><label>${tr('循环次数 N', 'Cycles N')}<input id="geometry-cycles" type="number" min="1" max="100" step="1" value="${esc(state.geometryCycles)}"></label><label>${tr('触点停留 M / 秒', 'Dwell M / s')}<input id="geometry-dwell" type="number" min="0" max="30" step="0.5" value="${esc(state.geometryDwell)}"></label><label>${tr('回放速度比例', 'Playback speed')}<input id="geometry-rate" type="number" min="0.05" max="1" step="0.05" value="${esc(state.geometryRate)}"></label>${replaySelectionHtml('geometry', running)}</div>
      <p class="hint">${tr('结束示教后机械臂处于安全点 A，可直接开始循环；系统在原生控制层校验当前关节位置与轨迹终点 A 的一致性；原先从 A 到 B 录制的轨迹不可直接在此模式使用', 'Teaching ends at safe point A, ready for cycling. Native control checks alignment to recorded end A. Old A-to-B paths cannot be used in this mode')}</p>
      <div class="workspace-actions"><button id="geometry-repeat-play" class="button primary" ${!active || !replayTrajectoryAvailable(replay) || !state.geometryContactReady || running || state.geometryRepeatSupported !== true ? 'disabled' : ''}>${tr('确认安全并开始循环', 'Confirm safety and run')}</button><button id="geometry-repeat-stop" class="button danger" ${replay.phase !== 'playing' ? 'disabled' : ''}>${tr('停止并保持', 'Stop and hold')}</button></div>
      <details id="geometry-withdraw-options" class="geometry-step"><summary>${tr('可选：中途停在触点 B 时撤离', 'Optional: withdraw if stopped at contact B')}</summary><p class="hint">${tr('只在机械臂停在 B 点附近时使用，从 B 沿示教方向撤离到 A，若已在 A 点则无需点击', 'Use only when stopped near B to withdraw along the taught path to A; unnecessary if already at A')}</p><button id="geometry-repeat-return" class="button" ${!active || !replayTrajectoryAvailable(replay) || !state.geometryContactReady || running || state.geometryRepeatSupported !== true ? 'disabled' : ''}>${tr('沿轨迹撤离到安全点 A', 'Withdraw to safe A')}</button></details>
      <div class="geometry-result"><span id="geometry-repeat-progress">${tr('已完成循环', 'Completed cycles')} ${Math.floor(Number(replay.completed || 0) / 2)} / ${state.geometryCycles}</span></div>
      <p class="geometry-note">${tr('本测试不自动测量接触误差，触点是否对准及目标直径由用户自行目视判断，不可直接将针尖直径当作重复定位精度', 'No automatic contact-error measurement is made; assess alignment visually, and do not equate tip diameter with calibrated repeatability')}</p>
      <p class="hint">${tr('自动运行前务必确保回放终点不会向刚性针尖施加危险碰撞力，建议先在不接触针尖的位置以低速检查整条轨迹', 'Verify the complete trajectory at low speed with safe clearance before replay; no automatic collision detection is provided')}</p>
    </section></div>`;
    return `${calibrationQuickControlsHtml()}${tab}${state.geometryTab === 'fixed' ? fixedContent : repeatContent}`;
}
function refreshGeometryPanel(resetScroll = false) {
    if (state.page !== 'geometry') return;
    const panel = document.querySelector('.geometry-panel');
    if (!panel) return;
    const scrollContainer = panel.closest('.content');
    const scroll = resetScroll ? 0 : scrollContainer?.scrollTop || 0;
    const details = new Map([...panel.querySelectorAll('details')].map((item, index) => [item.id || item.querySelector('summary')?.textContent.trim() || String(index), item.open]));
    const active = document.activeElement;
    const field = active?.closest?.('.studio-select')?.querySelector('select') || active;
    const focus = panel.contains(active) && field.id ? {id:field.id, trigger:field !== active, value:field.value, checked:field.checked, start:field.selectionStart, end:field.selectionEnd} : null;
    panel.innerHTML = geometryPanelHtml();
    for (const [index, item] of [...panel.querySelectorAll('details')].entries()) {
        const key = item.id || item.querySelector('summary')?.textContent.trim() || String(index);
        if (details.has(key)) item.open = details.get(key);
    }
    wireGeometry(); wireCalibrationQuickControls();
    window.serialArmStudio?.enhanceSelects();
    if (state.page === 'profile_setup') restoreProfileFields();
    if (state.setupFocus?.page === state.page) focusSetupTarget(); refreshCalibrationQuickControls();
    if (focus) {
        const input = $(focus.id);
        if (input && !input.disabled) {
            if (typeof focus.value === 'string' && ['INPUT', 'TEXTAREA'].includes(input.tagName)) input.value = focus.value;
            if (input.type === 'checkbox') input.checked = focus.checked;
            (focus.trigger ? input.nextElementSibling : input)?.focus({preventScroll:true});
            if (input.type === 'text' && focus.start != null) input.setSelectionRange(focus.start, focus.end);
        }
    }
    if (scrollContainer) scrollContainer.scrollTop = scroll;
}
function geometryPage() {
    const tr = (zh, en) => state.language === 'en' ? en : zh;
    return pageHead('geometryTitle', 'geometryDesc', workspaceSessionActionHtml()) + workbenchStatusBar() + workbenchRecoveryNotice()
        + `<div class="workspace-provenance"><b>${tr('几何实验', 'Geometry experiment')}</b><span>${tr('实验采样与候选独立保存，求解和验证不会修改 URDF 或 Profile，成果由用户确认后导出', 'Samples and candidates are stored separately; fitting and validation leave the URDF and Profile unchanged, and results are exported on request')}</span></div>
        <div class="workbench-layout"><section class="workbench-model"><div id="workbench-viewport">${state.modelData ? '' : `<div class="empty-state">${t('modelNotLoaded')}</div>`}</div><div class="workbench-model-note">${t(workbenchConnected() ? 'actualTelemetry' : 'offlineWaiting')}</div></section><section class="workbench-task"><div class="workspace-panel calibration-panel geometry-panel">${geometryPanelHtml()}</div></section></div>`;
}

function calibrationPage() {
    return pageHead('calibrationTitle', 'calibrationDesc', workspaceSessionActionHtml()) + workbenchStatusBar() + `<div id="workspace-recovery-notice">${workbenchRecoveryNotice()}</div><div class="workspace-provenance"><b>${state.language === 'en' ? 'Candidate' : '标定候选'}</b><span>${state.language === 'en' ? 'Task records and exports stay separate from saved Profile parameters. Temporary candidate application does not save core.yaml' : '标定任务记录与成果独立保存，临时应用候选不会修改 core.yaml 中的持久化配置'}</span></div><div class="workbench-layout"><section class="workbench-model"><div id="workbench-viewport">${state.modelData ? '' : `<div class="empty-state">${t('modelNotLoaded')}</div>`}</div>${calibrationComOverlayHtml()}<div class="workbench-model-note">${workspaceModelNoteHtml()}</div></section><section class="workbench-task">${calibrationPanelHtml()}</section></div>`;
}
let workspaceMount = null;
async function mountWorkbenchView() {
    const host = $('workbench-viewport'), payload = state.modelData, page = state.page;
    if (!['workbench', 'geometry', 'calibration'].includes(page) || !payload || !host) return;
    if (workspaceMount?.host === host && workspaceMount.payload === payload) return workspaceMount.promise;
    if (modelView?.container === host && modelView.payload === payload) return;
    const record = previewRecord(host, payload);
    if (graphicsState.effectiveMode === 'rescue' || record.fault) { showGraphicsFailure(record); return; }
    const request = {host, payload}; workspaceMount = request;
    request.promise = (async () => {
        let view, phase = 'renderer';
        const current = () => workspaceMount === request && $('workbench-viewport') === host && state.modelData === payload && state.page === page;
        try {
            if (!modelModule) modelModule = await import('./model-view.js');
            if (!current()) return;
            disposeModelPreview();
            view = new modelModule.SerialArmModelView(host, file => api.modelResource(file), state.language, {mode: graphicsState.effectiveMode, onStatus: report => reportPreviewState(record, view, report)});
            modelView = view;
            phase = 'model';
            await view.setModel(payload);
            if (!current() || modelView !== view) { view.dispose(); return; }
            view.setLayers({visual:true, collision:false, linkFrames:false, jointAxes:false, com:false, inertia:false, labels:false});
            if (record.viewState) view.restoreViewState(record.viewState);
            view.renderOnce();
            updateWorkbenchModel(); updateModelCalibrationComparison(); updateStartGhost(true);
        } catch (error) {
            if (current()) failModelPreview(record, error, view, phase);
            else view?.dispose();
        } finally { if (workspaceMount === request) workspaceMount = null; }
    })();
    return request.promise;
}

function updateWorkbenchModel() { if (!modelView || !telemetryFresh(state.telemetry,500)) return; const frames = state.telemetry.frames || []; modelView.updatePose({ state: { frames, positions: state.telemetry.joint?.pos || [] } }); updateModelCalibrationComparison(); }
function controlOperationProblem(action) {
    const en = state.language === 'en', x = state.telemetry || {};
    const msg = (zh, english) => en ? english : zh;
    if (workbenchRecoveryRequired()) return msg('控制会话无响应，请先恢复连接', 'Recover the unresponsive control session first');
    if (!workbenchConnected()) return msg('请先连接工作台', 'Connect the workspace first');
    const emergency = action === 'deactivate';
    if (!emergency && !['park','hold','rigid_hold'].includes(action) && state.modelReloadPending) return msg('模型已修改，请断开并重新连接', 'The model changed; disconnect and reconnect');
    if (!emergency && (x.write_enabled ?? state.readiness?.connection_readiness?.write_enabled) === false) return msg('当前使用离线后端，不读取电机反馈；在运动与交互控制中开放写入并重新连接后使用实机', 'This is an offline backend without motor feedback; enable writing in Motion and reconnect for hardware control');
    const fault = x.robot_state === 'FAULT', active = x.robot_state === 'ACTIVE', inactive = x.robot_state === 'INACTIVE';
    const configured = mode => ['configured', 'verified', 'stale'].includes(state.readiness?.modes?.[mode]);
    if (action === 'deactivate') return active || fault ? '' : msg('机械臂已失能', 'The robot is already disabled');
    if (['clear_fault','fault_compliant','fault_rigid'].includes(action) || (fault && action === 'rigid_hold')) {
        if (!fault) return msg('当前没有故障', 'There is no active fault');
        return ['fault_rigid','rigid_hold'].includes(action) && !configured('RIGID_HOLD') ? msg('请先配置刚性保持增益', 'Configure rigid hold gains first') : '';
    }
    if (fault) return msg('先处理当前故障，再恢复运行', 'Resolve the current fault before resuming');
    if (action === 'activate' && !inactive) return msg('机械臂已使能', 'The robot is already enabled');
    if (action !== 'activate' && !active) return msg('请先使能机械臂', 'Activate the robot first');
    if (!['hold','rigid_hold'].includes(action) && (modelCalibrationActive() || x.gravity_verification?.active || ['recording','playing'].includes(x.teach_replay?.phase))) return msg('请先结束或停止当前任务', 'Finish or stop the current task first');
    if (['activate','hold','rigid_hold'].includes(action) && !configured('RIGID_HOLD')) return msg('请先配置刚性保持增益', 'Configure rigid hold gains first');
    if (action === 'park' && !(state.readiness?.control_available && ['RIGID_TRACKING','COMPLIANT_TRACKING'].every(configured))) return msg('请先配置停放轨迹与跟踪增益', 'Configure parking and tracking gains first');
    return '';
}
function updateWorkbenchControlState() {
    const x = state.telemetry || {}, recovering = workbenchRecoveryRequired(), pending = state.workbenchPending || {};
    const robotState = workbenchConnected() ? x.robot_state || 'UNKNOWN' : 'OFFLINE';
    if ($('control-task-state')) $('control-task-state').textContent = robotState;
    const dot = document.querySelector('.robot-operation-dot');
    if (dot) dot.className = 'robot-operation-dot' + (robotState === 'ACTIVE' ? ' on' : ' off');
    document.querySelectorAll('[data-robot-connect]').forEach(button => button.hidden = workbenchConnected() || !!$('start-workbench'));
    document.querySelectorAll('[data-workbench-action]').forEach(button => {
        const action = button.dataset.workbenchAction, reason = controlOperationProblem(action);
        button.disabled = !!reason; button.title = reason;
    });
    if ($('control-operation-hint')) $('control-operation-hint').textContent = !workbenchConnected() ? busyText('连接工作台后可使能机械臂', 'Connect the workspace to activate the robot') : state.modelReloadPending ? busyText('模型已修改，请停放并重新连接后使用新模型', 'The model changed; park and reconnect to load it') : x.robot_state === 'FAULT' ? busyText('处理故障原因后选择恢复方式；立即失能可释放执行器', 'Resolve the fault, then choose recovery; Disable releases the actuators') : controlOperationProblem(x.robot_state === 'ACTIVE' ? 'park' : 'activate');
    if ($('control-operation-next')) {
        const field = state.readiness?.connection_readiness?.write_enabled === false ? 'core.control.runtime.write_enabled' : !['configured','verified','stale'].includes(state.readiness?.modes?.RIGID_HOLD) ? 'core.control.controller.rigid_hold.kp' : '';
        const node=$('control-operation-next');
        node.innerHTML = field && workbenchConnected() ? setupActionHtml({page:'profile_setup',group:'motion',field},busyText('前往配置','Open configuration')) : '';
        node.querySelectorAll('[data-setup-target]').forEach(button=>button.onclick=()=>operation(()=>openSetupTarget(JSON.parse(button.dataset.setupTarget))));
    }
    document.querySelectorAll('[data-gain-purpose]').forEach(button => {
        const purpose=button.dataset.gainPurpose;
        button.classList.toggle('active',x.gain_profile===purpose);button.disabled=!controlGainPurposeMayChange(purpose);
        button.title=!button.disabled ? '' : !workbenchConnected() ? busyText('请先连接工作台','Connect first') : x.robot_state!=='ACTIVE' ? busyText('请先使能机械臂','Activate first') : referenceBehavior()==='MODEL_ADMITTANCE' && !['SERVO_TRACKING','DRAG'].includes(purpose) ? busyText('模型导纳使用伺服跟踪增益','Model admittance uses servo gains') : !gainPurposeConfigured(purpose,x.impedance_mode?.endsWith('_TRACKING')) ? busyText('请先配置该用途的增益','Configure gains for this use first') : busyText('请先结束当前任务','Finish the current task first');
    });
    document.querySelectorAll('[data-impedance-mode]').forEach(button => {
        const mode=button.dataset.impedanceMode, configured=['configured','verified','stale'].includes(state.readiness?.modes?.[mode]);
        button.classList.toggle('active',x.impedance_mode===mode);button.disabled=recovering || x.robot_state!=='ACTIVE' || pending.kind==='impedance' || !configured;
        button.title=!configured ? readinessResultLabel(state.readiness?.modes?.[mode]) : x.robot_state==='ACTIVE' ? '' : busyText('请先使能机械臂','Activate first');
    });
    document.querySelectorAll('[data-feedforward-mode]').forEach(button=>{
        button.classList.toggle('active',x.model_feedforward_mode===button.dataset.feedforwardMode);
        button.disabled=recovering || x.robot_state!=='INACTIVE' || pending.kind==='feedforward' || state.modelReloadPending;
        button.title=button.disabled ? busyText('停放并失能后可修改模型前馈','Park and disable before changing model feedforward') : '';
    });
}
function applyWorkbenchSnapshot(snapshot) { if (!snapshot || typeof snapshot !== 'object') return; acceptTelemetrySnapshot(snapshot); updateWorkbenchLive(); }
async function switchWorkbenchMode(kind, mode) { if (state.workbenchPending?.kind || state.blockingBusy) return; state.workbenchPending = { kind, value: mode }; updateWorkbenchControlState(); const message = kind === 'impedance' ? busyText(`正在切换阻抗模式至 ${mode}...`, `Switching impedance mode to ${mode}...`) : busyText(`正在切换模型前馈至 ${mode}...`, `Switching model feedforward to ${mode}...`); try { return await blockingOperation(message, async () => { await new Promise(resolve => requestAnimationFrame(resolve)); const method = kind === 'impedance' ? 'set_impedance_mode' : 'set_model_feedforward_mode'; const snapshot = await workbenchRequest(method, { mode }); applyWorkbenchSnapshot(snapshot); return snapshot; }); } finally { state.workbenchPending = { kind: '', value: '' }; updateWorkbenchControlState(); } }
function updateWorkbenchLive() {
    if ($('robot-operation')) updateWorkbenchControlState();
    if (state.page === 'profile_setup') { refreshRuntimeSavePreview(); return; }
    if (!['workbench', 'geometry', 'calibration', 'diagnostics'].includes(state.page)) return; const x = state.telemetry || {};
    refreshDamiaoJointCheck();
    if (state.page === 'geometry') {
        const phase = x.teach_replay?.phase || 'idle';
        const active = x.robot_state === 'ACTIVE';
        const connected = workbenchConnected();
        const running = phase === 'recording' || phase === 'playing';
        const recovering = workbenchRecoveryRequired();
        const update = (id, permitted) => { const button = $(id); if (button) button.disabled = recovering || !permitted; };
        update('geometry-fixed-drag', active && !running);
        update('geometry-repeat-drag', active && !running);
        update('geometry-repeat-record', active && !running && state.geometryRepeatSupported === true);
        update('geometry-repeat-mark', phase === 'recording' && state.geometryRepeatSupported === true);
        update('geometry-load-traj', connected && !running && !!state.geometrySelectedId && state.geometryRepeatSupported === true);
        update('geometry-save-traj', connected && phase === 'ready' && state.geometryContactReady && state.geometryRepeatSupported === true);
        update('geometry-repeat-return', active && !graphicsAvailabilityProblem() && replayTrajectoryAvailable(x.teach_replay) && state.geometryContactReady && state.geometryRepeatSupported === true);
        update('geometry-repeat-play', active && !graphicsAvailabilityProblem() && replayTrajectoryAvailable(x.teach_replay) && state.geometryContactReady && state.geometryRepeatSupported === true);
        update('geometry-repeat-stop', phase === 'playing' && state.geometryRepeatSupported === true);
        if (state.geometryTab === 'repeat') {
            refreshReplaySelection('geometry', x.teach_replay);
            const progress = $('geometry-repeat-progress');
            if (progress) progress.textContent = busyText('已完成循环 ', 'Completed cycles ') + Math.floor(Number(x.teach_replay?.completed || 0) / 2) + ' / ' + state.geometryCycles;
            if ($('geometry-record-context')) $('geometry-record-context').textContent=recordingContextText(phase);
            const status = $('geometry-repeat-status');
            if (status) status.textContent = ({ recording: busyText('正在示教撤离', 'Recording withdrawal'), ready: busyText('轨迹已就绪', 'Ready'), playing: busyText('正在循环', 'Playing'), failed: busyText('任务失败', 'Failed') })[phase] || busyText('尚无轨迹', 'No trajectory');
        }
    }
    if (state.page === 'workbench' && state.workbenchTab === 'replay') {
        const phase = x.teach_replay?.phase || 'idle', active = x.robot_state === 'ACTIVE', recovering = workbenchRecoveryRequired();
        const selected = state.teachTrajectories.find(t => t.id === state.teachSelectedId);
        const enable = (id, yes) => { const b = $(id); if (b) b.disabled = recovering || !yes; };
        enable('teach-record', active && !['recording', 'playing'].includes(phase));
        enable('teach-record-stop', phase === 'recording');
        enable('teach-save', phase === 'ready');
        enable('teach-play', active && replayTrajectoryAvailable(x.teach_replay) && !!selected && selected.id === state.teachLoadedId);
        enable('teach-return', active && replayTrajectoryAvailable(x.teach_replay) && !!selected && selected.id === state.teachLoadedId);
        enable('teach-stop', phase === 'playing');
        refreshReplaySelection('teach', x.teach_replay);
    }
    if (state.page === 'workbench' && state.workbenchTab === 'tuning') {
        const c = admittanceCapabilityState();
        const chip = $('capability-runtime-status'), reason = $('capability-runtime-reason');
        if (chip) { chip.textContent = c.text; chip.className = 'readiness-chip ' + (c.key === 'running' ? 'ready' : c.key === 'ready' ? 'pending' : c.key === 'suspended' ? 'warning' : 'muted'); }
        if (reason) reason.textContent = c.reason;
        const can = admittanceMayTune();
        document.querySelectorAll('[data-tune-key]').forEach(input => input.disabled = !can);
        for (const id of ['apply-tuning', 'reset-tuning']) if ($(id)) $(id).disabled = !can;
        document.querySelectorAll('[data-goal-context="runtime"],[data-goal-joint="runtime"],[data-goal-generate="runtime"]').forEach(input => input.disabled = !can);
    }
    refreshCapabilityDiagnostics();
    refreshWorkbenchTargetFields();
    if ($('control-gain-context')) $('control-gain-context').innerHTML = controlGainContextHtml();
    if (state.page === 'calibration' && $('gravity-verify-phase')) {
        const proof = x.gravity_verification || {};
        const phaseLabel = $('gravity-verify-phase');
        if (phaseLabel) phaseLabel.textContent = gravityVerificationPhaseText(proof.phase || 'idle',proof);
        const samples = $('gravity-verify-samples');
        if (samples) samples.textContent = `${busyText('有效姿态','Valid poses')} ${Number(proof.sample_count || 0)} / 3`;
        document.querySelectorAll('[data-verify-scale-index]').forEach(input => {
            const actual = input.parentElement?.querySelector('small');
            if (actual) actual.textContent = Number(x.gravity_scale?.[Number(input.dataset.verifyScaleIndex)] ?? 0).toFixed(3);
        });
        const enabled = proof.active === true;
        const stateActive = x.robot_state === 'ACTIVE';
        const dragging = enabled && proof.phase === 'dragging' && x.impedance_mode === 'COMPLIANT_DRAG';
        const states = {
            'gravity-verify-begin': !enabled && x.robot_state === 'INACTIVE' && (state.readiness?.connection_readiness?.basic_calibration_ready ?? state.readiness?.control_available) === true,
            'gravity-verify-activate': enabled && x.robot_state === 'INACTIVE' && proof.phase === 'prepared',
            'gravity-verify-enter-drag': enabled && stateActive && proof.phase === 'prepared' && proof.ramp_settled,
            'gravity-verify-apply': dragging, 'gravity-verify-sample': dragging && proof.ramp_settled,
            'gravity-verify-park': enabled && stateActive, 'gravity-verify-finish': enabled && !stateActive && x.robot_state === 'INACTIVE' && proof.phase === 'stopped' && proof.sample_count >= 3,
            'gravity-verify-cancel': enabled && x.robot_state === 'INACTIVE', 'gravity-verify-evidence': proof.phase === 'completed'
        };
        Object.entries(states).forEach(([id, can]) => { const button = $(id); if (button) button.disabled = !can; });
        document.querySelectorAll('[data-verify-scale-index],[data-verify-scale-slider]').forEach(input => input.disabled = !dragging);
    }
    refreshWorkbenchStatus(); updateWorkbenchControlState(); refreshCalibrationQuickControls(); refreshCalibrationActionControls(); refreshGainApplyControls(); updateWorkbenchModel(); if (state.page === 'workbench' && state.workbenchTab === 'replay') { const r = x.teach_replay || {}; const phase = $('teach-replay-phase'); if (phase) phase.textContent = ({idle:busyText('未开始','Idle'),recording:busyText('正在记录','Recording'),ready:busyText('轨迹已就绪','Ready'),playing:busyText('正在回放','Playing'),failed:busyText('任务失败','Failed')})[r.phase] || busyText('未开始','Idle'); const completed = $('teach-completed'); if (completed) completed.textContent = busyText('已完成单程 ','Completed one-way trips ') + Number(r.completed || 0); const error = $('teach-replay-error'); if (error) error.textContent = r.error ? teachReplayShortError(r.error) : ''; } if (['calibration','geometry'].includes(state.page) || (state.page === 'workbench' && state.workbenchTab === 'replay')) updateStartGhost(); updateMotionCompletion(x); const c = x.calibration || {}; const kind = $('cal-task-kind'), phase = $('cal-task-phase'), progress = $('cal-task-progress'), trajectory = $('cal-task-trajectory'), error = $('cal-task-error'); if (kind) kind.textContent = calibrationTaskKindText(c.kind || 'none'); if (phase) phase.textContent = calibrationTaskPhaseText(c.phase || 'idle'); if (progress) progress.textContent = c.expected ? `${c.captured || 0}/${c.expected}` : '—'; if (trajectory) trajectory.textContent = String(c.trajectory_samples ?? '—'); if (error) error.textContent = c.error ? localizedError(c.error) : '—'; if (c.phase === 'complete' && state.calibrationFlow === 'friction') { state.calibrationFlow = 'complete'; operation(loadAdmittance); } refreshFrictionReplayControls(); updateModelCalibrationLive(); updateModelCalibrationTaskOverlay(); refreshWorkspaceTelemetryRows(); if (state.page === 'diagnostics') drawWorkbenchChart();
}
function updateModelCalibrationComparison() {
    if (!modelView || state.page !== 'calibration') return;
    const binding = modelCalibrationModelBinding();
    let result = state.calibrationEntry === 'gravity' && binding.comparable ? currentModelCalibration()?.gravity_result : null;
    if (state.calibrationEntry === 'dynamics') {
        const report = state.idimReport;
        result = null;
        if (binding.comparable && state.idimPreview !== false && report?.physical_pass && report.task_id === currentModelCalibration()?.task_id) {
            result = {first_moments:(report.physical_review || []).map((l,i)=>({link_name:l.link,value:report.selected_dynamic_parameters.slice(i*10+1,i*10+4)})),candidate_masses:Object.fromEntries((report.physical_review || []).map((l,i)=>[l.link,report.selected_dynamic_parameters[i*10]]))};
        }
    }
    const scale = state.modelCalibrationDisplayScale || 1;
    const key = result?.first_moments ? `${binding.actual}:${binding.expected}:${JSON.stringify(result.first_moments)}:${scale}:${state.modelCalibrationShowUnchanged}:${binding.historical}` : '';
    if (state.modelCalibrationComparisonView === modelView && state.modelCalibrationComparisonKey === key) return;
    modelView.setGravityComparison(result || null, { displayScale: scale, showUnchanged: state.modelCalibrationShowUnchanged, sourceInertials: binding.historical ? binding.alignment.source_inertials : null, candidateMasses: result?.candidate_masses || null });
    state.modelCalibrationComparisonView = modelView;
    state.modelCalibrationComparisonKey = key;
}
function updateModelCalibrationLive() {
    refreshGainProbeStatus();
    if (state.page !== 'calibration' || state.modelCalibrationOffline) return;
    const status = state.telemetry?.model_calibration; if (!status) return;
    // Offline assessment runs after a saved record is loaded, never in a live control session
    const set = (id, value) => { const node = $(id); if (node) node.textContent = String(value); };
    set('model-calibration-phase', modelCalibrationPhaseText(status.phase));
    set('model-calibration-task-id-inline', status.task_id || '—');
    set('model-calibration-progress-inline', `${Math.round(Number(status.progress || 0) * 100)}%`);
    set('model-calibration-trajectory-inline', status.trajectory_samples ?? '—');
    const poses = status.pose_targets || []; set('model-calibration-groups-inline', `${poses.filter(x => !x.validation).length} / ${poses.filter(x => x.validation).length}`);
    set('model-calibration-static-inline', status.valid_static_samples ?? '—');
    set('model-calibration-recorder-inline', `${status.recorder?.accepted ?? '—'} / ${status.recorder?.dropped ?? '—'}`);
    if (status.source_task_id && status.phase === 'waiting_replay_confirmation') {
        const aligned = calibrationAlignment(status);
        const label = $('model-calibration-align-status');
        if (label) label.textContent = calibrationAlignmentSummary(aligned,status.alignment_active);
        const body = $('model-calibration-align-rows');
        if (body) body.innerHTML = calibrationAlignmentRowsHtml(aligned);
        const start = $('model-calibration-replay-confirm');
        if (start) start.disabled = !aligned.aligned || !!graphicsAvailabilityProblem();
    }
    updateModelCalibrationComparison();
}
let calibrationTaskPreview = null;
const calibrationModalInertNodes = new Set();
function restoreCalibrationTaskPreview() {
    if (!calibrationTaskPreview) return;
    const {node, marker} = calibrationTaskPreview; calibrationTaskPreview = null;
    if (marker.isConnected) marker.replaceWith(node);
    else node.remove();
    const slot = $('model-calibration-task-preview'); if (slot) slot.hidden = true;
    modelView?.resize?.();
}
function setCalibrationTaskModalState(show, previewTask = false) {
    const overlay = $('model-calibration-overlay'), slot = $('model-calibration-task-preview');
    const window = overlay?.querySelector('.model-calibration-task-window');
    window?.classList.toggle('is-preview-task', show && previewTask);
    if (!show || !previewTask || !['calibration','geometry'].includes(state.page) || !slot) restoreCalibrationTaskPreview();
    else if (!calibrationTaskPreview) {
        const node = $('page')?.querySelector('.workbench-model');
        if (node) {
            const marker = document.createComment('calibration preview original position');
            node.before(marker); calibrationTaskPreview = {node, marker};
            slot.hidden = false; slot.append(node); modelView?.resize?.();
        }
    }
    const shells = show ? new Set(document.querySelectorAll('.shell')) : new Set();
    for (const node of calibrationModalInertNodes) if (!shells.has(node)) { node.inert = false; calibrationModalInertNodes.delete(node); }
    for (const node of shells) if (!node.inert) { node.inert = true; calibrationModalInertNodes.add(node); }
}
function activeCalibrationTaskWindow() {
    const status = state.telemetry?.model_calibration;
    if (modelCalibrationLongRunning(status) && status.phase !== 'teaching') return {kind:'model', status};
    const legacy = state.telemetry?.calibration;
    if (legacy?.kind === 'friction_replaying' && ['replaying','fitting'].includes(legacy.phase)) return {kind:'friction', status:legacy};
    const replay = state.telemetry?.teach_replay;
    if (state.page === 'geometry' && state.geometryTab === 'repeat' && replay?.phase === 'playing') return {kind:'geometry', status:replay};
    return null;
}
function calibrationTaskJointRows() {
    const snapshot = state.telemetry || {}, names = snapshot.joint_names || [];
    const probe = gainProbeGhostSource()?.q;
    const reference = probe || snapshot.joint?.nominal_ref_pos || snapshot.joint?.ref_pos;
    const fresh = telemetryFresh(snapshot);
    return names.map((name,index) => {
        const target = Array.isArray(reference) && reference.length === names.length && Number.isFinite(reference[index]) ? reference[index] : null;
        const actual = fresh && Number.isFinite(snapshot.joint?.pos?.[index]) ? snapshot.joint.pos[index] : null;
        return `<tr><th>${esc(name)}</th><td>${jointDisplayValue(actual,name,'position',3)}</td><td>${jointDisplayValue(target,name,'position',3)}</td><td>${jointDisplayValue(actual !== null && target !== null ? target-actual : null,name,'position',3)}</td></tr>`;
    }).join('');
}
async function calibrationTaskAction(action) {
    const task = activeCalibrationTaskWindow(); if (!task) return;
    const epoch = state.workbenchEpoch;
    if (task.kind === 'model') return modelCalibrationQuick(({ 'stop-recording':'model_calibration_teach_stop', cancel:'model_calibration_cancel' })[action] || action);
    return blockingOperation(busyText('正在更新标定任务状态','Updating calibration task'), async () => {
        if (action === 'hold' || action === 'deactivate') await workbenchRequest(action,{});
        else if (task.kind === 'geometry') {
            const result = await geometryRepeatRequest(action === 'stop-recording' ? 'record_stop' : 'stop');
            if (epoch !== state.workbenchEpoch) return;
            state.teachRuntime = result; if (state.telemetry) state.telemetry.teach_replay = result;
        } else {
            const result = await workbenchRequest(action === 'stop-recording' ? 'friction_record_stop' : 'calibration_cancel',{});
            if (epoch !== state.workbenchEpoch) return;
            if (state.telemetry) state.telemetry.calibration = result;
        }
        if (epoch === state.workbenchEpoch) updateModelCalibrationTaskOverlay();
    });
}
function updateAuxiliaryCalibrationTaskWindow(task) {
    const en = state.language === 'en', status = task.status;
    const recording = status.phase === 'recording', geometry = task.kind === 'geometry';
    const passes = Number(status.completed || 0), total = Number(status.plan?.total_passes || 0);
    const progress = total > 0 ? Math.max(0,Math.min(1,passes / total)) : 0;
    $('model-calibration-task-kicker').textContent = geometry ? busyText('定点重复到位测试','Point repeatability test') : t('frictionCalibration');
    $('model-calibration-task-title').textContent = recording ? busyText('示教记录中','Recording demonstration') : status.phase === 'fitting' ? busyText('参数拟合中','Fitting parameters') : busyText('沿示教轨迹采集','Collecting along the demonstrated path');
    const bar = $('model-calibration-task-progress-bar');
    bar.parentElement.classList.toggle('is-indeterminate',total <= 0); bar.style.width = total > 0 ? `${progress * 100}%` : '35%';
    $('model-calibration-task-progress').textContent = total > 0 ? `${busyText('已完成单程','Completed passes')} ${passes} / ${total}` : recording ? busyText('手动示教，结束后检查轨迹','Record manually, then check the trajectory') : busyText('正在等待采集完成','Waiting for acquisition to complete');
    $('model-calibration-task-samples').textContent = `${busyText('轨迹样本','Trajectory samples')} ${status.trajectory_samples ?? status.samples ?? '—'}`;
    $('model-calibration-task-feedback').textContent = `${t('feedbackAge')} ${Number.isFinite(state.telemetry?.feedback_age_ms) ? state.telemetry.feedback_age_ms.toFixed(0) + ' ms' : '—'}`;
    $('model-calibration-task-id').textContent = `${t('taskIdentifier')} ${status.task_id || status.plan?.kind || (geometry ? 'geometry-repeat' : status.kind)}`;
    $('model-calibration-task-error').textContent = status.error ? localizedError(status.error) : '';
    $('model-calibration-task-stop-teach').hidden = !recording;
    $('model-calibration-task-stop-teach').textContent = geometry ? busyText('到达安全点并结束记录','Finish recording at the safe point') : t('stopRecording');
    $('model-calibration-task-pause').hidden = true; $('model-calibration-task-resume').hidden = true;
    $('model-calibration-task-hold').disabled = state.telemetry?.robot_state !== 'ACTIVE';
    $('model-calibration-task-disable').disabled = state.telemetry?.robot_state === 'INACTIVE';
}
function updateModelCalibrationTaskOverlay() {
    const overlay = $('model-calibration-overlay'); if (!overlay) return;
    const task = activeCalibrationTaskWindow(), status = task?.status;
    const show = !priorityErrorOpen && !!task && state.telemetry?.robot_state !== 'FAULT';
    overlay.hidden = !show; overlay.setAttribute('aria-hidden', show ? 'false' : 'true');
    const adapting = status?.phase === 'gain_adaptation';
    setCalibrationTaskModalState(show, true);
    refreshGainProbeStatus(task?.kind === 'model' ? status : {});
    if (!show) return;
    const rows = $('model-calibration-task-joints'); if (rows) rows.innerHTML = calibrationTaskJointRows();
    if (task.kind !== 'model') { updateAuxiliaryCalibrationTaskWindow(task); return; }
    const passes = Math.max(0, Number(status.gain_adaptation_passes || 0));
    const revisions = Math.max(0, Number(status.gain_adaptation_revisions || 0));
    const maxGainPasses = Number(status.gain_adaptation_max_passes || 0);
    const progress = Math.max(0, Math.min(1, Number(status.progress || 0)));
    const jointPath = adapting && status.gain_protocol === 'taught_trajectory';
    const budget = Number(status.gain_time_budget_s || 0), elapsed = Number(status.gain_elapsed_s || 0);
    const budgetProgress = budget > 0 ? Math.max(0,Math.min(1,elapsed / budget)) : 0;
    if ($('model-calibration-task-kicker')) $('model-calibration-task-kicker').textContent = adapting ? busyText('执行器增益自动标定', 'Automatic actuator gain calibration') : busyText('标定任务', 'Calibration task');
    $('model-calibration-task-title').textContent = modelCalibrationPhaseText(status.phase);
    $('model-calibration-task-progress-bar').parentElement.classList.toggle('is-indeterminate', adapting && !jointPath);
    $('model-calibration-task-progress-bar').style.width = jointPath ? `${Math.round(budgetProgress * 100)}%` : adapting ? '35%' : `${Math.round(progress * 100)}%`;
    $('model-calibration-task-progress').textContent = jointPath
        ? `${busyText('自动阶段用时','Automatic stage elapsed')} ${elapsed.toFixed(1)} / ${budget.toFixed(1)} s · ${busyText('剩余','Remaining')} ${Number(status.gain_remaining_s || 0).toFixed(1)} s · ${busyText('整轨回放','Full-path pass')} ${Number(status.gain_path_pass_current || 0)} / ${Number(status.gain_path_pass_limit || 0)} · ${busyText('增益调整','Gain revisions')} ${revisions}`
        : adapting
        ? (state.language === 'en' ? `Completed ${passes} measurement windows${maxGainPasses > 0 ? ' · planned upper bound ' + maxGainPasses : ''} · revisions ${revisions}` : `已完成 ${passes} 个测量窗${maxGainPasses > 0 ? ' · 计划上限 ' + maxGainPasses : ''} · 增益调整 ${revisions} 次`)
        : status.phase === 'teaching' ? `${state.language === 'en' ? 'Free teaching' : '自由示教'} ${Number(status.teaching_wall_duration_s || 0).toFixed(1)} s · ${state.language === 'en' ? 'No duration cap' : '无时长上限'}`
            : `${Math.round(progress * 100)}% · ${Number(status.completed || 0)}/${Number(status.total || 0)}`;
    $('model-calibration-task-samples').textContent = adapting ? `${gainPurposeLabel(status.gain_active_target)} · ${jointPath ? busyText('全部关节联合测量','All joints measured together') : busyText('原生计划上限估时', 'Native plan upper-bound estimate')} ${Number(status.estimated_duration_s || 0) > 0 ? Number(status.estimated_duration_s).toFixed(1) + ' s' : '—'}` : `${t('acceptedDroppedFrames')} ${status.recorder?.accepted ?? '—'} / ${status.recorder?.dropped ?? '—'} · ${t('validStaticSamples')} ${status.valid_static_samples ?? '—'}`;
    $('model-calibration-task-feedback').textContent = `${t('feedbackAge')} ${Number.isFinite(state.telemetry?.feedback_age_ms) ? state.telemetry.feedback_age_ms.toFixed(0) + ' ms' : '—'}`;
    $('model-calibration-task-id').textContent = `${t('taskIdentifier')} ${status.task_id || '—'}`;
    $('model-calibration-task-error').textContent = (status.error ? localizedError(status.error) : '') || (status.recorder?.data_gap ? (state.language === 'en' ? 'Recorded data contains gaps' : '记录数据存在缺口') : status.recorder?.write_failed ? (state.language === 'en' ? 'Raw data write failed' : '原始数据写入失败') : '');
    $('model-calibration-task-stop-teach').hidden = status.phase !== 'teaching';
    $('model-calibration-task-stop-teach').textContent = t('stopTeaching');
    $('model-calibration-task-pause').hidden = status.gain_protocol === 'taught_trajectory' || !['gain_adaptation', 'static_reverse', 'static_forward', 'gravity_fitting', 'friction_reverse_slow', 'friction_forward_slow', 'friction_reverse_fast', 'friction_forward_fast', 'friction_fitting'].includes(status.phase);
    $('model-calibration-task-resume').hidden = status.phase !== 'paused';
    $('model-calibration-task-resume').disabled = !!graphicsAvailabilityProblem();
    $('model-calibration-task-hold').disabled = state.telemetry?.robot_state !== 'ACTIVE';
    $('model-calibration-task-disable').disabled = state.telemetry?.robot_state === 'INACTIVE';
}
function modelCalibrationTaskParams(params = {}) {
    const taskId = state.telemetry?.model_calibration?.task_id || '';
    return taskId ? { ...params, task_id: taskId } : { ...params };
}
async function finishTeachingSafely(cleanupOnly = false) {
    const epoch=state.workbenchEpoch;
    return blockingOperation(busyText('正在结束示教并同步任务状态...', 'Closing demonstration and syncing task state...'), async () => {
        const method=cleanupOnly?'model_calibration_cancel':'model_calibration_teach_stop';
        try {
            const result=await workbenchRequest(method,modelCalibrationTaskParams());
            if(epoch===state.workbenchEpoch && result && state.telemetry) {
                state.telemetry.model_calibration=result;
                state.modelCalibrationDirectory=result.directory || '';
            }
        } catch(error) {
            if(epoch===state.workbenchEpoch)throw error;
        } finally {
            if(epoch===state.workbenchEpoch) {
                try {
                    const latest=await workbenchRequest('model_calibration_status',{});
                    if(epoch===state.workbenchEpoch && latest && state.telemetry) {
                        state.telemetry.model_calibration=latest;
                        state.modelCalibrationDirectory=latest.directory || '';
                    }
                } catch { /* Preserve the original close error */ }
                if(epoch===state.workbenchEpoch)render();
            }
        }
    });
}

async function modelCalibrationQuick(method, params = {}) {
    const epoch=state.workbenchEpoch;
    const before=state.telemetry?.model_calibration?.phase;
    try {
        const requestParams = method === 'model_calibration_teach_begin' ? params : modelCalibrationTaskParams(params);
        const result = await workbenchRequest(method, requestParams);
        if(epoch!==state.workbenchEpoch)return null;
        if (method.startsWith('model_calibration_') && result && state.telemetry) { state.telemetry.model_calibration = result; if (result.directory) state.modelCalibrationDirectory = result.directory; }
        if (result?.phase !== before && ['waiting_replay_confirmation', 'complete', 'cancelled', 'failed'].includes(result?.phase)) render();
        else { updateModelCalibrationTaskOverlay(); updateModelCalibrationLive(); }
        return result;
    } catch (error) {
        if(epoch!==state.workbenchEpoch)return null;
        // Refresh immediately: teach_stop can transition into FAILED before
        // the next telemetry event. Never leave a stale teaching mask onscreen.
        try {
            const latest = await workbenchRequest('model_calibration_status', {});
            if (epoch===state.workbenchEpoch && latest && state.telemetry) state.telemetry.model_calibration = latest;
        } catch { /* preserve the original request error */ }
        if(epoch!==state.workbenchEpoch)return null;
        showPriorityError(error);
        if (state.telemetry?.model_calibration?.phase !== before) render();
        else { updateModelCalibrationTaskOverlay(); updateModelCalibrationLive(); }
        return null;
    }
}
async function resumeModelCalibration() {
    try { return await modelCalibrationQuick('model_calibration_resume', { confirmed: false }); }
    catch { return null; }
}
function advancedTrustedLinks(record) {
    const prior = record?.result?.gravity_result || {};
    const options = record?.metadata?.calibration_options || {};
    const mode = prior.identification_mode || options.mode;
    const selected = prior.locked_links || options.locked_links;
    if (mode !== 'local' || !Array.isArray(selected)) return [];
    const valid = new Set(Array.isArray(record?.identifiable_links) ? record.identifiable_links.map(link => link.name) : []);
    return [...new Set(selected.filter(name => valid.has(name)))];
}
function calibrationSelectableLinks() {
    // Historical tasks must use their verified original URDF snapshot, never
    // silently substitute a potentially modified live robot model
    const historic = state.modelCalibrationOffline;
    if (historic) return Array.isArray(historic.identifiable_links) ? historic.identifiable_links : [];
    const live = state.telemetry?.model_calibration?.identifiable_links;
    if (Array.isArray(live) && live.length) return live;
    // An active native session is authoritative and must expose its own
    // Pinocchio link mapping: a GUI guess must not override an outdated Core
    if (workbenchConnected()) return [];
    // Without an active session, the same read-only URDF preview used by the
    // model workbench can populate the selector without enabling hardware
    const preview = state.modelData;
    if (!preview || (state.config.profile && preview.profile !== state.config.profile)) return [];
    const controlled = new Set(preview.native?.config?.joint_names || []);
    const links = new Map((preview.model?.links || []).map(item => [item.name, item]));
    const selections = [];
    for (const joint of preview.model?.joints || []) {
        if (!controlled.has(joint.name)) continue;
        const link = links.get(joint.child);
        const mass = Number(link?.inertial?.mass);
        if (!link?.inertial || !Number.isFinite(mass) || mass <= 0) continue;
        selections.push({ name: link.name, joint: joint.name, mass });
    }
    return selections;
}
async function ensureCalibrationSelectableLinks() {
    let choices = calibrationSelectableLinks();
    if (choices.length) return choices;
    if (!workbenchConnected() && !state.modelCalibrationOffline && state.info?.available?.model) {
        await loadModelData();
        choices = calibrationSelectableLinks();
    }
    if (!choices.length) {
        throw new Error(workbenchConnected()
            ? (state.language === 'en' ? 'The connected Core did not report identifiable links; rebuild/restart the Core before local calibration' : '当前 Core 未提供可辨识 Link，请重新编译并连接最新 Core 后再选择局部辨识')
            : (state.language === 'en' ? 'Load a model or a saved calibration record with a verified URDF first' : '请先加载可用模型或带原始 URDF 快照的标定记录'));
    }
    return choices;
}
function editCalibrationLockedLinks({ onCancel = null } = {}) {
    const choices = calibrationSelectableLinks();
    if (!choices.length) throw new Error(state.language === 'en' ? 'No identifiable links available; load a model or recorded calibration' : '当前没有可辨识 Link，请加载模型或历史标定记录');
    const current = state.modelCalibrationOptions;
    const allowed = choices.map(x => x.name);
    const checked = new Set(current.locked_links || []);
    const dialog = $('confirm-dialog');
    dialog.innerHTML = `<h3>${state.language === 'en' ? 'Trusted Link parameters' : '选择可信 Link'}</h3><p>${state.language === 'en' ? 'Checked = preserve original first moments; only controlled inertial links are listed. Massless tool frames, fixed attachments and unsampled branches cannot be identified independently' : '勾选代表锁定原始质量一阶矩，未勾选代表允许校正；清单只显示受控关节关联的有效惯量 Link，无质量 tool0、固定附属及未采样分支不能单独辨识'}</p><div class="calibration-link-picker">${choices.map(x => `<label class="calibration-link-option"><input type="checkbox" data-gravity-locked="${esc(x.name)}" ${checked.has(x.name) ? 'checked' : ''}><span>${esc(x.name)}</span><small>${x.joint ? esc(x.joint) + ' · ' : ''}${Number.isFinite(x.mass) ? Number(x.mass).toFixed(4) + ' kg' : ''}</small></label>`).join('')}</div><div class="actions"><button id="cal-link-lock-all" class="button">${state.language === 'en' ? 'Lock all' : '全部锁定'}</button><button id="cal-link-unlock-all" class="button">${state.language === 'en' ? 'Unlock all' : '全部解锁'}</button></div><div class="actions"><button id="cal-link-cancel" class="button">${t('cancelTask')}</button><button id="cal-link-confirm" class="button primary">${state.language === 'en' ? 'Confirm' : '确认选择'}</button></div>`;
    let confirmed = false;
    dialog.onclose = () => {
        dialog.onclose = null;
        if (!confirmed && onCancel) onCancel();
    };
    dialog.showModal();
    const inputs = () => [...dialog.querySelectorAll('[data-gravity-locked]')];
    $('cal-link-lock-all').onclick = () => inputs().forEach(x => x.checked = true);
    $('cal-link-unlock-all').onclick = () => inputs().forEach(x => x.checked = false);
    $('cal-link-cancel').onclick = () => dialog.close();
    $('cal-link-confirm').onclick = () => {
        const selected = inputs().filter(x => x.checked).map(x => x.dataset.gravityLocked);
        if (selected.length >= allowed.length) { toast(state.language === 'en' ? 'At least one link must be unlocked' : '至少保留一个允许校正的 Link', true); return; }
        current.locked_links = selected;
        state.modelCalibrationSavePreview = null;
        confirmed = true;
        dialog.close(); render();
    };
}
function captureModelCalibrationOptions({ validate = true } = {}) {
    const read = (id, fallback) => { const value = Number($(id)?.value); return Number.isFinite(value) ? value : fallback; };
    state.modelCalibrationOptions = {
        mode: state.modelCalibrationOptions.mode || 'global',
        load_description: $('model-cal-load')?.value.trim() || state.modelCalibrationOptions.load_description || 'unspecified',
        locked_links: state.modelCalibrationOptions.mode === 'local' ? [...(state.modelCalibrationOptions.locked_links || [])] : [],
        pose_budget: Math.round(read('model-cal-pose-budget', state.modelCalibrationOptions.pose_budget)),
        validation_fraction: read('model-cal-validation', state.modelCalibrationOptions.validation_fraction),
        regularization: read('model-cal-regularization', state.modelCalibrationOptions.regularization),
        svd_relative_threshold: read('model-cal-svd', state.modelCalibrationOptions.svd_relative_threshold),
        minimum_information_score: read('model-cal-information', state.modelCalibrationOptions.minimum_information_score),
    };
    // Validation is for starting or recomputing a task, not navigating the UI
    // while a local selection is incomplete or awaiting the picker dialog
    if (validate && state.modelCalibrationOptions.mode === 'local') {
        const allowed = calibrationSelectableLinks().map(item => item.name);
        const locked = state.modelCalibrationOptions.locked_links;
        if (!allowed.length || locked.some(name => !allowed.includes(name)) || locked.length >= allowed.length)
            throw new Error('局部辨识 Link 选择无效，请重新确认可信 Link 清单');
    }
    return { ...state.modelCalibrationOptions };
}
async function modelCalibrationReplayConfirm() {
    if (graphicsAvailabilityProblem()) return;
    const dialog = $('confirm-dialog');
    const plan = currentModelCalibration() || {};
    if (plan.source_task_id && !calibrationAlignment(plan).aligned) { showPriorityError(state.language === 'en' ? 'Replay start not aligned. Use guided alignment and wait for fresh feedback.' : '当前姿态与回放起点不匹配：请先完成引导对齐，并等待新鲜的关节反馈'); return; }
    if (plan.planner_id !== 'local_hermite' || !(Number(plan.single_pass_duration_s) > 0)) { showPriorityError(state.language === 'en' ? 'Native Core was not rebuilt. Reinstall before replay.' : 'C++ Core 仍是旧版，请重新编译安装后再执行自动回放'); return; }
    const gainPlan = state.calibrationEntry === 'gain';
    const gainRequirements = gainPlan ? {local_range_rad:.04, ...readGainRequirements()} : null;
    const motionRange = gainPlan ? (gainRequirements.local_range_rad * 180 / Math.PI).toFixed(2) : '';
    let gainSchedule = null;
    const gainMode = state.gainMode;
    if (gainPlan) {
        const taskId = plan.task_id, epoch = state.workbenchEpoch;
        try {
            gainSchedule = await workbenchRequest('model_calibration_gain_plan', modelCalibrationTaskParams({gain_mode:gainMode}));
        } catch (error) { showPriorityError(error); return; }
        if (epoch !== state.workbenchEpoch || taskId !== currentModelCalibration()?.task_id || state.calibrationEntry !== 'gain' || gainMode !== state.gainMode) return;
        if (gainSchedule?.valid !== true || !(Number(gainSchedule.budget_s) > 0) || !(Number(gainSchedule.estimated_duration_s) > 0)) {
            showPriorityError(gainSchedule?.reason || busyText('当前增益回放计划无效，请检查轨迹与运行配置','The gain replay plan is invalid; check the trajectory and runtime configuration')); return;
        }
    }
    const oneWayDuration = Number(plan.single_pass_duration_s || 0);
    const estimate = gainPlan && oneWayDuration > 0
        ? Number(gainSchedule.estimated_duration_s)
        : Number(plan.estimated_duration_s || 0);
    const teaching = Number(plan.teaching_wall_duration_s || 0);
    const totalEstimate = gainPlan ? teaching + estimate : Number(plan.estimated_total_duration_s || teaching + estimate);
    const oneWay = Number(plan.single_pass_duration_s || 0);
    const poses = Array.isArray(plan.pose_targets) ? plan.pose_targets.length : 0;
    const formula = gainPlan
        ? `${state.language === 'en' ? 'Safe replay per pass' : '安全回放单程'} ${oneWay.toFixed(1)} s · ${state.language === 'en' ? 'Full-path pass limit' : '整轨回放上限'} ${Number(gainSchedule.max_path_passes)} · ${state.language === 'en' ? 'Automatic-stage deadline' : '自动阶段时限'} ${Number(gainSchedule.budget_s).toFixed(1)} s (${Number(gainSchedule.budget_source_duration_s).toFixed(1)} s × ${Number(gainSchedule.budget_multiplier)})`
        : `${state.language === 'en' ? 'Plan' : '计算'}：T自动 ≈ 8 × ${oneWay.toFixed(1)} + 2 × 1.15 × ${poses} + 2`;
    dialog.innerHTML = `<h3>${gainPlan ? (state.language === 'en' ? 'Automatic actuator gain calibration' : '执行器增益自动标定') : t('modelCalibration')}</h3><p>${gainPlan ? (state.language === 'en' ? 'The robot will replay the entire demonstrated path in alternating directions. All joints are measured together; candidate adjustment and independent validation share one time budget. Compliant tracking is tested at 65% speed' : '机械臂将沿整条示教路径往返，所有关节联合测量；候选调整与独立验收共用同一时间预算，柔性跟踪按 65% 速度测试') : (state.language === 'en' ? 'Six traversals: reverse/forward static holds, then reverse/forward at half speed and full planned speed' : '六次遍历：双向静态保持，随后半速双向、计划速度双向；控制限幅和取消路径保持启用')}</p>${gainPlan ? `<p class="gain-motion-range">${state.language === 'en' ? `The demonstrated path defines the tracking motion range. Additional drag-release tests stay within ±${motionRange}° per joint of the pose reached after replay, further constrained by Safety limits` : `跟踪测试使用整条示教路径的运动范围；额外拖拽释放测试限制在回放结束姿态附近，每轴最多 ±${motionRange}°，并受 Safety 限制约束`}</p>` : ''}<div class="callout"><b>${state.language === 'en' ? 'Estimated automatic stage' : '预计自动阶段'}：${estimate > 0 ? estimate.toFixed(1) + ' s' : '—'} · ${state.language === 'en' ? 'Including teaching' : '加上示教已用'} ${estimate > 0 ? totalEstimate.toFixed(1) : teaching.toFixed(1)} s</b><div>${formula}</div>${gainPlan && gainSchedule.warnings?.length ? `<ul class="gain-quality-warning">${gainSchedule.warnings.map(value=>`<li>${esc(gainWarningLabel(value))}</li>`).join('')}</ul>` : ''}<div>${gainPlan ? (state.language === 'en' ? 'The plan includes independent measurements, hold and drag tests. At the deadline, motion stops in hold; completed uses can be applied or exported and unfinished uses remain unmeasured' : '计划包含独立测量、保持与拖拽测试；到达时限后进入保持，已完成测量的用途可应用或导出，未完成用途保留为未测量') : (state.language === 'en' ? 'Feedback waiting and settling can extend actual duration' : '实际反馈等待与停稳可能延长耗时')}</div></div><label class="confirmation"><input id="model-calibration-released" type="checkbox"><span>${state.language === 'en' ? 'I checked the robot, load, cables, and the entire replay area, and I have fully released the robot' : '我已检查机械臂、负载、线缆和整条回放路径，并已完全松手'}</span></label><div class="actions"><button id="model-calibration-confirm-cancel" class="button">${t('cancel')}</button><button id="model-calibration-confirm-start" class="button danger" disabled>${t('modelCalibrationConfirmReplay')}</button></div>`;
    dialog.showModal(); $('model-calibration-released').onchange = () => $('model-calibration-confirm-start').disabled = !$('model-calibration-released').checked || !!graphicsAvailabilityProblem(); $('model-calibration-confirm-cancel').onclick = () => dialog.close();
    $('model-calibration-confirm-start').onclick = () => { if (graphicsAvailabilityProblem()) return; dialog.close(); blockingOperation(busyText('正在启动自动采集任务...', 'Starting automatic acquisition...'), async () => { if (gainPlan) { state.calibrationToolSelected.gain = null; state.gainCandidateLoadAttempted = ''; state.gainCandidateLoadError = ''; } const result = await workbenchRequest('model_calibration_start', modelCalibrationTaskParams({ released: true, workflow: gainPlan ? 'gain' : 'gravity', gain_mode: gainMode, ...(gainPlan ? {gain_requirements:gainRequirements,gain_motion_range_confirmed:true,gain_trajectory_confirmed:true} : {}) })); if (state.telemetry) state.telemetry.model_calibration = result; state.modelCalibrationOffline = null; state.modelCalibrationCandidateSource = 'online'; render(); updateModelCalibrationTaskOverlay(); }); };
}
async function resumeModelCalibrationWithConfirmation() {
    if (graphicsAvailabilityProblem()) return null;
    try {
        const result = await workbenchRequest('model_calibration_resume', modelCalibrationTaskParams({ confirmed: false })); if (state.telemetry) state.telemetry.model_calibration = result; updateModelCalibrationTaskOverlay(); return result;
    } catch (error) {
        if (!String(error.message).includes('explicit resume confirmation')) { toast(error, true); return null; }
        const dialog = $('confirm-dialog'); dialog.innerHTML = `<h3>${t('resumeTask')}</h3><p>${state.language === 'en' ? 'The robot moved while paused; confirm the current position is safely consistent with the paused path before continuing' : '暂停期间机械臂位置发生变化，请确认当前位置与暂停路径安全一致后继续'}</p><div class="actions"><button id="model-cal-resume-cancel" class="button">${t('cancel')}</button><button id="model-cal-resume-confirm" class="button danger">${state.language === 'en' ? 'Confirm and continue' : '确认继续'}</button></div>`; dialog.showModal(); $('model-cal-resume-cancel').onclick = () => dialog.close(); $('model-cal-resume-confirm').onclick = () => { if (graphicsAvailabilityProblem()) return; dialog.close(); modelCalibrationQuick('model_calibration_resume', { confirmed: true }); };
        return null;
    }
}

function drawWorkbenchChart() { const canvas = $('workbench-chart'); if (!canvas) return; const ctx = canvas.getContext('2d'); const rect = canvas.getBoundingClientRect(); canvas.width = Math.max(400, Math.floor(rect.width * devicePixelRatio)); canvas.height = Math.floor(180 * devicePixelRatio); ctx.scale(devicePixelRatio, devicePixelRatio); const w = rect.width, h = 180; ctx.clearRect(0, 0, w, h); const data = state.telemetryHistory.slice(-180); if (data.length < 2) return; const values = data.map(x => Number(x.joint?.residual_raw?.[0] ?? 0)); const max = Math.max(0.1, ...values.map(Math.abs)); ctx.beginPath(); values.forEach((v, i) => { const x = i / (values.length - 1) * w, y = h / 2 - v / max * (h * 0.42); i ? ctx.lineTo(x, y) : ctx.moveTo(x, y); }); ctx.strokeStyle = '#df7b00'; ctx.lineWidth = 1.5; ctx.stroke(); ctx.fillStyle = '#687080'; ctx.fillText('joint1 residual_raw', 8, 16); }

function runPage() {
    return pageHead('runtimeTitle', 'runtimeDesc') + `<div class="mode-grid mode-grid-three">${['terminal', 'hardware', 'moveit'].map(mode => `<button class="mode-card ${state.mode === mode ? 'active' : ''}" data-mode="${mode}"><span class="mode-type ${state.info?.available?.[mode] ? 'is-ready' : 'is-unavailable'}">${t(state.info?.available?.[mode] ? 'ready' : 'unavailable')}</span>${icon(mode)}<h3>${t(mode)}</h3><p>${t(mode + 'Desc')}</p></button>`).join('')}</div><div class="card launch-panel"><div><h3>${t('selectedRobot')} · ${esc(state.config.profile || '—')}</h3><p>${esc(state.info?.available?.[state.mode] ? t('real') : state.info?.unavailable_reasons?.[state.mode] || t('modeMissing'))}</p><div class="launch-command">${esc(state.info?.profile_file || state.config.profile_file || '')}</div></div>${button('launch-session', 'launch', 'run', 'primary')}</div>`;
}
function diagnosticsPage() {
    const checks = [{ name: 'installation', ok: !!state.status.installed, detail: state.status.prefix || t('notInstalled') }, ...(state.info?.checks || [])];
    const connect = `<div class="workspace-actions">${workspaceSessionActionHtml()}${button('check-now', 'checkNow', 'refresh')}</div>`;
    const snapshot = workbenchConnected() && state.telemetry?.valid ? `<section class="card diagnostic-live"><div class="card-head"><div><h3>${state.language === 'en' ? 'Live actuator diagnostics' : '实时执行器诊断'}</h3><p>${state.language === 'en' ? 'Feedback from the current native session' : '来自当前原生控制会话的反馈'}</p></div></div><div class="card-body">${workbenchStatusBar()}${diagnosticsPanelHtml()}</div></section>` : `<div class="callout">${state.language === 'en' ? 'No live hardware feedback, check installation and connection or connect the workspace explicitly' : '当前没有实时硬件反馈，请检查连接，必要时明确启动控制工作台会话'}</div>`;
    return pageHead('diagTitle', 'diagDesc', connect) + `<details class="diagnostic-settings" open><summary>${state.language === 'en' ? 'Profile and connection overrides' : 'Profile 与连接覆盖设置'}</summary>${connectionSettingsHtml()}</details>` + `<div class="grid">${card('checks', 'checkSub', 'diagnostics', checks.map(c => `<div class="diag-row"><span class="dot ${c.ok ? 'ready' : 'error'}"></span><span class="name">${t(c.name)}</span><span class="detail">${esc(c.detail)}</span><span class="state ${c.ok ? '' : 'error'}">${t(c.ok ? 'pass' : 'fail')}</span></div>`).join(''))}${card('installHelp', 'installHelpSub', 'terminal', `<p class="hint">${t('installHint')}</p><p>${busyText('独立运行 Core 与图形工作台','Standalone Core and desktop workspace')}</p><pre class="install-command"><code>./install.sh --preset standalone --yes</code></pre><p>${busyText('可选 ROS2 与 MoveIt 适配层','Optional ROS2 and MoveIt adapters')}</p><pre class="install-command"><code>./install.sh --preset ros2 --yes</code></pre><div class="callout">${t('validationHint')}</div>`)}</div>${snapshot}`;

}
function settingsPage() {
    return pageHead('settingsTitle', 'settingsDesc', `<button id="settings-back" class="button">${busyText('返回', 'Back')}</button>`) + `<div class="grid settings-grid">${card('appearance', 'appearanceSub', 'sun', `<div class="segmented">${['system', 'dark', 'light'].map(theme => `<button data-theme="${theme}" aria-pressed="${theme === state.theme}" class="${theme === state.theme ? 'active' : ''}">${t(theme)}</button>`).join('')}</div>`)}${card('language', 'languageSub', 'settings', `<div class="segmented"><button data-language="zh-CN" aria-pressed="${state.language === 'zh-CN'}" class="${state.language === 'zh-CN' ? 'active' : ''}">简体中文</button><button data-language="en" aria-pressed="${state.language === 'en'}" class="${state.language === 'en' ? 'active' : ''}">English</button></div>`)}<section class="card graphics-settings"><div class="card-head">${icon('model')}<div><h3>${busyText('三维渲染', '3D rendering')}</h3><p>${busyText('选择模型预览的渲染方式', 'Choose how model previews are rendered')}</p></div></div><div class="card-body" id="graphics-settings-body">${graphicsSettingsBodyHtml()}</div></section></div><div class="callout">${t('prefsNote')}</div>`;
}

async function mountDescriptionView() {
    return mountPageModelView('description', 'description-viewport', () => state.descriptionAnalysis?.preview, view => {
        view.setLayers({ visual: true, collision: false, linkFrames: true, jointAxes: true, com: false, inertia: false, labels: true });
    });
}
async function refreshLibrary() {
    const data = await api.request('profile_library'); state.library = data.profiles || []; state.libraryPath = data.path || '';
}
async function openLibraryEntry(id) {
    const entry = state.library.find(x => x.id === id); if (!entry) throw new Error('Profile Library entry not found');
    resetProfileWorkspaceState();
    state.activeLibraryId = id; state.gainRequirements = null; state.createdProfileWizard = false; state.source = entry.source === 'builtin' ? 'builtin' : 'external';

    state.config = { profile: entry.profile, profile_file: entry.source === 'builtin' ? '' : entry.profile_file, resource_paths: entry.resource_paths || '', serial_port: '', baudrate: '', bus: '' };
    const listing = await api.request('profiles', state.config); state.profiles = listing.profiles || []; state.info = null; state.modelData = null; state.profileEditor = null; state.capabilityProfile = null; state.capabilityProfileError = ''; state.admittance = null; state.tuneDraft = null; state.teachTrajectories = []; state.teachSelectedId = ''; state.teachLoadedId = ''; state.ghostRequestKey = '';
    await inspect(); state.readiness = await api.request('readiness', { config: state.config });
    state.profileDirty = false; clearProfileFormState(); state.profileGroup = 'model';
    state.page = 'lifecycle';
    await save(); shell();
}
async function importProfileSource(source) {
    const found = await blockingOperation(busyText('正在检查 Profile...', 'Inspecting Profile...'), () => api.request('profile_source_inspect', { source }));
    if (!found) return;
    const candidates = found.candidates || []; if (!candidates.length) throw new Error('No Profile was found');
    let selected = candidates[0];
    if (candidates.length > 1) {
        selected = await new Promise(resolve => {
            const dialog = $('confirm-dialog');
            dialog.innerHTML = `<h3>${t('importExternalProfile')}</h3><p>${state.language === 'en' ? 'Choose the Profile to register' : '选择要加入 Profile Library 的 Profile'}</p><div class="field"><select id="import-profile-choice">${candidates.map((x, i) => `<option value="${i}">${esc(x.profile)} · ${esc(x.profile_file)}</option>`).join('')}</select></div><div class="actions"><button id="import-cancel" class="button">${t('cancel')}</button><button id="import-confirm" class="button primary">${t('addToLibrary')}</button></div>`;
            dialog.showModal();
            $('import-cancel').onclick = () => { dialog.close(); resolve(null); };
            $('import-confirm').onclick = () => { const choice = candidates[Number($('import-profile-choice').value) || 0]; dialog.close(); resolve(choice); };
        });
        if (!selected) return;
    }
    await blockingOperation(busyText('正在导入 Profile...', 'Importing Profile...'), async () => {
        await api.request('profile_library_register', { ...selected, policy: 'suffix' });
        await refreshLibrary();
        render();
    });
}
function collectDescriptionDraft() {
    const d = state.descriptionDraft;
    if ($('description-source')) {
        const source = $('description-source').value.trim();
        if (source !== d.source) d.model_relative = '';
        d.source = source;
    }
    if ($('draft-base')) d.base_frame = $('draft-base').value;
    if ($('draft-tool')) d.tool_frame = $('draft-tool').value;
    if ($('draft-profile')) d.profile = $('draft-profile').value.trim();
    if ($('draft-destination')) d.destination = $('draft-destination').value.trim();
    syncDescriptionChain();
}
async function analyzeDescription() {
    collectDescriptionDraft();
    if (!state.descriptionDraft.source) throw new Error(state.language === 'en' ? 'Choose a Description first' : '请先选择 Description 文件');
    state.descriptionAnalysis = await api.request('description_inspect', { source: state.descriptionDraft.source, model_relative: state.descriptionDraft.model_relative || '' });
    const a = state.descriptionAnalysis, d = state.descriptionDraft;
    d.base_frame = a.roots?.[0] || a.links?.[0] || '';
    d.tool_frame = a.preview?.native?.config?.tool_frame || a.leaves?.[0] || '';
    d.profile = (a.package_name || 'serial_arm').replace(/_description$/, '').replace(/[^A-Za-z0-9_-]/g, '_');
    state.descriptionStep = 1;
    syncDescriptionChain(); render();
}
async function createDescriptionProfile() {
    collectDescriptionDraft();
    const d = state.descriptionDraft;
    if (!d.joint_names.length) throw new Error(state.language === 'en' ? 'Choose a valid controlled chain first' : '请先选择有效控制链');
    if (!d.profile || !/^[A-Za-z0-9_-]+$/.test(d.profile)) throw new Error(state.language === 'en' ? 'Use letters, numbers, underscores or hyphens for the Profile name' : 'Profile 标识只能包含英文数字下划线及连字符');
    if (!d.destination) throw new Error(state.language === 'en' ? 'Choose a destination directory' : '请选择 Profile 保存目录');
    // Do not invent hardware mappings before the user reaches Profile setup
    const result = await api.request('description_create_profile', {
        source: d.source, model_relative: d.model_relative || '', profile: d.profile, destination: d.destination,
        base_frame: d.base_frame, tool_frame: d.tool_frame, joint_names: d.joint_names, hardware_plugin: ''
    });
    resetProfileWorkspaceState();
    state.config = { ...result.config };
    state.source = 'external'; state.info = null; state.modelData = null;
    // Creation succeeds independently of Library registration so the generated files remain recoverable
    try {
        const entry = await api.request('profile_library_register', {
            profile_file: result.profile_file, profile: result.profile,
            resource_paths: result.resource_paths, policy: 'suffix'
        });
        state.activeLibraryId = entry.id;
        await refreshLibrary();
    } catch (error) {
        toast((state.language === 'en' ? 'Profile created, but Library registration failed: ' : 'Profile 已生成，但加入 Library 失败：') + error.message, true);
    }
    await inspect();
    state.readiness = await api.request('readiness', { config: state.config });
    state.profileInteractionGoal = null; state.profileEditor = await api.request('profile_editor_load', { config: state.config });
    state.profileGroup = 'hardware'; state.profileDirty = false; clearProfileFormState();
    state.createdProfileWizard = true; state.descriptionStep = 4;
    state.page = 'profile_setup'; await save(); shell();
}

function collectProfileEditor() {
    const e = state.profileEditor;
    if (!e) return null;
    for (const field of document.querySelectorAll('[data-profile-field]')) {
        if (!field.disabled) { try { readProfileInput(field); } catch { /* Retained below per field */ } }
    }
    const first = Object.keys(state.profileFieldErrors || {})[0];
    if (first) {
        state.profileGroup = profileGroupForField(first);
        state.setupFocus = {page:'profile_setup',field:first};
        state.profileError = state.profileFieldErrors[first];
        render();
        throw new Error(state.profileError);
    }
    return {core_sha:e.core_sha,hardware_sha:e.hardware_sha,profile_sha:e.profile_sha,core:e.core,hardware:e.hardware,profile_entry:e.profile_entry,hardware_imported:e.hardware_imported === true,hardware_import_source:e.hardware_import_source};
}
function clearProfileFormState() {
    state.profileFieldErrors = {}; state.profileRawInputs = {}; state.profileError = ''; state.profileSaveNotice = false;
}
function restoreProfileFields() {
    for (const field of document.querySelectorAll('[data-profile-field]')) {
        const id = JSON.parse(field.dataset.profileField).join('.');
        const raw = state.profileRawInputs?.[id];
        if (raw !== undefined && field.type !== 'checkbox') field.value = raw;
        field.setCustomValidity?.(state.profileFieldErrors?.[id] || '');
    }
}
function setProfileFieldError(input, message) {
    const id = JSON.parse(input.dataset.profileField).join('.');
    state.profileFieldErrors ||= {}; state.profileFieldErrors[id] = message;
    state.profileRawInputs ||= {}; state.profileRawInputs[id] = input.value;
    state.profileDirty = true;
    input.setCustomValidity?.(message);
    const wrapper = input.closest('.parameter-field');
    wrapper?.classList.add('invalid');
    let node = wrapper?.querySelector('[data-field-error]');
    if (!node && wrapper) {node = document.createElement('em');node.dataset.fieldError='';wrapper.append(node);}
    if (node) node.textContent = message;
}

function updateProfileSwitch(input) {
    const switchNode = input.closest('.parameter-switch');
    if (!switchNode) return;
    switchNode.dataset.switchState = input.checked ? 'on' : 'off';
    input.setAttribute('aria-checked', String(input.checked));
}
function readProfileInput(input) {
    const path = JSON.parse(input.dataset.profileField), id = path.join('.'), root = state.profileEditor;
    if (!root || input.disabled) return;
    let target = root;
    for (const key of path.slice(0,-1)) target = target[key];
    const key = path.at(-1), old = target[key];
    let value;
    try {
        if (typeof old === 'boolean') value = input.checked;
        else if (typeof old === 'number') {
            if (!input.value.trim() || !Number.isFinite(Number(input.value))) throw new Error(busyText('请输入有限数值','Enter a finite number'));
            value = Number(input.value);
        } else if (Array.isArray(old)) {
            value = input.value.split(',').map(x=>x.trim());
            if (typeof old[0] === 'number') {
                if (value.some(x=>!x || !Number.isFinite(Number(x)))) throw new Error(busyText('每项都需要有效数字','Each item must be a valid number'));
                value = value.map(Number);
            }
            if (value.length !== old.length) throw new Error(busyText('数组长度不能改变','Keep the array length unchanged'));
        } else if (old === null) {
            if (!input.value.trim()) throw new Error(busyText('请填写配置值','Enter a configuration value'));
            try {value=JSON.parse(input.value);} catch {value=input.value;}
        } else value = input.value;
    } catch (error) {setProfileFieldError(input,`${id} · ${error.message}`);throw error;}
    if (JSON.stringify(old) !== JSON.stringify(value)) state.profileDirty = true;
    target[key] = value;
    if (state.profileFieldErrors) delete state.profileFieldErrors[id];
    if (state.profileRawInputs) delete state.profileRawInputs[id];
    input.setCustomValidity?.('');
    const wrapper = input.closest('.parameter-field');
    wrapper?.classList.remove('invalid');wrapper?.querySelector('[data-field-error]')?.remove();
    if (state.profileError?.startsWith(id)) state.profileError = '';
    const label = $('profile-dirty-state');
    if (label) label.textContent = state.profileDirty ? busyText('有未保存修改','Unsaved changes') : busyText('已加载默认配置','Saved defaults loaded');
}

let previewFrame = 0, previewInFlight = false, previewDirty = false;
let renderedPage = '';
let calibrationRefreshPending = false;
function calibrationDialogOpen() {
    return !!$('confirm-dialog')?.open;
}
function refreshCalibrationAfterDialog() {
    if (!calibrationRefreshPending) return;
    calibrationRefreshPending = false;
    if (state.page === 'calibration' && !calibrationDialogOpen()) render();
}
function refreshCalibrationOrDefer() {
    if (calibrationDialogOpen()) {
        calibrationRefreshPending = true;
        updateWorkbenchLive();
        return;
    }
    render();
}
function queuePreview() { previewDirty = true; if (previewFrame || previewInFlight) return; previewFrame = requestAnimationFrame(async () => { previewFrame = 0; if (previewInFlight) return; previewInFlight = true; previewDirty = false; try { await previewModel(); } catch (error) { toast(error, true); } finally { previewInFlight = false; if (previewDirty) queuePreview(); } }); }
function reducedMotion() { return matchMedia('(prefers-reduced-motion: reduce)').matches; }
async function pageExit() {
    const page = $('page');
    if (!page || reducedMotion()) return;
    page.classList.remove('page-transition-in');
    page.classList.add('page-transition-out');
    await new Promise(resolve => setTimeout(resolve, 70));
}
function pageEnter(pageChanged = true) {
    const page = $('page');
    if (!page || reducedMotion() || !pageChanged) return;
    page.classList.remove('page-transition-out', 'page-transition-in');
    void page.offsetWidth;
    page.classList.add('page-transition-in');
}
let workspaceRenderHost = null, workspaceRenderModel = null, workspaceRenderSubview = '';
function refreshWorkspacePanel() {
    if (!['workbench', 'calibration'].includes(state.page)) return false;
    const host = $('workbench-viewport'), task = document.querySelector('.workbench-task');
    if (!host || !task || host !== workspaceRenderHost || state.modelData !== workspaceRenderModel) return false;
    const subview = state.page + ':' + (state.page === 'calibration' ? state.calibrationEntry : state.workbenchTab);
    const samePanel = subview === workspaceRenderSubview;
    const scroll = task.closest('.content'), top = scroll?.scrollTop || 0;
    const details = samePanel ? new Map([...task.querySelectorAll('details')].map((item, index) => [item.id || item.querySelector('summary')?.textContent.trim() || String(index), item.open])) : new Map();
    const active = document.activeElement, select = active?.closest?.('.studio-select')?.querySelector('select');
    const focusId = samePanel && task.contains(active) ? select?.id || active.id : '';
    task.innerHTML = state.page === 'calibration' ? calibrationPanelHtml() : `<div class="workspace-panel workspace-operation-panel">${robotOperationHtml()}</div><div class="workspace-panel">${workbenchTabs()}</div><div class="workbench-task-content">${workbenchPanelHtml()}</div>`;
    for (const [index, item] of [...task.querySelectorAll('details')].entries()) {
        const key = item.id || item.querySelector('summary')?.textContent.trim() || String(index);
        if (details.has(key)) item.open = details.get(key);
    }
    const head = $('page')?.querySelector('.page-head');
    if (head) head.outerHTML = pageHead(state.page === 'calibration' ? 'calibrationTitle' : 'workbenchTitle', state.page === 'calibration' ? 'calibrationDesc' : 'workbenchDesc', workspaceSessionActionHtml());
    const status = document.querySelector('.workspace-status'); if (status) status.outerHTML = workbenchStatusBar();
    const recovery = $('workspace-recovery-notice'); if (recovery) recovery.innerHTML = workbenchRecoveryNotice();
    const left = document.querySelector('.workbench-model'), overlay = left?.querySelector('.calibration-com-overlay');
    const overlayHtml = state.page === 'calibration' ? calibrationComOverlayHtml() : '';
    if (overlay) overlay.outerHTML = overlayHtml;
    else if (overlayHtml) left.insertAdjacentHTML('beforeend', overlayHtml);
    const note = left?.querySelector('.workbench-model-note'); if (note) note.innerHTML = workspaceModelNoteHtml();
    workspaceRenderSubview = subview;
    statusUI(); bindPage(); window.serialArmStudio?.enhanceSelects();
    if (focusId) {
        const field = $(focusId);
        const focus = field?.classList.contains('studio-native-select') ? field.closest('.studio-select')?.querySelector('.ros-select-trigger') : field;
        if (focus && !focus.disabled) focus.focus({preventScroll:true});
    }
    if (scroll) scroll.scrollTop = top;
    if (state.page === 'calibration') updateModelCalibrationComparison();
    else modelView?.clearGravityComparison?.();
    updateStartGhost(true);
    refreshCalibrationQuickControls(); refreshCalibrationActionControls(); refreshGainApplyControls(); updateModelCalibrationTaskOverlay();
    if (state.setupFocus?.page === state.page) focusSetupTarget();
    if (state.page === 'workbench' && state.workbenchTab === 'monitor') drawWorkbenchChart();
    if (state.modelData && !modelView && !workspaceMount) operation(mountWorkbenchView);
    return true;
}
function render() {
    if (renderedPage === state.page && refreshWorkspacePanel()) return;
    workspaceMount = null;
    pageModelMount = null;
    disposeModelPreview();
    const pages = { start: startPage, library: libraryPage, description: descriptionPage, lifecycle: lifecyclePage, profile_setup: profileSetupPage, robot: diagnosticsPage, model: modelPage, geometry: geometryPage, calibration: calibrationPage, workbench: workbenchPage, run: runPage, diagnostics: diagnosticsPage, settings: settingsPage };
    const factory = pages[state.page] || startPage;
    const pageChanged = renderedPage !== state.page;
    const page = $('page');
    page.innerHTML = factory();
    const studio = !!state.config.profile && state.page !== 'settings';
    if (studio && !['start','library','description','model','lifecycle','profile_setup'].includes(state.page) && !page.querySelector('#robot-operation')) {
        const head = page.querySelector(':scope > .page-head');
        if (head) head.insertAdjacentHTML('afterend', calibrationQuickControlsHtml());
        else page.insertAdjacentHTML('afterbegin', calibrationQuickControlsHtml());
    }
    page.classList.toggle('studio-page', studio);
    const head = page.querySelector(':scope > .page-head');
    if (studio && head) {
        const body = document.createElement('div'); body.className = 'studio-page-body';
        for (const node of [...page.childNodes]) if (node !== head) body.append(node);
        page.replaceChildren(head, body);
    }
    const main = page.closest('.content');
    main.classList.toggle('studio-content', studio);
    document.querySelector('.shell').dataset.page = state.page;
    pageEnter(pageChanged);
    renderedPage = state.page;
    workspaceRenderHost = ['workbench', 'calibration'].includes(state.page) ? $('workbench-viewport') : null;
    workspaceRenderModel = state.modelData;
    workspaceRenderSubview = state.page + ':' + (state.page === 'calibration' ? state.calibrationEntry : state.workbenchTab);
    const runtime = $('runtime-panel'); if (runtime) runtime.classList.toggle('is-hidden', !['run', 'diagnostics'].includes(state.page) && !active());
    document.querySelectorAll('[data-page]').forEach(b => {
        const selected = b.dataset.page === state.page;
        b.classList.toggle('active', selected);
        if (selected) b.setAttribute('aria-current', 'page'); else b.removeAttribute('aria-current');
    });
    $('back-home')?.classList.toggle('active', ['start', 'library', 'description'].includes(state.page));
    if (['start', 'library', 'description'].includes(state.page)) $('back-home')?.setAttribute('aria-current', 'page');
    else $('back-home')?.removeAttribute('aria-current');
    statusUI(); bindPage();
    window.serialArmStudio?.enhanceSelects();
    if (state.page === 'profile_setup') restoreProfileFields();
    if (state.setupFocus?.page === state.page) focusSetupTarget();
    if ($('priority-error-ack')) $('priority-error-ack').onclick = acknowledgePriorityError;
    if ($('priority-error-overlay')) $('priority-error-overlay').addEventListener('cancel', event => event.preventDefault());
    if (state.page === 'run') setTimeout(() => { try { fit?.fit(); } catch { } }, 50);
    if (state.page === 'model') setTimeout(() => operation(mountModelView), 0);
    if (['workbench', 'geometry', 'calibration'].includes(state.page)) setTimeout(() => operation(mountWorkbenchView), 0);
    if (state.page === 'description' && state.descriptionAnalysis) setTimeout(() => operation(mountDescriptionView), 0);
}
function capture() { for (const key of Object.keys(state.config)) { const element = $(key); if (element) state.config[key] = element.value; } if ($('profile-select')) state.config.profile = $('profile-select').value; }
async function artifactExport(button, fn) {
    if (button?.dataset.exportBusy) return null;
    const label=button?.textContent;
    if(button) {button.dataset.exportBusy='true';button.disabled=true;button.textContent=busyText('正在导出…','Exporting…');}
    try {return await operation(fn);}
    finally {if(button) {delete button.dataset.exportBusy;button.disabled=false;button.textContent=label;}}
}
async function operation(fn) { try { return await fn(); } catch (error) { toast(error, true); return null; } }
function busyText(zh, en) { return state.language === 'en' ? en : zh; }
function setOperationOverlay(message = '') { const overlay = $('operation-overlay'); if (!overlay) return; const show = !!message && !priorityErrorOpen; overlay.hidden = !show; overlay.setAttribute('aria-hidden', show ? 'false' : 'true'); const text = $('operation-message'); if (text && show) text.textContent = message; refreshParkingActions(); }
async function blockingOperation(message, fn, minimumMs = 320) { if (state.blockingBusy) return null; state.blockingBusy = true; setOperationOverlay(message || busyText('正在处理...', 'Working...')); const started = performance.now(); try { return await fn(); } catch (error) { if (String(error?.serialarm?.technical || error?.message || error).includes('park interrupted by operator')) toast(busyText('停放已中断，机械臂状态以实时反馈为准', 'Parking interrupted; check live robot state')); else toast(error, true); return null; } finally { const remaining = minimumMs - (performance.now() - started); if (remaining > 0) await new Promise(resolve => setTimeout(resolve, remaining)); state.blockingBusy = false; setOperationOverlay(''); } }
function refreshParkingActions() {
    const actions = $('operation-safety-actions');
    if (!actions) return;
    actions.hidden = state.parkingBusy !== true;
    for (const id of ['operation-hold', 'operation-disable']) {
        const button = $(id);
        if (button) button.disabled = state.parkingInterruptPending === true || !workbenchConnected() || workbenchRecoveryRequired();
    }
    const force = $('operation-force-disconnect');
    if (force) force.disabled = !workbenchConnected();
}
async function interruptibleParking(fn) {
    state.parkingBusy = true;
    refreshParkingActions();
    try { return await fn(); }
    finally { state.parkingBusy = false; refreshParkingActions(); }
}
async function interruptParking(action) {
    if (!state.parkingBusy || state.parkingInterruptPending || !['hold', 'deactivate'].includes(action)) return;
    state.parkingInterruptPending = true;
    refreshParkingActions();
    try { await operation(() => workbenchRequest(action)); }
    finally { state.parkingInterruptPending = false; refreshParkingActions(); }
}
async function requestWorkbenchStop(params = {}) {
    return params.force ? api.request('workbench_stop', params) : interruptibleParking(() => api.request('workbench_stop', params));
}
function actionBusyMessage(action) { const labels = { activate: ['正在使能...', 'Activating...'], park: ['正在停放并失能...', 'Parking and disabling...'], deactivate: ['正在立即失能...', 'Disabling...'], hold: ['正在保持当前位置...', 'Holding current position...'], rigid_hold: ['正在刚性保持当前位置...', 'Capturing rigid hold...'], clear_fault: ['正在清除故障...', 'Clearing fault...'], fault_compliant: ['正在进入柔性恢复...', 'Entering compliant recovery...'], fault_rigid: ['正在返回刚性保持...', 'Returning to rigid hold...'] }; const value = labels[action] || ['正在处理...', 'Working...']; return busyText(value[0], value[1]); }
async function refreshProfiles() {
    capture(); if (state.source === 'external' && !state.config.profile_file) throw new Error(t('fileHint')); const data = await api.request('profiles', state.config); state.profiles = data.profiles;
    if (!state.profiles.some(p => p.name === state.config.profile)) state.config.profile = state.profiles[0]?.name || '';
    state.info = null; render(); await save(); await inspect(); render();
}
function adoptInspectionInfo(info) {
    const identity=info.model_identity || null;
    if(identity && state.modelIdentity && JSON.stringify(identity)!==JSON.stringify(state.modelIdentity)) {
        pageModelMount=null;workspaceMount=null;disposeModelPreview();graphicsState.records.clear();graphicsState.current=null;
        state.modelData=null;state.modelPositions=[];state.modelSelection='';state.modelVisibility={links:{},linkFrames:{},joints:{}};
        state.workbenchTargets=[];state.workbenchTargetEpoch=null;state.pendingTarget=null;state.pendingMotionId=null;
        state.modelCalibrationPreviewAlignment=null;state.modelCalibrationComparisonKey='';state.ghostRequestKey='';
        state.modelReloadPending=workbenchConnected();
    }
    if(identity) state.modelIdentity=structuredClone(identity);
    state.info=info;
}
async function inspect() {
    capture(); if (state.source === 'external' && !state.config.profile_file) throw new Error(t('fileHint')); const serial = ++state.inspecting; const snapshot = { ...state.config };
    const status = await api.request('status'); const info = await api.request('inspect', snapshot);
    if (serial !== state.inspecting) return; state.status = status; adoptInspectionInfo(info); await save(); statusUI();
}
async function navigate(page) {
    if (page === state.page) return;
    if (state.page === 'profile_setup' && state.profileDirty && !await safetyConfirmation(state.language === 'en' ? 'Discard changes' : '放弃未保存的参数', state.language === 'en' ? 'Leaving this page discards unsaved changes' : '离开当前页面将放弃未保存的修改', '', [], 'general')) return;
    capture(); await operation(save); await pageExit();
    if (state.page === 'profile_setup') state.createdProfileWizard = false;
    const previousPage = state.page;
    state.page = page;
    try {
        if (page === 'lifecycle' && state.config.profile) state.readiness = await api.request('readiness', { config: state.config });
        if (page === 'profile_setup' && state.config.profile) {
            state.profileInteractionGoal = null; state.profileEditor = await api.request('profile_editor_load', { config: state.config });
            state.profileDirty = false; clearProfileFormState();
        }
        if (['run', 'model', 'workbench', 'geometry', 'calibration', 'diagnostics'].includes(page)) await operation(inspect);
        if (['workbench', 'geometry', 'calibration', 'diagnostics'].includes(page) && state.config.profile) state.readiness = await api.request('readiness', { config: state.config });
        if (page === 'geometry' && state.config.profile) {
            state.geometryState = await api.request('geometry_fixed_state', { config: state.config });
            if (state.geometryState?.active) state.geometryPurpose = state.geometryState.purpose;
            state.geometryFrame = state.geometryState?.parent_frame || state.geometryState?.suggested_frame || '';
            const listing = await api.request('trajectory_list', { config: state.config });
            state.geometryTrajectories = listing.items || [];
            await refreshGeometryRepeatStatus();
        }
        if (page === 'workbench') {
            state.capabilityProfile = null; state.capabilityProfileError = '';
            try { state.capabilityProfile = await api.request('profile_editor_load', { config: state.config }); }
            catch (error) { state.capabilityProfileError = error.message || String(error); }
        }
        if (['model', 'workbench', 'geometry', 'calibration'].includes(page) && !state.modelData && state.info?.available?.model) await operation(() => loadModelData());
        if (previousPage === 'profile_setup') {
            state.profileDirty = false; state.profileEditor = null; state.profileInteractionGoal = null; clearProfileFormState();
        }
        render();
    } catch (error) {
        // Restore the previous page even if an IPC request failed during entry.
        // Otherwise pageExit() leaves the content faded out and navigation appears blank.
        state.page = previousPage;
        const node = $('page');
        if (node) node.classList.remove('page-transition-out', 'page-transition-in');
        render();
        toast(t('error') + ': ' + (error?.message || String(error)), true);
    }
}
function bindPage() {
    bindGraphicsActions();
    wireCalibrationQuickControls();
    observeRobotOperationHeader();
    document.querySelectorAll('[data-start-intent]').forEach(button => button.onclick = () => operation(() => startIntent(button.dataset.startIntent)));
    bindProfileManager($('page'));
    if ($('library-home')) $('library-home').onclick = () => operation(resetToStart);
    if ($('library-import-dir')) $('library-import-dir').onclick = () => operation(async () => { const source = await api.selectProfilePackage(); if (source) await importProfileSource(source); });
    if ($('library-import-file')) $('library-import-file').onclick = () => operation(async () => { const source = await api.selectProfile(); if (source) await importProfileSource(source); });
    if ($('description-home')) $('description-home').onclick = resetToStart;
    if ($('description-browse')) $('description-browse').onclick = () => operation(async () => { const file = await api.selectDescription(); if (file) { state.descriptionDraft.source = file; state.descriptionDraft.model_relative = ''; const node = $('description-source'); if (node) node.value = file; } });
    if ($('description-browse-dir')) $('description-browse-dir').onclick = () => operation(async () => { const dir = await api.selectProfilePackage(); if (dir) { state.descriptionDraft.source = dir; state.descriptionDraft.model_relative = ''; const node = $('description-source'); if (node) node.value = dir; } });
    if ($('description-model')) $('description-model').onchange = event => {
        if (!event.target.value) return;
        state.descriptionDraft.model_relative = event.target.value;
        blockingOperation(busyText('正在分析模型...', 'Analyzing model...'), analyzeDescription);
    };
    if ($('description-analyze')) $('description-analyze').onclick = () => blockingOperation(busyText('正在分析 Description...', 'Analyzing Description...'), analyzeDescription);
    if ($('description-reset')) $('description-reset').onclick = () => { state.descriptionAnalysis = null; state.descriptionStep = 1; state.descriptionDraft = { source: '', profile: '', destination: '', base_frame: '', tool_frame: '', joint_names: [] }; render(); };
    if ($('draft-base')) $('draft-base').onchange = () => { collectDescriptionDraft(); render(); };
    if ($('draft-tool')) $('draft-tool').onchange = () => { collectDescriptionDraft(); render(); };
    if ($('description-next-chain')) $('description-next-chain').onclick = () => { state.descriptionStep = 2; render(); };
    if ($('description-next-profile')) $('description-next-profile').onclick = () => {
        collectDescriptionDraft();
        if (!state.descriptionDraft.joint_names.length) { toast(state.language === 'en' ? 'Select a valid controlled chain' : '请先选择有效控制链', true); return; }
        state.descriptionStep = 3; render();
    };
    if ($('description-back')) $('description-back').onclick = () => { collectDescriptionDraft(); state.descriptionStep = Math.max(1, state.descriptionStep - 1); render(); };
    if ($('draft-destination-browse')) $('draft-destination-browse').onclick = () => operation(async () => { const dir = await api.selectDestination(); if (dir) { state.descriptionDraft.destination = dir; $('draft-destination').value = dir; } });
    if ($('draft-create')) $('draft-create').onclick = () => blockingOperation(busyText('正在生成 Profile 包...', 'Creating Profile package...'), createDescriptionProfile);
    document.querySelectorAll('[data-setup-target]').forEach(button=>button.onclick=()=>operation(()=>openSetupTarget(JSON.parse(button.dataset.setupTarget))));
    document.querySelectorAll('[data-review-stage]').forEach(button => button.onclick = () => showReadinessReview(button.dataset.reviewStage));
    bindDamiaoJointCheck();
    bindReadinessSections();
    if ($('approve-gain-mode')) $('approve-gain-mode').onclick = showGainAdmission;
    if ($('ready-control')) $('ready-control').onclick = () => navigate('workbench');
    if ($('profile-return-ready')) $('profile-return-ready').onclick = () => operation(returnToPreparation);
    document.querySelectorAll('[data-profile-complete]').forEach(button=>button.onclick=()=>completeProfileFields(button.dataset.profileComplete));
    document.querySelectorAll('[data-profile-group]').forEach(button => button.onclick = () => {
        state.profileGroup = button.dataset.profileGroup; render();
    });
    const sourcePanel = document.querySelector('.model-inspector') || document.querySelector('.parameter-card');
    if (sourcePanel) sourcePanel.onclick = event => {if (event.target.closest('[data-urdf-source]')) operation(()=>api.revealFile(state.page === 'model' ? state.modelData.urdf : state.profileEditor.urdf_path));};
    document.querySelectorAll('[data-choice-selector]').forEach(select => select.addEventListener('change', () => {
        const wrapper = select.closest('.hardware-choice');
        const input = wrapper?.querySelector('[data-profile-field]');
        if (!input) return;
        const custom = select.value === '__custom__';
        input.hidden = !custom;
        const hint = wrapper.querySelector('.hardware-custom-hint');
        if (hint) hint.hidden = !custom;
        if (custom) { input.focus(); return; }
        input.value = select.value;
        try { readProfileInput(input); refreshProfileGuidance(); }
        catch (error) { state.profileError = error.message; }
    }));
    document.querySelectorAll('[data-profile-field]').forEach(field => {
        field.addEventListener('change', () => {
            try { readProfileInput(field); updateProfileSwitch(field); refreshProfileGuidance(); refreshRuntimeSavePreview(); }
            catch (error) { state.profileError = error.message; }
        });
        field.addEventListener('input', () => {
            try { readProfileInput(field); updateProfileSwitch(field); refreshProfileGuidance(); refreshRuntimeSavePreview(); }
            catch { /* Keep the field error visible while editing */ }
        });
    });
    if ($('profile-add-hold-compensation')) $('profile-add-hold-compensation').onclick = () => {
        const names = state.profileEditor.joint_names, named = value => Object.fromEntries(names.map(name => [name, value]));
        state.profileEditor.core.control.hold_compensation = {enabled:false, integral_gain:named(.15), max_torque:named(.15), max_rate:named(.05), contact_threshold:named(.2), velocity_threshold:.02, error_threshold:.03};
        state.profileDirty = true; render();
    };
    if ($('profile-add-identification')) $('profile-add-identification').onclick = () => { state.profileEditor.core.control.controller.servo_tracking = structuredClone(state.profileEditor.core.control.controller.rigid_tracking); state.profileDirty = true; render(); };
    if ($('profile-add-filter')) $('profile-add-filter').onclick = () => { state.profileEditor.core.control.runtime.joint_acc_filter_alpha = 0.2; state.profileDirty = true; render(); };
    if ($('profile-add-park-ratio')) $('profile-add-park-ratio').onclick = () => { state.profileEditor.core.shutdown.compliant_phase_ratio = 0.75; state.profileDirty = true; render(); };
    if ($('profile-add-continuity')) $('profile-add-continuity').onclick = () => { state.profileEditor.core.safety_policy.require_continuous_cmd = false; state.profileDirty = true; render(); };
    if ($('profile-add-correction')) $('profile-add-correction').onclick = () => { state.profileEditor.core.model.gravity_correction_path = ''; state.profileDirty = true; render(); };
    if ($('profile-add-effort')) $('profile-add-effort').onclick = () => {
        const names = state.profileEditor.joint_names;
        const limit = state.profileEditor.urdf_effort_limits || {};
        state.profileEditor.core.safety_policy.max_effort_override = Object.fromEntries(names.map(j => [j, Math.max(0, Number(limit[j] ?? 0))]));
        state.profileDirty = true; render();
    };
    if ($('profile-init-damiao')) $('profile-init-damiao').onclick = () => {
        state.profileEditor.hardware = profileHardwareDefaults(state.profileEditor.joint_names);
        state.profileEditor.profile_entry.hardware.plugin = 'serial_arm_hardware_damiao';
        state.profileDirty = true; render();
    };
    bindHardwareTools();
    if($('control-settings'))$('control-settings').ontoggle=()=>{state.controlSettingsOpen=$('control-settings').open;};
    if ($('profile-reload')) $('profile-reload').onclick = () => operation(async () => {
        if (state.profileDirty && !await safetyConfirmation(state.language === 'en' ? 'Reload Profile' : '重新加载 Profile', state.language === 'en' ? 'Unsaved changes will be discarded before reloading' : '重新读取磁盘配置前将放弃未保存的参数', '', [], 'general')) return;
        state.profileInteractionGoal = null; state.profileEditor = await api.request('profile_editor_load', { config: state.config });
        state.profileDirty = false; clearProfileFormState(); await inspect();state.readiness=await api.request('readiness',{config:state.config});render();
    });
    if ($('profile-save')) $('profile-save').onclick = async () => {
        let payload;
        try { payload = collectProfileEditor(); } catch (error) { toast(error.message); return; }
        const editor = state.profileEditor;
        const newTrust = editor?.core?.safety_policy?.fault_recovery?.gravity_model_validated === true;
        const oldTrust = editor?.original_gravity_model_validated === true;
        let gravityRecoveryConfirmed = false;
        if (newTrust && !oldTrust) {
            gravityRecoveryConfirmed = await safetyConfirmation(
                state.language === 'en' ? 'Confirm verified gravity model' : '确认重力模型真实有效',
                state.language === 'en' ? 'This enables lower-gain fault recovery, including supported position-limit faults. An inaccurate gravity model can cause the arm to drop. A checkbox does not validate the model' : '开启后将允许符合条件的低增益故障柔性恢复，包括位置越限故障，错误的重力模型可能造成机械臂下坠，勾选确认不代表系统已自动验证',
                '',
                state.language === 'en' ? ['I verified the gravity direction and compensation with this robot and its current payload', 'Reliable physical support and emergency stop are available'] : ['已针对这台机械臂和当前末端负载真机确认重力补偿方向与效果', '已准备可靠机械支撑并确认急停与失能途径']);
            if (!gravityRecoveryConfirmed) return;
        }
        return blockingOperation(busyText('正在验证并保存参数...', 'Validating and saving configuration...'), async () => {
            try {
                payload.gravity_recovery_confirmed = gravityRecoveryConfirmed;
                state.profileEditor = await api.request('profile_editor_save', { config: state.config, payload });
                state.profileDirty = false; clearProfileFormState(); state.profileSavedForSession = workbenchConnected(); state.profileSaveNotice = true;
                state.info = null; await inspect();
                state.readiness = await api.request('readiness', { config: state.config });
                await refreshLibrary(); toast(t('saved')); render();
            } catch (error) {
                state.profileError = error.message || String(error);
                const target = error.serialarm?.action;
                if (target?.page === 'profile_setup' && target.field) {
                    state.profileGroup=target.group || profileGroupForField(target.field);
                    state.setupFocus=target;state.profileFieldErrors ||= {};state.profileFieldErrors[target.field]=state.profileError;
                }
                const found = !target?.field && profileGroups.find(([, , path]) => state.profileError.startsWith('core.' + path) || state.profileError.startsWith(path));
                if (found) state.profileGroup = found[0];
                else if (!target?.field && state.profileError.includes('capability.admittance')) state.profileGroup = 'motion';
                else if (!target?.field && (state.profileError.startsWith('hardware.') || state.profileError.startsWith('profile.'))) state.profileGroup = 'hardware';
                render(); throw error;
            }
        });
    };
        if ($('preview-config')) $('preview-config').onclick = () => blockingOperation(busyText('正在生成配置差异', 'Preparing configuration diff'), async () => { if (state.profileDirty) return; state.savePreview = await api.request('workbench_config_preview', { config: state.config }); refreshRuntimeSavePreview(); });
        if ($('save-config')) $('save-config').onclick = () => blockingOperation(busyText('正在保存配置', 'Saving defaults'), async () => {
            if (!state.savePreview?.changed || state.profileDirty) return;
            const result = await api.request('workbench_config_save', { config: state.config, expected_sha: state.savePreview.sha256 });
            state.savePreview = null; state.profileEditor = await api.request('profile_editor_load', {config:state.config}); state.capabilityProfile = structuredClone(state.profileEditor); Object.assign(state.profileEditor,{gravity_validation_reset:result.gravity_validation_reset,gravity_validation_changes:result.gravity_validation_changes}); state.profileDirty = false; clearProfileFormState(); state.readiness = await api.request('readiness', {config:state.config}); render(); toast(`${busyText('默认配置已保存', 'Saved defaults')} · ${result.path}`);
        });
    bindInteractionGoals();
    if ($('open-persistent-config')) $('open-persistent-config').onclick = () => operation(()=>openSetupTarget({page:'profile_setup',group:'motion',field:'core.capability.admittance.enabled'}));
    if ($('profile-open-auto-calibration')) $('profile-open-auto-calibration').onclick = () => operation(()=>openSetupTarget({page:'calibration',tool:'gain'}));
    if ($('open-capability-calibration')) $('open-capability-calibration').onclick = () => operation(()=>openSetupTarget({page:'calibration',tool:'gain'}));
    if ($('settings-back')) $('settings-back').onclick = () => operation(() => navigate(state.previousPage || 'start'));
    for (const key of Object.keys(state.config)) {
        const element = $(key); if (element) element.onchange = () => { capture(); state.info = null; operation(save); };
    }
    if ($('serial_port')) $('serial_port').setAttribute('list', 'ports');
    if ($('source-builtin')) {
        $('source-builtin').onclick = () => operation(async () => { capture(); state.source = 'builtin'; state.config.profile_file = ''; await refreshProfiles(); });
        $('source-external').onclick = () => { capture(); state.source = 'external'; state.info = null; state.profiles = []; state.config.profile = ''; render(); };
        $('browse-profile')?.addEventListener('click', () => operation(async () => { capture(); const file = await api.selectProfile(); if (file) { state.config.profile_file = file; if ($('profile_file')) $('profile_file').value = file; await refreshProfiles(); } }));
        $('profile-select').onchange = () => operation(async () => { capture(); await inspect(); render(); });
        if ($('profile_file')) $('profile_file').onchange = () => operation(refreshProfiles);
        $('refresh-profiles').onclick = () => blockingOperation(busyText('正在刷新 Profile...', 'Refreshing profiles...'), refreshProfiles);
        $('inspect-profile').onclick = () => blockingOperation(busyText('正在检查配置...', 'Inspecting configuration...'), async () => { await inspect(); render(); });
        $('go-run').onclick = () => navigate('run');
        $('add-root').onclick = () => operation(async () => { capture(); const dir = await api.selectResources(); if (dir) { state.config.resource_paths = [state.config.resource_paths, dir].filter(Boolean).join(':'); await inspect(); render(); } });
    }
    if (state.page === 'model') {
        if ($('model-reload')) $('model-reload').onclick = () => blockingOperation(busyText('正在重新加载模型...', 'Reloading model...'), async () => { await loadModelData(); render(); });
        if ($('model-zero')) $('model-zero').onclick = () => { state.modelPositions = state.modelPositions.map(() => 0); document.querySelectorAll('[data-model-joint]').forEach((input, i) => { input.value = '0'; const number = document.querySelector(`[data-model-number="${i}"]`); if (number) number.value = '0.000'; const value = $('joint-value-' + i); if (value) value.textContent = jointDisplayValue(0,state.modelData?.native?.config?.joint_names?.[i]); }); queuePreview(); };
        document.querySelectorAll('[data-model-layer]').forEach(input => input.onchange = () => { state.modelLayers[input.dataset.modelLayer] = input.checked; modelView?.setLayers(state.modelLayers); });
        document.querySelectorAll('[data-model-inertia-source]').forEach(input => input.onchange = () => { if (!input.checked) return; state.modelInertiaSource = input.value; modelView?.setInertiaSource(input.value); const legend = $('model-com-legend'); if (legend) legend.innerHTML = `<i class="${input.value === 'core' ? 'legend-effective' : 'legend-com'}"></i>${input.value === 'core' ? 'Core COM' : 'Link COM'}`; });
        document.querySelectorAll('[data-model-select]').forEach(button => button.onclick = () => { const next = button.dataset.modelSelect; state.modelSelection = state.modelSelection === next ? '' : next; document.querySelector('.model-inspector').innerHTML = modelInspectorHtml(); document.querySelectorAll('.tree-row').forEach(x => x.classList.toggle('active', x.querySelector('[data-model-select]')?.dataset.modelSelect === state.modelSelection)); modelView?.select(state.modelSelection); });
        document.querySelectorAll('[data-model-visible]').forEach(button => button.onclick = event => { event.stopPropagation(); const kind = button.dataset.modelVisible, name = button.dataset.modelName; const next = !modelObjectVisible(kind, name); state.modelVisibility[kind][name] = next; button.classList.toggle('on', next); if (button.classList.contains('tree-eye')) button.textContent = next ? '●' : '○'; modelView?.setObjectVisible(kind, name, next); });
        document.querySelectorAll('[data-model-frame]').forEach(button => button.onclick = event => { event.stopPropagation(); const name = button.dataset.modelFrame; const next = !modelObjectVisible('linkFrames', name); state.modelVisibility.linkFrames[name] = next; button.classList.toggle('on', next); button.setAttribute('aria-pressed', next ? 'true' : 'false'); modelView?.setLinkFrameVisible(name, next); });
        const setAllObjects = visible => { for (const link of state.modelData?.model?.links || []) { state.modelVisibility.links[link.name] = visible; state.modelVisibility.linkFrames[link.name] = visible; modelView?.setObjectVisible('links', link.name, visible); modelView?.setLinkFrameVisible(link.name, visible); } for (const joint of state.modelData?.model?.joints || []) { state.modelVisibility.joints[joint.name] = visible; modelView?.setObjectVisible('joints', joint.name, visible); } document.querySelectorAll('[data-model-visible]').forEach(button => { button.classList.toggle('on', visible); if (button.classList.contains('tree-eye')) button.textContent = visible ? '●' : '○'; }); document.querySelectorAll('[data-model-frame]').forEach(button => { button.classList.toggle('on', visible); button.setAttribute('aria-pressed', visible ? 'true' : 'false'); }); };
        if ($('model-show-all')) $('model-show-all').onclick = () => setAllObjects(true);
        if ($('model-hide-all')) $('model-hide-all').onclick = () => setAllObjects(false);
        const setAllLayers = visible => { for (const key of Object.keys(state.modelLayers)) state.modelLayers[key] = visible; document.querySelectorAll('[data-model-layer]').forEach(input => input.checked = visible); modelView?.setLayers(state.modelLayers); };
        if ($('model-layers-all')) $('model-layers-all').onclick = () => setAllLayers(true);
        if ($('model-layers-none')) $('model-layers-none').onclick = () => setAllLayers(false);
        document.querySelectorAll('[data-model-joint]').forEach(input => input.oninput = () => { const i = Number(input.dataset.modelJoint), name = state.modelData?.native?.config?.joint_names?.[i]; state.modelPositions[i] = jointInputToNative(name,input.value); const number = document.querySelector(`[data-model-number="${i}"]`); if (number) number.value = Number(input.value).toFixed(3); const value = $('joint-value-' + i); if (value) value.textContent = jointDisplayValue(state.modelPositions[i],name); queuePreview(); });
        document.querySelectorAll('[data-model-number]').forEach(input => input.onchange = () => { const i = Number(input.dataset.modelNumber), name = state.modelData?.native?.config?.joint_names?.[i]; const slider = document.querySelector(`[data-model-joint="${i}"]`); let value = Number(input.value); if (!Number.isFinite(value)) return; value = Math.max(Number(slider.min), Math.min(Number(slider.max), value)); state.modelPositions[i] = jointInputToNative(name,value); slider.value = String(value); input.value = value.toFixed(3); const label = $('joint-value-' + i); if (label) label.textContent = jointDisplayValue(state.modelPositions[i],name); queuePreview(); });
    }
    if (['workbench', 'geometry', 'calibration', 'diagnostics'].includes(state.page)) {
        document.querySelectorAll('[data-workbench-tab]').forEach(b => b.onclick = () => { state.workbenchTab = b.dataset.workbenchTab; render(); if (state.workbenchTab === 'replay') operation(async () => { await refreshTeachTrajectories(); render(); }); });
        wireTeachReplay();
        if (state.page === 'geometry') wireGeometry();
        document.querySelectorAll('[data-calibration-com-scale]').forEach(button => button.onclick = () => { state.modelCalibrationDisplayScale = Number(button.dataset.calibrationComScale); document.querySelectorAll('[data-calibration-com-scale]').forEach(other => { const active = Number(other.dataset.calibrationComScale) === state.modelCalibrationDisplayScale; other.classList.toggle('active', active); other.setAttribute('aria-pressed', String(active)); }); const hint = document.querySelector('.calibration-com-hint'); if (hint) hint.textContent = state.modelCalibrationDisplayScale === 1 ? (state.language === 'en' ? 'True 3D locations' : '真实三维位置') : (state.language === 'en' ? 'Visual displacement only' : '仅放大显示位移，不更改实际坐标'); updateModelCalibrationComparison(); });
        if ($('calibration-com-unchanged')) $('calibration-com-unchanged').onchange = event => { state.modelCalibrationShowUnchanged = event.target.checked; updateModelCalibrationComparison(); };
        if ($('start-workbench')) $('start-workbench').onclick = () => startWorkbench();
        if ($('stop-workbench')) $('stop-workbench').onclick = () => confirmWorkbenchDisconnect();
        document.querySelectorAll('[data-gain-purpose]').forEach(button => button.onclick = () => switchWorkbenchGainPurpose(button.dataset.gainPurpose));
        document.querySelectorAll('[data-feedforward-mode]').forEach(b => b.onclick = () => switchWorkbenchMode('feedforward', b.dataset.feedforwardMode));
        document.querySelectorAll('[data-reference-behavior]').forEach(button => button.onclick = () => blockingOperation(busyText('正在切换控制方式', 'Switching control method'), async () => {
            if (!referenceBehaviorMayChange()) throw new Error(busyText('请先停放并失能，且退出标定或回放任务', 'Park and disable, and finish calibration or replay first'));
            const epoch = state.workbenchEpoch;
            const snapshot = await workbenchRequest('set_reference_behavior', { behavior: button.dataset.referenceBehavior });
            if (epoch !== state.workbenchEpoch || !workbenchConnected()) return;
            applyWorkbenchSnapshot(snapshot);
            await loadAdmittance();
            refreshCapabilityDiagnostics();
        }));
        if ($('response-start')) $('response-start').onclick = () => {
            if (!responseMayRecord()) return;
            state.responseCapture = { recording: true, conditions:responseConditions(), gain_profile:state.telemetry?.gain_profile, effective_kp:structuredClone(state.telemetry?.effective_kp || []),effective_kd:structuredClone(state.telemetry?.effective_kd || []), session_id: state.session?.session_id, profile: state.config.profile,
                core: state.info?.resources?.core || '', urdf_fingerprint: state.modelData?.native?.config?.urdf_fingerprint || null,
                model_identity: structuredClone(state.info?.model_identity || null),
                behavior: referenceBehavior(), impedance_mode: state.telemetry?.impedance_mode,
                model_feedforward_mode: state.telemetry?.model_feedforward_mode,
                gravity_scale: structuredClone(state.telemetry?.gravity_scale || []),
                joint_names: [...(state.telemetry?.joint_names || [])], joint_units: (state.telemetry?.joint_names || []).map(name => state.modelData?.model?.joints?.find(joint => joint.name === name)?.type === 'prismatic' ? 'm' : 'rad'), parameters: structuredClone(state.admittance || {}),
                release_time_s: null, samples: [], source: 'passive_native_cycle_snapshots' };
            state.responseAnalysis = null; state.externalTcpDraft = {confirmed:false,x:'',y:'',z:'',tolerance:''};
            if ($('response-tcp-measured')) $('response-tcp-measured').checked = false;
            document.querySelectorAll('[data-response-tcp]').forEach(input => input.value = '');
            if ($('response-tcp-state')) $('response-tcp-state').textContent = busyText('针尖外部回位未验证', 'External tool-tip return is not verified');
            state.responseJoint = Math.min(state.responseJoint || 0, Math.max(0, state.responseCapture.joint_names.length - 1));
            recordResponseSnapshot(state.telemetry); refreshCapabilityDiagnostics();
        };
        if ($('response-release')) $('response-release').onclick = () => {
            const capture = state.responseCapture; if (!capture?.recording || !capture.samples.length || capture.release_time_s != null) return;
            capture.release_time_s = capture.samples.at(-1).time_s; refreshCapabilityDiagnostics();
        };
        if ($('response-stop')) $('response-stop').onclick = () => { if (state.responseCapture) state.responseCapture.recording = false; refreshCapabilityDiagnostics(); };
        if ($('response-joint')) $('response-joint').onchange = event => { state.responseJoint = Number(event.target.value); refreshCapabilityDiagnostics(); };
        document.querySelectorAll('[data-response-tolerance]').forEach(input => input.onchange = () => {
            const name = input.dataset.responseTolerance, capture = state.responseCapture;
            const index = (capture?.joint_names || state.telemetry?.joint_names || []).indexOf(name);
            const spec = jointDisplaySpec(name,capture?.joint_units?.[index]), value = Number(input.value) / spec.factor;
            if (input.value.trim() && Number.isFinite(value) && value > 0) {
                state.responseTolerances = {...(state.responseTolerances || {}),[name]:value};
                state.responseAnalysis = null; input.setCustomValidity('');
            } else input.setCustomValidity(busyText('稳定带必须大于零','The return band must be positive'));
            refreshCapabilityDiagnostics();
        });
        if ($('response-tcp-measured')) $('response-tcp-measured').onchange = event => { state.externalTcpDraft = {...(state.externalTcpDraft || {}), confirmed:event.target.checked}; state.responseAnalysis = null; if ($('response-analysis')) $('response-analysis').innerHTML = ''; if ($('response-tcp-state')) $('response-tcp-state').textContent = event.target.checked ? busyText('导出时检查并保存外部测量', 'External measurement will be checked at export') : busyText('针尖外部回位未验证', 'External tool-tip return is not verified'); };
        document.querySelectorAll('[data-response-tcp]').forEach(input => input.oninput = () => { state.externalTcpDraft = {...(state.externalTcpDraft || {}), [input.dataset.responseTcp]:input.value}; state.responseAnalysis = null; if ($('response-analysis')) $('response-analysis').innerHTML = ''; });
        if ($('response-export')) $('response-export').onclick = () => artifactExport($('response-export'), async () => {
            const capture = state.responseCapture; if (!capture?.samples.length || capture.recording) return;
            const toleranceInputs = [...document.querySelectorAll('[data-response-tolerance]')];
            if (toleranceInputs.some(input => !input.reportValidity())) return;
            const response_window = { ...capture, return_tolerance: responseToleranceValues(), external_tcp_measurement: readExternalTcpMeasurement(), summary: SerialArmResponse.summarize(capture.samples, capture.release_time_s, responseToleranceValues()) };
            const result = await api.request('workbench_export', { profile: capture.profile, core: capture.core, response_window });
            state.responseAnalysis = result.interaction_response || null;
            refreshCapabilityDiagnostics();
            toast(`${busyText('响应记录已导出', 'Response recording exported')} · ${result.path}`);
        });
        document.querySelectorAll('[data-target-joint]').forEach(input => input.oninput = () => editWorkbenchTarget(Number(input.dataset.targetJoint), input.value));
        document.querySelectorAll('[data-target-range]').forEach(input => input.oninput = () => editWorkbenchTarget(Number(input.dataset.targetRange), input.value));
        if ($('workbench-speed')) { const input=$('workbench-speed');input.oninput=()=>updateWorkbenchSpeed(input.value);input.onchange=()=>updateWorkbenchSpeed(input.value);input.setCustomValidity(state.speedInputError || ''); }
        if ($('target-sync')) $('target-sync').onclick = () => {
            if (!workbenchTargetFeedbackReady()) return;
            state.workbenchTargets = state.telemetry.joint.pos.slice(); state.workbenchTargetEpoch = state.workbenchEpoch;
            state.workbenchTargetBaseline = state.telemetry.sequence; state.workbenchTargetDirty = false;
            state.commandState = 'idle'; state.pendingTarget = null; state.pendingMotionId = null; refreshWorkbenchTargetFields(true); queueWorkbenchTargetGhost();
        };
        if ($('target-zero')) $('target-zero').onclick = () => {
            if (!workbenchTargetFeedbackReady()) return;
            state.workbenchTargets = state.telemetry.joint_names.map(() => 0); state.workbenchTargetDirty = true;
            state.commandState = 'edited'; state.pendingTarget = null; state.pendingMotionId = null; refreshWorkbenchTargetFields(true); queueWorkbenchTargetGhost();
        };
        if ($('target-move')) $('target-move').onclick = () => blockingOperation(busyText('正在移动到目标', 'Moving to the target'), async () => {
            if (graphicsAvailabilityProblem(true)) return;
            updateWorkbenchSpeed($('workbench-speed')?.value ?? state.speedScale);
            if (!workbenchTargetMayMove()) throw new Error(workbenchTargetProblem() || graphicsAvailabilityProblem(true) || busyText('请先使能并进入合法跟踪状态', 'Activate and enter a configured tracking state first'));
            const epoch = state.workbenchEpoch, target = state.workbenchTargets.slice();
            state.commandState = 'submitting'; refreshWorkbenchTargetFields();
            try {
                const motion=await workbenchRequest('move_to', { positions: target, speed_scale: state.speedScale });
                if (epoch !== state.workbenchEpoch) return;
                state.pendingMotionId=motion?.motion?.id ?? motion?.id ?? null;
                state.pendingTarget = target; state.commandState = 'accepted';
                if(motion?.state) updateMotionCompletion({motion});
            } catch (error) { if (epoch === state.workbenchEpoch) state.commandState = 'failed'; throw error; }
            finally { if (epoch === state.workbenchEpoch) refreshWorkbenchTargetFields(); }
        });
        setTimeout(() => refreshWorkbenchTargetFields(true), 0);
        bindTuningInputs();
        if ($('apply-tuning')) $('apply-tuning').onclick = () => blockingOperation(busyText('正在应用临时导纳参数...', 'Applying temporary admittance parameters...'), applyTuning);
        if ($('reset-tuning')) $('reset-tuning').onclick = () => { state.tuneDraft = structuredClone(state.admittance || {}); state.tuneDraftOrigin = 'runtime'; state.runtimeInteractionGoal = null; state.savePreview = null; refreshRuntimeSavePreview(); state.savePreview = null; refreshTuningInputs(); refreshRuntimeSavePreview(); const goal = $('runtime-interaction-goal'); if (goal) { goal.innerHTML = interactionGoalFieldsHtml('runtime'); bindInteractionGoals(); } };
        document.querySelectorAll('[data-calibration-begin]').forEach(b => b.onclick = () => blockingOperation(busyText('正在启动标定任务...', 'Starting calibration task...'), () => workbenchRequest('calibration_begin', { kind: b.dataset.calibrationBegin })));
        if ($('calibration-capture')) $('calibration-capture').onclick = () => blockingOperation(busyText('正在采集当前姿态...', 'Capturing current pose...'), () => workbenchRequest('calibration_capture'));
        if ($('calibration-finish')) $('calibration-finish').onclick = () => blockingOperation(busyText('正在拟合并验证标定结果...', 'Fitting and validating calibration...'), finishCalibrationStage);
        if ($('calibration-cancel')) $('calibration-cancel').onclick = () => blockingOperation(busyText('正在取消标定任务...', 'Cancelling calibration task...'), async () => { await workbenchRequest('calibration_cancel'); state.calibrationFlow = ''; render(); });
        if ($('friction-record-start')) $('friction-record-start').onclick = () => blockingOperation(busyText('正在开始示教记录...', 'Starting demonstration recording...'), async () => { if (state.calibrationFlow === 'friction-ready') state.calibrationFlow = 'friction'; await workbenchRequest('friction_record_begin'); render(); });
        if ($('friction-record-stop')) $('friction-record-stop').onclick = () => blockingOperation(busyText('正在结束示教并处理轨迹...', 'Stopping demonstration and processing trajectory...'), () => workbenchRequest('friction_record_stop'));
        if ($('friction-replay')) $('friction-replay').onclick = () => frictionConfirm();
        if ($('export-session')) $('export-session').onclick = () => artifactExport($('export-session'), async () => { const r = await api.request('workbench_export', { profile: state.config.profile, core: state.info?.resources?.core || '' }); toast(`Exported ${r.samples} samples · ${r.path}`); });
        const gravityAction = async (method, params = {}) => {
            const snapshot = await workbenchRequest(method, params);
            if (snapshot && typeof snapshot === 'object') applyWorkbenchSnapshot(snapshot);
            state.gravityVerifyDraft = null;
            render();
            return snapshot;
        };
        const gravityHandlers = {
            'gravity-verify-begin': async () => {
                if (!await safetyConfirmation('开始重力补偿验证', '开始前确认机械臂有可靠支撑，Safety 配置及重力方向已核对，急停可用')) return;
                return gravityAction('gravity_verify_begin');
            },
            'gravity-verify-activate': () => gravityAction('activate'),
            'gravity-verify-enter-drag': async () => {
                if (!await safetyConfirmation('拖拽定位', '机械臂会响应外力，请确认已安排机械支撑且人员远离运动空间')) return;
                return gravityAction('gravity_verify_drag');
            },
            'gravity-verify-apply': () => {
                const inputs = [...document.querySelectorAll('[data-verify-scale-index]')].sort((a, b) => Number(a.dataset.verifyScaleIndex) - Number(b.dataset.verifyScaleIndex));
                const values = inputs.map(x => x.value.trim() === '' ? NaN : Number(x.value));
                if (!values.length || values.some(x => !Number.isFinite(x) || x < 0 || x > 2)) throw new Error('重力比例必须在 0 至 2 之间且每个关节都填写有限数值');
                return gravityAction('gravity_verify_target', { values });
            },
            'gravity-verify-sample': () => gravityAction('gravity_verify_sample'),
            'gravity-verify-park': async () => {
                if (!await safetyConfirmation('结束重力验证', '请确认机械臂已稳定且获得可靠机械支撑，随后切回刚性保持并失能')) return;
                return gravityAction('gravity_verify_stop', { supported: true });
            },
            'gravity-verify-finish': () => gravityAction('gravity_verify_finish'),
            'gravity-verify-cancel': () => gravityAction('gravity_verify_cancel'),
        };
        Object.entries(gravityHandlers).forEach(([id, handler]) => { if ($(id)) $(id).onclick = () => blockingOperation('正在确认重力验证状态...', handler); });
        if ($('gravity-verify-evidence')) $('gravity-verify-evidence').onclick = () => showReadinessReview('gravity_verification');
        document.querySelectorAll('[data-verify-scale-slider]').forEach(slider => slider.oninput = () => {
            const number = document.querySelector(`[data-verify-scale-index="${slider.dataset.verifyScaleSlider}"]`);
            if (number) number.value = slider.value;
        });
        document.querySelectorAll('[data-verify-scale-index]').forEach(number => number.oninput = () => {
            const slider = document.querySelector(`[data-verify-scale-slider="${number.dataset.verifyScaleIndex}"]`);
            if (slider && Number.isFinite(Number(number.value))) slider.value = number.value;
        });
        if ($('diagnostic-frame')) $('diagnostic-frame').onchange = () => { state.selectedFrame = $('diagnostic-frame').value; render(); };
        document.querySelectorAll('[data-calibration-entry]').forEach(button => button.onclick = () => operation(async () => {
            if (modelCalibrationActive() && button.dataset.calibrationEntry !== state.calibrationEntry) return;
            const entry=button.dataset.calibrationEntry;
            state.calibrationEntry=entry;
            const selected=state.calibrationCandidateSelection[entry];
            if (selected?.directory && ['gravity','dynamics'].includes(entry)) await selectCalibrationToolRecord(entry,{directory:selected.directory},selected);
            else render();
        }));
        document.querySelectorAll('[data-tool-scan]').forEach(button => button.onclick = () => blockingOperation(busyText('正在扫描任务记录...', 'Scanning task records...'), async () => {
            const kind = button.dataset.toolScan;
            await scanCalibrationHistory(kind);
        }));
        document.querySelectorAll('[data-tool-page]').forEach(button=>button.onclick=()=>blockingOperation(busyText('正在查找记录','Finding records'),()=>scanCalibrationHistory(button.dataset.toolPage,Number(button.dataset.offset))));
        document.querySelectorAll('[data-candidate-run]').forEach(input=>input.onchange=()=>blockingOperation(busyText('正在读取所选轮次','Loading selected run'),async()=>{
            const kind=input.dataset.candidateRun,item=state.calibrationCandidateRounds[kind].find(row=>row.id===input.value);
            state.calibrationCandidateSelection[kind]=item;await selectCalibrationToolRecord(kind,state.calibrationToolSelected[kind] || {directory:state.modelCalibrationDirectory},item);
        }));
        document.querySelectorAll('[data-candidate-compare]').forEach(input=>input.onchange=()=>{state.calibrationCandidateCompare ||= {};state.calibrationCandidateCompare[input.dataset.candidateCompare]=input.value;render()});
        document.querySelectorAll('[data-tool-load]').forEach(button => button.onclick = () => blockingOperation(busyText('正在检查任务记录...', 'Loading task...'), async () => {
            const kind = button.dataset.toolLoad;
            const directory = await api.selectResources();
            if (!directory) return;
            const selected = await api.request('calibration_tool_load', { kind, directory });
            await selectCalibrationToolRecord(kind, selected);
        }));
        document.querySelectorAll('[data-tool-select]').forEach(button => button.onclick = () => blockingOperation(busyText('正在读取历史任务...', 'Loading task...'), async () => {
            const kind = button.dataset.toolSelect;
            await selectCalibrationToolRecord(kind, state.calibrationToolRecords[kind][Number(button.dataset.toolIndex)]);
        }));
        document.querySelectorAll('[data-tool-export]').forEach(button => button.onclick = () => artifactExport(button, async () => {
            const kind = button.dataset.toolExport;
            const directory = (kind === 'gravity' ? gravityCandidatePreviewState().directory : null)
                || state.calibrationToolSelected[kind]?.directory || state.modelCalibrationDirectory
                || state.telemetry?.model_calibration?.directory;
            if (!directory) throw new Error('请先选择含有有效标定成果的任务');
            const destination = await api.selectResources();
            if (!destination) return;
            const report = kind === 'dynamics' ? (state.advancedReport?.report || state.advancedReport?.candidate_urdf?.replace(/\/[^/]+$/, '/identification-report.json') || '') : '';
            const gainPurposes = kind === 'gain' ? selectedGainPurposes() : null;
            if (gainPurposes && !gainPurposes.length) throw new Error(busyText('请先勾选已完成测量的用途','Select measured gain uses first'));
            const result = await api.request('calibration_tool_export', { kind, directory, destination, report, candidate_source: kind === 'gravity' ? state.modelCalibrationCandidateSource : 'online', ...selectedCandidateParams(kind), ...(gainPurposes ? {gain_purposes:gainPurposes} : {}) });
            toast((state.language === 'en' ? 'Calibration result exported: ' : '标定成果已导出 ') + result.path + (result.review_required ? (state.language === 'en' ? ' · Review only, friction disabled when unverified' : ' · 仅供人工审核，未通过的摩擦补偿已禁用') : ''));
        }));
        if ($('calibration-finish-teaching')) $('calibration-finish-teaching').onclick = () => blockingOperation(busyText('正在保存示教任务...', 'Saving demonstration...'), async () => {
            const result = await workbenchRequest('model_calibration_finish_teaching', modelCalibrationTaskParams());
            if (state.telemetry) state.telemetry.model_calibration = result;
            state.modelCalibrationDirectory = result.directory || '';
            render();
        });
        if ($('calibration-tool-import')) $('calibration-tool-import').onclick = () => blockingOperation(busyText('正在导入轨迹...', 'Importing saved trajectory...'), async () => {
            const record = state.calibrationToolSelected[state.calibrationEntry];
            if (!record?.directory) throw new Error('请先选择示教轨迹任务');
            const result = await workbenchRequest('model_calibration_import_trajectory', { directory: record.directory, ...captureModelCalibrationOptions({ validate: false }) });
            if (state.telemetry) state.telemetry.model_calibration = result;
            state.modelCalibrationDirectory = result.directory;
            render();
            toast('轨迹已导入，但尚未执行任何自动运动；请先检查并对齐起点');
        });
        document.querySelectorAll('[data-candidate-undo]').forEach(b => b.onclick = () => temporaryCandidateAction('restore', b.dataset.candidateUndo));
        if ($('calibration-gain-load-live')) $('calibration-gain-load-live').onclick = () => blockingOperation(busyText('正在读取当前增益任务...', 'Loading current gain task...'), async () => {
            const directory = state.telemetry?.model_calibration?.directory;
            if (!directory) throw new Error('当前任务尚无有效目录');
            const record = await api.request('calibration_tool_load', { kind: 'gain', directory });
            state.calibrationToolSelected.gain = record; render();
        });
        document.querySelectorAll('[data-gain-requirement]').forEach(input => input.oninput = () => { state.gainRequirements = {...(state.gainRequirements || {}), [input.dataset.gainRequirement]:input.value.trim() === '' ? '' : Number(input.value)/Number(input.dataset.gainFactor || 1)}; });
        if ($('gain-target')) $('gain-target').onchange = () => { state.gainMode = $('gain-target').value; };
        document.querySelectorAll('[data-gain-candidate-purpose]').forEach(input => input.onchange = () => {
            const candidate = state.calibrationToolSelected.gain?.gain_candidate;
            const selected = [...document.querySelectorAll('[data-gain-candidate-purpose]:checked')].map(item=>item.dataset.gainCandidatePurpose);
            state.gainPurposeSelections = {...(state.gainPurposeSelections || {}),[gainSelectionKey(candidate)]:selected};
            refreshGainApplyControls();
        });
        if ($('gain-path-purpose')) $('gain-path-purpose').onchange = () => { state.gainTrajectoryPurpose = $('gain-path-purpose').value; refreshCalibrationOrDefer(); };
        if ($('gain-path-validate')) $('gain-path-validate').onclick = () => blockingOperation(busyText('正在验收记录路径','Checking the recorded path'),validateGainPathRecording);
        refreshGainApplyControls();
        if ($('calibration-gain-temp-apply')) $('calibration-gain-temp-apply').onclick = () => temporaryCandidateAction('apply', 'gain');
        if ($('calibration-gain-temp-restore')) $('calibration-gain-temp-restore').onclick = () => temporaryCandidateAction('restore', 'gain');
        if ($('calibration-gravity-temp-apply')) $('calibration-gravity-temp-apply').onclick = () => temporaryCandidateAction('apply', 'gravity');
        if ($('calibration-gravity-temp-restore')) $('calibration-gravity-temp-restore').onclick = () => temporaryCandidateAction('restore', 'gravity');
        if ($('advanced-temp-apply')) $('advanced-temp-apply').onclick = () => temporaryCandidateAction('apply', 'dynamics');
        if ($('advanced-temp-restore')) $('advanced-temp-restore').onclick = () => temporaryCandidateAction('restore', 'dynamics');
        if ($('advanced-temp-restore-unconditional')) $('advanced-temp-restore-unconditional').onclick = () => temporaryCandidateAction('restore', 'dynamics');
        if ($('advanced-target')) $('advanced-target').onchange = () => { state.advancedTarget = $('advanced-target').value; state.advancedReport = null; render(); };
        document.querySelectorAll('[data-advanced-locked]').forEach(input => input.onchange = () => { const name = input.dataset.advancedLocked; const names = new Set(state.advancedLocked); if (input.checked) names.add(name); else names.delete(name); state.advancedLocked = Array.from(names); state.advancedReport = null; const run = $('advanced-run'); if (run) run.disabled = state.advancedLocked.length >= calibrationSelectableLinks().length; });
        if ($('advanced-load-record')) $('advanced-load-record').onclick = () => blockingOperation(busyText('正在读取历史标定...', 'Loading calibration...'), async () => { const directory = await api.selectResources(); if (!directory) return; const loaded = await api.request('model_calibration_load', { directory }); state.modelCalibrationOffline = loaded; state.modelCalibrationDirectory = loaded.directory || directory; state.advancedAssessment = null; state.advancedReport = null; state.advancedLocked = advancedTrustedLinks(loaded); render(); await alignHistoricalCalibrationPreview(loaded); await assessLoadedCalibrationRecord(loaded); });
        if ($('idim-scope')) $('idim-scope').onchange = event => { state.idimScope = event.target.value; document.querySelectorAll('[data-idim-locked]').forEach(x => x.disabled = state.idimScope !== 'local'); idimDraftChanged(); };
        document.querySelectorAll('[data-idim-locked]').forEach(input => input.onchange = () => { const locks = new Set(state.idimLocked || []); if (input.checked) locks.add(input.dataset.idimLocked); else locks.delete(input.dataset.idimLocked); state.idimLocked = [...locks]; idimDraftChanged(); });
        if ($('idim-preview')) $('idim-preview').onchange = event => { state.idimPreview=event.target.checked; updateModelCalibrationComparison(); };
        if ($('idim-load')) $('idim-load').onclick = () => blockingOperation('正在读取采集任务...',async()=>{ const directory=await api.selectResources(); if (!directory) return; await selectCalibrationToolRecord('dynamics',{directory}); const record=state.modelCalibrationOffline; state.idimAudit=null; state.idimLocked=[]; render(); await alignHistoricalCalibrationPreview(record); });
        if ($('idim-estimator')) $('idim-estimator').onchange = event => { state.idimEstimator = event.target.value; idimDraftChanged(); };
        if ($('idim-cutoff')) $('idim-cutoff').onchange = event => { state.idimCutoff = Number(event.target.value); idimDraftChanged(); };
        if ($('idim-load-description')) $('idim-load-description').oninput = event => {state.idimLoadDescription=event.target.value;idimDraftChanged();};
        if ($('idim-fixed-joints')) $('idim-fixed-joints').oninput = event => { state.idimFixedJointText = event.target.value; idimDraftChanged(); };
        if ($('idim-validation')) $('idim-validation').oninput = event => { state.idimValidationDirectory = event.target.value; idimDraftChanged(); };
        if ($('idim-audit')) $('idim-audit').onclick = () => blockingOperation(busyText('正在检查完整辨识数据...', 'Auditing dynamics observations...'), async () => {
            state.idimAudit = await api.request('model_calibration_idim_audit', { directory: state.modelCalibrationDirectory }); if ($('idim-audit-result')) $('idim-audit-result').innerHTML=idimAuditHtml();
        });
        if ($('idim-run')) $('idim-run').onclick = () => blockingOperation(busyText('正在统一滤波并验证完整动力学模型...', 'Filtering, identifying and validating full dynamics...'), async () => {
            const locks = state.idimScope === 'local' ? [...(state.idimLocked || [])] : [];
            if (state.idimScope === 'local' && !locks.length) throw new Error('请选择至少一个可信 Link；全部自由请用全局模式');
            const fixed = {};
            for (const item of (state.idimFixedJointText || '').split(',').map(x => x.trim()).filter(Boolean)) {
                const parts = item.split('='); const value = Number(parts[1]);
                if (parts.length !== 2 || !parts[0].trim() || !parts[1].trim() || !Number.isFinite(value) || Object.hasOwn(fixed, parts[0].trim())) throw new Error('固定关节填写格式为 joint7=0，不能重复');
                fixed[parts[0].trim()] = value;
            }
            state.idimReport = await api.request('model_calibration_idim', { directory: state.modelCalibrationDirectory,
                mode: state.idimScope, locked_links: locks, estimator: state.idimEstimator,
                cutoff_hz: state.idimCutoff, fixed_joint_positions: fixed, load_description:state.idimLoadDescription.trim() || null, validation_directory: state.idimValidationDirectory.trim() || null }); await refreshCalibrationCandidates('dynamics',state.modelCalibrationDirectory,state.idimReport.report_path || state.idimReport.run_id); await selectCalibrationToolRecord('dynamics',{directory:state.modelCalibrationDirectory},state.calibrationCandidateSelection.dynamics); refreshIdimResults();
        });
        bindIdimExports();
        if ($('advanced-assess-record')) $('advanced-assess-record').onclick = () => blockingOperation(busyText('正在检查采样信息量...', 'Assessing data...'), async () => { state.advancedAssessment = await api.request('model_calibration_assess', { directory: state.modelCalibrationDirectory || currentModelCalibration()?.directory }); render(); });
        if ($('advanced-run')) $('advanced-run').onclick = () => blockingOperation(busyText('正在离线辨识并检查候选模型...', 'Identifying and validating offline candidate...'), async () => {
            if (!state.modelCalibrationOffline?.metadata) throw new Error('请先加载带有效来源信息的历史任务'); const dir = state.modelCalibrationDirectory || state.modelCalibrationOffline.directory; const available = calibrationSelectableLinks().map(l => l.name);
            if (!available.length) throw new Error('历史任务缺少可验证的可辨识 Link 清单');
            const invalid = state.advancedLocked.filter(n => !available.includes(n));
            if (invalid.length) throw new Error('可信 Link 与当前任务模型不一致，请重新选择');
            if (state.advancedLocked.length === available.length) throw new Error('所有可辨识 Link 均已锁定，请至少保留一个');
            const locked = available.filter(n => state.advancedLocked.includes(n));
            const result = await api.request('model_calibration_advanced', { directory: dir, mode: state.advancedTarget, locked_links: locked }); await refreshCalibrationCandidates('dynamics',dir,result.report || result.report_path || result.candidate_run_id || result.run_id); await selectCalibrationToolRecord('dynamics',{directory:dir},state.calibrationCandidateSelection.dynamics); render();
        });
        if ($('calibration-all-in-one')) $('calibration-all-in-one').onclick = () => blockingOperation(busyText('正在启动完整标定流程...', 'Starting full calibration workflow...'), async () => { state.calibrationFlow = 'static'; await workbenchRequest('calibration_begin', { kind: 'static' }); render(); });
        if ($('calibration-preview-save')) $('calibration-preview-save').onclick = () => blockingOperation(busyText('正在整理标定结果差异...', 'Preparing calibration result diff...'), async () => { state.admittance = await workbenchRequest('get_admittance'); state.savePreview = await api.request('workbench_config_preview', { config: state.config }); render(); });
        if ($('calibration-save-config')) $('calibration-save-config').onclick = () => blockingOperation(busyText('正在保存标定参数...', 'Saving calibration parameters...'), async () => { const result = await api.request('workbench_config_save', { config: state.config, expected_sha: state.savePreview.sha256 }); state.savePreview = null; toast(`Saved ${result.path}`); render(); });
        for (const id of ['model-cal-pose-budget', 'model-cal-validation', 'model-cal-regularization', 'model-cal-svd', 'model-cal-information']) {
            const input = $(id);
            if (input) input.onchange = () => operation(() => captureModelCalibrationOptions({ validate: false }));
        }
        if ($('model-cal-mode-global')) $('model-cal-mode-global').onclick = () => operation(() => {
            captureModelCalibrationOptions({ validate: false });
            state.modelCalibrationOptions.mode = 'global';
            state.modelCalibrationOptions.locked_links = [];
            state.modelCalibrationSavePreview = null;
            render();
        });
        if ($('model-cal-mode-local')) $('model-cal-mode-local').onclick = () => operation(async () => {
            captureModelCalibrationOptions({ validate: false });
            const choices = await ensureCalibrationSelectableLinks();
            if (state.modelCalibrationOptions.mode === 'local') {
                editCalibrationLockedLinks();
                return;
            }
            state.modelCalibrationOptions.mode = 'local';
            // First entry locks everything until the user explicitly confirms
            // which identifiable Link is allowed to change
            state.modelCalibrationOptions.locked_links = choices.map(x => x.name);
            state.modelCalibrationSavePreview = null;
            render();
            editCalibrationLockedLinks({
                onCancel: () => {
                    state.modelCalibrationOptions.mode = 'global';
                    state.modelCalibrationOptions.locked_links = [];
                    render();
                }
            });
        });
        if ($('model-cal-lock-select')) $('model-cal-lock-select').onclick = () => operation(async () => {
            await ensureCalibrationSelectableLinks();
            editCalibrationLockedLinks();
        });
        if ($('model-calibration-teach-start')) $('model-calibration-teach-start').onclick = () => blockingOperation(busyText('正在启动拖动示教...', 'Starting demonstration...'), async () => {
            const epoch=state.workbenchEpoch;
            captureModelCalibrationOptions();
            const result=await workbenchRequest('model_calibration_teach_begin',state.modelCalibrationOptions);
            if(epoch!==state.workbenchEpoch || !result)return;
            state.modelCalibrationCandidateSource='online';
            state.modelCalibrationOffline=null;
            state.modelCalibrationSavePreview=null;
            state.modelCalibrationExport=null;
            state.modelCalibrationSaved=null;
            if(state.telemetry)state.telemetry.model_calibration=result;
            state.modelCalibrationDirectory=result.directory || '';
            render();updateModelCalibrationTaskOverlay();
        });
        if ($('model-calibration-teach-stop')) $('model-calibration-teach-stop').onclick = () => finishTeachingSafely();
        if ($('model-calibration-teach-cleanup')) $('model-calibration-teach-cleanup').onclick = () => finishTeachingSafely(true);
        if ($('model-calibration-replay-confirm')) $('model-calibration-replay-confirm').onclick = modelCalibrationReplayConfirm;
        if ($('model-calibration-pause')) $('model-calibration-pause').onclick = () => modelCalibrationQuick('model_calibration_pause');
        if ($('model-calibration-resume')) $('model-calibration-resume').onclick = resumeModelCalibrationWithConfirmation;
        if ($('model-calibration-cancel')) $('model-calibration-cancel').onclick = () => modelCalibrationQuick('model_calibration_cancel');
        if ($('model-calibration-apply')) $('model-calibration-apply').onclick = () => blockingOperation(busyText('正在应用候选重力校正...', 'Applying candidate gravity correction...'), async () => { const result = await workbenchRequest('model_calibration_apply', modelCalibrationTaskParams()); if (state.telemetry) state.telemetry.model_calibration = result; render(); });
        if ($('model-calibration-restore')) $('model-calibration-restore').onclick = () => blockingOperation(busyText('正在恢复运行时原配置...', 'Restoring original runtime configuration...'), async () => { const result = await workbenchRequest('model_calibration_restore', modelCalibrationTaskParams()); if (state.telemetry) state.telemetry.model_calibration = result; render(); });
        if ($('model-calibration-load-record')) $('model-calibration-load-record').onclick = () => blockingOperation(busyText('正在加载模型校正记录...', 'Loading model calibration record...'), async () => {
            const directory = await api.selectResources(); if (!directory) return; await selectCalibrationToolRecord('gravity',{directory}); const loaded=state.modelCalibrationOffline; state.advancedAssessment = null; state.advancedReport = null; state.advancedLocked = advancedTrustedLinks(loaded); state.modelCalibrationDirectory = loaded.directory || directory; if (loaded.metadata?.calibration_options) state.modelCalibrationOptions = { ...state.modelCalibrationOptions, ...loaded.metadata.calibration_options };
            if (loaded.result?.gravity_result?.identification_mode) state.modelCalibrationOptions = { ...state.modelCalibrationOptions, mode: loaded.result.gravity_result.identification_mode, locked_links: loaded.result.gravity_result.locked_links || [] }; state.modelCalibrationSavePreview = null; state.modelCalibrationExport = null; state.modelCalibrationSaved = null; render(); await alignHistoricalCalibrationPreview(loaded); await assessLoadedCalibrationRecord(loaded);
        });
        if ($('model-calibration-scan-records')) $('model-calibration-scan-records').onclick = () => blockingOperation(busyText('正在检查已有任务...', 'Scanning saved tasks...'), async () => { state.modelCalibrationRecords = await api.request('model_calibration_records', { config: state.config }); render(); });
        if ($('model-calibration-refresh-record')) $('model-calibration-refresh-record').onclick = () => blockingOperation(busyText('正在重新检查记录...', 'Refreshing record...'), async () => { const current = state.modelCalibrationOffline; if (!current?.directory) return; await selectCalibrationToolRecord('gravity',{directory:current.directory},state.calibrationCandidateSelection.gravity); render(); await alignHistoricalCalibrationPreview(state.modelCalibrationOffline); await assessLoadedCalibrationRecord(state.modelCalibrationOffline); });
        state.modelCalibrationRecords.forEach((saved, index) => {
            const button = $(`model-calibration-record-${index}`);
            if (button) button.onclick = () => blockingOperation(busyText('正在读取任务记录...', 'Reading task...'), async () => { await selectCalibrationToolRecord('gravity',{directory:saved.directory}); const fresh=state.modelCalibrationOffline; state.advancedAssessment = null; state.advancedReport = null; state.advancedLocked = advancedTrustedLinks(fresh); state.modelCalibrationDirectory = fresh.directory; render(); await alignHistoricalCalibrationPreview(fresh); await assessLoadedCalibrationRecord(fresh); });
        });
        if ($('model-calibration-back-live')) $('model-calibration-back-live').onclick = () => { state.modelCalibrationOffline = null; state.modelCalibrationCandidateSource = 'online'; state.calibrationCandidateSelection.gravity=null;state.calibrationCandidateRounds.gravity=[]; state.modelCalibrationPreviewAlignment = null; state.modelCalibrationDirectory = state.telemetry?.model_calibration?.directory || ''; render(); };
        if ($('model-calibration-alignment-begin')) $('model-calibration-alignment-begin').onclick = () => {
            const dialog = $('confirm-dialog');
            dialog.innerHTML = `<h3>${state.language === 'en' ? 'Supervised manual alignment' : '人工引导返回回放起点'}</h3><p>${state.language === 'en' ? 'The arm will enter manual drag and may sag under gravity. Support it, check the load and cables, and keep the motion area clear' : '即将进入人工拖拽，请支撑机械臂，检查负载、线缆并清空运动范围；重力可能使关节下垂'}</p><label class="confirmation"><input type="checkbox" id="model-cal-alignment-supported"><span>${state.language === 'en' ? 'Arm is supported and manual dragging is safe in this area' : '机械臂已被支撑，周围允许安全地人工拖拽'}</span></label><div class="actions"><button id="model-cal-alignment-cancel" class="button">${t('cancel')}</button><button id="model-cal-alignment-accept" class="button danger" disabled>${state.language === 'en' ? 'Enter manual drag' : '确认拖拽定位'}</button></div>`;
            dialog.showModal();
            $('model-cal-alignment-supported').onchange = () => { $('model-cal-alignment-accept').disabled = !$('model-cal-alignment-supported').checked; };
            $('model-cal-alignment-cancel').onclick = () => dialog.close();
            $('model-cal-alignment-accept').onclick = () => { dialog.close(); blockingOperation(busyText('正在进入引导拖拽...', 'Entering guided drag...'), async () => { const result = await workbenchRequest('model_calibration_alignment_begin', modelCalibrationTaskParams({ supported: true })); if (state.telemetry) state.telemetry.model_calibration = result; render(); }); };
        };
        if ($('model-calibration-alignment-finish')) $('model-calibration-alignment-finish').onclick = () => blockingOperation(busyText('正在保持当前姿态...', 'Holding current pose...'), async () => { const result = await workbenchRequest('model_calibration_alignment_finish', modelCalibrationTaskParams()); if (state.telemetry) state.telemetry.model_calibration = result; render(); });
        if ($('model-calibration-import-trajectory')) $('model-calibration-import-trajectory').onclick = () => blockingOperation(busyText('正在检查并恢复示教轨迹...', 'Importing demonstrated trajectory...'), async () => {
            // Always reread the chosen folder. A live fault may have saved the
            // trajectory after the GUI last rendered the record panel.
            const selected = state.modelCalibrationOffline;
            if (!selected?.directory) throw new Error('请先选择任务记录');
            const record = await api.request('model_calibration_load', { directory: selected.directory });
            state.modelCalibrationOffline = record;
    state.idimReport = record?.idim_candidate || null;
    state.idimAudit = null;
            if (!record.can_resume) { render(); throw new Error('当前记录尚无有效的 trajectory.csv 或 trajectory.checkpoint.csv，请刷新任务列表并确认目录'); }
            if (!workbenchConnected() || state.telemetry?.robot_state !== 'ACTIVE') throw new Error('恢复轨迹需要工作台已连接且机械臂 ACTIVE');
            const current = state.telemetry?.model_calibration?.phase || 'idle';
            if (!['idle', 'complete', 'cancelled', 'failed'].includes(current)) throw new Error('请先取消或结束当前标定任务');
            // No motor movement: this only stages a checked copy of the trajectory.
            const restored = await workbenchRequest('model_calibration_import_trajectory', { directory: record.directory, ...captureModelCalibrationOptions() });
            if (state.telemetry) state.telemetry.model_calibration = restored;
            state.modelCalibrationOffline = null;
            state.modelCalibrationCandidateSource = 'online';
            state.modelCalibrationDirectory = restored.directory || '';
            render();
            toast(state.language === 'en' ? 'Trajectory imported. Confirm replay separately to move.' : '轨迹已恢复，机械臂尚未回放，请检查预计时间后单独确认');
        });
        if ($('model-cal-load')) $('model-cal-load').oninput = event => { state.modelCalibrationOptions.load_description = event.target.value; };
        if ($('model-calibration-objective')) $('model-calibration-objective').onchange = event => { state.modelCalibrationObjective = event.target.value; };
        if ($('model-calibration-recompute')) $('model-calibration-recompute').onclick = () => blockingOperation(busyText('正在离线重算候选模型...', 'Recomputing candidate offline...'), async () => { if (!state.modelCalibrationDirectory) throw new Error('model calibration task directory is missing'); const recomputed = await api.request('model_calibration_recompute', { config: state.config, directory: state.modelCalibrationDirectory, ...captureModelCalibrationOptions(), objective: state.modelCalibrationObjective }); await refreshCalibrationCandidates('gravity',state.modelCalibrationDirectory,recomputed.candidate?.candidate_run_id || recomputed.path); await selectCalibrationToolRecord('gravity',{directory:state.modelCalibrationDirectory},state.calibrationCandidateSelection.gravity); state.modelCalibrationSavePreview = null; state.modelCalibrationExport = null; state.modelCalibrationFrictionExport = null; render(); });
        if ($('model-calibration-preview-save')) $('model-calibration-preview-save').onclick = () => blockingOperation(busyText('正在准备候选保存差异...', 'Preparing candidate save diff...'), async () => { const directory = state.modelCalibrationDirectory || currentModelCalibration()?.directory; if (!directory) throw new Error('model calibration task directory is missing'); state.modelCalibrationSavePreview = await api.request('model_calibration_preview_save', { config: state.config, directory, ...selectedCandidateParams('gravity') }); render(); });
        if ($('model-calibration-save')) $('model-calibration-save').onclick = () => blockingOperation(busyText('正在保存候选重力校正...', 'Saving candidate gravity correction...'), async () => { const directory = state.modelCalibrationDirectory || currentModelCalibration()?.directory; if (!directory || !state.modelCalibrationSavePreview) throw new Error('save preview is required'); state.modelCalibrationSaved = await api.request('model_calibration_save', { config: state.config, directory, expected_sha: state.modelCalibrationSavePreview.sha256, ...selectedCandidateParams('gravity') }); state.modelCalibrationSavePreview = null; await inspect(); state.readiness = await api.request('readiness',{config:state.config}); toast(`${busyText('已保存','Saved')} ${state.modelCalibrationSaved.gravity_correction_path}`); render(); });
        if ($('model-calibration-restore-config')) $('model-calibration-restore-config').onclick = () => blockingOperation(busyText('正在恢复保存前配置...', 'Restoring pre-save configuration...'), async () => { const directory = state.modelCalibrationDirectory || currentModelCalibration()?.directory; if (!directory) throw new Error('model calibration task directory is missing'); const result = await api.request('model_calibration_restore_config', { config: state.config, directory }); state.modelCalibrationSaved = null; state.modelCalibrationSavePreview = null; await inspect(); state.readiness = await api.request('readiness',{config:state.config}); toast(`${busyText('已恢复','Restored')} ${result.core_path}${result.gravity_validation_reset ? ' · '+busyText('重力模型确认已撤销','Gravity model confirmation reset') : ''}`); render(); });
        if (state.page === 'diagnostics') setTimeout(drawWorkbenchChart, 0);
        setTimeout(updateWorkbenchControlState, 0);
        refreshGraphicsDependentActions();
    }
    document.querySelectorAll('[data-mode]').forEach(b => { b.disabled = active(); b.onclick = () => { if (active()) return; state.mode = b.dataset.mode; operation(save); render(); }; });
    if (['robot', 'diagnostics'].includes(state.page) && active()) document.querySelectorAll('#page input,#page select,#source-builtin,#source-external,#browse-profile,#refresh-profiles,#add-root').forEach(e => e.disabled = true);
    if ($('launch-session')) { $('launch-session').disabled = state.busy || active() || !state.info?.available?.[state.mode]; $('launch-session').onclick = () => blockingOperation(busyText('正在检查启动条件...', 'Checking launch conditions...'), async () => { await inspect(); if (!state.info?.available?.[state.mode]) throw new Error(t('modeMissing')); hardwareConfirm(); }); }
    if ($('check-now')) $('check-now').onclick = () => blockingOperation(busyText('正在检查环境与资源...', 'Checking environment and resources...'), async () => { await inspect(); render(); });
    document.querySelectorAll('[data-theme]').forEach(b => b.onclick = event => operation(() => switchTheme(b.dataset.theme, event, true)));
    document.querySelectorAll('[data-language]').forEach(b => b.onclick = async () => { state.language = b.dataset.language; document.documentElement.lang = state.language; await operation(save); shell(); });
}
async function workbenchRequest(method, params = {}) { if (workbenchRecoveryRequired()) throw new Error('native session is unresponsive; force disconnect after making hardware safe'); if (!workbenchConnected()) throw new Error('workspace session is not connected'); return api.request('workbench_request', { method, params }); }
async function loadAdmittance() {
    if (!workbenchConnected()) return;
    const epoch = state.workbenchEpoch;
    const runtime = await workbenchRequest('get_admittance');
    if (epoch !== state.workbenchEpoch || !workbenchConnected()) return;
    state.admittance = runtime; state.tuneDraft = structuredClone(runtime); state.tuneDraftOrigin = 'runtime'; state.runtimeInteractionGoal = null; state.savePreview = null; refreshRuntimeSavePreview();
}
async function startWorkbench() {
    // Finish the inspection overlay before opening a confirmation or starting the connection
    // Both confirmed and remembered paths then use one connection overlay only
    const inspected = await blockingOperation(busyText('正在检查工作台连接条件...', 'Checking workspace connection...'), async () => {
        await inspect();
        if (!state.info?.available?.workbench) throw new Error('Structured workspace is unavailable');
        return true;
    });
    if (!inspected) return;
    if (state.info.write_enabled === false)
        return blockingOperation(busyText('正在连接控制工作台...', 'Connecting control workspace...'), () => connectWorkbench(false));
    return workbenchConfirm();
}
async function connectWorkbench(confirmed) { state.readiness = await api.request('readiness', { config: state.config }); state.busy = true; render(); try { synchronizeWorkbenchSession(await api.request('workbench_start', { config: state.config, confirmed, fingerprint: state.info?.fingerprint })); await loadAdmittance(); await refreshTeachTrajectories(); if (state.page === 'geometry') await refreshGeometryRepeatStatus(); state.calibrationRuntimeCandidates = await api.request('calibration_candidate_status', {}); state.teachLoadedId = ''; } finally { state.busy = false; render(); } }
function confirmCalibrationPark(action) {
    if (action !== 'park' || state.page !== 'calibration' || !modelCalibrationActive()) return false;
    const dialog = $('confirm-dialog');
    if (!dialog || dialog.open) return true;
    const en = state.language === 'en';
    dialog.innerHTML = `<h3>${en ? 'Park during calibration' : '标定过程中停放机械臂'}</h3><p>${en ? 'Parking may interrupt the current capture or cancel pending playback. Saved task files are retained. Check robot support, cables and the surrounding area first' : '停放可能中断当前采集或取消待执行回放，已经保存的任务文件不会删除，请先确认机械臂支撑、线缆和周围空间安全'}</p><div class="actions"><button class="button" id="calibration-park-cancel">${t('cancel')}</button><button class="button danger" id="calibration-park-confirm">${t('parkRobot')}</button></div>`;
    dialog.showModal();
    $('calibration-park-cancel').onclick = () => dialog.close();
    $('calibration-park-confirm').onclick = () => {
        dialog.close();
        blockingOperation(actionBusyMessage('park'), async () => {
            const snapshot = await workbenchCommand('park');
            if (snapshot && typeof snapshot === 'object' && snapshot.robot_state) applyWorkbenchSnapshot(snapshot);
            refreshCalibrationOrDefer();
        });
    };
    return true;
}
function activeTemporaryOverrides() {
    const status = state.calibrationRuntimeCandidates || {};
    return ['gain', 'gravity', 'dynamics'].filter(kind => status[kind + '_applied']);
}
function confirmUnresponsiveDisconnect() {
    if (!workbenchForceDisconnectAvailable()) return;
    const dialog = $('confirm-dialog');
    if (!dialog || dialog.open) return;
    const en = state.language === 'en';
    dialog.innerHTML = `<h3>${en ? 'Force disconnect control session' : '确认强制断开控制会话'}</h3><p>${en ? 'Hardware communication failed or shutdown could not be confirmed. This action stops the control process but cannot verify motor disable, safe parking, or temporary candidate restoration' : '硬件通信故障或正常退出未能确认完成，强制断连只能终止控制进程，无法确认电机已失能、机械臂已停放或临时参数已正常撤销'}</p><p>${en ? 'First physically secure the robot and cut actuator power if necessary. Do not rely on software controls. Reconnect only after hardware power and wiring are verified' : '请先确认机械臂获得可靠支撑，必要时通过硬件急停或断电确保安全，不要依赖失去响应的软件控制，检查供电和线缆后再重新连接'}</p><div class="actions"><button class="button" id="cancel-unresponsive-disconnect">${t('cancel')}</button><button class="button danger" id="confirm-unresponsive-disconnect">${en ? 'I secured hardware, disconnect' : '已确认硬件安全，强制断连'}</button></div>`;
    dialog.showModal();
    $('cancel-unresponsive-disconnect').onclick = () => dialog.close();
    $('confirm-unresponsive-disconnect').onclick = () => {
        dialog.close(); operation(async () => {
            await requestWorkbenchStop({ force: true });
            toast(busyText('已请求终止控制进程，未确认电机失能，请检查硬件再重新连接', 'Control process termination requested; hardware disable was not verified'));
        });
    };
}
function confirmWorkbenchDisconnect() {
    if (workbenchRecoveryRequired() || state.session?.disconnect_blocked === true) return confirmUnresponsiveDisconnect();
    const en = state.language === 'en';
    const kinds = activeTemporaryOverrides();
    const safePark = (state.readiness?.control_available === true && ['RIGID_TRACKING', 'COMPLIANT_TRACKING'].every(mode => ['configured', 'verified', 'stale'].includes(state.readiness?.modes?.[mode])));
    if (state.telemetry?.robot_state === 'ACTIVE' && (!safePark || kinds.length)) {
        return toast(en ? 'Support the robot and disable it before disconnecting, as automatic parking is not safe for this session' : '当前不能使用未验收的自动停放，请先可靠支撑机械臂，执行立即失能，再断开工作台', true);
    }
    if (!kinds.length) {
        return blockingOperation(busyText('正在安全断开工作台...', 'Safely disconnecting workspace...'), async () => {
            await requestWorkbenchStop();
        });
    }
    const dialog = $('confirm-dialog');
    if (!dialog || dialog.open) return;
    const names = kinds.map(kind => ({ gain: en ? 'gain' : '增益', gravity: en ? 'gravity' : '重力', dynamics: en ? 'dynamics' : '动力学' }[kind]));
    dialog.innerHTML = `<h3>${en ? 'Disconnect and remove temporary candidates' : '断连后撤销临时应用'}</h3><p>${en ? 'The following temporary runtime candidates are active' : '以下临时参数目前正在生效'}：<strong>${esc(names.join(' / '))}</strong></p><p>${en ? 'The robot must already be INACTIVE; the original runtime parameters will be restored before disconnecting. Saved YAML / URDF files do not change' : '请先在机械支撑下使机械臂处于 INACTIVE，断连仅恢复临时覆盖并结束会话，不修改 YAML 或 URDF'}</p><p>${en ? 'If safe parking or restoration fails, the session stays connected and you must inspect the robot state' : '如果安全停放或参数恢复失败，将保留工作台连接，请先排查机械臂状态'}</p><div class="actions"><button class="button" id="cancel-disconnect">${t('cancel')}</button><button class="button danger" id="confirm-disconnect">${en ? 'Restore and disconnect' : '撤销临时参数并断连'}</button></div>`;
    dialog.showModal();
    $('cancel-disconnect').onclick = () => dialog.close();
    $('confirm-disconnect').onclick = () => {
        dialog.close(); blockingOperation(busyText('正在撤销临时参数并断连...', 'Restoring temporary candidates and disconnecting...'), async () => {
            await requestWorkbenchStop();
            toast(en ? 'Temporary candidates removed before disconnect' : '临时应用已在断连前撤销');
        });
    };
}
// Reminder choices are private to a single configured Profile
// A new Profile, a different Profile path, or changed configuration must prompt again
function robotReminderStorageKey(kind) {
    const profile = String(state.config?.profile || '').trim();
    const path = String(state.info?.profile_file || state.config?.profile_file || '').trim();
    const fingerprint = String(state.info?.fingerprint || '').trim();
    if (!kind || !profile || !path || !fingerprint) return '';
    return 'serialarm.robot.reminder:' + JSON.stringify([kind, profile, path, fingerprint]);
}
function robotReminderDismissed(kind) {
    const key = robotReminderStorageKey(kind);
    if (!key) return false;
    try { return window.localStorage.getItem(key) === 'true'; } catch { return false; }
}
function rememberRobotReminder(kind) {
    const key = robotReminderStorageKey(kind);
    if (!key) return;
    try { window.localStorage.setItem(key, 'true'); } catch { /* Storage unavailable: show again next time */ }
}
function robotReminderCheckbox() {
    return `<label class="robot-reminder-choice"><input type="checkbox" id="robot-never-remind"><span>${state.language === 'en' ? 'Do not show this reminder again for this robot and configuration' : '永不提醒（仅限此机械臂的当前配置）'}</span></label>`;
}
function workbenchConfirm() {
    const kind = 'connect_workspace';
    // Explicitly clicking Connect remains necessary; only its repeated informational popup is optional
    if (robotReminderDismissed(kind)) {
        // Inspection has finished, so the remembered path must still issue workbench_start
        return blockingOperation(busyText('正在连接控制工作台...', 'Connecting control workspace...'), () => connectWorkbench(true));
    }
    const info = state.info, dialog = $('confirm-dialog');
    if (!info || !dialog || dialog.open) return;
    dialog.innerHTML = `<div class="eyebrow">${t('real')}</div><h3>${t('workbenchTitle')}</h3><p>${t('realDesc')}</p><dl class="kv"><dt>${t('profileName')}</dt><dd>${esc(info.profile)}</dd><dt>${t('serial')}</dt><dd>${esc(info.devices.join(', ') || '—')}</dd><dt>${t('writeEnabled')}</dt><dd>${t(info.write_enabled === true ? 'enabled' : 'unknown')}</dd></dl><p class="hint">${state.language === 'en' ? 'Connecting does not activate motors. Before enabling, check Safety acceleration, URDF position and velocity limits, joint mapping and mechanical support' : '连接不会使能电机，使能前请核对 Safety 加速度、URDF 位置及速度限制、电机映射和机械支撑'}</p><label class="confirmation"><input id="workspace-confirmed" type="checkbox"><span>${t('confirm')}</span></label>${robotReminderCheckbox()}<div class="actions"><button id="workspace-cancel" class="button">${t('cancel')}</button><button id="workspace-connect" class="button primary" disabled>${t('startWorkbench')}</button></div>`;
    dialog.showModal();
    $('workspace-confirmed').onchange = () => $('workspace-connect').disabled = !$('workspace-confirmed').checked;
    $('workspace-cancel').onclick = () => dialog.close();
    $('workspace-connect').onclick = () => {
        const remember = $('robot-never-remind')?.checked === true;
        dialog.close();
        blockingOperation(busyText('正在连接控制工作台...', 'Connecting control workspace...'), async () => {
            await connectWorkbench(true);
            if (remember) rememberRobotReminder(kind);
        });
    };
}
function safetyConfirmation(title, message, details = '', acknowledgements = [], category = 'safety', reminderKind = '') {
    const dialog = $('confirm-dialog');
    if (reminderKind && robotReminderDismissed(reminderKind)) return Promise.resolve({ accepted: true, remember: false });
    if (!dialog || dialog.open) return Promise.resolve(reminderKind ? { accepted: false, remember: false } : false);
    const checks = acknowledgements.map((label, index) => `<label class="confirmation"><input type="checkbox" data-safety-ack="${index}"><span>${esc(label)}</span></label>`).join('');
    dialog.innerHTML = `<div class="eyebrow">${category === 'general' ? 'SERIALARM · WORKSPACE' : 'SERIALARM · SAFETY'}</div><h3>${esc(title)}</h3><p>${esc(message)}</p>${details}<div class="safety-acknowledgements">${checks}</div>${reminderKind ? robotReminderCheckbox() : ''}<div class="actions"><button type="button" class="button" id="safety-dismiss">${t('cancel')}</button><button type="button" class="button primary" id="safety-accept" ${acknowledgements.length ? 'disabled' : ''}>${state.language === 'en' ? 'Confirm and continue' : '确认继续'}</button></div>`;
    return new Promise(resolve => {
        let settled = false;
        const finish = ok => { if (settled) return; settled = true; const remember = ok && dialog.querySelector('#robot-never-remind')?.checked === true; dialog.close(); resolve(reminderKind ? { accepted: ok, remember } : ok); };
        $('safety-dismiss').onclick = () => finish(false);
        $('safety-accept').onclick = () => finish(true);
        dialog.addEventListener('close', () => { if (!settled) { settled = true; resolve(reminderKind ? { accepted: false, remember: false } : false); } }, { once: true });
        dialog.querySelectorAll('[data-safety-ack]').forEach(box => box.onchange = () => {
            $('safety-accept').disabled = ![...dialog.querySelectorAll('[data-safety-ack]')].every(item => item.checked);
        });
        dialog.showModal();
    });
}
function safetySettingsDetails() {
    const summary = state.readiness?.safety_summary || {};
    const acc = summary.max_acc || {};
    const joints = summary.joint_names || Object.keys(acc);
    const en = state.language === 'en';
    const number = raw => raw == null || String(raw).trim() === '' ? null : Number.isFinite(Number(raw)) ? Number(raw) : null;
    const rows = joints.map((name,index) => `<div class="diag-row"><span class="name">${esc(name)}</span><span class="detail">${jointDisplayValue(number(Array.isArray(acc)?acc[index]:acc[name]),name,'acceleration')}</span></div>`).join('');
    const limits = summary.urdf_limits || {};
    const limitRows = joints.map((name,index) => {
        const x = limits[name] || {};
        const range = x.lower != null && x.upper != null ? `${jointDisplayValue(number(x.lower),name)} ~ ${jointDisplayValue(number(x.upper),name)}` : (en ? 'Continuous rotation' : '连续旋转');
        const joint = state.modelData?.model?.joints?.find(item=>item.name===name) || {type:state.profileEditor?.urdf_joint_limits?.[name]?.type,limit:x};
        const allowed = effectiveJointBounds(name,index,joint);
        const effective = !allowed.limited ? range : `${jointDisplayValue(allowed.lower,name)} ~ ${jointDisplayValue(allowed.upper,name)}`;
        return `<div class="diag-row"><span class="name">${esc(name)}</span><span class="detail">URDF ${esc(range)} · ${en ? 'Allowed target' : '可执行目标'} ${esc(effective)} · ${en ? 'Velocity' : '速度'} ${jointDisplayValue(number(x.velocity),name,'velocity')}</span></div>`;
    }).join('');
    const warning = summary.require_continuous_cmd === false ? `<p class="safety-step-warning">${en ? 'Continuous command checking is OFF, so max_acc is not a universal per-cycle acceleration limiter' : '当前连续命令检查为 OFF，max_acc 不代表所有控制命令都会进行逐周期加速度限幅'}</p>` : '';
    return `<div class="safety-parameter-review"><b>${en ? 'Safety acceleration limits' : 'Safety 加速度上限'}</b>${rows || `<p>${en ? 'Acceleration limits are missing' : '缺少加速度上限'}</p>`}<b class="safety-limit-title">${en ? 'Position and velocity limits' : '位置与速度限位'}</b>${limitRows}<p>${en ? 'Check these loaded limits against the physical zero, joint mapping and available workspace before activation' : '使能前，请将以上限制与实机零位、关节映射和可用运动空间核对'}</p>${warning}</div>`;
}
async function workbenchCommand(action) {
    if (action === 'clear_fault' && !await safetyConfirmation(busyText('确认清除故障','Confirm fault recovery'),busyText('已建立故障保持时，清除故障会恢复原位置保持；启动失败时则保持失能，请先排除故障原因并确认机械支撑和运动空间','Clearing a maintained fault resumes position hold; an unsuccessful startup remains disabled. Resolve the fault and check mechanical support and clearance first'),'',[busyText('已排除故障原因并确认可以恢复保持','The fault is resolved and position hold can safely resume')])) return null;
    const needsConfirmation = action === 'activate' && state.confirmedActivationPid !== state.session?.pid;
    let rememberActivation = false;
    if (needsConfirmation) {
        const en = state.language === 'en';
        const choice = await safetyConfirmation(en ? 'Confirm first activation' : '首次使能安全确认',
            en ? 'The robot can move immediately after enabling. Review the loaded limits and confirm the physical setup' : '使能后机械臂可能立即产生力矩，请先核对下列配置及真实硬件状态',
            safetySettingsDetails(),
            en ? ['Acceleration limits and URDF position/velocity limits have been checked', 'Joint mapping, support, emergency stop and free workspace have been confirmed'] :
                ['已核对 Safety 加速度及 URDF 位置和速度限制', '已核对电机映射、机械支撑、急停及运动空间'], 'safety', 'first_activation');
        if (!choice.accepted) return null;
        rememberActivation = choice.remember;
    }
    const methods = { activate: 'activate', park: 'park', deactivate: 'deactivate', hold: 'hold', rigid_hold: state.telemetry?.robot_state === 'FAULT' ? 'fault_rigid' : 'hold', clear_fault: 'clear_fault', fault_compliant: 'fault_compliant', fault_rigid: 'fault_rigid' };
    const method = methods[action];
    if (!method) throw new Error('unknown workspace action');
    state.commandState = 'submitting';
    try {
        const result = action === 'park' ? await interruptibleParking(() => workbenchRequest(method)) : await workbenchRequest(method, action === 'rigid_hold' && method === 'hold' ? {rigid: true} : {});
        if (needsConfirmation) {
            state.confirmedActivationPid = state.session?.pid;
            if (rememberActivation) rememberRobotReminder('first_activation');
        }
        state.commandState = 'accepted';
        return result;
    } catch (error) { state.commandState = 'failed'; throw error; }
}

async function applyTuning() {
    if (!admittanceMayTune()) throw new Error(state.language === 'en' ? 'Runtime admittance tuning is not permitted in this state' : '当前机器人状态不允许在线导纳调节');
    const epoch = state.workbenchEpoch;
    const draft = state.tuneDraft || state.admittance;
    const actual = state.admittance;
    if (!draft || !actual) throw new Error('admittance parameters are not loaded');
    const params = {};
    const n = (state.telemetry?.joint_names || []).length;
    for (const key of ['mass', 'damping', 'stiffness']) {
        if (!Array.isArray(draft[key]) || draft[key].length !== n || !draft[key].every(v => typeof v === 'number' && Number.isFinite(v) && (key === 'mass' ? v > 0 : v >= 0)))
            throw new Error(`${key}: ${state.language === 'en' ? 'Mass must be positive; damping and stiffness must be nonnegative finite values' : '质量必须大于零，阻尼与刚度必须为非负有限数值'}`);
        if (draft[key].some((v, i) => v !== actual[key]?.[i])) params[key] = draft[key].slice();
    }
    if (!Object.keys(params).length) return;
    const en = state.language === 'en';
    if (!await safetyConfirmation(en ? 'Apply online admittance' : '应用在线导纳参数', en ? 'Update M/D/K while preserving the accepted virtual motion state; Profile settings stay unchanged. Review response and torque limits after the update' : '更新 M/D/K 并保留已接受的虚拟运动状态，不写入 Profile；更新后请观察响应与力矩限幅')) return;
    if (epoch !== state.workbenchEpoch || !admittanceMayTune()) return;
    await workbenchRequest('set_admittance', params);
    if (epoch !== state.workbenchEpoch || !workbenchConnected()) return;
    if (state.responseCapture?.recording) { state.responseCapture.recording = false; state.responseCapture.stop_reason = en ? 'Stopped: M/D/K changed; start a new comparison trial' : '记录停止：M/D/K 已变更，请开始新的对比记录'; }
    await loadAdmittance();
    if (epoch !== state.workbenchEpoch || !workbenchConnected()) return;
    state.savePreview = null; refreshRuntimeSavePreview();
    refreshTuningInputs();
}
async function finishCalibrationStage() { const before = state.calibrationFlow; await workbenchRequest('calibration_finish'); if (before === 'static') { state.calibrationFlow = 'validation'; await workbenchRequest('calibration_begin', { kind: 'validation' }); } else if (before === 'validation') { state.calibrationFlow = 'friction-ready'; } state.admittance = await workbenchRequest('get_admittance'); state.tuneDraft = structuredClone(state.admittance); render(); }
function frictionConfirm() { const problem = frictionReplayProblem(); if (problem) { toast(problem,true); return; } const dialog = $('confirm-dialog'); dialog.innerHTML = `<h3>${t('frictionCalibration')}</h3><p>${state.language === 'en' ? 'The robot will replay the demonstration path in reverse and forward directions; confirm people, cables, and surrounding space are safe, and fully release the robot' : '机械臂将反向和正向回放示教轨迹，请确认人员、线缆和周围空间安全，并确认已完全松手'}</p><div class="actions"><button id="friction-cancel-dialog" class="button">${t('cancel')}</button><button id="friction-confirm-replay" class="button danger">${t('startReplay')}</button></div>`; dialog.showModal(); $('friction-cancel-dialog').onclick = () => dialog.close(); $('friction-confirm-replay').onclick = () => { const problem = frictionReplayProblem(); if (problem) { toast(problem,true); return; } dialog.close(); blockingOperation(busyText('正在启动双向摩擦回放...', 'Starting bidirectional friction replay...'), () => workbenchRequest('friction_replay_start')); }; }
async function launch(confirmed) { state.busy = true; render(); try { state.session = await api.request('start', { mode: state.mode, config: state.config, confirmed, fingerprint: state.info?.fingerprint }); } finally { state.busy = false; render(); setTimeout(() => fit?.fit(), 50); } }
function hardwareConfirm() {
    const info = state.info; const dialog = $('confirm-dialog');
    dialog.innerHTML = `<div class="eyebrow">${t('real')}</div><h3 style="margin-top:10px">${t('realTitle')}</h3><p>${t('realDesc')}</p><dl class="kv"><dt>${t('profileName')}</dt><dd>${esc(info.profile)}</dd><dt>${t('serial')}</dt><dd>${esc(info.devices.join(', ') || '—')}</dd><dt>${t('writeEnabled')}</dt><dd style="color:var(--accent)">${t(info.write_enabled === true ? 'enabled' : info.write_enabled === false ? 'disabled' : 'unknown')}</dd><dt>${t('controllers')}</dt><dd>${esc(info.controllers.join(', '))}</dd></dl><label class="confirmation"><input id="hardware-confirmed" type="checkbox"><span>${t('confirm')}</span></label><div class="actions">${button('cancel-launch', 'cancel', '')}${button('confirm-launch', 'confirmLaunch', 'run', 'primary')}</div>`;
    dialog.showModal(); $('confirm-launch').disabled = true; $('hardware-confirmed').onchange = () => $('confirm-launch').disabled = !$('hardware-confirmed').checked;
    $('cancel-launch').onclick = () => dialog.close(); $('confirm-launch').onclick = () => { dialog.close(); blockingOperation(busyText('正在启动运行会话...', 'Starting runtime session...'), () => launch(true)); };
}
function forceConfirm() { const dialog = $('confirm-dialog'); dialog.innerHTML = `<h3>${t('forceTitle')}</h3><p>${t('forceDesc')}</p><div class="actions">${button('cancel-force', 'cancel', '')}${button('confirm-force', 'force', '', 'danger')}</div>`; dialog.showModal(); $('cancel-force').onclick = () => dialog.close(); $('confirm-force').onclick = () => { dialog.close(); blockingOperation(busyText('正在强制结束运行...', 'Force stopping runtime...'), () => api.request('stop', { force: true })); }; }
function onEvent(event) {
    if (event.event === 'output') { outputBuffer = (outputBuffer + event.data).slice(-1024 * 1024); term?.write(event.data); }
    if (event.event === 'native_log') { outputBuffer = (outputBuffer + event.data).slice(-1024 * 1024); term?.write(event.data); }
    if (event.event === 'telemetry') {
        if(!workbenchConnected() || (event.session_id && event.session_id!==state.session?.session_id))return;
        const previousGeometryReplayPhase = state.telemetry?.teach_replay?.phase;
        const previousCalibrationPhase = state.telemetry?.model_calibration?.phase;
        const previousRobotState = state.telemetry?.robot_state;
        const previousValid = state.telemetry?.valid === true;
        acceptTelemetrySnapshot(event.data); state.telemetryHistory.push(event.data); if (state.telemetryHistory.length > 600) state.telemetryHistory.shift();
        recordResponseSnapshot(event.data);
        checkRuntimeAlerts(event.data);
        if (state.page === 'workbench' && state.workbenchTab === 'replay' && previousGeometryReplayPhase === 'playing' && event.data?.teach_replay?.phase === 'ready') { state.teachPreviewDirection = null; updateStartGhost(true); }
        if (state.page === 'geometry' && state.geometryTab === 'repeat' && previousGeometryReplayPhase && previousGeometryReplayPhase !== event.data?.teach_replay?.phase) { refreshGeometryPanel(); }
        const calibration = event.data?.model_calibration;
        // Historical record selection belongs to the user, not the streaming
        // native session. A FAULT or task transition cannot clear it or swap
        // its directory for a different (possibly incomplete) live task.
        if (calibration?.directory && !state.modelCalibrationOffline) state.modelCalibrationDirectory = calibration.directory;
        const phaseBoundary = calibration?.phase !== previousCalibrationPhase && ['waiting_replay_confirmation', 'complete', 'cancelled', 'failed'].includes(calibration?.phase);
        const robotStateChanged = state.telemetry?.robot_state !== previousRobotState;
        if (state.page === 'diagnostics' && !previousValid && event.data?.valid === true) {
            render();
        } else if (phaseBoundary && state.page === 'calibration') {
            refreshCalibrationOrDefer();
        } else {
            updateWorkbenchLive();
        }
        if (robotStateChanged && state.page === 'geometry') refreshGeometryPanel();
        if (robotStateChanged) { refreshGainApplyControls(); refreshCalibrationQuickControls(); refreshCalibrationActionControls(); }
        syncCompletedGainCandidate(calibration);
    }
    if (event.event === 'session') { const hadTemporary=activeTemporaryOverrides().length>0; if(event.mode==='workbench') { if(!synchronizeWorkbenchSession(event))return; } else state.session=event; if (event.mode === 'workbench' && event.state === 'running' && !event.recovery_required && !state.admittance) setTimeout(() => operation(loadAdmittance), 0); if (event.state === 'exited') { state.teachLoadedId = ''; state.teachRuntime = null; state.geometryContactReady = false; state.geometryRepeatSupported = null; state.geometryRepeatError = ''; state.admittance = null; state.tuneDraft = null; state.calibrationRuntimeCandidates = null; if (event.force_stop_unverified && event.mode === 'workbench') toast(busyText('已强制终止控制进程，但无法确认执行器已经失能或临时参数已安全恢复，请检查硬件', 'Control process force-terminated; motor disable and safe temporary parameter restoration are unverified'), true); else if (hadTemporary && event.mode === 'workbench') toast(busyText('工作台已断连，临时参数不再生效，重新连接后将使用原始配置', 'Workspace disconnected; temporary parameters no longer apply, original settings will be used on reconnect')); const overlay = $('model-calibration-overlay'); if (overlay) { overlay.hidden = true; overlay.setAttribute('aria-hidden', 'true'); } } render(); }
    if (event.event === 'error') { state.backendError = event.error; toast(event.error, true); statusUI(); }
}
async function boot() {
    if (!api) { $('app').innerHTML = '<div class="empty-state">请通过 ./launch.sh 启动桌面应用 / Start the desktop application using ./launch.sh</div>'; return; }
    try {
        const prefs = await api.prefs(); state.language = prefs.language || 'zh-CN'; state.theme = prefs.theme || 'system'; state.mode = ['terminal', 'hardware', 'moveit'].includes(prefs.mode) ? prefs.mode : 'terminal';
        state.config = { profile: '', profile_file: '', serial_port: '', baudrate: '', bus: '', resource_paths: '' }; state.source = 'builtin'; state.page = 'start';
        await refreshGraphicsStatus();
        applyTheme(); document.documentElement.lang = state.language; shell();
        $('priority-error-ack').onclick = acknowledgePriorityError;
        api.onEvent(onEvent);
        state.status = await api.request('status'); state.session = state.status.session || { state: 'idle' }; if(state.session.mode==='workbench')synchronizeWorkbenchSession(state.session); outputBuffer = await api.request('logs'); if (outputBuffer) term?.write(outputBuffer);
        await refreshLibrary(); render();
    } catch (error) { state.backendError = error.message; if ($('page')) { render(); toast(error, true); } }
}
window.addEventListener('resize', () => { if (state.page === 'run') { try { fit?.fit(); } catch { } } if (['model', 'workbench', 'geometry', 'calibration'].includes(state.page)) modelView?.resize(); });
document.addEventListener('visibilitychange', () => { if (!document.hidden) updateStartGhost(true); });
setInterval(refreshTelemetryFreshness, 200);
matchMedia('(prefers-color-scheme: light)').addEventListener('change', () => applyTheme());
boot();
