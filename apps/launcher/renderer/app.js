'use strict';
const api = window.serialArm;
const words = {
    ros2: ['ROS2', 'ROS2'],
    workspace: ['机械臂工作台', 'Manipulator workspace'], subtitle: ['串联机械臂配置与运行', 'Serial manipulator workspace'], subline: ['控制 · 动力学 · 交互', 'Control · Dynamics · Interaction'],
    navigation: ['工作流', 'WORKFLOW'], application: ['应用', 'APPLICATION'], robot: ['机器人与连接', 'Robot & connection'], run: ['运行', 'Run'], diagnostics: ['诊断', 'Diagnostics'], settings: ['设置', 'Settings'],
    ready: ['就绪', 'Ready'], unavailable: ['不可用', 'Unavailable'], connected: ['安装已就绪', 'Installation ready'], notInstalled: ['尚未安装', 'Not installed'], idle: ['空闲', 'Idle'], running: ['运行中', 'Running'], stopping: ['正在停止', 'Stopping'], exited: ['已退出', 'Exited'],
    robotTitle: ['选择机器人，配置连接', 'Choose a robot; Configure its connection'], robotDesc: ['使用内置或外部 Profile，将设备设置作为本次运行的覆盖值', 'Use a built-in or external profile and set connection overrides for this session'],
    profile: ['Robot Profile', 'Robot Profile'], profileSub: ['配置的唯一入口', 'A single configuration entry'], source: ['配置来源', 'Profile source'], builtin: ['内置 Profile', 'Built-in profiles'], external: ['外部 Profile 文件', 'External profile file'], profileFile: ['Profile 文件路径', 'Profile file path'], browse: ['选择文件', 'Browse'], refresh: ['刷新', 'Refresh'], profileName: ['机器人', 'Robot'],
    connection: ['连接设置', 'Connection'], connectionSub: ['不修改原始 YAML 配置', 'Original YAML files stay unchanged'], serial: ['串口设备', 'Serial device'], automatic: ['使用 Profile 默认值', 'Use profile default'], baudrate: ['波特率', 'Baudrate'], bus: ['Bus 名称', 'Bus name'], resourcePaths: ['资源搜索路径', 'Resource search paths'], addRoot: ['添加目录', 'Add directory'], rootsHint: ['多个目录用冒号分隔；支持下游工作区或安装前缀', 'Separate roots with a colon; downstream workspaces and install prefixes are supported'], overrideHint: ['留空使用 Profile 默认值', 'Leave blank to use the profile default'], inspect: ['检查配置', 'Inspect configuration'], toRun: ['选择运行模式', 'Choose run mode'],
    summary: ['配置概览', 'Profile overview'], summarySub: ['实际资源与执行器写入状态', 'Resolved resources and actuator write state'], hardwarePlugin: ['硬件后端', 'Hardware backend'], writeEnabled: ['执行器写入', 'Actuator writes'], enabled: ['已开启', 'Enabled'], disabled: ['已关闭', 'Disabled'], unknown: ['未知', 'Unknown'], controllers: ['Controllers', 'Controllers'], noProfile: ['请选择一个 Profile', 'Select a profile'], status: ['环境状态', 'Environment status'], statusSub: ['只读检查，不连接执行器', 'Read-only checks, no actuator connection'], resources: ['Profile 资源', 'Profile resources'], installation: ['安装状态', 'Installation'], nativeTerminal: ['C++ Terminal', 'C++ Terminal'],
    runtimeTitle: ['选择运行方式', 'Choose how to run'], runtimeDesc: ['选择 Terminal、Hardware 或 MoveIt 运行入口，日志和 Terminal 输入集中在这里', 'Choose Terminal, Hardware, or MoveIt; logs and Terminal input stay in one place'],
    model: ['Model', 'Model'], modelDesc: ['查看模型、坐标系与动力学参数', 'Inspect model, frames and dynamics'], terminal: ['Terminal', 'Terminal'], terminalDesc: ['进入 C++ 交互终端，调试机械臂', 'Open the C++ interactive terminal'], hardware: ['Hardware', 'Hardware'], hardwareDesc: ['运行 ros2_control 和 Controllers', 'Run ros2_control and controllers'], moveit: ['MoveIt', 'MoveIt'], moveitDesc: ['启动规划、硬件和 RViz', 'Start planning, hardware and RViz'], offline: ['离线', 'Offline'], real: ['真实硬件', 'Real hardware'],
    launch: ['启动', 'Launch'], stop: ['停止', 'Stop'], force: ['强制结束', 'Force stop'], clear: ['清空显示', 'Clear view'], copyAll: ['复制全部', 'Copy all'], paste: ['粘贴', 'Paste'], logs: ['运行终端', 'Runtime terminal'], terminalHint: ['拖选后 Ctrl+Shift+C 复制，Ctrl+Shift+V 粘贴，右键打开菜单；Ctrl+C 仍用于中断', 'Select text and press Ctrl+Shift+C to copy, Ctrl+Shift+V to paste, or right-click for the menu; Ctrl+C still interrupts'], noSession: ['等待启动会话', 'Waiting for a session'], modeMissing: ['此模式缺少安装组件或 Profile 资源，请查看诊断', 'This mode needs installed components or profile resources; Check diagnostics'], selectedRobot: ['当前机器人', 'Selected robot'],
    diagTitle: ['检查环境与资源', 'Inspect environment and resources'], diagDesc: ['核对安装状态、Profile 路径和设备权限，定位启动条件', 'Check installation state, profile paths and device permissions before launching'], checkNow: ['重新检查', 'Check again'], checks: ['检查结果', 'Check results'], checkSub: ['不实例化 Hardware，也不打开设备', 'Does not instantiate hardware or open devices'], pass: ['通过', 'Pass'], fail: ['未通过', 'Failed'], installHelp: ['安装与环境', 'Installation & environment'], installHelpSub: ['从仓库根目录执行', 'Run from the repository root'], installHint: ['安装器会按用途配置组件，GUI 自动加载安装后的环境', 'The installer configures selected components; the GUI loads their environment'], validationHint: ['资源检查不代表真机验收；Core 在运行入口中执行配置与硬件能力校验', 'Resource checks are not a hardware acceptance test; Core validates configuration and hardware capabilities at runtime'],
    settingsTitle: ['让工作台适合你', 'Make the workspace yours'], settingsDesc: ['偏好会自动保存，不影响正在运行的会话', 'Preferences are saved automatically without restarting an active session'], appearance: ['外观', 'Appearance'], appearanceSub: ['选择工作台主题', 'Choose a workspace theme'], system: ['跟随系统', 'System'], dark: ['深色', 'Dark'], light: ['浅色', 'Light'], language: ['语言', 'Language'], languageSub: ['随时切换界面语言', 'Switch the interface language at any time'], prefsNote: ['语言、主题和窗口偏好会自动保存；机械臂每次从启动入口明确选择', 'Language, theme, and window preferences are saved; choose the robot explicitly from the start workflow each time'],
    realTitle: ['启动真机会话', 'Start a hardware session'], realDesc: ['该入口可能连接并驱动真实执行器，请核对下面的运行配置', 'This entry point can connect to and drive real actuators; Review the configuration below'], confirm: ['我已确认机器人工作区可运行，并了解当前执行器写入状态', 'I have checked the robot workspace and understand the actuator write state'], cancel: ['取消', 'Cancel'], confirmLaunch: ['确认并启动', 'Confirm and launch'], forceTitle: ['强制结束运行进程', 'Force stop the session'], forceDesc: ['强制结束会中断运行进程的清理流程；请先确认机器人状态', 'Force stop interrupts process cleanup; Check the robot state first'], error: ['操作失败', 'Operation failed'], fileHint: ['选择 robot_profiles.yaml', 'Choose robot_profiles.yaml'], saved: ['已保存', 'Saved'], core: ['Core YAML', 'Core YAML'], description: ['模型文件', 'Model file'], ros2_control: ['ros2_control Xacro', 'ros2_control Xacro'], device: ['设备权限', 'Device access'], backendMissing: ['运行后端不可用', 'Backend unavailable'], backend: ['运行后端', 'Runtime backend'],
    modelTitle: ['模型工作台', 'Model workspace'], modelPageDesc: ['查看机器人结构、坐标系、关节运动与动力学参数', 'Inspect robot structure, frames, joint motion, and dynamics'],
    workbenchNav: ['控制工作台', 'Control workspace'], workbenchTitle: ['机械臂控制工作台', 'Manipulator control workspace'], workbenchDesc: ['完成控制、导纳调参、标定与运行诊断', 'Control, tune admittance, calibrate, and diagnose in one shared session'],
    controlTab: ['控制', 'Control'], tuningTab: ['调参', 'Tuning'], calibrationTab: ['标定', 'Calibration'], runtimeDiagTab: ['诊断', 'Diagnostics'], startWorkbench: ['连接工作台', 'Connect workspace'], stopWorkbench: ['停放并断开', 'Park & disconnect'], activateRobot: ['使能', 'Activate'], deactivateRobot: ['立即失能', 'Immediate disable'], holdRobot: ['当前位置保持', 'Hold current'], parkRobot: ['停放并失能', 'Park & disable'], clearFault: ['清除故障', 'Clear fault'], compliantRecovery: ['柔性恢复', 'Compliant recovery'], rigidRecovery: ['返回刚性保持', 'Return rigid hold'], apply: ['应用', 'Apply'], reset: ['重置', 'Reset'], previewSave: ['预览保存差异', 'Preview save diff'], saveConfig: ['确认保存', 'Save confirmed changes'], exportData: ['导出会话数据', 'Export session data'], actual: ['实际', 'Actual'], target: ['目标', 'Target'], delta: ['相对增量', 'Relative delta'], speedScale: ['速度比例', 'Speed scale'], executeAbsolute: ['执行绝对目标', 'Execute absolute target'], executeRelative: ['执行相对目标', 'Execute relative target'], observerMode: ['Observer 模式', 'Observer mode'], staticCalibration: ['静态残差标定', 'Static residual calibration'], staticValidation: ['独立静态验证', 'Independent static validation'], frictionCalibration: ['双向摩擦标定', 'Bidirectional friction calibration'], capturePose: ['采集当前姿态', 'Capture current pose'], finishFit: ['完成拟合', 'Finish fit'], cancelTask: ['取消任务', 'Cancel task'], startRecording: ['开始示教记录', 'Start demonstration'], stopRecording: ['结束示教记录', 'Stop demonstration'], startReplay: ['确认安全并开始回放', 'Confirm safety & replay'], feedbackAge: ['反馈年龄', 'Feedback age'], commandState: ['命令状态', 'Command state'],
    modelCalibration: ['重力模型校正', 'Gravity model calibration'], modelCalibrationDesc: ['一次拖动示教后自动完成静态采样、重力校正与摩擦验证', 'One demonstration drives automatic static sampling, gravity calibration, and friction validation'], modelCalibrationStart: ['开始拖动示教', 'Start demonstration'], modelCalibrationStopTeach: ['结束示教', 'Stop demonstration'], modelCalibrationConfirmReplay: ['确认松手并开始自动回放', 'Confirm release and start replay'], pauseTask: ['暂停', 'Pause'], resumeTask: ['继续', 'Resume'], applyCandidate: ['应用候选重力', 'Apply candidate gravity'], restoreCandidate: ['恢复校正前运行配置', 'Restore pre-calibration runtime configuration'], saveCandidate: ['保存候选', 'Save candidate'], exportCandidate: ['导出候选 URDF', 'Export candidate URDF'], loadRecord: ['加载任务记录', 'Load task record'], recomputeRecord: ['离线重算', 'Recompute offline'],
    impedanceMode: ['阻抗模式', 'Impedance mode'], modelFeedforward: ['模型前馈', 'Model feedforward'], jointCommand: ['关节命令', 'Joint command'], joint: ['关节', 'Joint'], admittanceParameters: ['导纳 M / D / K', 'Admittance M / D / K'], gravityScale: ['重力补偿比例', 'Gravity scale'], noChanges: ['没有变化', 'No changes'],
    sourceLive: ['在线会话', 'Live session'], sourceOffline: ['离线记录', 'Offline record'], sourceWaiting: ['离线', 'Offline'], poseBudget: ['姿态数量上限', 'Pose budget'], validationFraction: ['留出验证比例', 'Validation fraction'], regularizationStrength: ['正则化起点（按姿态自动交叉验证）', 'Regularization baseline (pose-group CV)'], svdRelativeThreshold: ['SVD 相对阈值', 'SVD relative threshold'], minimumInformationScore: ['最小信息量', 'Minimum information score'], taskIdentifier: ['任务标识', 'Task identifier'], progressLabel: ['任务进度', 'Progress'], trajectorySamples: ['轨迹采样点', 'Trajectory samples'], trainingValidationGroups: ['训练 / 留出姿态组', 'Training / validation groups'], validStaticSamples: ['有效静态样本', 'Valid static samples'], acceptedDroppedFrames: ['已接收 / 丢弃原始帧', 'Accepted / dropped raw frames'],
    currentUrdfRms: ['当前 URDF RMS', 'Current URDF RMS'], currentScaleRms: ['当前 gravity_scale RMS', 'Current gravity_scale RMS'], candidateRms: ['候选 RMS', 'Candidate RMS'], candidateP99: ['候选 P99', 'Candidate P99'], candidateMaximum: ['候选最大误差', 'Candidate maximum'], noiseRms: ['噪声 RMS', 'Noise RMS'], staticPassed: ['静态验证通过', 'Static validation passed'], staticFailed: ['静态验证未通过', 'Static validation failed'], frictionPassed: ['摩擦验证通过', 'Friction validation passed'], frictionFailed: ['摩擦验证未通过', 'Friction validation failed'], dynamicsNotIdentified: ['完整惯性参数尚未验证', 'Full inertial parameters not yet validated'], numericalRank: ['数值秩', 'Numerical rank'], candidateComparison: ['候选模型比较', 'Candidate model comparison'], candidateScope: ['仅校正静态重力 · 质量与质心处惯量保持先验 · 质量矩阵与科氏项不替换', 'Static gravity correction only · mass and inertia-at-COM remain priors · mass matrix and Coriolis terms are not replaced'], fixedMass: ['固定质量', 'Fixed mass'], currentCom: ['当前 COM', 'Current COM'], candidateCom: ['候选 COM', 'Candidate COM'], comOffset: ['偏移', 'Offset'], observableAxes: ['可观测轴', 'Observable axes'], restoreSavedConfig: ['恢复校正前保存配置', 'Restore configuration from before calibration save'], taskDirectory: ['任务目录', 'Task directory'], singleCalibrationTools: ['单项标定工具', 'Individual calibration tools'], staticCalibrationDesc: ['逐姿态采集并拟合静态残差', 'Capture poses and fit static residuals'], start: ['开始', 'Start'], startValidation: ['开始验证', 'Start validation'], taskType: ['任务类型', 'Task type'], taskPhase: ['任务阶段', 'Task phase'], errorLabel: ['错误', 'Error'], replayReadyHint: ['示教路径已冻结', 'Demonstration path is frozen'], samplePoints: ['采样姿态', 'Sampling poses'], replayRate: ['回放速度比例', 'Replay rate'], estimatedDuration: ['预计自动阶段', 'Estimated automatic stage'], durationLimit: ['任务上限', 'Task limit'], releaseSafetyHint: ['请检查机械臂路径、负载、线缆和周围空间，并完全松手后确认回放', 'Check the robot path, load, cables, and surrounding space, then fully release the robot before confirming replay'],
    telemetryTitle: ['实时遥测', 'Live telemetry'], frameTitle: ['坐标系', 'Frame'], dynamicsTitle: ['动力学', 'Dynamics'], jointActuatorTitle: ['关节 / 执行器', 'Joint / actuator'], onlineLabel: ['在线', 'Online'], enabledLabel: ['使能', 'Enabled'], positionLabel: ['位置', 'Position'], quaternionLabel: ['四元数 xyzw', 'Quaternion xyzw'], gravityLabel: ['重力项', 'Gravity'], gravityCompLabel: ['重力补偿', 'Gravity compensation'], coriolisLabel: ['科氏项', 'Coriolis'], inverseDynamicsLabel: ['逆动力学', 'Inverse dynamics'], actualTelemetry: ['实际状态 · 实时反馈', 'Actual state · live telemetry'], offlineWaiting: ['离线状态 · 等待工作台连接', 'Offline · waiting for workspace connection'], modelNotLoaded: ['模型尚未加载', 'Model is not loaded'], processing: ['正在处理...', 'Processing...'], stopTeaching: ['结束示教', 'Stop demonstration'],
    modelLoad: ['加载模型', 'Load model'], modelReload: ['重新加载', 'Reload'], modelTree: ['模型树', 'Model tree'], modelViewport: ['模型视图', 'Model view'], modelInspector: ['参数检查器', 'Inspector'],
    visualLayer: ['Visual', 'Visual'], collisionLayer: ['Collision', 'Collision'], linkFrameLayer: ['Link Frame', 'Link frames'], jointAxisLayer: ['Joint Axis', 'Joint axes'], comLayer: ['质心', 'COM'], inertiaLayer: ['惯性', 'Inertia'], labelsLayer: ['标签', 'Labels'], allLayers: ['全选图层', 'All layers'], noLayers: ['全不选', 'Clear layers'], showAllObjects: ['全部显示', 'Show all'], hideAllObjects: ['全部隐藏', 'Hide all'], resetView: ['复位视角', 'Reset view'],
    offlinePreview: ['模型预览', 'Model preview'], jointPreview: ['关节预览', 'Joint preview'], urdfInertial: ['URDF &lt;inertial&gt;', 'URDF &lt;inertial&gt;'], coreReducedInertia: ['Core 约简后惯性', 'Core reduced inertia'], effectiveInertia: ['Core 有效惯性', 'Core effective inertia'],
    modelUnavailable: ['Model 原生只读入口尚不可用，请先完成 Standalone 安装', 'The native read-only Model entry is unavailable; install Standalone first'], noModel: ['尚未加载模型', 'Model is not loaded'],
    startQuestion: ['准备从哪里开始', 'Where do you want to start?'], startDesc: ['选择最符合当前机械臂状态的入口，工作台会继续引导下一步', 'Choose the entry that matches the robot state; the workspace will guide the next step'],
    newDescription: ['新的机械臂', 'New manipulator description'], newDescriptionDesc: ['从 SerialArm Description 包、URDF 或 Xacro 创建新的 Profile', 'Create a Profile from a SerialArm Description package, URDF, or Xacro'],
    continueProfile: ['继续配置机械臂', 'Continue configuring a manipulator'], continueProfileDesc: ['打开仍需检查、校准、标定或验证的 Profile', 'Open a Profile that still needs inspection, calibration, or validation'],
    useProfile: ['使用机械臂', 'Use a manipulator'], useProfileDesc: ['打开已经验证完成的 Profile 并进入正常控制流程', 'Open a validated Profile and enter the normal control workflow'],
    continueLibrary: ['待配置的 Profile', 'Profiles to configure'], readyLibrary: ['可使用的 Profile', 'Ready Profiles'], newModelCheck: ['模型检查', 'Model check'], newModelHint: ['仅检查模型能否解析和控制链是否完整，不代表已完成真机或动力学验证', 'Checks model parsing and chain structure only, not hardware or dynamics validation'], newHardwareHint: ['硬件驱动、总线、通信协议及执行器映射请在下一步 Profile 配置中填写', 'Configure hardware, buses, communication and actuators in the next Profile setup step'], wizardBack: ['上一步', 'Back'], wizardNext: ['下一步', 'Continue'], profileLibrary: ['Profile Library', 'Profile Library'], manageProfiles: ['管理 Profile', 'Manage Profiles'], importExternalProfile: ['导入外部 Profile', 'Import external Profile'], removeFromLibrary: ['从 Library 移除', 'Remove from Library'],
    configuring: ['配置中', 'Configuring'], pendingValidation: ['待验证', 'Pending validation'], needsRevalidation: ['需要重新验证', 'Needs revalidation'], usable: ['可使用', 'Ready to use'],
    descriptionImport: ['Description 导入', 'Description import'], selectDescription: ['选择 Description 文件', 'Choose Description'], descriptionSource: ['Description 来源', 'Description source'], analyzeDescription: ['分析 Description', 'Analyze Description'],
    controlChain: ['控制链', 'Controlled chain'], hardwareMapping: ['硬件映射', 'Hardware mapping'], createProfile: ['生成 Profile', 'Create Profile'], addToLibrary: ['加入 Profile Library', 'Add to Profile Library'], notNow: ['暂不加入', 'Not now'],
    baseFrame: ['Base Frame', 'Base Frame'], toolFrame: ['Tool Frame', 'Tool Frame'], controlledJoints: ['受控关节', 'Controlled joints'], profileId: ['Profile 标识', 'Profile id'], destination: ['保存位置', 'Destination'], chooseDirectory: ['选择目录', 'Choose directory'],
    actuatorName: ['执行器名称', 'Actuator name'], motorId: ['Motor ID', 'Motor ID'], masterId: ['Master ID', 'Master ID'], motorType: ['电机型号', 'Motor type'], hardwareDriver: ['Hardware Driver', 'Hardware Driver'],
    gravityStage: ['重力补偿验证', 'Gravity verification'],
    readiness: ['机械臂准备状态', 'Manipulator readiness'], readinessDesc: ['按机械臂生命周期完成检查、校准、标定与验证', 'Complete inspection, calibration, and validation along the manipulator lifecycle'], nextStep: ['下一步', 'Next step'], confirmStage: ['确认当前阶段完成', 'Confirm current stage'],
    descriptionStage: ['描述模型', 'Description'], profileSetupStage: ['Profile 配置', 'Profile Setup'], modelCheckStage: ['模型检查', 'Model check'], bringupStage: ['真机检查', 'Hardware bring-up'], mappingStage: ['映射校准', 'Mapping calibration'], geometryStage: ['几何校准', 'Geometry calibration'], dynamicsStage: ['动力学校准', 'Dynamics calibration'], validationStage: ['验证', 'Validation'], readyStage: ['可使用', 'Ready'],
    expertTools: ['专业工具', 'Expert tools'], backHome: ['返回开始页', 'Back to start'], profileSetup: ['Profile 配置', 'Profile setup'], editProfile: ['编辑 Profile', 'Edit Profile'], saveProfile: ['保存 Profile', 'Save Profile'], profileManagedHint: ['Profile Library 只管理引用，从 Library 移除不会删除磁盘文件', 'Profile Library manages references only; removing an entry does not delete files'],
};
const paths = { workbench: '<path d="M4 5h16v14H4zM8 9h8M8 13h5M6 3v4M18 3v4"/>', robot: '<path d="M5 21v-4h14v4M8 17l-3-5 4-3 4 3-2 5M9 9l3-5 6 3-5 5M18 7l2-3M2 21h20"/>', run: '<path d="m8 5 11 7-11 7Z"/>', diagnostics: '<path d="M10 3h4v4h-4zM5 8h14v12H5zM9 12h6M9 16h3"/>', settings: '<path d="M4 7h16M4 17h16M8 4v6M16 14v6"/>', plug: '<path d="M8 3v4M16 3v4M6 7h12v4a6 6 0 0 1-12 0zM12 17v4"/>', folder: '<path d="M3 6h7l2 2h9v12H3z"/>', model: '<path d="m12 3 9 5v9l-9 5-9-5V8zM3 8l9 5 9-5M12 13v9"/>', terminal: '<path d="m5 7 5 5-5 5M13 17h6"/>', hardware: '<path d="M7 7h10v10H7zM4 9h3M4 15h3M17 9h3M17 15h3M9 4v3M15 4v3M9 17v3M15 17v3"/>', moveit: '<path d="M4 19V5h16M4 19h16M8 15l4-7 6 5M7 15h2M17 13h2"/>', refresh: '<path d="M20 7v5h-5M4 17v-5h5M5 8a8 8 0 0 1 13-3l2 2M4 17l2 2a8 8 0 0 0 13-3"/>', check: '<path d="m5 12 4 4L19 6"/>', close: '<path d="m6 6 12 12M18 6 6 18"/>', minus: '<path d="M5 12h14"/>', max: '<rect x="5" y="5" width="14" height="14" rx="1"/>', sun: '<circle cx="12" cy="12" r="4"/><path d="M12 2v2M12 20v2M2 12h2M20 12h2M5 5l1 1M18 18l1 1M5 19l1-1M18 6l1-1"/>', stop: '<rect x="6" y="6" width="12" height="12" rx="1"/>' };
const icon = name => `<svg viewBox="0 0 24 24" aria-hidden="true">${paths[name] || paths.robot}</svg>`;
const esc = value => String(value ?? '').replace(/[&<>"']/g, c => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;', "'": '&#39;' }[c]));
const $ = id => document.getElementById(id);
let state = { page: 'start', previousPage: 'start', language: 'zh-CN', theme: 'system', mode: 'terminal', source: 'builtin', entryIntent: '', library: [], libraryPath: '', activeLibraryId: '', readiness: null, descriptionAnalysis: null, descriptionStep: 1, createdProfileWizard: false, descriptionDraft: { source: '', profile: '', destination: '', base_frame: '', tool_frame: '', joint_names: [], hardware_plugin: 'serial_arm_hardware_damiao', bus: 'main_can', device: '/dev/ttyACM0', baudrate: '921600', actuators: [] }, profileEditor: null, config: { profile: '', profile_file: '', serial_port: '', baudrate: '', bus: '', resource_paths: '' }, info: null, status: {}, profiles: [], session: { state: 'idle' }, busy: false, backendError: '', inspecting: 0, modelData: null, modelLoading: false, modelPositions: [], modelSelection: '', modelLayers: { visual: true, collision: false, linkFrames: true, jointAxes: true, com: true, inertia: true, labels: true }, modelVisibility: { links: {}, linkFrames: {}, joints: {} }, workbenchTab: 'control', telemetry: null, telemetryHistory: [], admittance: null, tuneDraft: null, workbenchTargets: [], workbenchDelta: [], speedScale: 0.2, commandState: 'idle', pendingTarget: null, savePreview: null, lastAction: '', calibrationFlow: '', selectedFrame: '', telemetryReceivedAt: 0, workbenchPending: { kind: '', value: '' }, blockingBusy: false, modelCalibrationOffline: null, modelCalibrationRecords: [], modelCalibrationDirectory: '', modelCalibrationSavePreview: null, modelCalibrationExport: null, modelCalibrationCandidateSource: 'online', modelCalibrationSaved: null, modelCalibrationFrictionExport: null, modelCalibrationDisplayScale: 1, modelCalibrationShowUnchanged: false, modelCalibrationPreviewAlignment: null, calibrationEntry: 'teaching', calibrationToolRecords: {teaching:[],gain:[],gravity:[],dynamics:[]}, calibrationToolSelected: {teaching:null,gain:null,gravity:null,dynamics:null}, calibrationRuntimeCandidates: null, gainCandidateLoadPending: '', gainCandidateLoadAttempted: '', gainCandidateLoadError: '', gainMode: 'identification', advancedTarget: 'inertia', advancedAssessment: null, advancedReport: null, advancedLocked: [], advancedProgress: '', autoAssessedTaskId: '', modelCalibrationOptions: { mode: 'global', locked_links: [], pose_budget: 12, validation_fraction: 0.25, regularization: 0.01, svd_relative_threshold: 0.0001, minimum_information_score: 0.0001 } };
let term, fit, outputBuffer = '', modelView = null, modelModule = null;
const t = key => (words[key] || [key, key])[state.language === 'en' ? 1 : 0];
function commandStateText(value) {
    const labels = {
        idle: ['空闲', 'Idle'], edited: ['已编辑，等待提交', 'Edited, awaiting submission'], submitting: ['正在提交', 'Submitting'], accepted: ['命令已接受', 'Command accepted'], complete: ['目标已到达', 'Target reached'],
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
const reportedCalibrationErrors = new Set();
function showNextPriorityError() {
    const layer = $('priority-error-overlay');
    if (!layer || priorityErrorOpen || !priorityErrorQueue.length) return;
    activePriorityError = priorityErrorQueue.shift();
    priorityErrorOpen = true;
    paintPriorityError();
}
function paintPriorityError() {
    const entry = activePriorityError;
    if (!entry || !$('priority-error-overlay')) return;
    const layer = $('priority-error-overlay');
    const confirm = $('confirm-dialog');
    if (confirm?.open) confirm.close(); // A safety error outranks any confirmation dialog.
    setOperationOverlay(''); // A critical alert preempts every visual busy layer.
    const calibrationOverlay = $('model-calibration-overlay');
    if (calibrationOverlay) { calibrationOverlay.hidden = true; calibrationOverlay.setAttribute('aria-hidden', 'true'); }
    $('priority-error-title').textContent = entry.kind === 'fault' ? busyText('机械臂安全故障', 'Robot safety fault') : t('error');
    $('priority-error-detail').textContent = typeof localizedError === 'function' ? localizedError(entry.message) : entry.message;
    const raw = $('priority-error-raw'); if (raw) raw.textContent = entry.message;
    $('priority-error-guidance').textContent = entry.kind === 'fault'
        ? busyText('机器人进入故障状态 请先确认供电及机械臂安全，再使用“清除故障”或重新使能 关闭提示不会自动恢复运动 ',
            'The robot is in FAULT. Inspect its state and cause before manually clearing the fault. Dismissing does not recover motion.')
        : busyText('该任务可能未成功完成关闭提示仅确认已阅读，不会自动重试或恢复机械臂',
            'This operation may have failed. Dismissing does not retry or recover the robot.');
    // CSS z-index cannot outrank another native showModal() dialog.
    // A modal <dialog> enters Chromium's top layer and traps focus correctly.
    if (!layer.open) layer.showModal();
    layer.setAttribute('aria-hidden', 'false');
    $('priority-error-ack').focus();
}
function showPriorityError(message, kind = 'operation') {
    const text = String(message || 'Unknown error').trim();
    if (!text) return;
    // Hardware FAULT must interrupt even an already-visible noncritical alert.
    if (kind === 'fault' && activePriorityError?.kind !== 'fault') {
        if (activePriorityError) priorityErrorQueue.unshift(activePriorityError);
        activePriorityError = { message: text, kind };
        priorityErrorOpen = true;
        paintPriorityError();
        return;
    }
    if (activePriorityError?.message === text) return;
    const top = priorityErrorQueue[priorityErrorQueue.length - 1];
    if (top?.message !== text) {
        if (kind === 'fault') priorityErrorQueue.unshift({ message: text, kind });
        else priorityErrorQueue.push({ message: text, kind });
    }
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
    const isDt = f.safety_code === 'INVALID_DT';
    const value = Number.isFinite(f.value) ? `${isDt ? (f.value * 1000).toFixed(2) : Number(f.value).toFixed(4)}` : null;
    const limit = Number.isFinite(f.limit) ? `${isDt ? (f.limit * 1000).toFixed(2) : Number(f.limit).toFixed(4)}` : null;
    const unit = isDt ? ' ms' : ['JOINT_VEL_LIMIT', 'CMD_VEL_LIMIT'].includes(f.safety_code) ? ' rad/s' : '';
    const effective = snapshot?.safety_limits || {};
    const jointIndex = Number.isInteger(f.joint_index) ? f.joint_index : (snapshot?.joint_names || []).indexOf(name);
    const cmdLimit = Number(effective.max_cmd_vel?.[jointIndex]);
    const stateLimit = Number(effective.max_state_vel?.[jointIndex]);
    const effectiveBounds = jointIndex >= 0 && Number.isFinite(cmdLimit) && Number.isFinite(stateLimit)
        ? `\n${state.language === 'en' ? 'Effective command/state velocity limits' : '实际生效命令/状态速度上限'}：${cmdLimit.toFixed(4)} / ${stateLimit.toFixed(4)} rad/s` : '';
    return [reason, name, value === null ? '' : `value=${value}${unit}`, limit === null ? '' : `limit=${limit}${unit}`, effectiveBounds].filter(Boolean).join(' / ');
}
function checkRuntimeAlerts(snapshot) {
    if (snapshot?.robot_state === 'FAULT') {
        const key = JSON.stringify(snapshot.fault || snapshot.last_fault || 'FAULT');
        if (key !== lastRuntimeFaultKey) { lastRuntimeFaultKey = key; showPriorityError(describeRuntimeFault(snapshot), 'fault'); }
    } else lastRuntimeFaultKey = '';
    const cal = snapshot?.model_calibration;
    if (cal?.phase === 'failed' && cal.error) {
        const key = `${cal.task_id || 'unknown'}:${cal.error}`;
        if (!reportedCalibrationErrors.has(key)) { reportedCalibrationErrors.add(key); showPriorityError(cal.error); }
    }
    const singleCal = snapshot?.calibration;
    if (singleCal?.phase === 'failed' && singleCal.error) {
        const key = `single:${singleCal.kind}:${singleCal.error}`;
        if (!reportedCalibrationErrors.has(key)) { reportedCalibrationErrors.add(key); showPriorityError(singleCal.error); }
    }
}
const button = (id, key, ico, cls = '') => `<button id="${id}" class="button ${cls}">${ico ? icon(ico) : ''}${t(key)}</button>`;
const card = (title, sub, ico, body) => `<section class="card"><div class="card-head">${icon(ico)}<div><h3>${t(title)}</h3><p>${t(sub)}</p></div></div><div class="card-body">${body}</div></section>`;
function pageHead(title, desc, action = '') { return `<header class="page-head"><div><div class="eyebrow">SERIALARM · WORKSPACE</div><h2>${t(title)}</h2><p>${t(desc)}</p></div>${action}</header>`; }
function configField(key, label, placeholder = '') { return `<div class="field"><label for="${key}">${t(label)}</label><input id="${key}" value="${esc(state.config[key])}" placeholder="${esc(placeholder)}" autocomplete="off"></div>`; }
function active() { return ['running', 'stopping'].includes(state.session.state); }
async function save() { if (api) await api.savePrefs({ ...state.config, mode: state.mode, theme: state.theme, language: state.language }); }
function themeIsLight(theme = state.theme) { return theme === 'light' || (theme === 'system' && matchMedia('(prefers-color-scheme: light)').matches); }
function applyTheme(theme = state.theme) { document.documentElement.className = themeIsLight(theme) ? 'light' : 'dark'; }
async function switchTheme(nextTheme, event = null, rerender = false) {
    if (!['system', 'dark', 'light'].includes(nextTheme) || nextTheme === state.theme) return;
    const previousLight = themeIsLight(state.theme), nextLight = themeIsLight(nextTheme);
    const reduceMotion = matchMedia('(prefers-reduced-motion: reduce)').matches;
    const canTransition = typeof document.startViewTransition === 'function' && !reduceMotion && previousLight !== nextLight;
    if (!canTransition) { state.theme = nextTheme; applyTheme(); if (rerender) render(); await save(); return; }
    const source = event?.currentTarget || event?.target;
    const rect = source?.getBoundingClientRect?.();
    const x = Number.isFinite(event?.clientX) && event.clientX > 0 ? event.clientX : (rect ? rect.left + rect.width / 2 : innerWidth / 2);
    const y = Number.isFinite(event?.clientY) && event.clientY > 0 ? event.clientY : (rect ? rect.top + rect.height / 2 : innerHeight / 2);
    const radius = Math.hypot(Math.max(x, innerWidth - x), Math.max(y, innerHeight - y));
    const root = document.documentElement;
    root.style.setProperty('--theme-ripple-x', `${x}px`);
    root.style.setProperty('--theme-ripple-y', `${y}px`);
    root.style.setProperty('--theme-ripple-radius', `${Math.ceil(radius)}px`);
    root.style.setProperty('--theme-ripple-overshoot', `${Math.ceil(radius * 1.018)}px`);
    const transition = document.startViewTransition(() => { state.theme = nextTheme; applyTheme(); if (rerender) render(); });
    try { await transition.finished; } finally { root.style.removeProperty('--theme-ripple-x'); root.style.removeProperty('--theme-ripple-y'); root.style.removeProperty('--theme-ripple-radius'); root.style.removeProperty('--theme-ripple-overshoot'); }
    await save();
}
function shell() {
    // A new Shell intentionally preserves the pending fault alert, if any.
    const onboarding = ['start', 'library', 'description'].includes(state.page) || (state.page === 'settings' && !state.config.profile);
    const quickTheme = onboarding ? '' : `<button id="theme-quick" class="title-button" aria-label="${t('appearance')}">${icon('sun')}</button>`;
    const titlebar = `<header class="titlebar"><div class="brand-mini">${icon('robot')}SerialArm Launcher</div><div class="title-center">${onboarding ? 'SerialArm' : esc(state.config.profile || t('workspace'))}</div><div class="title-actions"><button id="settings-quick" class="title-button" aria-label="${t('settings')}">${icon('settings')}</button>${quickTheme}<button id="win-min" class="title-button" aria-label="Minimize">${icon('minus')}</button><button id="win-max" class="title-button" aria-label="Maximize">${icon('max')}</button><button id="win-close" class="title-button close" aria-label="Close">${icon('close')}</button></div></header>`;
    const runtimePanel = `<section id="runtime-panel" class="card terminal-card is-hidden"><div class="terminal-head">${icon('terminal')}<b>${t('logs')}</b><span class="terminal-state" id="terminal-state"></span>${button('copy-log', 'copyAll', '')}${button('paste-log', 'paste', '')}${button('clear-log', 'clear', '')}${button('stop-runtime', 'stop', 'stop')}${button('force-runtime', 'force', '', 'danger')}</div><div id="terminal"></div><div class="terminal-note">${t('terminalHint')}</div></section>`;
    const overlays = `<div id="toasts" aria-live="polite"></div><dialog id="confirm-dialog"></dialog><dialog id="priority-error-overlay" class="priority-error-overlay" role="alertdialog" aria-modal="true" aria-labelledby="priority-error-title" aria-hidden="true"><div class="priority-error-card"><div class="priority-error-kicker">SAFETY / ERROR</div><h3 id="priority-error-title"></h3><pre id="priority-error-detail"></pre><details class="error-technical"><summary>${state.language === 'en' ? 'Raw diagnostic details' : '展开原始技术诊断'}</summary><pre id="priority-error-raw"></pre></details><p id="priority-error-guidance"></p><div class="actions"><button id="priority-error-ack" class="button danger">${state.language === 'en' ? 'Understood (no recovery)' : '我已知晓（不恢复运行）'}</button></div></div></dialog><div id="operation-overlay" class="operation-overlay" hidden aria-hidden="true"><div class="operation-progress" role="status" aria-live="assertive" aria-busy="true"><span class="operation-spinner" aria-hidden="true"></span><div><div class="operation-kicker">SERIALARM</div><div id="operation-message" class="operation-message">${t('processing')}</div></div></div></div><div id="model-calibration-overlay" class="model-calibration-overlay" hidden aria-hidden="true"><div class="model-calibration-task-window" role="dialog" aria-modal="true" aria-labelledby="model-calibration-task-title"><div class="task-window-head"><span class="operation-spinner" aria-hidden="true"></span><div><div class="operation-kicker">${t('modelCalibration')}</div><div id="model-calibration-task-title" class="operation-message">${t('modelCalibration')}</div></div></div><div class="task-progress-track"><span id="model-calibration-task-progress-bar"></span></div><div class="task-live-grid"><span id="model-calibration-task-progress">0%</span><span id="model-calibration-task-samples">${t('validStaticSamples')} —</span><span id="model-calibration-task-feedback">${t('feedbackAge')} —</span><span id="model-calibration-task-id">${t('taskIdentifier')} —</span></div><div class="task-window-actions"><button id="model-calibration-task-stop-teach" class="button primary">${t('stopTeaching')}</button><button id="model-calibration-task-pause" class="button">${t('pauseTask')}</button><button id="model-calibration-task-resume" class="button">${t('resumeTask')}</button><button id="model-calibration-task-hold" class="button">${t('holdRobot')}</button><button id="model-calibration-task-cancel" class="button danger">${t('cancelTask')}</button><button id="model-calibration-task-disable" class="button danger">${t('deactivateRobot')}</button></div><div id="model-calibration-task-error" class="workspace-error"></div></div></div>`;
    if (onboarding) {
        $('app').innerHTML = `<div class="shell onboarding-shell">${titlebar}<div class="onboarding-body"><main class="onboarding-content"><div class="page" id="page"></div>${runtimePanel}</main></div></div>${overlays}`;
    } else {
        const lifecycle = [['lifecycle', 'readiness', 'check'], ['profile_setup', 'profileSetup', 'robot']];
        const expert = [['model', 'model', 'model'], ['workbench', 'workbenchNav', 'workbench'], ['run', 'run', 'run'], ['diagnostics', 'diagnostics', 'diagnostics'], ['robot', 'connection', 'plug']];
        $('app').innerHTML = `<div class="shell">${titlebar}<div class="body"><aside class="sidebar"><div class="hero compact"><div class="eyebrow">SERIALARM</div><h1>${esc(state.config.profile || '—')}</h1><p>${readinessStateText(state.readiness?.state)}</p></div><div class="nav-caption">${state.language === 'en' ? 'ROBOT LIFECYCLE' : '机械臂生命周期'}</div><nav>${lifecycle.map(([name, key, ico]) => `<button class="nav-item ${state.page === name ? 'active' : ''}" data-page="${name}">${icon(ico)}<span>${t(key)}</span></button>`).join('')}</nav><div class="nav-caption">${t('expertTools')}</div><nav>${expert.map(([name, key, ico]) => `<button class="nav-item ${state.page === name ? 'active' : ''}" data-page="${name}">${icon(ico)}<span>${t(key)}</span></button>`).join('')}</nav><div class="sidebar-spacer"></div><button id="back-home" class="nav-item subtle">${icon('folder')}<span>${t('backHome')}</span></button><div class="runtime-card" id="sidebar-runtime"></div></aside><section class="content-wrap"><div class="top-status"><span class="breadcrumb" id="breadcrumb"></span><span class="pill" id="env-pill"></span><span class="pill" id="session-pill"></span></div><main class="content"><div class="page" id="page"></div>${runtimePanel}</main></section></div></div>${overlays}`;
    }
    document.querySelectorAll('[data-page]').forEach(b => b.onclick = () => navigate(b.dataset.page));
    ['min', 'max', 'close'].forEach((n, i) => { const node = $('win-' + n); if (node) node.onclick = () => api.window(['minimize', 'maximize', 'close'][i]); });
    if ($('settings-quick')) $('settings-quick').onclick = async () => { await pageExit(); if (state.page === 'settings') { state.page = state.previousPage && state.previousPage !== 'settings' ? state.previousPage : 'start'; renderedPage = ''; shell(); return; } state.previousPage = state.page; state.page = 'settings'; renderedPage = ''; shell(); };
    if ($('back-home')) $('back-home').onclick = () => { resetToStart(); };
    if ($('theme-quick')) $('theme-quick').onclick = event => switchTheme(document.documentElement.classList.contains('light') ? 'dark' : 'light', event);
    if ($('copy-log')) $('copy-log').onclick = () => operation(() => copyTerminal(true));
    if ($('paste-log')) $('paste-log').onclick = () => operation(pasteTerminal);
    if ($('clear-log')) $('clear-log').onclick = () => { outputBuffer = ''; term?.clear(); };
    if ($('stop-runtime')) $('stop-runtime').onclick = () => blockingOperation(busyText('正在停止运行...', 'Stopping runtime...'), () => api.request('stop'));
    if ($('force-runtime')) $('force-runtime').onclick = () => forceConfirm();
    if ($('model-calibration-task-stop-teach')) $('model-calibration-task-stop-teach').onclick = () => modelCalibrationQuick('model_calibration_teach_stop');
    if ($('model-calibration-task-pause')) $('model-calibration-task-pause').onclick = () => modelCalibrationQuick('model_calibration_pause');
    if ($('model-calibration-task-resume')) $('model-calibration-task-resume').onclick = () => resumeModelCalibrationWithConfirmation();
    if ($('model-calibration-task-hold')) $('model-calibration-task-hold').onclick = () => modelCalibrationQuick('hold');
    if ($('model-calibration-task-cancel')) $('model-calibration-task-cancel').onclick = () => modelCalibrationQuick('model_calibration_cancel');
    if ($('model-calibration-task-disable')) $('model-calibration-task-disable').onclick = () => modelCalibrationQuick('deactivate');
    mountTerminal(); render();
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
    const sidebar = $('sidebar-runtime'); if (sidebar) sidebar.innerHTML = `<span class="dot ${ready ? 'ready' : ''}"></span>${t(ready ? 'connected' : 'notInstalled')}<small>${esc(state.info?.profile || state.config.profile || '—')}</small><small>${state.backendError ? esc(localizedError(state.backendError)) : 'ROS2 ' + esc(state.status.ros_distro || '—')}</small>`;
    const env = $('env-pill'); if (env) env.innerHTML = `<span class="dot ${ready ? 'ready' : ''}"></span>Core ${t(ready ? 'ready' : 'unavailable')}`;
    const session = $('session-pill'); if (session) session.innerHTML = `<span class="dot ${active() ? 'ready' : ''}"></span>${t(state.session.state || 'idle')}`;
    const breadcrumb = $('breadcrumb'); if (breadcrumb) breadcrumb.textContent = t('workspace') + ' / ' + (words[state.page] ? t(state.page) : state.page);
    const terminalState = $('terminal-state'); if (terminalState) terminalState.textContent = active() ? `${t(state.session.state)} · PID ${state.session.pid}` : state.session.state === 'exited' ? `${t('exited')} · ${state.session.exit_code}` : t('noSession');
    const stop = $('stop-runtime'); if (stop) stop.disabled = !active() || state.session.state === 'stopping'; const force = $('force-runtime'); if (force) force.classList.toggle('is-hidden', state.session.state !== 'stopping');
}

async function resetToStart() {
    if (state.page === 'profile_setup' && state.profileDirty && !confirm(state.language === 'en' ? 'Discard unsaved parameters' : '有未保存的参数，确认放弃修改后返回首页')) return;
    if (active()) { toast(state.language === 'en' ? 'Stop the active session before returning to the start page' : '请先停止当前运行会话', true); return; }
    await pageExit();
    state.page = 'start'; state.entryIntent = ''; state.descriptionStep = 1; state.createdProfileWizard = false; state.activeLibraryId = ''; state.readiness = null; state.info = null; state.modelData = null; state.profileEditor = null; state.config = { profile: '', profile_file: '', serial_port: '', baudrate: '', bus: '', resource_paths: '' }; renderedPage = ''; shell();
}
function startPage() {
    const option = (intent, title, desc, ico) => `<button class="start-choice" data-start-intent="${intent}"><span class="start-choice-icon">${icon(ico)}</span><span><b>${t(title)}</b><small>${t(desc)}</small></span><span class="choice-arrow">›</span></button>`;
    return `<section class="start-hero"><div class="eyebrow">SERIALARM · WORKSPACE</div><h1>${t('startQuestion')}</h1><p>${t('startDesc')}</p></section><div class="start-choice-grid">${option('description','newDescription','newDescriptionDesc','model')}${option('continue','continueProfile','continueProfileDesc','robot')}${option('use','useProfile','useProfileDesc','run')}</div>`;
}
function libraryStateClass(value) { return value === 'ready' ? 'ready' : value === 'needs_revalidation' ? 'warning' : value === 'configuring' ? 'muted' : 'pending'; }
function libraryPage() {
    const intent = state.entryIntent || 'continue';
    // The two entrances are filtered views of one Library, not duplicate stores
    const items = state.library.filter(item => intent === 'use' ? item.readiness?.state === 'ready' : item.readiness?.state !== 'ready');
    const cards = items.map(item => `<article class="library-card"><div class="library-card-main"><div><span class="readiness-chip ${libraryStateClass(item.readiness?.state)}">${readinessStateText(item.readiness?.state)}</span><h3>${esc(item.profile)}</h3><p>${item.source === 'builtin' ? (state.language === 'en' ? 'Built-in Profile' : '内置 Profile') : esc(item.profile_file)}</p><small>${state.language === 'en' ? 'Next: ' : '下一步'}${readinessStageText(item.readiness?.next_stage)}</small></div><div class="library-actions"><button class="button primary" data-library-open="${esc(item.id)}">${intent === 'use' ? t('useProfile') : t('editProfile')}</button><button class="button" data-library-remove="${esc(item.id)}">${t('removeFromLibrary')}</button></div></div></article>`).join('');
    const empty = `<div class="empty-state">${intent === 'use' ? (state.language === 'en' ? 'No ready Profiles in the Library' : '暂无可使用的 Profile') : (state.language === 'en' ? 'No Profiles waiting for configuration' : '暂无需要继续配置的 Profile')}</div>`;
    const importTools = intent === 'use' ? '' : `<div class="library-toolbar"><button id="library-import-dir" class="button primary">${t('importExternalProfile')}</button><button id="library-import-file" class="button">${state.language === 'en' ? 'Choose robot_profiles.yaml' : '选择 robot_profiles.yaml'}</button><span class="hint">${esc(state.libraryPath || '')}</span></div>`;
    return pageHead(intent === 'use' ? 'readyLibrary' : 'continueLibrary', intent === 'use' ? 'useProfileDesc' : 'continueProfileDesc', `<button id="library-home" class="button">${t('backHome')}</button>`) + importTools + `<div class="library-list">${cards || empty}</div>`;
}
function readinessStageText(value) { const keys = { description: 'descriptionStage', profile_setup: 'profileSetupStage', model_check: 'modelCheckStage', bringup: 'bringupStage', mapping_calibration: 'mappingStage', geometry_calibration: 'geometryStage', gravity_verification: 'gravityStage', dynamics_calibration: 'dynamicsStage', validation: 'validationStage', ready: 'readyStage' }; return t(keys[value] || 'unknown'); }
function computeChain(base, tool, joints) { const byChild = new Map(joints.map(j => [j.child, j])); const reverse = []; let node = tool; const seen = new Set(); while (node && node !== base && !seen.has(node)) { seen.add(node); const j = byChild.get(node); if (!j) return []; reverse.push(j); node = j.parent; } if (node !== base) return []; return reverse.reverse(); }
function syncDescriptionChain() {
    const a = state.descriptionAnalysis, d = state.descriptionDraft;
    if (!a) return;
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
    syncDescriptionChain();
    const movable = a.joints.filter(j => j.type !== 'fixed' && !j.mimic);
    const basicOk = !!(a.links.length && a.roots?.length && movable.length);
    const linkOptions = a.links.map(x => `<option value="${esc(x)}" ${x === d.base_frame ? 'selected' : ''}>${esc(x)}</option>`).join('');
    const toolOptions = a.links.map(x => `<option value="${esc(x)}" ${x === d.tool_frame ? 'selected' : ''}>${esc(x)}</option>`).join('');
    const chain = computeChain(d.base_frame, d.tool_frame, a.joints);
    const jointSummary = chain.map(j => `<span class="chain-joint ${j.type === 'fixed' ? 'fixed' : ''}">${esc(j.name)}<small>${esc(j.type)}</small></span>`).join('') || `<span class="warning-text">${state.language === 'en' ? 'Base and Tool are not on one connected chain' : 'Base 与 Tool 不构成连续控制链'}</span>`;
    const stats = `<div class="description-facts"><span>${a.links.length} Links</span><span>${a.joints.length} Joints</span><span>${movable.length} ${state.language === 'en' ? 'Movable' : '可动关节'}</span></div>`;
    const preview = `<section class="card description-preview"><div class="card-head">${icon('model')}<div><h3>${esc(a.package_name)}</h3><p>${esc(a.urdf)}</p></div></div><div id="description-viewport"></div>${stats}</section>`;
    let form = '';
    if (step === 1) {
        form = `<h3>${t('newModelCheck')}</h3><p class="parameter-guidance">${t('newModelHint')}</p><div class="wizard-check ${basicOk ? 'passed' : 'warning'}"><span>${basicOk ? '✓' : '!'}</span><div><b>${basicOk ? (state.language === 'en' ? 'Basic model check passed' : '模型基础检查通过') : (state.language === 'en' ? 'Missing a movable chain' : '模型缺少可动控制链')}</b><small>${state.language === 'en' ? 'URDF parsed and model topology loaded' : 'URDF 已解析，模型结构已读取'}</small></div></div><dl class="kv"><dt>Base</dt><dd>${esc(a.roots?.[0] || '—')}</dd><dt>Tool</dt><dd>${esc(d.tool_frame || '—')}</dd><dt>${t('controlledJoints')}</dt><dd>${movable.length}</dd></dl><div class="wizard-actions"><button id="description-next-chain" class="button primary" ${basicOk ? '' : 'disabled'}>${t('wizardNext')} →</button></div>`;
    } else if (step === 2) {
        form = `<h3>${t('controlChain')}</h3><p class="parameter-guidance">${state.language === 'en' ? 'Select Base and Tool to derive the controlled joint sequence' : '选择 Base 与 Tool，自动识别沿途受控关节及其顺序'}</p><div class="row"><div class="field"><label>${t('baseFrame')}</label><select id="draft-base">${linkOptions}</select></div><div class="field"><label>${t('toolFrame')}</label><select id="draft-tool">${toolOptions}</select></div></div><div class="chain-preview">${jointSummary}</div><div class="field"><label>${t('controlledJoints')}</label><div class="chip-row">${d.joint_names.map(x => `<span class="chip">${esc(x)}</span>`).join('') || '—'}</div></div><div class="wizard-actions"><button id="description-back" class="button">← ${t('wizardBack')}</button><button id="description-next-profile" class="button primary" ${d.joint_names.length ? '' : 'disabled'}>${t('wizardNext')} →</button></div>`;
    } else {
        form = `<h3>${t('createProfile')}</h3><p class="parameter-guidance">${t('newHardwareHint')}</p><div class="wizard-summary"><div><small>Base → Tool</small><b>${esc(d.base_frame)} → ${esc(d.tool_frame)}</b></div><div><small>${t('controlledJoints')}</small><b>${d.joint_names.map(esc).join(' · ')}</b></div></div><div class="field"><label>${t('profileId')}</label><input id="draft-profile" value="${esc(d.profile)}"></div><div class="field"><label>${t('destination')}</label><div class="input-row"><input id="draft-destination" value="${esc(d.destination)}"><button id="draft-destination-browse" class="button">${t('chooseDirectory')}</button></div></div><div class="wizard-actions"><button id="description-back" class="button">← ${t('wizardBack')}</button><button id="draft-create" class="button primary">${t('createProfile')} →</button></div>`;
    }
    return pageTitle + descriptionStepper(step) + `<div class="description-layout">${preview}<section class="card setup-form"><div class="card-body">${form}</div></section></div>`;
}
const readinessReviews = {
    model_check: { title: '模型检查', keys: [['chain', '受控链、关节与坐标系一致'], ['limits', '运动范围和安全限位已核对'], ['inertial', '惯性参数与模型结构已检查']] },
    bringup: { title: '真机连接检查', keys: [['connection', '已在 INACTIVE 状态检查电机通信与所有关节反馈']] },
    startup_hold: { title: '启动保持安全验收', keys: [['hold_gains', '全部关节保持增益与上限已核对'], ['supported', '已采取机械支撑与清空运动空间等措施'], ['emergency_stop', '硬件急停和失能途径已确认']] },
    mapping_calibration: { title: '映射校准', keys: [['joint_ids', '每个关节与电机 ID 已实测对应'], ['directions', '每个关节运动方向已实测正确'], ['offsets', '零位与偏移量已实测确认']] },
    geometry_calibration: { title: '几何校准', keys: [['frames', 'Base 与 Tool 坐标系已实测核对'], ['zero_pose', '零位与机构位置一致'], ['tool_pose', 'TCP 与工具姿态已核对']] },
    validation: { title: '综合验收', keys: [['tracking', '已检查获准控制模式的真实跟踪效果'], ['stopping', '已检查正常停放与失能过程'], ['faults', '已核对故障与紧急处理结果']] },
};
function readinessResultLabel(value) {
    const labels = { verified: ['已验证', 'Verified'], pending: ['待验证', 'Pending'], blocked: ['前置条件未满足', 'Blocked'], stale: ['相关参数已变化', 'Stale'], failed: ['配置错误', 'Failed'], unconfigured: ['参数尚未满足要求', 'Unconfigured'] };
    return (labels[value] || labels.pending)[state.language === 'en' ? 1 : 0];
}
function lifecyclePage() {
    const r = state.readiness || {}; const checks = r.checks || {}, modes = r.modes || {};
    const stages = [['description', 'descriptionStage', null], ['profile_setup', 'profileSetupStage', 'profile_setup'],
        ['model_check', 'modelCheckStage', 'model'], ['bringup', 'bringupStage', 'workbench'],
        ['mapping_calibration', 'mappingStage', 'profile_setup'], ['geometry_calibration', 'geometryStage', 'model'],
        ['gravity_verification', 'gravityStage', 'workbench'], ['dynamics_calibration', 'dynamicsStage', 'workbench'],
        ['validation', 'validationStage', 'workbench']];
    const rows = stages.map(([id, key, page]) => {
        const value = checks[id] || 'pending';
        const canReview = Boolean(readinessReviews[id]);
        const evidence = r.evidence?.[id];
        const action = canReview && value !== 'verified' ? `<button class="button ${id === r.next_stage ? 'primary' : ''}" data-review-stage="${id}">${state.language === 'en' ? 'Review evidence' : '记录验收'}</button>` : '';
        return `<article class="lifecycle-step ${esc(value)}"><span class="stage-dot"></span><div><b>${t(key)}</b><small>${readinessResultLabel(value)}</small>${evidence?.note ? `<small>${esc(evidence.note)}</small>` : ''}</div><div class="lifecycle-actions">${page ? `<button class="button" data-stage-open="${page}" data-stage="${id}">${state.language === 'en' ? 'Open' : '进入'}</button>` : ''}${action}</div></article>`;
    }).join('');
    const modeRows = Object.entries(modes).map(([mode,value]) => `<div class="mode-admission-row"><code>${esc(mode)}</code><span class="readiness-chip ${value === 'verified' ? 'ready' : value === 'stale' ? 'warning' : 'pending'}">${readinessResultLabel(value)}</span></div>`).join('');
    const startup = modes.RIGID_HOLD !== 'verified' ? `<button class="button" data-review-stage="startup_hold">${state.language === 'en' ? 'Review startup hold' : '启动保持验收'}</button>` : '';
    const admission = `<section class="card mode-admission"><div class="card-head"><div><h3>${state.language === 'en' ? 'Control admission' : '控制能力准入'}</h3><p>${state.language === 'en' ? 'Numeric configuration is not permission to activate' : '数值配置合法不等于允许使能与运动，控制权限由后端独立验证'}</p></div></div><div class="card-body">${modeRows}<div class="workspace-actions">${startup}${checks.gravity_verification === 'verified' ? `<button class="button" id="approve-gain-mode">${state.language === 'en' ? 'Import validated gain evidence' : '导入增益验收记录'}</button>` : ''}</div></div></section>`;
    return pageHead('readiness', 'readinessDesc', `<span class="readiness-chip ${libraryStateClass(r.state)}">${readinessStateText(r.state)}</span>`) +
        `<div class="readiness-summary card"><div><div class="eyebrow">${esc(state.config.profile || '—')}</div><h3>${r.state === 'ready' ? t('usable') : t('nextStep') + ' · ' + readinessStageText(r.next_stage)}</h3><p>${state.language === 'en' ? 'Bundled profiles still require local hardware evidence and per-mode approval' : '内置配置仅代表来源可信，不能代替本机实测与独立模式增益验收'}</p>${r.reason ? `<p class="workspace-error">${esc(r.reason)}</p>` : ''}</div>${r.state === 'ready' ? `<button id="ready-control" class="button primary">${t('useProfile')}</button>` : ''}</div><div class="lifecycle-list">${rows}</div>${admission}`;
}
function showReadinessReview(stage) {
    const review = readinessReviews[stage]; if (!review) return;
    const dialog = $('confirm-dialog');
    const live = stage !== 'model_check';
    dialog.innerHTML = `<h3>${esc(review.title)}</h3><p class="hint">${live ? '需要真实控制工作台反馈与具体观察记录，必须先完成对应安全前提' : '请核对模型并记录实际检查依据'}</p><div class="readiness-review-items">${review.keys.map(([name,label]) => `<label><input type="checkbox" data-review-key="${name}">${esc(label)}</label>`).join('')}</div><label class="field"><span>检查记录</span><textarea id="readiness-review-note" rows="3" maxlength="600" placeholder="记录检查过程与观察结果，至少 12 字"></textarea></label><div class="actions"><button class="button" id="readiness-review-cancel">${t('cancel')}</button><button class="button primary" id="readiness-review-submit">保存验收记录</button></div>`;
    dialog.showModal();
    $('readiness-review-cancel').onclick = () => dialog.close();
    $('readiness-review-submit').onclick = () => {
        const answers = {note: $('readiness-review-note').value};
        dialog.querySelectorAll('[data-review-key]').forEach(node => { answers[node.dataset.reviewKey] = node.checked; });
        dialog.close();
        blockingOperation('正在检查验收依据...', async () => {
            state.readiness = await api.request('readiness_record', {config: state.config, stage, answers});
            await refreshLibrary(); render();
        });
    };
}
function showGainAdmission() {
    const dialog = $('confirm-dialog');
    dialog.innerHTML = `<h3>增益验收记录</h3><p class="hint">仅接受已经通过独立质量验证且与当前 Core 增益完全一致的候选</p><label class="field"><span>控制模式</span><select id="gain-approval-mode"><option value="RIGID_TRACKING">RIGID_TRACKING</option><option value="COMPLIANT_TRACKING">COMPLIANT_TRACKING</option></select></label><label class="field"><span>gain-candidate.json 路径</span><input id="gain-approval-path" placeholder="/path/to/gain-candidate.json"></label><div class="actions"><button class="button" id="gain-approval-cancel">${t('cancel')}</button><button class="button primary" id="gain-approval-submit">校验并导入</button></div>`;
    dialog.showModal(); $('gain-approval-cancel').onclick = () => dialog.close();
    $('gain-approval-submit').onclick = () => { const mode = $('gain-approval-mode').value, candidate_path = $('gain-approval-path').value; dialog.close(); blockingOperation('正在核验增益候选...', async () => {state.readiness = await api.request('readiness_approve_mode',{config:state.config,mode,candidate_path});await refreshLibrary();render();}); };
}
// One authoritative draft is shared by creation and continued configuration
const profileGroups = [
    ['model', '模型与重力', 'model'],
    ['mapping', '关节映射', 'calibration'],
    ['runtime', '运行参数', 'control.runtime'],
    ['gains', '控制增益', 'control.controller'],
    ['safety', '安全策略', 'safety_policy'],
    ['shutdown', '安全停放', 'shutdown'],
    ['admittance', '导纳能力', 'capability.admittance'],
    ['hardware', '硬件与总线', 'hardware'],
];
const profileEnums = {
    'core.control.runtime.model_feedforward_mode': ['NONE', 'GRAVITY', 'FULL_INVERSE_DYNAMICS'],
    'core.control.runtime.tracking_impedance_mode': ['RIGID_TRACKING', 'COMPLIANT_TRACKING'],
    'core.safety_policy.fault_recovery.default_mode': ['rigid_hold'],
    'core.capability.admittance.observer.mode': ['MOMENTUM', 'FULL_ID'],
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
    write_enabled: '允许硬件命令写入', model_feedforward_mode: '模型前馈', tracking_impedance_mode: '默认跟踪阻抗',
    allow_full_cmd: '允许完整关节命令', rigid_hold: '刚性保持', rigid_tracking: '刚性跟踪',
    compliant_hold: '柔性保持', compliant_drag: '柔性拖拽', compliant_tracking: '柔性跟踪',
    identification_tracking: '辨识跟踪', kp: '位置刚度 Kp', kd: '速度阻尼 Kd',
    position_margin: '关节位置余量 rad', cmd_vel_scale: '命令速度比例', state_vel_scale: '状态速度比例',
    max_acc: '关节最大加速度 rad/s²', max_effort_override: '最大力矩覆盖 Nm',
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
    enabled: '启用', joint_enabled: '参与导纳的关节', observer: '力矩观测器',
    mode: '模式', momentum_gain: '动量观测器增益', calibration: '标定参数',
    torque_bias: '残差偏置 Nm', torque_threshold: '残差死区 Nm', friction: '摩擦补偿',
    velocity_transition: '摩擦速度过渡 rad/s', positive_coulomb: '正向库仑项',
    positive_viscous: '正向黏性项', negative_coulomb: '反向库仑项', negative_viscous: '反向黏性项',
    controller: '控制参数', mass: '虚拟质量 M', damping: '虚拟阻尼 D', stiffness: '虚拟刚度 K',
    max_delta_q: '最大位置退让 rad', max_delta_q_dot: '最大速度退让 rad/s',
    buses: '总线配置', damiao: '达妙执行器', bus: '使用总线', type: '总线类型', backend: '通信后端',
    device: '设备路径', baudrate: '波特率', refresh_state_in_read: '读取时刷新状态',
    feedback_timeout_s: '电机反馈超时 s', activation_retries: '使能重试次数',
    startup_read_cycles: '启动读取周期数', stop_kp: '停止 Kp', stop_kd: '停止 Kd',
    stop_cycles: '停止周期数', actuators: '执行器映射', name: '名称',
    motor_id: '电机 ID', master_id: '主机 ID', motor_type: '电机型号', plugin: '硬件驱动',
};
const paramLabel = key => profileLabels[key] || key;
function profileGet(draft, path) { return path.reduce((item, key) => item?.[key], draft); }
function profileField(path, value) {
    const id = path.join('.');
    const label = paramLabel(path.at(-1));
    const readonly = id === 'core.model.joint_names' || id === 'core.safety_policy.fault_recovery.gravity_model_validated' || (id === 'core.capability.admittance.enabled' && !value);
    const opts = profileEnums[id] ? [...new Set([...profileEnums[id], String(value)])] : null;
    const hardwareChoice = typeof value === 'string' ? hardwareChoiceField(path, value, readonly) : null;
    const data = `data-profile-field="${esc(JSON.stringify(path))}"`;
    const units = id.includes('gravity_model_validated') ? (state.language === 'en' ? 'Only validated by the later hardware workflow' : '须经后续真机验证后再确认') :
        id.includes('joint_names') ? (state.language === 'en' ? 'Determined when creating the control chain' : '关节顺序由新建时选择的控制链决定') :
        id === 'core.capability.admittance.enabled' ? (state.language === 'en' ? 'Enable after verification' : '仅在完成后续验证后启用') : '';
    let input;
    if (hardwareChoice) input = hardwareChoice;
    else if (opts) input = `<select ${data} ${readonly ? 'disabled' : ''}>${opts.map(o => `<option value="${esc(o)}" ${value === o ? 'selected' : ''}>${esc(o || '未配置')}</option>`).join('')}</select>`;
    else if (typeof value === 'boolean') input = `<div class="parameter-switch" data-switch-state="${value ? 'on' : 'off'}"><span class="switch-value switch-on">ON</span><input type="checkbox" role="switch" aria-label="${esc(label)}" aria-checked="${value}" ${data} ${value ? 'checked' : ''} ${readonly ? 'disabled' : ''}><span class="switch-value switch-off">OFF</span></div>`;
    else if (typeof value === 'number') input = `<input type="number" step="any" ${data} value="${esc(value)}" ${readonly ? 'disabled' : ''}>`;
    else if (typeof value === 'string') input = `<input type="text" ${data} value="${esc(value)}" ${readonly ? 'disabled' : ''}>`;
    else if (Array.isArray(value)) input = `<input type="text" ${data} value="${esc(value.join(', '))}" ${readonly ? 'disabled' : ''} title="${state.language === 'en' ? 'Comma-separated values' : '用英文逗号分隔多个值'}">`;
    else return '';
    const err = state.profileError?.startsWith(id) ? state.profileError : '';
    return `<label class="parameter-field ${err ? 'invalid' : ''}"><span>${esc(label)}</span>${input}<small>${esc(id)}${units ? ' · ' + esc(units) : ''}</small>${err ? `<em>${esc(err)}</em>` : ''}</label>`;
}
function profileTree(value, path, depth = 0) {
    if (value && !Array.isArray(value) && typeof value === 'object') {
        const children = Object.entries(value).map(([key, child]) => profileTree(child, [...path,key], depth+1)).join('');
        if (depth === 0) return `<div class="parameter-fields">${children}</div>`;
        const leaves = Object.keys(value).length;
        const isHardware = path[0] === 'hardware';
        if (!isHardware && depth > 1 && leaves <= 12 && Object.values(value).every(item => item === null || typeof item !== 'object' || Array.isArray(item)))
            return `<fieldset class="parameter-subgroup"><legend>${esc(paramLabel(path.at(-1)))}</legend><div class="parameter-fields">${children}</div></fieldset>`;
        const expanded = isHardware ? depth <= 1 : true;
        return `<details class="parameter-subgroup ${isHardware ? 'hardware-subgroup' : ''}" ${expanded ? 'open' : ''}><summary>${esc(paramLabel(path.at(-1)))}${isHardware ? `<small>${leaves} ${state.language === 'en' ? 'items' : '项'}</small>` : ''}</summary><div class="parameter-fields">${children}</div></details>`;
    }
    return profileField(path, value);
}
function profileHardwareDefaults(names) {
    const a = Object.fromEntries(names.map((j,i) => [j,{name:`actuator${i+1}`,motor_id:i+1,master_id:0,motor_type:''}]));
    return {buses:{main_can:{type:'can',backend:'damiao_usb2can',device:'/dev/ttyACM0',baudrate:921600}},damiao:{bus:'main_can',refresh_state_in_read:false,feedback_timeout_s:0.05,activation_retries:3,startup_read_cycles:5,stop_kp:10,stop_kd:0.15,stop_cycles:5,actuators:a}};
}
function profileSetupPage() {
    const e = state.profileEditor;
    if (!e) return pageHead('profileSetup', 'profileManagedHint') + `<div class="empty-state">${state.language === 'en' ? 'Loading configuration' : '正在加载机械臂参数'}</div>`;
    const key = state.profileGroup || 'model';
    const current = profileGroups.find(g => g[0] === key) || profileGroups[0];
    const value = key === 'hardware' ? e.hardware : profileGet(e.core, current[2].split('.'));
    const info = key === 'gains' ? (state.language === 'en' ? 'Unverified gains are only valid numeric placeholders and are not proof of safe operation' : '未标定增益只是合法数值占位，拖拽全零阻尼不代表安全，也不表示控制模式已可使用') :
        key === 'model' ? (state.language === 'en' ? 'URDF model and joint ordering are shared by all groups' : '模型和受控关节顺序由全部参数分组共享') :
        key === 'hardware' ? (state.language === 'en' ? 'Choose the supported hardware plugin and complete every actuator type before validation' : '选择已支持的硬件驱动并完成执行器型号与总线配置') : '';
    const profilePlugin = profileField(['profile_entry','hardware','plugin'], e.profile_entry?.hardware?.plugin || '');
    const optional = [
        key === 'gains' && !('identification_tracking' in e.core.control.controller) ? `<button class="button" id="profile-add-identification">${state.language === 'en' ? 'Add identification tracking gains' : '添加辨识跟踪增益'}</button>` : '',
        key === 'model' && !('gravity_correction_path' in e.core.model) ? `<button class="button" id="profile-add-correction">${state.language === 'en' ? 'Add optional correction path' : '添加可选重力校正文件路径'}</button>` : '',
        key === 'safety' && !('max_effort_override' in e.core.safety_policy) ? `<button class="button" id="profile-add-effort">${state.language === 'en' ? 'Add optional torque limits' : '添加可选最大力矩覆盖'}</button>` : '',
        key === 'runtime' && !('joint_acc_filter_alpha' in e.core.control.runtime) ? `<button class="button" id="profile-add-filter">${state.language === 'en' ? 'Add acceleration filter' : '添加加速度滤波系数'}</button>` : '',
        key === 'shutdown' && !('compliant_phase_ratio' in e.core.shutdown) ? `<button class="button" id="profile-add-park-ratio">${state.language === 'en' ? 'Add compliant park ratio' : '添加柔性停放时间比例'}</button>` : '',
        key === 'safety' && !('require_continuous_cmd' in e.core.safety_policy) ? `<button class="button" id="profile-add-continuity">${state.language === 'en' ? 'Add command continuity check' : '添加命令连续性配置'}</button>` : '',
    ].join('');
    const noHardware = key === 'hardware' && !Object.keys(e.hardware || {}).length;
    const hardwarePanel = noHardware ? `<div class="parameter-empty">${state.language === 'en' ? 'Hardware parameters are not configured yet' : '还未配置硬件参数'}<button id="profile-init-damiao" class="button">${state.language === 'en' ? 'Initialize supported Damiao fields' : '初始化达妙硬件字段'}</button></div>` : profileTree(value, ['hardware']);
    const body = key === 'hardware' ? `${profilePlugin}${hardwarePanel}` : profileTree(value, ['core', ...current[2].split('.')]);
    const changed = state.profileDirty;
    const err = state.profileError ? `<div class="parameter-error" role="alert">${esc(state.profileError)}</div>` : '';
    return (state.createdProfileWizard ? descriptionStepper(4) + `<div class="wizard-profile-note">${t('newHardwareHint')}</div>` : '') + pageHead('profileSetup', 'profileManagedHint', `<div class="parameter-top-actions"><span id="profile-dirty-state" class="hint">${changed ? (state.language === 'en' ? 'Unsaved changes' : '存在未保存修改') : (state.language === 'en' ? 'Saved on disk' : '当前内容已从磁盘加载')}</span><button id="profile-reload" class="button">${state.language === 'en' ? 'Reload' : '重新加载'}</button><button id="profile-save" class="button primary">${t('saveProfile')}</button></div>`) + `<section class="parameter-layout"><nav class="parameter-groups">${profileGroups.map(([id,title]) => `<button class="parameter-group ${id === key ? 'active' : ''}" data-profile-group="${id}">${esc(title)}${id==='gains'?'<small>6 Modes · Kp / Kd</small>':''}</button>`).join('')}</nav><section class="card parameter-card"><div class="card-head"><div><h3>${esc(current[1])}</h3><p>${esc(key === 'hardware' ? e.hardware_path : e.core_path)}</p></div></div><div class="card-body">${info ? `<p class="parameter-guidance">${info}</p>` : ''}${err}${body}${optional}</div></section></section>`;
}
function robotPage() {
    const info = state.info;
    const selectOptions = `<option value="">${t('noProfile')}</option>` + state.profiles.map(p => `<option value="${esc(p.name)}" ${p.name === state.config.profile ? 'selected' : ''}>${esc(p.name)}</option>`).join('');
    const profileBody = `<div class="field"><label>${t('source')}</label><div class="segmented"><button id="source-builtin" class="${state.source === 'builtin' ? 'active' : ''}">${t('builtin')}</button><button id="source-external" class="${state.source === 'external' ? 'active' : ''}">${t('external')}</button></div></div>${state.source === 'external' ? `<div class="field"><label for="profile_file">${t('profileFile')}</label><div class="input-row"><input id="profile_file" value="${esc(state.config.profile_file)}" placeholder="${t('fileHint')}">${button('browse-profile', 'browse', 'folder')}</div></div>` : ''}<div class="field"><label for="profile-select">${t('profileName')}</label><div class="input-row"><select id="profile-select">${selectOptions}</select>${button('refresh-profiles', 'refresh', 'refresh')}</div></div>`;
    const ports = (state.status.devices || []).map(p => `<option value="${esc(p)}">`).join('');
    const connBody = `${configField('serial_port', 'serial', info?.default_port || t('automatic'))}<datalist id="ports">${ports}</datalist><div class="row">${configField('baudrate', 'baudrate', String(info?.default_baudrate || '921600'))}${configField('bus', 'bus', info?.bus || t('automatic'))}</div><div class="field"><label for="resource_paths">${t('resourcePaths')}</label><div class="input-row"><input id="resource_paths" value="${esc(state.config.resource_paths)}" placeholder="/path/to/workspace">${button('add-root', 'addRoot', 'folder')}</div><div class="hint">${t('rootsHint')}</div></div><div class="actions">${button('inspect-profile', 'inspect', 'diagnostics')}${button('go-run', 'toRun', 'run', 'primary')}</div><div class="hint">${t('overrideHint')}</div>`;
    const summary = `<div class="summary-caption">ROBOT PROFILE</div><div class="summary-name">${esc(info?.profile || state.config.profile || '—')}</div><dl class="kv"><dt>${t('hardwarePlugin')}</dt><dd>${esc(info?.hardware_plugin || '—')}</dd><dt>${t('serial')}</dt><dd>${esc(state.config.serial_port || info?.default_port || '—')}</dd><dt>${t('writeEnabled')}</dt><dd style="color:var(--accent)">${t(info?.write_enabled === true ? 'enabled' : info?.write_enabled === false ? 'disabled' : 'unknown')}</dd><dt>${t('controllers')}</dt><dd>${info?.controllers?.map(name => `<div>${esc(name)}</div>`).join('') || '—'}</dd></dl><div class="callout">${t('validationHint')}</div>`;
    const checks = `<ul class="check-list">${[['installation', !!state.status.installed], ['resources', !!info && info.checks.filter(c => c.name !== 'device').every(c => c.ok)], ['nativeTerminal', !!info?.available?.terminal], ['ros2', !!state.status.ros2]].map(([key, ok]) => `<li><span class="dot ${ok ? 'ready' : ''}"></span>${t(key)}<span class="state">${t(ok ? 'ready' : 'unavailable')}</span></li>`).join('')}</ul>`;
    return pageHead('robotTitle', 'robotDesc') + `<div class="grid"><div class="card-stack">${card('profile', 'profileSub', 'robot', profileBody)}${card('connection', 'connectionSub', 'plug', connBody)}</div><div class="card-stack">${card('summary', 'summarySub', 'model', summary)}${card('status', 'statusSub', 'diagnostics', checks)}</div></div>`;
}
function vectorText(values, digits = 5) { return Array.isArray(values) ? values.map(v => Number(v).toFixed(digits)).join(', ') : '—'; }
function inertiaFields(matrix) { if (!Array.isArray(matrix) || matrix.length !== 3) return []; return [['ixx', matrix[0][0]], ['ixy', matrix[0][1]], ['ixz', matrix[0][2]], ['iyy', matrix[1][1]], ['iyz', matrix[1][2]], ['izz', matrix[2][2]]]; }
function inspectorSection(title, rows) { return `<section class="inspector-section"><h4>${title}</h4><dl class="kv">${rows.map(([k, v, cls = '']) => `<dt>${k}</dt><dd class="${cls}">${v}</dd>`).join('')}</dl></section>`; }
function modelInspectorHtml() {
    const data = state.modelData; if (!data) return `<div class="empty-state">${t('noModel')}</div>`;
    const link = data.model.links.find(x => x.name === state.modelSelection); const joint = data.model.joints.find(x => x.name === state.modelSelection);
    if (link) {
        const inertial = link.inertial; const childJoint = data.model.joints.find(j => j.child === link.name && j.controlled); const effective = data.native.info.effective_inertias.find(x => x.joint_name === childJoint?.name);
        const urdfRows = inertial ? [['&lt;origin xyz&gt;', vectorText(inertial.origin?.xyz)], ['&lt;origin rpy&gt;', vectorText(inertial.origin?.rpy)], ['&lt;mass value&gt;', `${Number(inertial.mass).toFixed(6)} kg`], ...inertiaFields(inertial.inertia).map(([k, v]) => [`&lt;inertia ${k}&gt;`, Number(v).toPrecision(6)]), ['校验', inertial.valid ? t('pass') : esc(inertial.reason || '—')]] : [['状态', 'URDF 未定义 &lt;inertial&gt;']];
        const effectiveRows = effective ? [['joint_name', esc(effective.joint_name)], ['mass', `${Number(effective.mass).toFixed(6)} kg`], ['center_of_mass', vectorText(effective.center_of_mass)], ['inertia matrix', effective.inertia.map(r => r.map(v => Number(v).toPrecision(6)).join('  ')).join('\n'), 'mono-block']] : [['状态', '当前 Link 无对应 Core 约简惯性']];
        return `<div class="summary-caption">LINK</div><div class="summary-name">${esc(link.name)}</div>${inspectorSection(t('urdfInertial'), urdfRows)}${inspectorSection(t('coreReducedInertia'), effectiveRows)}`;
    }
    if (joint) {
        const lim = joint.limit || {}; const origin = joint.origin || {}; const urdfRows = [['&lt;joint type&gt;', esc(joint.type)], ['&lt;parent link&gt;', esc(joint.parent)], ['&lt;child link&gt;', esc(joint.child)], ['&lt;origin xyz&gt;', vectorText(origin.xyz)], ['&lt;origin rpy&gt;', vectorText(origin.rpy)], ['&lt;axis xyz&gt;', vectorText(joint.axis, 4)]];
        for (const key of ['lower', 'upper', 'effort', 'velocity']) if (Number.isFinite(lim[key])) urdfRows.push([`&lt;limit ${key}&gt;`, `${Number(lim[key]).toPrecision(7)}${['lower', 'upper'].includes(key) ? ' rad' : ''}`]);
        if (joint.mimic) { urdfRows.push(['&lt;mimic joint&gt;', esc(joint.mimic.joint)]); urdfRows.push(['&lt;mimic multiplier&gt;', Number(joint.mimic.multiplier).toPrecision(7)]); urdfRows.push(['&lt;mimic offset&gt;', Number(joint.mimic.offset).toPrecision(7)]); }
        const coreRows = [['controlled', joint.controlled ? t('enabled') : t('disabled')], ['preview_status', esc(joint.preview_status || '—')]];
        return `<div class="summary-caption">JOINT</div><div class="summary-name">${esc(joint.name)}</div>${inspectorSection('URDF &lt;joint&gt;', urdfRows)}${inspectorSection('Core mapping', coreRows)}`;
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
function modelJointHtml() { const data = state.modelData; if (!data) return ''; const controlled = new Map(data.model.joints.filter(x => x.controlled).map(x => [x.name, x])); return data.native.config.joint_names.map((name, index) => { const joint = controlled.get(name) || {}; const lim = joint.limit || {}; const lower = Number.isFinite(lim.lower) ? lim.lower : -Math.PI; const upper = Number.isFinite(lim.upper) ? lim.upper : Math.PI; const value = state.modelPositions[index] ?? 0; return `<div class="joint-preview-row"><label>${esc(name)} <span id="joint-value-${index}">${Number(value).toFixed(3)} rad</span></label><input type="range" data-model-joint="${index}" min="${lower}" max="${upper}" step="0.001" value="${value}"><input class="joint-number" data-model-number="${index}" type="number" min="${lower}" max="${upper}" step="0.001" value="${Number(value).toFixed(4)}"></div>`; }).join(''); }
function modelPage() {
    const available = state.info?.available?.model; const action = `${button('model-reload', state.modelData ? 'modelReload' : 'modelLoad', 'refresh', 'primary')}`;
    const empty = !state.modelData ? `<div class="model-empty"><div>${icon('model')}</div><h3>${available ? t('noModel') : t('modelUnavailable')}</h3><p>${t('offlinePreview')}</p></div>` : '';
    return pageHead('modelTitle', 'modelPageDesc', action) + `<div class="model-toolbar"><label><input type="checkbox" data-model-layer="visual" ${state.modelLayers.visual ? 'checked' : ''}>${t('visualLayer')}</label><label><input type="checkbox" data-model-layer="collision" ${state.modelLayers.collision ? 'checked' : ''}>${t('collisionLayer')}</label><label><input type="checkbox" data-model-layer="linkFrames" ${state.modelLayers.linkFrames ? 'checked' : ''}>${t('linkFrameLayer')}</label><label><input type="checkbox" data-model-layer="jointAxes" ${state.modelLayers.jointAxes ? 'checked' : ''}>${t('jointAxisLayer')}</label><label><input type="checkbox" data-model-layer="com" ${state.modelLayers.com ? 'checked' : ''}>${t('comLayer')}</label><label><input type="checkbox" data-model-layer="inertia" ${state.modelLayers.inertia ? 'checked' : ''}>${t('inertiaLayer')}</label><label><input type="checkbox" data-model-layer="labels" ${state.modelLayers.labels ? 'checked' : ''}>${t('labelsLayer')}</label><div class="layer-actions"><button id="model-layers-all" class="tree-action">${t('allLayers')}</button><button id="model-layers-none" class="tree-action">${t('noLayers')}</button></div><span class="model-legend"><i class="legend-frame"></i>XYZ <i class="legend-axis"></i>Axis <i class="legend-com"></i>COM <i class="legend-total-com"></i>Total COM</span>${button('model-reset', 'resetView', '', '')}</div><div class="model-workspace"><aside class="model-tree">${modelTreeHtml()}</aside><section class="model-center"><div id="model-viewport">${empty}</div><div class="joint-preview"><div class="joint-preview-head"><b>${t('jointPreview')}</b><button id="model-zero" class="button">Zero</button></div>${modelJointHtml()}</div></section><aside class="model-inspector">${modelInspectorHtml()}</aside></div>`;
}
async function loadModelData(positions = null) {
    if (!state.info) await inspect(); if (!state.info?.available?.model) throw new Error(t('modelUnavailable')); state.modelLoading = true;
    try { const data = await api.request('model', { config: state.config, positions: positions || undefined }); state.modelData = data; state.modelPositions = data.native.state.positions.slice(); if (!state.modelSelection) state.modelSelection = data.model.links[0]?.name || ''; }
    finally { state.modelLoading = false; }
}
async function mountModelView() {
    if (state.page !== 'model' || !state.modelData || !$('model-viewport')) return;
    try { if (!modelModule) modelModule = await import('./model-view.js'); if (modelView) modelView.dispose(); modelView = new modelModule.SerialArmModelView($('model-viewport'), file => api.modelResource(file)); await modelView.setModel(state.modelData); modelView.setLayers(state.modelLayers); for (const [name, value] of Object.entries(state.modelVisibility.links)) modelView.setObjectVisible('links', name, value); for (const [name, value] of Object.entries(state.modelVisibility.linkFrames)) modelView.setLinkFrameVisible(name, value); for (const [name, value] of Object.entries(state.modelVisibility.joints)) modelView.setObjectVisible('joints', name, value); modelView.select(state.modelSelection); } catch (error) { toast('Model: ' + error.message, true); }
}
async function previewModel() {
    const native = await api.request('model_preview', { config: state.config, positions: state.modelPositions }); state.modelData.native = native; modelView?.updatePose(native);
}
function workbenchConnected() { return state.session?.state === 'running' && state.session?.mode === 'workbench'; }
function workbenchTabs() { return `<div class="workspace-tabs">${[['control', 'controlTab'], ['tuning', 'tuningTab'], ['calibration', 'calibrationTab'], ['diagnostics', 'runtimeDiagTab']].map(([key, label]) => `<button data-workbench-tab="${key}" class="${state.workbenchTab === key ? 'active' : ''}">${t(label)}</button>`).join('')}</div>`; }
function workbenchStatusBar() { const x = state.telemetry || {}; const fault = x.fault?.code || ''; const age = Number.isFinite(x.feedback_age_ms) ? `${x.feedback_age_ms.toFixed(0)} ms` : '—'; const device = (state.session?.devices || []).join(', ') || t('sourceWaiting'); return `<div class="workspace-status"><span><b>${esc(state.config.profile || '—')}</b></span><span>${esc(device)}</span><span>${esc(x.robot_state || '—')}</span><span>${esc(x.impedance_mode || '—')}</span><span>${t('feedbackAge')} ${age}</span><span class="${fault ? 'workspace-error' : ''}">${fault ? esc(fault) : 'OK'}</span></div>`; }
function workbenchJointRows() { const x = state.telemetry || {}; const names = x.joint_names || state.modelData?.native?.config?.joint_names || []; const actual = x.joint?.pos || names.map(() => 0); if (state.workbenchTargets.length !== names.length) state.workbenchTargets = actual.slice(); if (state.workbenchDelta.length !== names.length) state.workbenchDelta = names.map(() => 0); return names.map((name, i) => `<tr><td>${esc(name)}</td><td>${Number(actual[i] ?? 0).toFixed(4)}</td><td><input class="workspace-number" data-target-joint="${i}" type="number" step="0.001" value="${Number(state.workbenchTargets[i] ?? 0).toFixed(4)}"></td><td><input class="workspace-number" data-delta-joint="${i}" type="number" step="0.001" value="${Number(state.workbenchDelta[i] ?? 0).toFixed(4)}"></td><td>${Number(x.joint?.vel?.[i] ?? 0).toFixed(4)}</td><td>${Number(x.joint?.tor?.[i] ?? 0).toFixed(4)}</td></tr>`).join(''); }
function controlPanelHtml() { const x = state.telemetry || {}; const modes = ['RIGID_HOLD', 'RIGID_TRACKING', 'COMPLIANT_HOLD', 'COMPLIANT_DRAG', 'COMPLIANT_TRACKING']; const feeds = ['NONE', 'GRAVITY', 'FULL_INVERSE_DYNAMICS']; const active = x.robot_state === 'ACTIVE'; const inactive = x.robot_state === 'INACTIVE'; const activeHint = state.language === 'en' ? 'The robot must be ACTIVE before changing impedance mode' : '机器人需要处于 ACTIVE 才能切换阻抗模式'; const inactiveHint = state.language === 'en' ? 'Model feedforward can only be changed while the robot is INACTIVE' : '模型前馈只能在机器人处于 INACTIVE 时修改'; return `<div class="workspace-panel"><div class="workspace-actions"><button data-workbench-action="activate" class="button primary">${t('activateRobot')}</button><button data-workbench-action="park" class="button">${t('parkRobot')}</button><button data-workbench-action="deactivate" class="button danger">${t('deactivateRobot')}</button><button data-workbench-action="hold" class="button">${t('holdRobot')}</button><button data-workbench-action="clear_fault" class="button">${t('clearFault')}</button><button data-workbench-action="fault_compliant" class="button">${t('compliantRecovery')}</button><button data-workbench-action="fault_rigid" class="button">${t('rigidRecovery')}</button></div><section class="workspace-section"><h3>${t('impedanceMode')}</h3><div class="mode-buttons">${modes.map(mode => `<button data-impedance-mode="${mode}" class="button ${x.impedance_mode === mode ? 'active' : ''}" ${active && state.readiness?.modes?.[mode] === 'verified' ? '' : 'disabled'} title="${state.readiness?.modes?.[mode] === 'verified' ? (active ? '' : activeHint) : readinessResultLabel(state.readiness?.modes?.[mode])}">${mode}</button>`).join('')}</div></section><section class="workspace-section"><h3>${t('modelFeedforward')}</h3><div class="mode-buttons">${feeds.map(mode => `<button data-feedforward-mode="${mode}" class="button ${x.model_feedforward_mode === mode ? 'active' : ''}" ${inactive && (mode === 'NONE' || (state.readiness?.checks?.[mode === 'GRAVITY' ? 'gravity_verification' : 'dynamics_calibration'] === 'verified')) ? '' : 'disabled'} title="${inactive ? '' : inactiveHint}">${mode}</button>`).join('')}</div></section><section class="workspace-section"><div class="workspace-section-head"><h3>${t('jointCommand')}</h3><label>${t('speedScale')} <input id="workbench-speed" type="number" min="0.01" max="1" step="0.01" value="${state.speedScale}"></label></div><div class="table-scroll"><table class="workspace-table"><thead><tr><th>${t('joint')}</th><th>${t('actual')}</th><th>${t('target')}</th><th>${t('delta')}</th><th>dq</th><th>τ</th></tr></thead><tbody>${workbenchJointRows()}</tbody></table></div><div class="workspace-actions"><button id="execute-absolute" class="button primary">${t('executeAbsolute')}</button><button id="execute-relative" class="button">${t('executeRelative')}</button><button id="hold-motion" class="button">${t('holdRobot')}</button><span class="command-state">${t('commandState')} ${esc(commandStateText(state.commandState))}</span></div></section></div>`; }
function tuningRows() { const a = state.tuneDraft || state.admittance; if (!a) return `<div class="empty-state">${state.language === 'en' ? 'Connect the workspace to load runtime parameters' : '请先连接工作台以加载运行时参数'}</div>`; const names = state.telemetry?.joint_names || []; return names.map((name, i) => { const M = Number(a.mass?.[i] ?? 0), D = Number(a.damping?.[i] ?? 0), K = Number(a.stiffness?.[i] ?? 0); const wn = M > 0 && K > 0 ? Math.sqrt(K / M) : NaN; const dcrit = M >= 0 && K >= 0 ? 2 * Math.sqrt(M * K) : NaN; const z = dcrit > 0 ? D / dcrit : NaN; return `<tr><td>${esc(name)}</td>${[['mass', M], ['damping', D], ['stiffness', K], ['max_delta_q', a.max_delta_q?.[i]], ['max_delta_q_dot', a.max_delta_q_dot?.[i]], ['momentum_gain', a.momentum_gain?.[i]]].map(([key, value]) => `<td><input class="workspace-number" data-tune-key="${key}" data-tune-index="${i}" type="number" step="0.001" value="${Number(value ?? 0).toFixed(4)}"></td>`).join('')}<td>${Number.isFinite(wn) ? wn.toFixed(3) : '—'}</td><td>${Number.isFinite(dcrit) ? dcrit.toFixed(3) : '—'}</td><td>${Number.isFinite(z) ? z.toFixed(3) : '—'}</td><td>${K > 0 ? (1 / K).toFixed(4) : '—'}</td></tr>`; }).join(''); }
function tuningPanelHtml() { const a = state.tuneDraft || state.admittance; return `<div class="workspace-panel"><section class="workspace-section"><div class="workspace-section-head"><div><h3>${t('admittanceParameters')}</h3><p class="hint">ωn=√(K/M) · Dcrit=2√(MK) · ζ=D/Dcrit · Δqss=τ/K</p></div><label>${t('observerMode')} <select id="observer-mode"><option ${a?.observer_mode === 'FULL_ID' ? 'selected' : ''}>FULL_ID</option><option ${a?.observer_mode === 'MOMENTUM' ? 'selected' : ''}>MOMENTUM</option></select></label></div><div class="table-scroll"><table class="workspace-table wide"><thead><tr><th>${t('joint')}</th><th>M</th><th>D</th><th>K</th><th>${state.language === 'en' ? 'Maximum Δq' : '最大 Δq'}</th><th>${state.language === 'en' ? 'Maximum Δq̇' : '最大 Δq̇'}</th><th>MOMENTUM gain</th><th>ωn</th><th>Dcrit</th><th>ζ</th><th>1 Nm Δq</th></tr></thead><tbody>${tuningRows()}</tbody></table></div><div class="workspace-actions"><button id="apply-tuning" class="button primary">${t('apply')}</button><button id="reset-tuning" class="button">${t('reset')}</button><button id="preview-config" class="button">${t('previewSave')}</button>${state.savePreview?.changed ? `<button id="save-config" class="button danger">${t('saveConfig')}</button>` : ''}</div>${state.savePreview ? `<div class="diff-box"><b>${esc(state.savePreview.path)}</b>${(state.savePreview.changes || []).map(x => `<div><del>${esc(x.before)}</del><ins>${esc(x.after)}</ins></div>`).join('') || `<p>${t('noChanges')}</p>`}</div>` : ''}</section><section class="workspace-section"><h3>${t('gravityScale')}</h3><p class="hint">${state.language === 'en' ? 'Apply only while INACTIVE; this is handled separately from runtime admittance parameters' : '仅允许在 INACTIVE 状态应用，并与导纳运行时参数分开处理'}</p><div class="gravity-grid">${(state.telemetry?.joint_names || []).map((name, i) => `<label>${esc(name)}<input class="workspace-number" data-gravity-index="${i}" type="number" min="0" max="2" step="0.001" value="${Number(state.telemetry?.gravity_scale?.[i] ?? 1).toFixed(4)}"></label>`).join('')}</div><button id="apply-gravity" class="button">${t('apply')}</button></section></div>`; }
// A persisted offline fit is a separate, explicitly selected gravity source
// No live control command is sent when reading or displaying it
function adoptGravityRecord(record) {
    state.modelCalibrationOffline = record;
    const offline = record?.recomputed_candidate;
    if (offline) {
        record.result = {
            ...(record.result || {}), gravity_result: offline,
            static_pass: offline.static_pass === true,
            friction_status: 'requires_refit_or_review', friction_pass: null,
        };
        state.modelCalibrationCandidateSource = 'offline';
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
    return { offline, directory, validated, comparable: binding.comparable,
        historical: binding.historical === true,
        applied, canApply: !!directory && validated && binding.comparable && !binding.historical && inactive && !applied };
}
function currentModelCalibration() {
    if (state.modelCalibrationOffline?.result) return state.modelCalibrationOffline.result;
    return state.telemetry?.model_calibration || null;
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
    try { alignment = await api.request('model_calibration_model_alignment', {config: state.config, directory: record.directory}); }
    catch (error) { alignment = {comparable: false, expected, actual, warnings: [], error: `历史候选对齐失败: ${error.message}`}; }
    if (state.modelCalibrationOffline?.directory !== record.directory) return;
    state.modelCalibrationPreviewAlignment = {...alignment, directory: record.directory};
    render();
}

function modelCalibrationPhaseText(phase) {
    const labels = {
        idle: ['未启动', 'Idle'], teaching: ['拖动示教中', 'Teaching'], waiting_replay_confirmation: ['等待松手与回放确认', 'Waiting for replay confirmation'], gain_adaptation: ['自动整定与验证跟踪增益', 'Adapting and validating tracking gains'],
        recorded_only: ['已录制轨迹 · 未标定', 'Recorded · Not calibrated'],
        static_reverse: ['反向回放 · 重力姿态及动力学数据', 'Reverse replay · gravity and dynamic data'], static_forward: ['正向静态停留采样', 'Forward static sampling'], gravity_fitting: ['拟合候选重力模型', 'Fitting gravity candidate'],
        friction_reverse_slow: ['低速反向摩擦采集', 'Slow reverse friction sampling'], friction_forward_slow: ['低速正向摩擦采集', 'Slow forward friction sampling'], friction_reverse_fast: ['高速反向摩擦采集', 'Fast reverse friction sampling'], friction_forward_fast: ['正向回放 · 动力学采集', 'Forward dynamic sampling'],
        friction_fitting: ['拟合并验证摩擦模型', 'Fitting and validating friction'], paused: ['已暂停并保持当前位置', 'Paused and holding'], complete: ['任务完成', 'Complete'], cancelled: ['任务已取消', 'Cancelled'], failed: ['任务失败', 'Failed'],
    };
    const value = labels[phase] || [phase || '—', phase || '—']; return state.language === 'en' ? value[1] : value[0];
}
function modelCalibrationActive(status = state.telemetry?.model_calibration) { return !!status && !['idle', 'complete', 'cancelled', 'failed', 'waiting_replay_confirmation'].includes(status.phase); }
function gainCalibrationSeconds(oneWay, mode) {
    const rigid = oneWay * (1 / 0.8 + 3) + 1.5;
    const compliant = oneWay * (1 / 0.55 + 1 / 0.75 + 2 / 0.65) + 1.5;
    const identification = oneWay * (1 / 0.8 + 3);
    return (mode === 'all' ? rigid + compliant + identification :
        mode === 'compliant' ? compliant : mode === 'identification' ? identification : rigid) + 2;
}
function gainCandidateDeployable(candidate) {
    if (!candidate || candidate.validated !== true) return false;
    const mode = candidate.category;
    const soft = mode === 'all' ? candidate.targets?.compliant_tracking :
        mode === 'compliant_tracking' ? candidate : null;
    return !soft || (soft.validated === true &&
        soft.compliant_separation_validated === true &&
        soft.hold_verification?.validated === true &&
        Array.isArray(soft.rigid_reference_kp) &&
        Array.isArray(soft.rigid_reference_kd) &&
        Number(soft.tested_speed_fraction) > 0);
}
function gainVerificationDescription(candidate, en) {
    if (!candidate) return '';
    const stages = candidate.category === 'all' ? (candidate.targets || {}) : {[candidate.category]: candidate};
    return Object.entries(stages).map(([key, data]) => {
        const name = key === 'compliant_tracking' ? (en ? 'Compliant' : '柔性') :
            key === 'rigid_tracking' ? (en ? 'Rigid' : '刚性') : (en ? 'Identification' : '辨识');
        const speed = Math.round(Number(data.tested_speed_fraction || 1) * 100);
        const hold = data.hold_verification;
        return `${name} ${data.validated ? 'PASS' : 'REVIEW'} · ${speed}% ${en ? 'replay' : '回放'}${hold ? ` · ${en ? 'Hold' : '保持'} ${hold.validated ? 'PASS' : 'REVIEW'}` : ''}`;
    }).join(' · ');
}
function modelCalibrationLongRunning(status = state.telemetry?.model_calibration) { return !!status && ['teaching', 'gain_adaptation', 'static_reverse', 'static_forward', 'gravity_fitting', 'friction_reverse_slow', 'friction_forward_slow', 'friction_reverse_fast', 'friction_forward_fast', 'friction_fitting', 'paused'].includes(status.phase); }
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
    const rows = names.map((name, i) => `<tr><td>${esc(name)}</td><td>${metric(original, 'rms', i)}</td><td>${metric(scaled, 'rms', i)}</td><td>${metric(candidate, 'rms', i)}</td><td>${metric(candidate, 'p99', i)}</td><td>${metric(candidate, 'max', i)}</td><td>${metric({noise:result.noise_rms}, 'noise', i)}</td></tr>`).join('');
    const testName = offline ? (en ? 'Offline static holdout' : '离线静态留出评价') : (en ? 'Online static holdout' : '在线静态留出评价');
    const testResult = result.static_validation_status === 'unavailable' ? (en ? 'Unavailable' : '不可用')
        : result.static_validation_status === 'limited' ? (en ? 'Limited validation · review only' : '验证姿态不足 · 仅供审核')
        : (result.static_pass ? (en ? 'Passes own criteria' : '满足本算法门槛') : (en ? 'Fails own criteria' : '未满足本算法门槛'));
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
    const age = Number(telemetry?.feedback_age_ms);
    const valid = !!status?.source_task_id && Array.isArray(target) && target.length > 0 &&
        Array.isArray(pos) && pos.length === target.length &&
        Array.isArray(vel) && vel.length === target.length &&
        telemetry?.robot_state === 'ACTIVE' && telemetry?.valid === true &&
        Number.isFinite(age) && age >= 0 && age <= 500 &&
        [...target, ...pos, ...vel].every(x => Number.isFinite(Number(x)));
    if (!valid) return {valid: false, aligned: false, maxError: Infinity, maxSpeed: Infinity, rows: []};
    const rows = target.map((q, i) => ({name: (status.joint_names || telemetry.joint_names || [])[i] || `joint${i+1}`, now: Number(pos[i]), goal: Number(q), delta: Number(q)-Number(pos[i]), speed: Number(vel[i])}));
    const maxError = Math.max(...rows.map(x => Math.abs(x.delta)));
    const maxSpeed = Math.max(...rows.map(x => Math.abs(x.speed)));
    return {valid: true, aligned: maxError <= 0.08 && maxSpeed <= 0.05 && !status.alignment_active,
        maxError, maxSpeed, rows};
}
function calibrationAlignmentStatus(align, manual) {
    if (!align.valid) return state.language === 'en' ? 'Waiting for fresh robot feedback (max 500 ms)' : '等待新鲜关节反馈（最大 500 ms）';
    if (manual) return state.language === 'en' ? 'Guided drag active; support arm and move slowly' : '手动拖拽引导中：请支撑机械臂缓慢调整';
    if (align.aligned) return state.language === 'en' ? 'Ready for separate replay confirmation' : '姿态已对齐，可以再次确认自动回放';
    return state.language === 'en' ? 'Pose not aligned; replay blocked' : '姿态尚未对齐，暂不允许自动回放';
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
        if (status.source_task_id) {
            actions.push(status.alignment_active
                ? `<button id="model-calibration-alignment-finish" class="button primary">${state.language === 'en' ? 'Stop drag and hold here' : '结束引导并保持当前位置'}</button>`
                : `<button id="model-calibration-alignment-begin" class="button" ${robotActive ? '' : 'disabled'}>${state.language === 'en' ? 'Guided manual return to replay start' : '引导返回回放起点（人工拖拽）'}</button>`);
        }
        actions.push(`<button id="model-calibration-replay-confirm" class="button danger" ${status.source_task_id && !alignment.aligned ? 'disabled' : ''}>${t('modelCalibrationConfirmReplay')}</button>`);
    }
    if (live && ['gain_adaptation', 'static_reverse', 'static_forward', 'gravity_fitting', 'friction_reverse_slow', 'friction_forward_slow', 'friction_reverse_fast', 'friction_forward_fast', 'friction_fitting'].includes(phase)) actions.push(`<button id="model-calibration-pause" class="button">${t('pauseTask')}</button>`);
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
        ? gainCalibrationSeconds(singlePass, state.gainMode)
        : Number(status.estimated_duration_s || 0);
    const groups = Array.isArray(status.pose_targets) ? status.pose_targets.length : 0;
    const teaching = Number(status.teaching_wall_duration_s || 0);
    const totalEstimate = gainPreview ? teaching + estimate : Number(status.estimated_total_duration_s || teaching + estimate);
    const recorded = Number(status.recorded_duration_s || 0);
    const planningReady = live && phase === 'waiting_replay_confirmation' && status.planner_id === 'local_hermite' && status.calibration_strategy === 'two_pass_combined' && Number.isFinite(estimate) && estimate > 0 && singlePass > 0;
    const replayHint = planningReady ? `<div class="callout"><b>${state.language === 'en' ? 'Online capture time estimate' : '已选示教轨迹 · 在线采集耗时预估'}</b>
      <div>${state.language === 'en' ? 'Demonstration' : '自由示教'} ${teaching.toFixed(1)} s · ${state.language === 'en' ? 'Recorded track' : '原始轨迹'} ${recorded.toFixed(1)} s · ${state.language === 'en' ? 'Retimed one-way pass' : '优化后单程'} ${singlePass.toFixed(1)} s</div>
      <div>${state.language === 'en' ? 'Parking now cancels the pending automatic replay; saved task records remain on disk.' : '此时选择停放会结束待确认的自动回放，已记录的任务数据仍保存在磁盘中'}</div>
      <div>${state.language === 'en' ? 'Trajectory path' : '轨迹路径长度'} ${Number(status.joint_path_length_rad || 0).toFixed(2)} rad · ${state.language === 'en' ? 'Reduced waypoints' : '几何关键点'} ${Number(status.geometric_waypoints || 0)} / ${Number(status.original_samples || 0)} · ${state.language === 'en' ? 'Planner' : '规划器'} local_hermite</div>
      <div>${gainPreview ? (state.language === 'en' ? 'Gain calibration: two round trips per target; compliant validation at 65% and separate hold check' : '增益标定：每项目标两次往返，柔性采用 65% 速度验收并独立检查保持稳定性') : (state.language === 'en' ? 'Gravity correction: reverse + forward measurement passes and static holds' : '重力校正：正反向正式采集及静态停留')} ≈ <b>${totalEstimate.toFixed(1)} s</b> (${state.language === 'en' ? 'automatic stage' : '自动阶段'} ${estimate.toFixed(1)} s)</div>
      <div>${gainPreview ? (state.language === 'en' ? 'Estimation excludes extra feedback settling; unsuccessful candidates remain saved but cannot be applied.' : '这是不含跟踪等待的计划估算；未通过验收的增益候选会保存，但不能直接应用') : (state.language === 'en' ? 'Actual settling and fitting may take longer; offline full inertial fitting is not included.' : '实际停稳与拟合可能更久，且不包含后续离线惯量计算')} ${t('releaseSafetyHint')}</div></div>` : (phase === 'teaching' ? `<div class="callout">${state.language === 'en' ? 'Teaching is not time-limited.' : '示教时间不设上限，结束示教后根据轨迹运动量和关节安全限制计算自动回放时间'}</div>` : (phase === 'waiting_replay_confirmation' ? `<div class="callout error-callout">${state.language === 'en' ? 'Native Core is outdated; reinstall before automatic playback.' : '正在运行的 C++ Core 未加载新的轨迹规划器，请重新安装编译后再自动回放'}</div>` : ''));
    const resumeSafetyHint = live && status.source_task_id ? `<div class="callout"><b>${state.language === 'en' ? 'Imported demonstration' : '已恢复历史轨迹'}：${esc(status.source_task_id)}</b>
      <div>${status.original_start_selected ? (state.language === 'en' ? 'Current pose was closer to original FIRST pose: imported path was reversed. The same recorded path is replayed, without an extra move.' : '当前位置更接近原示教起点：已自动选择从这一端开始，沿原示教路径采集，无须额外归位') : (state.language === 'en' ? 'Current pose was closer to original LAST pose: standard reverse/forward collection.' : '当前位置更接近原示教终点：采用原反向／正向采集流程')}</div>
      <div>${state.language === 'en' ? 'Target joint angles (rad)' : '回放起点关节角（rad）'}：${esc((status.replay_start_joint_positions || []).map(x => Number(x).toFixed(3)).join('，'))}</div>
      <div id="model-calibration-align-status"><b>${esc(calibrationAlignmentStatus(alignment, status.alignment_active))}</b> ${alignment.valid ? `· Δmax ${alignment.maxError.toFixed(3)} rad · |dq|max ${alignment.maxSpeed.toFixed(3)} rad/s` : ''}</div>
      <div class="table-scroll"><table class="workspace-table"><thead><tr><th>${state.language === 'en' ? 'Joint' : '关节'}</th><th>${state.language === 'en' ? 'Actual' : '当前'}</th><th>${state.language === 'en' ? 'Target' : '目标'}</th><th>Δ(rad)</th></tr></thead><tbody id="model-calibration-align-rows">${alignment.rows.map(x => `<tr><td>${esc(x.name)}</td><td>${x.now.toFixed(3)}</td><td>${x.goal.toFixed(3)}</td><td>${x.delta >= 0 ? '+' : ''}${x.delta.toFixed(3)}</td></tr>`).join('')}</tbody></table></div>
      <div>${state.language === 'en' ? 'Limits: max |Δq| 0.08 rad, max |dq| 0.05 rad/s; live feedback age <= 500 ms.' : '门槛：最大关节误差 0.08 rad、最大关节速度 0.05 rad/s；反馈须在 500 ms 内'}</div>
      <div>${state.language === 'en' ? 'No environment collision planner is available in standalone Core. Guided return is MANUAL: support the arm and check the workspace; the software never drives directly from park to target.' : 'Standalone Core 暂无环境碰撞规划器这里只提供人工拖拽引导：支撑机械臂、确认周围空间后缓慢调整，不会自动从停放位向目标直线运动'}</div></div>` : '';
    const recordInventory = state.modelCalibrationRecords.length ? `<div class="callout"><b>${state.language === 'en' ? 'Saved task inventory' : '已保存任务记录'}</b>${state.modelCalibrationRecords.map((r, i) => {
        const fields = r.files || {};
        const items = ['frames.csv', 'trajectory.csv', 'trajectory.checkpoint.csv', 'result.json'];
        return `<div class="task-directory"><button id="model-calibration-record-${i}" class="button" title="${esc(r.directory || '')}">${esc(r.task_id || '任务')}</button> ${items.map(name => `<span class="result-badge ${fields[name] ? 'pass' : 'neutral'}">${esc(name)} ${fields[name] ? '✓' : '—'}</span>`).join(' ')}<div>${esc(r.note || '')}</div></div>`;
    }).join('')}</div>` : '';
    const offlineRecord = state.modelCalibrationOffline;
    const offlineHint = offlineRecord ? `<div class="callout"><b>${esc(state.language === 'en' ? ({teaching_only:'Recorded demonstration',interrupted:'Interrupted task',completed:'Completed calibration'}[offlineRecord.record_kind] || 'Task record') : ({teaching_only:'仅有示教轨迹',interrupted:'标定中断',completed:'标定已完成'}[offlineRecord.record_kind] || '任务记录'))}</b><div>${esc(offlineRecord.note || '')}</div><div>${state.language === 'en' ? 'Saved trajectory' : '保存的轨迹'}：${Number(offlineRecord.trajectory?.samples || 0)} ${state.language === 'en' ? 'samples' : '帧'} · ${Number(offlineRecord.trajectory?.duration_s || 0).toFixed(1)} s</div><div>${esc(Object.entries(offlineRecord.files || {}).map(([name, exists]) => `${name}: ${exists ? '✓' : '—'}`).join(' · '))}</div><div>${offlineRecord.trajectory?.source === 'trajectory.checkpoint.csv' ? (state.language === 'en' ? 'Partial teaching checkpoint · must verify before replay' : '中断示教检查点 · 回放前必须重新校验') : ''}</div><div>${state.language === 'en' ? 'Reading a record never moves the robot. Importing creates a NEW task; replay always starts at the beginning and requires a second confirmation.' : '加载记录不会使机械臂运动恢复会创建新任务，自动采集必须从头执行并再次确认，不能从中断点直接续动'}</div></div>` : '';
    const resultBasis = state.modelCalibrationCandidateSource === 'offline'
        ? (state.language === 'en' ? 'Offline refined URDF · global: mass + first moments; local: legacy fit with selected links preserved · not applied automatically' : '离线精修 URDF · 全局：质量与一阶质量矩联合求解；局部：旧版回归并保护所选 Link · 不自动应用')
        : (state.language === 'en' ? 'Online quick preview · fixed mass, first moments only · current runtime calibration; not interchangeable with offline' : '在线快速预览 · 固定质量，仅辨识一阶质量矩 · 可供当前运行时应用，与离线结果不等价');
    const resultBlock = result?.first_moments?.length && ['complete', 'failed'].includes(phase) ? `<div class="callout"><b>${state.language === 'en' ? 'Candidate meaning' : '当前候选含义'}</b><div>${resultBasis}</div><div>${state.language === 'en' ? 'Passed/failed refers only to this solver and its evaluation poses; no whole-workspace or real-hardware certification' : '通过或未通过只对应当前算法及其留出姿态，不能作为跨工作空间或真机验收结论'}</div></div><div class="candidate-compare-head"><h4>${t('candidateComparison')}</h4><span>${t('candidateScope')}</span></div>${(!result.static_pass || result.review_warnings?.length) ? `<div class="callout"><b>${state.language === 'en' ? 'Needs human review' : '需要人工审核'}</b> · ${esc(result.failure_reason || 'holdout result not approved')} · ${state.language === 'en' ? 'Candidate may be exported for manual review if values are finite and physically valid, but automatic application requires passing validation' : '未通过自动验收的候选仍可导出供人工审核，但不会被允许临时应用到真机'}</div>` : ''}${result.identification_mode === 'local' ? `<div class="callout">${state.language === 'en' ? 'Local fit' : '局部辨识'} · ${state.language === 'en' ? 'Locked' : '锁定'} ${esc((result.locked_links || []).join(', ') || '—')} · ${state.language === 'en' ? 'Fitted' : '允许校正'} ${esc((result.fitted_links || []).join(', ') || '—')}</div>` : ''}${status.friction_status === 'requires_refit_or_review' ? `<div class="callout">${state.language === 'en' ? 'Friction result belongs to a different gravity candidate; do not reuse it' : '摩擦参数属于原来的重力候选，当前离线重算结果尚未重新匹配摩擦模型，不可直接复用'}</div>` : ''}${modelCalibrationMetricTable(status)}${modelCalibrationCandidateTable(status)}<div class="workspace-actions">${result.static_pass && state.modelCalibrationCandidateSource !== 'offline' ? `<button id="model-calibration-preview-save" class="button">${t('previewSave')}</button>${state.modelCalibrationSavePreview ? `<button id="model-calibration-save" class="button primary">${t('saveCandidate')}</button>` : ''}` : ''}<span class="gravity-export-hint">${state.modelCalibrationCandidateSource === 'offline' ? (state.language === 'en' ? 'Offline candidate mass and COM, computed from saved data; source inertia-at-COM remains unchanged; review holdout separately' : '离线重算的质量与质心候选，质量和质心可同时变化；惯性张量保留原值，须独立评估留出表现') : (state.language === 'en' ? 'Original masses and inertia tensors are retained; only fitted COM values are updated' : '保留原始质量和惯性张量，仅更新辨识质心')}</span>${state.modelCalibrationSaved ? `<button id="model-calibration-restore-config" class="button">${t('restoreSavedConfig')}</button>` : ''}</div>${state.modelCalibrationSavePreview ? `<div class="diff-box"><b>${esc(state.modelCalibrationSavePreview.path)}</b>${(state.modelCalibrationSavePreview.changes || []).map(x => `<div><del>${esc(x.before)}</del><ins>${esc(x.after)}</ins></div>`).join('') || `<p>${t('noChanges')}</p>`}</div>` : ''}${state.modelCalibrationFrictionExport ? `<div class="callout"><b>${state.language === 'en' ? 'Friction candidate · review only' : '摩擦候选 · 仅供审核'}</b><div class="path-chip">${esc(state.modelCalibrationFrictionExport.path || '')}</div><div>${state.modelCalibrationFrictionExport.matches_current_candidate ? (state.language === 'en' ? 'Gravity source matches' : '重力来源一致') : (state.language === 'en' ? 'Gravity model has changed; friction fit needs revalidation' : '重力候选已变化，原摩擦结果需要重新验证')}</div></div>` : ''}${state.modelCalibrationExport ? `<div class="callout"><b>${t('exportCandidate')}</b><div class="path-chip">${esc(state.modelCalibrationExport.path || state.modelCalibrationExport.candidate_urdf || '')}</div>${state.modelCalibrationExport.verification_warning ? `<div class="callout">${esc(state.modelCalibrationExport.verification_warning)}</div>` : ''}${state.modelCalibrationExport.verification ? `<div>gravity RMS ${Number(state.modelCalibrationExport.verification.gravity_rms_nm || 0).toExponential(3)} Nm · FK Δp ${Number(state.modelCalibrationExport.verification.fk_max_position_error_m || 0).toExponential(3)} m</div>` : ''}</div>` : ''}` : '';
    return `<section class="workspace-section model-calibration-card"><div class="workspace-section-head"><div><h3>${t('modelCalibration')}</h3><p class="hint">${t('modelCalibrationDesc')}</p></div><div class="result-badges">${sourceTag}<span id="model-calibration-phase" class="result-badge neutral">${esc(modelCalibrationPhaseText(phase))}</span></div></div>${calibrationTaskGuidance('gravity')}<div class="model-calibration-identification"><b>${state.language === 'en' ? 'Gravity identification' : '重力模型辨识范围'}</b><div class="mode-buttons"><button id="model-cal-mode-global" type="button" class="button ${options.mode !== 'local' ? 'active' : ''}" ${optionsDisabled ? 'disabled' : ''}>${state.language === 'en' ? 'Global' : '全局辨识'}</button><button id="model-cal-mode-local" type="button" class="button ${options.mode === 'local' ? 'active' : ''}" ${optionsDisabled ? 'disabled' : ''}>${state.language === 'en' ? 'Local' : '局部辨识'}</button>${options.mode === 'local' ? `<button id="model-cal-lock-select" type="button" class="button" ${optionsDisabled ? 'disabled' : ''}>${state.language === 'en' ? 'Trusted links' : '选择可信 Link'}</button><span>${state.language === 'en' ? 'Locked' : '已锁定'} ${(options.locked_links || []).length}</span>` : ''}</div><p class="hint">${options.mode === 'local' ? (state.language === 'en' ? 'Local scope locks trusted links; online estimates first moments while offline uses the legacy fit on unlocked links' : '局部范围锁定可信 Link；在线仅修正一阶质量矩，离线按旧版回归修正未锁定 Link') : (state.language === 'en' ? 'Global selects all controlled links; online fixes mass, offline fits mass + first moments' : '全局选择全部受控 Link；在线固定质量，离线联合估计质量与一阶质量矩')}${!calibrationSelectableLinks().length ? (state.language === 'en' ? ' · No link list yet: load the model or a calibration record' : ' · 尚未取得 Link 清单，请加载模型或历史任务') : ''}</p></div><div class="model-calibration-options"><label>${t('poseBudget')}<input id="model-cal-pose-budget" type="number" min="5" max="16" step="1" value="${Number(options.pose_budget)}" ${optionsDisabled ? 'disabled' : ''}></label><label>${t('validationFraction')}<input id="model-cal-validation" type="number" min="0.15" max="0.45" step="0.05" value="${Number(options.validation_fraction)}" ${optionsDisabled ? 'disabled' : ''}></label><label>${t('regularizationStrength')}<input id="model-cal-regularization" type="number" min="0.00000001" max="10" step="0.001" value="${Number(options.regularization)}" ${optionsDisabled ? 'disabled' : ''}></label><label>${t('svdRelativeThreshold')}<input id="model-cal-svd" type="number" min="0.00000001" max="0.2" step="0.0001" value="${Number(options.svd_relative_threshold)}" ${optionsDisabled ? 'disabled' : ''}></label><label>${t('minimumInformationScore')}<input id="model-cal-information" type="number" min="0.00000001" max="1000" step="0.0001" value="${Number(options.minimum_information_score)}" ${optionsDisabled ? 'disabled' : ''}></label></div><div class="workspace-actions">${actions.join('')}<button id="model-calibration-load-record" class="button">${t('loadRecord')}</button><button id="model-calibration-scan-records" class="button">${state.language === 'en' ? 'Scan saved tasks' : '扫描已有任务'}</button>${state.modelCalibrationOffline ? `<button id="model-calibration-refresh-record" class="button">${state.language === 'en' ? 'Refresh record' : '重新检查记录'}</button>` : ''}${directory && (!state.modelCalibrationOffline || state.modelCalibrationOffline.has_result) ? `<button id="model-calibration-recompute" class="button">${t('recomputeRecord')}</button>` : ''}</div><div class="model-calibration-summary"><div><span>${t('taskIdentifier')}</span><b id="model-calibration-task-id-inline">${esc(status.task_id || '—')}</b></div><div><span>${t('progressLabel')}</span><b id="model-calibration-progress-inline">${Math.round(Number(status.progress || 0) * 100)}%</b></div><div><span>${t('trajectorySamples')}</span><b id="model-calibration-trajectory-inline">${esc(status.trajectory_samples ?? '—')}</b></div><div><span>${t('trainingValidationGroups')}</span><b id="model-calibration-groups-inline">${training} / ${validation}</b></div><div><span>${t('validStaticSamples')}</span><b id="model-calibration-static-inline">${esc(status.valid_static_samples ?? '—')}</b></div><div><span>${t('acceptedDroppedFrames')}</span><b id="model-calibration-recorder-inline">${esc(status.recorder?.accepted ?? '—')} / ${esc(status.recorder?.dropped ?? '—')}</b></div></div>${status.error ? `<div class="callout error-callout">${esc(localizedError(status.error))}</div>` : ''}${!modelCalibrationModelBinding().comparable ? `<div class="callout error-callout">${esc(modelCalibrationModelBinding().reason)}</div>` : ''}${modelCalibrationModelBinding().historical ? `<div class="callout"><b>${state.language === 'en' ? 'Historical model preview · read only' : '历史模型对照 · 仅预览'}</b><div>${state.language === 'en' ? 'Only inertial parameters differ, geometry and joints match, this does not validate the updated dynamics or authorize replay or application' : '已核对只有惯量参数改变，几何与关节保持一致，历史 RMS 不代表当前模型的预测精度，此预览不授权回放或参数应用'}</div><div>${state.language === 'en' ? 'Changed links' : '变化的 Link'} ${esc(modelCalibrationModelBinding().alignment.changed_links.join(', '))}</div>${(modelCalibrationModelBinding().alignment.warnings || []).map(w => `<div class="workspace-error">${esc(w)}</div>`).join('')}</div>` : ''}${replayHint}${resumeSafetyHint}${offlineHint}${recordInventory}${resultBlock}${directory ? `<div class="task-directory"><span>${t('taskDirectory')}</span><code>${esc(directory)}</code></div>` : ''}</section>`;
}
function singleCalibrationToolsHtml() {
    const c = state.telemetry?.calibration || { kind: 'none', phase: 'idle', captured: 0, expected: 0 }; const progress = c.expected ? `${c.captured}/${c.expected}` : '';
    return `<details class="workspace-section single-calibration-tools"><summary>${t('singleCalibrationTools')}</summary><div class="single-calibration-tools-body"><section><h4>${t('staticCalibration')}</h4><p>${t('staticCalibrationDesc')}</p><div class="workspace-actions"><button data-calibration-begin="static" class="button">${t('start')}</button><button id="calibration-capture" class="button">${t('capturePose')}</button><button id="calibration-finish" class="button">${t('finishFit')}</button></div></section><section><h4>${t('staticValidation')}</h4><button data-calibration-begin="validation" class="button">${t('startValidation')}</button></section><section><h4>${t('frictionCalibration')}</h4><div class="workspace-actions"><button id="friction-record-start" class="button">${t('startRecording')}</button><button id="friction-record-stop" class="button">${t('stopRecording')}</button><button id="friction-replay" class="button danger">${t('startReplay')}</button></div></section><dl class="kv compact"><dt>${t('taskType')}</dt><dd id="cal-task-kind">${esc(calibrationTaskKindText(c.kind))}</dd><dt>${t('taskPhase')}</dt><dd id="cal-task-phase">${esc(calibrationTaskPhaseText(c.phase))}</dd><dt>${t('progressLabel')}</dt><dd id="cal-task-progress">${esc(progress || '—')}</dd><dt>${t('trajectorySamples')}</dt><dd id="cal-task-trajectory">${esc(c.trajectory_samples ?? '—')}</dd><dt>${t('errorLabel')}</dt><dd id="cal-task-error" class="workspace-error">${esc(c.error ? localizedError(c.error) : '—')}</dd></dl><div class="workspace-actions"><button id="calibration-cancel" class="button danger">${t('cancelTask')}</button><button id="export-session" class="button">${t('exportData')}</button></div></div></details>`;
}
function advancedIdentificationPanelHtml() {
    const en = state.language === 'en';
    const dir = state.modelCalibrationDirectory || currentModelCalibration()?.directory || '';
    const status = currentModelCalibration() || {};
    const options = [
        ['mass',en?'Mass (COM prior)':'质量辨识 · 固定 COM'],
        ['com',en?'COM (mass prior)':'质心辨识 · 固定质量'],
        ['mass_com',en?'Joint mass and COM':'质量与质心联合辨识'],
        ['inertia',en?'Inertia tensor (mass/COM prior)':'惯性张量辨识 · 固定质量和质心'],
        ['full',en?'Full inertial identification':'完整惯性参数联合辨识'],
    ];
    const assessment = state.advancedAssessment?.directory === dir ? state.advancedAssessment : null;
    const result = state.advancedReport?.task_id === status.task_id ? state.advancedReport : null;
    const dynamic = ['inertia','full'].includes(state.advancedTarget);
    const missing = !dir || !state.modelCalibrationOffline?.metadata;
    const disable = missing || workbenchConnected() || !assessment || (dynamic && !assessment.dynamic_precheck) || (!dynamic && !assessment.gravity_ready) || (calibrationSelectableLinks().length > 0 && state.advancedLocked.length >= calibrationSelectableLinks().length);
    const limits = en ? 'Offline-only candidate, no motor action or automatic application. Rank, physical consistency and validation are checked separately' : '仅离线分析，不发送电机指令或自动应用参数，分别核验回归秩、物理一致性和留出误差';
    const available = calibrationSelectableLinks();
    const locks = new Set(state.advancedLocked);
    const lockList = available.map(link => `<label class="calibration-advanced-link"><input type="checkbox" data-advanced-locked="${esc(link.name)}" aria-label="${en ? 'Keep original parameters for ' : '保留原始参数 ' }${esc(link.name)}" ${locks.has(link.name)?'checked':''}><span>${esc(link.name)}</span></label>`).join('');
    return `<section class="workspace-section calibration-advanced-card">
      <div class="workspace-section-head"><div><h3>${en?'Advanced dynamics identification':'高级动力学辨识'}</h3><p class="hint">${en?'Continue from saved data, identify only missing parameters':'复用现有数据 · 仅补充未完成的参数辨识'}</p></div><span class="result-badge neutral">${en?'Offline only':'离线专用'}</span></div>
      ${calibrationTaskGuidance('dynamics')}
      <div class="calibration-workflow-steps">
        <div class="calibration-workflow-step ${missing ? 'current' : 'complete'}"><span>1</span>${en?'Load saved task':'加载任务'}</div>
        <div class="calibration-workflow-step ${!missing && !assessment ? 'current' : assessment ? 'complete' : ''}"><span>2</span>${en?'Assess data':'评估数据'}</div>
        <div class="calibration-workflow-step ${assessment && !result ? 'current' : result ? 'complete' : ''}"><span>3</span>${en?'Set target and identify':'选择目标并辨识'}</div>
        <div class="calibration-workflow-step ${result ? 'current' : ''}"><span>4</span>${en?'Review and export':'审核并导出'}</div>
      </div>
      <div class="workspace-actions"><button id="advanced-temp-restore-unconditional" class="button" ${!state.calibrationRuntimeCandidates?.dynamics_applied ? 'hidden' : ''} ${state.calibrationRuntimeCandidates?.dynamics_applied && workbenchConnected() && state.telemetry?.robot_state==='INACTIVE' ? '':'disabled'}>${en?'Undo temporary dynamics':'撤销临时动力学模型'}</button><button id="advanced-load-record" class="button">${t('loadRecord')}</button><button id="advanced-assess-record" class="button" ${!missing?'':'disabled'}>${en?'Assess saved data':'分析已有数据'}</button><span class="hint">${esc(status.task_id|| (en?'No record selected':'尚未选择任务'))}</span></div>
      ${assessment ? `<div class="calibration-capability-grid"><div>${en?'Static pose groups':'静态姿态组'}<b>${assessment.training_groups} / ${assessment.holdout_groups}</b><small>${en?'Train / holdout':'训练 / 留出'}</small></div><div>${en?'Dynamic samples':'有效动态帧'}<b>${assessment.valid_dynamic_samples}</b><small>${assessment.dynamic_precheck ? (en?'Precheck ready':'初步具备动态数据') : (en?'Insufficient paired replay':'双向激励可能不足')}</small></div></div>` : `<div class="calibration-muted-note">${en?'Load a completed task to evaluate static poses and bidirectional dynamic samples':'先加载已完成任务，检查静态姿态组和动态激励数据是否足够'}</div>`}
      <label class="calibration-advanced-target-label">${en?'Identification target':'辨识目标'}<select id="advanced-target">${options.map(([id,name])=>`<option value="${id}" ${state.advancedTarget===id?'selected':''}>${name}</option>`).join('')}</select></label>
      <div class="calibration-muted-note">${dynamic ? (en ? 'Requires bidirectional dynamic excitation, sufficient velocity and acceleration samples, and an independent holdout check' : '需要双向动态激励及足够的速度、加速度样本，辨识后还必须检查物理一致性与留出误差') : (en ? 'Uses gravity information from static pose groups; dynamic excitation is not required for this target' : '使用静态姿态组中的重力信息，不要求动态激励数据')}</div>
      <details class="calibration-locks"><summary>${en?'Lock trusted links':'锁定可信 Link'} · ${state.advancedLocked.length}</summary><div class="calibration-advanced-links">${lockList|| (en?'Load URDF / record first':'请先加载 URDF 或标定任务')}</div></details>
      <p class="hint">${en ? (state.advancedLocked.length ? 'Locked links will keep their original inertial data' : 'No locked links: all sampled links are eligible for fitting') : (state.advancedLocked.length ? '已锁定 Link 保留原始惯性参数' : '未锁定 Link，当前将尝试全局辨识')}</p><div class="callout">${limits}</div>
      <div class="calibration-advanced-next">${result?(en?'Candidate generated: review validation results, then export or reconnect to apply':'候选已生成，请先审核验证指标并导出成果，如需临时应用再连接工作台'):missing?(en?'Next: load a completed task':'下一步：加载已完成任务'):!assessment?(en?'Next: assess data':'下一步：分析已有数据'):workbenchConnected()?(en?'Next: safely disconnect to run offline':'下一步：安全停放并断开工作台，再离线辨识'):disable?(dynamic&&!assessment.dynamic_precheck?(en?'Dynamic data precheck did not pass. Select a static target or collect more data':'动态数据预检查未通过，请改选静态目标或重新采集双向激励数据'):!dynamic&&!assessment.gravity_ready?(en?'Static data is insufficient':'静态数据不足，需重新采集'):en?'Check trusted links and available targets':'请检查可信 Link 选择和可辨识范围'):(en?'Ready: review target and start offline identification':'数据具备初步条件，可确认目标并开始离线辨识')}</div><div class="workspace-actions"><button class="button primary" id="advanced-run" ${disable?'disabled':''}>${en?'Fit offline candidate for review':'离线辨识并生成审核候选'}</button>${workbenchConnected()?`<span class="hint">${en?'Park and disconnect the workspace first':'请先安全停放并断开工作台，再执行耗时离线计算'}</span>`:missing?`<span class="hint">${en?'Requires a saved completed task':'需要加载具有合法来源信息的历史任务记录'}</span>`:''}</div>
      ${result ? `<div class="callout"><b>${en?'Identification result':'高级辨识结果'} · ${esc(result.identification_mode)}</b><div><span class="result-badge ${result.status==='review_passed_offline'?'pass':'warn'}">${result.status==='review_passed_offline'?(en?'Passed offline review':'离线审核通过'):(en?'Needs review':'需要审核')}</span> ${en?'Rank':'数值秩'} ${Number(result.regressor_rank)}/${Number(result.regressor_columns)}</div><div>${en?'Holdout RMS':'留出 RMS'} ${Number(result.original_holdout_rms_nm).toFixed(4)} → ${Number(result.candidate_holdout_rms_nm).toFixed(4)} Nm</div><div class="path-chip">${esc(result.candidate_urdf||'')}</div>${(result.warnings||[]).map(w=>`<div>${esc(w)}</div>`).join('')}<div>${en?'Validation scope':'评价范围'} ${esc(result.validation_scope === 'static_gravity' ? (en?'Static gravity only':'仅静态重力') : (en?'Dynamic inertial model':'动态惯量'))}</div><div>${en?'Physical inertia check':'惯量物理一致性'} ${result.full_dynamics_physical_pass ? (en?'Passed':'通过') : (en?'Failed; do not apply':'未通过，不可应用')}</div><div>${en?'No automatic application or hardware validation':'未自动应用，也不代表已完成真机验证'}</div><div class="workspace-actions"><button id="advanced-temp-apply" class="button" ${result.status==='review_passed_offline'&&result.validation_scope==='dynamic_inertia'&&result.full_dynamics_physical_pass&&workbenchConnected()&&state.telemetry?.robot_state==='INACTIVE'?'':'disabled'}>${en?'Temporarily apply candidate (INACTIVE)':'临时应用惯量候选（需 INACTIVE）'}</button><button id="advanced-temp-restore" class="button" ${state.calibrationRuntimeCandidates?.dynamics_applied&&workbenchConnected()&&state.telemetry?.robot_state==='INACTIVE'?'':'disabled'}>${en?'Restore original model':'撤销并恢复原模型'}</button></div></div>`:''}
    </section>`;
}
async function assessLoadedCalibrationRecord(record) {
    // The CSV scan never competes with a running robot's control IPC
    if (!record?.directory || workbenchConnected()) return;
    try {
        const report = await api.request('model_calibration_assess', {directory: record.directory});
        if (state.modelCalibrationOffline?.directory !== record.directory) return;
        state.advancedAssessment = report;
        state.advancedProgress = '';
        if (state.page === 'workbench' && state.workbenchTab === 'calibration') render();
    } catch (error) {
        if (state.modelCalibrationOffline?.directory !== record.directory) return;
        state.advancedProgress = localizedError(String(error.message || error));
        if (state.page === 'workbench' && state.workbenchTab === 'calibration') render();
    }
}
function calibrationUnifiedResultsHtml() {
    const en = state.language === 'en';
    const record = currentModelCalibration() || {};
    const gravity = record.gravity_result || {};
    const assessment = state.advancedAssessment?.task_id === record.task_id ? state.advancedAssessment : null;
    const report = state.advancedReport?.task_id === record.task_id ? state.advancedReport : null;
    const stages = [
        [en?'Gravity model':'重力模型',!gravity.first_moments?.length?'neutral':gravity.static_pass?'pass':'warn',!gravity.first_moments?.length?(en?'Not run':'未执行'):gravity.static_pass?(en?'Static validation passed':'静态验证通过'):(en?'Needs review':'需要审核')],
        [en?'Friction model':'摩擦模型',!record.friction&&!record.friction_pass?'neutral':record.friction_status==='requires_refit_or_review'?'warn':record.friction_pass?'pass':'warn',record.friction_status==='requires_refit_or_review'?(en?'Gravity source changed':'重力来源不匹配'):record.friction_pass?(en?'Verified':'已通过'):(en?'Not verified':'尚未验证')],
        [en?'Dynamic inertias':'动态惯量',report?.status==='review_passed_offline'?'warn':'neutral',report?(en?'Offline candidate':'离线候选'):(assessment?.dynamic_precheck?(en?'Data may be sufficient':'待离线辨识'):(en?'Unverified':'尚未验证'))],
    ];
    return `<section class="workspace-section calibration-results-card"><h3>${en?'Calibration records and candidates':'标定记录与候选结果'}</h3><div class="calibration-result-stages">${stages.map(([name,tone,label])=>`<div><b>${name}</b><span class="result-badge ${tone}">${label}</span></div>`).join('')}</div><p class="hint">${en?'Pass / review / insufficient data / not run are separate statuses. Candidates never overwrite the live model without explicit approval':'已通过、需要审核、信息不足和未执行分别记录 · 离线候选不会自动替换运行中的模型'}</p>${assessment?`<p class="hint">${en?'Data precheck':'数据质量预评估'} · ${assessment.training_groups} / ${assessment.holdout_groups} ${en?'static poses':'静态姿态'} · ${assessment.valid_dynamic_samples} ${en?'potential dynamic frames':'动态候选帧'} · ${en?'excitation observability not yet validated':'惯量可辨识性仍需正式计算'}</p>`:''}${state.advancedProgress?`<p class="workspace-error">${esc(state.advancedProgress)}</p>`:''}${record.directory?`<div class="task-directory">${esc(record.directory)}</div>`:''}</section>`;
}
const calibrationToolExportLabels = {
    teaching: ['导出示教轨迹数据包', 'Export teaching trajectory'],
    gain: ['导出增益标定结果', 'Export calibrated gains'],
    gravity: ['导出重力模型校正结果', 'Export gravity correction'],
    dynamics: ['导出高级动力学辨识结果', 'Export dynamics identification'],
};
const calibrationToolDefinitions = [
    ['teaching','拖拽示教','Teaching','轨迹采集与复用','Record and reuse trajectories'],
    ['gain','自适应增益标定','Adaptive gains','刚性、柔性和辨识控制候选','Rigid, compliant and identification controllers'],
    ['gravity','重力模型校正','Gravity correction','全局 / 局部 · 静态采样验证','Global / local gravity'],
    ['dynamics','高级动力学辨识','Advanced dynamics','惯性参数与离线验证','Inertial identification'],
];
function calibrationToolHistoryHtml(kind) {
    const selected=state.calibrationToolSelected[kind];
    const records=state.calibrationToolRecords[kind]||[];
    const recent=state.telemetry?.model_calibration || {};
    const canExport=!!selected || (kind==='dynamics' && !!state.advancedReport?.candidate_urdf) ||
      (kind==='gravity' && !!state.modelCalibrationOffline?.result?.gravity_result) ||
      ((kind==='teaching'||kind==='gain'||kind==='gravity') && recent.workflow_kind===kind && !!recent.directory);
    const en=state.language==='en';
    return `<section class="workspace-section calibration-tool-history">
      <div class="workspace-section-head"><div><h3>${en?'Task records':'任务记录'}</h3><p class="hint">${en?'Records can be reused across tools; no automatic robot movement':'四个工具共享来源数据，加载记录不会导致机械臂运动'}</p></div></div>
      <div class="workspace-actions"><button class="button" data-tool-scan="${kind}">${en?'Scan saved tasks':'扫描已有任务'}</button><button class="button" data-tool-load="${kind}">${en?'Load task folder':'加载任务记录'}</button><button class="button primary" data-tool-export="${kind}" ${canExport?'':'disabled'}>${calibrationToolExportLabels[kind][en?1:0]}</button></div>
      ${selected?`<div class="callout"><b>${esc(selected.task_id||'')}</b><div>${esc(selected.directory)}</div><div>${esc(selected.note||'')}</div></div>`:''}
      ${records.length?`<div class="calibration-tool-list">${records.map((r,i)=>`<button class="button calibration-tool-record" data-tool-select="${kind}" data-tool-index="${i}"><b>${esc(r.task_id||'')}</b><small>${esc(r.record_kind||'')} · ${esc(r.trajectory?.samples??0)} ${en?'frames':'帧'}</small></button>`).join('')}</div>`: `<p class="hint">${en?'Scan to view saved data for this tool':'点击“扫描已有任务”显示本工具可用的记录'}</p>`}
    </section>`;
}
async function selectCalibrationToolRecord(kind,selection) {
    if(!selection?.directory)throw new Error('请选择有效任务');
    state.calibrationToolSelected[kind]=selection;
    if(kind==='gravity'||kind==='dynamics') {
        const record=await api.request('model_calibration_load',{directory:selection.directory});
        adoptGravityRecord(record);
        state.modelCalibrationDirectory=record.directory;
        state.advancedReport=null;
        state.advancedAssessment=null;
        state.advancedLocked=advancedTrustedLinks(record);
        if(kind==='dynamics' && selection.dynamic_candidates?.length) {
            const item=selection.dynamic_candidates[0];
            // Loaded candidate remains explicitly review-only until Core
            // checks its physical report, model hash and robot lifecycle.
            state.advancedReport={...item,task_id:record.task_id};
        }
    }
    render();
}
// Read the completed candidate automatically; task records never change motor state
function syncCompletedGainCandidate(calibration = state.telemetry?.model_calibration) {
    if (!workbenchConnected() || calibration?.workflow_kind !== 'gain' ||
        calibration?.phase !== 'complete' || !calibration.directory) return;
    const directory = calibration.directory;
    if (state.calibrationToolSelected.gain?.directory === directory ||
        state.gainCandidateLoadPending === directory || state.gainCandidateLoadAttempted === directory) return;
    state.gainCandidateLoadPending = directory;
    api.request('calibration_tool_load', {kind: 'gain', directory}).then(record => {
        if (state.telemetry?.model_calibration?.directory !== directory) return;
        state.calibrationToolSelected.gain = record;
        state.gainCandidateLoadError = '';
        if (state.page === 'workbench' && state.workbenchTab === 'calibration' && state.calibrationEntry === 'gain') refreshCalibrationOrDefer();
    }).catch(error => {
        state.gainCandidateLoadError = error.message || String(error);
        if (state.page === 'workbench' && state.workbenchTab === 'calibration' && state.calibrationEntry === 'gain') refreshCalibrationOrDefer();
    }).finally(() => {
        state.gainCandidateLoadPending = '';
        state.gainCandidateLoadAttempted = directory;
    });
}
async function temporaryCandidateAction(action,kind) {
    return blockingOperation(busyText('正在安全切换临时参数...', 'Switching candidate safely...'),async()=>{
        if(!workbenchConnected())throw new Error('请先连接工作台');
        if(state.telemetry?.robot_state!=='INACTIVE')throw new Error('临时应用与撤销需要先安全停放机械臂（INACTIVE）');
        const preview=kind==='gravity'?gravityCandidatePreviewState():null;
        const directory=preview?.directory||state.calibrationToolSelected[kind]?.directory||state.modelCalibrationDirectory;
        if(action==='apply'&&!directory)throw new Error('请选择候选任务');
        if(action==='apply'&&kind==='gravity'&&!preview.canApply)
            throw new Error('重力候选未通过静态验收，或当前模型不匹配，请检查任务和机械臂状态');
        const report=kind==='dynamics' ? (state.advancedReport?.candidate_urdf ? `${state.advancedReport.candidate_urdf.replace(/\/[^/]+$/, '')}/identification-report.json` : '') : '';
        if(action==='apply'&&kind==='dynamics'&&!report)throw new Error('请先运行高级惯量辨识并获得离线审核通过的候选');
        state.calibrationRuntimeCandidates=await api.request('calibration_candidate_'+action,{
            kind,directory,report,prefer_recomputed:kind==='gravity'&&preview?.offline===true
        });
        render();
    });
}
function calibrationStandalonePanelHtml(kind) {
    const en=state.language==='en';
    const status=currentModelCalibration()||{};
    const phase=status.phase||'idle';
    const live=workbenchConnected() && !state.modelCalibrationOffline;
    const robotActive=state.telemetry?.robot_state==='ACTIVE';
    const selected=state.calibrationToolSelected[kind];
    const isTeach=kind==='teaching';
    const waiting=live && phase==='waiting_replay_confirmation';
    const active=live && modelCalibrationActive(status);
    const buttons=[];
    if(live && ['idle','complete','cancelled','failed'].includes(phase) && isTeach)
        buttons.push(`<button id="model-calibration-teach-start" class="button primary" ${robotActive?'':'disabled'}>${en?'Start demonstration':'开始拖拽示教'}</button>`);
    if(live && phase==='teaching' && isTeach)
        buttons.push(`<button id="model-calibration-teach-stop" class="button primary">${robotActive?(en?'Stop demonstration':'结束示教'):(en?'Close interrupted demonstration':'结束中断的示教并清理')}</button>`);
    if(live && isTeach && status.teaching_cleanup_pending && ['failed','cancelled'].includes(phase))
        buttons.push(`<button id="model-calibration-teach-cleanup" class="button">${en?'Finish interrupted teaching cleanup':'清理中断示教'}</button>`);
    if(isTeach && waiting)
        buttons.push(`<button id="calibration-finish-teaching" class="button primary">${en?'Save trajectory and finish':'保存轨迹并结束任务'}</button>`);
    if(kind==='gain' && waiting)
        buttons.push(`<button id="model-calibration-replay-confirm" class="button danger" ${status.source_task_id && !calibrationAlignment(status).aligned?'disabled':''}>${en?'Confirm gain adaptation':'确认开始增益标定'}</button>`);
    if(kind==='gain' && selected && live && ['idle','complete','failed','cancelled'].includes(phase))
        buttons.push(`<button id="calibration-tool-import" class="button primary" ${robotActive?'':'disabled'} title="${robotActive?'':(en?'Activate the robot before importing the trajectory':'请先使能机械臂再导入轨迹')}">${en?'Import selected trajectory (no motion)':'导入所选示教轨迹（不运动）'}</button>`);
    if(kind==='gain' && live && status.workflow_kind==='gain' && status.directory && ['complete','failed'].includes(phase))
        buttons.push(`<button id="calibration-gain-load-live" class="button">${en?'Review this gain task':'查看本次增益候选'}</button>`);
    if(kind==='gain' && waiting && status.source_task_id)
        buttons.push(status.alignment_active ? `<button id="model-calibration-alignment-finish" class="button">${en?'Finish guided alignment':'完成引导对齐'}</button>`:
            `<button id="model-calibration-alignment-begin" class="button">${en?'Guided alignment':'引导对齐回放起点'}</button>`);
    if(active && kind==='gain')
        buttons.push(`<button id="model-calibration-cancel" class="button danger">${en?'Cancel current task':'取消当前任务'}</button>`);
    const gainCandidate=(selected?.gain_candidate || null);
    const isInactive=state.telemetry?.robot_state==='INACTIVE';
    const gainValidated=gainCandidateDeployable(selected?.gain_candidate);
    const gainApplied=!!state.calibrationRuntimeCandidates?.gain_applied;
    const gainCanApply=gainValidated && isInactive && workbenchConnected() && !gainApplied;
    let gainActionHint='';
    if(kind==='gain') {
        buttons.push(`<button id="calibration-gain-temp-apply" class="button" ${gainCanApply?'':'disabled'} title="${esc(!workbenchConnected()?(en?'Connect the workspace first':'请先连接工作台'):!gainValidated?(en?'Load a verified candidate first':'请先加载通过验收的候选'):gainApplied?(en?'Restore the existing temporary candidate first':'请先撤销当前临时增益'):!isInactive?(en?'Park and disable the robot first':'请先停放并失能机械臂'):'')}">${en?'Temporarily apply validated gain':'临时应用已验收增益'}</button>`);
        buttons.push(`<button id="calibration-gain-temp-restore" class="button" ${gainApplied && isInactive?'':'disabled'}>${en?'Undo temporary gains':'撤销增益临时应用'}</button>`);
        if(gainValidated && workbenchConnected() && !isInactive) {
            gainActionHint=en?'Candidate passed, but runtime gain changes require an INACTIVE robot. Confirm the workspace is clear and park the robot first':'候选已通过验收，但临时应用要求机械臂处于 INACTIVE 状态，请确认机械臂已被安全支撑及周围环境无障碍后先停放并失能';
        } else if(!gainValidated && status.workflow_kind==='gain' && status.phase==='complete') {
            gainActionHint=state.gainCandidateLoadPending?(en?'Loading verified candidate':'正在自动加载已验收候选'):
                (state.gainCandidateLoadError || (en?'Please review this completed task to load its candidate':'正在读取候选记录，必要时点击查看本次增益候选'));
        } else if(gainValidated && gainApplied) {
            gainActionHint=en?'A temporary gain candidate is already active. Park the robot before restoring it':'临时增益已应用，恢复原参数前请先停放机械臂';
        }
    }
    return `<section class="workspace-section calibration-standalone-card"><div class="workspace-section-head"><div><h3>${isTeach?(en?'Demonstration':'拖拽示教'):(en?'Adaptive gain calibration':'自适应增益标定')}</h3><p class="hint">${isTeach ? (en?'Capture once, reuse across calibration tools':'只记录安全示教轨迹，不强制运行辨识') : (en?'Rigid and identification: frozen full-speed validation · Compliant: lower-stiffness search, 65% validation and stationary hold check · two round trips each':'刚性与辨识采用全速冻结验收 · 柔性以较低刚度搜索、65% 速度验收并检查静态保持 · 每项两次往返')}</p></div><span class="result-badge neutral">${esc(modelCalibrationPhaseText(phase))}</span></div>
       <div class="workspace-actions">${buttons.join('')}</div>
       ${kind==='gain'?`<label>${en?'Gain target':'标定目标'} <select id="gain-target"><option value="rigid" ${state.gainMode==='rigid'?'selected':''}>${en?'Rigid gains':'刚性增益'}</option><option value="compliant" ${state.gainMode==='compliant'?'selected':''}>${en?'Compliant gains':'柔性增益'}</option><option value="identification" ${state.gainMode==='identification'?'selected':''}>${en?'Identification gains':'辨识增益'}</option><option value="all" ${state.gainMode==='all'?'selected':''}>${en?'All gains':'全部增益'}</option></select></label><div class="callout">${en?({rigid:'Rigid gains are used for precise trajectory tracking and high-stiffness position control',compliant:'Compliant calibration uses rigid-relative stiffness limits, 65% replay validation and a stationary hold check · external-force compliance is not measured',identification:'Identification gains are dedicated to repeatable calibration playback and dynamics data collection',all:'Calibrate rigid, compliant and identification sequentially · compliant uses 65% validation and hold check · drag stays manual'}[state.gainMode]):({rigid:'刚性增益用于精确轨迹执行和较高刚度的位置跟踪',compliant:'柔性增益相对刚性限制刚度和阻尼，以 65% 速度验收跟踪并检查静态保持 · 不代表已测量外力柔顺性',identification:'辨识增益专用于标定轨迹回放、重力模型校正和动力学数据采集',all:'依次标定刚性、柔性与辨识增益，共六次往返 · 柔性按较低速度及保持稳定性独立验收 · 柔性拖拽仅手动修改'}[state.gainMode])}${state.gainMode==='all' && active?`<div>${en?'Current target':'当前标定'}: ${status.gain_active_target==='compliant'?(en?'Compliant gains':'柔性增益'):status.gain_active_target==='identification'?(en?'Identification gains':'辨识增益'):(en?'Rigid gains':'刚性增益')} · ${Number(status.gain_adaptation_passes||0)}/12</div>`:''}${state.gainMode==='all'?`<div>${en?'Rigid → rigid hold + rigid tracking · Compliant → compliant hold + compliant tracking · Identification → identification tracking · compliant drag unchanged':'刚性 → 刚性保持与刚性跟踪 · 柔性 → 柔性保持与柔性跟踪 · 辨识 → 辨识跟踪 · 柔性拖拽不变'}</div>`:''}${!calibrationTaskLoaded('gain') ? calibrationTaskGuidance('gain') : ''}${gainCandidate?`<div>${en?'Frozen mode-specific verification':'冻结增益分类验收'}: ${gainCandidate.validated?'PASS':'NOT YET'} · ${Number(gainCandidate.passes||0)} ${en?'passes':'次单程'}${gainCandidate?` · ${gainVerificationDescription(gainCandidate,en)}`:''}${gainCandidate.verification?` · ${en?'Mean':'平均'} ${Number(gainCandidate.verification.mean_ratio||0).toFixed(3)} · ${en?'Peak':'最大'} ${Number(gainCandidate.verification.worst_ratio||0).toFixed(3)} · ${en?'Pauses':'暂停'} ${gainCandidate.verification.pauses} · ${en?'Revisions':'修改'} ${gainCandidate.verification.revisions}`:''}</div>`:''}</div>`:''}
       ${kind==='gain'?`<div id="calibration-gain-action-hint" class="callout" ${gainActionHint?'':'hidden'}>${esc(gainActionHint)}</div>`:''}
       ${status.error?`<div class="callout error-callout">${esc(localizedError(status.error))}</div>`:''}
       ${status.directory?`<div class="task-directory">${esc(status.directory)}</div>`:''}
       </section>${calibrationToolHistoryHtml(kind)}`;
}
function refreshGainApplyControls() {
    if (state.page !== 'workbench' || state.workbenchTab !== 'calibration' || state.calibrationEntry !== 'gain') return;
    const apply = $('calibration-gain-temp-apply');
    const restore = $('calibration-gain-temp-restore');
    const hint = $('calibration-gain-action-hint');
    if (!apply || !restore || !hint) return;
    const en = state.language === 'en';
    const connected = workbenchConnected();
    const inactive = state.telemetry?.robot_state === 'INACTIVE';
    const validated = gainCandidateDeployable(state.calibrationToolSelected.gain?.gain_candidate);
    const applied = !!state.calibrationRuntimeCandidates?.gain_applied;
    const status = state.telemetry?.model_calibration || {};
    apply.disabled = !(connected && inactive && validated && !applied);
    restore.disabled = !(connected && inactive && applied);
    const reason = !connected ? (en?'Connect the workspace first':'请先连接工作台')
        : !validated ? (en?'Load a verified candidate first':'请先加载通过验收的候选')
        : applied ? (en?'Restore the existing temporary candidate first':'请先撤销当前临时增益')
        : !inactive ? (en?'Park and disable the robot first':'请先停放并失能机械臂') : '';
    apply.title = reason;
    const message = connected && validated && !inactive
        ? (en?'Candidate passed, but runtime gain changes require an INACTIVE robot. Confirm the workspace is clear and park the robot first':'候选已通过验收，但临时应用要求机械臂处于 INACTIVE 状态，请确认机械臂已被安全支撑及周围环境无障碍后先停放并失能')
        : !validated && status.workflow_kind === 'gain' && status.phase === 'complete'
          ? (state.gainCandidateLoadPending ? (en?'Loading verified candidate':'正在自动加载已验收候选')
             : state.gainCandidateLoadError || (en?'Please review this completed task to load its candidate':'正在读取候选记录，必要时点击查看本次增益候选'))
        : validated && applied ? (en?'A temporary gain candidate is already active. Park the robot before restoring it':'临时增益已应用，恢复原参数前请先停放机械臂') : '';
    hint.textContent = message;
    hint.hidden = !message;
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
function calibrationQuickControlsHtml() {
    const en = state.language === 'en';
    const connected = workbenchConnected();
    const robotState = connected ? (state.telemetry?.robot_state || 'UNKNOWN') : 'OFFLINE';
    const inactive = robotState === 'INACTIVE';
    const active = robotState === 'ACTIVE';
    const phase = state.telemetry?.model_calibration?.phase || 'idle';
    const busy = modelCalibrationActive();
    const safeEnable = ['idle', 'complete', 'cancelled', 'failed', 'waiting_replay_confirmation'].includes(phase);
    const runningTask = !safeEnable || robotState === 'FAULT';
    const hint = workbenchRecoveryRequired() ? (en ? 'Control session unresponsive, physically secure the robot before reconnecting' : '控制会话无响应，请先确保机械臂硬件安全，再恢复连接')
        : !connected ? (en ? 'Connect the workspace to control the robot from calibration' : '请先连接工作台，连接后可在标定页面直接使能或停放机械臂')
        : robotState === 'FAULT' ? (en ? 'Check motor power and cabling, then clear fault or disable here' : '机械臂存在故障，请先检查电机供电及线缆，再清除故障或失能')
        : busy ? (en ? 'A calibration task is in progress. Parking may cancel the current replay' : '标定任务正在执行，停放可能中止当前采集或回放')
        : inactive ? (en ? 'Robot is parked. Activate only after checking the work area' : '机械臂已失能，确认工作区域安全后可使能')
        : active ? (en ? 'Robot is enabled. Park and disable before switching temporary parameters' : '机械臂已使能，切换临时参数前请先停放并失能')
        : (en ? 'Waiting for robot status' : '等待机械臂状态反馈');
    return `<section class="calibration-quick-bar" aria-label="${en ? 'Robot controls' : '标定控制'}">
      <div class="calibration-quick-state"><span class="calibration-state-dot ${connected && active ? 'on' : connected && inactive ? 'standby' : ''}"></span><strong>${en ? 'Robot' : '机械臂状态'}</strong><span id="calibration-robot-state">${esc(robotState)}</span></div>
      <div class="calibration-quick-actions">
        <button type="button" class="button" id="calibration-connect-workbench" ${connected ? 'hidden' : ''}>${t('startWorkbench')}</button>
        <button type="button" class="button primary" id="calibration-activate" ${connected && inactive && !runningTask && !workbenchRecoveryRequired() ? '' : 'disabled'}>${t('activateRobot')}</button>
        <button type="button" class="button" id="calibration-safe-park" ${connected && active && !workbenchRecoveryRequired() ? '' : 'disabled'}>${t('parkRobot')}</button>
        <button type="button" class="button" id="calibration-fault-clear" ${connected && robotState === 'FAULT' && !workbenchRecoveryRequired() ? '' : 'hidden'}>${t('clearFault')}</button>
        <button type="button" class="button danger" id="calibration-fault-disable" ${connected && robotState === 'FAULT' && !workbenchRecoveryRequired() ? '' : 'hidden'}>${t('deactivateRobot')}</button>
      </div>
      <p id="calibration-quick-hint" class="hint calibration-quick-hint">${hint}</p>
    </section>`;
}
// Refresh controls that depend on live robot state without replacing the panel or dialogs
function refreshCalibrationActionControls() {
    if (state.page !== 'workbench' || state.workbenchTab !== 'calibration') return;
    const connected = workbenchConnected() && !workbenchRecoveryRequired();
    const robotState = connected ? state.telemetry?.robot_state : 'OFFLINE';
    const active = robotState === 'ACTIVE';
    const inactive = robotState === 'INACTIVE';
    const live = !state.modelCalibrationOffline;
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
    setButton('model-calibration-replay-confirm', connected && active && live && phase === 'waiting_replay_confirmation'
        && (!state.telemetry?.model_calibration?.source_task_id || calibrationAlignment(state.telemetry.model_calibration).aligned),
        en ? 'Check robot activation and replay alignment first' : '请先使能机械臂并检查回放起点对齐状态');
    setButton('model-calibration-apply', connected && inactive && live && !modelCalibrationActive(),
        en ? 'Park the robot before applying a candidate' : '应用候选前请先停放并失能机械臂');
    setButton('model-calibration-restore', connected && inactive && live && !modelCalibrationActive(),
        en ? 'Park the robot before restoring a candidate' : '撤销候选前请先停放并失能机械臂');
    if (state.calibrationEntry === 'gravity') {
        const gravity=gravityCandidatePreviewState();
        const message=!gravity.directory?(en?'Load a gravity task first':'请先加载重力校正任务')
            : !gravity.validated?(en?'Independent static validation has not passed':'尚未通过独立静态验收')
            : !gravity.comparable?(en?'Source URDF differs from the running model':'源 URDF 与当前模型不一致，不能临时应用')
            : gravity.historical?(en?'Historical COM comparison is read-only, the live URDF fingerprint differs':'历史候选仅供只读对照，当前 URDF 指纹不同，不能临时应用')
            : !connected?(en?'Connect the workspace first':'请先连接工作台')
            : !inactive?(en?'Park and disable the robot first':'请先停放并失能机械臂')
            : gravity.applied?(en?'A temporary gravity override is already applied':'当前已有临时重力覆盖') : '';
        setButton('calibration-gravity-temp-apply',gravity.canApply,message);
        setButton('calibration-gravity-temp-restore',gravity.applied&&connected&&inactive,
            en?'Park the robot to restore the original gravity':'请先停放并失能机械臂，再撤销临时重力');
        const hint=$('calibration-gravity-preview-hint');
        if (hint) { hint.textContent=message; hint.hidden=!message; }
    }
}
function refreshCalibrationQuickControls() {
    if (state.page !== 'workbench' || state.workbenchTab !== 'calibration') return;
    const connect = $('calibration-connect-workbench');
    const enable = $('calibration-activate');
    const park = $('calibration-safe-park');
    const status = $('calibration-robot-state');
    const hint = $('calibration-quick-hint');
    if (!enable || !park || !status || !hint) return;
    const connected = workbenchConnected();
    const robotState = connected ? (state.telemetry?.robot_state || 'UNKNOWN') : 'OFFLINE';
    if (connect) connect.hidden = connected;
    enable.disabled = !(connected && !workbenchRecoveryRequired() && robotState === 'INACTIVE' && ['idle', 'complete', 'cancelled', 'failed', 'waiting_replay_confirmation'].includes(state.telemetry?.model_calibration?.phase || 'idle'));
    park.disabled = !(connected && !workbenchRecoveryRequired() && robotState === 'ACTIVE');
    for (const [id] of [['calibration-fault-clear'], ['calibration-fault-disable']]) { const button = $(id); if (button) button.hidden = !connected || robotState !== 'FAULT' || workbenchRecoveryRequired(); }
    status.textContent = robotState;
    const dot = document.querySelector('.calibration-state-dot');
    if (dot) dot.className = 'calibration-state-dot' + (connected && robotState === 'ACTIVE' ? ' on' : connected && robotState === 'INACTIVE' ? ' standby' : '');
    hint.textContent = workbenchRecoveryRequired() ? busyText('控制会话无响应，请先采取物理安全措施，再使用页面上方的恢复连接', 'Control session not responding, secure hardware then recover connection')
        : !connected ? busyText('请先连接工作台，连接后可在标定页面直接使能或停放机械臂', 'Connect the workspace to control the robot from calibration')
        : robotState === 'FAULT' ? busyText('机械臂存在故障，请检查电机供电和线缆，可在这里清除故障或失能', 'Check motor power and wiring, then clear fault or disable here')
        : modelCalibrationActive() ? busyText('标定任务正在执行，停放可能中止当前采集或回放', 'A calibration task is in progress. Parking may cancel the current replay')
        : robotState === 'INACTIVE' ? busyText('机械臂已失能，确认工作区域安全后可使能', 'Robot is parked. Activate only after checking the work area')
        : robotState === 'ACTIVE' ? busyText('机械臂已使能，切换临时参数前请先停放并失能', 'Robot is enabled. Park before switching temporary parameters')
        : busyText('等待机械臂状态反馈', 'Waiting for robot status');
}
function calibrationPanelHtml() {
    const en=state.language==='en';
    const locked=modelCalibrationActive();
    const entry=state.calibrationEntry;
    return `<div class="workspace-panel calibration-panel">${calibrationQuickControlsHtml()}<div class="calibration-entry-grid">${calibrationToolDefinitions.map(([key,zh,enName,zhDesc,enDesc])=>`<button data-calibration-entry="${key}" type="button" class="calibration-entry ${entry===key?'selected':''}" ${locked&&entry!==key?'disabled':''}><span class="calibration-entry-title">${en?enName:zh}</span><span>${en?enDesc:zhDesc}</span></button>`).join('')}</div>
      ${(state.calibrationRuntimeCandidates?.gain_applied||state.calibrationRuntimeCandidates?.gravity_applied||state.calibrationRuntimeCandidates?.dynamics_applied)?`<section class="workspace-section"><h3>${en?'Active temporary candidates':'当前临时覆盖'}</h3><p class="hint">${en?'Disconnect parks and removes all temporary overrides. Manual undo requires INACTIVE; saved config is unchanged':'断连将安全停放并撤销所有临时覆盖，原配置不变；手动撤销也需要先停放至 INACTIVE'}</p><div class="workspace-actions">${[['gain','gain_applied',en?'Gains':'增益'],['gravity','gravity_applied',en?'Gravity':'重力'],['dynamics','dynamics_applied',en?'Dynamics':'动力学']].filter(([key,flag])=>state.calibrationRuntimeCandidates?.[flag]).map(([key,flag,name])=>`<button class="button" data-candidate-undo="${key}" ${workbenchConnected()&&state.telemetry?.robot_state==='INACTIVE'?'':'disabled'}>${en?'Undo ':'撤销'}${name}</button>`).join('')}</div></section>`:''}
      ${entry==='teaching'?calibrationStandalonePanelHtml('teaching'):
        entry==='gain'?calibrationStandalonePanelHtml('gain'):
        entry==='gravity'?modelCalibrationPanelHtml()+calibrationToolHistoryHtml('gravity')+gravityCandidatePreviewPanelHtml():
        advancedIdentificationPanelHtml()+calibrationToolHistoryHtml('dynamics')}
      ${entry==='gravity'||entry==='dynamics'?calibrationUnifiedResultsHtml():''}
      ${entry==='gravity'?singleCalibrationToolsHtml():''}
    </div>`;
}

function matrixHtml(matrix) { if (!Array.isArray(matrix)) return '—'; return `<table class="matrix-table">${matrix.map(row => `<tr>${row.map(v => `<td>${Number(v).toFixed(4)}</td>`).join('')}</tr>`).join('')}</table>`; }
function diagnosticsPanelHtml() { const x = state.telemetry || {}; const names = x.joint_names || []; const frames = x.frames || []; if (!state.selectedFrame && frames.length) state.selectedFrame = frames[0].name; const frame = frames.find(f => f.name === state.selectedFrame) || frames[0]; const yes = state.language === 'en' ? 'yes' : '是'; const no = state.language === 'en' ? 'no' : '否'; return `<div class="workspace-panel"><section class="workspace-section"><h3>${t('jointActuatorTitle')}</h3><div class="table-scroll"><table class="workspace-table"><thead><tr><th>${t('joint')}</th><th>q</th><th>dq</th><th>τ</th><th>ref</th><th>τmodel</th><th>residual</th><th>τext</th></tr></thead><tbody>${names.map((name, i) => `<tr><td>${esc(name)}</td><td>${Number(x.joint?.pos?.[i] ?? 0).toFixed(4)}</td><td>${Number(x.joint?.vel?.[i] ?? 0).toFixed(4)}</td><td>${Number(x.joint?.tor?.[i] ?? 0).toFixed(4)}</td><td>${Number(x.joint?.ref_pos?.[i] ?? 0).toFixed(4)}</td><td>${Number(x.joint?.model_feedforward?.[i] ?? 0).toFixed(4)}</td><td>${Number(x.joint?.residual_raw?.[i] ?? 0).toFixed(4)}</td><td>${Number(x.joint?.tau_ext_hat?.[i] ?? 0).toFixed(4)}</td></tr>`).join('')}</tbody></table></div><div class="table-scroll"><table class="workspace-table"><thead><tr><th>Actuator</th><th>${t('onlineLabel')}</th><th>${t('enabledLabel')}</th><th>${t('errorLabel')}</th><th>q</th><th>dq</th><th>τ</th></tr></thead><tbody>${(x.actuator_info || []).map((a, i) => `<tr><td>${esc(a.name)}</td><td>${x.actuator?.online?.[i] ? yes : no}</td><td>${x.actuator?.enabled?.[i] ? yes : no}</td><td>${esc(x.actuator?.err_code?.[i] ?? '—')}</td><td>${Number(x.actuator?.pos?.[i] ?? 0).toFixed(4)}</td><td>${Number(x.actuator?.vel?.[i] ?? 0).toFixed(4)}</td><td>${Number(x.actuator?.tor?.[i] ?? 0).toFixed(4)}</td></tr>`).join('')}</tbody></table></div></section><section class="workspace-section"><h3>${t('telemetryTitle')}</h3><canvas id="workbench-chart" height="180"></canvas></section><section class="workspace-section"><div class="workspace-section-head"><h3>${t('frameTitle')}</h3><select id="diagnostic-frame">${frames.map(f => `<option ${f.name === state.selectedFrame ? 'selected' : ''}>${esc(f.name)}</option>`).join('')}</select></div><dl class="kv"><dt>${t('positionLabel')}</dt><dd>${esc(vectorText(frame?.position))}</dd><dt>${t('quaternionLabel')}</dt><dd>${esc(vectorText(frame?.quaternion))}</dd></dl></section><section class="workspace-section"><h3>${t('dynamicsTitle')}</h3><dl class="kv"><dt>${t('gravityLabel')}</dt><dd>${esc(vectorText(x.dynamics?.gravity))}</dd><dt>${t('gravityCompLabel')}</dt><dd>${esc(vectorText(x.dynamics?.gravity_compensation))}</dd><dt>${t('coriolisLabel')}</dt><dd>${esc(vectorText(x.dynamics?.coriolis))}</dd><dt>${t('inverseDynamicsLabel')}</dt><dd>${esc(vectorText(x.dynamics?.inverse_dynamics))}</dd><dt>COM</dt><dd>${esc(vectorText(x.dynamics?.center_of_mass))}</dd></dl>${matrixHtml(x.dynamics?.mass_matrix)}</section></div>`; }
function workbenchPanelHtml() { if (state.workbenchTab === 'tuning') return tuningPanelHtml(); if (state.workbenchTab === 'calibration') return calibrationPanelHtml(); if (state.workbenchTab === 'diagnostics') return diagnosticsPanelHtml(); return controlPanelHtml(); }
function gravityCandidatePreviewPanelHtml() {
    const en=state.language==='en';
    const candidate=gravityCandidatePreviewState();
    const viewSource=candidate.offline?(en?'Offline mass and COM candidate':'离线质量与质心候选'):(en?'Online COM candidate':'在线质心候选');
    const reason=!candidate.directory?(en?'Load a gravity task first':'请先加载重力校正任务')
        : !candidate.validated?(en?'Independent static validation has not passed':'尚未通过独立静态验收')
        : !candidate.comparable?(en?'Source URDF differs from the running model':'源 URDF 与当前模型不一致，不能临时应用')
        : candidate.historical?(en?'Historical COM comparison is read-only, the live URDF fingerprint differs':'历史候选仅供只读对照，当前 URDF 指纹不同，不能临时应用')
        : !workbenchConnected()?(en?'Connect the workspace first':'请先连接工作台')
        : state.telemetry?.robot_state!=='INACTIVE'?(en?'Park and disable the robot first':'请先停放并失能机械臂')
        : candidate.applied?(en?'A temporary gravity override is already applied':'当前已有临时重力覆盖') : '';
    const description=candidate.offline
        ? (en?'COM markers on the left show the offline result. A temporary gravity-only preview changes gravity first moments, not runtime masses or inertias. It never writes the original URDF or YAML'
            :'左侧质心标记显示离线候选 · 临时应用仅替换重力一阶矩，不修改运行时质量或惯性张量，也不覆盖原始 URDF 和 YAML')
        : (en?'The left model shows the candidate COM. Temporary application changes gravity only and leaves the original files untouched'
            :'左侧模型显示候选质心 · 临时应用只调整重力项，不覆盖原始文件');
    return `<section class="workspace-section"><div class="workspace-section-head"><div><h3>${en?'Gravity candidate preview':'重力候选预览'}</h3><p class="hint">${viewSource}</p></div><span class="result-badge ${candidate.validated?'pass':'warn'}">${candidate.validated?(en?'Static check passed':'静态评价通过'):(en?'Needs review':'尚待审核')}</span></div><div class="calibration-muted-note">${description}</div><div class="workspace-actions"><button id="calibration-gravity-temp-apply" class="button" ${candidate.canApply?'':'disabled'} title="${esc(reason)}">${en?'Apply gravity-only preview (INACTIVE)':'临时应用候选重力项'}</button><button id="calibration-gravity-temp-restore" class="button" ${candidate.applied&&workbenchConnected()&&state.telemetry?.robot_state==='INACTIVE'?'':'disabled'}>${en?'Undo temporary gravity':'撤销重力临时应用'}</button></div>${reason?`<p id="calibration-gravity-preview-hint" class="hint">${esc(reason)}</p>`:`<p id="calibration-gravity-preview-hint" class="hint" hidden></p>`}</section>`;
}
function calibrationComOverlayHtml() {
    if (state.workbenchTab !== 'calibration') return '';
    const binding = modelCalibrationModelBinding();
    const data = currentModelCalibration()?.gravity_result;
    const hasComparison = binding.comparable && Array.isArray(data?.first_moments) && data.first_moments.length > 0;
    const scale = state.modelCalibrationDisplayScale || 1;
    const title = state.language === 'en' ? 'Gravity COM comparison' : '重力质心对比';
    const pending = state.language === 'en' ? 'Available once a model candidate is fitted' : '完成重力候选拟合后自动显示';
    const original = binding.historical ? (state.language === 'en' ? 'Current URDF COM' : '当前 URDF 质心') : (state.language === 'en' ? 'Original COM' : '原始质心');
    const candidate = binding.historical ? (state.language === 'en' ? 'Historical candidate COM' : '历史候选质心') : (state.language === 'en' ? 'Candidate COM' : '候选质心');
    const zoom = state.language === 'en' ? 'Visual displacement only' : '仅放大显示位移，不更改实际坐标';
    return `<div class="calibration-com-overlay" aria-label="${title}"><div class="calibration-com-heading">${title}</div><div class="calibration-com-legend"><span><i class="com-swatch original"></i>${original}</span><span><i class="com-swatch candidate"></i>${candidate}</span><span class="com-connector">─</span>${state.language === 'en' ? 'Offset' : '质心偏移'}</div>${hasComparison ? `<div class="calibration-com-scale"><span>${state.language === 'en' ? 'Display' : '显示比例'}</span>${[1,12,25].map(value=>`<button type="button" class="${scale===value?'active':''}" data-calibration-com-scale="${value}" aria-pressed="${scale===value}">×${value}</button>`).join('')}</div><label class="calibration-com-unchanged"><input type="checkbox" id="calibration-com-unchanged" ${state.modelCalibrationShowUnchanged?'checked':''}>${state.language==='en'?'Show unchanged links':'显示未变化的 Link'}</label><div class="calibration-com-hint">${scale===1?(state.language==='en'?'True 3D locations':'真实三维位置'):zoom}</div>` : `<div class="calibration-com-hint">${binding.comparable?pending:esc(binding.reason)}</div>`}</div>`;
}
function workbenchRecoveryRequired() { return workbenchConnected() && state.session?.recovery_required === true; }
function workbenchRecoveryNotice() {
    if (!workbenchRecoveryRequired()) return '';
    const en = state.language === 'en';
    return `<section class="workspace-section" role="alert"><h3>${en ? 'Control session not responding' : '控制会话无响应'}</h3><p class="hint">${en ? 'The previous hardware command timed out. It may still be executing, and the displayed joint state may be stale. Do not repeat enable or disable. Secure the robot using physical safety measures, then explicitly disconnect and reconnect' : '上一条硬件命令已超时，原生命令可能仍在执行，界面关节反馈可能过期，请勿重复使能或失能，请先采取物理安全措施，再确认强制断连并重新连接'}</p><button id="workbench-recover-disconnect" class="button danger">${en ? 'Disconnect unresponsive session' : '恢复连接（需安全确认）'}</button></section>`;
}
function workbenchPage() { const connected = workbenchConnected(); const recovering = workbenchRecoveryRequired(); const action = connected ? (recovering ? `<button id="workbench-force-disconnect" class="button danger">${state.language === 'en' ? 'Recover connection' : '恢复连接'}</button>` : `<button id="stop-workbench" class="button danger">${t('stopWorkbench')}</button>`) : `<button id="start-workbench" class="button primary">${t('startWorkbench')}</button>`; return pageHead('workbenchTitle', 'workbenchDesc', action) + workbenchStatusBar() + workbenchTabs() + workbenchRecoveryNotice() + `<div class="workbench-layout"><section class="workbench-model"><div id="workbench-viewport">${state.modelData ? '' : `<div class="empty-state">${t('modelNotLoaded')}</div>`}</div>${calibrationComOverlayHtml()}<div class="workbench-model-note">${t(connected ? 'actualTelemetry' : 'offlineWaiting')}</div></section><section class="workbench-task">${workbenchPanelHtml()}</section></div>`; }
async function mountWorkbenchView() { if (state.page !== 'workbench' || !state.modelData || !$('workbench-viewport')) return; try { if (!modelModule) modelModule = await import('./model-view.js'); modelView?.dispose(); const view = new modelModule.SerialArmModelView($('workbench-viewport'), file => api.modelResource(file)); modelView = view; await view.setModel(state.modelData); if (modelView !== view || state.page !== 'workbench') return; view.setLayers({ visual: true, collision: false, linkFrames: false, jointAxes: false, com: false, inertia: false, labels: false }); updateWorkbenchModel(); updateModelCalibrationComparison(); } catch (error) { toast((state.language === 'en' ? 'Workspace model: ' : '工作台模型: ') + error.message, true); } }
function updateWorkbenchModel() { if (!modelView || !state.telemetry || !state.telemetry.valid) return; if (Number.isFinite(state.telemetry.feedback_age_ms) && state.telemetry.feedback_age_ms > 500) return; const frames = state.telemetry.frames || []; modelView.updatePose({ state: { frames, positions: state.telemetry.joint?.pos || [] } }); updateModelCalibrationComparison(); }
function updateWorkbenchControlState() { const x = state.telemetry || {}; const recovering = workbenchRecoveryRequired(); document.querySelectorAll('[data-workbench-action]').forEach(button => { const action = button.dataset.workbenchAction; const fault = x.robot_state === 'FAULT'; const active = x.robot_state === 'ACTIVE'; const inactive = x.robot_state === 'INACTIVE'; const permitted = ({activate:inactive && state.readiness?.modes?.RIGID_HOLD === 'verified' && state.readiness?.checks?.bringup === 'verified', park:active && state.readiness?.modes?.RIGID_HOLD === 'verified', deactivate:active||fault, hold:active && state.readiness?.modes?.RIGID_HOLD === 'verified', clear_fault:fault, fault_compliant:fault && state.readiness?.checks?.gravity_verification === 'verified', fault_rigid:fault && state.readiness?.modes?.RIGID_HOLD === 'verified'})[action] ?? false; button.disabled = recovering || !permitted; button.title = recovering ? (state.language === 'en' ? 'Recover the connection before issuing hardware commands' : '原生会话无响应，请先恢复连接') : !permitted ? (state.language === 'en' ? 'Not available in the current robot state' : '当前机械臂状态不允许此操作') : ''; }); const pending = state.workbenchPending || {}; const active = x.robot_state === 'ACTIVE'; const inactive = x.robot_state === 'INACTIVE'; document.querySelectorAll('[data-impedance-mode]').forEach(button => { const mode = button.dataset.impedanceMode; button.classList.toggle('active', x.impedance_mode === mode); button.disabled = recovering || !active || pending.kind === 'impedance' || state.readiness?.modes?.[mode] !== 'verified'; button.title = state.readiness?.modes?.[mode] !== 'verified' ? readinessResultLabel(state.readiness?.modes?.[mode]) : active ? '' : (state.language === 'en' ? 'The robot must be ACTIVE before changing impedance mode' : '机器人需要处于 ACTIVE 才能切换阻抗模式'); }); document.querySelectorAll('[data-feedforward-mode]').forEach(button => { const mode = button.dataset.feedforwardMode; button.classList.toggle('active', x.model_feedforward_mode === mode); button.disabled = recovering || !inactive || pending.kind === 'feedforward' || (mode !== 'NONE' && state.readiness?.checks?.[mode === 'GRAVITY' ? 'gravity_verification' : 'dynamics_calibration'] !== 'verified'); button.title = inactive ? '' : (state.language === 'en' ? 'Model feedforward can only be changed while the robot is INACTIVE' : '模型前馈只能在机器人处于 INACTIVE 时修改'); }); }
function applyWorkbenchSnapshot(snapshot) { if (!snapshot || typeof snapshot !== 'object') return; state.telemetry = snapshot; state.telemetryReceivedAt = Date.now(); updateWorkbenchLive(); }
async function switchWorkbenchMode(kind, mode) { if (state.workbenchPending?.kind || state.blockingBusy) return; state.workbenchPending = { kind, value: mode }; updateWorkbenchControlState(); const message = kind === 'impedance' ? busyText(`正在切换阻抗模式至 ${mode}...`, `Switching impedance mode to ${mode}...`) : busyText(`正在切换模型前馈至 ${mode}...`, `Switching model feedforward to ${mode}...`); try { return await blockingOperation(message, async () => { await new Promise(resolve => requestAnimationFrame(resolve)); const method = kind === 'impedance' ? 'set_impedance_mode' : 'set_model_feedforward_mode'; const snapshot = await workbenchRequest(method, { mode }); applyWorkbenchSnapshot(snapshot); return snapshot; }); } finally { state.workbenchPending = { kind: '', value: '' }; updateWorkbenchControlState(); } }
function updateWorkbenchLive() { if (state.page !== 'workbench') return; const x = state.telemetry || {}; const age = document.querySelector('.workspace-status span:nth-child(5)'); if (age) age.textContent = `${t('feedbackAge')} ${Number.isFinite(x.feedback_age_ms) ? x.feedback_age_ms.toFixed(0) + ' ms' : '—'}`; updateWorkbenchControlState(); refreshCalibrationQuickControls(); refreshCalibrationActionControls(); refreshGainApplyControls(); updateWorkbenchModel(); if (state.pendingTarget && x.valid && Array.isArray(x.joint?.pos)) { const err = Math.max(...x.joint.pos.map((v, i) => Math.abs(v - state.pendingTarget[i]))); const vel = Math.max(...(x.joint.vel || []).map(v => Math.abs(v))); if (err < 0.02 && vel < 0.05) { state.commandState = 'complete'; state.pendingTarget = null; const node = document.querySelector('.command-state'); if (node) node.textContent = `${t('commandState')} ${commandStateText(state.commandState)}`; } } const c = x.calibration || {}; const kind = $('cal-task-kind'), phase = $('cal-task-phase'), progress = $('cal-task-progress'), trajectory = $('cal-task-trajectory'), error = $('cal-task-error'); if (kind) kind.textContent = calibrationTaskKindText(c.kind || 'none'); if (phase) phase.textContent = calibrationTaskPhaseText(c.phase || 'idle'); if (progress) progress.textContent = c.expected ? `${c.captured || 0}/${c.expected}` : '—'; if (trajectory) trajectory.textContent = String(c.trajectory_samples ?? '—'); if (error) error.textContent = c.error ? localizedError(c.error) : '—'; if (c.phase === 'complete' && state.calibrationFlow === 'friction') { state.calibrationFlow = 'complete'; operation(loadAdmittance); } updateModelCalibrationLive(); updateModelCalibrationTaskOverlay(); if (state.workbenchTab === 'diagnostics') drawWorkbenchChart(); }
function updateModelCalibrationComparison() {
    if (!modelView || state.page !== 'workbench') return;
    const binding = modelCalibrationModelBinding();
    const result = state.workbenchTab === 'calibration' && binding.comparable ? currentModelCalibration()?.gravity_result : null;
    const scale = state.modelCalibrationDisplayScale || 1;
    const key = result?.first_moments ? `${binding.actual}:${binding.expected}:${JSON.stringify(result.first_moments)}:${scale}:${state.modelCalibrationShowUnchanged}:${binding.historical}` : '';
    if (state.modelCalibrationComparisonView === modelView && state.modelCalibrationComparisonKey === key) return;
    modelView.setGravityComparison(result || null, {displayScale: scale, showUnchanged: state.modelCalibrationShowUnchanged, sourceInertials: binding.historical ? binding.alignment.source_inertials : null, candidateMasses: result?.candidate_masses || null});
    state.modelCalibrationComparisonView = modelView;
    state.modelCalibrationComparisonKey = key;
}
function updateModelCalibrationLive() {
    if (state.page !== 'workbench' || state.workbenchTab !== 'calibration' || state.modelCalibrationOffline) return;
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
        if (label) label.textContent = `${calibrationAlignmentStatus(aligned, status.alignment_active)}${aligned.valid ? ` · Δmax ${aligned.maxError.toFixed(3)} rad · |dq|max ${aligned.maxSpeed.toFixed(3)} rad/s` : ''}`;
        const body = $('model-calibration-align-rows');
        if (body) body.innerHTML = aligned.rows.map(x => `<tr><td>${esc(x.name)}</td><td>${x.now.toFixed(3)}</td><td>${x.goal.toFixed(3)}</td><td>${x.delta >= 0 ? '+' : ''}${x.delta.toFixed(3)}</td></tr>`).join('');
        const start = $('model-calibration-replay-confirm');
        if (start) start.disabled = !aligned.aligned;
    }
    updateModelCalibrationComparison();
}
function updateModelCalibrationTaskOverlay() {
    const overlay = $('model-calibration-overlay'); if (!overlay) return;
    const status = state.telemetry?.model_calibration; const show = !priorityErrorOpen && modelCalibrationLongRunning(status) && state.telemetry?.robot_state !== 'FAULT';
    overlay.hidden = !show; overlay.setAttribute('aria-hidden', show ? 'false' : 'true');
    if (!show) return;
    const adapting = status.phase === 'gain_adaptation';
    const passes = Math.max(0, Number(status.gain_adaptation_passes || 0));
    const revisions = Math.max(0, Number(status.gain_adaptation_revisions || 0));
    const maxGainPasses = Math.max(1, Number(status.gain_adaptation_max_passes || 4));
    const progress = adapting ? Math.min(1, passes / maxGainPasses) : Math.max(0, Math.min(1, Number(status.progress || 0)));
    $('model-calibration-task-title').textContent = modelCalibrationPhaseText(status.phase);
    $('model-calibration-task-progress-bar').style.width = `${Math.round(progress * 100)}%`;
    $('model-calibration-task-progress').textContent = adapting
        ? (state.language === 'en' ? `Gain traversals ${passes}/${maxGainPasses} (maximum) · revisions ${revisions}` : `增益标定单程 ${passes}/${maxGainPasses}（最多）· 增益调整 ${revisions} 次`)
        : status.phase === 'teaching' ? `${state.language === 'en' ? 'Free teaching' : '自由示教'} ${Number(status.teaching_wall_duration_s || 0).toFixed(1)} s · ${state.language === 'en' ? 'No duration cap' : '无时长上限'}`
        : `${Math.round(progress * 100)}% · ${Number(status.completed || 0)}/${Number(status.total || 0)}`;
    $('model-calibration-task-samples').textContent = `${t('acceptedDroppedFrames')} ${status.recorder?.accepted ?? '—'} / ${status.recorder?.dropped ?? '—'} · ${t('validStaticSamples')} ${status.valid_static_samples ?? '—'}`;
    $('model-calibration-task-feedback').textContent = `${t('feedbackAge')} ${Number.isFinite(state.telemetry?.feedback_age_ms) ? state.telemetry.feedback_age_ms.toFixed(0) + ' ms' : '—'}`;
    $('model-calibration-task-id').textContent = `${t('taskIdentifier')} ${status.task_id || '—'}`;
    $('model-calibration-task-error').textContent = (status.error ? localizedError(status.error) : '') || (status.recorder?.data_gap ? (state.language === 'en' ? 'Recorded data contains gaps' : '记录数据存在缺口') : status.recorder?.write_failed ? (state.language === 'en' ? 'Raw data write failed' : '原始数据写入失败') : '');
    $('model-calibration-task-stop-teach').hidden = status.phase !== 'teaching';
    $('model-calibration-task-pause').hidden = !['gain_adaptation', 'static_reverse', 'static_forward', 'gravity_fitting', 'friction_reverse_slow', 'friction_forward_slow', 'friction_reverse_fast', 'friction_forward_fast', 'friction_fitting'].includes(status.phase);
    $('model-calibration-task-resume').hidden = status.phase !== 'paused';
    $('model-calibration-task-hold').disabled = state.telemetry?.robot_state !== 'ACTIVE';
    $('model-calibration-task-disable').disabled = state.telemetry?.robot_state === 'INACTIVE';
}
function modelCalibrationTaskParams(params = {}) {
    const taskId = currentModelCalibration()?.task_id || state.telemetry?.model_calibration?.task_id || '';
    return taskId ? { ...params, task_id: taskId } : { ...params };
}
async function finishTeachingSafely(cleanupOnly = false) {
    return blockingOperation(busyText('正在结束示教并同步任务状态...', 'Closing demonstration and syncing task state...'), async () => {
        const method = cleanupOnly ? 'model_calibration_cancel' : 'model_calibration_teach_stop';
        try {
            const result = await workbenchRequest(method, modelCalibrationTaskParams());
            if (result && state.telemetry) state.telemetry.model_calibration = result;
            if (result?.directory) state.modelCalibrationDirectory = result.directory;
        } finally {
            // Native teaching may have ended asynchronously on disable or FAULT
            // Always read the authoritative phase even if the close request failed
            try {
                const latest = await workbenchRequest('model_calibration_status', {});
                if (latest && state.telemetry) state.telemetry.model_calibration = latest;
                if (latest?.directory) state.modelCalibrationDirectory = latest.directory;
            } catch { /* Preserve the original close error */ }
            render();
        }
    });
}
async function modelCalibrationQuick(method, params = {}) {
    try {
        const before = state.telemetry?.model_calibration?.phase;
        const requestParams = method === 'model_calibration_teach_begin' ? params : modelCalibrationTaskParams(params);
        const result = await workbenchRequest(method, requestParams);
        if (method.startsWith('model_calibration_') && result && state.telemetry) { state.telemetry.model_calibration = result; if (result.directory) state.modelCalibrationDirectory = result.directory; }
        if (result?.phase !== before && ['waiting_replay_confirmation', 'complete', 'cancelled', 'failed'].includes(result?.phase)) render();
        else { updateModelCalibrationTaskOverlay(); updateModelCalibrationLive(); }
        return result;
    } catch (error) {
        // Refresh immediately: teach_stop can transition into FAILED before
        // the next telemetry event. Never leave a stale teaching mask onscreen.
        try {
            const latest = await workbenchRequest('model_calibration_status', {});
            if (latest && state.telemetry) state.telemetry.model_calibration = latest;
        } catch { /* preserve the original request error */ }
        showPriorityError(t('error') + ': ' + error.message);
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
        selections.push({name: link.name, joint: joint.name, mass});
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
function editCalibrationLockedLinks({onCancel = null} = {}) {
    const choices = calibrationSelectableLinks();
    if (!choices.length) throw new Error(state.language === 'en' ? 'No identifiable links available; load a model or recorded calibration' : '当前没有可辨识 Link，请加载模型或历史标定记录');
    const current = state.modelCalibrationOptions;
    const allowed = choices.map(x => x.name);
    const checked = new Set(current.locked_links || []);
    const dialog = $('confirm-dialog');
    dialog.innerHTML = `<h3>${state.language === 'en' ? 'Trusted Link parameters' : '选择可信 Link'}</h3><p>${state.language === 'en' ? 'Checked = preserve original first moments; only controlled inertial links are listed. Massless tool frames, fixed attachments and unsampled branches cannot be identified independently' : '勾选代表锁定原始质量一阶矩，未勾选代表允许校正；清单只显示受控关节关联的有效惯量 Link，无质量 tool0、固定附属及未采样分支不能单独辨识'}</p><div class="calibration-link-picker">${choices.map(x => `<label class="calibration-link-option"><input type="checkbox" data-gravity-locked="${esc(x.name)}" ${checked.has(x.name) ? 'checked' : ''}><span>${esc(x.name)}</span><small>${x.joint ? esc(x.joint)+' · ' : ''}${Number.isFinite(x.mass) ? Number(x.mass).toFixed(4)+' kg' : ''}</small></label>`).join('')}</div><div class="actions"><button id="cal-link-lock-all" class="button">${state.language === 'en' ? 'Lock all' : '全部锁定'}</button><button id="cal-link-unlock-all" class="button">${state.language === 'en' ? 'Unlock all' : '全部解锁'}</button></div><div class="actions"><button id="cal-link-cancel" class="button">${t('cancelTask')}</button><button id="cal-link-confirm" class="button primary">${state.language === 'en' ? 'Confirm' : '确认选择'}</button></div>`;
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
function captureModelCalibrationOptions({validate = true} = {}) {
    const read = (id, fallback) => { const value = Number($(id)?.value); return Number.isFinite(value) ? value : fallback; };
    state.modelCalibrationOptions = {
        mode: state.modelCalibrationOptions.mode || 'global',
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
function modelCalibrationReplayConfirm() {
    const dialog = $('confirm-dialog');
    const plan = currentModelCalibration() || {};
    if (plan.source_task_id && !calibrationAlignment(plan).aligned) { showPriorityError(state.language === 'en' ? 'Replay start not aligned. Use guided alignment and wait for fresh feedback.' : '当前姿态与回放起点不匹配：请先完成引导对齐，并等待新鲜的关节反馈'); return; }
    if (plan.planner_id !== 'local_hermite' || !(Number(plan.single_pass_duration_s) > 0)) { showPriorityError(state.language === 'en' ? 'Native Core was not rebuilt. Reinstall before replay.' : 'C++ Core 仍是旧版，请重新编译安装后再执行自动回放'); return; }
    const gainPlan = state.calibrationEntry === 'gain';
    const oneWayDuration = Number(plan.single_pass_duration_s || 0);
    const estimate = gainPlan && oneWayDuration > 0
        ? gainCalibrationSeconds(oneWayDuration, state.gainMode)
        : Number(plan.estimated_duration_s || 0);
    const teaching = Number(plan.teaching_wall_duration_s || 0);
    const totalEstimate = gainPlan ? teaching + estimate : Number(plan.estimated_total_duration_s || teaching + estimate);
    const oneWay = Number(plan.single_pass_duration_s || 0);
    const poses = Array.isArray(plan.pose_targets) ? plan.pose_targets.length : 0;
    const formula = gainPlan
        ? `${state.language === 'en' ? 'Plan' : '计算'}：${state.gainMode === 'all' ? (state.language === 'en' ? 'Rigid + Compliant + Identification' : '刚性 + 柔性 + 辨识') : state.gainMode === 'compliant' ? (state.language === 'en' ? '55% + 75% search, 65% frozen validation + hold' : '55% 与 75% 搜索，65% 冻结验收及保持验证') : (state.language === 'en' ? '80% + 100% search, 100% frozen validation' : '80% 与 100% 搜索，100% 冻结验收')} · ${oneWay.toFixed(1)} s ${state.language === 'en' ? 'one-way' : '单程'}`
        : `${state.language === 'en' ? 'Plan' : '计算'}：T自动 ≈ 2 × ${oneWay.toFixed(1)} + 1.5 × ${poses} + 2`;
    dialog.innerHTML = `<h3>${gainPlan ? (state.language === 'en' ? 'Gain calibration' : '自适应增益标定') : t('modelCalibration')}</h3><p>${gainPlan ? (state.language === 'en' ? state.gainMode==='all' ? 'Rigid, compliant and identification gains are calibrated consecutively, each with two round trips, twelve traversals total' : 'Search round trip followed by frozen validation · compliant target is tested at 65% replay speed and checked for stationary hold stability' : state.gainMode==='all' ? '依次标定刚性、柔性和辨识增益，各两次往返，共十二次单程，柔性拖拽保持不变' : '首轮搜索一次往返，次轮冻结增益独立验收一次往返 · 柔性按 65% 速度验收并增加静态保持检查') : (state.language === 'en' ? 'The checked path is replayed once in reverse and once forward for gravity measurement.' : '沿已检查轨迹进行一次反向与正向的重力采集')}</p><div class="callout"><b>${state.language === 'en' ? 'Estimated automatic stage' : '预计自动阶段'}：${estimate.toFixed(1)} s · ${state.language === 'en' ? 'Including teaching' : '加上示教已用'} ${totalEstimate.toFixed(1)} s</b><div>${formula}</div><div>${state.language === 'en' ? 'Estimate only: feedback pauses and settling may extend real time; safety limits remain unchanged.' : '仅为计划运动估算：实际跟踪等待可能延长耗时，安全保护不变'}</div></div><label class="confirmation"><input id="model-calibration-released" type="checkbox"><span>${state.language === 'en' ? 'I checked the robot, load, cables, and replay area, and I have fully released the robot' : '我已检查机械臂、负载、线缆和回放区域，并已完全松手'}</span></label><div class="actions"><button id="model-calibration-confirm-cancel" class="button">${t('cancel')}</button><button id="model-calibration-confirm-start" class="button danger" disabled>${t('modelCalibrationConfirmReplay')}</button></div>`;
    dialog.showModal(); $('model-calibration-released').onchange = () => $('model-calibration-confirm-start').disabled = !$('model-calibration-released').checked; $('model-calibration-confirm-cancel').onclick = () => dialog.close();
    $('model-calibration-confirm-start').onclick = () => { dialog.close(); blockingOperation(busyText('正在启动自动采集任务...', 'Starting automatic acquisition...'), async () => { if (state.calibrationEntry === 'gain') { state.calibrationToolSelected.gain = null; state.gainCandidateLoadAttempted = ''; state.gainCandidateLoadError = ''; } const result = await workbenchRequest('model_calibration_start', modelCalibrationTaskParams({ released: true, workflow: state.calibrationEntry === 'gain' ? 'gain' : 'gravity', gain_mode: state.gainMode })); if (state.telemetry) state.telemetry.model_calibration = result; state.modelCalibrationOffline = null; state.modelCalibrationCandidateSource = 'online'; render(); updateModelCalibrationTaskOverlay(); }); };
}
async function resumeModelCalibrationWithConfirmation() {
    try {
        const result = await workbenchRequest('model_calibration_resume', modelCalibrationTaskParams({ confirmed: false })); if (state.telemetry) state.telemetry.model_calibration = result; updateModelCalibrationTaskOverlay(); return result;
    } catch (error) {
        if (!String(error.message).includes('explicit resume confirmation')) { toast(t('error') + ': ' + error.message, true); return null; }
        const dialog = $('confirm-dialog'); dialog.innerHTML = `<h3>${t('resumeTask')}</h3><p>${state.language === 'en' ? 'The robot moved while paused; confirm the current position is safely consistent with the paused path before continuing' : '暂停期间机械臂位置发生变化，请确认当前位置与暂停路径安全一致后继续'}</p><div class="actions"><button id="model-cal-resume-cancel" class="button">${t('cancel')}</button><button id="model-cal-resume-confirm" class="button danger">${state.language === 'en' ? 'Confirm and continue' : '确认继续'}</button></div>`; dialog.showModal(); $('model-cal-resume-cancel').onclick = () => dialog.close(); $('model-cal-resume-confirm').onclick = () => { dialog.close(); modelCalibrationQuick('model_calibration_resume', { confirmed: true }); };
        return null;
    }
}

function drawWorkbenchChart() { const canvas = $('workbench-chart'); if (!canvas) return; const ctx = canvas.getContext('2d'); const rect = canvas.getBoundingClientRect(); canvas.width = Math.max(400, Math.floor(rect.width * devicePixelRatio)); canvas.height = Math.floor(180 * devicePixelRatio); ctx.scale(devicePixelRatio, devicePixelRatio); const w = rect.width, h = 180; ctx.clearRect(0, 0, w, h); const data = state.telemetryHistory.slice(-180); if (data.length < 2) return; const values = data.map(x => Number(x.joint?.residual_raw?.[0] ?? 0)); const max = Math.max(0.1, ...values.map(Math.abs)); ctx.beginPath(); values.forEach((v, i) => { const x = i / (values.length - 1) * w, y = h / 2 - v / max * (h * 0.42); i ? ctx.lineTo(x, y) : ctx.moveTo(x, y); }); ctx.strokeStyle = '#df7b00'; ctx.lineWidth = 1.5; ctx.stroke(); ctx.fillStyle = '#687080'; ctx.fillText('joint1 residual_raw', 8, 16); }

function runPage() {
    return pageHead('runtimeTitle', 'runtimeDesc') + `<div class="mode-grid mode-grid-three">${['terminal', 'hardware', 'moveit'].map(mode => `<button class="mode-card ${state.mode === mode ? 'active' : ''}" data-mode="${mode}"><span class="mode-type">${t('real')}</span>${icon(mode)}<h3>${t(mode)}</h3><p>${t(mode + 'Desc')}</p></button>`).join('')}</div><div class="card launch-panel"><div><h3>${t('selectedRobot')} · ${esc(state.config.profile || '—')}</h3><p>${state.info?.available?.[state.mode] ? t('real') : t('modeMissing')}</p><div class="launch-command">${esc(state.info?.profile_file || state.config.profile_file || '')}</div></div>${button('launch-session', 'launch', 'run', 'primary')}</div>`;
}
function diagnosticsPage() {
    const checks = [{ name: 'installation', ok: !!state.status.installed, detail: state.status.prefix || t('notInstalled') }, ...(state.info?.checks || [])];
    return pageHead('diagTitle', 'diagDesc', button('check-now', 'checkNow', 'refresh')) + `<div class="grid">${card('checks', 'checkSub', 'diagnostics', checks.map(c => `<div class="diag-row"><span class="dot ${c.ok ? 'ready' : 'error'}"></span><span class="name">${t(c.name)}</span><span class="detail">${esc(c.detail)}</span><span class="state ${c.ok ? '' : 'error'}">${t(c.ok ? 'pass' : 'fail')}</span></div>`).join(''))}${card('installHelp', 'installHelpSub', 'terminal', `<p class="hint">${t('installHint')}</p><pre class="install-command"><code>./install.sh</code></pre><pre class="install-command"><code>./install.sh --preset ros2 --yes</code></pre><div class="callout">${t('validationHint')}</div>`)}</div>`;
}
function settingsPage() {
    return pageHead('settingsTitle', 'settingsDesc', `<button id="settings-back" class="button">${state.language === 'en' ? 'Back' : '返回'}</button>`) + `<div class="grid">${card('appearance', 'appearanceSub', 'sun', `<div class="segmented">${['system', 'dark', 'light'].map(theme => `<button data-theme="${theme}" class="${theme === state.theme ? 'active' : ''}">${t(theme)}</button>`).join('')}</div>`)}${card('language', 'languageSub', 'settings', `<div class="segmented"><button data-language="zh-CN" class="${state.language === 'zh-CN' ? 'active' : ''}">简体中文</button><button data-language="en" class="${state.language === 'en' ? 'active' : ''}">English</button></div>`)}</div><div class="callout">${t('prefsNote')}</div>`;
}

async function mountDescriptionView() {
    if (state.page !== 'description' || !state.descriptionAnalysis || !$('description-viewport')) return;
    if (!modelModule) modelModule = await import('./model-view.js');
    modelView?.dispose(); modelView = new modelModule.SerialArmModelView($('description-viewport'), file => api.modelResource(file));
    await modelView.setModel(state.descriptionAnalysis.preview);
    modelView.setLayers({ visual: true, collision: false, linkFrames: true, jointAxes: true, com: false, inertia: false, labels: true });
}
async function refreshLibrary() {
    const data = await api.request('profile_library'); state.library = data.profiles || []; state.libraryPath = data.path || '';
}
async function openLibraryEntry(id) {
    const entry = state.library.find(x => x.id === id); if (!entry) throw new Error('Profile Library entry not found');
    state.activeLibraryId = id; state.createdProfileWizard = false; state.source = entry.source === 'builtin' ? 'builtin' : 'external';
    state.config = { profile: entry.profile, profile_file: entry.source === 'builtin' ? '' : entry.profile_file, resource_paths: entry.resource_paths || '', serial_port: '', baudrate: '', bus: '' };
    const listing = await api.request('profiles', state.config); state.profiles = listing.profiles || []; state.info = null; state.modelData = null; state.profileEditor = null;
    await inspect(); state.readiness = await api.request('readiness', { config: state.config });
    state.profileDirty = false; state.profileError = ''; state.profileGroup = 'model';
    if (state.entryIntent === 'continue') {
        state.profileEditor = await api.request('profile_editor_load', {config:state.config});
        state.page = 'profile_setup';
    } else state.page = 'lifecycle';
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
    if ($('description-source')) d.source = $('description-source').value.trim();
    if ($('draft-base')) d.base_frame = $('draft-base').value;
    if ($('draft-tool')) d.tool_frame = $('draft-tool').value;
    if ($('draft-profile')) d.profile = $('draft-profile').value.trim();
    if ($('draft-destination')) d.destination = $('draft-destination').value.trim();
    syncDescriptionChain();
}
async function analyzeDescription() {
    collectDescriptionDraft();
    if (!state.descriptionDraft.source) throw new Error(state.language === 'en' ? 'Choose a Description first' : '请先选择 Description 文件');
    state.descriptionAnalysis = await api.request('description_inspect', { source: state.descriptionDraft.source });
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
    const result = await api.request('description_create_profile', { source:d.source, profile:d.profile, destination:d.destination,
        base_frame:d.base_frame, tool_frame:d.tool_frame, joint_names:d.joint_names, hardware_plugin:'' });
    state.config = { ...result.config };
    state.source = 'external'; state.info = null; state.modelData = null;
    // Creation succeeds independently of Library registration so the generated files remain recoverable
    try {
        const entry = await api.request('profile_library_register', { profile_file:result.profile_file, profile:result.profile,
            resource_paths:result.resource_paths, policy:'suffix' });
        state.activeLibraryId = entry.id;
        await refreshLibrary();
    } catch (error) {
        toast((state.language === 'en' ? 'Profile created, but Library registration failed: ' : 'Profile 已生成，但加入 Library 失败：') + error.message, true);
    }
    await inspect();
    state.readiness = await api.request('readiness', { config:state.config });
    state.profileEditor = await api.request('profile_editor_load', { config:state.config });
    state.profileGroup = 'hardware'; state.profileDirty = false; state.profileError = '';
    state.createdProfileWizard = true; state.descriptionStep = 4;
    state.page = 'profile_setup'; await save(); shell();
}

function collectProfileEditor() {
    const e = state.profileEditor;
    if (!e) return null;
    return {core_sha:e.core_sha,hardware_sha:e.hardware_sha,profile_sha:e.profile_sha,
        core:e.core,hardware:e.hardware,profile_entry:e.profile_entry};
}
function updateProfileSwitch(input) {
    const switchNode = input.closest('.parameter-switch');
    if (!switchNode) return;
    switchNode.dataset.switchState = input.checked ? 'on' : 'off';
    input.setAttribute('aria-checked', String(input.checked));
}
function readProfileInput(input) {
    const path = JSON.parse(input.dataset.profileField);
    const root = state.profileEditor;
    if (!root) return;
    let target = root;
    for (const key of path.slice(0,-1)) target = target[key];
    const key = path.at(-1);
    const old = target[key];
    let value;
    if (typeof old === 'boolean') value = input.checked;
    else if (typeof old === 'number') {
        if (!input.value.trim() || !Number.isFinite(Number(input.value))) throw new Error(path.join('.') + ' 请输入有限数值');
        value = Number(input.value);
    } else if (Array.isArray(old)) {
        value = input.value.split(',').map(x => x.trim());
        if (typeof old[0] === 'number') {
            if (value.some(x => !x || !Number.isFinite(Number(x)))) throw new Error(path.join('.') + ' 数组需要有效数字');
            value = value.map(Number);
        }
        if (value.length !== old.length) throw new Error(path.join('.') + ' 数组长度不能改变');
    } else value = input.value;
    target[key] = value;
    state.profileDirty = true;
    state.profileError = '';
    const label = $('profile-dirty-state');
    if (label) label.textContent = state.language === 'en' ? 'Unsaved changes' : '存在未保存修改';
    input.closest('.parameter-field')?.classList.remove('invalid');
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
    if (state.page === 'workbench' && state.workbenchTab === 'calibration' && !calibrationDialogOpen()) render();
}
function refreshCalibrationOrDefer() {
    if (calibrationDialogOpen()) {
        calibrationRefreshPending = true;
        updateWorkbenchLive();
        return;
    }
    render();
}
function queuePreview() { previewDirty = true; if (previewFrame || previewInFlight) return; previewFrame = requestAnimationFrame(async () => { previewFrame = 0; if (previewInFlight) return; previewInFlight = true; previewDirty = false; try { await previewModel(); } catch (error) { toast(t('error') + ': ' + error.message, true); } finally { previewInFlight = false; if (previewDirty) queuePreview(); } }); }
function reducedMotion() { return matchMedia('(prefers-reduced-motion: reduce)').matches; }
async function pageExit() {
    const page = $('page');
    if (!page || reducedMotion()) return;
    page.classList.remove('page-transition-in');
    page.classList.add('page-transition-out');
    await new Promise(resolve => setTimeout(resolve, 80));
}
function pageEnter(pageChanged = true) {
    const page = $('page');
    if (!page || reducedMotion() || !pageChanged) return;
    page.classList.remove('page-transition-out', 'page-transition-in');
    void page.offsetWidth;
    page.classList.add('page-transition-in');
}
function render() {
    if (modelView) { modelView.dispose(); modelView = null; }
    const pages = { start: startPage, library: libraryPage, description: descriptionPage, lifecycle: lifecyclePage, profile_setup: profileSetupPage, robot: robotPage, model: modelPage, workbench: workbenchPage, run: runPage, diagnostics: diagnosticsPage, settings: settingsPage };
    const factory = pages[state.page] || startPage;
    const pageChanged = renderedPage !== state.page;
    $('page').innerHTML = factory();
    pageEnter(pageChanged);
    renderedPage = state.page;
    const runtime = $('runtime-panel'); if (runtime) runtime.classList.toggle('is-hidden', state.page !== 'run' && !active());
    document.querySelectorAll('[data-page]').forEach(b => b.classList.toggle('active', b.dataset.page === state.page));
    statusUI(); bindPage();
    if ($('priority-error-ack')) $('priority-error-ack').onclick = acknowledgePriorityError;
    if ($('priority-error-overlay')) $('priority-error-overlay').addEventListener('cancel', event => event.preventDefault());
    if (state.page === 'run') setTimeout(() => { try { fit?.fit(); } catch { } }, 50);
    if (state.page === 'model') setTimeout(() => operation(mountModelView), 0);
    if (state.page === 'workbench') setTimeout(() => operation(mountWorkbenchView), 0);
    if (state.page === 'description' && state.descriptionAnalysis) setTimeout(() => operation(mountDescriptionView), 0);
}
function capture() { for (const key of Object.keys(state.config)) { const element = $(key); if (element) state.config[key] = element.value; } if ($('profile-select')) state.config.profile = $('profile-select').value; }
async function operation(fn) { try { return await fn(); } catch (error) { toast(t('error') + ': ' + error.message, true); return null; } }
function busyText(zh, en) { return state.language === 'en' ? en : zh; }
function setOperationOverlay(message = '') { const overlay = $('operation-overlay'); if (!overlay) return; const show = !!message && !priorityErrorOpen; overlay.hidden = !show; overlay.setAttribute('aria-hidden', show ? 'false' : 'true'); const text = $('operation-message'); if (text && show) text.textContent = message; }
async function blockingOperation(message, fn, minimumMs = 320) { if (state.blockingBusy) return null; state.blockingBusy = true; setOperationOverlay(message || busyText('正在处理...', 'Working...')); const started = performance.now(); try { return await fn(); } catch (error) { toast(t('error') + ': ' + error.message, true); return null; } finally { const remaining = minimumMs - (performance.now() - started); if (remaining > 0) await new Promise(resolve => setTimeout(resolve, remaining)); state.blockingBusy = false; setOperationOverlay(''); } }
function actionBusyMessage(action) { const labels = { activate: ['正在使能...', 'Activating...'], park: ['正在停放并失能...', 'Parking and disabling...'], deactivate: ['正在立即失能...', 'Disabling...'], hold: ['正在保持当前位置...', 'Holding current position...'], clear_fault: ['正在清除故障...', 'Clearing fault...'], fault_compliant: ['正在进入柔性恢复...', 'Entering compliant recovery...'], fault_rigid: ['正在返回刚性保持...', 'Returning to rigid hold...'] }; const value = labels[action] || ['正在处理...', 'Working...']; return busyText(value[0], value[1]); }
async function refreshProfiles() {
    capture(); if (state.source === 'external' && !state.config.profile_file) throw new Error(t('fileHint')); const data = await api.request('profiles', state.config); state.profiles = data.profiles;
    if (!state.profiles.some(p => p.name === state.config.profile)) state.config.profile = state.profiles[0]?.name || '';
    state.info = null; render(); await save(); await inspect(); render();
}
async function inspect() {
    capture(); if (state.source === 'external' && !state.config.profile_file) throw new Error(t('fileHint')); const serial = ++state.inspecting; const snapshot = { ...state.config };
    const status = await api.request('status'); const info = await api.request('inspect', snapshot);
    if (serial !== state.inspecting) return; state.status = status; state.info = info; await save(); statusUI();
}
async function navigate(page) {
    if (page === state.page) return;
    if (state.page === 'profile_setup' && state.profileDirty && !confirm(state.language === 'en' ? 'Discard unsaved parameter changes' : '尚有未保存参数，确认放弃后离开')) return;
    capture(); await operation(save); await pageExit();
    if (state.page === 'profile_setup') state.createdProfileWizard = false;
    state.page = page;
    if (page === 'lifecycle' && state.config.profile) state.readiness = await api.request('readiness', { config: state.config });
    if (page === 'profile_setup' && state.config.profile) {
        state.profileEditor = await api.request('profile_editor_load', {config:state.config});
        state.profileDirty = false; state.profileError = '';
    }
    if (['run', 'model', 'workbench', 'diagnostics'].includes(page)) await operation(inspect);
    if (page === 'workbench' && state.config.profile) state.readiness = await api.request('readiness', {config: state.config});
    if (['model', 'workbench'].includes(page) && !state.modelData && state.info?.available?.model) await operation(() => loadModelData());
    render();
}
function bindPage() {
    document.querySelectorAll('[data-start-intent]').forEach(button => button.onclick = () => operation(async () => { const intent = button.dataset.startIntent; if (intent === 'description') { await pageExit(); state.entryIntent = 'description'; state.descriptionStep = 1; state.createdProfileWizard = false; state.descriptionAnalysis = null; state.descriptionDraft = {source:'',profile:'',destination:'',base_frame:'',tool_frame:'',joint_names:[]}; state.page = 'description'; render(); return; } state.entryIntent = intent; await refreshLibrary(); await pageExit(); state.page = 'library'; render(); }));
    if ($('library-home')) $('library-home').onclick = resetToStart;
    if ($('library-import-dir')) $('library-import-dir').onclick = () => operation(async () => { const source = await api.selectProfilePackage(); if (source) await importProfileSource(source); });
    if ($('library-import-file')) $('library-import-file').onclick = () => operation(async () => { const source = await api.selectProfile(); if (source) await importProfileSource(source); });
    document.querySelectorAll('[data-library-open]').forEach(button => button.onclick = () => blockingOperation(busyText('正在打开 Profile...', 'Opening Profile...'), () => openLibraryEntry(button.dataset.libraryOpen)));
    document.querySelectorAll('[data-library-remove]').forEach(button => button.onclick = () => { const id = button.dataset.libraryRemove; const entry = state.library.find(x => x.id === id); const dialog = $('confirm-dialog'); dialog.innerHTML = `<h3>${t('removeFromLibrary')}</h3><p>${state.language === 'en' ? 'Only the Library reference will be removed. Files on disk will not be deleted' : '只会移除 Profile Library 中的引用，不会删除磁盘上的 Profile 文件'}</p><dl class="kv"><dt>${t('profileName')}</dt><dd>${esc(entry?.profile || id)}</dd></dl><div class="actions"><button id="remove-cancel" class="button">${t('cancel')}</button><button id="remove-confirm" class="button danger">${t('removeFromLibrary')}</button></div>`; dialog.showModal(); $('remove-cancel').onclick = () => dialog.close(); $('remove-confirm').onclick = () => { dialog.close(); blockingOperation(busyText('正在从 Profile Library 移除...', 'Removing from Profile Library...'), async () => { await api.request('profile_library_remove', { id }); await refreshLibrary(); render(); }); }; });
    if ($('description-home')) $('description-home').onclick = resetToStart;
    if ($('description-browse')) $('description-browse').onclick = () => operation(async () => { const file = await api.selectDescription(); if (file) { state.descriptionDraft.source = file; const node = $('description-source'); if (node) node.value = file; } });
    if ($('description-browse-dir')) $('description-browse-dir').onclick = () => operation(async () => { const dir = await api.selectProfilePackage(); if (dir) { state.descriptionDraft.source = dir; const node = $('description-source'); if (node) node.value = dir; } });
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
    document.querySelectorAll('[data-stage-open]').forEach(button => button.onclick = () => { if (button.dataset.stage === 'dynamics_calibration') state.workbenchTab = 'calibration'; navigate(button.dataset.stageOpen); });
    document.querySelectorAll('[data-review-stage]').forEach(button => button.onclick = () => showReadinessReview(button.dataset.reviewStage));
    if ($('approve-gain-mode')) $('approve-gain-mode').onclick = showGainAdmission;
    if ($('ready-control')) $('ready-control').onclick = () => navigate('workbench');
    document.querySelectorAll('[data-profile-group]').forEach(button => button.onclick = () => {
        state.profileGroup = button.dataset.profileGroup;
        state.profileError = ''; render();
    });
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
        try { readProfileInput(input); }
        catch(error) { state.profileError = error.message; toast(error.message,true); }
    }));
    document.querySelectorAll('[data-profile-field]').forEach(field => {
        field.addEventListener('change', () => {
            try { readProfileInput(field); updateProfileSwitch(field); }
            catch(error) { state.profileError = error.message; field.closest('.parameter-field')?.classList.add('invalid'); toast(error.message,true); }
        });
        field.addEventListener('input', () => {
            if (field.type === 'number' && !field.value.trim()) return;
            try { readProfileInput(field); updateProfileSwitch(field); }
            catch { /* Validate incomplete fields on change */ }
        });
    });
    if ($('profile-add-identification')) $('profile-add-identification').onclick = () => { state.profileEditor.core.control.controller.identification_tracking = structuredClone(state.profileEditor.core.control.controller.rigid_tracking); state.profileDirty = true; render(); };
    if ($('profile-add-filter')) $('profile-add-filter').onclick = () => { state.profileEditor.core.control.runtime.joint_acc_filter_alpha=0.2;state.profileDirty=true;render(); };
    if ($('profile-add-park-ratio')) $('profile-add-park-ratio').onclick = () => { state.profileEditor.core.shutdown.compliant_phase_ratio=0.75;state.profileDirty=true;render(); };
    if ($('profile-add-continuity')) $('profile-add-continuity').onclick = () => { state.profileEditor.core.safety_policy.require_continuous_cmd=false;state.profileDirty=true;render(); };
    if ($('profile-add-correction')) $('profile-add-correction').onclick = () => { state.profileEditor.core.model.gravity_correction_path=''; state.profileDirty = true; render(); };
    if ($('profile-add-effort')) $('profile-add-effort').onclick = () => {
        const names = state.profileEditor.joint_names;
        const limit = state.profileEditor.urdf_effort_limits || {};
        state.profileEditor.core.safety_policy.max_effort_override = Object.fromEntries(names.map(j=>[j,Math.max(0,Number(limit[j] ?? 0))]));
        state.profileDirty = true; render();
    };
    if ($('profile-init-damiao')) $('profile-init-damiao').onclick = () => {
        state.profileEditor.hardware = profileHardwareDefaults(state.profileEditor.joint_names);
        state.profileEditor.profile_entry.hardware.plugin = 'serial_arm_hardware_damiao';
        state.profileDirty = true; render();
    };
    if ($('profile-reload')) $('profile-reload').onclick = () => operation(async () => {
        if (state.profileDirty && !confirm(state.language === 'en' ? 'Discard unsaved changes' : '放弃未保存的修改并重新读取磁盘配置')) return;
        state.profileEditor = await api.request('profile_editor_load',{config:state.config});
        state.profileDirty = false; state.profileError = ''; render();
    });
    if ($('profile-save')) $('profile-save').onclick = () => blockingOperation(busyText('正在验证并保存参数...', 'Validating and saving configuration...'), async () => {
        try {
            const payload = collectProfileEditor();
            state.profileEditor = await api.request('profile_editor_save',{config:state.config,payload});
            state.profileDirty=false; state.profileError='';
            state.info=null; await inspect();
            state.readiness=await api.request('readiness',{config:state.config});
            await refreshLibrary(); toast(t('saved')); render();
        } catch(error) {
            state.profileError = error.message || String(error);
            const found = profileGroups.find(([, , path]) => state.profileError.startsWith('core.'+path) || state.profileError.startsWith(path));
            if (found) state.profileGroup = found[0];
            else if (state.profileError.startsWith('hardware.') || state.profileError.startsWith('profile.')) state.profileGroup='hardware';
            render(); throw error;
        }
    });
    if ($('settings-back')) $('settings-back').onclick = async () => { await pageExit(); state.page = state.previousPage || 'start'; renderedPage = ''; shell(); };
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
        if ($('model-reset')) $('model-reset').onclick = () => modelView?.resetView();
        if ($('model-zero')) $('model-zero').onclick = () => { state.modelPositions = state.modelPositions.map(() => 0); document.querySelectorAll('[data-model-joint]').forEach((input, i) => { input.value = '0'; const number = document.querySelector(`[data-model-number="${i}"]`); if (number) number.value = '0.0000'; const value = $('joint-value-' + i); if (value) value.textContent = '0.000 rad'; }); queuePreview(); };
        document.querySelectorAll('[data-model-layer]').forEach(input => input.onchange = () => { state.modelLayers[input.dataset.modelLayer] = input.checked; modelView?.setLayers(state.modelLayers); });
        document.querySelectorAll('[data-model-select]').forEach(button => button.onclick = () => { const next = button.dataset.modelSelect; state.modelSelection = state.modelSelection === next ? '' : next; document.querySelector('.model-inspector').innerHTML = modelInspectorHtml(); document.querySelectorAll('.tree-row').forEach(x => x.classList.toggle('active', x.querySelector('[data-model-select]')?.dataset.modelSelect === state.modelSelection)); modelView?.select(state.modelSelection); });
        document.querySelectorAll('[data-model-visible]').forEach(button => button.onclick = event => { event.stopPropagation(); const kind = button.dataset.modelVisible, name = button.dataset.modelName; const next = !modelObjectVisible(kind, name); state.modelVisibility[kind][name] = next; button.classList.toggle('on', next); if (button.classList.contains('tree-eye')) button.textContent = next ? '●' : '○'; modelView?.setObjectVisible(kind, name, next); });
        document.querySelectorAll('[data-model-frame]').forEach(button => button.onclick = event => { event.stopPropagation(); const name = button.dataset.modelFrame; const next = !modelObjectVisible('linkFrames', name); state.modelVisibility.linkFrames[name] = next; button.classList.toggle('on', next); button.setAttribute('aria-pressed', next ? 'true' : 'false'); modelView?.setLinkFrameVisible(name, next); });
        const setAllObjects = visible => { for (const link of state.modelData?.model?.links || []) { state.modelVisibility.links[link.name] = visible; state.modelVisibility.linkFrames[link.name] = visible; modelView?.setObjectVisible('links', link.name, visible); modelView?.setLinkFrameVisible(link.name, visible); } for (const joint of state.modelData?.model?.joints || []) { state.modelVisibility.joints[joint.name] = visible; modelView?.setObjectVisible('joints', joint.name, visible); } document.querySelectorAll('[data-model-visible]').forEach(button => { button.classList.toggle('on', visible); if (button.classList.contains('tree-eye')) button.textContent = visible ? '●' : '○'; }); document.querySelectorAll('[data-model-frame]').forEach(button => { button.classList.toggle('on', visible); button.setAttribute('aria-pressed', visible ? 'true' : 'false'); }); };
        if ($('model-show-all')) $('model-show-all').onclick = () => setAllObjects(true);
        if ($('model-hide-all')) $('model-hide-all').onclick = () => setAllObjects(false);
        const setAllLayers = visible => { for (const key of Object.keys(state.modelLayers)) state.modelLayers[key] = visible; document.querySelectorAll('[data-model-layer]').forEach(input => input.checked = visible); modelView?.setLayers(state.modelLayers); };
        if ($('model-layers-all')) $('model-layers-all').onclick = () => setAllLayers(true);
        if ($('model-layers-none')) $('model-layers-none').onclick = () => setAllLayers(false);
        document.querySelectorAll('[data-model-joint]').forEach(input => input.oninput = () => { const i = Number(input.dataset.modelJoint); state.modelPositions[i] = Number(input.value); const number = document.querySelector(`[data-model-number="${i}"]`); if (number) number.value = Number(input.value).toFixed(4); const value = $('joint-value-' + i); if (value) value.textContent = Number(input.value).toFixed(3) + ' rad'; queuePreview(); });
        document.querySelectorAll('[data-model-number]').forEach(input => input.onchange = () => { const i = Number(input.dataset.modelNumber); const slider = document.querySelector(`[data-model-joint="${i}"]`); let value = Number(input.value); if (!Number.isFinite(value)) return; value = Math.max(Number(slider.min), Math.min(Number(slider.max), value)); state.modelPositions[i] = value; slider.value = String(value); input.value = value.toFixed(4); queuePreview(); });
    }
    if (state.page === 'workbench') {
        document.querySelectorAll('[data-workbench-tab]').forEach(b => b.onclick = () => { state.workbenchTab = b.dataset.workbenchTab; render(); });
        document.querySelectorAll('[data-calibration-com-scale]').forEach(button => button.onclick = () => { state.modelCalibrationDisplayScale = Number(button.dataset.calibrationComScale); document.querySelectorAll('[data-calibration-com-scale]').forEach(other => { const active = Number(other.dataset.calibrationComScale) === state.modelCalibrationDisplayScale; other.classList.toggle('active', active); other.setAttribute('aria-pressed', String(active)); }); const hint = document.querySelector('.calibration-com-hint'); if (hint) hint.textContent = state.modelCalibrationDisplayScale === 1 ? (state.language === 'en' ? 'True 3D locations' : '真实三维位置') : (state.language === 'en' ? 'Visual displacement only' : '仅放大显示位移，不更改实际坐标'); updateModelCalibrationComparison(); });
        if ($('calibration-com-unchanged')) $('calibration-com-unchanged').onchange = event => { state.modelCalibrationShowUnchanged = event.target.checked; updateModelCalibrationComparison(); };
        if ($('start-workbench')) $('start-workbench').onclick = () => blockingOperation(busyText('正在检查工作台连接条件...', 'Checking workspace connection...'), startWorkbench);
        if ($('calibration-connect-workbench')) $('calibration-connect-workbench').onclick = () => blockingOperation(busyText('正在检查工作台连接条件...', 'Checking workspace connection...'), startWorkbench);
        for (const [id, action] of [['calibration-activate', 'activate'], ['calibration-safe-park', 'park'], ['calibration-fault-clear', 'clear_fault'], ['calibration-fault-disable', 'deactivate']]) {
            const button = $(id);
            if (button) button.onclick = () => {
                if (confirmCalibrationPark(action)) return;
                return blockingOperation(actionBusyMessage(action), async () => {
                const snapshot = await workbenchCommand(action);
                if (snapshot && typeof snapshot === 'object' && snapshot.robot_state) applyWorkbenchSnapshot(snapshot);
                refreshCalibrationQuickControls();
                refreshCalibrationActionControls();
                refreshGainApplyControls();
                });
            };
        }
        if ($('stop-workbench')) $('stop-workbench').onclick = () => confirmWorkbenchDisconnect();
        for (const id of ['workbench-force-disconnect', 'workbench-recover-disconnect']) if ($(id)) $(id).onclick = () => confirmUnresponsiveDisconnect();
        document.querySelectorAll('[data-workbench-action]').forEach(b => b.onclick = () => blockingOperation(actionBusyMessage(b.dataset.workbenchAction), () => workbenchCommand(b.dataset.workbenchAction)));
        document.querySelectorAll('[data-impedance-mode]').forEach(b => b.onclick = () => switchWorkbenchMode('impedance', b.dataset.impedanceMode));
        document.querySelectorAll('[data-feedforward-mode]').forEach(b => b.onclick = () => switchWorkbenchMode('feedforward', b.dataset.feedforwardMode));
        document.querySelectorAll('[data-target-joint]').forEach(input => input.oninput = () => { state.workbenchTargets[Number(input.dataset.targetJoint)] = Number(input.value); state.commandState = 'edited'; document.querySelector('.command-state')?.replaceChildren(document.createTextNode(`${t('commandState')} ${commandStateText(state.commandState)}`)); });
        document.querySelectorAll('[data-delta-joint]').forEach(input => input.oninput = () => { state.workbenchDelta[Number(input.dataset.deltaJoint)] = Number(input.value); state.commandState = 'edited'; });
        if ($('workbench-speed')) $('workbench-speed').onchange = () => { const v = Number($('workbench-speed').value); if (Number.isFinite(v) && v > 0 && v <= 1) state.speedScale = v; };
        if ($('execute-absolute')) $('execute-absolute').disabled = state.readiness?.modes?.[state.telemetry?.impedance_mode] !== 'verified';
        if ($('execute-relative')) $('execute-relative').disabled = state.readiness?.modes?.[state.telemetry?.impedance_mode] !== 'verified';
        if ($('execute-absolute')) $('execute-absolute').onclick = () => blockingOperation(busyText('正在提交绝对关节目标...', 'Submitting absolute joint target...'), async () => { state.commandState = 'submitting'; await workbenchRequest('move_to', { positions: state.workbenchTargets, speed_scale: state.speedScale }); state.pendingTarget = state.workbenchTargets.slice(); state.commandState = 'accepted'; render(); });
        if ($('execute-relative')) $('execute-relative').onclick = () => blockingOperation(busyText('正在提交相对关节目标...', 'Submitting relative joint target...'), async () => { state.commandState = 'submitting'; const base = state.telemetry?.joint?.pos || []; await workbenchRequest('move_relative', { delta: state.workbenchDelta, speed_scale: state.speedScale }); state.pendingTarget = base.map((v, i) => v + (state.workbenchDelta[i] || 0)); state.commandState = 'accepted'; render(); });
        if ($('hold-motion')) $('hold-motion').onclick = () => blockingOperation(busyText('正在保持当前位置...', 'Holding current position...'), () => workbenchRequest('hold'));
        if ($('observer-mode')) $('observer-mode').onchange = () => { if (!state.tuneDraft) state.tuneDraft = structuredClone(state.admittance || {}); state.tuneDraft.observer_mode = $('observer-mode').value; };
        document.querySelectorAll('[data-tune-key]').forEach(input => input.onchange = () => { if (!state.tuneDraft) state.tuneDraft = structuredClone(state.admittance || {}); const key = input.dataset.tuneKey, i = Number(input.dataset.tuneIndex); state.tuneDraft[key] = Array.isArray(state.tuneDraft[key]) ? state.tuneDraft[key].slice() : []; state.tuneDraft[key][i] = Number(input.value); });
        if ($('apply-tuning')) $('apply-tuning').onclick = () => blockingOperation(busyText('正在应用导纳参数...', 'Applying admittance parameters...'), applyTuning);
        if ($('reset-tuning')) $('reset-tuning').onclick = () => { state.tuneDraft = structuredClone(state.admittance || {}); render(); };
        if ($('preview-config')) $('preview-config').onclick = () => blockingOperation(busyText('正在生成配置差异...', 'Preparing configuration diff...'), async () => { state.savePreview = await api.request('workbench_config_preview', { config: state.config }); render(); });
        if ($('save-config')) $('save-config').onclick = () => blockingOperation(busyText('正在保存配置...', 'Saving configuration...'), async () => { const result = await api.request('workbench_config_save', { config: state.config, expected_sha: state.savePreview.sha256 }); state.savePreview = null; toast(`Saved ${result.path}`); render(); });
        document.querySelectorAll('[data-calibration-begin]').forEach(b => b.onclick = () => blockingOperation(busyText('正在启动标定任务...', 'Starting calibration task...'), () => workbenchRequest('calibration_begin', { kind: b.dataset.calibrationBegin })));
        if ($('calibration-capture')) $('calibration-capture').onclick = () => blockingOperation(busyText('正在采集当前姿态...', 'Capturing current pose...'), () => workbenchRequest('calibration_capture'));
        if ($('calibration-finish')) $('calibration-finish').onclick = () => blockingOperation(busyText('正在拟合并验证标定结果...', 'Fitting and validating calibration...'), finishCalibrationStage);
        if ($('calibration-cancel')) $('calibration-cancel').onclick = () => blockingOperation(busyText('正在取消标定任务...', 'Cancelling calibration task...'), async () => { await workbenchRequest('calibration_cancel'); state.calibrationFlow = ''; render(); });
        if ($('friction-record-start')) $('friction-record-start').onclick = () => blockingOperation(busyText('正在开始示教记录...', 'Starting demonstration recording...'), async () => { if (state.calibrationFlow === 'friction-ready') state.calibrationFlow = 'friction'; await workbenchRequest('friction_record_begin'); render(); });
        if ($('friction-record-stop')) $('friction-record-stop').onclick = () => blockingOperation(busyText('正在结束示教并处理轨迹...', 'Stopping demonstration and processing trajectory...'), () => workbenchRequest('friction_record_stop'));
        if ($('friction-replay')) $('friction-replay').onclick = () => frictionConfirm();
        if ($('export-session')) $('export-session').onclick = () => blockingOperation(busyText('正在导出会话数据...', 'Exporting session data...'), async () => { const r = await api.request('workbench_export', { profile: state.config.profile, core: state.info?.resources?.core || '' }); toast(`Exported ${r.samples} samples · ${r.path}`); });
        if ($('apply-gravity')) $('apply-gravity').onclick = () => blockingOperation(busyText('正在应用重力比例...', 'Applying gravity scale...'), async () => { const values = [...document.querySelectorAll('[data-gravity-index]')].sort((a, b) => Number(a.dataset.gravityIndex) - Number(b.dataset.gravityIndex)).map(x => Number(x.value)); await workbenchRequest('set_gravity_scale', { values }); state.savePreview = null; });
        if ($('diagnostic-frame')) $('diagnostic-frame').onchange = () => { state.selectedFrame = $('diagnostic-frame').value; render(); };
        document.querySelectorAll('[data-calibration-entry]').forEach(button => button.onclick=()=>{
            if (modelCalibrationActive() && button.dataset.calibrationEntry!==state.calibrationEntry) return;
            state.calibrationEntry=button.dataset.calibrationEntry;
            render();
        });
        document.querySelectorAll('[data-tool-scan]').forEach(button=>button.onclick=()=>blockingOperation(busyText('正在扫描任务记录...', 'Scanning task records...'),async()=>{
            const kind=button.dataset.toolScan;
            state.calibrationToolRecords[kind]=await api.request('calibration_tool_records',{kind});
            render();
        }));
        document.querySelectorAll('[data-tool-load]').forEach(button=>button.onclick=()=>blockingOperation(busyText('正在检查任务记录...', 'Loading task...'),async()=>{
            const kind=button.dataset.toolLoad;
            const directory=await api.selectResources();
            if(!directory)return;
            const selected=await api.request('calibration_tool_load',{kind,directory});
            await selectCalibrationToolRecord(kind,selected);
        }));
        document.querySelectorAll('[data-tool-select]').forEach(button=>button.onclick=()=>blockingOperation(busyText('正在读取历史任务...', 'Loading task...'),async()=>{
            const kind=button.dataset.toolSelect;
            await selectCalibrationToolRecord(kind,state.calibrationToolRecords[kind][Number(button.dataset.toolIndex)]);
        }));
        document.querySelectorAll('[data-tool-export]').forEach(button=>button.onclick=()=>blockingOperation(busyText('正在生成标定成果...', 'Exporting calibration result...'),async()=>{
            const kind=button.dataset.toolExport;
            const directory=(kind==='gravity' ? gravityCandidatePreviewState().directory : null)
                || state.calibrationToolSelected[kind]?.directory||state.modelCalibrationDirectory
                || state.telemetry?.model_calibration?.directory;
            if(!directory)throw new Error('请先选择含有有效标定成果的任务');
            const destination=await api.selectResources();
            if(!destination)return;
            const report=kind==='dynamics' ? (state.advancedReport?.report||state.advancedReport?.candidate_urdf?.replace(/\/[^/]+$/, '/identification-report.json')||'') : '';
            const result=await api.request('calibration_tool_export',{kind,directory,destination,report,candidate_source:kind==='gravity'?state.modelCalibrationCandidateSource:'online'});
            toast((state.language==='en'?'Calibration result exported: ':'标定成果已导出 ') + result.path + (result.review_required ? (state.language==='en'?' · Review only, friction disabled when unverified':' · 仅供人工审核，未通过的摩擦补偿已禁用') : '')); 
        }));
        if ($('calibration-finish-teaching')) $('calibration-finish-teaching').onclick=()=>blockingOperation(busyText('正在保存示教任务...', 'Saving demonstration...'),async()=>{
            const result=await workbenchRequest('model_calibration_finish_teaching',modelCalibrationTaskParams());
            if(state.telemetry)state.telemetry.model_calibration=result;
            state.modelCalibrationDirectory=result.directory||'';
            render();
        });
        if ($('calibration-tool-import')) $('calibration-tool-import').onclick=()=>blockingOperation(busyText('正在导入轨迹...', 'Importing saved trajectory...'),async()=>{
            const record=state.calibrationToolSelected[state.calibrationEntry];
            if(!record?.directory)throw new Error('请先选择示教轨迹任务');
            const result=await workbenchRequest('model_calibration_import_trajectory',{directory:record.directory,...captureModelCalibrationOptions({validate:false})});
            if(state.telemetry)state.telemetry.model_calibration=result;
            state.modelCalibrationDirectory=result.directory;
            render();
            toast('轨迹已导入，但尚未执行任何自动运动；请先检查并对齐起点');
        });
        document.querySelectorAll('[data-candidate-undo]').forEach(b=>b.onclick=()=>temporaryCandidateAction('restore',b.dataset.candidateUndo));
        if ($('calibration-gain-load-live')) $('calibration-gain-load-live').onclick=()=>blockingOperation(busyText('正在读取当前增益任务...', 'Loading current gain task...'),async()=>{
            const directory=state.telemetry?.model_calibration?.directory;
            if(!directory)throw new Error('当前任务尚无有效目录');
            const record=await api.request('calibration_tool_load',{kind:'gain',directory});
            state.calibrationToolSelected.gain=record;render();
        });
        if ($('gain-target')) $('gain-target').onchange=()=>{ state.gainMode=$('gain-target').value; render(); };
        if ($('calibration-gain-temp-apply')) $('calibration-gain-temp-apply').onclick=()=>temporaryCandidateAction('apply','gain');
        if ($('calibration-gain-temp-restore')) $('calibration-gain-temp-restore').onclick=()=>temporaryCandidateAction('restore','gain');
        if ($('calibration-gravity-temp-apply')) $('calibration-gravity-temp-apply').onclick=()=>temporaryCandidateAction('apply','gravity');
        if ($('calibration-gravity-temp-restore')) $('calibration-gravity-temp-restore').onclick=()=>temporaryCandidateAction('restore','gravity');
        if ($('advanced-temp-apply')) $('advanced-temp-apply').onclick=()=>temporaryCandidateAction('apply','dynamics');
        if ($('advanced-temp-restore')) $('advanced-temp-restore').onclick=()=>temporaryCandidateAction('restore','dynamics');
        if ($('advanced-temp-restore-unconditional')) $('advanced-temp-restore-unconditional').onclick=()=>temporaryCandidateAction('restore','dynamics');
        if ($('advanced-target')) $('advanced-target').onchange = () => { state.advancedTarget=$('advanced-target').value; state.advancedReport=null; render(); };
        document.querySelectorAll('[data-advanced-locked]').forEach(input => input.onchange=()=>{ const name=input.dataset.advancedLocked; const names=new Set(state.advancedLocked); if(input.checked)names.add(name);else names.delete(name);state.advancedLocked=Array.from(names);state.advancedReport=null;const wasOpen=!!input.closest('details')?.open;render();if(wasOpen){const picker=document.querySelector('.calibration-locks');if(picker)picker.open=true;} });
        if ($('advanced-load-record')) $('advanced-load-record').onclick = () => blockingOperation(busyText('正在读取历史标定...', 'Loading calibration...'), async()=>{ const directory=await api.selectResources();if(!directory)return;const loaded=await api.request('model_calibration_load',{directory});state.modelCalibrationOffline=loaded;state.modelCalibrationDirectory=loaded.directory||directory;state.advancedAssessment=null;state.advancedReport=null;state.advancedLocked=advancedTrustedLinks(loaded);render();await alignHistoricalCalibrationPreview(loaded);await assessLoadedCalibrationRecord(loaded); });
        if ($('advanced-assess-record')) $('advanced-assess-record').onclick = () => blockingOperation(busyText('正在检查采样信息量...', 'Assessing data...'), async()=>{ state.advancedAssessment=await api.request('model_calibration_assess',{directory:state.modelCalibrationDirectory||currentModelCalibration()?.directory});render(); });
        if ($('advanced-run')) $('advanced-run').onclick = () => blockingOperation(busyText('正在离线辨识并检查候选模型...', 'Identifying and validating offline candidate...'), async()=>{ if(!state.modelCalibrationOffline?.metadata)throw new Error('请先加载带有效来源信息的历史任务');const dir=state.modelCalibrationDirectory||state.modelCalibrationOffline.directory;const available=calibrationSelectableLinks().map(l=>l.name);
            if (!available.length) throw new Error('历史任务缺少可验证的可辨识 Link 清单');
            const invalid=state.advancedLocked.filter(n=>!available.includes(n));
            if (invalid.length) throw new Error('可信 Link 与当前任务模型不一致，请重新选择');
            if (state.advancedLocked.length === available.length) throw new Error('所有可辨识 Link 均已锁定，请至少保留一个');
            const locked=available.filter(n=>state.advancedLocked.includes(n));
            const result=await api.request('model_calibration_advanced',{directory:dir,mode:state.advancedTarget,locked_links:locked});state.advancedReport=result;state.calibrationToolSelected.dynamics=await api.request('calibration_tool_load',{kind:'dynamics',directory:dir});render(); });
        if ($('calibration-all-in-one')) $('calibration-all-in-one').onclick = () => blockingOperation(busyText('正在启动完整标定流程...', 'Starting full calibration workflow...'), async () => { state.calibrationFlow = 'static'; await workbenchRequest('calibration_begin', { kind: 'static' }); render(); });
        if ($('calibration-preview-save')) $('calibration-preview-save').onclick = () => blockingOperation(busyText('正在整理标定结果差异...', 'Preparing calibration result diff...'), async () => { state.admittance = await workbenchRequest('get_admittance'); state.savePreview = await api.request('workbench_config_preview', { config: state.config }); render(); });
        if ($('calibration-save-config')) $('calibration-save-config').onclick = () => blockingOperation(busyText('正在保存标定参数...', 'Saving calibration parameters...'), async () => { const result = await api.request('workbench_config_save', { config: state.config, expected_sha: state.savePreview.sha256 }); state.savePreview = null; toast(`Saved ${result.path}`); render(); });
        for (const id of ['model-cal-pose-budget', 'model-cal-validation', 'model-cal-regularization', 'model-cal-svd', 'model-cal-information']) {
            const input = $(id);
            if (input) input.onchange = () => operation(() => captureModelCalibrationOptions({validate: false}));
        }
        if ($('model-cal-mode-global')) $('model-cal-mode-global').onclick = () => operation(() => {
            captureModelCalibrationOptions({validate: false});
            state.modelCalibrationOptions.mode = 'global';
            state.modelCalibrationOptions.locked_links = [];
            state.modelCalibrationSavePreview = null;
            render();
        });
        if ($('model-cal-mode-local')) $('model-cal-mode-local').onclick = () => operation(async () => {
            captureModelCalibrationOptions({validate: false});
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
            editCalibrationLockedLinks({onCancel: () => {
                state.modelCalibrationOptions.mode = 'global';
                state.modelCalibrationOptions.locked_links = [];
                render();
            }});
        });
        if ($('model-cal-lock-select')) $('model-cal-lock-select').onclick = () => operation(async () => {
            await ensureCalibrationSelectableLinks();
            editCalibrationLockedLinks();
        });
        if ($('model-calibration-teach-start')) $('model-calibration-teach-start').onclick = () => blockingOperation(busyText('正在启动拖动示教...', 'Starting demonstration...'), async () => { captureModelCalibrationOptions(); state.modelCalibrationCandidateSource = 'online'; state.modelCalibrationOffline = null; state.modelCalibrationSavePreview = null; state.modelCalibrationExport = null; state.modelCalibrationSaved = null; const result = await workbenchRequest('model_calibration_teach_begin', state.modelCalibrationOptions); if (state.telemetry) state.telemetry.model_calibration = result; state.modelCalibrationDirectory = result.directory || ''; render(); updateModelCalibrationTaskOverlay(); });
        if ($('model-calibration-teach-stop')) $('model-calibration-teach-stop').onclick = () => finishTeachingSafely();
        if ($('model-calibration-teach-cleanup')) $('model-calibration-teach-cleanup').onclick = () => finishTeachingSafely(true);
        if ($('model-calibration-replay-confirm')) $('model-calibration-replay-confirm').onclick = modelCalibrationReplayConfirm;
        if ($('model-calibration-pause')) $('model-calibration-pause').onclick = () => modelCalibrationQuick('model_calibration_pause');
        if ($('model-calibration-resume')) $('model-calibration-resume').onclick = resumeModelCalibrationWithConfirmation;
        if ($('model-calibration-cancel')) $('model-calibration-cancel').onclick = () => modelCalibrationQuick('model_calibration_cancel');
        if ($('model-calibration-apply')) $('model-calibration-apply').onclick = () => blockingOperation(busyText('正在应用候选重力校正...', 'Applying candidate gravity correction...'), async () => { const result = await workbenchRequest('model_calibration_apply', modelCalibrationTaskParams()); if (state.telemetry) state.telemetry.model_calibration = result; render(); });
        if ($('model-calibration-restore')) $('model-calibration-restore').onclick = () => blockingOperation(busyText('正在恢复运行时原配置...', 'Restoring original runtime configuration...'), async () => { const result = await workbenchRequest('model_calibration_restore', modelCalibrationTaskParams()); if (state.telemetry) state.telemetry.model_calibration = result; render(); });
        if ($('model-calibration-load-record')) $('model-calibration-load-record').onclick = () => blockingOperation(busyText('正在加载模型校正记录...', 'Loading model calibration record...'), async () => { const directory = await api.selectResources(); if (!directory) return; const loaded = await api.request('model_calibration_load', { directory }); adoptGravityRecord(loaded); state.advancedAssessment=null; state.advancedReport=null; state.advancedLocked=advancedTrustedLinks(loaded); state.modelCalibrationDirectory = loaded.directory || directory; if (loaded.metadata?.calibration_options) state.modelCalibrationOptions = { ...state.modelCalibrationOptions, ...loaded.metadata.calibration_options };
            if (loaded.result?.gravity_result?.identification_mode) state.modelCalibrationOptions = { ...state.modelCalibrationOptions, mode: loaded.result.gravity_result.identification_mode, locked_links: loaded.result.gravity_result.locked_links || [] }; state.modelCalibrationSavePreview = null; state.modelCalibrationExport = null; state.modelCalibrationSaved = null; render(); await alignHistoricalCalibrationPreview(loaded); await assessLoadedCalibrationRecord(loaded); });
        if ($('model-calibration-scan-records')) $('model-calibration-scan-records').onclick = () => blockingOperation(busyText('正在检查已有任务...', 'Scanning saved tasks...'), async () => { state.modelCalibrationRecords = await api.request('model_calibration_records', {}); render(); });
        if ($('model-calibration-refresh-record')) $('model-calibration-refresh-record').onclick = () => blockingOperation(busyText('正在重新检查记录...', 'Refreshing record...'), async () => { const current = state.modelCalibrationOffline; if (!current?.directory) return; adoptGravityRecord(await api.request('model_calibration_load', {directory: current.directory})); render(); await alignHistoricalCalibrationPreview(state.modelCalibrationOffline); await assessLoadedCalibrationRecord(state.modelCalibrationOffline); });
        state.modelCalibrationRecords.forEach((saved, index) => {
            const button = $(`model-calibration-record-${index}`);
            if (button) button.onclick = () => blockingOperation(busyText('正在读取任务记录...', 'Reading task...'), async () => { const fresh = await api.request('model_calibration_load', {directory: saved.directory}); adoptGravityRecord(fresh); state.advancedAssessment=null; state.advancedReport=null; state.advancedLocked=advancedTrustedLinks(fresh); state.modelCalibrationDirectory = fresh.directory; render(); await alignHistoricalCalibrationPreview(fresh); await assessLoadedCalibrationRecord(fresh); });
        });
        if ($('model-calibration-back-live')) $('model-calibration-back-live').onclick = () => { state.modelCalibrationOffline = null; state.modelCalibrationCandidateSource = 'online'; state.modelCalibrationPreviewAlignment = null; state.modelCalibrationDirectory = state.telemetry?.model_calibration?.directory || ''; render(); };
        if ($('model-calibration-alignment-begin')) $('model-calibration-alignment-begin').onclick = () => {
            const dialog = $('confirm-dialog');
            dialog.innerHTML = `<h3>${state.language === 'en' ? 'Supervised manual alignment' : '人工引导返回回放起点'}</h3><p>${state.language === 'en' ? 'This does NOT command the robot to the target. The arm will enter compliant drag; gravity can cause it to move or sag. Support the robot physically, check load/cables and keep others clear.' : '这不会发送自动归位轨迹机械臂将进入柔性拖拽，重力可能导致关节移动或下垂请先支撑机械臂，检查负载、线缆和周围空间，禁止他人进入'}</p><label class="confirmation"><input type="checkbox" id="model-cal-alignment-supported"><span>${state.language === 'en' ? 'Arm is supported and manual dragging is safe in this area' : '机械臂已被支撑，周围允许安全地人工拖拽'}</span></label><div class="actions"><button id="model-cal-alignment-cancel" class="button">${t('cancel')}</button><button id="model-cal-alignment-accept" class="button danger" disabled>${state.language === 'en' ? 'Enter manual drag' : '确认进入柔性拖拽'}</button></div>`;
            dialog.showModal();
            $('model-cal-alignment-supported').onchange = () => { $('model-cal-alignment-accept').disabled = !$('model-cal-alignment-supported').checked; };
            $('model-cal-alignment-cancel').onclick = () => dialog.close();
            $('model-cal-alignment-accept').onclick = () => { dialog.close(); blockingOperation(busyText('正在进入引导拖拽...', 'Entering guided drag...'), async () => { const result = await workbenchRequest('model_calibration_alignment_begin', modelCalibrationTaskParams({supported: true})); if (state.telemetry) state.telemetry.model_calibration = result; render(); }); };
        };
        if ($('model-calibration-alignment-finish')) $('model-calibration-alignment-finish').onclick = () => blockingOperation(busyText('正在保持当前姿态...', 'Holding current pose...'), async () => { const result = await workbenchRequest('model_calibration_alignment_finish', modelCalibrationTaskParams()); if (state.telemetry) state.telemetry.model_calibration = result; render(); });
        if ($('model-calibration-import-trajectory')) $('model-calibration-import-trajectory').onclick = () => blockingOperation(busyText('正在检查并恢复示教轨迹...', 'Importing demonstrated trajectory...'), async () => {
            // Always reread the chosen folder. A live fault may have saved the
            // trajectory after the GUI last rendered the record panel.
            const selected = state.modelCalibrationOffline;
            if (!selected?.directory) throw new Error('请先选择任务记录');
            const record = await api.request('model_calibration_load', {directory: selected.directory});
            state.modelCalibrationOffline = record;
            if (!record.can_resume) { render(); throw new Error('当前记录尚无有效的 trajectory.csv 或 trajectory.checkpoint.csv，请刷新任务列表并确认目录'); }
            if (!workbenchConnected() || state.telemetry?.robot_state !== 'ACTIVE') throw new Error('恢复轨迹需要工作台已连接且机械臂 ACTIVE');
            const current = state.telemetry?.model_calibration?.phase || 'idle';
            if (!['idle', 'complete', 'cancelled', 'failed'].includes(current)) throw new Error('请先取消或结束当前标定任务');
            // No motor movement: this only stages a checked copy of the trajectory.
            const restored = await workbenchRequest('model_calibration_import_trajectory', {directory: record.directory, ...captureModelCalibrationOptions()});
            if (state.telemetry) state.telemetry.model_calibration = restored;
            state.modelCalibrationOffline = null;
            state.modelCalibrationCandidateSource = 'online';
            state.modelCalibrationDirectory = restored.directory || '';
            render();
            toast(state.language === 'en' ? 'Trajectory imported. Confirm replay separately to move.' : '轨迹已恢复，机械臂尚未回放，请检查预计时间后单独确认');
        });
        if ($('model-calibration-recompute')) $('model-calibration-recompute').onclick = () => blockingOperation(busyText('正在离线重算候选模型...', 'Recomputing candidate offline...'), async () => { if (!state.modelCalibrationDirectory) throw new Error('model calibration task directory is missing'); const recomputed = await api.request('model_calibration_recompute', { config: state.config, directory: state.modelCalibrationDirectory, ...captureModelCalibrationOptions() }); if (!state.modelCalibrationOffline) adoptGravityRecord(await api.request('model_calibration_load', {directory: state.modelCalibrationDirectory})); state.modelCalibrationOffline.recomputed_candidate=recomputed.candidate; adoptGravityRecord(state.modelCalibrationOffline); state.modelCalibrationSavePreview = null; state.modelCalibrationExport = null; state.modelCalibrationFrictionExport = null; render(); });
        if ($('model-calibration-preview-save')) $('model-calibration-preview-save').onclick = () => blockingOperation(busyText('正在准备候选保存差异...', 'Preparing candidate save diff...'), async () => { const directory = state.modelCalibrationDirectory || currentModelCalibration()?.directory; if (!directory) throw new Error('model calibration task directory is missing'); state.modelCalibrationSavePreview = await api.request('model_calibration_preview_save', { config: state.config, directory }); render(); });
        if ($('model-calibration-save')) $('model-calibration-save').onclick = () => blockingOperation(busyText('正在保存候选重力校正...', 'Saving candidate gravity correction...'), async () => { const directory = state.modelCalibrationDirectory || currentModelCalibration()?.directory; if (!directory || !state.modelCalibrationSavePreview) throw new Error('save preview is required'); if (state.modelCalibrationCandidateSource === 'offline') throw new Error('离线动力学候选不能直接保存为在线静态重力参数'); state.modelCalibrationSaved = await api.request('model_calibration_save', { config: state.config, directory, expected_sha: state.modelCalibrationSavePreview.sha256 }); state.modelCalibrationSavePreview = null; toast(`Saved ${state.modelCalibrationSaved.gravity_correction_path}`); render(); });
        if ($('model-calibration-restore-config')) $('model-calibration-restore-config').onclick = () => blockingOperation(busyText('正在恢复保存前配置...', 'Restoring pre-save configuration...'), async () => { const directory = state.modelCalibrationDirectory || currentModelCalibration()?.directory; if (!directory) throw new Error('model calibration task directory is missing'); const result = await api.request('model_calibration_restore_config', { config: state.config, directory }); state.modelCalibrationSaved = null; state.modelCalibrationSavePreview = null; toast(`Restored ${result.core_path}`); render(); });
        if (state.workbenchTab === 'diagnostics') setTimeout(drawWorkbenchChart, 0);
        setTimeout(updateWorkbenchControlState, 0);
    }
    document.querySelectorAll('[data-mode]').forEach(b => { b.disabled = active(); b.onclick = () => { if (active()) return; state.mode = b.dataset.mode; operation(save); render(); }; });
    if (state.page === 'robot' && active()) document.querySelectorAll('#page input,#page select,#source-builtin,#source-external,#browse-profile,#refresh-profiles,#add-root').forEach(e => e.disabled = true);
    if ($('launch-session')) { $('launch-session').disabled = state.busy || active() || !state.info?.available?.[state.mode]; $('launch-session').onclick = () => blockingOperation(busyText('正在检查启动条件...', 'Checking launch conditions...'), async () => { await inspect(); if (!state.info?.available?.[state.mode]) throw new Error(t('modeMissing')); hardwareConfirm(); }); }
    if ($('check-now')) $('check-now').onclick = () => blockingOperation(busyText('正在检查环境与资源...', 'Checking environment and resources...'), async () => { await inspect(); render(); });
    document.querySelectorAll('[data-theme]').forEach(b => b.onclick = event => operation(() => switchTheme(b.dataset.theme, event, true)));
    document.querySelectorAll('[data-language]').forEach(b => b.onclick = async () => { state.language = b.dataset.language; document.documentElement.lang = state.language; await operation(save); shell(); });
}
async function workbenchRequest(method, params = {}) { if (workbenchRecoveryRequired()) throw new Error('native session is unresponsive; force disconnect after making hardware safe'); if (!workbenchConnected()) throw new Error('workspace session is not connected'); return api.request('workbench_request', { method, params }); }
async function loadAdmittance() { if (!workbenchConnected()) return; state.admittance = await workbenchRequest('get_admittance'); state.tuneDraft = structuredClone(state.admittance); }
async function startWorkbench() { await inspect(); if (!state.info?.available?.workbench) throw new Error('Structured workspace is unavailable'); if (state.info.write_enabled === false) return connectWorkbench(false); return workbenchConfirm(); }
async function connectWorkbench(confirmed) { state.readiness = await api.request('readiness', {config: state.config}); state.busy = true; render(); try { state.session = await api.request('workbench_start', { config: state.config, confirmed, fingerprint: state.info?.fingerprint }); await loadAdmittance(); state.calibrationRuntimeCandidates = await api.request('calibration_candidate_status',{}); } finally { state.busy = false; render(); } }
function confirmCalibrationPark(action) {
    if (action !== 'park' || !modelCalibrationActive()) return false;
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
    if (!workbenchRecoveryRequired()) return;
    const dialog = $('confirm-dialog');
    if (!dialog || dialog.open) return;
    const en = state.language === 'en';
    dialog.innerHTML = `<h3>${en ? 'Disconnect unresponsive control process' : '确认强制断开无响应的控制进程'}</h3><p>${en ? 'The prior hardware command may still be running. This action stops the control process but cannot verify motor disable, safe parking, or temporary candidate restoration' : '上一条硬件操作可能仍在执行，强制断连只能终止控制进程，无法确认电机已失能、机械臂已停放或临时参数已正常撤销'}</p><p>${en ? 'First physically secure the robot and cut actuator power if necessary. Do not rely on software controls. Reconnect only after hardware power and wiring are verified' : '请先确认机械臂获得可靠支撑，必要时通过硬件急停或断电确保安全，不要依赖失去响应的软件控制，检查供电和线缆后再重新连接'}</p><div class="actions"><button class="button" id="cancel-unresponsive-disconnect">${t('cancel')}</button><button class="button danger" id="confirm-unresponsive-disconnect">${en ? 'I secured hardware, disconnect' : '已确认硬件安全，强制断连'}</button></div>`;
    dialog.showModal();
    $('cancel-unresponsive-disconnect').onclick = () => dialog.close();
    $('confirm-unresponsive-disconnect').onclick = () => { dialog.close(); blockingOperation(busyText('正在终止无响应的控制会话...', 'Terminating unresponsive control session...'), async () => {
        await api.request('workbench_stop', {force:true});
        toast(busyText('已请求终止控制进程，未确认电机失能，请检查硬件再重新连接', 'Control process termination requested; hardware disable was not verified'));
    }); };
}
function confirmWorkbenchDisconnect() {
    const en = state.language === 'en';
    const kinds = activeTemporaryOverrides();
    if (!kinds.length) {
        return blockingOperation(busyText('正在安全断开工作台...', 'Safely disconnecting workspace...'), async () => {
            await api.request('workbench_stop', {});
        });
    }
    const dialog = $('confirm-dialog');
    if (!dialog || dialog.open) return;
    const names = kinds.map(kind => ({ gain: en ? 'gain' : '增益', gravity: en ? 'gravity' : '重力', dynamics: en ? 'dynamics' : '动力学' }[kind]));
    dialog.innerHTML = `<h3>${en ? 'Disconnect and remove temporary candidates' : '断连后撤销临时应用'}</h3><p>${en ? 'The following temporary runtime candidates are active' : '以下临时参数目前正在生效'}：<strong>${esc(names.join(' / '))}</strong></p><p>${en ? 'Disconnect will first park and disable the robot, restore the original runtime parameters, and then close the session. Saved YAML / URDF files will not change' : '断连将先停放并失能机械臂，依次撤销临时覆盖，再结束工作台连接，原始 YAML 和 URDF 文件不受影响'}</p><p>${en ? 'If safe parking or restoration fails, the session stays connected and you must inspect the robot state' : '如果安全停放或参数恢复失败，将保留工作台连接，请先排查机械臂状态'}</p><div class="actions"><button class="button" id="cancel-disconnect">${t('cancel')}</button><button class="button danger" id="confirm-disconnect">${en ? 'Park, restore and disconnect' : '停放、撤销并断连'}</button></div>`;
    dialog.showModal();
    $('cancel-disconnect').onclick = () => dialog.close();
    $('confirm-disconnect').onclick = () => { dialog.close(); blockingOperation(busyText('正在停放、撤销临时参数并断连...', 'Parking, restoring and disconnecting...'), async () => {
        await api.request('workbench_stop', {});
        toast(en ? 'Temporary candidates removed before disconnect' : '临时应用已在断连前撤销');
    }); };
}
function workbenchConfirm() { const info = state.info, dialog = $('confirm-dialog'); dialog.innerHTML = `<div class="eyebrow">${t('real')}</div><h3>${t('workbenchTitle')}</h3><p>${t('realDesc')}</p><dl class="kv"><dt>${t('profileName')}</dt><dd>${esc(info.profile)}</dd><dt>${t('serial')}</dt><dd>${esc(info.devices.join(', ') || '—')}</dd><dt>${t('writeEnabled')}</dt><dd>${t(info.write_enabled === true ? 'enabled' : 'unknown')}</dd></dl><label class="confirmation"><input id="workspace-confirmed" type="checkbox"><span>${t('confirm')}</span></label><div class="actions"><button id="workspace-cancel" class="button">${t('cancel')}</button><button id="workspace-connect" class="button primary" disabled>${t('startWorkbench')}</button></div>`; dialog.showModal(); $('workspace-confirmed').onchange = () => $('workspace-connect').disabled = !$('workspace-confirmed').checked; $('workspace-cancel').onclick = () => dialog.close(); $('workspace-connect').onclick = () => { dialog.close(); blockingOperation(busyText('正在连接控制工作台...', 'Connecting control workspace...'), () => connectWorkbench(true)); }; }
async function workbenchCommand(action) { const methods = { activate: 'activate', park: 'park', deactivate: 'deactivate', hold: 'hold', clear_fault: 'clear_fault', fault_compliant: 'fault_compliant', fault_rigid: 'fault_rigid' }; const method = methods[action]; if (!method) throw new Error('unknown workspace action'); state.commandState = 'submitting'; try { const result = await workbenchRequest(method); state.commandState = 'accepted'; return result; } catch (error) { state.commandState = 'failed'; throw error; } }
async function applyTuning() { const draft = state.tuneDraft || state.admittance; if (!draft) throw new Error('admittance parameters are not loaded'); const params = { observer_mode: draft.observer_mode, mass: draft.mass, damping: draft.damping, stiffness: draft.stiffness, max_delta_q: draft.max_delta_q, max_delta_q_dot: draft.max_delta_q_dot, momentum_gain: draft.momentum_gain }; state.admittance = await workbenchRequest('set_admittance', params); state.tuneDraft = structuredClone(state.admittance); state.savePreview = null; render(); }
async function finishCalibrationStage() { const before = state.calibrationFlow; await workbenchRequest('calibration_finish'); if (before === 'static') { state.calibrationFlow = 'validation'; await workbenchRequest('calibration_begin', { kind: 'validation' }); } else if (before === 'validation') { state.calibrationFlow = 'friction-ready'; } state.admittance = await workbenchRequest('get_admittance'); state.tuneDraft = structuredClone(state.admittance); render(); }
function frictionConfirm() { const dialog = $('confirm-dialog'); dialog.innerHTML = `<h3>${t('frictionCalibration')}</h3><p>${state.language === 'en' ? 'The robot will replay the demonstration path in reverse and forward directions; confirm people, cables, and surrounding space are safe, and fully release the robot' : '机械臂将反向和正向回放示教轨迹，请确认人员、线缆和周围空间安全，并确认已完全松手'}</p><div class="actions"><button id="friction-cancel-dialog" class="button">${t('cancel')}</button><button id="friction-confirm-replay" class="button danger">${t('startReplay')}</button></div>`; dialog.showModal(); $('friction-cancel-dialog').onclick = () => dialog.close(); $('friction-confirm-replay').onclick = () => { dialog.close(); blockingOperation(busyText('正在启动双向摩擦回放...', 'Starting bidirectional friction replay...'), () => workbenchRequest('friction_replay_start')); }; }
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
        const previousCalibrationPhase = state.telemetry?.model_calibration?.phase;
        const previousRobotState = state.telemetry?.robot_state;
        state.telemetry = event.data; state.telemetryReceivedAt = Date.now(); state.telemetryHistory.push(event.data); if (state.telemetryHistory.length > 600) state.telemetryHistory.shift();
        checkRuntimeAlerts(event.data);
        const calibration = event.data?.model_calibration;
        // Historical record selection belongs to the user, not the streaming
        // native session. A FAULT or task transition cannot clear it or swap
        // its directory for a different (possibly incomplete) live task.
        if (calibration?.directory && !state.modelCalibrationOffline) state.modelCalibrationDirectory = calibration.directory;
        const phaseBoundary = calibration?.phase !== previousCalibrationPhase && ['waiting_replay_confirmation', 'complete', 'cancelled', 'failed'].includes(calibration?.phase);
        const robotStateChanged = state.telemetry?.robot_state !== previousRobotState;
        if (phaseBoundary && state.page === 'workbench' && state.workbenchTab === 'calibration') {
            refreshCalibrationOrDefer();
        } else {
            updateWorkbenchLive();
        }
        if (robotStateChanged) { refreshGainApplyControls(); refreshCalibrationQuickControls(); refreshCalibrationActionControls(); }
        syncCompletedGainCandidate(calibration);
    }
    if (event.event === 'session') { state.session = event; if (event.mode === 'workbench' && event.state === 'running' && !event.recovery_required && !state.admittance) setTimeout(() => operation(loadAdmittance), 0); if (event.state === 'exited') { const hadTemporary = activeTemporaryOverrides().length > 0; state.calibrationRuntimeCandidates = null; if (event.force_stop_unverified && event.mode === 'workbench') toast(busyText('已强制终止控制进程，但无法确认执行器已经失能或临时参数已安全恢复，请检查硬件', 'Control process force-terminated; motor disable and safe temporary parameter restoration are unverified'), true); else if (hadTemporary && event.mode === 'workbench') toast(busyText('工作台已断连，临时参数不再生效，重新连接后将使用原始配置', 'Workspace disconnected; temporary parameters no longer apply, original settings will be used on reconnect')); const overlay = $('model-calibration-overlay'); if (overlay) { overlay.hidden = true; overlay.setAttribute('aria-hidden', 'true'); } } render(); }
    if (event.event === 'error') { state.backendError = event.error; toast(event.error, true); statusUI(); }
}
async function boot() {
    if (!api) { $('app').innerHTML = '<div class="empty-state">请通过 ./launch.sh 启动桌面应用 / Start the desktop application using ./launch.sh</div>'; return; }
    try {
        const prefs = await api.prefs(); state.language = prefs.language || 'zh-CN'; state.theme = prefs.theme || 'system'; state.mode = ['terminal', 'hardware', 'moveit'].includes(prefs.mode) ? prefs.mode : 'terminal';
        state.config = { profile: '', profile_file: '', serial_port: '', baudrate: '', bus: '', resource_paths: '' }; state.source = 'builtin'; state.page = 'start';
        applyTheme(); document.documentElement.lang = state.language; shell();
        $('priority-error-ack').onclick = acknowledgePriorityError;
        api.onEvent(onEvent);
        state.status = await api.request('status'); state.session = state.status.session || { state: 'idle' }; outputBuffer = await api.request('logs'); if (outputBuffer) term?.write(outputBuffer);
        await refreshLibrary(); render();
    } catch (error) { state.backendError = error.message; if ($('page')) { render(); toast(t('error') + ': ' + error.message, true); } }
}
window.addEventListener('resize', () => { if (state.page === 'run') { try { fit?.fit(); } catch { } } if (['model', 'workbench'].includes(state.page)) modelView?.resize(); });
matchMedia('(prefers-color-scheme: light)').addEventListener('change', applyTheme);
boot();
