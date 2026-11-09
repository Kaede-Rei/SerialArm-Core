/* Read-only fixture for the complete production Electron renderer */
const fs=require('node:fs');
const path=require('node:path');
const {spawnSync}=require('node:child_process');
const {makeCandidate}=require('./gain-ui-fixture.cjs');
const launcher=path.resolve(__dirname,'..'),repo=path.resolve(launcher,'../..');
function buildFixture(directory='/tmp/serialarm-studio-ui',options={}){
 const out=path.resolve(directory);fs.mkdirSync(out,{recursive:true});
 const result=spawnSync('python3',['-c',[
  'import sys,json,pathlib,xml.etree.ElementTree as ET,yaml',
  'root=pathlib.Path(sys.argv[1]);sys.path.insert(0,str(root/"apps/launcher/backend"))',
  'from control_parameters import canonical_core',
  'base=root/"src/robot_supports/robots/dm_arm/description"',
  'core_path=base/"config/core/white.yaml";hardware_path=base/"config/hardware.yaml"',
  'core=canonical_core(yaml.safe_load(core_path.read_text()));hardware=yaml.safe_load(hardware_path.read_text())',
  'urdf=(core_path.parent/core["model"]["urdf_path"]).resolve()',
  'limits={node.get("name"):{"type":node.get("type"),**{k:float(v) for k,v in node.find("limit").attrib.items()}} for node in ET.parse(urdf).getroot().findall("joint") if node.find("limit") is not None}',
  'print(json.dumps(dict(profile="dm_arm_white",core=core,hardware=hardware,joint_names=core["model"]["joint_names"],urdf_joint_limits=limits,core_path=str(core_path),hardware_path=str(hardware_path),urdf_path=str(urdf),profile_entry={"hardware":{"plugin":"serial_arm_hardware_damiao"}},setup_issues=[])))'
 ].join('\n'),repo],{encoding:'utf8'});
 if(result.status!==0)throw Error(result.stderr);
 const editor=JSON.parse(result.stdout);
 editor.core.model.gravity=[0.012345678901234568,-0.0012345678901234567,-9.81];
 editor.core.model.gravity_scale.joint1=1.1234567890123457;
 const candidate=makeCandidate();
 Object.assign(candidate.model_initialization,{unconstrained_kp:candidate.model_initialization.kp.map(v=>v*4),unconstrained_kd:candidate.model_initialization.kd.map(v=>v*2),pre_modal_kp:candidate.model_initialization.kp.map(v=>v*1.5),pre_modal_kd:candidate.model_initialization.kd.map(v=>v*1.2),modal_adjustments:['sample=0;mode=2;axis=2;stiffness_participation=.85;action=allocate_damping','sample=0;mode=3;axis=3;action=reduce_participating_frequency'],physical_prior:'urdf_geometry_mass_com_inertia',response_demand_source:'checked_demonstration_and_user_acceptance_targets',constraint_role:'execution_guards_not_mechanical_identification'});
 const data={editor,candidate,info:{profile:editor.profile,profile_file:'/offline/robot_profiles.yaml',hardware_plugin:'serial_arm_hardware_damiao',available:{terminal:true,hardware:true,model:true,moveit:false},resources:{core:editor.core_path,hardware:editor.hardware_path},checks:[{name:'core',ok:true,detail:editor.core_path},{name:'description',ok:true,detail:editor.urdf_path},{name:'hardware',ok:true,detail:'serial_arm_hardware_damiao'},{name:'ros2_control',ok:false,detail:'未安装 ROS2 适配层'}]},library:[{id:'fixture-white',profile:'dm_arm_white',source:'builtin',readiness:{state:'configuring'}},{id:'fixture-tomato',profile:'tomato_picker',source:'external',profile_file:'/offline/tomato_picker/robot_profiles.yaml',readiness:{state:'ready'}}]};
 if(options.receipt){data.receipt=JSON.parse(fs.readFileSync(path.resolve(options.receipt),'utf8'));data.receiptLabel=path.basename(data.receipt.runtime_baseline_source||'model-receipt','.yaml')+'（只读回执）';}
 if(options.record){
  const directory=path.resolve(options.record),read=name=>JSON.parse(fs.readFileSync(path.join(directory,name),'utf8'));
  const frame=spawnSync('python3',['-c','import csv,json,sys\nwith open(sys.argv[1]) as f:\n rows=csv.DictReader(f);last=next(rows,{})\n for row in rows:last=row\nprint(json.dumps(last))',path.join(directory,'frames.csv')],{encoding:'utf8'});
  if(frame.status!==0)throw Error(frame.stderr);
  data.overlayRecord={directory,candidate:read('gain-candidate.json'),result:read('result.json'),metadata:read('metadata.json'),frame:JSON.parse(frame.stdout)};
 }
 fs.writeFileSync(path.join(out,'data.json'),JSON.stringify(data,null,2));
 fs.writeFileSync(path.join(out,'preload.cjs'),`const {contextBridge}=require('electron');const data=require(${JSON.stringify(path.join(out,'data.json'))});const calls=[];const fail=name=>{throw Error('Offline UI fixture rejects device/write request: '+name);};contextBridge.exposeInMainWorld('studioUiData',data);contextBridge.exposeInMainWorld('serialArm',{prefs:async()=>({theme:'light',language:'zh-CN'}),savePrefs:async()=>({}),request:async(name)=>{calls.push(name);if(name==='status')return {installed:true,prefix:'/offline/install',session:{state:'idle'}};if(name==='logs')return '';if(name==='profile_library')return {profiles:data.library};if(name==='hardware_inspect')return {plugin:'serial_arm_hardware_damiao',drivers:[{plugin:'serial_arm_hardware_damiao',available:true,label:'达妙 / Damiao'}]};return fail(name);},graphics:{status:async()=>({effectiveMode:'normal',requestedMode:'normal',requiresRestart:false,status:'ready'}),report:async()=>({}),setMode:async()=>fail('graphics.setMode')},onEvent:()=>{},onCloseRequest:()=>{},window:()=>{},clipboard:{write:async()=>{},read:async()=>''},selectProfile:async()=>null,selectProfilePackage:async()=>null,revealFile:async()=>{},fixtureCalls:async()=>calls});`);
 return {directory:out,preload:path.join(out,'preload.cjs'),index:path.join(launcher,'renderer/index.html')};
}
if(require.main===module)console.log(JSON.stringify(buildFixture(process.argv[2])));
module.exports={buildFixture};
