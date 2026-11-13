'use strict';
const fs=require('node:fs');
const os=require('node:os');
const path=require('node:path');
const assert=require('node:assert/strict');
const {spawnSync}=require('node:child_process');
const {nativeEnvironment,developmentBackend,nativePythonPaths}=require('../desktop/native-runtime.cjs');
const root=path.resolve(__dirname,'../../..');
const python=process.env.SERIAL_ARM_LAUNCHER_PYTHON||'/usr/bin/python3';
const run=(argv,env=process.env)=>{
  const result=spawnSync(argv[0],argv.slice(1),{env,encoding:'utf8',timeout:10000});
  assert.equal(result.status,0,result.error?.message||result.stderr||result.stdout);
  return result.stdout.trim();
};
const abi=run([python,'-I','-c','import sys; print(str(sys.version_info.major)+"."+str(sys.version_info.minor))']);
const [major,minor]=abi.split('.').map(Number),otherAbi=`${major}.${minor+1}`;
const temporary=fs.mkdtempSync(path.join(os.tmpdir(),'serialarm-native-python-'));
try {
  const prefix=path.join(temporary,'core prefix'),vendor=path.join(temporary,'vendor'),ros=path.join(temporary,'ros-overlay');
  const site=(base,version=abi,lib='lib')=>path.join(base,lib,`python${version}`,'site-packages');
  const coreSite=site(prefix),depSite=site(vendor),rosSite=site(ros),alienSite=site(vendor,otherAbi);
  for(const folder of [coreSite,depSite,rosSite,alienSite])fs.mkdirSync(folder,{recursive:true});
  fs.writeFileSync(path.join(coreSite,'native_fixture.py'),'value = "core"\n');
  fs.writeFileSync(path.join(depSite,'dep_fixture.py'),'value = "dependency"\n');
  fs.writeFileSync(path.join(alienSite,'alien_fixture.py'),'value = "wrong-abi"\n');
  const libraryPaths=[path.join(vendor,'lib','platform-triplet'),path.join(ros,'lib'),path.join(temporary,'missing','lib')];
  const inherited={...process.env,AMENT_PREFIX_PATH:ros,COLCON_PREFIX_PATH:ros,ROS_VERSION:'2',ROS_DISTRO:'fixture',
    PYTHONPATH:rosSite,LD_LIBRARY_PATH:path.join(ros,'lib'),CMAKE_PREFIX_PATH:ros,PYTHONNOUSERSITE:'0'};
  const environment=nativeEnvironment(prefix,inherited,'',libraryPaths,python);
  const modules=environment.PYTHONPATH.split(path.delimiter);
  assert(modules.includes(coreSite)&&modules.includes(depSite),'Core and native dependency Python modules must both be discovered');
  assert(!modules.includes(alienSite)&&!modules.includes(rosSite),'Different Python ABIs and ROS overlays stay excluded');
  assert(!('AMENT_PREFIX_PATH' in environment)&&!('COLCON_PREFIX_PATH' in environment)&&!('ROS_VERSION' in environment));
  assert(!environment.LD_LIBRARY_PATH.includes(ros)&&!environment.CMAKE_PREFIX_PATH.includes(ros));
  assert.equal(environment.PYTHONNOUSERSITE,'1');
  assert.equal(run([python,'-c','import native_fixture,dep_fixture,importlib.util; assert importlib.util.find_spec("alien_fixture") is None; print(native_fixture.value+" "+dep_fixture.value)'],environment),'core dependency');
  assert.deepEqual(nativePythonPaths('',[path.join(temporary,'absent')],abi),[],'Missing dependency prefixes stay missing');
  const bundlePython=path.join(temporary,'bundle-python');fs.mkdirSync(bundlePython);fs.writeFileSync(path.join(bundlePython,'bundle_fixture.py'),'value = "bundle"\n');
  const bundled=nativeEnvironment(prefix,inherited,bundlePython,libraryPaths,python);
  assert.equal(bundled.PYTHONPATH.split(path.delimiter)[0],bundlePython,'Packaged pure-Python modules retain their existing precedence');
  assert.equal(run([python,'-c','import bundle_fixture; print(bundle_fixture.value)'],bundled),'bundle');
  const fakeRoot=path.join(temporary,'checkout');fs.mkdirSync(path.join(fakeRoot,'.install'),{recursive:true});
  fs.writeFileSync(path.join(fakeRoot,'.install/manifest.json'),JSON.stringify({install_prefix:prefix,native_library_paths:libraryPaths,gui_python:'/missing/manifest-python'}));
  const selected=developmentBackend(fakeRoot,{...inherited,SERIAL_ARM_LAUNCHER_PYTHON:python});
  assert.equal(selected.python,python,'The explicit backend interpreter override selects the ABI as well as the executable');
  assert(selected.env.PYTHONPATH.split(path.delimiter).includes(depSite));
  assert.equal(selected.script,path.join(fakeRoot,'apps/launcher/backend/bridge.py'));
  assert.throws(()=>nativeEnvironment(prefix,inherited,'',libraryPaths,path.join(temporary,'missing-python')),/Python ABI/,'An unavailable interpreter cannot load modules from arbitrary ABI folders');
  // Exercise the generated installer environment, not just its path helper.
  const originalSetup=path.join(temporary,'dependency-setup.bash'),generated=path.join(temporary,'native-setup.bash');
  fs.writeFileSync(originalSetup,':\n');
  const installerScript=`import json,sys\nfrom pathlib import Path\nfrom types import SimpleNamespace\nsys.path.insert(0,sys.argv[1])\nfrom runner import Runner,native_python_paths\nrunner=Runner.__new__(Runner)\nrunner.prefix=Path(sys.argv[2]); runner.setup=Path(sys.argv[3]); runner.python=Path(sys.argv[4])\nrunner.env=dict(AMENT_PREFIX_PATH=sys.argv[5],COLCON_PREFIX_PATH=sys.argv[5])\nrunner.plan=SimpleNamespace(ros2=False,python=False)\npaths=json.loads(sys.argv[6]); runner.dependency_library_paths=lambda: paths\nPath(sys.argv[7]).write_text(runner.setup_text())\nprint(json.dumps([str(p) for p in native_python_paths(runner.prefix,paths,runner.python,runner.env)]))\n`;
  const installerModules=JSON.parse(run([python,'-B','-c',installerScript,path.join(root,'tools/install'),prefix,originalSetup,python,ros,JSON.stringify(libraryPaths),generated]));
  assert(installerModules.includes(coreSite)&&installerModules.includes(depSite)&&!installerModules.includes(alienSite)&&!installerModules.includes(rosSite));
  const clean={...process.env,PYTHONPATH:'',LD_LIBRARY_PATH:'',PYTHONNOUSERSITE:'1'};
  assert.equal(run(['bash','-c','source "$1" && exec "$2" -c \'import native_fixture,dep_fixture; print(native_fixture.value+" "+dep_fixture.value)\'','serialarm-native-test',generated,python],clean),'core dependency');
  const manifest=path.join(root,'.install/manifest.json');
  if(fs.existsSync(manifest)) {
    const installed=JSON.parse(fs.readFileSync(manifest,'utf8'));
    if(installed.install_prefix&&installed.gui_python&&fs.existsSync(installed.gui_python)) {
      const backend=developmentBackend(root,process.env);
      const available=run([backend.python,'-I','-c','import importlib.util; print(importlib.util.find_spec("pinocchio") is not None)'],backend.env)==='True'||(backend.env.PYTHONPATH||'').split(path.delimiter).some(folder=>fs.existsSync(path.join(folder,'pinocchio','__init__.py')));
      if(available) {
      const result=run([backend.python,'-B','-c','import json,sys; sys.path.insert(0,sys.argv[1]); import bridge,pinocchio,numpy; print(json.dumps({"python":sys.executable,"numpy":numpy.__version__,"version":pinocchio.__version__,"module":pinocchio.__file__}))',path.join(root,'apps/launcher/backend')],backend.env);
      console.log('GUI_NATIVE_PINOCCHIO_IMPORT '+result);
      } else console.log('GUI_NATIVE_PINOCCHIO_NOT_INSTALLED');
    }
  }
  console.log('LAUNCHER_NATIVE_PYTHON_RUNTIME_PASS');
} finally {fs.rmSync(temporary,{recursive:true,force:true});}
