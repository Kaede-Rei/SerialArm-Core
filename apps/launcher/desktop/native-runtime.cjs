// Core and the desktop workbench use native libraries directly; ROS is loaded
// separately by the optional adapter launcher, never by desktop startup
const fs = require('node:fs');
const path = require('node:path');
const {spawnSync} = require('node:child_process');

function pythonAbi(python, env) {
  // Query the actual backend interpreter in isolation; inherited Python paths
  // cannot import a foreign extension while deciding which ABI to expose.
  const result = spawnSync(python, ['-I', '-c', 'import sys,sysconfig,json; print(json.dumps({"abi":str(sys.version_info.major)+"."+str(sys.version_info.minor),"venv_site":sysconfig.get_path("purelib") if sys.prefix!=sys.base_prefix else ""}))'],
    {env, encoding:'utf8', timeout:5000});
  let version={};
  try {version=JSON.parse(result.stdout || '{}');} catch {}
  if (result.status !== 0 || !/^\d+\.\d+$/.test(version.abi || ''))
    throw new Error(`Cannot inspect backend Python ABI (${python}): ${result.error?.message || result.stderr || 'interpreter unavailable'}`);
  return version;
}

function nativePythonPaths(prefix, libraryPaths, abi, isRosPath = () => false) {
  const roots = prefix ? ['lib','lib64','local/lib','python-venv/lib'].map(folder => path.join(prefix,folder)) : [];
  for (const value of libraryPaths) {
    if (!value || isRosPath(value)) continue;
    let folder=path.resolve(value);
    roots.push(folder);
    // A linker can report a multiarch directory below lib, or a CMake package
    // can come from lib64. Recover that library root, never an arbitrary host
    // Python version or a ROS overlay.
    while (path.dirname(folder)!==folder) {
      if (['lib','lib64'].includes(path.basename(folder))) {roots.push(folder);break;}
      folder=path.dirname(folder);
    }
  }
  return [...new Set(roots.flatMap(folder => ['site-packages','dist-packages']
    .map(site => path.join(folder,`python${abi}`,site))).filter(folder => fs.existsSync(folder)))];
}

function nativeEnvironment(prefix, inherited = process.env, pythonRoot = '', libraryPaths = [], python = inherited.SERIAL_ARM_LAUNCHER_PYTHON || '/usr/bin/python3') {
  const env = {...inherited};
  const rosPrefixes = [env.AMENT_PREFIX_PATH, env.COLCON_PREFIX_PATH]
    .filter(Boolean).flatMap(value => value.split(path.delimiter)).filter(Boolean);
  const isRosPath = value => value.startsWith('/opt/ros/') || rosPrefixes.some(base => value === base || value.startsWith(base + path.sep));
  for (const key of ['PATH', 'PYTHONPATH', 'LD_LIBRARY_PATH', 'CMAKE_PREFIX_PATH']) {
    env[key] = (env[key] || '').split(path.delimiter).filter(value => value && !isRosPath(value)).join(path.delimiter);
  }
  for (const key of Object.keys(env)) {
    if (key.startsWith('ROS_') || ['AMENT_PREFIX_PATH', 'COLCON_PREFIX_PATH'].includes(key)) delete env[key];
  }
  const prepend = (key, values) => {
    env[key] = [...values.filter(value => fs.existsSync(value)), ...(env[key] || '').split(path.delimiter).filter(Boolean)]
      .filter((value, i, all) => all.indexOf(value) === i).join(path.delimiter);
  };
  const dependencies=libraryPaths.filter(value=>value&&!isRosPath(value));
  if (prefix) {
    prepend('PATH', [path.join(prefix, 'bin')]);
    prepend('LD_LIBRARY_PATH', [path.join(prefix, 'lib'), path.join(prefix, 'lib64'), ...dependencies]);
    // Legacy ROS installations did not record native dependency paths. The
    // offline model tool has the linker's dependency search paths, so recover
    // those resolved library directories without loading any ROS environment
    const probe = path.join(prefix, 'bin/serial_arm_model_probe');
    if (fs.existsSync(probe)) {
      const result = spawnSync('ldd', [probe], {env, encoding: 'utf8', timeout: 5000});
      const directories = [...(result.stdout || '').matchAll(/=> (\/\S+)/g)]
        .map(match => path.dirname(match[1])).filter(value => !isRosPath(value));
      prepend('LD_LIBRARY_PATH', directories);
      dependencies.push(...directories);
    }
    prepend('SERIAL_ARM_RESOURCE_PATH', [prefix]);
  }
  const interpreter=pythonAbi(python,env);
  prepend('PYTHONPATH', nativePythonPaths(prefix,dependencies,interpreter.abi,isRosPath));
  // Match bridge.py: the selected venv owns NumPy/SciPy and their extension
  // ABI, while Core's separate binding venv only supplies missing modules.
  if (interpreter.venv_site) prepend('PYTHONPATH',[interpreter.venv_site]);
  if (pythonRoot) prepend('PYTHONPATH', [pythonRoot]);
  env.PYTHONNOUSERSITE = '1';
  return env;
}

function developmentBackend(root, inherited = process.env) {
  const receipt = path.join(root, '.install/manifest.json');
  const manifest = fs.existsSync(receipt) ? JSON.parse(fs.readFileSync(receipt, 'utf8')) : {};
  const python=inherited.SERIAL_ARM_LAUNCHER_PYTHON || manifest.gui_python || '/usr/bin/python3';
  const env = nativeEnvironment(manifest.install_prefix || '', inherited, '', manifest.native_library_paths || [], python);
  return {python,
    script: path.join(root, 'apps/launcher/backend/bridge.py'), env};
}
module.exports = {nativeEnvironment, developmentBackend, nativePythonPaths};
