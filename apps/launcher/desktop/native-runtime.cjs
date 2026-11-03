// Core and the desktop workbench use native libraries directly; ROS is loaded
// separately by the optional adapter launcher, never by desktop startup
const fs = require('node:fs');
const path = require('node:path');
const {spawnSync} = require('node:child_process');

function nativeEnvironment(prefix, inherited = process.env, pythonRoot = '', libraryPaths = []) {
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
  if (prefix) {
    prepend('PATH', [path.join(prefix, 'bin')]);
    prepend('LD_LIBRARY_PATH', [path.join(prefix, 'lib'), path.join(prefix, 'lib64'), ...libraryPaths]);
    // Legacy ROS installations did not record native dependency paths. The
    // offline model tool has the linker's dependency search paths, so recover
    // those resolved library directories without loading any ROS environment
    const probe = path.join(prefix, 'bin/serial_arm_model_probe');
    if (fs.existsSync(probe)) {
      const result = spawnSync('ldd', [probe], {env, encoding: 'utf8', timeout: 5000});
      const directories = [...(result.stdout || '').matchAll(/=> (\/\S+)/g)]
        .map(match => path.dirname(match[1])).filter(value => !isRosPath(value));
      prepend('LD_LIBRARY_PATH', directories);
    }
    prepend('SERIAL_ARM_RESOURCE_PATH', [prefix]);
    const libraries = ['lib', 'lib64', 'local/lib', 'python-venv/lib'];
    const modules = [];
    for (const folder of libraries) {
      const library = path.join(prefix, folder);
      if (!fs.existsSync(library)) continue;
      for (const entry of fs.readdirSync(library)) {
        if (/^python\d+\.\d+$/.test(entry)) {
          for (const site of ['site-packages', 'dist-packages']) modules.push(path.join(library, entry, site));
        }
      }
    }
    prepend('PYTHONPATH', modules);
  }
  if (pythonRoot) prepend('PYTHONPATH', [pythonRoot]);
  env.PYTHONNOUSERSITE = '1';
  return env;
}

function developmentBackend(root, inherited = process.env) {
  const receipt = path.join(root, '.install/manifest.json');
  const manifest = fs.existsSync(receipt) ? JSON.parse(fs.readFileSync(receipt, 'utf8')) : {};
  const env = nativeEnvironment(manifest.install_prefix || '', inherited, '', manifest.native_library_paths || []);
  return {python: inherited.SERIAL_ARM_LAUNCHER_PYTHON || manifest.gui_python || '/usr/bin/python3',
    script: path.join(root, 'apps/launcher/backend/bridge.py'), env};
}
module.exports = {nativeEnvironment, developmentBackend};
