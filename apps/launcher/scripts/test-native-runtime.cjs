const assert = require('node:assert/strict');
const fs = require('node:fs');
const os = require('node:os');
const path = require('node:path');
const {spawnSync} = require('node:child_process');
const {nativeEnvironment, developmentBackend} = require('../desktop/native-runtime.cjs');
const temporary = fs.mkdtempSync(path.join(os.tmpdir(), 'serialarm-native-env-'));
try {
  const prefix = path.join(temporary, 'native');
  const python = path.join(prefix, 'local/lib/python3.10/dist-packages');
  for (const folder of ['bin', 'lib', 'lib64', 'local/lib/python3.10/dist-packages']) fs.mkdirSync(path.join(prefix, folder), {recursive: true});
  const env = nativeEnvironment(prefix, {PATH: '/opt/ros/humble/bin:/usr/bin:/bin',
    AMENT_PREFIX_PATH: '/opt/ros/humble:/tmp/ros-overlay', COLCON_PREFIX_PATH: '/tmp/ros-overlay',
    ROS_DISTRO: 'humble', ROS_VERSION: '2', LD_LIBRARY_PATH: '/opt/ros/humble/lib:/tmp/ros-overlay/lib:/native-dependency/lib',
    PYTHONPATH: '/tmp/ros-overlay/local/lib/python3.10/dist-packages', CMAKE_PREFIX_PATH: '/opt/ros/humble:/opt/openrobots'});
  assert.equal(env.ROS_DISTRO, undefined);
  assert.equal(env.AMENT_PREFIX_PATH, undefined);
  assert.equal(env.COLCON_PREFIX_PATH, undefined);
  assert.ok(!env.PATH.includes('/opt/ros/'));
  assert.ok(!env.LD_LIBRARY_PATH.includes('/tmp/ros-overlay'));
  assert.ok(env.LD_LIBRARY_PATH.includes('/native-dependency/lib'));
  assert.ok(env.PYTHONPATH.includes(python));
  assert.equal(env.CMAKE_PREFIX_PATH, '/opt/openrobots');
  assert.equal(env.PYTHONNOUSERSITE, '1');
  const repo = path.resolve(__dirname, '../../..');
  const backend = developmentBackend(repo, {PATH: '/usr/bin:/bin',
    SERIAL_ARM_LAUNCHER_PYTHON: path.join(repo, '.install/gui-venv/bin/python'),
    PYTHONDONTWRITEBYTECODE: '1', OPENBLAS_NUM_THREADS: '1'});
  if (fs.existsSync(backend.python) && fs.existsSync(path.join(repo, '.install/manifest.json'))) {
    for (const args of [['-c', 'import serial_arm; print("CLEAN_NATIVE_PYTHON_PASS")'], [backend.script, '--check']]) {
      const result = spawnSync(backend.python, args, {env: backend.env, encoding: 'utf8', timeout: 30000});
      assert.equal(result.status, 0, result.stderr);
    }
    const probe = path.join(JSON.parse(fs.readFileSync(path.join(repo, '.install/manifest.json'))).install_prefix, 'bin/serial_arm_model_probe');
    const config = path.join(repo, 'src/robot_supports/robots/dm_arm/description/config/core/gray.yaml');
    const result = spawnSync(probe, ['--config', config], {env: backend.env, encoding: 'utf8', timeout: 30000});
    assert.equal(result.status, 0, result.stderr);
    assert.equal(JSON.parse(result.stdout).ok, true);
  }
  console.log('NATIVE_RUNTIME_NO_ROS_PASS');
} finally {fs.rmSync(temporary, {recursive: true, force: true});}
