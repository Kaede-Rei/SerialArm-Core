from pathlib import Path
import shutil
import subprocess
import sys
import tempfile

build=Path(sys.argv[1]); template=Path(__file__).with_name('sdk_consumer.cmake')

def materialize_symlink_sdk(prefix):
    cache={}
    for line in (build/'CMakeCache.txt').read_text().splitlines():
        if '=' in line and not line.startswith(('#','//')):
            key,value=line.split('=',1)
            cache[key.split(':',1)[0]]=value
    if cache.get('AMENT_CMAKE_SYMLINK_INSTALL','').upper() not in ('1','ON','TRUE','YES'):
        return
    # ament's generated symlink installer uses its configured absolute prefix,
    # even when cmake --install receives a different --prefix. Materialize only
    # SDK headers/configuration from that manifest; never replace the freshly
    # installed temporary libraries or their export files with installed copies.
    normal_prefix=Path(cache['CMAKE_INSTALL_PREFIX'])
    manifest=build/'symlink_install_manifest.txt'
    for entry in manifest.read_text().splitlines():
        source=Path(entry)
        relative=source.relative_to(normal_prefix)
        if relative.parts[0]!='include' and relative.parts[:3]!=('lib','cmake','serial_arm_core'):
            continue
        destination=prefix/relative
        if destination.exists():
            continue
        destination.parent.mkdir(parents=True,exist_ok=True)
        shutil.copy2(source,destination,follow_symlinks=True)

with tempfile.TemporaryDirectory(prefix='serialarm-sdk-') as temporary:
    root=Path(temporary);prefix=root/'relocated prefix';source=root/'consumer';source.mkdir()
    subprocess.run(['cmake','--install',str(build),'--prefix',str(prefix)],check=True,stdout=subprocess.DEVNULL)
    materialize_symlink_sdk(prefix)
    targets=prefix/'lib/cmake/serial_arm_core/serial_arm_targets.cmake'
    if '/opt/openrobots/include' in targets.read_text():raise RuntimeError('SDK exports build-host include paths')
    (source/'CMakeLists.txt').write_text(template.read_text())
    (source/'main.cpp').write_text('''#include <serial_arm/robot.hpp>
#include <serial_arm/dynamics/dynamics.hpp>
#include <serial_arm/runtime/control_runtime.hpp>
int main(){
    serial_arm::Robot robot;
    serial_arm::Dynamics dynamics;
    serial_arm::ControlRuntime runtime;
    const auto snapshot=runtime.snapshot();
    return robot.get_state()!=serial_arm::RobotState::UNCONFIGURED ||
        snapshot.state!=serial_arm::RobotState::UNCONFIGURED || snapshot.sequence!=0;
}
''')
    subprocess.run(['cmake','-S',str(source),'-B',str(root/'build'),'-DCMAKE_PREFIX_PATH='+str(prefix)+';/opt/openrobots',
        '-Dserial_arm_core_DIR='+str(prefix/'lib/cmake/serial_arm_core')],check=True)
    subprocess.run(['cmake','--build',str(root/'build'),'--parallel','1'],check=True)
    subprocess.run([str(root/'build/consumer')],check=True)
print('SDK_RELOCATION_PASS')
