from pathlib import Path
import shutil
import subprocess
import sys
import tempfile

build=Path(sys.argv[1]); template=Path(__file__).with_name('sdk_consumer.cmake')
with tempfile.TemporaryDirectory(prefix='serialarm-sdk-') as temporary:
    root=Path(temporary);prefix=root/'relocated prefix';source=root/'consumer';source.mkdir()
    subprocess.run(['cmake','--install',str(build),'--prefix',str(prefix)],check=True,stdout=subprocess.DEVNULL)
    targets=prefix/'lib/cmake/serial_arm_core/serial_arm_targets.cmake'
    if '/opt/openrobots/include' in targets.read_text():raise RuntimeError('SDK exports build-host include paths')
    (source/'CMakeLists.txt').write_text(template.read_text())
    (source/'main.cpp').write_text('#include <serial_arm/robot.hpp>\n#include <serial_arm/dynamics/dynamics.hpp>\nint main(){serial_arm::Robot robot;serial_arm::Dynamics dynamics;return robot.get_state()!=serial_arm::RobotState::UNCONFIGURED;}\n')
    subprocess.run(['cmake','-S',str(source),'-B',str(root/'build'),'-DCMAKE_PREFIX_PATH='+str(prefix)+';/opt/openrobots'],check=True)
    subprocess.run(['cmake','--build',str(root/'build'),'--parallel','1'],check=True)
    subprocess.run([str(root/'build/consumer')],check=True)
print('SDK_RELOCATION_PASS')
