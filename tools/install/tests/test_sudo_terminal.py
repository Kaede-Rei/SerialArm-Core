"""Exercise password prompts through a real controlling terminal without sudo"""
import os
from pathlib import Path
import pty
import select
import subprocess
import sys
import tempfile
import time
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from plan import parse_plan
from runner import Runner, InstallError


class SudoTerminalTests(unittest.TestCase):
    def test_password_prompt_is_visible_and_password_never_enters_log(self):
        with tempfile.TemporaryDirectory(prefix='installer tty ') as tmp:
            root = Path(tmp)
            fake = root / 'bin'
            fake.mkdir()
            sudo = fake / 'sudo'
            sudo.write_text('''#!/usr/bin/env python3
import os, sys, termios
try:
    fd = os.open('/dev/tty', os.O_RDWR)
except OSError:
    sys.stderr.write('sudo: a terminal is required to read the password\\n')
    sys.exit(1)
saved = termios.tcgetattr(fd)
quiet = termios.tcgetattr(fd)
quiet[3] &= ~termios.ECHO
termios.tcsetattr(fd, termios.TCSANOW, quiet)
try:
    sys.stderr.write('Password for installer test: ')
    sys.stderr.flush()
    password = os.read(fd, 1024).strip()
finally:
    termios.tcsetattr(fd, termios.TCSANOW, saved)
    os.close(fd)
if password != b'fixture-password': sys.exit(2)
os.execvp(sys.argv[1], sys.argv[1:])
''')
            sudo.chmod(0o755)
            apt = fake / 'apt-get'
            apt.write_text('#!/bin/sh\nprintf "apt step %s\\n" "$1"\n')
            apt.chmod(0o755)
            helper = root / 'exercise.py'
            helper.write_text('''import fcntl, os, sys, termios
from pathlib import Path
os.setsid()
fcntl.ioctl(0, termios.TIOCSCTTY, 0)
sys.path.insert(0, sys.argv[1])
from plan import parse_plan
from runner import Runner
os.geteuid = lambda: 1000
r = Runner(parse_plan(['--preset', 'core'], Path(sys.argv[2])))
r.apt(['fixture-package'])
print('INSTALLER_TTY_PASS', flush=True)
''')
            master, slave = pty.openpty()
            env = dict(os.environ, PATH=str(fake) + ':' + os.environ['PATH'])
            proc = subprocess.Popen([sys.executable, str(helper), str(Path(__file__).resolve().parents[1]), str(root)],
                                    stdin=slave, stdout=slave, stderr=slave, env=env)
            os.close(slave)
            output = b''
            answered = 0
            deadline = time.monotonic() + 10
            try:
                while time.monotonic() < deadline:
                    ready, _, _ = select.select([master], [], [], .1)
                    if ready:
                        try: chunk = os.read(master, 8192)
                        except OSError: break
                        if not chunk: break
                        output += chunk
                        prompts = output.count(b'Password for installer test: ')
                        while answered < prompts:
                            os.write(master, b'fixture-password\n')
                            answered += 1
                    elif proc.poll() is not None:
                        break
                if proc.poll() is None: proc.kill()
                proc.wait(timeout=3)
            finally:
                if proc.poll() is None: proc.kill(); proc.wait()
                os.close(master)
            self.assertEqual(proc.returncode, 0, output.decode(errors='replace'))
            self.assertEqual(answered, 2, output.decode(errors='replace'))
            self.assertIn(b'INSTALLER_TTY_PASS', output)
            self.assertNotIn(b'fixture-password', output)
            logs = list((root / 'log').rglob('*.log'))
            self.assertEqual(len(logs), 2)
            for log in logs:
                self.assertIn('apt step', log.read_text())
                self.assertNotIn('fixture-password', log.read_text())

    def test_noninteractive_password_requirement_has_actionable_error(self):
        with tempfile.TemporaryDirectory() as tmp:
            r = Runner(parse_plan(['--preset', 'core', '--yes'], Path(tmp)))
            with patch('runner.os.geteuid', return_value=1000), \
                 patch('runner.shutil.which', return_value='/usr/bin/fake'), \
                 patch('runner.sys.stdin.isatty', return_value=False), \
                 patch.object(r, 'run', side_effect=InstallError('sudo: a password is required')):
                with self.assertRaisesRegex(InstallError, '--yes.*sudo|sudo.*--yes'):
                    r.apt(['fixture'])

    def test_rosdep_install_keeps_password_terminal(self):
        with tempfile.TemporaryDirectory() as tmp:
            r = Runner(parse_plan(['--preset', 'ros2'], Path(tmp)))
            r.plan.package_paths = lambda: ['/tmp/core']
            with patch.object(r, 'apt'), patch.object(r, 'run') as command:
                r.ros()
            call = next(c for c in command.call_args_list if c.args[0] == 'rosdep-install')
            self.assertTrue(call.kwargs.get('terminal'))

    def test_terminal_ctrl_c_stops_command_without_killing_parent_group(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            helper = root / 'interrupt.py'
            helper.write_text('''import fcntl, os, sys, termios
from pathlib import Path
os.setsid()
fcntl.ioctl(0, termios.TIOCSCTTY, 0)
sys.path.insert(0, sys.argv[1])
from plan import parse_plan
from runner import Runner
r = Runner(parse_plan(['--preset', 'core'], Path(sys.argv[2])))
try:
    r.run('interrupt-test', [sys.executable, '-c', 'import os,time; print("CHILD_PID="+str(os.getpid()),flush=True); time.sleep(30)'], terminal=True)
except KeyboardInterrupt:
    print('INTERRUPT_CLEANUP_PASS', flush=True)
    sys.exit(130)
''')
            master, slave = pty.openpty()
            proc = subprocess.Popen([sys.executable, str(helper), str(Path(__file__).resolve().parents[1]), str(root)],
                                    stdin=slave, stdout=slave, stderr=slave)
            os.close(slave)
            output = b''
            child = None
            deadline = time.monotonic() + 10
            try:
                while time.monotonic() < deadline:
                    ready, _, _ = select.select([master], [], [], .1)
                    if ready:
                        try: chunk = os.read(master, 8192)
                        except OSError: break
                        if not chunk: break
                        output += chunk
                        import re
                        found = re.search(rb'CHILD_PID=(\d+)\r?\n', output)
                        if found and child is None:
                            child = int(found.group(1))
                            os.write(master, b'\x03')
                    elif proc.poll() is not None:
                        break
                if proc.poll() is None: proc.kill()
                proc.wait(timeout=3)
            finally:
                if proc.poll() is None: proc.kill(); proc.wait()
                os.close(master)
            self.assertEqual(proc.returncode, 130, output.decode(errors='replace'))
            self.assertIn(b'INTERRUPT_CLEANUP_PASS', output)
            self.assertIsNotNone(child)
            with self.assertRaises(ProcessLookupError): os.kill(child, 0)


if __name__ == '__main__':
    unittest.main()
