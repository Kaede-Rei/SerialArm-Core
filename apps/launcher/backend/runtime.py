"""Linux PTY sessions with process groups and advisory device leases"""
import codecs
import fcntl
import hashlib
import os
from pathlib import Path
import pty
import signal
import select
import shutil
import sys
import struct
import subprocess
import termios
import threading
import time


class Supervisor:
    def __init__(self, emit, lock_dir):
        self.emit = emit
        self.lock_dir = Path(lock_dir)
        self.proc = None
        self.fd = None
        self.leases = []
        self.info = {'state': 'idle'}
        self.mutex = threading.RLock()
        self.thread = None

    def status(self):
        with self.mutex: return dict(self.info)

    def _release(self):
        for lease in self.leases:
            fcntl.flock(lease, fcntl.LOCK_UN)
            lease.close()
        self.leases = []

    def start(self, argv, env, devices, mode):
        with self.mutex:
            if self.info['state'] in ('running', 'stopping'): raise ValueError('a session is already running')
            self.lock_dir.mkdir(parents=True, exist_ok=True, mode=0o700)
            master = slave = None
            try:
                for device in sorted(set(os.path.realpath(d) for d in devices)):
                    name = hashlib.sha256(device.encode()).hexdigest() + '.lock'
                    lease = (self.lock_dir / name).open('a+')
                    try: fcntl.flock(lease, fcntl.LOCK_EX | fcntl.LOCK_NB)
                    except BlockingIOError:
                        lease.close()
                        raise ValueError('device is owned by another Launcher: ' + device)
                    self.leases.append(lease)
                if not argv or not shutil.which(str(argv[0]), path=env.get('PATH')):
                    raise FileNotFoundError(str(argv[0]) if argv else 'empty command')
                master, slave = pty.openpty()
                child = Path(__file__).with_name('pty_child.py')
                self.proc = subprocess.Popen([sys.executable, '-S', str(child), *argv], env=env, stdin=slave, stdout=slave, stderr=slave,
                                             start_new_session=True, close_fds=True)
                os.close(slave)
                slave = None
                self.fd = master
                self.info = {'state': 'running', 'pid': self.proc.pid, 'mode': mode,
                             'started_at': time.time(), 'devices': devices, 'command': argv}
                self.emit({'event': 'session', **self.info})
                self.thread = threading.Thread(target=self._read, args=(self.proc, master), daemon=True)
                self.thread.start()
                return self.status()
            except BaseException:
                if master is not None: os.close(master)
                if slave is not None: os.close(slave)
                self._release()
                raise

    def _read(self, proc, fd):
        decode = codecs.getincrementaldecoder('utf-8')('replace')
        try:
            ended = None
            while True:
                if proc.poll() is not None:
                    if ended is None:
                        ended = time.monotonic()
                        try: os.killpg(proc.pid, signal.SIGTERM)
                        except ProcessLookupError: pass
                    elif time.monotonic() - ended > 1:
                        try: os.killpg(proc.pid, signal.SIGKILL)
                        except ProcessLookupError: pass
                if not select.select([fd], [], [], .1)[0]:
                    if ended is not None and time.monotonic() - ended > 1.5: break
                    continue
                data = os.read(fd, 8192)
                if not data: break
                self.emit({'event': 'output', 'data': decode.decode(data)})
        except OSError: pass
        finally:
            tail = decode.decode(b'', final=True)
            if tail: self.emit({'event': 'output', 'data': tail})
            code = proc.wait()
            with self.mutex:
                os.close(fd)
                self.fd = None
                self._release()
                self.info.update(state='exited', exit_code=code)
                self.emit({'event': 'session', **self.info})

    def input(self, data):
        if not isinstance(data, str) or len(data) > 65536: raise ValueError('invalid terminal input')
        with self.mutex:
            if self.fd is None: raise ValueError('no active terminal')
            os.write(self.fd, data.encode())
        return True

    def resize(self, cols, rows):
        if not isinstance(cols, int) or not isinstance(rows, int) or not 10 <= cols <= 500 or not 2 <= rows <= 200:
            raise ValueError('invalid terminal dimensions')
        with self.mutex:
            if self.fd is not None: fcntl.ioctl(self.fd, termios.TIOCSWINSZ, struct.pack('HHHH', rows, cols, 0, 0))
        return True

    def stop(self, force=False):
        with self.mutex:
            if not self.proc or self.info['state'] not in ('running', 'stopping'): return self.status()
            self.info['state'] = 'stopping'
            self.emit({'event': 'session', **self.info})
            try: os.killpg(self.proc.pid, signal.SIGKILL if force else signal.SIGINT)
            except ProcessLookupError: pass
            return self.status()

    def close(self):
        self.stop()
        if self.thread:
            self.thread.join(timeout=3)
            if self.thread.is_alive():
                self.stop(force=True)
                self.thread.join(timeout=3)
