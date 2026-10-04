"""Structured native workspace session with bounded JSON line transport and device leases"""
import fcntl
import hashlib
import json
import os
from pathlib import Path
import subprocess
import threading
import time
from collections import deque


class NativeSession:
    def __init__(self, emit, lock_dir):
        self.emit = emit
        self.lock_dir = Path(lock_dir)
        self.proc = None
        self.pending = {}
        self.leases = []
        self.seq = 0
        self.state = {'state': 'idle', 'mode': 'workbench'}
        self.mutex = threading.RLock()
        self.reader = None
        self.stderr_reader = None
        self.telemetry_history = deque(maxlen=20000)

    def status(self):
        with self.mutex:
            return dict(self.state)

    def _lease(self, devices):
        self.lock_dir.mkdir(parents=True, exist_ok=True, mode=0o700)
        for device in sorted(set(os.path.realpath(d) for d in devices)):
            name = hashlib.sha256(device.encode()).hexdigest() + '.lock'
            lease = (self.lock_dir / name).open('a+')
            try:
                fcntl.flock(lease, fcntl.LOCK_EX | fcntl.LOCK_NB)
            except BlockingIOError:
                lease.close()
                self._release()
                raise ValueError('device is owned by another Launcher: ' + device)
            self.leases.append(lease)

    def _release(self):
        for lease in self.leases:
            try: fcntl.flock(lease, fcntl.LOCK_UN)
            finally: lease.close()
        self.leases = []

    def start(self, argv, env, devices):
        with self.mutex:
            if self.state.get('state') in ('running', 'stopping'):
                raise ValueError('a workspace session is already running')
            self._lease(devices)
            try:
                self.proc = subprocess.Popen(argv, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                                             env=env, text=True, bufsize=1, start_new_session=True)
            except BaseException:
                self._release()
                raise
            self.state = {'state': 'running', 'mode': 'workbench', 'pid': self.proc.pid,
                          'started_at': time.time(), 'devices': list(devices), 'command': list(argv)}
            self.emit({'event': 'session', **self.state})
            self.reader = threading.Thread(target=self._read_stdout, daemon=True)
            self.stderr_reader = threading.Thread(target=self._read_stderr, daemon=True)
            self.reader.start(); self.stderr_reader.start()
            return self.status()

    def _read_stdout(self):
        proc = self.proc
        try:
            for line in proc.stdout:
                try: message = json.loads(line)
                except json.JSONDecodeError:
                    self.emit({'event': 'error', 'error': 'Invalid native session message'})
                    continue
                if message.get('event'):
                    if message.get('event') == 'telemetry':
                        with self.mutex:
                            self.telemetry_history.append(message)
                    self.emit(message)
                    continue
                request = None
                with self.mutex: request = self.pending.pop(message.get('id'), None)
                if request:
                    request['message'] = message
                    request['event'].set()
        finally:
            code = proc.wait()
            with self.mutex:
                for request in self.pending.values():
                    request['message'] = {'error': 'native session closed'}
                    request['event'].set()
                self.pending.clear()
                self._release()
                self.state.update(state='exited', exit_code=code)
                self.emit({'event': 'session', **self.state})

    def _read_stderr(self):
        proc = self.proc
        for line in proc.stderr:
            if line.strip(): self.emit({'event': 'native_log', 'data': line[-8192:]})

    def request(self, method, params=None, timeout=8):
        if not isinstance(method, str) or not method or len(method) > 128: raise ValueError('invalid native request method')
        if params is None: params = {}
        if not isinstance(params, dict): raise ValueError('native request params must be an object')
        with self.mutex:
            if not self.proc or self.state.get('state') not in ('running', 'stopping'): raise ValueError('workspace session is not running')
            self.seq += 1
            number = self.seq
            event = threading.Event()
            request = {'event': event, 'message': None}
            self.pending[number] = request
            try:
                self.proc.stdin.write(json.dumps({'id': number, 'method': method, 'params': params}, separators=(',', ':')) + '\n')
                self.proc.stdin.flush()
            except BaseException:
                self.pending.pop(number, None)
                raise
        if not event.wait(timeout):
            with self.mutex: self.pending.pop(number, None)
            raise TimeoutError('native session request timed out')
        message = request['message'] or {}
        if message.get('error'): raise ValueError(message['error'])
        return message.get('result')


    def telemetry(self):
        with self.mutex:
            return list(self.telemetry_history)

    def stop(self, force=False):
        with self.mutex:
            if not self.proc or self.state.get('state') not in ('running', 'stopping'): return self.status()
            proc = self.proc
        graceful = False
        if not force and self.state.get('state') == 'running':
            try:
                self.request('shutdown', timeout=120)
                proc.wait(timeout=2)
                graceful = True
            except (Exception, subprocess.TimeoutExpired):
                pass
        with self.mutex:
            if self.state.get('state') != 'exited':
                self.state['state'] = 'stopping'
                self.emit({'event': 'session', **self.state})
        if not graceful and proc.poll() is None:
            try: os.killpg(proc.pid, 9 if force else 15)
            except ProcessLookupError: pass
        return self.status()

    def close(self):
        self.stop()
        if self.reader: self.reader.join(timeout=6)
        if self.stderr_reader: self.stderr_reader.join(timeout=2)
        if self.proc and self.proc.poll() is None:
            self.stop(force=True)
            if self.reader: self.reader.join(timeout=2)
            if self.stderr_reader: self.stderr_reader.join(timeout=2)
        if self.proc:
            for stream in (self.proc.stdin, self.proc.stdout, self.proc.stderr):
                try: stream.close()
                except Exception: pass
